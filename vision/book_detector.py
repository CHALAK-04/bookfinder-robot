"""
book_detector.py  —  Book spine detector

Finds a target book on a shelf photo by fusing two signals:

  1. PaddleOCR reads text off the spines. Keywords extracted from the target
     cover image are matched against them with fuzzy string matching, which
     tolerates the OCR errors that vertical text on a curved spine produces.

  2. SigLIP2 scores overlapping vertical strips of the shelf against a text
     prompt built from the same keywords, catching books whose text the OCR
     misses entirely.

Both run in parallel. Output is an annotated image plus a JSON file with the
pixel position and confidence of every hit.

The OCR result parser handles both the legacy and the current PaddleOCR
output formats, so the script runs across versions.
"""

import time
import os
import json
import concurrent.futures

import cv2
import numpy as np
from PIL import Image

import torch
from transformers import AutoProcessor, AutoModel

from paddleocr import PaddleOCR
from rapidfuzz import fuzz


# ─────────────────────────────────────────────────────────────────────────────
# SETTINGS
# ─────────────────────────────────────────────────────────────────────────────

TARGET_IMG = "Games people play.jpeg"
SHELF_IMG = "row_2.jpg"
OUTPUT_IMG = "result_viz.jpg"
DATA_FILE = "detection_results.json"
TARGET_CACHE = "_target_cache.json"

# PP-OCRv5 attempt — will gracefully fall back if not supported by your version
TRY_PP_OCRV5 = True

# SigLIP2
SIGLIP_MODEL = "google/siglip2-base-patch16-384"
SIGLIP_THRESHOLD = 0.80
N_STRIPS = 10
STRIP_OVERLAP = 0.5

# OCR matching
OCR_FUZZY_THRESHOLD = 70
MIN_KEYWORD_LEN = 3

# This is kept only for clarity in your project.
# Newer PaddleOCR versions do NOT accept use_gpu=...
# Torch/SigLIP still automatically uses CUDA if available.
USE_GPU = False


# ─────────────────────────────────────────────────────────────────────────────
# PREPROCESSING
# ─────────────────────────────────────────────────────────────────────────────

def preprocess(bgr):
    """
    Improve the image before OCR:
      - white balance correction
      - contrast enhancement using CLAHE
      - sharpening
    """
    avg = bgr.reshape(-1, 3).mean(0)
    gain = avg.mean() / (avg + 1e-6)
    bgr = np.clip(bgr * gain, 0, 255).astype(np.uint8)

    lab = cv2.cvtColor(bgr, cv2.COLOR_BGR2LAB)
    clahe = cv2.createCLAHE(clipLimit=2.0, tileGridSize=(8, 8))
    lab[:, :, 0] = clahe.apply(lab[:, :, 0])
    bgr = cv2.cvtColor(lab, cv2.COLOR_LAB2BGR)

    blur = cv2.GaussianBlur(bgr, (0, 0), 1.5)
    return cv2.addWeighted(bgr, 1.5, blur, -0.5, 0)


# ─────────────────────────────────────────────────────────────────────────────
# OCR INITIALIZATION
# ─────────────────────────────────────────────────────────────────────────────

def init_ocr():
    """
    Initialize PaddleOCR.

    Important:
    New PaddleOCR versions no longer accept:
        use_gpu=...
        det_db_thresh=...
        det_db_box_thresh=...
        det_limit_side_len=...

    So we use the newer names:
        text_det_thresh
        text_det_box_thresh
        text_det_limit_side_len
    """

    base_args = dict(
        use_textline_orientation=True,
        lang="en",
        text_det_thresh=0.05,
        text_det_box_thresh=0.3,
        text_det_limit_side_len=2592,
    )

    if TRY_PP_OCRV5:
        attempts = [
            {
                **base_args,
                "ocr_version": "PP-OCRv5",
            },
            {
                **base_args,
                "text_detection_model_name": "PP-OCRv5_server_det",
                "text_recognition_model_name": "PP-OCRv5_server_rec",
            },
            {
                **base_args,
                "text_detection_model_name": "PP-OCRv5_mobile_det",
                "text_recognition_model_name": "PP-OCRv5_mobile_rec",
            },
        ]

        for i, kwargs in enumerate(attempts, start=1):
            try:
                ocr = PaddleOCR(**kwargs)
                extra_keys = [k for k in kwargs.keys() if k not in base_args]
                print(f"  ✓ Using PP-OCRv5 attempt {i}: {extra_keys}")
                return ocr
            except Exception as e:
                print(f"  ✗ PP-OCRv5 attempt {i} failed: {type(e).__name__}: {e}")

    print("  → Falling back to PaddleOCR default")
    return PaddleOCR(**base_args)


# ─────────────────────────────────────────────────────────────────────────────
# OCR RESULT NORMALIZATION
# ─────────────────────────────────────────────────────────────────────────────

def normalize_ocr_result(result):
    """
    Convert PaddleOCR output into a simple list:
        [
            {
                "text": str,
                "score": float,
                "box": [[x1, y1], [x2, y2], [x3, y3], [x4, y4]]
            },
            ...
        ]

    This supports both:
      - older PaddleOCR output:
            [[box, (text, score)], ...]
      - newer PaddleOCR output:
            dict/list containing rec_texts, rec_scores, rec_polys/dt_polys/rec_boxes
    """

    items = []

    if result is None:
        return items

    # Newer PaddleOCR sometimes returns a dict or a list of dicts.
    if isinstance(result, dict):
        result = [result]

    if isinstance(result, list):
        for block in result:
            # New-style dict output
            if isinstance(block, dict):
                texts = (
                    block.get("rec_texts")
                    or block.get("texts")
                    or block.get("text")
                    or []
                )
                scores = (
                    block.get("rec_scores")
                    or block.get("scores")
                    or block.get("score")
                    or []
                )
                boxes = (
                    block.get("rec_polys")
                    or block.get("dt_polys")
                    or block.get("rec_boxes")
                    or block.get("boxes")
                    or []
                )

                if isinstance(texts, str):
                    texts = [texts]
                if isinstance(scores, (float, int)):
                    scores = [scores]

                for i, text in enumerate(texts):
                    score = float(scores[i]) if i < len(scores) else 0.0
                    box = boxes[i] if i < len(boxes) else None
                    box = normalize_box(box)
                    items.append({
                        "text": str(text),
                        "score": score,
                        "box": box,
                    })

            # Old-style list output
            elif isinstance(block, list):
                for line in block:
                    parsed = parse_old_style_line(line)
                    if parsed is not None:
                        items.append(parsed)

    return items


def parse_old_style_line(line):
    """
    Parse old PaddleOCR line:
        [box, (text, score)]
    """
    try:
        box = line[0]
        text = line[1][0]
        score = line[1][1]
        return {
            "text": str(text),
            "score": float(score),
            "box": normalize_box(box),
        }
    except Exception:
        return None


def normalize_box(box):
    """
    Convert different PaddleOCR box formats into 4 corner points.

    Handles:
      - 4 points: [[x,y], [x,y], [x,y], [x,y]]
      - rectangle: [x1, y1, x2, y2]
      - numpy arrays
    """
    if box is None:
        return []

    try:
        arr = np.array(box)

        # Polygon format: shape (4, 2)
        if arr.ndim == 2 and arr.shape[1] == 2:
            return [[int(p[0]), int(p[1])] for p in arr]

        # Rectangle format: [x1, y1, x2, y2]
        if arr.size == 4:
            x1, y1, x2, y2 = arr.flatten().tolist()
            return [
                [int(x1), int(y1)],
                [int(x2), int(y1)],
                [int(x2), int(y2)],
                [int(x1), int(y2)],
            ]

    except Exception:
        pass

    return []


# ─────────────────────────────────────────────────────────────────────────────
# TARGET CACHE
# ─────────────────────────────────────────────────────────────────────────────

def get_target_keywords(ocr, target_path):
    """
    OCR the target book cover once, then cache the extracted keywords.
    """
    target_mtime = os.path.getmtime(target_path)

    if os.path.exists(TARGET_CACHE):
        try:
            with open(TARGET_CACHE, "r", encoding="utf-8") as f:
                cache = json.load(f)

            if cache.get("mtime") == target_mtime and cache.get("keywords"):
                print(f"         Keywords (cached): {cache['keywords']}")
                return cache["keywords"]
        except Exception:
            pass

    target_bgr = cv2.imread(target_path)
    if target_bgr is None:
        raise FileNotFoundError(target_path)

    proc = preprocess(target_bgr)
    temp_path = "_target_proc.jpg"
    cv2.imwrite(temp_path, proc)

    result = ocr.ocr(temp_path)
    ocr_items = normalize_ocr_result(result)

    texts = [item["text"] for item in ocr_items if item["text"].strip()]
    keywords = [
        k.lower().strip()
        for k in texts
        if len(k.strip()) >= MIN_KEYWORD_LEN
    ]

    if os.path.exists(temp_path):
        os.remove(temp_path)

    with open(TARGET_CACHE, "w", encoding="utf-8") as f:
        json.dump(
            {
                "mtime": target_mtime,
                "keywords": keywords,
            },
            f,
            ensure_ascii=False,
            indent=4,
        )

    print(f"         Keywords (fresh): {keywords}")
    return keywords


# ─────────────────────────────────────────────────────────────────────────────
# WORKER 1: OCR SHELF SCAN
# ─────────────────────────────────────────────────────────────────────────────

def ocr_worker(ocr, shelf_proc_path, keywords):
    """
    Run shelf OCR + fuzzy matching.

    Returns:
      dict with OCR results, matched boxes, and elapsed time.
    """
    t = time.time()

    result = ocr.ocr(shelf_proc_path)
    ocr_items = normalize_ocr_result(result)

    shelf_texts = [item["text"] for item in ocr_items]
    shelf_boxes = [item["box"] for item in ocr_items]
    shelf_confs = [item["score"] for item in ocr_items]

    ocr_matches = []
    matched_idxs = set()

    for i, s_text in enumerate(shelf_texts):
        s_text_clean = s_text.lower().strip()

        for kw in keywords or []:
            kw_clean = kw.lower().strip()
            if not kw_clean:
                continue

            score = fuzz.partial_ratio(kw_clean, s_text_clean)

            if score >= OCR_FUZZY_THRESHOLD:
                ocr_matches.append({
                    "found_text": s_text,
                    "keyword": kw,
                    "fuzzy_score": round(score, 2),
                    "ocr_confidence": round(float(shelf_confs[i]), 4),
                    "box_points": shelf_boxes[i],
                })
                matched_idxs.add(i)
                break

    return {
        "elapsed_ms": (time.time() - t) * 1000,
        "matched": len(ocr_matches) > 0,
        "matches": ocr_matches,
        "n_regions": len(shelf_texts),
        "boxes": shelf_boxes,
        "texts": shelf_texts,
        "confidences": shelf_confs,
        "matched_idxs": matched_idxs,
    }


# ─────────────────────────────────────────────────────────────────────────────
# WORKER 2: SIGLIP2 SLIDING WINDOW
# ─────────────────────────────────────────────────────────────────────────────

def build_prompt(keywords):
    """
    Build the text prompt used by SigLIP2.
    """
    stopwords = {"the", "of", "a", "an", "by"}
    content = [k for k in keywords if k.lower() not in stopwords]

    if content:
        title = " ".join(w.capitalize() for w in content[:4])
    else:
        title = " ".join(keywords[:3])

    return f"a book spine titled '{title}'"


def make_strips(bgr, overlap):
    """
    Split the shelf image into vertical overlapping strips.
    """
    h, w = bgr.shape[:2]

    sw = int(w * 0.15)
    sw = max(1, sw)

    stride = max(1, int(sw * (1 - overlap)))
    strips = []

    x = 0
    while x + sw <= w:
        crop = cv2.cvtColor(bgr[:, x:x + sw], cv2.COLOR_BGR2RGB)
        strips.append((Image.fromarray(crop), x, x + sw))
        x += stride

    if strips:
        last_x1 = strips[-1][1]
    else:
        last_x1 = -1

    if w - sw > last_x1:
        crop = cv2.cvtColor(bgr[:, w - sw:w], cv2.COLOR_BGR2RGB)
        strips.append((Image.fromarray(crop), w - sw, w))

    return strips


def siglip_worker(model, processor, shelf_bgr, prompt, device):
    """
    Run SigLIP2 sliding window across shelf.
    """
    t = time.time()

    strips = make_strips(shelf_bgr, STRIP_OVERLAP)

    if not strips:
        return {
            "elapsed_ms": (time.time() - t) * 1000,
            "max_prob": 0.0,
            "matched": False,
            "best_x1": 0,
            "best_x2": 0,
            "n_strips": 0,
        }

    images = [s[0] for s in strips]
    probs_all = []

    batch_size = 16

    for i in range(0, len(images), batch_size):
        batch = images[i:i + batch_size]

        inputs = processor(
            text=[prompt],
            images=batch,
            return_tensors="pt",
            padding="max_length",
        ).to(device)

        if device == "cuda":
            inputs = {
                k: (v.half() if hasattr(v, "is_floating_point") and v.is_floating_point() else v)
                for k, v in inputs.items()
            }

        with torch.inference_mode():
            out = model(**inputs)
            probs = torch.sigmoid(out.logits_per_image).float()

        probs_all.extend(probs[:, 0].cpu().tolist())

    best = int(np.argmax(probs_all))
    max_prob = float(probs_all[best])

    return {
        "elapsed_ms": (time.time() - t) * 1000,
        "max_prob": max_prob,
        "matched": max_prob >= SIGLIP_THRESHOLD,
        "best_x1": strips[best][1],
        "best_x2": strips[best][2],
        "n_strips": len(strips),
    }


# ─────────────────────────────────────────────────────────────────────────────
# VISUALIZATION
# ─────────────────────────────────────────────────────────────────────────────

def save_visualization(shelf_bgr, ocr_result, siglip_result, siglip_matched):
    """
    Save an image showing:
      - OCR boxes in red
      - matched OCR boxes in green
      - SigLIP best strip in blue/orange if matched
    """
    img_viz = shelf_bgr.copy()

    for i, box in enumerate(ocr_result["boxes"]):
        if not box:
            continue

        pts = np.array(box, np.int32)
        is_match = i in ocr_result["matched_idxs"]

        color = (0, 255, 0) if is_match else (0, 0, 255)
        thickness = 3 if is_match else 1

        cv2.polylines(img_viz, [pts], True, color, thickness)

    if siglip_matched:
        cv2.rectangle(
            img_viz,
            (int(siglip_result["best_x1"]), 0),
            (int(siglip_result["best_x2"]), shelf_bgr.shape[0]),
            (255, 100, 0),
            2,
        )

    cv2.imwrite(OUTPUT_IMG, img_viz)


def save_json_results(
    book_found,
    confidence,
    reason,
    total_ms,
    ocr_result,
    siglip_result,
    parallel_ms,
    keywords,
):
    """
    Save machine-readable detection results.
    """
    with open(DATA_FILE, "w", encoding="utf-8") as f:
        json.dump(
            {
                "book_found": book_found,
                "confidence": confidence,
                "reason": reason,
                "total_ms": round(total_ms, 1),
                "ocr_matched": ocr_result["matched"],
                "siglip_matched": siglip_result["matched"],
                "siglip_max": round(siglip_result["max_prob"], 4),
                "ocr_elapsed_ms": round(ocr_result["elapsed_ms"], 1),
                "siglip_elapsed_ms": round(siglip_result["elapsed_ms"], 1),
                "parallel_ms": round(parallel_ms, 1),
                "keywords": keywords,
                "ocr_matches": ocr_result["matches"],
                "ocr_all_texts": ocr_result["texts"],
            },
            f,
            ensure_ascii=False,
            indent=4,
        )


# ─────────────────────────────────────────────────────────────────────────────
# MAIN
# ─────────────────────────────────────────────────────────────────────────────

def main():
    t_start = time.time()
    device = "cuda" if torch.cuda.is_available() else "cpu"

    def log(label, t0, extra=""):
        ms = (time.time() - t0) * 1000
        print(f"  [{time.time() - t_start:6.3f}s] {label:<35} → {ms:6.0f} ms  {extra}")

    print("=" * 68)
    print("  BOOK HUNTER  FINAL v4  —  PaddleOCR API Fix + Parallel Scan")
    print("=" * 68)
    print(f"  Target: {TARGET_IMG}  |  Shelf: {SHELF_IMG}  |  Device: {device}")
    print()

    # ── Load images ──────────────────────────────────────────
    t = time.time()

    target_bgr = cv2.imread(TARGET_IMG)
    shelf_bgr = cv2.imread(SHELF_IMG)

    if target_bgr is None:
        raise FileNotFoundError(f"Target image not found: {TARGET_IMG}")

    if shelf_bgr is None:
        raise FileNotFoundError(f"Shelf image not found: {SHELF_IMG}")

    log(
        "Load images",
        t,
        f"target={target_bgr.shape[:2]} shelf={shelf_bgr.shape[:2]}",
    )

    # ── Preprocess shelf ─────────────────────────────────────
    t = time.time()

    shelf_proc = preprocess(shelf_bgr)
    shelf_proc_path = "_shelf_proc.jpg"
    cv2.imwrite(shelf_proc_path, shelf_proc)

    log("Preprocess shelf", t)

    # ── Load models ──────────────────────────────────────────
    print()
    print("  ── Model loading ─────────────────────────────────────────")

    t = time.time()
    ocr = init_ocr()
    log("OCR model load", t)

    t = time.time()
    keywords = get_target_keywords(ocr, TARGET_IMG)
    log("Target keywords", t)

    if not keywords:
        print("  ERROR: No keywords were extracted from the target image.")
        print("  Try using a clearer target image or manually set keywords.")
        cleanup_temp_files([shelf_proc_path])
        return

    t = time.time()
    siglip_processor = AutoProcessor.from_pretrained(SIGLIP_MODEL)
    siglip_model = AutoModel.from_pretrained(SIGLIP_MODEL).to(device).eval()

    if device == "cuda":
        siglip_model = siglip_model.half()

    log("SigLIP2 model load", t)

    prompt = build_prompt(keywords)
    print(f"  SigLIP2 prompt: \"{prompt}\"")

    # ── Parallel scan ────────────────────────────────────────
    print()
    print("  ── Parallel scan: OCR + SigLIP2 ──────────────────────────")

    t_parallel = time.time()

    with concurrent.futures.ThreadPoolExecutor(max_workers=2) as executor:
        ocr_future = executor.submit(
            ocr_worker,
            ocr,
            shelf_proc_path,
            keywords,
        )

        siglip_future = executor.submit(
            siglip_worker,
            siglip_model,
            siglip_processor,
            shelf_proc,
            prompt,
            device,
        )

        ocr_result = ocr_future.result()
        siglip_result = siglip_future.result()

    parallel_ms = (time.time() - t_parallel) * 1000

    print(
        f"  [{time.time() - t_start:6.3f}s] OCR thread:    "
        f"{ocr_result['elapsed_ms']:6.0f} ms  "
        f"({ocr_result['n_regions']} text regions, "
        f"{'MATCH ✓' if ocr_result['matched'] else 'no match'})"
    )

    print(
        f"  [{time.time() - t_start:6.3f}s] SigLIP2 thread: "
        f"{siglip_result['elapsed_ms']:6.0f} ms  "
        f"({siglip_result['n_strips']} strips, "
        f"max={siglip_result['max_prob']:.4f}, "
        f"{'MATCH ✓' if siglip_result['matched'] else 'no match'})"
    )

    print(
        f"  [{time.time() - t_start:6.3f}s] WALL-CLOCK parallel time: "
        f"{parallel_ms:6.0f} ms  "
        f"(would be ~{ocr_result['elapsed_ms'] + siglip_result['elapsed_ms']:.0f} ms sequential)"
    )

    if ocr_result["matches"]:
        for match in ocr_result["matches"]:
            print(
                f"         OCR HIT: '{match['keyword']}' ≈ "
                f"'{match['found_text']}' "
                f"(fuzzy={match['fuzzy_score']}, "
                f"ocr_conf={match['ocr_confidence']})"
            )

    # ── Verdict + confidence ─────────────────────────────────
    ocr_matched = ocr_result["matched"]
    siglip_matched = siglip_result["matched"]

    book_found = ocr_matched or siglip_matched

    if ocr_matched and siglip_matched:
        confidence = "HIGH"
        reason = "both OCR and SigLIP2 agree"
    elif ocr_matched and not siglip_matched:
        confidence = "MEDIUM"
        reason = "OCR matched, SigLIP2 did not — possibly different cover edition"
    elif siglip_matched and not ocr_matched:
        confidence = "MEDIUM"
        reason = "SigLIP2 matched, OCR did not — possibly stylized or unreadable text"
    else:
        if siglip_result["max_prob"] < 0.6:
            confidence = "HIGH"
            reason = "both systems agree the book is likely not on the shelf"
        else:
            confidence = "MEDIUM"
            reason = "no match, but SigLIP2 was borderline — take another photo to verify"

    total_ms = (time.time() - t_start) * 1000

    print()
    print("=" * 68)
    print(f"  VERDICT:  {'BOOK FOUND' if book_found else 'BOOK NOT ON SHELF'}")
    print(f"  Confidence: {confidence}  —  {reason}")
    print("=" * 68)
    print(f"  OCR:     {'MATCH ✓' if ocr_matched else 'no match'}")
    print(
        f"  SigLIP2: {'MATCH ✓' if siglip_matched else 'no match'}  "
        f"(prob={siglip_result['max_prob']:.4f}, threshold={SIGLIP_THRESHOLD})"
    )
    print()
    print(f"  Total time: {total_ms:.0f} ms")
    print("=" * 68)

    # ── Save outputs ─────────────────────────────────────────
    save_visualization(
        shelf_bgr=shelf_bgr,
        ocr_result=ocr_result,
        siglip_result=siglip_result,
        siglip_matched=siglip_matched,
    )

    save_json_results(
        book_found=book_found,
        confidence=confidence,
        reason=reason,
        total_ms=total_ms,
        ocr_result=ocr_result,
        siglip_result=siglip_result,
        parallel_ms=parallel_ms,
        keywords=keywords,
    )

    print(f"  {OUTPUT_IMG}  +  {DATA_FILE}  saved.")

    cleanup_temp_files([shelf_proc_path])


def cleanup_temp_files(paths):
    """
    Delete temporary files safely.
    """
    for path in paths:
        try:
            if os.path.exists(path):
                os.remove(path)
        except Exception:
            pass


if __name__ == "__main__":
    main()
