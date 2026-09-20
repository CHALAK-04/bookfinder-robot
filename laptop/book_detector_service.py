#!/usr/bin/env python3
"""
Persistent book detector service.

Important: this keeps PaddleOCR + SigLIP loaded for the whole robot session.
Do NOT call hunt_book_FINAL_v3.py as a subprocess per photo, because that reloads
models and costs about a second or more each time.
"""
import concurrent.futures
import json
import os
import tempfile
import time
from dataclasses import dataclass
from typing import Any, Dict, List, Optional

import cv2
import numpy as np
import torch
from transformers import AutoProcessor, AutoModel

# Reuse your tested detector functions/thresholds from the uploaded script.
import hunt_book_FINAL_v3_original as hb


@dataclass
class DetectionResult:
    found: bool
    confidence_label: str
    confidence_score: float
    reason: str
    x_px: Optional[int]
    best_box: Optional[List[List[int]]]
    ocr_matches: List[Dict[str, Any]]
    siglip_max: float
    elapsed_ms: float

    def to_dict(self) -> Dict[str, Any]:
        return {
            "found": self.found,
            "confidence_label": self.confidence_label,
            "confidence_score": round(float(self.confidence_score), 3),
            "reason": self.reason,
            "x_px": self.x_px,
            "best_box": self.best_box,
            "ocr_matches": self.ocr_matches,
            "siglip_max": round(float(self.siglip_max), 4),
            "elapsed_ms": round(float(self.elapsed_ms), 1),
        }


class PersistentBookDetector:
    def __init__(self, target_image_path: str, use_gpu: bool = True):
        self.target_image_path = target_image_path
        self.device = "cuda" if (use_gpu and torch.cuda.is_available()) else "cpu"
        # PaddleOCR should not be forced into GPU mode if CUDA is unavailable.
        hb.USE_GPU = (self.device == "cuda")

        print("=" * 70)
        print("Loading book detector ONCE. Keep this process open during scanning.")
        print(f"Device: {self.device}")
        print("=" * 70)

        t0 = time.time()
        self.ocr = hb.init_ocr()
        self.keywords = hb.get_target_keywords(self.ocr, target_image_path)
        if not self.keywords:
            raise RuntimeError("Target book OCR produced no keywords. Use a clearer target spine photo.")

        self.prompt = hb.build_prompt(self.keywords)
        self.siglip_processor = AutoProcessor.from_pretrained(hb.SIGLIP_MODEL)
        self.siglip_model = AutoModel.from_pretrained(hb.SIGLIP_MODEL).to(self.device).eval()
        if self.device == "cuda":
            self.siglip_model = self.siglip_model.half()
        print(f"Target keywords: {self.keywords}")
        print(f"SigLIP prompt: {self.prompt}")
        print(f"Model load complete in {time.time() - t0:.1f}s\n")

    def scan_bytes(self, jpeg_bytes: bytes, save_viz_path: Optional[str] = None) -> DetectionResult:
        t0 = time.time()
        arr = np.frombuffer(jpeg_bytes, dtype=np.uint8)
        shelf_bgr = cv2.imdecode(arr, cv2.IMREAD_COLOR)
        if shelf_bgr is None:
            raise ValueError("Could not decode shelf JPEG bytes")
        return self.scan_bgr(shelf_bgr, save_viz_path=save_viz_path, t0=t0)

    def scan_file(self, shelf_image_path: str, save_viz_path: Optional[str] = None) -> DetectionResult:
        t0 = time.time()
        shelf_bgr = cv2.imread(shelf_image_path)
        if shelf_bgr is None:
            raise FileNotFoundError(shelf_image_path)
        return self.scan_bgr(shelf_bgr, save_viz_path=save_viz_path, t0=t0)

    def scan_bgr(self, shelf_bgr, save_viz_path: Optional[str] = None, t0: Optional[float] = None) -> DetectionResult:
        if t0 is None:
            t0 = time.time()

        shelf_proc = hb.preprocess(shelf_bgr)
        with tempfile.NamedTemporaryFile(suffix=".jpg", delete=False) as tmp:
            tmp_path = tmp.name
        cv2.imwrite(tmp_path, shelf_proc)

        try:
            with concurrent.futures.ThreadPoolExecutor(max_workers=2) as executor:
                ocr_future = executor.submit(hb.ocr_worker, self.ocr, tmp_path, self.keywords)
                siglip_future = executor.submit(
                    hb.siglip_worker,
                    self.siglip_model,
                    self.siglip_processor,
                    shelf_proc,
                    self.prompt,
                    self.device,
                )
                ocr_result = ocr_future.result()
                siglip_result = siglip_future.result()
        finally:
            try:
                os.remove(tmp_path)
            except OSError:
                pass

        ocr_m = bool(ocr_result["matched"])
        siglip_m = bool(siglip_result["matched"])
        found = ocr_m or siglip_m

        if ocr_m and siglip_m:
            label, reason = "HIGH", "OCR and SigLIP agree"
            score = 0.90 + min(0.09, max(0.0, siglip_result["max_prob"] - hb.SIGLIP_THRESHOLD) * 0.2)
        elif ocr_m:
            label, reason = "MEDIUM", "OCR matched; visual model did not confirm"
            score = 0.72
        elif siglip_m:
            label, reason = "MEDIUM", "Visual model matched; OCR did not read the title"
            score = float(siglip_result["max_prob"])
        else:
            label = "LOW"
            reason = "No reliable match"
            score = float(siglip_result["max_prob"])

        best_box = None
        x_px = None
        if ocr_result["matches"]:
            best_box = ocr_result["matches"][0].get("box_points")
            if best_box:
                xs = [p[0] for p in best_box]
                x_px = int(round(sum(xs) / len(xs)))
        elif siglip_m:
            x_px = int(round((siglip_result["best_x1"] + siglip_result["best_x2"]) / 2))

        if save_viz_path:
            img_viz = shelf_bgr.copy()
            for i, box in enumerate(ocr_result["boxes"]):
                pts = np.array(box, np.int32)
                is_m = i in ocr_result["matched_idxs"]
                color = (0, 255, 0) if is_m else (0, 0, 255)
                cv2.polylines(img_viz, [pts], True, color, 3 if is_m else 1)
            if siglip_m:
                cv2.rectangle(
                    img_viz,
                    (siglip_result["best_x1"], 0),
                    (siglip_result["best_x2"], shelf_bgr.shape[0]),
                    (255, 100, 0),
                    2,
                )
            cv2.imwrite(save_viz_path, img_viz)

        return DetectionResult(
            found=found,
            confidence_label=label,
            confidence_score=score,
            reason=reason,
            x_px=x_px,
            best_box=best_box,
            ocr_matches=ocr_result["matches"],
            siglip_max=float(siglip_result["max_prob"]),
            elapsed_ms=(time.time() - t0) * 1000,
        )
