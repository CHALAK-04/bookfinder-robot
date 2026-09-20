#!/usr/bin/env python3
"""
FINAL PC SERVER for the Book-Finder Robot.

Flow:
1) Loads the book detector ONCE and keeps it alive.
2) Runs the existing Phase-1 Z calibration.
3) Starts a TOP-DOWN serpentine shelf scan at 1080p.
4) Ranks all detections, deduplicates overlap, and asks the user which result to go to.
5) Sends GO_TO_RESULT to the Pi.

Run on laptop:
    python final_pc_server.py
"""
import json
import os
import socket
from dataclasses import dataclass, asdict
from datetime import datetime
from typing import Dict, List, Optional, Tuple

import cv2
import numpy as np

from protocol import MessageStream
from apriltag_processor import process_calib_photo, compute_final_z_positions, CAMERA_PARAMS_DESCRIPTION
from book_detector_service import PersistentBookDetector

try:
    from pupil_apriltags import Detector
except Exception:
    Detector = None

_END_TAG_DETECTOR = Detector(families="tag36h11") if Detector is not None else None

HOST = "0.0.0.0"
PORT = 5000

# Z calibration
INITIAL_STEP_SIZE = 100
MIN_DETECTIONS_PER_TAG = 3
MAX_CALIB_PHOTOS = None

# Scan behavior. 12 cm gives strong overlap for typical 1080p shelf photos.
# Increase to 15-18 for faster scan, decrease to 8-10 if books are missed.
SCAN_STEP_CM = 12.0
END_FINE_CM = 3.0
SETTLE_SECONDS = 2.0
BOOK_PHOTO_WIDTH = 1920
BOOK_PHOTO_HEIGHT = 1080
TAG_PHOTO_WIDTH = 1280
TAG_PHOTO_HEIGHT = 720
END_TAG_CENTER_TOL_PX = 120
START_EDGE_TOL_CM = 0.75
MAX_SCAN_CM = 300.0  # hard safety if first row end tag is missed

# Overlap deduplication: if two detections are on the same row and their
# capture positions are close enough, treat them as the same physical book.
# 1.25 * SCAN_STEP_CM intentionally merges detections from consecutive
# overlapping photos while keeping clearly separate shelf areas as separate hits.
OVERLAP_MERGE_FACTOR = 1.25

RESULTS_DIR = "scan_results"
ZPHOTOS_DIR = "Z-photos"
Z_POS_FILE_COPY = "z_positions_final.json"


@dataclass
class RankedHit:
    row_tag: str
    row_index_top_down: int
    x_cm: float
    direction: str
    photo_seq: int
    confidence_label: str
    confidence_score: float
    detector: dict
    image_path: str
    viz_path: str


def ask_int(prompt: str, default: int) -> int:
    raw = input(f"{prompt} [{default}]: ").strip()
    return default if not raw else int(raw)


def ask_str(prompt: str, default: str) -> str:
    raw = input(f"{prompt} [{default}]: ").strip()
    return default if not raw else raw


def listen_for_pi() -> Tuple[MessageStream, socket.socket]:
    server_sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    server_sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    server_sock.bind((HOST, PORT))
    server_sock.listen(1)
    print(f"Waiting for Raspberry Pi on port {PORT} ...")
    pi_sock, addr = server_sock.accept()
    print(f"Pi connected from {addr}")
    return MessageStream(pi_sock), server_sock


def run_z_calibration(stream: MessageStream, expected_rows: int, session_dir: str, end_tag_id: int) -> Optional[Dict[str, int]]:
    print("\n" + "=" * 72)
    print("PHASE 1 — Z calibration")
    print(f"Camera intrinsics: {CAMERA_PARAMS_DESCRIPTION}")
    print("=" * 72)

    os.makedirs(session_dir, exist_ok=True)
    stream.send_message({
        "cmd": "START_CALIB",
        "expected_rows": expected_rows,
        "step_size": INITIAL_STEP_SIZE,
        "resolution": [TAG_PHOTO_WIDTH, TAG_PHOTO_HEIGHT],
    })

    tag_estimates: Dict[int, List[int]] = {}
    last_seen = set()
    rows_completed = set()
    n_photos = 0

    while True:
        msg, payload = stream.recv_message()
        if msg is None:
            print("Pi disconnected during calibration.")
            return None
        typ = msg.get("type")

        if typ == "CALIB_PHOTO":
            n_photos += 1
            seq = int(msg["seq"])
            z_steps = int(msg["z_steps"])
            img_path = os.path.join(session_dir, f"{seq:03d}_z{z_steps}.jpg")
            with open(img_path, "wb") as f:
                f.write(payload or b"")

            if not payload:
                stream.send_message({"cmd": "CALIB_ERROR", "reason": "empty_calib_photo"})
                print("Calibration stopped: empty photo payload.")
                return None

            detections = process_calib_photo(payload, z_steps)
            current = set()
            for tag_id, z_est in detections:
                tag_id = int(tag_id)
                if tag_id == int(end_tag_id):
                    # The left/end tag is not a shelf-level Z tag.
                    continue
                tag_estimates.setdefault(tag_id, []).append(int(z_est))
                current.add(tag_id)

            valid_tags = {tid for tid, samples in tag_estimates.items() if len(samples) >= MIN_DETECTIONS_PER_TAG}
            print(f"calib photo={seq:03d} z={z_steps:6d} tags={sorted(current)} valid={sorted(valid_tags)} size={len(payload or b'')/1024:.1f}KB")

            lost = last_seen - current
            for tag_id in lost:
                if tag_id not in rows_completed and len(tag_estimates.get(tag_id, [])) >= MIN_DETECTIONS_PER_TAG:
                    rows_completed.add(tag_id)
                    samples = tag_estimates[tag_id]
                    z_opt = int(round(sum(samples) / len(samples)))
                    print(f"  row/tag {tag_id} DONE: {len(samples)} samples -> z={z_opt}")
            last_seen = current

            # Finish as soon as every expected right-side row tag has enough samples.
            # Do not wait for the top tag to disappear, otherwise the mast can keep
            # climbing unnecessarily after the highest level has already been measured.
            if len(valid_tags) >= expected_rows:
                valid_estimates = {tid: tag_estimates[tid] for tid in sorted(valid_tags)[:expected_rows]}
                final_z = compute_final_z_positions(valid_estimates)
                final_z = {str(k): int(v) for k, v in final_z.items()}
                stream.send_message({"cmd": "CALIB_DONE", "z_positions": final_z})
                with open(Z_POS_FILE_COPY, "w", encoding="utf-8") as f:
                    json.dump(final_z, f, indent=2)
                print(f"Z calibration complete: {final_z}")
                return final_z

            if MAX_CALIB_PHOTOS is not None and n_photos >= MAX_CALIB_PHOTOS:
                print("WARNING: many calibration photos taken, but continuing until all tags are found or MAX_Z is reached.")

            stream.send_message({"cmd": "CONTINUE", "step_size": INITIAL_STEP_SIZE})

        elif typ == "CALIB_MAX_Z":
            valid_estimates = {
                tid: samples for tid, samples in tag_estimates.items()
                if len(samples) >= MIN_DETECTIONS_PER_TAG
            }
            final_z = compute_final_z_positions(valid_estimates)
            final_z = {str(k): int(v) for k, v in final_z.items()}
            if len(final_z) < expected_rows:
                stream.send_message({"cmd": "CALIB_ERROR", "reason": "missing_rows", "partial_z_positions": final_z})
                print(f"Max Z reached, incomplete: {final_z}")
                return None
            stream.send_message({"cmd": "CALIB_DONE", "z_positions": final_z})
            with open(Z_POS_FILE_COPY, "w", encoding="utf-8") as f:
                json.dump(final_z, f, indent=2)
            return final_z

        elif typ == "CALIB_ERROR":
            print(f"Pi calibration error: {msg}")
            return None


def detect_end_tag(jpeg_bytes: bytes, end_tag_id: int) -> Tuple[bool, bool, Optional[int], int]:
    """Return (seen, centered, x_center_px, image_width)."""
    if _END_TAG_DETECTOR is None:
        return False, False, None, BOOK_PHOTO_WIDTH
    arr = np.frombuffer(jpeg_bytes, np.uint8)
    bgr = cv2.imdecode(arr, cv2.IMREAD_COLOR)
    if bgr is None:
        return False, False, None, BOOK_PHOTO_WIDTH
    gray = cv2.cvtColor(bgr, cv2.COLOR_BGR2GRAY)
    detections = _END_TAG_DETECTOR.detect(gray, estimate_tag_pose=False)
    h, w = gray.shape[:2]
    for d in detections:
        if int(d.tag_id) == int(end_tag_id):
            x = int(round(d.center[0]))
            centered = abs(x - w / 2) <= END_TAG_CENTER_TOL_PX
            return True, centered, x, w
    return False, False, None, w


def add_or_merge_hit(hits: List[RankedHit], hit: RankedHit) -> None:
    """Merge overlap duplicates: same row and nearby X -> keep stronger result."""
    for i, old in enumerate(hits):
        if old.row_tag == hit.row_tag and abs(old.x_cm - hit.x_cm) <= SCAN_STEP_CM * OVERLAP_MERGE_FACTOR:
            if hit.confidence_score > old.confidence_score:
                hits[i] = hit
            return
    hits.append(hit)


def print_ranked(hits: List[RankedHit]) -> None:
    ranked = sorted(hits, key=lambda h: h.confidence_score, reverse=True)
    print("\nCurrent ranked detections:")
    if not ranked:
        print("  no detections yet")
        return
    for idx, h in enumerate(ranked, start=1):
        print(f"  {idx:2d}) {h.confidence_score*100:5.1f}% {h.confidence_label:6s} | row tag {h.row_tag:>3s} | x={h.x_cm:7.1f} cm | photo {h.photo_seq} | {h.viz_path}")


def run_scan(stream: MessageStream, detector: PersistentBookDetector, z_positions: Dict[str, int], start_dir: str, end_tag_id: int) -> List[RankedHit]:
    print("\n" + "=" * 72)
    print("PHASE 2 — TOP-DOWN SERPENTINE SCAN")
    print("Detector stays loaded; every 1080p photo is processed immediately.")
    print("=" * 72)

    session_dir = os.path.join(RESULTS_DIR, datetime.now().strftime("%Y%m%d_%H%M%S"))
    raw_dir = os.path.join(session_dir, "raw")
    viz_dir = os.path.join(session_dir, "viz")
    os.makedirs(raw_dir, exist_ok=True)
    os.makedirs(viz_dir, exist_ok=True)

    stream.send_message({
        "cmd": "START_SCAN",
        "z_positions": z_positions,
        "start_dir": start_dir,
        "scan_step_cm": SCAN_STEP_CM,
        "fine_step_cm": END_FINE_CM,
        "settle_s": SETTLE_SECONDS,
        "max_scan_cm": MAX_SCAN_CM,
        "resolution": [BOOK_PHOTO_WIDTH, BOOK_PHOTO_HEIGHT],
    })

    hits: List[RankedHit] = []
    outward_dir = "BACK" if str(start_dir).upper().startswith("B") else "FWD"
    known_row_length_cm: Optional[float] = None
    end_seen_once: Dict[str, int] = {}

    while True:
        msg, payload = stream.recv_message()
        if msg is None:
            print("Pi disconnected during scan.")
            break
        typ = msg.get("type")

        if typ == "SCAN_PHOTO":
            seq = int(msg["seq"])
            row_tag = str(msg["row_tag"])
            row_index = int(msg.get("row_index_top_down", 0))
            x_cm = float(msg.get("x_cm", 0.0))
            direction = str(msg.get("direction", "FWD"))

            raw_path = os.path.join(raw_dir, f"scan_{seq:04d}_row{row_tag}_x{x_cm:.1f}.jpg")
            viz_path = os.path.join(viz_dir, f"scan_{seq:04d}_row{row_tag}_x{x_cm:.1f}_viz.jpg")
            with open(raw_path, "wb") as f:
                f.write(payload or b"")

            try:
                det = detector.scan_bytes(payload or b"", save_viz_path=viz_path)
            except Exception as exc:
                print(f"photo={seq:04d} row={row_tag} x={x_cm:7.1f} cm dir={direction:4s} -> detector error: {exc}")
                det = None

            if det is not None:
                print(f"photo={seq:04d} row={row_tag} x={x_cm:7.1f} cm dir={direction:4s} -> found={det.found} score={det.confidence_score*100:5.1f}% {det.confidence_label} ({det.elapsed_ms:.0f}ms)")

            if det is not None and det.found:
                hit = RankedHit(
                    row_tag=row_tag,
                    row_index_top_down=row_index,
                    x_cm=x_cm,
                    direction=direction,
                    photo_seq=seq,
                    confidence_label=det.confidence_label,
                    confidence_score=float(det.confidence_score),
                    detector=det.to_dict(),
                    image_path=raw_path,
                    viz_path=viz_path,
                )
                add_or_merge_hit(hits, hit)
                print_ranked(hits)

            # Row-end logic must respect serpentine direction.
            # The physical left/end tag is only useful while moving from the
            # start side toward the far/end side. On the return pass, the robot
            # starts next to that tag, so detecting it would otherwise end the
            # row immediately.
            moving_outward = direction.upper().startswith(outward_dir[0])

            if moving_outward and known_row_length_cm is not None:
                remaining = known_row_length_cm - x_cm
                if remaining <= START_EDGE_TOL_CM:
                    print("  known row length reached -> row end")
                    stream.send_message({"cmd": "ROW_END"})
                else:
                    step = min(SCAN_STEP_CM, max(START_EDGE_TOL_CM, remaining))
                    stream.send_message({"cmd": "CONTINUE_SCAN", "step_cm": step})
            elif moving_outward:
                seen_end, centered, tag_x, img_w = detect_end_tag(payload or b"", end_tag_id)
                if seen_end:
                    end_seen_once[row_tag] = end_seen_once.get(row_tag, 0) + 1
                if seen_end and not centered:
                    print(f"  end tag seen at x={tag_x}/{img_w}; fine-centering with {END_FINE_CM} cm step")
                    stream.send_message({"cmd": "CONTINUE_SCAN", "step_cm": END_FINE_CM})
                elif seen_end and centered:
                    print("  end tag centered -> row end")
                    stream.send_message({"cmd": "ROW_END"})
                elif end_seen_once.get(row_tag, 0) >= 1:
                    print("  end tag was seen then lost -> treating as row end to avoid overshooting")
                    stream.send_message({"cmd": "ROW_END"})
                elif x_cm >= MAX_SCAN_CM:
                    print(f"  safety max scan length {MAX_SCAN_CM:.1f} cm reached -> row end")
                    stream.send_message({"cmd": "ROW_END"})
                else:
                    stream.send_message({"cmd": "CONTINUE_SCAN", "step_cm": SCAN_STEP_CM})
            else:
                # Returning toward the start/right side. Stop by encoder X=0,
                # not by the left/end AprilTag.
                if x_cm <= START_EDGE_TOL_CM:
                    print("  returned to start side -> row end")
                    stream.send_message({"cmd": "ROW_END"})
                else:
                    step = min(SCAN_STEP_CM, max(START_EDGE_TOL_CM, x_cm))
                    stream.send_message({"cmd": "CONTINUE_SCAN", "step_cm": step})

        elif typ == "SCAN_ROW_DONE":
            print(f"Row done: {msg}")
            if known_row_length_cm is None and float(msg.get("x_cm", 0.0)) > START_EDGE_TOL_CM:
                known_row_length_cm = float(msg.get("x_cm", 0.0))
                print(f"Learned shelf row length: {known_row_length_cm:.1f} cm")

        elif typ == "SCAN_DONE":
            print("Scan complete.")
            break

        elif typ == "SCAN_ERROR":
            print(f"Scan error from Pi: {msg}")
            break

        elif typ == "STATUS":
            print(f"STATUS: {msg}")

    # Save final ranking
    ranked = sorted(hits, key=lambda h: h.confidence_score, reverse=True)
    with open(os.path.join(session_dir, "ranked_hits.json"), "w", encoding="utf-8") as f:
        json.dump([asdict(h) for h in ranked], f, indent=2)
    print_ranked(ranked)
    print(f"Saved scan session: {session_dir}")
    return ranked


def choose_and_go_to(stream: MessageStream, hits: List[RankedHit]) -> None:
    ranked = sorted(hits, key=lambda h: h.confidence_score, reverse=True)
    if not ranked:
        print("No hits to navigate to.")
        return
    while True:
        choice = input("\nChoose result number to show the book, or ENTER to skip: ").strip()
        if not choice:
            return
        if not choice.isdigit() or not (1 <= int(choice) <= len(ranked)):
            print("Invalid choice.")
            continue
        h = ranked[int(choice) - 1]
        print(f"Sending robot to result {choice}: row={h.row_tag}, x={h.x_cm:.1f} cm")
        stream.send_message({
            "cmd": "GO_TO_RESULT",
            "row_tag": h.row_tag,
            "x_cm": h.x_cm,
            "direction_at_capture": h.direction,
        })
        while True:
            msg, _ = stream.recv_message()
            if msg is None:
                print("Pi disconnected while navigating.")
                return
            print(f"NAV: {msg}")
            if msg.get("type") in {"ARRIVED_RESULT", "NAV_ERROR"}:
                return


def main():
    print("Book-Finder FINAL PC app")
    target_path = ask_str("Target book spine image path", "target_book.jpg")
    expected_rows = ask_int("Number of shelf levels / expected right-side tags", 4)
    start_dir = ask_str("First row scan direction after calibration (FWD or BACK)", "FWD").upper()
    if start_dir not in {"FWD", "BACK"}:
        start_dir = "FWD"
    end_tag_id = ask_int("Left/end-of-shelf AprilTag ID", 99)

    detector = PersistentBookDetector(target_path, use_gpu=True)
    stream, server_sock = listen_for_pi()

    try:
        calib_dir = os.path.join(ZPHOTOS_DIR, datetime.now().strftime("%Y%m%d_%H%M%S"))
        z_positions = run_z_calibration(stream, expected_rows, calib_dir, end_tag_id)
        if not z_positions:
            print("Stopping because calibration failed.")
            return
        hits = run_scan(stream, detector, z_positions, start_dir, end_tag_id)
        choose_and_go_to(stream, hits)
        stream.send_message({"cmd": "EXIT"})
    finally:
        stream.close()
        server_sock.close()


if __name__ == "__main__":
    main()
