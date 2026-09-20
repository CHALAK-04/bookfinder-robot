"""
==============================================================================
PC SERVER  (runs on the LAPTOP)  -  Phase 1: Z Calibration + Verification
==============================================================================

OVERALL FLOW:
  1. Start TCP server, wait for Pi to connect.
  2. CALIBRATION PHASE: tell Pi to start, receive photos, detect AprilTags,
     compute optimal Z per row. Done when all expected rows are found
     (or max-z reached).
  3. VERIFICATION PHASE (NEW): interactive. User picks a row, robot drives
     there, user visually confirms. Repeat. User types '.' to finish.
  4. CLEANUP: robot returns to z=0. Photo folder is deleted (saves disk
     space) -- but only on full success. On failure we keep them for debug.

PROTOCOL (PC side):
  We send:
    {"cmd":"START_CALIB","expected_rows":3,"step_size":100}
    {"cmd":"CONTINUE","step_size":100}
    {"cmd":"CALIB_DONE","z_positions":{...}}              -- ends calib phase
    {"cmd":"CALIB_ERROR","reason":"...", ...}             -- aborts session
    {"cmd":"GOTO_ROW","tag_id":N,"target_z":N}            -- NEW (verify)
    {"cmd":"RETURN_HOME"}                                 -- NEW (verify)
    {"cmd":"EXIT"}                                        -- NEW (shutdown)

  We receive:
    {"type":"CALIB_PHOTO","seq":N,"z_steps":N,"len":N} + binary JPEG
    {"type":"CALIB_MAX_Z","z_steps":N}
    {"type":"CALIB_ERROR","reason":"..."}
    {"type":"ARRIVED","tag_id":N,"current_z":N}           -- NEW
    {"type":"AT_HOME","current_z":0}                      -- NEW

USAGE:
    python pc_server.py
==============================================================================
"""

import socket
import os
import shutil
from datetime import datetime

from protocol import MessageStream
from apriltag_processor import (
    process_calib_photo,
    compute_final_z_positions,
    CAMERA_PARAMS_DESCRIPTION,
)


# ============================================================================
# CONFIGURATION
# ============================================================================
HOST = '0.0.0.0'              # listen on every network interface
PORT = 5000                   # both sides must use the same port
EXPECTED_ROWS = 4             # how many shelves should we find?
INITIAL_STEP_SIZE = 100       # stepper steps between consecutive captures
MIN_DETECTIONS_PER_TAG = 3    # need this many photos before trusting a row
ZPHOTOS_DIR = 'Z-photos'      # folder where calibration photos are saved
OUTPUT_DIR  = 'output'         # folder where verification photos are saved
                               # (one per row: row_1.jpg, row_2.jpg, ...)
RUN_VERIFICATION = True       # set False to skip the interactive verify step
DELETE_PHOTOS_ON_SUCCESS = True  # auto-delete photo folder on full success


# ============================================================================
# MAIN
# ============================================================================
def main():
    # Per-session folder so we never overwrite previous runs.
    session_dir = os.path.join(
        ZPHOTOS_DIR,
        datetime.now().strftime("%Y%m%d_%H%M%S"),
    )
    os.makedirs(session_dir, exist_ok=True)
    print(f"Saving photos to: {session_dir}")
    print(f"Camera intrinsics: {CAMERA_PARAMS_DESCRIPTION}")

    server_sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    server_sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    server_sock.bind((HOST, PORT))
    server_sock.listen(1)
    print(f"\nWaiting for Pi to connect on port {PORT}...")

    pi_sock, addr = server_sock.accept()
    print(f"Pi connected from {addr}\n")
    stream = MessageStream(pi_sock)

    try:
        # --- Phase 1a: calibration ---
        result = run_calibration(stream, session_dir)
        if result is None:
            print("Calibration aborted. Photo folder kept for debugging:")
            print(f"  {session_dir}")
            return

        final_z = result   # dict {tag_id: z_steps}

        # --- Phase 1b: verification ---
        if RUN_VERIFICATION:
            run_verification(stream, final_z)

        # --- Clean shutdown of Pi side ---
        try:
            stream.send_message({"cmd": "EXIT"})
        except (OSError, BrokenPipeError):
            pass  # Pi might already be gone

        # --- Cleanup photos on success ---
        if DELETE_PHOTOS_ON_SUCCESS:
            cleanup_photos(session_dir)

    finally:
        stream.close()
        server_sock.close()


# ============================================================================
# CALIBRATION PHASE
# ============================================================================
def run_calibration(stream, photo_dir):
    """
    Run the scan loop. Returns the final {tag_id: z_steps} dict on success,
    or None if calibration was aborted before all rows were found.
    """
    print("=" * 60)
    print("Phase 1a: Z calibration scan")
    print("=" * 60)
    stream.send_message({
        "cmd": "START_CALIB",
        "expected_rows": EXPECTED_ROWS,
        "step_size": INITIAL_STEP_SIZE,
    })

    tag_estimates = {}            # {tag_id: [z_est_1, z_est_2, ...]}
    last_seen_tags = set()
    rows_completed = set()

    while True:
        msg, payload = stream.recv_message()
        if msg is None:
            print("Pi disconnected unexpectedly.")
            return None

        msg_type = msg.get('type')

        # -------- CALIB_PHOTO --------
        if msg_type == 'CALIB_PHOTO':
            seq = msg['seq']
            z_steps = msg['z_steps']

            # Save JPEG to disk for offline inspection (deleted on success)
            path = os.path.join(photo_dir, f"{seq:03d}_z{z_steps}.jpg")
            with open(path, 'wb') as f:
                f.write(payload)

            # Detect tags + accumulate Z estimates
            detections = process_calib_photo(payload, z_steps)
            current_tags = set()
            for tag_id, z_optimal_estimate in detections:
                tag_estimates.setdefault(tag_id, []).append(z_optimal_estimate)
                current_tags.add(tag_id)

            tag_str = f"tags={sorted(current_tags)}" if current_tags else "no tags"
            print(
                f"Photo {seq:3d} | z={z_steps:5d} | "
                f"{len(payload) / 1024:6.1f} KB | {tag_str}"
            )

            # "Row done" = tag just disappeared after being seen, with enough samples
            lost_tags = last_seen_tags - current_tags
            for tag_id in lost_tags:
                already_done = tag_id in rows_completed
                enough_samples = len(tag_estimates[tag_id]) >= MIN_DETECTIONS_PER_TAG
                if not already_done and enough_samples:
                    rows_completed.add(tag_id)
                    samples = tag_estimates[tag_id]
                    z_opt = int(round(sum(samples) / len(samples)))
                    print(
                        f"   --> Row tag={tag_id} DONE: "
                        f"{len(samples)} samples, optimal_z={z_opt} steps"
                    )
            last_seen_tags = current_tags

            # All rows found?
            if len(rows_completed) >= EXPECTED_ROWS:
                final_z = compute_final_z_positions(tag_estimates)
                stream.send_message({"cmd": "CALIB_DONE", "z_positions": final_z})
                print()
                print("=" * 60)
                print("Calibration COMPLETE")
                print("=" * 60)
                print(f"Final Z positions: {final_z}")
                return final_z

            # Not done -> Pi elevates and captures again
            stream.send_message({
                "cmd": "CONTINUE",
                "step_size": INITIAL_STEP_SIZE,
            })

        # -------- CALIB_MAX_Z --------
        elif msg_type == 'CALIB_MAX_Z':
            print(f"\nPi reports max Z reached at z={msg.get('z_steps')}.")
            final_z = compute_final_z_positions(tag_estimates)
            if len(final_z) < EXPECTED_ROWS:
                missing = EXPECTED_ROWS - len(final_z)
                print(f"WARNING: only found {len(final_z)}/{EXPECTED_ROWS} rows.")
                print(f"Partial result: {final_z}")
                stream.send_message({
                    "cmd": "CALIB_ERROR",
                    "reason": f"missing_{missing}_rows",
                    "partial_z_positions": final_z,
                })
                return None   # calibration failed
            else:
                stream.send_message({"cmd": "CALIB_DONE", "z_positions": final_z})
                print(f"Final Z positions: {final_z}")
                return final_z

        # -------- CALIB_ERROR (Pi side) --------
        elif msg_type == 'CALIB_ERROR':
            print(f"Pi reports error: {msg.get('reason')}")
            return None

        else:
            print(f"Unexpected message type from Pi: {msg_type}")


# ============================================================================
# VERIFICATION PHASE  (NEW)
# ============================================================================
def run_verification(stream, final_z):
    """
    Interactive: user picks a row by number, robot drives there, takes a
    photo, and sends it back. Photo is saved in OUTPUT_DIR as row_N.jpg
    so the user can verify centering from their laptop.
    User types '.' to finish -> robot returns to z=0 then program exits.
    """
    # Create output folder for verification photos
    os.makedirs(OUTPUT_DIR, exist_ok=True)

    # Sort tags by Z ascending: lowest tag = row 1 (first physical shelf).
    sorted_tags = sorted(final_z.items(), key=lambda kv: kv[1])
    # 1-indexed: row_table[row_number] = (tag_id, z_steps)
    row_table = {i + 1: (tag_id, z) for i, (tag_id, z) in enumerate(sorted_tags)}

    print()
    print("=" * 60)
    print("Phase 1b: Verification (visually confirm each row)")
    print("=" * 60)
    print(f"Verification photos will be saved to: {OUTPUT_DIR}/")
    print()
    print("Calibrated rows:")
    for row_num, (tag_id, z) in row_table.items():
        print(f"  Row {row_num}  ->  tag {tag_id}  ->  z = {z} steps")
    print()
    print("Type a row number to move there and capture a photo.")
    print("Type '.' (period) to finish: robot returns home and program exits.")
    print()

    while True:
        try:
            choice = input("Select row (or . to quit): ").strip()
        except (EOFError, KeyboardInterrupt):
            print()
            break

        if choice == '.':
            break

        if not choice.isdigit():
            print("  (not a number - type 1, 2, 3, ... or '.')")
            continue

        row_num = int(choice)
        if row_num not in row_table:
            valid = ", ".join(str(k) for k in row_table.keys())
            print(f"  (no such row - valid options: {valid})")
            continue

        tag_id, target_z = row_table[row_num]
        print(f"  Moving to row {row_num} (tag {tag_id}, z={target_z})...")
        stream.send_message({
            "cmd": "GOTO_ROW",
            "tag_id": tag_id,
            "target_z": target_z,
        })

        # Wait for ARRIVED + verification photo
        msg, photo = stream.recv_message()
        if msg is None:
            print("  Pi disconnected during move.")
            return
        if msg.get('type') == 'ARRIVED':
            print(f"  Arrived. Camera now at z={msg.get('current_z')}.")
            if photo:
                photo_path = os.path.join(OUTPUT_DIR, f"row_{row_num}.jpg")
                with open(photo_path, 'wb') as f:
                    f.write(photo)
                print(f"  Photo saved: {photo_path}  ({len(photo)/1024:.1f} KB)")
                print(f"  --> Open {photo_path} to check if row {row_num} is centered.")
            else:
                print("  (no photo received)")
        else:
            print(f"  Unexpected response: {msg}")

    # User said done -> drive back to z=0
    print("\nReturning to home (z=0)...")
    stream.send_message({"cmd": "RETURN_HOME"})
    msg, _ = stream.recv_message()
    if msg is not None and msg.get('type') == 'AT_HOME':
        print(f"Pi at home (z={msg.get('current_z')}). Verification finished.")
    else:
        print(f"Did not get AT_HOME confirmation. Got: {msg}")


# ============================================================================
# CLEANUP
# ============================================================================
def cleanup_photos(photo_dir):
    """Delete the per-session photo folder to save disk space."""
    try:
        shutil.rmtree(photo_dir)
        print(f"Deleted photo folder: {photo_dir}")
    except OSError as e:
        print(f"Could not delete {photo_dir}: {e}")


if __name__ == '__main__':
    main()
