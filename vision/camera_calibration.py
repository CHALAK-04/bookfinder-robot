"""
==============================================================================
CAMERA CALIBRATION  (runs on the LAPTOP)  -  OPTIONAL, do this LATER
==============================================================================

WHAT THIS DOES:
Computes the exact "intrinsic parameters" of your Pi Camera v1 (focal length,
optical center, lens distortion). Replaces the approximate values currently
hard-coded in apriltag_processor.py.

WHEN TO RUN THIS:
You DON'T need this to get Phase 1 working. Run it later when you want better
accuracy (e.g. ±2mm instead of ±2cm on tag positions). Required for Phase 2
if you want precise distance-to-shelf measurements for book OCR.

WHY APPROXIMATE VALUES ARE NOT GOOD ENOUGH FOREVER:
- Each individual camera module has slight variations in lens placement.
- Cheap plastic lenses have noticeable barrel distortion (~2-3% near edges).
- The datasheet FoV is approximate; real value can differ by ~5%.

STEP-BY-STEP USAGE:
1. Print a checkerboard.
   - Search: "OpenCV 9x6 checkerboard PDF"
   - "9x6" means 9 by 6 INTERNAL CORNERS, which is a 10x7 grid of squares.
   - Print on plain A4. Tape FLAT to a stiff backing (cardboard, clipboard).
     Any warp/bend ruins the calibration.

2. Measure one square accurately with a ruler.
   - Most prints come out around 25mm per square.
   - Update SQUARE_SIZE_MM below with YOUR measured value.

3. Take ~25 photos with the Pi camera AT THE EXACT RESOLUTION YOU USE
   IN PRODUCTION (so 1280x720 if you keep the default in pi_client.py).
   - Different angles (tilt left, right, up, down)
   - Different distances (close, far)
   - Different positions (checkerboard near corners of frame, not just center)
   - Make sure the WHOLE checkerboard is visible and IN FOCUS in every photo
   - Save them as .jpg files into a folder named "calib_photos/"

   Easy way to take them: temporarily modify pi_client.py to capture a
   bunch of photos and save them, OR use a phone in pi_camera_server.py
   request mode (just spam CAPTURE while moving the checkerboard around).

4. Run this script:
       python camera_calibration.py

5. Check the output:
   - "Reprojection error" should be UNDER 1.0 pixel.
     - Under 0.5: excellent
     - 0.5 - 1.0: good
     - 1.0 - 2.0: mediocre (take more/better photos and retry)
     - Over 2.0: bad (something wrong, recheck checkerboard, focus, etc.)
   - A file named "camera_calib.npz" gets saved.

6. Move "camera_calib.npz" into the same folder as apriltag_processor.py
   (i.e. pc_laptop/). The next time you run pc_server.py, it will load the
   calibrated values automatically.

DEPENDENCIES:
    pip install opencv-python numpy
==============================================================================
"""

import cv2
import numpy as np
import glob
import os
import sys


# ============================================================================
# CONFIGURATION  -  EDIT THESE FOR YOUR PRINTED CHECKERBOARD
# ============================================================================
CHECKERBOARD = (9, 6)              # (internal corners X, Y).
                                   # For a printed pattern with 10x7 SQUARES,
                                   # there are 9x6 INTERNAL CORNERS.
                                   # Count them carefully - getting this
                                   # wrong gives nonsense calibration.

SQUARE_SIZE_MM = 25.0              # ACTUAL measured size of one square,
                                   # in millimeters. Measure with a ruler.

PHOTOS_DIR = "calib_photos"        # Folder containing your checkerboard
                                   # photos (.jpg files).

OUTPUT_FILE = "camera_calib.npz"   # Output file (move to pc_laptop/ folder)


def main():
    if not os.path.isdir(PHOTOS_DIR):
        print(f"ERROR: folder '{PHOTOS_DIR}' does not exist.")
        print(f"Create it and put your checkerboard JPG photos in it.")
        sys.exit(1)

    images = sorted(glob.glob(os.path.join(PHOTOS_DIR, "*.jpg")))
    if not images:
        print(f"ERROR: no .jpg files found in '{PHOTOS_DIR}'.")
        sys.exit(1)

    print(f"Found {len(images)} photos in '{PHOTOS_DIR}'")
    print(f"Looking for {CHECKERBOARD[0]}x{CHECKERBOARD[1]} internal corners")
    print(f"Square size: {SQUARE_SIZE_MM} mm\n")

    # --- Build the "ideal" 3D corner positions ---
    # The checkerboard is flat on Z=0. Corners are at (0, 0, 0), (sq, 0, 0),
    # (2*sq, 0, 0), ..., for an X by Y grid.
    objp = np.zeros((CHECKERBOARD[0] * CHECKERBOARD[1], 3), np.float32)
    objp[:, :2] = np.mgrid[0:CHECKERBOARD[0], 0:CHECKERBOARD[1]].T.reshape(-1, 2)
    objp *= SQUARE_SIZE_MM

    # Accumulate matched 3D world coords and 2D pixel coords across all photos
    objpoints = []   # 3D points (same set every time)
    imgpoints = []   # 2D points (different for each photo)
    img_shape = None

    for fname in images:
        img = cv2.imread(fname)
        if img is None:
            print(f"  WARNING: could not read {fname}")
            continue

        gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
        if img_shape is None:
            img_shape = gray.shape[::-1]  # (width, height)

        # Try to locate the checkerboard corners
        ret, corners = cv2.findChessboardCorners(gray, CHECKERBOARD, None)
        if ret:
            # Refine corner positions to sub-pixel accuracy
            criteria = (
                cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001
            )
            corners_refined = cv2.cornerSubPix(
                gray, corners, (11, 11), (-1, -1), criteria
            )
            objpoints.append(objp)
            imgpoints.append(corners_refined)
            print(f"  OK   {fname}")
        else:
            print(f"  SKIP {fname} (corners not found)")

    if len(objpoints) < 10:
        print(f"\nWARNING: only {len(objpoints)} photos were usable.")
        print(f"Calibration may be unreliable. Aim for 20+ good photos.")

    if len(objpoints) == 0:
        print("\nERROR: no photos worked. Check your CHECKERBOARD dimensions.")
        sys.exit(1)

    print(f"\nCalibrating using {len(objpoints)} photos...")
    ret, K, dist, rvecs, tvecs = cv2.calibrateCamera(
        objpoints, imgpoints, img_shape, None, None
    )

    print(f"\nReprojection error: {ret:.4f} pixels")
    if ret > 2.0:
        print("  -> BAD. Recheck checkerboard, focus, photo variety.")
    elif ret > 1.0:
        print("  -> MEDIOCRE. Take more photos and rerun.")
    elif ret > 0.5:
        print("  -> GOOD.")
    else:
        print("  -> EXCELLENT.")

    print(f"\nResolution used: {img_shape[0]} x {img_shape[1]}")
    print(f"\nCamera matrix K =")
    print(K)
    print(f"\nDistortion coefficients =")
    print(dist.ravel())

    # Save to disk
    np.savez(OUTPUT_FILE, K=K, dist=dist, resolution=img_shape)
    print(f"\nSaved to '{OUTPUT_FILE}'.")
    print(f"Move this file to the pc_laptop/ folder (next to apriltag_processor.py).")
    print(f"It will be auto-loaded next time you run pc_server.py.")


if __name__ == '__main__':
    main()
