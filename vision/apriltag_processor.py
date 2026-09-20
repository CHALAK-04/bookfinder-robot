"""
==============================================================================
APRILTAG PROCESSOR  (runs on the LAPTOP)
==============================================================================
This module is called by pc_server.py for each photo received from the Pi.

WHAT IT DOES:
1. Decode the JPEG bytes the Pi sent.
2. Detect AprilTags in the image (the red squares on the shelves).
3. For each detected tag, use POSE ESTIMATION to figure out the tag's
   real-world position in CENTIMETERS relative to the camera.
4. From that, compute "the stepper Z value that would have put this tag
   at the center of the camera frame" - this is the OPTIMAL Z for that row.

WHY POSE ESTIMATION:
We know the physical size of the printed tag (5 cm). The library compares
that to how big the tag looks in pixels in the photo, and gives us the
tag's 3D position in real units. One photo = one solid estimate. Multiple
photos = even better (we average them).

DEPENDENCIES (install on the laptop):
    pip install opencv-python numpy pupil-apriltags
==============================================================================
"""

import numpy as np
import cv2
from pupil_apriltags import Detector


# ============================================================================
# CAMERA INTRINSICS
# ============================================================================
# The AprilTag pose estimator needs to know the camera's focal length and
# optical center. These are called "intrinsics" and they describe how the
# lens projects 3D space onto the 2D image sensor.
#
# Proper way: print a checkerboard, take 25 photos, run camera_calibration.py.
# That gives a camera_calib.npz file which we auto-load below.
#
# For now: we use APPROXIMATE values derived from the OV5647 datasheet
# (Pi Camera v1). The camera spec says:
#   - Full sensor FoV: 54 deg horizontal, 41 deg vertical
#   - We capture at 1280x720 (matches the capture resolution used on the Pi).
#     Picamera2's video config for 720p uses a partial-FoV mode, so the
#     effective FoV is roughly 55 deg horizontal, 33 deg vertical for 16:9.
#
# These are estimates and could be off by 5-10%. For "find which row I'm at"
# this is plenty accurate. For final book OCR distance, calibrate properly.
#
# Format expected by pupil_apriltags: (fx, fy, cx, cy)
# ============================================================================

# Pinhole formula:  fx = (width / 2) / tan(HFoV / 2)
#                   fy = (height / 2) / tan(VFoV / 2)
APPROX_INTRINSICS_1280x720 = (
    1136.0,   # fx  = 640 / tan(27.5°)  ~= 1136
    1232.0,   # fy  = 360 / tan(16.3°)  ~= 1232
    640.0,    # cx  = width / 2
    360.0,    # cy  = height / 2
)

# Try to load real calibrated values if present
try:
    _calib = np.load("camera_calib.npz")
    _K = _calib["K"]
    CAMERA_PARAMS = (
        float(_K[0, 0]),  # fx
        float(_K[1, 1]),  # fy
        float(_K[0, 2]),  # cx
        float(_K[1, 2]),  # cy
    )
    DIST_COEFFS = _calib["dist"]
    _K_MATRIX = _K
    CAMERA_PARAMS_DESCRIPTION = "calibrated (loaded from camera_calib.npz)"
except (FileNotFoundError, KeyError):
    CAMERA_PARAMS = APPROX_INTRINSICS_1280x720
    DIST_COEFFS = None
    _K_MATRIX = None
    CAMERA_PARAMS_DESCRIPTION = (
        f"APPROXIMATE {APPROX_INTRINSICS_1280x720} "
        f"(run camera_calibration.py later for ~10x better accuracy)"
    )


# ============================================================================
# PHYSICAL CONSTANTS  -- MEASURE THESE ON YOUR HARDWARE
# ============================================================================
TAG_SIZE_M = 0.0365   # Physical size of the printed AprilTag, IN METERS (5 cm).
                    # Measure the BLACK border edge to edge with a ruler.

STEPS_PER_CM = 353   # How many stepper steps make the mast go up 1 cm?
                    # Measurement procedure:
                    #   1. Mark the current mast position with tape.
                    #   2. Command the ESP32 to do exactly 1000 steps up.
                    #   3. Measure how far the mast moved (e.g. 4.0 cm).
                    #   4. STEPS_PER_CM = 1000 / 4.0 = 250
                    # The value 25 is a placeholder - REPLACE IT.


# ============================================================================
# DETECTOR (created once, reused for every photo)
# ============================================================================
# tag36h11 is the most common AprilTag family. If you printed a different
# family (e.g. tag25h9), change the string. The robust_pose_estimation flag
# means we get tag position in real-world units, not just pixel position.
_detector = Detector(families="tag36h11")


def process_calib_photo(jpeg_bytes, z_steps_at_capture):
    """
    Detect AprilTags in one photo. For each detected tag, compute what
    stepper Z position would have placed that tag at the camera's optical
    center (i.e. exactly centered vertically in the frame).

    Args:
        jpeg_bytes (bytes): raw JPEG image bytes received from the Pi.
        z_steps_at_capture (int): the stepper position when the photo
                                  was taken (the Pi tells us this).

    Returns:
        List of (tag_id, z_optimal_estimate_in_steps) tuples.
        Empty list if no tags were detected (camera below first row or
        between two rows).
    """
    # --- Decode JPEG to grayscale ---
    # AprilTag detection only needs grayscale, and grayscale is 3x faster
    # than color processing.
    arr = np.frombuffer(jpeg_bytes, dtype=np.uint8)
    img = cv2.imdecode(arr, cv2.IMREAD_GRAYSCALE)
    if img is None:
        return []  # Bad JPEG; skip this photo

    # --- Undistort if we have real calibration data ---
    # Pi camera lens has noticeable barrel distortion near the edges.
    # If we have distortion coefficients (only after running calibration),
    # we straighten the image first. Without them, tag pose near the
    # image edges will be slightly off; still works for centered tags.
    if DIST_COEFFS is not None and _K_MATRIX is not None:
        img = cv2.undistort(img, _K_MATRIX, DIST_COEFFS)

    # --- Run the AprilTag detector ---
    detections = _detector.detect(
        img,
        estimate_tag_pose=True,         # we want real-world position, not just pixels
        camera_params=CAMERA_PARAMS,    # intrinsics tuple (fx, fy, cx, cy)
        tag_size=TAG_SIZE_M,            # physical tag size in METERS
    )

    results = []
    for d in detections:
        # d.pose_t is a (3, 1) translation vector in METERS.
        # Camera coordinate convention (standard OpenCV):
        #   X = right
        #   Y = DOWN
        #   Z = forward (into the scene)
        #
        # So d.pose_t[1] (the Y component) tells us where the tag is
        # vertically RELATIVE to the camera's optical axis:
        #   - negative value: tag is ABOVE optical axis (camera too low)
        #   - positive value: tag is BELOW optical axis (camera too high)
        y_offset_m = float(d.pose_t[1][0])
        y_offset_cm = y_offset_m * 100.0

        # --- Convert to "what stepper Z would center this tag?" ---
        #
        # ASSUMPTION:  stepper_z increases <=> mast goes UP
        # (If your hardware is wired so steps INCREASE = mast goes DOWN,
        # flip the sign of the formula below.)
        #
        # Case 1: tag is above optical axis (y_offset_cm < 0)
        #   We need to RAISE the camera by |y_offset_cm|.
        #   So target_z = z_at_capture + |y_offset_cm| * STEPS_PER_CM
        #               = z_at_capture - y_offset_cm * STEPS_PER_CM   (since y is negative)
        #
        # Case 2: tag is below optical axis (y_offset_cm > 0)
        #   We need to LOWER the camera by y_offset_cm.
        #   So target_z = z_at_capture - y_offset_cm * STEPS_PER_CM
        #
        # Same formula in both cases:
        z_optimal = z_steps_at_capture - int(round(y_offset_cm * STEPS_PER_CM))

        results.append((int(d.tag_id), z_optimal))

    return results


def compute_final_z_positions(tag_estimates):
    """
    Given all the per-photo Z estimates per tag, compute the final answer
    for each row by averaging.

    Why average? Each photo gives a slightly different estimate due to
    lens noise, blur, sub-pixel detection differences. Averaging across
    many photos cancels out random errors.

    Args:
        tag_estimates: dict {tag_id: [z_est_1, z_est_2, ...]}

    Returns:
        dict {tag_id: final_z_steps_int}
    """
    return {
        int(tag_id): int(round(float(np.mean(estimates))))
        for tag_id, estimates in tag_estimates.items()
        if len(estimates) >= 1
    }
