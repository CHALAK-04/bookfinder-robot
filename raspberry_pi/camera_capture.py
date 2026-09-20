#!/usr/bin/env python3
"""Fast Pi camera wrapper with persistent camera object and resolution switching."""
import glob
import os
import time


class CameraCaptureFinal:
    def __init__(self, default_resolution=(1280, 720), jpeg_quality=85):
        self.resolution = tuple(default_resolution)
        self.jpeg_quality = jpeg_quality
        self.mock = os.environ.get("MOCK_CAMERA") == "1"
        self._mock_idx = 0

        if self.mock:
            self._mock_files = sorted(glob.glob("mock_photos/*.jpg"))
            print(f"[MOCK CAMERA] {len(self._mock_files)} photos in mock_photos/")
            return

        from picamera2 import Picamera2
        self.cam = Picamera2()
        self._configure(self.resolution)
        self.cam.start()
        time.sleep(1.0)
        print(f"Camera ready at {self.resolution}, quality={self.jpeg_quality}")

    def _configure(self, resolution):
        config = self.cam.create_video_configuration(
            main={"size": tuple(resolution), "format": "RGB888"},
            buffer_count=3,
        )
        self.cam.configure(config)

    def set_resolution(self, resolution):
        resolution = tuple(resolution)
        if resolution == self.resolution:
            return
        self.resolution = resolution
        if self.mock:
            return
        self.cam.stop()
        self._configure(resolution)
        self.cam.start()
        time.sleep(0.6)  # short AE/AWB settle after mode switch
        print(f"Camera switched to {resolution[0]}x{resolution[1]}")

    def capture_jpeg(self, resolution=None):
        if resolution is not None:
            self.set_resolution(tuple(resolution))
        if self.mock:
            if not self._mock_files:
                raise RuntimeError("MOCK_CAMERA=1 but mock_photos/ is empty")
            path = self._mock_files[self._mock_idx % len(self._mock_files)]
            self._mock_idx += 1
            with open(path, "rb") as f:
                return f.read()

        import cv2
        frame_rgb = self.cam.capture_array()
        frame_bgr = cv2.cvtColor(frame_rgb, cv2.COLOR_RGB2BGR)
        ok, encoded = cv2.imencode(".jpg", frame_bgr, [cv2.IMWRITE_JPEG_QUALITY, self.jpeg_quality])
        if not ok:
            raise RuntimeError("JPEG encoding failed")
        return encoded.tobytes()

    def close(self):
        if not self.mock and hasattr(self, "cam"):
            self.cam.stop()
            self.cam.close()
