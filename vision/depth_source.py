"""Astra Pro depth via OpenNI2, hardware-registered onto the RGB camera's pixel grid.

Three facts about this camera that the code below depends on, all verified on the hardware:
- Depth only. The Pro's RGB is a separate UVC device (see color_source.py); asking OpenNI2 for a
  color stream hangs or segfaults.
- Mirroring must be OFF. OpenNI2 mirrors depth by default and the UVC color is not mirrored, so
  with the default the registered depth lands left-right flipped against the RGB.
- read_frame() has no timeout. Every read is gated by wait_for_any_stream(timeout) so a USB
  hiccup costs one empty poll instead of a thread blocked forever.
"""
import logging
import os
import sys

import numpy as np
from openni import openni2
from openni import _openni2 as c_api

from frame_grabber import LatestFrameGrabber

WIDTH, HEIGHT, FPS = 640, 480, 30   # must match the color mode: D2C targets the RGB pixel grid
READ_TIMEOUT_S = 0.5
# Fallback when OPENNI2_REDIST is unset, Windows only: the SDK zip unpacked on the dev laptop.
# Not vendored into the repo. On the Jetson set OPENNI2_REDIST to the Linux ARM64 redist.
WINDOWS_DEFAULT_REDIST = r"C:\Users\admin\Downloads\openni_sdk\extracted\OpenNI_2.3.0.86_202210111950_4c8f5aa4_beta6_windows\Win64-Release\sdk\libs"

logger = logging.getLogger(__name__)


def resolve_redist_path():
    path = os.environ.get("OPENNI2_REDIST") or (WINDOWS_DEFAULT_REDIST if sys.platform == "win32" else None)
    if not path or not os.path.isdir(path):
        raise RuntimeError(
            f"OpenNI2 redist not found at '{path}'. Set OPENNI2_REDIST to the folder "
            "containing OpenNI2.dll / libOpenNI2.so (e.g. .../sdk/libs)."
        )
    return path


class DepthSource(LatestFrameGrabber):
    """Registered depth frames as uint16 millimeters (0 = no return), HEIGHT x WIDTH."""

    name = "depth"

    def __init__(self, redist_path):
        super().__init__()
        self._redist_path = redist_path
        self._initialized = False
        self._device = None
        self._stream = None
        self.fov = None   # (horizontal, vertical) radians, set once the stream is open

    def close(self):
        clean = super().close()
        if clean and self._initialized:
            openni2.unload()
        return clean

    def _open(self):
        if not self._initialized:
            openni2.initialize(self._redist_path)
            self._initialized = True
        self._device = openni2.Device.open_any()
        self._stream = self._device.create_depth_stream()
        self._stream.set_video_mode(c_api.OniVideoMode(
            pixelFormat=c_api.OniPixelFormat.ONI_PIXEL_FORMAT_DEPTH_1_MM,
            resolutionX=WIDTH, resolutionY=HEIGHT, fps=FPS))
        self._stream.set_mirroring_enabled(False)
        self._stream.start()
        d2c = c_api.OniImageRegistrationMode.ONI_IMAGE_REGISTRATION_DEPTH_TO_COLOR
        if self._device.is_image_registration_mode_supported(d2c):
            self._device.set_image_registration_mode(d2c)
        else:
            logger.error("D2C registration unsupported - depth will be offset from RGB "
                         "(~15 px measured on the dev laptop); distances near object edges will be wrong")
        self.fov = (self._stream.get_horizontal_fov(), self._stream.get_vertical_fov())
        info = self._device.get_device_info()
        logger.info("opened %s %s, registration=%s", info.vendor, info.name,
                    self._device.get_image_registration_mode())

    def _read(self):
        if openni2.wait_for_any_stream([self._stream], timeout=READ_TIMEOUT_S) is None:
            return None
        frame = self._stream.read_frame()
        # Copy: the buffer belongs to the frame and is recycled once the frame is released.
        return np.frombuffer(frame.get_buffer_as_uint16(), dtype=np.uint16).reshape(
            frame.height, frame.width).copy()

    def _close(self):
        stream, device = self._stream, self._device
        self._stream = self._device = None
        try:
            if stream is not None:
                stream.stop()
        finally:
            if device is not None:
                device.close()
