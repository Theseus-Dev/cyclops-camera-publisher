#!/usr/bin/env python3
"""End-to-end example: publish a USB camera into Cyclops.

    USB camera -> OpenCV -> CyclopsCameraPublisher -> eCAL S1/cama -> Cyclops

Swap `cv2.VideoCapture` for your camera SDK; the publishing call is the
same. Simpler than `/usr/bin/usb_camera_publisher.py`, which also handles
auto-detection, hot-plug, v4l2 introspection, and thermal cameras.

Usage:
    python3 example_usb_camera.py
    python3 example_usb_camera.py --device /dev/video2 --config my_cam.yaml

Dependencies and schema lookup are documented in `cyclops_camera_ecal.py`.

Troubleshooting:
- "Failed to open camera": wrong device, or another process has it.
  Run `v4l2-ctl --list-devices`.
- Cyclops doesn't see frames: confirm the topic with `ecal_mon_gui` or
  `example_image_subscriber.py`. Different eCAL networks won't see each
  other.
- Calibration wrong on subscriber: check you edited the YAML at `--config`,
  not the example.
"""

from __future__ import annotations

import argparse
import logging
import signal
import sys
import time
from pathlib import Path

import cv2
import yaml

from cyclops_camera_ecal import (
    CyclopsCameraPublisher,
    load_camera_calibration,
)

logger = logging.getLogger("example_usb_camera")


def open_camera(device: str, width: int, height: int, fps: int) -> cv2.VideoCapture:
    """Open a USB camera with OpenCV. Replace with your own camera SDK."""
    capture_target: int | str = int(device) if device.lstrip("-").isdigit() else device
    camera = cv2.VideoCapture(capture_target)
    if not camera.isOpened():
        raise RuntimeError(f"Failed to open camera '{device}'")

    camera.set(cv2.CAP_PROP_FRAME_WIDTH, width)
    camera.set(cv2.CAP_PROP_FRAME_HEIGHT, height)
    camera.set(cv2.CAP_PROP_FPS, fps)

    actual_w = int(camera.get(cv2.CAP_PROP_FRAME_WIDTH))
    actual_h = int(camera.get(cv2.CAP_PROP_FRAME_HEIGHT))
    actual_fps = camera.get(cv2.CAP_PROP_FPS)
    logger.info(f"Opened {device}: {actual_w}x{actual_h} @ {actual_fps:.1f} fps")
    return camera


def load_yaml(path: Path) -> dict:
    """Read raw YAML for capture parameters not in `CameraCalibration` (resolution, fps)."""
    with path.open("r", encoding="utf-8") as handle:
        return yaml.safe_load(handle)


def main() -> int:
    parser = argparse.ArgumentParser(description="Publish a USB camera to Cyclops on S1/cama")
    parser.add_argument(
        "--device",
        default="0",
        help="OpenCV camera index (e.g. 0) or device path (e.g. /dev/video0). Default: 0.",
    )
    parser.add_argument(
        "--config",
        type=Path,
        default=Path(__file__).resolve().parent / "camera_params.example.yaml",
        help="Path to camera_params.yaml. Default: camera_params.example.yaml next to this script.",
    )
    parser.add_argument("--log-level", default="INFO", choices=["DEBUG", "INFO", "WARNING", "ERROR"])
    args = parser.parse_args()

    logging.basicConfig(
        level=getattr(logging, args.log_level),
        format="%(asctime)s [%(levelname)s] %(name)s: %(message)s",
    )

    if not args.config.exists():
        logger.error(f"Config not found: {args.config}")
        return 1

    # One YAML feeds both the calibration (capnp Image fields) and the
    # cv2 capture properties — published intrinsics match what's captured.
    calibration = load_camera_calibration(args.config)
    raw_config = load_yaml(args.config)
    resolution = raw_config.get("resolution", {})
    width = int(resolution.get("width", 640))
    height = int(resolution.get("height", 512))
    fps = int(raw_config.get("fps", 30))

    camera = open_camera(args.device, width, height, fps)
    publisher = CyclopsCameraPublisher()

    stop_requested = {"value": False}

    def request_stop(sig, _frame):
        logger.info(f"Signal {sig} received, stopping")
        stop_requested["value"] = True

    signal.signal(signal.SIGINT, request_stop)
    signal.signal(signal.SIGTERM, request_stop)

    frame_interval = 1.0 / max(1, fps)
    consecutive_failures = 0

    try:
        while not stop_requested["value"]:
            t0 = time.time()
            ok, frame = camera.read()
            if not ok or frame is None:
                consecutive_failures += 1
                if consecutive_failures >= 10:
                    logger.error("Camera read failed 10 times in a row; exiting")
                    return 1
                time.sleep(0.05)
                continue
            consecutive_failures = 0

            try:
                publisher.publish_frame(frame, calibration)
            except Exception as exc:
                logger.warning(f"Publish failed: {exc}")

            # Bound publish rate; cv2 often already blocks at native fps.
            elapsed = time.time() - t0
            if elapsed < frame_interval:
                time.sleep(frame_interval - elapsed)
    finally:
        camera.release()
        publisher.close()
        logger.info("Shutdown complete")

    return 0


if __name__ == "__main__":
    sys.exit(main())
