#!/usr/bin/env python3
"""Verification subscriber: inspect a Cyclops camera topic.

Inverse of `example_usb_camera.py`. Use it to confirm your publisher is
sending the right wire contract before pointing Cyclops at it.

Usage:
    python3 example_image_subscriber.py
    python3 example_image_subscriber.py --no-display
    python3 example_image_subscriber.py --print-every 1
    python3 example_image_subscriber.py --topic S1/camb

In the printed summary:
- `encoding=jpeg` and `width x height` match the camera.
- `seq` increasing monotonically; `fps~=` matches the capture rate.
- `intrinsics` and `body_frame` match the calibration YAML.
- `imu_frame` is identity.

Dependencies match `cyclops_camera_ecal.py`. The schema is loaded from
`./capnp/`, so this runs without an SDK install on PYTHONPATH.
"""

from __future__ import annotations

import argparse
import logging
import signal
import sys
import threading
import time
from collections import deque
from typing import Any, Optional

import cv2
import numpy as np

from cyclops_camera_ecal import DEFAULT_TOPIC, load_image_capnp

logger = logging.getLogger("example_image_subscriber")


class ImageSubscriber:
    """Minimal capnp Image subscriber.

    Builds an eCAL `MessageSubscriber` for `capnp:Image`, decodes via the
    loaded schema, and exposes the fields. Rolling FPS and latest-frame
    caching exist for the inspection UI.
    """

    def __init__(self, topic: str, image_capnp: Any) -> None:
        self.topic = topic
        self.image_capnp = image_capnp
        self.lock = threading.Lock()
        self.latest_frame: Optional[np.ndarray] = None
        self.latest_summary: Optional[dict] = None
        self.recv_times: deque[float] = deque(maxlen=30)
        self.message_count = 0

        # Lazy import so --help works without eCAL installed.
        from ecal.core.subscriber import MessageSubscriber

        class _CapnpSubscriber(MessageSubscriber):
            def __init__(self, name: str) -> None:
                self.topic_type = "capnp:Image"
                super().__init__(name, self.topic_type)

            def set_callback(self, cb):
                self.c_subscriber.set_callback(cb)

        self._subscriber = _CapnpSubscriber(topic)
        self._subscriber.set_callback(self._on_message)

    def _on_message(self, topic_name: str, msg: bytes, _ts: int) -> None:
        try:
            with self.image_capnp.Image.from_bytes(msg) as image:
                summary = _summarize(image)
                frame = _decode_payload(image)
        except Exception as exc:
            logger.warning(f"Failed to decode message on {topic_name}: {exc}")
            return

        with self.lock:
            self.latest_frame = frame
            self.latest_summary = summary
            self.recv_times.append(time.time())
            self.message_count += 1

    def estimated_fps(self) -> float:
        with self.lock:
            if len(self.recv_times) < 2:
                return 0.0
            elapsed = self.recv_times[-1] - self.recv_times[0]
            return (len(self.recv_times) - 1) / elapsed if elapsed > 0 else 0.0


def _summarize(image: Any) -> dict:
    summary = {
        "seq": image.header.seq,
        "stampMonotonic": image.header.stampMonotonic,
        "clockOffset": image.header.clockOffset,
        "encoding": str(image.encoding),
        "width": image.width,
        "height": image.height,
        "exposureUSec": image.exposureUSec,
        "gain": image.gain,
        "sensorIdx": image.sensorIdx,
        "streamName": image.streamName,
    }

    # Intrinsics + extrinsics may be unset on transformed/derived topics.
    try:
        kb4 = image.intrinsic.kb4
        summary["intrinsics"] = {
            "fx": kb4.pinhole.fx,
            "fy": kb4.pinhole.fy,
            "cx": kb4.pinhole.cx,
            "cy": kb4.pinhole.cy,
            "k1": kb4.k1,
            "k2": kb4.k2,
            "k3": kb4.k3,
            "k4": kb4.k4,
        }
    except Exception:
        summary["intrinsics"] = None

    try:
        body = image.extrinsic.bodyFrame
        imu = image.extrinsic.imuFrame
        summary["body_frame"] = {
            "position": (body.position.x, body.position.y, body.position.z),
            "orientation": (body.orientation.w, body.orientation.x, body.orientation.y, body.orientation.z),
        }
        summary["imu_frame"] = {
            "position": (imu.position.x, imu.position.y, imu.position.z),
            "orientation": (imu.orientation.w, imu.orientation.x, imu.orientation.y, imu.orientation.z),
        }
    except Exception:
        summary["body_frame"] = None
        summary["imu_frame"] = None

    return summary


def _decode_payload(image: Any) -> Optional[np.ndarray]:
    encoding = str(image.encoding)
    raw = np.frombuffer(image.data, dtype=np.uint8)
    if encoding == "jpeg":
        return cv2.imdecode(raw, cv2.IMREAD_GRAYSCALE)
    if encoding == "mono8":
        return raw.reshape((image.height, image.width))
    if encoding == "bgr8":
        return raw.reshape((image.height, image.width, 3))
    logger.warning(f"Unsupported encoding for display: {encoding}")
    return None


def _format_summary(summary: dict, fps: float) -> str:
    lines = [
        f"  seq={summary['seq']}  stamp={summary['stampMonotonic']}  offset={summary['clockOffset']}",
        f"  encoding={summary['encoding']}  size={summary['width']}x{summary['height']}  fps~={fps:.1f}",
        f"  exposure={summary['exposureUSec']}us  gain={summary['gain']}  "
        f"sensor={summary['sensorIdx']}  stream={summary['streamName']!r}",
    ]
    intr = summary["intrinsics"]
    if intr:
        lines.append(
            f"  intrinsics: fx={intr['fx']:.2f} fy={intr['fy']:.2f} "
            f"cx={intr['cx']:.2f} cy={intr['cy']:.2f} "
            f"k=[{intr['k1']:.4f}, {intr['k2']:.4f}, {intr['k3']:.4f}, {intr['k4']:.4f}]"
        )
    else:
        lines.append("  intrinsics: <not set>")

    body = summary["body_frame"]
    if body:
        bp = body["position"]
        bo = body["orientation"]
        lines.append(
            f"  body_frame: pos=({bp[0]:.3f}, {bp[1]:.3f}, {bp[2]:.3f}) "
            f"quat_wxyz=({bo[0]:.5f}, {bo[1]:.5f}, {bo[2]:.5f}, {bo[3]:.5f})"
        )
    else:
        lines.append("  body_frame: <not set>")

    imu = summary["imu_frame"]
    if imu:
        ip = imu["position"]
        io = imu["orientation"]
        lines.append(
            f"  imu_frame:  pos=({ip[0]:.3f}, {ip[1]:.3f}, {ip[2]:.3f}) "
            f"quat_wxyz=({io[0]:.5f}, {io[1]:.5f}, {io[2]:.5f}, {io[3]:.5f})"
        )
    else:
        lines.append("  imu_frame:  <not set>")

    return "\n".join(lines)


def main() -> int:
    parser = argparse.ArgumentParser(description="Subscribe to a Cyclops camera topic and verify its contents")
    parser.add_argument("--topic", default=DEFAULT_TOPIC, help=f"eCAL topic to subscribe to. Default: {DEFAULT_TOPIC}")
    parser.add_argument("--no-display", action="store_true", help="Do not open a cv2 window; run headless")
    parser.add_argument(
        "--print-every",
        type=int,
        default=30,
        help="Print a summary every N messages. Use 1 for every frame. Default: 30.",
    )
    parser.add_argument("--log-level", default="INFO", choices=["DEBUG", "INFO", "WARNING", "ERROR"])
    args = parser.parse_args()

    logging.basicConfig(
        level=getattr(logging, args.log_level),
        format="%(asctime)s [%(levelname)s] %(name)s: %(message)s",
    )

    # Lazy-import eCAL so --help works without it installed.
    import ecal.core.core as ecal_core

    image_capnp = load_image_capnp()
    ecal_core.initialize(sys.argv, "CyclopsExampleSubscriber")
    ecal_core.set_process_state(1, 1, f"Subscribing to {args.topic}")

    subscriber = ImageSubscriber(args.topic, image_capnp)
    logger.info(f"Subscribed to {args.topic}. Waiting for messages... (Ctrl-C to exit)")

    stop_requested = {"value": False}

    def request_stop(sig, _frame):
        logger.info(f"Signal {sig} received, stopping")
        stop_requested["value"] = True

    signal.signal(signal.SIGINT, request_stop)
    signal.signal(signal.SIGTERM, request_stop)

    last_printed_count = 0
    try:
        while not stop_requested["value"] and ecal_core.ok():
            with subscriber.lock:
                count = subscriber.message_count
                summary = subscriber.latest_summary
                frame = subscriber.latest_frame

            if summary is not None and (count - last_printed_count) >= args.print_every:
                logger.info(f"Message #{count} on {args.topic}:\n{_format_summary(summary, subscriber.estimated_fps())}")
                last_printed_count = count

            if not args.no_display and frame is not None:
                cv2.imshow(f"Cyclops camera: {args.topic}", frame)
                if cv2.waitKey(1) & 0xFF == ord("q"):
                    break

            time.sleep(0.02)
    finally:
        if not args.no_display:
            cv2.destroyAllWindows()
        ecal_core.finalize()
        logger.info(f"Received {subscriber.message_count} messages total")

    return 0


if __name__ == "__main__":
    sys.exit(main())
