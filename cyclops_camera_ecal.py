#!/usr/bin/env python3
"""Cyclops camera publishing API.

Wire contract Cyclops subscribes to:

    Topic     : S1/cama          (DEFAULT_TOPIC)
    eCAL type : capnp:Image      (DEFAULT_ECAL_TYPE)
    Schema    : capnp/image.capnp  (vkc::Image)
    Encoding  : JPEG, grayscale
    Required  : header.stampMonotonic, header.seq, encoding, width, height,
                data, intrinsic.kb4, extrinsic.bodyFrame

Don't change DEFAULT_TOPIC or DEFAULT_ECAL_TYPE without a coordinated
Cyclops-side change. Field-level details are in the README.

Timing: `stampMonotonic` is the capture timestamp; Cyclops adds
`clockOffset` before matching to odometry. Use `clockOffset=0` if your
camera stamps in the local host clock domain.

Dependencies:
    pip : numpy, opencv-python, pycapnp, PyYAML
    apt : ecal, python3-ecal5  (PPA: ecal/ecal-5.11; pre-installed on devices)

Schema lookup order for `image.capnp`:
    1. `schema_dir` arg to `load_image_capnp(...)`
    2. `$VNS_CAPNP_SCHEMA_DIR`
    3. `./capnp/` next to this file
    4. `/usr/include/capnp`, `/usr/local/include/capnp`

Quick start:

    from cyclops_camera_ecal import CyclopsCameraPublisher, load_camera_calibration

    calibration = load_camera_calibration("camera_params.example.yaml")
    publisher = CyclopsCameraPublisher()
    publisher.publish_frame(numpy_frame, calibration)
    publisher.close()

End-to-end example: `example_usb_camera.py`.
Verification subscriber: `example_image_subscriber.py`.
"""

from __future__ import annotations

import importlib
import os
import sys
import time
from dataclasses import dataclass
from pathlib import Path
from typing import Any, Optional

# Wire contract — don't change without a coordinated Cyclops-side change.
DEFAULT_TOPIC = "S1/cama"
DEFAULT_ECAL_TYPE = "capnp:Image"

# Informational. Keep stable for log/recording comparability.
DEFAULT_PROCESS_NAME = "CyclopsExternalCamera"
DEFAULT_STREAM_NAME = "external_nadir_camera"


@dataclass(frozen=True)
class KB4Intrinsics:
    """Kannala-Brandt 4 fisheye intrinsics in pixels.

    `fx, fy, cx, cy` are the standard pinhole parameters; `k1..k4` are the
    KB4 distortion coefficients. Source from the Vozilla calibration page.
    """

    fx: float
    fy: float
    cx: float
    cy: float
    k1: float = 0.0
    k2: float = 0.0
    k3: float = 0.0
    k4: float = 0.0


@dataclass(frozen=True)
class Pose3D:
    """6-DoF pose: XYZ in meters + quaternion (w, x, y, z).

    Body frame is X forward, Y right, Z down (NED). The schema uses w-first
    quaternions; convert if you're pulling from scipy/ROS (x, y, z, w).
    """

    position_xyz_m: tuple[float, float, float] = (0.0, 0.0, 0.0)
    orientation_wxyz: tuple[float, float, float, float] = (1.0, 0.0, 0.0, 0.0)


@dataclass(frozen=True)
class CameraCalibration:
    """Calibration fields required by Cyclops camera messages.

    `imu_frame` is legacy and not consumed by Cyclops. Leave it unset; the
    helper publishes identity.
    """

    intrinsics: KB4Intrinsics
    body_frame: Pose3D
    imu_frame: Optional[Pose3D] = None
    rectified: bool = True

    def resolved_imu_frame(self) -> Pose3D:
        return self.imu_frame if self.imu_frame is not None else self.body_frame


class CyclopsCameraPublisher:
    """Publish Cyclops-compatible camera images on eCAL.

    Construct, then call `publish_frame(numpy_array, calibration)` per
    frame (or `publish_jpeg(...)` if your stack produces JPEG). Close
    explicitly or use as a context manager.

    By default the publisher manages the eCAL lifecycle. Pass
    `initialize_ecal=False` if eCAL is already initialized in your process.
    """

    def __init__(
        self,
        topic: str = DEFAULT_TOPIC,
        process_name: str = DEFAULT_PROCESS_NAME,
        schema_dir: str | os.PathLike[str] | None = None,
        capnp_include_dir: str | os.PathLike[str] | None = None,
        initialize_ecal: bool = True,
        finalize_ecal_on_close: bool = True,
    ):
        self.topic = topic
        self.seq = 0
        self._finalize_ecal_on_close = finalize_ecal_on_close
        self._ecal_initialized = False

        self.image_capnp = load_image_capnp(schema_dir=schema_dir, capnp_include_dir=capnp_include_dir)
        self.ecal_core = importlib.import_module("ecal.core.core")

        if initialize_ecal:
            self.ecal_core.initialize(sys.argv, process_name)
            self.ecal_core.set_process_state(1, 1, f"Publishing Cyclops camera topic {topic}")
            self._ecal_initialized = True

        self.publisher = self._make_publisher(topic)

    def _make_publisher(self, topic: str) -> Any:
        try:
            return self.ecal_core.publisher(topic, DEFAULT_ECAL_TYPE)
        except TypeError:
            # Older eCAL Python bindings only accept the topic name.
            return self.ecal_core.publisher(topic)

    def publish_frame(
        self,
        frame: Any,
        calibration: CameraCalibration,
        *,
        capture_time_ns: int | None = None,
        clock_offset_ns: int = 0,
        exposure_usec: int = 10000,
        gain: int = 100,
        sensor_idx: int = 0,
        stream_name: str = DEFAULT_STREAM_NAME,
        jpeg_quality: int = 95,
    ) -> int:
        """Encode a numpy/OpenCV frame as grayscale JPEG and publish.

        Accepts BGR color, single-channel mono, or uint16 thermal frames.

        Args:
            frame: 2D grayscale, BGR color, or uint16 thermal numpy array.
            calibration: Intrinsics + extrinsics, stamped on each frame.
            capture_time_ns: Capture timestamp in the Cyclops local clock
                domain. Defaults to `time.time_ns()`.
            clock_offset_ns: Added to `capture_time_ns` before odometry
                matching. `0` for local host timestamps.
            exposure_usec, gain: From the camera if available.
            sensor_idx: `0` for the primary nadir camera.
            stream_name: Stable camera label.
            jpeg_quality: 1..100. Lower trades fine detail for bandwidth.

        Returns:
            The published sequence number.
        """

        jpeg_bytes, width, height = encode_grayscale_jpeg(frame, jpeg_quality=jpeg_quality)
        return self.publish_jpeg(
            jpeg_bytes,
            width=width,
            height=height,
            calibration=calibration,
            capture_time_ns=capture_time_ns,
            clock_offset_ns=clock_offset_ns,
            exposure_usec=exposure_usec,
            gain=gain,
            sensor_idx=sensor_idx,
            stream_name=stream_name,
        )

    def publish_jpeg(
        self,
        jpeg_bytes: bytes,
        *,
        width: int,
        height: int,
        calibration: CameraCalibration,
        capture_time_ns: int | None = None,
        clock_offset_ns: int = 0,
        exposure_usec: int = 10000,
        gain: int = 100,
        sensor_idx: int = 0,
        stream_name: str = DEFAULT_STREAM_NAME,
    ) -> int:
        """Publish an already-encoded JPEG frame.

        For cameras that produce JPEG natively (thermal, some industrial).
        Avoids the re-encode that `publish_frame` does.

        Args:
            jpeg_bytes: JPEG payload, must decode to grayscale.
            width, height: Decoded dimensions in pixels.
            calibration, capture_time_ns, clock_offset_ns, exposure_usec,
                gain, sensor_idx, stream_name: see `publish_frame`.

        Returns:
            The published sequence number.
        """

        seq = self.seq
        msg = build_image_message(
            self.image_capnp,
            jpeg_bytes=jpeg_bytes,
            width=width,
            height=height,
            calibration=calibration,
            seq=seq,
            capture_time_ns=capture_time_ns,
            clock_offset_ns=clock_offset_ns,
            exposure_usec=exposure_usec,
            gain=gain,
            sensor_idx=sensor_idx,
            stream_name=stream_name,
        )
        self.publisher.send(msg.to_bytes())
        self.seq += 1
        return seq

    def close(self) -> None:
        destroy = getattr(self.publisher, "destroy", None)
        if callable(destroy):
            destroy()
        if self._ecal_initialized and self._finalize_ecal_on_close:
            self.ecal_core.finalize()
            self._ecal_initialized = False

    def __enter__(self) -> "CyclopsCameraPublisher":
        return self

    def __exit__(self, *_: Any) -> None:
        self.close()


def build_image_message(
    image_capnp: Any,
    *,
    jpeg_bytes: bytes,
    width: int,
    height: int,
    calibration: CameraCalibration,
    seq: int = 0,
    capture_time_ns: int | None = None,
    clock_offset_ns: int = 0,
    exposure_usec: int = 10000,
    gain: int = 100,
    sensor_idx: int = 0,
    stream_name: str = DEFAULT_STREAM_NAME,
) -> Any:
    """Build a `vkc::Image` Cap'n Proto message without publishing.

    Use this to inspect, log, or fan out across multiple transports
    yourself. The common case is `CyclopsCameraPublisher.publish_frame`.
    """

    validate_calibration(calibration, width=width, height=height)

    msg = image_capnp.Image.new_message()
    msg.header.stampMonotonic = int(capture_time_ns if capture_time_ns is not None else time.time_ns())
    msg.header.clockOffset = int(clock_offset_ns)
    msg.header.seq = int(seq)

    msg.encoding = image_capnp.Image.Encoding.jpeg
    msg.width = int(width)
    msg.height = int(height)
    msg.step = 0
    msg.data = bytes(jpeg_bytes)

    msg.exposureUSec = max(0, int(exposure_usec))
    msg.gain = max(0, int(gain))
    msg.sensorIdx = int(sensor_idx)
    msg.streamName = stream_name

    intr = calibration.intrinsics
    msg.intrinsic.kb4.pinhole.fx = float(intr.fx)
    msg.intrinsic.kb4.pinhole.fy = float(intr.fy)
    msg.intrinsic.kb4.pinhole.cx = float(intr.cx)
    msg.intrinsic.kb4.pinhole.cy = float(intr.cy)
    msg.intrinsic.kb4.k1 = float(intr.k1)
    msg.intrinsic.kb4.k2 = float(intr.k2)
    msg.intrinsic.kb4.k3 = float(intr.k3)
    msg.intrinsic.kb4.k4 = float(intr.k4)
    msg.intrinsic.rectified = bool(calibration.rectified)
    msg.intrinsic.lastModified = time.time_ns()

    _write_pose(msg.extrinsic.bodyFrame, calibration.body_frame)
    _write_pose(msg.extrinsic.imuFrame, calibration.resolved_imu_frame())
    msg.extrinsic.lastModified = time.time_ns()

    return msg


def publish_camera_frame(
    publisher: CyclopsCameraPublisher,
    frame: Any,
    calibration: CameraCalibration,
    **kwargs: Any,
) -> int:
    """Convenience function for one-line frame publishing."""

    return publisher.publish_frame(frame, calibration, **kwargs)


def encode_grayscale_jpeg(frame: Any, *, jpeg_quality: int = 95) -> tuple[bytes, int, int]:
    """Convert a numpy/OpenCV frame to 8-bit grayscale JPEG.

    Accepts BGR color, single-channel mono, or uint16 thermal. Returned
    width/height are the decoded dimensions; use them on the published
    message.
    """

    cv2 = importlib.import_module("cv2")
    np = importlib.import_module("numpy")

    if frame is None:
        raise ValueError("frame must not be None")
    if not hasattr(frame, "shape"):
        raise TypeError("frame must be a numpy/OpenCV array with a shape")

    if frame.dtype == np.uint16:
        frame_8bit = cv2.normalize(frame, None, 0, 255, cv2.NORM_MINMAX, dtype=cv2.CV_8U)
    elif len(frame.shape) == 3:
        frame_8bit = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    elif len(frame.shape) == 2:
        frame_8bit = frame
    else:
        raise ValueError(f"unsupported frame shape: {frame.shape}")

    ok, jpeg = cv2.imencode(".jpg", frame_8bit, [int(cv2.IMWRITE_JPEG_QUALITY), int(jpeg_quality)])
    if not ok or jpeg is None:
        raise RuntimeError("JPEG encoding failed")

    height, width = frame_8bit.shape[:2]
    return jpeg.tobytes(), int(width), int(height)


def load_camera_calibration(path: str | os.PathLike[str]) -> CameraCalibration:
    """Load `CameraCalibration` from the SDK `camera_params.yaml` shape.

    Mirrors the on-device file at
    `/usr/etc/vns-sdk/camera_params.yaml`. See `camera_params.example.yaml`
    for a template. Missing `imu_frame` is left unset (publisher sends
    identity).
    """

    yaml = importlib.import_module("yaml")
    with Path(path).open("r", encoding="utf-8") as file:
        config = yaml.safe_load(file)

    intr = config["intrinsics"]
    extr = config["extrinsics"]

    imu_frame_cfg = extr.get("imu_frame")
    return CameraCalibration(
        intrinsics=KB4Intrinsics(
            fx=float(intr["fx"]),
            fy=float(intr["fy"]),
            cx=float(intr["cx"]),
            cy=float(intr["cy"]),
            k1=float(intr.get("k1", 0.0)),
            k2=float(intr.get("k2", 0.0)),
            k3=float(intr.get("k3", 0.0)),
            k4=float(intr.get("k4", 0.0)),
        ),
        body_frame=_pose_from_config(extr["body_frame"]),
        imu_frame=_pose_from_config(imu_frame_cfg) if imu_frame_cfg is not None else None,
        rectified=bool(config.get("rectified", True)),
    )


def load_image_capnp(
    *,
    schema_dir: str | os.PathLike[str] | None = None,
    capnp_include_dir: str | os.PathLike[str] | None = None,
) -> Any:
    """Load `image_capnp`: generated headers if available, else dynamic
    parsing via pycapnp against the schema files in `./capnp/`.

    Schema search order is in the module docstring.
    """

    for import_dir in _candidate_schema_dirs(schema_dir):
        if import_dir.exists():
            sys.path.insert(0, str(import_dir))

    try:
        return importlib.import_module("image_capnp")
    except ImportError:
        pass

    capnp = importlib.import_module("capnp")
    schema_path = _resolve_schema_path(schema_dir)
    include_dir = _resolve_capnp_include_dir(capnp_include_dir)

    return capnp.load(str(schema_path), imports=[str(schema_path.parent), str(include_dir)])


def validate_calibration(calibration: CameraCalibration, *, width: int, height: int) -> None:
    """Sanity-check calibration before publishing.

    Catches zero/negative focal lengths, principal points outside the image,
    and non-normalized quaternions. Called by `publish_jpeg`; call directly
    only when using `build_image_message` without the publisher.
    """
    intr = calibration.intrinsics
    if width <= 0 or height <= 0:
        raise ValueError("width and height must be positive")
    if intr.fx <= 0 or intr.fy <= 0:
        raise ValueError("fx and fy must be positive")
    if not (0 <= intr.cx <= width):
        raise ValueError(f"cx must be inside the image width: cx={intr.cx}, width={width}")
    if not (0 <= intr.cy <= height):
        raise ValueError(f"cy must be inside the image height: cy={intr.cy}, height={height}")

    _validate_pose(calibration.body_frame, "body_frame")
    _validate_pose(calibration.resolved_imu_frame(), "imu_frame")


def _write_pose(target: Any, pose: Pose3D) -> None:
    x, y, z = pose.position_xyz_m
    w, qx, qy, qz = pose.orientation_wxyz

    target.position.x = float(x)
    target.position.y = float(y)
    target.position.z = float(z)
    target.orientation.w = float(w)
    target.orientation.x = float(qx)
    target.orientation.y = float(qy)
    target.orientation.z = float(qz)


def _pose_from_config(config: dict[str, Any]) -> Pose3D:
    pos = config["position"]
    orient = config["orientation"]
    return Pose3D(
        position_xyz_m=(float(pos["x"]), float(pos["y"]), float(pos["z"])),
        orientation_wxyz=(float(orient["w"]), float(orient["x"]), float(orient["y"]), float(orient["z"])),
    )


def _validate_pose(pose: Pose3D, name: str) -> None:
    if len(pose.position_xyz_m) != 3:
        raise ValueError(f"{name}.position_xyz_m must have three values")
    if len(pose.orientation_wxyz) != 4:
        raise ValueError(f"{name}.orientation_wxyz must have four values")

    norm_sq = sum(v * v for v in pose.orientation_wxyz)
    if norm_sq <= 0.0:
        raise ValueError(f"{name}.orientation_wxyz must be a non-zero quaternion")
    if abs(norm_sq - 1.0) > 0.05:
        raise ValueError(f"{name}.orientation_wxyz should be normalized; norm^2={norm_sq:.3f}")


def _resolve_schema_path(schema_dir: str | os.PathLike[str] | None) -> Path:
    candidates = [schema_path / "image.capnp" for schema_path in _candidate_schema_dirs(schema_dir)]

    for candidate in candidates:
        if candidate.exists():
            return candidate

    searched = ", ".join(str(path) for path in candidates)
    raise FileNotFoundError(f"could not find image.capnp; searched: {searched}")


def _candidate_schema_dirs(schema_dir: str | os.PathLike[str] | None) -> list[Path]:
    candidates: list[Path] = []
    if schema_dir is not None:
        candidates.append(Path(schema_dir))
    if os.environ.get("VNS_CAPNP_SCHEMA_DIR"):
        candidates.append(Path(os.environ["VNS_CAPNP_SCHEMA_DIR"]))
    candidates.extend(
        [
            Path(__file__).resolve().parent / "capnp",
            Path("/usr/include/capnp"),
            Path("/usr/local/include/capnp"),
        ]
    )
    return candidates


def _resolve_capnp_include_dir(capnp_include_dir: str | os.PathLike[str] | None) -> Path:
    candidates: list[Path] = []
    if capnp_include_dir is not None:
        candidates.append(Path(capnp_include_dir))
    if os.environ.get("CAPNP_INCLUDE_DIR"):
        candidates.append(Path(os.environ["CAPNP_INCLUDE_DIR"]))
    candidates.extend([Path("/usr/include"), Path("/usr/local/include")])

    for candidate in candidates:
        if (candidate / "capnp/c++.capnp").exists():
            return candidate

    searched = ", ".join(str(path) for path in candidates)
    raise FileNotFoundError(f"could not find capnp/c++.capnp; searched: {searched}")
