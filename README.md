# Cyclops External Camera Publisher

Camera-only integration kit for teams feeding their own camera stack into
Cyclops over eCAL. This repo covers `S1/cama` and nothing else. For full
system setup, see the public docs:

- [Getting Started](https://docs.theseus.us/cyclops/getting-started)
- [Pre-Requisites](https://docs.theseus.us/cyclops/pre-requisites)
- [Hardware Setup](https://docs.theseus.us/cyclops/cyclops-computer/pi-5-walkthrough/hardware-setup)
- [Camera Calibration](https://docs.theseus.us/cyclops/vehicle-integration/camera-calibration)
- [Bench Test](https://docs.theseus.us/cyclops/validation/bench-test)

## Quickstart

eCAL comes from the Theseus apt repo (and is preinstalled on Cyclops devices):

```bash
sudo add-apt-repository ppa:ecal/ecal-5.11
sudo apt-get update
sudo apt-get install -y ecal python3-ecal5
```

### Python

```bash
python3 -m pip install numpy opencv-python pycapnp PyYAML

# Terminal 1 — publish a USB camera on S1/cama
python3 example_usb_camera.py --device 0 --config camera_params.example.yaml

# Terminal 2 — verify what's on the wire
python3 example_image_subscriber.py --topic S1/cama
```

### C++

```bash
sudo apt-get install -y libopencv-dev libyaml-cpp-dev capnproto libcapnp-dev

cmake -S . -B build
cmake --build build

./build/example_usb_camera --device 0 --config camera_params.example.yaml
# verify with example_image_subscriber.py as above
```

If frames show up in the subscriber, the wire contract is correct. From
there, replace the OpenCV capture with your camera SDK; the publishing
call stays the same.

## Contents

| File | Purpose |
| --- | --- |
| `cyclops_camera_ecal.py` | Python library. Wraps eCAL + Cap'n Proto. |
| `cyclops_camera_ecal.{hpp,cpp}` | C++ library. Same contract. |
| `example_usb_camera.py` | Python end-to-end: OpenCV USB capture → publish. |
| `example_usb_camera.cpp` | C++ counterpart. Built when `libyaml-cpp-dev` is present. |
| `example_image_subscriber.py` | Subscribes to `S1/cama`, prints calibration + timing, displays JPEG. |
| `camera_params.example.yaml` | Calibration template. Mirrors the on-device YAML shape. |
| `samples/` | Real production calibrations. Reference only — do not transfer between units. |
| `capnp/` | Cap'n Proto schemas for the standalone C++ build. |
| `CMakeLists.txt` | C++ build. |

## Helper vs. Bundled Publisher

These are different things:

- **`/usr/bin/usb_camera_publisher.py`** (stock Cyclops) — a complete
  publisher: USB hot-plug, `v4l2-ctl`, thermal support, edge-API
  side-channel. Replace this if you want Cyclops to consume your camera.
- **`cyclops_camera_ecal.{py,cpp}`** (this repo) — a thin library used
  *by* a publisher. No device discovery, no capture loop. You own the
  bring-up; the helper owns the wire format.

To see the helper running against a real camera, run `example_usb_camera.py`.
For production, write your equivalent of `usb_camera_publisher.py` on top of
the helper.

### Bundled service

On a stock Cyclops device, the bundled publisher is installed by the SDK
package. The relevant paths:

| | |
| --- | --- |
| Unit file | `/lib/systemd/system/usb-camera.service` |
| Publisher | `/usr/bin/usb_camera_publisher.py` |
| Config | `/usr/etc/vns-sdk/camera_params.yaml` |
| Python venv | `/opt/vns-sdk/venv` |

`systemctl cat usb-camera.service` on the device prints (with substitutions
applied):

```ini
[Unit]
Description=USB Nadir Camera Publisher
After=network.target
Before=cyclops.service

[Service]
Type=simple
User=pi
Group=pi
SupplementaryGroups=video
Environment="PATH=/opt/vns-sdk/venv/bin:/usr/local/sbin:/usr/local/bin:/usr/sbin:/usr/bin:/sbin:/bin"
ExecStart=/opt/vns-sdk/venv/bin/python3 /usr/bin/usb_camera_publisher.py \
    --config /usr/etc/vns-sdk/camera_params.yaml
Restart=on-failure
RestartSec=5

[Install]
WantedBy=multi-user.target
```

The service user defaults to `pi`; some builds use a different account
(e.g. `theseus`) — check `systemctl cat` to confirm.

Two ordering constraints to mirror in your replacement:

- `Before=cyclops.service` — MapMatcher comes up degraded if `S1/cama` is
  silent at startup.
- `Restart=on-failure` — your replacement should be similarly resilient or
  run behind an equivalent restart policy.

Operating the service:

```bash
sudo systemctl status usb-camera.service
sudo journalctl -u usb-camera.service -f
sudo systemctl restart usb-camera.service
```

### Replacing it

**On the device.** Disable the bundled service so it doesn't fight yours
for the camera, then drop in your own:

```bash
sudo systemctl disable --now usb-camera.service

# unit file should declare Before=cyclops.service
sudo cp my-camera.service /etc/systemd/system/
sudo systemctl daemon-reload
sudo systemctl enable --now my-camera.service

ecal_mon_gui                                     # confirm S1/cama
sudo systemctl restart cyclops.service           # if cyclops started before your stream
```

**From another host on the same eCAL network.** Don't touch the device
filesystem. Disable the bundled service and run your publisher wherever
your camera lives. This is the fastest path during integration.

Either way, the contract is:

| | |
| --- | --- |
| Topic | `S1/cama` |
| eCAL type | `capnp:Image` |
| Schema | `vkc::Image` (`capnp/image.capnp`) |
| Encoding | JPEG, grayscale |
| Ordering | Up before `cyclops.service`, or restart `cyclops.service` after |

## Data path

```text
camera stack -> eCAL S1/cama -> MapMatcher -> Cyclops output
```

## Calibration sources

In a stock install, calibration lives in one YAML on the device and is
stamped on every frame. As an integrator you replace the publisher; you
still publish the same fields, but you can source them however you like.

```text
Vozilla camera UI -> Calibration REST API -> camera_params.yaml -> usb_camera_publisher.py -> S1/cama
                                              ^                    ^
                                              source of truth       loads YAML, fills capnp Image
```

| Layer | Where | Role |
| --- | --- | --- |
| Vozilla calibration UI | desktop app, "Camera" panel | Circle-grid calibration. Computes intrinsics; extrinsics are a hardcoded nadir default. |
| Calibration REST API | `theseus-edge-api` on the edge device | `GET/POST /api/v2/calibration/cameras`, `/api/v2/calibration/active`. |
| `camera_params.yaml` | `/usr/etc/vns-sdk/camera_params.yaml` | Active calibration on disk. Not exposed via `/api/v2/params/`; managed via the Calibration API. |
| `usb_camera_publisher.py` | `/usr/bin/usb_camera_publisher.py` | Reads the YAML at startup, stamps `intrinsic.kb4` and `extrinsic.bodyFrame` on each frame. |

`camera_params.example.yaml` here mirrors the on-device YAML shape, so a
calibration produced by the Vozilla UI drops in unchanged.

For the calibration procedure itself, see the
[Camera Calibration](https://docs.theseus.us/cyclops/vehicle-integration/camera-calibration)
docs.

### Two calibrations, one of them yours

| Calibration | Captures | Lives in | Computed by | Yours? |
| --- | --- | --- | --- | --- |
| Camera (intrinsics + nadir-default extrinsics) | KB4 lens model: `fx, fy, cx, cy, k1..k4` | `camera_params.yaml`, stamped on every Image | Vozilla circle-grid (intrinsics); extrinsics hardcoded | Yes |
| Vehicle (`b_T_vb`) | Body frame ↔ IMU mount | `vehicle_params.yaml` | Vozilla vehicle-config UI; consumed by `ekf_transformer` / `vio` / `timesync` | No |

`extrinsic.bodyFrame` looks like a per-mount calibrated transform; it isn't.
Runtime mount geometry comes from `b_T_vb` in vehicle calibration. The
camera-message field is a legacy default — populate it with the SDK constant.

## Topic contract

| Topic | eCAL type | Schema |
| --- | --- | --- |
| `S1/cama` | `capnp:Image` | `capnp/image.capnp` |

Don't change the topic or type. Cyclops looks for `S1/cama` and decodes it
as `capnp:Image`. The helpers expose these as defaults so you don't drift
from the wire contract.

Payload is JPEG. The bundled publisher normalizes incoming frames to 8-bit
grayscale before encoding.

### Required fields

| Field | Value | Used for |
| --- | --- | --- |
| `header.stampMonotonic` | Capture time in the local host clock domain. The bundled Python publisher uses `time.time_ns()`. | Matching images to odometry. |
| `header.clockOffset` | `0` if `stampMonotonic` is already in the local clock domain. | Added to `stampMonotonic` before buffering. |
| `header.seq` | Monotonic frame counter. | Debugging, recordings. |
| `encoding` | `jpeg`. | MapMatcher decodes via OpenCV as grayscale. |
| `width`, `height` | Decoded pixel dimensions. | Camera model + image validation. |
| `data` | JPEG bytes. | Source image. |
| `intrinsic.kb4` | Calibrated KB4: `fx, fy, cx, cy, k1..k4`. | Camera model when no separate calibration file is supplied. |
| `extrinsic.bodyFrame` | Nadir default: `(w=0.00202, x=0.70568, y=0.70853, z=0.00170)`, zero translation. Legacy; runtime body-frame handling lives in vehicle calibration. | Historical projection field. |
| `extrinsic.imuFrame` | Identity: `(0,0,0)` / `(w=1, x=0, y=0, z=0)`. Helpers default to identity. | Reserved; not consumed. |

### Recommended fields

| Field | Value |
| --- | --- |
| `exposureUSec` | From the camera if available, else a stable estimate. |
| `gain` | From the camera if available. |
| `sensorIdx` | `0` for the primary nadir camera. |
| `streamName` | Stable name, e.g. `external_nadir_camera`. |

Timing is the part that breaks integrations. If your camera stamps in a
different clock domain, convert before publishing or pass the offset via
`header.clockOffset` (`clock_offset_ns` in the helper).

## Python helper

`cyclops_camera_ecal.py` wraps the eCAL + Cap'n Proto details. Don't change
`DEFAULT_TOPIC` or `DEFAULT_ECAL_TYPE` — they're the wire contract.

```python
import cv2
from cyclops_camera_ecal import CyclopsCameraPublisher, load_camera_calibration

calibration = load_camera_calibration("camera_params.example.yaml")
publisher = CyclopsCameraPublisher()

camera = cv2.VideoCapture(0)
while True:
    ok, frame = camera.read()
    if not ok:
        break
    publisher.publish_frame(frame, calibration)
```

If your stack already produces JPEG:

```python
publisher.publish_jpeg(
    jpeg_bytes,
    width=640,
    height=512,
    calibration=calibration,
    exposure_usec=10000,
    gain=100,
    stream_name="external_nadir_camera",
)
```

The YAML shape matches `camera_params.example.yaml`. Drop in your calibrated
intrinsics and extrinsics.

### Runnable example

`example_usb_camera.py` reads `resolution` and `fps` from the YAML and
applies them to the cv2 capture, so published intrinsics match captured
frames. `--device` accepts an OpenCV index or `/dev/videoN`. Ctrl-C exits
cleanly. The script omits the hot-plug, v4l2 introspection, and thermal
handling the bundled publisher does — it's a starting point, not a
drop-in.

### Verifying output

```bash
python3 example_image_subscriber.py --topic S1/cama
python3 example_image_subscriber.py --topic S1/cama --no-display --print-every 1
```

In the printed summary:

- `encoding=jpeg` and the expected `width x height`.
- `seq` increasing monotonically; `fps~=` matches the camera rate.
- `intrinsics` match `camera_params.yaml`.
- `body_frame` = `(0.00202, 0.70568, 0.70853, 0.00170)`.
- `imu_frame` = identity.

The subscriber loads the schema from `capnp/`, so it runs without the
Cyclops SDK on `PYTHONPATH`.

## C++ helper

`cyclops_camera_ecal.{hpp,cpp}`. Depends on eCAL, Cap'n Proto, OpenCV, and
the generated `image.capnp.h` (built from `capnp/`). Don't change
`kDefaultTopic` or `kDefaultEcalType`.

```cpp
#include "cyclops_camera_ecal.hpp"
#include <opencv2/videoio.hpp>

int main() {
    namespace cam = theseus::cyclops::camera;

    cam::CameraCalibration calibration;
    calibration.intrinsics.fx = 411.37f;
    calibration.intrinsics.fy = 411.22f;
    calibration.intrinsics.cx = 317.80f;
    calibration.intrinsics.cy = 257.89f;
    calibration.intrinsics.k1 = 0.133336f;
    calibration.intrinsics.k4 = 0.52395f;
    calibration.body_frame.orientation_wxyz = {0.00202, 0.70568, 0.70853, 0.00170};

    cam::CyclopsCameraPublisher publisher;

    cv::VideoCapture capture(0);
    cv::Mat frame;
    while (capture.read(frame)) {
        publisher.publishFrame(frame, calibration);
    }
}
```

JPEG passthrough:

```cpp
cam::PublishOptions options;
options.exposure_usec = 10000;
options.gain = 100;
options.stream_name = "external_nadir_camera";

publisher.publishJpeg(jpeg_bytes, width, height, calibration, options);
```

### Build

```bash
sudo apt-get install -y libopencv-dev libyaml-cpp-dev capnproto libcapnp-dev
cmake -S . -B build
cmake --build build
```

The bundled `CMakeLists.txt` generates `image.capnp.h` from `capnp/`, so
vns-sdk isn't required. To embed in your own project, drop the two C++
files in or add this folder as a subdirectory and link
`cyclops_camera_ecal_cpp`.

### Runnable example

`example_usb_camera.cpp` mirrors the Python example line-for-line; the
only meaningful difference is the YAML loader (`yaml-cpp` here,
`load_camera_calibration` there). Built automatically when
`libyaml-cpp-dev` is present.

```bash
./build/example_usb_camera
./build/example_usb_camera --device /dev/video2 --config my_cam.yaml
```

## Integration checklist

1. Start your publisher on the same host or eCAL network as Cyclops.
2. Confirm `S1/cama` shows in `ecal_mon_gui` or the Vozilla System page.
3. Run `example_image_subscriber.py --topic S1/cama` and check calibration,
   timing, and payload.
4. Confirm a stable rate near your camera frame rate.
5. Use the Vozilla snapshot/calibration page to verify image orientation
   and that nothing is occluded.
6. Run the Cyclops bench-test flow from the public docs.
