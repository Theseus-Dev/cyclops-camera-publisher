// End-to-end example: publish a USB camera into Cyclops (C++).
//
//     USB camera -> OpenCV -> CyclopsCameraPublisher -> eCAL S1/cama -> Cyclops
//
// C++ counterpart to example_usb_camera.py. Swap cv::VideoCapture for your
// camera SDK; the publishing call is the same.
//
// Build: cmake -S . -B build && cmake --build build
// Run  : ./build/example_usb_camera [--device /dev/videoN] [--config my_cam.yaml]
//
// Dependencies: eCAL, Cap'n Proto, OpenCV (libopencv-dev), yaml-cpp
// (libyaml-cpp-dev). Schema lookup details live in cyclops_camera_ecal.hpp.

#include "cyclops_camera_ecal.hpp"

#include <yaml-cpp/yaml.h>
#include <opencv2/imgproc.hpp>
#include <opencv2/videoio.hpp>

#include <atomic>
#include <chrono>
#include <csignal>
#include <cstdlib>
#include <cstring>
#include <iostream>
#include <stdexcept>
#include <string>
#include <thread>

namespace cam = theseus::cyclops::camera;

namespace {
std::atomic<bool> g_stop_requested{false};

void requestStop(int sig) {
    std::cerr << "\nSignal " << sig << " received, stopping\n";
    g_stop_requested.store(true);
}
}  // namespace

struct CliArgs {
    std::string device = "0";
    std::string config = "camera_params.example.yaml";
};

CliArgs parseArgs(int argc, char** argv) {
    CliArgs args;
    for (int i = 1; i < argc; ++i) {
        std::string flag(argv[i]);
        if ((flag == "--device" || flag == "--config") && i + 1 < argc) {
            const std::string value(argv[++i]);
            if (flag == "--device") args.device = value;
            else if (flag == "--config") args.config = value;
        } else if (flag == "-h" || flag == "--help") {
            std::cout << "Usage: " << argv[0] << " [--device <idx|/dev/videoN>] [--config <yaml>]\n";
            std::exit(0);
        } else {
            std::cerr << "Unknown argument: " << flag << "\n";
            std::exit(2);
        }
    }
    return args;
}

// Open a USB camera with OpenCV. Replace with your own camera SDK.
cv::VideoCapture openCamera(const std::string& device, int width, int height, int fps) {
    cv::VideoCapture camera;
    char* end = nullptr;
    const long parsed = std::strtol(device.c_str(), &end, 10);
    const bool is_index = (end != device.c_str() && *end == '\0');
    if (is_index) {
        camera.open(static_cast<int>(parsed));
    } else {
        camera.open(device);
    }
    if (!camera.isOpened()) {
        throw std::runtime_error("Failed to open camera '" + device + "'");
    }
    camera.set(cv::CAP_PROP_FRAME_WIDTH, width);
    camera.set(cv::CAP_PROP_FRAME_HEIGHT, height);
    camera.set(cv::CAP_PROP_FPS, fps);

    const int actual_w = static_cast<int>(camera.get(cv::CAP_PROP_FRAME_WIDTH));
    const int actual_h = static_cast<int>(camera.get(cv::CAP_PROP_FRAME_HEIGHT));
    const double actual_fps = camera.get(cv::CAP_PROP_FPS);
    std::cerr << "Opened " << device << ": " << actual_w << "x" << actual_h << " @ " << actual_fps << " fps\n";
    return camera;
}

// Load camera_params.yaml. The C++ helper doesn't ship a YAML loader; this
// mirrors the Python helper's load_camera_calibration shape.
cam::CameraCalibration loadCalibration(const std::string& path) {
    YAML::Node root = YAML::LoadFile(path);
    cam::CameraCalibration cal;

    const auto& intr = root["intrinsics"];
    cal.intrinsics.fx = intr["fx"].as<float>();
    cal.intrinsics.fy = intr["fy"].as<float>();
    cal.intrinsics.cx = intr["cx"].as<float>();
    cal.intrinsics.cy = intr["cy"].as<float>();
    cal.intrinsics.k1 = intr["k1"].as<float>(0.0f);
    cal.intrinsics.k2 = intr["k2"].as<float>(0.0f);
    cal.intrinsics.k3 = intr["k3"].as<float>(0.0f);
    cal.intrinsics.k4 = intr["k4"].as<float>(0.0f);

    const auto& body = root["extrinsics"]["body_frame"];
    cal.body_frame.position_x_m = body["position"]["x"].as<double>();
    cal.body_frame.position_y_m = body["position"]["y"].as<double>();
    cal.body_frame.position_z_m = body["position"]["z"].as<double>();
    cal.body_frame.orientation_wxyz = {
        body["orientation"]["w"].as<double>(),
        body["orientation"]["x"].as<double>(),
        body["orientation"]["y"].as<double>(),
        body["orientation"]["z"].as<double>(),
    };

    return cal;
}

int main(int argc, char** argv) {
    const CliArgs args = parseArgs(argc, argv);

    std::signal(SIGINT, requestStop);
    std::signal(SIGTERM, requestStop);

    // One YAML feeds both the calibration and the cv::VideoCapture properties.
    YAML::Node raw = YAML::LoadFile(args.config);
    const int width = raw["resolution"]["width"].as<int>(640);
    const int height = raw["resolution"]["height"].as<int>(512);
    const int fps = raw["fps"].as<int>(30);
    const cam::CameraCalibration calibration = loadCalibration(args.config);

    cv::VideoCapture camera = openCamera(args.device, width, height, fps);
    cam::CyclopsCameraPublisher publisher;

    const auto frame_interval = std::chrono::duration<double>(1.0 / std::max(1, fps));
    int consecutive_failures = 0;
    cv::Mat frame;

    while (!g_stop_requested.load()) {
        const auto t0 = std::chrono::steady_clock::now();

        if (!camera.read(frame) || frame.empty()) {
            if (++consecutive_failures >= 10) {
                std::cerr << "Camera read failed 10 times in a row; exiting\n";
                return 1;
            }
            std::this_thread::sleep_for(std::chrono::milliseconds(50));
            continue;
        }
        consecutive_failures = 0;

        try {
            publisher.publishFrame(frame, calibration);
        } catch (const std::exception& exc) {
            std::cerr << "Publish failed: " << exc.what() << "\n";
        }

        // Bound publish rate; cv::VideoCapture often blocks at native fps.
        const auto elapsed = std::chrono::steady_clock::now() - t0;
        if (elapsed < frame_interval) {
            std::this_thread::sleep_for(frame_interval - elapsed);
        }
    }

    camera.release();
    std::cerr << "Shutdown complete\n";
    return 0;
}
