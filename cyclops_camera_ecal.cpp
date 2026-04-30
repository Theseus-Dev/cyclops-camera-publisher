#include "cyclops_camera_ecal.hpp"

#include <capnp/serialize.h>
#include <kj/array.h>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>

#include <chrono>
#include <cmath>
#include <sstream>
#include <stdexcept>
#include <utility>

#include "image.capnp.h"

namespace theseus::cyclops::camera {
namespace {

template <typename PoseBuilder>
void writePose(PoseBuilder pose_builder, const Pose3D& pose) {
    auto position = pose_builder.initPosition();
    position.setX(pose.position_x_m);
    position.setY(pose.position_y_m);
    position.setZ(pose.position_z_m);

    auto orientation = pose_builder.initOrientation();
    orientation.setW(pose.orientation_wxyz[0]);
    orientation.setX(pose.orientation_wxyz[1]);
    orientation.setY(pose.orientation_wxyz[2]);
    orientation.setZ(pose.orientation_wxyz[3]);
}

const Pose3D& resolvedImuFrame(const CameraCalibration& calibration) {
    return calibration.imu_frame.has_value() ? *calibration.imu_frame : calibration.body_frame;
}

void validatePose(const Pose3D& pose, const char* name) {
    const double q_norm_sq = pose.orientation_wxyz[0] * pose.orientation_wxyz[0] +
                             pose.orientation_wxyz[1] * pose.orientation_wxyz[1] +
                             pose.orientation_wxyz[2] * pose.orientation_wxyz[2] +
                             pose.orientation_wxyz[3] * pose.orientation_wxyz[3];
    if (q_norm_sq <= 0.0) {
        throw std::invalid_argument(std::string(name) + " quaternion must be non-zero");
    }
    if (std::abs(q_norm_sq - 1.0) > 0.05) {
        std::ostringstream error;
        error << name << " quaternion should be normalized; norm^2=" << q_norm_sq;
        throw std::invalid_argument(error.str());
    }
}

}  // namespace

CyclopsCameraPublisher::CyclopsCameraPublisher(std::string topic, std::string process_name, bool initialize_ecal,
                                               bool finalize_ecal_on_destroy)
    : topic_(std::move(topic)), finalize_ecal_on_destroy_(finalize_ecal_on_destroy) {
    if (initialize_ecal && !eCAL::IsInitialized()) {
        eCAL::Initialize(0, nullptr, process_name.c_str());
        owns_ecal_init_ = true;
    }

    if (!publisher_.Create(topic_, kDefaultEcalType, "Cyclops nadir camera image")) {
        throw std::runtime_error("failed to create eCAL camera publisher on topic " + topic_);
    }
}

CyclopsCameraPublisher::~CyclopsCameraPublisher() {
    publisher_.Destroy();
    if (owns_ecal_init_ && finalize_ecal_on_destroy_) {
        eCAL::Finalize();
    }
}

size_t CyclopsCameraPublisher::publishFrame(const cv::Mat& frame, const CameraCalibration& calibration,
                                            PublishOptions options) {
    EncodedJpeg encoded = encodeGrayscaleJpeg(frame, options.jpeg_quality);
    return publishJpeg(encoded.bytes, encoded.width, encoded.height, calibration, std::move(options));
}

size_t CyclopsCameraPublisher::publishJpeg(const std::vector<uint8_t>& jpeg_bytes, uint32_t width, uint32_t height,
                                           const CameraCalibration& calibration, PublishOptions options) {
    if (jpeg_bytes.empty()) {
        throw std::invalid_argument("jpeg_bytes must not be empty");
    }
    validateCalibration(calibration, width, height);

    const uint64_t capture_time_ns =
        options.capture_time_ns != 0 ? options.capture_time_ns : currentHostTimeNs();

    capnp::MallocMessageBuilder message;
    vkc::Image::Builder image = message.initRoot<vkc::Image>();

    auto header = image.initHeader();
    header.setStampMonotonic(capture_time_ns);
    header.setClockOffset(options.clock_offset_ns);
    header.setSeq(next_sequence_);

    image.setEncoding(vkc::Image::Encoding::JPEG);
    image.setWidth(width);
    image.setHeight(height);
    image.setStep(0);
    image.setData(kj::arrayPtr(jpeg_bytes.data(), jpeg_bytes.size()));

    image.setExposureUSec(options.exposure_usec);
    image.setGain(options.gain);
    image.setSensorIdx(options.sensor_idx);
    image.setStreamName(options.stream_name);

    auto intrinsic = image.initIntrinsic();
    auto kb4 = intrinsic.initKb4();
    auto pinhole = kb4.initPinhole();
    pinhole.setFx(calibration.intrinsics.fx);
    pinhole.setFy(calibration.intrinsics.fy);
    pinhole.setCx(calibration.intrinsics.cx);
    pinhole.setCy(calibration.intrinsics.cy);
    kb4.setK1(calibration.intrinsics.k1);
    kb4.setK2(calibration.intrinsics.k2);
    kb4.setK3(calibration.intrinsics.k3);
    kb4.setK4(calibration.intrinsics.k4);
    intrinsic.setRectified(calibration.rectified);
    intrinsic.setLastModified(currentHostTimeNs());

    auto extrinsic = image.initExtrinsic();
    writePose(extrinsic.initBodyFrame(), calibration.body_frame);
    writePose(extrinsic.initImuFrame(), resolvedImuFrame(calibration));
    extrinsic.setLastModified(currentHostTimeNs());

    kj::Array<capnp::word> words = capnp::messageToFlatArray(message);
    kj::ArrayPtr<const char> bytes(reinterpret_cast<const char*>(words.begin()), words.size() * sizeof(capnp::word));
    const size_t sent = publisher_.Send(bytes.begin(), bytes.size());

    ++next_sequence_;
    return sent;
}

EncodedJpeg encodeGrayscaleJpeg(const cv::Mat& frame, int jpeg_quality) {
    if (frame.empty()) {
        throw std::invalid_argument("frame must not be empty");
    }
    if (jpeg_quality < 1 || jpeg_quality > 100) {
        throw std::invalid_argument("jpeg_quality must be in the range [1, 100]");
    }

    cv::Mat gray;
    if (frame.depth() == CV_16U) {
        cv::Mat normalized;
        cv::normalize(frame, normalized, 0, 255, cv::NORM_MINMAX, CV_8U);
        if (normalized.channels() == 1) {
            gray = normalized;
        } else if (normalized.channels() == 3) {
            cv::cvtColor(normalized, gray, cv::COLOR_BGR2GRAY);
        } else if (normalized.channels() == 4) {
            cv::cvtColor(normalized, gray, cv::COLOR_BGRA2GRAY);
        } else {
            throw std::invalid_argument("unsupported uint16 frame channel count");
        }
    } else if (frame.type() == CV_8UC1) {
        gray = frame;
    } else if (frame.type() == CV_8UC3) {
        cv::cvtColor(frame, gray, cv::COLOR_BGR2GRAY);
    } else if (frame.type() == CV_8UC4) {
        cv::cvtColor(frame, gray, cv::COLOR_BGRA2GRAY);
    } else {
        throw std::invalid_argument("unsupported frame type; expected 8-bit gray/BGR/BGRA or uint16 thermal");
    }

    std::vector<uint8_t> bytes;
    if (!cv::imencode(".jpg", gray, bytes, {cv::IMWRITE_JPEG_QUALITY, jpeg_quality})) {
        throw std::runtime_error("JPEG encoding failed");
    }

    EncodedJpeg encoded;
    encoded.bytes = std::move(bytes);
    encoded.width = static_cast<uint32_t>(gray.cols);
    encoded.height = static_cast<uint32_t>(gray.rows);
    return encoded;
}

uint64_t currentHostTimeNs() {
    return static_cast<uint64_t>(std::chrono::duration_cast<std::chrono::nanoseconds>(
                                     std::chrono::system_clock::now().time_since_epoch())
                                     .count());
}

void validateCalibration(const CameraCalibration& calibration, uint32_t width, uint32_t height) {
    if (width == 0 || height == 0) {
        throw std::invalid_argument("width and height must be positive");
    }
    if (calibration.intrinsics.fx <= 0.0f || calibration.intrinsics.fy <= 0.0f) {
        throw std::invalid_argument("fx and fy must be positive");
    }
    if (calibration.intrinsics.cx < 0.0f || calibration.intrinsics.cx > static_cast<float>(width)) {
        throw std::invalid_argument("cx must be inside the image width");
    }
    if (calibration.intrinsics.cy < 0.0f || calibration.intrinsics.cy > static_cast<float>(height)) {
        throw std::invalid_argument("cy must be inside the image height");
    }

    validatePose(calibration.body_frame, "body_frame");
    validatePose(resolvedImuFrame(calibration), "imu_frame");
}

}  // namespace theseus::cyclops::camera
