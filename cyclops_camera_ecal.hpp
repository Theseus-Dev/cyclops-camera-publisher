#pragma once

#include <ecal/ecal.h>
#include <opencv2/core.hpp>

#include <array>
#include <cstdint>
#include <optional>
#include <string>
#include <vector>

namespace theseus::cyclops::camera {

// Wire contract — don't change without a coordinated Cyclops-side change.
inline constexpr char kDefaultTopic[] = "S1/cama";
inline constexpr char kDefaultEcalType[] = "capnp:Image";

// Informational. Keep stable for log/recording comparability.
inline constexpr char kDefaultProcessName[] = "CyclopsExternalCamera";
inline constexpr char kDefaultStreamName[] = "external_nadir_camera";

struct KB4Intrinsics {
    float fx{0.0f};
    float fy{0.0f};
    float cx{0.0f};
    float cy{0.0f};
    float k1{0.0f};
    float k2{0.0f};
    float k3{0.0f};
    float k4{0.0f};
};

struct Pose3D {
    double position_x_m{0.0};
    double position_y_m{0.0};
    double position_z_m{0.0};

    // Quaternion in w, x, y, z order.
    std::array<double, 4> orientation_wxyz{1.0, 0.0, 0.0, 0.0};
};

struct CameraCalibration {
    KB4Intrinsics intrinsics;
    Pose3D body_frame;
    // Legacy; not consumed by Cyclops. Helper publishes identity if unset.
    std::optional<Pose3D> imu_frame;
    bool rectified{true};
};

struct PublishOptions {
    // If zero, the helper uses currentHostTimeNs().
    uint64_t capture_time_ns{0};
    // Cyclops adds this to capture_time_ns before matching images to odometry.
    // Use zero when capture_time_ns is already in the Cyclops host clock domain.
    int64_t clock_offset_ns{0};
    uint32_t exposure_usec{10000};
    uint32_t gain{100};
    int8_t sensor_idx{0};
    std::string stream_name{kDefaultStreamName};
    int jpeg_quality{95};
};

struct EncodedJpeg {
    std::vector<uint8_t> bytes;
    uint32_t width{0};
    uint32_t height{0};
};

class CyclopsCameraPublisher {
   public:
    explicit CyclopsCameraPublisher(std::string topic = kDefaultTopic,
                                    std::string process_name = kDefaultProcessName,
                                    bool initialize_ecal = true,
                                    bool finalize_ecal_on_destroy = true);
    ~CyclopsCameraPublisher();

    CyclopsCameraPublisher(const CyclopsCameraPublisher&) = delete;
    CyclopsCameraPublisher& operator=(const CyclopsCameraPublisher&) = delete;

    size_t publishFrame(const cv::Mat& frame, const CameraCalibration& calibration,
                        PublishOptions options = PublishOptions{});

    size_t publishJpeg(const std::vector<uint8_t>& jpeg_bytes, uint32_t width, uint32_t height,
                       const CameraCalibration& calibration, PublishOptions options = PublishOptions{});

    uint64_t nextSequence() const { return next_sequence_; }
    const std::string& topic() const { return topic_; }

   private:
    std::string topic_;
    bool owns_ecal_init_{false};
    bool finalize_ecal_on_destroy_{true};
    uint64_t next_sequence_{0};
    eCAL::CPublisher publisher_;
};

EncodedJpeg encodeGrayscaleJpeg(const cv::Mat& frame, int jpeg_quality = 95);
uint64_t currentHostTimeNs();
void validateCalibration(const CameraCalibration& calibration, uint32_t width, uint32_t height);

}  // namespace theseus::cyclops::camera
