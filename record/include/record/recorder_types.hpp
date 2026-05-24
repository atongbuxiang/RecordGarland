#pragma once

#include <array>
#include <chrono>
#include <cstdint>

#include <sensor_msgs/msg/image.hpp>

namespace record {

constexpr size_t kNumCameras = 4;

struct CameraFrame {
  sensor_msgs::msg::Image::ConstSharedPtr msg;
  uint64_t frame_index{0};
  double stamp{0.0};
};

struct SyncFrameRef {
  uint64_t frame_index{0};
  double stamp{0.0};
};

struct PoseSample {
  double stamp{0.0};
  double px{0.0};
  double py{0.0};
  double pz{0.0};
  double qx{0.0};
  double qy{0.0};
  double qz{0.0};
  double qw{1.0};
};

struct PendingFrameGroup {
  uint64_t group_index{0};
  std::array<double, kNumCameras> camera_stamps{};
  std::array<uint64_t, kNumCameras> camera_frame_indices{};
  double group_stamp{0.0};
  std::chrono::steady_clock::time_point created_at;
};

} // namespace record
