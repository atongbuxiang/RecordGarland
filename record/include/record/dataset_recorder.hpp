#pragma once

#include <array>
#include <atomic>
#include <condition_variable>
#include <deque>
#include <fstream>
#include <memory>
#include <mutex>
#include <optional>
#include <string>
#include <thread>
#include <vector>

#include <nav_msgs/msg/odometry.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp/node_options.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/imu.hpp>

#include <record/blocking_queue.hpp>
#include <record/recorder_types.hpp>
#include <record/video_writer.hpp>

namespace record {

class DatasetRecorder : public rclcpp::Node, public VideoWriterEvents {
public:
  explicit DatasetRecorder(const rclcpp::NodeOptions &options = rclcpp::NodeOptions());
  ~DatasetRecorder() override;

  void record_camera_frame(size_t camera_index, uint64_t frame_index, double stamp) override;
  void report_writer_error(const std::string &camera_name, const std::string &message) override;

private:
  void validate_parameters();
  void setup_output_files();
  void setup_writers();
  void setup_subscribers();
  void start_recording_if_ready_locked();

  void image_callback(size_t camera_index, const sensor_msgs::msg::Image::ConstSharedPtr &msg);
  void imu_callback(const sensor_msgs::msg::Imu::ConstSharedPtr &msg);
  void odom_callback(const nav_msgs::msg::Odometry::ConstSharedPtr &msg);

  void try_form_frame_groups_locked();
  void metadata_loop();
  void wait_for_pose_window(const PendingFrameGroup &group);
  std::optional<PoseSample> find_nearest_pose(double stamp) const;
  void write_frame_group(const PendingFrameGroup &group, const std::optional<PoseSample> &pose);
  void print_fps();
  void finalize_json_files();
  void write_calibration_json();

  std::vector<std::string> camera_names_;
  std::vector<std::string> camera_topics_;
  std::string imu_topic_;
  std::string odom_topic_;
  std::string output_dir_;
  std::string session_name_;
  std::filesystem::path session_dir_;
  std::filesystem::path videos_dir_;
  double sync_threshold_sec_{0.010};
  double pose_lookahead_sec_{0.020};
  std::chrono::duration<double, std::milli> pose_wait_timeout_{150.0};
  double pose_buffer_sec_{10.0};
  double video_fps_{40.0};
  std::string video_fourcc_{"MJPG"};
  std::string video_extension_{".avi"};
  size_t queue_warning_frames_{1000};
  std::string estimator_config_path_;
  std::string imucam_config_path_;
  std::string imu_config_path_;

  std::vector<std::unique_ptr<CameraVideoWriter>> video_writers_;
  std::vector<rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr> image_subscriptions_;
  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_subscription_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_subscription_;
  rclcpp::TimerBase::SharedPtr fps_timer_;

  std::array<std::atomic<uint64_t>, kNumCameras> camera_frame_counts_{};
  std::array<std::atomic<uint64_t>, kNumCameras> camera_fps_counts_{};
  std::atomic<uint64_t> imu_fps_count_{0};
  std::atomic<uint64_t> grouped_frame_fps_count_{0};
  std::atomic<uint64_t> unmatched_frame_count_{0};
  std::atomic<uint64_t> grouped_span_samples_{0};
  std::atomic<uint64_t> grouped_span_sum_us_{0};
  std::atomic<uint64_t> grouped_span_max_us_{0};
  std::atomic<uint64_t> unmatched_span_samples_{0};
  std::atomic<uint64_t> unmatched_span_sum_us_{0};
  std::atomic<uint64_t> unmatched_span_max_us_{0};

  mutable std::mutex sync_mutex_;
  std::array<std::deque<SyncFrameRef>, kNumCameras> pending_camera_frames_;
  uint64_t grouped_frame_count_{0};
  BlockingQueue<PendingFrameGroup> pending_groups_;
  std::thread metadata_thread_;

  mutable std::mutex pose_mutex_;
  std::condition_variable pose_cv_;
  std::deque<PoseSample> pose_buffer_;
  double latest_pose_stamp_{0.0};

  std::ofstream imu_file_;
  std::ofstream frames_file_;
  std::ofstream camera_frames_file_;
  std::mutex imu_file_mutex_;
  std::mutex frames_file_mutex_;
  std::mutex camera_frames_mutex_;
  bool imu_first_{true};
  bool frames_first_{true};
  bool camera_frames_first_{true};
  std::atomic<bool> wait_for_imu_and_pose_{true};
  std::atomic<bool> recording_started_{false};
  std::atomic<bool> have_imu_{false};
  std::atomic<bool> have_pose_{false};
  std::atomic<bool> announced_waiting_{false};
};

} // namespace record
