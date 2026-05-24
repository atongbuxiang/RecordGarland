#include <record/dataset_recorder.hpp>

#include <algorithm>
#include <filesystem>
#include <iomanip>
#include <limits>
#include <opencv2/videoio.hpp>
#include <sstream>
#include <stdexcept>

#include <record/recorder_utils.hpp>

namespace fs = std::filesystem;
using namespace std::chrono_literals;

namespace record {

DatasetRecorder::DatasetRecorder() : Node("oak_dataset_recorder") {
  camera_names_ = declare_parameter<std::vector<std::string>>("camera_names", {"CAM_A", "CAM_B", "CAM_C", "CAM_D"});
  camera_topics_ =
      declare_parameter<std::vector<std::string>>("camera_topics", {"/CAM_A/image", "/CAM_B/image", "/CAM_C/image", "/CAM_D/image"});
  imu_topic_ = declare_parameter<std::string>("imu_topic", "/imu");
  odom_topic_ = declare_parameter<std::string>("odom_topic", "/ov_msckf/odomimu");
  output_dir_ = expand_user_path(declare_parameter<std::string>("output_dir", "~/oak_datasets"));
  session_name_ = declare_parameter<std::string>("session_name", "");
  if (session_name_.empty()) {
    session_name_ = make_default_session_name();
  }

  sync_threshold_sec_ = declare_parameter<double>("sync_threshold_ms", 10.0) * 1e-3;
  pose_lookahead_sec_ = declare_parameter<double>("pose_lookahead_ms", 20.0) * 1e-3;
  pose_wait_timeout_ = std::chrono::duration<double, std::milli>(declare_parameter<double>("pose_wait_timeout_ms", 150.0));
  pose_buffer_sec_ = declare_parameter<double>("pose_buffer_sec", 10.0);
  video_fps_ = declare_parameter<double>("video_fps", 40.0);
  video_fourcc_ = declare_parameter<std::string>("video_fourcc", "MJPG");
  video_extension_ = normalize_extension(declare_parameter<std::string>("video_extension", ".avi"));
  queue_warning_frames_ = static_cast<size_t>(declare_parameter<int>("queue_warning_frames", 1000));
  estimator_config_path_ = declare_parameter<std::string>("estimator_config_path", "");
  imucam_config_path_ = declare_parameter<std::string>("imucam_config_path", "");
  imu_config_path_ = declare_parameter<std::string>("imu_config_path", "");

  validate_parameters();
  setup_output_files();
  setup_writers();
  setup_subscribers();

  metadata_thread_ = std::thread(&DatasetRecorder::metadata_loop, this);
  fps_timer_ = create_wall_timer(1s, std::bind(&DatasetRecorder::print_fps, this));

  RCLCPP_INFO(get_logger(), "recording dataset to %s", session_dir_.c_str());
}

DatasetRecorder::~DatasetRecorder() {
  for (auto &sub : image_subscriptions_) {
    sub.reset();
  }
  imu_subscription_.reset();
  odom_subscription_.reset();

  pending_groups_.close();
  if (metadata_thread_.joinable()) {
    metadata_thread_.join();
  }

  for (auto &writer : video_writers_) {
    writer->stop();
  }
  finalize_json_files();
  write_calibration_json();
}

void DatasetRecorder::record_camera_frame(size_t camera_index, uint64_t frame_index, double stamp) {
  std::lock_guard<std::mutex> lock(camera_frames_mutex_);
  if (!camera_frames_first_) {
    camera_frames_file_ << ",\n";
  }
  camera_frames_first_ = false;
  camera_frames_file_ << "    {\"camera\":\"" << json_escape(camera_names_.at(camera_index)) << "\",\"frame_index\":" << frame_index
                      << ",\"stamp\":" << std::fixed << std::setprecision(9) << stamp << "}";
}

void DatasetRecorder::report_writer_error(const std::string &camera_name, const std::string &message) {
  RCLCPP_ERROR(get_logger(), "[%s] %s", camera_name.c_str(), message.c_str());
}

void DatasetRecorder::validate_parameters() {
  if (camera_names_.size() != kNumCameras || camera_topics_.size() != kNumCameras) {
    throw std::runtime_error("camera_names and camera_topics must each contain exactly 4 entries");
  }
  if (video_fps_ <= 0.0) {
    throw std::runtime_error("video_fps must be positive");
  }
  if (sync_threshold_sec_ <= 0.0) {
    throw std::runtime_error("sync_threshold_ms must be positive");
  }
  fourcc_from_string(video_fourcc_);
}

void DatasetRecorder::setup_output_files() {
  session_dir_ = fs::path(output_dir_) / session_name_;
  videos_dir_ = session_dir_ / "videos";
  fs::create_directories(videos_dir_);

  imu_file_.open(session_dir_ / "imu.json", std::ios::out | std::ios::trunc);
  frames_file_.open(session_dir_ / "video_frames.json", std::ios::out | std::ios::trunc);
  camera_frames_file_.open(session_dir_ / "camera_frames.json", std::ios::out | std::ios::trunc);
  if (!imu_file_ || !frames_file_ || !camera_frames_file_) {
    throw std::runtime_error("failed to open dataset metadata files");
  }

  imu_file_ << "{\n  \"topic\":\"" << json_escape(imu_topic_) << "\",\n  \"messages\":[\n";
  frames_file_ << "{\n  \"sync_threshold_sec\":" << std::fixed << std::setprecision(9) << sync_threshold_sec_
               << ",\n  \"pose_topic\":\"" << json_escape(odom_topic_)
               << "\",\n  \"pose_format\":\"xyz_qxyzw\",\n  \"camera_order\":[";
  for (size_t i = 0; i < kNumCameras; ++i) {
    if (i != 0) {
      frames_file_ << ",";
    }
    frames_file_ << "\"" << json_escape(camera_names_.at(i)) << "\"";
  }
  frames_file_ << "],\n  \"frames\":[\n";
  camera_frames_file_ << "{\n  \"cameras\":[";
  for (size_t i = 0; i < kNumCameras; ++i) {
    if (i != 0) {
      camera_frames_file_ << ",";
    }
    camera_frames_file_ << "{\"name\":\"" << json_escape(camera_names_.at(i)) << "\",\"topic\":\""
                        << json_escape(camera_topics_.at(i)) << "\"}";
  }
  camera_frames_file_ << "],\n  \"frames\":[\n";
}

void DatasetRecorder::setup_writers() {
  const int fourcc = fourcc_from_string(video_fourcc_);
  for (size_t i = 0; i < kNumCameras; ++i) {
    fs::path video_path = videos_dir_ / (camera_names_.at(i) + video_extension_);
    video_writers_.push_back(
        std::make_unique<CameraVideoWriter>(this, i, camera_names_.at(i), video_path, video_fps_, fourcc, queue_warning_frames_));
  }
}

void DatasetRecorder::setup_subscribers() {
  auto image_qos = rclcpp::SensorDataQoS().keep_last(256);
  auto imu_qos = rclcpp::SensorDataQoS().keep_last(2048);
  auto odom_qos = rclcpp::QoS(rclcpp::KeepLast(64)).best_effort();

  for (size_t i = 0; i < kNumCameras; ++i) {
    image_subscriptions_.push_back(create_subscription<sensor_msgs::msg::Image>(
        camera_topics_.at(i), image_qos, [this, i](sensor_msgs::msg::Image::ConstSharedPtr msg) { image_callback(i, msg); }));
    RCLCPP_INFO(get_logger(), "subscribing to %s", camera_topics_.at(i).c_str());
  }

  imu_subscription_ =
      create_subscription<sensor_msgs::msg::Imu>(imu_topic_, imu_qos,
                                                 [this](sensor_msgs::msg::Imu::ConstSharedPtr msg) { imu_callback(msg); });
  odom_subscription_ =
      create_subscription<nav_msgs::msg::Odometry>(odom_topic_, odom_qos,
                                                   [this](nav_msgs::msg::Odometry::ConstSharedPtr msg) { odom_callback(msg); });
}

void DatasetRecorder::image_callback(size_t camera_index, const sensor_msgs::msg::Image::ConstSharedPtr &msg) {
  const double stamp = stamp_to_seconds(msg->header.stamp);
  const uint64_t frame_index = camera_frame_counts_.at(camera_index).fetch_add(1, std::memory_order_relaxed);
  camera_fps_counts_.at(camera_index).fetch_add(1, std::memory_order_relaxed);

  video_writers_.at(camera_index)->enqueue(CameraFrame{msg, frame_index, stamp});

  {
    std::lock_guard<std::mutex> lock(sync_mutex_);
    pending_camera_frames_.at(camera_index).push_back(SyncFrameRef{frame_index, stamp});
    try_form_frame_groups_locked();
  }
}

void DatasetRecorder::imu_callback(const sensor_msgs::msg::Imu::ConstSharedPtr &msg) {
  const double stamp = stamp_to_seconds(msg->header.stamp);
  imu_fps_count_.fetch_add(1, std::memory_order_relaxed);
  std::lock_guard<std::mutex> lock(imu_file_mutex_);
  if (!imu_first_) {
    imu_file_ << ",\n";
  }
  imu_first_ = false;
  imu_file_ << "    {\"stamp\":" << std::fixed << std::setprecision(9) << stamp << ",\"angular_velocity\":["
            << msg->angular_velocity.x << "," << msg->angular_velocity.y << "," << msg->angular_velocity.z
            << "],\"linear_acceleration\":[" << msg->linear_acceleration.x << "," << msg->linear_acceleration.y << ","
            << msg->linear_acceleration.z << "],\"orientation\":[" << msg->orientation.x << "," << msg->orientation.y << ","
            << msg->orientation.z << "," << msg->orientation.w << "]}";
}

void DatasetRecorder::odom_callback(const nav_msgs::msg::Odometry::ConstSharedPtr &msg) {
  PoseSample pose;
  pose.stamp = stamp_to_seconds(msg->header.stamp);
  pose.px = msg->pose.pose.position.x;
  pose.py = msg->pose.pose.position.y;
  pose.pz = msg->pose.pose.position.z;
  pose.qx = msg->pose.pose.orientation.x;
  pose.qy = msg->pose.pose.orientation.y;
  pose.qz = msg->pose.pose.orientation.z;
  pose.qw = msg->pose.pose.orientation.w;

  {
    std::lock_guard<std::mutex> lock(pose_mutex_);
    pose_buffer_.push_back(pose);
    const double min_stamp = pose.stamp - pose_buffer_sec_;
    while (!pose_buffer_.empty() && pose_buffer_.front().stamp < min_stamp) {
      pose_buffer_.pop_front();
    }
    latest_pose_stamp_ = pose.stamp;
  }
  pose_cv_.notify_all();
}

void DatasetRecorder::try_form_frame_groups_locked() {
  while (std::all_of(pending_camera_frames_.begin(), pending_camera_frames_.end(), [](const auto &frames) { return !frames.empty(); })) {
    double min_stamp = std::numeric_limits<double>::max();
    double max_stamp = -std::numeric_limits<double>::max();
    size_t min_index = 0;
    for (size_t i = 0; i < kNumCameras; ++i) {
      const double stamp = pending_camera_frames_.at(i).front().stamp;
      if (stamp < min_stamp) {
        min_stamp = stamp;
        min_index = i;
      }
      max_stamp = std::max(max_stamp, stamp);
    }

    if (max_stamp - min_stamp <= sync_threshold_sec_) {
      PendingFrameGroup group;
      group.group_index = grouped_frame_count_++;
      group.created_at = std::chrono::steady_clock::now();
      double sum = 0.0;
      for (size_t i = 0; i < kNumCameras; ++i) {
        const SyncFrameRef ref = pending_camera_frames_.at(i).front();
        pending_camera_frames_.at(i).pop_front();
        group.camera_stamps.at(i) = ref.stamp;
        group.camera_frame_indices.at(i) = ref.frame_index;
        sum += ref.stamp;
      }
      group.group_stamp = sum / static_cast<double>(kNumCameras);
      pending_groups_.push(group);
    } else {
      unmatched_frame_count_.fetch_add(1, std::memory_order_relaxed);
      pending_camera_frames_.at(min_index).pop_front();
    }
  }
}

void DatasetRecorder::metadata_loop() {
  PendingFrameGroup group;
  while (pending_groups_.pop(group)) {
    wait_for_pose_window(group);
    const auto pose = find_nearest_pose(group.group_stamp);
    write_frame_group(group, pose);
  }
}

void DatasetRecorder::wait_for_pose_window(const PendingFrameGroup &group) {
  std::unique_lock<std::mutex> lock(pose_mutex_);
  const auto deadline = group.created_at + std::chrono::duration_cast<std::chrono::steady_clock::duration>(pose_wait_timeout_);
  pose_cv_.wait_until(lock, deadline,
                      [this, &group] { return latest_pose_stamp_ >= group.group_stamp + pose_lookahead_sec_ || !rclcpp::ok(); });
}

std::optional<PoseSample> DatasetRecorder::find_nearest_pose(double stamp) const {
  std::lock_guard<std::mutex> lock(pose_mutex_);
  if (pose_buffer_.empty()) {
    return std::nullopt;
  }
  auto best = pose_buffer_.begin();
  double best_dt = std::abs(best->stamp - stamp);
  for (auto it = std::next(pose_buffer_.begin()); it != pose_buffer_.end(); ++it) {
    const double dt = std::abs(it->stamp - stamp);
    if (dt < best_dt) {
      best = it;
      best_dt = dt;
    }
  }
  return *best;
}

void DatasetRecorder::write_frame_group(const PendingFrameGroup &group, const std::optional<PoseSample> &pose) {
  std::lock_guard<std::mutex> lock(frames_file_mutex_);
  if (!frames_first_) {
    frames_file_ << ",\n";
  }
  frames_first_ = false;

  frames_file_ << "    {\"i\":" << group.group_index << ",\"t\":" << std::fixed << std::setprecision(9) << group.group_stamp
               << ",\"t_cam\":[";
  for (size_t i = 0; i < kNumCameras; ++i) {
    if (i != 0) {
      frames_file_ << ",";
    }
    frames_file_ << group.camera_stamps.at(i);
  }
  frames_file_ << "],\"video_i\":[";
  for (size_t i = 0; i < kNumCameras; ++i) {
    if (i != 0) {
      frames_file_ << ",";
    }
    frames_file_ << group.camera_frame_indices.at(i);
  }
  frames_file_ << "],\"pose\":";
  if (pose) {
    const double dt = pose->stamp - group.group_stamp;
    frames_file_ << "{\"t\":" << pose->stamp << ",\"dt\":" << dt << ",\"xyz_qxyzw\":[" << pose->px << "," << pose->py << ","
                 << pose->pz << "," << pose->qx << "," << pose->qy << "," << pose->qz << "," << pose->qw << "]}";
  } else {
    frames_file_ << "null";
  }
  frames_file_ << "}";
}

void DatasetRecorder::print_fps() {
  std::ostringstream line;
  line << "[dataset fps]";
  for (size_t i = 0; i < kNumCameras; ++i) {
    const auto count = camera_fps_counts_.at(i).exchange(0, std::memory_order_relaxed);
    line << " " << camera_names_.at(i) << "=" << count;
    const size_t backlog = video_writers_.at(i)->queue_size();
    if (backlog > queue_warning_frames_) {
      line << "(queue=" << backlog << ")";
    }
  }
  line << " imu=" << imu_fps_count_.exchange(0, std::memory_order_relaxed);
  const auto unmatched = unmatched_frame_count_.exchange(0, std::memory_order_relaxed);
  if (unmatched != 0) {
    line << " unmatched=" << unmatched;
  }
  RCLCPP_INFO(get_logger(), "%s", line.str().c_str());
}

void DatasetRecorder::finalize_json_files() {
  {
    std::lock_guard<std::mutex> lock(imu_file_mutex_);
    if (imu_file_.is_open()) {
      imu_file_ << "\n  ]\n}\n";
      imu_file_.close();
    }
  }
  {
    std::lock_guard<std::mutex> lock(frames_file_mutex_);
    if (frames_file_.is_open()) {
      frames_file_ << "\n  ]\n}\n";
      frames_file_.close();
    }
  }
  {
    std::lock_guard<std::mutex> lock(camera_frames_mutex_);
    if (camera_frames_file_.is_open()) {
      camera_frames_file_ << "\n  ]\n}\n";
      camera_frames_file_.close();
    }
  }
}

void DatasetRecorder::write_calibration_json() {
  std::ofstream out(session_dir_ / "calibration.json", std::ios::out | std::ios::trunc);
  if (!out) {
    RCLCPP_ERROR(get_logger(), "failed to write calibration.json");
    return;
  }

  out << "{\n";
  out << "  \"camera_topics\":[";
  for (size_t i = 0; i < kNumCameras; ++i) {
    if (i != 0) {
      out << ",";
    }
    out << "{\"name\":\"" << json_escape(camera_names_.at(i)) << "\",\"image_topic\":\"" << json_escape(camera_topics_.at(i)) << "\"}";
  }
  out << "],\n";
  out << "  \"config_files\":{\n";
  out << "    \"estimator_config\":{\"path\":\"" << json_escape(estimator_config_path_) << "\",\"contents\":\""
      << json_escape(read_text_file(estimator_config_path_)) << "\"},\n";
  out << "    \"imucam_config\":{\"path\":\"" << json_escape(imucam_config_path_) << "\",\"contents\":\""
      << json_escape(read_text_file(imucam_config_path_)) << "\"},\n";
  out << "    \"imu_config\":{\"path\":\"" << json_escape(imu_config_path_) << "\",\"contents\":\""
      << json_escape(read_text_file(imu_config_path_)) << "\"}\n";
  out << "  }\n";
  out << "}\n";
}

} // namespace record
