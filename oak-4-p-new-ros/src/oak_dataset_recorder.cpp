#include "ros2_oak_ffc_sync/oak_dataset_recorder.hpp"

#include <cstdlib>
#include <fstream>
#include <iomanip>
#include <limits>
#include <sstream>
#include <stdexcept>

#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/videoio.hpp>

#include <depthai_bridge/depthaiUtility.hpp>
#include <yaml-cpp/yaml.h>

namespace fs = std::filesystem;

namespace ros2_oak_ffc_sync {

namespace {
cv::Mat planar_to_bgr(const std::vector<uint8_t>& data, int width, int height, bool rgb_order) {
    const size_t plane_size = static_cast<size_t>(width) * static_cast<size_t>(height);
    if (data.size() < plane_size * 3) {
        return {};
    }

    std::vector<cv::Mat> planes(3);
    planes[0] = cv::Mat(height, width, CV_8UC1, const_cast<uint8_t*>(data.data()));
    planes[1] = cv::Mat(height, width, CV_8UC1, const_cast<uint8_t*>(data.data() + plane_size));
    planes[2] = cv::Mat(height, width, CV_8UC1, const_cast<uint8_t*>(data.data() + plane_size * 2));
    if (rgb_order) {
        std::swap(planes[0], planes[2]);
    }

    cv::Mat image;
    cv::merge(planes, image);
    return image.clone();
}

cv::Mat frame_to_bgr(const std::shared_ptr<dai::ImgFrame>& frame) {
    const auto& data = frame->getData();
    const auto width = static_cast<int>(frame->getWidth());
    const auto height = static_cast<int>(frame->getHeight());

    switch (frame->getType()) {
        case dai::RawImgFrame::Type::BITSTREAM:
            return cv::imdecode(data, cv::IMREAD_COLOR);
        case dai::RawImgFrame::Type::BGR888i:
            if (data.size() < static_cast<size_t>(width) * height * 3) {
                return {};
            }
            return cv::Mat(height, width, CV_8UC3, const_cast<uint8_t*>(data.data())).clone();
        case dai::RawImgFrame::Type::RGB888i: {
            if (data.size() < static_cast<size_t>(width) * height * 3) {
                return {};
            }
            cv::Mat rgb(height, width, CV_8UC3, const_cast<uint8_t*>(data.data()));
            cv::Mat bgr;
            cv::cvtColor(rgb, bgr, cv::COLOR_RGB2BGR);
            return bgr;
        }
        case dai::RawImgFrame::Type::BGR888p:
            return planar_to_bgr(data, width, height, false);
        case dai::RawImgFrame::Type::RGB888p:
            return planar_to_bgr(data, width, height, true);
        case dai::RawImgFrame::Type::GRAY8:
        case dai::RawImgFrame::Type::YUV400p: {
            if (data.size() < static_cast<size_t>(width) * height) {
                return {};
            }
            cv::Mat gray(height, width, CV_8UC1, const_cast<uint8_t*>(data.data()));
            cv::Mat bgr;
            cv::cvtColor(gray, bgr, cv::COLOR_GRAY2BGR);
            return bgr;
        }
        case dai::RawImgFrame::Type::NV12: {
            if (data.size() < static_cast<size_t>(width) * height * 3 / 2) {
                return {};
            }
            cv::Mat yuv(height * 3 / 2, width, CV_8UC1, const_cast<uint8_t*>(data.data()));
            cv::Mat bgr;
            cv::cvtColor(yuv, bgr, cv::COLOR_YUV2BGR_NV12);
            return bgr;
        }
        case dai::RawImgFrame::Type::NV21: {
            if (data.size() < static_cast<size_t>(width) * height * 3 / 2) {
                return {};
            }
            cv::Mat yuv(height * 3 / 2, width, CV_8UC1, const_cast<uint8_t*>(data.data()));
            cv::Mat bgr;
            cv::cvtColor(yuv, bgr, cv::COLOR_YUV2BGR_NV21);
            return bgr;
        }
        case dai::RawImgFrame::Type::YUV420p: {
            if (data.size() < static_cast<size_t>(width) * height * 3 / 2) {
                return {};
            }
            cv::Mat yuv(height * 3 / 2, width, CV_8UC1, const_cast<uint8_t*>(data.data()));
            cv::Mat bgr;
            cv::cvtColor(yuv, bgr, cv::COLOR_YUV2BGR_I420);
            return bgr;
        }
        default:
            return cv::imdecode(data, cv::IMREAD_COLOR);
    }
}

std::string normalize_extension(std::string extension) {
    if (extension.empty()) {
        return ".avi";
    }
    if (extension.front() != '.') {
        extension.insert(extension.begin(), '.');
    }
    return extension;
}

std::string yaml_scalar_to_string(const YAML::Node& node) {
    if (!node || !node.IsScalar()) {
        return {};
    }
    return node.as<std::string>();
}

std::array<double, 16> yaml_matrix4x4_to_array(const YAML::Node& node) {
    std::array<double, 16> values{};
    size_t idx = 0;
    for (const auto& row : node) {
        for (const auto& col : row) {
            if (idx < values.size()) {
                values[idx++] = col.as<double>();
            }
        }
    }
    if (idx != values.size()) {
        throw std::runtime_error("expected 4x4 matrix");
    }
    return values;
}

std::array<double, 4> yaml_vector4_to_array(const YAML::Node& node) {
    std::array<double, 4> values{};
    if (!node || !node.IsSequence() || node.size() != 4) {
        throw std::runtime_error("expected vector with 4 entries");
    }
    for (size_t i = 0; i < 4; ++i) {
        values[i] = node[i].as<double>();
    }
    return values;
}

std::vector<double> yaml_sequence_to_vector(const YAML::Node& node, const std::string& field_name) {
    if (!node || !node.IsSequence()) {
        throw std::runtime_error("expected sequence for " + field_name);
    }
    std::vector<double> values;
    values.reserve(node.size());
    for (const auto& value : node) {
        values.push_back(value.as<double>());
    }
    return values;
}

void write_double_array(std::ostream& out, const std::vector<double>& values) {
    out << "[";
    for (size_t i = 0; i < values.size(); ++i) {
        if (i) {
            out << ",";
        }
        out << values[i];
    }
    out << "]";
}

void write_double_array(std::ostream& out, const std::array<double, 4>& values) {
    out << "[" << values[0] << "," << values[1] << "," << values[2] << "," << values[3] << "]";
}
}  // namespace

OakDatasetRecorder::BlockingQueue::BlockingQueue(size_t capacity) : capacity_(capacity) {}

void OakDatasetRecorder::BlockingQueue::push(FrameGroup item) {
    std::unique_lock<std::mutex> lock(mutex_);
    not_full_.wait(lock, [&] { return closed_ || queue_.size() < capacity_; });
    if (closed_) {
        return;
    }
    queue_.push(std::move(item));
    not_empty_.notify_one();
}

bool OakDatasetRecorder::BlockingQueue::pop(FrameGroup& item) {
    std::unique_lock<std::mutex> lock(mutex_);
    not_empty_.wait(lock, [&] { return closed_ || !queue_.empty(); });
    if (queue_.empty()) {
        return false;
    }
    item = std::move(queue_.front());
    queue_.pop();
    not_full_.notify_one();
    return true;
}

void OakDatasetRecorder::BlockingQueue::close() {
    {
        std::lock_guard<std::mutex> lock(mutex_);
        closed_ = true;
    }
    not_empty_.notify_all();
    not_full_.notify_all();
}

OakDatasetRecorder::CameraVideoWriter::CameraVideoWriter(fs::path path, double fps, int fourcc, std::string camera_name)
    : path_(std::move(path)), fps_(fps), fourcc_(fourcc), camera_name_(std::move(camera_name)) {}

OakDatasetRecorder::CameraVideoWriter::~CameraVideoWriter() {
    stop();
}

void OakDatasetRecorder::CameraVideoWriter::start() {
    if (!started_) {
        started_ = true;
        worker_ = std::thread(&CameraVideoWriter::run, this);
    }
}

void OakDatasetRecorder::CameraVideoWriter::stop() {
    {
        std::lock_guard<std::mutex> lock(mutex_);
        if (stopped_) {
            return;
        }
        stopped_ = true;
    }
    cv_.notify_all();
    if (worker_.joinable()) {
        worker_.join();
    }
}

void OakDatasetRecorder::CameraVideoWriter::enqueue(CameraFrame frame) {
    std::unique_lock<std::mutex> lock(mutex_);
    cv_.wait(lock, [&] { return stopped_ || frames_.size() < capacity_; });
    if (stopped_) {
        return;
    }
    frames_.push(std::move(frame));
    lock.unlock();
    cv_.notify_one();
}

bool OakDatasetRecorder::CameraVideoWriter::pop(CameraFrame& frame) {
    std::unique_lock<std::mutex> lock(mutex_);
    cv_.wait(lock, [&] { return stopped_ || !frames_.empty(); });
    if (frames_.empty()) {
        return false;
    }
    frame = std::move(frames_.front());
    frames_.pop();
    lock.unlock();
    cv_.notify_one();
    return true;
}

void OakDatasetRecorder::CameraVideoWriter::run() {
    cv::VideoWriter writer;
    CameraFrame item;
    while (pop(item)) {
        cv::Mat image = frame_to_bgr(item.frame);
        if (image.empty()) {
            continue;
        }
        if (!writer.isOpened()) {
            if (!writer.open(path_.string(), fourcc_, fps_, image.size(), true)) {
                const auto path = path_.string();
                RCLCPP_ERROR(rclcpp::get_logger("oak_dataset_recorder"), "failed to open video writer for %s: %s", camera_name_.c_str(), path.c_str());
                break;
            }
        }
        writer.write(image);
    }
    if (writer.isOpened()) {
        writer.release();
    }
}

OakDatasetRecorder::OakDatasetRecorder(std::shared_ptr<rclcpp::Node> node,
                                       std::vector<CameraConfig> cameras,
                                       std::string output_root,
                                       std::string session_name,
                                       std::string imu_topic,
                                       std::string pose_topic,
                                       std::string config_path,
                                       double video_fps,
                                       std::string video_fourcc,
                                       std::string video_extension,
                                       bool wait_for_imu_and_pose)
    : node_(std::move(node)),
      cameras_(std::move(cameras)),
      output_root_(expand_user_path(output_root)),
      session_name_(std::move(session_name)),
      imu_topic_(std::move(imu_topic)),
      pose_topic_(std::move(pose_topic)),
      config_path_(std::move(config_path)),
      video_fps_(video_fps),
      video_fourcc_(std::move(video_fourcc)),
      video_extension_(normalize_extension(std::move(video_extension))),
      wait_for_imu_and_pose_(wait_for_imu_and_pose),
      ros_base_time_(node_->get_clock()->now()),
      steady_base_time_(std::chrono::steady_clock::now()) {
    if (cameras_.size() != kNumCameras) {
        throw std::runtime_error("OakDatasetRecorder requires exactly four cameras");
    }
    if (video_fps_ <= 0.0) {
        throw std::runtime_error("video_fps must be positive");
    }
    if (session_name_.empty()) {
        session_name_ = make_session_name();
    }
    setup_output_layout();
    setup_metadata_files();
    setup_camera_writers();
    write_calibration_json();
    setup_subscriptions();
    if (!wait_for_imu_and_pose_) {
        recording_started_.store(true);
    }
    shutdown_callback_handle_ = node_->get_node_base_interface()->get_context()->add_on_shutdown_callback([this]() { stop(); });
    metadata_thread_ = std::thread(&OakDatasetRecorder::metadata_loop, this);
    fps_timer_ = node_->create_wall_timer(std::chrono::seconds(1), std::bind(&OakDatasetRecorder::print_fps, this));
    RCLCPP_INFO(node_->get_logger(), "oak dataset recording to %s", session_dir_.c_str());
}

OakDatasetRecorder::~OakDatasetRecorder() {
    stop();
    node_->get_node_base_interface()->get_context()->remove_on_shutdown_callback(shutdown_callback_handle_);
}

void OakDatasetRecorder::stop() {
    bool expected = false;
    if (!shutting_down_.compare_exchange_strong(expected, true)) {
        return;
    }
    metadata_queue_.close();
    if (metadata_thread_.joinable()) {
        metadata_thread_.join();
    }
    for (auto& writer : camera_writers_) {
        if (writer) {
            writer->stop();
        }
    }
    close_json_files();
}

void OakDatasetRecorder::setup_output_layout() {
    session_dir_ = output_root_ / session_name_;
    videos_dir_ = session_dir_ / "videos";
    fs::create_directories(videos_dir_);
}

void OakDatasetRecorder::setup_metadata_files() {
    imu_file_.open(session_dir_ / "imu.json", std::ios::out | std::ios::trunc);
    frames_file_.open(session_dir_ / "video_frames.json", std::ios::out | std::ios::trunc);
    calibration_file_.open(session_dir_ / "calibration.json", std::ios::out | std::ios::trunc);
    if (!imu_file_ || !frames_file_ || !calibration_file_) {
        throw std::runtime_error("failed to open dataset metadata files");
    }

    imu_file_ << "{\n  \"topic\":\"" << json_escape(imu_topic_) << "\",\n  \"messages\":[\n";
    frames_file_ << "{\n  \"pose_topic\":\"" << json_escape(pose_topic_)
                 << "\",\n  \"pose_format\":\"xyz_qxyzw\",\n  \"camera_order\":[";
    for (size_t i = 0; i < kNumCameras; ++i) {
        if (i) {
            frames_file_ << ",";
        }
        frames_file_ << "\"" << json_escape(cameras_.at(i).name) << "\"";
    }
    frames_file_ << "],\n  \"frames\":[\n";
    files_open_.store(true);
}

void OakDatasetRecorder::setup_camera_writers() {
    const int fourcc = fourcc_from_string(video_fourcc_);
    for (size_t i = 0; i < kNumCameras; ++i) {
        const auto& cam = cameras_.at(i);
        camera_names_[i] = cam.name;
        camera_writers_[i] = std::make_unique<CameraVideoWriter>(videos_dir_ / (cam.name + video_extension_), video_fps_, fourcc, cam.name);
        camera_writers_[i]->start();
    }
}

void OakDatasetRecorder::setup_subscriptions() {
    auto imu_qos = rclcpp::SensorDataQoS().keep_last(2048);
    auto pose_qos = rclcpp::QoS(rclcpp::KeepLast(256)).best_effort();
    imu_subscriptions_.push_back(
        node_->create_subscription<sensor_msgs::msg::Imu>(imu_topic_, imu_qos, [this](sensor_msgs::msg::Imu::ConstSharedPtr msg) { imu_callback(msg); }));
    pose_subscription_ = node_->create_subscription<nav_msgs::msg::Odometry>(
        pose_topic_, pose_qos, [this](nav_msgs::msg::Odometry::ConstSharedPtr msg) { odom_callback(msg); });
}

void OakDatasetRecorder::onCameraGroup(const std::shared_ptr<dai::MessageGroup>& group) {
    if (!group || shutting_down_.load()) {
        return;
    }
    if (wait_for_imu_and_pose_ && !recording_started_.load()) {
        return;
    }

    FrameGroup frame_group;
    frame_group.group_index = next_group_index_++;
    double stamp_sum = 0.0;

    for (size_t i = 0; i < kNumCameras; ++i) {
        auto frame = group->get<dai::ImgFrame>(camera_names_[i]);
        if (!frame) {
            return;
        }
        const auto stamp = dai::ros::getFrameTime(ros_base_time_, steady_base_time_, frame->getTimestamp());
        frame_group.frames[i] = frame;
        frame_group.camera_stamps[i] = stamp_to_seconds(stamp);
        frame_group.camera_frame_indices[i] = camera_frame_counts_[i].fetch_add(1, std::memory_order_relaxed);
        camera_fps_counts_[i].fetch_add(1, std::memory_order_relaxed);
        stamp_sum += frame_group.camera_stamps[i];
    }
    frame_group.group_stamp_sec = stamp_sum / static_cast<double>(kNumCameras);

    for (size_t i = 0; i < kNumCameras; ++i) {
        camera_writers_[i]->enqueue(CameraFrame{frame_group.frames[i], frame_group.camera_frame_indices[i], frame_group.camera_stamps[i]});
    }

    metadata_queue_.push(std::move(frame_group));
    grouped_fps_count_.fetch_add(1, std::memory_order_relaxed);
}

void OakDatasetRecorder::imu_callback(const sensor_msgs::msg::Imu::ConstSharedPtr& msg) {
    have_imu_.store(true);
    imu_count_.fetch_add(1, std::memory_order_relaxed);
    start_if_ready();
    if (!recording_started_.load()) {
        return;
    }

    std::lock_guard<std::mutex> lock(imu_json_.mutex);
    if (!imu_json_.first) {
        imu_file_ << ",\n";
    }
    imu_json_.first = false;
    imu_file_ << std::fixed << std::setprecision(9);
    imu_file_ << "    {\"t\":" << stamp_to_seconds(msg->header.stamp)
              << ",\"angular_velocity\":[" << msg->angular_velocity.x << "," << msg->angular_velocity.y << "," << msg->angular_velocity.z
              << "],\"linear_acceleration\":[" << msg->linear_acceleration.x << "," << msg->linear_acceleration.y << ","
              << msg->linear_acceleration.z << "],\"orientation\":[" << msg->orientation.x << "," << msg->orientation.y << ","
              << msg->orientation.z << "," << msg->orientation.w << "]}";
}

void OakDatasetRecorder::odom_callback(const nav_msgs::msg::Odometry::ConstSharedPtr& msg) {
    PoseSample pose;
    pose.stamp_sec = stamp_to_seconds(msg->header.stamp);
    pose.x = msg->pose.pose.position.x;
    pose.y = msg->pose.pose.position.y;
    pose.z = msg->pose.pose.position.z;
    pose.qx = msg->pose.pose.orientation.x;
    pose.qy = msg->pose.pose.orientation.y;
    pose.qz = msg->pose.pose.orientation.z;
    pose.qw = msg->pose.pose.orientation.w;

    {
        std::lock_guard<std::mutex> lock(pose_mutex_);
        pose_buffer_.push_back(pose);
        latest_pose_stamp_sec_ = pose.stamp_sec;
        const double min_stamp = latest_pose_stamp_sec_ - 10.0;
        while (!pose_buffer_.empty() && pose_buffer_.front().stamp_sec < min_stamp) {
            pose_buffer_.pop_front();
        }
    }
    have_pose_.store(true);
    start_if_ready();
}

void OakDatasetRecorder::start_if_ready() {
    if (recording_started_.load()) {
        return;
    }
    if (!wait_for_imu_and_pose_ || (have_imu_.load() && have_pose_.load())) {
        recording_started_.store(true);
        RCLCPP_INFO(node_->get_logger(), "oak dataset recorder started");
    }
}

std::optional<OakDatasetRecorder::PoseSample> OakDatasetRecorder::find_nearest_pose(double stamp_sec) const {
    std::lock_guard<std::mutex> lock(pose_mutex_);
    if (pose_buffer_.empty()) {
        return std::nullopt;
    }

    auto best = pose_buffer_.begin();
    double best_dt = std::numeric_limits<double>::max();
    for (auto it = pose_buffer_.begin(); it != pose_buffer_.end(); ++it) {
        const double dt = std::abs(it->stamp_sec - stamp_sec);
        if (dt < best_dt) {
            best_dt = dt;
            best = it;
        }
    }
    return *best;
}

void OakDatasetRecorder::metadata_loop() {
    FrameGroup group;
    while (metadata_queue_.pop(group)) {
        write_frame_group(group, find_nearest_pose(group.group_stamp_sec));
    }
}

void OakDatasetRecorder::write_frame_group(const FrameGroup& group, const std::optional<PoseSample>& pose) {
    std::lock_guard<std::mutex> lock(frames_json_.mutex);
    if (!frames_json_.first) {
        frames_file_ << ",\n";
    }
    frames_json_.first = false;
    frames_file_ << std::fixed << std::setprecision(9);
    frames_file_ << "    {\"i\":" << group.group_index << ",\"t\":" << group.group_stamp_sec << ",\"t_cam\":[";
    for (size_t i = 0; i < kNumCameras; ++i) {
        if (i) {
            frames_file_ << ",";
        }
        frames_file_ << group.camera_stamps[i];
    }
    frames_file_ << "],\"video_i\":[";
    for (size_t i = 0; i < kNumCameras; ++i) {
        if (i) {
            frames_file_ << ",";
        }
        frames_file_ << group.camera_frame_indices[i];
    }
    frames_file_ << "]";
    if (pose) {
        frames_file_ << ",\"pose\":{\"t\":" << pose->stamp_sec << ",\"dt\":" << std::abs(pose->stamp_sec - group.group_stamp_sec)
                     << ",\"xyz_qxyzw\":[" << pose->x << "," << pose->y << "," << pose->z << ","
                     << pose->qx << "," << pose->qy << "," << pose->qz << "," << pose->qw << "]}";
    }
    frames_file_ << "}";
}

void OakDatasetRecorder::write_calibration_json() {
    const fs::path estimator_path = resolve_config_path(config_path_);
    const YAML::Node estimator = YAML::LoadFile(estimator_path.string());
    const fs::path config_dir = estimator_path.parent_path();
    const std::string imucam_name = yaml_scalar_to_string(estimator["relative_config_imucam"]);
    const std::string imu_name = yaml_scalar_to_string(estimator["relative_config_imu"]);
    if (imucam_name.empty() || imu_name.empty()) {
        throw std::runtime_error("estimator config missing relative_config_imucam or relative_config_imu");
    }

    const fs::path imucam_path = config_dir / imucam_name;
    const fs::path imu_path = config_dir / imu_name;
    const YAML::Node imucam = YAML::LoadFile(imucam_path.string());
    const YAML::Node imu = YAML::LoadFile(imu_path.string());

    calibration_file_ << "{\n";
    calibration_file_ << "  \"source\":\"src/configs\",\n";
    calibration_file_ << "  \"files\":{\n";
    calibration_file_ << "    \"estimator_config\":\"" << json_escape(estimator_path.string()) << "\",\n";
    calibration_file_ << "    \"imucam_config\":\"" << json_escape(imucam_path.string()) << "\",\n";
    calibration_file_ << "    \"imu_config\":\"" << json_escape(imu_path.string()) << "\"\n";
    calibration_file_ << "  },\n";
    calibration_file_ << "  \"cameras\":{\n";

    for (size_t i = 0; i < kNumCameras; ++i) {
        if (i) {
            calibration_file_ << ",\n";
        }
        const auto cam_key = std::string("cam") + std::to_string(i);
        calibration_file_ << "    \"" << cam_key << "\":";
        if (!imucam[cam_key]) {
            calibration_file_ << "null";
            RCLCPP_WARN(node_->get_logger(), "missing %s in %s", cam_key.c_str(), imucam_path.c_str());
            continue;
        }

        const auto cam = imucam[cam_key];
        const auto topic = yaml_scalar_to_string(cam["rostopic"]);
        const auto timeshift = cam["timeshift_cam_imu"] ? cam["timeshift_cam_imu"].as<double>() : 0.0;
        const auto intrinsics = yaml_vector4_to_array(cam["intrinsics"]);
        const auto distortion = cam["distortion_coeffs"] ? yaml_sequence_to_vector(cam["distortion_coeffs"], "distortion_coeffs") : std::vector<double>{};
        const auto resolution = yaml_sequence_to_vector(cam["resolution"], "resolution");
        if (resolution.size() != 2) {
            throw std::runtime_error(cam_key + ".resolution must contain width and height");
        }
        const auto T = yaml_matrix4x4_to_array(cam["T_imu_cam"]);

        calibration_file_ << "{\n";
        calibration_file_ << "      \"topic\":\"" << json_escape(topic) << "\",\n";
        calibration_file_ << "      \"resolution\":[" << static_cast<int>(resolution[0]) << "," << static_cast<int>(resolution[1]) << "],\n";
        calibration_file_ << "      \"model\":\"" << json_escape(yaml_scalar_to_string(cam["camera_model"])) << "\",\n";
        calibration_file_ << "      \"distortion_model\":\"" << json_escape(yaml_scalar_to_string(cam["distortion_model"])) << "\",\n";
        calibration_file_ << "      \"intrinsics\":";
        write_double_array(calibration_file_, intrinsics);
        calibration_file_ << ",\n";
        calibration_file_ << "      \"distortion_coeffs\":";
        write_double_array(calibration_file_, distortion);
        calibration_file_ << ",\n";
        calibration_file_ << "      \"timeshift_cam_imu\":" << timeshift << ",\n";
        calibration_file_ << "      \"T_imu_cam\":[\n";
        for (int row = 0; row < 4; ++row) {
            calibration_file_ << "        ["
                              << T[row * 4 + 0] << ","
                              << T[row * 4 + 1] << ","
                              << T[row * 4 + 2] << ","
                              << T[row * 4 + 3] << "]";
            calibration_file_ << (row < 3 ? ",\n" : "\n");
        }
        calibration_file_ << "      ]\n";
        calibration_file_ << "    }";
    }

    calibration_file_ << "\n  },\n";
    calibration_file_ << "  \"imu\":{\n";
    calibration_file_ << "    \"imu0\":{\n";
    calibration_file_ << "      \"topic\":\"" << json_escape(yaml_scalar_to_string(imu["imu0"]["rostopic"])) << "\",\n";
    calibration_file_ << "      \"update_rate\":" << imu["imu0"]["update_rate"].as<double>() << ",\n";
    calibration_file_ << "      \"time_offset\":" << imu["imu0"]["time_offset"].as<double>() << ",\n";
    calibration_file_ << "      \"model\":\"" << json_escape(yaml_scalar_to_string(imu["imu0"]["model"])) << "\",\n";
    calibration_file_ << "      \"accelerometer_noise_density\":" << imu["imu0"]["accelerometer_noise_density"].as<double>() << ",\n";
    calibration_file_ << "      \"accelerometer_random_walk\":" << imu["imu0"]["accelerometer_random_walk"].as<double>() << ",\n";
    calibration_file_ << "      \"gyroscope_noise_density\":" << imu["imu0"]["gyroscope_noise_density"].as<double>() << ",\n";
    calibration_file_ << "      \"gyroscope_random_walk\":" << imu["imu0"]["gyroscope_random_walk"].as<double>() << ",\n";
    calibration_file_ << "      \"T_i_b\":[\n";
    const auto T_i_b = yaml_matrix4x4_to_array(imu["imu0"]["T_i_b"]);
    for (int row = 0; row < 4; ++row) {
        calibration_file_ << "        ["
                          << T_i_b[row * 4 + 0] << ","
                          << T_i_b[row * 4 + 1] << ","
                          << T_i_b[row * 4 + 2] << ","
                          << T_i_b[row * 4 + 3] << "]";
        calibration_file_ << (row < 3 ? ",\n" : "\n");
    }
    calibration_file_ << "      ]\n";
    calibration_file_ << "    }\n";
    calibration_file_ << "  }\n";
    calibration_file_ << "}\n";
    calibration_file_.flush();
}

void OakDatasetRecorder::print_fps() {
    std::ostringstream oss;
    oss << "[dataset fps]";
    for (size_t i = 0; i < kNumCameras; ++i) {
        oss << " " << camera_names_[i] << "=" << camera_fps_counts_[i].exchange(0, std::memory_order_relaxed);
    }
    oss << " imu=" << imu_count_.exchange(0, std::memory_order_relaxed);
    oss << " grouped=" << grouped_fps_count_.exchange(0, std::memory_order_relaxed);
    if (wait_for_imu_and_pose_ && !recording_started_.load()) {
        oss << " waiting_for=imu_pose";
    }
    RCLCPP_INFO(node_->get_logger(), "%s", oss.str().c_str());
}

void OakDatasetRecorder::close_json_files() {
    if (!files_open_.exchange(false)) {
        return;
    }
    if (imu_file_.is_open()) {
        imu_file_ << "\n  ]\n}\n";
        imu_file_.close();
    }
    if (frames_file_.is_open()) {
        frames_file_ << "\n  ]\n}\n";
        frames_file_.close();
    }
    if (calibration_file_.is_open()) {
        calibration_file_.close();
    }
}

std::string OakDatasetRecorder::json_escape(const std::string& value) {
    std::ostringstream oss;
    for (char c : value) {
        switch (c) {
            case '\\': oss << "\\\\"; break;
            case '"': oss << "\\\""; break;
            case '\n': oss << "\\n"; break;
            case '\r': oss << "\\r"; break;
            case '\t': oss << "\\t"; break;
            default: oss << c; break;
        }
    }
    return oss.str();
}

int OakDatasetRecorder::fourcc_from_string(const std::string& fourcc) {
    if (fourcc.size() != 4) {
        throw std::runtime_error("record_video_fourcc must contain exactly four characters");
    }
    return cv::VideoWriter::fourcc(fourcc[0], fourcc[1], fourcc[2], fourcc[3]);
}

double OakDatasetRecorder::stamp_to_seconds(const builtin_interfaces::msg::Time& stamp) {
    return static_cast<double>(stamp.sec) + static_cast<double>(stamp.nanosec) * 1e-9;
}

double OakDatasetRecorder::stamp_to_seconds(const rclcpp::Time& stamp) {
    return static_cast<double>(stamp.nanoseconds()) * 1e-9;
}

fs::path OakDatasetRecorder::expand_user_path(const std::string& path) {
    if (!path.empty() && path.front() == '~') {
        const char* home = std::getenv("HOME");
        if (home) {
            if (path.size() == 1) {
                return fs::path(home);
            }
            if (path[1] == '/') {
                return fs::path(home) / path.substr(2);
            }
        }
    }
    return fs::path(path);
}

fs::path OakDatasetRecorder::resolve_config_path(const std::string& path) {
    if (!path.empty()) {
        return expand_user_path(path);
    }
    fs::path cwd = fs::current_path();
    for (size_t i = 0; i < 6; ++i) {
        fs::path candidate = cwd / "src/configs/estimator_config.yaml";
        if (fs::exists(candidate)) {
            return candidate;
        }
        if (!cwd.has_parent_path() || cwd == cwd.parent_path()) {
            break;
        }
        cwd = cwd.parent_path();
    }
    return fs::path("src/configs/estimator_config.yaml");
}

std::string OakDatasetRecorder::make_session_name() {
    const auto now = std::chrono::system_clock::now();
    const auto t = std::chrono::system_clock::to_time_t(now);
    std::tm tm{};
    localtime_r(&t, &tm);
    std::ostringstream oss;
    oss << std::put_time(&tm, "%Y%m%d_%H%M%S");
    return oss.str();
}

std::string OakDatasetRecorder::socket_to_string(dai::CameraBoardSocket socket) {
    switch (socket) {
        case dai::CameraBoardSocket::CAM_A: return "CAM_A";
        case dai::CameraBoardSocket::CAM_B: return "CAM_B";
        case dai::CameraBoardSocket::CAM_C: return "CAM_C";
        case dai::CameraBoardSocket::CAM_D: return "CAM_D";
        default: return "UNKNOWN";
    }
}

}  // namespace ros2_oak_ffc_sync
