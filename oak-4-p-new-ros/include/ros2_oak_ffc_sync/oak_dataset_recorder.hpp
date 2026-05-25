#pragma once

#include <array>
#include <atomic>
#include <chrono>
#include <condition_variable>
#include <cstdint>
#include <deque>
#include <filesystem>
#include <fstream>
#include <memory>
#include <mutex>
#include <optional>
#include <queue>
#include <string>
#include <thread>
#include <vector>

#include <nav_msgs/msg/odometry.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/imu.hpp>

#include <depthai/depthai.hpp>

namespace ros2_oak_ffc_sync {

class OakDatasetRecorder {
public:
    struct CameraConfig {
        std::string name;
        dai::CameraBoardSocket socket;
        int width;
        int height;
    };

    OakDatasetRecorder(std::shared_ptr<rclcpp::Node> node,
                      std::vector<CameraConfig> cameras,
                      std::string output_root,
                      std::string session_name,
                      std::string imu_topic,
                      std::string pose_topic,
                      std::string config_path,
                      double video_fps,
                      std::string video_fourcc,
                      std::string video_extension,
                      bool wait_for_imu_and_pose = true);
    ~OakDatasetRecorder();

    void onCameraGroup(const std::shared_ptr<dai::MessageGroup>& group);
    void stop();

private:
    struct CameraFrame {
        std::shared_ptr<dai::ImgFrame> frame;
        uint64_t frame_index{0};
        double stamp_sec{0.0};
    };

    struct PoseSample {
        double stamp_sec{0.0};
        double x{0.0};
        double y{0.0};
        double z{0.0};
        double qx{0.0};
        double qy{0.0};
        double qz{0.0};
        double qw{1.0};
    };

    struct FrameGroup {
        uint64_t group_index{0};
        std::array<std::shared_ptr<dai::ImgFrame>, 4> frames{};
        std::array<double, 4> camera_stamps{};
        std::array<uint64_t, 4> camera_frame_indices{};
        double group_stamp_sec{0.0};
    };

    class BlockingQueue {
    public:
        explicit BlockingQueue(size_t capacity = 256);
        void push(FrameGroup item);
        bool pop(FrameGroup& item);
        void close();

    private:
        size_t capacity_;
        std::mutex mutex_;
        std::condition_variable not_empty_;
        std::condition_variable not_full_;
        std::queue<FrameGroup> queue_;
        bool closed_{false};
    };

    class CameraVideoWriter {
    public:
        CameraVideoWriter(std::filesystem::path path, double fps, int fourcc, std::string camera_name);
        ~CameraVideoWriter();

        void start();
        void stop();
        void enqueue(CameraFrame frame);

    private:
        void run();
        bool pop(CameraFrame& frame);

        std::filesystem::path path_;
        double fps_;
        int fourcc_;
        std::string camera_name_;
        std::thread worker_;
        std::mutex mutex_;
        std::condition_variable cv_;
        std::queue<CameraFrame> frames_;
        size_t capacity_{256};
        bool started_{false};
        bool stopped_{false};
    };

    struct JsonState {
        bool first{true};
        std::mutex mutex;
    };

    static constexpr size_t kNumCameras = 4;

    void setup_output_layout();
    void setup_metadata_files();
    void setup_camera_writers();
    void setup_subscriptions();
    void write_calibration_json();
    void write_frame_group(const FrameGroup& group, const std::optional<PoseSample>& pose);
    void imu_callback(const sensor_msgs::msg::Imu::ConstSharedPtr& msg);
    void odom_callback(const nav_msgs::msg::Odometry::ConstSharedPtr& msg);
    void start_if_ready();
    std::optional<PoseSample> find_nearest_pose(double stamp_sec) const;
    void metadata_loop();
    void print_fps();
    void close_json_files();
    static std::string json_escape(const std::string& value);
    static int fourcc_from_string(const std::string& fourcc);
    static double stamp_to_seconds(const builtin_interfaces::msg::Time& stamp);
    static double stamp_to_seconds(const rclcpp::Time& stamp);
    static std::filesystem::path expand_user_path(const std::string& path);
    static std::filesystem::path resolve_config_path(const std::string& path);
    static std::string make_session_name();
    static std::string socket_to_string(dai::CameraBoardSocket socket);

    std::shared_ptr<rclcpp::Node> node_;
    std::vector<CameraConfig> cameras_;
    std::array<std::string, kNumCameras> camera_names_{};

    std::filesystem::path output_root_;
    std::filesystem::path session_dir_;
    std::filesystem::path videos_dir_;
    std::string session_name_;
    std::string imu_topic_;
    std::string pose_topic_;
    std::string config_path_;
    double video_fps_{30.0};
    std::string video_fourcc_{"MJPG"};
    std::string video_extension_{".avi"};
    bool wait_for_imu_and_pose_{true};
    rclcpp::Time ros_base_time_;
    std::chrono::time_point<std::chrono::steady_clock> steady_base_time_;
    rclcpp::OnShutdownCallbackHandle shutdown_callback_handle_;

    std::unique_ptr<CameraVideoWriter> camera_writers_[kNumCameras];
    std::vector<rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr> imu_subscriptions_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr pose_subscription_;
    rclcpp::TimerBase::SharedPtr fps_timer_;
    std::thread metadata_thread_;

    BlockingQueue metadata_queue_{512};

    mutable std::mutex pose_mutex_;
    std::deque<PoseSample> pose_buffer_;
    double latest_pose_stamp_sec_{0.0};

    std::ofstream imu_file_;
    std::ofstream frames_file_;
    std::ofstream calibration_file_;
    JsonState imu_json_;
    JsonState frames_json_;
    std::atomic<bool> files_open_{false};
    std::atomic<bool> recording_started_{false};
    std::atomic<bool> have_imu_{false};
    std::atomic<bool> have_pose_{false};
    std::atomic<bool> shutting_down_{false};
    std::atomic<uint64_t> grouped_fps_count_{0};
    uint64_t next_group_index_{0};
    std::array<std::atomic<uint64_t>, kNumCameras> camera_frame_counts_{};
    std::array<std::atomic<uint64_t>, kNumCameras> camera_fps_counts_{};
    std::atomic<uint64_t> imu_count_{0};
};

}  // namespace ros2_oak_ffc_sync
