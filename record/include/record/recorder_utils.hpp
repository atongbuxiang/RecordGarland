#pragma once

#include <filesystem>
#include <string>

#include <builtin_interfaces/msg/time.hpp>
#include <opencv2/core.hpp>
#include <sensor_msgs/msg/image.hpp>

namespace record {

double stamp_to_seconds(const builtin_interfaces::msg::Time &stamp);
std::string json_escape(const std::string &input);
std::string read_text_file(const std::string &path);
std::string expand_user_path(std::string path);
std::string make_default_session_name();
std::string normalize_extension(std::string ext);
int fourcc_from_string(const std::string &value);
cv::Mat image_to_bgr(const sensor_msgs::msg::Image::ConstSharedPtr &msg);

} // namespace record
