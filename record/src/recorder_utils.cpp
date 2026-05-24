#include <record/recorder_utils.hpp>

#include <cstdlib>
#include <ctime>
#include <fstream>
#include <iomanip>
#include <opencv2/imgproc.hpp>
#include <opencv2/videoio.hpp>
#include <sensor_msgs/image_encodings.hpp>
#include <sstream>
#include <stdexcept>

#include <cv_bridge/cv_bridge.h>

namespace fs = std::filesystem;

namespace record {

double stamp_to_seconds(const builtin_interfaces::msg::Time &stamp) {
  return static_cast<double>(stamp.sec) + static_cast<double>(stamp.nanosec) * 1e-9;
}

std::string json_escape(const std::string &input) {
  std::ostringstream out;
  for (const char c : input) {
    switch (c) {
    case '"':
      out << "\\\"";
      break;
    case '\\':
      out << "\\\\";
      break;
    case '\b':
      out << "\\b";
      break;
    case '\f':
      out << "\\f";
      break;
    case '\n':
      out << "\\n";
      break;
    case '\r':
      out << "\\r";
      break;
    case '\t':
      out << "\\t";
      break;
    default:
      if (static_cast<unsigned char>(c) < 0x20) {
        out << "\\u" << std::hex << std::setw(4) << std::setfill('0') << static_cast<int>(c);
      } else {
        out << c;
      }
    }
  }
  return out.str();
}

std::string read_text_file(const std::string &path) {
  if (path.empty() || !fs::exists(path)) {
    return "";
  }
  std::ifstream in(path, std::ios::binary);
  std::ostringstream buffer;
  buffer << in.rdbuf();
  return buffer.str();
}

std::string expand_user_path(std::string path) {
  if (path.empty() || path[0] != '~') {
    return path;
  }
  const char *home = std::getenv("HOME");
  if (home == nullptr) {
    return path;
  }
  if (path.size() == 1) {
    return std::string(home);
  }
  if (path[1] == '/') {
    return std::string(home) + path.substr(1);
  }
  return path;
}

std::string make_default_session_name() {
  const auto now = std::chrono::system_clock::now();
  const auto time = std::chrono::system_clock::to_time_t(now);
  std::tm tm{};
  localtime_r(&time, &tm);
  std::ostringstream out;
  out << std::put_time(&tm, "%Y%m%d_%H%M%S");
  return out.str();
}

std::string normalize_extension(std::string ext) {
  if (ext.empty()) {
    return ".avi";
  }
  if (ext.front() != '.') {
    ext.insert(ext.begin(), '.');
  }
  return ext;
}

int fourcc_from_string(const std::string &value) {
  if (value.size() != 4) {
    throw std::runtime_error("video_fourcc must have exactly 4 characters");
  }
  return cv::VideoWriter::fourcc(value[0], value[1], value[2], value[3]);
}

cv::Mat image_to_bgr(const sensor_msgs::msg::Image::ConstSharedPtr &msg) {
  namespace enc = sensor_msgs::image_encodings;
  const cv_bridge::CvImageConstPtr cv_ptr = cv_bridge::toCvShare(msg);
  const cv::Mat &image = cv_ptr->image;
  cv::Mat bgr;

  if (msg->encoding == enc::BGR8 || msg->encoding == "8UC3") {
    bgr = image;
  } else if (msg->encoding == enc::RGB8) {
    cv::cvtColor(image, bgr, cv::COLOR_RGB2BGR);
  } else if (msg->encoding == enc::MONO8 || msg->encoding == "8UC1") {
    cv::cvtColor(image, bgr, cv::COLOR_GRAY2BGR);
  } else if (msg->encoding == enc::BGRA8) {
    cv::cvtColor(image, bgr, cv::COLOR_BGRA2BGR);
  } else if (msg->encoding == enc::RGBA8) {
    cv::cvtColor(image, bgr, cv::COLOR_RGBA2BGR);
  } else {
    throw std::runtime_error("unsupported image encoding: " + msg->encoding);
  }

  if (!bgr.isContinuous()) {
    return bgr.clone();
  }
  return bgr;
}

} // namespace record
