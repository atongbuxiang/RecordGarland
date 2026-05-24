#pragma once

#include <atomic>
#include <filesystem>
#include <string>
#include <thread>

#include <record/blocking_queue.hpp>
#include <record/recorder_types.hpp>

namespace record {

class VideoWriterEvents {
public:
  virtual ~VideoWriterEvents() = default;
  virtual void record_camera_frame(size_t camera_index, uint64_t frame_index, double stamp) = 0;
  virtual void report_writer_error(const std::string &camera_name, const std::string &message) = 0;
};

class CameraVideoWriter {
public:
  CameraVideoWriter(VideoWriterEvents *events, size_t camera_index, std::string camera_name, std::filesystem::path output_path,
                    double video_fps, int fourcc, size_t queue_warning_frames);
  ~CameraVideoWriter();

  void enqueue(CameraFrame frame);
  void stop();
  size_t queue_size() const;

private:
  void run();

  VideoWriterEvents *events_;
  size_t camera_index_;
  std::string camera_name_;
  std::filesystem::path output_path_;
  double video_fps_;
  int fourcc_;
  size_t queue_warning_frames_;
  BlockingQueue<CameraFrame> queue_;
  std::thread thread_;
  std::atomic<bool> stopped_{false};
};

} // namespace record
