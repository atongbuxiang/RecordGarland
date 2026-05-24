#include <record/video_writer.hpp>

#include <opencv2/videoio.hpp>

#include <record/recorder_utils.hpp>

namespace record {

CameraVideoWriter::CameraVideoWriter(VideoWriterEvents *events, size_t camera_index, std::string camera_name,
                                     std::filesystem::path output_path, double video_fps, int fourcc,
                                     size_t queue_warning_frames)
    : events_(events), camera_index_(camera_index), camera_name_(std::move(camera_name)), output_path_(std::move(output_path)),
      video_fps_(video_fps), fourcc_(fourcc), queue_warning_frames_(queue_warning_frames),
      thread_(&CameraVideoWriter::run, this) {}

CameraVideoWriter::~CameraVideoWriter() { stop(); }

void CameraVideoWriter::enqueue(CameraFrame frame) { queue_.push(std::move(frame)); }

void CameraVideoWriter::stop() {
  if (stopped_.exchange(true)) {
    return;
  }
  queue_.close();
  if (thread_.joinable()) {
    thread_.join();
  }
}

size_t CameraVideoWriter::queue_size() const { return queue_.size(); }

void CameraVideoWriter::run() {
  cv::VideoWriter writer;
  CameraFrame frame;
  while (queue_.pop(frame)) {
    try {
      cv::Mat bgr = image_to_bgr(frame.msg);
      if (!writer.isOpened()) {
        if (!writer.open(output_path_.string(), fourcc_, video_fps_, bgr.size(), true)) {
          events_->report_writer_error(camera_name_, "failed to open video file: " + output_path_.string());
          return;
        }
      }
      writer.write(bgr);
      events_->record_camera_frame(camera_index_, frame.frame_index, frame.stamp);
      if (queue_.size() > queue_warning_frames_) {
        events_->report_writer_error(camera_name_, "video writer is falling behind; queued frames=" + std::to_string(queue_.size()));
      }
    } catch (const std::exception &e) {
      events_->report_writer_error(camera_name_, e.what());
    }
  }
  if (writer.isOpened()) {
    writer.release();
  }
}

} // namespace record
