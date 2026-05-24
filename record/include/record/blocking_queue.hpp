#pragma once

#include <condition_variable>
#include <deque>
#include <mutex>
#include <utility>

namespace record {

template <typename T> class BlockingQueue {
public:
  void push(T value) {
    {
      std::lock_guard<std::mutex> lock(mutex_);
      queue_.push_back(std::move(value));
    }
    cv_.notify_one();
  }

  bool pop(T &value) {
    std::unique_lock<std::mutex> lock(mutex_);
    cv_.wait(lock, [this] { return closed_ || !queue_.empty(); });
    if (queue_.empty()) {
      return false;
    }
    value = std::move(queue_.front());
    queue_.pop_front();
    return true;
  }

  size_t size() const {
    std::lock_guard<std::mutex> lock(mutex_);
    return queue_.size();
  }

  void close() {
    {
      std::lock_guard<std::mutex> lock(mutex_);
      closed_ = true;
    }
    cv_.notify_all();
  }

private:
  mutable std::mutex mutex_;
  std::condition_variable cv_;
  std::deque<T> queue_;
  bool closed_{false};
};

} // namespace record
