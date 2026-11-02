#pragma once

#include <algorithm>
#include <atomic>
#include <condition_variable>
#include <functional>
#include <future>
#include <mutex>
#include <queue>
#include <thread>
#include <vector>

namespace road_finder {

// One pool per detector. A run completes every task before returning, including
// when a task throws, so worker tasks never outlive their frame-local buffers.
class WorkerPool {
 public:
  explicit WorkerPool(int count);
  ~WorkerPool();
  WorkerPool(const WorkerPool&) = delete;
  WorkerPool& operator=(const WorkerPool&) = delete;

  void run(int count, const std::function<void(int)>& function);

 private:
  void stop();
  std::mutex mutex_;
  std::condition_variable ready_;
  bool stopping_ = false;
  std::queue<std::function<void()>> tasks_;
  std::vector<std::thread> threads_;
};

}  // namespace road_finder
