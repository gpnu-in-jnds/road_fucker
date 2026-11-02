#include "worker_pool.hpp"

namespace road_finder {

WorkerPool::WorkerPool(int count) {
  if (count <= 1) return;
  try {
    for (int i = 0; i < count; ++i) {
      threads_.emplace_back([this] {
        for (;;) {
          std::function<void()> task;
          {
            std::unique_lock<std::mutex> lock(mutex_);
            ready_.wait(lock, [this] { return stopping_ || !tasks_.empty(); });
            if (stopping_ && tasks_.empty()) return;
            task = std::move(tasks_.front());
            tasks_.pop();
          }
          task();
        }
      });
    }
  } catch (...) {
    stop();
    throw;
  }
}
WorkerPool::~WorkerPool() { stop(); }

void WorkerPool::run(int count, const std::function<void(int)>& function) {
  if (threads_.empty()) {
    for (int i = 0; i < count; ++i) function(i);
    return;
  }
  std::atomic<int> next{0};
  std::vector<std::future<void>> completions;
  const int jobs = std::min(count, static_cast<int>(threads_.size()));
  completions.reserve(jobs);
  // Prepare all allocations before enqueueing tasks that reference locals.
  std::vector<std::shared_ptr<std::packaged_task<void()>>> jobs_to_submit;
  for (int i = 0; i < jobs; ++i) {
    auto task = std::make_shared<std::packaged_task<void()>>([&] {
      for (;;) {
        const int index = next.fetch_add(1);
        if (index >= count) return;
        function(index);
      }
    });
    completions.push_back(task->get_future());
    jobs_to_submit.push_back(std::move(task));
  }
  std::exception_ptr error;
  try {
    std::lock_guard<std::mutex> lock(mutex_);
    for (const auto& job : jobs_to_submit) tasks_.push([job] { (*job)(); });
  } catch (...) {
    // Allocation failure while enqueueing: discard unsubmitted futures;
    // shared jobs already queued still finish before frame locals disappear.
    error = std::current_exception();
  }
  ready_.notify_all();
  for (std::size_t i = 0; i < completions.size(); ++i) {
    // Dropping the final packaged_task reference breaks unsubmitted futures.
    jobs_to_submit[i].reset();
  }
  for (auto& completion : completions) {
    try { completion.get(); } catch (...) { if (!error) error = std::current_exception(); }
  }
  if (error) std::rethrow_exception(error);
}

void WorkerPool::stop() {
  {
    std::lock_guard<std::mutex> lock(mutex_);
    stopping_ = true;
  }
  ready_.notify_all();
  for (auto& thread : threads_) if (thread.joinable()) thread.join();
}

}  // namespace road_finder
