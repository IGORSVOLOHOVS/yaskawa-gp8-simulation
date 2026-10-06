#ifndef THREAD_POOL_HPP
#define THREAD_POOL_HPP

#include <vector>
#include <thread>
#include <queue>
#include <mutex>
#include <condition_variable>
#include <functional>
#include <future>
#include <memory>
#include <type_traits>

namespace yaskawa {

class ThreadPoolScheduler {
public:
    explicit ThreadPoolScheduler(size_t threads = std::thread::hardware_concurrency()) 
        : stop_(false) {
        if (threads == 0) threads = 4;
        workers_.reserve(threads);
        for (size_t i = 0; i < threads; ++i) {
            workers_.emplace_back([this](std::stop_token st) {
                while (!st.stop_requested()) {
                    std::function<void()> task;
                    {
                        std::unique_lock<std::mutex> lock(this->queue_mutex_);
                        this->cv_.wait(lock, [this, &st] {
                            return this->stop_ || !this->tasks_.empty() || st.stop_requested();
                        });
                        if ((this->stop_ || st.stop_requested()) && this->tasks_.empty()) {
                            return;
                        }
                        if (this->tasks_.empty()) continue;
                        task = std::move(this->tasks_.front());
                        this->tasks_.pop();
                    }
                    task();
                }
            });
        }
    }

    template <class F, class... Args>
    auto submit(F&& f, Args&&... args) 
        -> std::future<typename std::invoke_result_t<F, Args...>> {
        using return_type = typename std::invoke_result_t<F, Args...>;

        auto task = std::make_shared<std::packaged_task<return_type()>>(
            std::bind(std::forward<F>(f), std::forward<Args>(args)...)
        );

        std::future<return_type> res = task->get_future();
        {
            std::unique_lock<std::mutex> lock(queue_mutex_);
            if (stop_) {
                throw std::runtime_error("ThreadPoolScheduler is stopped");
            }
            tasks_.emplace([task]() { (*task)(); });
        }
        cv_.notify_one();
        return res;
    }

    size_t workerCount() const {
        return workers_.size();
    }

    ~ThreadPoolScheduler() {
        {
            std::unique_lock<std::mutex> lock(queue_mutex_);
            stop_ = true;
        }
        cv_.notify_all();
        for (auto& worker : workers_) {
            worker.request_stop();
        }
    }

private:
    std::vector<std::jthread> workers_;
    std::queue<std::function<void()>> tasks_;
    std::mutex queue_mutex_;
    std::condition_variable cv_;
    bool stop_;
};

} // namespace yaskawa

#endif // THREAD_POOL_HPP
