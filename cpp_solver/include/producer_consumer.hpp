#ifndef PRODUCER_CONSUMER_HPP
#define PRODUCER_CONSUMER_HPP

#include <atomic>
#include <vector>
#include <optional>
#include <cstddef>

namespace yaskawa {

template <typename T, size_t Capacity = 4096>
class LockFreeRingBuffer {
public:
    LockFreeRingBuffer() : head_(0), tail_(0) {
        buffer_.resize(Capacity);
    }

    bool push(const T& item) {
        size_t current_tail = tail_.load(std::memory_order_relaxed);
        size_t next_tail = (current_tail + 1) % Capacity;
        if (next_tail == head_.load(std::memory_order_acquire)) {
            return false; // Buffer full
        }
        buffer_[current_tail] = item;
        tail_.store(next_tail, std::memory_order_release);
        return true;
    }

    std::optional<T> pop() {
        size_t current_head = head_.load(std::memory_order_relaxed);
        if (current_head == tail_.load(std::memory_order_acquire)) {
            return std::nullopt; // Buffer empty
        }
        T item = buffer_[current_head];
        head_.store((current_head + 1) % Capacity, std::memory_order_release);
        return item;
    }

    bool empty() const {
        return head_.load(std::memory_order_relaxed) == tail_.load(std::memory_order_relaxed);
    }

    size_t size() const {
        size_t head = head_.load(std::memory_order_relaxed);
        size_t tail = tail_.load(std::memory_order_relaxed);
        if (tail >= head) return tail - head;
        return Capacity + tail - head;
    }

private:
    std::vector<T> buffer_;
    alignas(64) std::atomic<size_t> head_;
    alignas(64) std::atomic<size_t> tail_;
};

} // namespace yaskawa

#endif // PRODUCER_CONSUMER_HPP
