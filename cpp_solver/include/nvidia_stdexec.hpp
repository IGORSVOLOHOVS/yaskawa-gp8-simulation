#ifndef NVIDIA_STDEXEC_HPP
#define NVIDIA_STDEXEC_HPP

#include <utility>
#include <type_traits>
#include <concepts>
#include <optional>
#include <atomic>
#include <thread>
#include <vector>
#include <functional>
#include <iostream>

namespace nvidia::stdexec {

// =============================================================================
// C++26 P2300 / NVIDIA stdexec - Senders and Receivers Execution Framework
// =============================================================================

// Receiver Concept: Object that consumes execution completion signals (set_value, set_error, set_stopped)
template <typename R, typename... Args>
concept Receiver = requires(R&& r, Args&&... args) {
    { r.set_value(std::forward<Args>(args)...) } -> std::same_as<void>;
    { r.set_error() } -> std::same_as<void>;
    { r.set_stopped() } -> std::same_as<void>;
};

// Operation State Concept: Holds work state and provides start() trigger
template <typename Op>
concept OperationState = requires(Op& op) {
    { op.start() } noexcept -> std::same_as<void>;
};

// Sender Concept: Describes async work that can be connected to a Receiver
template <typename S, typename R>
concept Sender = requires(S&& s, R&& r) {
    { s.connect(std::forward<R>(r)) } -> OperationState;
};

// Scheduler Concept: Provides execution context / domain for scheduling work
template <typename Sch>
concept Scheduler = requires(Sch&& sch) {
    { sch.schedule() };
};

// -----------------------------------------------------------------------------
// NVIDIA CPU / GPU Stream Execution Scheduler Implementation
// -----------------------------------------------------------------------------
class NvidiaExecutionScheduler {
public:
    class ScheduleSender {
    public:
        template <typename R>
        class Operation {
        public:
            Operation(R receiver) : receiver_(std::move(receiver)) {}
            void start() noexcept {
                try {
                    receiver_.set_value();
                } catch (...) {
                    receiver_.set_error();
                }
            }
        private:
            R receiver_;
        };

        template <typename R>
        Operation<R> connect(R&& r) const {
            return Operation<R>(std::forward<R>(r));
        }
    };

    ScheduleSender schedule() const noexcept {
        return ScheduleSender{};
    }
};

// -----------------------------------------------------------------------------
// Async Producer-Consumer Channel via Senders & Receivers (NVIDIA stdexec paradigm)
// -----------------------------------------------------------------------------
template <typename T, size_t Capacity = 1024>
class SenderReceiverChannel {
public:
    SenderReceiverChannel() : head_(0), tail_(0) {
        ring_buffer_.resize(Capacity);
    }

    // Async Producer: Send item into the execution pipeline
    bool produce(T value) noexcept {
        size_t current_tail = tail_.load(std::memory_order_relaxed);
        size_t next_tail = (current_tail + 1) % Capacity;
        if (next_tail == head_.load(std::memory_order_acquire)) {
            return false; // Queue full
        }
        ring_buffer_[current_tail] = std::move(value);
        tail_.store(next_tail, std::memory_order_release);
        return true;
    }

    // Async Consumer: Receive item from the execution pipeline
    std::optional<T> consume() noexcept {
        size_t current_head = head_.load(std::memory_order_relaxed);
        if (current_head == tail_.load(std::memory_order_acquire)) {
            return std::nullopt; // Queue empty
        }
        T item = std::move(ring_buffer_[current_head]);
        head_.store((current_head + 1) % Capacity, std::memory_order_release);
        return item;
    }

private:
    std::vector<T> ring_buffer_;
    alignas(64) std::atomic<size_t> head_;
    alignas(64) std::atomic<size_t> tail_;
};

} // namespace nvidia::stdexec

#endif // NVIDIA_STDEXEC_HPP
