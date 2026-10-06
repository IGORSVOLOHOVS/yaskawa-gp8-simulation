#ifndef ATOMIC_STRUCTURES_HPP
#define ATOMIC_STRUCTURES_HPP

#include <atomic>
#include <memory>
#include <optional>
#include <cstdint>
#include <array>
#include <iostream>

namespace yaskawa::atomic {

// =============================================================================
// 1. Lock-Free SPSC (Single-Producer Single-Consumer) Queue
//    MEMORY ORDERS: relaxed (local counter), acquire (read head/tail), release (publish)
//    USE-CASE: High-frequency (1000Hz) telemetry log streaming between control loop
//              and background logger thread without lock contention.
// =============================================================================
template <typename T, size_t Capacity = 1024>
class LockFreeSPSCQueue {
public:
    LockFreeSPSCQueue() : head_(0), tail_(0) {}

    // Producer thread pushes items into the queue
    bool push(const T& val) noexcept {
        size_t current_tail = tail_.load(std::memory_order_relaxed);
        size_t next_tail = (current_tail + 1) % Capacity;
        // Synchronize with consumer's head update using ACQUIRE
        if (next_tail == head_.load(std::memory_order_acquire)) {
            return false; // Queue full
        }
        buffer_[current_tail] = val;
        // Publish written item to consumer thread using RELEASE
        tail_.store(next_tail, std::memory_order_release);
        return true;
    }

    // Consumer thread pops items from the queue
    std::optional<T> pop() noexcept {
        size_t current_head = head_.load(std::memory_order_relaxed);
        // Synchronize with producer's tail update using ACQUIRE
        if (current_head == tail_.load(std::memory_order_acquire)) {
            return std::nullopt; // Queue empty
        }
        T val = buffer_[current_head];
        // Publish read position update using RELEASE
        head_.store((current_head + 1) % Capacity, std::memory_order_release);
        return val;
    }

    bool empty() const noexcept {
        return head_.load(std::memory_order_relaxed) == tail_.load(std::memory_order_relaxed);
    }

private:
    std::array<T, Capacity> buffer_;
    alignas(64) std::atomic<size_t> head_{0};
    alignas(64) std::atomic<size_t> tail_{0};
};

// =============================================================================
// 2. Lock-Free Triple Buffer (Zero-Wait Reader/Writer Buffer)
//    MEMORY ORDERS: acq_rel (swap index exchange), acquire/release (buffer sync)
//    USE-CASE: Telemetry state exchange between 1000Hz physical control loop (Writer)
//              and 60Hz WebGL Visualizer / Dashboard (Reader). Writer NEVER blocks.
// =============================================================================
template <typename T>
class LockFreeTripleBuffer {
public:
    LockFreeTripleBuffer() {
        flags_.store(0x00010200, std::memory_order_relaxed); // write=0, dirty=1, read=2, new_flag=0
    }

    // Writer thread (1000Hz) writes latest data without waiting for Reader
    void write(const T& data) noexcept {
        uint32_t current_flags = flags_.load(std::memory_order_relaxed);
        uint8_t write_idx = (current_flags >> 24) & 0xFF;

        buffers_[write_idx] = data;

        // Atomically mark dirty buffer as newest written and swap with clean buffer
        uint32_t new_flags;
        do {
            uint8_t dirty_idx = (current_flags >> 16) & 0xFF;
            new_flags = (dirty_idx << 24) | (write_idx << 16) | (current_flags & 0xFFFF) | 0x1;
        } while (!flags_.compare_exchange_weak(
            current_flags, new_flags,
            std::memory_order_acq_rel,
            std::memory_order_relaxed
        ));
    }

    // Reader thread (60Hz GUI) reads latest state without locking Writer
    bool read(T& out_data) noexcept {
        uint32_t current_flags = flags_.load(std::memory_order_relaxed);
        if (!(current_flags & 0x1)) {
            return false; // No new write since last read
        }

        uint32_t new_flags;
        do {
            uint8_t dirty_idx = (current_flags >> 16) & 0xFF;
            uint8_t read_idx = (current_flags >> 8) & 0xFF;
            new_flags = (current_flags & 0xFFFF0000) | (dirty_idx << 8); // Swap read & dirty
        } while (!flags_.compare_exchange_weak(
            current_flags, new_flags,
            std::memory_order_acq_rel,
            std::memory_order_relaxed
        ));

        uint8_t new_read_idx = (new_flags >> 8) & 0xFF;
        out_data = buffers_[new_read_idx];
        return true;
    }

private:
    std::array<T, 3> buffers_;
    // Packed layout: [write_idx (8b) | dirty_idx (8b) | read_idx (8b) | new_write_flag (8b)]
    alignas(64) std::atomic<uint32_t> flags_{0};
};

// =============================================================================
// 3. Lock-Free Treiber Stack using CAS & Acquire-Release Memory Orderings
//    MEMORY ORDERS: relaxed (initial load), release (push CAS), acquire (pop CAS)
//    USE-CASE: Zero-allocation node recycling pool in kinematic matrix solvers.
// =============================================================================
template <typename T>
class LockFreeStack {
private:
    struct Node {
        T data;
        Node* next;
        explicit Node(const T& val) : data(val), next(nullptr) {}
    };

    alignas(64) std::atomic<Node*> head_{nullptr};

public:
    LockFreeStack() = default;
    ~LockFreeStack() {
        while (pop().has_value());
    }

    // Lock-free Push using compare_exchange_weak with std::memory_order_release
    void push(const T& val) {
        Node* new_node = new Node(val);
        new_node->next = head_.load(std::memory_order_relaxed);
        while (!head_.compare_exchange_weak(
            new_node->next,
            new_node,
            std::memory_order_release,
            std::memory_order_relaxed
        )) {}
    }

    // Lock-free Pop using compare_exchange_weak with std::memory_order_acquire
    std::optional<T> pop() {
        Node* old_head = head_.load(std::memory_order_relaxed);
        while (old_head && !head_.compare_exchange_weak(
            old_head,
            old_head->next,
            std::memory_order_acquire,
            std::memory_order_relaxed
        )) {}

        if (!old_head) return std::nullopt;
        T res = old_head->data;
        delete old_head;
        return res;
    }

    bool empty() const noexcept {
        return head_.load(std::memory_order_relaxed) == nullptr;
    }
};

// =============================================================================
// 4. Lock-Free Atomic Flag Spinlock Guard (Acquire-Release Memory Order)
//    MEMORY ORDERS: acquire (test_and_set lock), release (clear unlock)
//    USE-CASE: Low-overhead CPU-paused critical sections on hardware.
// =============================================================================
class LockFreeSpinlock {
public:
    void lock() noexcept {
        while (flag_.test_and_set(std::memory_order_acquire)) {
#if defined(__x86_64__) || defined(_M_X64)
            __builtin_ia32_pause(); // Low-latency CPU pause instruction
#endif
        }
    }

    void unlock() noexcept {
        flag_.clear(std::memory_order_release);
    }

private:
    alignas(64) std::atomic_flag flag_ = ATOMIC_FLAG_INIT;
};

// =============================================================================
// 5. C++20 std::atomic_ref Wrapper for Raw Non-Atomic Arrays
//    MEMORY ORDERS: relaxed (fast updates), acquire/release (synchronized updates)
//    USE-CASE: Atomic updates to non-atomic double arrays (e.g. q[6] joint positions)
//              without requiring std::atomic<double> storage overhead.
// =============================================================================
template <typename T, size_t N>
class AtomicRefArray {
public:
    explicit AtomicRefArray(std::array<T, N>& raw_data) : data_(raw_data) {}

    void store_element(size_t index, T value, std::memory_order order = std::memory_order_release) noexcept {
        if (index < N) {
            std::atomic_ref<T> ref(data_[index]);
            ref.store(value, order);
        }
    }

    T load_element(size_t index, std::memory_order order = std::memory_order_acquire) const noexcept {
        if (index < N) {
            std::atomic_ref<T> ref(const_cast<T&>(data_[index]));
            return ref.load(order);
        }
        return T{};
    }

private:
    std::array<T, N>& data_;
};

// =============================================================================
// 6. Sequentially-Consistent Emergency Stop Flag (std::memory_order_seq_cst)
//    MEMORY ORDERS: std::memory_order_seq_cst (Global Total Order)
//    USE-CASE: Emergency Stop (E-Stop) triggering across all CPU cores.
//              Guarantees strict global order execution across threads.
// =============================================================================
class AtomicEmergencyStop {
public:
    void triggerEmergencyStop() noexcept {
        e_stop_.store(true, std::memory_order_seq_cst);
    }

    bool isEmergencyStopped() const noexcept {
        return e_stop_.load(std::memory_order_seq_cst);
    }

    void reset() noexcept {
        e_stop_.store(false, std::memory_order_seq_cst);
    }

private:
    alignas(64) std::atomic<bool> e_stop_{false};
};

// =============================================================================
// 7. Atomic Metrics Counter (Demonstrating Memory Order Spectrum)
// =============================================================================
class AtomicMetricsCounter {
public:
    // Memory Order 1: Relaxed (High Throughput, no synchronization barrier)
    void incrementRelaxed() noexcept {
        count_.fetch_add(1, std::memory_order_relaxed);
    }

    // Memory Order 2: Acquire-Release (Read-Modify-Write synchronization)
    void incrementAcqRel() noexcept {
        count_.fetch_add(1, std::memory_order_acq_rel);
    }

    // Memory Order 3: Sequentially Consistent
    void incrementSeqCst() noexcept {
        count_.fetch_add(1, std::memory_order_seq_cst);
    }

    uint64_t get() const noexcept {
        return count_.load(std::memory_order_relaxed);
    }

    bool isLockFree() const noexcept {
        return count_.is_lock_free();
    }

private:
    alignas(64) std::atomic<uint64_t> count_{0};
};

} // namespace yaskawa::atomic

#endif // ATOMIC_STRUCTURES_HPP

