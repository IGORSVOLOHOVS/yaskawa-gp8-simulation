#ifndef MEMORY_POOL_HPP
#define MEMORY_POOL_HPP

#include <vector>
#include <cstddef>
#include <memory>
#include <mutex>
#include <stdexcept>

namespace yaskawa {

template <typename T, size_t Capacity = 10000>
class FixedMemoryPool {
public:
    FixedMemoryPool() : free_index_(0) {
        pool_.resize(Capacity);
        free_pointers_.reserve(Capacity);
        for (size_t i = 0; i < Capacity; ++i) {
            free_pointers_.push_back(&pool_[i]);
        }
    }

    template <typename... Args>
    T* allocate(Args&&... args) {
        std::lock_guard<std::mutex> lock(mutex_);
        if (free_pointers_.empty()) {
            throw std::runtime_error("FixedMemoryPool capacity exceeded!");
        }
        T* ptr = free_pointers_.back();
        free_pointers_.pop_back();
        new (ptr) T(std::forward<Args>(args)...);
        return ptr;
    }

    void deallocate(T* ptr) {
        if (!ptr) return;
        ptr->~T();
        std::lock_guard<std::mutex> lock(mutex_);
        free_pointers_.push_back(ptr);
    }

    size_t available() const {
        std::lock_guard<std::mutex> lock(mutex_);
        return free_pointers_.size();
    }

private:
    std::vector<T> pool_;
    std::vector<T*> free_pointers_;
    size_t free_index_;
    mutable std::mutex mutex_;
};

} // namespace yaskawa

#endif // MEMORY_POOL_HPP
