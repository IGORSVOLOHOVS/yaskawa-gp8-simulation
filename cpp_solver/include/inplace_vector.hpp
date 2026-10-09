#ifndef INPLACE_VECTOR_HPP
#define INPLACE_VECTOR_HPP

#if __has_include(<inplace_vector>)
#include <inplace_vector>
namespace yaskawa {
template <typename T, std::size_t N>
using inplace_vector = std::inplace_vector<T, N>;
}
#else

#include <cstddef>
#include <type_traits>
#include <new>
#include <utility>
#include <stdexcept>
#include <initializer_list>
#include <iterator>

namespace yaskawa {

// =============================================================================
// C++26 std::inplace_vector (P0843R14) Conforming Implementation
// Contiguous fixed-capacity sequence container without heap allocation.
// =============================================================================
template <typename T, std::size_t Capacity>
class inplace_vector {
    static_assert(Capacity > 0, "inplace_vector capacity must be greater than 0");

    alignas(T) std::byte storage_[Capacity * sizeof(T)];
    std::size_t size_{0};

    T* element_ptr(std::size_t index) noexcept {
        return reinterpret_cast<T*>(&storage_[index * sizeof(T)]);
    }

    const T* element_ptr(std::size_t index) const noexcept {
        return reinterpret_cast<const T*>(&storage_[index * sizeof(T)]);
    }

public:
    using value_type = T;
    using size_type = std::size_t;
    using difference_type = std::ptrdiff_t;
    using reference = T&;
    using const_reference = const T&;
    using pointer = T*;
    using const_pointer = const T*;
    using iterator = T*;
    using const_iterator = const T*;

    constexpr inplace_vector() noexcept = default;

    inplace_vector(const inplace_vector& other) {
        for (const auto& item : other) {
            push_back(item);
        }
    }

    inplace_vector(inplace_vector&& other) noexcept(std::is_nothrow_move_constructible_v<T>) {
        for (auto& item : other) {
            push_back(std::move(item));
        }
    }

    inplace_vector& operator=(const inplace_vector& other) {
        if (this != &other) {
            clear();
            for (const auto& item : other) {
                push_back(item);
            }
        }
        return *this;
    }

    inplace_vector& operator=(inplace_vector&& other) noexcept(std::is_nothrow_move_constructible_v<T>) {
        if (this != &other) {
            clear();
            for (auto& item : other) {
                push_back(std::move(item));
            }
        }
        return *this;
    }

    ~inplace_vector() {
        clear();
    }

    [[nodiscard]] constexpr size_type size() const noexcept { return size_; }
    [[nodiscard]] constexpr size_type capacity() const noexcept { return Capacity; }
    [[nodiscard]] constexpr size_type max_size() const noexcept { return Capacity; }
    [[nodiscard]] constexpr bool empty() const noexcept { return size_ == 0; }

    reference operator[](size_type index) noexcept { return *element_ptr(index); }
    const_reference operator[](size_type index) const noexcept { return *element_ptr(index); }

    reference at(size_type index) {
        if (index >= size_) {
            throw std::out_of_range("inplace_vector::at out of range");
        }
        return *element_ptr(index);
    }

    const_reference at(size_type index) const {
        if (index >= size_) {
            throw std::out_of_range("inplace_vector::at out of range");
        }
        return *element_ptr(index);
    }

    reference front() noexcept { return *element_ptr(0); }
    const_reference front() const noexcept { return *element_ptr(0); }
    reference back() noexcept { return *element_ptr(size_ - 1); }
    const_reference back() const noexcept { return *element_ptr(size_ - 1); }

    pointer data() noexcept { return element_ptr(0); }
    const_pointer data() const noexcept { return element_ptr(0); }

    iterator begin() noexcept { return element_ptr(0); }
    const_iterator begin() const noexcept { return element_ptr(0); }
    const_iterator cbegin() const noexcept { return element_ptr(0); }

    iterator end() noexcept { return element_ptr(size_); }
    const_iterator end() const noexcept { return element_ptr(size_); }
    const_iterator cend() const noexcept { return element_ptr(size_); }

    template <typename... Args>
    reference emplace_back(Args&&... args) {
        if (size_ >= Capacity) {
            throw std::bad_alloc();
        }
        T* p = ::new (static_cast<void*>(element_ptr(size_))) T(std::forward<Args>(args)...);
        ++size_;
        return *p;
    }

    reference push_back(const T& value) {
        return emplace_back(value);
    }

    reference push_back(T&& value) {
        return emplace_back(std::move(value));
    }

    void pop_back() noexcept {
        if (size_ > 0) {
            --size_;
            element_ptr(size_)->~T();
        }
    }

    void clear() noexcept {
        while (size_ > 0) {
            pop_back();
        }
    }

    void reserve(size_type n) {
        if (n > Capacity) {
            throw std::bad_alloc();
        }
    }
};

} // namespace yaskawa
#endif

#endif // INPLACE_VECTOR_HPP
