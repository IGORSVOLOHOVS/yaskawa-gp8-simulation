#ifndef COROUTINE_GENERATOR_HPP
#define COROUTINE_GENERATOR_HPP

#include <coroutine>
#include <exception>
#include <optional>
#include <utility>

namespace yaskawa {

// C++20/C++23 Lazy Generator Coroutine Type
template <typename T>
class Generator {
public:
    struct promise_type {
        T current_value;
        std::exception_ptr exception;

        Generator get_return_object() {
            return Generator{std::coroutine_handle<promise_type>::from_promise(*this)};
        }

        std::suspend_always initial_suspend() noexcept { return {}; }
        std::suspend_always final_suspend() noexcept { return {}; }

        std::suspend_always yield_value(T value) noexcept {
            current_value = std::move(value);
            return {};
        }

        void return_void() noexcept {}

        void unhandled_exception() {
            exception = std::current_exception();
        }
    };

    using handle_type = std::coroutine_handle<promise_type>;

    explicit Generator(handle_type h) : handle_(h) {}
    ~Generator() {
        if (handle_) handle_.destroy();
    }

    Generator(const Generator&) = delete;
    Generator& operator=(const Generator&) = delete;

    Generator(Generator&& other) noexcept : handle_(std::exchange(other.handle_, nullptr)) {}
    Generator& operator=(Generator&& other) noexcept {
        if (this != &other) {
            if (handle_) handle_.destroy();
            handle_ = std::exchange(other.handle_, nullptr);
        }
        return *this;
    }

    // Advance coroutine and check if more values are available
    bool next() {
        if (handle_ && !handle_.done()) {
            handle_.resume();
            if (handle_.promise().exception) {
                std::rethrow_exception(handle_.promise().exception);
            }
            return !handle_.done();
        }
        return false;
    }

    // Get reference to current lazy-evaluated value
    const T& value() const {
        return handle_.promise().current_value;
    }

private:
    handle_type handle_;
};

} // namespace yaskawa

#endif // COROUTINE_GENERATOR_HPP
