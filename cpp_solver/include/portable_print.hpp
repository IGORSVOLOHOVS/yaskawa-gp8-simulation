#pragma once

#if __has_include(<print>)
#include <print>
#else
#include <iostream>
#include <format>
#include <string_view>
#include <utility>

namespace std {

template <typename... Args>
inline void print(format_string<Args...> fmt, Args&&... args) {
    std::cout << std::format(fmt, std::forward<Args>(args)...);
}

template <typename... Args>
inline void println(format_string<Args...> fmt, Args&&... args) {
    std::cout << std::format(fmt, std::forward<Args>(args)...) << "\n";
}

inline void println(const std::string_view s) {
    std::cout << s << "\n";
}

inline void println() {
    std::cout << "\n";
}

} // namespace std
#endif
