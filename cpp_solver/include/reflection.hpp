#ifndef REFLECTION_HPP
#define REFLECTION_HPP

#include "yaskawa_kinematics.hpp"
#include <string_view>
#include <array>
#include <tuple>
#include <type_traits>
#include <utility>
#include <iostream>

namespace yaskawa::meta {

// =============================================================================
// C++26 Compile-Time Static Reflection Paradigm
// =============================================================================

// Reflect Type Name
template <typename T>
constexpr std::string_view reflect_type_name() noexcept {
#if defined(__clang__) || defined(__GNUC__)
    std::string_view name = __PRETTY_FUNCTION__;
    size_t start = name.find("T = ") + 4;
    size_t end = name.find_last_of("]");
    return name.substr(start, end - start);
#else
    return "UnknownType";
#endif
}

// Reflect Robot Joint Names at Compile Time
constexpr std::array<std::string_view, DOF> JOINT_NAMES = {
    "joint_1_s",
    "joint_2_l",
    "joint_3_u",
    "joint_4_r",
    "joint_5_b",
    "joint_6_t"
};

// Reflect Joint Limit Bounds at Compile Time
template <size_t Index>
constexpr auto reflect_joint_info() noexcept {
    static_assert(Index < DOF, "Joint index out of bounds for reflection!");
    constexpr auto limit = GP8_JOINT_LIMITS[Index];
    return std::make_tuple(JOINT_NAMES[Index], limit.min_angle, limit.max_angle, limit.max_vel);
}

// Reflect Struct Members / Properties Iteration
template <typename Func, std::size_t... Is>
constexpr void reflect_for_each_joint_impl(Func&& f, std::index_sequence<Is...>) {
    (f.template operator()<Is>(JOINT_NAMES[Is], GP8_JOINT_LIMITS[Is]), ...);
}

template <typename Func>
constexpr void reflect_for_each_joint(Func&& f) {
    reflect_for_each_joint_impl(std::forward<Func>(f), std::make_index_sequence<DOF>{});
}

} // namespace yaskawa::meta

#endif // REFLECTION_HPP
