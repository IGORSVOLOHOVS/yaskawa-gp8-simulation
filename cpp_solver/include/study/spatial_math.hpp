#ifndef YASKAWA_STUDY_SPATIAL_MATH_HPP
#define YASKAWA_STUDY_SPATIAL_MATH_HPP

// Course 3883 "Robotics Modelling" (M-407-01),
// Block 1 "Spatial Descriptions and Transformations".
//
// The reference study module: rotations, their parameterisations, homogeneous
// transforms, and the link frames of the GP8 taken from study/gp8_model.hpp.
//
// Nothing here orthonormalises a matrix behind the student's back. Every op
// that produces or consumes a rotation also reports
//     orthonormality_error = || R^T R - I ||_F
// so drift is visible instead of hidden.

#include "study/study_module.hpp"

#include <Eigen/Dense>
#include <Eigen/Geometry>

#include <string>
#include <string_view>

namespace yaskawa::study {

// Free functions, so later modules (dynamics, control) reuse the same maps
// instead of rewriting them.
namespace spatial {

// Fixed-axis XYZ roll-pitch-yaw: R = Rz(yaw) Ry(pitch) Rx(roll).
[[nodiscard]] Eigen::Matrix3d rotation_from_rpy(const Eigen::Vector3d& rpy) noexcept;

// Moving-axis (intrinsic) ZYZ Euler angles: R = Rz(a) Ry(b) Rz(c).
[[nodiscard]] Eigen::Matrix3d rotation_from_euler_zyz(const Eigen::Vector3d& angles) noexcept;

// Moving-axis (intrinsic) ZYX Euler angles: R = Rz(a) Ry(b) Rx(c).
[[nodiscard]] Eigen::Matrix3d rotation_from_euler_zyx(const Eigen::Vector3d& angles) noexcept;

struct RpyBranches {
    Eigen::Vector3d primary{Eigen::Vector3d::Zero()};    // pitch in [-pi/2, pi/2]
    Eigen::Vector3d alternate{Eigen::Vector3d::Zero()};  // pitch outside it
    double gimbal_margin = 1.0;  // |cos(pitch)|, 0 exactly at gimbal lock
    bool gimbal_lock = false;    // margin below the numerical threshold
};

// Inverse of rotation_from_rpy. Both branches are returned, because both are
// correct: a rotation matrix has two RPY pre-images away from gimbal lock.
[[nodiscard]] RpyBranches rpy_from_rotation(const Eigen::Matrix3d& R) noexcept;

// || R^T R - I ||_F, the Frobenius drift away from a true rotation.
[[nodiscard]] double orthonormality_error(const Eigen::Matrix3d& R) noexcept;

// Homogeneous transform from a roll-pitch-yaw triple and a translation.
[[nodiscard]] Eigen::Isometry3d transform_from_rpy_p(const Eigen::Vector3d& rpy,
                                                     const Eigen::Vector3d& p) noexcept;

}  // namespace spatial

class SpatialMathModule final : public StudyModule {
public:
    [[nodiscard]] std::string_view name() const noexcept override { return "spatial_math"; }
    [[nodiscard]] ModuleDescription describe() const override;
    [[nodiscard]] json::Value invoke(std::string_view op, const json::Value& args) const override;
};

}  // namespace yaskawa::study

#endif  // YASKAWA_STUDY_SPATIAL_MATH_HPP
