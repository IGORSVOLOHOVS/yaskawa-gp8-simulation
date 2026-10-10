#ifndef YASKAWA_STUDY_JACOBIAN_STATICS_HPP
#define YASKAWA_STUDY_JACOBIAN_STATICS_HPP

// Course 3883 "Robotics Modelling" (M-407-01), Block 3
// "Differential Kinematics and Statics".
//
// One matrix carries this whole block. J(q) maps joint rates to the tool
// twist, its transpose maps a tool wrench back to joint torques, its singular
// values say how well conditioned both directions are, and its rank says
// whether a direction of motion exists at all. Every op below is a reading of
// the same J, built from the link frames of study/gp8_model.hpp.
//
// Twist and wrench ordering is [linear; angular] throughout, matching
// YaskawaKinematics::computeJacobian and the IK error vector of
// yaskawa_kinematics.cpp, so a row index means the same thing everywhere.

#include "study/gp8_model.hpp"
#include "study/study_module.hpp"

#include <Eigen/Dense>
#include <Eigen/Geometry>

#include <array>
#include <string_view>

namespace yaskawa::study {

namespace jac {

using JointVector = Eigen::Matrix<double, 6, 1>;
using Twist = Eigen::Matrix<double, 6, 1>;   // [vx vy vz wx wy wz]
using Wrench = Eigen::Matrix<double, 6, 1>;  // [fx fy fz mx my mz]
using Jacobian = Eigen::Matrix<double, 6, 6>;

// Geometric Jacobian in the base frame. For the all-revolute GP8, column i is
// [z_{i-1} x (p_e - o_{i-1}); z_{i-1}]. No finite differences anywhere.
[[nodiscard]] Jacobian geometric_jacobian(const JointVector& q) noexcept;

// Central-difference Jacobian of the DH forward kinematics. It exists to be
// compared against the analytic one, which is the single check that proves the
// frame assignment. The test suite and the geometric_jacobian op report that
// comparison; no answer is ever computed from it.
[[nodiscard]] Jacobian numeric_jacobian(const JointVector& q, double step = 1e-6) noexcept;

// Rotation vector of the relative rotation after * before^T: the small angular
// displacement between two orientations, used by numeric_jacobian.
[[nodiscard]] Eigen::Vector3d rotation_difference(const Eigen::Matrix3d& after,
                                                  const Eigen::Matrix3d& before) noexcept;

// Damped least squares: qdot = J^T (J J^T + lambda^2 I)^{-1} twist.
// The residual is what the damping costs; it is returned, not absorbed.
struct DampedSolution {
    JointVector qdot{JointVector::Zero()};
    Twist achieved{Twist::Zero()};
    Twist residual{Twist::Zero()};
    double residual_norm = 0.0;
};

[[nodiscard]] DampedSolution damped_least_squares(const Jacobian& J, const Twist& twist,
                                                  double lambda) noexcept;

// Singular values, descending and non-negative by construction.
[[nodiscard]] Eigen::Matrix<double, 6, 1> singular_values(const Jacobian& J) noexcept;

// Yoshikawa's measure sqrt(det(J J^T)), the product of the singular values and
// the volume of the six-dimensional velocity ellipsoid.
[[nodiscard]] double manipulability(const Jacobian& J) noexcept;

// Principal axes and radii of the translational velocity ellipsoid at the
// tool: the eigenvectors of J_v J_v^T and the square roots of its eigenvalues.
// This is the ellipsoid the 3D view draws at the flange.
struct VelocityEllipsoid {
    std::array<Eigen::Vector3d, 3> axes{};           // unit vectors, base frame
    Eigen::Vector3d radii{Eigen::Vector3d::Zero()};  // [m/s] per unit norm of qdot
    double volume = 0.0;
};

[[nodiscard]] VelocityEllipsoid velocity_ellipsoid(const Jacobian& J) noexcept;

}  // namespace jac

class JacobianStaticsModule final : public StudyModule {
public:
    [[nodiscard]] std::string_view name() const noexcept override { return "jacobian_statics"; }
    [[nodiscard]] ModuleDescription describe() const override;
    [[nodiscard]] json::Value invoke(std::string_view op, const json::Value& args) const override;
};

}  // namespace yaskawa::study

#endif  // YASKAWA_STUDY_JACOBIAN_STATICS_HPP
