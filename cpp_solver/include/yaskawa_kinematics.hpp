#ifndef YASKAWA_KINEMATICS_HPP
#define YASKAWA_KINEMATICS_HPP

#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <concepts>
#include <array>
#include <type_traits>
#include <cmath>
#include <iostream>

namespace yaskawa {

constexpr size_t DOF = 6;

// 64-byte Cache-Line Aligned Joint Limit Structure (L1 Cache Optimized)
struct alignas(64) constexpr_joint_limit {
    double min_angle;
    double max_angle;
    double max_vel;
    double _pad[5]; // Pad to exactly 64 bytes

    constexpr bool is_valid(double angle) const noexcept {
        return angle >= min_angle && angle <= max_angle;
    }

    constexpr double clamp(double angle) const noexcept {
        return (angle < min_angle) ? min_angle : ((angle > max_angle) ? max_angle : angle);
    }
};

static_assert(sizeof(constexpr_joint_limit) == 64, "constexpr_joint_limit must be exactly 64 bytes!");

constexpr std::array<constexpr_joint_limit, DOF> GP8_JOINT_LIMITS = {{
    {-2.967,  2.967, 7.85},  // Joint 1
    {-1.134,  2.530, 6.70},  // Joint 2
    {-1.221,  3.316, 9.07},  // Joint 3
    {-3.316,  3.316, 9.60},  // Joint 4
    {-2.356,  2.356, 9.60},  // Joint 5
    {-6.283,  6.283, 17.45}  // Joint 6
}};

// Pre-Allocated Thread-Local Workspace Buffer for Zero-Allocation IK Solving
struct alignas(64) IKWorkspaceBuffer {
    Eigen::Matrix<double, 6, DOF> J;
    Eigen::Matrix<double, 6, 6> JJt;
    Eigen::Matrix<double, 6, 1> error;
    Eigen::Matrix<double, DOF, 1> dq;
    Eigen::Matrix<double, DOF, 1> q_plus;
    Eigen::Isometry3d T_curr;
    Eigen::Isometry3d T_plus;
    std::array<double, DOF> cos_q;
    std::array<double, DOF> sin_q;
};

// C++20/C++23 Concepts
template <typename VectorType>
concept JointVectorConcept = requires(VectorType v) {
    { v.size() } -> std::same_as<std::ptrdiff_t>;
    { v[0] } -> std::convertible_to<double>;
};

template <typename SolverType>
concept KinematicsSolverConcept = requires(SolverType solver, const Eigen::Matrix<double, DOF, 1>& q, const Eigen::Isometry3d& pose) {
    { solver.forwardKinematics(q) } -> std::same_as<Eigen::Isometry3d>;
    { solver.computeJacobian(q) } -> std::same_as<Eigen::Matrix<double, 6, DOF>>;
    { solver.inverseKinematics(pose, q, std::declval<Eigen::Matrix<double, DOF, 1>&>()) } -> std::same_as<bool>;
};

class YaskawaKinematics {
public:
    constexpr YaskawaKinematics() noexcept = default;

    // Fast Forward Kinematics with Trigonometry Cache
    Eigen::Isometry3d forwardKinematics(const Eigen::Matrix<double, DOF, 1>& q) const noexcept;

    // Fast Forward Kinematics with Pre-computed Sin/Cos Cache
    Eigen::Isometry3d forwardKinematicsCached(
        const std::array<double, DOF>& sin_q,
        const std::array<double, DOF>& cos_q
    ) const noexcept;

    // Analytical/Numerical Jacobian
    Eigen::Matrix<double, 6, DOF> computeJacobian(const Eigen::Matrix<double, DOF, 1>& q) const noexcept;

    // Zero-Allocation IK Solver using Pre-Allocated Workspace & Trigonometry Cache
    bool inverseKinematics(
        const Eigen::Isometry3d& target_pose,
        const Eigen::Matrix<double, DOF, 1>& q_init,
        Eigen::Matrix<double, DOF, 1>& q_out,
        double pos_tol = 1e-4,
        double rot_tol = 1e-3,
        int max_iters = 100
    ) const noexcept;

    Eigen::Matrix<double, DOF, 1> clampJoints(const Eigen::Matrix<double, DOF, 1>& q) const noexcept {
        Eigen::Matrix<double, DOF, 1> q_clamped;
        for (size_t i = 0; i < DOF; ++i) {
            q_clamped[i] = GP8_JOINT_LIMITS[i].clamp(q[i]);
        }
        return q_clamped;
    }
};

static_assert(KinematicsSolverConcept<YaskawaKinematics>, "YaskawaKinematics must satisfy KinematicsSolverConcept!");

} // namespace yaskawa

#endif // YASKAWA_KINEMATICS_HPP
