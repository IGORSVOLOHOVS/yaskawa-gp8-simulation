#ifndef YASKAWA_STUDY_DH_KINEMATICS_HPP
#define YASKAWA_STUDY_DH_KINEMATICS_HPP

// Course 3883 "Robotics Modelling" (M-407-01), Block 2
// "Kinematic Chains: the DH Convention, Forward and Inverse Kinematics",
// with course 3286 "Linear Algebra" as the secondary reference wherever a
// result is really a statement about a matrix: the rank of the Jacobian in
// singularity_scan, the 2x2 linear solve that recovers q2, and the amplitude
// condition on A cos(q3) + B sin(q3) that decides reachability.
//
// Everything here is built on the standard-DH table of study/gp8_model.hpp and
// on nothing else, so the chain the student reads in dh_table is literally the
// chain that produces every pose, point cloud and determinant below.
//
// The GP8 wrist is spherical and d6 = a6 = 0, so the flange origin IS the
// wrist centre. That is what makes a closed form possible: position decouples
// onto (q1, q2, q3) and orientation onto (q4, q5, q6).

#include "study/gp8_model.hpp"
#include "study/study_module.hpp"

#include <Eigen/Dense>
#include <Eigen/Geometry>

#include <array>
#include <cstddef>
#include <string>
#include <string_view>
#include <vector>

namespace yaskawa::study {

// Free functions, so jacobian_statics, dynamics and trajectory_profiles reuse
// one inverse instead of each growing their own.
namespace dhk {

using JointVector = Eigen::Matrix<double, 6, 1>;

// The per-joint link transforms A_1 ... A_6, so the chain product is visible
// one factor at a time and not only as its result.
[[nodiscard]] std::array<Eigen::Isometry3d, GP8_DOF> joint_transforms(
    const JointVector& q) noexcept;

// Centre of the spherical wrist, which for this robot coincides with the
// flange origin and therefore depends only on q1, q2, q3.
[[nodiscard]] Eigen::Vector3d wrist_center(const JointVector& q) noexcept;

// Angle of the relative rotation A * B^T: how far apart two orientations are,
// in radians. Zero when they are equal.
[[nodiscard]] double orientation_error(const Eigen::Matrix3d& A, const Eigen::Matrix3d& B) noexcept;

// One closed-form solution carrying everything needed to judge it: which
// branch it is, whether the robot can hold it, and what its own
// forward-kinematics round trip costs. A wrong branch cannot hide behind a
// plausible-looking joint vector.
struct IkBranch {
    JointVector q{JointVector::Zero()};
    int shoulder = 1;                      // +1 front, -1 reaching backwards
    int elbow = 1;                         // +1 / -1, the two acos branches
    int wrist = 1;                         // +1 / -1, the wrist flip
    double position_error = 0.0;           // [m]   norm of FK(q).p - target.p
    double orientation_error = 0.0;        // [rad] angle between the rotations
    bool within_limits = true;
    std::size_t first_violated_joint = GP8_DOF;  // GP8_DOF when none is
};

struct IkGeometric {
    std::vector<IkBranch> branches;    // every branch the decoupling finds
    std::size_t within_limits = 0;     // how many of them the robot can hold
    bool reachable = false;            // branches non-empty
    double required_cosine = 0.0;      // |K| of the law-of-cosines test
    double available_amplitude = 0.0;  // the amplitude |K| must not exceed
    std::string note;                  // what it found, or why it failed
};

// Closed-form inverse kinematics by position/orientation decoupling.
// `target` is a flange pose in the same convention as
// YaskawaKinematics::forwardKinematics, i.e. the DH chain right-multiplied by
// the flange correction of study/gp8_model.hpp.
[[nodiscard]] IkGeometric solve_ik_geometric(const Eigen::Isometry3d& target);

// Which degeneracy a configuration sits in, and how far it is from each.
struct SingularityVerdict {
    double wrist_measure = 0.0;     // |sin(q5)|, axes 4 and 6 aligned at zero
    double elbow_measure = 0.0;     // |a3 sin(q3) + d4 cos(q3)|, 0 when stretched
    double shoulder_measure = 0.0;  // wrist-centre distance from the q1 axis
    std::string type;               // "wrist", "elbow", "shoulder" or "none"
    std::string text;               // one sentence for the panel
};

[[nodiscard]] SingularityVerdict classify_singularity(const JointVector& q,
                                                      double threshold) noexcept;

}  // namespace dhk

class DhKinematicsModule final : public StudyModule {
public:
    [[nodiscard]] std::string_view name() const noexcept override { return "dh_kinematics"; }
    [[nodiscard]] ModuleDescription describe() const override;
    [[nodiscard]] json::Value invoke(std::string_view op, const json::Value& args) const override;
};

}  // namespace yaskawa::study

#endif  // YASKAWA_STUDY_DH_KINEMATICS_HPP
