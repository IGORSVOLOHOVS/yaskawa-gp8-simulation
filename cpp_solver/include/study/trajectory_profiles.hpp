#ifndef YASKAWA_STUDY_TRAJECTORY_PROFILES_HPP
#define YASKAWA_STUDY_TRAJECTORY_PROFILES_HPP

// Course 3883 "Robotics Modelling" (M-407-01),
// Block 5 "Trajectory planning: joint-space polynomials, via points, linear
// segments with parabolic blends, Cartesian paths and the assessment of a
// profile against actuator limits, jerk and cycle time".
//
// Every generator here produces the same `Samples` structure, and
// `check_constraints` consumes that structure alone. That is deliberate: the
// feasibility verdict is then the same verdict for a cubic, a quintic, an LSPB,
// a via-point spline or a Cartesian blend, instead of six near-copies that can
// disagree. The limits it checks against are the real ones - joint range and
// maximum speed come from study/gp8_model.hpp, which reads them from
// GP8_JOINT_LIMITS, and the acceleration, jerk and cycle-time ceilings are
// parameters because the datasheet does not publish them.

#include "study/gp8_model.hpp"
#include "study/study_module.hpp"

#include <Eigen/Dense>

#include <array>
#include <cstddef>
#include <string>
#include <string_view>
#include <vector>

namespace yaskawa::study {

namespace traj {

using Vector6 = Eigen::Matrix<double, 6, 1>;

constexpr std::size_t kMaxDegree = 5;
constexpr int kMaxSamples = 2000;
constexpr std::size_t kMaxViaPoints = 16;

struct Sample {
    double t = 0.0;
    Vector6 q{Vector6::Zero()};
    Vector6 qd{Vector6::Zero()};
    Vector6 qdd{Vector6::Zero()};
    Vector6 qddd{Vector6::Zero()};
};

// ---------------------------------------------------------------------------
// Joint-space polynomials
// ---------------------------------------------------------------------------

// One polynomial per joint, q_i(t) = sum_k c[i][k] t^k, degree 3 or 5.
struct PolynomialProfile {
    std::array<std::array<double, kMaxDegree + 1>, GP8_DOF> c{};
    std::size_t degree = 3;
    double duration = 1.0;

    [[nodiscard]] Sample at(double t) const noexcept;
};

// Cubic: position and velocity matched at both ends. Acceleration steps.
[[nodiscard]] PolynomialProfile cubic_profile(const Vector6& q0, const Vector6& q1, double duration,
                                              double v0, double v1);

// Quintic: position, velocity and acceleration matched at both ends.
[[nodiscard]] PolynomialProfile quintic_profile(const Vector6& q0, const Vector6& q1,
                                                double duration, double v0, double v1, double a0,
                                                double a1);

// ---------------------------------------------------------------------------
// Linear segment with parabolic blends
// ---------------------------------------------------------------------------

// The blend is applied to a single path parameter s in [0, 1] and every joint
// follows q0 + s(t) (q1 - q0), so the six axes stay synchronised and the three
// phases are the same three phases for all of them.
struct LspbProfile {
    Vector6 q0{Vector6::Zero()};
    Vector6 delta{Vector6::Zero()};
    double duration = 1.0;
    double blend_time = 0.25;   // t_b, the parabolic part at each end
    double s_dot_cruise = 0.0;  // plateau rate of the path parameter [1/s]
    double s_ddot = 0.0;        // blend acceleration of the path parameter [1/s^2]

    [[nodiscard]] Sample at(double t) const noexcept;
};

// Throws StudyError when the combination is physically impossible: the blend
// has to fit twice inside the duration, and a cruise velocity below
// delta/duration can never cover the distance in the time allowed.
[[nodiscard]] LspbProfile lspb_from_blend_time(const Vector6& q0, const Vector6& q1,
                                               double duration, double blend_time);

// The same profile specified by the cruise velocity of one joint instead.
[[nodiscard]] LspbProfile lspb_from_cruise_velocity(const Vector6& q0, const Vector6& q1,
                                                    double duration, std::size_t joint,
                                                    double cruise_velocity);

// ---------------------------------------------------------------------------
// Via points with continuous velocity and acceleration
// ---------------------------------------------------------------------------

// Quintic per segment, with the interior velocities and accelerations shared by
// the two segments that meet there, so the spline is C2 by construction.
struct ViaProfile {
    std::vector<Vector6> points;
    std::vector<double> durations;    // one per segment
    std::vector<double> start_times;  // one per segment, cumulative
    std::vector<PolynomialProfile> segments;
    std::vector<Vector6> velocities;      // the interior velocity actually used
    std::vector<Vector6> accelerations;   // the interior acceleration actually used
    double total_duration = 0.0;

    [[nodiscard]] Sample at(double t) const noexcept;
};

// `durations` may be empty, in which case `total_duration` is split between the
// segments in proportion to their largest joint displacement.
[[nodiscard]] ViaProfile via_profile(const std::vector<Vector6>& points,
                                     const std::vector<double>& durations, double total_duration);

// ---------------------------------------------------------------------------
// Sampling and the feasibility verdict
// ---------------------------------------------------------------------------

struct Samples {
    std::vector<Sample> points;
    double duration = 0.0;
    bool jerk_bounded = true;  // false for a profile with stepped acceleration
    std::string kind;
};

[[nodiscard]] Samples sample_polynomial(const PolynomialProfile& profile, int count,
                                        std::string kind);
[[nodiscard]] Samples sample_lspb(const LspbProfile& profile, int count);
[[nodiscard]] Samples sample_via(const ViaProfile& profile, int count);

struct JointVerdict {
    double q_min = 0.0;
    double q_max = 0.0;
    double peak_speed = 0.0;
    double peak_accel = 0.0;
    double peak_jerk = 0.0;
    bool range_ok = true;
    bool velocity_ok = true;
    bool accel_ok = true;
    bool jerk_ok = true;
    double worst_ratio = 0.0;         // worst of the four, 1.0 is exactly at the limit
    std::string binding_constraint;   // which of the four is worst on this joint
};

struct ConstraintVerdict {
    std::array<JointVerdict, GP8_DOF> joints{};
    bool range_ok = true;
    bool velocity_ok = true;
    bool accel_ok = true;
    bool jerk_ok = true;
    bool cycle_time_ok = true;
    bool feasible = false;
    double cycle_time = 0.0;
    double worst_ratio = 0.0;
    std::size_t worst_joint = 0;
    std::string binding_constraint;  // the single sentence the course outcome asks for
    std::string verdict;             // "FEASIBLE" or "INFEASIBLE"
};

[[nodiscard]] ConstraintVerdict check_constraints(const Samples& samples, double accel_limit,
                                                  double jerk_limit, double cycle_time_limit);

}  // namespace traj

class TrajectoryProfilesModule final : public StudyModule {
public:
    [[nodiscard]] std::string_view name() const noexcept override { return "trajectory_profiles"; }
    [[nodiscard]] ModuleDescription describe() const override;
    [[nodiscard]] json::Value invoke(std::string_view op, const json::Value& args) const override;
};

}  // namespace yaskawa::study

#endif  // YASKAWA_STUDY_TRAJECTORY_PROFILES_HPP
