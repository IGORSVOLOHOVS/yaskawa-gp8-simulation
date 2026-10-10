#include "study/trajectory_profiles.hpp"

#include "yaskawa_kinematics.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <numbers>
#include <string>
#include <utility>

namespace yaskawa::study {

namespace {

using traj::Sample;
using traj::Samples;
using traj::Vector6;

constexpr double kPi = std::numbers::pi;
constexpr double kMinDuration = 0.02;    // [s]
constexpr double kMaxDuration = 120.0;   // [s]
constexpr double kBoundaryRateGuard = 20.0;   // [rad/s] on a boundary velocity
constexpr double kBoundaryAccelGuard = 200.0;  // [rad/s^2] on a boundary acceleration
constexpr double kEpsTime = 1e-12;

[[nodiscard]] const char* axis_name(std::size_t joint) noexcept {
    return GP8_AXIS_NAMES[joint];
}

[[nodiscard]] std::string joint_label(std::size_t joint) {
    return std::string("axis ") + axis_name(joint) + " (joint " + std::to_string(joint + 1) + ")";
}

[[nodiscard]] double clamp_time(double t, double duration) noexcept {
    return (t < 0.0) ? 0.0 : ((t > duration) ? duration : t);
}

}  // namespace

namespace traj {

// ---------------------------------------------------------------------------
// Polynomials
// ---------------------------------------------------------------------------

Sample PolynomialProfile::at(double t) const noexcept {
    const double time = clamp_time(t, duration);
    Sample s;
    s.t = time;
    // Each derivative gets its own power series rather than being differenced:
    // the boundary values then come out exactly, which is what the test checks.
    for (std::size_t j = 0; j < GP8_DOF; ++j) {
        const auto index = static_cast<Eigen::Index>(j);
        double position = 0.0;
        double velocity = 0.0;
        double acceleration = 0.0;
        double jerk = 0.0;
        double p = 1.0;  // t^k
        for (std::size_t k = 0; k <= degree; ++k) {
            position += c[j][k] * p;
            p *= time;
        }
        p = 1.0;
        for (std::size_t k = 1; k <= degree; ++k) {
            velocity += c[j][k] * static_cast<double>(k) * p;
            p *= time;
        }
        p = 1.0;
        for (std::size_t k = 2; k <= degree; ++k) {
            acceleration += c[j][k] * static_cast<double>(k) * static_cast<double>(k - 1) * p;
            p *= time;
        }
        p = 1.0;
        for (std::size_t k = 3; k <= degree; ++k) {
            jerk += c[j][k] * static_cast<double>(k) * static_cast<double>(k - 1) *
                    static_cast<double>(k - 2) * p;
            p *= time;
        }
        s.q(index) = position;
        s.qd(index) = velocity;
        s.qdd(index) = acceleration;
        s.qddd(index) = jerk;
    }
    return s;
}

PolynomialProfile cubic_profile(const Vector6& q0, const Vector6& q1, double duration, double v0,
                                double v1) {
    if (!(duration > 0.0)) {
        throw StudyError("parameter 'duration' must be positive");
    }
    PolynomialProfile profile;
    profile.degree = 3;
    profile.duration = duration;
    const double T = duration;
    for (std::size_t j = 0; j < GP8_DOF; ++j) {
        const auto index = static_cast<Eigen::Index>(j);
        const double h = q1(index) - q0(index);
        profile.c[j].fill(0.0);
        profile.c[j][0] = q0(index);
        profile.c[j][1] = v0;
        profile.c[j][2] = (3.0 * h - (2.0 * v0 + v1) * T) / (T * T);
        profile.c[j][3] = (-2.0 * h + (v0 + v1) * T) / (T * T * T);
    }
    return profile;
}

PolynomialProfile quintic_profile(const Vector6& q0, const Vector6& q1, double duration, double v0,
                                  double v1, double a0, double a1) {
    if (!(duration > 0.0)) {
        throw StudyError("parameter 'duration' must be positive");
    }
    PolynomialProfile profile;
    profile.degree = 5;
    profile.duration = duration;
    const double T = duration;
    const double T2 = T * T;
    const double T3 = T2 * T;
    const double T4 = T3 * T;
    const double T5 = T4 * T;
    for (std::size_t j = 0; j < GP8_DOF; ++j) {
        const auto index = static_cast<Eigen::Index>(j);
        const double h = q1(index) - q0(index);
        profile.c[j].fill(0.0);
        profile.c[j][0] = q0(index);
        profile.c[j][1] = v0;
        profile.c[j][2] = 0.5 * a0;
        profile.c[j][3] = (20.0 * h - (8.0 * v1 + 12.0 * v0) * T - (3.0 * a0 - a1) * T2) /
                          (2.0 * T3);
        profile.c[j][4] = (-30.0 * h + (14.0 * v1 + 16.0 * v0) * T + (3.0 * a0 - 2.0 * a1) * T2) /
                          (2.0 * T4);
        profile.c[j][5] = (12.0 * h - 6.0 * (v1 + v0) * T - (a0 - a1) * T2) / (2.0 * T5);
    }
    return profile;
}

// ---------------------------------------------------------------------------
// LSPB
// ---------------------------------------------------------------------------

Sample LspbProfile::at(double t) const noexcept {
    const double time = clamp_time(t, duration);
    const double cruise_end = duration - blend_time;
    double s = 0.0;
    double sd = 0.0;
    double sdd = 0.0;
    if (time < blend_time) {
        s = 0.5 * s_ddot * time * time;
        sd = s_ddot * time;
        sdd = s_ddot;
    } else if (time <= cruise_end) {
        s = s_dot_cruise * (time - 0.5 * blend_time);
        sd = s_dot_cruise;
        sdd = 0.0;
    } else {
        const double remaining = duration - time;
        s = 1.0 - 0.5 * s_ddot * remaining * remaining;
        sd = s_ddot * remaining;
        sdd = -s_ddot;
    }
    Sample out;
    out.t = time;
    out.q = q0 + s * delta;
    out.qd = sd * delta;
    out.qdd = sdd * delta;
    out.qddd = Vector6::Zero();  // zero inside every phase, impulsive at the joins
    return out;
}

LspbProfile lspb_from_blend_time(const Vector6& q0, const Vector6& q1, double duration,
                                 double blend_time) {
    if (!(duration > 0.0)) {
        throw StudyError("parameter 'duration' must be positive");
    }
    if (!(blend_time > 0.0)) {
        throw StudyError("parameter 'blend_time' must be positive: a blend of zero length is a "
                         "step in velocity, which no actuator can deliver");
    }
    if (blend_time > 0.5 * duration + kEpsTime) {
        throw StudyError("parameter 'blend_time' = " + json::number_to_string(blend_time) +
                         " s cannot exceed half the duration (" +
                         json::number_to_string(0.5 * duration) +
                         " s): the two parabolic blends would overlap and there would be no "
                         "linear segment between them");
    }
    LspbProfile profile;
    profile.q0 = q0;
    profile.delta = q1 - q0;
    profile.duration = duration;
    profile.blend_time = std::min(blend_time, 0.5 * duration);
    profile.s_dot_cruise = 1.0 / (duration - profile.blend_time);
    profile.s_ddot = profile.s_dot_cruise / profile.blend_time;
    return profile;
}

LspbProfile lspb_from_cruise_velocity(const Vector6& q0, const Vector6& q1, double duration,
                                      std::size_t joint, double cruise_velocity) {
    if (!(duration > 0.0)) {
        throw StudyError("parameter 'duration' must be positive");
    }
    if (joint >= GP8_DOF) {
        throw StudyError("parameter 'joint' must select one of the six axes");
    }
    if (!(cruise_velocity > 0.0)) {
        throw StudyError("parameter 'cruise_velocity' must be positive; it is the magnitude of "
                         "the plateau speed, and the direction comes from the two poses");
    }
    const double travel = std::abs(q1(static_cast<Eigen::Index>(joint)) -
                                   q0(static_cast<Eigen::Index>(joint)));
    if (travel < 1e-9) {
        throw StudyError("the cruise velocity is read on " + joint_label(joint) +
                         ", but that axis does not move between the two poses (travel " +
                         json::number_to_string(travel) +
                         " rad): pick a joint that moves, or specify the blend time instead");
    }
    const double average_speed = travel / duration;
    if (cruise_velocity <= average_speed * (1.0 + 1e-12)) {
        throw StudyError("cruise velocity " + json::number_to_string(cruise_velocity) +
                         " rad/s is not above the average speed this move needs (" +
                         json::number_to_string(average_speed) + " rad/s = " +
                         json::number_to_string(travel) + " rad in " +
                         json::number_to_string(duration) +
                         " s): the plateau has to be faster than the average, because the blends "
                         "are slower than it");
    }
    if (cruise_velocity > 2.0 * average_speed * (1.0 + 1e-12)) {
        throw StudyError("cruise velocity " + json::number_to_string(cruise_velocity) +
                         " rad/s exceeds twice the average speed (" +
                         json::number_to_string(2.0 * average_speed) +
                         " rad/s): the triangular profile, with no plateau at all, is the fastest "
                         "a symmetric blend can cruise in this duration, so the blend time would "
                         "come out negative");
    }
    const double s_dot = cruise_velocity / travel;
    LspbProfile profile;
    profile.q0 = q0;
    profile.delta = q1 - q0;
    profile.duration = duration;
    profile.blend_time = std::min(duration - 1.0 / s_dot, 0.5 * duration);
    profile.s_dot_cruise = s_dot;
    profile.s_ddot = s_dot / profile.blend_time;
    return profile;
}

// ---------------------------------------------------------------------------
// Via points
// ---------------------------------------------------------------------------

ViaProfile via_profile(const std::vector<Vector6>& points, const std::vector<double>& durations,
                       double total_duration) {
    if (points.size() < 2) {
        throw StudyError("parameter 'points' needs at least 2 waypoints, received " +
                         std::to_string(points.size()));
    }
    if (points.size() > kMaxViaPoints) {
        throw StudyError("parameter 'points' accepts at most " + std::to_string(kMaxViaPoints) +
                         " waypoints, received " + std::to_string(points.size()));
    }
    const std::size_t segments = points.size() - 1;

    ViaProfile profile;
    profile.points = points;
    profile.durations.assign(segments, 0.0);

    if (!durations.empty()) {
        if (durations.size() != segments) {
            throw StudyError("parameter 'durations' must hold one value per segment (" +
                             std::to_string(segments) + "), received " +
                             std::to_string(durations.size()));
        }
        for (std::size_t k = 0; k < segments; ++k) {
            if (!(durations[k] > 0.0)) {
                throw StudyError("segment duration " + std::to_string(k) +
                                 " must be positive, received " +
                                 json::number_to_string(durations[k]));
            }
            profile.durations[k] = durations[k];
        }
    } else {
        if (!(total_duration > 0.0)) {
            throw StudyError("parameter 'duration' must be positive when 'durations' is absent");
        }
        // Split the total time in proportion to the largest joint displacement
        // of each segment, so a long segment is not given the same time as a
        // short one; a zero-length segment still gets a share so it has a time.
        double weight_sum = 0.0;
        std::vector<double> weights(segments, 0.0);
        for (std::size_t k = 0; k < segments; ++k) {
            weights[k] = (points[k + 1] - points[k]).cwiseAbs().maxCoeff() + 1e-6;
            weight_sum += weights[k];
        }
        for (std::size_t k = 0; k < segments; ++k) {
            profile.durations[k] = total_duration * weights[k] / weight_sum;
        }
    }

    profile.start_times.assign(segments, 0.0);
    double clock = 0.0;
    for (std::size_t k = 0; k < segments; ++k) {
        profile.start_times[k] = clock;
        clock += profile.durations[k];
    }
    profile.total_duration = clock;

    // Interior velocities: the duration-weighted average of the two adjacent
    // segment slopes, zeroed where the direction reverses so the spline does
    // not overshoot through a corner. Interior accelerations: the slope change
    // over the mean segment time. Both are shared by the segments that meet at
    // the waypoint, which is what makes the result C2 rather than only C1.
    profile.velocities.assign(points.size(), Vector6::Zero());
    profile.accelerations.assign(points.size(), Vector6::Zero());
    for (std::size_t k = 1; k + 1 < points.size(); ++k) {
        const double T_prev = profile.durations[k - 1];
        const double T_next = profile.durations[k];
        for (Eigen::Index j = 0; j < 6; ++j) {
            const double slope_prev = (points[k](j) - points[k - 1](j)) / T_prev;
            const double slope_next = (points[k + 1](j) - points[k](j)) / T_next;
            const bool reverses = (slope_prev * slope_next) <= 0.0;
            profile.velocities[k](j) =
                reverses ? 0.0 : (T_prev * slope_next + T_next * slope_prev) / (T_prev + T_next);
            profile.accelerations[k](j) = 2.0 * (slope_next - slope_prev) / (T_prev + T_next);
        }
    }

    profile.segments.reserve(segments);
    for (std::size_t k = 0; k < segments; ++k) {
        // One quintic per joint per segment, matching the shared waypoint
        // position, velocity and acceleration at both of its ends.
        PolynomialProfile piece;
        piece.degree = 5;
        piece.duration = profile.durations[k];
        const double T = piece.duration;
        const double T2 = T * T;
        const double T3 = T2 * T;
        const double T4 = T3 * T;
        const double T5 = T4 * T;
        for (std::size_t j = 0; j < GP8_DOF; ++j) {
            const auto index = static_cast<Eigen::Index>(j);
            const double p0 = profile.points[k](index);
            const double p1 = profile.points[k + 1](index);
            const double v0 = profile.velocities[k](index);
            const double v1 = profile.velocities[k + 1](index);
            const double a0 = profile.accelerations[k](index);
            const double a1 = profile.accelerations[k + 1](index);
            const double h = p1 - p0;
            piece.c[j].fill(0.0);
            piece.c[j][0] = p0;
            piece.c[j][1] = v0;
            piece.c[j][2] = 0.5 * a0;
            piece.c[j][3] =
                (20.0 * h - (8.0 * v1 + 12.0 * v0) * T - (3.0 * a0 - a1) * T2) / (2.0 * T3);
            piece.c[j][4] =
                (-30.0 * h + (14.0 * v1 + 16.0 * v0) * T + (3.0 * a0 - 2.0 * a1) * T2) / (2.0 * T4);
            piece.c[j][5] = (12.0 * h - 6.0 * (v1 + v0) * T - (a0 - a1) * T2) / (2.0 * T5);
        }
        profile.segments.push_back(piece);
    }
    return profile;
}

Sample ViaProfile::at(double t) const noexcept {
    const double time = clamp_time(t, total_duration);
    std::size_t index = 0;
    while (index + 1 < segments.size() && time >= start_times[index + 1]) {
        ++index;
    }
    Sample s = segments[index].at(time - start_times[index]);
    s.t = time;
    return s;
}

// ---------------------------------------------------------------------------
// Sampling
// ---------------------------------------------------------------------------

namespace {

[[nodiscard]] int checked_count(int count) {
    if (count < 2) {
        throw StudyError("parameter 'samples' needs at least 2 samples, received " +
                         std::to_string(count));
    }
    if (count > kMaxSamples) {
        throw StudyError("parameter 'samples' accepts at most " + std::to_string(kMaxSamples) +
                         " samples, received " + std::to_string(count));
    }
    return count;
}

}  // namespace

Samples sample_polynomial(const PolynomialProfile& profile, int count, std::string kind) {
    const int n = checked_count(count);
    Samples out;
    out.duration = profile.duration;
    out.jerk_bounded = true;
    out.kind = std::move(kind);
    out.points.reserve(static_cast<std::size_t>(n));
    const double step = profile.duration / static_cast<double>(n - 1);
    for (int i = 0; i < n; ++i) {
        out.points.push_back(profile.at(static_cast<double>(i) * step));
    }
    return out;
}

Samples sample_lspb(const LspbProfile& profile, int count) {
    const int n = checked_count(count);
    Samples out;
    out.duration = profile.duration;
    out.jerk_bounded = false;  // the acceleration steps at both blend joins
    out.kind = "lspb";
    out.points.reserve(static_cast<std::size_t>(n));
    const double step = profile.duration / static_cast<double>(n - 1);
    for (int i = 0; i < n; ++i) {
        out.points.push_back(profile.at(static_cast<double>(i) * step));
    }
    return out;
}

Samples sample_via(const ViaProfile& profile, int count) {
    const int n = checked_count(count);
    Samples out;
    out.duration = profile.total_duration;
    out.jerk_bounded = true;
    out.kind = "via_points";
    out.points.reserve(static_cast<std::size_t>(n));
    const double step = profile.total_duration / static_cast<double>(n - 1);
    for (int i = 0; i < n; ++i) {
        out.points.push_back(profile.at(static_cast<double>(i) * step));
    }
    return out;
}

// ---------------------------------------------------------------------------
// The feasibility verdict
// ---------------------------------------------------------------------------

ConstraintVerdict check_constraints(const Samples& samples, double accel_limit, double jerk_limit,
                                    double cycle_time_limit) {
    if (samples.points.empty()) {
        throw StudyError("the profile has no samples to check");
    }
    if (!(accel_limit > 0.0)) {
        throw StudyError("parameter 'accel_limit' must be positive");
    }
    if (!(jerk_limit > 0.0)) {
        throw StudyError("parameter 'jerk_limit' must be positive");
    }
    if (!(cycle_time_limit > 0.0)) {
        throw StudyError("parameter 'cycle_time_limit' must be positive");
    }

    ConstraintVerdict verdict;
    verdict.cycle_time = samples.duration;
    verdict.cycle_time_ok = samples.duration <= cycle_time_limit;

    for (std::size_t j = 0; j < GP8_DOF; ++j) {
        const auto index = static_cast<Eigen::Index>(j);
        JointVerdict& jv = verdict.joints[j];
        jv.q_min = std::numeric_limits<double>::infinity();
        jv.q_max = -std::numeric_limits<double>::infinity();
        for (const Sample& s : samples.points) {
            jv.q_min = std::min(jv.q_min, s.q(index));
            jv.q_max = std::max(jv.q_max, s.q(index));
            jv.peak_speed = std::max(jv.peak_speed, std::abs(s.qd(index)));
            jv.peak_accel = std::max(jv.peak_accel, std::abs(s.qdd(index)));
            jv.peak_jerk = std::max(jv.peak_jerk, std::abs(s.qddd(index)));
        }

        jv.range_ok = jv.q_min >= joint_min(j) && jv.q_max <= joint_max(j);
        jv.velocity_ok = jv.peak_speed <= joint_max_velocity(j);
        jv.accel_ok = jv.peak_accel <= accel_limit;
        jv.jerk_ok = samples.jerk_bounded && jv.peak_jerk <= jerk_limit;

        const double range_ratio =
            std::max(jv.q_max > 0.0 ? jv.q_max / joint_max(j) : 0.0,
                     jv.q_min < 0.0 ? jv.q_min / joint_min(j) : 0.0);
        const double velocity_ratio = jv.peak_speed / joint_max_velocity(j);
        const double accel_ratio = jv.peak_accel / accel_limit;
        // An unbounded jerk is not expressed as an infinite ratio: that would
        // make every other number in the row meaningless. It is carried by the
        // jerk_ok flag and named in the binding constraint instead.
        const double jerk_ratio = samples.jerk_bounded ? jv.peak_jerk / jerk_limit : 0.0;

        jv.worst_ratio = range_ratio;
        jv.binding_constraint = "joint range";
        if (velocity_ratio > jv.worst_ratio) {
            jv.worst_ratio = velocity_ratio;
            jv.binding_constraint = "joint velocity";
        }
        if (accel_ratio > jv.worst_ratio) {
            jv.worst_ratio = accel_ratio;
            jv.binding_constraint = "joint acceleration";
        }
        if (jerk_ratio > jv.worst_ratio) {
            jv.worst_ratio = jerk_ratio;
            jv.binding_constraint = "joint jerk";
        }
        if (!samples.jerk_bounded) {
            jv.binding_constraint = "joint jerk (unbounded: the acceleration steps)";
        }

        verdict.range_ok = verdict.range_ok && jv.range_ok;
        verdict.velocity_ok = verdict.velocity_ok && jv.velocity_ok;
        verdict.accel_ok = verdict.accel_ok && jv.accel_ok;
        verdict.jerk_ok = verdict.jerk_ok && jv.jerk_ok;
        if (jv.worst_ratio > verdict.worst_ratio) {
            verdict.worst_ratio = jv.worst_ratio;
            verdict.worst_joint = j;
        }
    }

    verdict.feasible = verdict.range_ok && verdict.velocity_ok && verdict.accel_ok &&
                       verdict.jerk_ok && verdict.cycle_time_ok;
    verdict.verdict = verdict.feasible ? "FEASIBLE" : "INFEASIBLE";

    const double cycle_ratio = samples.duration / cycle_time_limit;
    const JointVerdict& worst = verdict.joints[verdict.worst_joint];
    if (verdict.feasible) {
        verdict.binding_constraint =
            "nothing binds: the tightest margin is " + worst.binding_constraint + " on " +
            joint_label(verdict.worst_joint) + " at " +
            json::number_to_string(100.0 * verdict.worst_ratio) +
            " percent of its limit, and the cycle time is at " +
            json::number_to_string(100.0 * cycle_ratio) + " percent of its budget";
    } else if (!samples.jerk_bounded && !verdict.jerk_ok) {
        verdict.binding_constraint =
            "joint jerk: this profile's acceleration steps at the phase joins, so its jerk is "
            "unbounded and no finite jerk limit can be met - that is the price of an LSPB and the "
            "reason the quintic exists";
    } else if (!verdict.cycle_time_ok && cycle_ratio >= verdict.worst_ratio) {
        verdict.binding_constraint =
            "cycle time: " + json::number_to_string(samples.duration) + " s against the " +
            json::number_to_string(cycle_time_limit) + " s budget, a factor of " +
            json::number_to_string(cycle_ratio) + " over";
    } else {
        verdict.binding_constraint =
            worst.binding_constraint + " on " + joint_label(verdict.worst_joint) + ": " +
            json::number_to_string(verdict.worst_ratio) + " times its limit";
    }
    if (cycle_ratio > verdict.worst_ratio) {
        verdict.worst_ratio = cycle_ratio;
    }
    return verdict;
}

}  // namespace traj

// ---------------------------------------------------------------------------
// Self-description
// ---------------------------------------------------------------------------

namespace {

[[nodiscard]] Vector6 default_start() noexcept { return Vector6::Zero(); }

[[nodiscard]] Vector6 default_goal() noexcept {
    Vector6 q = Vector6::Zero();
    q(1) = 0.5 * kPi;  // the course's worked example: joint 2 swings 0 to 90 degrees
    return q;
}

[[nodiscard]] ParamSpec param_start() {
    return ParamSpec::vec6("q_start", "Start configuration", "rad", -widest_joint_range(),
                           widest_joint_range(), default_start());
}

[[nodiscard]] ParamSpec param_goal() {
    return ParamSpec::vec6("q_end", "Goal configuration", "rad", -widest_joint_range(),
                           widest_joint_range(), default_goal());
}

[[nodiscard]] ParamSpec param_duration(double default_value) {
    return ParamSpec::scalar("duration", "Move duration", "s", kMinDuration, kMaxDuration,
                             default_value);
}

[[nodiscard]] ParamSpec param_joint() {
    return ParamSpec::integer("joint", "Joint to plot (1 = S ... 6 = T)", "", 1, 6, 2);
}

[[nodiscard]] ParamSpec param_samples(int default_value) {
    return ParamSpec::integer("samples", "Number of samples", "", 2, traj::kMaxSamples,
                              default_value);
}

}  // namespace

ModuleDescription TrajectoryProfilesModule::describe() const {
    ModuleDescription d;
    d.name = "trajectory_profiles";
    d.title = "Trajectory Planning: Polynomials, Blends, Via Points and Feasibility";
    d.course = CourseRef{3883, "M-407-01", "Robotics Modelling"};
    d.topics = {"Block 5 - Joint-space and Cartesian trajectory planning"};
    d.source = "cpp_solver/include/study/trajectory_profiles.hpp";
    d.summary =
        "Joint-space cubic, quintic, LSPB and via-point profiles for the GP8, the same blend "
        "applied to a straight Cartesian tool path through the existing IK, and one unambiguous "
        "pass/fail verdict for any of them against joint range, speed, acceleration, jerk and "
        "cycle time.";

    {
        OpSpec op;
        op.name = "cubic";
        op.title = "Cubic polynomial joint move";
        op.formula =
            "q(t) = a_0 + a_1 t + a_2 t^2 + a_3 t^3, \\quad a_2 = \\frac{3\\Delta q - (2v_0 + "
            "v_f) t_f}{t_f^2}, \\quad a_3 = \\frac{-2\\Delta q + (v_0 + v_f) t_f}{t_f^3}";
        op.explain =
            "Four boundary conditions - the position and the velocity at each end - fix the four "
            "coefficients of a cubic, so this is the cheapest polynomial that starts and stops at "
            "rest. Its signature is the acceleration trace: a step up at t = 0, a step down at "
            "t = t_f and a straight line in between, which means the jerk is a constant inside "
            "the move and an impulse at each end. For a rest-to-rest move the peak speed is "
            "always 3 dq / (2 t_f) at the midpoint - the 90 degree, 2 s example gives 67.5 deg/s, "
            "and that is the number to check the panel against by hand.";
        op.params = {
            param_start(), param_goal(), param_duration(2.0), param_joint(),
            ParamSpec::scalar("v_start", "Boundary velocity at the start", "rad/s",
                              -kBoundaryRateGuard, kBoundaryRateGuard, 0.0),
            ParamSpec::scalar("v_end", "Boundary velocity at the goal", "rad/s",
                              -kBoundaryRateGuard, kBoundaryRateGuard, 0.0),
            param_samples(200),
        };
        op.outputs = {
            OutputSpec::make("profiles", "series_set",
                             "Position, velocity, acceleration and jerk of the selected joint"),
            OutputSpec::make("coefficients", "table", "a_0 ... a_3 for every joint"),
            OutputSpec::make("boundary_conditions", "table", "Requested against achieved, at both "
                                                             "ends"),
            OutputSpec::make("peaks", "table", "Peak speed, acceleration and jerk per joint"),
            OutputSpec::make("peak_velocity", "scalar", "Peak speed of the selected joint",
                             "rad/s"),
            OutputSpec::make("peak_velocity_time", "scalar", "When that peak occurs", "s"),
            OutputSpec::make("travel", "scalar", "Displacement of the selected joint", "rad"),
            OutputSpec::make("integrated_travel", "scalar",
                             "Trapezoidal integral of the velocity trace, which must equal it",
                             "rad"),
            OutputSpec::make("joint_axis", "text", "Which axis is plotted"),
        };
        d.ops.push_back(std::move(op));
    }
    {
        OpSpec op;
        op.name = "quintic";
        op.title = "Quintic polynomial joint move";
        op.formula =
            "q(t) = \\sum_{k=0}^{5} a_k t^k, \\quad q, \\dot q, \\ddot q \\text{ matched at both "
            "ends}, \\quad \\dddot q(t) = 6a_3 + 24a_4 t + 60a_5 t^2";
        op.explain =
            "Six boundary conditions - position, velocity and acceleration at each end - need a "
            "fifth-order polynomial. The gain over the cubic is that the acceleration now starts "
            "and ends at zero, so the jerk is finite everywhere instead of an impulse at each "
            "end: the gearbox is not hit with a torque step and a payload does not slosh. The "
            "cost is a higher peak acceleration in the middle, 5.77 dq / t_f^2 against the "
            "cubic's 6 dq / t_f^2 at the ends, and a peak speed of 1.875 dq / t_f rather than "
            "1.5 dq / t_f. Overlay the two jerk traces for the same move and that trade is the "
            "whole answer to the exam question.";
        op.params = {
            param_start(), param_goal(), param_duration(2.0), param_joint(),
            ParamSpec::scalar("v_start", "Boundary velocity at the start", "rad/s",
                              -kBoundaryRateGuard, kBoundaryRateGuard, 0.0),
            ParamSpec::scalar("v_end", "Boundary velocity at the goal", "rad/s",
                              -kBoundaryRateGuard, kBoundaryRateGuard, 0.0),
            ParamSpec::scalar("a_start", "Boundary acceleration at the start", "rad/s^2",
                              -kBoundaryAccelGuard, kBoundaryAccelGuard, 0.0),
            ParamSpec::scalar("a_end", "Boundary acceleration at the goal", "rad/s^2",
                              -kBoundaryAccelGuard, kBoundaryAccelGuard, 0.0),
            param_samples(200),
        };
        op.outputs = {
            OutputSpec::make("profiles", "series_set",
                             "Position, velocity, acceleration and jerk of the selected joint"),
            OutputSpec::make("coefficients", "table", "a_0 ... a_5 for every joint"),
            OutputSpec::make("boundary_conditions", "table",
                             "Requested against achieved, at both ends"),
            OutputSpec::make("peaks", "table", "Peak speed, acceleration and jerk per joint"),
            OutputSpec::make("peak_velocity", "scalar", "Peak speed of the selected joint",
                             "rad/s"),
            OutputSpec::make("peak_velocity_time", "scalar", "When that peak occurs", "s"),
            OutputSpec::make("travel", "scalar", "Displacement of the selected joint", "rad"),
            OutputSpec::make("integrated_travel", "scalar",
                             "Trapezoidal integral of the velocity trace", "rad"),
            OutputSpec::make("joint_axis", "text", "Which axis is plotted"),
        };
        d.ops.push_back(std::move(op));
    }
    {
        OpSpec op;
        op.name = "lspb";
        op.title = "Linear segment with parabolic blends";
        op.formula =
            "\\dot s_c = \\frac{1}{t_f - t_b}, \\quad \\ddot s = \\frac{\\dot s_c}{t_b}, \\quad "
            "q(t) = q_0 + s(t)\\,\\Delta q, \\quad \\frac{\\Delta q}{t_f} < v_c \\le "
            "\\frac{2\\Delta q}{t_f}";
        op.explain =
            "The trapezoidal profile every industrial controller actually runs: accelerate at a "
            "constant rate, cruise, decelerate. The blend is applied to one path parameter s and "
            "every joint rides it, so the six axes stay synchronised and the three phases are the "
            "same three phases for all of them. Two numbers cannot both be free: pick the blend "
            "time and the cruise velocity follows, or pick the cruise velocity and the blend time "
            "follows. The feasible window is narrow and worth knowing - the plateau must be "
            "faster than the average speed (the blends are slower than it) and no faster than "
            "twice the average (at exactly twice, the plateau has vanished and the profile is "
            "triangular). Anything outside that window is rejected with the arithmetic that "
            "rejected it, not with a silently clamped number.";
        op.params = {
            param_start(), param_goal(), param_duration(2.0), param_joint(),
            ParamSpec::enumeration("tune", "Which of the two is the free parameter",
                                   {"blend_time", "cruise_velocity"}, "blend_time"),
            ParamSpec::scalar("blend_time", "Blend time t_b, used when tuning by blend time", "s",
                              0.001, kMaxDuration, 0.5),
            ParamSpec::scalar("cruise_velocity",
                              "Plateau speed of the selected joint, used when tuning by velocity",
                              "rad/s", 0.0, kBoundaryRateGuard, 1.0),
            param_samples(200),
        };
        op.outputs = {
            OutputSpec::make("profiles", "series_set",
                             "Position, velocity and acceleration of the selected joint"),
            OutputSpec::make("phases", "table", "The three phases with their exact times"),
            OutputSpec::make("blend_time", "scalar", "t_b", "s"),
            OutputSpec::make("cruise_velocity", "scalar", "Plateau speed of the selected joint",
                             "rad/s"),
            OutputSpec::make("blend_acceleration", "scalar",
                             "Constant acceleration of the selected joint during a blend",
                             "rad/s^2"),
            OutputSpec::make("velocity_continuity", "table",
                             "Velocity on each side of both blend joins"),
            OutputSpec::make("max_velocity_jump", "scalar",
                             "Worst velocity discontinuity at a join, which must be zero",
                             "rad/s"),
            OutputSpec::make("triangular", "bool", "True when the plateau has vanished"),
            OutputSpec::make("peaks", "table", "Peak speed and acceleration per joint"),
            OutputSpec::make("note", "text", "What this parameter combination produced"),
        };
        d.ops.push_back(std::move(op));
    }
    {
        OpSpec op;
        op.name = "via_points";
        op.title = "Via-point sequence with C2 continuity";
        op.formula =
            "q^{(k)}(t) = \\sum_{i=0}^{5} a_i^{(k)} t^i, \\qquad q^{(k)}(T_k) = q^{(k+1)}(0), "
            "\\quad \\dot q^{(k)}(T_k) = \\dot q^{(k+1)}(0), \\quad \\ddot q^{(k)}(T_k) = \\ddot "
            "q^{(k+1)}(0)";
        op.explain =
            "A real pick-move-place path is a list of waypoints, and the question is what happens "
            "at the joins. Here each segment is a quintic, and the velocity and acceleration at "
            "an interior waypoint are computed once and handed to both segments that meet there, "
            "which makes the result C2 by construction rather than by hope. The interior velocity "
            "is the duration-weighted average of the two segment slopes, and it is set to zero "
            "where the direction reverses so the arm does not overshoot through a corner. The "
            "continuity table is the proof: every jump in it is zero. Segment durations can be "
            "given explicitly or split from one total in proportion to how far each segment "
            "travels.";
        op.params = {
            ParamSpec::structured(
                "points", "Waypoints, one row of six joint angles each", "table", "rad",
                json::from_table({"S", "L", "U", "R", "B", "T"},
                                 std::vector<std::vector<double>>{
                                     {0.0, 0.0, 0.0, 0.0, 0.0, 0.0},
                                     {0.4, 0.6, -0.3, 0.0, 0.5, 0.0},
                                     {0.9, 0.3, 0.4, 0.2, 0.8, 0.0},
                                     {1.2, 0.0, 0.0, 0.0, 0.0, 0.0}})),
            param_duration(6.0),
            ParamSpec::structured("durations", "Per-segment durations, empty to split the total",
                                  "table", "s",
                                  json::from_table({"duration_s"},
                                                   std::vector<std::vector<double>>{})),
            param_joint(),
            param_samples(400),
        };
        op.outputs = {
            OutputSpec::make("profiles", "series_set",
                             "Position, velocity, acceleration and jerk of the selected joint"),
            OutputSpec::make("segments", "table", "Per-segment duration, start and end time"),
            OutputSpec::make("waypoint_states", "table",
                             "The velocity and acceleration used at each waypoint"),
            OutputSpec::make("continuity", "table",
                             "Position, velocity and acceleration jump at every join"),
            OutputSpec::make("max_velocity_jump", "scalar", "Worst velocity jump", "rad/s"),
            OutputSpec::make("max_acceleration_jump", "scalar", "Worst acceleration jump",
                             "rad/s^2"),
            OutputSpec::make("total_duration", "scalar", "Sum of the segment durations", "s"),
            OutputSpec::make("peaks", "table", "Peak speed, acceleration and jerk per joint"),
            OutputSpec::make("joint_axis", "text", "Which axis is plotted"),
        };
        d.ops.push_back(std::move(op));
    }
    {
        OpSpec op;
        op.name = "cartesian_lspb";
        op.title = "Straight Cartesian path with a trapezoidal blend";
        op.formula =
            "p(t) = p_0 + s(t)\\,(p_f - p_0), \\qquad q(t) = \\mathrm{IK}\\big(R_0, p(t)\\big), "
            "\\qquad s(t) \\text{ the LSPB path parameter}";
        op.explain =
            "The same trapezoidal blend, but now the thing being blended is the tool position, "
            "and the joint angles are whatever the inverse kinematics has to ask for to keep the "
            "tool on that line. This is the lesson of the block in one picture: a straight line "
            "in space is not a straight line in joint space. The tool path comes back for the 3D "
            "view, the six joint curves come back next to it, and the deviation of each joint "
            "curve from a straight joint-space interpolation is measured so the curvature is a "
            "number. The orientation is held at the seed pose's orientation, the IK is the "
            "repository's own Levenberg-Marquardt solver seeded with the previous sample, and "
            "any sample it fails to reach is counted rather than hidden.";
        op.params = {
            // The default line runs through the seed pose's own tool position,
            // (0.476, 0, 0.733) m, so the straight path is reachable with the
            // orientation held fixed and the IK has a solution at every sample.
            ParamSpec::vec3("p_start", "Start tool position", "m", -1.5, 1.5,
                            Eigen::Vector3d(0.476, -0.20, 0.733)),
            ParamSpec::vec3("p_end", "Goal tool position", "m", -1.5, 1.5,
                            Eigen::Vector3d(0.476, 0.20, 0.733)),
            param_duration(2.0),
            ParamSpec::scalar("blend_time", "Blend time t_b", "s", 0.001, kMaxDuration, 0.5),
            ParamSpec::vec6("q_seed", "Seed configuration, which also fixes the orientation", "rad",
                            -widest_joint_range(), widest_joint_range(), []() {
                                Vector6 q;
                                q << 0.0, 0.3, -0.4, 0.0, 0.6, 0.0;
                                return q;
                            }()),
            param_samples(60),
        };
        op.outputs = {
            OutputSpec::make("tool_path", "points", "Commanded tool positions", "m"),
            OutputSpec::make("joint_profiles", "series_set", "The six joint angles against time",
                             "rad"),
            OutputSpec::make("joint_rates", "series_set", "Joint rates by finite difference",
                             "rad/s"),
            OutputSpec::make("tool_speed", "series", "Tool speed along the path", "m/s"),
            OutputSpec::make("curvature", "table",
                             "How far each joint curve departs from a straight interpolation"),
            OutputSpec::make("max_joint_nonlinearity", "scalar",
                             "Worst departure from straight joint motion", "rad"),
            OutputSpec::make("straightness_error", "scalar",
                             "Worst deviation of the commanded path from the line", "m"),
            OutputSpec::make("ik_failures", "int", "Samples the IK did not reach"),
            OutputSpec::make("max_ik_position_error", "scalar",
                             "Worst achieved-against-commanded tool error", "m"),
            OutputSpec::make("path_length", "scalar", "Length of the commanded path", "m"),
            OutputSpec::make("note", "text", "What the joint curves show"),
        };
        d.ops.push_back(std::move(op));
    }
    {
        OpSpec op;
        op.name = "constraint_check";
        op.title = "Feasibility against actuator limits, jerk and cycle time";
        op.formula =
            "\\max_t |q_i| \\le q_i^{\\max}, \\quad \\max_t |\\dot q_i| \\le \\dot q_i^{\\max}, "
            "\\quad \\max_t |\\ddot q_i| \\le \\ddot q^{\\lim}, \\quad \\max_t |\\dddot q_i| \\le "
            "\\dddot q^{\\lim}, \\quad t_f \\le t_{\\text{cycle}}";
        op.explain =
            "This is the op that answers the learning outcome: is the trajectory you just planned "
            "actually runnable on this machine? Generate any of the profiles, then check every "
            "sample of it against the real limits. The joint ranges and the maximum speeds are "
            "the GP8's own, read from GP8_JOINT_LIMITS through study/gp8_model.hpp; the "
            "acceleration and jerk ceilings are parameters because Yaskawa does not publish them. "
            "The verdict is deliberately blunt: FEASIBLE or INFEASIBLE, one named binding "
            "constraint, and a per-joint table of worst-case values with the ratio to each limit, "
            "so 'the second axis asks for 2.8 times its rated speed' replaces 'it looks a bit "
            "fast'. An LSPB is reported as failing any finite jerk limit, because its "
            "acceleration genuinely steps - that is not a rounding artefact and it is not going "
            "to be smoothed over here.";
        op.params = {
            ParamSpec::enumeration("profile", "Which profile to generate and then judge",
                                   {"cubic", "quintic", "lspb"}, "cubic"),
            param_start(), param_goal(), param_duration(2.0),
            ParamSpec::scalar("blend_time", "Blend time, used when the profile is an LSPB", "s",
                              0.001, kMaxDuration, 0.5),
            ParamSpec::scalar("accel_limit", "Acceleration ceiling", "rad/s^2", 0.01, 500.0, 15.0),
            ParamSpec::scalar("jerk_limit", "Jerk ceiling", "rad/s^3", 0.01, 5000.0, 150.0),
            ParamSpec::scalar("cycle_time_limit", "Cycle-time budget", "s", kMinDuration,
                              kMaxDuration, 3.0),
            param_samples(400),
        };
        op.outputs = {
            OutputSpec::make("verdict", "text", "FEASIBLE or INFEASIBLE"),
            OutputSpec::make("feasible", "bool", "The same verdict as a flag"),
            OutputSpec::make("binding_constraint", "text", "The one constraint that decides it"),
            OutputSpec::make("per_joint", "table",
                             "Worst-case value against every limit, with a verdict per joint"),
            OutputSpec::make("constraints", "table", "One row per constraint with its verdict"),
            OutputSpec::make("worst_ratio", "scalar", "Worst value divided by its limit"),
            OutputSpec::make("cycle_time", "scalar", "Duration of the profile", "s"),
            OutputSpec::make("cycle_time_ok", "bool", "True when it fits the budget"),
            OutputSpec::make("jerk_bounded", "bool",
                             "False when the profile's acceleration steps"),
            OutputSpec::make("profiles", "series_set",
                             "Velocity and acceleration of the worst joint"),
        };
        d.ops.push_back(std::move(op));
    }

    return d;
}

// ---------------------------------------------------------------------------
// Ops
// ---------------------------------------------------------------------------

namespace {

[[nodiscard]] std::size_t read_joint_index(const json::Value& args) {
    return static_cast<std::size_t>(optional_int(args, "joint", 2, 1, 6) - 1);
}

// The configuration an op is about is required: the UI always sends it from
// the describe() default, so an absent one is a bug worth reporting rather
// than quietly planning a move nobody asked for.
[[nodiscard]] Vector6 require_pose(const json::Value& args, std::string_view key) {
    return require_vec6(args, key, -widest_joint_range(), widest_joint_range());
}

// The seed of the Cartesian op is a knob, so it keeps its default.
[[nodiscard]] Vector6 read_pose(const json::Value& args, std::string_view key,
                                const Vector6& fallback) {
    return optional_vec6(args, key, fallback, -widest_joint_range(), widest_joint_range());
}

[[nodiscard]] json::Value series_of(const traj::Samples& samples, std::size_t joint,
                                    bool with_jerk) {
    const auto index = static_cast<Eigen::Index>(joint);
    std::vector<double> t;
    std::vector<double> position;
    std::vector<double> velocity;
    std::vector<double> acceleration;
    std::vector<double> jerk;
    t.reserve(samples.points.size());
    position.reserve(samples.points.size());
    velocity.reserve(samples.points.size());
    acceleration.reserve(samples.points.size());
    jerk.reserve(samples.points.size());
    for (const traj::Sample& s : samples.points) {
        t.push_back(s.t);
        position.push_back(s.q(index));
        velocity.push_back(s.qd(index));
        acceleration.push_back(s.qdd(index));
        jerk.push_back(s.qddd(index));
    }
    json::Value out = json::Value::array();
    out.push_back(json::from_series(std::string("position ") + axis_name(joint) + " [rad]", t,
                                   position));
    out.push_back(json::from_series(std::string("velocity ") + axis_name(joint) + " [rad/s]", t,
                                   velocity));
    out.push_back(json::from_series(std::string("acceleration ") + axis_name(joint) + " [rad/s^2]",
                                   t, acceleration));
    if (with_jerk) {
        out.push_back(json::from_series(std::string("jerk ") + axis_name(joint) + " [rad/s^3]", t,
                                       jerk));
    }
    return out;
}

struct Peak {
    double value = 0.0;
    double time = 0.0;
};

[[nodiscard]] Peak peak_of(const traj::Samples& samples, std::size_t joint, int derivative) {
    const auto index = static_cast<Eigen::Index>(joint);
    Peak best;
    for (const traj::Sample& s : samples.points) {
        double value = 0.0;
        switch (derivative) {
            case 1: value = std::abs(s.qd(index)); break;
            case 2: value = std::abs(s.qdd(index)); break;
            default: value = std::abs(s.qddd(index)); break;
        }
        if (value > best.value) {
            best.value = value;
            best.time = s.t;
        }
    }
    return best;
}

[[nodiscard]] json::Value peaks_table(const traj::Samples& samples, bool with_jerk) {
    std::vector<std::vector<json::Value>> rows;
    rows.reserve(GP8_DOF);
    for (std::size_t j = 0; j < GP8_DOF; ++j) {
        const Peak speed = peak_of(samples, j, 1);
        const Peak accel = peak_of(samples, j, 2);
        const Peak jerk = peak_of(samples, j, 3);
        std::vector<json::Value> row = {
            json::Value(axis_name(j)),
            json::Value(speed.value),
            json::Value(speed.time),
            json::Value(accel.value),
            json::Value(accel.time),
        };
        if (with_jerk) {
            row.push_back(json::Value(jerk.value));
        }
        row.push_back(json::Value(joint_max_velocity(j)));
        rows.push_back(std::move(row));
    }
    std::vector<std::string> columns = {"axis", "peak_speed_rad_s", "at_s", "peak_accel_rad_s2",
                                        "at_s"};
    if (with_jerk) {
        columns.emplace_back("peak_jerk_rad_s3");
    }
    columns.emplace_back("speed_limit_rad_s");
    return json::from_table(columns, rows);
}

// Trapezoidal integral of one joint's velocity trace, which must come back as
// the displacement the profile actually travelled.
[[nodiscard]] double integrate_velocity(const traj::Samples& samples, std::size_t joint) {
    const auto index = static_cast<Eigen::Index>(joint);
    double total = 0.0;
    for (std::size_t i = 1; i < samples.points.size(); ++i) {
        const double dt = samples.points[i].t - samples.points[i - 1].t;
        total += 0.5 * dt * (samples.points[i].qd(index) + samples.points[i - 1].qd(index));
    }
    return total;
}

[[nodiscard]] json::Value coefficient_table(const traj::PolynomialProfile& profile) {
    std::vector<std::string> columns = {"axis"};
    for (std::size_t k = 0; k <= profile.degree; ++k) {
        columns.push_back("a" + std::to_string(k));
    }
    std::vector<std::vector<json::Value>> rows;
    rows.reserve(GP8_DOF);
    for (std::size_t j = 0; j < GP8_DOF; ++j) {
        std::vector<json::Value> row = {json::Value(axis_name(j))};
        for (std::size_t k = 0; k <= profile.degree; ++k) {
            row.emplace_back(profile.c[j][k]);
        }
        rows.push_back(std::move(row));
    }
    return json::from_table(columns, rows);
}

[[nodiscard]] json::Value boundary_table(const traj::PolynomialProfile& profile, std::size_t joint,
                                         const Vector6& q0, const Vector6& q1, double v0, double v1,
                                         double a0, double a1, bool with_accel) {
    const auto index = static_cast<Eigen::Index>(joint);
    const traj::Sample start = profile.at(0.0);
    const traj::Sample end = profile.at(profile.duration);
    std::vector<std::vector<json::Value>> rows = {
        {json::Value("position"), json::Value(q0(index)), json::Value(start.q(index)),
         json::Value(q1(index)), json::Value(end.q(index))},
        {json::Value("velocity"), json::Value(v0), json::Value(start.qd(index)), json::Value(v1),
         json::Value(end.qd(index))},
    };
    if (with_accel) {
        rows.push_back({json::Value("acceleration"), json::Value(a0),
                        json::Value(start.qdd(index)), json::Value(a1),
                        json::Value(end.qdd(index))});
    } else {
        rows.push_back({json::Value("acceleration"), json::Value("free"),
                        json::Value(start.qdd(index)), json::Value("free"),
                        json::Value(end.qdd(index))});
    }
    return json::from_table({"quantity", "requested_at_0", "achieved_at_0", "requested_at_tf",
                             "achieved_at_tf"},
                            rows);
}

[[nodiscard]] json::Value op_polynomial(const json::Value& args, bool quintic) {
    const Vector6 q0 = require_pose(args, "q_start");
    const Vector6 q1 = require_pose(args, "q_end");
    const double duration = optional_scalar(args, "duration", 2.0, kMinDuration, kMaxDuration);
    const std::size_t joint = read_joint_index(args);
    const double v0 = optional_scalar(args, "v_start", 0.0, -kBoundaryRateGuard,
                                      kBoundaryRateGuard);
    const double v1 = optional_scalar(args, "v_end", 0.0, -kBoundaryRateGuard, kBoundaryRateGuard);
    const double a0 = optional_scalar(args, "a_start", 0.0, -kBoundaryAccelGuard,
                                      kBoundaryAccelGuard);
    const double a1 = optional_scalar(args, "a_end", 0.0, -kBoundaryAccelGuard,
                                      kBoundaryAccelGuard);
    const int samples = optional_int(args, "samples", 200, 2, traj::kMaxSamples);

    const traj::PolynomialProfile profile =
        quintic ? traj::quintic_profile(q0, q1, duration, v0, v1, a0, a1)
                : traj::cubic_profile(q0, q1, duration, v0, v1);
    const traj::Samples sampled =
        traj::sample_polynomial(profile, samples, quintic ? "quintic" : "cubic");

    const auto index = static_cast<Eigen::Index>(joint);
    const Peak speed = peak_of(sampled, joint, 1);

    json::Value out = json::Value::object();
    out.set("profiles", series_of(sampled, joint, true));
    out.set("coefficients", coefficient_table(profile));
    out.set("boundary_conditions",
            boundary_table(profile, joint, q0, q1, v0, v1, a0, a1, quintic));
    out.set("peaks", peaks_table(sampled, true));
    out.set("peak_velocity", json::Value(speed.value));
    out.set("peak_velocity_time", json::Value(speed.time));
    out.set("travel", json::Value(q1(index) - q0(index)));
    out.set("integrated_travel", json::Value(integrate_velocity(sampled, joint)));
    out.set("duration", json::Value(duration));
    out.set("joint_axis", json::Value(joint_label(joint)));
    return out;
}

[[nodiscard]] json::Value op_lspb(const json::Value& args) {
    const Vector6 q0 = require_pose(args, "q_start");
    const Vector6 q1 = require_pose(args, "q_end");
    const double duration = optional_scalar(args, "duration", 2.0, kMinDuration, kMaxDuration);
    const std::size_t joint = read_joint_index(args);
    const std::string tune =
        optional_enum(args, "tune", "blend_time", {"blend_time", "cruise_velocity"});
    const int samples = optional_int(args, "samples", 200, 2, traj::kMaxSamples);

    traj::LspbProfile profile;
    if (tune == "blend_time") {
        const double blend = optional_scalar(args, "blend_time", 0.5, 0.0, kMaxDuration);
        profile = traj::lspb_from_blend_time(q0, q1, duration, blend);
    } else {
        const double cruise = optional_scalar(args, "cruise_velocity", 1.0, 0.0,
                                              kBoundaryRateGuard);
        profile = traj::lspb_from_cruise_velocity(q0, q1, duration, joint, cruise);
    }
    const traj::Samples sampled = traj::sample_lspb(profile, samples);

    const auto index = static_cast<Eigen::Index>(joint);
    const double delta = profile.delta(index);
    const double cruise_velocity = profile.s_dot_cruise * delta;
    const double blend_accel = profile.s_ddot * delta;
    const double t_b = profile.blend_time;
    const double t_f = profile.duration;

    std::vector<std::vector<json::Value>> phase_rows = {
        {json::Value("blend in (parabolic)"), json::Value(0.0), json::Value(t_b),
         json::Value(t_b), json::Value(blend_accel)},
        {json::Value("cruise (linear)"), json::Value(t_b), json::Value(t_f - t_b),
         json::Value(t_f - 2.0 * t_b), json::Value(0.0)},
        {json::Value("blend out (parabolic)"), json::Value(t_f - t_b), json::Value(t_f),
         json::Value(t_b), json::Value(-blend_accel)},
    };

    // Velocity on each side of both joins, evaluated analytically rather than
    // by reading the sampled series, so the continuity claim is exact.
    const double eps = std::min(1e-9, 0.25 * t_b);
    const double v_before_1 = profile.at(t_b - eps).qd(index);
    const double v_after_1 = profile.at(t_b + eps).qd(index);
    const double v_before_2 = profile.at(t_f - t_b - eps).qd(index);
    const double v_after_2 = profile.at(t_f - t_b + eps).qd(index);
    const double jump = std::max(std::abs(v_after_1 - v_before_1),
                                 std::abs(v_after_2 - v_before_2));
    std::vector<std::vector<json::Value>> continuity_rows = {
        {json::Value("end of blend in"), json::Value(t_b), json::Value(v_before_1),
         json::Value(v_after_1), json::Value(std::abs(v_after_1 - v_before_1))},
        {json::Value("start of blend out"), json::Value(t_f - t_b), json::Value(v_before_2),
         json::Value(v_after_2), json::Value(std::abs(v_after_2 - v_before_2))},
    };

    const bool triangular = std::abs(t_b - 0.5 * t_f) < 1e-9;
    std::string note;
    if (triangular) {
        note =
            "Degenerate triangular case: the blend takes up both halves of the move, so there is "
            "no cruise phase at all and the peak speed is exactly twice the average. No symmetric "
            "blend can cover this distance in this time any faster.";
    } else {
        note = "Trapezoidal: " + json::number_to_string(100.0 * (t_f - 2.0 * t_b) / t_f) +
               " percent of the move is spent cruising at " +
               json::number_to_string(cruise_velocity) + " rad/s on " + joint_label(joint) +
               ", and the blends need " + json::number_to_string(std::abs(blend_accel)) +
               " rad/s^2. Lower the cruise velocity towards the average speed and the blends grow "
               "until the profile becomes triangular.";
    }

    json::Value out = json::Value::object();
    out.set("profiles", series_of(sampled, joint, false));
    out.set("phases",
            json::from_table({"phase", "t_start_s", "t_end_s", "duration_s", "accel_rad_s2"},
                             phase_rows));
    out.set("blend_time", json::Value(t_b));
    out.set("cruise_velocity", json::Value(cruise_velocity));
    out.set("blend_acceleration", json::Value(blend_accel));
    out.set("velocity_continuity",
            json::from_table({"join", "t_s", "velocity_before", "velocity_after", "jump"},
                             continuity_rows));
    out.set("max_velocity_jump", json::Value(jump));
    out.set("triangular", json::Value(triangular));
    out.set("peaks", peaks_table(sampled, false));
    out.set("joint_axis", json::Value(joint_label(joint)));
    out.set("note", json::Value(note));
    return out;
}

[[nodiscard]] std::vector<Vector6> read_waypoints(const json::Value& args) {
    const json::Value& value = require_present(args, "points");
    const json::Value* rows = nullptr;
    if (value.is_array()) {
        rows = &value;
    } else if (value.is_object() && value["rows"].is_array()) {
        rows = &value["rows"];
    } else {
        throw StudyError("parameter 'points' must be a list of 6-element joint vectors or a table "
                         "with a 'rows' array");
    }
    if (rows->size() < 2) {
        throw StudyError("parameter 'points' needs at least 2 waypoints, received " +
                         std::to_string(rows->size()));
    }
    if (rows->size() > traj::kMaxViaPoints) {
        throw StudyError("parameter 'points' accepts at most " +
                         std::to_string(traj::kMaxViaPoints) + " waypoints, received " +
                         std::to_string(rows->size()));
    }
    std::vector<Vector6> out;
    out.reserve(rows->size());
    for (std::size_t i = 0; i < rows->size(); ++i) {
        const json::Value& row = (*rows)[i];
        if (!row.is_array() || row.size() != GP8_DOF) {
            throw StudyError("waypoint " + std::to_string(i) + " must hold 6 joint angles, "
                             "received " + std::to_string(row.size()));
        }
        Vector6 q;
        for (std::size_t j = 0; j < GP8_DOF; ++j) {
            const json::Value& cell = row[j];
            if (!cell.is_number() || !std::isfinite(cell.as_double())) {
                throw StudyError("waypoint " + std::to_string(i) + " element " +
                                 std::to_string(j) + " must be a finite number");
            }
            const double angle = cell.as_double();
            if (angle < joint_min(j) || angle > joint_max(j)) {
                throw StudyError("waypoint " + std::to_string(i) + " puts " + joint_label(j) +
                                 " at " + json::number_to_string(angle) +
                                 " rad, outside its range [" + json::number_to_string(joint_min(j)) +
                                 ", " + json::number_to_string(joint_max(j)) + "]");
            }
            q(static_cast<Eigen::Index>(j)) = angle;
        }
        out.push_back(q);
    }
    return out;
}

[[nodiscard]] std::vector<double> read_durations(const json::Value& args) {
    if (is_absent(args, "durations")) {
        return {};
    }
    const json::Value& value = args["durations"];
    // Accepted shapes, because the UI table and a hand-written request do not
    // agree on one: [1.5, 2.0], [[1.5], [2.0]] and {"rows": [[1.5], [2.0]]}.
    const json::Value* rows = nullptr;
    if (value.is_array()) {
        rows = &value;
    } else if (value.is_object() && value["rows"].is_array()) {
        rows = &value["rows"];
    } else {
        throw StudyError("parameter 'durations' must be an array of positive numbers or a table "
                         "with a 'rows' array");
    }
    std::vector<double> out;
    out.reserve(rows->size());
    for (std::size_t i = 0; i < rows->size(); ++i) {
        const json::Value& entry = (*rows)[i];
        const json::Value& cell = entry.is_array() ? entry[static_cast<std::size_t>(0)] : entry;
        if (!cell.is_number() || !std::isfinite(cell.as_double()) || cell.as_double() <= 0.0) {
            throw StudyError("parameter 'durations' element " + std::to_string(i) +
                             " must be a positive finite number");
        }
        out.push_back(cell.as_double());
    }
    return out;
}

[[nodiscard]] json::Value op_via_points(const json::Value& args) {
    const std::vector<Vector6> points = read_waypoints(args);
    const std::vector<double> durations = read_durations(args);
    const double duration = optional_scalar(args, "duration", 6.0, kMinDuration, kMaxDuration);
    const std::size_t joint = read_joint_index(args);
    const int samples = optional_int(args, "samples", 400, 2, traj::kMaxSamples);

    const traj::ViaProfile profile = traj::via_profile(points, durations, duration);
    const traj::Samples sampled = traj::sample_via(profile, samples);
    const auto index = static_cast<Eigen::Index>(joint);

    std::vector<std::vector<json::Value>> segment_rows;
    segment_rows.reserve(profile.segments.size());
    for (std::size_t k = 0; k < profile.segments.size(); ++k) {
        segment_rows.push_back({
            json::Value(static_cast<int>(k + 1)),
            json::Value(profile.durations[k]),
            json::Value(profile.start_times[k]),
            json::Value(profile.start_times[k] + profile.durations[k]),
            json::Value((profile.points[k + 1] - profile.points[k]).cwiseAbs().maxCoeff()),
        });
    }

    std::vector<std::vector<json::Value>> waypoint_rows;
    waypoint_rows.reserve(profile.points.size());
    for (std::size_t k = 0; k < profile.points.size(); ++k) {
        waypoint_rows.push_back({
            json::Value(static_cast<int>(k)),
            json::Value(profile.points[k](index)),
            json::Value(profile.velocities[k](index)),
            json::Value(profile.accelerations[k](index)),
        });
    }

    // The join jumps, evaluated on both sides of every interior waypoint.
    std::vector<std::vector<json::Value>> continuity_rows;
    double max_velocity_jump = 0.0;
    double max_acceleration_jump = 0.0;
    for (std::size_t k = 1; k < profile.segments.size(); ++k) {
        const double T_prev = profile.durations[k - 1];
        const traj::Sample before = profile.segments[k - 1].at(T_prev);
        const traj::Sample after = profile.segments[k].at(0.0);
        const double dq = (before.q - after.q).cwiseAbs().maxCoeff();
        const double dv = (before.qd - after.qd).cwiseAbs().maxCoeff();
        const double da = (before.qdd - after.qdd).cwiseAbs().maxCoeff();
        max_velocity_jump = std::max(max_velocity_jump, dv);
        max_acceleration_jump = std::max(max_acceleration_jump, da);
        continuity_rows.push_back({
            json::Value(static_cast<int>(k)),
            json::Value(profile.start_times[k]),
            json::Value(dq),
            json::Value(dv),
            json::Value(da),
        });
    }

    json::Value out = json::Value::object();
    out.set("profiles", series_of(sampled, joint, true));
    out.set("segments", json::from_table({"segment", "duration_s", "t_start_s", "t_end_s",
                                          "max_joint_travel_rad"},
                                         segment_rows));
    out.set("waypoint_states",
            json::from_table({"waypoint", "position_rad", "velocity_rad_s", "accel_rad_s2"},
                             waypoint_rows));
    out.set("continuity", json::from_table({"join", "t_s", "position_jump", "velocity_jump",
                                            "acceleration_jump"},
                                           continuity_rows));
    out.set("max_velocity_jump", json::Value(max_velocity_jump));
    out.set("max_acceleration_jump", json::Value(max_acceleration_jump));
    out.set("total_duration", json::Value(profile.total_duration));
    out.set("peaks", peaks_table(sampled, true));
    out.set("joint_axis", json::Value(joint_label(joint)));
    return out;
}

[[nodiscard]] json::Value op_cartesian_lspb(const json::Value& args) {
    const Eigen::Vector3d p_start = require_vec3(args, "p_start", -1.5, 1.5);
    const Eigen::Vector3d p_end = require_vec3(args, "p_end", -1.5, 1.5);
    const double duration = optional_scalar(args, "duration", 2.0, kMinDuration, kMaxDuration);
    const double blend = optional_scalar(args, "blend_time", 0.5, 0.0, kMaxDuration);
    Vector6 seed;
    seed << 0.0, 0.3, -0.4, 0.0, 0.6, 0.0;
    const Vector6 q_seed = read_pose(args, "q_seed", seed);
    const int samples = optional_int(args, "samples", 60, 2, 400);

    const double travel = (p_end - p_start).norm();
    if (travel < 1e-6) {
        throw StudyError("'p_start' and 'p_end' are " + json::number_to_string(travel) +
                         " m apart: there is no path to blend along");
    }

    // The blend is applied to the path parameter, exactly as in the joint-space
    // op: one LSPB on s in [0, 1] drives the straight line in space.
    Vector6 unit = Vector6::Zero();
    unit(0) = 1.0;
    const traj::LspbProfile s_profile =
        traj::lspb_from_blend_time(Vector6::Zero(), unit, duration, blend);

    const yaskawa::YaskawaKinematics kinematics;
    const Eigen::Isometry3d seed_pose = kinematics.forwardKinematics(q_seed);
    const Eigen::Matrix3d orientation = seed_pose.linear();

    const auto count = static_cast<std::size_t>(samples);
    std::vector<Eigen::Vector3d> path;
    std::vector<double> times;
    std::vector<double> speeds;
    std::vector<std::vector<double>> joint_tracks(GP8_DOF);
    path.reserve(count);
    times.reserve(count);
    speeds.reserve(count);
    for (auto& track : joint_tracks) {
        track.reserve(count);
    }

    int ik_failures = 0;
    double max_ik_error = 0.0;
    double straightness = 0.0;
    Vector6 q = q_seed;
    const double step = duration / static_cast<double>(samples - 1);
    const Eigen::Vector3d direction = (p_end - p_start) / travel;
    for (std::size_t i = 0; i < count; ++i) {
        const double t = static_cast<double>(i) * step;
        const traj::Sample s = s_profile.at(t);
        const double parameter = s.q(0);
        const Eigen::Vector3d target_position = p_start + parameter * (p_end - p_start);

        Eigen::Isometry3d target = Eigen::Isometry3d::Identity();
        target.linear() = orientation;
        target.translation() = target_position;

        Vector6 solution = q;
        const bool reached = kinematics.inverseKinematics(target, q, solution);
        if (!reached) {
            ++ik_failures;
        }
        q = solution;
        const Eigen::Vector3d achieved = kinematics.forwardKinematics(q).translation();
        max_ik_error = std::max(max_ik_error, (achieved - target_position).norm());

        const Eigen::Vector3d offset = target_position - p_start;
        straightness = std::max(straightness, (offset - offset.dot(direction) * direction).norm());

        times.push_back(t);
        speeds.push_back(std::abs(s.qd(0)) * travel);
        path.push_back(target_position);
        for (std::size_t j = 0; j < GP8_DOF; ++j) {
            joint_tracks[j].push_back(q(static_cast<Eigen::Index>(j)));
        }
    }

    // The point of the op: a straight line in space is curved in joint space.
    // Measure that as the worst departure of each joint curve from the straight
    // interpolation between its own end values.
    std::vector<std::vector<json::Value>> curvature_rows;
    curvature_rows.reserve(GP8_DOF);
    double worst_nonlinearity = 0.0;
    json::Value joint_series = json::Value::array();
    json::Value rate_series = json::Value::array();
    for (std::size_t j = 0; j < GP8_DOF; ++j) {
        const std::vector<double>& track = joint_tracks[j];
        const double first = track.front();
        const double last = track.back();
        double deviation = 0.0;
        for (std::size_t i = 0; i < count; ++i) {
            const double fraction = static_cast<double>(i) / static_cast<double>(count - 1);
            deviation = std::max(deviation, std::abs(track[i] - (first + fraction * (last - first))));
        }
        worst_nonlinearity = std::max(worst_nonlinearity, deviation);

        std::vector<double> rates(count, 0.0);
        double peak_rate = 0.0;
        for (std::size_t i = 1; i < count; ++i) {
            rates[i] = (track[i] - track[i - 1]) / step;
            peak_rate = std::max(peak_rate, std::abs(rates[i]));
        }
        if (count > 1) {
            rates[0] = rates[1];
        }
        curvature_rows.push_back({
            json::Value(axis_name(j)),
            json::Value(first),
            json::Value(last),
            json::Value(deviation),
            json::Value(peak_rate),
            json::Value(joint_max_velocity(j)),
            json::Value(peak_rate <= joint_max_velocity(j)),
        });
        joint_series.push_back(
            json::from_series(std::string("q ") + axis_name(j) + " [rad]", times, track));
        rate_series.push_back(
            json::from_series(std::string("qdot ") + axis_name(j) + " [rad/s]", times, rates));
    }

    std::string note =
        "The commanded tool path is straight to " + json::number_to_string(straightness) +
        " m, and the joint curves depart from a straight joint-space interpolation by up to " +
        json::number_to_string(worst_nonlinearity) +
        " rad: that difference is the whole reason Cartesian planning needs IK at every sample "
        "instead of interpolating the two end configurations. ";
    note += (ik_failures == 0)
                ? "Every sample was reached by the IK within its tolerance."
                : (std::to_string(ik_failures) +
                   " of the samples were not reached within the IK tolerance, which means the "
                   "path leaves the dexterous workspace somewhere along its length.");

    json::Value out = json::Value::object();
    out.set("tool_path", json::from_points(path));
    out.set("joint_profiles", std::move(joint_series));
    out.set("joint_rates", std::move(rate_series));
    out.set("tool_speed", json::from_series("tool speed [m/s]", times, speeds));
    out.set("curvature", json::from_table({"axis", "q_start_rad", "q_end_rad",
                                           "nonlinearity_rad", "peak_rate_rad_s",
                                           "rate_limit_rad_s", "ok"},
                                          curvature_rows));
    out.set("max_joint_nonlinearity", json::Value(worst_nonlinearity));
    out.set("straightness_error", json::Value(straightness));
    out.set("ik_failures", json::Value(ik_failures));
    out.set("max_ik_position_error", json::Value(max_ik_error));
    out.set("path_length", json::Value(travel));
    out.set("blend_time", json::Value(s_profile.blend_time));
    out.set("note", json::Value(note));
    return out;
}

[[nodiscard]] json::Value op_constraint_check(const json::Value& args) {
    const std::string kind = optional_enum(args, "profile", "cubic", {"cubic", "quintic", "lspb"});
    const Vector6 q0 = require_pose(args, "q_start");
    const Vector6 q1 = require_pose(args, "q_end");
    const double duration = optional_scalar(args, "duration", 2.0, kMinDuration, kMaxDuration);
    const double blend = optional_scalar(args, "blend_time", 0.5, 0.0, kMaxDuration);
    const double accel_limit = optional_scalar(args, "accel_limit", 15.0, 0.01, 500.0);
    const double jerk_limit = optional_scalar(args, "jerk_limit", 150.0, 0.01, 5000.0);
    const double cycle_time_limit =
        optional_scalar(args, "cycle_time_limit", 3.0, kMinDuration, kMaxDuration);
    const int samples = optional_int(args, "samples", 400, 2, traj::kMaxSamples);

    traj::Samples sampled;
    if (kind == "cubic") {
        sampled = traj::sample_polynomial(traj::cubic_profile(q0, q1, duration, 0.0, 0.0), samples,
                                          "cubic");
    } else if (kind == "quintic") {
        sampled = traj::sample_polynomial(
            traj::quintic_profile(q0, q1, duration, 0.0, 0.0, 0.0, 0.0), samples, "quintic");
    } else {
        sampled = traj::sample_lspb(traj::lspb_from_blend_time(q0, q1, duration, blend), samples);
    }

    const traj::ConstraintVerdict verdict =
        traj::check_constraints(sampled, accel_limit, jerk_limit, cycle_time_limit);

    std::vector<std::vector<json::Value>> joint_rows;
    joint_rows.reserve(GP8_DOF);
    for (std::size_t j = 0; j < GP8_DOF; ++j) {
        const traj::JointVerdict& jv = verdict.joints[j];
        joint_rows.push_back({
            json::Value(axis_name(j)),
            json::Value(jv.q_min),
            json::Value(jv.q_max),
            json::Value(jv.range_ok),
            json::Value(jv.peak_speed),
            json::Value(joint_max_velocity(j)),
            json::Value(jv.velocity_ok),
            json::Value(jv.peak_accel),
            json::Value(jv.accel_ok),
            json::Value(sampled.jerk_bounded ? json::Value(jv.peak_jerk)
                                             : json::Value("unbounded")),
            json::Value(jv.jerk_ok),
            json::Value(jv.worst_ratio),
            json::Value(jv.binding_constraint),
        });
    }

    std::vector<std::vector<json::Value>> constraint_rows = {
        {json::Value("joint range"), json::Value("GP8_JOINT_LIMITS min/max"),
         json::Value(verdict.range_ok ? "PASS" : "FAIL")},
        {json::Value("joint velocity"), json::Value("GP8_JOINT_LIMITS max_vel"),
         json::Value(verdict.velocity_ok ? "PASS" : "FAIL")},
        {json::Value("joint acceleration"), json::Value(json::number_to_string(accel_limit) +
                                                        " rad/s^2 (parameter)"),
         json::Value(verdict.accel_ok ? "PASS" : "FAIL")},
        {json::Value("joint jerk"),
         json::Value(json::number_to_string(jerk_limit) + " rad/s^3 (parameter)"),
         json::Value(verdict.jerk_ok ? "PASS" : "FAIL")},
        {json::Value("cycle time"),
         json::Value(json::number_to_string(cycle_time_limit) + " s (parameter)"),
         json::Value(verdict.cycle_time_ok ? "PASS" : "FAIL")},
    };

    json::Value out = json::Value::object();
    out.set("verdict", json::Value(verdict.verdict));
    out.set("feasible", json::Value(verdict.feasible));
    out.set("binding_constraint", json::Value(verdict.binding_constraint));
    out.set("per_joint",
            json::from_table({"axis", "q_min_rad", "q_max_rad", "range_ok", "peak_speed_rad_s",
                              "speed_limit_rad_s", "velocity_ok", "peak_accel_rad_s2", "accel_ok",
                              "peak_jerk_rad_s3", "jerk_ok", "worst_ratio", "binding"},
                             joint_rows));
    out.set("constraints", json::from_table({"constraint", "limit", "verdict"}, constraint_rows));
    out.set("worst_ratio", json::Value(verdict.worst_ratio));
    out.set("worst_joint", json::Value(joint_label(verdict.worst_joint)));
    out.set("cycle_time", json::Value(verdict.cycle_time));
    out.set("cycle_time_ok", json::Value(verdict.cycle_time_ok));
    out.set("jerk_bounded", json::Value(sampled.jerk_bounded));
    out.set("profile", json::Value(kind));
    out.set("profiles", series_of(sampled, verdict.worst_joint, sampled.jerk_bounded));
    return out;
}

}  // namespace

json::Value TrajectoryProfilesModule::invoke(std::string_view op, const json::Value& args) const {
    if (op == "cubic") {
        return op_polynomial(args, false);
    }
    if (op == "quintic") {
        return op_polynomial(args, true);
    }
    if (op == "lspb") {
        return op_lspb(args);
    }
    if (op == "via_points") {
        return op_via_points(args);
    }
    if (op == "cartesian_lspb") {
        return op_cartesian_lspb(args);
    }
    if (op == "constraint_check") {
        return op_constraint_check(args);
    }
    unknown_op(name(), op);
}

}  // namespace yaskawa::study
