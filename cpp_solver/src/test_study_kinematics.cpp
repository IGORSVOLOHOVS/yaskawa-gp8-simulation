// Test suite for the kinematics half of the study layer: the dh_kinematics
// and jacobian_statics modules. Plain asserts in the style of
// src/test_study_modules.cpp - there is no gtest in this project and there
// will not be one.
//
// Every check asserts a property, never a printed string:
//   - the DH chain and the repository's own forward kinematics agree to 1e-9
//   - every closed-form IK branch round-trips through FK to 1e-6
//   - an unreachable target comes back as a failure, not as a wrong answer
//   - the analytic Jacobian matches a central difference of FK to 1e-6
//     (the single most valuable check in this file: it is what proves the
//     frame assignment, and nothing else here would catch a wrong one)
//   - J^T applied twice is self-consistent
//   - singular values are non-negative and ordered
//   - det J really does collapse at a configuration built to be singular

#include "study/dh_kinematics.hpp"
#include "study/gp8_model.hpp"
#include "study/jacobian_statics.hpp"
#include "study/json.hpp"
#include "study/module_registry.hpp"
#include "study/study_module.hpp"
#include "yaskawa_kinematics.hpp"

#include <Eigen/Dense>
#include <Eigen/Geometry>

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <iostream>
#include <memory>
#include <numbers>
#include <random>
#include <string>
#include <vector>

using yaskawa::study::json::Value;
namespace json = yaskawa::study::json;
namespace study = yaskawa::study;
namespace dhk = yaskawa::study::dhk;
namespace jac = yaskawa::study::jac;

namespace {

constexpr double kPi = std::numbers::pi;

int g_checks = 0;
int g_failures = 0;
std::vector<std::string> g_failed_names;

void check(bool ok, const std::string& what) {
    ++g_checks;
    if (ok) {
        std::cout << "[PASS] " << what << "\n";
    } else {
        ++g_failures;
        g_failed_names.push_back(what);
        std::cout << "[FAIL] " << what << "\n";
    }
}

void check_below(double actual, double bound, const std::string& what) {
    const bool ok = std::isfinite(actual) && actual <= bound;
    if (!ok) {
        std::cout << "       value " << actual << " is not within " << bound << "\n";
    }
    check(ok, what);
}

using JointVector = dhk::JointVector;

// A deterministic generator: a failure found here is a failure anyone can
// reproduce, which a clock-seeded one would not be.
class JointSampler {
public:
    explicit JointSampler(unsigned seed) : generator_(seed) {}

    [[nodiscard]] JointVector sample() {
        JointVector q;
        for (std::size_t i = 0; i < study::GP8_DOF; ++i) {
            const double low = study::joint_min(i);
            const double high = study::joint_max(i);
            q(static_cast<Eigen::Index>(i)) = low + unit_(generator_) * (high - low);
        }
        return q;
    }

    // The same, but with every joint inside (-pi, pi], so that an IK solution
    // expressed in the principal branch can be compared against the sample
    // that produced it. Axes 3, 4 and 6 have ranges wider than pi.
    [[nodiscard]] JointVector sample_principal() {
        JointVector q;
        for (std::size_t i = 0; i < study::GP8_DOF; ++i) {
            const double low = std::max(study::joint_min(i), -kPi + 1e-6);
            const double high = std::min(study::joint_max(i), kPi - 1e-6);
            q(static_cast<Eigen::Index>(i)) = low + unit_(generator_) * (high - low);
        }
        return q;
    }

private:
    std::mt19937 generator_;
    std::uniform_real_distribution<double> unit_{0.0, 1.0};
};

// ---------------------------------------------------------------------------
// Forward kinematics: the DH table against the repository's own chain
// ---------------------------------------------------------------------------

void test_forward_kinematics() {
    std::cout << "\n--- dh_kinematics: forward kinematics ---\n";

    const yaskawa::YaskawaKinematics engine;
    JointSampler sampler(12345U);

    double worst_position = 0.0;
    double worst_orientation = 0.0;
    for (int trial = 0; trial < 500; ++trial) {
        const JointVector q = sampler.sample();
        const Eigen::Isometry3d mine = study::forward_kinematics_dh(q);
        const Eigen::Isometry3d theirs = engine.forwardKinematics(q);
        worst_position =
            std::max(worst_position, (mine.translation() - theirs.translation()).norm());
        worst_orientation =
            std::max(worst_orientation, dhk::orientation_error(Eigen::Matrix3d(mine.linear()),
                                                               Eigen::Matrix3d(theirs.linear())));
    }
    std::cout << "       worst position disagreement " << worst_position << " m, worst orientation "
              << worst_orientation << " rad over 500 random configurations\n";
    check_below(worst_position, 1e-9,
                "the DH chain and YaskawaKinematics::forwardKinematics agree on position to 1e-9");
    check_below(worst_orientation, 1e-9,
                "the DH chain and YaskawaKinematics::forwardKinematics agree on orientation to "
                "1e-9");

    // The chain product really is the product of the factors it reports.
    const JointVector q = sampler.sample();
    const std::array<Eigen::Isometry3d, study::GP8_DOF> A = dhk::joint_transforms(q);
    Eigen::Isometry3d product = Eigen::Isometry3d::Identity();
    for (const auto& factor : A) {
        product = product * factor;
    }
    check_below((product.matrix() - study::dh_chain(q).matrix()).norm(), 1e-12,
                "A_1 ... A_6 reproduces the DH chain exactly");

    // The wrist is spherical with d6 = 0: joints 4, 5 and 6 cannot move the
    // flange. The whole closed-form inverse rests on this.
    JointVector moved = q;
    moved(3) += 0.7;
    moved(4) -= 0.4;
    moved(5) += 1.1;
    check_below((study::forward_kinematics_dh(moved).translation() -
                 study::forward_kinematics_dh(q).translation())
                    .norm(),
                1e-12, "the wrist joints rotate the tool without moving it");
    check_below((dhk::wrist_center(q) - study::forward_kinematics_dh(q).translation()).norm(),
                1e-12, "the wrist centre coincides with the flange origin");
}

// ---------------------------------------------------------------------------
// Inverse kinematics
// ---------------------------------------------------------------------------

void test_inverse_kinematics() {
    std::cout << "\n--- dh_kinematics: inverse kinematics ---\n";

    JointSampler sampler(99991U);

    double worst_position = 0.0;
    double worst_orientation = 0.0;
    double worst_recovery = 0.0;
    std::size_t total_branches = 0;
    std::size_t poses = 0;
    bool every_pose_solved = true;
    bool every_pose_recovered = true;

    for (int trial = 0; trial < 60; ++trial) {
        const JointVector q = sampler.sample_principal();
        const Eigen::Isometry3d target = study::forward_kinematics_dh(q);
        const dhk::IkGeometric solution = dhk::solve_ik_geometric(target);

        if (!solution.reachable) {
            every_pose_solved = false;
            continue;
        }
        ++poses;
        total_branches += solution.branches.size();

        double recovery = 1e9;
        for (const auto& branch : solution.branches) {
            worst_position = std::max(worst_position, branch.position_error);
            worst_orientation = std::max(worst_orientation, branch.orientation_error);
            recovery = std::min(recovery, (branch.q - q).cwiseAbs().maxCoeff());
        }
        worst_recovery = std::max(worst_recovery, recovery);
        if (recovery > 1e-6) {
            every_pose_recovered = false;
        }
    }

    std::cout << "       " << poses << " poses solved, " << total_branches
              << " branches in total (" << (static_cast<double>(total_branches) /
                                            static_cast<double>(std::max<std::size_t>(poses, 1)))
              << " per pose), worst branch position error " << worst_position
              << " m, worst orientation error " << worst_orientation << " rad\n";

    check(every_pose_solved, "every pose generated by forward kinematics is solved by the closed "
                             "form");
    check_below(worst_position, 1e-6, "every geometric IK branch round-trips through FK to 1e-6 in "
                                      "position");
    check_below(worst_orientation, 1e-6,
                "every geometric IK branch round-trips through FK to 1e-6 in orientation");
    check(every_pose_recovered && worst_recovery <= 1e-6,
          "the closed form recovers the exact joint vector that generated each pose");
    check(total_branches >= 4 * poses,
          "a reachable pose yields at least four branches (shoulder x elbow x wrist)");

    // A reachable pose well inside the envelope should expose all eight
    // branches of a 6R arm with a spherical wrist.
    JointVector mid;
    mid << 0.2, 0.4, 0.5, 0.3, 0.7, -0.2;
    const dhk::IkGeometric rich = dhk::solve_ik_geometric(study::forward_kinematics_dh(mid));
    std::cout << "       a typical interior pose yields " << rich.branches.size()
              << " branches, " << rich.within_limits << " of them inside the joint limits\n";
    check(rich.branches.size() == 8,
          "an interior pose yields the full eight closed-form branches");
    check(rich.within_limits >= 1 && rich.within_limits <= rich.branches.size(),
          "at least one branch, and never more than all of them, lies inside the joint limits");

    // Unreachable: two metres away is outside the 0.727 m envelope.
    Eigen::Isometry3d far = Eigen::Isometry3d::Identity();
    far.translation() = Eigen::Vector3d(2.0, 0.0, 0.4);
    const dhk::IkGeometric unreachable = dhk::solve_ik_geometric(far);
    check(!unreachable.reachable && unreachable.branches.empty(),
          "an out-of-reach target returns no solution rather than a wrong one");
    check(unreachable.required_cosine > unreachable.available_amplitude,
          "the reachability test names the reason: |K| exceeds the available amplitude");

    // The envelope boundary is sharp, and it sits where the DH table says:
    // u_max = sqrt(a2^2 + a3^2 + d4^2 + |(M, N)|) + a1 = 0.72734 m at shoulder
    // height. One centimetre either side of it must flip the answer.
    Eigen::Isometry3d at_boundary = Eigen::Isometry3d::Identity();
    at_boundary.translation() = Eigen::Vector3d(0.720, 0.0, 0.330);
    Eigen::Isometry3d past_boundary = Eigen::Isometry3d::Identity();
    past_boundary.translation() = Eigen::Vector3d(0.730, 0.0, 0.330);
    const dhk::IkGeometric inside = dhk::solve_ik_geometric(at_boundary);
    const dhk::IkGeometric outside = dhk::solve_ik_geometric(past_boundary);
    check(inside.reachable && !inside.branches.empty(),
          "a target 7 mm inside the envelope is solved");
    check(!outside.reachable && outside.branches.empty(),
          "a target 3 mm outside the envelope returns no solution rather than the nearest pose");
}

// ---------------------------------------------------------------------------
// The Jacobian
// ---------------------------------------------------------------------------

void test_jacobian() {
    std::cout << "\n--- jacobian_statics: analytic against numeric ---\n";

    JointSampler sampler(777U);

    double worst = 0.0;
    double worst_linear = 0.0;
    double worst_angular = 0.0;
    for (int trial = 0; trial < 25; ++trial) {
        const JointVector q = sampler.sample();
        const jac::Jacobian analytic = jac::geometric_jacobian(q);
        const jac::Jacobian numeric = jac::numeric_jacobian(q, 1e-6);
        const jac::Jacobian difference = analytic - numeric;
        worst = std::max(worst, difference.cwiseAbs().maxCoeff());
        worst_linear = std::max(worst_linear, difference.block<3, 6>(0, 0).cwiseAbs().maxCoeff());
        worst_angular = std::max(worst_angular, difference.block<3, 6>(3, 0).cwiseAbs().maxCoeff());
    }
    std::cout << "       worst element-wise disagreement " << worst << " (linear " << worst_linear
              << ", angular " << worst_angular << ") over 25 random configurations\n";
    check_below(worst, 1e-6,
                "the analytic Jacobian matches a central difference of the forward kinematics to "
                "1e-6 at 25 random configurations");

    // The same matrix, reached through the repository's forward-difference
    // Jacobian. It is only first-order accurate, so 1e-4 is the honest bound.
    const yaskawa::YaskawaKinematics engine;
    double worst_engine = 0.0;
    JointSampler engine_sampler(31337U);
    for (int trial = 0; trial < 25; ++trial) {
        const JointVector q = engine_sampler.sample();
        worst_engine = std::max(
            worst_engine,
            (jac::geometric_jacobian(q) - engine.computeJacobian(q)).cwiseAbs().maxCoeff());
    }
    std::cout << "       worst disagreement with YaskawaKinematics::computeJacobian "
              << worst_engine << "\n";
    check_below(worst_engine, 1e-4,
                "the analytic Jacobian matches the engine's forward-difference Jacobian to the "
                "accuracy that difference can carry");

    // Velocity propagation: the recursion and the matrix product are the same
    // statement, so their difference is machine noise.
    JointSampler rate_sampler(4242U);
    double worst_recursion = 0.0;
    for (int trial = 0; trial < 25; ++trial) {
        const JointVector q = rate_sampler.sample();
        const JointVector qdot = rate_sampler.sample() * 0.3;
        const jac::Jacobian J = jac::geometric_jacobian(q);
        const std::array<Eigen::Isometry3d, study::GP8_DOF> frames = study::link_frames(q);
        Eigen::Vector3d omega = Eigen::Vector3d::Zero();
        Eigen::Vector3d v = Eigen::Vector3d::Zero();
        Eigen::Vector3d z_prev(0.0, 0.0, 1.0);
        Eigen::Vector3d o_prev = Eigen::Vector3d::Zero();
        for (std::size_t i = 0; i < study::GP8_DOF; ++i) {
            const Eigen::Vector3d o_i = frames[i].translation();
            omega += z_prev * qdot(static_cast<Eigen::Index>(i));
            v += omega.cross(o_i - o_prev);
            z_prev = frames[i].linear().col(2);
            o_prev = o_i;
        }
        jac::Twist propagated;
        propagated.head<3>() = v;
        propagated.tail<3>() = omega;
        worst_recursion = std::max(worst_recursion, (propagated - J * qdot).norm());
    }
    check_below(worst_recursion, 1e-12,
                "the link-by-link velocity recursion equals J qdot to machine precision");
}

void test_statics_and_conditioning() {
    std::cout << "\n--- jacobian_statics: duality and conditioning ---\n";

    JointSampler sampler(2026U);

    double worst_duality = 0.0;
    double worst_work = 0.0;
    bool ordered = true;
    bool non_negative = true;
    for (int trial = 0; trial < 25; ++trial) {
        const JointVector q = sampler.sample();
        const jac::Jacobian J = jac::geometric_jacobian(q);

        // J^T applied twice: tau = J^T F, then the wrench that tau implies,
        // then J^T of that must land back on tau.
        jac::Wrench wrench;
        wrench << 10.0, -4.0, 7.0, 0.5, -1.5, 2.0;
        const JointVector tau = J.transpose() * wrench;
        const Eigen::ColPivHouseholderQR<jac::Jacobian> qr(J.transpose());
        const jac::Wrench recovered = qr.solve(tau);
        worst_duality =
            std::max(worst_duality, (J.transpose() * recovered - tau).cwiseAbs().maxCoeff());

        // Virtual work is the same number in either space.
        const JointVector qdot = sampler.sample() * 0.1;
        worst_work = std::max(worst_work, std::abs(wrench.dot(J * qdot) - tau.dot(qdot)));

        const Eigen::Matrix<double, 6, 1> sv = jac::singular_values(J);
        for (Eigen::Index i = 0; i < 6; ++i) {
            non_negative = non_negative && sv(i) >= 0.0;
            if (i > 0) {
                ordered = ordered && sv(i - 1) >= sv(i);
            }
        }
    }
    std::cout << "       worst J^T round-trip error " << worst_duality
              << ", worst virtual-work mismatch " << worst_work << "\n";
    check_below(worst_duality, 1e-9, "J^T applied twice is self-consistent");
    check_below(worst_work, 1e-9, "F . (J qdot) equals tau . qdot, which is why tau = J^T F");
    check(non_negative, "singular values are non-negative");
    check(ordered, "singular values come back in descending order");

    // Built to be singular: q5 = 0 aligns axis 4 with axis 6, so the wrist
    // loses a rotational degree of freedom.
    JointVector wrist_singular;
    wrist_singular << 0.3, 0.2, 0.4, 0.5, 0.0, 0.1;
    const jac::Jacobian J_singular = jac::geometric_jacobian(wrist_singular);
    const Eigen::Matrix<double, 6, 1> sv_singular = jac::singular_values(J_singular);
    std::cout << "       at q5 = 0: det J = " << J_singular.determinant() << ", sigma_min = "
              << sv_singular(5) << ", manipulability = " << jac::manipulability(J_singular) << "\n";
    check_below(std::abs(J_singular.determinant()), 1e-12,
                "det J collapses to zero at the wrist singularity q5 = 0");
    check_below(sv_singular(5), 1e-12,
                "the smallest singular value collapses to zero at the wrist singularity");
    check_below(jac::manipulability(J_singular), 1e-12,
                "the manipulability measure vanishes at the wrist singularity");
    check(dhk::classify_singularity(wrist_singular, 1e-3).type == "wrist",
          "the configuration is named as a wrist singularity, not merely reported as small");

    // Built to be singular the other way: the elbow measure a3 sin(q3) +
    // d4 cos(q3) vanishes when the arm is stretched.
    const double a3 = study::GP8_DH[2].a;
    const double d4 = study::GP8_DH[3].d;
    JointVector elbow_singular;
    elbow_singular << 0.1, 0.3, std::atan2(-d4, a3) + kPi, 0.2, 0.9, -0.3;
    check_below(std::abs(a3 * std::sin(elbow_singular(2)) + d4 * std::cos(elbow_singular(2))),
                1e-12, "the constructed elbow configuration zeroes a3 sin(q3) + d4 cos(q3)");
    check_below(std::abs(jac::geometric_jacobian(elbow_singular).determinant()), 1e-12,
                "det J collapses to zero at the stretched-elbow singularity");
    check(dhk::classify_singularity(elbow_singular, 1e-3).type == "elbow",
          "the stretched arm is named as an elbow singularity");

    // A generic configuration is not singular, otherwise the checks above
    // would pass for the wrong reason.
    JointVector generic;
    generic << 0.3, 0.4, 0.5, 0.6, 0.8, 0.2;
    const jac::Jacobian J_generic = jac::geometric_jacobian(generic);
    check(std::abs(J_generic.determinant()) > 1e-4,
          "a generic configuration is comfortably non-singular");
    check(jac::singular_values(J_generic)(5) > 1e-3,
          "a generic configuration keeps its smallest singular value away from zero");

    // Damped least squares: zero damping is exact away from a singularity, and
    // damping costs residual rather than hiding it.
    jac::Twist twist;
    twist << 0.1, -0.05, 0.08, 0.2, -0.1, 0.15;
    const jac::DampedSolution exact = jac::damped_least_squares(J_generic, twist, 0.0);
    const jac::DampedSolution damped = jac::damped_least_squares(J_generic, twist, 0.1);
    check_below(exact.residual_norm, 1e-9,
                "undamped least squares reproduces the commanded twist exactly off a singularity");
    check(damped.residual_norm > exact.residual_norm,
          "damping costs tracking accuracy, and the cost is reported");
    check(damped.qdot.norm() < exact.qdot.norm(),
          "damping buys that accuracy back as smaller joint rates");

    // The velocity ellipsoid: radii ordered, and its volume agrees with the
    // product of the radii.
    const jac::VelocityEllipsoid ellipsoid = jac::velocity_ellipsoid(J_generic);
    check(ellipsoid.radii(0) >= ellipsoid.radii(1) && ellipsoid.radii(1) >= ellipsoid.radii(2) &&
              ellipsoid.radii(2) >= 0.0,
          "the velocity ellipsoid radii are non-negative and ordered");
    bool unit_axes = true;
    for (std::size_t i = 0; i < 3; ++i) {
        unit_axes = unit_axes && std::abs(ellipsoid.axes[i].norm() - 1.0) < 1e-12;
    }
    check(unit_axes, "the velocity ellipsoid axes are unit vectors");
    check_below(std::abs(ellipsoid.volume - (4.0 / 3.0) * kPi * ellipsoid.radii(0) *
                                                ellipsoid.radii(1) * ellipsoid.radii(2)),
                1e-12, "the reported ellipsoid volume is the product of its own radii");
}

// ---------------------------------------------------------------------------
// The module surface: describe() drives invoke(), and the registry wraps both
// ---------------------------------------------------------------------------

[[nodiscard]] Value args_from_defaults(const study::OpSpec& op) {
    Value args = Value::object();
    for (const auto& param : op.params) {
        args.set(param.name, param.default_value);
    }
    return args;
}

void exercise_module(const study::StudyModule& module) {
    const study::ModuleDescription description = module.describe();
    check(!description.name.empty() && !description.summary.empty() && !description.ops.empty(),
          std::string(module.name()) + " describes itself with a name, a summary and ops");
    check(description.source.find("cpp_solver/include/study/") == 0,
          std::string(module.name()) + " points at its own header");
    check(description.course.id == 3883,
          std::string(module.name()) + " is attributed to course 3883");

    for (const auto& op : description.ops) {
        bool ok = true;
        std::string failure;
        for (const auto& param : op.params) {
            if (param.type == "enum" && param.options.empty()) {
                ok = false;
                failure = "enum parameter " + param.name + " carries no options";
            }
            if (param.default_value.is_null()) {
                ok = false;
                failure = "parameter " + param.name + " has no default";
            }
        }
        check(ok && !op.formula.empty() && !op.explain.empty() && !op.outputs.empty(),
              std::string(module.name()) + "." + op.name +
                  " is fully described (formula, explain, outputs, parameter defaults)" +
                  (failure.empty() ? "" : " - " + failure));

        // Every op must run from the defaults its own description advertises.
        try {
            const Value result = module.invoke(op.name, args_from_defaults(op));
            check(result.is_object() && !result.empty(),
                  std::string(module.name()) + "." + op.name +
                      " runs from its advertised defaults");
            for (const auto& output : op.outputs) {
                if (!result.contains(output.name)) {
                    check(false, std::string(module.name()) + "." + op.name +
                                     " returns the advertised output " + output.name);
                }
            }
        } catch (const std::exception& error) {
            check(false, std::string(module.name()) + "." + op.name +
                             " runs from its advertised defaults - threw " + error.what());
        }
    }

    bool rejected = false;
    try {
        const Value ignored = module.invoke("no_such_op", Value::object());
        (void)ignored;
    } catch (const study::StudyError&) {
        rejected = true;
    } catch (...) {
        rejected = false;
    }
    check(rejected, std::string(module.name()) + " rejects an unknown op with StudyError");
}

void test_module_surface() {
    std::cout << "\n--- module surface ---\n";

    const study::DhKinematicsModule dh;
    const study::JacobianStaticsModule jacobian;
    exercise_module(dh);
    exercise_module(jacobian);

    // Numbers the panels promise, read back out of the JSON rather than
    // recomputed, so a wiring mistake in an op body is caught too.
    Value fk_args = Value::object();
    fk_args.set("q", Value::array({Value(0.2), Value(0.4), Value(0.5), Value(0.3), Value(0.7),
                                   Value(-0.2)}));
    const Value fk = dh.invoke("forward_kinematics", fk_args);
    check_below(fk["engine_position_error"].as_double(), 1e-9,
                "forward_kinematics reports agreement with the engine below 1e-9");
    check_below(fk["orthonormality_error"].as_double(), 1e-12,
                "forward_kinematics returns an orthonormal tool rotation");
    check(fk["frames"].size() == study::GP8_DOF && fk["A"].size() == study::GP8_DOF &&
              fk["origins"].size() == study::GP8_DOF + 1,
          "forward_kinematics returns six frames, six A_i and seven origins");

    Value ik_args = Value::object();
    ik_args.set("p", Value::array({Value(0.45), Value(0.0), Value(0.60)}));
    ik_args.set("rpy", Value::array({Value(0.0), Value(0.5), Value(0.0)}));
    ik_args.set("method", Value("both"));
    const Value ik = dh.invoke("inverse_kinematics", ik_args);
    check(ik["reachable"].as_bool() && ik["solution_count"].as_int() > 0,
          "inverse_kinematics solves a reachable target through the module surface");
    check_below(ik["worst_position_error"].as_double(), 1e-6,
                "inverse_kinematics reports every branch round-tripping below 1e-6");
    check(ik["numeric_converged"].as_bool() && ik["numeric_iterations"].as_int() >= 1,
          "the numeric method converges and reports a real iteration count");
    check_below(ik["numeric_position_error"].as_double(), 1e-4,
                "the numeric solution lands on the target");

    Value unreachable_args = Value::object();
    unreachable_args.set("p", Value::array({Value(2.0), Value(0.0), Value(0.4)}));
    unreachable_args.set("rpy", Value::array({Value(0.0), Value(0.0), Value(0.0)}));
    unreachable_args.set("method", Value("both"));
    const Value failed = dh.invoke("inverse_kinematics", unreachable_args);
    check(!failed["reachable"].as_bool() && failed["solution_count"].as_int() == 0 &&
              !failed["numeric_converged"].as_bool(),
          "inverse_kinematics reports an unreachable target as a failure through both methods");

    Value sweep_args = Value::object();
    sweep_args.set("q", Value::array({Value(0.0), Value(0.3), Value(0.6), Value(0.0), Value(0.8),
                                      Value(0.0)}));
    sweep_args.set("joint", Value(5));
    sweep_args.set("samples", Value(361));
    sweep_args.set("threshold", Value(0.02));
    const Value scan = dh.invoke("singularity_scan", sweep_args);
    check(scan["determinant_series"]["x"].size() == 361 &&
              scan["sigma_min_series"]["y"].size() == 361,
          "singularity_scan returns both series at the requested resolution");
    check(scan["singular_angle_count"].as_int() > 0 &&
              scan["singularity_type"].as_string() == "wrist",
          "sweeping axis B finds the wrist singularity and names it");

    Value workspace_args = Value::object();
    workspace_args.set("samples", Value(1500));
    workspace_args.set("slice", Value("full_cloud"));
    const Value cloud = dh.invoke("workspace_sample", workspace_args);
    check(cloud["points"].size() == 1500,
          "workspace_sample returns the requested number of points");
    const double sampled_reach = cloud["max_horizontal_reach_m"].as_double();
    const double table_reach = cloud["theoretical_reach_m"].as_double();
    check(sampled_reach <= table_reach + 1e-9 && sampled_reach > 0.6,
          "the sampled reach approaches, and never exceeds, the reach the DH table allows");
    check_below(std::abs(table_reach - study::GP8_HORIZONTAL_REACH_M), 1e-3,
                "the reach implied by the DH table matches the 0.727 m datasheet figure");

    Value xz_args = Value::object();
    xz_args.set("samples", Value(400));
    xz_args.set("slice", Value("xz_plane"));
    const Value xz = dh.invoke("workspace_sample", xz_args);
    bool planar = xz["points"].size() > 0;
    for (std::size_t i = 0; i < xz["points"].size(); ++i) {
        planar = planar && std::abs(xz["points"][i][1].as_double()) < 1e-12;
    }
    check(planar, "the XZ slice really lies in the plane y = 0");

    Value xy_args = Value::object();
    xy_args.set("samples", Value(400));
    xy_args.set("slice", Value("xy_plane"));
    xy_args.set("height", Value(0.675));
    const Value xy = dh.invoke("workspace_sample", xy_args);
    bool level = xy["points"].size() > 0;
    for (std::size_t i = 0; i < xy["points"].size(); ++i) {
        level = level && std::abs(xy["points"][i][2].as_double() - 0.675) < 1e-12;
    }
    check(level, "the XY slice really lies at the requested height");

    Value velocity_args = Value::object();
    velocity_args.set("q", Value::array({Value(0.0), Value(0.3), Value(0.6), Value(0.0),
                                         Value(0.8), Value(0.0)}));
    velocity_args.set("qdot", Value::array({Value(0.0), Value(1.0), Value(0.0), Value(0.0),
                                            Value(0.0), Value(0.0)}));
    const Value propagation = jacobian.invoke("velocity_propagation", velocity_args);
    check_below(propagation["recursion_residual"].as_double(), 1e-12,
                "velocity_propagation reports the recursion and J qdot agreeing");

    Value duality_args = Value::object();
    duality_args.set("q", Value::array({Value(0.3), Value(0.4), Value(0.5), Value(0.6), Value(0.8),
                                        Value(0.2)}));
    duality_args.set("wrench", Value::array({Value(10.0), Value(0.0), Value(0.0), Value(0.0),
                                             Value(0.0), Value(0.0)}));
    duality_args.set("tau", Value::array({Value(1.0), Value(-2.0), Value(3.0), Value(0.4),
                                          Value(-0.5), Value(0.1)}));
    const Value duality = jacobian.invoke("force_torque_duality", duality_args);
    check_below(duality["duality_residual"].as_double(), 1e-9,
                "force_torque_duality round-trips joint torques through the wrench and back");
    check_below(duality["wrench_residual"].as_double(), 1e-9,
                "force_torque_duality round-trips the wrench through the torques and back");
    check_below(duality["virtual_work_check"].as_double(), 1e-9,
                "force_torque_duality confirms the virtual-work identity it claims");

    Value manip_args = Value::object();
    manip_args.set("q", Value::array({Value(0.3), Value(0.4), Value(0.5), Value(0.6), Value(0.8),
                                      Value(0.2)}));
    const Value manip = jacobian.invoke("manipulability", manip_args);
    check(manip["rank"].as_int() == 6 && manip["singularity_type"].as_string() == "none",
          "manipulability reports full rank away from a singularity");
    check(manip["ellipsoid_radii"].size() == 3 && manip["ellipsoid"]["rows"].size() == 3,
          "manipulability returns three principal radii and three axes");

    // Bad input is an error response, not a crash or a silent default.
    bool rejected_missing = false;
    try {
        const Value ignored = jacobian.invoke("geometric_jacobian", Value::object());
        (void)ignored;
    } catch (const study::StudyError&) {
        rejected_missing = true;
    }
    check(rejected_missing, "a missing joint vector is rejected with StudyError");

    bool rejected_range = false;
    try {
        Value bad = Value::object();
        bad.set("q", Value::array({Value(0.0), Value(0.0), Value(0.0), Value(0.0), Value(0.0),
                                   Value(0.0)}));
        bad.set("damping", Value(-1.0));
        bad.set("twist", Value::array({Value(0.0), Value(0.0), Value(0.0), Value(0.0), Value(0.0),
                                       Value(0.0)}));
        const Value ignored = jacobian.invoke("inverse_velocity", bad);
        (void)ignored;
    } catch (const study::StudyError&) {
        rejected_range = true;
    }
    check(rejected_range, "a negative damping factor is rejected with StudyError");
}

void test_registry_protocol() {
    std::cout << "\n--- wire protocol ---\n";

    study::ModuleRegistry registry;
    registry.add(std::make_unique<study::DhKinematicsModule>());
    registry.add(std::make_unique<study::JacobianStaticsModule>());
    check(registry.size() == 2 && registry.find("dh_kinematics") != nullptr &&
              registry.find("jacobian_statics") != nullptr,
          "both modules register and are findable by name");

    Value request = Value::object();
    request.set("id", Value(7));
    request.set("module", Value("dh_kinematics"));
    request.set("op", Value("dh_table"));
    const Value response = registry.handle(request);
    check(response["ok"].as_bool() && response["id"].as_int() == 7 &&
              response["result"]["dh"]["rows"].size() == study::GP8_DOF,
          "a dh_table request comes back ok with six DH rows");

    Value bad = Value::object();
    bad.set("id", Value(8));
    bad.set("module", Value("jacobian_statics"));
    bad.set("op", Value("geometric_jacobian"));
    bad.set("args", Value::object({{"q", Value::array({Value(0.0)})}}));
    const Value error = registry.handle(bad);
    check(!error["ok"].as_bool() && error["id"].as_int() == 8 &&
              !error["error"].as_string().empty(),
          "a malformed joint vector comes back as an ok:false line, not a crash");

    const Value catalogue = registry.describe_all();
    check(catalogue["modules"].size() == 2,
          "describe_all carries both modules for the browser to build panels from");
}

}  // namespace

int main() {
    std::cout << "====================================================\n";
    std::cout << "   RUNNING YASKAWA STUDY KINEMATICS TEST SUITE      \n";
    std::cout << "====================================================\n";

    test_forward_kinematics();
    test_inverse_kinematics();
    test_jacobian();
    test_statics_and_conditioning();
    test_module_surface();
    test_registry_protocol();

    std::cout << "====================================================\n";
    std::cout << "checks run: " << g_checks << ", failed: " << g_failures << "\n";
    if (g_failures != 0) {
        for (const auto& name : g_failed_names) {
            std::cout << "FAILED: " << name << "\n";
        }
        std::cout << "STUDY KINEMATICS TESTS FAILED\n";
        return 1;
    }
    std::cout << "          ALL TESTS PASSED                          \n";
    std::cout << "====================================================\n";
    return 0;
}
