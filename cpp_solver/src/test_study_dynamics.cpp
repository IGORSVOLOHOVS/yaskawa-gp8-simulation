// Test suite for the two Block 4 / Block 5 study modules: `dynamics` and
// `trajectory_profiles`. Plain asserts in the style of src/test_study_modules.cpp -
// there is no gtest in this project and there will not be one.
//
// Every check asserts a property, never a printed string. The two that matter
// most, and the reason this file exists at all:
//
//   * the Lagrangian closed form and the Newton-Euler recursion, written
//     independently, agree on random states to 1e-8 N m;
//   * forward dynamics inverts inverse dynamics to 1e-8 rad/s^2.
//
// The energy-conservation tolerance is measured and printed rather than
// guessed: a torque-free, frictionless leapfrog integration over 0.5 s with a
// 2e-5 s step keeps T + V to 2.6e-4 J out of 119 J, the assertion is twice
// that, and the measured drift is printed next to it so a regression shows up
// as a number instead of as a green tick.

#include "study/dynamics.hpp"
#include "study/gp8_model.hpp"
#include "study/json.hpp"
#include "study/module_registry.hpp"
#include "study/study_module.hpp"
#include "study/trajectory_profiles.hpp"
#include "yaskawa_kinematics.hpp"

#include <Eigen/Dense>

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
namespace dyn = yaskawa::study::dyn;
namespace traj = yaskawa::study::traj;

namespace {

using dyn::Matrix6;
using dyn::Vector6;

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

void check_near(double actual, double expected, double tolerance, const std::string& what) {
    const double error = std::abs(actual - expected);
    const bool ok = std::isfinite(error) && error <= tolerance;
    if (!ok) {
        std::cout << "       expected " << expected << ", got " << actual << " (error " << error
                  << " > " << tolerance << ")\n";
    }
    check(ok, what);
}

void check_below(double actual, double tolerance, const std::string& what) {
    const bool ok = std::isfinite(actual) && actual <= tolerance;
    if (!ok) {
        std::cout << "       value " << actual << " exceeds the tolerance " << tolerance << "\n";
    }
    check(ok, what);
}

// ---------------------------------------------------------------------------
// Random states inside the real joint ranges
// ---------------------------------------------------------------------------

class StateSampler {
public:
    explicit StateSampler(unsigned seed) : rng_(seed) {}

    [[nodiscard]] Vector6 pose() {
        Vector6 q;
        for (std::size_t i = 0; i < study::GP8_DOF; ++i) {
            // Stay 10 percent inside each end of the range: the dynamics is
            // defined everywhere, but a pose on a hard stop is not a pose the
            // machine is ever asked to move through.
            const double low = study::joint_min(i);
            const double high = study::joint_max(i);
            const double margin = 0.1 * (high - low);
            q(static_cast<Eigen::Index>(i)) =
                low + margin + unit_(rng_) * (high - low - 2.0 * margin);
        }
        return q;
    }

    [[nodiscard]] Vector6 velocity(double scale) {
        Vector6 qd;
        for (Eigen::Index i = 0; i < 6; ++i) {
            qd(i) = (2.0 * unit_(rng_) - 1.0) * scale;
        }
        return qd;
    }

    [[nodiscard]] double payload() { return unit_(rng_) * study::GP8_PAYLOAD_KG; }

private:
    std::mt19937 rng_;
    std::uniform_real_distribution<double> unit_{0.0, 1.0};
};

// ---------------------------------------------------------------------------
// dynamics: the inertia matrix
// ---------------------------------------------------------------------------

void test_mass_matrix() {
    std::cout << "\n--- dynamics: M(q) ---\n";

    StateSampler sampler(20260410U);
    double worst_asymmetry = 0.0;
    double smallest_eigenvalue = std::numeric_limits<double>::infinity();
    double worst_inertia_mismatch = 0.0;
    bool all_positive_definite = true;
    bool all_rotor_positive = true;

    constexpr int kPoses = 240;
    for (int trial = 0; trial < kPoses; ++trial) {
        const Vector6 q = sampler.pose();
        const double payload = (trial % 3 == 0) ? sampler.payload() : 0.0;
        const Matrix6 M = dyn::mass_matrix(q, payload);

        worst_asymmetry = std::max(worst_asymmetry, (M - M.transpose()).cwiseAbs().maxCoeff());
        const Vector6 eigenvalues = dyn::symmetric_eigenvalues(M);
        smallest_eigenvalue = std::min(smallest_eigenvalue, eigenvalues.minCoeff());
        all_positive_definite = all_positive_definite && eigenvalues.minCoeff() > 0.0;

        // Positive definiteness the way it is actually used: v^T M v > 0 for
        // every direction, and the Cholesky factorisation succeeding.
        const Vector6 v = sampler.velocity(1.0);
        const double quadratic_form = v.dot(M * v);
        all_positive_definite = all_positive_definite && (quadratic_form > 0.0 || v.norm() == 0.0);
        const Eigen::LLT<Matrix6> llt(M);
        all_positive_definite = all_positive_definite && (llt.info() == Eigen::Success);

        // The rotor contribution is the only difference between the two forms.
        const Matrix6 rigid = dyn::rigid_mass_matrix(q, payload);
        Matrix6 expected = rigid;
        expected.diagonal() += dyn::rotor_inertia_diagonal();
        worst_inertia_mismatch =
            std::max(worst_inertia_mismatch, (M - expected).cwiseAbs().maxCoeff());

        // Kinetic energy is the same quadratic form, so it must agree with it.
        const double kinetic = dyn::energy(q, v, payload).kinetic;
        worst_inertia_mismatch = std::max(worst_inertia_mismatch,
                                          std::abs(kinetic - 0.5 * quadratic_form));
    }
    for (std::size_t i = 0; i < study::GP8_DOF; ++i) {
        all_rotor_positive = all_rotor_positive && study::reflected_rotor_inertia(i) > 0.0;
    }

    check_below(worst_asymmetry, 1e-12,
                "M(q) is symmetric at " + std::to_string(kPoses) + " random poses");
    check(all_positive_definite,
          "M(q) is positive definite at every pose: positive eigenvalues, a positive quadratic "
          "form and a successful Cholesky factorisation");
    check(smallest_eigenvalue > 0.0,
          "the smallest eigenvalue over every pose stays positive (" +
              json::number_to_string(smallest_eigenvalue) + " kg m^2)");
    check_below(worst_inertia_mismatch, 1e-12,
                "M = M_rigid + diag(n^2 J_rotor) and T = 0.5 qd^T M qd agree with M itself");
    check(all_rotor_positive, "every axis carries a positive reflected rotor inertia");

    // A heavier payload can only make the arm harder to accelerate.
    const Vector6 q = sampler.pose();
    bool monotone = true;
    double previous = dyn::mass_matrix(q, 0.0).trace();
    for (double payload = 1.0; payload <= study::GP8_PAYLOAD_KG + 1e-9; payload += 1.0) {
        const double current = dyn::mass_matrix(q, payload).trace();
        monotone = monotone && current > previous;
        previous = current;
    }
    check(monotone, "trace(M) grows strictly with the payload from 0 to 8 kg");
}

// ---------------------------------------------------------------------------
// dynamics: the two formulations
// ---------------------------------------------------------------------------

double test_formulation_agreement() {
    std::cout << "\n--- dynamics: Lagrangian against Newton-Euler ---\n";

    StateSampler sampler(77001U);
    double worst = 0.0;
    double worst_torque = 0.0;
    constexpr int kStates = 400;
    for (int trial = 0; trial < kStates; ++trial) {
        const Vector6 q = sampler.pose();
        const Vector6 qd = sampler.velocity(3.0);
        const Vector6 qdd = sampler.velocity(10.0);
        const double payload = (trial % 2 == 0) ? sampler.payload() : 0.0;
        const bool friction = (trial % 3 != 0);

        const Vector6 lagrangian =
            dyn::inverse_dynamics_lagrangian(q, qd, qdd, payload, friction);
        const Vector6 newton_euler =
            dyn::inverse_dynamics_newton_euler(q, qd, qdd, payload, friction);
        worst = std::max(worst, (lagrangian - newton_euler).cwiseAbs().maxCoeff());
        worst_torque = std::max(worst_torque, lagrangian.cwiseAbs().maxCoeff());
    }
    std::cout << "       worst disagreement over " << kStates << " random states: " << worst
              << " N m, on torques up to " << worst_torque << " N m\n";
    check_below(worst, 1e-8,
                "the Lagrangian closed form and the Newton-Euler recursion agree to 1e-8 N m over "
                + std::to_string(kStates) + " random states");

    // The static case on its own, because gravity is where a lever arm is most
    // easily dropped: both formulations must produce the same holding torque.
    double worst_static = 0.0;
    for (int trial = 0; trial < 60; ++trial) {
        const Vector6 q = sampler.pose();
        const double payload = sampler.payload();
        const Vector6 closed_form = dyn::gravity_torque(q, payload);
        const Vector6 recursive = dyn::inverse_dynamics_newton_euler(
            q, Vector6::Zero(), Vector6::Zero(), payload, false);
        worst_static = std::max(worst_static, (closed_form - recursive).cwiseAbs().maxCoeff());
    }
    check_below(worst_static, 1e-9,
                "g(q) from the potential-energy gradient equals the recursion's static torque");

    // C(q,qd) is verified against its own identity rather than against itself:
    // qd^T (Mdot - 2C) qd = 0 holds only for a correct Christoffel expansion.
    double worst_passivity = 0.0;
    double worst_quadratic = 0.0;
    for (int trial = 0; trial < 60; ++trial) {
        const Vector6 q = sampler.pose();
        const Vector6 qd = sampler.velocity(2.0);
        const Matrix6 C = dyn::coriolis_matrix(q, qd, 0.0);
        const Matrix6 Mdot = dyn::mass_matrix_rate(q, qd, 0.0);
        worst_passivity = std::max(worst_passivity, std::abs(qd.dot((Mdot - 2.0 * C) * qd)));
        const Matrix6 C_double = dyn::coriolis_matrix(q, 2.0 * qd, 0.0);
        worst_quadratic = std::max(worst_quadratic,
                                   (C_double * (2.0 * qd) - 4.0 * (C * qd)).cwiseAbs().maxCoeff());
    }
    check_below(worst_passivity, 1e-9,
                "the passivity identity qd^T (Mdot - 2C) qd = 0 holds, so C comes from the "
                "Christoffel symbols of this very M");
    check_below(worst_quadratic, 1e-9,
                "C(q,qd) qd is exactly quadratic in speed: doubling qd quadruples the term");

    return worst;
}

void test_forward_dynamics() {
    std::cout << "\n--- dynamics: forward dynamics inverts inverse dynamics ---\n";

    StateSampler sampler(31337U);
    double worst = 0.0;
    double worst_residual = 0.0;
    bool always_positive_definite = true;
    constexpr int kStates = 200;
    for (int trial = 0; trial < kStates; ++trial) {
        const Vector6 q = sampler.pose();
        const Vector6 qd = sampler.velocity(2.0);
        const Vector6 qdd = sampler.velocity(8.0);
        const double payload = (trial % 2 == 0) ? sampler.payload() : 0.0;
        const bool friction = (trial % 2 == 0);

        const Vector6 tau = dyn::inverse_dynamics_lagrangian(q, qd, qdd, payload, friction);
        const dyn::ForwardResult result = dyn::forward_dynamics(q, qd, tau, payload, friction);
        worst = std::max(worst, (result.qddot - qdd).cwiseAbs().maxCoeff());
        worst_residual = std::max(worst_residual, result.residual);
        always_positive_definite = always_positive_definite && result.positive_definite;
    }
    std::cout << "       worst acceleration round-trip error: " << worst << " rad/s^2\n";
    check_below(worst, 1e-8,
                "forward dynamics returns the acceleration inverse dynamics was given, to 1e-8 "
                "rad/s^2, over " + std::to_string(kStates) + " random states");
    check_below(worst_residual, 1e-9, "the LDLT solve residual || M qdd - rhs || stays at zero");
    check(always_positive_definite, "M(q) is reported positive definite at every state solved");

    // The recursive formulation has to invert the same way.
    const Vector6 q = sampler.pose();
    const Vector6 qd = sampler.velocity(1.5);
    const Vector6 qdd = sampler.velocity(5.0);
    const Vector6 tau = dyn::inverse_dynamics_newton_euler(q, qd, qdd, 3.0, true);
    const dyn::ForwardResult result = dyn::forward_dynamics(q, qd, tau, 3.0, true);
    check_below((result.qddot - qdd).cwiseAbs().maxCoeff(), 1e-8,
                "forward dynamics also inverts the Newton-Euler torque, payload included");
}

void test_gravity() {
    std::cout << "\n--- dynamics: gravity ---\n";

    StateSampler sampler(4242U);
    double worst_zero_gravity = 0.0;
    for (int trial = 0; trial < 50; ++trial) {
        const Vector6 q = sampler.pose();
        const Vector6 g = dyn::gravity_torque(q, sampler.payload(), Eigen::Vector3d::Zero());
        worst_zero_gravity = std::max(worst_zero_gravity, g.cwiseAbs().maxCoeff());
    }
    check(worst_zero_gravity == 0.0,
          "with gravity switched off the gravity term is exactly zero at every pose and payload");

    // A stretched-out pose: the L-axis horizontal, the forearm straight out, so
    // the payload hangs on the longest lever the machine has.
    Vector6 stretched;
    stretched << 0.0, 0.5 * kPi, 0.0, 0.0, 0.0, 0.0;
    bool monotone_joint2 = true;
    bool monotone_joint3 = true;
    double previous_joint2 = std::abs(dyn::gravity_torque(stretched, 0.0)(1));
    double previous_joint3 = std::abs(dyn::gravity_torque(stretched, 0.0)(2));
    const double unloaded_joint2 = previous_joint2;
    for (double payload = 0.5; payload <= study::GP8_PAYLOAD_KG + 1e-9; payload += 0.5) {
        const Vector6 g = dyn::gravity_torque(stretched, payload);
        const double joint2 = std::abs(g(1));
        const double joint3 = std::abs(g(2));
        monotone_joint2 = monotone_joint2 && joint2 > previous_joint2;
        monotone_joint3 = monotone_joint3 && joint3 > previous_joint3;
        previous_joint2 = joint2;
        previous_joint3 = joint3;
    }
    std::cout << "       stretched pose, joint 2 holding torque: " << unloaded_joint2
              << " N m unloaded, " << previous_joint2 << " N m with 8 kg\n";
    check(monotone_joint2,
          "at a stretched-out pose the joint 2 (L) holding torque rises monotonically with the "
          "payload from 0 to 8 kg");
    check(monotone_joint3,
          "the joint 3 (U) holding torque rises monotonically with the payload as well");
    check(previous_joint2 > unloaded_joint2 * 1.5,
          "an 8 kg payload more than halves the joint 2 torque margin at full stretch");

    // Joints whose axis is parallel to gravity carry none of it.
    Vector6 upright = Vector6::Zero();
    const Vector6 g_upright = dyn::gravity_torque(upright, 8.0);
    check_below(std::abs(g_upright(0)), 1e-12,
                "joint 1 (S) carries no gravity torque: its axis is vertical, so the weight has "
                "no moment about it");

    // Gravity is the gradient of the potential energy, so a finite difference of
    // V has to reproduce it. This is an independent check on the sign as well.
    const Vector6 q = sampler.pose();
    const double payload = 4.0;
    const Vector6 analytic = dyn::gravity_torque(q, payload);
    const double h = 1e-6;
    double worst_gradient = 0.0;
    for (Eigen::Index i = 0; i < 6; ++i) {
        Vector6 plus = q;
        Vector6 minus = q;
        plus(i) += h;
        minus(i) -= h;
        const double numeric = (dyn::energy(plus, Vector6::Zero(), payload).potential -
                                dyn::energy(minus, Vector6::Zero(), payload).potential) /
                               (2.0 * h);
        worst_gradient = std::max(worst_gradient, std::abs(numeric - analytic(i)));
    }
    check_below(worst_gradient, 1e-6,
                "g(q) matches a central difference of the potential energy, which fixes its sign "
                "as well as its size");
}

double test_energy_conservation() {
    std::cout << "\n--- dynamics: energy consistency ---\n";

    // A torque-free, frictionless fall, integrated with a leapfrog scheme:
    // half-kick the velocity, drift the position, recompute the acceleration at
    // the half-step velocity, half-kick again. It is only symplectic-ish,
    // because the acceleration depends on the velocity through C(q,qd), but it
    // is the cheapest scheme that does not bleed energy by construction.
    Vector6 q;
    q << 0.0, 0.6, -0.3, 0.0, 0.4, 0.0;
    Vector6 qd = Vector6::Zero();
    const double payload = 2.0;
    const double dt = 2.0e-5;
    const double horizon = 0.5;
    const auto steps = static_cast<int>(horizon / dt);

    const dyn::EnergyTerms start = dyn::energy(q, qd, payload);
    double worst_drift = 0.0;
    double kinetic_peak = 0.0;

    Vector6 qdd = dyn::forward_dynamics(q, qd, Vector6::Zero(), payload, false).qddot;
    for (int step = 0; step < steps; ++step) {
        const Vector6 qd_half = qd + 0.5 * dt * qdd;
        q += dt * qd_half;
        qdd = dyn::forward_dynamics(q, qd_half, Vector6::Zero(), payload, false).qddot;
        qd = qd_half + 0.5 * dt * qdd;

        const dyn::EnergyTerms now = dyn::energy(q, qd, payload);
        worst_drift = std::max(worst_drift, std::abs(now.total - start.total));
        kinetic_peak = std::max(kinetic_peak, now.kinetic);
    }

    const dyn::EnergyTerms end = dyn::energy(q, qd, payload);
    const double relative = worst_drift / std::max(kinetic_peak, 1e-12);
    std::cout << "       horizon " << horizon << " s, step " << dt << " s, " << steps
              << " steps\n       T + V start " << start.total << " J, end " << end.total
              << " J, worst drift " << worst_drift << " J (" << 100.0 * relative
              << " percent of the " << kinetic_peak << " J peak kinetic energy)\n";

    // The tolerance, stated honestly: the measured worst drift at this step is
    // 2.6e-4 J out of a 119 J total, and it falls off linearly with dt
    // (1.28e-3 J at dt = 1e-4 s, 6.4e-4 J at 5e-5 s, 2.6e-4 J at 2e-5 s) -
    // first order rather than second, because the acceleration depends on the
    // velocity through C(q,qd) and the scheme is therefore only
    // symplectic-ish. The assertion is twice the measured value, so it catches
    // a regression in the dynamics without pretending the integrator is exact.
    check_below(worst_drift, 5.0e-4,
                "T + V is conserved to 5e-4 J over a 0.5 s torque-free frictionless fall "
                "(leapfrog, dt = 2e-5 s), against a measured drift of 2.6e-4 J");
    check(kinetic_peak > 1.0,
          "the fall actually moved: the peak kinetic energy is above 1 J, so the conservation "
          "check is not a check on a stationary arm");
    check_below(relative, 1.0e-5,
                "the drift stays below 0.001 percent of the peak kinetic energy");

    // Friction can only take energy out, never put it in.
    Vector6 moving;
    moving << 0.2, 0.3, -0.2, 0.1, 0.4, 0.0;
    const Vector6 speed = Vector6::Constant(1.0);
    const Vector6 friction = dyn::friction_torque(speed);
    check(friction.dot(speed) > 0.0,
          "friction opposes the motion: its torque has a positive power against the velocity, so "
          "it always removes energy");
    check(dyn::friction_torque(Vector6::Zero()).cwiseAbs().maxCoeff() == 0.0,
          "friction is zero at rest, so a static hold is not asked for Coulomb torque");
    check(dyn::energy(moving, Vector6::Zero(), 0.0).kinetic == 0.0,
          "kinetic energy is zero when the arm is at rest");

    return worst_drift;
}

// ---------------------------------------------------------------------------
// trajectory_profiles
// ---------------------------------------------------------------------------

void test_polynomial_profiles() {
    std::cout << "\n--- trajectory_profiles: cubic and quintic ---\n";

    Vector6 q0 = Vector6::Zero();
    Vector6 q1 = Vector6::Zero();
    q1(1) = 0.5 * kPi;  // the course's worked example: 0 to 90 degrees
    const double duration = 2.0;

    const traj::PolynomialProfile cubic = traj::cubic_profile(q0, q1, duration, 0.0, 0.0);
    const traj::Sample cubic_start = cubic.at(0.0);
    const traj::Sample cubic_end = cubic.at(duration);
    check_below((cubic_start.q - q0).cwiseAbs().maxCoeff(), 1e-14,
                "the cubic starts exactly at q_start");
    check_below((cubic_end.q - q1).cwiseAbs().maxCoeff(), 1e-14,
                "the cubic ends exactly at q_end");
    check_below(cubic_start.qd.cwiseAbs().maxCoeff(), 1e-14, "the cubic starts exactly at rest");
    check_below(cubic_end.qd.cwiseAbs().maxCoeff(), 1e-14, "the cubic ends exactly at rest");

    // The hand result the course asks for: a2 = 3 dq / tf^2, a3 = -2 dq / tf^3,
    // peak speed 1.5 dq / tf at the midpoint.
    const double travel = q1(1) - q0(1);
    check_near(cubic.c[1][2], 3.0 * travel / (duration * duration), 1e-12,
               "the cubic's a2 is 3 dq / tf^2");
    check_near(cubic.c[1][3], -2.0 * travel / (duration * duration * duration), 1e-12,
               "the cubic's a3 is -2 dq / tf^3");
    check_near(cubic.at(0.5 * duration).qd(1), 1.5 * travel / duration, 1e-12,
               "the cubic's peak speed is 1.5 dq / tf at the midpoint (67.5 deg/s for the 90 "
               "degree, 2 s move)");
    check_near(cubic.at(0.0).qdd(1), 6.0 * travel / (duration * duration), 1e-12,
               "the cubic's acceleration steps to 6 dq / tf^2 at t = 0 rather than ramping");

    const traj::PolynomialProfile quintic =
        traj::quintic_profile(q0, q1, duration, 0.0, 0.0, 0.0, 0.0);
    const traj::Sample quintic_start = quintic.at(0.0);
    const traj::Sample quintic_end = quintic.at(duration);
    check_below((quintic_start.q - q0).cwiseAbs().maxCoeff(), 1e-14,
                "the quintic starts exactly at q_start");
    check_below((quintic_end.q - q1).cwiseAbs().maxCoeff(), 1e-13,
                "the quintic ends exactly at q_end");
    check_below(quintic_start.qd.cwiseAbs().maxCoeff(), 1e-14, "the quintic starts at rest");
    check_below(quintic_end.qd.cwiseAbs().maxCoeff(), 1e-13, "the quintic ends at rest");
    check_below(quintic_start.qdd.cwiseAbs().maxCoeff(), 1e-14,
                "the quintic starts at zero acceleration, which is what it buys over the cubic");
    check_below(quintic_end.qdd.cwiseAbs().maxCoeff(), 1e-12,
                "the quintic ends at zero acceleration");
    check_near(quintic.at(0.5 * duration).qd(1), 1.875 * travel / duration, 1e-12,
               "the quintic's peak speed is 1.875 dq / tf, higher than the cubic's 1.5 dq / tf");

    // Non-zero boundary conditions are honoured too, or the op would be lying
    // about its own parameters.
    const traj::PolynomialProfile shaped =
        traj::quintic_profile(q0, q1, duration, 0.2, -0.3, 0.5, -0.4);
    check_near(shaped.at(0.0).qd(1), 0.2, 1e-12, "a requested start velocity is reproduced");
    check_near(shaped.at(duration).qd(1), -0.3, 1e-10, "a requested end velocity is reproduced");
    check_near(shaped.at(0.0).qdd(1), 0.5, 1e-12, "a requested start acceleration is reproduced");
    check_near(shaped.at(duration).qdd(1), -0.4, 1e-9, "a requested end acceleration is reproduced");

    // The velocity trace has to integrate back to the distance travelled.
    for (const auto& entry : {std::pair<const char*, traj::PolynomialProfile>{"cubic", cubic},
                              std::pair<const char*, traj::PolynomialProfile>{"quintic", quintic}}) {
        const traj::Samples samples = traj::sample_polynomial(entry.second, 2000, entry.first);
        double integral = 0.0;
        for (std::size_t i = 1; i < samples.points.size(); ++i) {
            const double dt = samples.points[i].t - samples.points[i - 1].t;
            integral += 0.5 * dt * (samples.points[i].qd(1) + samples.points[i - 1].qd(1));
        }
        check_near(integral, travel, 1e-5,
                   std::string("the ") + entry.first +
                       " velocity trace integrates back to the travelled distance");

        // And the acceleration trace integrates back to the velocity change.
        double velocity_integral = 0.0;
        for (std::size_t i = 1; i < samples.points.size(); ++i) {
            const double dt = samples.points[i].t - samples.points[i - 1].t;
            velocity_integral +=
                0.5 * dt * (samples.points[i].qdd(1) + samples.points[i - 1].qdd(1));
        }
        check_near(velocity_integral, 0.0, 1e-6,
                   std::string("the ") + entry.first +
                       " acceleration trace integrates back to zero net velocity change");
    }

    // The quintic's jerk is finite; the cubic's is constant inside the move.
    const traj::Samples cubic_samples = traj::sample_polynomial(cubic, 500, "cubic");
    double cubic_jerk_spread = 0.0;
    for (const traj::Sample& s : cubic_samples.points) {
        cubic_jerk_spread = std::max(cubic_jerk_spread,
                                     std::abs(s.qddd(1) - cubic_samples.points[0].qddd(1)));
    }
    check_below(cubic_jerk_spread, 1e-9,
                "the cubic's jerk is constant inside the move, so its acceleration is a straight "
                "line");
}

void test_lspb() {
    std::cout << "\n--- trajectory_profiles: LSPB ---\n";

    Vector6 q0 = Vector6::Zero();
    Vector6 q1 = Vector6::Zero();
    q1(1) = 0.5 * kPi;
    const double duration = 2.0;
    const double blend = 0.5;

    const traj::LspbProfile profile = traj::lspb_from_blend_time(q0, q1, duration, blend);
    const double travel = q1(1) - q0(1);

    // Continuity of velocity at both joins, evaluated from both sides.
    const double eps = 1e-9;
    const double before_first = profile.at(blend - eps).qd(1);
    const double after_first = profile.at(blend + eps).qd(1);
    const double before_second = profile.at(duration - blend - eps).qd(1);
    const double after_second = profile.at(duration - blend + eps).qd(1);
    check_below(std::abs(after_first - before_first), 1e-7,
                "the LSPB velocity is continuous at the first blend join");
    check_below(std::abs(after_second - before_second), 1e-7,
                "the LSPB velocity is continuous at the second blend join");
    check_near(profile.at(blend).qd(1), profile.s_dot_cruise * travel, 1e-12,
               "the velocity at the join equals the cruise velocity");

    // Position continuity at the joins, and the endpoints.
    check_below(std::abs(profile.at(blend + eps).q(1) - profile.at(blend - eps).q(1)), 1e-8,
                "the LSPB position is continuous at the first blend join");
    check_below((profile.at(0.0).q - q0).cwiseAbs().maxCoeff(), 1e-14,
                "the LSPB starts exactly at q_start");
    check_below((profile.at(duration).q - q1).cwiseAbs().maxCoeff(), 1e-14,
                "the LSPB ends exactly at q_end");
    check_below(profile.at(0.0).qd.cwiseAbs().maxCoeff(), 1e-14, "the LSPB starts at rest");
    check_below(profile.at(duration).qd.cwiseAbs().maxCoeff(), 1e-14, "the LSPB ends at rest");

    // The area under a trapezoid: the analytic cycle-time relation.
    check_near(profile.s_dot_cruise, 1.0 / (duration - blend), 1e-14,
               "the cruise rate of the path parameter is 1 / (tf - tb)");
    const traj::Samples samples = traj::sample_lspb(profile, 2000);
    double integral = 0.0;
    for (std::size_t i = 1; i < samples.points.size(); ++i) {
        const double dt = samples.points[i].t - samples.points[i - 1].t;
        integral += 0.5 * dt * (samples.points[i].qd(1) + samples.points[i - 1].qd(1));
    }
    check_near(integral, travel, 1e-4,
               "the LSPB velocity trace integrates back to the travelled distance");

    // The triangular limit: the blend fills both halves.
    const traj::LspbProfile triangular =
        traj::lspb_from_blend_time(q0, q1, duration, 0.5 * duration);
    check_near(triangular.at(0.5 * duration).qd(1), 2.0 * travel / duration, 1e-12,
               "the triangular case peaks at exactly twice the average speed");

    // Impossible combinations are rejected, with a StudyError and not a number.
    const auto rejects = [&](const std::string& what, auto&& call) {
        bool threw = false;
        std::string message;
        try {
            call();
        } catch (const study::StudyError& error) {
            threw = true;
            message = error.what();
        } catch (...) {
            threw = false;
        }
        check(threw && !message.empty(), "LSPB rejects " + what + " with a StudyError");
    };
    rejects("a blend longer than half the duration",
            [&] { (void)traj::lspb_from_blend_time(q0, q1, duration, 1.5); });
    rejects("a blend time of zero",
            [&] { (void)traj::lspb_from_blend_time(q0, q1, duration, 0.0); });
    rejects("a cruise velocity below the average speed",
            [&] { (void)traj::lspb_from_cruise_velocity(q0, q1, duration, 1, 0.5); });
    rejects("a cruise velocity above twice the average speed",
            [&] { (void)traj::lspb_from_cruise_velocity(q0, q1, duration, 1, 3.0); });
    rejects("a cruise velocity read on a joint that does not move",
            [&] { (void)traj::lspb_from_cruise_velocity(q0, q1, duration, 5, 1.0); });

    // Tuning by cruise velocity lands on the velocity that was asked for.
    const double requested = 1.1;  // between pi/4 = 0.785 and pi/2 = 1.571 rad/s
    const traj::LspbProfile tuned =
        traj::lspb_from_cruise_velocity(q0, q1, duration, 1, requested);
    check_near(tuned.s_dot_cruise * travel, requested, 1e-12,
               "tuning by cruise velocity reproduces the requested plateau speed");
    check_below((tuned.at(duration).q - q1).cwiseAbs().maxCoeff(), 1e-12,
                "the velocity-tuned LSPB still arrives exactly at the goal");
}

void test_via_points() {
    std::cout << "\n--- trajectory_profiles: via points ---\n";

    std::vector<Vector6> points;
    Vector6 p = Vector6::Zero();
    points.push_back(p);
    p << 0.4, 0.6, -0.3, 0.0, 0.5, 0.0;
    points.push_back(p);
    p << 0.9, 0.3, 0.4, 0.2, 0.8, 0.0;
    points.push_back(p);
    p << 1.2, 0.0, 0.0, 0.0, 0.0, 0.0;
    points.push_back(p);

    const traj::ViaProfile profile = traj::via_profile(points, {}, 6.0);
    check_near(profile.total_duration, 6.0, 1e-12,
               "the segment durations split the requested total exactly");
    check(profile.segments.size() == points.size() - 1,
          "one polynomial segment per pair of waypoints");

    double worst_position_jump = 0.0;
    double worst_velocity_jump = 0.0;
    double worst_acceleration_jump = 0.0;
    for (std::size_t k = 1; k < profile.segments.size(); ++k) {
        const traj::Sample before = profile.segments[k - 1].at(profile.durations[k - 1]);
        const traj::Sample after = profile.segments[k].at(0.0);
        worst_position_jump =
            std::max(worst_position_jump, (before.q - after.q).cwiseAbs().maxCoeff());
        worst_velocity_jump =
            std::max(worst_velocity_jump, (before.qd - after.qd).cwiseAbs().maxCoeff());
        worst_acceleration_jump =
            std::max(worst_acceleration_jump, (before.qdd - after.qdd).cwiseAbs().maxCoeff());
    }
    check_below(worst_position_jump, 1e-12, "the via-point spline is continuous in position");
    check_below(worst_velocity_jump, 1e-12, "the via-point spline is continuous in velocity");
    check_below(worst_acceleration_jump, 1e-12,
                "the via-point spline is continuous in acceleration, so it is C2 and not only C1");

    // Every waypoint is actually passed through, at its own start time.
    double worst_waypoint_error = 0.0;
    for (std::size_t k = 0; k < points.size(); ++k) {
        const double t = (k < profile.start_times.size()) ? profile.start_times[k]
                                                          : profile.total_duration;
        worst_waypoint_error =
            std::max(worst_waypoint_error, (profile.at(t).q - points[k]).cwiseAbs().maxCoeff());
    }
    check_below(worst_waypoint_error, 1e-12, "the spline passes through every waypoint");
    check_below(profile.at(0.0).qd.cwiseAbs().maxCoeff(), 1e-12,
                "the via-point sequence starts at rest");
    check_below(profile.at(profile.total_duration).qd.cwiseAbs().maxCoeff(), 1e-12,
                "the via-point sequence ends at rest");

    // Explicit durations are used as given, and a bad list is rejected.
    const traj::ViaProfile timed = traj::via_profile(points, {1.0, 2.0, 1.5}, 0.0);
    check_near(timed.total_duration, 4.5, 1e-12, "explicit segment durations are used as given");
    bool threw = false;
    try {
        (void)traj::via_profile(points, {1.0, 2.0}, 0.0);
    } catch (const study::StudyError&) {
        threw = true;
    }
    check(threw, "a durations list of the wrong length is rejected with a StudyError");
    threw = false;
    try {
        (void)traj::via_profile({points[0]}, {}, 2.0);
    } catch (const study::StudyError&) {
        threw = true;
    }
    check(threw, "a single waypoint is rejected: there is no segment to plan");
}

void test_constraint_check() {
    std::cout << "\n--- trajectory_profiles: constraint check ---\n";

    Vector6 q0 = Vector6::Zero();
    Vector6 q1 = Vector6::Zero();
    q1(1) = 0.5 * kPi;

    // A comfortable move passes everything.
    const traj::Samples easy =
        traj::sample_polynomial(traj::quintic_profile(q0, q1, 2.0, 0.0, 0.0, 0.0, 0.0), 400,
                                "quintic");
    const traj::ConstraintVerdict easy_verdict = traj::check_constraints(easy, 15.0, 150.0, 3.0);
    check(easy_verdict.feasible && easy_verdict.verdict == "FEASIBLE",
          "a 90 degree quintic in 2 s is declared FEASIBLE against the real GP8 limits");
    check(easy_verdict.worst_ratio < 1.0,
          "and its worst value sits below every limit (" +
              json::number_to_string(easy_verdict.worst_ratio) + " of the tightest one)");

    // A move built to break the joint-2 speed rating: 2.5 rad in 0.2 s needs a
    // peak of 1.5 * 2.5 / 0.2 = 18.75 rad/s against the 6.70 rad/s rating.
    Vector6 fast_goal = Vector6::Zero();
    fast_goal(1) = 2.5;
    const traj::Samples fast =
        traj::sample_polynomial(traj::cubic_profile(q0, fast_goal, 0.2, 0.0, 0.0), 400, "cubic");
    const traj::ConstraintVerdict fast_verdict =
        traj::check_constraints(fast, 1.0e6, 1.0e6, 10.0);
    const double expected_peak = 1.5 * 2.5 / 0.2;
    std::cout << "       joint 2 peak speed " << fast_verdict.joints[1].peak_speed
              << " rad/s against the " << study::joint_max_velocity(1) << " rad/s rating\n";
    check(!fast_verdict.feasible && fast_verdict.verdict == "INFEASIBLE",
          "a profile that asks joint 2 for 18.75 rad/s is declared INFEASIBLE");
    check(!fast_verdict.velocity_ok && !fast_verdict.joints[1].velocity_ok,
          "the velocity constraint is the one marked as failed, and on joint 2");
    check_near(fast_verdict.joints[1].peak_speed, expected_peak, 1e-3,
               "the reported peak speed is the analytic 1.5 dq / tf");
    check(fast_verdict.joints[1].binding_constraint == "joint velocity",
          "joint 2's binding constraint is named as the joint velocity");
    check(fast_verdict.binding_constraint.find("joint velocity") != std::string::npos &&
              fast_verdict.binding_constraint.find("joint 2") != std::string::npos,
          "the overall binding constraint names both the constraint and the axis: \"" +
              fast_verdict.binding_constraint + "\"");
    check(fast_verdict.worst_ratio > 2.5,
          "the reported worst ratio is above 2.5, which is the factor by which joint 2 is over "
          "its rating");

    // A move that is legal everywhere but takes too long fails on cycle time.
    const traj::Samples slow =
        traj::sample_polynomial(traj::quintic_profile(q0, q1, 8.0, 0.0, 0.0, 0.0, 0.0), 400,
                                "quintic");
    const traj::ConstraintVerdict slow_verdict = traj::check_constraints(slow, 15.0, 150.0, 3.0);
    check(!slow_verdict.feasible && !slow_verdict.cycle_time_ok && slow_verdict.velocity_ok,
          "a slow but legal move fails on cycle time alone");
    check(slow_verdict.binding_constraint.find("cycle time") != std::string::npos,
          "and the binding constraint is named as the cycle time");

    // An acceleration ceiling that bites, with everything else comfortable.
    const traj::ConstraintVerdict accel_verdict = traj::check_constraints(easy, 0.5, 150.0, 3.0);
    check(!accel_verdict.accel_ok && accel_verdict.velocity_ok,
          "a 0.5 rad/s^2 acceleration ceiling fails the same profile that passed at 15 rad/s^2");

    // An LSPB cannot meet any finite jerk limit, and says so.
    const traj::Samples lspb =
        traj::sample_lspb(traj::lspb_from_blend_time(q0, q1, 2.0, 0.5), 400);
    const traj::ConstraintVerdict lspb_verdict =
        traj::check_constraints(lspb, 15.0, 1.0e9, 3.0);
    check(!lspb.jerk_bounded && !lspb_verdict.jerk_ok,
          "an LSPB is reported as having unbounded jerk, so no finite jerk limit is met");
    check(lspb_verdict.binding_constraint.find("jerk") != std::string::npos,
          "and jerk is named as its binding constraint");
    check(lspb_verdict.velocity_ok && lspb_verdict.range_ok,
          "while its speeds and joint positions are still inside the machine's limits");

    // A profile that drives a joint past its hard stop fails on range.
    Vector6 out_of_range = Vector6::Zero();
    out_of_range(1) = 2.4;  // inside the limit at the endpoint
    const traj::Samples overshoot = traj::sample_polynomial(
        traj::cubic_profile(q0, out_of_range, 4.0, 3.0, 0.0), 400, "cubic");
    const traj::ConstraintVerdict range_verdict =
        traj::check_constraints(overshoot, 1.0e6, 1.0e6, 100.0);
    check(!range_verdict.range_ok && !range_verdict.joints[1].range_ok,
          "a profile that overshoots joint 2 past its +2.530 rad hard stop fails on joint range");

    // Bad limits are rejected rather than silently accepted.
    bool threw = false;
    try {
        (void)traj::check_constraints(easy, -1.0, 150.0, 3.0);
    } catch (const study::StudyError&) {
        threw = true;
    }
    check(threw, "a non-positive acceleration limit is rejected with a StudyError");
}

void test_cartesian_lspb() {
    std::cout << "\n--- trajectory_profiles: Cartesian LSPB through the IK ---\n";

    const study::TrajectoryProfilesModule module;
    Value args = Value::object();
    args.set("p_start", json::from_vec3(Eigen::Vector3d(0.476, -0.20, 0.733)));
    args.set("p_end", json::from_vec3(Eigen::Vector3d(0.476, 0.20, 0.733)));
    args.set("duration", Value(2.0));
    args.set("blend_time", Value(0.5));
    Vector6 seed;
    seed << 0.0, 0.3, -0.4, 0.0, 0.6, 0.0;
    args.set("q_seed", json::from_vec6(seed));
    args.set("samples", Value(60));
    const Value result = module.invoke("cartesian_lspb", args);

    check(result["tool_path"].size() == 60 && result["joint_profiles"].size() == study::GP8_DOF,
          "the Cartesian op returns one tool point per sample and one curve per joint");
    check(result["ik_failures"].as_int() == 0,
          "the IK reaches every sample of the default straight-line move");
    check_below(result["max_ik_position_error"].as_double(), 5e-4,
                "and the achieved tool position tracks the commanded one to under 0.5 mm");
    check_below(result["straightness_error"].as_double(), 1e-12,
                "the commanded path is exactly straight in Cartesian space");
    std::cout << "       worst joint departure from a straight joint-space interpolation: "
              << result["max_joint_nonlinearity"].as_double() << " rad\n";
    check(result["max_joint_nonlinearity"].as_double() > 0.01,
          "yet the joint curves are measurably not straight, which is the lesson of the op: a "
          "straight line in space is curved joint motion");
    check_near(result["path_length"].as_double(), 0.4, 1e-12,
               "the reported path length is the distance between the two points");

    // The same geometry with no distance to travel has nothing to blend.
    bool threw = false;
    try {
        Value degenerate = args;
        degenerate.set("p_end", json::from_vec3(Eigen::Vector3d(0.476, -0.20, 0.733)));
        (void)module.invoke("cartesian_lspb", degenerate);
    } catch (const study::StudyError&) {
        threw = true;
    }
    check(threw, "a zero-length Cartesian path is rejected with a StudyError");
}

// ---------------------------------------------------------------------------
// Both modules through the wire protocol
// ---------------------------------------------------------------------------

[[nodiscard]] Value args_from_defaults(const study::OpSpec& op) {
    Value args = Value::object();
    for (const auto& param : op.params) {
        args.set(param.name, param.default_value);
    }
    return args;
}

void exercise_module(const study::StudyModule& module, std::size_t expected_ops) {
    const study::ModuleDescription description = module.describe();
    const std::string name(module.name());

    check(description.name == name, name + ": describe() names the module it came from");
    check(description.ops.size() == expected_ops,
          name + ": describes " + std::to_string(expected_ops) + " ops");
    check(description.course.id == 3883 && !description.course.code.empty(),
          name + ": carries the Moodle course id the topic is assessed in");
    check(description.source == "cpp_solver/include/study/" + name + ".hpp",
          name + ": points at its own header, by a repository-relative path");

    bool specs_complete = true;
    for (const auto& op : description.ops) {
        specs_complete = specs_complete && !op.title.empty() && !op.formula.empty() &&
                         op.explain.size() > 120 && !op.outputs.empty();
        for (const auto& param : op.params) {
            specs_complete = specs_complete && !param.type.empty() && !param.label.empty() &&
                             !param.default_value.is_null();
            if (param.type == "enum") {
                specs_complete = specs_complete && !param.options.empty();
            }
            if (param.type == "scalar" || param.type == "int" || param.type == "vec3" ||
                param.type == "vec6") {
                specs_complete = specs_complete && param.has_range && param.min < param.max;
            }
        }
        for (const auto& output : op.outputs) {
            specs_complete = specs_complete && !output.type.empty() && !output.label.empty();
        }
    }
    check(specs_complete,
          name + ": every op carries a title, a formula, an explanation, ranged numeric params "
                 "and named outputs");

    // The description has to survive the wire: dump, parse, compare.
    const Value described = description.to_json();
    bool round_trips = false;
    try {
        round_trips = (json::parse(json::dump(described)) == described);
    } catch (const std::exception& error) {
        std::cout << "       parse threw " << error.what() << "\n";
    }
    check(round_trips, name + ": its description survives a JSON round trip");

    // Every described op must be invocable with nothing but its own defaults.
    bool all_invocable = true;
    for (const auto& op : description.ops) {
        try {
            const Value result = module.invoke(op.name, args_from_defaults(op));
            if (!result.is_object() || result.empty()) {
                all_invocable = false;
                std::cout << "       op " << op.name << " returned no object\n";
            }
            for (const auto& output : op.outputs) {
                if (!result.contains(output.name)) {
                    all_invocable = false;
                    std::cout << "       op " << op.name << " never produced its declared output '"
                              << output.name << "'\n";
                }
            }
            const std::string text = json::dump(result);
            if (json::parse(text) != result) {
                all_invocable = false;
                std::cout << "       op " << op.name << " does not survive a JSON round trip\n";
            }
        } catch (const std::exception& error) {
            all_invocable = false;
            std::cout << "       op " << op.name << " threw " << error.what() << "\n";
        }
    }
    check(all_invocable,
          name + ": every op runs on its own declared defaults and produces every output it "
                 "declares");

    // An unknown op is an error, not a crash.
    bool unknown_rejected = false;
    try {
        (void)module.invoke("no_such_op", Value::object());
    } catch (const study::StudyError&) {
        unknown_rejected = true;
    } catch (...) {
        unknown_rejected = false;
    }
    check(unknown_rejected, name + ": an unknown op throws StudyError");

    // A missing required parameter is an error too.
    bool missing_rejected = false;
    try {
        (void)module.invoke(description.ops.front().name, Value::object());
    } catch (const study::StudyError&) {
        missing_rejected = true;
    } catch (...) {
        missing_rejected = false;
    }
    check(missing_rejected || description.ops.front().params.empty(),
          name + ": a request with no parameters at all is rejected with StudyError");
}

void test_modules_through_registry() {
    std::cout << "\n--- both modules through the registry ---\n";

    study::DynamicsModule dynamics;
    study::TrajectoryProfilesModule profiles;
    exercise_module(dynamics, 6);
    exercise_module(profiles, 6);

    study::ModuleRegistry registry;
    registry.add(std::make_unique<study::DynamicsModule>());
    registry.add(std::make_unique<study::TrajectoryProfilesModule>());
    check(registry.size() == 2 && registry.find("dynamics") != nullptr &&
              registry.find("trajectory_profiles") != nullptr,
          "both modules register and are found by name");

    Value request = Value::object();
    request.set("id", Value(11));
    request.set("module", Value("dynamics"));
    request.set("op", Value("inverse_dynamics"));
    Value args = Value::object();
    args.set("q", json::from_vec6(Vector6::Zero()));
    args.set("qdot", json::from_vec6(Vector6::Constant(0.5)));
    args.set("qddot", json::from_vec6(Vector6::Constant(1.0)));
    args.set("formulation", Value("newton_euler"));
    request.set("args", args);
    const Value response = registry.handle(request);
    check(response["ok"].is_bool() && response["ok"].as_bool() && response["id"].as_int() == 11,
          "the registry answers an inverse_dynamics request with ok: true and the echoed id");
    const Value& result = response["result"];
    check(result["max_disagreement"].is_number() && result["max_disagreement"].as_double() < 1e-8,
          "and the response itself reports the two formulations agreeing to better than 1e-8 N m");

    // A bad payload is a failure response, not a thrown exception.
    Value bad = request;
    Value bad_args = args;
    bad_args.set("payload", Value(42.0));  // the GP8 is rated for 8 kg
    bad.set("args", bad_args);
    const Value bad_response = registry.handle(bad);
    check(bad_response["ok"].is_bool() && !bad_response["ok"].as_bool() &&
              bad_response["error"].is_string() &&
              bad_response["error"].as_string().find("payload") != std::string::npos,
          "an out-of-range payload comes back as ok: false with the parameter named");

    const Value catalogue = registry.describe_all();
    check(catalogue["modules"].size() == 2 && catalogue["courses"].size() == 1,
          "the catalogue lists both modules under the one course they belong to");
}

// ---------------------------------------------------------------------------
// The two modules against each other: torque feasibility of a planned move
// ---------------------------------------------------------------------------

void test_cross_module() {
    std::cout << "\n--- trajectory feasibility against the dynamics ---\n";

    Vector6 q0 = Vector6::Zero();
    Vector6 q1 = Vector6::Zero();
    q1(1) = 0.5 * kPi;
    const traj::Samples samples =
        traj::sample_polynomial(traj::quintic_profile(q0, q1, 2.0, 0.0, 0.0, 0.0, 0.0), 200,
                                "quintic");

    double worst_disagreement = 0.0;
    double worst_torque = 0.0;
    for (const traj::Sample& s : samples.points) {
        const Vector6 lagrangian = dyn::inverse_dynamics_lagrangian(s.q, s.qd, s.qdd, 5.0, true);
        const Vector6 newton = dyn::inverse_dynamics_newton_euler(s.q, s.qd, s.qdd, 5.0, true);
        worst_disagreement =
            std::max(worst_disagreement, (lagrangian - newton).cwiseAbs().maxCoeff());
        worst_torque = std::max(worst_torque, lagrangian.cwiseAbs().maxCoeff());
    }
    std::cout << "       peak torque along the planned move with 5 kg: " << worst_torque
              << " N m\n";
    check_below(worst_disagreement, 1e-8,
                "the two formulations also agree at every sample of a planned trajectory");
    check(worst_torque > 1.0,
          "the planned move actually loads the machine, so the comparison is not against zero");
}

}  // namespace

int main() {
    std::cout << "====================================================\n";
    std::cout << "   RUNNING GP8 DYNAMICS AND TRAJECTORY TEST SUITE   \n";
    std::cout << "====================================================\n";

    double agreement = 0.0;
    double energy_drift = 0.0;
    try {
        test_mass_matrix();
        agreement = test_formulation_agreement();
        test_forward_dynamics();
        test_gravity();
        energy_drift = test_energy_conservation();
        test_polynomial_profiles();
        test_lspb();
        test_via_points();
        test_constraint_check();
        test_cartesian_lspb();
        test_modules_through_registry();
        test_cross_module();
    } catch (const std::exception& error) {
        std::cout << "[FAIL] an exception escaped a test: " << error.what() << "\n";
        ++g_failures;
        g_failed_names.emplace_back("unexpected exception");
    }

    std::cout << "====================================================\n";
    std::cout << "Lagrangian vs Newton-Euler worst disagreement: " << agreement << " N m\n";
    std::cout << "Energy drift over 0.5 s, dt = 2e-5 s, leapfrog: " << energy_drift << " J\n";
    std::cout << "checks run: " << g_checks << ", failed: " << g_failures << "\n";
    if (g_failures != 0) {
        for (const auto& name : g_failed_names) {
            std::cout << "FAILED: " << name << "\n";
        }
        std::cout << "STUDY DYNAMICS TESTS FAILED\n";
        return 1;
    }
    std::cout << "          ALL TESTS PASSED                          \n";
    std::cout << "====================================================\n";
    return 0;
}
