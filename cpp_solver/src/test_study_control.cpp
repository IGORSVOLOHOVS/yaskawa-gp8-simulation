// Test suite for the control_system and digital_control study modules.
//
// Plain asserts in the style of src/test_study_modules.cpp - there is no gtest
// in this project and there will not be one.
//
// Every check asserts a property against an independent route: polynomial
// arithmetic against hand-computed coefficients, a second-order step response
// against the closed-form overshoot and settling formulas, Routh-Hurwitz
// against the actual root locations, the Nyquist encirclement verdict against
// the pole-count verdict, Bode margins against the analytic crossover algebra,
// and the discrete models against the continuous one as the sample period
// shrinks. No test asserts a printed string.

#include "study/control_system.hpp"
#include "study/digital_control.hpp"
#include "study/gp8_model.hpp"
#include "study/json.hpp"
#include "study/study_module.hpp"

#include <Eigen/Dense>

#include <algorithm>
#include <cmath>
#include <complex>
#include <iostream>
#include <limits>
#include <numbers>
#include <random>
#include <string>
#include <vector>

namespace json = yaskawa::study::json;
namespace study = yaskawa::study;
namespace control = yaskawa::study::control;
namespace digital = yaskawa::study::digital;

using control::Complex;
using control::Poly;
using control::TransferFunction;

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

void check_near(double actual, double expected, double tolerance, const std::string& what) {
    const double error = std::abs(actual - expected);
    const bool ok = std::isfinite(error) && error <= tolerance;
    if (!ok) {
        std::cout << "       expected " << expected << ", got " << actual << " (error " << error
                  << " > " << tolerance << ")\n";
    }
    check(ok, what);
}

[[nodiscard]] bool poly_equal(const Poly& a, const Poly& b, double tolerance) {
    if (a.size() != b.size()) {
        return false;
    }
    for (std::size_t i = 0; i < a.size(); ++i) {
        if (std::abs(a[i] - b[i]) > tolerance) {
            return false;
        }
    }
    return true;
}

// Sorted real parts, so a root set can be compared without caring about order.
[[nodiscard]] std::vector<double> sorted_real_parts(const std::vector<Complex>& values) {
    std::vector<double> out;
    out.reserve(values.size());
    for (const auto& v : values) {
        out.push_back(v.real());
    }
    std::sort(out.begin(), out.end());
    return out;
}

[[nodiscard]] int rhp_root_count(const Poly& p, double tolerance = 1e-9) {
    int count = 0;
    for (const auto& root : control::poly_roots(p)) {
        if (root.real() > tolerance) {
            ++count;
        }
    }
    return count;
}

[[nodiscard]] bool has_axis_root(const Poly& p, double tolerance = 1e-7) {
    for (const auto& root : control::poly_roots(p)) {
        if (std::abs(root.real()) <= tolerance) {
            return true;
        }
    }
    return false;
}

// The gain at which a unity-feedback loop first loses stability, found from the
// roots. The module computes the same number; this is the independent route.
[[nodiscard]] double first_unstable_gain(const TransferFunction& base, double low, double high) {
    auto stable_at = [&](double k) {
        const Poly characteristic =
            control::poly_add(control::poly_scale(base.numerator(), k), base.denominator());
        for (const auto& root : control::poly_roots(characteristic)) {
            if (root.real() >= -1e-12) {
                return false;
            }
        }
        return true;
    };
    if (!stable_at(low)) {
        return 0.0;
    }
    double bracket_low = low;
    double bracket_high = 0.0;
    for (int i = 1; i <= 400; ++i) {
        const double frac = static_cast<double>(i) / 400.0;
        const double k =
            std::pow(10.0, std::log10(low) + frac * (std::log10(high) - std::log10(low)));
        if (!stable_at(k)) {
            bracket_high = k;
            break;
        }
        bracket_low = k;
    }
    if (bracket_high == 0.0) {
        return 0.0;
    }
    for (int i = 0; i < 80; ++i) {
        const double mid = 0.5 * (bracket_low + bracket_high);
        if (stable_at(mid)) {
            bracket_low = mid;
        } else {
            bracket_high = mid;
        }
    }
    return bracket_high;
}

// ---------------------------------------------------------------------------
// Polynomials and the transfer-function core
// ---------------------------------------------------------------------------

void test_polynomials() {
    std::cout << "\n--- polynomial arithmetic ---\n";

    // (2s + 3)(s^2 - s + 4) = 2s^3 + s^2 + 5s + 12, multiplied out by hand.
    const Poly a{2.0, 3.0};
    const Poly b{1.0, -1.0, 4.0};
    check(poly_equal(control::poly_multiply(a, b), Poly{2.0, 1.0, 5.0, 12.0}, 1e-12),
          "poly_multiply (2s+3)(s^2-s+4) = 2s^3 + s^2 + 5s + 12");

    // Addition aligns at the lowest power, not the highest.
    check(poly_equal(control::poly_add(Poly{1.0, 2.0, 3.0}, Poly{5.0, 7.0}),
                     Poly{1.0, 7.0, 10.0}, 1e-12),
          "poly_add s^2+2s+3 and 5s+7 = s^2 + 7s + 10");
    // Subtraction keeps the degree it was given - it is poly_trim's job, not
    // poly_subtract's, to decide a polynomial has collapsed.
    check(poly_equal(control::poly_subtract(Poly{1.0, 2.0, 3.0}, Poly{1.0, 2.0, 3.0}),
                     Poly{0.0, 0.0, 0.0}, 1e-12) &&
              poly_equal(control::poly_trim(
                             control::poly_subtract(Poly{1.0, 2.0, 3.0}, Poly{1.0, 2.0, 3.0})),
                         Poly{0.0}, 1e-12),
          "poly_subtract of equal polynomials is all zeros, and poly_trim collapses it");

    // (s + 1)^3 = s^3 + 3s^2 + 3s + 1.
    check(poly_equal(control::poly_power(Poly{1.0, 1.0}, 3), Poly{1.0, 3.0, 3.0, 1.0}, 1e-12),
          "poly_power (s+1)^3 = s^3 + 3s^2 + 3s + 1");

    // Horner against a hand evaluation: 2(2)^3 + 1(2)^2 + 5(2) + 12 = 42.
    check_near(control::poly_eval(Poly{2.0, 1.0, 5.0, 12.0}, Complex(2.0, 0.0)).real(), 42.0, 1e-12,
               "poly_eval 2s^3+s^2+5s+12 at s = 2 is 42");

    // (s+1)(s+2)(s+3) = s^3 + 6s^2 + 11s + 6, so the roots must come back.
    const std::vector<double> roots = sorted_real_parts(
        control::poly_roots(Poly{1.0, 6.0, 11.0, 6.0}));
    bool roots_ok = roots.size() == 3;
    if (roots_ok) {
        roots_ok = std::abs(roots[0] + 3.0) < 1e-8 && std::abs(roots[1] + 2.0) < 1e-8 &&
                   std::abs(roots[2] + 1.0) < 1e-8;
    }
    check(roots_ok, "poly_roots of s^3+6s^2+11s+6 are -3, -2, -1");

    // A polynomial reconstructed from its own roots must come back unchanged.
    {
        const Poly original{1.0, 4.0, 6.0, 4.0, 1.0};  // (s+1)^4
        const Poly rebuilt = digital::poly_from_roots(control::poly_roots(original));
        check(poly_equal(rebuilt, original, 1e-6),
              "poly_from_roots inverts poly_roots on (s+1)^4");
    }

    // Composition rules, each against the algebra done by hand.
    const TransferFunction g({1.0}, {1.0, 1.0});   // 1/(s+1)
    const TransferFunction h({2.0}, {1.0, 3.0});   // 2/(s+3)
    check(poly_equal(g.series(h).denominator(), Poly{1.0, 4.0, 3.0}, 1e-12) &&
              poly_equal(g.series(h).numerator(), Poly{2.0}, 1e-12),
          "series of 1/(s+1) and 2/(s+3) is 2/(s^2+4s+3)");
    check(poly_equal(g.parallel(h).numerator(), Poly{3.0, 5.0}, 1e-12),
          "parallel of 1/(s+1) and 2/(s+3) has numerator 3s+5");
    check(poly_equal(g.feedback(h, true).denominator(), Poly{1.0, 4.0, 5.0}, 1e-12),
          "negative feedback puts (s+1)(s+3) + 2 = s^2+4s+5 in the denominator");
    check(poly_equal(g.feedback(h, false).denominator(), Poly{1.0, 4.0, 1.0}, 1e-12),
          "positive feedback puts (s+1)(s+3) - 2 = s^2+4s+1 in the denominator");

    // The state-space realisation must reproduce the transfer function it came
    // from, both in its characteristic polynomial and pointwise.
    {
        const TransferFunction tf({2.0, 1.0}, {1.0, 5.0, 6.0, 0.0});
        const control::StateSpace ss = tf.state_space();
        const control::CharacteristicPoly cp = control::characteristic_poly(ss.A);
        check(poly_equal(cp.coefficients, Poly{1.0, 5.0, 6.0, 0.0}, 1e-9),
              "characteristic_poly of the realisation reproduces the denominator");
        const Complex s(0.7, 1.3);
        const Eigen::Index n = ss.A.rows();
        const Eigen::MatrixXcd resolvent =
            (s * Eigen::MatrixXcd::Identity(n, n) - ss.A.cast<Complex>()).inverse();
        const Complex from_state_space =
            (ss.C.cast<Complex>() * resolvent * ss.B.cast<Complex>())(0) + ss.D;
        check(std::abs(from_state_space - tf.evaluate(s)) < 1e-9,
              "C(sI-A)^-1 B + D equals the transfer function at a complex s");
    }

    // e^0 = I and e^(diag) = diag(e^.), so the exponential is not guesswork.
    {
        Eigen::MatrixXd a(2, 2);
        a << -1.0, 0.0, 0.0, -4.0;
        const Eigen::MatrixXd e = control::matrix_exponential(a);
        check(std::abs(e(0, 0) - std::exp(-1.0)) < 1e-12 &&
                  std::abs(e(1, 1) - std::exp(-4.0)) < 1e-12 && std::abs(e(0, 1)) < 1e-14,
              "matrix_exponential of a diagonal matrix is the exponential of the diagonal");
    }
}

// ---------------------------------------------------------------------------
// The highest-value check: a known second-order system against the closed form
// ---------------------------------------------------------------------------

void test_second_order_closed_form() {
    std::cout << "\n--- second-order step response against the closed form ---\n";

    struct Case {
        double zeta;
        double omega_n;
    };
    const std::vector<Case> cases = {{0.1, 2.0},  {0.2, 5.0},   {0.3, 1.0},
                                     {0.5, 10.0}, {0.707, 4.0}, {0.8, 20.0}};

    for (const auto& c : cases) {
        const TransferFunction tf = TransferFunction::second_order(1.0, c.zeta, c.omega_n);
        const double damped = c.omega_n * std::sqrt(1.0 - c.zeta * c.zeta);
        const double duration = 14.0 / (c.zeta * c.omega_n);
        const std::size_t samples = 8001;
        const control::Response response = tf.step_response(duration, samples);
        const control::TransientSpec spec = control::transient_spec(response, 1.0, 1.0, 0.02);

        const std::string tag = "zeta = " + json::number_to_string(c.zeta) + ", omega_n = " +
                                json::number_to_string(c.omega_n);

        // Overshoot: sigma = 100 exp(-pi zeta / sqrt(1 - zeta^2)), exact.
        const double expected_overshoot =
            100.0 * std::exp(-kPi * c.zeta / std::sqrt(1.0 - c.zeta * c.zeta));
        check_near(spec.overshoot_percent, expected_overshoot, 0.2,
                   "overshoot matches 100 exp(-pi zeta / sqrt(1-zeta^2)) to 0.2 percentage "
                   "points (" + tag + ")");

        // Peak time: t_p = pi / omega_d, exact.
        check_near(spec.peak_time, kPi / damped, 0.01 * kPi / damped,
                   "peak time matches pi / omega_d to 1 % (" + tag + ")");

        // Rise time: the linear 10-90 % approximation (2.16 zeta + 0.60)/omega_n
        // is itself only a curve fit, and only over 0.3 <= zeta <= 0.8, so it
        // is asserted exactly where it is valid and nowhere else.
        if (c.zeta >= 0.3 && c.zeta <= 0.8) {
            const double expected_rise = (2.16 * c.zeta + 0.60) / c.omega_n;
            check_near(spec.rise_time, expected_rise, 0.12 * expected_rise,
                       "rise time matches (2.16 zeta + 0.60)/omega_n to 12 %, inside that fit's "
                       "0.3 <= zeta <= 0.8 validity range (" + tag + ")");
        } else {
            // Outside the fit's range the measurement still has to be a real
            // rise time: positive, and shorter than the time to the peak.
            check(spec.rise_measured && spec.rise_time > 0.0 &&
                      spec.rise_time < spec.peak_time,
                  "rise time is measured and precedes the peak (" + tag + ")");
        }

        // Settling time: the exponential envelope leaves the 2 % band at
        // t = -ln(0.02 sqrt(1-zeta^2)) / (zeta omega_n). The measured value is
        // the last band exit, which lands within one damped half period of the
        // envelope estimate - that is the stated tolerance.
        const double envelope =
            -std::log(0.02 * std::sqrt(1.0 - c.zeta * c.zeta)) / (c.zeta * c.omega_n);
        check_near(spec.settling_time, envelope, kPi / damped,
                   "settling time matches the 2 % exponential envelope to within one damped "
                   "half period (" + tag + ")");

        // The same system through the pole locations, which must agree with the
        // zeta and omega_n it was built from.
        const std::vector<Complex> poles = tf.poles();
        bool poles_ok = poles.size() == 2;
        if (poles_ok) {
            const double magnitude = std::abs(poles[0]);
            poles_ok = std::abs(magnitude - c.omega_n) < 1e-9 &&
                       std::abs(-poles[0].real() / magnitude - c.zeta) < 1e-9;
        }
        check(poles_ok, "the pole pair carries back the zeta and omega_n it was built from (" +
                            tag + ")");
        check(spec.settled && std::abs(spec.steady_state_error) < 1e-6,
              "the unit-gain second-order loop settles on its setpoint (" + tag + ")");
    }
}

// ---------------------------------------------------------------------------
// Routh-Hurwitz against the actual roots
// ---------------------------------------------------------------------------

void test_routh_against_roots() {
    std::cout << "\n--- Routh-Hurwitz against the root locations ---\n";

    // Hand-worked cases first.
    check(control::routh_array(Poly{1.0, 6.0, 11.0, 6.0}).stable,
          "Routh calls (s+1)(s+2)(s+3) stable");
    check(!control::routh_array(Poly{1.0, 1.0, 1.0, 6.0}).stable,
          "Routh calls s^3+s^2+s+6 unstable");
    check(control::routh_array(Poly{1.0, 1.0, 1.0, 6.0}).sign_changes ==
              rhp_root_count(Poly{1.0, 1.0, 1.0, 6.0}),
          "the sign-change count of s^3+s^2+s+6 equals its right-half-plane root count");
    // A whole row vanishes: s^3 + 2s^2 + 4s + 8 has the pair +/- 2j.
    {
        const control::RouthArray marginal = control::routh_array(Poly{1.0, 2.0, 4.0, 8.0});
        check(!marginal.stable && marginal.auxiliary_used,
              "Routh detects the vanished row of s^3+2s^2+4s+8 and refuses to call it stable");
    }

    std::mt19937 rng(20261010u);
    std::uniform_real_distribution<double> magnitude(0.2, 6.0);
    std::uniform_int_distribution<int> degree_pick(2, 6);
    std::uniform_int_distribution<int> side(0, 3);

    int agreements = 0;
    int trials = 0;
    int marginal_detected = 0;
    int marginal_trials = 0;

    for (int trial = 0; trial < 220; ++trial) {
        const int degree = degree_pick(rng);
        std::vector<Complex> roots;
        roots.reserve(static_cast<std::size_t>(degree));
        while (static_cast<int>(roots.size()) < degree) {
            const int which = side(rng);
            if (which == 0 && static_cast<int>(roots.size()) + 1 < degree) {
                // A complex pair in the left half plane.
                const double re = -magnitude(rng);
                const double im = magnitude(rng);
                roots.emplace_back(re, im);
                roots.emplace_back(re, -im);
            } else if (which == 1 && static_cast<int>(roots.size()) + 1 < degree) {
                // A complex pair in the right half plane.
                const double re = magnitude(rng);
                const double im = magnitude(rng);
                roots.emplace_back(re, im);
                roots.emplace_back(re, -im);
            } else if (which == 2) {
                roots.emplace_back(-magnitude(rng), 0.0);
            } else {
                roots.emplace_back(magnitude(rng), 0.0);
            }
        }
        Poly characteristic = digital::poly_from_roots(roots);
        if (characteristic[0] < 0.0) {
            characteristic = control::poly_scale(characteristic, -1.0);
        }
        if (has_axis_root(characteristic, 1e-6)) {
            continue;
        }
        const control::RouthArray routh = control::routh_array(characteristic);
        const int expected = rhp_root_count(characteristic);
        ++trials;
        if (routh.sign_changes == expected && routh.stable == (expected == 0)) {
            ++agreements;
        } else {
            std::cout << "       disagreement: Routh says " << routh.sign_changes
                      << " sign changes, the roots say " << expected << " RHP roots\n";
        }

        // The same root set with an imaginary pair bolted on: marginal, and
        // never asymptotically stable whatever the rest of the roots do.
        std::vector<Complex> marginal_roots = roots;
        const double omega = magnitude(rng);
        marginal_roots.emplace_back(0.0, omega);
        marginal_roots.emplace_back(0.0, -omega);
        Poly marginal = digital::poly_from_roots(marginal_roots);
        if (marginal[0] < 0.0) {
            marginal = control::poly_scale(marginal, -1.0);
        }
        ++marginal_trials;
        if (!control::routh_array(marginal).stable) {
            ++marginal_detected;
        }
    }

    check(trials > 150, "the random Routh sweep produced a usable number of trials");
    check(agreements == trials,
          "Routh-Hurwitz agrees with the actual root count on all " + std::to_string(trials) +
              " random polynomials of degree 2 to 6");
    check(marginal_detected == marginal_trials,
          "every one of the " + std::to_string(marginal_trials) +
              " marginal polynomials (a pair of roots on the imaginary axis) is refused");
}

// ---------------------------------------------------------------------------
// Nyquist against the pole-count verdict
// ---------------------------------------------------------------------------

void test_nyquist_against_poles() {
    std::cout << "\n--- Nyquist encirclements against the closed-loop poles ---\n";

    const study::control::JointPlant plant =
        control::build_joint_plant(1, Eigen::Matrix<double, 6, 1>::Zero(), 0.0, 0.25, 1.0);

    std::vector<TransferFunction> loops;
    loops.push_back(TransferFunction({1.0}, {1.0, 1.0}));                       // stable, P = 0
    loops.push_back(TransferFunction({5.0}, {1.0, 1.0}));
    loops.push_back(TransferFunction({-2.0}, {1.0, 1.0}));                      // Z = 1
    loops.push_back(TransferFunction({2.0}, {1.0, -1.0}));                      // P = 1, stable
    loops.push_back(TransferFunction({0.5}, {1.0, -1.0}));                      // P = 1, unstable
    loops.push_back(TransferFunction({1.0}, {1.0, 3.0, 3.0, 1.0}));             // 1/(s+1)^3
    loops.push_back(TransferFunction({20.0}, {1.0, 3.0, 3.0, 1.0}));            // over the limit
    loops.push_back(plant.tf);                                                  // the real joint
    loops.push_back(plant.tf.scaled(40.0));
    loops.push_back(plant.tf.series(TransferFunction::pade_delay(0.004)));
    loops.push_back(plant.tf.series(TransferFunction::pade_delay(0.004)).scaled(30.0));
    loops.push_back(plant.tf.series(TransferFunction::pade_delay(0.05)).scaled(60.0));

    int agreements = 0;
    for (std::size_t i = 0; i < loops.size(); ++i) {
        const control::NyquistResult result = control::nyquist(loops[i], 1e-4, 1e5, 4000);
        const bool by_poles = loops[i].closed_loop().is_stable();
        if (result.stable == by_poles) {
            ++agreements;
        } else {
            std::cout << "       loop " << i << ": Nyquist says " << result.stable
                      << " (N = " << result.encirclements << ", P = " << result.open_loop_rhp_poles
                      << "), the poles say " << by_poles << "\n";
        }
    }
    check(agreements == static_cast<int>(loops.size()),
          "the Nyquist verdict Z = N + P agrees with the closed-loop pole locations on all " +
              std::to_string(loops.size()) + " loops, including two with an unstable plant");

    // P is a count of open-loop right-half-plane poles, not a free parameter.
    check(control::nyquist(TransferFunction({2.0}, {1.0, -1.0}), 1e-4, 1e5, 2000)
                  .open_loop_rhp_poles == 1,
          "Nyquist counts the one right-half-plane pole of 2/(s-1)");
}

// ---------------------------------------------------------------------------
// Bode margins against the analytic crossover algebra
// ---------------------------------------------------------------------------

void test_margins_against_analytic() {
    std::cout << "\n--- margins from the Bode data against the analytic values ---\n";

    // L = K/(s(Ts+1)): the magnitude crosses 0 dB where K^2 = w^2 (1 + T^2w^2),
    // which solves in closed form, and the phase there gives the margin.
    const std::vector<std::pair<double, double>> first_cases = {{5.0, 0.2}, {20.0, 0.05},
                                                                {17.3, 0.85}};
    for (const auto& [k, t] : first_cases) {
        const TransferFunction loop({k}, {t, 1.0, 0.0});
        const control::Margins m = control::loop_margins(loop, 1e-3, 1e5, 6000);
        const double omega_squared =
            (-1.0 + std::sqrt(1.0 + 4.0 * k * k * t * t)) / (2.0 * t * t);
        const double omega = std::sqrt(omega_squared);
        const double expected_pm = 90.0 - std::atan(t * omega) * 180.0 / kPi;
        const std::string tag = "K = " + json::number_to_string(k) + ", T = " +
                                json::number_to_string(t);
        check(m.has_phase_margin, "K/(s(Ts+1)) has a gain crossover (" + tag + ")");
        check_near(m.gain_crossover_omega, omega, 0.01 * omega,
                   "the gain crossover frequency matches the closed form to 1 % (" + tag + ")");
        check_near(m.phase_margin_deg, expected_pm, 0.5,
                   "the phase margin matches 90 - atan(T w_c) to 0.5 degrees (" + tag + ")");
        check(!m.has_gain_margin,
              "a second-order type-1 loop never reaches -180 degrees, so it reports no gain "
              "margin (" + tag + ")");
    }

    // L = K/(s(T1 s+1)(T2 s+1)): the phase hits -180 degrees exactly where
    // T1 T2 w^2 = 1, so both the crossover and the gain margin are analytic.
    const std::vector<std::array<double, 3>> second_cases = {{10.0, 0.1, 0.02},
                                                             {4.0, 0.5, 0.05},
                                                             {50.0, 0.2, 0.01}};
    for (const auto& c : second_cases) {
        const double k = c[0];
        const double t1 = c[1];
        const double t2 = c[2];
        const TransferFunction loop({k}, control::poly_multiply(Poly{t1 * t2, t1 + t2, 1.0},
                                                                Poly{1.0, 0.0}));
        const control::Margins m = control::loop_margins(loop, 1e-3, 1e5, 8000);
        const double omega = 1.0 / std::sqrt(t1 * t2);
        const double magnitude = std::abs(loop.frequency_response(omega));
        const double expected_gm_db = -20.0 * std::log10(magnitude);
        const std::string tag = "K = " + json::number_to_string(k) + ", T1 = " +
                                json::number_to_string(t1) + ", T2 = " +
                                json::number_to_string(t2);
        check(m.has_gain_margin, "the third-order loop reaches -180 degrees (" + tag + ")");
        check_near(m.phase_crossover_omega, omega, 0.01 * omega,
                   "the phase crossover matches 1/sqrt(T1 T2) to 1 % (" + tag + ")");
        check_near(m.gain_margin_db, expected_gm_db, 0.2,
                   "the gain margin matches -20 log10 |L(j w_180)| to 0.2 dB (" + tag + ")");

        // A gain margin is a prediction: raising the gain by exactly that
        // factor has to put the closed-loop poles on the imaginary axis.
        const double critical = first_unstable_gain(loop, 1e-3, 1e4);
        check_near(m.gain_margin_linear, critical, 0.03 * critical,
                   "the linear gain margin equals the gain at which the roots reach the axis, to "
                   "3 % (" + tag + ")");
    }
}

// ---------------------------------------------------------------------------
// The GP8 joint plant and a Ziegler-Nichols loop on it
// ---------------------------------------------------------------------------

void test_joint_plant_and_tuning() {
    std::cout << "\n--- the GP8 joint plant and Ziegler-Nichols on it ---\n";

    const Eigen::Matrix<double, 6, 1> home = Eigen::Matrix<double, 6, 1>::Zero();
    const control::JointPlant plant = control::build_joint_plant(1, home, 0.0, 0.25, 1.0);

    std::cout << "       joint 2 plant: W(s) = " << plant.tf.to_string() << "\n";
    std::cout << "       J_link = " << plant.link_inertia
              << " kg m^2, n^2 J_rotor = " << plant.reflected_inertia
              << " kg m^2, J_tot = " << plant.total_inertia << " kg m^2\n";
    std::cout << "       b_eq = " << plant.damping << " N m s/rad, tau_drive = "
              << plant.drive_torque << " N m, K = " << plant.dc_gain << " rad/s per command, T = "
              << plant.time_constant << " s\n";

    check(plant.link_inertia > 0.0 && plant.reflected_inertia > 0.0,
          "the joint-2 inertia is positive both from the links and from the rotor");
    check_near(plant.reflected_inertia,
               study::GP8_LINKS[1].gear_ratio * study::GP8_LINKS[1].gear_ratio *
                   study::GP8_LINKS[1].rotor_inertia,
               1e-12, "the reflected inertia is exactly n^2 J_rotor from gp8_model.hpp");
    check_near(plant.time_constant, plant.total_inertia / plant.damping, 1e-12,
               "T = J_tot / b_eq exactly");
    check_near(plant.dc_gain, plant.drive_torque / plant.damping, 1e-12,
               "K = tau_drive / b_eq exactly");
    check_near(plant.coulomb_equivalent, plant.coulomb / plant.omega_reference, 1e-12,
               "the Coulomb torque is linearised as tau_c / omega_ref");
    check_near(plant.omega_reference, 0.25 * study::joint_max_velocity(1), 1e-12,
               "omega_ref is a fraction of the axis speed rating from gp8_model.hpp");

    // The plant must be K/(s(Ts+1)): one pole at the origin, one at -1/T.
    const std::vector<double> poles = sorted_real_parts(plant.tf.poles());
    bool pole_shape = poles.size() == 2;
    if (pole_shape) {
        pole_shape = std::abs(poles[0] + 1.0 / plant.time_constant) < 1e-6 &&
                     std::abs(poles[1]) < 1e-9;
    }
    check(pole_shape, "the joint plant has exactly one pole at the origin and one at -1/T");

    // A heavier payload can only slow the joint down.
    const control::JointPlant loaded = control::build_joint_plant(1, home, 8.0, 0.25, 1.0);
    check(loaded.total_inertia > plant.total_inertia &&
              loaded.time_constant > plant.time_constant,
          "8 kg at the flange raises the joint-2 inertia and its time constant");

    // Ziegler-Nichols on the ultimate gain, with a realistic 4 ms of loop
    // latency so an ultimate gain exists at all.
    const TransferFunction delayed =
        plant.tf.series(TransferFunction::pade_delay(0.004));
    const double ku = first_unstable_gain(delayed, 1e-3, 1e5);
    check(ku > 0.0, "the joint loop with 4 ms of latency has a finite ultimate gain");
    const double omega_u = [&]() {
        const Poly characteristic = control::poly_add(
            control::poly_scale(delayed.numerator(), ku), delayed.denominator());
        double best = 0.0;
        double closest = std::numeric_limits<double>::max();
        for (const auto& root : control::poly_roots(characteristic)) {
            if (std::abs(root.real()) < closest && std::abs(root.imag()) > 1e-9) {
                closest = std::abs(root.real());
                best = std::abs(root.imag());
            }
        }
        return best;
    }();
    check(omega_u > 0.0, "the ultimate gain comes with a real oscillation frequency");
    const double pu = 2.0 * kPi / omega_u;
    const double kp = 0.6 * ku;
    const double ki = kp / (0.5 * pu);
    const double kd = kp * 0.125 * pu;
    const TransferFunction tuned =
        control::pid_controller(kp, ki, kd, 0.01).series(delayed).closed_loop();
    check(tuned.is_stable(),
          "the Ziegler-Nichols ultimate-gain loop on the real joint-2 plant is stable");
    const control::Response response = tuned.step_response(10.0, 4001);
    const control::TransientSpec spec =
        control::transient_spec(response, 1.0, tuned.dc_gain(), 0.02);
    check(spec.settled && spec.overshoot_percent > 0.0 && spec.overshoot_percent < 100.0,
          "that loop settles, and overshoots as the aggressive Ziegler-Nichols rule predicts");
    std::cout << "       ZN ultimate: Ku = " << ku << ", Pu = " << pu << " s, Kp = " << kp
              << ", Ki = " << ki << ", Kd = " << kd << ", overshoot = "
              << spec.overshoot_percent << " %\n";

    // The error constants of a type-1 loop: zero step error, finite ramp error.
    const TransferFunction proportional_loop = plant.tf.scaled(1.0);
    check(std::abs(proportional_loop.closed_loop().dc_gain() - 1.0) < 1e-9,
          "a type-1 joint loop under proportional control has exactly zero steady-state step "
          "error");
}

// ---------------------------------------------------------------------------
// Discretisation
// ---------------------------------------------------------------------------

void test_discretisation() {
    std::cout << "\n--- discretisation ---\n";

    const control::JointPlant plant =
        control::build_joint_plant(1, Eigen::Matrix<double, 6, 1>::Zero(), 0.0, 0.25, 1.0);
    const TransferFunction closed = plant.tf.scaled(1.0).closed_loop();

    // Bilinear maps the whole left half plane inside the unit circle, for every
    // sample period: that is the property that makes Tustin safe.
    std::mt19937 rng(7u);
    std::uniform_real_distribution<double> real_part(0.05, 30.0);
    std::uniform_real_distribution<double> imag_part(0.0, 30.0);
    std::uniform_real_distribution<double> period_pick(1e-4, 0.5);
    int mapped_inside = 0;
    int mapped_trials = 0;
    for (int trial = 0; trial < 150; ++trial) {
        std::vector<Complex> roots;
        const double re = -real_part(rng);
        const double im = imag_part(rng);
        roots.emplace_back(re, im);
        roots.emplace_back(re, -im);
        roots.emplace_back(-real_part(rng), 0.0);
        const TransferFunction stable_tf(Poly{1.0}, digital::poly_from_roots(roots));
        const double period = period_pick(rng);
        const digital::DiscreteTransferFunction d =
            digital::discretise(stable_tf, period, digital::Method::Bilinear);
        ++mapped_trials;
        if (d.is_stable() && d.spectral_radius() < 1.0) {
            ++mapped_inside;
        } else {
            std::cout << "       bilinear left a pole at |z| = " << d.spectral_radius()
                      << " for T = " << period << "\n";
        }
    }
    check(mapped_inside == mapped_trials,
          "bilinear discretisation maps every left-half-plane pole inside the unit circle, on all " +
              std::to_string(mapped_trials) + " random third-order plants and sample periods");

    // And it maps the right half plane outside, which is the other half of the
    // same claim: a bad plant must not be discretised into a good model.
    {
        const TransferFunction unstable(Poly{1.0}, Poly{1.0, -3.0, 2.0});  // poles +1 and +2
        bool all_outside = true;
        for (const double period : {1e-3, 1e-2, 0.1, 0.5}) {
            const digital::DiscreteTransferFunction d =
                digital::discretise(unstable, period, digital::Method::Bilinear);
            for (const auto& pole : d.poles()) {
                if (std::abs(pole) <= 1.0) {
                    all_outside = false;
                }
            }
        }
        check(all_outside,
              "bilinear discretisation keeps right-half-plane poles outside the unit circle");
    }

    // Zero-order hold is exact for a held input, so its poles must be exactly
    // e^{s T} - checked against the continuous poles directly.
    {
        const double period = 0.01;
        const digital::DiscreteTransferFunction d =
            digital::discretise(closed, period, digital::Method::ZeroOrderHold);
        const std::vector<Complex> s_poles = closed.poles();
        const std::vector<double> expected = [&]() {
            std::vector<double> out;
            for (const auto& p : s_poles) {
                out.push_back(std::abs(std::exp(p * period)));
            }
            std::sort(out.begin(), out.end());
            return out;
        }();
        std::vector<double> actual;
        for (const auto& p : d.poles()) {
            actual.push_back(std::abs(p));
        }
        std::sort(actual.begin(), actual.end());
        bool ok = actual.size() == expected.size();
        for (std::size_t i = 0; ok && i < actual.size(); ++i) {
            ok = std::abs(actual[i] - expected[i]) < 1e-9;
        }
        check(ok, "zero-order-hold poles are exactly e^{s T} of the continuous poles");
        check(d.dc_gain() > 0.0 && std::abs(d.dc_gain() - closed.dc_gain()) < 1e-6,
              "the zero-order-hold model keeps the dc gain of the continuous system");
    }

    // The difference equation must reproduce the discrete transfer function's
    // own step response, which is the only thing a controller can execute.
    {
        const digital::DiscreteTransferFunction d =
            digital::discretise(closed, 0.01, digital::Method::ZeroOrderHold);
        // 4000 samples at 10 ms is 40 s, which is several settling times of
        // this loop - a shorter run would only be testing the transient.
        const std::vector<double> step = d.step_response(4000);
        check(std::abs(step.back() - d.dc_gain()) < 1e-3,
              "the difference equation's step response converges on the z = 1 gain");
        check(!d.difference_equation().empty() && d.difference_equation().front() == 'y',
              "the difference equation is emitted in executable y[k] = ... form");
    }

    // Convergence: as T shrinks, every method's sampled step response has to
    // approach the continuous one, and a second-order method has to do it
    // faster than first order.
    for (const auto method : {digital::Method::Bilinear, digital::Method::ZeroOrderHold,
                              digital::Method::BackwardEuler}) {
        std::vector<double> errors;
        for (const double period : {0.2, 0.1, 0.05, 0.025}) {
            const std::size_t steps = static_cast<std::size_t>(std::floor(12.0 / period)) + 1;
            const double aligned = static_cast<double>(steps - 1) * period;
            const control::Response sampled = closed.step_response(aligned, steps);
            const std::vector<double> discrete =
                digital::discretise(closed, period, method).step_response(steps);
            double worst = 0.0;
            for (std::size_t i = 0; i < steps; ++i) {
                worst = std::max(worst, std::abs(discrete[i] - sampled.y[i]));
            }
            errors.push_back(worst);
        }
        bool monotone = true;
        for (std::size_t i = 1; i < errors.size(); ++i) {
            if (!(errors[i] < errors[i - 1])) {
                monotone = false;
            }
        }
        const std::string name = digital::method_name(method);
        check(monotone, "the " + name +
                            " step response converges on the continuous one as T shrinks");
        // The rate of convergence is the method's order, so each method is held
        // to its own order and not to a single shared number.
        if (method == digital::Method::Bilinear) {
            check(errors.back() < 0.25 * errors.front(),
                  "bilinear is second order, so halving T twice cuts its error at least "
                  "fourfold");
        } else if (method == digital::Method::ZeroOrderHold) {
            check(errors.front() < 1e-5,
                  "zero-order hold is exact for a step, so its error is already at the level of "
                  "the continuous reference's own integration error at T = 0.2 s");
        } else {
            check(errors.back() < 0.5 * errors.front(),
                  "backward Euler is first order, so halving T twice cuts its error at least "
                  "twofold, not fourfold");
        }
        std::cout << "       " << name << " error at T = 0.2, 0.1, 0.05, 0.025 s: " << errors[0]
                  << ", " << errors[1] << ", " << errors[2] << ", " << errors[3] << "\n";
    }

    // Forward Euler is the one method that can invent an unstable model, which
    // is exactly why the course warns about it.
    {
        const TransferFunction fast(Poly{1.0}, Poly{1.0, 100.0});  // pole at -100
        const digital::DiscreteTransferFunction d =
            digital::discretise(fast, 0.05, digital::Method::ForwardEuler);
        check(!d.is_stable(),
              "forward Euler turns a stable pole at -100 into an unstable model at T = 50 ms, "
              "while bilinear does not");
        check(digital::discretise(fast, 0.05, digital::Method::Bilinear).is_stable(),
              "bilinear keeps that same pole stable at the same sample period");
    }

    // Matched poles put the poles exactly where e^{sT} says, by construction.
    {
        const double period = 0.02;
        const digital::DiscreteTransferFunction d =
            digital::discretise(closed, period, digital::Method::MatchedPoles);
        double worst = 0.0;
        const std::vector<Complex> s_poles = closed.poles();
        std::vector<Complex> z_poles = d.poles();
        for (const auto& s : s_poles) {
            const Complex expected = std::exp(s * period);
            double best = std::numeric_limits<double>::max();
            for (const auto& z : z_poles) {
                best = std::min(best, std::abs(z - expected));
            }
            worst = std::max(worst, best);
        }
        check(worst < 1e-8, "matched-pole discretisation places every pole at exactly e^{sT}");
    }
}

// ---------------------------------------------------------------------------
// The sampled-data loop and the sample-rate boundary
// ---------------------------------------------------------------------------

void test_sampled_loop() {
    std::cout << "\n--- the sampled-data loop ---\n";

    const control::JointPlant plant =
        control::build_joint_plant(1, Eigen::Matrix<double, 6, 1>::Zero(), 0.0, 0.25, 1.0);

    // Anti-windup can only help: with the actuator deliberately undersized the
    // integrator must wind up further without it than with it.
    digital::SampledLoopOptions options;
    options.sample_period = 0.004;
    options.duration = 6.0;
    options.setpoint = 1.0;
    options.kp = 8.0;
    options.ki = 6.0;
    options.kd = 1.0;
    options.derivative_filter = 0.02;
    options.command_limit = 0.08;
    options.anti_windup = true;
    const digital::SampledLoopRun with = digital::simulate_sampled_loop(plant.tf, options);
    options.anti_windup = false;
    const digital::SampledLoopRun without = digital::simulate_sampled_loop(plant.tf, options);
    check(without.saturated_fraction > 0.0,
          "the undersized actuator really does saturate, so windup has a chance to happen");
    check(without.peak_integral > with.peak_integral,
          "the integrator winds up further with conditional integration switched off");
    check(with.peak_command <= options.command_limit + 1e-12 &&
              without.peak_command <= options.command_limit + 1e-12,
          "the command never leaves the actuator limit in either run");
    std::cout << "       peak integral with anti-windup " << with.peak_integral << ", without "
              << without.peak_integral << "\n";

    // Quantisation can only be visible as a difference from the ideal run.
    options.anti_windup = true;
    options.command_limit = 1.0;
    options.setpoint = 0.5;
    options.measurement_quantum = 2.0 * kPi / (std::pow(2.0, 10.0) * plant.gear_ratio);
    const digital::SampledLoopRun quantised = digital::simulate_sampled_loop(plant.tf, options);
    options.measurement_quantum = 0.0;
    const digital::SampledLoopRun ideal = digital::simulate_sampled_loop(plant.tf, options);
    double difference = 0.0;
    for (std::size_t i = 0; i < quantised.y.size() && i < ideal.y.size(); ++i) {
        difference = std::max(difference, std::abs(quantised.y[i] - ideal.y[i]));
    }
    check(difference > 0.0 && difference < 0.05,
          "a 10-bit encoder changes the trajectory measurably but not wildly");

    // The whole point of session 26: there is a sample rate at which this loop
    // is unstable, and the module has to find it.
    const study::DigitalControlModule module;
    const json::Value swept = module.invoke("sample_rate_effect", json::Value::object());
    const double boundary = swept["unstable_sample_rate_hz"].as_double();
    check(boundary > 0.0,
          "sample_rate_effect finds a real sample rate at which the joint loop goes unstable");
    std::cout << "       the joint-2 PID loop goes unstable below " << boundary << " Hz\n";

    // And that boundary has to be a boundary: stable just above, unstable just
    // below, judged by the z-domain poles rather than by the sweep that found
    // it.
    if (boundary > 0.0) {
        const double period = 1.0 / boundary;
        auto closed_radius = [&](double t) {
            const digital::DiscreteTransferFunction controller = [&]() {
                const Poly den =
                    control::poly_multiply(Poly{1.0, -1.0}, Poly{1.0, 0.0});
                Poly num = control::poly_scale(den, 8.0);
                num = control::poly_add(num, control::poly_scale(Poly{1.0, 0.0, 0.0}, 4.0 * t));
                num = control::poly_add(
                    num, control::poly_scale(
                             control::poly_multiply(Poly{1.0, -1.0}, Poly{1.0, -1.0}), 1.0 / t));
                return digital::DiscreteTransferFunction(num, den, t);
            }();
            const digital::DiscreteTransferFunction discretised =
                digital::discretise(plant.tf, t, digital::Method::ZeroOrderHold);
            const digital::DiscreteTransferFunction loop(
                control::poly_multiply(controller.numerator(), discretised.numerator()),
                control::poly_multiply(controller.denominator(), discretised.denominator()), t);
            return loop.closed_loop().spectral_radius();
        };
        check(closed_radius(period * 0.9) < 1.0 && closed_radius(period * 1.1) > 1.0,
              "the reported rate is the boundary: stable 10 % faster, unstable 10 % slower");
    }
}

// ---------------------------------------------------------------------------
// Every op of both modules answers, and answers about this robot
// ---------------------------------------------------------------------------

void test_module_surface() {
    std::cout << "\n--- module surface ---\n";

    const study::ControlSystemModule control_module;
    const study::DigitalControlModule digital_module;

    for (const study::StudyModule* module :
         {static_cast<const study::StudyModule*>(&control_module),
          static_cast<const study::StudyModule*>(&digital_module)}) {
        const study::ModuleDescription description = module->describe();
        check(description.name == module->name() && description.course.id == 3884 &&
                  description.course.code == "M-408-01",
              std::string(module->name()) + " describes itself as part of course 3884");
        check(!description.ops.empty() && !description.summary.empty() &&
                  !description.source.empty(),
              std::string(module->name()) + " carries ops, a summary and a source path");

        bool specs_complete = true;
        for (const auto& op : description.ops) {
            if (op.title.empty() || op.formula.empty() || op.explain.size() < 80 ||
                op.outputs.empty()) {
                specs_complete = false;
                std::cout << "       incomplete op spec: " << op.name << "\n";
            }
            for (const auto& param : op.params) {
                if (param.name.empty() || param.type.empty() || param.label.empty()) {
                    specs_complete = false;
                }
                if (param.type == "enum" && param.options.empty()) {
                    specs_complete = false;
                }
                if ((param.type == "scalar" || param.type == "int") && !param.has_range) {
                    specs_complete = false;
                }
            }
        }
        check(specs_complete,
              std::string(module->name()) +
                  " gives every op a title, a formula, an explanation and complete parameter "
                  "specifications");

        // Every op must answer with its declared outputs when called with no
        // arguments at all, because that is what the UI does on first paint.
        for (const auto& op : description.ops) {
            bool ok = false;
            std::string missing;
            try {
                const json::Value result = module->invoke(op.name, json::Value::object());
                ok = result.is_object() && !result.empty();
                for (const auto& output : op.outputs) {
                    if (!result.contains(output.name)) {
                        ok = false;
                        missing += " " + output.name;
                    }
                }
                // The response must survive the wire, which is the only thing
                // the Python relay does with it.
                const std::string text = json::dump(result);
                if (json::parse(text) != result) {
                    ok = false;
                    missing += " <round-trip>";
                }
            } catch (const std::exception& error) {
                std::cout << "       " << op.name << " threw: " << error.what() << "\n";
            }
            if (!missing.empty()) {
                std::cout << "       " << op.name << " is missing declared outputs:" << missing
                          << "\n";
            }
            check(ok, std::string(module->name()) + "." + op.name +
                          " answers with every declared output and survives a JSON round trip");
        }

        // Bad input is an error, never a crash and never a silent answer.
        bool rejected = false;
        try {
            const json::Value result =
                module->invoke(description.ops[0].name,
                               json::Value::object({{"joint", json::Value(99)}}));
            (void)result;
        } catch (const study::StudyError&) {
            rejected = true;
        } catch (...) {
            rejected = false;
        }
        check(rejected, std::string(module->name()) +
                            " rejects an out-of-range joint index with StudyError");

        bool unknown_rejected = false;
        try {
            const json::Value result = module->invoke("not_an_op", json::Value::object());
            (void)result;
        } catch (const study::StudyError&) {
            unknown_rejected = true;
        } catch (...) {
            unknown_rejected = false;
        }
        check(unknown_rejected,
              std::string(module->name()) + " rejects an unknown op with StudyError");
    }

    // The three stability criteria must agree with each other through the
    // module's own interface, not just inside the core.
    {
        bool all_agree = true;
        for (const double gain : {0.5, 5.0, 50.0, 500.0}) {
            for (const double delay : {0.0, 4.0, 20.0}) {
                bool verdicts[3] = {false, false, false};
                const char* names[3] = {"algebraic", "nyquist", "roots"};
                for (int i = 0; i < 3; ++i) {
                    const json::Value result = control_module.invoke(
                        "stability", json::Value::object({{"criterion", json::Value(names[i])},
                                                          {"loop_gain", json::Value(gain)},
                                                          {"delay_ms", json::Value(delay)}}));
                    verdicts[i] = result["stable"].as_bool();
                    if (!result["agrees_with_roots"].as_bool()) {
                        all_agree = false;
                    }
                }
                if (verdicts[0] != verdicts[1] || verdicts[1] != verdicts[2]) {
                    all_agree = false;
                    std::cout << "       criteria disagree at gain " << gain << ", delay " << delay
                              << " ms\n";
                }
            }
        }
        check(all_agree,
              "Routh, Nyquist and the root locations return the same verdict at every gain and "
              "latency tested");
    }

    // The root locus and the Routh array have to name the same critical gain.
    {
        const json::Value locus = control_module.invoke(
            "root_locus", json::Value::object({{"delay_ms", json::Value(20.0)},
                                               {"gain_max", json::Value(10000.0)}}));
        const json::Value routh = control_module.invoke(
            "stability", json::Value::object({{"delay_ms", json::Value(20.0)}}));
        const double from_locus = locus["critical_gain"].as_double();
        const double from_routh = routh["critical_gain"].as_double();
        check(from_locus > 0.0 && from_routh > 0.0 &&
                  std::abs(from_locus - from_routh) < 0.02 * from_routh,
              "the root locus and the Routh array agree on the critical gain to 2 %");
    }

    // A type-1 loop has to report type 1, and adding the integral term has to
    // move it to type 2 - the ladder session 15 is about.
    {
        const json::Value p = control_module.invoke(
            "steady_state_accuracy",
            json::Value::object({{"controller", json::Value("proportional")}}));
        const json::Value pi = control_module.invoke(
            "steady_state_accuracy", json::Value::object({{"controller", json::Value("pi")}}));
        check(p["system_type"].as_int() == 1 && pi["system_type"].as_int() == 2,
              "the bare joint loop is type 1 and the PI loop is type 2");
    }

    // Robustness must find the payload range it was asked about, and a heavier
    // payload must never improve the phase margin.
    {
        const json::Value result = control_module.invoke(
            "robustness", json::Value::object({{"samples", json::Value(5)},
                                               {"response_samples", json::Value(201)}}));
        check(result["sweep"]["rows"].size() == 15,
              "the robustness sweep runs five payloads against three inertia variants");
        check(result["worst_payload"].as_double() >= 0.0 &&
                  result["worst_payload"].as_double() <= study::GP8_PAYLOAD_KG,
              "the worst case lands inside the rated payload range");
    }
}

}  // namespace

int main() {
    std::cout << "====================================================\n";
    std::cout << "   RUNNING CONTROL AND DIGITAL CONTROL TEST SUITE   \n";
    std::cout << "====================================================\n";

    try {
        test_polynomials();
        test_second_order_closed_form();
        test_routh_against_roots();
        test_nyquist_against_poles();
        test_margins_against_analytic();
        test_joint_plant_and_tuning();
        test_discretisation();
        test_sampled_loop();
        test_module_surface();
    } catch (const std::exception& error) {
        std::cout << "[FAIL] an exception escaped a test: " << error.what() << "\n";
        ++g_failures;
        g_failed_names.emplace_back("uncaught exception");
    }

    std::cout << "====================================================\n";
    std::cout << "checks run: " << g_checks << ", failed: " << g_failures << "\n";
    if (g_failures != 0) {
        for (const auto& name : g_failed_names) {
            std::cout << "FAILED: " << name << "\n";
        }
        std::cout << "CONTROL STUDY TESTS FAILED\n";
        return 1;
    }
    std::cout << "          ALL TESTS PASSED                          \n";
    std::cout << "====================================================\n";
    return 0;
}
