#include "study/digital_control.hpp"

#include "study/gp8_model.hpp"

#include <algorithm>
#include <array>
#include <cmath>
#include <complex>
#include <cstddef>
#include <limits>
#include <numbers>
#include <string>
#include <utility>
#include <vector>

namespace yaskawa::study {

namespace digital {

namespace {

constexpr double kEps = 1e-14;

}  // namespace

// ---------------------------------------------------------------------------
// DiscreteTransferFunction
// ---------------------------------------------------------------------------

DiscreteTransferFunction::DiscreteTransferFunction() : num_{1.0}, den_{1.0} {}

DiscreteTransferFunction::DiscreteTransferFunction(Poly numerator, Poly denominator,
                                                   double sample_period) {
    num_ = control::poly_trim(std::move(numerator));
    den_ = control::poly_trim(std::move(denominator));
    if (den_.size() == 1 && std::abs(den_[0]) <= kEps) {
        throw StudyError("a discrete transfer function needs a non-zero denominator");
    }
    if (!(sample_period > 0.0)) {
        throw StudyError("the sample period must be positive");
    }
    sample_period_ = sample_period;
}

std::vector<Complex> DiscreteTransferFunction::poles() const { return control::poly_roots(den_); }

std::vector<Complex> DiscreteTransferFunction::zeros() const { return control::poly_roots(num_); }

double DiscreteTransferFunction::spectral_radius() const {
    double worst = 0.0;
    for (const auto& p : poles()) {
        worst = std::max(worst, std::abs(p));
    }
    return worst;
}

bool DiscreteTransferFunction::is_stable() const { return spectral_radius() < 1.0 - 1e-9; }

Complex DiscreteTransferFunction::evaluate(const Complex& z) const {
    const Complex d = control::poly_eval(den_, z);
    if (std::abs(d) <= 0.0) {
        return Complex(std::numeric_limits<double>::infinity(), 0.0);
    }
    return control::poly_eval(num_, z) / d;
}

double DiscreteTransferFunction::dc_gain() const {
    const double d = control::poly_eval(den_, Complex(1.0, 0.0)).real();
    const double n = control::poly_eval(num_, Complex(1.0, 0.0)).real();
    if (std::abs(d) <= 1e-12) {
        return (std::abs(n) <= 1e-12) ? 0.0
                                      : std::copysign(std::numeric_limits<double>::infinity(), n);
    }
    return n / d;
}

std::vector<double> DiscreteTransferFunction::response_to(const std::vector<double>& input) const {
    const std::size_t n = den_.size() - 1;
    Poly b(den_.size(), 0.0);
    for (std::size_t i = 0; i < num_.size() && i < b.size(); ++i) {
        b[b.size() - num_.size() + i] = num_[i];
    }
    const double a0 = den_[0];
    std::vector<double> y(input.size(), 0.0);
    for (std::size_t k = 0; k < input.size(); ++k) {
        double accumulator = 0.0;
        for (std::size_t i = 0; i <= n; ++i) {
            if (k >= i) {
                accumulator += b[i] * input[k - i];
            }
        }
        for (std::size_t i = 1; i <= n; ++i) {
            if (k >= i) {
                accumulator -= den_[i] * y[k - i];
            }
        }
        y[k] = accumulator / a0;
    }
    return y;
}

std::vector<double> DiscreteTransferFunction::step_response(std::size_t steps,
                                                            double amplitude) const {
    if (steps < 2) {
        throw StudyError("a discrete step response needs at least 2 steps");
    }
    const std::vector<double> input(steps, amplitude);
    return response_to(input);
}

std::string DiscreteTransferFunction::difference_equation() const {
    const std::size_t n = den_.size() - 1;
    Poly b(den_.size(), 0.0);
    for (std::size_t i = 0; i < num_.size() && i < b.size(); ++i) {
        b[b.size() - num_.size() + i] = num_[i];
    }
    const double a0 = den_[0];
    std::string out = "y[k] = ";
    bool first = true;
    for (std::size_t i = 0; i <= n; ++i) {
        const double c = b[i] / a0;
        if (std::abs(c) <= 1e-15) {
            continue;
        }
        if (!first) {
            out += (c < 0.0) ? " - " : " + ";
        } else if (c < 0.0) {
            out += "-";
        }
        first = false;
        out += json::number_to_string(std::abs(c)) + " u[k";
        out += (i == 0) ? "]" : ("-" + std::to_string(i) + "]");
    }
    for (std::size_t i = 1; i <= n; ++i) {
        const double c = -den_[i] / a0;
        if (std::abs(c) <= 1e-15) {
            continue;
        }
        if (!first) {
            out += (c < 0.0) ? " - " : " + ";
        } else if (c < 0.0) {
            out += "-";
        }
        first = false;
        out += json::number_to_string(std::abs(c)) + " y[k-" + std::to_string(i) + "]";
    }
    if (first) {
        out += "0";
    }
    return out;
}

std::string DiscreteTransferFunction::to_string() const {
    return "(" + control::poly_to_string(num_, "z") + ") / (" +
           control::poly_to_string(den_, "z") + ")";
}

DiscreteTransferFunction DiscreteTransferFunction::closed_loop() const {
    return DiscreteTransferFunction(num_, control::poly_add(den_, num_), sample_period_);
}

// ---------------------------------------------------------------------------
// Discretisation
// ---------------------------------------------------------------------------

Method method_from_string(std::string_view name) {
    if (name == "zero_order_hold") {
        return Method::ZeroOrderHold;
    }
    if (name == "bilinear") {
        return Method::Bilinear;
    }
    if (name == "forward_euler") {
        return Method::ForwardEuler;
    }
    if (name == "backward_euler") {
        return Method::BackwardEuler;
    }
    if (name == "matched_poles") {
        return Method::MatchedPoles;
    }
    throw StudyError("unknown discretisation method '" + std::string(name) + "'");
}

std::string method_name(Method method) {
    switch (method) {
        case Method::ZeroOrderHold: return "zero_order_hold";
        case Method::Bilinear: return "bilinear";
        case Method::ForwardEuler: return "forward_euler";
        case Method::BackwardEuler: return "backward_euler";
        case Method::MatchedPoles: return "matched_poles";
    }
    return "zero_order_hold";
}

std::string method_substitution(Method method, double sample_period) {
    const std::string t = json::number_to_string(sample_period);
    switch (method) {
        case Method::ZeroOrderHold:
            return "exact for a piecewise-constant input: A_d = e^{A T}, B_d = int_0^T e^{A s} B "
                   "ds, with T = " + t + " s";
        case Method::Bilinear:
            return "s = (2/T)(z-1)/(z+1), T = " + t +
                   " s - the whole left half plane folds into the unit disc, so a stable plant "
                   "stays stable at every T";
        case Method::ForwardEuler:
            return "s = (z-1)/T, T = " + t +
                   " s - cheapest and least safe: the stability region is a unit circle centred "
                   "at +1, so a fast pole leaves it as soon as T grows";
        case Method::BackwardEuler:
            return "s = (z-1)/(T z), T = " + t +
                   " s - maps the left half plane into a small circle inside the unit disc, so it "
                   "never produces an unstable result, but it distorts the damping";
        case Method::MatchedPoles:
            return "z_i = e^{s_i T}, T = " + t +
                   " s - each pole and zero mapped individually, gain matched, which keeps the "
                   "time constants exactly right";
    }
    return "";
}

Poly poly_from_roots(const std::vector<Complex>& roots) {
    std::vector<Complex> coefficients{Complex(1.0, 0.0)};
    for (const auto& root : roots) {
        std::vector<Complex> next(coefficients.size() + 1, Complex(0.0, 0.0));
        for (std::size_t i = 0; i < coefficients.size(); ++i) {
            next[i] += coefficients[i];
            next[i + 1] -= coefficients[i] * root;
        }
        coefficients = std::move(next);
    }
    Poly out;
    out.reserve(coefficients.size());
    for (const auto& c : coefficients) {
        out.push_back(c.real());
    }
    return out;
}

namespace {

// Substitutes s = p(z)/q(z) into a polynomial in s of degree d, lifted to a
// common denominator q^m so numerator and denominator stay polynomials.
[[nodiscard]] Poly substitute_rational(const Poly& coefficients, const Poly& p, const Poly& q,
                                       std::size_t m) {
    const std::size_t degree = coefficients.size() - 1;
    Poly out{0.0};
    for (std::size_t k = 0; k <= degree; ++k) {
        const std::size_t power = degree - k;
        if (power > m) {
            throw StudyError("the substitution cannot raise the denominator high enough");
        }
        Poly term = control::poly_multiply(control::poly_power(p, power),
                                           control::poly_power(q, m - power));
        out = control::poly_add(out, control::poly_scale(term, coefficients[k]));
    }
    return out;
}

[[nodiscard]] DiscreteTransferFunction substitution_discretise(const TransferFunction& continuous,
                                                               double sample_period, const Poly& p,
                                                               const Poly& q) {
    const std::size_t dn = continuous.numerator().size() - 1;
    const std::size_t dd = continuous.denominator().size() - 1;
    const std::size_t m = std::max(dn, dd);
    return DiscreteTransferFunction(substitute_rational(continuous.numerator(), p, q, m),
                                    substitute_rational(continuous.denominator(), p, q, m),
                                    sample_period);
}

[[nodiscard]] DiscreteTransferFunction zoh_discretise(const TransferFunction& continuous,
                                                      double sample_period) {
    const control::StateSpace ss = continuous.state_space();
    const Eigen::Index n = ss.A.rows();
    if (n == 0) {
        return DiscreteTransferFunction({ss.D}, {1.0}, sample_period);
    }
    // The augmented exponential gives A_d and B_d in one step and stays exact
    // when A is singular - which it always is here, because the joint has a
    // pole at the origin.
    Eigen::MatrixXd augmented = Eigen::MatrixXd::Zero(n + 1, n + 1);
    augmented.topLeftCorner(n, n) = ss.A * sample_period;
    augmented.topRightCorner(n, 1) = ss.B * sample_period;
    const Eigen::MatrixXd exponential = control::matrix_exponential(augmented);
    const Eigen::MatrixXd ad = exponential.topLeftCorner(n, n);
    const Eigen::MatrixXd bd = exponential.topRightCorner(n, 1);

    const control::CharacteristicPoly cp = control::characteristic_poly(ad);
    Poly numerator = control::poly_scale(cp.coefficients, ss.D);
    for (std::size_t k = 1; k <= static_cast<std::size_t>(n); ++k) {
        const Eigen::MatrixXd contribution = ss.C * cp.adjugate[k - 1] * bd;
        numerator[k] += contribution(0, 0);
    }
    return DiscreteTransferFunction(std::move(numerator), cp.coefficients, sample_period);
}

[[nodiscard]] DiscreteTransferFunction matched_discretise(const TransferFunction& continuous,
                                                          double sample_period) {
    std::vector<Complex> z_poles;
    std::vector<Complex> z_zeros;
    for (const auto& p : continuous.poles()) {
        z_poles.push_back(std::exp(p * sample_period));
    }
    for (const auto& z : continuous.zeros()) {
        z_zeros.push_back(std::exp(z * sample_period));
    }
    Poly den = poly_from_roots(z_poles);
    Poly num = poly_from_roots(z_zeros);
    // Pad the numerator with (z + 1) factors so the discrete model is biproper,
    // the usual modified matched-pole-zero choice.
    const std::size_t deficit = (den.size() > num.size()) ? (den.size() - num.size()) : 0;
    if (deficit > 0) {
        num = control::poly_multiply(num, control::poly_power(Poly{1.0, 1.0}, deficit));
    }
    DiscreteTransferFunction candidate(num, den, sample_period);

    const double continuous_dc = continuous.dc_gain();
    double scale = 1.0;
    if (std::isfinite(continuous_dc) && std::abs(continuous_dc) > 1e-12 &&
        std::isfinite(candidate.dc_gain()) && std::abs(candidate.dc_gain()) > 1e-12) {
        scale = continuous_dc / candidate.dc_gain();
    } else {
        // A free integrator makes both dc gains infinite, so match a decade
        // below the Nyquist frequency instead.
        const double omega = 0.1 * std::numbers::pi / sample_period;
        const double continuous_magnitude = std::abs(continuous.frequency_response(omega));
        const double discrete_magnitude =
            std::abs(candidate.evaluate(std::exp(Complex(0.0, omega * sample_period))));
        if (discrete_magnitude > 1e-300) {
            scale = continuous_magnitude / discrete_magnitude;
        }
    }
    return DiscreteTransferFunction(control::poly_scale(num, scale), den, sample_period);
}

}  // namespace

DiscreteTransferFunction discretise(const TransferFunction& continuous, double sample_period,
                                   Method method) {
    if (!(sample_period > 0.0)) {
        throw StudyError("the sample period must be positive");
    }
    switch (method) {
        case Method::ZeroOrderHold:
            return zoh_discretise(continuous, sample_period);
        case Method::Bilinear:
            return substitution_discretise(continuous, sample_period,
                                           Poly{2.0 / sample_period, -2.0 / sample_period},
                                           Poly{1.0, 1.0});
        case Method::ForwardEuler:
            return substitution_discretise(continuous, sample_period, Poly{1.0, -1.0},
                                           Poly{sample_period});
        case Method::BackwardEuler:
            return substitution_discretise(continuous, sample_period, Poly{1.0, -1.0},
                                           Poly{sample_period, 0.0});
        case Method::MatchedPoles:
            return matched_discretise(continuous, sample_period);
    }
    throw StudyError("unknown discretisation method");
}

// ---------------------------------------------------------------------------
// The sampled-data loop, with the nonlinearities a transfer function cannot
// carry: command saturation, integral windup, encoder and word-length quanta.
// ---------------------------------------------------------------------------

SampledLoopRun simulate_sampled_loop(const TransferFunction& plant,
                                     const SampledLoopOptions& options) {
    if (!(options.sample_period > 0.0)) {
        throw StudyError("the sample period must be positive");
    }
    if (!(options.duration > options.sample_period)) {
        throw StudyError("the run must be longer than one sample period");
    }
    const control::StateSpace ss = plant.state_space();
    const Eigen::Index n = ss.A.rows();
    const std::size_t steps =
        static_cast<std::size_t>(std::floor(options.duration / options.sample_period)) + 1;
    if (steps > 2000000) {
        throw StudyError("that sample period and duration would need more than 2e6 steps");
    }

    SampledLoopRun run;
    run.t.reserve(steps);
    run.y.reserve(steps);
    run.u.reserve(steps);
    run.integral.reserve(steps);

    double fastest = 0.0;
    for (const auto& p : plant.poles()) {
        fastest = std::max(fastest, std::abs(p));
    }
    std::size_t substeps = 4;
    if (fastest > 0.0) {
        substeps = static_cast<std::size_t>(
            std::clamp(std::ceil(options.sample_period * fastest / 0.02), 4.0, 512.0));
    }
    const double h = options.sample_period / static_cast<double>(substeps);

    Eigen::VectorXd x = Eigen::VectorXd::Zero(n);
    Eigen::VectorXd k1 = Eigen::VectorXd::Zero(n);
    Eigen::VectorXd k2 = Eigen::VectorXd::Zero(n);
    Eigen::VectorXd k3 = Eigen::VectorXd::Zero(n);
    Eigen::VectorXd k4 = Eigen::VectorXd::Zero(n);
    Eigen::VectorXd tmp = Eigen::VectorXd::Zero(n);

    double integral = 0.0;
    double derivative = 0.0;
    double previous_error = 0.0;
    std::size_t saturated_samples = 0;

    for (std::size_t k = 0; k < steps; ++k) {
        const double time = static_cast<double>(k) * options.sample_period;
        double y = (n > 0) ? (ss.C * x)(0) : 0.0;
        const double measured =
            (options.measurement_quantum > 0.0)
                ? std::round(y / options.measurement_quantum) * options.measurement_quantum
                : y;
        const double error = options.setpoint - measured;

        const double trial_integral = integral + error * options.sample_period;
        if (options.derivative_filter > 0.0) {
            const double alpha =
                options.derivative_filter / (options.derivative_filter + options.sample_period);
            derivative = alpha * derivative +
                         (1.0 - alpha) * (error - previous_error) / options.sample_period;
        } else {
            derivative = (k == 0) ? 0.0 : (error - previous_error) / options.sample_period;
        }

        double command = options.kp * error + options.ki * trial_integral + options.kd * derivative;
        const bool saturating = std::abs(command) > options.command_limit;
        if (saturating) {
            ++saturated_samples;
            command = std::clamp(command, -options.command_limit, options.command_limit);
        }
        // Conditional integration: when the actuator is already at its limit,
        // integrating further only winds the state up without moving anything.
        const bool winding = saturating && (error * command > 0.0);
        integral = (options.anti_windup && winding) ? integral : trial_integral;
        if (options.anti_windup) {
            command = options.kp * error + options.ki * integral + options.kd * derivative;
            command = std::clamp(command, -options.command_limit, options.command_limit);
        }
        if (options.command_quantum > 0.0) {
            command = std::round(command / options.command_quantum) * options.command_quantum;
        }
        previous_error = error;

        double applied = command;
        if (options.disturbance != 0.0 && time >= options.disturbance_time) {
            applied += options.disturbance;
        }

        run.t.push_back(time);
        run.y.push_back(y);
        run.u.push_back(command);
        run.integral.push_back(integral);
        run.peak_command = std::max(run.peak_command, std::abs(command));
        run.peak_integral = std::max(run.peak_integral, std::abs(integral));

        if (k + 1 == steps || n == 0) {
            continue;
        }
        for (std::size_t s = 0; s < substeps; ++s) {
            k1.noalias() = ss.A * x;
            k1 += ss.B * applied;
            tmp = x + (0.5 * h) * k1;
            k2.noalias() = ss.A * tmp;
            k2 += ss.B * applied;
            tmp = x + (0.5 * h) * k2;
            k3.noalias() = ss.A * tmp;
            k3 += ss.B * applied;
            tmp = x + h * k3;
            k4.noalias() = ss.A * tmp;
            k4 += ss.B * applied;
            x += (h / 6.0) * (k1 + 2.0 * k2 + 2.0 * k3 + k4);
        }
    }
    run.final_value = run.y.empty() ? 0.0 : run.y.back();
    run.saturated_fraction =
        steps > 0 ? static_cast<double>(saturated_samples) / static_cast<double>(steps) : 0.0;
    return run;
}

}  // namespace digital

namespace {

constexpr double kPi = std::numbers::pi;

using digital::DiscreteTransferFunction;
using digital::Method;
using digital::SampledLoopOptions;

[[nodiscard]] Eigen::Matrix<double, 6, 1> zero6() noexcept {
    return Eigen::Matrix<double, 6, 1>::Zero();
}

const std::vector<std::string> kMethods = {"zero_order_hold", "bilinear", "forward_euler",
                                           "backward_euler", "matched_poles"};

void add_plant_params(std::vector<ParamSpec>& into) {
    into.push_back(ParamSpec::integer("joint", "Joint (1 = S .. 6 = T)", "", 1, 6, 2));
    into.push_back(ParamSpec::vec6("q", "Arm configuration", "rad", -widest_joint_range(),
                                   widest_joint_range(), zero6()));
    into.push_back(ParamSpec::scalar("payload_kg", "Payload at the flange", "kg", 0.0,
                                     GP8_PAYLOAD_KG, 0.0));
    into.push_back(ParamSpec::scalar("velocity_fraction",
                                     "Velocity where Coulomb friction is linearised",
                                     "fraction of axis rating", 0.02, 1.0, 0.25));
    into.push_back(ParamSpec::scalar("torque_fraction", "Drive torque at command u = 1",
                                     "fraction of axis rating", 0.05, 1.0, 1.0));
}

[[nodiscard]] control::JointPlant read_plant(const json::Value& args) {
    const int joint = optional_int(args, "joint", 2, 1, 6);
    const Eigen::Matrix<double, 6, 1> q =
        optional_vec6(args, "q", zero6(), -widest_joint_range(), widest_joint_range());
    const double payload = optional_scalar(args, "payload_kg", 0.0, 0.0, GP8_PAYLOAD_KG);
    const double velocity_fraction = optional_scalar(args, "velocity_fraction", 0.25, 0.02, 1.0);
    const double torque_fraction = optional_scalar(args, "torque_fraction", 1.0, 0.05, 1.0);
    return control::build_joint_plant(static_cast<std::size_t>(joint - 1), q, payload,
                                      velocity_fraction, torque_fraction);
}

[[nodiscard]] double flange_moment_arm(std::size_t joint,
                                       const Eigen::Matrix<double, 6, 1>& q) noexcept {
    const std::array<Eigen::Isometry3d, GP8_DOF> frames = link_frames(q);
    Eigen::Vector3d axis = Eigen::Vector3d::UnitZ();
    Eigen::Vector3d point = Eigen::Vector3d::Zero();
    if (joint > 0) {
        axis = frames[joint - 1].linear().col(2);
        point = frames[joint - 1].translation();
    }
    const Eigen::Vector3d flange = (frames[GP8_DOF - 1] * flange_correction()).translation();
    const Eigen::Vector3d r = flange - point;
    return (r - axis * r.dot(axis)).norm();
}

[[nodiscard]] json::Value poly_table(const control::Poly& p) {
    std::vector<std::vector<json::Value>> rows;
    rows.reserve(p.size());
    const std::size_t degree = p.size() - 1;
    for (std::size_t i = 0; i < p.size(); ++i) {
        rows.push_back({json::Value(static_cast<int>(degree - i)), json::Value(p[i])});
    }
    return json::from_table({"power of z", "coefficient"}, rows);
}

[[nodiscard]] json::Value unit_circle(std::size_t points) {
    std::vector<digital::Complex> circle;
    circle.reserve(points + 1);
    for (std::size_t i = 0; i <= points; ++i) {
        const double theta = 2.0 * kPi * static_cast<double>(i) / static_cast<double>(points);
        circle.emplace_back(std::cos(theta), std::sin(theta));
    }
    return json::from_complex_set(circle);
}

[[nodiscard]] std::vector<double> sample_times(std::size_t steps, double period) {
    std::vector<double> t;
    t.reserve(steps);
    for (std::size_t i = 0; i < steps; ++i) {
        t.push_back(static_cast<double>(i) * period);
    }
    return t;
}

// The discrete PID a controller actually runs, in its parallel form:
//   C(z) = Kp + Ki T z/(z-1) + (Kd/T)(z-1)/z
[[nodiscard]] DiscreteTransferFunction discrete_pid(double kp, double ki, double kd, double period) {
    const control::Poly den = control::poly_multiply(control::Poly{1.0, -1.0},
                                                     control::Poly{1.0, 0.0});
    control::Poly num = control::poly_scale(den, kp);
    num = control::poly_add(num, control::poly_scale(control::Poly{1.0, 0.0, 0.0}, ki * period));
    num = control::poly_add(
        num, control::poly_scale(control::poly_multiply(control::Poly{1.0, -1.0},
                                                        control::Poly{1.0, -1.0}),
                                 kd / period));
    return DiscreteTransferFunction(num, den, period);
}

[[nodiscard]] DiscreteTransferFunction discrete_series(const DiscreteTransferFunction& a,
                                                       const DiscreteTransferFunction& b) {
    return DiscreteTransferFunction(
        control::poly_multiply(a.numerator(), b.numerator()),
        control::poly_multiply(a.denominator(), b.denominator()), a.sample_period());
}

struct RatePoint {
    double period = 0.0;
    double rate_hz = 0.0;
    double phase_margin = 0.0;
    double overshoot = 0.0;
    double spectral_radius = 0.0;
    bool stable = false;
};

[[nodiscard]] RatePoint evaluate_rate(const control::TransferFunction& plant, double kp, double ki,
                                      double kd, double period, double duration) {
    RatePoint point;
    point.period = period;
    point.rate_hz = 1.0 / period;

    const DiscreteTransferFunction loop =
        discrete_series(discrete_pid(kp, ki, kd, period),
                        digital::discretise(plant, period, Method::ZeroOrderHold));
    const DiscreteTransferFunction closed = loop.closed_loop();
    point.spectral_radius = closed.spectral_radius();
    point.stable = closed.is_stable();

    // The continuous equivalent of the hold is half a sample of dead time,
    // which is the omega T / 2 phase lag session 26 asks the student to find.
    const control::TransferFunction continuous_loop =
        control::pid_controller(kp, ki, kd, 0.01)
            .series(plant)
            .series(control::TransferFunction::pade_delay(0.5 * period));
    const control::Margins margins = control::loop_margins(continuous_loop, 1e-2, 1e4, 700);
    point.phase_margin = margins.has_phase_margin ? margins.phase_margin_deg : 180.0;

    const std::size_t steps =
        static_cast<std::size_t>(std::clamp(std::floor(duration / period) + 1.0, 8.0, 40000.0));
    const std::vector<double> response = closed.step_response(steps);
    const double final_value = point.stable ? closed.dc_gain() : 0.0;
    if (point.stable && std::abs(final_value) > 1e-12) {
        double peak = 0.0;
        for (const double v : response) {
            peak = std::max(peak, v);
        }
        point.overshoot = std::max(0.0, (peak - final_value) / std::abs(final_value) * 100.0);
    } else {
        point.overshoot = 1000.0;  // diverged: shown as off the top of the axis
    }
    return point;
}

}  // namespace

// ---------------------------------------------------------------------------
// Self-description
// ---------------------------------------------------------------------------

ModuleDescription DigitalControlModule::describe() const {
    ModuleDescription d;
    d.name = "digital_control";
    d.title = "Digital Control: Sampling, the z Domain and What They Cost";
    d.course = CourseRef{3884, "M-408-01", "Robot Control and Feedback Systems"};
    d.topics = {"Sessions 25-26 · Sampling, discretisation, the z domain, sample-rate and "
                "quantisation effects"};
    d.source = "cpp_solver/include/study/digital_control.hpp";
    d.summary =
        "Takes the continuous GP8 joint loop into the z domain by five different methods, shows "
        "the left half plane folding into the unit circle, finds the sample rate at which the "
        "joint loop actually goes unstable, runs the discrete PID difference equation with "
        "anti-windup and derivative filtering as toggles, and turns encoder resolution and "
        "controller word length into position ripple measured against the GP8's 20 micrometre "
        "repeatability.";

    {
        OpSpec op;
        op.name = "discretise";
        op.title = "From s to z, five ways";
        op.formula =
            "\\text{ZOH: } G(z) = (1-z^{-1})\\mathcal{Z}\\left\\{\\frac{G(s)}{s}\\right\\}, \\quad "
            "\\text{Tustin: } s = \\frac{2}{T}\\frac{z-1}{z+1}, \\quad "
            "\\text{Euler: } s = \\frac{z-1}{T} \\text{ or } \\frac{z-1}{Tz}, \\quad "
            "\\text{matched: } z_i = e^{s_iT}";
        op.explain =
            "A continuous pole at s maps to a discrete pole at z = e^{sT}, so the left half plane "
            "becomes the inside of the unit circle and the stability test changes from 'negative "
            "real part' to 'modulus below one'. The five methods disagree about everything else. "
            "Zero-order hold is exact for a command that is held constant between samples, which "
            "is what a controller actually does. The bilinear (Tustin) substitution maps the "
            "whole left half plane inside the circle, so it can never turn a stable plant into an "
            "unstable model, at the price of warping the frequency axis. Forward Euler is the "
            "cheapest and the only one of the five that can produce an unstable model from a "
            "stable plant - watch its pole leave the circle as T grows. The overlaid step "
            "responses are the whole point: the gap between them is the approximation error you "
            "are choosing to accept.";
        add_plant_params(op.params);
        op.params.push_back(ParamSpec::enumeration("method", "Discretisation method", kMethods,
                                                   "zero_order_hold"));
        op.params.push_back(ParamSpec::scalar("sample_period", "Sample period T", "s", 1e-4, 1.0,
                                              0.01));
        op.params.push_back(ParamSpec::enumeration("system", "What to discretise",
                                                   {"plant", "closed_loop"}, "closed_loop"));
        op.params.push_back(ParamSpec::scalar("loop_gain", "Proportional gain for the closed loop",
                                              "", 0.01, 500.0, 1.0));
        op.params.push_back(ParamSpec::scalar("duration", "Window", "s", 0.01, 60.0, 5.0));
        op.outputs = {
            OutputSpec::make("numerator", "table", "z-domain numerator coefficients"),
            OutputSpec::make("denominator", "table", "z-domain denominator coefficients"),
            OutputSpec::make("transfer_function", "text", "G(z)"),
            OutputSpec::make("difference_equation", "text",
                             "The recursion a controller would execute"),
            OutputSpec::make("s_poles", "complex_set", "Continuous poles"),
            OutputSpec::make("z_poles", "complex_set", "Their images in the z plane"),
            OutputSpec::make("z_zeros", "complex_set", "Discrete zeros"),
            OutputSpec::make("unit_circle", "complex_set", "The stability boundary"),
            OutputSpec::make("mapping", "table", "s pole, z pole, modulus, inside the circle"),
            OutputSpec::make("responses", "series_set",
                             "Continuous and discrete step response, overlaid"),
            OutputSpec::make("max_error", "scalar", "Largest gap at the sample instants", "rad"),
            OutputSpec::make("rms_error", "scalar", "Root-mean-square gap", "rad"),
            OutputSpec::make("spectral_radius", "scalar", "Largest |z| pole"),
            OutputSpec::make("stable", "bool", "Every discrete pole inside the unit circle"),
            OutputSpec::make("nyquist_frequency", "scalar", "pi/T, the aliasing limit", "rad/s"),
            OutputSpec::make("substitution", "text", "What this method actually does"),
        };
        d.ops.push_back(std::move(op));
    }
    {
        OpSpec op;
        op.name = "sample_rate_effect";
        op.title = "The sample rate at which the joint loop goes unstable";
        op.formula =
            "\\varphi_{hold} \\approx -\\frac{\\omega T}{2}, \\qquad "
            "\\text{unstable when } \\max_i |z_i| \\ge 1";
        op.explain =
            "Holding the command between samples is, to the loop, half a sample period of pure "
            "dead time: it costs omega T / 2 radians of phase and gives nothing back. Sweep the "
            "sample period and the phase margin falls almost linearly while the overshoot grows, "
            "until the discrete closed-loop poles leave the unit circle and the joint oscillates "
            "with growing amplitude. The number this op exists to produce is that boundary rate, "
            "found by bisecting the exact z-domain pole locations rather than by eyeballing the "
            "curve - and the rule of thumb to carry away is that a loop needs a sample rate "
            "around twenty times its own crossover frequency, not two.";
        add_plant_params(op.params);
        op.params.push_back(ParamSpec::scalar("kp", "Kp", "", 0.0, 500.0, 8.0));
        op.params.push_back(ParamSpec::scalar("ki", "Ki", "", 0.0, 500.0, 4.0));
        op.params.push_back(ParamSpec::scalar("kd", "Kd", "", 0.0, 100.0, 1.0));
        op.params.push_back(ParamSpec::scalar("period_min", "Fastest sample period", "s", 1e-5,
                                              0.1, 0.0005));
        op.params.push_back(ParamSpec::scalar("period_max", "Slowest sample period", "s", 1e-4,
                                              2.0, 0.5));
        op.params.push_back(ParamSpec::integer("samples", "Periods in the sweep", "", 5, 200, 40));
        op.params.push_back(ParamSpec::scalar("duration", "Window per point", "s", 0.1, 60.0, 8.0));
        op.outputs = {
            OutputSpec::make("unstable_sample_rate_hz", "scalar",
                             "THE ANSWER: the rate below which this loop is unstable", "Hz"),
            OutputSpec::make("unstable_sample_period", "scalar", "Its period", "s"),
            OutputSpec::make("recommended_sample_rate_hz", "scalar",
                             "Rate that keeps the phase margin at or above 45 degrees", "Hz"),
            OutputSpec::make("verdict", "text", "What the sweep says, in one paragraph"),
            OutputSpec::make("margin_curve", "series_set",
                             "Phase margin and overshoot against sample rate"),
            OutputSpec::make("stability_curve", "series_set",
                             "Largest |z| pole against sample rate"),
            OutputSpec::make("sweep", "table", "Every sweep point"),
            OutputSpec::make("continuous_phase_margin", "scalar",
                             "Phase margin with no sampling at all", "deg"),
            OutputSpec::make("crossover_frequency", "scalar", "Loop crossover frequency",
                             "rad/s"),
            OutputSpec::make("rate_to_crossover_ratio", "scalar",
                             "Boundary sample rate divided by the crossover frequency in Hz"),
        };
        d.ops.push_back(std::move(op));
    }
    {
        OpSpec op;
        op.name = "digital_pid";
        op.title = "The discrete PID that runs on the controller";
        op.formula =
            "u[k] = u[k-1] + q_0 e[k] + q_1 e[k-1] + q_2 e[k-2], \\quad "
            "q_0 = K_p + \\frac{K_iT}{2} + \\frac{K_d}{T}, \\; "
            "q_1 = -K_p + \\frac{K_iT}{2} - \\frac{2K_d}{T}, \\; q_2 = \\frac{K_d}{T}";
        op.explain =
            "This is the controller as three multiply-accumulates and two stored errors - the "
            "form that fits in an interrupt handler. Two things that do not exist in the "
            "continuous theory now decide whether it works. Integral windup: while the actuator "
            "is saturated the integrator keeps accumulating an error it cannot fix, and when the "
            "joint finally arrives the stored integral drives it straight past the target; "
            "conditional integration stops that. Derivative noise: (e[k]-e[k-1])/T amplifies "
            "encoder quantisation by 1/T, so the derivative term needs the first-order filter. "
            "Both are toggles here, and the integrator trace is plotted so the windup is visible "
            "rather than described.";
        add_plant_params(op.params);
        op.params.push_back(ParamSpec::scalar("kp", "Kp", "", 0.0, 500.0, 8.0));
        op.params.push_back(ParamSpec::scalar("ki", "Ki", "", 0.0, 500.0, 6.0));
        op.params.push_back(ParamSpec::scalar("kd", "Kd", "", 0.0, 100.0, 1.0));
        op.params.push_back(ParamSpec::scalar("sample_period", "Sample period T", "s", 1e-4, 0.5,
                                              0.004));
        op.params.push_back(ParamSpec::boolean("anti_windup", "Conditional integration", true));
        op.params.push_back(ParamSpec::boolean("derivative_filter", "Filter the derivative", true));
        op.params.push_back(ParamSpec::scalar("command_limit", "Actuator limit",
                                              "fraction of rated torque", 0.02, 1.0, 0.25));
        op.params.push_back(ParamSpec::scalar("setpoint", "Step size", "rad", -kPi, kPi, 1.0));
        op.params.push_back(ParamSpec::scalar("duration", "Window", "s", 0.05, 60.0, 6.0));
        op.outputs = {
            OutputSpec::make("coefficients", "table", "q0, q1, q2 and the gains behind them"),
            OutputSpec::make("difference_equation", "text", "The recursion, written out"),
            OutputSpec::make("controller_z", "text", "C(z) in the parallel form"),
            OutputSpec::make("responses", "series_set",
                             "Angle with the toggles on and off, plus the setpoint"),
            OutputSpec::make("commands", "series_set", "Command written, both cases"),
            OutputSpec::make("integrators", "series_set", "Integrator state, both cases"),
            OutputSpec::make("comparison", "table", "What each toggle changes"),
            OutputSpec::make("peak_integral_protected", "scalar", "Worst integral with protection"),
            OutputSpec::make("peak_integral_unprotected", "scalar", "Worst integral without it"),
            OutputSpec::make("saturated_fraction", "scalar",
                             "Fraction of samples spent against the limit"),
            OutputSpec::make("overshoot_protected", "scalar", "Overshoot with protection", "%"),
            OutputSpec::make("overshoot_unprotected", "scalar", "Overshoot without it", "%"),
            OutputSpec::make("verdict", "text", "What windup cost on this move"),
        };
        d.ops.push_back(std::move(op));
    }
    {
        OpSpec op;
        op.name = "quantisation";
        op.title = "Encoder resolution, word length, and the 20 micrometre specification";
        op.formula =
            "\\Delta q = \\frac{2\\pi}{2^{N}n}, \\qquad \\Delta x = \\Delta q \\cdot r_\\perp, "
            "\\qquad u_{LSB} = \\frac{2u_{max}}{2^{M}}";
        op.explain =
            "An encoder of N bits on the motor shaft, behind a gearbox of ratio n, resolves the "
            "joint to 2 pi / (2^N n) radians - and at the flange that angle becomes a distance, "
            "which is the number the datasheet's 0.02 mm repeatability has to be compared "
            "against. Below that quantum the loop cannot know where it is, so it hunts: the "
            "command alternates between neighbouring codes and the joint settles into a limit "
            "cycle instead of a point. Coarsen the encoder or shorten the controller word and "
            "watch the ripple grow past the specification; the amplitude and period reported here "
            "are measured from the last quarter of the run, not estimated.";
        add_plant_params(op.params);
        op.params.push_back(ParamSpec::integer("encoder_bits", "Encoder bits per motor revolution",
                                               "bit", 8, 24, 17));
        op.params.push_back(ParamSpec::integer("word_length_bits", "Controller output word length",
                                               "bit", 6, 24, 12));
        op.params.push_back(ParamSpec::scalar("kp", "Kp", "", 0.0, 500.0, 8.0));
        op.params.push_back(ParamSpec::scalar("ki", "Ki", "", 0.0, 500.0, 6.0));
        op.params.push_back(ParamSpec::scalar("kd", "Kd", "", 0.0, 100.0, 1.0));
        op.params.push_back(ParamSpec::scalar("sample_period", "Sample period T", "s", 1e-4, 0.1,
                                              0.004));
        op.params.push_back(ParamSpec::scalar("command_limit", "Actuator limit",
                                              "fraction of rated torque", 0.02, 1.0, 1.0));
        op.params.push_back(ParamSpec::scalar("setpoint", "Step size", "rad", -kPi, kPi, 0.5));
        op.params.push_back(ParamSpec::scalar("duration", "Window", "s", 0.2, 60.0, 12.0));
        op.outputs = {
            OutputSpec::make("encoder_resolution_rad", "scalar", "Joint-side encoder quantum",
                             "rad"),
            OutputSpec::make("encoder_resolution_deg", "scalar", "The same quantum", "deg"),
            OutputSpec::make("flange_moment_arm", "scalar",
                             "Distance from this axis to the flange", "m"),
            OutputSpec::make("resolution_at_flange_m", "scalar",
                             "What one encoder count is worth at the flange", "m"),
            OutputSpec::make("command_lsb", "scalar", "One output code", "fraction of rating"),
            OutputSpec::make("command_lsb_torque", "scalar", "One output code", "N m"),
            OutputSpec::make("ripple_rad", "scalar",
                             "Peak-to-peak position ripple the quantisation adds", "rad"),
            OutputSpec::make("ripple_at_flange_m", "scalar", "The same ripple at the flange", "m"),
            OutputSpec::make("limit_cycle_amplitude_rad", "scalar", "Half the ripple", "rad"),
            OutputSpec::make("limit_cycle_period", "scalar", "Period of the hunting", "s"),
            OutputSpec::make("repeatability_spec_m", "scalar", "GP8 datasheet repeatability", "m"),
            OutputSpec::make("ripple_vs_spec", "scalar",
                             "Flange ripple divided by the datasheet repeatability"),
            OutputSpec::make("meets_repeatability", "bool",
                             "Is the quantisation ripple inside the datasheet figure"),
            OutputSpec::make("responses", "series_set",
                             "Quantised and ideal angle, plus the command"),
            OutputSpec::make("budget", "table", "Every term of the resolution budget"),
            OutputSpec::make("verdict", "text", "Whether this encoder can hold the specification"),
        };
        d.ops.push_back(std::move(op));
    }
    return d;
}

namespace {

[[nodiscard]] json::Value op_discretise(const json::Value& args) {
    const control::JointPlant plant = read_plant(args);
    const std::string method_text = optional_enum(args, "method", "zero_order_hold", kMethods);
    const Method method = digital::method_from_string(method_text);
    const double period = optional_scalar(args, "sample_period", 0.01, 1e-4, 1.0);
    const std::string system =
        optional_enum(args, "system", "closed_loop", {"plant", "closed_loop"});
    const double loop_gain = optional_scalar(args, "loop_gain", 1.0, 0.01, 500.0);
    const double duration = optional_scalar(args, "duration", 5.0, 0.01, 60.0);

    const control::TransferFunction continuous =
        (system == "closed_loop") ? plant.tf.scaled(loop_gain).closed_loop() : plant.tf;
    const DiscreteTransferFunction discrete = digital::discretise(continuous, period, method);

    const std::size_t steps = static_cast<std::size_t>(
        std::clamp(std::floor(duration / period) + 1.0, 4.0, 20000.0));
    const double aligned = static_cast<double>(steps - 1) * period;
    const control::Response sampled = continuous.step_response(aligned, steps);
    const std::vector<double> discrete_steps = discrete.step_response(steps);
    const std::size_t fine_points = std::clamp<std::size_t>(steps * 8, 200, 4000);
    const control::Response fine = continuous.step_response(aligned, fine_points);

    double max_error = 0.0;
    double sum_squares = 0.0;
    for (std::size_t i = 0; i < steps; ++i) {
        const double error = discrete_steps[i] - sampled.y[i];
        max_error = std::max(max_error, std::abs(error));
        sum_squares += error * error;
    }
    const double rms_error = std::sqrt(sum_squares / static_cast<double>(steps));

    const std::vector<digital::Complex> s_poles = continuous.poles();
    const std::vector<digital::Complex> z_poles = discrete.poles();
    std::vector<std::vector<json::Value>> mapping;
    mapping.reserve(std::max(s_poles.size(), z_poles.size()));
    for (std::size_t i = 0; i < s_poles.size(); ++i) {
        const digital::Complex exact = std::exp(s_poles[i] * period);
        const digital::Complex actual = (i < z_poles.size()) ? z_poles[i] : exact;
        mapping.push_back({json::Value(s_poles[i].real()), json::Value(s_poles[i].imag()),
                           json::Value(actual.real()), json::Value(actual.imag()),
                           json::Value(std::abs(actual)), json::Value(std::abs(exact)),
                           json::Value(std::abs(actual) < 1.0)});
    }

    json::Value response_set = json::Value::array();
    response_set.push_back(json::from_series("continuous", fine.t, fine.y));
    response_set.push_back(
        json::from_series("discrete (" + method_text + ")", sample_times(steps, period),
                          discrete_steps));

    json::Value out = json::Value::object();
    out.set("numerator", poly_table(discrete.numerator()));
    out.set("denominator", poly_table(discrete.denominator()));
    out.set("transfer_function", json::Value(discrete.to_string()));
    out.set("difference_equation", json::Value(discrete.difference_equation()));
    out.set("s_poles", json::from_complex_set(s_poles));
    out.set("z_poles", json::from_complex_set(z_poles));
    out.set("z_zeros", json::from_complex_set(discrete.zeros()));
    out.set("unit_circle", unit_circle(180));
    out.set("mapping", json::from_table({"Re(s)", "Im(s)", "Re(z)", "Im(z)", "|z|",
                                         "|e^{sT}|", "inside unit circle"},
                                        mapping));
    out.set("responses", std::move(response_set));
    out.set("max_error", json::Value(max_error));
    out.set("rms_error", json::Value(rms_error));
    out.set("spectral_radius", json::Value(discrete.spectral_radius()));
    out.set("stable", json::Value(discrete.is_stable()));
    out.set("nyquist_frequency", json::Value(kPi / period));
    out.set("substitution", json::Value(digital::method_substitution(method, period)));
    return out;
}

[[nodiscard]] json::Value op_sample_rate_effect(const json::Value& args) {
    const control::JointPlant plant = read_plant(args);
    const double kp = optional_scalar(args, "kp", 8.0, 0.0, 500.0);
    const double ki = optional_scalar(args, "ki", 4.0, 0.0, 500.0);
    const double kd = optional_scalar(args, "kd", 1.0, 0.0, 100.0);
    const double period_min = optional_scalar(args, "period_min", 0.0005, 1e-5, 0.1);
    const double period_max = optional_scalar(args, "period_max", 0.5, 1e-4, 2.0);
    const std::size_t points = static_cast<std::size_t>(optional_int(args, "samples", 40, 5, 200));
    const double duration = optional_scalar(args, "duration", 8.0, 0.1, 60.0);
    if (period_max <= period_min) {
        throw StudyError("parameter 'period_max' must be greater than 'period_min'");
    }

    std::vector<double> rates;
    std::vector<double> phase_margins;
    std::vector<double> overshoots;
    std::vector<double> radii;
    std::vector<std::vector<json::Value>> sweep;
    rates.reserve(points);
    phase_margins.reserve(points);
    overshoots.reserve(points);
    radii.reserve(points);
    sweep.reserve(points);

    double slowest_stable_period = 0.0;
    double fastest_unstable_period = 0.0;
    double slowest_period_meeting_45 = 0.0;

    for (std::size_t i = 0; i < points; ++i) {
        const double frac = static_cast<double>(i) / static_cast<double>(points - 1);
        const double period = std::pow(10.0, std::log10(period_min) +
                                                frac * (std::log10(period_max) -
                                                        std::log10(period_min)));
        const RatePoint point = evaluate_rate(plant.tf, kp, ki, kd, period, duration);
        rates.push_back(point.rate_hz);
        phase_margins.push_back(point.phase_margin);
        overshoots.push_back(std::min(point.overshoot, 500.0));
        radii.push_back(point.spectral_radius);
        sweep.push_back({json::Value(point.rate_hz), json::Value(point.period),
                         json::Value(point.phase_margin),
                         json::Value(std::min(point.overshoot, 1000.0)),
                         json::Value(point.spectral_radius), json::Value(point.stable)});
        if (point.stable) {
            slowest_stable_period = std::max(slowest_stable_period, period);
        } else if (fastest_unstable_period == 0.0 || period < fastest_unstable_period) {
            fastest_unstable_period = period;
        }
        if (point.phase_margin >= 45.0) {
            slowest_period_meeting_45 = std::max(slowest_period_meeting_45, period);
        }
    }

    double boundary_period = 0.0;
    if (slowest_stable_period > 0.0 && fastest_unstable_period > slowest_stable_period) {
        double low = slowest_stable_period;
        double high = fastest_unstable_period;
        for (int i = 0; i < 50; ++i) {
            const double mid = 0.5 * (low + high);
            if (evaluate_rate(plant.tf, kp, ki, kd, mid, duration).stable) {
                low = mid;
            } else {
                high = mid;
            }
        }
        boundary_period = high;
    }

    const control::TransferFunction continuous_loop =
        control::pid_controller(kp, ki, kd, 0.01).series(plant.tf);
    const control::Margins continuous_margins =
        control::loop_margins(continuous_loop, 1e-2, 1e4, 900);
    const double crossover = continuous_margins.gain_crossover_omega;
    const double boundary_rate = (boundary_period > 0.0) ? 1.0 / boundary_period : 0.0;

    std::string verdict;
    if (boundary_period > 0.0) {
        verdict = "This loop goes unstable at a sample rate of " +
                  json::number_to_string(boundary_rate) + " Hz (T = " +
                  json::number_to_string(boundary_period) +
                  " s): below that rate a discrete closed-loop pole leaves the unit circle and "
                  "the joint oscillates with growing amplitude. The continuous loop crosses over "
                  "at " + json::number_to_string(crossover) + " rad/s = " +
                  json::number_to_string(crossover / (2.0 * kPi)) + " Hz, so the boundary rate is " +
                  json::number_to_string(boundary_rate / std::max(crossover / (2.0 * kPi), 1e-9)) +
                  " times the crossover frequency - which is why 'sample at twice the bandwidth' "
                  "is a sampling theorem for signals and not a design rule for loops.";
    } else {
        verdict = "No sample period inside [" + json::number_to_string(period_min) + ", " +
                  json::number_to_string(period_max) +
                  "] s destabilises this loop; widen period_max or raise the gains to find the "
                  "boundary.";
    }
    if (slowest_period_meeting_45 > 0.0) {
        verdict += " A phase margin of 45 degrees survives down to " +
                   json::number_to_string(1.0 / slowest_period_meeting_45) + " Hz.";
    }

    json::Value margin_set = json::Value::array();
    margin_set.push_back(json::from_series("phase margin [deg]", rates, phase_margins));
    margin_set.push_back(json::from_series("overshoot [%]", rates, overshoots));
    json::Value stability_set = json::Value::array();
    stability_set.push_back(json::from_series("max |z| pole", rates, radii));
    std::vector<double> boundary(rates.size(), 1.0);
    stability_set.push_back(json::from_series("unit circle", rates, boundary));

    json::Value out = json::Value::object();
    out.set("unstable_sample_rate_hz", json::Value(boundary_rate));
    out.set("unstable_sample_period", json::Value(boundary_period));
    out.set("recommended_sample_rate_hz",
            json::Value(slowest_period_meeting_45 > 0.0 ? 1.0 / slowest_period_meeting_45 : 0.0));
    out.set("verdict", json::Value(verdict));
    out.set("margin_curve", std::move(margin_set));
    out.set("stability_curve", std::move(stability_set));
    out.set("sweep", json::from_table({"rate [Hz]", "period [s]", "PM [deg]", "overshoot [%]",
                                       "max |z|", "stable"},
                                      sweep));
    out.set("continuous_phase_margin",
            json::Value(continuous_margins.has_phase_margin ? continuous_margins.phase_margin_deg
                                                            : 180.0));
    out.set("crossover_frequency", json::Value(crossover));
    out.set("rate_to_crossover_ratio",
            json::Value(boundary_rate / std::max(crossover / (2.0 * kPi), 1e-9)));
    return out;
}

[[nodiscard]] double overshoot_of(const std::vector<double>& y, double reference) {
    if (y.empty() || std::abs(reference) <= 1e-12) {
        return 0.0;
    }
    double peak = y.front();
    for (const double v : y) {
        peak = std::max(peak, v);
    }
    return std::max(0.0, (peak - reference) / std::abs(reference) * 100.0);
}

[[nodiscard]] json::Value op_digital_pid(const json::Value& args) {
    const control::JointPlant plant = read_plant(args);
    const double kp = optional_scalar(args, "kp", 8.0, 0.0, 500.0);
    const double ki = optional_scalar(args, "ki", 6.0, 0.0, 500.0);
    const double kd = optional_scalar(args, "kd", 1.0, 0.0, 100.0);
    const double period = optional_scalar(args, "sample_period", 0.004, 1e-4, 0.5);
    const bool anti_windup = optional_bool(args, "anti_windup", true);
    const bool filter = optional_bool(args, "derivative_filter", true);
    const double limit = optional_scalar(args, "command_limit", 0.25, 0.02, 1.0);
    const double setpoint = optional_scalar(args, "setpoint", 1.0, -kPi, kPi);
    const double duration = optional_scalar(args, "duration", 6.0, 0.05, 60.0);

    SampledLoopOptions options;
    options.sample_period = period;
    options.duration = duration;
    options.setpoint = setpoint;
    options.kp = kp;
    options.ki = ki;
    options.kd = kd;
    options.derivative_filter = filter ? std::max(4.0 * period, 0.01) : 0.0;
    options.command_limit = limit;
    options.anti_windup = true;
    const digital::SampledLoopRun protected_run =
        digital::simulate_sampled_loop(plant.tf, options);
    options.anti_windup = false;
    const digital::SampledLoopRun unprotected_run =
        digital::simulate_sampled_loop(plant.tf, options);

    const double q0 = kp + 0.5 * ki * period + kd / period;
    const double q1 = -kp + 0.5 * ki * period - 2.0 * kd / period;
    const double q2 = kd / period;

    std::vector<std::vector<json::Value>> coefficients;
    coefficients.push_back({json::Value("q0 = Kp + Ki T/2 + Kd/T"), json::Value(q0)});
    coefficients.push_back({json::Value("q1 = -Kp + Ki T/2 - 2 Kd/T"), json::Value(q1)});
    coefficients.push_back({json::Value("q2 = Kd/T"), json::Value(q2)});
    coefficients.push_back({json::Value("Kp"), json::Value(kp)});
    coefficients.push_back({json::Value("Ki"), json::Value(ki)});
    coefficients.push_back({json::Value("Kd"), json::Value(kd)});
    coefficients.push_back({json::Value("T"), json::Value(period)});
    coefficients.push_back({json::Value("derivative filter time constant"),
                            json::Value(options.derivative_filter)});

    const double overshoot_protected = overshoot_of(protected_run.y, setpoint);
    const double overshoot_unprotected = overshoot_of(unprotected_run.y, setpoint);

    std::vector<std::vector<json::Value>> comparison;
    comparison.push_back({json::Value("overshoot [%]"), json::Value(overshoot_protected),
                          json::Value(overshoot_unprotected)});
    comparison.push_back({json::Value("peak integrator state"),
                          json::Value(protected_run.peak_integral),
                          json::Value(unprotected_run.peak_integral)});
    comparison.push_back({json::Value("final angle [rad]"), json::Value(protected_run.final_value),
                          json::Value(unprotected_run.final_value)});
    comparison.push_back({json::Value("samples against the limit [fraction]"),
                          json::Value(protected_run.saturated_fraction),
                          json::Value(unprotected_run.saturated_fraction)});

    json::Value response_set = json::Value::array();
    response_set.push_back(json::from_series("angle, anti-windup on", protected_run.t,
                                             protected_run.y));
    response_set.push_back(json::from_series("angle, anti-windup off", unprotected_run.t,
                                             unprotected_run.y));
    std::vector<double> reference(protected_run.t.size(), setpoint);
    response_set.push_back(json::from_series("setpoint", protected_run.t, reference));

    json::Value command_set = json::Value::array();
    command_set.push_back(json::from_series("command, anti-windup on", protected_run.t,
                                            protected_run.u));
    command_set.push_back(json::from_series("command, anti-windup off", unprotected_run.t,
                                            unprotected_run.u));
    json::Value integrator_set = json::Value::array();
    integrator_set.push_back(json::from_series("integrator, anti-windup on", protected_run.t,
                                               protected_run.integral));
    integrator_set.push_back(json::from_series("integrator, anti-windup off", unprotected_run.t,
                                               unprotected_run.integral));

    const double windup_ratio =
        (protected_run.peak_integral > 1e-12)
            ? unprotected_run.peak_integral / protected_run.peak_integral
            : 0.0;
    std::string verdict;
    if (unprotected_run.saturated_fraction <= 0.0) {
        verdict = "the actuator never reached its limit on this move, so windup had no chance to "
                  "happen: shrink command_limit or enlarge the step and the two curves will part";
    } else {
        verdict = "the actuator spent " +
                  json::number_to_string(100.0 * unprotected_run.saturated_fraction) +
                  " % of the samples against its limit; without conditional integration the "
                  "integrator wound up to " +
                  json::number_to_string(unprotected_run.peak_integral) + " (" +
                  json::number_to_string(windup_ratio) +
                  " times the protected peak) and the overshoot went from " +
                  json::number_to_string(overshoot_protected) + " % to " +
                  json::number_to_string(overshoot_unprotected) +
                  " %. The gains are identical in both runs - the only difference is one "
                  "conditional in the interrupt handler.";
    }
    if (!anti_windup) {
        verdict += " (anti_windup is off in the parameters, so the unprotected curve is the one "
                   "this configuration would actually run.)";
    }

    json::Value out = json::Value::object();
    out.set("coefficients", json::from_table({"coefficient", "value"}, coefficients));
    out.set("difference_equation",
            json::Value("u[k] = u[k-1] + " + json::number_to_string(q0) + " e[k] + (" +
                        json::number_to_string(q1) + ") e[k-1] + " + json::number_to_string(q2) +
                        " e[k-2], clamped to +/-" + json::number_to_string(limit) +
                        (anti_windup ? " with conditional integration" : " with no windup guard")));
    out.set("controller_z", json::Value(discrete_pid(kp, ki, kd, period).to_string()));
    out.set("responses", std::move(response_set));
    out.set("commands", std::move(command_set));
    out.set("integrators", std::move(integrator_set));
    out.set("comparison", json::from_table({"indicator", "anti-windup on", "anti-windup off"},
                                           comparison));
    out.set("peak_integral_protected", json::Value(protected_run.peak_integral));
    out.set("peak_integral_unprotected", json::Value(unprotected_run.peak_integral));
    out.set("saturated_fraction", json::Value(unprotected_run.saturated_fraction));
    out.set("overshoot_protected", json::Value(overshoot_protected));
    out.set("overshoot_unprotected", json::Value(overshoot_unprotected));
    out.set("verdict", json::Value(verdict));
    return out;
}

[[nodiscard]] json::Value op_quantisation(const json::Value& args) {
    const control::JointPlant plant = read_plant(args);
    const Eigen::Matrix<double, 6, 1> q =
        optional_vec6(args, "q", zero6(), -widest_joint_range(), widest_joint_range());
    const int encoder_bits = optional_int(args, "encoder_bits", 17, 8, 24);
    const int word_bits = optional_int(args, "word_length_bits", 12, 6, 24);
    const double kp = optional_scalar(args, "kp", 8.0, 0.0, 500.0);
    const double ki = optional_scalar(args, "ki", 6.0, 0.0, 500.0);
    const double kd = optional_scalar(args, "kd", 1.0, 0.0, 100.0);
    const double period = optional_scalar(args, "sample_period", 0.004, 1e-4, 0.1);
    const double limit = optional_scalar(args, "command_limit", 1.0, 0.02, 1.0);
    const double setpoint = optional_scalar(args, "setpoint", 0.5, -kPi, kPi);
    const double duration = optional_scalar(args, "duration", 12.0, 0.2, 60.0);

    const double counts = std::pow(2.0, static_cast<double>(encoder_bits));
    const double resolution = 2.0 * kPi / (counts * plant.gear_ratio);
    const double arm = flange_moment_arm(plant.joint, q);
    const double flange_resolution = resolution * arm;
    const double command_lsb = 2.0 * limit / std::pow(2.0, static_cast<double>(word_bits));
    const double command_lsb_torque = command_lsb * plant.drive_torque;

    SampledLoopOptions options;
    options.sample_period = period;
    options.duration = duration;
    options.setpoint = setpoint;
    options.kp = kp;
    options.ki = ki;
    options.kd = kd;
    options.derivative_filter = std::max(4.0 * period, 0.01);
    options.command_limit = limit;
    options.anti_windup = true;
    options.measurement_quantum = resolution;
    options.command_quantum = command_lsb;
    const digital::SampledLoopRun quantised = digital::simulate_sampled_loop(plant.tf, options);
    options.measurement_quantum = 0.0;
    options.command_quantum = 0.0;
    const digital::SampledLoopRun ideal = digital::simulate_sampled_loop(plant.tf, options);

    // The ripple is measured, not estimated - and it is measured against the
    // infinite-resolution run rather than against the setpoint, because the
    // quantity asked for is what QUANTISATION costs. Measuring the quantised
    // trace against the setpoint would report the tail of the tracking
    // transient, which is a property of the gains and not of the encoder.
    const std::size_t total = std::min(quantised.y.size(), ideal.y.size());
    const std::size_t start = total - total / 4;
    double lowest = std::numeric_limits<double>::max();
    double highest = -std::numeric_limits<double>::max();
    double mean = 0.0;
    for (std::size_t i = start; i < total; ++i) {
        const double difference = quantised.y[i] - ideal.y[i];
        lowest = std::min(lowest, difference);
        highest = std::max(highest, difference);
        mean += difference;
    }
    const std::size_t tail = total - start;
    mean /= static_cast<double>(std::max<std::size_t>(tail, 1));
    const double ripple = (tail > 0) ? (highest - lowest) : 0.0;

    std::size_t crossings = 0;
    for (std::size_t i = start + 1; i < total; ++i) {
        const double a = (quantised.y[i - 1] - ideal.y[i - 1]) - mean;
        const double b = (quantised.y[i] - ideal.y[i]) - mean;
        if ((a > 0.0 && b <= 0.0) || (a < 0.0 && b >= 0.0)) {
            ++crossings;
        }
    }
    const double tail_duration = static_cast<double>(tail) * period;
    const double limit_cycle_period =
        (crossings >= 2) ? (2.0 * tail_duration / static_cast<double>(crossings)) : 0.0;

    const double flange_ripple = ripple * arm;
    const bool meets = flange_ripple <= GP8_REPEATABILITY_M;

    std::vector<std::vector<json::Value>> budget;
    budget.push_back({json::Value("encoder bits"), json::Value(encoder_bits), json::Value("bit"),
                      json::Value("parameter")});
    budget.push_back({json::Value("counts per motor revolution"), json::Value(counts),
                      json::Value("count"), json::Value("2^bits")});
    budget.push_back({json::Value("gear ratio n"), json::Value(plant.gear_ratio),
                      json::Value("motor/joint"),
                      json::Value("gp8_model.hpp GP8_LINKS[" + std::to_string(plant.joint) +
                                  "].gear_ratio")});
    budget.push_back({json::Value("joint-side quantum"), json::Value(resolution),
                      json::Value("rad"), json::Value("2 pi / (2^bits n)")});
    budget.push_back({json::Value("moment arm to the flange"), json::Value(arm), json::Value("m"),
                      json::Value("perpendicular distance from this axis to the DH flange at q")});
    budget.push_back({json::Value("one count at the flange"), json::Value(flange_resolution),
                      json::Value("m"), json::Value("quantum x arm")});
    budget.push_back({json::Value("measured ripple at the flange"), json::Value(flange_ripple),
                      json::Value("m"),
                      json::Value("peak to peak of (quantised - infinite resolution) over the "
                                  "last quarter of the run")});
    budget.push_back({json::Value("datasheet repeatability"), json::Value(GP8_REPEATABILITY_M),
                      json::Value("m"), json::Value("gp8_model.hpp GP8_REPEATABILITY_M")});

    json::Value response_set = json::Value::array();
    response_set.push_back(json::from_series("angle, quantised", quantised.t, quantised.y));
    response_set.push_back(json::from_series("angle, infinite resolution", ideal.t, ideal.y));
    response_set.push_back(json::from_series("command, quantised", quantised.t, quantised.u));

    std::string verdict =
        "One encoder count is " + json::number_to_string(flange_resolution * 1e6) +
        " micrometres at the flange and the ripple the quantisation adds is " +
        json::number_to_string(flange_ripple * 1e6) + " micrometres, against the datasheet's " +
        json::number_to_string(GP8_REPEATABILITY_M * 1e6) + " micrometres. ";
    verdict += meets ? "This resolution and word length hold the specification."
                     : "This resolution and word length cannot hold the specification: the loop "
                       "hunts across more than one count, so either the encoder needs more bits "
                       "or the gains need to be softer near the target.";
    if (limit_cycle_period > 0.0) {
        verdict += " The hunting period is " + json::number_to_string(limit_cycle_period) +
                   " s, i.e. " + json::number_to_string(1.0 / limit_cycle_period) + " Hz.";
    } else {
        verdict += " No limit cycle was detected in the last quarter of the run: the joint came "
                   "to rest inside one count.";
    }

    json::Value out = json::Value::object();
    out.set("encoder_resolution_rad", json::Value(resolution));
    out.set("encoder_resolution_deg", json::Value(resolution * 180.0 / kPi));
    out.set("flange_moment_arm", json::Value(arm));
    out.set("resolution_at_flange_m", json::Value(flange_resolution));
    out.set("command_lsb", json::Value(command_lsb));
    out.set("command_lsb_torque", json::Value(command_lsb_torque));
    out.set("ripple_rad", json::Value(ripple));
    out.set("ripple_at_flange_m", json::Value(flange_ripple));
    out.set("limit_cycle_amplitude_rad", json::Value(0.5 * ripple));
    out.set("limit_cycle_period", json::Value(limit_cycle_period));
    out.set("repeatability_spec_m", json::Value(GP8_REPEATABILITY_M));
    out.set("ripple_vs_spec", json::Value(flange_ripple / GP8_REPEATABILITY_M));
    out.set("meets_repeatability", json::Value(meets));
    out.set("responses", std::move(response_set));
    out.set("budget", json::from_table({"term", "value", "unit", "source"}, budget));
    out.set("verdict", json::Value(verdict));
    return out;
}

}  // namespace

json::Value DigitalControlModule::invoke(std::string_view op, const json::Value& args) const {
    if (op == "discretise") {
        return op_discretise(args);
    }
    if (op == "sample_rate_effect") {
        return op_sample_rate_effect(args);
    }
    if (op == "digital_pid") {
        return op_digital_pid(args);
    }
    if (op == "quantisation") {
        return op_quantisation(args);
    }
    unknown_op(name(), op);
}

}  // namespace yaskawa::study
