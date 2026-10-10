#include "study/control_system.hpp"

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

// ---------------------------------------------------------------------------
// The reusable transfer-function core of namespace control. Everything the
// module ops do above it - block algebra, stability, the locus, PID - is built
// from these few functions, so there is one polynomial multiply and one
// integrator in the project rather than one per op.
// ---------------------------------------------------------------------------

namespace control {

namespace {

constexpr double kPolyEps = 1e-14;
constexpr double kStabilityTol = 1e-9;

}  // namespace

// ---------------------------------------------------------------------------
// Polynomials
// ---------------------------------------------------------------------------

Poly poly_trim(Poly p) {
    if (p.empty()) {
        return Poly{0.0};
    }
    double scale = 0.0;
    for (const double c : p) {
        scale = std::max(scale, std::abs(c));
    }
    const double threshold = kPolyEps * std::max(1.0, scale);
    std::size_t first = 0;
    while (first + 1 < p.size() && std::abs(p[first]) <= threshold) {
        ++first;
    }
    return Poly(p.begin() + static_cast<std::ptrdiff_t>(first), p.end());
}

Poly poly_multiply(const Poly& a, const Poly& b) {
    if (a.empty() || b.empty()) {
        return Poly{0.0};
    }
    Poly out(a.size() + b.size() - 1, 0.0);
    for (std::size_t i = 0; i < a.size(); ++i) {
        for (std::size_t j = 0; j < b.size(); ++j) {
            out[i + j] += a[i] * b[j];
        }
    }
    return out;
}

Poly poly_add(const Poly& a, const Poly& b) {
    const std::size_t n = std::max(a.size(), b.size());
    Poly out(n, 0.0);
    for (std::size_t i = 0; i < a.size(); ++i) {
        out[n - a.size() + i] += a[i];
    }
    for (std::size_t i = 0; i < b.size(); ++i) {
        out[n - b.size() + i] += b[i];
    }
    return out;
}

Poly poly_subtract(const Poly& a, const Poly& b) { return poly_add(a, poly_scale(b, -1.0)); }

Poly poly_scale(const Poly& a, double k) {
    Poly out = a;
    for (double& c : out) {
        c *= k;
    }
    return out;
}

Poly poly_power(const Poly& a, std::size_t exponent) {
    Poly out{1.0};
    for (std::size_t i = 0; i < exponent; ++i) {
        out = poly_multiply(out, a);
    }
    return out;
}

Complex poly_eval(const Poly& a, const Complex& s) {
    Complex out(0.0, 0.0);
    for (const double c : a) {
        out = out * s + c;
    }
    return out;
}

std::string poly_to_string(const Poly& a, std::string_view variable) {
    const Poly p = poly_trim(a);
    const std::size_t degree = p.size() - 1;
    std::string out;
    for (std::size_t i = 0; i < p.size(); ++i) {
        const double c = p[i];
        if (std::abs(c) <= kPolyEps && p.size() > 1) {
            continue;
        }
        const std::size_t power = degree - i;
        if (!out.empty()) {
            out += (c < 0.0) ? " - " : " + ";
        } else if (c < 0.0) {
            out += "-";
        }
        const double m = std::abs(c);
        const bool unit = (std::abs(m - 1.0) <= kPolyEps) && power > 0;
        if (!unit) {
            out += json::number_to_string(m);
            if (power > 0) {
                out += " ";
            }
        }
        if (power > 0) {
            out += variable;
            if (power > 1) {
                out += "^" + std::to_string(power);
            }
        }
    }
    return out.empty() ? std::string("0") : out;
}

std::vector<Complex> poly_roots(const Poly& a) {
    const Poly p = poly_trim(a);
    if (p.size() <= 1) {
        return {};
    }
    const Eigen::Index n = static_cast<Eigen::Index>(p.size() - 1);
    Eigen::MatrixXd companion = Eigen::MatrixXd::Zero(n, n);
    for (Eigen::Index k = 0; k < n; ++k) {
        companion(0, k) = -p[static_cast<std::size_t>(k) + 1] / p[0];
    }
    for (Eigen::Index k = 1; k < n; ++k) {
        companion(k, k - 1) = 1.0;
    }
    Eigen::EigenSolver<Eigen::MatrixXd> solver(companion, false);
    std::vector<Complex> out;
    out.reserve(static_cast<std::size_t>(n));
    const auto values = solver.eigenvalues();
    for (Eigen::Index i = 0; i < values.size(); ++i) {
        out.emplace_back(values(i).real(), values(i).imag());
    }
    return out;
}

CharacteristicPoly characteristic_poly(const Eigen::MatrixXd& A) {
    const Eigen::Index n = A.rows();
    CharacteristicPoly out;
    out.coefficients.assign(static_cast<std::size_t>(n) + 1, 0.0);
    out.coefficients[0] = 1.0;
    if (n == 0) {
        return out;
    }
    const Eigen::MatrixXd I = Eigen::MatrixXd::Identity(n, n);
    Eigen::MatrixXd M = I;
    out.adjugate.reserve(static_cast<std::size_t>(n));
    for (Eigen::Index k = 1; k <= n; ++k) {
        if (k > 1) {
            M = A * M + out.coefficients[static_cast<std::size_t>(k) - 1] * I;
        }
        out.adjugate.push_back(M);
        out.coefficients[static_cast<std::size_t>(k)] =
            -(A * M).trace() / static_cast<double>(k);
    }
    return out;
}

Eigen::MatrixXd matrix_exponential(const Eigen::MatrixXd& A) {
    const Eigen::Index n = A.rows();
    if (n == 0) {
        return A;
    }
    double norm = A.cwiseAbs().rowwise().sum().maxCoeff();
    int squarings = 0;
    while (norm > 0.5 && squarings < 60) {
        norm *= 0.5;
        ++squarings;
    }
    const Eigen::MatrixXd scaled = A / std::pow(2.0, squarings);
    Eigen::MatrixXd result = Eigen::MatrixXd::Identity(n, n);
    Eigen::MatrixXd term = Eigen::MatrixXd::Identity(n, n);
    for (int k = 1; k <= 20; ++k) {
        term = (term * scaled) / static_cast<double>(k);
        result += term;
    }
    for (int i = 0; i < squarings; ++i) {
        result = result * result;
    }
    return result;
}

// ---------------------------------------------------------------------------
// TransferFunction
// ---------------------------------------------------------------------------

TransferFunction::TransferFunction() : num_{1.0}, den_{1.0} {}

TransferFunction::TransferFunction(Poly numerator, Poly denominator) {
    num_ = poly_trim(std::move(numerator));
    den_ = poly_trim(std::move(denominator));
    if (den_.size() == 1 && std::abs(den_[0]) <= kPolyEps) {
        throw StudyError("transfer-function denominator must not be the zero polynomial");
    }
    for (const double c : num_) {
        if (!std::isfinite(c)) {
            throw StudyError("transfer-function numerator holds a non-finite coefficient");
        }
    }
    for (const double c : den_) {
        if (!std::isfinite(c)) {
            throw StudyError("transfer-function denominator holds a non-finite coefficient");
        }
    }
}

TransferFunction TransferFunction::gain(double k) { return TransferFunction({k}, {1.0}); }

TransferFunction TransferFunction::integrator(double k) {
    return TransferFunction({k}, {1.0, 0.0});
}

TransferFunction TransferFunction::differentiator(double k) {
    // A real differentiator is always filtered; the ideal K s is improper and
    // has no state-space realisation, so the pole is explicit instead of
    // hidden. tau is one hundredth of the gain's own time scale.
    const double tau = 1.0e-2 * std::max(1.0, std::abs(k)) / std::max(1.0, std::abs(k));
    return TransferFunction({k, 0.0}, {tau, 1.0});
}

TransferFunction TransferFunction::first_order_lag(double k, double tau) {
    if (!(tau > 0.0)) {
        throw StudyError("first-order lag needs a positive time constant");
    }
    return TransferFunction({k}, {tau, 1.0});
}

TransferFunction TransferFunction::second_order(double k, double zeta, double omega_n) {
    if (!(omega_n > 0.0)) {
        throw StudyError("second-order element needs a positive natural frequency");
    }
    return TransferFunction({k * omega_n * omega_n},
                            {1.0, 2.0 * zeta * omega_n, omega_n * omega_n});
}

TransferFunction TransferFunction::lead_lag(double k, double lead_tau, double lag_tau) {
    if (!(lag_tau > 0.0) || !(lead_tau >= 0.0)) {
        throw StudyError("lead-lag needs a positive lag time constant and a non-negative lead");
    }
    return TransferFunction({k * lead_tau, k}, {lag_tau, 1.0});
}

TransferFunction TransferFunction::pade_delay(double dead_time) {
    if (dead_time <= 0.0) {
        return TransferFunction();
    }
    const double h = 0.5 * dead_time;
    return TransferFunction({-h, 1.0}, {h, 1.0});
}

Complex TransferFunction::evaluate(const Complex& s) const {
    const Complex d = poly_eval(den_, s);
    const Complex n = poly_eval(num_, s);
    if (std::abs(d) <= 0.0) {
        return Complex(std::numeric_limits<double>::infinity(), 0.0);
    }
    return n / d;
}

Complex TransferFunction::frequency_response(double omega) const {
    return evaluate(Complex(0.0, omega));
}

double TransferFunction::dc_gain() const {
    const double d = den_.back();
    const double n = num_.back();
    if (std::abs(d) <= kPolyEps) {
        return (std::abs(n) <= kPolyEps) ? 0.0
                                         : std::copysign(std::numeric_limits<double>::infinity(), n);
    }
    return n / d;
}

std::vector<Complex> TransferFunction::poles() const { return poly_roots(den_); }

std::vector<Complex> TransferFunction::zeros() const { return poly_roots(num_); }

bool TransferFunction::is_stable() const {
    for (const auto& p : poles()) {
        if (p.real() >= -kStabilityTol) {
            return false;
        }
    }
    return true;
}

int TransferFunction::rhp_pole_count() const {
    int count = 0;
    for (const auto& p : poles()) {
        if (p.real() > kStabilityTol) {
            ++count;
        }
    }
    return count;
}

int TransferFunction::imaginary_axis_pole_count() const {
    int count = 0;
    for (const auto& p : poles()) {
        if (std::abs(p.real()) <= 1e-7) {
            ++count;
        }
    }
    return count;
}

TransferFunction TransferFunction::scaled(double k) const {
    return TransferFunction(poly_scale(num_, k), den_);
}

TransferFunction TransferFunction::series(const TransferFunction& other) const {
    return TransferFunction(poly_multiply(num_, other.num_), poly_multiply(den_, other.den_));
}

TransferFunction TransferFunction::parallel(const TransferFunction& other) const {
    return TransferFunction(poly_add(poly_multiply(num_, other.den_),
                                     poly_multiply(other.num_, den_)),
                            poly_multiply(den_, other.den_));
}

TransferFunction TransferFunction::feedback(const TransferFunction& h, bool negative) const {
    const Poly forward = poly_multiply(num_, h.denominator());
    const Poly loop = poly_multiply(num_, h.numerator());
    const Poly base = poly_multiply(den_, h.denominator());
    const Poly denominator = negative ? poly_add(base, loop) : poly_subtract(base, loop);
    return TransferFunction(forward, denominator);
}

TransferFunction TransferFunction::closed_loop() const {
    return TransferFunction(num_, poly_add(den_, num_));
}

StateSpace TransferFunction::state_space() const {
    if (num_.size() > den_.size()) {
        throw StudyError("an improper transfer function (numerator degree " +
                         std::to_string(num_.size() - 1) + " > denominator degree " +
                         std::to_string(den_.size() - 1) + ") has no state-space realisation");
    }
    const double lead = den_[0];
    Poly d = poly_scale(den_, 1.0 / lead);
    Poly n = poly_scale(num_, 1.0 / lead);

    const Eigen::Index order = static_cast<Eigen::Index>(d.size() - 1);
    StateSpace ss;
    if (order == 0) {
        ss.A = Eigen::MatrixXd::Zero(0, 0);
        ss.B = Eigen::VectorXd::Zero(0);
        ss.C = Eigen::RowVectorXd::Zero(0);
        ss.D = n[0];
        return ss;
    }

    Poly b(d.size(), 0.0);
    for (std::size_t i = 0; i < n.size(); ++i) {
        b[d.size() - n.size() + i] = n[i];
    }

    ss.A = Eigen::MatrixXd::Zero(order, order);
    for (Eigen::Index k = 0; k < order; ++k) {
        ss.A(0, k) = -d[static_cast<std::size_t>(k) + 1];
    }
    for (Eigen::Index k = 1; k < order; ++k) {
        ss.A(k, k - 1) = 1.0;
    }
    ss.B = Eigen::VectorXd::Zero(order);
    ss.B(0) = 1.0;
    ss.C = Eigen::RowVectorXd::Zero(order);
    for (Eigen::Index k = 0; k < order; ++k) {
        const std::size_t i = static_cast<std::size_t>(k) + 1;
        ss.C(k) = b[i] - b[0] * d[i];
    }
    ss.D = b[0];
    return ss;
}

Response TransferFunction::simulate(double duration, std::size_t samples, double input_level,
                                    bool from_impulse) const {
    if (!(duration > 0.0)) {
        throw StudyError("simulation duration must be positive");
    }
    if (samples < 2) {
        throw StudyError("a simulation needs at least 2 samples");
    }
    const StateSpace ss = state_space();
    const Eigen::Index n = ss.A.rows();
    const double dt = duration / static_cast<double>(samples - 1);

    double fastest = 0.0;
    for (const auto& p : poles()) {
        fastest = std::max(fastest, std::abs(p));
    }
    std::size_t substeps = 1;
    if (fastest > 0.0) {
        const double wanted = std::ceil(dt * fastest / 0.05);
        substeps = static_cast<std::size_t>(std::clamp(wanted, 1.0, 1024.0));
    }
    const double h = dt / static_cast<double>(substeps);

    Response out;
    out.t.reserve(samples);
    out.y.reserve(samples);

    // Every buffer is sized once; the sample loop below allocates nothing.
    Eigen::VectorXd x = Eigen::VectorXd::Zero(n);
    Eigen::VectorXd k1 = Eigen::VectorXd::Zero(n);
    Eigen::VectorXd k2 = Eigen::VectorXd::Zero(n);
    Eigen::VectorXd k3 = Eigen::VectorXd::Zero(n);
    Eigen::VectorXd k4 = Eigen::VectorXd::Zero(n);
    Eigen::VectorXd tmp = Eigen::VectorXd::Zero(n);

    double u = input_level;
    if (from_impulse) {
        // y(t) = C e^{At} B: the impulse enters as the initial state. The
        // D delta(t) term of a non-strictly-proper plant is not drawn.
        x = ss.B * input_level;
        u = 0.0;
    }

    for (std::size_t k = 0; k < samples; ++k) {
        out.t.push_back(static_cast<double>(k) * dt);
        double y = ss.D * u;
        if (n > 0) {
            y += (ss.C * x)(0);
        }
        out.y.push_back(y);
        if (k + 1 == samples || n == 0) {
            if (k + 1 == samples) {
                break;
            }
            continue;
        }
        for (std::size_t s = 0; s < substeps; ++s) {
            k1.noalias() = ss.A * x;
            k1 += ss.B * u;
            tmp = x + (0.5 * h) * k1;
            k2.noalias() = ss.A * tmp;
            k2 += ss.B * u;
            tmp = x + (0.5 * h) * k2;
            k3.noalias() = ss.A * tmp;
            k3 += ss.B * u;
            tmp = x + h * k3;
            k4.noalias() = ss.A * tmp;
            k4 += ss.B * u;
            x += (h / 6.0) * (k1 + 2.0 * k2 + 2.0 * k3 + k4);
        }
    }
    return out;
}

Response TransferFunction::step_response(double duration, std::size_t samples,
                                         double amplitude) const {
    return simulate(duration, samples, amplitude, false);
}

Response TransferFunction::impulse_response(double duration, std::size_t samples) const {
    return simulate(duration, samples, 1.0, true);
}

std::string TransferFunction::to_string(std::string_view variable) const {
    return "(" + poly_to_string(num_, variable) + ") / (" + poly_to_string(den_, variable) + ")";
}

// ---------------------------------------------------------------------------
// Transient specification
// ---------------------------------------------------------------------------

TransientSpec transient_spec(const Response& response, double reference, double final_value,
                             double band) {
    TransientSpec spec;
    spec.settling_band = band;
    spec.steady_state_value = final_value;
    spec.steady_state_error = reference - final_value;
    if (response.y.empty()) {
        return spec;
    }
    const double scale = std::max(std::abs(final_value), 1e-12);
    const double sign = (final_value < 0.0) ? -1.0 : 1.0;

    std::size_t peak_index = 0;
    for (std::size_t i = 1; i < response.y.size(); ++i) {
        if (sign * response.y[i] > sign * response.y[peak_index]) {
            peak_index = i;
        }
    }
    spec.peak_value = response.y[peak_index];
    spec.peak_time = response.t[peak_index];
    spec.overshoot_percent = std::max(0.0, (sign * (spec.peak_value - final_value)) / scale * 100.0);

    const double low = 0.1 * final_value;
    const double high = 0.9 * final_value;
    double t_low = 0.0;
    double t_high = 0.0;
    bool have_low = false;
    bool have_high = false;
    for (std::size_t i = 1; i < response.y.size(); ++i) {
        const double a = response.y[i - 1];
        const double b = response.y[i];
        if (!have_low && ((a - low) * (b - low) <= 0.0) && a != b) {
            t_low = response.t[i - 1] + (low - a) / (b - a) * (response.t[i] - response.t[i - 1]);
            have_low = true;
        }
        if (have_low && !have_high && ((a - high) * (b - high) <= 0.0) && a != b) {
            t_high = response.t[i - 1] + (high - a) / (b - a) * (response.t[i] - response.t[i - 1]);
            have_high = true;
            break;
        }
    }
    spec.rise_measured = have_low && have_high;
    spec.rise_time = spec.rise_measured ? (t_high - t_low) : 0.0;

    const double tolerance = band * scale;
    std::size_t last_outside = response.y.size();
    for (std::size_t i = response.y.size(); i-- > 0;) {
        if (std::abs(response.y[i] - final_value) > tolerance) {
            last_outside = i;
            break;
        }
    }
    if (last_outside == response.y.size()) {
        spec.settling_time = 0.0;
        spec.settled = true;
    } else if (last_outside + 1 < response.y.size()) {
        spec.settling_time = response.t[last_outside + 1];
        spec.settled = true;
    } else {
        spec.settling_time = response.t.back();
        spec.settled = false;
    }
    return spec;
}

// ---------------------------------------------------------------------------
// Frequency response and margins
// ---------------------------------------------------------------------------

BodeData bode(const TransferFunction& loop, double omega_min, double omega_max,
              std::size_t points) {
    if (!(omega_min > 0.0) || !(omega_max > omega_min)) {
        throw StudyError("the frequency range needs 0 < omega_min < omega_max");
    }
    if (points < 2) {
        throw StudyError("a Bode sweep needs at least 2 points");
    }
    BodeData data;
    data.omega.reserve(points);
    data.magnitude_db.reserve(points);
    data.phase_deg.reserve(points);

    const double log_min = std::log10(omega_min);
    const double log_max = std::log10(omega_max);
    double previous = 0.0;
    double offset = 0.0;
    for (std::size_t i = 0; i < points; ++i) {
        const double frac = static_cast<double>(i) / static_cast<double>(points - 1);
        const double omega = std::pow(10.0, log_min + frac * (log_max - log_min));
        const Complex value = loop.frequency_response(omega);
        const double magnitude = std::abs(value);
        double phase = std::arg(value);
        if (i > 0) {
            // Unwrap, so a margin search sees a continuous curve.
            while (phase + offset - previous > std::numbers::pi) {
                offset -= 2.0 * std::numbers::pi;
            }
            while (phase + offset - previous < -std::numbers::pi) {
                offset += 2.0 * std::numbers::pi;
            }
        }
        phase += offset;
        previous = phase;
        data.omega.push_back(omega);
        data.magnitude_db.push_back(20.0 * std::log10(std::max(magnitude, 1e-300)));
        data.phase_deg.push_back(phase * 180.0 / std::numbers::pi);
    }
    return data;
}

Margins margins_from_bode(const BodeData& data) {
    Margins m;
    const std::size_t n = data.omega.size();
    for (std::size_t i = 1; i < n && !m.has_phase_margin; ++i) {
        const double a = data.magnitude_db[i - 1];
        const double b = data.magnitude_db[i];
        if ((a > 0.0 && b <= 0.0) || (a < 0.0 && b >= 0.0)) {
            const double frac = (a == b) ? 0.0 : a / (a - b);
            m.gain_crossover_omega =
                std::pow(10.0, std::log10(data.omega[i - 1]) +
                                   frac * (std::log10(data.omega[i]) - std::log10(data.omega[i - 1])));
            const double phase =
                data.phase_deg[i - 1] + frac * (data.phase_deg[i] - data.phase_deg[i - 1]);
            m.phase_margin_deg = 180.0 + phase;
            m.has_phase_margin = true;
        }
    }
    for (std::size_t i = 1; i < n && !m.has_gain_margin; ++i) {
        const double a = data.phase_deg[i - 1] + 180.0;
        const double b = data.phase_deg[i] + 180.0;
        if ((a > 0.0 && b <= 0.0) || (a < 0.0 && b >= 0.0)) {
            const double frac = (a == b) ? 0.0 : a / (a - b);
            m.phase_crossover_omega =
                std::pow(10.0, std::log10(data.omega[i - 1]) +
                                   frac * (std::log10(data.omega[i]) - std::log10(data.omega[i - 1])));
            const double magnitude =
                data.magnitude_db[i - 1] + frac * (data.magnitude_db[i] - data.magnitude_db[i - 1]);
            m.gain_margin_db = -magnitude;
            m.gain_margin_linear = std::pow(10.0, m.gain_margin_db / 20.0);
            m.has_gain_margin = true;
        }
    }
    return m;
}

Margins loop_margins(const TransferFunction& loop, double omega_min, double omega_max,
                     std::size_t points) {
    return margins_from_bode(bode(loop, omega_min, omega_max, points));
}

// ---------------------------------------------------------------------------
// Routh-Hurwitz
// ---------------------------------------------------------------------------

RouthArray routh_array(const Poly& characteristic) {
    const Poly p = poly_trim(characteristic);
    RouthArray out;
    if (p.size() < 2) {
        out.note = "a characteristic polynomial of degree 0 has no roots to judge";
        out.stable = true;
        return out;
    }
    const std::size_t degree = p.size() - 1;
    const std::size_t width = degree / 2 + 1;

    double scale = 0.0;
    for (const double c : p) {
        scale = std::max(scale, std::abs(c));
    }
    const double epsilon = 1e-9 * std::max(1.0, scale);

    std::vector<std::vector<double>> rows(degree + 1, std::vector<double>(width, 0.0));
    for (std::size_t i = 0; i <= degree; ++i) {
        const std::size_t column = i / 2;
        if ((i % 2) == 0) {
            rows[0][column] = p[i];
        } else {
            rows[1][column] = p[i];
        }
    }

    bool all_zero_first_row_two = true;
    for (std::size_t c = 0; c < width; ++c) {
        if (std::abs(rows[1][c]) > 0.0) {
            all_zero_first_row_two = false;
        }
    }
    if (all_zero_first_row_two && degree >= 1) {
        // Degree-1 polynomials put nothing in the second row but the constant
        // term; a genuinely empty row means an even/odd polynomial.
        out.auxiliary_used = true;
        for (std::size_t c = 0; c + 1 < width || c == 0; ++c) {
            const double power = static_cast<double>(degree) - 2.0 * static_cast<double>(c);
            rows[1][c] = rows[0][c] * power;
            if (c + 1 >= width) {
                break;
            }
        }
    }

    for (std::size_t r = 2; r <= degree; ++r) {
        if (std::abs(rows[r - 1][0]) <= epsilon) {
            bool whole_row_zero = true;
            for (std::size_t c = 0; c < width; ++c) {
                if (std::abs(rows[r - 1][c]) > epsilon) {
                    whole_row_zero = false;
                    break;
                }
            }
            if (whole_row_zero) {
                // Replace the vanished row by the derivative of the auxiliary
                // polynomial it stands for: the marginal-stability case.
                out.auxiliary_used = true;
                const std::size_t order = degree - (r - 2);
                for (std::size_t c = 0; c < width; ++c) {
                    const double power = static_cast<double>(order) - 2.0 * static_cast<double>(c);
                    rows[r - 1][c] = rows[r - 2][c] * power;
                }
            } else {
                out.epsilon_used = true;
                rows[r - 1][0] = epsilon;
            }
        }
        for (std::size_t c = 0; c + 1 < width; ++c) {
            const double a = rows[r - 2][0];
            const double b = rows[r - 1][0];
            rows[r][c] = (b * rows[r - 2][c + 1] - a * rows[r - 1][c + 1]) / b;
        }
    }

    out.rows = std::move(rows);
    out.labels.reserve(degree + 1);
    for (std::size_t i = 0; i <= degree; ++i) {
        out.labels.push_back("s^" + std::to_string(degree - i));
    }

    const double first = out.rows[0][0];
    const double reference = (first < 0.0) ? -1.0 : 1.0;
    int changes = 0;
    double previous = reference * first;
    for (std::size_t r = 1; r <= degree; ++r) {
        const double value = reference * out.rows[r][0];
        if (std::abs(value) <= 0.0) {
            continue;
        }
        if (value * previous < 0.0) {
            ++changes;
        }
        previous = value;
    }
    out.sign_changes = changes;
    out.stable = (changes == 0) && !out.auxiliary_used;
    if (out.auxiliary_used) {
        out.note =
            "a row of the array vanished, so the polynomial has a pair of roots symmetric about "
            "the origin: the loop is marginally stable at best, never asymptotically stable";
    } else if (out.epsilon_used) {
        out.note =
            "a leading element was zero and was replaced by a small positive epsilon, the "
            "standard special case; the sign count below is the limit as epsilon -> 0+";
    } else {
        out.note = out.stable ? "no sign change in the first column: every root is in the left half plane"
                              : "the first column changes sign " + std::to_string(changes) +
                                    " time(s), so that many roots lie in the right half plane";
    }
    return out;
}

// ---------------------------------------------------------------------------
// Nyquist
// ---------------------------------------------------------------------------

NyquistResult nyquist(const TransferFunction& loop, double omega_min, double omega_max,
                      std::size_t points) {
    if (!(omega_min > 0.0) || !(omega_max > omega_min)) {
        throw StudyError("the Nyquist sweep needs 0 < omega_min < omega_max");
    }
    if (points < 8) {
        throw StudyError("a Nyquist contour needs at least 8 points per segment");
    }
    NyquistResult out;
    out.open_loop_rhp_poles = loop.rhp_pole_count();
    const int axis_poles = loop.imaginary_axis_pole_count();

    std::vector<Complex> path;
    path.reserve(3 * points + 32);
    const double log_min = std::log10(omega_min);
    const double log_max = std::log10(omega_max);

    // Up the imaginary axis from -omega_max to -omega_min.
    for (std::size_t i = 0; i < points; ++i) {
        const double frac = static_cast<double>(i) / static_cast<double>(points - 1);
        const double omega = std::pow(10.0, log_max + frac * (log_min - log_max));
        path.emplace_back(0.0, -omega);
    }
    // The detour to the right of any pole on the axis at the origin.
    if (axis_poles > 0) {
        const std::size_t arc = 64;
        for (std::size_t i = 0; i <= arc; ++i) {
            const double theta = -std::numbers::pi / 2.0 +
                                 std::numbers::pi * static_cast<double>(i) /
                                     static_cast<double>(arc);
            path.emplace_back(omega_min * std::cos(theta), omega_min * std::sin(theta));
        }
    }
    // Up the imaginary axis from +omega_min to +omega_max.
    out.omega.reserve(points);
    for (std::size_t i = 0; i < points; ++i) {
        const double frac = static_cast<double>(i) / static_cast<double>(points - 1);
        const double omega = std::pow(10.0, log_min + frac * (log_max - log_min));
        path.emplace_back(0.0, omega);
        out.omega.push_back(omega);
    }
    // The closing arc at infinity; for a strictly proper loop it maps to 0.
    {
        const std::size_t arc = 64;
        for (std::size_t i = 0; i <= arc; ++i) {
            const double theta = std::numbers::pi / 2.0 -
                                 std::numbers::pi * static_cast<double>(i) /
                                     static_cast<double>(arc);
            path.emplace_back(omega_max * std::cos(theta), omega_max * std::sin(theta));
        }
    }

    out.contour.reserve(path.size());
    double total = 0.0;
    double previous = 0.0;
    for (std::size_t i = 0; i < path.size(); ++i) {
        const Complex value = loop.evaluate(path[i]);
        out.contour.push_back(value);
        const Complex shifted = value + Complex(1.0, 0.0);
        const double angle = std::arg(shifted);
        if (i > 0) {
            double delta = angle - previous;
            while (delta > std::numbers::pi) {
                delta -= 2.0 * std::numbers::pi;
            }
            while (delta < -std::numbers::pi) {
                delta += 2.0 * std::numbers::pi;
            }
            total += delta;
        }
        previous = angle;
    }

    // The contour is traversed clockwise around the right half plane, so the
    // counterclockwise winding of 1 + L measured with omega increasing is -N,
    // and the argument principle gives Z = P - W.
    const double winding = total / (2.0 * std::numbers::pi);
    const int z = static_cast<int>(std::lround(static_cast<double>(out.open_loop_rhp_poles) - winding));
    out.closed_loop_rhp_poles = std::max(0, z);
    out.encirclements = z - out.open_loop_rhp_poles;
    out.stable = (z == 0);
    out.note = out.stable
                   ? "N + P = " + std::to_string(out.encirclements) + " + " +
                         std::to_string(out.open_loop_rhp_poles) +
                         " = 0 closed-loop poles in the right half plane: stable"
                   : "N + P = " + std::to_string(out.encirclements) + " + " +
                         std::to_string(out.open_loop_rhp_poles) + " = " + std::to_string(z) +
                         " closed-loop pole(s) in the right half plane: unstable";
    return out;
}

// ---------------------------------------------------------------------------
// The GP8 joint plant
// ---------------------------------------------------------------------------

double joint_side_inertia(std::size_t joint, const Eigen::Matrix<double, 6, 1>& q,
                          double payload_kg) {
    if (joint >= GP8_DOF) {
        throw StudyError("joint index out of range");
    }
    const std::array<Eigen::Isometry3d, GP8_DOF> frames = link_frames(q);

    Eigen::Vector3d axis = Eigen::Vector3d::UnitZ();
    Eigen::Vector3d point = Eigen::Vector3d::Zero();
    if (joint > 0) {
        axis = frames[joint - 1].linear().col(2);
        point = frames[joint - 1].translation();
    }

    double total = 0.0;
    for (std::size_t k = joint; k < GP8_DOF; ++k) {
        const Eigen::Matrix3d R = frames[k].linear();
        const Eigen::Vector3d com = frames[k] * link_com(k);
        const Eigen::Matrix3d inertia_base = R * link_inertia(k) * R.transpose();
        const Eigen::Vector3d r = com - point;
        const Eigen::Vector3d perpendicular = r - axis * r.dot(axis);
        total += axis.dot(inertia_base * axis) + GP8_LINKS[k].mass * perpendicular.squaredNorm();
    }
    if (payload_kg > 0.0) {
        const Eigen::Vector3d flange = (frames[GP8_DOF - 1] * flange_correction()).translation();
        const Eigen::Vector3d r = flange - point;
        const Eigen::Vector3d perpendicular = r - axis * r.dot(axis);
        total += payload_kg * perpendicular.squaredNorm();
    }
    return total;
}

JointPlant build_joint_plant(std::size_t joint, const Eigen::Matrix<double, 6, 1>& q,
                             double payload_kg, double velocity_fraction, double torque_fraction) {
    if (joint >= GP8_DOF) {
        throw StudyError("parameter 'joint' must select one of the six GP8 axes");
    }
    if (!(velocity_fraction > 0.0)) {
        throw StudyError("the Coulomb linearisation velocity must be positive");
    }
    JointPlant plant;
    plant.joint = joint;
    plant.link_inertia = joint_side_inertia(joint, q, 0.0);
    plant.payload_inertia = joint_side_inertia(joint, q, payload_kg) - plant.link_inertia;
    plant.reflected_inertia = reflected_rotor_inertia(joint);
    plant.total_inertia = plant.link_inertia + plant.payload_inertia + plant.reflected_inertia;
    plant.gear_ratio = GP8_LINKS[joint].gear_ratio;
    plant.viscous = GP8_LINKS[joint].viscous_friction;
    plant.coulomb = GP8_LINKS[joint].coulomb_friction;
    plant.omega_reference = velocity_fraction * joint_max_velocity(joint);
    plant.coulomb_equivalent = plant.coulomb / plant.omega_reference;
    plant.damping = plant.viscous + plant.coulomb_equivalent;
    plant.drive_torque = torque_fraction * GP8_LINKS[joint].max_torque;
    plant.dc_gain = plant.drive_torque / plant.damping;
    plant.time_constant = plant.total_inertia / plant.damping;
    plant.corner_frequency = 1.0 / plant.time_constant;
    plant.tf = TransferFunction({plant.drive_torque}, {plant.total_inertia, plant.damping, 0.0});
    return plant;
}

TransferFunction pid_controller(double kp, double ki, double kd, double derivative_tau) {
    if (std::abs(kd) <= 0.0) {
        if (std::abs(ki) <= 0.0) {
            return TransferFunction::gain(kp);
        }
        return TransferFunction({kp, ki}, {1.0, 0.0});
    }
    const double tau = (derivative_tau > 0.0) ? derivative_tau : 1.0e-3;
    // Kp + Ki/s + Kd s/(tau s + 1), over the common denominator s (tau s + 1).
    return TransferFunction({kp * tau + kd, kp + ki * tau, ki}, {tau, 1.0, 0.0});
}

}  // namespace control
namespace {

constexpr double kPi = std::numbers::pi;

using control::Complex;
using control::Poly;
using control::TransferFunction;

[[nodiscard]] Eigen::Matrix<double, 6, 1> zero6() noexcept {
    return Eigen::Matrix<double, 6, 1>::Zero();
}

// ---------------------------------------------------------------------------
// Shared parameter block: which joint, in which pose, carrying what, with how
// much loop latency. Every op that touches the real plant takes these, so a
// student changing the payload sees every panel move together.
// ---------------------------------------------------------------------------

void add_plant_params(std::vector<ParamSpec>& into, bool with_delay) {
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
    if (with_delay) {
        into.push_back(ParamSpec::scalar("delay_ms", "Measurement and computation latency", "ms",
                                         0.0, 100.0, 4.0));
    }
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

[[nodiscard]] double read_delay_seconds(const json::Value& args, double fallback_ms = 4.0) {
    return 1.0e-3 * optional_scalar(args, "delay_ms", fallback_ms, 0.0, 100.0);
}

// The plant as the loop actually sees it: the joint plus the latency of the
// encoder read and the controller cycle, as a first-order Pade stand-in.
[[nodiscard]] TransferFunction with_delay(const TransferFunction& tf, double delay_seconds) {
    if (delay_seconds <= 0.0) {
        return tf;
    }
    return tf.series(TransferFunction::pade_delay(delay_seconds));
}

// ---------------------------------------------------------------------------
// Emitters
// ---------------------------------------------------------------------------

[[nodiscard]] json::Value poly_table(const Poly& p) {
    std::vector<std::vector<json::Value>> rows;
    rows.reserve(p.size());
    const std::size_t degree = p.size() - 1;
    for (std::size_t i = 0; i < p.size(); ++i) {
        rows.push_back({json::Value(static_cast<int>(degree - i)), json::Value(p[i])});
    }
    return json::from_table({"power", "coefficient"}, rows);
}

[[nodiscard]] json::Value poly_default_table(const Poly& p) {
    std::vector<std::vector<double>> rows;
    rows.push_back(p);
    return json::from_table({"coefficients_descending"}, rows);
}

void emit_tf(json::Value& out, const TransferFunction& tf, const char* variable = "s",
             const char* prefix = "") {
    const std::string base = prefix;
    out.set(base + "numerator", poly_table(tf.numerator()));
    out.set(base + "denominator", poly_table(tf.denominator()));
    out.set(base + "transfer_function", json::Value(tf.to_string(variable)));
}

[[nodiscard]] json::Value series_of(const std::string& label, const control::Response& r) {
    return json::from_series(label, r.t, r.y);
}

[[nodiscard]] json::Value spec_table(const control::TransientSpec& spec) {
    std::vector<std::vector<json::Value>> rows;
    rows.push_back({json::Value("rise time (10-90 %)"),
                    json::Value(spec.rise_measured ? spec.rise_time : 0.0), json::Value("s")});
    rows.push_back({json::Value("peak time"), json::Value(spec.peak_time), json::Value("s")});
    rows.push_back({json::Value("peak value"), json::Value(spec.peak_value), json::Value("rad")});
    rows.push_back({json::Value("overshoot"), json::Value(spec.overshoot_percent),
                    json::Value("%")});
    rows.push_back({json::Value("settling time"), json::Value(spec.settling_time),
                    json::Value("s")});
    rows.push_back({json::Value("settling band"), json::Value(100.0 * spec.settling_band),
                    json::Value("% of final")});
    rows.push_back({json::Value("steady-state value"), json::Value(spec.steady_state_value),
                    json::Value("rad")});
    rows.push_back({json::Value("steady-state error"), json::Value(spec.steady_state_error),
                    json::Value("rad")});
    return json::from_table({"indicator", "value", "unit"}, rows);
}

void emit_spec(json::Value& out, const control::TransientSpec& spec) {
    out.set("rise_time", json::Value(spec.rise_time));
    out.set("peak_time", json::Value(spec.peak_time));
    out.set("overshoot_percent", json::Value(spec.overshoot_percent));
    out.set("settling_time", json::Value(spec.settling_time));
    out.set("steady_state_value", json::Value(spec.steady_state_value));
    out.set("steady_state_error", json::Value(spec.steady_state_error));
    out.set("settled", json::Value(spec.settled));
    out.set("spec", spec_table(spec));
}

void emit_margins(json::Value& out, const control::Margins& m) {
    out.set("gain_margin_db", json::Value(m.has_gain_margin ? m.gain_margin_db : 0.0));
    out.set("gain_margin_linear", json::Value(m.has_gain_margin ? m.gain_margin_linear : 0.0));
    out.set("phase_crossover_omega", json::Value(m.phase_crossover_omega));
    out.set("phase_margin_deg", json::Value(m.has_phase_margin ? m.phase_margin_deg : 0.0));
    out.set("gain_crossover_omega", json::Value(m.gain_crossover_omega));
    out.set("has_gain_margin", json::Value(m.has_gain_margin));
    out.set("has_phase_margin", json::Value(m.has_phase_margin));
}

[[nodiscard]] json::Value finite_or_text(double value) {
    if (!std::isfinite(value)) {
        return json::Value(value > 0.0 ? "infinite" : "-infinite");
    }
    return json::Value(value);
}

// Accepts a bare array of coefficients, a one-row table whose row *is* the
// coefficient list, or a column table with one coefficient per row.
[[nodiscard]] Poly read_poly(const json::Value& args, std::string_view key, const Poly& fallback) {
    if (is_absent(args, key)) {
        return fallback;
    }
    const json::Value& value = args[key];
    const json::Value* rows = nullptr;
    if (value.is_array()) {
        bool all_numbers = true;
        for (std::size_t i = 0; i < value.size(); ++i) {
            if (!value[i].is_number()) {
                all_numbers = false;
                break;
            }
        }
        if (all_numbers) {
            Poly out;
            out.reserve(value.size());
            for (std::size_t i = 0; i < value.size(); ++i) {
                const double c = value[i].as_double();
                if (!std::isfinite(c)) {
                    throw StudyError("parameter '" + std::string(key) + "' element " +
                                     std::to_string(i) + " must be a finite number");
                }
                out.push_back(c);
            }
            if (out.empty()) {
                throw StudyError("parameter '" + std::string(key) + "' needs at least one "
                                 "coefficient");
            }
            return out;
        }
        rows = &value;
    } else if (value.is_object() && value["rows"].is_array()) {
        rows = &value["rows"];
    } else {
        throw StudyError("parameter '" + std::string(key) +
                         "' must be an array of coefficients in descending powers, or a table "
                         "holding them");
    }
    if (rows->size() == 0) {
        throw StudyError("parameter '" + std::string(key) + "' needs at least one coefficient");
    }
    if (rows->size() == 1 && (*rows)[0].is_array()) {
        const json::Value& row = (*rows)[0];
        Poly out;
        out.reserve(row.size());
        for (std::size_t i = 0; i < row.size(); ++i) {
            if (!row[i].is_number() || !std::isfinite(row[i].as_double())) {
                throw StudyError("parameter '" + std::string(key) + "' element " +
                                 std::to_string(i) + " must be a finite number");
            }
            out.push_back(row[i].as_double());
        }
        return out;
    }
    Poly out;
    out.reserve(rows->size());
    for (std::size_t i = 0; i < rows->size(); ++i) {
        const json::Value& row = (*rows)[i];
        const json::Value& cell = row.is_array() ? row[row.size() - 1] : row;
        if (!cell.is_number() || !std::isfinite(cell.as_double())) {
            throw StudyError("parameter '" + std::string(key) + "' row " + std::to_string(i) +
                             " must end in a finite coefficient");
        }
        out.push_back(cell.as_double());
    }
    return out;
}

// ---------------------------------------------------------------------------
// The gain at which a loop first loses stability, found from the actual roots
// of the characteristic polynomial rather than from a rule of thumb.
// ---------------------------------------------------------------------------

struct GainLimit {
    bool found = false;
    double gain = 0.0;
    double omega = 0.0;
};

[[nodiscard]] bool loop_stable_at(const TransferFunction& base, double k) {
    const Poly characteristic =
        control::poly_add(control::poly_scale(base.numerator(), k), base.denominator());
    for (const auto& root : control::poly_roots(characteristic)) {
        if (root.real() >= -1e-12) {
            return false;
        }
    }
    return true;
}

[[nodiscard]] double crossing_frequency_at(const TransferFunction& base, double k) {
    const Poly characteristic =
        control::poly_add(control::poly_scale(base.numerator(), k), base.denominator());
    double best = 0.0;
    double closest = std::numeric_limits<double>::max();
    for (const auto& root : control::poly_roots(characteristic)) {
        if (std::abs(root.real()) < closest && std::abs(root.imag()) > 1e-9) {
            closest = std::abs(root.real());
            best = std::abs(root.imag());
        }
    }
    return best;
}

[[nodiscard]] GainLimit critical_gain(const TransferFunction& base, double k_min, double k_max) {
    GainLimit out;
    if (!loop_stable_at(base, k_min)) {
        return out;
    }
    const std::size_t scan = 240;
    double low = k_min;
    double high = 0.0;
    bool bracketed = false;
    for (std::size_t i = 1; i <= scan; ++i) {
        const double frac = static_cast<double>(i) / static_cast<double>(scan);
        const double k = std::pow(10.0, std::log10(k_min) + frac * (std::log10(k_max) -
                                                                    std::log10(k_min)));
        if (!loop_stable_at(base, k)) {
            high = k;
            bracketed = true;
            break;
        }
        low = k;
    }
    if (!bracketed) {
        return out;
    }
    for (int i = 0; i < 80; ++i) {
        const double mid = 0.5 * (low + high);
        if (loop_stable_at(base, mid)) {
            low = mid;
        } else {
            high = mid;
        }
    }
    out.found = true;
    out.gain = high;
    out.omega = crossing_frequency_at(base, high);
    return out;
}

// The nominal joint-2 plant, used as the default block in `block_diagram` so
// the panel opens on the real robot rather than on a made-up example.
[[nodiscard]] const control::JointPlant& nominal_plant() {
    static const control::JointPlant instance =
        control::build_joint_plant(1, Eigen::Matrix<double, 6, 1>::Zero(), 0.0, 0.25, 1.0);
    return instance;
}

}  // namespace

// ---------------------------------------------------------------------------
// Self-description
// ---------------------------------------------------------------------------

ModuleDescription ControlSystemModule::describe() const {
    ModuleDescription d;
    d.name = "control_system";
    d.title = "Robot Control and Feedback Systems";
    d.course = CourseRef{3884, "M-408-01", "Robot Control and Feedback Systems"};
    d.topics = {"Sessions 1-24 · Joint plant, block algebra, stability, quality, accuracy, PID"};
    d.source = "cpp_solver/include/study/control_system.hpp";
    d.summary =
        "Derives the open-loop transfer function of one GP8 joint from its own inertia, gearbox "
        "and friction, then analyses and controls that plant: block algebra, standard links, "
        "time and frequency response, Routh-Hurwitz, Nyquist, the root locus, steady-state "
        "accuracy, PID and its tuning rules, cascade and feedforward structures, and the "
        "robustness of all of it across the 0-8 kg payload range.";

    {
        OpSpec op;
        op.name = "joint_plant";
        op.title = "Open-loop transfer function of one GP8 joint";
        op.formula =
            "J_{tot}\\ddot{q} + b_{eq}\\dot{q} = \\tau_{drive} u \\;\\xrightarrow{\\mathcal{L}}\\; "
            "W(s) = \\frac{\\Theta(s)}{U(s)} = \\frac{\\tau_{drive}}{s\\,(J_{tot}s + b_{eq})} = "
            "\\frac{K}{s\\,(Ts+1)}, \\quad K = \\frac{\\tau_{drive}}{b_{eq}}, \\; T = "
            "\\frac{J_{tot}}{b_{eq}}";
        op.explain =
            "Start from Newton's second law for the axis: the drive torque fights the joint-side "
            "inertia and the friction, so J q'' + b q' = tau. Laplace-transform it with zero "
            "initial conditions - every derivative becomes a multiplication by s - and you get "
            "(J s^2 + b s) Theta(s) = Tau(s), which rearranges into the transfer function "
            "K/(s(Ts+1)). The free s in the denominator is the integration from velocity to "
            "angle, which is why a robot joint is a type-1 system before any controller is added. "
            "Every number here comes from this machine: the inertia is the composite-rigid-body "
            "term of the links outboard of the axis at the configuration you set, the reflected "
            "rotor inertia is n^2 J_rotor of that gearbox, and the Coulomb torque is linearised "
            "into an equivalent viscous coefficient at a stated velocity because a transfer "
            "function cannot represent a discontinuity.";
        add_plant_params(op.params, false);
        op.outputs = {
            OutputSpec::make("transfer_function", "text", "W(s) as written algebra"),
            OutputSpec::make("numerator", "table", "Numerator coefficients, descending powers"),
            OutputSpec::make("denominator", "table", "Denominator coefficients, descending powers"),
            OutputSpec::make("poles", "complex_set", "Open-loop poles"),
            OutputSpec::make("zeros", "complex_set", "Open-loop zeros"),
            OutputSpec::make("gain_K", "scalar", "K, the velocity DC gain", "rad/s per command"),
            OutputSpec::make("time_constant", "scalar", "T = J/b", "s"),
            OutputSpec::make("time_constants", "table", "Every time scale of the plant"),
            OutputSpec::make("derivation", "table", "Symbol, value, unit, source"),
            OutputSpec::make("total_inertia", "scalar", "Joint-side inertia", "kg m^2"),
            OutputSpec::make("damping", "scalar", "Equivalent viscous coefficient",
                             "N m s / rad"),
            OutputSpec::make("note", "text", "Why the position DC gain is infinite"),
        };
        d.ops.push_back(std::move(op));
    }
    {
        OpSpec op;
        op.name = "block_diagram";
        op.title = "Block-diagram algebra";
        op.formula =
            "\\text{series: } G_1G_2, \\quad \\text{parallel: } G_1+G_2, \\quad "
            "\\text{feedback: } \\frac{G}{1 \\pm GH}";
        op.explain =
            "Three rules collapse any single-loop diagram. Blocks in series multiply, blocks in "
            "parallel add over a common denominator, and a feedback path around G closes to "
            "G/(1 +/- GH) - plus in the denominator for negative feedback, minus for positive. "
            "The reduction steps are printed as polynomials so the result can be checked against "
            "the same algebra done by hand, coefficient by coefficient. The default G is the "
            "joint-2 plant, so the panel opens on the real loop.";
        op.params = {
            ParamSpec::enumeration("connection", "Connection",
                                   {"series", "parallel", "negative_feedback", "positive_feedback"},
                                   "negative_feedback"),
            ParamSpec::structured("g_numerator", "G numerator (descending powers)", "table", "",
                                  poly_default_table(nominal_plant().tf.numerator())),
            ParamSpec::structured("g_denominator", "G denominator (descending powers)", "table", "",
                                  poly_default_table(nominal_plant().tf.denominator())),
            ParamSpec::structured("h_numerator", "H numerator (descending powers)", "table", "",
                                  poly_default_table(Poly{1.0})),
            ParamSpec::structured("h_denominator", "H denominator (descending powers)", "table", "",
                                  poly_default_table(Poly{1.0})),
        };
        op.outputs = {
            OutputSpec::make("transfer_function", "text", "The reduced result"),
            OutputSpec::make("numerator", "table", "Result numerator"),
            OutputSpec::make("denominator", "table", "Result denominator"),
            OutputSpec::make("steps", "table", "Reduction, one row per rule applied"),
            OutputSpec::make("poles", "complex_set", "Poles of the result"),
            OutputSpec::make("zeros", "complex_set", "Zeros of the result"),
            OutputSpec::make("dc_gain", "scalar", "Result at s = 0"),
            OutputSpec::make("stable", "bool", "Every pole of the result in the left half plane"),
        };
        d.ops.push_back(std::move(op));
    }
    {
        OpSpec op;
        op.name = "standard_links";
        op.title = "The library of standard dynamic elements";
        op.formula =
            "K, \\quad \\frac{K}{s}, \\quad Ks, \\quad \\frac{K}{Ts+1}, \\quad "
            "\\frac{K\\omega_n^2}{s^2+2\\zeta\\omega_n s+\\omega_n^2}, \\quad e^{-\\tau s}, "
            "\\quad K\\frac{T_1s+1}{T_2s+1}";
        op.explain =
            "Every plant in the course is a product of these seven elements, so learning their "
            "step and Bode shapes once is worth more than analysing a hundred systems. Watch the "
            "slopes: the integrator falls at -20 dB/decade with a flat -90 degrees, the "
            "first-order lag breaks at 1/T, the second-order element peaks when zeta is small, "
            "and the dead time costs phase without costing magnitude - which is exactly why it "
            "kills loops. The ideal differentiator and the pure dead time have no proper rational "
            "form, so they appear here filtered and as a first-order Pade approximation, "
            "and both substitutions are named in the summary table rather than hidden.";
        op.params = {
            ParamSpec::scalar("gain", "K", "", 0.01, 100.0, 1.0),
            ParamSpec::scalar("time_constant", "T of the first-order links", "s", 0.001, 10.0, 0.2),
            ParamSpec::scalar("damping", "zeta of the second-order link", "", 0.0, 3.0, 0.3),
            ParamSpec::scalar("natural_frequency", "omega_n of the second-order link", "rad/s",
                              0.1, 500.0, 10.0),
            ParamSpec::scalar("dead_time", "tau of the dead-time link", "s", 0.0, 2.0, 0.05),
            ParamSpec::scalar("lead_time_constant", "T_1 of the lead-lag link", "s", 0.0, 10.0,
                              0.5),
            ParamSpec::scalar("duration", "Step-response window", "s", 0.01, 60.0, 2.0),
            ParamSpec::integer("samples", "Samples in the step response", "", 32, 4000, 401),
            ParamSpec::scalar("omega_min", "Lowest frequency", "rad/s", 1e-3, 100.0, 0.1),
            ParamSpec::scalar("omega_max", "Highest frequency", "rad/s", 1.0, 1e6, 1000.0),
        };
        op.outputs = {
            OutputSpec::make("step_responses", "series_set", "One step response per link"),
            OutputSpec::make("magnitude_db", "series_set", "Bode magnitude per link", "dB"),
            OutputSpec::make("phase_deg", "series_set", "Bode phase per link", "deg"),
            OutputSpec::make("summary", "table", "Link, transfer function, slope, note"),
        };
        d.ops.push_back(std::move(op));
    }
    {
        OpSpec op;
        op.name = "time_response";
        op.title = "Step and impulse response with the transient specification";
        op.formula =
            "\\sigma = \\frac{y_{max}-y_\\infty}{y_\\infty}\\cdot 100\\%, \\quad "
            "t_s = \\min\\{t : |y(\\theta)-y_\\infty| \\le \\Delta y_\\infty \\;\\forall\\, "
            "\\theta > t\\}, \\quad e_\\infty = r - y_\\infty";
        op.explain =
            "This is the half of the course that is graded in seconds and percent rather than in "
            "poles. Rise time is measured 10 % to 90 % of the final value, peak time is when the "
            "response tops out, overshoot is how far past the target it went, and settling time "
            "is the last moment it leaves the band you set - note that it is the LAST exit, not "
            "the first entry, because a slowly ringing loop re-enters and leaves the band many "
            "times. Raise the loop gain and watch overshoot and settling time move in opposite "
            "directions: that trade is the whole point of session 5.";
        add_plant_params(op.params, true);
        op.params.push_back(ParamSpec::scalar("loop_gain", "Proportional loop gain", "", 0.0, 500.0,
                                              1.0));
        op.params.push_back(ParamSpec::boolean("closed_loop", "Close the feedback loop", true));
        op.params.push_back(
            ParamSpec::enumeration("input", "Input signal", {"step", "impulse"}, "step"));
        op.params.push_back(ParamSpec::scalar("setpoint", "Step size", "rad", -kPi, kPi, 1.0));
        op.params.push_back(ParamSpec::scalar("settling_band", "Settling band", "% of final", 0.1,
                                              20.0, 2.0));
        op.params.push_back(ParamSpec::scalar("duration", "Window", "s", 0.01, 120.0, 10.0));
        op.params.push_back(ParamSpec::integer("samples", "Samples", "", 32, 8000, 801));
        op.outputs = {
            OutputSpec::make("responses", "series_set", "Response and the commanded input"),
            OutputSpec::make("transfer_function", "text", "The transfer function that was driven"),
            OutputSpec::make("numerator", "table", "Numerator of that transfer function"),
            OutputSpec::make("denominator", "table", "Denominator of that transfer function"),
            OutputSpec::make("spec", "table", "The graded transient specification"),
            OutputSpec::make("rise_time", "scalar", "10-90 % rise time", "s"),
            OutputSpec::make("peak_time", "scalar", "Time of the peak", "s"),
            OutputSpec::make("overshoot_percent", "scalar", "Overshoot", "%"),
            OutputSpec::make("settling_time", "scalar", "Settling time", "s"),
            OutputSpec::make("steady_state_value", "scalar", "Final value", "rad"),
            OutputSpec::make("steady_state_error", "scalar", "Setpoint minus final value", "rad"),
            OutputSpec::make("settled", "bool", "Did it settle inside the window"),
            OutputSpec::make("damping_ratio", "scalar", "zeta of the dominant pole pair"),
            OutputSpec::make("natural_frequency", "scalar", "omega_n of that pair", "rad/s"),
            OutputSpec::make("poles", "complex_set", "Poles of the simulated system"),
            OutputSpec::make("stable", "bool", "Is the simulated system stable"),
        };
        d.ops.push_back(std::move(op));
    }
    {
        OpSpec op;
        op.name = "frequency_response";
        op.title = "Bode, Nyquist and the stability margins";
        op.formula =
            "L(j\\omega) = |L|e^{j\\varphi}, \\quad GM = -20\\log_{10}|L(j\\omega_{-180})|, \\quad "
            "PM = 180^\\circ + \\varphi(\\omega_{0\\,dB})";
        op.explain =
            "Substituting s = j omega turns the transfer function into a complex number per "
            "frequency: its magnitude is how much the joint moves, its argument is how late it "
            "moves. The gain margin is how much more gain the loop tolerates before the phase "
            "crossover becomes a 0 dB crossover; the phase margin is how much extra lag it "
            "tolerates at the gain crossover. Add latency with delay_ms and watch the phase "
            "margin erode while the magnitude curve does not move at all - that is the lesson "
            "sessions 7 and 11 build to, and the reason sampling later costs so much.";
        add_plant_params(op.params, true);
        op.params.push_back(ParamSpec::scalar("loop_gain", "Proportional loop gain", "", 0.01,
                                              500.0, 1.0));
        op.params.push_back(ParamSpec::scalar("omega_min", "Lowest frequency", "rad/s", 1e-3, 100.0,
                                              0.01));
        op.params.push_back(ParamSpec::scalar("omega_max", "Highest frequency", "rad/s", 1.0, 1e6,
                                              1000.0));
        op.params.push_back(ParamSpec::integer("points", "Points in the sweep", "", 64, 8000, 800));
        op.outputs = {
            OutputSpec::make("bode", "series_set", "Magnitude in dB and phase in degrees"),
            OutputSpec::make("nyquist", "complex_set", "L(s) along the Nyquist contour"),
            OutputSpec::make("gain_margin_db", "scalar", "Gain margin", "dB"),
            OutputSpec::make("gain_margin_linear", "scalar", "Gain margin as a factor"),
            OutputSpec::make("phase_crossover_omega", "scalar", "Where the phase hits -180 deg",
                             "rad/s"),
            OutputSpec::make("phase_margin_deg", "scalar", "Phase margin", "deg"),
            OutputSpec::make("gain_crossover_omega", "scalar", "Where the magnitude hits 0 dB",
                             "rad/s"),
            OutputSpec::make("has_gain_margin", "bool", "Does the phase ever reach -180 deg"),
            OutputSpec::make("has_phase_margin", "bool", "Does the magnitude ever reach 0 dB"),
            OutputSpec::make("stable", "bool", "Closed-loop verdict from the pole locations"),
            OutputSpec::make("verdict", "text", "What the two margins mean for this loop"),
        };
        d.ops.push_back(std::move(op));
    }
    {
        OpSpec op;
        op.name = "stability";
        op.title = "Stability by three criteria that must agree";
        op.formula =
            "D(s) = \\sum a_i s^{n-i}: \\;\\text{Routh first column signs} \\;\\equiv\\; "
            "Z = N + P \\;\\equiv\\; \\max_i \\operatorname{Re}(s_i) < 0";
        op.explain =
            "Three routes to one verdict. The algebraic route builds the Routh array from the "
            "characteristic polynomial and counts sign changes in its first column - each sign "
            "change is one root in the right half plane, and no root-finding is needed. The "
            "Nyquist route counts clockwise encirclements of -1 by the loop locus and adds the "
            "open-loop right-half-plane poles: Z = N + P. The direct route simply locates the "
            "roots. If the three ever disagree on this plant, the arithmetic is wrong - not the "
            "theory - which is exactly the habit practical work 3 is trying to build.";
        add_plant_params(op.params, true);
        op.params.push_back(ParamSpec::enumeration("criterion", "Criterion",
                                                   {"algebraic", "nyquist", "roots"}, "algebraic"));
        op.params.push_back(ParamSpec::scalar("loop_gain", "Proportional loop gain", "", 0.01,
                                              5000.0, 1.0));
        op.outputs = {
            OutputSpec::make("stable", "bool", "The unambiguous verdict"),
            OutputSpec::make("criterion", "text", "Which criterion produced it"),
            OutputSpec::make("explanation", "text", "Why, in words"),
            OutputSpec::make("characteristic_polynomial", "text", "1 + K L(s) = 0, written out"),
            OutputSpec::make("routh", "table", "The full Routh array, first column leftmost"),
            OutputSpec::make("sign_changes", "int", "Sign changes in the first column"),
            OutputSpec::make("encirclements", "int", "Clockwise encirclements of -1 (N)"),
            OutputSpec::make("open_loop_rhp_poles", "int", "P"),
            OutputSpec::make("closed_loop_rhp_poles", "int", "Z = N + P"),
            OutputSpec::make("closed_loop_poles", "complex_set", "Roots of the characteristic "
                                                                 "polynomial"),
            OutputSpec::make("critical_gain", "scalar", "Gain at which stability is lost"),
            OutputSpec::make("critical_frequency", "scalar", "Oscillation frequency there",
                             "rad/s"),
            OutputSpec::make("agrees_with_roots", "bool", "Does the chosen criterion match the "
                                                          "root locations"),
        };
        d.ops.push_back(std::move(op));
    }
    {
        OpSpec op;
        op.name = "root_locus";
        op.title = "Root locus of the joint loop";
        op.formula =
            "1 + K L(s) = 0, \\quad \\sigma_a = \\frac{\\sum p_i - \\sum z_j}{n-m}, \\quad "
            "\\theta_k = \\frac{(2k+1)180^\\circ}{n-m}, \\quad \\frac{dK}{ds} = 0 "
            "\\text{ at a breakaway}";
        op.explain =
            "The locus is every closed-loop pole the loop can have as the gain sweeps from zero "
            "to infinity: it starts at the open-loop poles and ends at the open-loop zeros or at "
            "infinity along the asymptotes. Breakaway points are where two branches meet and turn "
            "into the complex plane, and they are the roots of N D' - N' D = 0. The gain where "
            "the locus crosses the imaginary axis is the stability limit, and it is the same "
            "number the Routh array gives - which is the cross-check to run.";
        add_plant_params(op.params, true);
        op.params.push_back(ParamSpec::scalar("gain_min", "Lowest gain", "", 1e-3, 1e4, 0.01));
        op.params.push_back(ParamSpec::scalar("gain_max", "Highest gain", "", 0.1, 1e6, 1000.0));
        op.params.push_back(ParamSpec::integer("samples", "Gains in the sweep", "", 16, 2000, 240));
        op.outputs = {
            OutputSpec::make("branches", "complex_set_set", "One trajectory per locus branch"),
            OutputSpec::make("open_loop_poles", "complex_set", "Where the branches start"),
            OutputSpec::make("open_loop_zeros", "complex_set", "Where they end"),
            OutputSpec::make("breakaway_points", "table", "Real breakaway points and their gains"),
            OutputSpec::make("asymptotes", "table", "Centroid and departure angles"),
            OutputSpec::make("asymptote_centroid", "scalar", "sigma_a"),
            OutputSpec::make("critical_gain", "scalar", "Gain at the imaginary-axis crossing"),
            OutputSpec::make("critical_frequency", "scalar", "Frequency there", "rad/s"),
            OutputSpec::make("crosses_imaginary_axis", "bool", "Does it cross inside the sweep"),
            OutputSpec::make("note", "text", "How to read the locus of this plant"),
        };
        d.ops.push_back(std::move(op));
    }
    {
        OpSpec op;
        op.name = "steady_state_accuracy";
        op.title = "System type and the error constants";
        op.formula =
            "K_p = \\lim_{s\\to 0} L(s), \\quad K_v = \\lim_{s\\to 0} sL(s), \\quad "
            "K_a = \\lim_{s\\to 0} s^2L(s), \\quad e_\\infty^{step} = \\frac{1}{1+K_p}, \\;"
            "e_\\infty^{ramp} = \\frac{1}{K_v}, \\; e_\\infty^{par} = \\frac{1}{K_a}";
        op.explain =
            "The system type is simply how many free integrators the loop has, and it decides "
            "which commands the joint can follow with no residual error at all. A bare GP8 joint "
            "is type 1: it holds a commanded angle exactly, but tracks a constant-velocity ramp "
            "with a fixed lag of 1/K_v, and loses a parabola completely. Add the integral term of "
            "a PID and the type rises to 2: the ramp error goes to zero and the parabola becomes "
            "finite. That ladder - one more integrator buys one more input class - is the whole "
            "content of session 15.";
        add_plant_params(op.params, false);
        op.params.push_back(ParamSpec::scalar("loop_gain", "Proportional loop gain", "", 0.01,
                                              500.0, 1.0));
        op.params.push_back(ParamSpec::enumeration("controller", "Controller",
                                                   {"proportional", "pi", "pid"}, "proportional"));
        op.params.push_back(ParamSpec::scalar("ki", "Integral gain, for pi and pid", "", 0.0, 500.0,
                                              1.0));
        op.params.push_back(ParamSpec::scalar("kd", "Derivative gain, for pid", "", 0.0, 100.0,
                                              0.1));
        op.outputs = {
            OutputSpec::make("system_type", "int", "Number of free integrators in the loop"),
            OutputSpec::make("kp", "scalar", "Position error constant"),
            OutputSpec::make("kv", "scalar", "Velocity error constant"),
            OutputSpec::make("ka", "scalar", "Acceleration error constant"),
            OutputSpec::make("errors", "table", "Input, error constant, steady-state error"),
            OutputSpec::make("loop_transfer_function", "text", "L(s) that was analysed"),
            OutputSpec::make("stable", "bool", "Is the loop stable (an error constant of an "
                                              "unstable loop is meaningless)"),
            OutputSpec::make("explanation", "text", "What this type can and cannot follow"),
        };
        d.ops.push_back(std::move(op));
    }
    {
        OpSpec op;
        op.name = "pid";
        op.title = "PID on the joint, with its actuator cost";
        op.formula =
            "C(s) = K_p + \\frac{K_i}{s} + \\frac{K_d s}{\\tau_d s + 1}, \\quad "
            "T(s) = \\frac{C(s)W(s)}{1 + C(s)W(s)}, \\quad "
            "U(s) = \\frac{C(s)}{1+C(s)W(s)}R(s)";
        op.explain =
            "P reacts to the error now, I to the error so far, D to where the error is going. The "
            "derivative term is filtered because an unfiltered one differentiates encoder noise "
            "into the motor; tau_d is that filter and it is a design choice, not a detail. The "
            "control-effort trace is the output that keeps the exercise honest: raise Kd and the "
            "overshoot falls, but the commanded torque spikes, and when that spike exceeds the "
            "axis rating the gains you chose exist only in simulation.";
        add_plant_params(op.params, true);
        op.params.push_back(ParamSpec::scalar("kp", "Kp", "", 0.0, 500.0, 8.0));
        op.params.push_back(ParamSpec::scalar("ki", "Ki", "", 0.0, 500.0, 4.0));
        op.params.push_back(ParamSpec::scalar("kd", "Kd", "", 0.0, 100.0, 1.0));
        op.params.push_back(ParamSpec::scalar("derivative_tau", "Derivative filter time constant",
                                              "s", 1e-4, 1.0, 0.01));
        op.params.push_back(ParamSpec::scalar("setpoint", "Step size", "rad", -kPi, kPi, 1.0));
        op.params.push_back(ParamSpec::scalar("settling_band", "Settling band", "% of final", 0.1,
                                              20.0, 2.0));
        op.params.push_back(ParamSpec::scalar("duration", "Window", "s", 0.01, 120.0, 5.0));
        op.params.push_back(ParamSpec::integer("samples", "Samples", "", 32, 8000, 801));
        op.outputs = {
            OutputSpec::make("responses", "series_set", "Setpoint, response and control effort"),
            OutputSpec::make("controller", "text", "C(s)"),
            OutputSpec::make("transfer_function", "text", "Closed-loop T(s)"),
            OutputSpec::make("numerator", "table", "Closed-loop numerator"),
            OutputSpec::make("denominator", "table", "Closed-loop denominator"),
            OutputSpec::make("poles", "complex_set", "Closed-loop poles"),
            OutputSpec::make("spec", "table", "Transient specification"),
            OutputSpec::make("rise_time", "scalar", "Rise time", "s"),
            OutputSpec::make("peak_time", "scalar", "Peak time", "s"),
            OutputSpec::make("overshoot_percent", "scalar", "Overshoot", "%"),
            OutputSpec::make("settling_time", "scalar", "Settling time", "s"),
            OutputSpec::make("steady_state_value", "scalar", "Final value", "rad"),
            OutputSpec::make("steady_state_error", "scalar", "Residual error", "rad"),
            OutputSpec::make("settled", "bool", "Settled inside the window"),
            OutputSpec::make("gain_margin_db", "scalar", "Gain margin", "dB"),
            OutputSpec::make("phase_margin_deg", "scalar", "Phase margin", "deg"),
            OutputSpec::make("peak_torque", "scalar", "Largest commanded torque", "N m"),
            OutputSpec::make("torque_rating", "scalar", "What this axis is rated for", "N m"),
            OutputSpec::make("torque_saturated", "bool", "Does the command exceed the rating"),
            OutputSpec::make("effort_note", "text", "The actuator cost of these gains"),
            OutputSpec::make("stable", "bool", "Closed-loop stability"),
        };
        d.ops.push_back(std::move(op));
    }
    {
        OpSpec op;
        op.name = "pid_tune";
        op.title = "Four tuning methods, compared rather than trusted";
        op.formula =
            "\\text{ZN step: } K_p=\\frac{1.2}{RL},\\,T_i=2L,\\,T_d=0.5L; \\quad "
            "\\text{ZN ultimate: } K_p=0.6K_u,\\,T_i=0.5P_u,\\,T_d=0.125P_u; \\quad "
            "\\text{pole placement: } J s^3 + (b+K_aK_d)s^2 + K_aK_p s + K_aK_i = "
            "J(s^2+2\\zeta\\omega_n s+\\omega_n^2)(s+\\alpha\\omega_n)";
        op.explain =
            "The two Ziegler-Nichols forms are measurements, not formulas: the step form draws "
            "the tangent at the steepest point of the open-loop step response and reads the "
            "apparent lag L and slope R off it, and the ultimate form raises the proportional "
            "gain until the loop oscillates and reads K_u and the period P_u there. Cohen-Coon is "
            "the same two numbers with the constants its authors derived for an integrating "
            "process. Pole placement is the only one that is exact: the PID gains are solved from "
            "matching the closed-loop characteristic polynomial to the one you asked for, so the "
            "measured overshoot has to come out where the placement said. All four are run on "
            "this plant and tabulated side by side, because a tuning rule is a starting point and "
            "the table is how you find out which starting point this joint prefers.";
        add_plant_params(op.params, true);
        op.params.push_back(ParamSpec::enumeration(
            "method", "Method",
            {"ziegler_nichols_step", "ziegler_nichols_ultimate", "cohen_coon", "pole_placement"},
            "ziegler_nichols_ultimate"));
        op.params.push_back(ParamSpec::scalar("target_damping", "Target zeta for pole placement",
                                              "", 0.1, 1.5, 0.707));
        op.params.push_back(ParamSpec::scalar("target_frequency",
                                              "Target omega_n for pole placement", "rad/s", 0.1,
                                              200.0, 6.0));
        op.params.push_back(ParamSpec::scalar("third_pole_ratio",
                                              "Third pole as a multiple of omega_n", "", 1.0, 20.0,
                                              4.0));
        op.params.push_back(ParamSpec::scalar("settling_band", "Settling band", "% of final", 0.1,
                                              20.0, 2.0));
        op.params.push_back(ParamSpec::scalar("duration", "Window", "s", 0.01, 120.0, 8.0));
        op.params.push_back(ParamSpec::integer("samples", "Samples", "", 32, 8000, 801));
        op.outputs = {
            OutputSpec::make("comparison", "table", "Every method's gains and resulting "
                                                    "specification"),
            OutputSpec::make("responses", "series_set", "One step response per method"),
            OutputSpec::make("selected", "text", "The method that was selected"),
            OutputSpec::make("kp", "scalar", "Kp of the selected method"),
            OutputSpec::make("ki", "scalar", "Ki of the selected method"),
            OutputSpec::make("kd", "scalar", "Kd of the selected method"),
            OutputSpec::make("controller", "text", "C(s) of the selected method"),
            OutputSpec::make("reaction_curve", "table", "L and R read off the step response"),
            OutputSpec::make("ultimate_gain", "scalar", "K_u"),
            OutputSpec::make("ultimate_period", "scalar", "P_u", "s"),
            OutputSpec::make("stable", "bool", "Is the selected method's loop stable"),
            OutputSpec::make("note", "text", "What the comparison says about this joint"),
        };
        d.ops.push_back(std::move(op));
    }
    {
        OpSpec op;
        op.name = "advanced_structures";
        op.title = "Cascade, feedforward and disturbance compensation";
        op.formula =
            "\\text{cascade: } T_v = \\frac{K_vP_v}{1+K_vP_v},\\; L = \\frac{K_{pos}T_v}{s}; "
            "\\quad \\text{feedforward: } Y = \\frac{(C+F)P}{1+CP}R; \\quad "
            "\\text{compensation: } \\frac{Y}{D} = \\frac{P(1-Q)}{1+CP}";
        op.explain =
            "Three structures, one measurement. A cascade puts a fast velocity loop inside the "
            "position loop, so the inner loop has already rejected most of the load torque before "
            "the outer loop notices. Feedforward inverts the plant model and injects the torque "
            "the trajectory is going to need, which removes the lag without touching the loop's "
            "stability margins at all. Disturbance compensation estimates the load from the "
            "measured motion and subtracts it, with a filter Q that decides how much it can "
            "cancel before it amplifies noise. Each is run against the same torque step with the "
            "structure on and off, so the improvement is a number rather than a claim.";
        add_plant_params(op.params, true);
        op.params.push_back(ParamSpec::enumeration(
            "structure", "Structure", {"cascade", "feedforward", "disturbance_compensation"},
            "cascade"));
        op.params.push_back(ParamSpec::scalar("kp", "Outer or single-loop Kp", "", 0.0, 500.0, 8.0));
        op.params.push_back(ParamSpec::scalar("ki", "Outer or single-loop Ki", "", 0.0, 500.0, 4.0));
        op.params.push_back(ParamSpec::scalar("kd", "Outer or single-loop Kd", "", 0.0, 100.0, 1.0));
        op.params.push_back(ParamSpec::scalar("derivative_tau", "Derivative filter", "s", 1e-4, 1.0,
                                              0.01));
        op.params.push_back(ParamSpec::scalar("inner_gain", "Velocity-loop gain, cascade only", "",
                                              0.1, 200.0, 10.0));
        op.params.push_back(ParamSpec::scalar("filter_tau", "Feedforward / observer filter", "s",
                                              1e-3, 1.0, 0.05));
        op.params.push_back(ParamSpec::scalar("setpoint", "Step size", "rad", -kPi, kPi, 1.0));
        op.params.push_back(ParamSpec::scalar("disturbance_torque", "Disturbance step", "N m", 0.0,
                                              100.0, 20.0));
        op.params.push_back(ParamSpec::scalar("disturbance_time", "When it is applied", "s", 0.0,
                                              60.0, 2.0));
        op.params.push_back(ParamSpec::scalar("duration", "Window", "s", 0.1, 120.0, 8.0));
        op.params.push_back(ParamSpec::integer("samples", "Samples", "", 64, 8000, 801));
        op.outputs = {
            OutputSpec::make("responses", "series_set", "Baseline and structured response under "
                                                        "the same disturbance"),
            OutputSpec::make("structure", "text", "Which structure was applied"),
            OutputSpec::make("peak_error_without", "scalar",
                             "Worst deviation without it, from the load step onwards", "rad"),
            OutputSpec::make("peak_error_with", "scalar",
                             "Worst deviation with it, from the load step onwards", "rad"),
            OutputSpec::make("improvement_factor", "scalar", "Ratio of the two"),
            OutputSpec::make("comparison", "table", "Specification with and without"),
            OutputSpec::make("loop_without", "text", "Baseline closed-loop transfer function"),
            OutputSpec::make("loop_with", "text", "Structured closed-loop transfer function"),
            OutputSpec::make("stable_without", "bool", "Baseline stability"),
            OutputSpec::make("stable_with", "bool", "Structured stability"),
            OutputSpec::make("note", "text", "What was bought and what it cost"),
        };
        d.ops.push_back(std::move(op));
    }
    {
        OpSpec op;
        op.name = "robustness";
        op.title = "Margins and transient quality across the payload range";
        op.formula =
            "J(m) = J_{link}(1+\\delta) + m r_\\perp^2 + n^2 J_{rotor}, \\quad "
            "S_{J}^{\\sigma} = \\frac{\\Delta\\sigma/\\sigma}{\\Delta J/J}";
        op.explain =
            "A GP8 tuned empty is a different machine with 8 kg in the gripper: the joint-side "
            "inertia can more than double, which moves the time constant, the margins and the "
            "overshoot together. This op sweeps the payload across the rated range and the link "
            "inertia across the tolerance you give it, and reports the worst case rather than the "
            "nominal one - because the worst case is what the robot will meet. The sensitivity "
            "figure is the relative change in overshoot per relative change in inertia, so a "
            "single number says whether the tuning is robust or merely lucky.";
        add_plant_params(op.params, true);
        op.params.push_back(ParamSpec::scalar("kp", "Kp", "", 0.0, 500.0, 8.0));
        op.params.push_back(ParamSpec::scalar("ki", "Ki", "", 0.0, 500.0, 4.0));
        op.params.push_back(ParamSpec::scalar("kd", "Kd", "", 0.0, 100.0, 1.0));
        op.params.push_back(ParamSpec::scalar("derivative_tau", "Derivative filter", "s", 1e-4, 1.0,
                                              0.01));
        op.params.push_back(ParamSpec::scalar("payload_min", "Lightest payload", "kg", 0.0,
                                              GP8_PAYLOAD_KG, 0.0));
        op.params.push_back(ParamSpec::scalar("payload_max", "Heaviest payload", "kg", 0.0,
                                              GP8_PAYLOAD_KG, GP8_PAYLOAD_KG));
        op.params.push_back(ParamSpec::scalar("inertia_tolerance", "Link-inertia tolerance", "%",
                                              0.0, 100.0, 20.0));
        op.params.push_back(ParamSpec::integer("samples", "Payloads in the sweep", "", 3, 200, 17));
        op.params.push_back(ParamSpec::scalar("required_gain_margin", "Required gain margin", "dB",
                                              0.0, 40.0, 6.0));
        op.params.push_back(ParamSpec::scalar("required_phase_margin", "Required phase margin",
                                              "deg", 0.0, 90.0, 45.0));
        op.params.push_back(ParamSpec::scalar("duration", "Window per point", "s", 0.1, 60.0, 8.0));
        op.params.push_back(ParamSpec::integer("response_samples", "Samples per point", "", 64,
                                               4000, 401));
        op.outputs = {
            OutputSpec::make("margins", "series_set", "Gain and phase margin against payload"),
            OutputSpec::make("quality", "series_set", "Overshoot and settling time against "
                                                      "payload"),
            OutputSpec::make("sweep", "table", "Every sweep point"),
            OutputSpec::make("worst_case", "table", "The worst point of each indicator"),
            OutputSpec::make("inertia_sensitivity", "scalar",
                             "Relative overshoot change per relative inertia change"),
            OutputSpec::make("margin_sensitivity", "scalar", "Phase-margin loss per kg",
                             "deg/kg"),
            OutputSpec::make("specification_met", "bool",
                             "Do both required margins hold across the whole sweep"),
            OutputSpec::make("worst_payload", "scalar", "Payload of the worst case", "kg"),
            OutputSpec::make("note", "text", "Verdict on this tuning across the range"),
        };
        d.ops.push_back(std::move(op));
    }
    return d;
}

namespace {

// a / (1 + L), the shape every sensitivity function in this module takes.
[[nodiscard]] TransferFunction over_one_plus(const TransferFunction& a,
                                             const TransferFunction& loop) {
    return TransferFunction(
        control::poly_multiply(a.numerator(), loop.denominator()),
        control::poly_multiply(a.denominator(),
                               control::poly_add(loop.denominator(), loop.numerator())));
}

[[nodiscard]] Poly poly_derivative(const Poly& p) {
    if (p.size() <= 1) {
        return Poly{0.0};
    }
    const std::size_t degree = p.size() - 1;
    Poly out(degree, 0.0);
    for (std::size_t i = 0; i < degree; ++i) {
        out[i] = p[i] * static_cast<double>(degree - i);
    }
    return out;
}

struct DominantPair {
    double zeta = 1.0;
    double omega_n = 0.0;
    bool oscillatory = false;
};

[[nodiscard]] DominantPair dominant_pair(const std::vector<Complex>& poles) {
    DominantPair out;
    double best = -std::numeric_limits<double>::max();
    for (const auto& p : poles) {
        if (p.real() > best) {
            best = p.real();
            out.omega_n = std::abs(p);
            out.oscillatory = std::abs(p.imag()) > 1e-9;
            out.zeta = (out.omega_n > 0.0) ? std::clamp(-p.real() / out.omega_n, -1.0, 1.0) : 1.0;
        }
    }
    return out;
}

// Superposition: the setpoint response plus the disturbance response shifted to
// the moment the load is applied. Both come from the same linear model, so this
// is exact rather than an approximation.
[[nodiscard]] control::Response add_shifted(const control::Response& base,
                                            const control::Response& extra, double amplitude,
                                            double shift_seconds) {
    control::Response out = base;
    if (base.t.size() < 2 || extra.y.empty() || amplitude == 0.0) {
        return out;
    }
    const double dt = base.t[1] - base.t[0];
    const std::size_t offset =
        static_cast<std::size_t>(std::max(0.0, std::round(shift_seconds / dt)));
    for (std::size_t i = offset; i < out.y.size(); ++i) {
        const std::size_t k = i - offset;
        if (k >= extra.y.size()) {
            break;
        }
        out.y[i] += amplitude * extra.y[k];
    }
    return out;
}

// The worst deviation from the setpoint from `after` onwards. The window
// matters: measured from t = 0 the answer is always the step itself, which says
// nothing about how well the load was rejected.
[[nodiscard]] double peak_deviation(const control::Response& r, double reference, double after) {
    double worst = 0.0;
    for (std::size_t i = 0; i < r.y.size(); ++i) {
        if (r.t[i] < after) {
            continue;
        }
        worst = std::max(worst, std::abs(reference - r.y[i]));
    }
    return worst;
}

// ---------------------------------------------------------------------------
// joint_plant (session 2)
// ---------------------------------------------------------------------------

[[nodiscard]] json::Value op_joint_plant(const json::Value& args) {
    const control::JointPlant p = read_plant(args);
    const std::size_t j = p.joint;
    const std::string axis = GP8_AXIS_NAMES[j];
    const std::string index = std::to_string(j);

    std::vector<std::vector<json::Value>> derivation;
    derivation.push_back({json::Value("J_link"), json::Value(p.link_inertia),
                          json::Value("kg m^2"),
                          json::Value("gp8_model.hpp link_inertia / GP8_LINKS[" + index +
                                      "..5], composite rigid body about this axis at q")});
    derivation.push_back({json::Value("J_payload"), json::Value(p.payload_inertia),
                          json::Value("kg m^2"),
                          json::Value("payload_kg as a point mass at the DH flange origin")});
    derivation.push_back({json::Value("n"), json::Value(p.gear_ratio), json::Value("motor/joint"),
                          json::Value("gp8_model.hpp GP8_LINKS[" + index + "].gear_ratio")});
    derivation.push_back({json::Value("J_rotor"), json::Value(GP8_LINKS[j].rotor_inertia),
                          json::Value("kg m^2"),
                          json::Value("gp8_model.hpp GP8_LINKS[" + index + "].rotor_inertia")});
    derivation.push_back({json::Value("n^2 J_rotor"), json::Value(p.reflected_inertia),
                          json::Value("kg m^2"),
                          json::Value("gp8_model.hpp reflected_rotor_inertia(" + index + ")")});
    derivation.push_back({json::Value("J_tot"), json::Value(p.total_inertia),
                          json::Value("kg m^2"),
                          json::Value("J_link + J_payload + n^2 J_rotor")});
    derivation.push_back({json::Value("b_v"), json::Value(p.viscous),
                          json::Value("N m s / rad"),
                          json::Value("gp8_model.hpp GP8_LINKS[" + index +
                                      "].viscous_friction")});
    derivation.push_back({json::Value("tau_c"), json::Value(p.coulomb), json::Value("N m"),
                          json::Value("gp8_model.hpp GP8_LINKS[" + index +
                                      "].coulomb_friction")});
    derivation.push_back({json::Value("omega_ref"), json::Value(p.omega_reference),
                          json::Value("rad/s"),
                          json::Value("velocity_fraction x gp8_model.hpp joint_max_velocity(" +
                                      index + ")")});
    derivation.push_back({json::Value("b_c = tau_c / omega_ref"),
                          json::Value(p.coulomb_equivalent), json::Value("N m s / rad"),
                          json::Value("equivalent-viscous linearisation of Coulomb friction")});
    derivation.push_back({json::Value("b_eq = b_v + b_c"), json::Value(p.damping),
                          json::Value("N m s / rad"), json::Value("sum of the two rows above")});
    derivation.push_back({json::Value("tau_drive"), json::Value(p.drive_torque),
                          json::Value("N m"),
                          json::Value("torque_fraction x gp8_model.hpp GP8_LINKS[" + index +
                                      "].max_torque (URDF effort limit), so u = 1 is full "
                                      "torque")});
    derivation.push_back({json::Value("K = tau_drive / b_eq"), json::Value(p.dc_gain),
                          json::Value("rad/s per command"),
                          json::Value("Laplace form K/(s(Ts+1))")});
    derivation.push_back({json::Value("T = J_tot / b_eq"), json::Value(p.time_constant),
                          json::Value("s"), json::Value("Laplace form K/(s(Ts+1))")});

    std::vector<std::vector<json::Value>> times;
    times.push_back({json::Value("mechanical time constant T"), json::Value(p.time_constant),
                     json::Value("s")});
    times.push_back({json::Value("corner frequency 1/T"), json::Value(p.corner_frequency),
                     json::Value("rad/s")});
    times.push_back({json::Value("velocity reaches 63 % of K at"), json::Value(p.time_constant),
                     json::Value("s")});
    times.push_back({json::Value("velocity reaches 98 % of K at"),
                     json::Value(4.0 * p.time_constant), json::Value("s")});

    json::Value out = json::Value::object();
    emit_tf(out, p.tf);
    out.set("poles", json::from_complex_set(p.tf.poles()));
    out.set("zeros", json::from_complex_set(p.tf.zeros()));
    out.set("gain_K", json::Value(p.dc_gain));
    out.set("time_constant", json::Value(p.time_constant));
    out.set("time_constants", json::from_table({"time scale", "value", "unit"}, times));
    out.set("derivation", json::from_table({"symbol", "value", "unit", "source"}, derivation));
    out.set("total_inertia", json::Value(p.total_inertia));
    out.set("damping", json::Value(p.damping));
    out.set("note",
            json::Value("Axis " + axis + ". The pole at the origin is the integration from "
                        "velocity to angle, so the POSITION dc gain is infinite and the number "
                        "reported as K is the velocity gain in rad/s per unit command. The "
                        "second pole sits at -1/T = " +
                        json::number_to_string(-p.corner_frequency) +
                        " rad/s. Coulomb friction is a discontinuity and cannot appear in a "
                        "transfer function: it is folded into b_eq at omega_ref, so a slower "
                        "move looks more damped than a faster one - which is exactly what the "
                        "real axis does."));
    return out;
}

// ---------------------------------------------------------------------------
// block_diagram (session 3)
// ---------------------------------------------------------------------------

[[nodiscard]] json::Value op_block_diagram(const json::Value& args) {
    const std::vector<std::string> options = {"series", "parallel", "negative_feedback",
                                              "positive_feedback"};
    const std::string connection = optional_enum(args, "connection", "negative_feedback", options);
    const TransferFunction g(read_poly(args, "g_numerator", nominal_plant().tf.numerator()),
                             read_poly(args, "g_denominator", nominal_plant().tf.denominator()));
    const TransferFunction h(read_poly(args, "h_numerator", Poly{1.0}),
                             read_poly(args, "h_denominator", Poly{1.0}));

    std::vector<std::vector<json::Value>> steps;
    steps.push_back({json::Value("1"), json::Value("G as given"),
                     json::Value(control::poly_to_string(g.numerator())),
                     json::Value(control::poly_to_string(g.denominator()))});
    steps.push_back({json::Value("2"), json::Value("H as given"),
                     json::Value(control::poly_to_string(h.numerator())),
                     json::Value(control::poly_to_string(h.denominator()))});

    TransferFunction result;
    if (connection == "series") {
        result = g.series(h);
        steps.push_back({json::Value("3"), json::Value("series: numerators multiply, "
                                                       "denominators multiply"),
                         json::Value(control::poly_to_string(result.numerator())),
                         json::Value(control::poly_to_string(result.denominator()))});
    } else if (connection == "parallel") {
        result = g.parallel(h);
        steps.push_back({json::Value("3"),
                         json::Value("parallel: G_n H_d + H_n G_d over G_d H_d"),
                         json::Value(control::poly_to_string(result.numerator())),
                         json::Value(control::poly_to_string(result.denominator()))});
    } else {
        const bool negative = (connection == "negative_feedback");
        const TransferFunction loop = g.series(h);
        steps.push_back({json::Value("3"), json::Value("loop gain G H"),
                         json::Value(control::poly_to_string(loop.numerator())),
                         json::Value(control::poly_to_string(loop.denominator()))});
        const Poly one_plus = negative
                                  ? control::poly_add(loop.denominator(), loop.numerator())
                                  : control::poly_subtract(loop.denominator(), loop.numerator());
        steps.push_back({json::Value("4"),
                         json::Value(negative ? "1 + G H over a common denominator"
                                              : "1 - G H over a common denominator"),
                         json::Value(control::poly_to_string(one_plus)),
                         json::Value(control::poly_to_string(loop.denominator()))});
        result = g.feedback(h, negative);
        steps.push_back({json::Value("5"),
                         json::Value(negative ? "close: G / (1 + G H)" : "close: G / (1 - G H)"),
                         json::Value(control::poly_to_string(result.numerator())),
                         json::Value(control::poly_to_string(result.denominator()))});
    }

    json::Value out = json::Value::object();
    emit_tf(out, result);
    out.set("steps", json::from_table({"step", "rule", "numerator", "denominator"}, steps));
    out.set("poles", json::from_complex_set(result.poles()));
    out.set("zeros", json::from_complex_set(result.zeros()));
    out.set("dc_gain", finite_or_text(result.dc_gain()));
    out.set("stable", json::Value(result.is_stable()));
    return out;
}

// ---------------------------------------------------------------------------
// standard_links (session 6)
// ---------------------------------------------------------------------------

[[nodiscard]] json::Value op_standard_links(const json::Value& args) {
    const double k = optional_scalar(args, "gain", 1.0, 0.01, 100.0);
    const double tau = optional_scalar(args, "time_constant", 0.2, 0.001, 10.0);
    const double zeta = optional_scalar(args, "damping", 0.3, 0.0, 3.0);
    const double omega_n = optional_scalar(args, "natural_frequency", 10.0, 0.1, 500.0);
    const double dead_time = optional_scalar(args, "dead_time", 0.05, 0.0, 2.0);
    const double lead = optional_scalar(args, "lead_time_constant", 0.5, 0.0, 10.0);
    const double duration = optional_scalar(args, "duration", 2.0, 0.01, 60.0);
    const std::size_t samples =
        static_cast<std::size_t>(optional_int(args, "samples", 401, 32, 4000));
    const double omega_min = optional_scalar(args, "omega_min", 0.1, 1e-3, 100.0);
    const double omega_max = optional_scalar(args, "omega_max", 1000.0, 1.0, 1e6);
    if (omega_max <= omega_min) {
        throw StudyError("parameter 'omega_max' must be greater than 'omega_min'");
    }

    struct Link {
        std::string name;
        TransferFunction tf;
        std::string slope;
        std::string note;
    };
    const double filter_tau = tau / 100.0;
    std::vector<Link> links;
    links.reserve(7);
    links.push_back({"proportional", TransferFunction::gain(k), "0 dB/decade",
                     "no memory at all: the output is the input scaled, phase 0 at every "
                     "frequency"});
    links.push_back({"integrating", TransferFunction::integrator(k), "-20 dB/decade",
                     "the joint itself contains one of these, from velocity to angle; constant "
                     "-90 degrees"});
    links.push_back({"differentiating", TransferFunction({k, 0.0}, {filter_tau, 1.0}),
                     "+20 dB/decade",
                     "the ideal K s is improper and unrealisable, so it is shown filtered with "
                     "tau = T/100 = " + json::number_to_string(filter_tau) + " s"});
    links.push_back({"first_order_lag", TransferFunction::first_order_lag(k, tau),
                     "0 then -20 dB/decade",
                     "breaks at 1/T = " + json::number_to_string(1.0 / tau) +
                         " rad/s, where the phase is exactly -45 degrees"});
    links.push_back({"second_order", TransferFunction::second_order(k, zeta, omega_n),
                     "0 then -40 dB/decade",
                     zeta < 0.707 ? "resonant: the magnitude peaks near omega_n and the step "
                                    "response overshoots"
                                  : "no resonant peak at this damping"});
    links.push_back({"dead_time", TransferFunction::pade_delay(dead_time).scaled(k),
                     "0 dB/decade",
                     dead_time > 0.0
                         ? "a pure delay costs phase without costing magnitude, which is why it "
                           "destroys loops; shown as a first-order Pade approximation, exact only "
                           "below about 1/tau"
                         : "dead time is zero, so this link is the proportional one"});
    links.push_back({"lead_lag", TransferFunction::lead_lag(k, lead, tau),
                     lead > tau ? "+20 then 0 dB/decade" : "0 then -20 dB/decade",
                     lead > tau ? "lead dominates: it adds phase, which is how a controller buys "
                                  "back phase margin"
                                : "lag dominates: it removes phase but attenuates high "
                                  "frequency"});

    json::Value step_set = json::Value::array();
    json::Value magnitude_set = json::Value::array();
    json::Value phase_set = json::Value::array();
    std::vector<std::vector<json::Value>> summary;
    step_set.reserve(links.size());
    magnitude_set.reserve(links.size());
    phase_set.reserve(links.size());
    summary.reserve(links.size());

    for (const auto& link : links) {
        const control::Response response = link.tf.step_response(duration, samples);
        step_set.push_back(series_of(link.name, response));
        const control::BodeData data = control::bode(link.tf, omega_min, omega_max, 400);
        magnitude_set.push_back(json::from_series(link.name, data.omega, data.magnitude_db));
        phase_set.push_back(json::from_series(link.name, data.omega, data.phase_deg));
        summary.push_back({json::Value(link.name), json::Value(link.tf.to_string()),
                           json::Value(link.slope), finite_or_text(link.tf.dc_gain()),
                           json::Value(link.note)});
    }

    json::Value out = json::Value::object();
    out.set("step_responses", std::move(step_set));
    out.set("magnitude_db", std::move(magnitude_set));
    out.set("phase_deg", std::move(phase_set));
    out.set("summary",
            json::from_table({"link", "W(s)", "high-frequency slope", "dc gain", "note"}, summary));
    return out;
}

// ---------------------------------------------------------------------------
// time_response (sessions 5 and 14)
// ---------------------------------------------------------------------------

[[nodiscard]] json::Value op_time_response(const json::Value& args) {
    const control::JointPlant plant = read_plant(args);
    const double delay = read_delay_seconds(args);
    const double loop_gain = optional_scalar(args, "loop_gain", 1.0, 0.0, 500.0);
    const bool closed = optional_bool(args, "closed_loop", true);
    const std::string input = optional_enum(args, "input", "step", {"step", "impulse"});
    const double setpoint = optional_scalar(args, "setpoint", 1.0, -kPi, kPi);
    const double band = 0.01 * optional_scalar(args, "settling_band", 2.0, 0.1, 20.0);
    const double duration = optional_scalar(args, "duration", 10.0, 0.01, 120.0);
    const std::size_t samples =
        static_cast<std::size_t>(optional_int(args, "samples", 801, 32, 8000));

    const TransferFunction loop = with_delay(plant.tf, delay).scaled(loop_gain);
    const TransferFunction driven = closed ? loop.closed_loop() : loop;

    control::Response response = (input == "step")
                                     ? driven.step_response(duration, samples, setpoint)
                                     : driven.impulse_response(duration, samples);
    if (input == "impulse" && setpoint != 1.0) {
        for (double& v : response.y) {
            v *= setpoint;
        }
    }

    const bool stable = driven.is_stable();
    double final_value = response.y.empty() ? 0.0 : response.y.back();
    if (input == "step" && closed && stable) {
        final_value = driven.dc_gain() * setpoint;
    } else if (input == "impulse") {
        final_value = 0.0;
    }
    control::TransientSpec spec =
        control::transient_spec(response, input == "step" ? setpoint : 0.0, final_value, band);
    if (!stable) {
        spec.settled = false;
    }

    json::Value series_set = json::Value::array();
    series_set.push_back(series_of("response", response));
    if (input == "step") {
        control::Response command = response;
        for (double& v : command.y) {
            v = setpoint;
        }
        series_set.push_back(series_of("setpoint", command));
    }

    const DominantPair dominant = dominant_pair(driven.poles());

    json::Value out = json::Value::object();
    out.set("responses", std::move(series_set));
    emit_tf(out, driven);
    emit_spec(out, spec);
    out.set("damping_ratio", json::Value(dominant.zeta));
    out.set("natural_frequency", json::Value(dominant.omega_n));
    out.set("poles", json::from_complex_set(driven.poles()));
    out.set("stable", json::Value(stable));
    return out;
}

// ---------------------------------------------------------------------------
// frequency_response (sessions 7 and 11)
// ---------------------------------------------------------------------------

[[nodiscard]] json::Value op_frequency_response(const json::Value& args) {
    const control::JointPlant plant = read_plant(args);
    const double delay = read_delay_seconds(args);
    const double loop_gain = optional_scalar(args, "loop_gain", 1.0, 0.01, 500.0);
    const double omega_min = optional_scalar(args, "omega_min", 0.01, 1e-3, 100.0);
    const double omega_max = optional_scalar(args, "omega_max", 1000.0, 1.0, 1e6);
    const std::size_t points =
        static_cast<std::size_t>(optional_int(args, "points", 800, 64, 8000));
    if (omega_max <= omega_min) {
        throw StudyError("parameter 'omega_max' must be greater than 'omega_min'");
    }

    const TransferFunction loop = with_delay(plant.tf, delay).scaled(loop_gain);
    const control::BodeData data = control::bode(loop, omega_min, omega_max, points);
    const control::Margins m = control::margins_from_bode(data);
    const control::NyquistResult contour =
        control::nyquist(loop, omega_min, omega_max, std::min<std::size_t>(points, 1200));
    const bool stable = loop.closed_loop().is_stable();

    json::Value bode_set = json::Value::array();
    bode_set.push_back(json::from_series("magnitude [dB]", data.omega, data.magnitude_db));
    bode_set.push_back(json::from_series("phase [deg]", data.omega, data.phase_deg));

    std::string verdict;
    if (!m.has_phase_margin) {
        verdict = "the magnitude never crosses 0 dB inside this range, so there is no gain "
                  "crossover and no phase margin to report: widen the range or raise the gain. ";
    } else {
        verdict = "phase margin " + json::number_to_string(m.phase_margin_deg) +
                  " deg at omega = " + json::number_to_string(m.gain_crossover_omega) +
                  " rad/s. ";
    }
    if (!m.has_gain_margin) {
        verdict += "The phase never reaches -180 degrees in this range, so the gain margin is "
                   "effectively infinite: with no latency this plant cannot be driven unstable "
                   "by gain alone.";
    } else {
        verdict += "Gain margin " + json::number_to_string(m.gain_margin_db) +
                   " dB at omega = " + json::number_to_string(m.phase_crossover_omega) +
                   " rad/s, i.e. the gain may rise by a factor " +
                   json::number_to_string(m.gain_margin_linear) + " before the loop oscillates.";
    }

    json::Value out = json::Value::object();
    out.set("bode", std::move(bode_set));
    out.set("nyquist", json::from_complex_set(contour.contour));
    emit_margins(out, m);
    out.set("stable", json::Value(stable));
    out.set("verdict", json::Value(verdict));
    return out;
}

// ---------------------------------------------------------------------------
// stability (sessions 10, 11, 12)
// ---------------------------------------------------------------------------

[[nodiscard]] json::Value op_stability(const json::Value& args) {
    const control::JointPlant plant = read_plant(args);
    const double delay = read_delay_seconds(args);
    const std::string criterion =
        optional_enum(args, "criterion", "algebraic", {"algebraic", "nyquist", "roots"});
    const double loop_gain = optional_scalar(args, "loop_gain", 1.0, 0.01, 5000.0);

    const TransferFunction base = with_delay(plant.tf, delay);
    const TransferFunction loop = base.scaled(loop_gain);
    const Poly characteristic = control::poly_add(loop.numerator(), loop.denominator());
    const std::vector<Complex> roots = control::poly_roots(characteristic);

    bool stable_by_roots = true;
    int rhp_roots = 0;
    for (const auto& r : roots) {
        if (r.real() >= -1e-9) {
            stable_by_roots = false;
        }
        if (r.real() > 1e-9) {
            ++rhp_roots;
        }
    }

    const control::RouthArray routh = control::routh_array(characteristic);
    const control::NyquistResult contour = control::nyquist(loop, 1e-3, 1e5, 1200);
    const GainLimit limit = critical_gain(base, 1e-3, 1e5);

    bool stable = stable_by_roots;
    std::string explanation;
    if (criterion == "algebraic") {
        stable = routh.stable;
        explanation = "Routh-Hurwitz on " + control::poly_to_string(characteristic) + ": " +
                      routh.note + ".";
    } else if (criterion == "nyquist") {
        stable = contour.stable;
        explanation = "Nyquist: " + contour.note + ".";
    } else {
        explanation = stable_by_roots
                          ? "every root of the characteristic polynomial has a negative real "
                            "part, so every mode decays"
                          : std::to_string(rhp_roots) +
                                " root(s) of the characteristic polynomial lie in the closed "
                                "right half plane, so at least one mode does not decay";
        explanation += ".";
    }
    if (limit.found) {
        explanation += " Stability is lost at loop gain " + json::number_to_string(limit.gain) +
                       ", where the loop oscillates at " + json::number_to_string(limit.omega) +
                       " rad/s.";
    } else {
        explanation += " No finite gain in [1e-3, 1e5] destabilises this loop: without latency a "
                       "type-1 second-order plant under proportional control is stable for every "
                       "positive gain.";
    }

    std::vector<std::string> columns;
    columns.push_back("row");
    const std::size_t width = routh.rows.empty() ? 0 : routh.rows[0].size();
    for (std::size_t c = 0; c < width; ++c) {
        columns.push_back("c" + std::to_string(c));
    }
    std::vector<std::vector<json::Value>> routh_rows;
    routh_rows.reserve(routh.rows.size());
    for (std::size_t r = 0; r < routh.rows.size(); ++r) {
        std::vector<json::Value> row;
        row.reserve(width + 1);
        row.push_back(json::Value(r < routh.labels.size() ? routh.labels[r] : std::to_string(r)));
        for (std::size_t c = 0; c < width; ++c) {
            row.push_back(json::Value(routh.rows[r][c]));
        }
        routh_rows.push_back(std::move(row));
    }

    json::Value out = json::Value::object();
    out.set("stable", json::Value(stable));
    out.set("criterion", json::Value(criterion));
    out.set("explanation", json::Value(explanation));
    out.set("characteristic_polynomial",
            json::Value(control::poly_to_string(characteristic) + " = 0"));
    out.set("routh", json::from_table(columns, routh_rows));
    out.set("sign_changes", json::Value(routh.sign_changes));
    out.set("encirclements", json::Value(contour.encirclements));
    out.set("open_loop_rhp_poles", json::Value(contour.open_loop_rhp_poles));
    out.set("closed_loop_rhp_poles", json::Value(contour.closed_loop_rhp_poles));
    out.set("closed_loop_poles", json::from_complex_set(roots));
    out.set("critical_gain", json::Value(limit.found ? limit.gain : 0.0));
    out.set("critical_frequency", json::Value(limit.found ? limit.omega : 0.0));
    out.set("agrees_with_roots", json::Value(stable == stable_by_roots));
    return out;
}

// ---------------------------------------------------------------------------
// root_locus (session 12)
// ---------------------------------------------------------------------------

[[nodiscard]] json::Value op_root_locus(const json::Value& args) {
    const control::JointPlant plant = read_plant(args);
    const double delay = read_delay_seconds(args);
    const double gain_min = optional_scalar(args, "gain_min", 0.01, 1e-3, 1e4);
    const double gain_max = optional_scalar(args, "gain_max", 1000.0, 0.1, 1e6);
    const std::size_t samples =
        static_cast<std::size_t>(optional_int(args, "samples", 240, 16, 2000));
    if (gain_max <= gain_min) {
        throw StudyError("parameter 'gain_max' must be greater than 'gain_min'");
    }

    const TransferFunction base = with_delay(plant.tf, delay);
    const std::vector<Complex> open_poles = base.poles();
    const std::vector<Complex> open_zeros = base.zeros();
    const std::size_t n = open_poles.size();

    std::vector<std::vector<Complex>> branches(n);
    for (auto& branch : branches) {
        branch.reserve(samples);
    }
    std::vector<Complex> previous = open_poles;
    for (std::size_t i = 0; i < samples; ++i) {
        const double frac = static_cast<double>(i) / static_cast<double>(samples - 1);
        const double k = std::pow(10.0, std::log10(gain_min) +
                                            frac * (std::log10(gain_max) - std::log10(gain_min)));
        std::vector<Complex> roots = control::poly_roots(
            control::poly_add(control::poly_scale(base.numerator(), k), base.denominator()));
        // Greedy nearest-neighbour matching keeps each branch continuous.
        std::vector<bool> taken(roots.size(), false);
        for (std::size_t b = 0; b < n && b < previous.size(); ++b) {
            std::size_t best = roots.size();
            double best_distance = std::numeric_limits<double>::max();
            for (std::size_t r = 0; r < roots.size(); ++r) {
                if (taken[r]) {
                    continue;
                }
                const double distance = std::abs(roots[r] - previous[b]);
                if (distance < best_distance) {
                    best_distance = distance;
                    best = r;
                }
            }
            if (best < roots.size()) {
                taken[best] = true;
                branches[b].push_back(roots[best]);
                previous[b] = roots[best];
            }
        }
    }

    // Breakaway points: N D' - D N' = 0, keeping the real roots whose gain is
    // positive, because only those lie on the locus.
    std::vector<std::vector<json::Value>> breakaway;
    {
        const Poly expression =
            control::poly_subtract(control::poly_multiply(base.numerator(),
                                                          poly_derivative(base.denominator())),
                                   control::poly_multiply(base.denominator(),
                                                          poly_derivative(base.numerator())));
        for (const auto& root : control::poly_roots(expression)) {
            if (std::abs(root.imag()) > 1e-6) {
                continue;
            }
            const Complex numerator = control::poly_eval(base.numerator(), root);
            if (std::abs(numerator) <= 1e-18) {
                continue;
            }
            const Complex gain = -control::poly_eval(base.denominator(), root) / numerator;
            if (gain.real() <= 0.0) {
                continue;
            }
            breakaway.push_back({json::Value(root.real()), json::Value(gain.real())});
        }
    }

    const std::size_t m = open_zeros.size();
    std::vector<std::vector<json::Value>> asymptotes;
    double centroid = 0.0;
    if (n > m) {
        double sum_poles = 0.0;
        double sum_zeros = 0.0;
        for (const auto& p : open_poles) {
            sum_poles += p.real();
        }
        for (const auto& z : open_zeros) {
            sum_zeros += z.real();
        }
        centroid = (sum_poles - sum_zeros) / static_cast<double>(n - m);
        for (std::size_t k = 0; k < n - m; ++k) {
            const double angle = (2.0 * static_cast<double>(k) + 1.0) * 180.0 /
                                 static_cast<double>(n - m);
            asymptotes.push_back({json::Value(static_cast<int>(k)), json::Value(centroid),
                                  json::Value(angle)});
        }
    }

    const GainLimit limit = critical_gain(base, gain_min, gain_max);

    json::Value branch_set = json::Value::array();
    branch_set.reserve(branches.size());
    for (const auto& branch : branches) {
        branch_set.push_back(json::from_complex_set(branch));
    }

    json::Value out = json::Value::object();
    out.set("branches", std::move(branch_set));
    out.set("open_loop_poles", json::from_complex_set(open_poles));
    out.set("open_loop_zeros", json::from_complex_set(open_zeros));
    out.set("breakaway_points", json::from_table({"sigma", "gain"}, breakaway));
    out.set("asymptotes", json::from_table({"branch", "centroid", "angle_deg"}, asymptotes));
    out.set("asymptote_centroid", json::Value(centroid));
    out.set("critical_gain", json::Value(limit.found ? limit.gain : 0.0));
    out.set("critical_frequency", json::Value(limit.found ? limit.omega : 0.0));
    out.set("crosses_imaginary_axis", json::Value(limit.found));
    out.set("note",
            json::Value(std::to_string(n) + " branches start at the open-loop poles and " +
                        std::to_string(m) + " of them end at the open-loop zeros; the other " +
                        std::to_string(n - m) + " run to infinity along asymptotes through " +
                        json::number_to_string(centroid) +
                        (limit.found
                             ? ". The locus crosses the imaginary axis at gain " +
                                   json::number_to_string(limit.gain) +
                                   ", which is the same number the Routh array gives."
                             : ". No branch crosses the imaginary axis inside the swept gain "
                               "range, so this loop is stable for every gain in it.")));
    return out;
}

// ---------------------------------------------------------------------------
// steady_state_accuracy (session 15)
// ---------------------------------------------------------------------------

[[nodiscard]] json::Value op_steady_state_accuracy(const json::Value& args) {
    const control::JointPlant plant = read_plant(args);
    const double loop_gain = optional_scalar(args, "loop_gain", 1.0, 0.01, 500.0);
    const std::string controller =
        optional_enum(args, "controller", "proportional", {"proportional", "pi", "pid"});
    const double ki = optional_scalar(args, "ki", 1.0, 0.0, 500.0);
    const double kd = optional_scalar(args, "kd", 0.1, 0.0, 100.0);

    TransferFunction c = TransferFunction::gain(loop_gain);
    if (controller == "pi") {
        c = control::pid_controller(loop_gain, ki, 0.0, 0.0);
    } else if (controller == "pid") {
        c = control::pid_controller(loop_gain, ki, kd, 0.01);
    }
    const TransferFunction loop = c.series(plant.tf);

    const Poly& den = loop.denominator();
    std::size_t type = 0;
    while (type < den.size() && std::abs(den[den.size() - 1 - type]) <= 1e-14) {
        ++type;
    }
    Poly reduced(den.begin(), den.end() - static_cast<std::ptrdiff_t>(type));
    const double numerator_at_zero = loop.numerator().back();
    const double reduced_at_zero = reduced.empty() ? 1.0 : reduced.back();
    const double finite_limit = numerator_at_zero / reduced_at_zero;
    const double infinity = std::numeric_limits<double>::infinity();

    const double kp_constant = (type == 0) ? finite_limit : infinity;
    const double kv_constant = (type == 0) ? 0.0 : ((type == 1) ? finite_limit : infinity);
    const double ka_constant = (type <= 1) ? 0.0 : ((type == 2) ? finite_limit : infinity);

    const double e_step = (type == 0) ? 1.0 / (1.0 + kp_constant) : 0.0;
    const double e_ramp = (kv_constant == 0.0) ? infinity : 1.0 / kv_constant;
    const double e_parabola = (ka_constant == 0.0) ? infinity : 1.0 / ka_constant;

    std::vector<std::vector<json::Value>> rows;
    rows.push_back({json::Value("step  r(t) = 1"), json::Value("Kp"), finite_or_text(kp_constant),
                    json::Value("1 / (1 + Kp)"), finite_or_text(e_step)});
    rows.push_back({json::Value("ramp  r(t) = t"), json::Value("Kv"), finite_or_text(kv_constant),
                    json::Value("1 / Kv"), finite_or_text(e_ramp)});
    rows.push_back({json::Value("parabola  r(t) = t^2/2"), json::Value("Ka"),
                    finite_or_text(ka_constant), json::Value("1 / Ka"),
                    finite_or_text(e_parabola)});

    std::string explanation = "This loop is type " + std::to_string(type) + ": ";
    if (type == 0) {
        explanation += "with no free integrator it holds a commanded angle only with a residual "
                       "error of 1/(1+Kp), and it cannot follow a ramp at all.";
    } else if (type == 1) {
        explanation += "the joint's own velocity-to-angle integration makes the step error "
                       "exactly zero, the ramp error a constant 1/Kv = " +
                       json::number_to_string(e_ramp) +
                       " rad, and the parabolic error unbounded.";
    } else {
        explanation += "the integral term adds a second integrator, so both the step and the ramp "
                       "are followed with no error at all and the parabola costs a finite 1/Ka. "
                       "That extra integrator is not free: it costs phase margin.";
    }

    json::Value out = json::Value::object();
    out.set("system_type", json::Value(static_cast<int>(type)));
    out.set("kp", finite_or_text(kp_constant));
    out.set("kv", finite_or_text(kv_constant));
    out.set("ka", finite_or_text(ka_constant));
    out.set("errors",
            json::from_table({"input", "constant", "value", "formula", "steady-state error"},
                             rows));
    out.set("loop_transfer_function", json::Value(loop.to_string()));
    out.set("stable", json::Value(loop.closed_loop().is_stable()));
    out.set("explanation", json::Value(explanation));
    return out;
}

}  // namespace

namespace {

// ---------------------------------------------------------------------------
// pid (session 20)
// ---------------------------------------------------------------------------

[[nodiscard]] json::Value op_pid(const json::Value& args) {
    const control::JointPlant plant = read_plant(args);
    const double delay = read_delay_seconds(args);
    const double kp = optional_scalar(args, "kp", 8.0, 0.0, 500.0);
    const double ki = optional_scalar(args, "ki", 4.0, 0.0, 500.0);
    const double kd = optional_scalar(args, "kd", 1.0, 0.0, 100.0);
    const double derivative_tau = optional_scalar(args, "derivative_tau", 0.01, 1e-4, 1.0);
    const double setpoint = optional_scalar(args, "setpoint", 1.0, -kPi, kPi);
    const double band = 0.01 * optional_scalar(args, "settling_band", 2.0, 0.1, 20.0);
    const double duration = optional_scalar(args, "duration", 5.0, 0.01, 120.0);
    const std::size_t samples =
        static_cast<std::size_t>(optional_int(args, "samples", 801, 32, 8000));

    const TransferFunction controller = control::pid_controller(kp, ki, kd, derivative_tau);
    const TransferFunction plant_tf = with_delay(plant.tf, delay);
    const TransferFunction loop = controller.series(plant_tf);
    const TransferFunction closed = loop.closed_loop();
    const TransferFunction effort = over_one_plus(controller, loop);

    const control::Response response = closed.step_response(duration, samples, setpoint);
    const control::Response command = effort.step_response(duration, samples, setpoint);

    const bool stable = closed.is_stable();
    const double final_value = stable ? closed.dc_gain() * setpoint
                                      : (response.y.empty() ? 0.0 : response.y.back());
    control::TransientSpec spec = control::transient_spec(response, setpoint, final_value, band);
    if (!stable) {
        spec.settled = false;
    }

    double peak_command = 0.0;
    for (const double v : command.y) {
        peak_command = std::max(peak_command, std::abs(v));
    }
    const double peak_torque = peak_command * plant.drive_torque;
    const double rating = GP8_LINKS[plant.joint].max_torque;
    const bool saturated = peak_torque > rating;

    control::Response torque = command;
    for (double& v : torque.y) {
        v *= plant.drive_torque;
    }
    control::Response reference = response;
    for (double& v : reference.y) {
        v = setpoint;
    }

    json::Value series_set = json::Value::array();
    series_set.push_back(series_of("setpoint [rad]", reference));
    series_set.push_back(series_of("response [rad]", response));
    series_set.push_back(series_of("control torque [N m]", torque));

    const control::Margins m = control::loop_margins(loop, 1e-2, 1e4, 900);

    json::Value out = json::Value::object();
    out.set("responses", std::move(series_set));
    out.set("controller", json::Value(controller.to_string()));
    emit_tf(out, closed);
    out.set("poles", json::from_complex_set(closed.poles()));
    emit_spec(out, spec);
    out.set("gain_margin_db", json::Value(m.has_gain_margin ? m.gain_margin_db : 0.0));
    out.set("phase_margin_deg", json::Value(m.has_phase_margin ? m.phase_margin_deg : 0.0));
    out.set("peak_torque", json::Value(peak_torque));
    out.set("torque_rating", json::Value(rating));
    out.set("torque_saturated", json::Value(saturated));
    out.set("effort_note",
            json::Value(saturated
                            ? "the commanded torque peaks at " +
                                  json::number_to_string(peak_torque) + " N m against a rating of " +
                                  json::number_to_string(rating) +
                                  " N m, so these gains cannot be realised on the real axis: the "
                                  "actuator saturates and the response you see above is optimistic"
                            : "the commanded torque peaks at " +
                                  json::number_to_string(peak_torque) + " N m, which is " +
                                  json::number_to_string(100.0 * peak_torque /
                                                         std::max(rating, 1e-9)) +
                                  " % of the axis rating, so these gains are physically "
                                  "realisable"));
    out.set("stable", json::Value(stable));
    return out;
}

// ---------------------------------------------------------------------------
// pid_tune (session 21)
// ---------------------------------------------------------------------------

struct ReactionCurve {
    double slope = 0.0;           // R, the steepest rate of the open-loop step
    double apparent_delay = 0.0;  // L, where that tangent crosses zero
    bool found = false;
};

[[nodiscard]] ReactionCurve reaction_curve(const TransferFunction& plant, double duration,
                                           std::size_t samples) {
    ReactionCurve out;
    const control::Response response = plant.step_response(duration, samples);
    if (response.t.size() < 3) {
        return out;
    }
    const double dt = response.t[1] - response.t[0];
    double best_slope = 0.0;
    std::size_t best_index = 0;
    for (std::size_t i = 1; i + 1 < response.y.size(); ++i) {
        const double slope = (response.y[i + 1] - response.y[i - 1]) / (2.0 * dt);
        if (slope > best_slope) {
            best_slope = slope;
            best_index = i;
        }
    }
    if (best_slope <= 0.0) {
        return out;
    }
    out.slope = best_slope;
    out.apparent_delay = response.t[best_index] - response.y[best_index] / best_slope;
    out.found = out.apparent_delay > 1e-9;
    return out;
}

struct TuningSet {
    std::string method;
    double kp = 0.0;
    double ki = 0.0;
    double kd = 0.0;
    bool available = false;
    std::string note;
};

[[nodiscard]] json::Value op_pid_tune(const json::Value& args) {
    const control::JointPlant plant = read_plant(args);
    const double delay = read_delay_seconds(args);
    const std::vector<std::string> methods = {"ziegler_nichols_step", "ziegler_nichols_ultimate",
                                              "cohen_coon", "pole_placement"};
    const std::string selected =
        optional_enum(args, "method", "ziegler_nichols_ultimate", methods);
    const double zeta = optional_scalar(args, "target_damping", 0.707, 0.1, 1.5);
    const double omega_n = optional_scalar(args, "target_frequency", 6.0, 0.1, 200.0);
    const double alpha = optional_scalar(args, "third_pole_ratio", 4.0, 1.0, 20.0);
    const double band = 0.01 * optional_scalar(args, "settling_band", 2.0, 0.1, 20.0);
    const double duration = optional_scalar(args, "duration", 8.0, 0.01, 120.0);
    const std::size_t samples =
        static_cast<std::size_t>(optional_int(args, "samples", 801, 32, 8000));

    const TransferFunction plant_tf = with_delay(plant.tf, delay);
    const ReactionCurve curve = reaction_curve(plant.tf, std::max(10.0 * plant.time_constant, 1.0),
                                               2000);
    const GainLimit ultimate = critical_gain(plant_tf, 1e-3, 1e5);
    const double ultimate_period =
        (ultimate.found && ultimate.omega > 0.0) ? 2.0 * kPi / ultimate.omega : 0.0;

    std::vector<TuningSet> sets;
    sets.reserve(4);
    {
        TuningSet s;
        s.method = "ziegler_nichols_step";
        if (curve.found) {
            const double kp = 1.2 / (curve.slope * curve.apparent_delay);
            const double ti = 2.0 * curve.apparent_delay;
            const double td = 0.5 * curve.apparent_delay;
            s.kp = kp;
            s.ki = kp / ti;
            s.kd = kp * td;
            s.available = true;
            s.note = "tangent read off the open-loop step: R = " +
                     json::number_to_string(curve.slope) + ", L = " +
                     json::number_to_string(curve.apparent_delay) + " s";
        } else {
            s.note = "the open-loop step response has no measurable tangent intercept";
        }
        sets.push_back(std::move(s));
    }
    {
        TuningSet s;
        s.method = "ziegler_nichols_ultimate";
        if (ultimate.found && ultimate_period > 0.0) {
            const double kp = 0.6 * ultimate.gain;
            const double ti = 0.5 * ultimate_period;
            const double td = 0.125 * ultimate_period;
            s.kp = kp;
            s.ki = kp / ti;
            s.kd = kp * td;
            s.available = true;
            s.note = "Ku = " + json::number_to_string(ultimate.gain) + ", Pu = " +
                     json::number_to_string(ultimate_period) + " s";
        } else {
            s.note = "this loop has no finite ultimate gain: with no latency a type-1 "
                     "second-order plant never oscillates under proportional control, so raise "
                     "delay_ms to use this method";
        }
        sets.push_back(std::move(s));
    }
    {
        TuningSet s;
        s.method = "cohen_coon";
        if (curve.found) {
            const double kp = 1.35 / (curve.slope * curve.apparent_delay);
            const double ti = 2.5 * curve.apparent_delay;
            const double td = 0.37 * curve.apparent_delay;
            s.kp = kp;
            s.ki = kp / ti;
            s.kd = kp * td;
            s.available = true;
            s.note = "Cohen-Coon in its integrating-process limit, the form that applies to a "
                     "K/(s(Ts+1)) joint";
        } else {
            s.note = "needs the same reaction curve the step method needs";
        }
        sets.push_back(std::move(s));
    }
    {
        TuningSet s;
        s.method = "pole_placement";
        const double j = plant.total_inertia;
        const double b = plant.damping;
        const double ka = plant.drive_torque;
        s.kd = (j * (2.0 * zeta + alpha) * omega_n - b) / ka;
        s.kp = j * omega_n * omega_n * (1.0 + 2.0 * zeta * alpha) / ka;
        s.ki = j * alpha * omega_n * omega_n * omega_n / ka;
        s.available = true;
        s.note = "exact: J s^3 + (b + Ka Kd) s^2 + Ka Kp s + Ka Ki matched to "
                 "J (s^2 + 2 zeta wn s + wn^2)(s + " +
                 json::number_to_string(alpha) + " wn)";
        sets.push_back(std::move(s));
    }

    std::vector<std::vector<json::Value>> comparison;
    comparison.reserve(sets.size());
    json::Value response_set = json::Value::array();
    TuningSet chosen = sets.back();
    for (const auto& s : sets) {
        if (s.method == selected) {
            chosen = s;
        }
        if (!s.available) {
            comparison.push_back({json::Value(s.method), json::Value("n/a"), json::Value("n/a"),
                                  json::Value("n/a"), json::Value("n/a"), json::Value("n/a"),
                                  json::Value("n/a"), json::Value("n/a"), json::Value(false),
                                  json::Value(s.note)});
            continue;
        }
        const TransferFunction c = control::pid_controller(s.kp, s.ki, s.kd, 0.01);
        const TransferFunction loop = c.series(plant_tf);
        const TransferFunction closed = loop.closed_loop();
        const bool stable = closed.is_stable();
        const control::Response response = closed.step_response(duration, samples);
        const double final_value =
            stable ? closed.dc_gain() : (response.y.empty() ? 0.0 : response.y.back());
        const control::TransientSpec spec =
            control::transient_spec(response, 1.0, final_value, band);
        const control::Margins m = control::loop_margins(loop, 1e-2, 1e4, 900);
        response_set.push_back(series_of(s.method, response));
        comparison.push_back({json::Value(s.method), json::Value(s.kp), json::Value(s.ki),
                              json::Value(s.kd), json::Value(spec.overshoot_percent),
                              json::Value(spec.settling_time),
                              json::Value(m.has_phase_margin ? m.phase_margin_deg : 0.0),
                              json::Value(m.has_gain_margin ? m.gain_margin_db : 0.0),
                              json::Value(stable), json::Value(s.note)});
    }

    const TransferFunction chosen_controller =
        control::pid_controller(chosen.kp, chosen.ki, chosen.kd, 0.01);
    const bool chosen_stable = chosen_controller.series(plant_tf).closed_loop().is_stable();

    std::vector<std::vector<json::Value>> reaction_rows;
    reaction_rows.push_back({json::Value("R, steepest slope"), json::Value(curve.slope),
                             json::Value("rad/s per command")});
    reaction_rows.push_back({json::Value("L, tangent intercept"),
                             json::Value(curve.apparent_delay), json::Value("s")});
    reaction_rows.push_back({json::Value("T, plant time constant"),
                             json::Value(plant.time_constant), json::Value("s")});

    json::Value out = json::Value::object();
    out.set("comparison",
            json::from_table({"method", "Kp", "Ki", "Kd", "overshoot [%]", "settling [s]",
                              "PM [deg]", "GM [dB]", "stable", "note"},
                             comparison));
    out.set("responses", std::move(response_set));
    out.set("selected", json::Value(chosen.method));
    out.set("kp", json::Value(chosen.kp));
    out.set("ki", json::Value(chosen.ki));
    out.set("kd", json::Value(chosen.kd));
    out.set("controller", json::Value(chosen_controller.to_string()));
    out.set("reaction_curve", json::from_table({"quantity", "value", "unit"}, reaction_rows));
    out.set("ultimate_gain", json::Value(ultimate.found ? ultimate.gain : 0.0));
    out.set("ultimate_period", json::Value(ultimate_period));
    out.set("stable", json::Value(chosen_stable));
    out.set("note",
            json::Value("The Ziegler-Nichols rules are deliberately aggressive - they were "
                        "designed for disturbance rejection in process plants, and on a joint "
                        "with an integrator of its own they overshoot hard. Pole placement is the "
                        "only row whose overshoot you chose in advance, which is why the lab "
                        "asks for it; the table exists so the difference is measured instead of "
                        "argued."));
    return out;
}

// ---------------------------------------------------------------------------
// advanced_structures (session 22)
// ---------------------------------------------------------------------------

[[nodiscard]] json::Value op_advanced_structures(const json::Value& args) {
    const control::JointPlant plant = read_plant(args);
    const double delay = read_delay_seconds(args);
    const std::string structure =
        optional_enum(args, "structure", "cascade",
                      {"cascade", "feedforward", "disturbance_compensation"});
    const double kp = optional_scalar(args, "kp", 8.0, 0.0, 500.0);
    const double ki = optional_scalar(args, "ki", 4.0, 0.0, 500.0);
    const double kd = optional_scalar(args, "kd", 1.0, 0.0, 100.0);
    const double derivative_tau = optional_scalar(args, "derivative_tau", 0.01, 1e-4, 1.0);
    const double inner_gain = optional_scalar(args, "inner_gain", 10.0, 0.1, 200.0);
    const double filter_tau = optional_scalar(args, "filter_tau", 0.05, 1e-3, 1.0);
    const double setpoint = optional_scalar(args, "setpoint", 1.0, -kPi, kPi);
    const double disturbance = optional_scalar(args, "disturbance_torque", 20.0, 0.0, 100.0);
    const double disturbance_time = optional_scalar(args, "disturbance_time", 2.0, 0.0, 60.0);
    const double duration = optional_scalar(args, "duration", 8.0, 0.1, 120.0);
    const std::size_t samples =
        static_cast<std::size_t>(optional_int(args, "samples", 801, 64, 8000));

    const double j = plant.total_inertia;
    const double b = plant.damping;
    const TransferFunction controller = control::pid_controller(kp, ki, kd, derivative_tau);
    const TransferFunction latency = (delay > 0.0) ? TransferFunction::pade_delay(delay)
                                                   : TransferFunction();
    const TransferFunction plant_tf = plant.tf.series(latency);
    const TransferFunction loop = controller.series(plant_tf);
    const TransferFunction baseline = loop.closed_loop();

    // Torque in, angle out. P_tau = 1/(J s^2 + b s) shares that denominator
    // with the loop, so the removable factor is cancelled here by hand rather
    // than left in a non-minimal realisation:
    //     P_tau/(1 + C P) = C_den * delay_den / (L_den + L_num).
    const auto sensitivity_of = [&latency](const TransferFunction& between,
                                           const TransferFunction& the_loop) {
        return TransferFunction(
            control::poly_multiply(between.denominator(), latency.denominator()),
            control::poly_add(the_loop.denominator(), the_loop.numerator()));
    };
    const TransferFunction baseline_sensitivity = sensitivity_of(controller, loop);

    TransferFunction structured = baseline;
    TransferFunction structured_sensitivity = baseline_sensitivity;
    bool stable_structured = loop.closed_loop().is_stable();
    std::string note;

    if (structure == "cascade") {
        const TransferFunction velocity_plant =
            with_delay(TransferFunction({plant.drive_torque}, {j, b}), delay);
        const TransferFunction inner_loop = velocity_plant.scaled(inner_gain);
        const TransferFunction inner_closed = inner_loop.closed_loop();
        // The outer controller of a cascade commands a VELOCITY, not a torque,
        // so its gains are not the single loop's gains: with the inner loop
        // flat out to well past the crossover, L_outer is about K_pos/s, which
        // crosses over at K_pos. Setting K_pos to the single loop's own
        // crossover frequency puts both loops at the same bandwidth, which is
        // the only way the comparison below measures the STRUCTURE instead of
        // measuring a gain change. The integral-to-proportional ratio of the
        // gains supplied is preserved; the derivative term is dropped because
        // damping is exactly the job the inner loop has taken over.
        const control::Margins single_margins = control::loop_margins(loop, 1e-2, 1e4, 900);
        const double outer_bandwidth = single_margins.has_phase_margin
                                           ? single_margins.gain_crossover_omega
                                           : 1.0 / plant.time_constant;
        const double integral_ratio = (kp > 1e-9) ? (ki / kp) : 0.0;
        const TransferFunction outer_controller = control::pid_controller(
            outer_bandwidth, outer_bandwidth * integral_ratio, 0.0, 0.0);
        const TransferFunction outer_loop =
            outer_controller.series(inner_closed.series(TransferFunction::integrator(1.0)));
        structured = outer_loop.closed_loop();
        structured_sensitivity = sensitivity_of(outer_controller, outer_loop);
        stable_structured = outer_loop.closed_loop().is_stable() &&
                            inner_loop.closed_loop().is_stable();
        note = "the inner velocity loop sees the load torque one integration earlier than the "
               "position loop does, so it corrects it before the angle has time to move, and it "
               "replaces the plant's own lag (Ts+1) with the inner loop's much faster one; the "
               "outer position gain is set to the single loop's own crossover frequency, " +
               json::number_to_string(outer_bandwidth) +
               " rad/s, so that both loops run at the same bandwidth and the comparison is about "
               "the structure rather than about a gain change. The price is that the inner loop's "
               "bandwidth now limits the outer one";
    } else if (structure == "feedforward") {
        // Model inversion (J s^2 + b s)/tau_drive, made proper by a filter.
        const Poly inverse_num = control::poly_scale(Poly{j, b, 0.0}, 1.0 / plant.drive_torque);
        const Poly filter = control::poly_multiply(Poly{filter_tau, 1.0}, Poly{filter_tau, 1.0});
        const TransferFunction feedforward(inverse_num, filter);
        // (C+F)P/(1+CP), again with the common C_den P_den factor cancelled:
        //     = (C_num F_den + F_num C_den) P_num / (F_den (L_den + L_num)).
        structured = TransferFunction(
            control::poly_multiply(controller.parallel(feedforward).numerator(),
                                   plant_tf.numerator()),
            control::poly_multiply(filter,
                                   control::poly_add(loop.denominator(), loop.numerator())));
        stable_structured = loop.closed_loop().is_stable();
        note = "feedforward injects the torque the trajectory is going to need before the error "
               "exists, so it removes tracking lag without entering the loop at all - the "
               "characteristic polynomial, and therefore every stability margin, is untouched; "
               "it does nothing for an unmeasured disturbance, which is what the two curves show";
    } else {
        // Disturbance observer: what it cannot estimate is (1 - Q).
        const TransferFunction high_pass({filter_tau, 0.0}, {filter_tau, 1.0});
        structured_sensitivity = baseline_sensitivity.series(high_pass);
        stable_structured = loop.closed_loop().is_stable();
        note = "the observer estimates the load from the measured motion and subtracts it; what "
               "survives is the part the filter cannot see, so the residual sensitivity is "
               "S(s)(1 - Q(s)) and the steady-state droop goes to zero while the first instant "
               "of the load step still gets through";
    }

    const control::Response track_without = baseline.step_response(duration, samples, setpoint);
    const control::Response track_with = structured.step_response(duration, samples, setpoint);
    const control::Response dist_without =
        baseline_sensitivity.step_response(duration, samples, 1.0);
    const control::Response dist_with =
        structured_sensitivity.step_response(duration, samples, 1.0);

    const control::Response total_without =
        add_shifted(track_without, dist_without, disturbance, disturbance_time);
    const control::Response total_with =
        add_shifted(track_with, dist_with, disturbance, disturbance_time);

    const double peak_without = peak_deviation(total_without, setpoint, disturbance_time);
    const double peak_with = peak_deviation(total_with, setpoint, disturbance_time);

    // Judged on the characteristic polynomial, not on the composed transfer
    // functions: a sensitivity function carries removable factors that would
    // otherwise be read as poles.
    const bool stable_without = loop.closed_loop().is_stable();
    const bool stable_with = stable_structured;
    const control::TransientSpec spec_without = control::transient_spec(
        track_without, setpoint, stable_without ? baseline.dc_gain() * setpoint : setpoint, 0.02);
    const control::TransientSpec spec_with = control::transient_spec(
        track_with, setpoint, stable_with ? structured.dc_gain() * setpoint : setpoint, 0.02);

    std::vector<std::vector<json::Value>> comparison;
    comparison.push_back({json::Value("overshoot [%]"),
                          json::Value(spec_without.overshoot_percent),
                          json::Value(spec_with.overshoot_percent)});
    comparison.push_back({json::Value("settling time [s]"), json::Value(spec_without.settling_time),
                          json::Value(spec_with.settling_time)});
    comparison.push_back({json::Value("rise time [s]"), json::Value(spec_without.rise_time),
                          json::Value(spec_with.rise_time)});
    comparison.push_back({json::Value("worst deviation from setpoint after the load step [rad]"),
                          json::Value(peak_without), json::Value(peak_with)});

    json::Value series_set = json::Value::array();
    series_set.push_back(series_of("single loop", total_without));
    series_set.push_back(series_of(structure, total_with));
    control::Response reference = total_without;
    for (double& v : reference.y) {
        v = setpoint;
    }
    series_set.push_back(series_of("setpoint", reference));

    json::Value out = json::Value::object();
    out.set("responses", std::move(series_set));
    out.set("structure", json::Value(structure));
    out.set("peak_error_without", json::Value(peak_without));
    out.set("peak_error_with", json::Value(peak_with));
    out.set("improvement_factor",
            json::Value(peak_with > 1e-15 ? peak_without / peak_with : 0.0));
    out.set("comparison", json::from_table({"indicator", "single loop", "with structure"},
                                           comparison));
    out.set("loop_without", json::Value(baseline.to_string()));
    out.set("loop_with", json::Value(structured.to_string()));
    out.set("stable_without", json::Value(stable_without));
    out.set("stable_with", json::Value(stable_with));
    out.set("note", json::Value(note));
    return out;
}

// ---------------------------------------------------------------------------
// robustness (sessions 18 and 19)
// ---------------------------------------------------------------------------

[[nodiscard]] json::Value op_robustness(const json::Value& args) {
    const int joint = optional_int(args, "joint", 2, 1, 6);
    const Eigen::Matrix<double, 6, 1> q =
        optional_vec6(args, "q", zero6(), -widest_joint_range(), widest_joint_range());
    const double velocity_fraction = optional_scalar(args, "velocity_fraction", 0.25, 0.02, 1.0);
    const double torque_fraction = optional_scalar(args, "torque_fraction", 1.0, 0.05, 1.0);
    const double delay = read_delay_seconds(args);
    const double kp = optional_scalar(args, "kp", 8.0, 0.0, 500.0);
    const double ki = optional_scalar(args, "ki", 4.0, 0.0, 500.0);
    const double kd = optional_scalar(args, "kd", 1.0, 0.0, 100.0);
    const double derivative_tau = optional_scalar(args, "derivative_tau", 0.01, 1e-4, 1.0);
    const double payload_min = optional_scalar(args, "payload_min", 0.0, 0.0, GP8_PAYLOAD_KG);
    const double payload_max =
        optional_scalar(args, "payload_max", GP8_PAYLOAD_KG, 0.0, GP8_PAYLOAD_KG);
    const double tolerance = 0.01 * optional_scalar(args, "inertia_tolerance", 20.0, 0.0, 100.0);
    const std::size_t points = static_cast<std::size_t>(optional_int(args, "samples", 17, 3, 200));
    const double required_gm = optional_scalar(args, "required_gain_margin", 6.0, 0.0, 40.0);
    const double required_pm = optional_scalar(args, "required_phase_margin", 45.0, 0.0, 90.0);
    const double duration = optional_scalar(args, "duration", 8.0, 0.1, 60.0);
    const std::size_t response_samples =
        static_cast<std::size_t>(optional_int(args, "response_samples", 401, 64, 4000));
    if (payload_max < payload_min) {
        throw StudyError("parameter 'payload_max' must not be below 'payload_min'");
    }

    const std::size_t index = static_cast<std::size_t>(joint - 1);
    const TransferFunction controller = control::pid_controller(kp, ki, kd, derivative_tau);

    struct Variant {
        std::string name;
        double scale;
    };
    const std::array<Variant, 3> variants = {Variant{"nominal inertia", 1.0},
                                             Variant{"inertia +tolerance", 1.0 + tolerance},
                                             Variant{"inertia -tolerance", 1.0 - tolerance}};

    std::vector<std::vector<json::Value>> sweep;
    sweep.reserve(points * variants.size());
    json::Value margin_set = json::Value::array();
    json::Value quality_set = json::Value::array();

    double worst_gm = std::numeric_limits<double>::max();
    double worst_pm = std::numeric_limits<double>::max();
    double worst_overshoot = 0.0;
    double worst_settling = 0.0;
    double worst_gm_payload = 0.0;
    double worst_pm_payload = 0.0;
    double worst_overshoot_payload = 0.0;
    double worst_settling_payload = 0.0;
    bool specification_met = true;
    double overshoot_light = 0.0;
    double overshoot_heavy = 0.0;
    double inertia_light = 1.0;
    double inertia_heavy = 1.0;
    double pm_light = 0.0;
    double pm_heavy = 0.0;

    for (const auto& variant : variants) {
        std::vector<double> payloads;
        std::vector<double> gains;
        std::vector<double> phases;
        std::vector<double> overshoots;
        std::vector<double> settlings;
        payloads.reserve(points);
        gains.reserve(points);
        phases.reserve(points);
        overshoots.reserve(points);
        settlings.reserve(points);

        for (std::size_t i = 0; i < points; ++i) {
            const double frac = (points == 1) ? 0.0
                                              : static_cast<double>(i) /
                                                    static_cast<double>(points - 1);
            const double payload = payload_min + frac * (payload_max - payload_min);
            const control::JointPlant nominal = control::build_joint_plant(
                index, q, payload, velocity_fraction, torque_fraction);
            const double inertia = nominal.link_inertia * variant.scale +
                                   nominal.payload_inertia + nominal.reflected_inertia;
            const TransferFunction scaled_plant =
                TransferFunction({nominal.drive_torque}, {inertia, nominal.damping, 0.0});
            const TransferFunction loop = controller.series(with_delay(scaled_plant, delay));
            const TransferFunction closed = loop.closed_loop();
            const control::Margins m = control::loop_margins(loop, 1e-2, 1e4, 700);
            const bool stable = closed.is_stable();
            const control::Response response = closed.step_response(duration, response_samples);
            const double final_value =
                stable ? closed.dc_gain() : (response.y.empty() ? 0.0 : response.y.back());
            const control::TransientSpec spec =
                control::transient_spec(response, 1.0, final_value, 0.02);

            const double gm = m.has_gain_margin ? m.gain_margin_db : 1000.0;
            const double pm = m.has_phase_margin ? m.phase_margin_deg : 180.0;

            payloads.push_back(payload);
            gains.push_back(gm);
            phases.push_back(pm);
            overshoots.push_back(spec.overshoot_percent);
            settlings.push_back(spec.settling_time);

            if (gm < worst_gm) {
                worst_gm = gm;
                worst_gm_payload = payload;
            }
            if (pm < worst_pm) {
                worst_pm = pm;
                worst_pm_payload = payload;
            }
            if (spec.overshoot_percent > worst_overshoot) {
                worst_overshoot = spec.overshoot_percent;
                worst_overshoot_payload = payload;
            }
            if (spec.settling_time > worst_settling) {
                worst_settling = spec.settling_time;
                worst_settling_payload = payload;
            }
            if (gm < required_gm || pm < required_pm || !stable) {
                specification_met = false;
            }
            if (variant.scale == 1.0 && i == 0) {
                overshoot_light = spec.overshoot_percent;
                inertia_light = inertia;
                pm_light = pm;
            }
            if (variant.scale == 1.0 && i + 1 == points) {
                overshoot_heavy = spec.overshoot_percent;
                inertia_heavy = inertia;
                pm_heavy = pm;
            }

            sweep.push_back({json::Value(variant.name), json::Value(payload),
                             json::Value(inertia), json::Value(gm), json::Value(pm),
                             json::Value(spec.overshoot_percent),
                             json::Value(spec.settling_time), json::Value(stable)});
        }

        margin_set.push_back(json::from_series("gain margin [dB], " + variant.name, payloads,
                                               gains));
        margin_set.push_back(json::from_series("phase margin [deg], " + variant.name, payloads,
                                               phases));
        quality_set.push_back(json::from_series("overshoot [%], " + variant.name, payloads,
                                                overshoots));
        quality_set.push_back(json::from_series("settling time [s], " + variant.name, payloads,
                                                settlings));
    }

    const double relative_inertia =
        (inertia_light > 0.0) ? (inertia_heavy - inertia_light) / inertia_light : 0.0;
    const double relative_overshoot =
        (overshoot_light > 1e-9) ? (overshoot_heavy - overshoot_light) / overshoot_light : 0.0;
    const double sensitivity =
        (std::abs(relative_inertia) > 1e-9) ? relative_overshoot / relative_inertia : 0.0;
    const double payload_span = payload_max - payload_min;
    const double margin_sensitivity =
        (payload_span > 1e-9) ? (pm_light - pm_heavy) / payload_span : 0.0;

    std::vector<std::vector<json::Value>> worst;
    worst.push_back({json::Value("lowest gain margin [dB]"), json::Value(worst_gm),
                     json::Value(worst_gm_payload)});
    worst.push_back({json::Value("lowest phase margin [deg]"), json::Value(worst_pm),
                     json::Value(worst_pm_payload)});
    worst.push_back({json::Value("largest overshoot [%]"), json::Value(worst_overshoot),
                     json::Value(worst_overshoot_payload)});
    worst.push_back({json::Value("longest settling time [s]"), json::Value(worst_settling),
                     json::Value(worst_settling_payload)});

    json::Value out = json::Value::object();
    out.set("margins", std::move(margin_set));
    out.set("quality", std::move(quality_set));
    out.set("sweep", json::from_table({"variant", "payload [kg]", "J [kg m^2]", "GM [dB]",
                                       "PM [deg]", "overshoot [%]", "settling [s]", "stable"},
                                      sweep));
    out.set("worst_case", json::from_table({"indicator", "worst value", "payload [kg]"}, worst));
    out.set("inertia_sensitivity", json::Value(sensitivity));
    out.set("margin_sensitivity", json::Value(margin_sensitivity));
    out.set("specification_met", json::Value(specification_met));
    out.set("worst_payload", json::Value(worst_pm_payload));
    out.set("note",
            json::Value(specification_met
                            ? "both required margins hold at every payload and every inertia "
                              "variant in the sweep, so this tuning is certified across the range "
                              "rather than at one operating point"
                            : "at least one sweep point falls below the required margins: the "
                              "worst-case table says where, and that payload - not the empty "
                              "robot - is the one the gains have to be designed for"));
    return out;
}

}  // namespace

json::Value ControlSystemModule::invoke(std::string_view op, const json::Value& args) const {
    if (op == "joint_plant") {
        return op_joint_plant(args);
    }
    if (op == "block_diagram") {
        return op_block_diagram(args);
    }
    if (op == "standard_links") {
        return op_standard_links(args);
    }
    if (op == "time_response") {
        return op_time_response(args);
    }
    if (op == "frequency_response") {
        return op_frequency_response(args);
    }
    if (op == "stability") {
        return op_stability(args);
    }
    if (op == "root_locus") {
        return op_root_locus(args);
    }
    if (op == "steady_state_accuracy") {
        return op_steady_state_accuracy(args);
    }
    if (op == "pid") {
        return op_pid(args);
    }
    if (op == "pid_tune") {
        return op_pid_tune(args);
    }
    if (op == "advanced_structures") {
        return op_advanced_structures(args);
    }
    if (op == "robustness") {
        return op_robustness(args);
    }
    unknown_op(name(), op);
}

}  // namespace yaskawa::study
