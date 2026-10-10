#include "study/dsp_system.hpp"

#include "study/gp8_model.hpp"

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <numbers>
#include <random>
#include <string>
#include <utility>
#include <vector>

namespace yaskawa::study {

namespace dsp {

namespace {

constexpr double kPi = std::numbers::pi;
constexpr double kTiny = 1e-300;

// Radix-2 decimation in time, in place. The twiddle factor is recomputed from
// its angle rather than accumulated, so a 4096-point transform still agrees
// with the direct DFT at 1e-13 instead of drifting.
void fft_radix2(std::vector<Complex>& a, bool inverse) {
    const std::size_t n = a.size();
    if (n <= 1) {
        return;
    }
    for (std::size_t i = 1, j = 0; i < n; ++i) {
        std::size_t bit = n >> 1;
        for (; (j & bit) != 0; bit >>= 1) {
            j ^= bit;
        }
        j ^= bit;
        if (i < j) {
            std::swap(a[i], a[j]);
        }
    }
    const double sign = inverse ? 1.0 : -1.0;
    for (std::size_t len = 2; len <= n; len <<= 1) {
        const std::size_t half = len >> 1;
        const double step = sign * 2.0 * kPi / static_cast<double>(len);
        for (std::size_t i = 0; i < n; i += len) {
            for (std::size_t k = 0; k < half; ++k) {
                const double angle = step * static_cast<double>(k);
                const Complex w(std::cos(angle), std::sin(angle));
                const Complex u = a[i + k];
                const Complex v = a[i + k + half] * w;
                a[i + k] = u + v;
                a[i + k + half] = u - v;
            }
        }
    }
    if (inverse) {
        for (auto& z : a) {
            z /= static_cast<double>(n);
        }
    }
}

// Bluestein's chirp-z transform: the exact N-point DFT for any N, obtained by
// writing j k = (j^2 + k^2 - (k - j)^2) / 2 and running the remaining
// convolution on a power-of-two radix-2 transform.
[[nodiscard]] std::vector<Complex> fft_bluestein(const std::vector<Complex>& x, bool inverse) {
    const std::size_t n = x.size();
    const std::size_t m = next_power_of_two(2 * n - 1);
    const double sign = inverse ? 1.0 : -1.0;
    const double two_n = 2.0 * static_cast<double>(n);

    std::vector<Complex> a(m, Complex(0.0, 0.0));
    std::vector<Complex> b(m, Complex(0.0, 0.0));
    for (std::size_t k = 0; k < n; ++k) {
        const double kk = std::fmod(static_cast<double>(k) * static_cast<double>(k), two_n);
        const double angle = sign * kPi * kk / static_cast<double>(n);
        const Complex chirp(std::cos(angle), std::sin(angle));
        a[k] = x[k] * chirp;
        b[k] = std::conj(chirp);
        if (k != 0) {
            b[m - k] = b[k];
        }
    }
    fft_radix2(a, false);
    fft_radix2(b, false);
    for (std::size_t i = 0; i < m; ++i) {
        a[i] *= b[i];
    }
    fft_radix2(a, true);

    std::vector<Complex> out(n);
    for (std::size_t k = 0; k < n; ++k) {
        const double kk = std::fmod(static_cast<double>(k) * static_cast<double>(k), two_n);
        const double angle = sign * kPi * kk / static_cast<double>(n);
        out[k] = a[k] * Complex(std::cos(angle), std::sin(angle));
        if (inverse) {
            out[k] /= static_cast<double>(n);
        }
    }
    return out;
}

}  // namespace

bool is_power_of_two(std::size_t n) noexcept { return n != 0 && (n & (n - 1)) == 0; }

std::size_t next_power_of_two(std::size_t n) noexcept {
    std::size_t p = 1;
    while (p < n) {
        p <<= 1;
    }
    return p;
}

std::vector<Complex> dft(const std::vector<Complex>& x, bool inverse) {
    const std::size_t n = x.size();
    std::vector<Complex> out(n, Complex(0.0, 0.0));
    if (n == 0) {
        return out;
    }
    const double sign = inverse ? 1.0 : -1.0;
    for (std::size_t k = 0; k < n; ++k) {
        Complex sum(0.0, 0.0);
        for (std::size_t j = 0; j < n; ++j) {
            const std::size_t jk = (j * k) % n;
            const double angle =
                sign * 2.0 * kPi * static_cast<double>(jk) / static_cast<double>(n);
            sum += x[j] * Complex(std::cos(angle), std::sin(angle));
        }
        out[k] = inverse ? sum / static_cast<double>(n) : sum;
    }
    return out;
}

std::vector<Complex> dft_real(const std::vector<double>& x) {
    std::vector<Complex> input(x.size());
    for (std::size_t i = 0; i < x.size(); ++i) {
        input[i] = Complex(x[i], 0.0);
    }
    return dft(input, false);
}

std::vector<Complex> fft(const std::vector<Complex>& x, bool inverse) {
    const std::size_t n = x.size();
    if (n <= 1) {
        return x;
    }
    if (is_power_of_two(n)) {
        std::vector<Complex> work = x;
        fft_radix2(work, inverse);
        return work;
    }
    return fft_bluestein(x, inverse);
}

std::vector<Complex> fft_real(const std::vector<double>& x) {
    std::vector<Complex> input(x.size());
    for (std::size_t i = 0; i < x.size(); ++i) {
        input[i] = Complex(x[i], 0.0);
    }
    return fft(input, false);
}

std::vector<double> ifft_real(const std::vector<Complex>& x) {
    const std::vector<Complex> z = fft(x, true);
    std::vector<double> out(z.size());
    for (std::size_t i = 0; i < z.size(); ++i) {
        out[i] = z[i].real();
    }
    return out;
}

std::vector<double> convolve(const std::vector<double>& a, const std::vector<double>& b) {
    if (a.empty() || b.empty()) {
        return {};
    }
    std::vector<double> out(a.size() + b.size() - 1, 0.0);
    for (std::size_t i = 0; i < a.size(); ++i) {
        for (std::size_t j = 0; j < b.size(); ++j) {
            out[i + j] += a[i] * b[j];
        }
    }
    return out;
}

std::vector<double> convolve_fft(const std::vector<double>& a, const std::vector<double>& b) {
    if (a.empty() || b.empty()) {
        return {};
    }
    const std::size_t length = a.size() + b.size() - 1;
    const std::size_t m = next_power_of_two(length);
    std::vector<Complex> fa(m, Complex(0.0, 0.0));
    std::vector<Complex> fb(m, Complex(0.0, 0.0));
    for (std::size_t i = 0; i < a.size(); ++i) {
        fa[i] = Complex(a[i], 0.0);
    }
    for (std::size_t i = 0; i < b.size(); ++i) {
        fb[i] = Complex(b[i], 0.0);
    }
    fa = fft(fa, false);
    fb = fft(fb, false);
    for (std::size_t i = 0; i < m; ++i) {
        fa[i] *= fb[i];
    }
    const std::vector<double> full = ifft_real(fa);
    return std::vector<double>(full.begin(), full.begin() + static_cast<std::ptrdiff_t>(length));
}

CorrelationResult cross_correlate(const std::vector<double>& x, const std::vector<double>& y,
                                  int max_lag) {
    CorrelationResult result;
    if (x.empty() || y.empty()) {
        return result;
    }
    const int nx = static_cast<int>(x.size());
    const int ny = static_cast<int>(y.size());
    const int limit = (max_lag < 0) ? std::max(nx, ny) - 1 : max_lag;
    result.lag_min = -limit;
    result.values.assign(static_cast<std::size_t>(2 * limit + 1), 0.0);

    double best = -1.0;
    for (int lag = -limit; lag <= limit; ++lag) {
        double sum = 0.0;
        const int from = std::max(0, -lag);
        const int to = std::min(nx, ny - lag);
        for (int n = from; n < to; ++n) {
            sum += x[static_cast<std::size_t>(n)] * y[static_cast<std::size_t>(n + lag)];
        }
        result.values[static_cast<std::size_t>(lag + limit)] = sum;
        if (std::abs(sum) > best) {
            best = std::abs(sum);
            result.peak_lag = lag;
            result.peak_value = sum;
        }
    }
    return result;
}

CorrelationResult auto_correlate(const std::vector<double>& x, int max_lag) {
    return cross_correlate(x, x, max_lag);
}

Complex frequency_response(const std::vector<double>& b, const std::vector<double>& a,
                           double omega) {
    Complex num(0.0, 0.0);
    for (std::size_t k = 0; k < b.size(); ++k) {
        const double angle = -omega * static_cast<double>(k);
        num += b[k] * Complex(std::cos(angle), std::sin(angle));
    }
    Complex den(1.0, 0.0);
    if (!a.empty()) {
        den = Complex(0.0, 0.0);
        for (std::size_t k = 0; k < a.size(); ++k) {
            const double angle = -omega * static_cast<double>(k);
            den += a[k] * Complex(std::cos(angle), std::sin(angle));
        }
    }
    if (std::abs(den) < kTiny) {
        return Complex(0.0, 0.0);
    }
    return num / den;
}

namespace {

// Re{ sum k c_k e^{-j w k} / sum c_k e^{-j w k} }, i.e. the phase slope of one
// polynomial. Zero where the polynomial itself vanishes, which is exactly
// where the group delay of the whole filter is undefined anyway.
[[nodiscard]] double phase_slope(const std::vector<double>& c, double omega) {
    if (c.empty()) {
        return 0.0;
    }
    Complex value(0.0, 0.0);
    Complex weighted(0.0, 0.0);
    for (std::size_t k = 0; k < c.size(); ++k) {
        const double angle = -omega * static_cast<double>(k);
        const Complex term = c[k] * Complex(std::cos(angle), std::sin(angle));
        value += term;
        weighted += static_cast<double>(k) * term;
    }
    if (std::abs(value) < 1e-14) {
        return 0.0;
    }
    return (weighted / value).real();
}

}  // namespace

double group_delay(const std::vector<double>& b, const std::vector<double>& a, double omega) {
    return phase_slope(b, omega) - phase_slope(a, omega);
}

std::vector<double> apply_filter(const std::vector<double>& b, const std::vector<double>& a,
                                 const std::vector<double>& x) {
    if (a.empty() || std::abs(a[0]) < kTiny) {
        throw StudyError("a denominator with a[0] = 0 is not a realisable filter");
    }
    std::vector<double> y(x.size(), 0.0);
    for (std::size_t n = 0; n < x.size(); ++n) {
        double acc = 0.0;
        for (std::size_t k = 0; k < b.size(); ++k) {
            if (k <= n) {
                acc += b[k] * x[n - k];
            }
        }
        for (std::size_t k = 1; k < a.size(); ++k) {
            if (k <= n) {
                acc -= a[k] * y[n - k];
            }
        }
        y[n] = acc / a[0];
    }
    return y;
}

std::vector<double> impulse_response(const std::vector<double>& b, const std::vector<double>& a,
                                     std::size_t n) {
    std::vector<double> delta(n, 0.0);
    if (n != 0) {
        delta[0] = 1.0;
    }
    return apply_filter(b, a, delta);
}

std::vector<Complex> polynomial_roots(const std::vector<double>& coefficients) {
    std::vector<double> c = coefficients;
    std::size_t lead = 0;
    while (lead < c.size() && std::abs(c[lead]) < 1e-15) {
        ++lead;
    }
    c.erase(c.begin(), c.begin() + static_cast<std::ptrdiff_t>(lead));
    if (c.size() <= 1) {
        return {};
    }
    std::size_t at_origin = 0;
    while (c.size() > 1 && std::abs(c.back()) < 1e-15) {
        c.pop_back();
        ++at_origin;
    }

    std::vector<Complex> roots;
    if (c.size() > 1) {
        const Eigen::Index n = static_cast<Eigen::Index>(c.size() - 1);
        Eigen::MatrixXd companion = Eigen::MatrixXd::Zero(n, n);
        for (Eigen::Index j = 0; j < n; ++j) {
            companion(0, j) = -c[static_cast<std::size_t>(j) + 1] / c[0];
        }
        for (Eigen::Index i = 1; i < n; ++i) {
            companion(i, i - 1) = 1.0;
        }
        const Eigen::EigenSolver<Eigen::MatrixXd> solver(companion);
        const auto values = solver.eigenvalues();
        roots.reserve(static_cast<std::size_t>(n) + at_origin);
        for (Eigen::Index i = 0; i < values.size(); ++i) {
            roots.emplace_back(values(i));
        }
    }
    for (std::size_t i = 0; i < at_origin; ++i) {
        roots.emplace_back(0.0, 0.0);
    }
    return roots;
}

Eigen::VectorXd least_squares(const Eigen::MatrixXd& A, const Eigen::VectorXd& y) {
    if (A.rows() != y.size()) {
        throw StudyError("least squares needs as many observations as rows");
    }
    if (A.rows() == 0 || A.cols() == 0) {
        return Eigen::VectorXd::Zero(A.cols());
    }
    // Rank-revealing QR for the usual over-determined design, and a Jacobi SVD
    // when the problem is rank deficient or under-determined so the answer is
    // the minimum-norm one instead of an arbitrary member of the solution set.
    // (Eigen 3.4.0's BDCSVD is deliberately not used here: it has been observed
    // to return a different answer for the same matrix in different builds.)
    const Eigen::ColPivHouseholderQR<Eigen::MatrixXd> qr(A);
    if (A.rows() >= A.cols() && qr.rank() == A.cols()) {
        return qr.solve(y);
    }
    Eigen::JacobiSVD<Eigen::MatrixXd> svd(A, Eigen::ComputeThinU | Eigen::ComputeThinV);
    svd.setThreshold(1e-12);
    return svd.solve(y);
}

PronyFit prony(const std::vector<double>& h, std::size_t numerator_order,
               std::size_t denominator_order) {
    const std::size_t q = numerator_order;
    const std::size_t p = denominator_order;
    if (h.size() < q + p + 2) {
        throw StudyError("Prony needs at least " + std::to_string(q + p + 2) +
                         " impulse-response samples for orders (" + std::to_string(q) + ", " +
                         std::to_string(p) + "), received " + std::to_string(h.size()));
    }

    // The impulse response is causal: h[n] = 0 for n < 0. Saying so explicitly
    // matters, because the first few equations of the recursion reach back past
    // the start of the record (n - k < 0 when q + 1 < p) and an unsigned index
    // would wrap instead.
    const auto sample = [&h](std::ptrdiff_t index) {
        if (index < 0 || index >= static_cast<std::ptrdiff_t>(h.size())) {
            return 0.0;
        }
        return h[static_cast<std::size_t>(index)];
    };

    PronyFit fit;
    fit.a.assign(p + 1, 0.0);
    fit.a[0] = 1.0;
    if (p != 0) {
        const std::size_t rows = h.size() - q - 1;
        Eigen::MatrixXd A(static_cast<Eigen::Index>(rows), static_cast<Eigen::Index>(p));
        Eigen::VectorXd rhs(static_cast<Eigen::Index>(rows));
        for (std::size_t i = 0; i < rows; ++i) {
            const std::ptrdiff_t n = static_cast<std::ptrdiff_t>(q + 1 + i);
            for (std::size_t k = 1; k <= p; ++k) {
                A(static_cast<Eigen::Index>(i), static_cast<Eigen::Index>(k - 1)) =
                    sample(n - static_cast<std::ptrdiff_t>(k));
            }
            rhs(static_cast<Eigen::Index>(i)) = -sample(n);
        }
        const Eigen::VectorXd sol = least_squares(A, rhs);
        for (std::size_t k = 0; k < p; ++k) {
            fit.a[k + 1] = sol(static_cast<Eigen::Index>(k));
        }
    }

    fit.b.assign(q + 1, 0.0);
    for (std::size_t n = 0; n <= q; ++n) {
        double sum = 0.0;
        const std::size_t kmax = std::min(n, p);
        for (std::size_t k = 0; k <= kmax; ++k) {
            sum += fit.a[k] * sample(static_cast<std::ptrdiff_t>(n) - static_cast<std::ptrdiff_t>(k));
        }
        fit.b[n] = sum;
    }

    const std::vector<double> fitted = impulse_response(fit.b, fit.a, h.size());
    double num = 0.0;
    double den = 0.0;
    for (std::size_t i = 0; i < h.size(); ++i) {
        const double e = fitted[i] - h[i];
        num += e * e;
        den += h[i] * h[i];
    }
    fit.relative_error = (den > 0.0) ? std::sqrt(num / den) : std::sqrt(num);

    const std::vector<Complex> poles = polynomial_roots(fit.a);
    for (const auto& pole : poles) {
        fit.max_pole_radius = std::max(fit.max_pole_radius, std::abs(pole));
    }
    fit.stable = fit.max_pole_radius < 1.0;
    return fit;
}

std::vector<double> hilbert_fir_kernel(std::size_t length) {
    if (length < 3 || length % 2 == 0) {
        throw StudyError("the Hilbert FIR length must be odd and at least 3, received " +
                         std::to_string(length));
    }
    const std::ptrdiff_t m = static_cast<std::ptrdiff_t>(length / 2);
    std::vector<double> h(length, 0.0);
    for (std::ptrdiff_t i = 0; i < static_cast<std::ptrdiff_t>(length); ++i) {
        const std::ptrdiff_t n = i - m;
        double value = 0.0;
        if (n != 0) {
            const double s = std::sin(kPi * static_cast<double>(n) / 2.0);
            value = 2.0 * s * s / (kPi * static_cast<double>(n));
        }
        const double window =
            0.54 - 0.46 * std::cos(2.0 * kPi * static_cast<double>(i) /
                                   static_cast<double>(length - 1));
        h[static_cast<std::size_t>(i)] = value * window;
    }
    return h;
}

std::vector<double> hilbert_fft(const std::vector<double>& x) {
    const std::size_t n = x.size();
    if (n < 4) {
        throw StudyError("the analytic signal needs at least 4 samples, received " +
                         std::to_string(n));
    }
    std::vector<Complex> spectrum = fft_real(x);
    const std::size_t half = n / 2;
    for (std::size_t k = 1; k < n; ++k) {
        if (k < half) {
            spectrum[k] *= 2.0;
        } else if (k == half && n % 2 == 0) {
            // the Nyquist bin stays as it is
        } else if (k > half || (k == half && n % 2 == 1)) {
            spectrum[k] = Complex(0.0, 0.0);
        }
    }
    const std::vector<Complex> analytic = fft(spectrum, true);
    std::vector<double> quadrature(n, 0.0);
    for (std::size_t i = 0; i < n; ++i) {
        quadrature[i] = analytic[i].imag();
    }
    return quadrature;
}

JointChannel joint_channel(std::size_t joint, double fs, double damping_ratio) {
    if (joint < 1 || joint > GP8_DOF) {
        throw StudyError("parameter 'joint' must be 1..6 (S, L, U, R, B, T), received " +
                         std::to_string(joint));
    }
    if (!(fs > 1.0) || !std::isfinite(fs)) {
        throw StudyError("parameter 'fs' must be greater than 1 Hz");
    }
    if (!(damping_ratio > 0.0) || damping_ratio >= 1.0) {
        throw StudyError("parameter 'damping_ratio' must lie in (0, 1)");
    }

    const std::size_t index = joint - 1;
    const auto& link = GP8_LINKS[index];
    const double structure =
        link_inertia(index)(2, 2) + link.mass * link_com(index).squaredNorm();
    const double joint_side = reflected_rotor_inertia(index) + structure;
    const double tau = joint_side / link.viscous_friction;
    const double physical_hz = std::sqrt(kGp8JointStiffnessNmPerRad / structure) / (2.0 * kPi);
    const double used_hz = std::min(physical_hz, 0.45 * fs);

    // The channel a student identifies is the CLOSED-LOOP command-to-measured
    // velocity response, not the bare motor: the open-loop mechanical time
    // constant J/b is several seconds and would make every channel an
    // integrator. A velocity loop cannot be closed much faster than a third of
    // the first structural mode without exciting it, which is the standard
    // rule of thumb and fixes the dominant pole from the same two numbers.
    const double servo_bandwidth = used_hz / 3.0;

    JointChannel channel;
    channel.fs = fs;
    channel.mechanical_time_constant = tau;
    channel.servo_bandwidth_hz = servo_bandwidth;
    channel.resonance_hz_physical = physical_hz;
    channel.resonance_hz = used_hz;
    channel.resonance_clamped = used_hz < physical_hz;
    channel.damping_ratio = damping_ratio;
    channel.joint_inertia = joint_side;
    channel.link_inertia = structure;

    // The drive pole from the mechanical time constant, in series with the
    // lightly damped structural mode of the link. Both poles are placed
    // directly in z, so the model is exactly the system the ops identify.
    const double drive_pole = std::exp(-2.0 * kPi * servo_bandwidth / fs);
    const double wn = 2.0 * kPi * used_hz;
    const double radius = std::exp(-damping_ratio * wn / fs);
    const double theta = wn * std::sqrt(1.0 - damping_ratio * damping_ratio) / fs;

    const std::vector<double> drive = {1.0, -drive_pole};
    const std::vector<double> mode = {1.0, -2.0 * radius * std::cos(theta), radius * radius};
    channel.tf.a = convolve(drive, mode);

    double dc = 0.0;
    for (const double coefficient : channel.tf.a) {
        dc += coefficient;
    }
    channel.tf.b = {dc};  // unity gain at DC, so the student reads 0 dB there
    return channel;
}

}  // namespace dsp

namespace {

using dsp::Complex;

constexpr double kPi = std::numbers::pi;
constexpr int kMinLength = 16;
constexpr int kMaxLength = 4096;
constexpr int kMaxOrder = 256;
constexpr std::size_t kResponsePoints = 257;

const std::vector<std::string> kSignalOptions = {"step",        "impulse", "chirp",
                                                 "white_noise", "prbs",    "multisine"};
const std::vector<std::string> kWindowOptions = {"rectangular", "hann", "hamming", "blackman",
                                                 "kaiser"};
const std::vector<std::string> kIdentificationMethods = {"generalised_correlation",
                                                         "least_squares_arx", "spectral_division"};
const std::vector<std::string> kFirMethods = {"frequency_sampling", "least_squares", "windowed"};
const std::vector<std::string> kFirSpecifications = {"servo_lowpass", "resonance_notch",
                                                     "vibration_bandpass"};
const std::vector<std::string> kPhasePresets = {"linear_phase", "minimum_phase", "maximum_phase",
                                                "mixed_phase"};
const std::vector<std::string> kEqualiserTypes = {"peaking", "low_shelf", "high_shelf", "notch"};
const std::vector<std::string> kStructures = {"direct_form_1", "direct_form_2",
                                              "transposed_direct_form_2", "cascade"};
const std::vector<std::string> kHilbertMethods = {"fir_kernel", "fft_analytic"};
const std::vector<std::string> kVibrationSignals = {"ringdown", "swept_motion", "bearing_fault"};
const std::vector<std::string> kSynthesisSpecs = {"antialias_exam", "bandpass_part_a",
                                                  "servo_antivibration"};
const std::vector<std::string> kSynthesisFamilies = {"fir_windowed", "fir_least_squares",
                                                     "iir_prony"};

// ---------------------------------------------------------------------------
// Parameter readers
// ---------------------------------------------------------------------------

[[nodiscard]] std::size_t read_length(const json::Value& args, int fallback) {
    return static_cast<std::size_t>(optional_int(args, "length", fallback, kMinLength, kMaxLength));
}

[[nodiscard]] double read_fs(const json::Value& args, double fallback = dsp::kGp8ControlRateHz) {
    return optional_scalar(args, "fs", fallback, 50.0, 48000.0);
}

[[nodiscard]] std::size_t read_joint(const json::Value& args, int fallback = 2) {
    return static_cast<std::size_t>(optional_int(args, "joint", fallback, 1, 6));
}

[[nodiscard]] unsigned read_seed(const json::Value& args) {
    return static_cast<unsigned>(optional_int(args, "seed", 20261010, 1, 2000000000));
}

// A signal or a coefficient set. The UI sends a flat array and the `matrix`
// vocabulary type sends a single row; both are accepted, nothing else is.
[[nodiscard]] std::vector<double> read_numeric_array(const json::Value& value, std::string_view key,
                                                     std::size_t min_n, std::size_t max_n) {
    const json::Value* row = &value;
    if (value.is_array() && value.size() == 1 && value[0].is_array()) {
        row = &value[0];
    }
    if (!row->is_array()) {
        throw StudyError("parameter '" + std::string(key) + "' must be an array of numbers, found " +
                         row->type_name());
    }
    if (row->size() < min_n || row->size() > max_n) {
        throw StudyError("parameter '" + std::string(key) + "' must hold between " +
                         std::to_string(min_n) + " and " + std::to_string(max_n) +
                         " numbers, received " + std::to_string(row->size()));
    }
    std::vector<double> out;
    out.reserve(row->size());
    for (std::size_t i = 0; i < row->size(); ++i) {
        const json::Value& element = (*row)[i];
        if (!element.is_number() || !std::isfinite(element.as_double())) {
            throw StudyError("parameter '" + std::string(key) + "' element " + std::to_string(i) +
                             " must be a finite number");
        }
        out.push_back(element.as_double());
    }
    return out;
}

[[nodiscard]] json::Value coefficient_matrix(const std::vector<double>& c) {
    json::Value row = json::Value::array();
    row.reserve(c.size());
    for (const double v : c) {
        row.push_back(json::Value(v));
    }
    json::Value out = json::Value::array();
    out.push_back(std::move(row));
    return out;
}

[[nodiscard]] json::Value empty_coefficient_default() {
    return json::Value::array({json::Value::array({json::Value(1.0)})});
}

// ---------------------------------------------------------------------------
// Windows, noise, metrics
// ---------------------------------------------------------------------------

[[nodiscard]] double bessel_i0(double x) noexcept {
    double sum = 1.0;
    double term = 1.0;
    for (int k = 1; k < 40; ++k) {
        term *= (x * x) / (4.0 * static_cast<double>(k) * static_cast<double>(k));
        sum += term;
        if (term < 1e-18 * sum) {
            break;
        }
    }
    return sum;
}

[[nodiscard]] std::vector<double> make_window(const std::string& type, std::size_t n, double beta) {
    std::vector<double> w(n, 1.0);
    if (n <= 1) {
        return w;
    }
    const double last = static_cast<double>(n - 1);
    for (std::size_t i = 0; i < n; ++i) {
        const double t = static_cast<double>(i) / last;
        if (type == "rectangular") {
            w[i] = 1.0;
        } else if (type == "hann") {
            w[i] = 0.5 - 0.5 * std::cos(2.0 * kPi * t);
        } else if (type == "hamming") {
            w[i] = 0.54 - 0.46 * std::cos(2.0 * kPi * t);
        } else if (type == "blackman") {
            w[i] = 0.42 - 0.5 * std::cos(2.0 * kPi * t) + 0.08 * std::cos(4.0 * kPi * t);
        } else {
            const double r = 2.0 * t - 1.0;
            w[i] = bessel_i0(beta * std::sqrt(std::max(0.0, 1.0 - r * r))) / bessel_i0(beta);
        }
    }
    return w;
}

[[nodiscard]] double rms(const std::vector<double>& x) {
    if (x.empty()) {
        return 0.0;
    }
    double sum = 0.0;
    for (const double v : x) {
        sum += v * v;
    }
    return std::sqrt(sum / static_cast<double>(x.size()));
}

[[nodiscard]] double max_abs(const std::vector<double>& x) {
    double best = 0.0;
    for (const double v : x) {
        best = std::max(best, std::abs(v));
    }
    return best;
}

[[nodiscard]] std::vector<double> add_noise(const std::vector<double>& x, double snr_db,
                                            unsigned seed) {
    if (snr_db >= 199.0) {
        return x;
    }
    const double signal_power = rms(x) * rms(x);
    const double sigma = std::sqrt(std::max(signal_power, 1e-30) / std::pow(10.0, snr_db / 10.0));
    std::mt19937 rng(seed);
    std::normal_distribution<double> noise(0.0, sigma);
    std::vector<double> out = x;
    for (auto& v : out) {
        v += noise(rng);
    }
    return out;
}

// The same absolute sensor noise for every record. Comparing excitations is
// only fair this way: the noise floor of an encoder does not shrink because
// the test signal was quieter.
[[nodiscard]] std::vector<double> add_absolute_noise(const std::vector<double>& x, double sigma,
                                                     unsigned seed) {
    if (!(sigma > 0.0)) {
        return x;
    }
    std::mt19937 rng(seed);
    std::normal_distribution<double> noise(0.0, sigma);
    std::vector<double> out = x;
    for (auto& v : out) {
        v += noise(rng);
    }
    return out;
}

// Maximal-length sequence from a 15-bit LFSR (taps 15 and 14): period 32767,
// so every record shorter than that is a genuine m-sequence segment.
[[nodiscard]] std::vector<double> prbs_sequence(std::size_t n, double amplitude, unsigned seed) {
    unsigned state = (seed & 0x7FFFu) | 1u;
    std::vector<double> out;
    out.reserve(n);
    for (std::size_t i = 0; i < n; ++i) {
        const unsigned bit = ((state >> 14) ^ (state >> 13)) & 1u;
        state = ((state << 1) | bit) & 0x7FFFu;
        out.push_back(bit != 0 ? amplitude : -amplitude);
    }
    return out;
}

[[nodiscard]] std::vector<double> generate_test_signal(const std::string& kind, std::size_t n,
                                                       double fs, double amplitude, unsigned seed) {
    std::vector<double> x(n, 0.0);
    const double dt = 1.0 / fs;
    if (kind == "step") {
        for (std::size_t i = n / 8; i < n; ++i) {
            x[i] = amplitude;
        }
    } else if (kind == "impulse") {
        x[n / 8] = amplitude;
    } else if (kind == "chirp") {
        const double f0 = 0.5;
        const double f1 = 0.45 * fs;
        const double duration = static_cast<double>(n) * dt;
        const double rate = (f1 - f0) / duration;
        for (std::size_t i = 0; i < n; ++i) {
            const double t = static_cast<double>(i) * dt;
            x[i] = amplitude * std::sin(2.0 * kPi * (f0 * t + 0.5 * rate * t * t));
        }
    } else if (kind == "white_noise") {
        std::mt19937 rng(seed);
        std::normal_distribution<double> dist(0.0, amplitude);
        for (std::size_t i = 0; i < n; ++i) {
            x[i] = dist(rng);
        }
        // Normalised to the same PEAK as the others: the robot is limited by
        // the velocity it may reach, not by the variance of the command.
        const double peak = max_abs(x);
        if (peak > 0.0) {
            for (auto& v : x) {
                v *= amplitude / peak;
            }
        }
    } else if (kind == "prbs") {
        x = prbs_sequence(n, amplitude, seed);
    } else {
        // Multisine with Schroeder phases: every harmonic of the servo band at
        // the same amplitude and the lowest achievable crest factor.
        const std::size_t harmonics = 24;
        for (std::size_t k = 1; k <= harmonics; ++k) {
            const double phase = -kPi * static_cast<double>(k) * static_cast<double>(k + 1) /
                                 static_cast<double>(harmonics);
            const double f = 0.45 * fs * static_cast<double>(k) / static_cast<double>(harmonics);
            for (std::size_t i = 0; i < n; ++i) {
                x[i] += std::sin(2.0 * kPi * f * static_cast<double>(i) * dt + phase);
            }
        }
        const double peak = max_abs(x);
        if (peak > 0.0) {
            for (auto& v : x) {
                v *= amplitude / peak;
            }
        }
    }
    return x;
}

// One-sided power spectrum, length n/2 + 1.
[[nodiscard]] std::vector<double> power_spectrum(const std::vector<double>& x) {
    const std::vector<Complex> X = dsp::fft_real(x);
    const std::size_t bins = x.size() / 2 + 1;
    std::vector<double> power(bins, 0.0);
    const double scale = 1.0 / static_cast<double>(std::max<std::size_t>(1, x.size()));
    for (std::size_t k = 0; k < bins; ++k) {
        power[k] = std::norm(X[k]) * scale;
    }
    return power;
}

// Geometric mean over arithmetic mean of the power spectrum: 1 for white, near
// 0 for a single tone. This is what makes a test signal a good identifier.
[[nodiscard]] double spectral_flatness(const std::vector<double>& x) {
    const std::vector<double> power = power_spectrum(x);
    if (power.size() < 3) {
        return 0.0;
    }
    double log_sum = 0.0;
    double sum = 0.0;
    std::size_t count = 0;
    for (std::size_t k = 1; k + 1 < power.size(); ++k) {
        const double p = std::max(power[k], 1e-20);
        log_sum += std::log(p);
        sum += p;
        ++count;
    }
    if (count == 0 || sum <= 0.0) {
        return 0.0;
    }
    return std::exp(log_sum / static_cast<double>(count)) / (sum / static_cast<double>(count));
}

[[nodiscard]] double crest_factor(const std::vector<double>& x) {
    const double r = rms(x);
    return (r > 0.0) ? max_abs(x) / r : 0.0;
}

// Fraction of the band whose excitation sits within `window_db` of the peak bin.
[[nodiscard]] double excited_fraction(const std::vector<double>& x, double window_db) {
    const std::vector<double> power = power_spectrum(x);
    double peak = 0.0;
    for (std::size_t k = 1; k + 1 < power.size(); ++k) {
        peak = std::max(peak, power[k]);
    }
    if (peak <= 0.0) {
        return 0.0;
    }
    const double floor_value = peak * std::pow(10.0, -window_db / 10.0);
    std::size_t inside = 0;
    std::size_t total = 0;
    for (std::size_t k = 1; k + 1 < power.size(); ++k) {
        ++total;
        if (power[k] >= floor_value) {
            ++inside;
        }
    }
    return (total == 0) ? 0.0 : static_cast<double>(inside) / static_cast<double>(total);
}

// ---------------------------------------------------------------------------
// Frequency-response helpers
// ---------------------------------------------------------------------------

struct Grid {
    std::vector<double> hz;
    std::vector<double> omega;
};

[[nodiscard]] Grid make_grid(double fs, std::size_t points) {
    Grid g;
    g.hz.reserve(points);
    g.omega.reserve(points);
    for (std::size_t i = 0; i < points; ++i) {
        const double f = 0.5 * fs * static_cast<double>(i) / static_cast<double>(points - 1);
        g.hz.push_back(f);
        g.omega.push_back(2.0 * kPi * f / fs);
    }
    return g;
}

[[nodiscard]] std::vector<double> magnitude_db(const std::vector<double>& b,
                                               const std::vector<double>& a, const Grid& grid) {
    std::vector<double> out;
    out.reserve(grid.omega.size());
    for (const double w : grid.omega) {
        out.push_back(20.0 * std::log10(std::max(std::abs(dsp::frequency_response(b, a, w)), 1e-12)));
    }
    return out;
}

[[nodiscard]] std::vector<double> phase_degrees(const std::vector<double>& b,
                                                const std::vector<double>& a, const Grid& grid) {
    std::vector<double> out;
    out.reserve(grid.omega.size());
    double previous = 0.0;
    double offset = 0.0;
    for (std::size_t i = 0; i < grid.omega.size(); ++i) {
        double phase = std::arg(dsp::frequency_response(b, a, grid.omega[i]));
        if (i != 0) {
            while (phase + offset - previous > kPi) {
                offset -= 2.0 * kPi;
            }
            while (phase + offset - previous < -kPi) {
                offset += 2.0 * kPi;
            }
        }
        previous = phase + offset;
        out.push_back(previous * 180.0 / kPi);
    }
    return out;
}

[[nodiscard]] std::vector<double> group_delay_series(const std::vector<double>& b,
                                                     const std::vector<double>& a,
                                                     const Grid& grid) {
    std::vector<double> out;
    out.reserve(grid.omega.size());
    for (const double w : grid.omega) {
        out.push_back(dsp::group_delay(b, a, w));
    }
    return out;
}

// ---------------------------------------------------------------------------
// The generalised correlation estimate (sessions 3, 4, 6, 7)
// ---------------------------------------------------------------------------

// Solves R_uu h = R_uy for an FIR estimate of the channel. Output noise is
// uncorrelated with the input, so it cancels in R_uy instead of biasing the
// estimate - that is the whole reason the course teaches this method.
[[nodiscard]] std::vector<double> estimate_fir_by_correlation(const std::vector<double>& u,
                                                              const std::vector<double>& y,
                                                              std::size_t taps) {
    const int lag = static_cast<int>(taps);
    const dsp::CorrelationResult ruu = dsp::auto_correlate(u, lag);
    const dsp::CorrelationResult ruy = dsp::cross_correlate(u, y, lag);
    const auto at = [](const dsp::CorrelationResult& r, int l) {
        const int index = l - r.lag_min;
        if (index < 0 || index >= static_cast<int>(r.values.size())) {
            return 0.0;
        }
        return r.values[static_cast<std::size_t>(index)];
    };

    Eigen::MatrixXd A(static_cast<Eigen::Index>(taps), static_cast<Eigen::Index>(taps));
    Eigen::VectorXd rhs(static_cast<Eigen::Index>(taps));
    for (std::size_t i = 0; i < taps; ++i) {
        for (std::size_t k = 0; k < taps; ++k) {
            A(static_cast<Eigen::Index>(i), static_cast<Eigen::Index>(k)) =
                at(ruu, static_cast<int>(i) - static_cast<int>(k));
        }
        rhs(static_cast<Eigen::Index>(i)) = at(ruy, static_cast<int>(i));
    }
    const Eigen::VectorXd h = dsp::least_squares(A, rhs);
    std::vector<double> out(taps, 0.0);
    for (std::size_t i = 0; i < taps; ++i) {
        out[i] = h(static_cast<Eigen::Index>(i));
    }
    return out;
}

// The parametric generalised correlation method (the instrumental-variable
// form of it): the ARX equation
//
//     y[n] + sum_k a_k y[n-k] = sum_k b_k u[n-k]
//
// is multiplied by the delayed input u[n-tau] and summed over one FIXED range
// of n, giving one equation per tau. Every sum runs over the same samples, so
// the equations are exactly the ones the data satisfies - no record-end
// mismatch, nothing truncated. The output noise is uncorrelated with u, so it
// averages away in each sum instead of biasing the solution the way it biases
// a plain least-squares fit, whose regressors contain the noisy output.
[[nodiscard]] dsp::Tf estimate_arx_by_correlation(const std::vector<double>& u,
                                                  const std::vector<double>& y, std::size_t q,
                                                  std::size_t p, std::size_t lags) {
    const std::size_t n = std::min(u.size(), y.size());
    const std::size_t first = lags + std::max(p, q) + 1;
    if (n <= first + p + q + 2) {
        throw StudyError("the record is too short to build " + std::to_string(lags + 1) +
                         " correlation equations: " + std::to_string(n) + " samples received");
    }

    const Eigen::Index rows = static_cast<Eigen::Index>(lags + 1);
    const Eigen::Index cols = static_cast<Eigen::Index>(p + q + 1);
    Eigen::MatrixXd A = Eigen::MatrixXd::Zero(rows, cols);
    Eigen::VectorXd rhs = Eigen::VectorXd::Zero(rows);
    for (std::size_t t = 0; t <= lags; ++t) {
        const Eigen::Index row = static_cast<Eigen::Index>(t);
        for (std::size_t sample = first; sample < n; ++sample) {
            const double instrument = u[sample - t];
            for (std::size_t k = 1; k <= p; ++k) {
                A(row, static_cast<Eigen::Index>(k - 1)) += instrument * y[sample - k];
            }
            for (std::size_t k = 0; k <= q; ++k) {
                A(row, static_cast<Eigen::Index>(p + k)) -= instrument * u[sample - k];
            }
            rhs(row) -= instrument * y[sample];
        }
    }
    const Eigen::VectorXd theta = dsp::least_squares(A, rhs);

    dsp::Tf tf;
    tf.a.assign(p + 1, 0.0);
    tf.a[0] = 1.0;
    for (std::size_t k = 0; k < p; ++k) {
        tf.a[k + 1] = theta(static_cast<Eigen::Index>(k));
    }
    tf.b.assign(q + 1, 0.0);
    for (std::size_t k = 0; k <= q; ++k) {
        tf.b[k] = theta(static_cast<Eigen::Index>(p + k));
    }
    return tf;
}

// ---------------------------------------------------------------------------
// Magnitude specifications and FIR design (sessions 8, 9, 10, 16)
// ---------------------------------------------------------------------------

struct BandSpec {
    double lo_hz = 0.0;
    double hi_hz = 0.0;
    double gain = 1.0;          // desired |H|
    double weight = 1.0;        // least-squares weight
    double tolerance_db = 1.0;  // pass-band ripple, or stop-band attenuation
    bool is_stop = false;
};

[[nodiscard]] bool band_at(const std::vector<BandSpec>& bands, double hz, BandSpec& out) {
    for (const auto& band : bands) {
        if (hz >= band.lo_hz && hz <= band.hi_hz) {
            out = band;
            return true;
        }
    }
    return false;
}

// Gain used by the methods that need a value everywhere: inside a band it is
// the specified gain, inside a transition it is a straight line between the
// neighbouring bands, which is what keeps a frequency-sampling design from
// ringing against an impossible brick wall.
[[nodiscard]] double interpolated_gain(const std::vector<BandSpec>& bands, double hz) {
    BandSpec hit;
    if (band_at(bands, hz, hit)) {
        return hit.gain;
    }
    const BandSpec* below = nullptr;
    const BandSpec* above = nullptr;
    for (const auto& band : bands) {
        if (band.hi_hz < hz && (below == nullptr || band.hi_hz > below->hi_hz)) {
            below = &band;
        }
        if (band.lo_hz > hz && (above == nullptr || band.lo_hz < above->lo_hz)) {
            above = &band;
        }
    }
    if (below == nullptr && above == nullptr) {
        return 0.0;
    }
    if (below == nullptr) {
        return above->gain;
    }
    if (above == nullptr) {
        return below->gain;
    }
    const double span = above->lo_hz - below->hi_hz;
    const double t = (span > 0.0) ? (hz - below->hi_hz) / span : 0.0;
    return below->gain + t * (above->gain - below->gain);
}

[[nodiscard]] std::vector<double> design_fir(const std::string& method, const std::string& window,
                                             double beta, std::size_t order, double fs,
                                             const std::vector<BandSpec>& bands) {
    if (order % 2 != 0) {
        ++order;  // Type I only: an even order has an integer group delay
    }
    const std::size_t length = order + 1;
    const std::size_t m = order / 2;
    std::vector<double> b(length, 0.0);

    if (method == "windowed") {
        // The ideal cutoff goes in the MIDDLE of each transition band, never on
        // the pass-band edge: an ideal band edge is the -6 dB point, so putting
        // it on the edge guarantees the pass-band ripple is violated there.
        std::vector<BandSpec> widened = bands;
        for (std::size_t bi = 0; bi < widened.size(); ++bi) {
            const BandSpec& original = bands[bi];
            for (const auto& other : bands) {
                if (other.lo_hz > original.hi_hz) {
                    widened[bi].hi_hz = std::max(widened[bi].hi_hz,
                                                 0.5 * (original.hi_hz + other.lo_hz));
                    break;
                }
            }
            for (std::size_t j = bands.size(); j-- > 0;) {
                if (bands[j].hi_hz < original.lo_hz) {
                    widened[bi].lo_hz = std::min(widened[bi].lo_hz,
                                                 0.5 * (original.lo_hz + bands[j].hi_hz));
                    break;
                }
            }
        }
        for (std::size_t i = 0; i < length; ++i) {
            const double n = static_cast<double>(i) - static_cast<double>(m);
            double value = 0.0;
            for (const auto& band : widened) {
                if (band.gain == 0.0) {
                    continue;
                }
                const double upper = 2.0 * std::min(band.hi_hz, 0.5 * fs) / fs;
                const double lower = 2.0 * std::max(band.lo_hz, 0.0) / fs;
                const double hi = (n == 0.0) ? upper : std::sin(kPi * upper * n) / (kPi * n);
                const double lo = (n == 0.0) ? lower : std::sin(kPi * lower * n) / (kPi * n);
                value += band.gain * (hi - lo);
            }
            b[i] = value;
        }
        const std::vector<double> w = make_window(window, length, beta);
        for (std::size_t i = 0; i < length; ++i) {
            b[i] *= w[i];
        }
    } else if (method == "least_squares") {
        const std::size_t points = 16 * length;
        std::vector<double> rows_hz;
        std::vector<double> rows_target;
        std::vector<double> rows_weight;
        rows_hz.reserve(points);
        rows_target.reserve(points);
        rows_weight.reserve(points);
        for (std::size_t i = 0; i < points; ++i) {
            const double hz = 0.5 * fs * static_cast<double>(i) / static_cast<double>(points - 1);
            BandSpec hit;
            if (!band_at(bands, hz, hit)) {
                continue;  // a transition band is a don't-care, not a target
            }
            rows_hz.push_back(hz);
            rows_target.push_back(hit.gain);
            rows_weight.push_back(std::sqrt(std::max(hit.weight, 1e-6)));
        }
        if (rows_hz.size() < m + 1) {
            throw StudyError("the specification leaves too few constrained frequencies for an "
                             "order-" + std::to_string(order) + " least-squares design");
        }
        Eigen::MatrixXd A(static_cast<Eigen::Index>(rows_hz.size()),
                          static_cast<Eigen::Index>(m + 1));
        Eigen::VectorXd rhs(static_cast<Eigen::Index>(rows_hz.size()));
        for (std::size_t r = 0; r < rows_hz.size(); ++r) {
            const double w = 2.0 * kPi * rows_hz[r] / fs;
            A(static_cast<Eigen::Index>(r), 0) = rows_weight[r];
            for (std::size_t k = 1; k <= m; ++k) {
                A(static_cast<Eigen::Index>(r), static_cast<Eigen::Index>(k)) =
                    rows_weight[r] * 2.0 * std::cos(w * static_cast<double>(k));
            }
            rhs(static_cast<Eigen::Index>(r)) = rows_weight[r] * rows_target[r];
        }
        const Eigen::VectorXd c = dsp::least_squares(A, rhs);
        b[m] = c(0);
        for (std::size_t k = 1; k <= m; ++k) {
            b[m + k] = c(static_cast<Eigen::Index>(k));
            b[m - k] = c(static_cast<Eigen::Index>(k));
        }
    } else {
        // Frequency sampling: a linear-phase target on the DFT grid, inverse
        // transformed and symmetrised.
        std::vector<Complex> spectrum(length, Complex(0.0, 0.0));
        for (std::size_t k = 0; k <= length / 2; ++k) {
            const double w = 2.0 * kPi * static_cast<double>(k) / static_cast<double>(length);
            const double hz = w * fs / (2.0 * kPi);
            const double gain = interpolated_gain(bands, hz);
            const double phase = -w * static_cast<double>(m);
            const Complex value = gain * Complex(std::cos(phase), std::sin(phase));
            spectrum[k] = value;
            if (k != 0 && length - k < length) {
                spectrum[length - k] = std::conj(value);
            }
        }
        const std::vector<double> raw = dsp::ifft_real(spectrum);
        for (std::size_t i = 0; i < length; ++i) {
            b[i] = 0.5 * (raw[i] + raw[length - 1 - i]);
        }
    }
    return b;
}

[[nodiscard]] std::vector<BandSpec> fir_specification(const std::string& name, double fs,
                                                      double pass_edge, double stop_edge,
                                                      double resonance_hz, double stop_db) {
    std::vector<BandSpec> bands;
    const double nyquist = 0.5 * fs;
    const double stop_gain = std::pow(10.0, -stop_db / 20.0);
    if (name == "servo_lowpass") {
        bands.push_back({0.0, pass_edge, 1.0, 1.0, 1.0, false});
        bands.push_back({stop_edge, nyquist, 0.0, 10.0, stop_db, true});
    } else if (name == "resonance_notch") {
        const double width = std::max(4.0, 0.25 * resonance_hz);
        bands.push_back({0.0, std::max(1.0, resonance_hz - 2.0 * width), 1.0, 1.0, 1.0, false});
        bands.push_back({std::max(0.5, resonance_hz - 0.5 * width),
                         std::min(nyquist, resonance_hz + 0.5 * width), 0.0, 10.0, stop_db, true});
        if (resonance_hz + 2.0 * width < nyquist) {
            bands.push_back({resonance_hz + 2.0 * width, nyquist, 1.0, 1.0, 1.0, false});
        }
    } else {
        bands.push_back({0.0, std::max(1.0, pass_edge - 0.5 * pass_edge), 0.0, 5.0, stop_db, true});
        bands.push_back({pass_edge, std::min(stop_edge, nyquist), 1.0, 1.0, 1.0, false});
        if (stop_edge * 1.5 < nyquist) {
            bands.push_back({stop_edge * 1.5, nyquist, 0.0, 5.0, stop_db, true});
        }
    }
    (void)stop_gain;
    return bands;
}

struct SpecCheck {
    json::Value table = json::Value::object();
    std::size_t passed = 0;
    std::size_t failed = 0;
    double passband_max_deviation_db = 0.0;
    double stopband_attenuation_db = 1000.0;
    double worst_margin_db = 1e9;
};

// Checks the realised coefficients against every specification point, which is
// the discipline the course project demands: the response is recomputed from
// the coefficients, never assumed from the design intent.
[[nodiscard]] SpecCheck check_specification(const std::vector<double>& b,
                                            const std::vector<double>& a, double fs,
                                            const std::vector<BandSpec>& bands,
                                            std::size_t points_per_band) {
    SpecCheck check;
    std::vector<std::vector<json::Value>> rows;
    for (const auto& band : bands) {
        for (std::size_t i = 0; i < points_per_band; ++i) {
            const double t = (points_per_band == 1)
                                 ? 0.5
                                 : static_cast<double>(i) / static_cast<double>(points_per_band - 1);
            const double hz = band.lo_hz + t * (band.hi_hz - band.lo_hz);
            const double w = 2.0 * kPi * hz / fs;
            const double achieved_db =
                20.0 * std::log10(std::max(std::abs(dsp::frequency_response(b, a, w)), 1e-12));
            double margin = 0.0;
            std::string requirement;
            if (band.is_stop) {
                margin = -achieved_db - band.tolerance_db;
                requirement = "<= " + json::number_to_string(-band.tolerance_db) + " dB";
                check.stopband_attenuation_db = std::min(check.stopband_attenuation_db, -achieved_db);
            } else {
                const double reference = 20.0 * std::log10(std::max(band.gain, 1e-12));
                margin = band.tolerance_db - std::abs(achieved_db - reference);
                requirement = "within +/-" + json::number_to_string(band.tolerance_db) + " dB";
                check.passband_max_deviation_db =
                    std::max(check.passband_max_deviation_db, std::abs(achieved_db - reference));
            }
            const bool ok = margin >= 0.0;
            if (ok) {
                ++check.passed;
            } else {
                ++check.failed;
            }
            check.worst_margin_db = std::min(check.worst_margin_db, margin);
            rows.push_back({json::Value(hz), json::Value(band.is_stop ? "stop" : "pass"),
                            json::Value(requirement), json::Value(achieved_db),
                            json::Value(margin), json::Value(ok ? "PASS" : "FAIL")});
        }
    }
    if (check.stopband_attenuation_db > 999.0) {
        check.stopband_attenuation_db = 0.0;
    }
    check.table = json::from_table(
        {"frequency_hz", "band", "requirement", "achieved_db", "margin_db", "verdict"}, rows);
    return check;
}

// ---------------------------------------------------------------------------
// Fixed-point realisation (sessions 12 and 16)
// ---------------------------------------------------------------------------

struct Quantised {
    std::vector<double> values;
    std::size_t overflow_count = 0;
    double step = 0.0;
    double max_abs_requested = 0.0;
};

// Q1.(bits-1): one sign bit, bits-1 fractional bits, representable range
// [-1, 1 - 2^-(bits-1)]. No automatic scaling, because the exam's point is
// exactly that a coefficient of -1.6 does not fit.
[[nodiscard]] Quantised quantise_coefficients(const std::vector<double>& c, int bits) {
    Quantised q;
    q.step = std::pow(2.0, -(static_cast<double>(bits) - 1.0));
    q.values.reserve(c.size());
    const double upper = 1.0 - q.step;
    for (const double v : c) {
        q.max_abs_requested = std::max(q.max_abs_requested, std::abs(v));
        double scaled = std::round(v / q.step) * q.step;
        if (scaled > upper || scaled < -1.0) {
            ++q.overflow_count;
            scaled = std::clamp(scaled, -1.0, upper);
        }
        q.values.push_back(scaled);
    }
    return q;
}

[[nodiscard]] json::Value complex_set(const std::vector<Complex>& values) {
    return json::from_complex_set(values);
}

[[nodiscard]] json::Value unit_circle_series() {
    std::vector<double> x;
    std::vector<double> y;
    x.reserve(181);
    y.reserve(181);
    for (std::size_t i = 0; i < 181; ++i) {
        const double angle = 2.0 * kPi * static_cast<double>(i) / 180.0;
        x.push_back(std::cos(angle));
        y.push_back(std::sin(angle));
    }
    return json::from_series("unit circle", x, y);
}

// A short, realistic joint-velocity burst: the robot accelerates, cruises and
// stops, and the structural mode rings on the way out.
[[nodiscard]] std::vector<double> velocity_burst(const dsp::JointChannel& channel, std::size_t n) {
    std::vector<double> command(n, 0.0);
    const std::size_t ramp = std::max<std::size_t>(4, n / 16);
    for (std::size_t i = 0; i < n; ++i) {
        double value = 0.0;
        if (i < ramp) {
            value = static_cast<double>(i) / static_cast<double>(ramp);
        } else if (i < n / 2) {
            value = 1.0;
        } else if (i < n / 2 + ramp) {
            value = 1.0 - static_cast<double>(i - n / 2) / static_cast<double>(ramp);
        }
        command[i] = value;
    }
    return dsp::apply_filter(channel.tf.b, channel.tf.a, command);
}

// ---------------------------------------------------------------------------
// Session 2 - test-signal selection
// ---------------------------------------------------------------------------

[[nodiscard]] json::Value op_test_signals(const json::Value& args) {
    const std::string kind = optional_enum(args, "signal", "prbs", kSignalOptions);
    const std::size_t n = read_length(args, 512);
    const double fs = read_fs(args);
    const double amplitude = optional_scalar(args, "amplitude", 1.0, 1e-6, 100.0);
    const double snr_db = optional_scalar(args, "snr_db", 30.0, -10.0, 200.0);
    const std::size_t joint = read_joint(args);
    const unsigned seed = read_seed(args);

    const dsp::JointChannel channel = dsp::joint_channel(joint, fs);
    const std::vector<double> u = generate_test_signal(kind, n, fs, amplitude, seed);
    const std::vector<double> y_clean = dsp::apply_filter(channel.tf.b, channel.tf.a, u);

    // One absolute noise level for the whole comparison, referred to the PRBS
    // record at the requested SNR. Every candidate then meets the same sensor.
    const std::vector<double> reference_output = dsp::apply_filter(
        channel.tf.b, channel.tf.a, prbs_sequence(n, amplitude, seed));
    const double noise_sigma = rms(reference_output) * std::pow(10.0, -snr_db / 20.0);
    const std::vector<double> y = add_absolute_noise(y_clean, noise_sigma, seed + 7u);

    std::vector<double> time;
    time.reserve(n);
    for (std::size_t i = 0; i < n; ++i) {
        time.push_back(static_cast<double>(i) / fs);
    }

    const std::vector<double> power = power_spectrum(u);
    std::vector<double> spectrum_hz;
    std::vector<double> spectrum_db;
    spectrum_hz.reserve(power.size());
    spectrum_db.reserve(power.size());
    for (std::size_t k = 0; k < power.size(); ++k) {
        spectrum_hz.push_back(fs * static_cast<double>(k) / static_cast<double>(n));
        spectrum_db.push_back(10.0 * std::log10(std::max(power[k], 1e-20)));
    }

    // The assessed point: which excitation actually identifies the joint. Every
    // candidate is run through the same channel at the same SNR and the same
    // 32-tap correlation estimate, and the recovered impulse response is
    // compared with the truth.
    const std::size_t taps = 32;
    const std::vector<double> truth = dsp::impulse_response(channel.tf.b, channel.tf.a, taps);
    double truth_norm = 0.0;
    for (const double v : truth) {
        truth_norm += v * v;
    }
    truth_norm = std::sqrt(std::max(truth_norm, 1e-30));

    std::vector<std::vector<json::Value>> rows;
    std::string best;
    double best_error = 1e30;
    for (const auto& candidate : kSignalOptions) {
        const std::vector<double> cu = generate_test_signal(candidate, n, fs, amplitude, seed);
        const std::vector<double> cy = add_absolute_noise(
            dsp::apply_filter(channel.tf.b, channel.tf.a, cu), noise_sigma, seed + 7u);
        double error = 1e30;
        if (n > 4 * taps) {
            const std::vector<double> h = estimate_fir_by_correlation(cu, cy, taps);
            double sum = 0.0;
            for (std::size_t i = 0; i < taps; ++i) {
                const double e = h[i] - truth[i];
                sum += e * e;
            }
            error = std::sqrt(sum) / truth_norm;
        }
        rows.push_back({json::Value(candidate), json::Value(crest_factor(cu)),
                        json::Value(spectral_flatness(cu)),
                        json::Value(excited_fraction(cu, 20.0)), json::Value(error)});
        if (error < best_error) {
            best_error = error;
            best = candidate;
        }
    }

    std::string why;
    if (best == "prbs" || best == "multisine" || best == "white_noise") {
        why = "it spreads the same total energy over every bin of the servo band, so every "
              "frequency the joint can respond at is excited well above the noise floor";
    } else {
        why = "at this record length and SNR it happened to put the most energy where the channel "
              "has gain; a broadband excitation is normally the better identifier";
    }

    json::Value out = json::Value::object();
    out.set("signal", json::from_series(kind + " excitation (joint " + std::to_string(joint) +
                                            " velocity command)",
                                        time, u));
    out.set("response", json::from_series("measured joint velocity at " +
                                              json::number_to_string(snr_db) + " dB SNR",
                                          time, y));
    out.set("spectrum", json::from_series("excitation spectrum", spectrum_hz, spectrum_db));
    out.set("comparison", json::from_table({"signal", "crest_factor", "spectral_flatness",
                                            "excited_band_fraction", "identification_error"},
                                           rows));
    out.set("best_signal", json::Value(best));
    out.set("identification_error", json::Value(best_error));
    out.set("crest_factor", json::Value(crest_factor(u)));
    out.set("spectral_flatness", json::Value(spectral_flatness(u)));
    out.set("resonance_hz", json::Value(channel.resonance_hz));
    out.set("verdict",
            json::Value("Best identifier at this setting: " + best + " - " + why +
                        ". A single step has a 1/f spectrum: it barely excites the band around "
                        "the " + json::number_to_string(channel.resonance_hz) +
                        " Hz structural mode of axis " + std::to_string(joint) +
                        ", so the high end of H(z) is fitted to noise. An impulse excites "
                        "everything but puts all its energy in one sample, so its peak amplitude "
                        "has to be huge before the measurement clears the noise - which is the "
                        "crest-factor column, and why PRBS and the multisine win in practice."));
    return out;
}

// ---------------------------------------------------------------------------
// Session 1 - minimum, maximum and linear phase
// ---------------------------------------------------------------------------

[[nodiscard]] std::vector<double> phase_preset_coefficients(const std::string& preset) {
    // Two real zero pairs, placed inside or outside the unit circle. Reflecting
    // a zero to 1/z* leaves |H| unchanged and changes only the phase, which is
    // the point of the session.
    const auto pair_factor = [](double radius, double angle) {
        return std::vector<double>{1.0, -2.0 * radius * std::cos(angle), radius * radius};
    };
    if (preset == "minimum_phase") {
        return dsp::convolve(pair_factor(0.5, 0.6 * kPi), pair_factor(0.8, 0.2 * kPi));
    }
    if (preset == "maximum_phase") {
        return dsp::convolve(pair_factor(2.0, 0.6 * kPi), pair_factor(1.25, 0.2 * kPi));
    }
    if (preset == "mixed_phase") {
        return dsp::convolve(pair_factor(0.5, 0.6 * kPi), pair_factor(1.25, 0.2 * kPi));
    }
    // Linear phase: a symmetric Hamming low-pass, the filter the encoder
    // channel is actually cleaned with.
    const std::vector<BandSpec> bands = {{0.0, 100.0, 1.0, 1.0, 1.0, false},
                                         {200.0, 500.0, 0.0, 10.0, 40.0, true}};
    return design_fir("windowed", "hamming", 5.0, 16, dsp::kGp8ControlRateHz, bands);
}

[[nodiscard]] json::Value op_phase_types(const json::Value& args) {
    const std::string preset = optional_enum(args, "preset", "linear_phase", kPhasePresets);
    const double fs = read_fs(args);
    const std::size_t joint = read_joint(args);
    const std::size_t n = read_length(args, 256);

    std::vector<double> b;
    bool custom = false;
    if (!is_absent(args, "b")) {
        // The parameter's declared default is a single tap, which is the "no
        // override" marker: a one-tap FIR has no phase type to classify. Anything
        // shorter than two taps therefore falls back to the selected preset.
        std::vector<double> supplied = read_numeric_array(require_present(args, "b"), "b", 1, 257);
        if (supplied.size() >= 2) {
            b = std::move(supplied);
            custom = true;
        } else {
            b = phase_preset_coefficients(preset);
        }
    } else {
        b = phase_preset_coefficients(preset);
    }
    if (max_abs(b) <= 0.0) {
        throw StudyError("parameter 'b' must contain at least one non-zero coefficient");
    }

    const std::vector<double> a = {1.0};
    const std::vector<Complex> zeros = dsp::polynomial_roots(b);
    double min_radius = 1e30;
    double max_radius = 0.0;
    std::size_t inside = 0;
    std::size_t outside = 0;
    std::size_t on_circle = 0;
    for (const auto& z : zeros) {
        const double r = std::abs(z);
        min_radius = std::min(min_radius, r);
        max_radius = std::max(max_radius, r);
        if (r < 1.0 - 1e-9) {
            ++inside;
        } else if (r > 1.0 + 1e-9) {
            ++outside;
        } else {
            ++on_circle;
        }
    }
    if (zeros.empty()) {
        min_radius = 0.0;
    }

    const double scale = max_abs(b);
    bool symmetric = true;
    bool antisymmetric = true;
    for (std::size_t i = 0; i < b.size(); ++i) {
        const double mirrored = b[b.size() - 1 - i];
        symmetric = symmetric && std::abs(b[i] - mirrored) <= 1e-9 * scale;
        antisymmetric = antisymmetric && std::abs(b[i] + mirrored) <= 1e-9 * scale;
    }

    std::string verdict;
    std::string reason;
    if (symmetric || antisymmetric) {
        const std::size_t length = b.size();
        const std::string type = symmetric ? (length % 2 == 1 ? "Type I" : "Type II")
                                           : (length % 2 == 1 ? "Type III" : "Type IV");
        verdict = "linear phase (" + type + ")";
        reason = std::string(symmetric ? "the coefficients are symmetric" : "the coefficients are "
                                                                            "antisymmetric") +
                 ", so every zero appears with its reciprocal and the phase is exactly "
                 "-omega (N-1)/2: the group delay is the constant " +
                 json::number_to_string(static_cast<double>(length - 1) / 2.0) +
                 " samples and the waveform shape of the velocity burst survives the filter";
        if (!symmetric) {
            reason += ". The antisymmetric types force a zero at z = +1, so they cannot pass DC";
        } else if (length % 2 == 0) {
            reason += ". Type II forces a zero at z = -1, which is why it can never be a high-pass";
        }
    } else if (outside == 0 && on_circle == 0) {
        verdict = "minimum phase";
        reason = "every one of the " + std::to_string(zeros.size()) +
                 " zeros lies strictly inside the unit circle (largest radius " +
                 json::number_to_string(max_radius) +
                 "), so the filter is causally invertible and its energy arrives as early as any "
                 "filter with this magnitude can deliver it";
    } else if (inside == 0 && on_circle == 0) {
        verdict = "maximum phase";
        reason = "every zero lies outside the unit circle (smallest radius " +
                 json::number_to_string(min_radius) +
                 "), so the same magnitude response is delivered with the largest possible phase "
                 "lag: the burst arrives skewed, late and with its shape changed";
    } else {
        verdict = "mixed phase";
        reason = std::to_string(inside) + " zeros inside, " + std::to_string(outside) +
                 " outside and " + std::to_string(on_circle) +
                 " on the unit circle, so the filter is neither minimum nor maximum phase and is "
                 "not stably invertible";
    }

    const Grid grid = make_grid(fs, kResponsePoints);
    const dsp::JointChannel channel = dsp::joint_channel(joint, fs);
    const std::vector<double> burst = velocity_burst(channel, n);
    const std::vector<double> filtered = dsp::apply_filter(b, a, burst);
    std::vector<double> time;
    time.reserve(n);
    for (std::size_t i = 0; i < n; ++i) {
        time.push_back(static_cast<double>(i) / fs);
    }

    // Shape distortion: the best achievable match after any pure delay is
    // removed. A linear-phase filter scores ~1, a maximum-phase one does not.
    const dsp::CorrelationResult alignment =
        dsp::cross_correlate(burst, filtered, static_cast<int>(std::min<std::size_t>(n / 2, 256)));
    const double shape_match =
        alignment.peak_value / std::max(1e-30, rms(burst) * rms(filtered) * static_cast<double>(n));

    std::vector<std::vector<json::Value>> zero_rows;
    for (const auto& z : zeros) {
        zero_rows.push_back({json::Value(z.real()), json::Value(z.imag()), json::Value(std::abs(z)),
                             json::Value(std::arg(z)),
                             json::Value(std::abs(z) < 1.0 ? "inside" : "outside")});
    }

    json::Value out = json::Value::object();
    out.set("coefficients", coefficient_matrix(b));
    out.set("source", json::Value(custom ? "coefficients supplied by the caller" : preset));
    out.set("zeros", complex_set(zeros));
    out.set("unit_circle", unit_circle_series());
    out.set("zero_table",
            json::from_table({"re", "im", "radius", "angle_rad", "position"}, zero_rows));
    out.set("magnitude", json::from_series("|H| of the candidate filter", grid.hz,
                                           magnitude_db(b, a, grid)));
    out.set("group_delay",
            json::from_series("group delay", grid.hz, group_delay_series(b, a, grid)));
    out.set("velocity_in", json::from_series("joint velocity burst", time, burst));
    out.set("velocity_out", json::from_series("after the filter", time, filtered));
    out.set("phase_class", json::Value(verdict));
    out.set("reason", json::Value(reason));
    out.set("is_linear_phase", json::Value(symmetric || antisymmetric));
    out.set("min_zero_radius", json::Value(min_radius));
    out.set("max_zero_radius", json::Value(max_radius));
    out.set("shape_match", json::Value(shape_match));
    out.set("peak_shift_samples", json::Value(static_cast<double>(alignment.peak_lag)));
    out.set("verdict",
            json::Value("This filter is " + verdict + ": " + reason +
                        ". Reflecting a zero from radius r to 1/r leaves |H| unchanged and the "
                        "group delay completely changed, which is why the magnitude plot alone "
                        "cannot tell you whether the joint-" + std::to_string(joint) +
                        " velocity burst will keep its shape. The two velocity traces show the "
                        "answer directly: the waveform match after the pure delay is removed is " +
                        json::number_to_string(shape_match) + ", at a shift of " +
                        std::to_string(alignment.peak_lag) + " samples."));
    return out;
}

// ---------------------------------------------------------------------------
// Sessions 3, 6, 7 - transfer-function determination
// ---------------------------------------------------------------------------

[[nodiscard]] json::Value op_identify_transfer_function(const json::Value& args) {
    const std::string method =
        optional_enum(args, "method", "generalised_correlation", kIdentificationMethods);
    const std::string excitation = optional_enum(args, "excitation", "prbs", kSignalOptions);
    const std::size_t n = read_length(args, 1024);
    const double fs = read_fs(args);
    const std::size_t joint = read_joint(args);
    const double snr_db = optional_scalar(args, "snr_db", 30.0, -10.0, 200.0);
    const std::size_t num_order =
        static_cast<std::size_t>(optional_int(args, "numerator_order", 2, 0, 20));
    const std::size_t den_order =
        static_cast<std::size_t>(optional_int(args, "denominator_order", 3, 1, 20));
    const double target_error = optional_scalar(args, "target_error", 0.05, 1e-4, 1.0);
    const unsigned seed = read_seed(args);

    const dsp::JointChannel channel = dsp::joint_channel(joint, fs);
    const std::vector<double> u = generate_test_signal(excitation, n, fs, 1.0, seed);
    const std::vector<double> y_clean = dsp::apply_filter(channel.tf.b, channel.tf.a, u);
    const std::vector<double> y = add_noise(y_clean, snr_db, seed + 11u);

    const std::size_t taps = std::clamp<std::size_t>(n / 4, 16, 192);
    std::vector<double> b_hat;
    std::vector<double> a_hat;
    std::string method_note;

    if (method == "least_squares_arx") {
        const std::size_t lag = std::max(num_order, den_order);
        const std::size_t rows = n - lag;
        Eigen::MatrixXd A(static_cast<Eigen::Index>(rows),
                          static_cast<Eigen::Index>(den_order + num_order + 1));
        Eigen::VectorXd rhs(static_cast<Eigen::Index>(rows));
        for (std::size_t i = 0; i < rows; ++i) {
            const std::size_t t = lag + i;
            const Eigen::Index row = static_cast<Eigen::Index>(i);
            for (std::size_t k = 1; k <= den_order; ++k) {
                A(row, static_cast<Eigen::Index>(k - 1)) = -y[t - k];
            }
            for (std::size_t k = 0; k <= num_order; ++k) {
                A(row, static_cast<Eigen::Index>(den_order + k)) = u[t - k];
            }
            rhs(row) = y[t];
        }
        const Eigen::VectorXd theta = dsp::least_squares(A, rhs);
        a_hat.assign(den_order + 1, 0.0);
        a_hat[0] = 1.0;
        for (std::size_t k = 0; k < den_order; ++k) {
            a_hat[k + 1] = theta(static_cast<Eigen::Index>(k));
        }
        b_hat.assign(num_order + 1, 0.0);
        for (std::size_t k = 0; k <= num_order; ++k) {
            b_hat[k] = theta(static_cast<Eigen::Index>(den_order + k));
        }
        method_note =
            "Least-squares ARX regresses y[n] on its own past and on the input. It is the "
            "cheapest estimator and the one that bites: the regressors contain the noisy y, so "
            "the estimate is biased towards a faster channel as the SNR falls.";
    } else if (method == "spectral_division") {
        const std::vector<Complex> U = dsp::fft_real(u);
        const std::vector<Complex> Y = dsp::fft_real(y);
        std::vector<Complex> H(U.size(), Complex(0.0, 0.0));
        double reference = 0.0;
        for (const auto& value : U) {
            reference = std::max(reference, std::abs(value));
        }
        for (std::size_t k = 0; k < U.size(); ++k) {
            const double magnitude = std::abs(U[k]);
            H[k] = (magnitude > 1e-6 * reference) ? Y[k] / U[k] : Complex(0.0, 0.0);
        }
        std::vector<double> h = dsp::ifft_real(H);
        h.resize(std::min(h.size(), taps));
        const dsp::PronyFit fit = dsp::prony(h, num_order, den_order);
        b_hat = fit.b;
        a_hat = fit.a;
        method_note =
            "Spectral division takes Y(k)/U(k) bin by bin. It needs no model order but it divides "
            "by the excitation, so every bin the test signal left empty becomes noise divided by "
            "noise - which is why the bins below a thousandth of the peak are dropped here.";
    } else {
        const dsp::Tf fit = estimate_arx_by_correlation(u, y, num_order, den_order,
                                                        std::min<std::size_t>(taps, 48));
        b_hat = fit.b;
        a_hat = fit.a;
        method_note =
            "The generalised correlation method replaces the data by its correlation functions: "
            "the model equation is multiplied by u[n-tau] and summed, which leaves a small linear "
            "system in the coefficients. Output noise is uncorrelated with the input, so it "
            "cancels there instead of biasing the estimate - this is the estimator the course "
            "centres on, and the reason it survives a record a least-squares fit cannot use.";
    }

    const Grid grid = make_grid(fs, kResponsePoints);
    const std::vector<double> true_mag = magnitude_db(channel.tf.b, channel.tf.a, grid);
    const std::vector<double> est_mag = magnitude_db(b_hat, a_hat, grid);
    const std::vector<double> true_phase = phase_degrees(channel.tf.b, channel.tf.a, grid);
    const std::vector<double> est_phase = phase_degrees(b_hat, a_hat, grid);

    // The comparison is made over the IDENTIFIABLE band only: 60 dB below the
    // channel's own peak there is no measurable signal left, and claiming an
    // error figure there would be claiming to have identified noise.
    double peak_true = -1e30;
    for (const double value : true_mag) {
        peak_true = std::max(peak_true, value);
    }
    const double floor_db = peak_true - 40.0;
    double max_mag_error = 0.0;
    double rms_mag_error = 0.0;
    double band_hz = 0.0;
    std::size_t counted = 0;
    for (std::size_t i = 0; i < true_mag.size(); ++i) {
        if (true_mag[i] < floor_db) {
            continue;
        }
        const double e = est_mag[i] - true_mag[i];
        max_mag_error = std::max(max_mag_error, std::abs(e));
        rms_mag_error += e * e;
        band_hz = std::max(band_hz, grid.hz[i]);
        ++counted;
    }
    rms_mag_error = std::sqrt(rms_mag_error / static_cast<double>(std::max<std::size_t>(1, counted)));

    const std::vector<double> y_hat = dsp::apply_filter(b_hat, a_hat, u);
    std::vector<double> residual(n, 0.0);
    for (std::size_t i = 0; i < n; ++i) {
        residual[i] = y[i] - y_hat[i];
    }
    const double residual_ratio = rms(residual) / std::max(1e-30, rms(y));

    // Record length for a wanted accuracy. The variance of a correlation
    // estimate falls as 1/N and as the SNR: relative error ~ sqrt(taps / (N S)),
    // so N = taps / (S * target^2).
    const double snr_linear = std::pow(10.0, snr_db / 10.0);
    const double required_length =
        std::ceil(static_cast<double>(taps) / (snr_linear * target_error * target_error));

    const std::vector<Complex> poles = dsp::polynomial_roots(a_hat);
    double max_pole = 0.0;
    for (const auto& p : poles) {
        max_pole = std::max(max_pole, std::abs(p));
    }

    std::vector<double> time;
    time.reserve(n);
    for (std::size_t i = 0; i < n; ++i) {
        time.push_back(static_cast<double>(i) / fs);
    }

    json::Value out = json::Value::object();
    out.set("b_estimated", coefficient_matrix(b_hat));
    out.set("a_estimated", coefficient_matrix(a_hat));
    out.set("b_true", coefficient_matrix(channel.tf.b));
    out.set("a_true", coefficient_matrix(channel.tf.a));
    out.set("magnitude_true", json::from_series("true |H|", grid.hz, true_mag));
    out.set("magnitude_estimated", json::from_series("estimated |H|", grid.hz, est_mag));
    out.set("phase_true", json::from_series("true phase", grid.hz, true_phase));
    out.set("phase_estimated", json::from_series("estimated phase", grid.hz, est_phase));
    out.set("residual", json::from_series("one-step prediction error", time, residual));
    out.set("max_magnitude_error_db", json::Value(max_mag_error));
    out.set("rms_magnitude_error_db", json::Value(rms_mag_error));
    out.set("identifiable_band_hz", json::Value(band_hz));
    out.set("residual_ratio", json::Value(residual_ratio));
    out.set("required_length", json::Value(required_length));
    out.set("poles", complex_set(poles));
    out.set("max_pole_radius", json::Value(max_pole));
    out.set("method", json::Value(method));
    out.set("verdict",
            json::Value(method_note + " Estimated from " + std::to_string(n) + " samples of " +
                        excitation + " at " + json::number_to_string(snr_db) +
                        " dB SNR on axis " + std::to_string(joint) +
                        ": across the identifiable band (up to " +
                        json::number_to_string(band_hz) +
                        " Hz, where the channel is still within 40 dB of its peak) the magnitude "
                        "is within " + json::number_to_string(max_mag_error) +
                        " dB of the truth and the slowest pole sits at radius " +
                        json::number_to_string(max_pole) + ". For a relative error of " +
                        json::number_to_string(target_error) + " this channel needs about " +
                        json::number_to_string(required_length) +
                        " samples at this SNR, which is " +
                        json::number_to_string(required_length / fs) + " s of robot motion."));
    return out;
}

// ---------------------------------------------------------------------------
// Session 3 - delay elimination
// ---------------------------------------------------------------------------

[[nodiscard]] json::Value op_delay_elimination(const json::Value& args) {
    const std::size_t n = read_length(args, 512);
    const double fs = read_fs(args);
    const std::size_t joint = read_joint(args);
    const int true_delay = optional_int(args, "true_delay", 7, 0, 64);
    const double snr_db = optional_scalar(args, "snr_db", 20.0, -10.0, 200.0);
    const bool sensor_filter = optional_bool(args, "include_sensor_filter", true);
    const int max_lag = optional_int(args, "max_lag", 64, 1, 512);
    const unsigned seed = read_seed(args);

    if (static_cast<std::size_t>(true_delay) + 8 >= n) {
        throw StudyError("parameter 'true_delay' = " + std::to_string(true_delay) +
                         " needs a record longer than " + std::to_string(true_delay + 8) +
                         " samples, received length " + std::to_string(n));
    }

    const dsp::JointChannel channel = dsp::joint_channel(joint, fs);

    // A jog with an identification dither on top: the trapezoid is the move
    // the operator asked for, the PRBS dither is what makes the record
    // broadband enough for a correlation peak one sample wide. Without it the
    // command is a slow ramp whose auto-correlation is hundreds of samples
    // wide, and no delay estimate from it is worth reading.
    const std::vector<double> envelope = velocity_burst(channel, n);
    const std::vector<double> dither = prbs_sequence(n, 0.4 * max_abs(envelope), seed + 5u);
    std::vector<double> commanded(n, 0.0);
    for (std::size_t i = 0; i < n; ++i) {
        commanded[i] = envelope[i] + dither[i];
    }

    // The measurement chain: a pure transport delay through the servo bus, an
    // optional 3-tap smoothing in the encoder-derived velocity (group delay
    // exactly 1 sample), and sensor noise.
    std::vector<double> delayed(n, 0.0);
    for (std::size_t i = static_cast<std::size_t>(true_delay); i < n; ++i) {
        delayed[i] = commanded[i - static_cast<std::size_t>(true_delay)];
    }
    const std::vector<double> sensor_b =
        sensor_filter ? std::vector<double>{1.0 / 3.0, 1.0 / 3.0, 1.0 / 3.0}
                      : std::vector<double>{1.0};
    const std::vector<double> measured =
        add_noise(dsp::apply_filter(sensor_b, {1.0}, delayed), snr_db, seed + 23u);

    // Two corrections before the peak is read, both of them mandatory on a
    // real record: the mean is removed (a velocity burst has a positive mean,
    // and the raw product sum would then simply peak where the overlap is
    // longest, at lag 0), and each lag is divided by the number of samples
    // that actually overlapped (otherwise long lags are penalised twice).
    const auto centred = [](const std::vector<double>& v) {
        double mean = 0.0;
        for (const double x : v) {
            mean += x;
        }
        mean /= static_cast<double>(std::max<std::size_t>(1, v.size()));
        std::vector<double> out = v;
        for (auto& x : out) {
            x -= mean;
        }
        return out;
    };
    // First differencing on top of the mean removal: it whitens the slow jog
    // out of the way so the dither carries the correlation, and the delay of a
    // difference is the delay of the signal.
    const auto whitened = [&](const std::vector<double>& v) {
        const std::vector<double> zero_mean = centred(v);
        std::vector<double> out(zero_mean.size(), 0.0);
        for (std::size_t i = 1; i < zero_mean.size(); ++i) {
            out[i] = zero_mean[i] - zero_mean[i - 1];
        }
        return out;
    };
    dsp::CorrelationResult correlation =
        dsp::cross_correlate(whitened(commanded), whitened(measured), max_lag);
    correlation.peak_value = 0.0;
    correlation.peak_lag = 0;
    double best_peak = -1.0;
    for (std::size_t i = 0; i < correlation.values.size(); ++i) {
        const int lag = correlation.lag_min + static_cast<int>(i);
        const int overlap = static_cast<int>(n) - std::abs(lag);
        if (overlap <= 0) {
            correlation.values[i] = 0.0;
            continue;
        }
        correlation.values[i] /= static_cast<double>(overlap);
        if (std::abs(correlation.values[i]) > best_peak) {
            best_peak = std::abs(correlation.values[i]);
            correlation.peak_lag = lag;
            correlation.peak_value = correlation.values[i];
        }
    }
    // The peak of a smoothed measurement is a plateau, not a spike: a
    // symmetric 3-tap smoother makes the lags d, d+1 and d+2 exactly equal, so
    // the argmax alone quantises the answer to the plateau's left edge. The
    // centroid of the plateau is the delay, and it also gives a sub-sample
    // estimate for free.
    double weight_total = 0.0;
    double weighted_lag = 0.0;
    for (std::size_t i = 0; i < correlation.values.size(); ++i) {
        const int lag = correlation.lag_min + static_cast<int>(i);
        if (std::abs(lag - correlation.peak_lag) > 4) {
            continue;
        }
        const double weight = std::abs(correlation.values[i]);
        if (weight < 0.5 * std::abs(correlation.peak_value)) {
            continue;
        }
        weight_total += weight;
        weighted_lag += weight * static_cast<double>(lag);
    }
    const double centroid_lag = (weight_total > 0.0) ? weighted_lag / weight_total
                                                     : static_cast<double>(correlation.peak_lag);
    const double sensor_delay = sensor_filter ? 1.0 : 0.0;
    const double fractional_transport = centroid_lag - sensor_delay;
    const int estimated_transport = static_cast<int>(std::lround(fractional_transport));

    const int shift = std::max(0, static_cast<int>(std::lround(centroid_lag)));
    std::vector<double> aligned(n, 0.0);
    for (std::size_t i = 0; i + static_cast<std::size_t>(shift) < n; ++i) {
        aligned[i] = measured[i + static_cast<std::size_t>(shift)];
    }

    std::vector<double> error_before(n, 0.0);
    std::vector<double> error_after(n, 0.0);
    for (std::size_t i = 0; i < n; ++i) {
        error_before[i] = measured[i] - commanded[i];
        error_after[i] = aligned[i] - commanded[i];
    }

    std::vector<double> lags;
    lags.reserve(correlation.values.size());
    for (std::size_t i = 0; i < correlation.values.size(); ++i) {
        lags.push_back(static_cast<double>(correlation.lag_min + static_cast<int>(i)));
    }
    std::vector<double> time;
    time.reserve(n);
    for (std::size_t i = 0; i < n; ++i) {
        time.push_back(static_cast<double>(i) / fs);
    }

    // The point of the session: with the delay in place a parametric estimate
    // is not minimum phase, and after it is stripped it is.
    const std::vector<double> h_delayed = estimate_fir_by_correlation(commanded, measured, 32);
    const std::vector<double> h_aligned = estimate_fir_by_correlation(commanded, aligned, 32);
    const auto largest_zero_radius = [](const std::vector<double>& h) {
        double worst = 0.0;
        for (const auto& z : dsp::polynomial_roots(h)) {
            worst = std::max(worst, std::abs(z));
        }
        return worst;
    };

    json::Value out = json::Value::object();
    out.set("correlation", json::from_series("R_uy(lag)", lags, correlation.values));
    out.set("peak_value", json::Value(correlation.peak_value));
    out.set("peak_lag", json::Value(static_cast<double>(correlation.peak_lag)));
    out.set("peak_centroid_lag", json::Value(centroid_lag));
    out.set("delay_fractional_samples", json::Value(fractional_transport));
    out.set("sensor_group_delay", json::Value(sensor_delay));
    out.set("delay_samples", json::Value(static_cast<double>(estimated_transport)));
    out.set("delay_seconds", json::Value(static_cast<double>(estimated_transport) / fs));
    out.set("true_delay", json::Value(static_cast<double>(true_delay)));
    out.set("commanded", json::from_series("commanded joint velocity", time, commanded));
    out.set("measured_before", json::from_series("measured, delayed", time, measured));
    out.set("measured_after", json::from_series("measured, delay removed", time, aligned));
    out.set("rms_error_before", json::Value(rms(error_before)));
    out.set("rms_error_after", json::Value(rms(error_after)));
    out.set("max_zero_radius_before", json::Value(largest_zero_radius(h_delayed)));
    out.set("max_zero_radius_after", json::Value(largest_zero_radius(h_aligned)));
    out.set("verdict",
            json::Value("The cross-correlation of the commanded and the measured joint-" +
                        std::to_string(joint) + " velocity peaks at lag " +
                        std::to_string(correlation.peak_lag) + " samples. Subtracting the " +
                        json::number_to_string(sensor_delay) +
                        "-sample group delay of the velocity-smoothing filter leaves a transport "
                        "delay of " + std::to_string(estimated_transport) + " samples = " +
                        json::number_to_string(1000.0 * static_cast<double>(estimated_transport) /
                                               fs) +
                        " ms, against the " + std::to_string(true_delay) +
                        " samples that were injected. Shifting the measurement back by the peak "
                        "lag drops the alignment error from " +
                        json::number_to_string(rms(error_before)) + " to " +
                        json::number_to_string(rms(error_after)) +
                        " rad/s RMS. A pure delay is all-pass in magnitude and pure lag in phase, "
                        "so it is invisible in |H| and fatal in any phase-based fit - strip it "
                        "first, identify second."));
    return out;
}

// ---------------------------------------------------------------------------
// Session 4 - smoothing a measured frequency response
// ---------------------------------------------------------------------------

[[nodiscard]] json::Value op_smooth_frequency_response(const json::Value& args) {
    const std::string window = optional_enum(args, "window", "hann", kWindowOptions);
    const int window_length = optional_int(args, "window_length", 9, 1, 101);
    const std::size_t records = static_cast<std::size_t>(optional_int(args, "records", 1, 1, 32));
    const std::size_t n = read_length(args, 512);
    const double fs = read_fs(args);
    const std::size_t joint = read_joint(args);
    const double snr_db = optional_scalar(args, "snr_db", 10.0, -20.0, 200.0);
    const double beta = optional_scalar(args, "kaiser_beta", 5.65, 0.0, 20.0);
    const unsigned seed = read_seed(args);

    if (window_length % 2 == 0) {
        throw StudyError("parameter 'window_length' must be odd so the smoother is centred, "
                         "received " + std::to_string(window_length));
    }

    const dsp::JointChannel channel = dsp::joint_channel(joint, fs);
    const std::size_t bins = n / 2 + 1;
    std::vector<double> suu(bins, 0.0);
    std::vector<Complex> suy(bins, Complex(0.0, 0.0));
    for (std::size_t r = 0; r < records; ++r) {
        const std::vector<double> u = prbs_sequence(n, 1.0, seed + 3u * static_cast<unsigned>(r));
        const std::vector<double> y =
            add_noise(dsp::apply_filter(channel.tf.b, channel.tf.a, u), snr_db,
                      seed + 101u + static_cast<unsigned>(r));
        const std::vector<Complex> U = dsp::fft_real(u);
        const std::vector<Complex> Y = dsp::fft_real(y);
        for (std::size_t k = 0; k < bins; ++k) {
            suu[k] += std::norm(U[k]);
            suy[k] += std::conj(U[k]) * Y[k];
        }
    }

    std::vector<double> hz(bins, 0.0);
    std::vector<double> raw_db(bins, 0.0);
    std::vector<double> true_db(bins, 0.0);
    for (std::size_t k = 0; k < bins; ++k) {
        hz[k] = fs * static_cast<double>(k) / static_cast<double>(n);
        const Complex estimate = (suu[k] > 0.0) ? suy[k] / suu[k] : Complex(0.0, 0.0);
        raw_db[k] = 20.0 * std::log10(std::max(std::abs(estimate), 1e-12));
        const double w = 2.0 * kPi * hz[k] / fs;
        true_db[k] = 20.0 * std::log10(
            std::max(std::abs(dsp::frequency_response(channel.tf.b, channel.tf.a, w)), 1e-12));
    }

    const std::vector<double> smoother =
        make_window(window, static_cast<std::size_t>(window_length), beta);
    double weight_sum = 0.0;
    for (const double v : smoother) {
        weight_sum += v;
    }
    const int half = window_length / 2;
    std::vector<double> smoothed_db(bins, 0.0);
    for (std::size_t k = 0; k < bins; ++k) {
        double acc = 0.0;
        for (int j = -half; j <= half; ++j) {
            int index = static_cast<int>(k) + j;
            if (index < 0) {
                index = -index;  // reflect, so the DC end is not pulled down
            }
            if (index >= static_cast<int>(bins)) {
                index = 2 * static_cast<int>(bins) - 2 - index;
            }
            index = std::clamp(index, 0, static_cast<int>(bins) - 1);
            acc += smoother[static_cast<std::size_t>(j + half)] *
                   raw_db[static_cast<std::size_t>(index)];
        }
        smoothed_db[k] = acc / std::max(weight_sum, 1e-30);
    }

    double raw_variance = 0.0;
    double smoothed_variance = 0.0;
    std::vector<double> residual(bins, 0.0);
    for (std::size_t k = 1; k + 1 < bins; ++k) {
        const double e_raw = raw_db[k] - true_db[k];
        const double e_smooth = smoothed_db[k] - true_db[k];
        residual[k] = e_smooth;
        raw_variance += e_raw * e_raw;
        smoothed_variance += e_smooth * e_smooth;
    }
    const double scale = static_cast<double>(std::max<std::size_t>(1, bins - 2));
    raw_variance /= scale;
    smoothed_variance /= scale;

    std::size_t peak_bin = 1;
    for (std::size_t k = 1; k + 1 < bins; ++k) {
        if (true_db[k] > true_db[peak_bin]) {
            peak_bin = k;
        }
    }
    const double peak_loss = true_db[peak_bin] - smoothed_db[peak_bin];
    const double resolution_hz = static_cast<double>(window_length) * fs / static_cast<double>(n);

    json::Value out = json::Value::object();
    out.set("raw", json::from_series("measured |H| (H1 estimator)", hz, raw_db));
    out.set("smoothed", json::from_series("smoothed estimate", hz, smoothed_db));
    out.set("truth", json::from_series("true |H| of the channel", hz, true_db));
    out.set("residual", json::from_series("smoothed minus true", hz, residual));
    out.set("noise_std_raw_db", json::Value(std::sqrt(raw_variance)));
    out.set("noise_std_smoothed_db", json::Value(std::sqrt(smoothed_variance)));
    out.set("variance_reduction",
            json::Value(raw_variance / std::max(smoothed_variance, 1e-30)));
    out.set("resonance_hz", json::Value(hz[peak_bin]));
    out.set("resonance_peak_true_db", json::Value(true_db[peak_bin]));
    out.set("resonance_peak_smoothed_db", json::Value(smoothed_db[peak_bin]));
    out.set("peak_loss_db", json::Value(peak_loss));
    out.set("resolution_hz", json::Value(resolution_hz));
    out.set("verdict",
            json::Value("Averaging " + std::to_string(records) + " record(s) and smoothing with a " +
                        std::to_string(window_length) + "-bin " + window +
                        " window cuts the estimate's scatter from " +
                        json::number_to_string(std::sqrt(raw_variance)) + " dB to " +
                        json::number_to_string(std::sqrt(smoothed_variance)) +
                        " dB RMS, and that is the whole of the good news. The price is bias: the " +
                        json::number_to_string(hz[peak_bin]) +
                        " Hz structural resonance of the arm reads " +
                        json::number_to_string(peak_loss) +
                        " dB lower than it is, and anything narrower than " +
                        json::number_to_string(resolution_hz) +
                        " Hz can no longer be resolved at all. Widen the window until the "
                        "resonance disappears and you have a beautifully smooth estimate of a "
                        "robot that does not exist."));
    return out;
}

// ---------------------------------------------------------------------------
// Sessions 5 and 8 - inverse transfer function and equalisation
// ---------------------------------------------------------------------------

template <typename Target>
[[nodiscard]] std::vector<double> frequency_sampled_fir(std::size_t order, double fs,
                                                        Target target) {
    if (order % 2 != 0) {
        ++order;
    }
    const std::size_t length = order + 1;
    const std::size_t m = order / 2;
    std::vector<Complex> spectrum(length, Complex(0.0, 0.0));
    for (std::size_t k = 0; k <= length / 2; ++k) {
        const double w = 2.0 * kPi * static_cast<double>(k) / static_cast<double>(length);
        const double gain = target(w * fs / (2.0 * kPi));
        const double phase = -w * static_cast<double>(m);
        const Complex value = gain * Complex(std::cos(phase), std::sin(phase));
        spectrum[k] = value;
        if (k != 0) {
            spectrum[length - k] = std::conj(value);
        }
    }
    const std::vector<double> raw = dsp::ifft_real(spectrum);
    std::vector<double> b(length, 0.0);
    for (std::size_t i = 0; i < length; ++i) {
        b[i] = 0.5 * (raw[i] + raw[length - 1 - i]);
    }
    return b;
}

[[nodiscard]] json::Value op_inverse_and_equalise(const json::Value& args) {
    const double fs = read_fs(args);
    const std::size_t joint = read_joint(args);
    const double epsilon = optional_scalar(args, "epsilon", 0.01, 0.0, 1.0);
    const double zero_radius = optional_scalar(args, "zero_radius", 1.2, 0.05, 3.0);
    const std::size_t order =
        static_cast<std::size_t>(optional_int(args, "equaliser_order", 48, 8, kMaxOrder));
    const double band_lo = optional_scalar(args, "band_lo", 1.0, 0.0, 0.49 * fs);
    const double band_hi = optional_scalar(args, "band_hi", 0.4 * fs, 1.0, 0.5 * fs);
    if (band_hi <= band_lo) {
        throw StudyError("parameter 'band_hi' = " + json::number_to_string(band_hi) +
                         " must be above 'band_lo' = " + json::number_to_string(band_lo));
    }

    const dsp::JointChannel channel = dsp::joint_channel(joint, fs);
    const double theta = 2.0 * kPi * channel.resonance_hz / fs;
    const std::vector<double> zero_pair = {1.0, -2.0 * zero_radius * std::cos(theta),
                                           zero_radius * zero_radius};
    std::vector<double> b_channel = dsp::convolve(channel.tf.b, zero_pair);
    const std::vector<double>& a_channel = channel.tf.a;
    const double dc = std::abs(dsp::frequency_response(b_channel, a_channel, 0.0));
    if (dc > 1e-12) {
        for (auto& v : b_channel) {
            v /= dc;
        }
    }

    const std::vector<Complex> channel_zeros = dsp::polynomial_roots(b_channel);
    double worst_zero = 0.0;
    for (const auto& z : channel_zeros) {
        worst_zero = std::max(worst_zero, std::abs(z));
    }
    const bool invertible = worst_zero < 1.0;

    const Grid grid = make_grid(fs, kResponsePoints);
    const auto regularised_gain = [&](double hz) {
        const double clamped = std::clamp(hz, band_lo, band_hi);
        const double w = 2.0 * kPi * clamped / fs;
        const double magnitude = std::abs(dsp::frequency_response(b_channel, a_channel, w));
        return magnitude / (magnitude * magnitude + epsilon);
    };
    const std::vector<double> b_equaliser = frequency_sampled_fir(order, fs, regularised_gain);

    const std::vector<double> b_cascade = dsp::convolve(b_equaliser, b_channel);
    const std::vector<double> channel_db = magnitude_db(b_channel, a_channel, grid);
    const std::vector<double> equaliser_db = magnitude_db(b_equaliser, {1.0}, grid);
    const std::vector<double> cascade_db = magnitude_db(b_cascade, a_channel, grid);
    std::vector<double> exact_inverse_db;
    exact_inverse_db.reserve(grid.hz.size());
    for (const double value : channel_db) {
        exact_inverse_db.push_back(-value);
    }

    double flat_min = 1e30;
    double flat_max = -1e30;
    double noise_gain_db = -1e30;
    for (std::size_t i = 0; i < grid.hz.size(); ++i) {
        if (grid.hz[i] < band_lo || grid.hz[i] > band_hi) {
            continue;
        }
        flat_min = std::min(flat_min, cascade_db[i]);
        flat_max = std::max(flat_max, cascade_db[i]);
        noise_gain_db = std::max(noise_gain_db, equaliser_db[i]);
    }
    if (flat_min > flat_max) {
        flat_min = 0.0;
        flat_max = 0.0;
        noise_gain_db = 0.0;
    }

    std::string action;
    if (!invertible) {
        action = "The channel has a zero at radius " + json::number_to_string(worst_zero) +
                 ", i.e. outside the unit circle, so 1/H(z) has a pole there: the exact inverse is "
                 "UNSTABLE as a causal filter and only exists as a non-causal (left-sided) "
                 "sequence. Nothing here pretends otherwise. What is built instead is a "
                 "least-squares FIR approximation of the Tikhonov-regularised inverse "
                 "conj(H)/(|H|^2 + eps), which is causal and stable by construction, matches the "
                 "magnitude and accepts a residual phase error. The textbook alternative - "
                 "reflecting the zero to 1/z* - would give the same magnitude with a stable "
                 "inverse and a different phase.";
    } else {
        action = "Every zero of the channel is inside the unit circle (largest radius " +
                 json::number_to_string(worst_zero) +
                 "), so the channel is minimum phase and its exact inverse A(z)/B(z) is stable and "
                 "causal. The FIR equaliser below is still the regularised inverse, because the "
                 "exact one amplifies the measurement noise wherever |H| is small.";
    }

    json::Value out = json::Value::object();
    out.set("b_channel", coefficient_matrix(b_channel));
    out.set("a_channel", coefficient_matrix(a_channel));
    out.set("equaliser_coefficients", coefficient_matrix(b_equaliser));
    out.set("magnitude_channel", json::from_series("channel |H|", grid.hz, channel_db));
    out.set("magnitude_exact_inverse", json::from_series("|1/H|", grid.hz, exact_inverse_db));
    out.set("magnitude_equaliser", json::from_series("FIR equaliser", grid.hz, equaliser_db));
    out.set("magnitude_cascade", json::from_series("equalised cascade", grid.hz, cascade_db));
    out.set("channel_zeros", complex_set(channel_zeros));
    out.set("unit_circle", unit_circle_series());
    out.set("max_zero_radius", json::Value(worst_zero));
    out.set("inverse_is_stable", json::Value(invertible));
    out.set("flatness_db", json::Value(flat_max - flat_min));
    out.set("noise_gain_db", json::Value(noise_gain_db));
    out.set("action_taken", json::Value(action));
    out.set("verdict",
            json::Value("Between " + json::number_to_string(band_lo) + " and " +
                        json::number_to_string(band_hi) + " Hz the equalised cascade is flat to " +
                        json::number_to_string(flat_max - flat_min) + " dB, with the equaliser "
                        "peaking at " + json::number_to_string(noise_gain_db) +
                        " dB - and that peak is also the factor by which the sensor noise floor is "
                        "lifted at the spectral null. Drive eps to 0 and the flatness improves "
                        "while the output SNR collapses; raise eps and the null is left "
                        "un-equalised but the measurement stays usable. The design parameter for "
                        "the GP8's joint-" + std::to_string(joint) +
                        " channel is exactly that trade."));
    return out;
}

// ---------------------------------------------------------------------------
// Sessions 9 and 10 - arbitrary-magnitude FIR design
// ---------------------------------------------------------------------------

[[nodiscard]] json::Value op_fir_arbitrary_magnitude(const json::Value& args) {
    const std::string method = optional_enum(args, "method", "least_squares", kFirMethods);
    const std::string specification =
        optional_enum(args, "specification", "servo_lowpass", kFirSpecifications);
    const std::string window = optional_enum(args, "window", "hamming", kWindowOptions);
    const double beta = optional_scalar(args, "kaiser_beta", 5.65, 0.0, 20.0);
    const std::size_t order = static_cast<std::size_t>(optional_int(args, "order", 64, 4, kMaxOrder));
    const double fs = read_fs(args);
    const std::size_t joint = read_joint(args);
    const double pass_edge = optional_scalar(args, "pass_edge", 0.06 * fs, 0.5, 0.49 * fs);
    const double stop_edge = optional_scalar(args, "stop_edge", 0.14 * fs, 1.0, 0.5 * fs);
    const double stopband_db = optional_scalar(args, "stopband_db", 40.0, 6.0, 120.0);
    const double passband_ripple_db = optional_scalar(args, "passband_ripple_db", 1.0, 0.01, 12.0);
    if (stop_edge <= pass_edge) {
        throw StudyError("parameter 'stop_edge' = " + json::number_to_string(stop_edge) +
                         " must be above 'pass_edge' = " + json::number_to_string(pass_edge));
    }

    const dsp::JointChannel channel = dsp::joint_channel(joint, fs);
    std::vector<BandSpec> bands =
        fir_specification(specification, fs, pass_edge, stop_edge, channel.resonance_hz, stopband_db);
    for (auto& band : bands) {
        band.tolerance_db = band.is_stop ? stopband_db : passband_ripple_db;
    }

    const std::vector<double> b = design_fir(method, window, beta, order, fs, bands);
    const std::vector<double> a = {1.0};
    const SpecCheck check = check_specification(b, a, fs, bands, 24);

    const Grid grid = make_grid(fs, kResponsePoints);
    std::vector<double> target_db;
    target_db.reserve(grid.hz.size());
    for (const double hz : grid.hz) {
        target_db.push_back(20.0 * std::log10(std::max(interpolated_gain(bands, hz), 1e-6)));
    }

    const std::size_t length = b.size();
    const double delay_samples = static_cast<double>(length - 1) / 2.0;

    json::Value out = json::Value::object();
    out.set("coefficients", coefficient_matrix(b));
    out.set("taps", json::Value(static_cast<double>(length)));
    out.set("magnitude_achieved", json::from_series("designed |H|", grid.hz, magnitude_db(b, a, grid)));
    out.set("magnitude_specification", json::from_series("specification", grid.hz, target_db));
    out.set("group_delay",
            json::from_series("group delay", grid.hz, group_delay_series(b, a, grid)));
    out.set("verification", check.table);
    out.set("passband_max_deviation_db", json::Value(check.passband_max_deviation_db));
    out.set("stopband_attenuation_db", json::Value(check.stopband_attenuation_db));
    out.set("meets_specification", json::Value(check.failed == 0));
    out.set("points_passed", json::Value(static_cast<double>(check.passed)));
    out.set("points_failed", json::Value(static_cast<double>(check.failed)));
    out.set("worst_margin_db", json::Value(check.worst_margin_db));
    out.set("group_delay_samples", json::Value(delay_samples));
    out.set("group_delay_ms", json::Value(1000.0 * delay_samples / fs));
    out.set("mac_per_second", json::Value(static_cast<double>(length) * fs));
    out.set("verdict",
            json::Value("Order " + std::to_string(length - 1) + " (" + std::to_string(length) +
                        " taps) by " + method + ": pass-band deviation " +
                        json::number_to_string(check.passband_max_deviation_db) + " dB against " +
                        json::number_to_string(passband_ripple_db) +
                        " dB allowed, stop-band attenuation " +
                        json::number_to_string(check.stopband_attenuation_db) + " dB against " +
                        json::number_to_string(stopband_db) + " dB required - " +
                        (check.failed == 0 ? "the specification is met at every checked point."
                                           : "FAILS at " + std::to_string(check.failed) +
                                                 " of the checked points.") +
                        " The cost is fixed and unavoidable: " +
                        json::number_to_string(delay_samples) + " samples = " +
                        json::number_to_string(1000.0 * delay_samples / fs) +
                        " ms of group delay inside the GP8's servo loop, and " +
                        json::number_to_string(static_cast<double>(length) * fs) +
                        " multiply-accumulates per second at this sample rate. A filter that "
                        "cleans the encoder-derived velocity beautifully and adds 30 ms of lag "
                        "will destabilise the loop it was meant to help."));
    return out;
}

// ---------------------------------------------------------------------------
// Session 11 - Prony's method
// ---------------------------------------------------------------------------

[[nodiscard]] json::Value op_iir_prony(const json::Value& args) {
    const std::size_t num_order =
        static_cast<std::size_t>(optional_int(args, "numerator_order", 2, 0, 12));
    const std::size_t den_order =
        static_cast<std::size_t>(optional_int(args, "denominator_order", 3, 1, 12));
    const std::size_t impulse_length =
        static_cast<std::size_t>(optional_int(args, "impulse_length", 128, 16, 1024));
    const double target_error = optional_scalar(args, "target_error", 0.02, 1e-6, 1.0);
    const double snr_db = optional_scalar(args, "snr_db", 200.0, 0.0, 200.0);
    const double fs = read_fs(args);
    const std::size_t joint = read_joint(args);
    const unsigned seed = read_seed(args);

    if (impulse_length < num_order + den_order + 2) {
        throw StudyError("parameter 'impulse_length' must exceed numerator_order + "
                         "denominator_order + 1 = " +
                         std::to_string(num_order + den_order + 1) + ", received " +
                         std::to_string(impulse_length));
    }

    const dsp::JointChannel channel = dsp::joint_channel(joint, fs);
    const std::vector<double> truth =
        dsp::impulse_response(channel.tf.b, channel.tf.a, impulse_length);
    const std::vector<double> measured = add_noise(truth, snr_db, seed + 31u);
    const dsp::PronyFit fit = dsp::prony(measured, num_order, den_order);

    std::size_t order_needed = 0;
    double order_needed_error = 0.0;
    for (std::size_t p = 1; p <= 12; ++p) {
        const std::size_t q = std::min(num_order, p);
        if (measured.size() < q + p + 2) {
            break;
        }
        const dsp::PronyFit trial = dsp::prony(measured, q, p);
        if (trial.relative_error <= target_error) {
            order_needed = p;
            order_needed_error = trial.relative_error;
            break;
        }
        order_needed_error = trial.relative_error;
    }

    // The FIR length that would be needed to hold the same relative error:
    // truncate the impulse response until the discarded tail is small enough.
    double total_energy = 0.0;
    for (const double v : truth) {
        total_energy += v * v;
    }
    std::size_t fir_taps = truth.size();
    double tail = 0.0;
    for (std::size_t i = truth.size(); i-- > 0;) {
        tail += truth[i] * truth[i];
        if (std::sqrt(tail / std::max(total_energy, 1e-30)) > target_error) {
            fir_taps = i + 2;
            break;
        }
    }

    const std::vector<double> fitted = dsp::impulse_response(fit.b, fit.a, impulse_length);
    std::vector<double> time;
    time.reserve(impulse_length);
    for (std::size_t i = 0; i < impulse_length; ++i) {
        time.push_back(static_cast<double>(i) / fs);
    }

    const Grid grid = make_grid(fs, kResponsePoints);
    const std::vector<Complex> poles = dsp::polynomial_roots(fit.a);
    const std::vector<Complex> zeros = dsp::polynomial_roots(fit.b);

    json::Value out = json::Value::object();
    out.set("b", coefficient_matrix(fit.b));
    out.set("a", coefficient_matrix(fit.a));
    out.set("a_true", coefficient_matrix(channel.tf.a));
    out.set("impulse_target", json::from_series("measured impulse response", time, measured));
    out.set("impulse_fitted", json::from_series("Prony fit", time, fitted));
    out.set("magnitude_true",
            json::from_series("true |H|", grid.hz, magnitude_db(channel.tf.b, channel.tf.a, grid)));
    out.set("magnitude_fitted",
            json::from_series("fitted |H|", grid.hz, magnitude_db(fit.b, fit.a, grid)));
    out.set("relative_error", json::Value(fit.relative_error));
    out.set("poles", complex_set(poles));
    out.set("zeros", complex_set(zeros));
    out.set("unit_circle", unit_circle_series());
    out.set("max_pole_radius", json::Value(fit.max_pole_radius));
    out.set("stable", json::Value(fit.stable));
    out.set("order_needed", json::Value(static_cast<double>(order_needed)));
    out.set("fir_taps_equivalent", json::Value(static_cast<double>(fir_taps)));
    out.set("verdict",
            json::Value("Prony fits the joint-" + std::to_string(joint) +
                        " impulse response with " + std::to_string(num_order + 1) +
                        " numerator and " + std::to_string(den_order) +
                        " denominator coefficients to a relative error of " +
                        json::number_to_string(fit.relative_error) + ". The poles sit at radius " +
                        json::number_to_string(fit.max_pole_radius) + ", so the model is " +
                        (fit.stable ? "stable" : "UNSTABLE - a pole has left the unit circle, "
                                                 "which is what an over-ordered fit to noisy data "
                                                 "does") +
                        ". Denominator order " + std::to_string(order_needed) +
                        " is the first that reaches the " + json::number_to_string(target_error) +
                        " target (best seen " + json::number_to_string(order_needed_error) +
                        "); an FIR would need about " + std::to_string(fir_taps) +
                        " taps for the same fidelity. That is the IIR bargain: a tenth of the "
                        "arithmetic, in exchange for a stability question and a group delay you "
                        "no longer control."));
    return out;
}

// ---------------------------------------------------------------------------
// Session 12 - parametric equaliser, realisation and word length
// ---------------------------------------------------------------------------

[[nodiscard]] json::Value op_parametric_equaliser(const json::Value& args) {
    const std::string type = optional_enum(args, "filter_type", "peaking", kEqualiserTypes);
    const std::string structure = optional_enum(args, "structure", "direct_form_1", kStructures);
    const double fs = read_fs(args);
    const std::size_t joint = read_joint(args);
    const dsp::JointChannel channel = dsp::joint_channel(joint, fs);
    const double f0 = optional_scalar(args, "f0", channel.resonance_hz, 0.5, 0.49 * fs);
    const double q_factor = optional_scalar(args, "q", 4.0, 0.1, 40.0);
    const double gain_db = optional_scalar(args, "gain_db", -12.0, -40.0, 40.0);
    const int bits = optional_int(args, "bits", 16, 4, 32);
    const std::size_t n = read_length(args, 512);

    const double w0 = 2.0 * kPi * f0 / fs;
    const double cosw = std::cos(w0);
    const double alpha = std::sin(w0) / (2.0 * q_factor);
    const double amp = std::pow(10.0, gain_db / 40.0);
    std::vector<double> b(3, 0.0);
    std::vector<double> a(3, 0.0);
    if (type == "peaking") {
        b = {1.0 + alpha * amp, -2.0 * cosw, 1.0 - alpha * amp};
        a = {1.0 + alpha / amp, -2.0 * cosw, 1.0 - alpha / amp};
    } else if (type == "notch") {
        b = {1.0, -2.0 * cosw, 1.0};
        a = {1.0 + alpha, -2.0 * cosw, 1.0 - alpha};
    } else if (type == "low_shelf") {
        const double beta = 2.0 * std::sqrt(amp) * alpha;
        b = {amp * ((amp + 1.0) - (amp - 1.0) * cosw + beta),
             2.0 * amp * ((amp - 1.0) - (amp + 1.0) * cosw),
             amp * ((amp + 1.0) - (amp - 1.0) * cosw - beta)};
        a = {(amp + 1.0) + (amp - 1.0) * cosw + beta,
             -2.0 * ((amp - 1.0) + (amp + 1.0) * cosw),
             (amp + 1.0) + (amp - 1.0) * cosw - beta};
    } else {
        const double beta = 2.0 * std::sqrt(amp) * alpha;
        b = {amp * ((amp + 1.0) + (amp - 1.0) * cosw + beta),
             -2.0 * amp * ((amp - 1.0) + (amp + 1.0) * cosw),
             amp * ((amp + 1.0) + (amp - 1.0) * cosw - beta)};
        a = {(amp + 1.0) - (amp - 1.0) * cosw + beta,
             2.0 * ((amp - 1.0) - (amp + 1.0) * cosw),
             (amp + 1.0) - (amp - 1.0) * cosw - beta};
    }
    const double a0 = a[0];
    for (auto& v : b) {
        v /= a0;
    }
    for (auto& v : a) {
        v /= a0;
    }

    const Quantised qb = quantise_coefficients(b, bits);
    const Quantised qa = quantise_coefficients(a, bits);
    const std::vector<Complex> poles = dsp::polynomial_roots(a);
    const std::vector<Complex> poles_q = dsp::polynomial_roots(qa.values);
    double radius = 0.0;
    double radius_q = 0.0;
    for (const auto& p : poles) {
        radius = std::max(radius, std::abs(p));
    }
    for (const auto& p : poles_q) {
        radius_q = std::max(radius_q, std::abs(p));
    }
    double displacement = 0.0;
    for (std::size_t i = 0; i < poles.size() && i < poles_q.size(); ++i) {
        displacement = std::max(displacement, std::abs(poles[i] - poles_q[i]));
    }

    const Grid grid = make_grid(fs, kResponsePoints);
    const std::vector<double> ideal_db = magnitude_db(b, a, grid);
    const std::vector<double> quantised_db = magnitude_db(qb.values, qa.values, grid);
    double max_deviation = 0.0;
    for (std::size_t i = 0; i < ideal_db.size(); ++i) {
        max_deviation = std::max(max_deviation, std::abs(quantised_db[i] - ideal_db[i]));
    }

    // Round-off noise gain of the realisation: where the rounding happens
    // decides what shapes it. Direct form I rounds once at the output, direct
    // form II rounds at the recursive node (shaped by the whole H), the
    // transposed form rounds into 1/A, and a cascade splits the shaping.
    double noise_power = 0.0;
    std::string structure_note;
    for (const double w : grid.omega) {
        double gain = 1.0;
        if (structure == "direct_form_1") {
            gain = 1.0;
        } else if (structure == "direct_form_2") {
            gain = std::abs(dsp::frequency_response(b, a, w));
        } else if (structure == "transposed_direct_form_2") {
            gain = std::abs(dsp::frequency_response({1.0}, a, w));
        } else {
            const double first = std::abs(dsp::frequency_response({1.0}, a, w));
            gain = std::sqrt(first * first + 1.0);
        }
        noise_power += gain * gain;
    }
    noise_power /= static_cast<double>(grid.omega.size());
    const double noise_gain_db = 10.0 * std::log10(std::max(noise_power, 1e-30));
    if (structure == "direct_form_1") {
        structure_note =
            "Direct form I keeps both delay lines separate and accumulates in one wide "
            "accumulator, so there is a single rounding at the output: no internal shaping, the "
            "largest state storage, and the most robust behaviour at a short word length.";
    } else if (structure == "direct_form_2") {
        structure_note =
            "Direct form II halves the delay elements by sharing them, but the shared node carries "
            "the recursive signal: its rounding noise is filtered by the whole of H(z), and its "
            "internal signal can overflow even when input and output are both in range.";
    } else if (structure == "transposed_direct_form_2") {
        structure_note =
            "The transposed form has the same transfer function and a different noise path: the "
            "rounding is shaped by 1/A(z) alone, which is why it usually wins for a high-Q section "
            "where the numerator would otherwise amplify the noise at the resonance.";
    } else {
        structure_note =
            "A cascade of sections lets each section be scaled separately, so a high-Q biquad can "
            "be split and the internal signal kept inside the word length. The pole pair is also "
            "less sensitive, which is why a 1 kHz servo chain on short coefficients is built as "
            "cascaded second-order sections and not as one high-order direct form.";
    }

    const std::vector<double> burst = velocity_burst(channel, n);
    const std::vector<double> filtered = dsp::apply_filter(qb.values, qa.values, burst);
    std::vector<double> time;
    time.reserve(n);
    for (std::size_t i = 0; i < n; ++i) {
        time.push_back(static_cast<double>(i) / fs);
    }
    std::vector<double> ring_before(burst.begin() + static_cast<std::ptrdiff_t>(n / 2), burst.end());
    std::vector<double> ring_after(filtered.begin() + static_cast<std::ptrdiff_t>(n / 2),
                                   filtered.end());

    std::vector<std::vector<json::Value>> rows;
    for (std::size_t i = 0; i < 3; ++i) {
        rows.push_back({json::Value("b" + std::to_string(i)), json::Value(b[i]),
                        json::Value(qb.values[i]), json::Value(qb.values[i] - b[i])});
        rows.push_back({json::Value("a" + std::to_string(i)), json::Value(a[i]),
                        json::Value(qa.values[i]), json::Value(qa.values[i] - a[i])});
    }

    json::Value out = json::Value::object();
    out.set("b", coefficient_matrix(b));
    out.set("a", coefficient_matrix(a));
    out.set("b_quantised", coefficient_matrix(qb.values));
    out.set("a_quantised", coefficient_matrix(qa.values));
    out.set("coefficient_table",
            json::from_table({"coefficient", "ideal", "quantised", "error"}, rows));
    out.set("magnitude_ideal", json::from_series("ideal |H|", grid.hz, ideal_db));
    out.set("magnitude_quantised",
            json::from_series("Q1." + std::to_string(bits - 1) + " |H|", grid.hz, quantised_db));
    out.set("max_deviation_db", json::Value(max_deviation));
    out.set("poles_ideal", complex_set(poles));
    out.set("poles_quantised", complex_set(poles_q));
    out.set("unit_circle", unit_circle_series());
    out.set("pole_radius_ideal", json::Value(radius));
    out.set("pole_radius_quantised", json::Value(radius_q));
    out.set("pole_displacement", json::Value(displacement));
    out.set("overflow_count",
            json::Value(static_cast<double>(qb.overflow_count + qa.overflow_count)));
    out.set("quantisation_step", json::Value(qb.step));
    out.set("largest_coefficient", json::Value(std::max(qb.max_abs_requested, qa.max_abs_requested)));
    out.set("stable_after_quantisation", json::Value(radius_q < 1.0));
    out.set("noise_gain_db", json::Value(noise_gain_db));
    out.set("structure", json::Value(structure));
    out.set("structure_note", json::Value(structure_note));
    out.set("vibration_before", json::from_series("joint velocity, equaliser off", time, burst));
    out.set("vibration_after", json::from_series("joint velocity, equaliser on", time, filtered));
    out.set("ringing_rms_before", json::Value(rms(ring_before)));
    out.set("ringing_rms_after", json::Value(rms(ring_after)));
    out.set("verdict",
            json::Value("A " + type + " section at " + json::number_to_string(f0) + " Hz, Q = " +
                        json::number_to_string(q_factor) + ", " + json::number_to_string(gain_db) +
                        " dB, realised as " + structure + " in Q1." + std::to_string(bits - 1) +
                        ". The largest coefficient is " +
                        json::number_to_string(std::max(qb.max_abs_requested,
                                                        qa.max_abs_requested)) +
                        " against a representable maximum of " +
                        json::number_to_string(1.0 - qb.step) + ", so " +
                        std::to_string(qb.overflow_count + qa.overflow_count) +
                        " coefficient(s) overflow and had to be saturated. Quantisation moves the "
                        "poles by " + json::number_to_string(displacement) +
                        " (radius " + json::number_to_string(radius) + " to " +
                        json::number_to_string(radius_q) + ", " +
                        (radius_q < 1.0 ? "still inside the unit circle"
                                        : "OUTSIDE the unit circle - the realised filter is "
                                          "unstable while the design is not") +
                        ") and changes the response by up to " +
                        json::number_to_string(max_deviation) +
                        " dB. Round-off noise gain of this realisation: " +
                        json::number_to_string(noise_gain_db) + " dB. " + structure_note +
                        " Applied to the joint-" + std::to_string(joint) +
                        " velocity trace, the ring-down after the move falls from " +
                        json::number_to_string(rms(ring_before)) + " to " +
                        json::number_to_string(rms(ring_after)) +
                        " rad/s RMS without touching a controller gain."));
    return out;
}

// ---------------------------------------------------------------------------
// Session 13 - FIR phase corrector
// ---------------------------------------------------------------------------

[[nodiscard]] json::Value op_fir_phase_corrector(const json::Value& args) {
    const std::size_t order = static_cast<std::size_t>(optional_int(args, "order", 64, 8, kMaxOrder));
    const double fs = read_fs(args);
    const std::size_t joint = read_joint(args);
    const dsp::JointChannel channel = dsp::joint_channel(joint, fs);
    const double band_lo = optional_scalar(args, "band_lo", 1.0, 0.0, 0.49 * fs);
    // By default the band stops just below the axis resonance: that is the
    // servo band the loop works in, and it is where a finite FIR can actually
    // flatten the delay. Inside the resonance the delay swings by hundreds of
    // samples and no order-64 corrector can follow it.
    const double band_hi = optional_scalar(
        args, "band_hi", std::min(0.8 * channel.resonance_hz, 0.25 * fs), 1.0, 0.5 * fs);
    const double requested_delay = optional_scalar(args, "target_delay", 0.0, 0.0, 400.0);
    if (band_hi <= band_lo) {
        throw StudyError("parameter 'band_hi' must be above 'band_lo'");
    }

    const Grid grid = make_grid(fs, kResponsePoints);

    // The fit grid is dense inside the band and independent of the display
    // grid: a narrow band would otherwise leave fewer equations than unknowns.
    const std::size_t fit_points = 8 * (order + 1);
    std::vector<double> band_omega;
    band_omega.reserve(fit_points);
    for (std::size_t i = 0; i < fit_points; ++i) {
        const double hz = band_lo + (band_hi - band_lo) * static_cast<double>(i) /
                                        static_cast<double>(fit_points - 1);
        band_omega.push_back(2.0 * kPi * hz / fs);
    }

    double channel_mean_delay = 0.0;
    for (const double w : band_omega) {
        channel_mean_delay += dsp::group_delay(channel.tf.b, channel.tf.a, w);
    }
    channel_mean_delay /= static_cast<double>(band_omega.size());
    // The corrector can only add delay between 0 and its own order, so the
    // target has to sit where a causal FIR can reach it.
    const double target_delay = (requested_delay > 0.0)
                                    ? requested_delay
                                    : channel_mean_delay + static_cast<double>(order) / 2.0;

    // Least squares on the complex response: the corrector should have unit
    // magnitude and whatever phase makes the cascade's total phase linear.
    Eigen::MatrixXd A(static_cast<Eigen::Index>(2 * band_omega.size()),
                      static_cast<Eigen::Index>(order + 1));
    Eigen::VectorXd rhs(static_cast<Eigen::Index>(2 * band_omega.size()));
    for (std::size_t i = 0; i < band_omega.size(); ++i) {
        const double w = band_omega[i];
        const double channel_phase = std::arg(dsp::frequency_response(channel.tf.b, channel.tf.a, w));
        const double wanted = -(w * target_delay) - channel_phase;
        for (std::size_t k = 0; k <= order; ++k) {
            A(static_cast<Eigen::Index>(2 * i), static_cast<Eigen::Index>(k)) =
                std::cos(w * static_cast<double>(k));
            A(static_cast<Eigen::Index>(2 * i + 1), static_cast<Eigen::Index>(k)) =
                -std::sin(w * static_cast<double>(k));
        }
        rhs(static_cast<Eigen::Index>(2 * i)) = std::cos(wanted);
        rhs(static_cast<Eigen::Index>(2 * i + 1)) = std::sin(wanted);
    }
    const Eigen::VectorXd solution = dsp::least_squares(A, rhs);
    std::vector<double> c(order + 1, 0.0);
    for (std::size_t k = 0; k <= order; ++k) {
        c[k] = solution(static_cast<Eigen::Index>(k));
    }

    const std::vector<double> b_cascade = dsp::convolve(c, channel.tf.b);
    const std::vector<double> before = group_delay_series(channel.tf.b, channel.tf.a, grid);
    const std::vector<double> after = group_delay_series(b_cascade, channel.tf.a, grid);
    const std::vector<double> corrector_db = magnitude_db(c, {1.0}, grid);

    // Metrics on the dense in-band grid, not on the sparse display grid.
    double before_min = 1e30;
    double before_max = -1e30;
    double after_min = 1e30;
    double after_max = -1e30;
    double magnitude_error = 0.0;
    for (const double w : band_omega) {
        const double raw = dsp::group_delay(channel.tf.b, channel.tf.a, w);
        const double corrected = dsp::group_delay(b_cascade, channel.tf.a, w);
        before_min = std::min(before_min, raw);
        before_max = std::max(before_max, raw);
        after_min = std::min(after_min, corrected);
        after_max = std::max(after_max, corrected);
        magnitude_error =
            std::max(magnitude_error,
                     std::abs(20.0 * std::log10(std::max(
                                         std::abs(dsp::frequency_response(c, {1.0}, w)), 1e-12))));
    }

    json::Value out = json::Value::object();
    out.set("coefficients", coefficient_matrix(c));
    out.set("group_delay_before", json::from_series("channel group delay", grid.hz, before));
    out.set("group_delay_after", json::from_series("corrected cascade", grid.hz, after));
    out.set("corrector_magnitude", json::from_series("|C|", grid.hz, corrector_db));
    out.set("delay_ripple_before", json::Value(before_max - before_min));
    out.set("delay_ripple_after", json::Value(after_max - after_min));
    out.set("target_delay_samples", json::Value(target_delay));
    out.set("magnitude_error_db", json::Value(magnitude_error));
    out.set("verdict",
            json::Value("Across " + json::number_to_string(band_lo) + " to " +
                        json::number_to_string(band_hi) +
                        " Hz the joint-" + std::to_string(joint) +
                        " channel's group delay swings by " +
                        json::number_to_string(before_max - before_min) +
                        " samples; the order-" + std::to_string(order) +
                        " FIR corrector flattens it to " +
                        json::number_to_string(after_max - after_min) + " samples about " +
                        json::number_to_string(target_delay) +
                        ". An FIR corrector is only approximately all-pass: this one disturbs the "
                        "magnitude by up to " + json::number_to_string(magnitude_error) +
                        " dB in the band, which is the honest price of fixing the phase with a "
                        "finite, causal, stable filter. Note that the correction is bought with "
                        "delay, not for free."));
    return out;
}

// ---------------------------------------------------------------------------
// Sessions 13 and 14 - the Hilbert transform and quadrature processing
// ---------------------------------------------------------------------------

[[nodiscard]] json::Value op_hilbert_transform(const json::Value& args) {
    const std::string method = optional_enum(args, "method", "fft_analytic", kHilbertMethods);
    const std::string signal = optional_enum(args, "signal", "ringdown", kVibrationSignals);
    const std::size_t kernel_length =
        static_cast<std::size_t>(optional_int(args, "kernel_length", 101, 11, 501));
    const std::size_t n = read_length(args, 1024);
    const double fs = read_fs(args);
    const std::size_t joint = read_joint(args);
    const double snr_db = optional_scalar(args, "snr_db", 200.0, 0.0, 200.0);
    const double modulation_hz = optional_scalar(args, "modulation_hz", 23.0, 0.5, 200.0);
    const unsigned seed = read_seed(args);

    if (kernel_length % 2 == 0) {
        throw StudyError("parameter 'kernel_length' must be odd, received " +
                         std::to_string(kernel_length));
    }
    if (kernel_length + 8 >= n) {
        throw StudyError("parameter 'kernel_length' = " + std::to_string(kernel_length) +
                         " needs a record longer than " + std::to_string(kernel_length + 8) +
                         " samples, received length " + std::to_string(n));
    }

    const dsp::JointChannel channel = dsp::joint_channel(joint, fs);
    const double f_res = channel.resonance_hz;
    const double decay = 1.0 / std::max(1e-6, channel.damping_ratio * 2.0 * kPi * f_res);
    std::vector<double> x(n, 0.0);
    for (std::size_t i = 0; i < n; ++i) {
        const double t = static_cast<double>(i) / fs;
        if (signal == "ringdown") {
            x[i] = std::exp(-t / decay) * std::sin(2.0 * kPi * f_res * t);
        } else if (signal == "swept_motion") {
            const double duration = static_cast<double>(n) / fs;
            const double f_start = 0.2 * f_res;
            const double f_end = std::min(1.8 * f_res, 0.45 * fs);
            const double rate = (f_end - f_start) / duration;
            x[i] = std::sin(2.0 * kPi * (f_start * t + 0.5 * rate * t * t));
        } else {
            x[i] = (1.0 + 0.6 * std::sin(2.0 * kPi * modulation_hz * t)) *
                   std::sin(2.0 * kPi * f_res * t);
        }
    }
    x = add_noise(x, snr_db, seed + 41u);

    std::vector<double> in_phase;
    std::vector<double> quadrature;
    std::size_t offset = 0;
    std::vector<double> kernel;
    if (method == "fir_kernel") {
        kernel = dsp::hilbert_fir_kernel(kernel_length);
        const std::size_t m = kernel_length / 2;
        const std::vector<double> convolved = dsp::apply_filter(kernel, {1.0}, x);
        in_phase.reserve(n - 2 * m);
        quadrature.reserve(n - 2 * m);
        for (std::size_t i = m; i + m < n; ++i) {
            in_phase.push_back(x[i - m]);
            quadrature.push_back(convolved[i]);
        }
        offset = m;
    } else {
        quadrature = dsp::hilbert_fft(x);
        in_phase = x;
    }

    const std::size_t count = in_phase.size();
    std::vector<double> envelope(count, 0.0);
    std::vector<double> phase(count, 0.0);
    for (std::size_t i = 0; i < count; ++i) {
        envelope[i] = std::sqrt(in_phase[i] * in_phase[i] + quadrature[i] * quadrature[i]);
        phase[i] = std::atan2(quadrature[i], in_phase[i]);
    }
    std::vector<double> instantaneous(count, 0.0);
    for (std::size_t i = 1; i < count; ++i) {
        double delta = phase[i] - phase[i - 1];
        while (delta > kPi) {
            delta -= 2.0 * kPi;
        }
        while (delta < -kPi) {
            delta += 2.0 * kPi;
        }
        instantaneous[i] = delta * fs / (2.0 * kPi);
    }
    if (count > 1) {
        instantaneous[0] = instantaneous[1];
    }

    // Envelope spectrum: a modulation that is invisible in the raw spectrum of
    // a bearing-fault signature shows up here as a line at the fault rate.
    double mean_envelope = 0.0;
    for (const double v : envelope) {
        mean_envelope += v;
    }
    mean_envelope /= static_cast<double>(std::max<std::size_t>(1, count));
    std::vector<double> centred(count, 0.0);
    for (std::size_t i = 0; i < count; ++i) {
        centred[i] = envelope[i] - mean_envelope;
    }
    const std::vector<double> envelope_power = power_spectrum(centred);
    std::vector<double> envelope_hz;
    std::vector<double> envelope_db;
    envelope_hz.reserve(envelope_power.size());
    envelope_db.reserve(envelope_power.size());
    std::size_t peak_bin = 1;
    for (std::size_t k = 0; k < envelope_power.size(); ++k) {
        envelope_hz.push_back(fs * static_cast<double>(k) / static_cast<double>(count));
        envelope_db.push_back(10.0 * std::log10(std::max(envelope_power[k], 1e-20)));
        if (k >= 1 && k + 1 < envelope_power.size() && envelope_power[k] > envelope_power[peak_bin]) {
            peak_bin = k;
        }
    }
    const double detected = envelope_hz[peak_bin];

    // Quadrature quality: the pair must sit 90 degrees apart. Measured as the
    // normalised inner product over the interior of the record.
    double dot = 0.0;
    double energy_i = 0.0;
    double energy_q = 0.0;
    const std::size_t guard = std::min<std::size_t>(count / 8, 64);
    for (std::size_t i = guard; i + guard < count; ++i) {
        dot += in_phase[i] * quadrature[i];
        energy_i += in_phase[i] * in_phase[i];
        energy_q += quadrature[i] * quadrature[i];
    }
    const double orthogonality = dot / std::sqrt(std::max(energy_i * energy_q, 1e-30));
    const double amplitude_balance = std::sqrt(energy_q / std::max(energy_i, 1e-30));

    std::vector<double> time;
    time.reserve(count);
    for (std::size_t i = 0; i < count; ++i) {
        time.push_back(static_cast<double>(i + offset) / fs);
    }

    json::Value out = json::Value::object();
    out.set("signal", json::from_series("in-phase vibration channel", time, in_phase));
    out.set("quadrature", json::from_series("quadrature (Hilbert) channel", time, quadrature));
    out.set("envelope", json::from_series("envelope", time, envelope));
    out.set("instantaneous_frequency",
            json::from_series("instantaneous frequency", time, instantaneous));
    out.set("envelope_spectrum", json::from_series("envelope spectrum", envelope_hz, envelope_db));
    out.set("kernel", kernel.empty() ? json::Value::array() : coefficient_matrix(kernel));
    out.set("carrier_hz", json::Value(f_res));
    out.set("detected_modulation_hz", json::Value(detected));
    out.set("orthogonality", json::Value(orthogonality));
    out.set("amplitude_balance", json::Value(amplitude_balance));
    out.set("method", json::Value(method));
    out.set("verdict",
            json::Value("The analytic signal of the " + signal + " on axis " +
                        std::to_string(joint) + " (carrier at the " +
                        json::number_to_string(f_res) +
                        " Hz structural mode) was built by " + method +
                        ". The in-phase and quadrature channels are orthogonal to " +
                        json::number_to_string(orthogonality) +
                        " (0 is a perfect 90 degrees) with an amplitude balance of " +
                        json::number_to_string(amplitude_balance) +
                        " (1 is perfect), and the envelope peaks in its own spectrum at " +
                        json::number_to_string(detected) +
                        " Hz. That line is the modulation rate: in a bearing-fault signature it is "
                        "invisible in the raw spectrum, where all the energy sits at the carrier, "
                        "and obvious in the envelope. Outside the band the kernel was designed "
                        "for, the pair stops being 90 degrees apart and the instantaneous "
                        "frequency becomes meaningless rather than merely noisy."));
    return out;
}

// ---------------------------------------------------------------------------
// Session 15 - IIR all-pass phase correction
// ---------------------------------------------------------------------------

[[nodiscard]] json::Value op_iir_allpass_corrector(const json::Value& args) {
    const std::size_t sections = static_cast<std::size_t>(optional_int(args, "sections", 2, 1, 6));
    const double fs = read_fs(args);
    const std::size_t joint = read_joint(args);
    const dsp::JointChannel channel = dsp::joint_channel(joint, fs);
    const double angle =
        optional_scalar(args, "pole_angle", 2.0 * kPi * channel.resonance_hz / fs, 0.0, kPi);
    const double sensor_cutoff = optional_scalar(args, "sensor_cutoff", 20.0, 1.0, 0.49 * fs);
    const double band_hi = optional_scalar(args, "band_hi", 0.25 * fs, 2.0, 0.5 * fs);

    const Grid grid = make_grid(fs, kResponsePoints);

    // The force-torque channel is the same mechanical path seen through the
    // sensor's own first-order anti-alias filter, so it arrives later than the
    // encoder channel. That lag is what the all-pass network has to match.
    const double sensor_pole = std::exp(-2.0 * kPi * sensor_cutoff / fs);
    const std::vector<double> ft_full_b = dsp::convolve(channel.tf.b, {1.0 - sensor_pole});
    const std::vector<double> ft_full_a = dsp::convolve(channel.tf.a, {1.0, -sensor_pole});

    const auto build_cascade = [&](double r) {
        std::vector<double> num = {1.0};
        std::vector<double> den = {1.0};
        for (std::size_t s = 0; s < sections; ++s) {
            const double spread = 1.0 + 0.2 * static_cast<double>(s);
            const double theta = std::clamp(angle * spread, 0.0, 0.98 * kPi);
            const double c = -2.0 * r * std::cos(theta);
            num = dsp::convolve(num, {r * r, c, 1.0});
            den = dsp::convolve(den, {1.0, c, r * r});
        }
        return std::pair<std::vector<double>, std::vector<double>>{num, den};
    };

    // Worst-case misalignment between the two channels over the band, for a
    // given pole radius.
    const auto misalignment = [&](const std::vector<double>& num, const std::vector<double>& den) {
        const std::vector<double> encoder_b = dsp::convolve(channel.tf.b, num);
        const std::vector<double> encoder_a = dsp::convolve(channel.tf.a, den);
        double worst = 0.0;
        for (std::size_t i = 0; i < grid.hz.size(); ++i) {
            if (grid.hz[i] > band_hi) {
                continue;
            }
            worst = std::max(worst, std::abs(dsp::group_delay(ft_full_b, ft_full_a, grid.omega[i]) -
                                             dsp::group_delay(encoder_b, encoder_a, grid.omega[i])));
        }
        return worst;
    };

    // If the caller fixes the pole radius, it is used. If not, the radius that
    // aligns the two channels best is searched for, because that is the design
    // question an all-pass network actually answers.
    double radius = optional_scalar(args, "pole_radius", 0.0, 0.0, 0.95);
    if (radius <= 0.0) {
        double best = 1e30;
        radius = 0.05;
        for (int step = 1; step <= 46; ++step) {
            const double candidate = 0.02 * static_cast<double>(step);
            const auto [num, den] = build_cascade(candidate);
            const double score = misalignment(num, den);
            if (score < best) {
                best = score;
                radius = candidate;
            }
        }
    }
    const auto [b_all, a_all] = build_cascade(radius);
    const std::vector<double> allpass_db = magnitude_db(b_all, a_all, grid);
    double max_magnitude_deviation = 0.0;
    for (const double value : allpass_db) {
        max_magnitude_deviation = std::max(max_magnitude_deviation, std::abs(value));
    }

    const std::vector<double> b_corrected = dsp::convolve(channel.tf.b, b_all);
    const std::vector<double> a_corrected = dsp::convolve(channel.tf.a, a_all);
    const std::vector<double> channel_db = magnitude_db(channel.tf.b, channel.tf.a, grid);
    const std::vector<double> corrected_db = magnitude_db(b_corrected, a_corrected, grid);
    double max_channel_change = 0.0;
    for (std::size_t i = 0; i < channel_db.size(); ++i) {
        max_channel_change = std::max(max_channel_change, std::abs(corrected_db[i] - channel_db[i]));
    }

    const std::vector<double> delay_before = group_delay_series(channel.tf.b, channel.tf.a, grid);
    const std::vector<double> delay_after = group_delay_series(b_corrected, a_corrected, grid);
    std::vector<double> added(grid.hz.size(), 0.0);
    for (std::size_t i = 0; i < grid.hz.size(); ++i) {
        added[i] = delay_after[i] - delay_before[i];
    }

    double misaligned_before = 0.0;
    double misaligned_after = 0.0;
    double mean_added = 0.0;
    std::size_t counted = 0;
    for (std::size_t i = 0; i < grid.hz.size(); ++i) {
        if (grid.hz[i] > band_hi) {
            continue;
        }
        const double encoder = delay_before[i];
        const double encoder_corrected = delay_after[i];
        const double force_torque = dsp::group_delay(ft_full_b, ft_full_a, grid.omega[i]);
        misaligned_before = std::max(misaligned_before, std::abs(force_torque - encoder));
        misaligned_after = std::max(misaligned_after, std::abs(force_torque - encoder_corrected));
        mean_added += added[i];
        ++counted;
    }
    mean_added /= static_cast<double>(std::max<std::size_t>(1, counted));

    json::Value out = json::Value::object();
    out.set("b", coefficient_matrix(b_all));
    out.set("a", coefficient_matrix(a_all));
    out.set("magnitude_allpass", json::from_series("|A| of the all-pass cascade", grid.hz, allpass_db));
    out.set("magnitude_channel", json::from_series("channel alone", grid.hz, channel_db));
    out.set("magnitude_corrected", json::from_series("channel + all-pass", grid.hz, corrected_db));
    out.set("max_magnitude_deviation_db", json::Value(max_magnitude_deviation));
    out.set("max_channel_change_db", json::Value(max_channel_change));
    out.set("group_delay_before", json::from_series("encoder channel", grid.hz, delay_before));
    out.set("group_delay_after", json::from_series("encoder channel + all-pass", grid.hz, delay_after));
    out.set("added_group_delay", json::from_series("delay added", grid.hz, added));
    out.set("mean_added_delay_samples", json::Value(mean_added));
    out.set("poles", complex_set(dsp::polynomial_roots(a_all)));
    out.set("zeros", complex_set(dsp::polynomial_roots(b_all)));
    out.set("unit_circle", unit_circle_series());
    out.set("misalignment_before_samples", json::Value(misaligned_before));
    out.set("misalignment_after_samples", json::Value(misaligned_after));
    out.set("verdict",
            json::Value(std::to_string(sections) +
                        " second-order all-pass section(s) at pole radius " +
                        json::number_to_string(radius) +
                        " change the magnitude of the joint-" + std::to_string(joint) +
                        " channel by at most " + json::number_to_string(max_channel_change) +
                        " dB - the cascade's own magnitude is flat to " +
                        json::number_to_string(max_magnitude_deviation) +
                        " dB, which is numerically exact because the numerator is the reversed "
                        "denominator - while adding a mean " +
                        json::number_to_string(mean_added) +
                        " samples of group delay below " + json::number_to_string(band_hi) +
                        " Hz. Used to line the encoder channel up with the force-torque channel, "
                        "the worst misalignment falls from " +
                        json::number_to_string(misaligned_before) + " to " +
                        json::number_to_string(misaligned_after) +
                        " samples. This is what an all-pass network is for: it buys phase with "
                        "delay and never touches the amplitude response."));
    return out;
}

// ---------------------------------------------------------------------------
// Session 16 - synthesis to an amplitude specification
// ---------------------------------------------------------------------------

[[nodiscard]] std::vector<BandSpec> synthesis_specification(const std::string& name, double& fs) {
    std::vector<BandSpec> bands;
    if (name == "antialias_exam") {
        // The mock paper's Q2: fs = 8 kHz, pass 0-1.2 kHz at 0.5 dB,
        // stop above 2.0 kHz at 60 dB.
        fs = 8000.0;
        bands.push_back({0.0, 1200.0, 1.0, 1.0, 0.5, false});
        bands.push_back({2000.0, 4000.0, 0.0, 20.0, 60.0, true});
    } else if (name == "bandpass_part_a") {
        // Part A of the examination: a 200-800 Hz band-pass at fs = 4 kHz.
        fs = 4000.0;
        bands.push_back({0.0, 100.0, 0.0, 10.0, 40.0, true});
        bands.push_back({200.0, 800.0, 1.0, 1.0, 1.0, false});
        bands.push_back({1200.0, 2000.0, 0.0, 10.0, 40.0, true});
    } else {
        // The GP8's own anti-vibration specification at the 1 kHz servo rate.
        fs = dsp::kGp8ControlRateHz;
        bands.push_back({0.0, 60.0, 1.0, 1.0, 1.0, false});
        bands.push_back({140.0, 500.0, 0.0, 20.0, 45.0, true});
    }
    return bands;
}

[[nodiscard]] json::Value op_synthesise_to_specification(const json::Value& args) {
    const std::string specification =
        optional_enum(args, "specification", "antialias_exam", kSynthesisSpecs);
    const std::string family = optional_enum(args, "family", "fir_windowed", kSynthesisFamilies);
    const std::string window = optional_enum(args, "window", "blackman", kWindowOptions);
    const double beta = optional_scalar(args, "kaiser_beta", 5.65, 0.0, 20.0);
    const std::size_t order = static_cast<std::size_t>(optional_int(args, "order", 72, 4, kMaxOrder));
    const int bits = optional_int(args, "bits", 16, 4, 32);

    double fs = dsp::kGp8ControlRateHz;
    const std::vector<BandSpec> bands = synthesis_specification(specification, fs);

    const auto design = [&](std::size_t trial_order) {
        dsp::Tf tf;
        if (family == "iir_prony") {
            const std::vector<double> reference =
                design_fir("least_squares", window, beta, std::max<std::size_t>(trial_order, 96),
                           fs, bands);
            const std::size_t taps = std::min<std::size_t>(reference.size(), 160);
            std::vector<double> h(reference.begin(),
                                  reference.begin() + static_cast<std::ptrdiff_t>(taps));
            const std::size_t model_order = std::clamp<std::size_t>(trial_order / 8, 2, 12);
            const dsp::PronyFit fit = dsp::prony(h, model_order, model_order);
            tf.b = fit.b;
            tf.a = fit.a;
        } else {
            const std::string method =
                (family == "fir_windowed") ? "windowed" : "least_squares";
            tf.b = design_fir(method, window, beta, trial_order, fs, bands);
            tf.a = {1.0};
        }
        return tf;
    };

    const dsp::Tf tf = design(order);
    const SpecCheck check = check_specification(tf.b, tf.a, fs, bands, 24);

    std::size_t minimum_order = 0;
    for (std::size_t trial = 8; trial <= 192; trial += 8) {
        const dsp::Tf candidate = design(trial);
        const SpecCheck trial_check = check_specification(candidate.b, candidate.a, fs, bands, 12);
        if (trial_check.failed == 0) {
            minimum_order = trial;
            break;
        }
    }

    const Quantised qb = quantise_coefficients(tf.b, bits);
    const Quantised qa = quantise_coefficients(tf.a, bits);
    const SpecCheck quantised_check =
        check_specification(qb.values, qa.values, fs, bands, 24);

    const Grid grid = make_grid(fs, kResponsePoints);
    std::vector<double> mask_db;
    mask_db.reserve(grid.hz.size());
    for (const double hz : grid.hz) {
        mask_db.push_back(20.0 * std::log10(std::max(interpolated_gain(bands, hz), 1e-6)));
    }
    const std::vector<double> achieved_db = magnitude_db(tf.b, tf.a, grid);
    const std::vector<double> quantised_db = magnitude_db(qb.values, qa.values, grid);
    double quantisation_deviation = 0.0;
    for (std::size_t i = 0; i < achieved_db.size(); ++i) {
        quantisation_deviation =
            std::max(quantisation_deviation, std::abs(quantised_db[i] - achieved_db[i]));
    }

    double max_pole = 0.0;
    for (const auto& p : dsp::polynomial_roots(qa.values)) {
        max_pole = std::max(max_pole, std::abs(p));
    }

    json::Value out = json::Value::object();
    out.set("specification", json::Value(specification));
    out.set("family", json::Value(family));
    out.set("fs", json::Value(fs));
    out.set("b", coefficient_matrix(tf.b));
    out.set("a", coefficient_matrix(tf.a));
    out.set("b_quantised", coefficient_matrix(qb.values));
    out.set("a_quantised", coefficient_matrix(qa.values));
    out.set("magnitude", json::from_series("realised |H|", grid.hz, achieved_db));
    out.set("magnitude_quantised",
            json::from_series("Q1." + std::to_string(bits - 1) + " |H|", grid.hz, quantised_db));
    out.set("mask", json::from_series("specification mask", grid.hz, mask_db));
    out.set("verification", check.table);
    out.set("verification_quantised", quantised_check.table);
    out.set("points_passed", json::Value(static_cast<double>(check.passed)));
    out.set("points_failed", json::Value(static_cast<double>(check.failed)));
    out.set("worst_margin_db", json::Value(check.worst_margin_db));
    out.set("passband_max_deviation_db", json::Value(check.passband_max_deviation_db));
    out.set("stopband_attenuation_db", json::Value(check.stopband_attenuation_db));
    out.set("meets_specification", json::Value(check.failed == 0));
    out.set("meets_specification_quantised", json::Value(quantised_check.failed == 0));
    out.set("minimum_order", json::Value(static_cast<double>(minimum_order)));
    out.set("quantisation_deviation_db", json::Value(quantisation_deviation));
    out.set("quantisation_overflows",
            json::Value(static_cast<double>(qb.overflow_count + qa.overflow_count)));
    out.set("max_pole_radius_quantised", json::Value(max_pole));
    out.set("verdict",
            json::Value("Specification '" + specification + "' at fs = " +
                        json::number_to_string(fs) + " Hz, designed as " + family +
                        " at order " + std::to_string(tf.b.size() - 1) + ": " +
                        std::to_string(check.passed) + " of " +
                        std::to_string(check.passed + check.failed) +
                        " specification points pass, worst margin " +
                        json::number_to_string(check.worst_margin_db) + " dB. " +
                        (minimum_order != 0
                             ? "The lowest order of this family that meets every point is " +
                                   std::to_string(minimum_order) + ". "
                             : "No order up to 192 in this family meets every point - the "
                               "specification needs a different family or a relaxed mask. ") +
                        "Crushed into Q1." + std::to_string(bits - 1) +
                        " the response moves by up to " +
                        json::number_to_string(quantisation_deviation) + " dB, " +
                        std::to_string(qb.overflow_count + qa.overflow_count) +
                        " coefficient(s) overflow, the largest pole radius becomes " +
                        json::number_to_string(max_pole) + ", and the realised filter " +
                        (quantised_check.failed == 0 ? "still meets the mask."
                                                     : "NO LONGER meets the mask at " +
                                                           std::to_string(quantised_check.failed) +
                                                           " point(s).") +
                        " That last check is the one the project brief insists on: the filter you "
                        "wrote has to be the filter you designed, verified from the realised "
                        "coefficients and not from the design intent."));
    return out;
}

}  // namespace

ModuleDescription DspSystemModule::describe() const {
    ModuleDescription d;
    d.name = "dsp_system";
    d.title = "Digital Signal Processing Algorithms and Systems on the GP8 Channels";
    d.course = CourseRef{3882, "M-403-02", "Digital Signal Processing Algorithms and Systems"};
    d.topics = {
        "Session 1 · Linear Discrete Systems: Minimum, Maximum and Linear Phase",
        "Session 2 · Transfer-Function Determination and Test-Signal Selection",
        "Session 3 · The Generalised Correlation Method and Delay Elimination",
        "Session 4 · Generalised Correlation: Smoothing of Frequency Responses",
        "Session 5 · Inverse Transfer Function and Amplitude-Response Equalisation",
        "Session 6 · Lab: Transfer-Function Determination (Part 1)",
        "Session 7 · Lab: Transfer-Function Determination (Part 2)",
        "Session 8 · From Identified Model to Design Specification",
        "Session 9 · Arbitrary-Magnitude Approximation with FIR Systems (Part 1)",
        "Session 10 · Arbitrary-Magnitude Approximation with FIR Systems (Part 2)",
        "Session 11 · IIR Approximation: Prony's Method",
        "Session 12 · IIR Approximation: Parametric Equalisers and Realisation",
        "Session 13 · Phase Correctors Based on FIR; the Hilbert Transform",
        "Session 14 · FIR Phase Correctors and Quadrature Processing",
        "Session 15 · Phase Correctors Based on IIR (All-Pass Networks)",
        "Session 16 · Lab: Synthesis of a Linear Discrete System with Specified Amplitude Response",
        "Course 3884 Session 28 · Sensor feedback, signal conditioning and filtering",
    };
    d.source = "cpp_solver/include/study/dsp_system.hpp";
    d.summary =
        "Identifies, filters, equalises and phase-corrects the signals the GP8 actually produces - "
        "joint encoder position and velocity, motor current, force-torque channels and the arm's "
        "vibration after a fast move - with every channel synthesised from the inertias, friction "
        "and gear ratios of study/gp8_model.hpp.";

    const auto joint_param = [] {
        return ParamSpec::integer("joint", "Axis (1 = S ... 6 = T)", "", 1, 6, 2);
    };
    const auto fs_param = [](double fallback) {
        return ParamSpec::scalar("fs", "Sampling rate", "Hz", 50.0, 48000.0, fallback);
    };
    const auto length_param = [](int fallback) {
        return ParamSpec::integer("length", "Record length", "samples", kMinLength, kMaxLength,
                                  fallback);
    };
    const auto snr_param = [](double fallback) {
        return ParamSpec::scalar("snr_db", "Measurement SNR", "dB", -10.0, 200.0, fallback);
    };
    const auto seed_param = [] {
        return ParamSpec::integer("seed", "Noise seed (reproducible records)", "", 1, 2000000000,
                                  20261010);
    };

    {
        OpSpec op;
        op.name = "test_signals";
        op.title = "Robot test signals and which one identifies a joint";
        op.formula =
            "\\mathrm{flatness} = \\frac{\\exp\\left(\\frac{1}{K}\\sum_k \\ln S_{uu}(k)\\right)}"
            "{\\frac{1}{K}\\sum_k S_{uu}(k)}, \\qquad "
            "\\mathrm{crest} = \\frac{\\max_n |u[n]|}{\\sqrt{\\frac{1}{N}\\sum_n u^2[n]}}";
        op.explain =
            "Generates the six excitation signals of session 2 as a joint-velocity command, runs "
            "each one through the joint's own channel at the same SNR, and estimates the channel "
            "back from the record with the generalised correlation method. The assessed point is "
            "in the comparison table: a PRBS or a multisine wins because it puts energy in every "
            "bin of the servo band at a crest factor near 1, so the amplitude the robot is "
            "allowed to move with buys the most measurement. A step has a 1/f spectrum and barely "
            "excites the arm's structural mode, and an impulse needs an unreachable peak amplitude "
            "before the high band clears the sensor noise.";
        op.params = {
            ParamSpec::enumeration("signal", "Test signal", kSignalOptions, "prbs"),
            length_param(512),
            fs_param(dsp::kGp8ControlRateHz),
            ParamSpec::scalar("amplitude", "Excitation amplitude", "rad/s", 1e-6, 100.0, 1.0),
            snr_param(30.0),
            joint_param(),
            seed_param(),
        };
        op.outputs = {
            OutputSpec::make("signal", "series", "Excitation applied to the joint", "rad/s"),
            OutputSpec::make("response", "series", "Measured joint velocity", "rad/s"),
            OutputSpec::make("spectrum", "series", "Excitation power spectrum", "dB"),
            OutputSpec::make("comparison", "table", "All six signals compared as identifiers"),
            OutputSpec::make("best_signal", "text", "Best identifier at this setting"),
            OutputSpec::make("identification_error", "scalar", "Relative impulse-response error"),
            OutputSpec::make("crest_factor", "scalar", "Peak over RMS of the chosen signal"),
            OutputSpec::make("spectral_flatness", "scalar", "1 for white, 0 for a single tone"),
            OutputSpec::make("resonance_hz", "scalar", "Structural mode of the axis", "Hz"),
            OutputSpec::make("verdict", "text", "Which signal identifies the joint, and why"),
        };
        d.ops.push_back(std::move(op));
    }
    {
        OpSpec op;
        op.name = "phase_types";
        op.title = "Minimum, maximum and linear phase";
        op.formula =
            "H(z) = b_0 \\prod_{k} (1 - z_k z^{-1}), \\qquad "
            "\\tau_g(\\omega) = -\\frac{d \\arg H(e^{j\\omega})}{d\\omega}";
        op.explain =
            "Classifies a filter from where its zeros sit: all inside the unit circle is minimum "
            "phase and causally invertible, all outside is maximum phase and delivers the same "
            "magnitude with the greatest possible lag, symmetric or antisymmetric coefficients are "
            "linear phase with a group delay of exactly (N-1)/2 samples. Reflecting a zero from "
            "0.5 to 2.0 leaves the magnitude response unchanged and the group delay completely "
            "changed, which is the exam question; the two velocity traces show what that does to "
            "the shape of a real GP8 move.";
        op.params = {
            ParamSpec::enumeration("preset", "Zero placement preset", kPhasePresets,
                                   "linear_phase"),
            ParamSpec::structured("b", "FIR coefficients (overrides the preset)", "matrix", "",
                                  empty_coefficient_default()),
            joint_param(),
            fs_param(dsp::kGp8ControlRateHz),
            length_param(256),
        };
        op.outputs = {
            OutputSpec::make("coefficients", "matrix", "Filter coefficients"),
            OutputSpec::make("zeros", "complex_set", "Zeros in the z plane"),
            OutputSpec::make("unit_circle", "series", "Unit circle for reference"),
            OutputSpec::make("zero_table", "table", "Radius and angle of every zero"),
            OutputSpec::make("magnitude", "series", "Magnitude response", "dB"),
            OutputSpec::make("group_delay", "series", "Group delay", "samples"),
            OutputSpec::make("velocity_in", "series", "Joint-velocity burst", "rad/s"),
            OutputSpec::make("velocity_out", "series", "After the filter", "rad/s"),
            OutputSpec::make("phase_class", "text", "Minimum, maximum, linear or mixed phase"),
            OutputSpec::make("reason", "text", "Why, from the zeros and the symmetry"),
            OutputSpec::make("is_linear_phase", "bool", "Coefficients symmetric or antisymmetric"),
            OutputSpec::make("min_zero_radius", "scalar", "Smallest zero radius"),
            OutputSpec::make("max_zero_radius", "scalar", "Largest zero radius"),
            OutputSpec::make("shape_match", "scalar", "Waveform match after the delay is removed"),
            OutputSpec::make("peak_shift_samples", "scalar", "Delay the filter introduced",
                             "samples"),
            OutputSpec::make("verdict", "text", "The phase class and what it does to the waveform"),
        };
        d.ops.push_back(std::move(op));
    }
    {
        OpSpec op;
        op.name = "identify_transfer_function";
        op.title = "Transfer-function determination from an input/output record";
        op.formula =
            "\\sum_k a_k \\sum_n u[n-\\tau]\\, y[n-k] = \\sum_k b_k \\sum_n u[n-\\tau]\\, u[n-k], \\qquad "
            "\\hat{H}(z) = \\frac{\\hat{b}_0 + \\hat{b}_1 z^{-1} + \\cdots}"
            "{1 + \\hat{a}_1 z^{-1} + \\cdots}, \\qquad "
            "N \\approx \\frac{M}{S\\,\\varepsilon^2}";
        op.explain =
            "Excites the joint channel, measures it with noise and estimates H(z) back. The "
            "generalised correlation method multiplies the model equation by a delayed input and "
            "sums over the record, which replaces the data by correlation functions: output noise "
            "is uncorrelated with the input, so it averages away there instead of biasing the "
            "answer. "
            "Least-squares ARX is cheaper and visibly biased at a low SNR because its regressors "
            "contain the noisy output, and spectral division needs no model order but divides by "
            "the excitation. The record length formula is the last output: the accuracy you want "
            "fixes how long the robot has to move.";
        op.params = {
            ParamSpec::enumeration("method", "Estimator", kIdentificationMethods,
                                   "generalised_correlation"),
            ParamSpec::enumeration("excitation", "Excitation signal", kSignalOptions, "prbs"),
            length_param(1024),
            fs_param(dsp::kGp8ControlRateHz),
            joint_param(),
            snr_param(30.0),
            ParamSpec::integer("numerator_order", "Numerator order", "", 0, 20, 2),
            ParamSpec::integer("denominator_order", "Denominator order", "", 1, 20, 3),
            ParamSpec::scalar("target_error", "Wanted relative accuracy", "", 1e-4, 1.0, 0.05),
            seed_param(),
        };
        op.outputs = {
            OutputSpec::make("b_estimated", "matrix", "Estimated numerator"),
            OutputSpec::make("a_estimated", "matrix", "Estimated denominator"),
            OutputSpec::make("b_true", "matrix", "True numerator of the model channel"),
            OutputSpec::make("a_true", "matrix", "True denominator of the model channel"),
            OutputSpec::make("magnitude_true", "series", "True magnitude", "dB"),
            OutputSpec::make("magnitude_estimated", "series", "Estimated magnitude", "dB"),
            OutputSpec::make("phase_true", "series", "True phase", "deg"),
            OutputSpec::make("phase_estimated", "series", "Estimated phase", "deg"),
            OutputSpec::make("residual", "series", "Prediction error", "rad/s"),
            OutputSpec::make("max_magnitude_error_db", "scalar", "Worst magnitude error", "dB"),
            OutputSpec::make("rms_magnitude_error_db", "scalar", "RMS magnitude error", "dB"),
            OutputSpec::make("identifiable_band_hz", "scalar", "Band the error was measured over",
                             "Hz"),
            OutputSpec::make("residual_ratio", "scalar", "Residual RMS over output RMS"),
            OutputSpec::make("required_length", "scalar", "Record length for the wanted accuracy",
                             "samples"),
            OutputSpec::make("poles", "complex_set", "Estimated poles"),
            OutputSpec::make("max_pole_radius", "scalar", "Largest estimated pole radius"),
            OutputSpec::make("method", "text", "Estimator that was applied"),
            OutputSpec::make("verdict", "text", "What the estimate is worth"),
        };
        d.ops.push_back(std::move(op));
    }
    {
        OpSpec op;
        op.name = "delay_elimination";
        op.title = "Transport-delay estimation and removal by cross-correlation";
        op.formula =
            "R_{uy}(\\lambda) = \\sum_n u[n]\\,y[n+\\lambda], \\qquad "
            "\\hat{d} = \\arg\\max_\\lambda |R_{uy}(\\lambda)| - \\tau_{\\mathrm{sensor}}";
        op.explain =
            "A servo bus, a fieldbus hop and an encoder-derived velocity each add pure delay, and "
            "pure delay is invisible in the magnitude response and fatal in any phase-based fit. "
            "The cross-correlation of the commanded and the measured joint velocity peaks at the "
            "total lag; subtracting the known group delay of the velocity smoother leaves the "
            "transport delay in samples and in seconds. Strip it before identifying: the "
            "parametric estimate is only minimum phase once the delay is gone.";
        op.params = {
            length_param(512),
            fs_param(dsp::kGp8ControlRateHz),
            joint_param(),
            ParamSpec::integer("true_delay", "Injected transport delay", "samples", 0, 64, 7),
            snr_param(20.0),
            ParamSpec::boolean("include_sensor_filter", "3-tap velocity smoothing in the chain",
                               true),
            ParamSpec::integer("max_lag", "Correlation search window", "samples", 1, 512, 64),
            seed_param(),
        };
        op.outputs = {
            OutputSpec::make("correlation", "series", "Cross-correlation against lag"),
            OutputSpec::make("peak_value", "scalar", "Correlation peak"),
            OutputSpec::make("peak_lag", "scalar", "Lag of the peak", "samples"),
            OutputSpec::make("peak_centroid_lag", "scalar", "Centroid of the correlation plateau",
                             "samples"),
            OutputSpec::make("delay_fractional_samples", "scalar", "Sub-sample delay estimate",
                             "samples"),
            OutputSpec::make("sensor_group_delay", "scalar", "Known smoother delay", "samples"),
            OutputSpec::make("delay_samples", "scalar", "Estimated transport delay", "samples"),
            OutputSpec::make("delay_seconds", "scalar", "Estimated transport delay", "s"),
            OutputSpec::make("true_delay", "scalar", "Delay that was injected", "samples"),
            OutputSpec::make("commanded", "series", "Commanded joint velocity", "rad/s"),
            OutputSpec::make("measured_before", "series", "Measured, still delayed", "rad/s"),
            OutputSpec::make("measured_after", "series", "Measured, delay removed", "rad/s"),
            OutputSpec::make("rms_error_before", "scalar", "Alignment error before", "rad/s"),
            OutputSpec::make("rms_error_after", "scalar", "Alignment error after", "rad/s"),
            OutputSpec::make("max_zero_radius_before", "scalar", "Largest zero with the delay in"),
            OutputSpec::make("max_zero_radius_after", "scalar", "Largest zero once it is removed"),
            OutputSpec::make("verdict", "text", "What the correlation peak proved"),
        };
        d.ops.push_back(std::move(op));
    }
    {
        OpSpec op;
        op.name = "smooth_frequency_response";
        op.title = "Smoothing a measured frequency response";
        op.formula =
            "\\hat{H}(k) = \\frac{\\sum_r U_r^*(k) Y_r(k)}{\\sum_r |U_r(k)|^2}, \\qquad "
            "\\tilde{H}(k) = \\frac{\\sum_j w_j \\hat{H}_{dB}(k+j)}{\\sum_j w_j}";
        op.explain =
            "A single identification record gives a ragged magnitude estimate; averaging records "
            "and smoothing across frequency makes it presentable. Both outputs are reported "
            "because the trade is the lesson: the scatter falls as the window widens, and so does "
            "the height of the arm's structural resonance. Widen the window far enough and a real "
            "40 Hz mode of the GP8 disappears from the estimate entirely - a beautifully smooth "
            "measurement of a robot that does not exist.";
        op.params = {
            ParamSpec::enumeration("window", "Smoothing window", kWindowOptions, "hann"),
            ParamSpec::integer("window_length", "Smoothing width (odd)", "bins", 1, 101, 9),
            ParamSpec::integer("records", "Averaged records", "", 1, 32, 1),
            length_param(512),
            fs_param(dsp::kGp8ControlRateHz),
            joint_param(),
            snr_param(10.0),
            ParamSpec::scalar("kaiser_beta", "Kaiser beta", "", 0.0, 20.0, 5.65),
            seed_param(),
        };
        op.outputs = {
            OutputSpec::make("raw", "series", "Measured magnitude", "dB"),
            OutputSpec::make("smoothed", "series", "Smoothed estimate", "dB"),
            OutputSpec::make("truth", "series", "True channel magnitude", "dB"),
            OutputSpec::make("residual", "series", "Smoothed minus true", "dB"),
            OutputSpec::make("noise_std_raw_db", "scalar", "Scatter before smoothing", "dB"),
            OutputSpec::make("noise_std_smoothed_db", "scalar", "Scatter after smoothing", "dB"),
            OutputSpec::make("variance_reduction", "scalar", "Variance ratio"),
            OutputSpec::make("resonance_hz", "scalar", "Resonance found in the truth", "Hz"),
            OutputSpec::make("resonance_peak_true_db", "scalar", "True peak height", "dB"),
            OutputSpec::make("resonance_peak_smoothed_db", "scalar", "Smoothed peak height", "dB"),
            OutputSpec::make("peak_loss_db", "scalar", "Bias introduced at the peak", "dB"),
            OutputSpec::make("resolution_hz", "scalar", "Resolution given up", "Hz"),
            OutputSpec::make("verdict", "text", "The bias-variance trade, quantified"),
        };
        d.ops.push_back(std::move(op));
    }
    {
        OpSpec op;
        op.name = "inverse_and_equalise";
        op.title = "Inverse transfer function and amplitude equalisation";
        op.formula =
            "H^{-1}(z) = \\frac{A(z)}{B(z)}, \\qquad "
            "C(\\omega) = \\frac{H^*(\\omega)}{|H(\\omega)|^2 + \\varepsilon}";
        op.explain =
            "The exact inverse of a channel swaps its poles and zeros, so a channel with a zero "
            "outside the unit circle has no stable causal inverse at all - this op says so instead "
            "of returning a number. What it builds instead is a least-squares FIR fit to the "
            "Tikhonov-regularised inverse, which is stable and causal by construction. The output "
            "to watch is the equaliser's own peak gain: that is the factor by which the sensor "
            "noise floor is lifted at the spectral null, and the reason epsilon exists.";
        op.params = {
            fs_param(dsp::kGp8ControlRateHz),
            joint_param(),
            ParamSpec::scalar("epsilon", "Regularisation", "", 0.0, 1.0, 0.01),
            ParamSpec::scalar("zero_radius", "Channel zero radius (>1 is non-invertible)", "", 0.05,
                              3.0, 1.2),
            ParamSpec::integer("equaliser_order", "Equaliser order", "", 8, kMaxOrder, 48),
            ParamSpec::scalar("band_lo", "Band to equalise, lower edge", "Hz", 0.0, 20000.0, 1.0),
            ParamSpec::scalar("band_hi", "Band to equalise, upper edge", "Hz", 1.0, 24000.0, 400.0),
        };
        op.outputs = {
            OutputSpec::make("b_channel", "matrix", "Channel numerator"),
            OutputSpec::make("a_channel", "matrix", "Channel denominator"),
            OutputSpec::make("equaliser_coefficients", "matrix", "FIR equaliser"),
            OutputSpec::make("magnitude_channel", "series", "Channel magnitude", "dB"),
            OutputSpec::make("magnitude_exact_inverse", "series", "Exact inverse magnitude", "dB"),
            OutputSpec::make("magnitude_equaliser", "series", "Equaliser magnitude", "dB"),
            OutputSpec::make("magnitude_cascade", "series", "Equalised cascade", "dB"),
            OutputSpec::make("channel_zeros", "complex_set", "Channel zeros"),
            OutputSpec::make("unit_circle", "series", "Unit circle for reference"),
            OutputSpec::make("max_zero_radius", "scalar", "Largest channel zero radius"),
            OutputSpec::make("inverse_is_stable", "bool", "Exact inverse stable and causal"),
            OutputSpec::make("flatness_db", "scalar", "Cascade flatness in the band", "dB"),
            OutputSpec::make("noise_gain_db", "scalar", "Noise lifted by the equaliser", "dB"),
            OutputSpec::make("action_taken", "text", "What was done about an unstable inverse"),
            OutputSpec::make("verdict", "text", "The flatness-versus-noise trade"),
        };
        d.ops.push_back(std::move(op));
    }
    {
        OpSpec op;
        op.name = "fir_arbitrary_magnitude";
        op.title = "FIR design to an arbitrary magnitude specification";
        op.formula =
            "A(\\omega) = b_M + 2\\sum_{k=1}^{M} b_{M+k}\\cos(k\\omega), \\qquad "
            "\\min_b \\sum_i W(\\omega_i)\\,|A(\\omega_i) - D(\\omega_i)|^2";
        op.explain =
            "Designs a symmetric Type-I FIR to a multi-band magnitude target: the servo low-pass, "
            "a notch on the arm's own resonance, or a vibration band-pass. Frequency sampling is "
            "the quickest and rings between its samples, windowing is predictable and pays for its "
            "attenuation with transition width, and weighted least squares does what the "
            "specification asks where the specification exists and ignores the transition bands. "
            "Every design is then checked against the mask it claims to meet, and the group delay "
            "and the multiply-accumulate rate are printed next to it, because both are charged to "
            "the 1 kHz control loop.";
        op.params = {
            ParamSpec::enumeration("method", "Design method", kFirMethods, "least_squares"),
            ParamSpec::enumeration("specification", "Magnitude specification", kFirSpecifications,
                                   "servo_lowpass"),
            ParamSpec::enumeration("window", "Window (windowed method)", kWindowOptions, "hamming"),
            ParamSpec::scalar("kaiser_beta", "Kaiser beta", "", 0.0, 20.0, 5.65),
            ParamSpec::integer("order", "Filter order", "", 4, kMaxOrder, 64),
            fs_param(dsp::kGp8ControlRateHz),
            joint_param(),
            ParamSpec::scalar("pass_edge", "Pass-band edge", "Hz", 0.5, 24000.0, 60.0),
            ParamSpec::scalar("stop_edge", "Stop-band edge", "Hz", 1.0, 24000.0, 140.0),
            ParamSpec::scalar("stopband_db", "Required stop-band attenuation", "dB", 6.0, 120.0,
                              40.0),
            ParamSpec::scalar("passband_ripple_db", "Allowed pass-band ripple", "dB", 0.01, 12.0,
                              1.0),
        };
        op.outputs = {
            OutputSpec::make("coefficients", "matrix", "FIR coefficients"),
            OutputSpec::make("taps", "scalar", "Number of taps"),
            OutputSpec::make("magnitude_achieved", "series", "Achieved magnitude", "dB"),
            OutputSpec::make("magnitude_specification", "series", "Specified magnitude", "dB"),
            OutputSpec::make("group_delay", "series", "Group delay", "samples"),
            OutputSpec::make("verification", "table", "Every specification point checked"),
            OutputSpec::make("passband_max_deviation_db", "scalar", "Worst pass-band error", "dB"),
            OutputSpec::make("stopband_attenuation_db", "scalar", "Achieved attenuation", "dB"),
            OutputSpec::make("meets_specification", "bool", "Every checked point passes"),
            OutputSpec::make("points_passed", "scalar", "Specification points passed"),
            OutputSpec::make("points_failed", "scalar", "Specification points failed"),
            OutputSpec::make("worst_margin_db", "scalar", "Smallest margin", "dB"),
            OutputSpec::make("group_delay_samples", "scalar", "Constant group delay", "samples"),
            OutputSpec::make("group_delay_ms", "scalar", "Group delay in the loop", "ms"),
            OutputSpec::make("mac_per_second", "scalar", "Multiply-accumulates per second"),
            OutputSpec::make("verdict", "text", "What was achieved and what it cost"),
        };
        d.ops.push_back(std::move(op));
    }
    {
        OpSpec op;
        op.name = "iir_prony";
        op.title = "Prony's method for IIR approximation";
        op.formula =
            "\\sum_{k=1}^{p} a_k h[n-k] = -h[n], \\; n > q, \\qquad "
            "b_n = \\sum_{k=0}^{\\min(n,p)} a_k h[n-k], \\; n \\le q";
        op.explain =
            "Prony splits the fit in two: for samples beyond the numerator order the impulse "
            "response obeys a pure linear recursion, so the denominator comes out of a linear "
            "least-squares problem, and the numerator then follows by filtering the first q+1 "
            "samples. It reaches a given error at a small fraction of the FIR length, and the "
            "outputs say what that costs: the pole radii, a stability verdict, and the FIR tap "
            "count that would have been needed for the same fidelity.";
        op.params = {
            ParamSpec::integer("numerator_order", "Numerator order q", "", 0, 12, 2),
            ParamSpec::integer("denominator_order", "Denominator order p", "", 1, 12, 3),
            ParamSpec::integer("impulse_length", "Impulse-response samples used", "", 16, 1024, 128),
            ParamSpec::scalar("target_error", "Error to beat", "", 1e-6, 1.0, 0.02),
            ParamSpec::scalar("snr_db", "SNR of the measured impulse response", "dB", 0.0, 200.0,
                              200.0),
            fs_param(dsp::kGp8ControlRateHz),
            joint_param(),
            seed_param(),
        };
        op.outputs = {
            OutputSpec::make("b", "matrix", "Fitted numerator"),
            OutputSpec::make("a", "matrix", "Fitted denominator"),
            OutputSpec::make("a_true", "matrix", "True denominator of the channel"),
            OutputSpec::make("impulse_target", "series", "Measured impulse response"),
            OutputSpec::make("impulse_fitted", "series", "Prony fit"),
            OutputSpec::make("magnitude_true", "series", "True magnitude", "dB"),
            OutputSpec::make("magnitude_fitted", "series", "Fitted magnitude", "dB"),
            OutputSpec::make("relative_error", "scalar", "||fit - h|| / ||h||"),
            OutputSpec::make("poles", "complex_set", "Fitted poles"),
            OutputSpec::make("zeros", "complex_set", "Fitted zeros"),
            OutputSpec::make("unit_circle", "series", "Unit circle for reference"),
            OutputSpec::make("max_pole_radius", "scalar", "Largest pole radius"),
            OutputSpec::make("stable", "bool", "Every pole inside the unit circle"),
            OutputSpec::make("order_needed", "scalar", "Denominator order that beats the target"),
            OutputSpec::make("fir_taps_equivalent", "scalar", "FIR taps for the same fidelity"),
            OutputSpec::make("verdict", "text", "The IIR bargain, priced"),
        };
        d.ops.push_back(std::move(op));
    }
    {
        OpSpec op;
        op.name = "parametric_equaliser";
        op.title = "Parametric equaliser, realisation structure and word length";
        op.formula =
            "H(z) = \\frac{1 + \\alpha A - 2\\cos\\omega_0 z^{-1} + (1 - \\alpha A) z^{-2}}"
            "{1 + \\alpha/A - 2\\cos\\omega_0 z^{-1} + (1 - \\alpha/A) z^{-2}}, \\quad "
            "\\alpha = \\frac{\\sin\\omega_0}{2Q}";
        op.explain =
            "The peaking, shelving and notching biquad family, tuned onto the arm's measured "
            "resonance and then realised honestly. The realisation structure decides where the "
            "rounding happens and therefore what shapes the round-off noise, and the word length "
            "decides whether the coefficients fit at all: a denominator coefficient of -1.6 does "
            "not fit in Q1.7, and saturating it moves the poles. The outputs put the ideal and the "
            "quantised response, the pole displacement and the stability verdict side by side, "
            "with the joint-velocity ring-down before and after the section is enabled.";
        op.params = {
            ParamSpec::enumeration("filter_type", "Section type", kEqualiserTypes, "peaking"),
            ParamSpec::enumeration("structure", "Realisation structure", kStructures,
                                   "direct_form_1"),
            fs_param(dsp::kGp8ControlRateHz),
            joint_param(),
            ParamSpec::scalar("f0", "Centre frequency", "Hz", 0.5, 24000.0, 25.0),
            ParamSpec::scalar("q", "Q factor", "", 0.1, 40.0, 4.0),
            ParamSpec::scalar("gain_db", "Gain", "dB", -40.0, 40.0, -12.0),
            ParamSpec::integer("bits", "Coefficient word length (Q1.bits-1)", "bit", 4, 32, 16),
            length_param(512),
        };
        op.outputs = {
            OutputSpec::make("b", "matrix", "Ideal numerator"),
            OutputSpec::make("a", "matrix", "Ideal denominator"),
            OutputSpec::make("b_quantised", "matrix", "Quantised numerator"),
            OutputSpec::make("a_quantised", "matrix", "Quantised denominator"),
            OutputSpec::make("coefficient_table", "table", "Ideal, quantised and error"),
            OutputSpec::make("magnitude_ideal", "series", "Ideal magnitude", "dB"),
            OutputSpec::make("magnitude_quantised", "series", "Quantised magnitude", "dB"),
            OutputSpec::make("max_deviation_db", "scalar", "Worst quantisation error", "dB"),
            OutputSpec::make("poles_ideal", "complex_set", "Ideal poles"),
            OutputSpec::make("poles_quantised", "complex_set", "Quantised poles"),
            OutputSpec::make("unit_circle", "series", "Unit circle for reference"),
            OutputSpec::make("pole_radius_ideal", "scalar", "Ideal pole radius"),
            OutputSpec::make("pole_radius_quantised", "scalar", "Quantised pole radius"),
            OutputSpec::make("pole_displacement", "scalar", "How far the poles moved"),
            OutputSpec::make("overflow_count", "scalar", "Coefficients that did not fit"),
            OutputSpec::make("quantisation_step", "scalar", "LSB of the chosen Q format"),
            OutputSpec::make("largest_coefficient", "scalar", "Largest coefficient magnitude"),
            OutputSpec::make("stable_after_quantisation", "bool", "Poles still inside the circle"),
            OutputSpec::make("noise_gain_db", "scalar", "Round-off noise gain of the structure",
                             "dB"),
            OutputSpec::make("structure", "text", "Realisation that was applied"),
            OutputSpec::make("structure_note", "text", "What the structure changes"),
            OutputSpec::make("vibration_before", "series", "Joint velocity, equaliser off", "rad/s"),
            OutputSpec::make("vibration_after", "series", "Joint velocity, equaliser on", "rad/s"),
            OutputSpec::make("ringing_rms_before", "scalar", "Ring-down before", "rad/s"),
            OutputSpec::make("ringing_rms_after", "scalar", "Ring-down after", "rad/s"),
            OutputSpec::make("verdict", "text", "Realisation and word length, priced"),
        };
        d.ops.push_back(std::move(op));
    }
    {
        OpSpec op;
        op.name = "fir_phase_corrector";
        op.title = "FIR phase corrector for a joint channel";
        op.formula =
            "\\min_c \\sum_i \\left| C(\\omega_i) - e^{-j(\\omega_i D + \\arg H(\\omega_i))} "
            "\\right|^2";
        op.explain =
            "A joint channel's group delay is not constant, so a velocity burst comes back with "
            "its shape changed even when the magnitude response is acceptable. This fits a causal "
            "FIR whose phase is whatever makes the cascade's total phase linear over the chosen "
            "band, by complex least squares. It is only approximately all-pass, and the output "
            "says by how much it disturbs the magnitude: an FIR corrector buys flat delay with "
            "more delay, and that bill is charged to the control loop.";
        op.params = {
            ParamSpec::integer("order", "Corrector order", "", 8, kMaxOrder, 64),
            fs_param(dsp::kGp8ControlRateHz),
            joint_param(),
            ParamSpec::scalar("band_lo", "Band to correct, lower edge", "Hz", 0.0, 20000.0, 1.0),
            ParamSpec::scalar("band_hi", "Band to correct, upper edge (default: just below the "
                              "axis resonance)", "Hz", 1.0, 24000.0, 20.0),
            ParamSpec::scalar("target_delay", "Target group delay (0 = automatic)", "samples", 0.0,
                              400.0, 0.0),
        };
        op.outputs = {
            OutputSpec::make("coefficients", "matrix", "Corrector coefficients"),
            OutputSpec::make("group_delay_before", "series", "Channel group delay", "samples"),
            OutputSpec::make("group_delay_after", "series", "Corrected cascade", "samples"),
            OutputSpec::make("corrector_magnitude", "series", "Corrector magnitude", "dB"),
            OutputSpec::make("delay_ripple_before", "scalar", "Delay swing before", "samples"),
            OutputSpec::make("delay_ripple_after", "scalar", "Delay swing after", "samples"),
            OutputSpec::make("target_delay_samples", "scalar", "Delay aimed at", "samples"),
            OutputSpec::make("magnitude_error_db", "scalar", "Magnitude disturbed by", "dB"),
            OutputSpec::make("verdict", "text", "What was flattened and what it cost"),
        };
        d.ops.push_back(std::move(op));
    }
    {
        OpSpec op;
        op.name = "hilbert_transform";
        op.title = "Hilbert transform, analytic signal and instantaneous frequency";
        op.formula =
            "x_a[n] = x[n] + j\\,\\mathcal{H}\\{x\\}[n], \\qquad "
            "A[n] = |x_a[n]|, \\qquad "
            "f_i[n] = \\frac{f_s}{2\\pi}\\left(\\arg x_a[n] - \\arg x_a[n-1]\\right)";
        op.explain =
            "Builds the quadrature partner of a vibration channel, either with an odd FIR kernel "
            "(exact group delay (L-1)/2, which is why the in-phase channel is delayed to match) or "
            "from the one-sided spectrum. The envelope and the instantaneous frequency follow from "
            "the analytic signal. The envelope spectrum is the payoff: a bearing-fault modulation "
            "that is invisible in the raw spectrum, where every bin of energy sits at the carrier, "
            "appears as a clear line at the fault rate.";
        op.params = {
            ParamSpec::enumeration("method", "Quadrature method", kHilbertMethods, "fft_analytic"),
            ParamSpec::enumeration("signal", "Vibration signal", kVibrationSignals, "ringdown"),
            ParamSpec::integer("kernel_length", "FIR kernel length (odd)", "", 11, 501, 101),
            length_param(1024),
            fs_param(dsp::kGp8ControlRateHz),
            joint_param(),
            ParamSpec::scalar("snr_db", "SNR of the vibration channel", "dB", 0.0, 200.0, 200.0),
            ParamSpec::scalar("modulation_hz", "Fault modulation rate", "Hz", 0.5, 200.0, 23.0),
            seed_param(),
        };
        op.outputs = {
            OutputSpec::make("signal", "series", "In-phase channel"),
            OutputSpec::make("quadrature", "series", "Quadrature channel"),
            OutputSpec::make("envelope", "series", "Envelope"),
            OutputSpec::make("instantaneous_frequency", "series", "Instantaneous frequency", "Hz"),
            OutputSpec::make("envelope_spectrum", "series", "Envelope spectrum", "dB"),
            OutputSpec::make("kernel", "matrix", "Hilbert FIR kernel, when one was used"),
            OutputSpec::make("carrier_hz", "scalar", "Carrier (structural mode)", "Hz"),
            OutputSpec::make("detected_modulation_hz", "scalar", "Peak of the envelope spectrum",
                             "Hz"),
            OutputSpec::make("orthogonality", "scalar", "Normalised I/Q inner product, 0 is ideal"),
            OutputSpec::make("amplitude_balance", "scalar", "RMS ratio of the pair, 1 is ideal"),
            OutputSpec::make("method", "text", "Quadrature method applied"),
            OutputSpec::make("verdict", "text", "What the analytic signal revealed"),
        };
        d.ops.push_back(std::move(op));
    }
    {
        OpSpec op;
        op.name = "iir_allpass_corrector";
        op.title = "IIR all-pass phase correction";
        op.formula =
            "A(z) = \\frac{r^2 - 2r\\cos\\theta\\,z^{-1} + z^{-2}}"
            "{1 - 2r\\cos\\theta\\,z^{-1} + r^2 z^{-2}}, \\qquad |A(e^{j\\omega})| = 1";
        op.explain =
            "An all-pass section's numerator is its denominator reversed, so its magnitude is "
            "exactly one at every frequency - not approximately, exactly - while its group delay "
            "is shaped by the pole radius and angle. That is what makes it the right tool for "
            "aligning two GP8 channels: the force-torque signal arrives later than the encoder "
            "signal because of the sensor's own anti-alias filter, and an all-pass network on the "
            "encoder path lines them up without touching either amplitude response.";
        op.params = {
            ParamSpec::integer("sections", "All-pass sections", "", 1, 6, 2),
            fs_param(dsp::kGp8ControlRateHz),
            joint_param(),
            ParamSpec::scalar("pole_radius", "Pole radius (0 = choose the best alignment)", "",
                              0.0, 0.95, 0.0),
            ParamSpec::scalar("pole_angle", "Pole angle", "rad", 0.0, 3.14159, 0.157),
            ParamSpec::scalar("sensor_cutoff", "Force-torque sensor cutoff", "Hz", 1.0, 24000.0,
                              80.0),
            ParamSpec::scalar("band_hi", "Band to align, upper edge", "Hz", 2.0, 24000.0, 250.0),
        };
        op.outputs = {
            OutputSpec::make("b", "matrix", "All-pass numerator"),
            OutputSpec::make("a", "matrix", "All-pass denominator"),
            OutputSpec::make("magnitude_allpass", "series", "All-pass magnitude", "dB"),
            OutputSpec::make("magnitude_channel", "series", "Channel alone", "dB"),
            OutputSpec::make("magnitude_corrected", "series", "Channel with the all-pass", "dB"),
            OutputSpec::make("max_magnitude_deviation_db", "scalar", "All-pass flatness", "dB"),
            OutputSpec::make("max_channel_change_db", "scalar", "Change to the channel", "dB"),
            OutputSpec::make("group_delay_before", "series", "Group delay before", "samples"),
            OutputSpec::make("group_delay_after", "series", "Group delay after", "samples"),
            OutputSpec::make("added_group_delay", "series", "Delay added by the network", "samples"),
            OutputSpec::make("mean_added_delay_samples", "scalar", "Mean delay added", "samples"),
            OutputSpec::make("poles", "complex_set", "All-pass poles"),
            OutputSpec::make("zeros", "complex_set", "All-pass zeros (reciprocal of the poles)"),
            OutputSpec::make("unit_circle", "series", "Unit circle for reference"),
            OutputSpec::make("misalignment_before_samples", "scalar", "Channel misalignment before",
                             "samples"),
            OutputSpec::make("misalignment_after_samples", "scalar", "Channel misalignment after",
                             "samples"),
            OutputSpec::make("verdict", "text", "Phase bought with delay, magnitude untouched"),
        };
        d.ops.push_back(std::move(op));
    }
    {
        OpSpec op;
        op.name = "synthesise_to_specification";
        op.title = "Synthesis of a discrete system to an amplitude specification";
        op.formula =
            "\\left|H(e^{j\\omega})\\right|_{dB} \\le M(\\omega) \\;\\; \\forall \\omega "
            "\\in \\Omega_{\\mathrm{spec}}, \\qquad "
            "\\hat{b}_k = \\frac{\\mathrm{round}(b_k 2^{B-1})}{2^{B-1}}";
        op.explain =
            "The final lab, end to end: take an amplitude specification, choose a family, design "
            "to it, and then prove the claim at every specification point from the realised "
            "coefficients - not from the design intent. The order search answers the other half of "
            "the question, which is how little arithmetic the specification can be met with. Then "
            "the coefficients are crushed into a Q-format word and the whole verification is run "
            "again, because that is where a design that looked finished stops meeting its mask or "
            "stops being stable.";
        op.params = {
            ParamSpec::enumeration("specification", "Amplitude specification", kSynthesisSpecs,
                                   "antialias_exam"),
            ParamSpec::enumeration("family", "Filter family", kSynthesisFamilies, "fir_windowed"),
            ParamSpec::enumeration("window", "Window", kWindowOptions, "blackman"),
            ParamSpec::scalar("kaiser_beta", "Kaiser beta", "", 0.0, 20.0, 5.65),
            ParamSpec::integer("order", "Order", "", 4, kMaxOrder, 72),
            ParamSpec::integer("bits", "Coefficient word length (Q1.bits-1)", "bit", 4, 32, 16),
        };
        op.outputs = {
            OutputSpec::make("specification", "text", "Specification that was applied"),
            OutputSpec::make("family", "text", "Family that was designed"),
            OutputSpec::make("fs", "scalar", "Sampling rate of the specification", "Hz"),
            OutputSpec::make("b", "matrix", "Numerator"),
            OutputSpec::make("a", "matrix", "Denominator"),
            OutputSpec::make("b_quantised", "matrix", "Quantised numerator"),
            OutputSpec::make("a_quantised", "matrix", "Quantised denominator"),
            OutputSpec::make("magnitude", "series", "Realised magnitude", "dB"),
            OutputSpec::make("magnitude_quantised", "series", "Quantised magnitude", "dB"),
            OutputSpec::make("mask", "series", "Specification mask", "dB"),
            OutputSpec::make("verification", "table", "Pass/fail at every specification point"),
            OutputSpec::make("verification_quantised", "table", "The same check after quantisation"),
            OutputSpec::make("points_passed", "scalar", "Points passed"),
            OutputSpec::make("points_failed", "scalar", "Points failed"),
            OutputSpec::make("worst_margin_db", "scalar", "Smallest margin", "dB"),
            OutputSpec::make("passband_max_deviation_db", "scalar", "Worst pass-band error", "dB"),
            OutputSpec::make("stopband_attenuation_db", "scalar", "Achieved attenuation", "dB"),
            OutputSpec::make("meets_specification", "bool", "Floating-point design passes"),
            OutputSpec::make("meets_specification_quantised", "bool", "Quantised design passes"),
            OutputSpec::make("minimum_order", "scalar", "Lowest order that meets every point"),
            OutputSpec::make("quantisation_deviation_db", "scalar", "Response moved by", "dB"),
            OutputSpec::make("quantisation_overflows", "scalar", "Coefficients that did not fit"),
            OutputSpec::make("max_pole_radius_quantised", "scalar", "Largest quantised pole radius"),
            OutputSpec::make("verdict", "text", "The verified result, pass or fail"),
        };
        d.ops.push_back(std::move(op));
    }
    return d;
}

json::Value DspSystemModule::invoke(std::string_view op, const json::Value& args) const {
    if (op == "test_signals") {
        return op_test_signals(args);
    }
    if (op == "phase_types") {
        return op_phase_types(args);
    }
    if (op == "identify_transfer_function") {
        return op_identify_transfer_function(args);
    }
    if (op == "delay_elimination") {
        return op_delay_elimination(args);
    }
    if (op == "smooth_frequency_response") {
        return op_smooth_frequency_response(args);
    }
    if (op == "inverse_and_equalise") {
        return op_inverse_and_equalise(args);
    }
    if (op == "fir_arbitrary_magnitude") {
        return op_fir_arbitrary_magnitude(args);
    }
    if (op == "iir_prony") {
        return op_iir_prony(args);
    }
    if (op == "parametric_equaliser") {
        return op_parametric_equaliser(args);
    }
    if (op == "fir_phase_corrector") {
        return op_fir_phase_corrector(args);
    }
    if (op == "hilbert_transform") {
        return op_hilbert_transform(args);
    }
    if (op == "iir_allpass_corrector") {
        return op_iir_allpass_corrector(args);
    }
    if (op == "synthesise_to_specification") {
        return op_synthesise_to_specification(args);
    }
    unknown_op(name(), op);
}


}  // namespace yaskawa::study
