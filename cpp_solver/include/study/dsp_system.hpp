#ifndef YASKAWA_STUDY_DSP_SYSTEM_HPP
#define YASKAWA_STUDY_DSP_SYSTEM_HPP

// Course 3882 "Digital Signal Processing Algorithms and Systems" (M-403-02),
// all 16 sessions, plus Session 28 of 3884 (sensor signal conditioning).
//
// This is not a generic DSP toolbox. Every op in this module operates on a
// signal the GP8 actually produces:
//
//   * joint encoder position     - quantised, 1 kHz, the controller's feedback
//   * joint velocity             - differentiated encoder, noisy by construction
//   * motor current              - the torque proxy, carrying drive ripple
//   * force-torque channels      - low-passed by the sensor's own mechanics
//   * arm vibration              - the ring-down after a fast move, at the
//                                  first structural mode of the moved link
//
// When no measurement is available the signal is SYNTHESISED from
// study/gp8_model.hpp: the joint-side inertia (link inertia plus reflected
// rotor inertia), the viscous friction and an estimated joint-side torsional
// stiffness give the channel its mechanical pole and its resonance. Nothing
// here invents a plant out of nowhere, and nothing here filters abstract
// noise: the point of the course project is a real channel.
//
// The numeric core (DFT, radix-2 FFT with a Bluestein fallback, convolution,
// correlation, the complex frequency response of a coefficient pair, Prony and
// a least-squares solve) is exposed as free functions so later modules reuse it
// instead of writing a second copy.

#include "study/study_module.hpp"

#include <Eigen/Dense>

#include <complex>
#include <cstddef>
#include <string>
#include <string_view>
#include <vector>

namespace yaskawa::study {

namespace dsp {

using Complex = std::complex<double>;

// The GP8's controller runs its servo loop at 1 kHz (YRC1000micro), so that is
// the sampling rate every op defaults to.
constexpr double kGp8ControlRateHz = 1000.0;

// ESTIMATED joint-side torsional stiffness of the gearbox + link pair [N m/rad].
// It is not published by Yaskawa. With the link inertias of GP8_LINKS it places
// the first structural mode of the L axis near 25 Hz and of the U axis near
// 37 Hz, which is the band the arm audibly rings in after a fast move.
constexpr double kGp8JointStiffnessNmPerRad = 6.8e3;

// ---------------------------------------------------------------------------
// Transforms
// ---------------------------------------------------------------------------

[[nodiscard]] bool is_power_of_two(std::size_t n) noexcept;
[[nodiscard]] std::size_t next_power_of_two(std::size_t n) noexcept;

// The textbook definition, O(N^2). It is the reference the FFT is tested
// against, and it is what a student should read first.
[[nodiscard]] std::vector<Complex> dft(const std::vector<Complex>& x, bool inverse = false);
[[nodiscard]] std::vector<Complex> dft_real(const std::vector<double>& x);

// Radix-2 decimation in time when the length is a power of two, Bluestein's
// chirp-z transform otherwise. The fallback is a transform, not a truncation
// and not a zero-pad: it returns exactly the N-point DFT of the N input
// samples for any N.
[[nodiscard]] std::vector<Complex> fft(const std::vector<Complex>& x, bool inverse = false);
[[nodiscard]] std::vector<Complex> fft_real(const std::vector<double>& x);
[[nodiscard]] std::vector<double> ifft_real(const std::vector<Complex>& x);

// ---------------------------------------------------------------------------
// Convolution and correlation
// ---------------------------------------------------------------------------

[[nodiscard]] std::vector<double> convolve(const std::vector<double>& a,
                                           const std::vector<double>& b);
[[nodiscard]] std::vector<double> convolve_fft(const std::vector<double>& a,
                                               const std::vector<double>& b);

struct CorrelationResult {
    int lag_min = 0;                // lag of values[0]
    std::vector<double> values;     // R_xy[lag], lag_min .. lag_min + size - 1
    int peak_lag = 0;               // argmax |R_xy|
    double peak_value = 0.0;        // R_xy[peak_lag]
};

// R_xy[lag] = sum_n x[n] y[n + lag]. With y[n] = x[n - d] the peak sits at
// lag = +d, i.e. a positive peak lag means y lags x by that many samples.
[[nodiscard]] CorrelationResult cross_correlate(const std::vector<double>& x,
                                                const std::vector<double>& y, int max_lag = -1);
[[nodiscard]] CorrelationResult auto_correlate(const std::vector<double>& x, int max_lag = -1);

// ---------------------------------------------------------------------------
// Rational transfer functions
// ---------------------------------------------------------------------------

struct Tf {
    std::vector<double> b{1.0};   // numerator, ascending powers of z^-1
    std::vector<double> a{1.0};   // denominator, a[0] must not be zero
};

// H(e^{j omega}) = B(e^{-j omega}) / A(e^{-j omega}), omega in rad/sample.
[[nodiscard]] Complex frequency_response(const std::vector<double>& b, const std::vector<double>& a,
                                         double omega);

// -d(arg H)/d omega in samples, evaluated analytically from the coefficients.
[[nodiscard]] double group_delay(const std::vector<double>& b, const std::vector<double>& a,
                                 double omega);

// Direct-form-I difference equation. Returns y with the same length as x.
[[nodiscard]] std::vector<double> apply_filter(const std::vector<double>& b,
                                               const std::vector<double>& a,
                                               const std::vector<double>& x);

[[nodiscard]] std::vector<double> impulse_response(const std::vector<double>& b,
                                                   const std::vector<double>& a, std::size_t n);

// Roots of c[0] z^N + c[1] z^{N-1} + ... + c[N], via the companion matrix.
[[nodiscard]] std::vector<Complex> polynomial_roots(const std::vector<double>& coefficients);

// min_x || A x - y ||_2, rank-revealing so an under-determined design still
// returns the minimum-norm answer instead of a NaN.
[[nodiscard]] Eigen::VectorXd least_squares(const Eigen::MatrixXd& A, const Eigen::VectorXd& y);

// ---------------------------------------------------------------------------
// Prony's method (session 11)
// ---------------------------------------------------------------------------

struct PronyFit {
    std::vector<double> b;          // numerator, q + 1 taps
    std::vector<double> a;          // denominator, p + 1 taps, a[0] = 1
    double relative_error = 0.0;    // ||h_fit - h|| / ||h||
    bool stable = true;             // every pole strictly inside the unit circle
    double max_pole_radius = 0.0;
};

[[nodiscard]] PronyFit prony(const std::vector<double>& h, std::size_t numerator_order,
                             std::size_t denominator_order);

// ---------------------------------------------------------------------------
// Analytic signal (sessions 13 and 14)
// ---------------------------------------------------------------------------

// Odd-length FIR Hilbert kernel, h[n] = 2 sin^2(pi n / 2) / (pi n), Hamming
// windowed. Group delay is exactly (length - 1) / 2 samples.
[[nodiscard]] std::vector<double> hilbert_fir_kernel(std::size_t length);

// Quadrature channel from the one-sided spectrum. Exact for a bin-centred
// sinusoid, with the usual wrap-around error at the record edges.
[[nodiscard]] std::vector<double> hilbert_fft(const std::vector<double>& x);

// ---------------------------------------------------------------------------
// The GP8 channel the ops identify, filter and equalise
// ---------------------------------------------------------------------------

struct JointChannel {
    Tf tf;                                  // discrete model at `fs`
    double fs = kGp8ControlRateHz;
    double mechanical_time_constant = 0.0;  // J_joint / b_viscous [s]
    double servo_bandwidth_hz = 0.0;        // closed-loop velocity bandwidth
    double resonance_hz = 0.0;              // first structural mode used
    double resonance_hz_physical = 0.0;     // before any Nyquist clamp
    double damping_ratio = 0.0;
    double joint_inertia = 0.0;             // joint side, incl. reflected rotor
    double link_inertia = 0.0;              // structure only
    bool resonance_clamped = false;         // true if the mode is above 0.45 fs
};

// `joint` is 1-based, as Yaskawa numbers the axes (1 = S ... 6 = T).
[[nodiscard]] JointChannel joint_channel(std::size_t joint, double fs = kGp8ControlRateHz,
                                         double damping_ratio = 0.03);

}  // namespace dsp

class DspSystemModule final : public StudyModule {
public:
    [[nodiscard]] std::string_view name() const noexcept override { return "dsp_system"; }
    [[nodiscard]] ModuleDescription describe() const override;
    [[nodiscard]] json::Value invoke(std::string_view op, const json::Value& args) const override;
};

}  // namespace yaskawa::study

#endif  // YASKAWA_STUDY_DSP_SYSTEM_HPP
