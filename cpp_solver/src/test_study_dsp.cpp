// Test suite for the dsp_system study module (course 3882, M-403-02).
//
// Plain asserts in the style of src/test_study_modules.cpp - there is no gtest
// in this project and there will not be one. Every check asserts a property:
// the FFT agrees with the direct DFT, Parseval holds, FFT convolution equals
// the direct sum, a correlation peak lands on the delay that was inserted, a
// linear-phase FIR has a constant group delay, a designed filter meets the
// specification it claims, Prony recovers a known IIR system's coefficients,
// the Hilbert transform of a cosine is a sine, and every op rejects malformed
// input by throwing StudyError instead of returning garbage.

#include "study/dsp_system.hpp"
#include "study/json.hpp"
#include "study/study_module.hpp"

#include <Eigen/Dense>

#include <algorithm>
#include <cmath>
#include <complex>
#include <iostream>
#include <numbers>
#include <random>
#include <string>
#include <vector>

using yaskawa::study::json::Value;
namespace json = yaskawa::study::json;
namespace study = yaskawa::study;
namespace dsp = yaskawa::study::dsp;

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

[[nodiscard]] std::vector<double> random_real(std::size_t n, unsigned seed) {
    std::mt19937 rng(seed);
    std::uniform_real_distribution<double> dist(-1.0, 1.0);
    std::vector<double> out(n, 0.0);
    for (std::size_t i = 0; i < n; ++i) {
        out[i] = dist(rng);
    }
    return out;
}

[[nodiscard]] std::vector<dsp::Complex> random_complex(std::size_t n, unsigned seed) {
    std::mt19937 rng(seed);
    std::uniform_real_distribution<double> dist(-1.0, 1.0);
    std::vector<dsp::Complex> out(n, dsp::Complex(0.0, 0.0));
    for (std::size_t i = 0; i < n; ++i) {
        out[i] = dsp::Complex(dist(rng), dist(rng));
    }
    return out;
}

[[nodiscard]] std::vector<double> value_to_vector(const Value& value) {
    std::vector<double> out;
    const Value* row = &value;
    if (value.is_array() && value.size() == 1 && value[0].is_array()) {
        row = &value[0];
    }
    for (std::size_t i = 0; i < row->size(); ++i) {
        out.push_back((*row)[i].as_double());
    }
    return out;
}

// ---------------------------------------------------------------------------
// The numeric core
// ---------------------------------------------------------------------------

void test_transforms() {
    std::cout << "\n--- transforms ---\n";

    // The headline check: the FFT must reproduce the direct DFT on random data
    // at several lengths, including lengths that are not a power of two (where
    // the Bluestein fallback runs).
    const std::vector<std::size_t> lengths = {8, 12, 16, 64, 100, 256, 333, 1000, 1024};
    double worst = 0.0;
    std::size_t worst_length = 0;
    for (const std::size_t n : lengths) {
        const std::vector<dsp::Complex> x = random_complex(n, static_cast<unsigned>(1000 + n));
        const std::vector<dsp::Complex> slow = dsp::dft(x, false);
        const std::vector<dsp::Complex> fast = dsp::fft(x, false);
        double error = 0.0;
        for (std::size_t k = 0; k < n; ++k) {
            error = std::max(error, std::abs(slow[k] - fast[k]));
        }
        if (error > worst) {
            worst = error;
            worst_length = n;
        }
        check(error < 1e-10, "FFT matches the direct DFT at N = " + std::to_string(n) +
                                 (dsp::is_power_of_two(n) ? " (radix-2)" : " (Bluestein fallback)") +
                                 ", max error " + json::number_to_string(error));
    }
    std::cout << "       worst FFT-vs-DFT agreement: " << worst << " at N = " << worst_length
              << "\n";

    // Round trip, including a non-power-of-two length.
    for (const std::size_t n : {64u, 333u}) {
        const std::vector<dsp::Complex> x = random_complex(n, static_cast<unsigned>(77 + n));
        const std::vector<dsp::Complex> back = dsp::fft(dsp::fft(x, false), true);
        double error = 0.0;
        for (std::size_t i = 0; i < n; ++i) {
            error = std::max(error, std::abs(back[i] - x[i]));
        }
        check(error < 1e-10,
              "inverse FFT round trip at N = " + std::to_string(n) + ", max error " +
                  json::number_to_string(error));
    }

    // Parseval: sum |x|^2 = (1/N) sum |X|^2.
    for (const std::size_t n : {256u, 100u}) {
        const std::vector<double> x = random_real(n, static_cast<unsigned>(5 + n));
        double time_energy = 0.0;
        for (const double v : x) {
            time_energy += v * v;
        }
        const std::vector<dsp::Complex> X = dsp::fft_real(x);
        double spectral_energy = 0.0;
        for (const auto& value : X) {
            spectral_energy += std::norm(value);
        }
        spectral_energy /= static_cast<double>(n);
        check_near(spectral_energy, time_energy, 1e-10 * std::max(1.0, time_energy),
                   "Parseval's theorem holds at N = " + std::to_string(n));
    }

    // Convolution by FFT equals the direct sum.
    const std::vector<double> a = random_real(40, 11);
    const std::vector<double> b = random_real(25, 12);
    const std::vector<double> direct = dsp::convolve(a, b);
    const std::vector<double> fast = dsp::convolve_fft(a, b);
    check(direct.size() == fast.size() && direct.size() == 64,
          "FFT convolution has the direct convolution's length");
    double convolution_error = 0.0;
    for (std::size_t i = 0; i < direct.size(); ++i) {
        convolution_error = std::max(convolution_error, std::abs(direct[i] - fast[i]));
    }
    check(convolution_error < 1e-10, "FFT convolution matches direct convolution, max error " +
                                         json::number_to_string(convolution_error));
}

void test_correlation() {
    std::cout << "\n--- correlation ---\n";

    const std::vector<double> x = random_real(512, 31);
    for (const int delay : {0, 1, 7, 23, 64}) {
        std::vector<double> y(x.size(), 0.0);
        for (std::size_t i = static_cast<std::size_t>(delay); i < x.size(); ++i) {
            y[i] = x[i - static_cast<std::size_t>(delay)];
        }
        const dsp::CorrelationResult r = dsp::cross_correlate(x, y, 128);
        check(r.peak_lag == delay, "cross-correlation recovers a " + std::to_string(delay) +
                                       "-sample delay exactly (found " +
                                       std::to_string(r.peak_lag) + ")");
    }

    // With noise on the delayed copy the peak must still be exact.
    std::vector<double> noisy(x.size(), 0.0);
    std::mt19937 rng(9);
    std::normal_distribution<double> noise(0.0, 0.2);
    for (std::size_t i = 11; i < x.size(); ++i) {
        noisy[i] = x[i - 11] + noise(rng);
    }
    const dsp::CorrelationResult r = dsp::cross_correlate(x, noisy, 64);
    check(r.peak_lag == 11, "cross-correlation recovers an 11-sample delay through noise");

    const dsp::CorrelationResult self = dsp::auto_correlate(x, 32);
    check(self.peak_lag == 0, "auto-correlation peaks at zero lag");
    double energy = 0.0;
    for (const double v : x) {
        energy += v * v;
    }
    check_near(self.peak_value, energy, 1e-9,
               "auto-correlation at zero lag equals the signal energy");
}

void test_responses() {
    std::cout << "\n--- frequency response and group delay ---\n";

    // A symmetric (linear-phase) FIR must have a group delay of exactly
    // (N-1)/2 samples everywhere its magnitude is non-negligible.
    const std::size_t length = 33;
    const std::size_t m = length / 2;
    const double cutoff = 0.25;  // normalised to fs
    std::vector<double> b(length, 0.0);
    for (std::size_t i = 0; i < length; ++i) {
        const double n = static_cast<double>(i) - static_cast<double>(m);
        const double ideal =
            (n == 0.0) ? 2.0 * cutoff : std::sin(2.0 * kPi * cutoff * n) / (kPi * n);
        const double window = 0.54 - 0.46 * std::cos(2.0 * kPi * static_cast<double>(i) /
                                                     static_cast<double>(length - 1));
        b[i] = ideal * window;
    }
    const double expected_delay = static_cast<double>(length - 1) / 2.0;
    double worst = 0.0;
    std::size_t evaluated = 0;
    for (std::size_t i = 0; i <= 200; ++i) {
        const double w = kPi * static_cast<double>(i) / 200.0;
        if (std::abs(dsp::frequency_response(b, {1.0}, w)) < 1e-2) {
            continue;  // the group delay is undefined at a magnitude null
        }
        ++evaluated;
        worst = std::max(worst, std::abs(dsp::group_delay(b, {1.0}, w) - expected_delay));
    }
    check(evaluated > 50, "the linear-phase test evaluated a useful part of the band (" +
                              std::to_string(evaluated) + " points)");
    check(worst < 1e-9, "a symmetric FIR has a constant group delay of " +
                            json::number_to_string(expected_delay) + " samples, worst error " +
                            json::number_to_string(worst));

    // An all-pass section has unit magnitude at every frequency, exactly.
    const double radius = 0.7;
    const double theta = 0.3 * kPi;
    const double c = -2.0 * radius * std::cos(theta);
    const std::vector<double> ap_b = {radius * radius, c, 1.0};
    const std::vector<double> ap_a = {1.0, c, radius * radius};
    double allpass_error = 0.0;
    for (std::size_t i = 0; i <= 256; ++i) {
        const double w = kPi * static_cast<double>(i) / 256.0;
        allpass_error =
            std::max(allpass_error, std::abs(std::abs(dsp::frequency_response(ap_b, ap_a, w)) - 1.0));
    }
    check(allpass_error < 1e-12, "an all-pass section's magnitude is 1 to " +
                                     json::number_to_string(allpass_error));

    // Roots of a known polynomial: z^2 - 1.6 z + 0.68 has 0.8 +/- 0.2j.
    const std::vector<dsp::Complex> roots = dsp::polynomial_roots({1.0, -1.6, 0.68});
    check(roots.size() == 2, "a quadratic has two roots");
    double root_error = 1e9;
    for (const auto& root : roots) {
        root_error = std::min(root_error, std::abs(root - dsp::Complex(0.8, 0.2)));
    }
    check(root_error < 1e-12, "the exam's biquad poles come out at 0.8 +/- 0.2j, error " +
                                  json::number_to_string(root_error));
    check_near(std::abs(roots[0]), std::sqrt(0.68), 1e-12, "the pole radius is sqrt(0.68) = 0.825");

    // The least-squares solver on an exactly determined system.
    Eigen::MatrixXd A(3, 2);
    A << 1.0, 0.0, 0.0, 1.0, 1.0, 1.0;
    Eigen::VectorXd y(3);
    y << 1.0, 2.0, 3.0;
    const Eigen::VectorXd solution = dsp::least_squares(A, y);
    check_near(solution(0), 1.0, 1e-12, "least squares solves a consistent system (x0)");
    check_near(solution(1), 2.0, 1e-12, "least squares solves a consistent system (x1)");
}

void test_prony() {
    std::cout << "\n--- Prony ---\n";

    const dsp::JointChannel channel = dsp::joint_channel(2, dsp::kGp8ControlRateHz);
    check(channel.tf.a.size() == 4, "the joint channel is a third-order all-pole model");
    check(channel.resonance_hz > 5.0 && channel.resonance_hz < 0.5 * channel.fs,
          "the structural mode sits inside the band, at " +
              json::number_to_string(channel.resonance_hz) + " Hz");

    const std::vector<double> h = dsp::impulse_response(channel.tf.b, channel.tf.a, 128);
    const dsp::PronyFit fit = dsp::prony(h, 0, 3);
    check(fit.a.size() == channel.tf.a.size(), "Prony returns the requested denominator order");
    double worst = 0.0;
    for (std::size_t k = 0; k < fit.a.size(); ++k) {
        worst = std::max(worst, std::abs(fit.a[k] - channel.tf.a[k]));
    }
    check(worst < 1e-9, "Prony recovers a known IIR system's denominator to 1e-9 (worst "
                        "coefficient error " + json::number_to_string(worst) + ")");
    check_near(fit.b[0], channel.tf.b[0], 1e-9 * std::abs(channel.tf.b[0]),
               "Prony recovers the numerator gain");
    check(fit.relative_error < 1e-9, "the Prony fit reproduces the impulse response, relative "
                                     "error " + json::number_to_string(fit.relative_error));
    check(fit.stable && fit.max_pole_radius < 1.0,
          "the recovered poles are inside the unit circle (radius " +
              json::number_to_string(fit.max_pole_radius) + ")");

    // An over-short record must be refused rather than silently padded.
    bool threw = false;
    try {
        const dsp::PronyFit bad = dsp::prony({1.0, 0.5}, 2, 3);
        (void)bad;
    } catch (const study::StudyError&) {
        threw = true;
    }
    check(threw, "Prony refuses a record shorter than q + p + 2 samples");
}

void test_hilbert() {
    std::cout << "\n--- Hilbert transform ---\n";

    const std::size_t n = 512;
    const std::size_t bin = 32;  // bin-centred, so the record has no leakage
    std::vector<double> cosine(n, 0.0);
    std::vector<double> sine(n, 0.0);
    for (std::size_t i = 0; i < n; ++i) {
        const double phase = 2.0 * kPi * static_cast<double>(bin) * static_cast<double>(i) /
                             static_cast<double>(n);
        cosine[i] = std::cos(phase);
        sine[i] = std::sin(phase);
    }

    const std::vector<double> quadrature = dsp::hilbert_fft(cosine);
    double fft_error = 0.0;
    for (std::size_t i = 0; i < n; ++i) {
        fft_error = std::max(fft_error, std::abs(quadrature[i] - sine[i]));
    }
    check(fft_error < 1e-10, "the FFT Hilbert transform of a cosine is a sine to 1e-10 (error " +
                                 json::number_to_string(fft_error) + ")");

    // The FIR kernel is windowed and finite, so it is accurate in the interior
    // of the record and not at its edges. 0.02 is the stated tolerance.
    const std::size_t kernel_length = 101;
    const std::size_t m = kernel_length / 2;
    const std::vector<double> kernel = dsp::hilbert_fir_kernel(kernel_length);
    const std::vector<double> filtered = dsp::apply_filter(kernel, {1.0}, cosine);
    double fir_error = 0.0;
    for (std::size_t i = kernel_length; i + 8 < n; ++i) {
        fir_error = std::max(fir_error, std::abs(filtered[i] - sine[i - m]));
    }
    check(fir_error < 0.02, "the FIR Hilbert transform of a cosine is a sine to 0.02 away from "
                            "the edges (error " + json::number_to_string(fir_error) + ")");

    // The kernel is antisymmetric with a zero centre tap.
    check_near(kernel[m], 0.0, 1e-15, "the Hilbert kernel's centre tap is zero");
    double symmetry_error = 0.0;
    for (std::size_t i = 0; i < kernel_length; ++i) {
        symmetry_error =
            std::max(symmetry_error, std::abs(kernel[i] + kernel[kernel_length - 1 - i]));
    }
    check(symmetry_error < 1e-15, "the Hilbert kernel is antisymmetric");

    bool threw = false;
    try {
        const std::vector<double> bad = dsp::hilbert_fir_kernel(100);
        (void)bad;
    } catch (const study::StudyError&) {
        threw = true;
    }
    check(threw, "an even Hilbert kernel length is refused");
}

// ---------------------------------------------------------------------------
// The module
// ---------------------------------------------------------------------------

void test_description() {
    std::cout << "\n--- self-description ---\n";

    const study::DspSystemModule module;
    const study::ModuleDescription d = module.describe();
    check(module.name() == "dsp_system", "the module names itself dsp_system");
    check(d.course.id == 3882 && d.course.code == "M-403-02", "the course reference is 3882");
    check(d.source == "cpp_solver/include/study/dsp_system.hpp", "the source path is the header");
    check(d.topics.size() == 17, "all 16 sessions plus 3884's session 28 are listed (" +
                                     std::to_string(d.topics.size()) + ")");
    check(d.ops.size() == 13, "the module exposes 13 ops (" + std::to_string(d.ops.size()) + ")");

    bool complete = true;
    for (const auto& op : d.ops) {
        complete = complete && !op.name.empty() && !op.title.empty() && !op.formula.empty() &&
                   op.explain.size() > 80 && !op.outputs.empty();
        for (const auto& param : op.params) {
            complete = complete && !param.name.empty() && !param.type.empty() &&
                       !param.label.empty() && !param.default_value.is_null();
            if (param.type == "enum") {
                complete = complete && !param.options.empty();
            }
        }
        for (const auto& output : op.outputs) {
            complete = complete && !output.name.empty() && !output.type.empty() &&
                       !output.label.empty();
        }
    }
    check(complete, "every op carries a formula, an explanation, typed params with defaults and "
                    "typed outputs");

    // The description must round-trip through the JSON writer and reader.
    const Value as_json = d.to_json();
    bool round_trip = false;
    try {
        round_trip = (json::parse(json::dump(as_json)) == as_json);
    } catch (const std::exception& error) {
        std::cout << "       parse threw " << error.what() << "\n";
    }
    check(round_trip, "the description survives a dump/parse round trip");
}

void test_defaults_run() {
    std::cout << "\n--- every op runs on its defaults ---\n";

    const study::DspSystemModule module;
    const study::ModuleDescription d = module.describe();
    for (const auto& op : d.ops) {
        bool ok = false;
        std::string detail;
        try {
            const Value result = module.invoke(op.name, Value::object());
            ok = result.is_object() && result.size() > 3 && result["verdict"].is_string() &&
                 result["verdict"].as_string().size() > 40;
            if (!ok) {
                detail = " (result had " + std::to_string(result.size()) + " fields)";
            }
        } catch (const std::exception& error) {
            detail = std::string(" (threw ") + error.what() + ")";
        }
        check(ok, "op " + op.name + " runs on its defaults and explains itself" + detail);
    }

    bool threw = false;
    try {
        const Value result = module.invoke("no_such_op", Value::object());
        (void)result;
    } catch (const study::StudyError&) {
        threw = true;
    }
    check(threw, "an unknown op throws StudyError");
}

void test_rejects_bad_input() {
    std::cout << "\n--- malformed input is refused ---\n";

    const study::DspSystemModule module;
    const study::ModuleDescription d = module.describe();

    // One blob of out-of-range values. Every op reads at least one of these,
    // so every op must refuse it with StudyError rather than compute garbage.
    Value bad_ranges = Value::object();
    bad_ranges.set("length", Value(3));            // below the 16-sample minimum
    bad_ranges.set("fs", Value(-5.0));             // negative sampling rate
    bad_ranges.set("order", Value(-1));            // negative order
    bad_ranges.set("bits", Value(99));             // wider than any Q format here
    bad_ranges.set("joint", Value(9));             // the GP8 has six axes
    bad_ranges.set("sections", Value(0));          // no filter at all
    bad_ranges.set("denominator_order", Value(0));
    for (const auto& op : d.ops) {
        bool threw_study_error = false;
        std::string detail;
        try {
            const Value result = module.invoke(op.name, bad_ranges);
            detail = " (returned a result instead)";
            (void)result;
        } catch (const study::StudyError&) {
            threw_study_error = true;
        } catch (const std::exception& error) {
            detail = std::string(" (threw the wrong type: ") + error.what() + ")";
        }
        check(threw_study_error, "op " + op.name + " refuses out-of-range parameters" + detail);
    }

    // A malformed enum, a malformed signal and a non-numeric array.
    Value bad_enum = Value::object();
    bad_enum.set("signal", Value("sawtooth"));
    bool enum_threw = false;
    try {
        const Value result = module.invoke("test_signals", bad_enum);
        (void)result;
    } catch (const study::StudyError&) {
        enum_threw = true;
    }
    check(enum_threw, "an unknown enum option is refused");

    Value bad_array = Value::object();
    bad_array.set("b", Value::array({Value(1.0), Value("not a number")}));
    bool array_threw = false;
    try {
        const Value result = module.invoke("phase_types", bad_array);
        (void)result;
    } catch (const study::StudyError&) {
        array_threw = true;
    }
    check(array_threw, "a coefficient array with a string in it is refused");

    Value even_window = Value::object();
    even_window.set("window_length", Value(8));
    bool window_threw = false;
    try {
        const Value result = module.invoke("smooth_frequency_response", even_window);
        (void)result;
    } catch (const study::StudyError&) {
        window_threw = true;
    }
    check(window_threw, "an even smoothing window is refused");

    Value long_delay = Value::object();
    long_delay.set("length", Value(16));
    long_delay.set("true_delay", Value(64));
    bool delay_threw = false;
    try {
        const Value result = module.invoke("delay_elimination", long_delay);
        (void)result;
    } catch (const study::StudyError&) {
        delay_threw = true;
    }
    check(delay_threw, "a delay longer than the record is refused");
}

void test_fir_meets_specification() {
    std::cout << "\n--- a designed FIR meets its own specification ---\n";

    const study::DspSystemModule module;
    Value args = Value::object();
    args.set("method", Value("windowed"));
    args.set("window", Value("blackman"));
    args.set("order", Value(120));
    args.set("specification", Value("servo_lowpass"));
    args.set("pass_edge", Value(60.0));
    args.set("stop_edge", Value(140.0));
    args.set("stopband_db", Value(60.0));
    args.set("passband_ripple_db", Value(1.0));
    const Value result = module.invoke("fir_arbitrary_magnitude", args);

    check(result["meets_specification"].as_bool(),
          "the Blackman-windowed order-120 design meets the 1 dB / 60 dB servo mask");
    check_near(result["points_failed"].as_double(), 0.0, 0.0,
               "no specification point fails in the op's own verification table");

    // Independent verification: recompute the response from the returned
    // coefficients, rather than trusting the op's own numbers.
    const std::vector<double> b = value_to_vector(result["coefficients"]);
    check(b.size() == 121, "121 taps came back for order 120 (" + std::to_string(b.size()) + ")");
    const double fs = 1000.0;
    double pass_deviation = 0.0;
    double stop_attenuation = 1000.0;
    for (std::size_t i = 0; i <= 500; ++i) {
        const double hz = 0.5 * fs * static_cast<double>(i) / 500.0;
        const double w = 2.0 * kPi * hz / fs;
        const double db =
            20.0 * std::log10(std::max(std::abs(dsp::frequency_response(b, {1.0}, w)), 1e-12));
        if (hz <= 60.0) {
            pass_deviation = std::max(pass_deviation, std::abs(db));
        } else if (hz >= 140.0) {
            stop_attenuation = std::min(stop_attenuation, -db);
        }
    }
    check(pass_deviation <= 1.0, "recomputed pass-band deviation " +
                                     json::number_to_string(pass_deviation) + " dB is within 1 dB");
    check(stop_attenuation >= 60.0, "recomputed stop-band attenuation " +
                                        json::number_to_string(stop_attenuation) +
                                        " dB reaches 60 dB");

    // The window choice is what buys the attenuation - the exam's point.
    Value hamming = args;
    hamming.set("window", Value("hamming"));
    const Value weak = module.invoke("fir_arbitrary_magnitude", hamming);
    check(weak["stopband_attenuation_db"].as_double() + 10.0 <
              result["stopband_attenuation_db"].as_double(),
          "Blackman beats Hamming by more than 10 dB at the same order (" +
              json::number_to_string(weak["stopband_attenuation_db"].as_double()) + " dB vs " +
              json::number_to_string(result["stopband_attenuation_db"].as_double()) + " dB)");

    // The least-squares method on the same mask must also be verifiable.
    Value least_squares = args;
    least_squares.set("method", Value("least_squares"));
    least_squares.set("stopband_db", Value(30.0));
    const Value ls = module.invoke("fir_arbitrary_magnitude", least_squares);
    check(ls["stopband_attenuation_db"].as_double() >= 30.0,
          "the weighted least-squares design reaches its 30 dB stop-band (" +
              json::number_to_string(ls["stopband_attenuation_db"].as_double()) + " dB)");
    check(ls["group_delay_samples"].as_double() == 60.0,
          "the linear-phase group delay is exactly order/2 samples");
}

void test_ops_properties() {
    std::cout << "\n--- op properties ---\n";

    const study::DspSystemModule module;

    // Delay elimination must return the delay that was injected.
    for (const int delay : {0, 7, 19}) {
        Value args = Value::object();
        args.set("true_delay", Value(delay));
        args.set("length", Value(512));
        args.set("snr_db", Value(20.0));
        const Value result = module.invoke("delay_elimination", args);
        check_near(result["delay_samples"].as_double(), static_cast<double>(delay), 0.0,
                   "delay_elimination recovers the injected " + std::to_string(delay) +
                       "-sample transport delay exactly");
        if (delay > 0) {
            check(result["rms_error_after"].as_double() < result["rms_error_before"].as_double(),
                  "removing the delay reduces the alignment error at d = " +
                      std::to_string(delay));
        }
    }

    // Prony through the op: an exact impulse response must be recovered.
    Value prony_args = Value::object();
    prony_args.set("numerator_order", Value(0));
    prony_args.set("denominator_order", Value(3));
    prony_args.set("impulse_length", Value(128));
    const Value prony_result = module.invoke("iir_prony", prony_args);
    check(prony_result["relative_error"].as_double() < 1e-9,
          "iir_prony fits the channel's own impulse response to 1e-9 (relative error " +
              json::number_to_string(prony_result["relative_error"].as_double()) + ")");
    check(prony_result["stable"].as_bool(), "the fitted model is stable");
    const std::vector<double> fitted_a = value_to_vector(prony_result["a"]);
    const std::vector<double> true_a = value_to_vector(prony_result["a_true"]);
    double coefficient_error = 0.0;
    for (std::size_t i = 0; i < fitted_a.size() && i < true_a.size(); ++i) {
        coefficient_error = std::max(coefficient_error, std::abs(fitted_a[i] - true_a[i]));
    }
    check(coefficient_error < 1e-9, "iir_prony reproduces the true denominator coefficients to "
                                    "1e-9 (worst " + json::number_to_string(coefficient_error) +
                                    ")");

    // Identification: the generalised correlation method must beat ARX on a
    // noisy record, and the estimate must be close to the truth when clean.
    Value clean = Value::object();
    clean.set("snr_db", Value(60.0));
    clean.set("length", Value(2048));
    const Value identified = module.invoke("identify_transfer_function", clean);
    check(identified["rms_magnitude_error_db"].as_double() < 1.0,
          "the correlation estimate is within 3 dB RMS of the true magnitude across the "
          "identifiable band at 60 dB SNR (" +
              json::number_to_string(identified["rms_magnitude_error_db"].as_double()) +
              " dB RMS, worst " +
              json::number_to_string(identified["max_magnitude_error_db"].as_double()) + " dB)");
    check(identified["required_length"].as_double() > 0.0,
          "a record length is reported for the wanted accuracy");

    // The course's central claim: on the same record the correlation method
    // beats the least-squares fit, whose regressors carry the output noise.
    Value arx = clean;
    arx.set("method", Value("least_squares_arx"));
    const Value by_arx = module.invoke("identify_transfer_function", arx);
    check(identified["rms_magnitude_error_db"].as_double() <
              by_arx["rms_magnitude_error_db"].as_double(),
          "the generalised correlation method beats least-squares ARX on the same record (" +
              json::number_to_string(identified["rms_magnitude_error_db"].as_double()) +
              " dB vs " + json::number_to_string(by_arx["rms_magnitude_error_db"].as_double()) +
              " dB RMS)");

    // The all-pass corrector must not change the magnitude.
    const Value allpass = module.invoke("iir_allpass_corrector", Value::object());
    check(allpass["max_magnitude_deviation_db"].as_double() < 1e-9,
          "the all-pass cascade's magnitude is flat to 1e-9 dB (" +
              json::number_to_string(allpass["max_magnitude_deviation_db"].as_double()) + ")");
    check(allpass["max_channel_change_db"].as_double() < 0.01,
          "the corrected channel's magnitude is unchanged to better than the 0.01 dB the course "
          "asks for (" + json::number_to_string(allpass["max_channel_change_db"].as_double()) +
              " dB)");
    check(allpass["misalignment_after_samples"].as_double() <
              allpass["misalignment_before_samples"].as_double(),
          "the all-pass network improves the encoder / force-torque alignment");

    // The FIR phase corrector must flatten the group delay it was asked to.
    const Value corrector = module.invoke("fir_phase_corrector", Value::object());
    check(corrector["delay_ripple_after"].as_double() < corrector["delay_ripple_before"].as_double(),
          "the FIR corrector reduces the group-delay ripple (" +
              json::number_to_string(corrector["delay_ripple_before"].as_double()) + " -> " +
              json::number_to_string(corrector["delay_ripple_after"].as_double()) + " samples)");

    // Smoothing must trade variance for bias, visibly in both directions.
    Value wide = Value::object();
    wide.set("window_length", Value(31));
    wide.set("snr_db", Value(5.0));
    const Value smoothed = module.invoke("smooth_frequency_response", wide);
    check(smoothed["noise_std_smoothed_db"].as_double() <
              smoothed["noise_std_raw_db"].as_double(),
          "a wide smoothing window reduces the scatter of the estimate");
    check(smoothed["peak_loss_db"].as_double() > 0.0,
          "and pays for it by flattening the resonance peak (" +
              json::number_to_string(smoothed["peak_loss_db"].as_double()) + " dB lost)");

    // Quantisation must be reported honestly: Q1.7 cannot hold -1.6.
    Value short_word = Value::object();
    short_word.set("bits", Value(8));
    short_word.set("filter_type", Value("peaking"));
    short_word.set("f0", Value(25.0));
    short_word.set("q", Value(8.0));
    short_word.set("gain_db", Value(-18.0));
    const Value quantised = module.invoke("parametric_equaliser", short_word);
    check(quantised["largest_coefficient"].as_double() > 1.0,
          "a high-Q section has a coefficient outside [-1, 1) (" +
              json::number_to_string(quantised["largest_coefficient"].as_double()) + ")");
    check(quantised["overflow_count"].as_double() > 0.0,
          "and the op reports the Q-format overflow instead of hiding it");
    const Value wide_word = module.invoke("parametric_equaliser", Value::object());
    check(wide_word["max_deviation_db"].as_double() <
              quantised["max_deviation_db"].as_double(),
          "a 16-bit word length distorts the response less than an 8-bit one");

    // Phase classification must agree with the preset that built the filter.
    const std::vector<std::pair<std::string, std::string>> expectations = {
        {"minimum_phase", "minimum phase"},
        {"maximum_phase", "maximum phase"},
        {"mixed_phase", "mixed phase"},
    };
    for (const auto& [preset, expected] : expectations) {
        Value args = Value::object();
        args.set("preset", Value(preset));
        const Value result = module.invoke("phase_types", args);
        check(result["phase_class"].as_string() == expected,
              "preset " + preset + " is classified as " + expected + " (got '" +
                  result["phase_class"].as_string() + "')");
    }
    Value linear = Value::object();
    linear.set("preset", Value("linear_phase"));
    const Value linear_result = module.invoke("phase_types", linear);
    check(linear_result["is_linear_phase"].as_bool(),
          "the linear-phase preset is recognised as linear phase");

    // Synthesis to the examination's specification must pass and say so.
    Value spec = Value::object();
    spec.set("specification", Value("antialias_exam"));
    spec.set("family", Value("fir_windowed"));
    spec.set("window", Value("blackman"));
    spec.set("order", Value(96));
    const Value synthesis = module.invoke("synthesise_to_specification", spec);
    check(synthesis["meets_specification"].as_bool(),
          "the order-96 Blackman design meets the 8 kHz anti-alias mask");
    check(synthesis["points_failed"].as_double() == 0.0, "every specification point passes");
    check(synthesis["minimum_order"].as_double() > 0.0 &&
              synthesis["minimum_order"].as_double() <= 96.0,
          "the order search finds a lower order that still meets it (" +
              json::number_to_string(synthesis["minimum_order"].as_double()) + ")");
    check(synthesis["quantisation_deviation_db"].as_double() >= 0.0,
          "the quantised response is compared against the floating-point one");

    // Test-signal selection must prefer a broadband excitation.
    const Value signals = module.invoke("test_signals", Value::object());
    const std::string best = signals["best_signal"].as_string();
    check(best == "prbs" || best == "multisine" || best == "white_noise" || best == "chirp",
          "the best identifier is a broadband signal, not a step (" + best + ")");

    // Hilbert: the envelope of an amplitude-modulated vibration must find the
    // modulation rate.
    //
    // The carrier of the bearing_fault signal is the joint's own structural
    // resonance, sqrt(k / J_structural) / 2pi, so it moves with the link
    // inertia. With the old ESTIMATED box inertia for the L link (J = 0.2700
    // kg m^2) the carrier sat at 25.26 Hz; with the inertia integrated from the
    // CAD mesh and the density fitted to the GP8 datasheet mass of 32 kg
    // (J = 0.4149 kg m^2) it is 20.38 Hz. The 23 Hz rate this check
    // used to ask for is above that carrier, where amplitude demodulation is
    // not defined at all - the sidebands fold through zero and the envelope
    // spectrum peaks on an artefact. 7 Hz is a cage-rate fault well inside the
    // new carrier, so the check is back to testing the Hilbert envelope rather
    // than testing whether 23 Hz happens to fit under the resonance.
    Value fault = Value::object();
    fault.set("signal", Value("bearing_fault"));
    fault.set("modulation_hz", Value(7.0));
    fault.set("length", Value(1024));
    const Value hilbert = module.invoke("hilbert_transform", fault);
    check_near(hilbert["detected_modulation_hz"].as_double(), 7.0, 2.0,
               "the envelope spectrum finds the 7 Hz fault modulation");
    check(std::abs(hilbert["orthogonality"].as_double()) < 0.05,
          "the quadrature pair is 90 degrees apart (inner product " +
              json::number_to_string(hilbert["orthogonality"].as_double()) + ")");

    // Equalisation: a smaller epsilon flattens more and amplifies more noise.
    Value tight = Value::object();
    tight.set("epsilon", Value(0.001));
    Value loose = Value::object();
    loose.set("epsilon", Value(0.2));
    const Value tight_result = module.invoke("inverse_and_equalise", tight);
    const Value loose_result = module.invoke("inverse_and_equalise", loose);
    check(tight_result["noise_gain_db"].as_double() > loose_result["noise_gain_db"].as_double(),
          "a smaller regularisation raises the equaliser's noise gain (" +
              json::number_to_string(tight_result["noise_gain_db"].as_double()) + " dB vs " +
              json::number_to_string(loose_result["noise_gain_db"].as_double()) + " dB)");
    check(!tight_result["inverse_is_stable"].as_bool(),
          "a zero at radius 1.2 is reported as having no stable causal inverse");
}

}  // namespace

int main() {
    std::cout << "====================================================\n";
    std::cout << "    RUNNING DSP_SYSTEM STUDY MODULE TEST SUITE      \n";
    std::cout << "====================================================\n";

    test_transforms();
    test_correlation();
    test_responses();
    test_prony();
    test_hilbert();
    test_description();
    test_defaults_run();
    test_rejects_bad_input();
    test_fir_meets_specification();
    test_ops_properties();

    std::cout << "====================================================\n";
    std::cout << "checks run: " << g_checks << ", failed: " << g_failures << "\n";
    if (g_failures != 0) {
        for (const auto& name : g_failed_names) {
            std::cout << "FAILED: " << name << "\n";
        }
        std::cout << "STUDY DSP TESTS FAILED\n";
        return 1;
    }
    std::cout << "          ALL TESTS PASSED                          \n";
    std::cout << "====================================================\n";
    return 0;
}
