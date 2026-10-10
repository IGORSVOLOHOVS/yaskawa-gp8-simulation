#include "study/sensing_models.hpp"

#include "study/gp8_model.hpp"

#include <algorithm>
#include <array>
#include <cmath>
#include <limits>
#include <numbers>
#include <string>
#include <utility>
#include <vector>

namespace yaskawa::study {

namespace {

constexpr double kPi = std::numbers::pi;
constexpr int kMaxSamples = 200000;
constexpr std::size_t kMaxSeriesPoints = 2000;
constexpr int kSeedMin = 0;
constexpr int kSeedMax = 1000000;

// A long record is plotted decimated: the browser cannot draw 200 000 points
// and the student cannot read them. The statistics always use every sample.
[[nodiscard]] std::vector<double> decimate(const std::vector<double>& values) {
    if (values.size() <= kMaxSeriesPoints) {
        return values;
    }
    const std::size_t stride = (values.size() + kMaxSeriesPoints - 1) / kMaxSeriesPoints;
    std::vector<double> out;
    out.reserve(values.size() / stride + 1);
    for (std::size_t i = 0; i < values.size(); i += stride) {
        out.push_back(values[i]);
    }
    return out;
}

[[nodiscard]] std::vector<double> ramp(std::size_t n, double step, double origin = 0.0) {
    std::vector<double> out;
    out.reserve(n);
    for (std::size_t i = 0; i < n; ++i) {
        out.push_back(origin + step * static_cast<double>(i));
    }
    return out;
}

struct SignalChoice {
    std::string name;
    std::string unit;
    double value = 0.0;
    std::string provenance;
};

// The signal the noise is added to is one the GP8 really produces, so the
// numbers on the histogram axis have a physical size.
[[nodiscard]] SignalChoice resolve_signal(const std::string& which) {
    SignalChoice out;
    out.name = which;
    if (which == "joint_position") {
        out.unit = "rad";
        out.value = 0.5 * joint_max(1);  // half of the L-axis upper travel
        out.provenance = "L-axis command at half of its datasheet upper travel";
        return out;
    }
    if (which == "joint_velocity") {
        out.unit = "rad/s";
        out.value = 0.3 * joint_max_velocity(1);
        out.provenance = "L-axis at 30 % of its datasheet maximum speed";
        return out;
    }
    out.unit = "m";
    const Eigen::Matrix<double, 6, 1> home = Eigen::Matrix<double, 6, 1>::Zero();
    out.value = forward_kinematics_dh(home).translation().z();
    out.provenance = "flange height at the q = 0 home pose, from the DH chain";
    return out;
}

// ---------------------------------------------------------------------------
// encoder
// ---------------------------------------------------------------------------

[[nodiscard]] json::Value op_encoder(const json::Value& args) {
    const int joint = optional_int(args, "joint", 1, 0, static_cast<int>(GP8_DOF) - 1);
    const int bits = optional_int(args, "bits", 17, 8, 24);
    const double amplitude = optional_scalar(args, "amplitude", 0.5, 1e-6, 3.0);
    const double frequency = optional_scalar(args, "frequency", 0.5, 0.01, 20.0);
    const double duration = optional_scalar(args, "duration", 2.0, 0.05, 20.0);
    const double sample_rate = optional_scalar(args, "sample_rate", 500.0, 10.0, 20000.0);

    const auto index = static_cast<std::size_t>(joint);
    const double gear_ratio = GP8_LINKS[index].gear_ratio;
    const double increment = sensing::joint_increment(bits, gear_ratio);
    const double counts_per_motor_rev = std::ldexp(1.0, bits);
    const double counts_per_joint_rev = counts_per_motor_rev * gear_ratio;

    const double dt = 1.0 / sample_rate;
    const auto n = static_cast<std::size_t>(duration * sample_rate) + 1;
    if (n < 4 || n > static_cast<std::size_t>(kMaxSamples)) {
        throw StudyError("duration x sample_rate must produce between 4 and " +
                         std::to_string(kMaxSamples) + " samples, asked for " +
                         std::to_string(n));
    }

    std::vector<double> q_true(n, 0.0);
    std::vector<double> q_meas(n, 0.0);
    std::vector<double> v_true(n, 0.0);
    std::vector<double> v_meas(n, 0.0);
    std::vector<double> position_error(n, 0.0);
    std::vector<double> velocity_error(n, 0.0);

    const double omega = 2.0 * kPi * frequency;
    for (std::size_t i = 0; i < n; ++i) {
        const double t = dt * static_cast<double>(i);
        q_true[i] = amplitude * std::sin(omega * t);
        q_meas[i] = sensing::quantise(q_true[i], increment);
        v_true[i] = amplitude * omega * std::cos(omega * t);
        position_error[i] = q_meas[i] - q_true[i];
    }
    // The measured velocity is what a controller really has: a first
    // difference of two quantised positions, so the quantisation error enters
    // twice and is then divided by the sampling period.
    v_meas[0] = (q_meas[1] - q_meas[0]) / dt;
    for (std::size_t i = 1; i < n; ++i) {
        v_meas[i] = (q_meas[i] - q_meas[i - 1]) / dt;
    }
    for (std::size_t i = 0; i < n; ++i) {
        velocity_error[i] = v_meas[i] - v_true[i];
    }

    const sensing::SampleStats position = sensing::summarise(position_error);
    const sensing::SampleStats velocity = sensing::summarise(velocity_error);

    // Quantisation error is uniform on [-delta/2, delta/2]: variance
    // delta^2/12. A first difference of two of them has twice that variance.
    const double theoretical_position_rms = increment / std::sqrt(12.0);
    const double theoretical_velocity_rms = increment / (dt * std::sqrt(6.0));

    const std::vector<double> time = ramp(n, dt);
    const std::vector<double> plotted_time = decimate(time);

    json::Value signals = json::Value::array();
    signals.push_back(json::from_series("q true [rad]", plotted_time, decimate(q_true)));
    signals.push_back(json::from_series("q measured [rad]", plotted_time, decimate(q_meas)));
    signals.push_back(json::from_series("dq/dt true [rad/s]", plotted_time, decimate(v_true)));
    signals.push_back(
        json::from_series("dq/dt from counts [rad/s]", plotted_time, decimate(v_meas)));

    json::Value errors = json::Value::array();
    errors.push_back(
        json::from_series("position error [rad]", plotted_time, decimate(position_error)));
    errors.push_back(
        json::from_series("velocity error [rad/s]", plotted_time, decimate(velocity_error)));

    const std::vector<std::vector<json::Value>> stats_rows = {
        {json::Value("position"), json::Value(position.rms), json::Value(position.maximum),
         json::Value(theoretical_position_rms), json::Value("rad")},
        {json::Value("velocity"), json::Value(velocity.rms), json::Value(velocity.maximum),
         json::Value(theoretical_velocity_rms), json::Value("rad/s")},
    };

    std::size_t outside_band = 0;
    for (const double e : position_error) {
        if (std::abs(e) > 0.5 * increment * 1.000001) {
            ++outside_band;
        }
    }

    json::Value out = json::Value::object();
    out.set("signals", std::move(signals));
    out.set("errors", std::move(errors));
    out.set("error_stats",
            json::from_table({"quantity", "rms_measured", "max_abs", "rms_theoretical", "unit"},
                             stats_rows));
    out.set("axis", json::Value(GP8_AXIS_NAMES[index]));
    out.set("gear_ratio", json::Value(gear_ratio));
    out.set("bits", json::Value(bits));
    out.set("counts_per_motor_rev", json::Value(counts_per_motor_rev));
    out.set("counts_per_joint_rev", json::Value(counts_per_joint_rev));
    out.set("joint_increment", json::Value(increment));
    out.set("joint_increment_deg", json::Value(increment * 180.0 / kPi));
    out.set("position_rms_error", json::Value(position.rms));
    out.set("position_max_error", json::Value(position.maximum));
    out.set("velocity_rms_error", json::Value(velocity.rms));
    out.set("theoretical_position_rms", json::Value(theoretical_position_rms));
    out.set("theoretical_velocity_rms", json::Value(theoretical_velocity_rms));
    out.set("samples_outside_half_increment", json::Value(static_cast<double>(outside_band)));
    out.set("verdict",
            json::Value(std::string("The encoder, not the controller, sets the smallest "
                                    "commandable increment of axis ") +
                        GP8_AXIS_NAMES[index] + ": 2 pi / (2^" + std::to_string(bits) + " x " +
                        json::number_to_string(gear_ratio) + ") = " +
                        json::number_to_string(increment) + " rad."));
    return out;
}

// ---------------------------------------------------------------------------
// noise_models
// ---------------------------------------------------------------------------

[[nodiscard]] json::Value op_noise_models(const json::Value& args) {
    const std::string distribution =
        optional_enum(args, "distribution", "gaussian", {"gaussian", "uniform", "poisson"});
    const std::string signal_name = optional_enum(
        args, "signal", "joint_position", {"joint_position", "joint_velocity", "flange_height"});
    const double sigma = optional_scalar(args, "sigma", 0.01, 1e-9, 10.0);
    const double lambda = optional_scalar(args, "lambda", 20.0, 0.5, 500.0);
    const int samples = optional_int(args, "samples", 4000, 16, kMaxSamples);
    const int bins = optional_int(args, "bins", 32, 8, 128);
    const int seed = optional_int(args, "seed", 7, kSeedMin, kSeedMax);

    const SignalChoice signal = resolve_signal(signal_name);
    const auto n = static_cast<std::size_t>(samples);

    // All three laws are scaled to the SAME standard deviation, so the panel
    // compares their SHAPE rather than their size. For Poisson that scaling is
    // the count quantum sigma / sqrt(lambda).
    const double poisson_quantum = sigma / std::sqrt(lambda);
    const double uniform_half_width = sigma * std::sqrt(3.0);

    sensing::SeededRng rng(seed);
    std::vector<double> values(n, 0.0);
    std::vector<double> noise(n, 0.0);
    std::vector<double> counts(n, 0.0);

    for (std::size_t i = 0; i < n; ++i) {
        double draw = 0.0;
        if (distribution == "gaussian") {
            draw = sigma * rng.gaussian();
        } else if (distribution == "uniform") {
            draw = rng.uniform(-uniform_half_width, uniform_half_width);
        } else {
            const int k = rng.poisson(lambda);
            counts[i] = static_cast<double>(k);
            draw = (static_cast<double>(k) - lambda) * poisson_quantum;
        }
        noise[i] = draw;
        values[i] = signal.value + draw;
    }

    const sensing::SampleStats stats = sensing::summarise(values);
    const sensing::SampleStats noise_stats = sensing::summarise(noise);

    // Histogram over the observed support, which is what a student would do by
    // hand with a ruler on the printout.
    const auto bin_count = static_cast<std::size_t>(bins);
    const double low = stats.minimum;
    double observed_max = values.front();
    for (const double v : values) {
        observed_max = std::max(observed_max, v);
    }
    const double width =
        (observed_max > low) ? (observed_max - low) / static_cast<double>(bin_count) : 1.0;
    std::vector<double> histogram(bin_count, 0.0);
    for (const double v : values) {
        auto bin = static_cast<std::size_t>((v - low) / width);
        if (bin >= bin_count) {
            bin = bin_count - 1;
        }
        histogram[bin] += 1.0;
    }

    std::vector<std::vector<json::Value>> histogram_rows;
    histogram_rows.reserve(bin_count);
    const auto total = static_cast<double>(n);
    for (std::size_t b = 0; b < bin_count; ++b) {
        const double centre = low + width * (static_cast<double>(b) + 0.5);
        const double density = histogram[b] / (total * width);
        // The reference curve is always the fitted normal: course 3286's point
        // is that it stays the right reference even when the data is not.
        const double z = (stats.stddev > 0.0) ? (centre - stats.mean) / stats.stddev : 0.0;
        const double normal_pdf =
            (stats.stddev > 0.0)
                ? std::exp(-0.5 * z * z) / (stats.stddev * std::sqrt(2.0 * kPi))
                : 0.0;
        histogram_rows.push_back({json::Value(centre), json::Value(histogram[b]),
                                  json::Value(density), json::Value(normal_pdf)});
    }

    // Fitted versus true parameters, in the parameterisation of each law.
    std::vector<std::vector<json::Value>> fit_rows;
    fit_rows.push_back({json::Value("mean"), json::Value(signal.value), json::Value(stats.mean),
                        json::Value(std::abs(stats.mean - signal.value))});
    fit_rows.push_back({json::Value("std deviation"), json::Value(sigma),
                        json::Value(stats.stddev), json::Value(std::abs(stats.stddev - sigma))});
    if (distribution == "uniform") {
        const double fitted_half = noise_stats.stddev * std::sqrt(3.0);
        fit_rows.push_back({json::Value("half width a"), json::Value(uniform_half_width),
                            json::Value(fitted_half),
                            json::Value(std::abs(fitted_half - uniform_half_width))});
    } else if (distribution == "poisson") {
        const sensing::SampleStats count_stats = sensing::summarise(counts);
        fit_rows.push_back({json::Value("lambda from the mean"), json::Value(lambda),
                            json::Value(count_stats.mean),
                            json::Value(std::abs(count_stats.mean - lambda))});
        fit_rows.push_back({json::Value("lambda from the variance"), json::Value(lambda),
                            json::Value(count_stats.variance),
                            json::Value(std::abs(count_stats.variance - lambda))});
    } else {
        fit_rows.push_back({json::Value("skewness"), json::Value(0.0),
                            json::Value(noise_stats.skewness),
                            json::Value(std::abs(noise_stats.skewness))});
    }

    std::size_t outside_three_sigma = 0;
    for (const double v : values) {
        if (std::abs(v - stats.mean) > 3.0 * stats.stddev) {
            ++outside_three_sigma;
        }
    }

    const std::vector<double> plotted = decimate(values);
    const std::vector<double> plotted_index =
        ramp(plotted.size(), static_cast<double>(n) / static_cast<double>(plotted.size()));

    json::Value out = json::Value::object();
    out.set("samples", json::from_series("measured " + signal.name + " [" + signal.unit + "]",
                                         plotted_index, plotted));
    out.set("histogram", json::from_table(
                             {"bin_centre", "count", "density", "fitted_normal_pdf"},
                             histogram_rows));
    out.set("fit", json::from_table({"parameter", "true", "fitted", "abs_error"}, fit_rows));
    out.set("distribution", json::Value(distribution));
    out.set("signal", json::Value(signal.name));
    out.set("unit", json::Value(signal.unit));
    out.set("signal_provenance", json::Value(signal.provenance));
    out.set("true_value", json::Value(signal.value));
    out.set("true_sigma", json::Value(sigma));
    out.set("fitted_mean", json::Value(stats.mean));
    out.set("fitted_sigma", json::Value(stats.stddev));
    out.set("mean_abs_error", json::Value(std::abs(stats.mean - signal.value)));
    out.set("sigma_abs_error", json::Value(std::abs(stats.stddev - sigma)));
    out.set("noise_skewness", json::Value(noise_stats.skewness));
    out.set("sample_count", json::Value(static_cast<double>(n)));
    out.set("seed", json::Value(seed));
    out.set("three_sigma_fraction_outside",
            json::Value(static_cast<double>(outside_three_sigma) / total));
    return out;
}

// ---------------------------------------------------------------------------
// conditioning_chain
// ---------------------------------------------------------------------------

[[nodiscard]] json::Value op_conditioning_chain(const json::Value& args) {
    const double frequency = optional_scalar(args, "signal_frequency", 2.0, 0.05, 200.0);
    const double amplitude = optional_scalar(args, "amplitude", 0.2, 1e-6, 5.0);
    const double offset = optional_scalar(args, "offset", 0.35, -5.0, 5.0);
    const double gain = optional_scalar(args, "gain", 4.0, 0.01, 1000.0);
    const double interference_frequency =
        optional_scalar(args, "interference_frequency", 150.0, 1.0, 5000.0);
    const double interference_amplitude =
        optional_scalar(args, "interference_amplitude", 0.08, 0.0, 5.0);
    const double noise_sigma = optional_scalar(args, "noise_sigma", 0.01, 0.0, 5.0);
    const double antialias_cutoff = optional_scalar(args, "antialias_cutoff", 40.0, 0.5, 5000.0);
    const double sample_rate = optional_scalar(args, "sample_rate", 200.0, 5.0, 10000.0);
    const double digital_cutoff = optional_scalar(args, "digital_cutoff", 10.0, 0.1, 2000.0);
    const double duration = optional_scalar(args, "duration", 1.0, 0.05, 10.0);
    const int seed = optional_int(args, "seed", 11, kSeedMin, kSeedMax);

    // The analogue stages are integrated on an oversampled grid, because an
    // anti-alias filter simulated at the sampling rate cannot alias at all.
    const std::size_t oversample = 16;
    const double fine_rate = sample_rate * static_cast<double>(oversample);
    const double dt = 1.0 / fine_rate;
    const auto n = static_cast<std::size_t>(duration * fine_rate) + 1;
    if (n > static_cast<std::size_t>(kMaxSamples)) {
        throw StudyError("duration x sample_rate is too long: " + std::to_string(n) +
                         " internal samples exceeds the " + std::to_string(kMaxSamples) +
                         " limit");
    }

    sensing::SeededRng rng(seed);
    std::vector<double> ideal(n, 0.0);
    std::vector<double> raw(n, 0.0);
    std::vector<double> unbiased(n, 0.0);
    std::vector<double> amplified(n, 0.0);
    std::vector<double> analogue_filtered(n, 0.0);
    std::vector<double> sampled(n, 0.0);
    std::vector<double> digital_filtered(n, 0.0);
    std::vector<double> scaled_ideal(n, 0.0);

    const double omega = 2.0 * kPi * frequency;
    const double omega_interference = 2.0 * kPi * interference_frequency;
    for (std::size_t i = 0; i < n; ++i) {
        const double t = dt * static_cast<double>(i);
        ideal[i] = amplitude * std::sin(omega * t);
        scaled_ideal[i] = gain * ideal[i];
        raw[i] = ideal[i] + offset +
                 interference_amplitude * std::sin(omega_interference * t) +
                 noise_sigma * rng.gaussian();
        unbiased[i] = raw[i] - offset;
        amplified[i] = gain * unbiased[i];
    }

    // Two cascaded one-pole RC sections: alpha = dt / (RC + dt).
    const double rc = 1.0 / (2.0 * kPi * antialias_cutoff);
    const double alpha = dt / (rc + dt);
    double stage_a = amplified[0];
    double stage_b = amplified[0];
    for (std::size_t i = 0; i < n; ++i) {
        stage_a += alpha * (amplified[i] - stage_a);
        stage_b += alpha * (stage_a - stage_b);
        analogue_filtered[i] = stage_b;
    }

    // Sample and hold at the real sampling rate, kept on the fine time base so
    // the staircase is visible against the analogue trace.
    const double digital_dt = 1.0 / sample_rate;
    const double digital_rc = 1.0 / (2.0 * kPi * digital_cutoff);
    const double digital_alpha = digital_dt / (digital_rc + digital_dt);
    double held = analogue_filtered[0];
    double digital_state = analogue_filtered[0];
    std::size_t sample_taken = 0;
    for (std::size_t i = 0; i < n; ++i) {
        if (i % oversample == 0) {
            held = analogue_filtered[i];
            digital_state += digital_alpha * (held - digital_state);
            ++sample_taken;
        }
        sampled[i] = held;
        digital_filtered[i] = digital_state;
    }

    // What each stage leaves behind, measured against the ideal signal scaled
    // by the gain that the stage has already applied.
    const auto deviation_rms = [&ideal, n](const std::vector<double>& stage, double scale) {
        double sum = 0.0;
        for (std::size_t i = 0; i < n; ++i) {
            const double e = stage[i] - scale * ideal[i];
            sum += e * e;
        }
        return std::sqrt(sum / static_cast<double>(n));
    };

    const double nyquist = 0.5 * sample_rate;
    const double folded = std::abs(interference_frequency -
                                   sample_rate * std::round(interference_frequency / sample_rate));
    const double ratio = interference_frequency / antialias_cutoff;
    const double analogue_attenuation = 1.0 / (1.0 + ratio * ratio);

    const std::vector<std::vector<json::Value>> stage_rows = {
        {json::Value("1 raw transducer"), json::Value(deviation_rms(raw, 1.0)),
         json::Value("offset, interference and broadband noise all present")},
        {json::Value("2 offset removal"), json::Value(deviation_rms(unbiased, 1.0)),
         json::Value("removes the DC bias so the amplifier keeps its headroom")},
        {json::Value("3 gain"), json::Value(deviation_rms(amplified, gain)),
         json::Value("matches the sensor span to the converter input range")},
        {json::Value("4 anti-alias low pass"), json::Value(deviation_rms(analogue_filtered, gain)),
         json::Value("must run BEFORE the sampler: the only stage that can stop aliasing")},
        {json::Value("5 sample and hold"), json::Value(deviation_rms(sampled, gain)),
         json::Value("discretises time; anything left above Nyquist folds down and is lost")},
        {json::Value("6 digital low pass"), json::Value(deviation_rms(digital_filtered, gain)),
         json::Value("cleans the residual noise and charges the loop group delay for it")},
    };

    const std::vector<double> time = ramp(n, dt);
    const std::vector<double> plotted_time = decimate(time);
    json::Value stages = json::Value::array();
    stages.push_back(json::from_series("1 raw", plotted_time, decimate(raw)));
    stages.push_back(json::from_series("2 offset removed", plotted_time, decimate(unbiased)));
    stages.push_back(json::from_series("3 amplified", plotted_time, decimate(amplified)));
    stages.push_back(
        json::from_series("4 anti-alias filtered", plotted_time, decimate(analogue_filtered)));
    stages.push_back(json::from_series("5 sampled and held", plotted_time, decimate(sampled)));
    stages.push_back(
        json::from_series("6 digitally filtered", plotted_time, decimate(digital_filtered)));
    stages.push_back(json::from_series("ideal x gain", plotted_time, decimate(scaled_ideal)));

    json::Value out = json::Value::object();
    out.set("stages", std::move(stages));
    out.set("stage_table",
            json::from_table({"stage", "rms_deviation_from_ideal", "what_it_does"}, stage_rows));
    out.set("nyquist_frequency", json::Value(nyquist));
    out.set("aliased_interference_frequency", json::Value(folded));
    out.set("interference_will_alias", json::Value(interference_frequency > nyquist));
    out.set("antialias_attenuation_at_interference", json::Value(analogue_attenuation));
    out.set("antialias_alpha", json::Value(alpha));
    out.set("digital_alpha", json::Value(digital_alpha));
    out.set("samples_taken", json::Value(static_cast<double>(sample_taken)));
    out.set("digital_group_delay",
            json::Value(digital_dt * (1.0 - digital_alpha) / digital_alpha));
    out.set("final_rms_deviation", json::Value(deviation_rms(digital_filtered, gain)));
    out.set("raw_rms_deviation", json::Value(deviation_rms(raw, 1.0)));
    out.set("verdict",
            json::Value(interference_frequency > nyquist
                            ? "The interference sits above Nyquist: without the anti-alias stage "
                              "it reappears at " +
                                  json::number_to_string(folded) +
                                  " Hz inside the signal band, where no digital filter can "
                                  "separate it from the measurement."
                            : "The interference is below Nyquist, so the sampler represents it "
                              "honestly and the digital stage can still remove it."));
    return out;
}

// ---------------------------------------------------------------------------
// bayes_fusion
// ---------------------------------------------------------------------------

[[nodiscard]] json::Value op_bayes_fusion(const json::Value& args) {
    const double truth = optional_scalar(args, "truth", 0.4, -kPi, kPi);
    const int steps = optional_int(args, "steps", 20, 1, 500);
    const double encoder_sigma = optional_scalar(args, "encoder_sigma", 0.02, 1e-6, 1.0);
    const double vision_sigma = optional_scalar(args, "vision_sigma", 0.06, 1e-6, 1.0);
    const double prior_mean = optional_scalar(args, "prior_mean", 0.0, -kPi, kPi);
    const double prior_sigma = optional_scalar(args, "prior_sigma", 0.5, 1e-6, 10.0);
    const int seed = optional_int(args, "seed", 3, kSeedMin, kSeedMax);

    sensing::SeededRng rng(seed);
    sensing::GaussianBelief belief{prior_mean, prior_sigma * prior_sigma};

    const auto n = static_cast<std::size_t>(steps);
    std::vector<std::vector<json::Value>> rows;
    rows.reserve(2 * n);
    std::vector<double> variance_history;
    variance_history.reserve(2 * n + 1);
    std::vector<double> mean_history;
    mean_history.reserve(2 * n + 1);
    variance_history.push_back(belief.variance);
    mean_history.push_back(belief.mean);

    double encoder_sum = 0.0;
    double vision_sum = 0.0;

    for (std::size_t k = 0; k < n; ++k) {
        const double z_encoder = truth + encoder_sigma * rng.gaussian();
        const double z_vision = truth + vision_sigma * rng.gaussian();
        encoder_sum += z_encoder;
        vision_sum += z_vision;

        const std::array<std::pair<const char*, std::pair<double, double>>, 2> readings = {{
            {"encoder", {z_encoder, encoder_sigma}},
            {"vision", {z_vision, vision_sigma}},
        }};
        for (const auto& reading : readings) {
            const sensing::GaussianBelief prior = belief;
            const double measurement = reading.second.first;
            const double sigma = reading.second.second;
            double gain = 0.0;
            belief = sensing::bayes_update(prior, measurement, sigma * sigma, gain);
            const double residual = measurement - prior.mean;
            const double likelihood = std::exp(-0.5 * residual * residual / (sigma * sigma)) /
                                      (sigma * std::sqrt(2.0 * kPi));
            rows.push_back({json::Value(static_cast<double>(k + 1)), json::Value(reading.first),
                            json::Value(prior.mean), json::Value(prior.variance),
                            json::Value(measurement), json::Value(sigma),
                            json::Value(likelihood), json::Value(gain), json::Value(belief.mean),
                            json::Value(belief.variance)});
            variance_history.push_back(belief.variance);
            mean_history.push_back(belief.mean);
        }
    }

    const double encoder_only = encoder_sum / static_cast<double>(n);
    const double vision_only = vision_sum / static_cast<double>(n);
    const double error_fused = std::abs(belief.mean - truth);
    const double error_encoder = std::abs(encoder_only - truth);
    const double error_vision = std::abs(vision_only - truth);

    json::Value curves = json::Value::array();
    const std::vector<double> update_index = ramp(variance_history.size(), 1.0);
    curves.push_back(json::from_series("posterior variance", update_index, variance_history));
    curves.push_back(json::from_series("posterior mean", update_index, mean_history));
    curves.push_back(json::from_series("ground truth", update_index,
                                       std::vector<double>(variance_history.size(), truth)));

    json::Value out = json::Value::object();
    out.set("updates",
            json::from_table({"step", "sensor", "prior_mean", "prior_variance", "measurement",
                              "likelihood_sigma", "likelihood_at_prior_mean", "kalman_gain",
                              "posterior_mean", "posterior_variance"},
                             rows));
    out.set("curves", std::move(curves));
    out.set("truth", json::Value(truth));
    out.set("final_estimate", json::Value(belief.mean));
    out.set("final_variance", json::Value(belief.variance));
    out.set("final_sigma", json::Value(std::sqrt(belief.variance)));
    out.set("encoder_variance", json::Value(encoder_sigma * encoder_sigma));
    out.set("vision_variance", json::Value(vision_sigma * vision_sigma));
    out.set("encoder_only_estimate", json::Value(encoder_only));
    out.set("vision_only_estimate", json::Value(vision_only));
    out.set("error_fused", json::Value(error_fused));
    out.set("error_encoder_only", json::Value(error_encoder));
    out.set("error_vision_only", json::Value(error_vision));
    out.set("beats_both_sensors",
            json::Value(error_fused < error_encoder && error_fused < error_vision));
    out.set("variance_below_best_sensor",
            json::Value(belief.variance <
                        std::min(encoder_sigma * encoder_sigma, vision_sigma * vision_sigma)));
    out.set("steps", json::Value(steps));
    out.set("seed", json::Value(seed));
    out.set("explain_step",
            json::Value("Bayes' rule as the course states it: "
                        "P(x | z) = P(z | x) P(x) / P(z). The prior P(x) is the Gaussian in the "
                        "prior_mean and prior_variance columns, the likelihood P(z | x) is the "
                        "sensor's Gaussian about its reading, P(z) is the normalising evidence, "
                        "and the posterior columns are that product renormalised. For Gaussians "
                        "the product is Gaussian again, which is why two numbers and a gain are "
                        "a complete row."));
    return out;
}

// ---------------------------------------------------------------------------
// fault_detection
// ---------------------------------------------------------------------------

[[nodiscard]] json::Value op_fault_detection(const json::Value& args) {
    const std::string fault =
        optional_enum(args, "fault", "drift", {"none", "stuck", "drift", "dropout"});
    const double onset = optional_scalar(args, "onset", 1.0, 0.0, 20.0);
    const double drift_rate = optional_scalar(args, "drift_rate", 0.05, -5.0, 5.0);
    const double dropout_probability = optional_scalar(args, "dropout_probability", 0.6, 0.0, 1.0);
    const double threshold_sigma = optional_scalar(args, "threshold_sigma", 4.0, 0.5, 12.0);
    const int consecutive = optional_int(args, "consecutive", 3, 1, 100);
    const double noise_sigma = optional_scalar(args, "noise_sigma", 0.004, 1e-9, 1.0);
    const double duration = optional_scalar(args, "duration", 3.0, 0.2, 20.0);
    const double sample_rate = optional_scalar(args, "sample_rate", 200.0, 10.0, 5000.0);
    const int seed = optional_int(args, "seed", 23, kSeedMin, kSeedMax);

    const double dt = 1.0 / sample_rate;
    const auto n = static_cast<std::size_t>(duration * sample_rate) + 1;
    if (n > static_cast<std::size_t>(kMaxSamples)) {
        throw StudyError("duration x sample_rate exceeds " + std::to_string(kMaxSamples) +
                         " samples");
    }
    if (onset >= duration) {
        throw StudyError("parameter 'onset' = " + json::number_to_string(onset) +
                         " must fall inside the run, which ends at " +
                         json::number_to_string(duration) + " s");
    }

    sensing::SeededRng rng(seed);
    const double amplitude = 0.5;
    const double omega = 2.0 * kPi * 0.4;

    std::vector<double> command(n, 0.0);
    std::vector<double> measured(n, 0.0);
    std::vector<double> residual(n, 0.0);
    std::vector<double> flag(n, 0.0);

    double frozen = 0.0;
    bool frozen_set = false;
    for (std::size_t i = 0; i < n; ++i) {
        const double t = dt * static_cast<double>(i);
        command[i] = amplitude * std::sin(omega * t);
        double value = command[i] + noise_sigma * rng.gaussian();
        const bool faulty = (t >= onset) && (fault != "none");
        if (faulty) {
            if (fault == "stuck") {
                if (!frozen_set) {
                    frozen = value;
                    frozen_set = true;
                }
                value = frozen;
            } else if (fault == "drift") {
                value += drift_rate * (t - onset);
            } else if (fault == "dropout") {
                if (rng.uniform01() < dropout_probability) {
                    value = 0.0;
                }
            }
        }
        measured[i] = value;
        residual[i] = measured[i] - command[i];
    }

    // The residual test of the course: a fixed multiple of the known
    // measurement sigma, confirmed over `consecutive` samples so that a single
    // tail draw is not reported as a fault.
    const double limit = threshold_sigma * noise_sigma;
    const auto onset_index = std::min<std::size_t>(static_cast<std::size_t>(onset * sample_rate),
                                                   n - 1);
    int run_length = 0;
    bool detected = false;
    std::size_t detection_index = 0;
    std::size_t pre_fault_alarms = 0;
    std::size_t post_fault_alarms = 0;

    for (std::size_t i = 0; i < n; ++i) {
        const bool over = std::abs(residual[i]) > limit;
        run_length = over ? (run_length + 1) : 0;
        const bool alarm = run_length >= consecutive;
        flag[i] = alarm ? 1.0 : 0.0;
        if (alarm) {
            if (i < onset_index) {
                ++pre_fault_alarms;
            } else {
                ++post_fault_alarms;
                if (!detected) {
                    detected = true;
                    detection_index = i;
                }
            }
        }
    }

    const double false_alarm_rate =
        (onset_index > 0)
            ? static_cast<double>(pre_fault_alarms) / static_cast<double>(onset_index)
            : 0.0;
    const double detection_delay = detected
                                       ? (dt * static_cast<double>(detection_index) - onset)
                                       : std::numeric_limits<double>::quiet_NaN();

    const std::vector<double> time = ramp(n, dt);
    const std::vector<double> plotted_time = decimate(time);
    json::Value traces = json::Value::array();
    traces.push_back(json::from_series("command [rad]", plotted_time, decimate(command)));
    traces.push_back(json::from_series("measured [rad]", plotted_time, decimate(measured)));
    traces.push_back(json::from_series("residual [rad]", plotted_time, decimate(residual)));
    traces.push_back(json::from_series("+threshold", plotted_time,
                                       std::vector<double>(plotted_time.size(), limit)));
    traces.push_back(json::from_series("-threshold", plotted_time,
                                       std::vector<double>(plotted_time.size(), -limit)));
    traces.push_back(json::from_series("alarm", plotted_time, decimate(flag)));

    const std::vector<std::vector<json::Value>> summary_rows = {
        {json::Value("fault injected"), json::Value(fault)},
        {json::Value("onset [s]"), json::Value(onset)},
        {json::Value("residual limit [rad]"), json::Value(limit)},
        {json::Value("confirmation samples"), json::Value(static_cast<double>(consecutive))},
        {json::Value("detected"), json::Value(detected)},
        {json::Value("detection delay [s]"), json::Value(detection_delay)},
        {json::Value("false alarms before onset"),
         json::Value(static_cast<double>(pre_fault_alarms))},
        {json::Value("alarms after onset"), json::Value(static_cast<double>(post_fault_alarms))},
    };

    json::Value out = json::Value::object();
    out.set("traces", std::move(traces));
    out.set("summary", json::from_table({"quantity", "value"}, summary_rows));
    out.set("fault", json::Value(fault));
    out.set("detected", json::Value(detected));
    out.set("detection_delay", json::Value(detection_delay));
    out.set("detection_delay_samples",
            json::Value(detected ? static_cast<double>(detection_index - onset_index)
                                 : std::numeric_limits<double>::quiet_NaN()));
    out.set("false_alarm_rate", json::Value(false_alarm_rate));
    out.set("residual_limit", json::Value(limit));
    out.set("pre_fault_samples", json::Value(static_cast<double>(onset_index)));
    out.set("alarms_after_onset", json::Value(static_cast<double>(post_fault_alarms)));
    out.set("seed", json::Value(seed));
    out.set("verdict",
            json::Value(detected ? "Detected " + json::number_to_string(detection_delay) +
                                       " s after onset at a false-alarm rate of " +
                                       json::number_to_string(false_alarm_rate) +
                                       " per pre-fault sample. Lower the threshold for a faster "
                                       "detection and watch that rate climb."
                                 : "Not detected: this residual test cannot see this fault at " +
                                       json::number_to_string(threshold_sigma) + " sigma."));
    return out;
}

}  // namespace

// ---------------------------------------------------------------------------
// sensing:: free functions
// ---------------------------------------------------------------------------

namespace sensing {

SampleStats summarise(const std::vector<double>& samples) noexcept {
    SampleStats out;
    out.count = samples.size();
    if (samples.empty()) {
        return out;
    }
    double sum = 0.0;
    double square_sum = 0.0;
    out.minimum = samples.front();
    out.maximum = std::abs(samples.front());
    for (const double v : samples) {
        sum += v;
        square_sum += v * v;
        out.minimum = std::min(out.minimum, v);
        out.maximum = std::max(out.maximum, std::abs(v));
    }
    const auto n = static_cast<double>(samples.size());
    out.mean = sum / n;
    out.rms = std::sqrt(square_sum / n);

    double centred_square = 0.0;
    double centred_cube = 0.0;
    for (const double v : samples) {
        const double d = v - out.mean;
        centred_square += d * d;
        centred_cube += d * d * d;
    }
    out.variance = (samples.size() > 1) ? centred_square / (n - 1.0) : 0.0;
    out.stddev = std::sqrt(out.variance);
    const double population_sigma = std::sqrt(centred_square / n);
    out.skewness =
        (population_sigma > 0.0)
            ? (centred_cube / n) / (population_sigma * population_sigma * population_sigma)
            : 0.0;
    return out;
}

double quantise(double value, double step) noexcept {
    if (!(step > 0.0)) {
        return value;
    }
    return std::round(value / step) * step;
}

double joint_increment(int bits, double gear_ratio) noexcept {
    const double counts = std::ldexp(1.0, bits) * gear_ratio;
    return (counts > 0.0) ? (2.0 * kPi / counts) : 0.0;
}

GaussianBelief bayes_update(const GaussianBelief& prior, double measurement,
                            double measurement_variance, double& gain) noexcept {
    const double denominator = prior.variance + measurement_variance;
    gain = (denominator > 0.0) ? (prior.variance / denominator) : 0.0;
    GaussianBelief posterior;
    posterior.mean = prior.mean + gain * (measurement - prior.mean);
    posterior.variance = (1.0 - gain) * prior.variance;
    return posterior;
}

}  // namespace sensing

// ---------------------------------------------------------------------------
// Self-description
// ---------------------------------------------------------------------------

ModuleDescription SensingModelsModule::describe() const {
    ModuleDescription d;
    d.name = "sensing_models";
    d.title = "Sensing Devices, Signal Conditioning and Sensor Fusion";
    d.course = CourseRef{3884, "M-408-01", "Robotic Systems Design"};
    d.topics = {
        "Session 27 · Information (sensing) devices of robotic systems",
        "Session 28 · Practical Work 6 — Sensor feedback, signal conditioning and filtering",
        "Course 3286 · Random variables, the normal distribution and Bayes",
    };
    d.source = "cpp_solver/include/study/sensing_models.hpp";
    d.summary =
        "Models the GP8's feedback channel end to end: encoder quantisation from bit count and "
        "gear ratio, the three noise laws of the probability course riding on a real robot "
        "signal, the offset-gain-antialias-sample-filter conditioning chain, recursive Bayesian "
        "fusion of a disagreeing encoder and camera, and a residual fault test with its "
        "detection delay and false-alarm rate.";

    {
        OpSpec op;
        op.name = "encoder";
        op.title = "Joint encoder resolution, quantisation and the error it causes";
        op.formula =
            "\\Delta q = \\frac{2\\pi}{2^{b} n}, \\qquad q_m = \\Delta q \\operatorname{round}"
            "\\!\\left(\\frac{q}{\\Delta q}\\right), \\qquad \\sigma_q = \\frac{\\Delta q}"
            "{\\sqrt{12}}, \\qquad \\sigma_{\\dot q} = \\frac{\\Delta q}{T\\sqrt{6}}";
        op.explain =
            "An absolute encoder on the motor shaft resolves 2^b steps per motor revolution, and "
            "the gearbox spreads those over one joint revolution, so the smallest joint increment "
            "the machine can even report is 2 pi / (2^b n) - the encoder sets it, not the "
            "controller. Rounding to that grid leaves an error that is uniform on plus or minus "
            "half an increment, whose standard deviation is the increment over root twelve. "
            "Velocity is worse: a first difference subtracts two independent quantisation errors "
            "and then divides by the sampling period, which is exactly why the velocity trace is "
            "visibly rougher than the position trace at the same resolution.";
        op.params = {
            ParamSpec::integer("joint", "Axis (0 = S ... 5 = T)", "", 0,
                               static_cast<int>(GP8_DOF) - 1, 1),
            ParamSpec::integer("bits", "Encoder resolution", "bit", 8, 24, 17),
            ParamSpec::scalar("amplitude", "Motion amplitude", "rad", 1e-6, 3.0, 0.5),
            ParamSpec::scalar("frequency", "Motion frequency", "Hz", 0.01, 20.0, 0.5),
            ParamSpec::scalar("duration", "Record length", "s", 0.05, 20.0, 2.0),
            ParamSpec::scalar("sample_rate", "Controller sampling rate", "Hz", 10.0, 20000.0,
                              500.0),
        };
        op.outputs = {
            OutputSpec::make("signals", "series_set", "True and measured position and velocity"),
            OutputSpec::make("errors", "series_set", "Measurement error of both quantities"),
            OutputSpec::make("error_stats", "table", "Measured against theoretical error"),
            OutputSpec::make("joint_increment", "scalar", "Smallest resolvable increment", "rad"),
            OutputSpec::make("counts_per_joint_rev", "scalar", "Counts per joint revolution"),
            OutputSpec::make("gear_ratio", "scalar", "Gear ratio of this axis (gp8_model.hpp)"),
            OutputSpec::make("position_rms_error", "scalar", "RMS position error", "rad"),
            OutputSpec::make("velocity_rms_error", "scalar", "RMS velocity error", "rad/s"),
            OutputSpec::make("verdict", "text", "What this resolution means for this axis"),
        };
        d.ops.push_back(std::move(op));
    }
    {
        OpSpec op;
        op.name = "noise_models";
        op.title = "Gaussian, uniform and Poisson noise on a robot signal";
        op.formula =
            "p_{\\mathcal{N}}(x) = \\frac{1}{\\sigma\\sqrt{2\\pi}} e^{-(x-\\mu)^2/2\\sigma^2}, "
            "\\qquad p_{\\mathcal{U}}(x) = \\frac{1}{2a},\\; a = \\sigma\\sqrt{3}, \\qquad "
            "P_{\\lambda}(k) = \\frac{\\lambda^{k} e^{-\\lambda}}{k!}";
        op.explain =
            "All three laws are scaled to the same standard deviation, so what changes on the "
            "histogram is the shape and not the size: the normal is symmetric with light tails, "
            "the uniform has hard edges and no tails at all, and the Poisson count noise stays "
            "visibly skewed until lambda grows. The fitted parameters come from the sample "
            "moments exactly as course 3286 defines them, and they are printed beside the true "
            "ones so that the estimator's own error is on screen. Raise the sample count and the "
            "gap closes like one over the square root of n - that convergence is the point of the "
            "panel, and the plus or minus three sigma fraction should settle near 0.27 %.";
        op.params = {
            ParamSpec::enumeration("distribution", "Noise law", {"gaussian", "uniform", "poisson"},
                                   "gaussian"),
            ParamSpec::enumeration("signal", "Robot signal the noise rides on",
                                   {"joint_position", "joint_velocity", "flange_height"},
                                   "joint_position"),
            ParamSpec::scalar("sigma", "Noise standard deviation", "signal unit", 1e-9, 10.0,
                              0.01),
            ParamSpec::scalar("lambda", "Poisson count rate", "counts", 0.5, 500.0, 20.0),
            ParamSpec::integer("samples", "Sample count", "", 16, kMaxSamples, 4000),
            ParamSpec::integer("bins", "Histogram bins", "", 8, 128, 32),
            ParamSpec::integer("seed", "Random seed", "", kSeedMin, kSeedMax, 7),
        };
        op.outputs = {
            OutputSpec::make("samples", "series", "The measured record"),
            OutputSpec::make("histogram", "table", "Histogram with the fitted normal density"),
            OutputSpec::make("fit", "table", "Fitted against true distribution parameters"),
            OutputSpec::make("fitted_sigma", "scalar", "Standard deviation from the samples"),
            OutputSpec::make("sigma_abs_error", "scalar", "|fitted sigma - true sigma|"),
            OutputSpec::make("noise_skewness", "scalar", "Sample skewness of the noise"),
            OutputSpec::make("three_sigma_fraction_outside", "scalar",
                             "Fraction outside the +/- 3 sigma band"),
            OutputSpec::make("signal_provenance", "text", "Where the true value came from"),
        };
        d.ops.push_back(std::move(op));
    }
    {
        OpSpec op;
        op.name = "conditioning_chain";
        op.title = "Offset, gain, anti-alias filter, sampling, digital filter";
        op.formula =
            "v_1 = v_{raw} - V_{os}, \\quad v_2 = G v_1, \\quad H_{aa}(s) = \\left(\\frac{1}"
            "{1 + s/\\omega_c}\\right)^{2}, \\quad y_k = y_{k-1} + \\alpha\\,(v_k - y_{k-1}), "
            "\\; \\alpha = \\frac{T}{RC + T}, \\quad f_{alias} = |f_i - m f_s|";
        op.explain =
            "This is the real path from a transducer to a number in the controller, one stage at "
            "a time: remove the DC offset so the amplifier keeps its headroom, apply the gain "
            "that matches the sensor span to the converter range, low-pass before sampling, "
            "sample and hold, then filter digitally. The order is not decorative - the anti-alias "
            "filter is the only stage that can stop an interference above Nyquist from folding "
            "into the signal band, and once it has folded no digital filter can tell it from the "
            "measurement. The last stage cleans what is left and charges you group delay for it, "
            "which is the delay that later destabilises a closed loop.";
        op.params = {
            ParamSpec::scalar("signal_frequency", "Signal frequency", "Hz", 0.05, 200.0, 2.0),
            ParamSpec::scalar("amplitude", "Signal amplitude", "V", 1e-6, 5.0, 0.2),
            ParamSpec::scalar("offset", "Transducer DC offset", "V", -5.0, 5.0, 0.35),
            ParamSpec::scalar("gain", "Amplifier gain", "", 0.01, 1000.0, 4.0),
            ParamSpec::scalar("interference_frequency", "Interference frequency", "Hz", 1.0,
                              5000.0, 150.0),
            ParamSpec::scalar("interference_amplitude", "Interference amplitude", "V", 0.0, 5.0,
                              0.08),
            ParamSpec::scalar("noise_sigma", "Broadband noise sigma", "V", 0.0, 5.0, 0.01),
            ParamSpec::scalar("antialias_cutoff", "Anti-alias cutoff", "Hz", 0.5, 5000.0, 40.0),
            ParamSpec::scalar("sample_rate", "Sampling rate", "Hz", 5.0, 10000.0, 200.0),
            ParamSpec::scalar("digital_cutoff", "Digital filter cutoff", "Hz", 0.1, 2000.0, 10.0),
            ParamSpec::scalar("duration", "Record length", "s", 0.05, 10.0, 1.0),
            ParamSpec::integer("seed", "Random seed", "", kSeedMin, kSeedMax, 11),
        };
        op.outputs = {
            OutputSpec::make("stages", "series_set", "The signal after every stage"),
            OutputSpec::make("stage_table", "table", "What each stage removes"),
            OutputSpec::make("nyquist_frequency", "scalar", "Half the sampling rate", "Hz"),
            OutputSpec::make("aliased_interference_frequency", "scalar",
                             "Where the interference lands after sampling", "Hz"),
            OutputSpec::make("antialias_attenuation_at_interference", "scalar",
                             "|H_aa| at the interference frequency"),
            OutputSpec::make("digital_group_delay", "scalar", "Group delay the last stage adds",
                             "s"),
            OutputSpec::make("final_rms_deviation", "scalar", "RMS error left at the output"),
            OutputSpec::make("verdict", "text", "Whether the chain can still recover the signal"),
        };
        d.ops.push_back(std::move(op));
    }
    {
        OpSpec op;
        op.name = "bayes_fusion";
        op.title = "Recursive Bayesian fusion of a disagreeing encoder and camera";
        op.formula =
            "P(x \\mid z) = \\frac{P(z \\mid x)\\,P(x)}{P(z)}, \\qquad K = \\frac{\\sigma^2_{-}}"
            "{\\sigma^2_{-} + \\sigma^2_{z}}, \\qquad \\hat{x}_{+} = \\hat{x}_{-} + K\\,(z - "
            "\\hat{x}_{-}), \\qquad \\sigma^2_{+} = (1 - K)\\,\\sigma^2_{-}";
        op.explain =
            "Two sensors disagree and neither is right: the encoder is precise but sees only the "
            "motor side of the gearbox, the camera sees the tool itself but with far more "
            "variance. Bayes' rule says the posterior is the likelihood times the prior over the "
            "evidence, and for a Gaussian belief and a Gaussian sensor that product is Gaussian "
            "again, so the whole recursion is two numbers and a gain. The gain is the fraction of "
            "the disagreement you should believe; the variance shrinks strictly with every "
            "reading, which is why the fused posterior ends tighter than either sensor on its "
            "own, and why the fused estimate beats both of them in RMS over repeated seeds.";
        op.params = {
            ParamSpec::scalar("truth", "True joint angle", "rad", -kPi, kPi, 0.4),
            ParamSpec::integer("steps", "Fusion steps (two readings each)", "", 1, 500, 20),
            ParamSpec::scalar("encoder_sigma", "Encoder noise sigma", "rad", 1e-6, 1.0, 0.02),
            ParamSpec::scalar("vision_sigma", "Vision noise sigma", "rad", 1e-6, 1.0, 0.06),
            ParamSpec::scalar("prior_mean", "Prior mean", "rad", -kPi, kPi, 0.0),
            ParamSpec::scalar("prior_sigma", "Prior sigma", "rad", 1e-6, 10.0, 0.5),
            ParamSpec::integer("seed", "Random seed", "", kSeedMin, kSeedMax, 3),
        };
        op.outputs = {
            OutputSpec::make("updates", "table", "Prior, likelihood and posterior at every step"),
            OutputSpec::make("curves", "series_set", "Posterior variance, mean and ground truth"),
            OutputSpec::make("final_estimate", "scalar", "Fused estimate", "rad"),
            OutputSpec::make("final_variance", "scalar", "Posterior variance", "rad^2"),
            OutputSpec::make("error_fused", "scalar", "|fused - truth|", "rad"),
            OutputSpec::make("error_encoder_only", "scalar", "|encoder mean - truth|", "rad"),
            OutputSpec::make("error_vision_only", "scalar", "|vision mean - truth|", "rad"),
            OutputSpec::make("variance_below_best_sensor", "bool",
                             "Posterior variance beats both sensors"),
            OutputSpec::make("explain_step", "text", "Bayes' rule written out"),
        };
        d.ops.push_back(std::move(op));
    }
    {
        OpSpec op;
        op.name = "fault_detection";
        op.title = "Injected sensor fault and the residual test that finds it";
        op.formula =
            "r_k = z_k - \\hat{z}_k, \\qquad \\text{alarm} \\iff |r_k| > \\kappa\\sigma "
            "\\text{ for } m \\text{ consecutive samples}, \\qquad P_{fa} = "
            "\\frac{N_{\\text{alarms before onset}}}{N_{\\text{pre-fault samples}}}";
        op.explain =
            "A fault is not a special signal; it is a residual that stops looking like noise. "
            "Stuck freezes the reading, drift adds a slow ramp that the controller cannot "
            "distinguish from real motion, and dropout replaces the reading with zero. The test "
            "compares the residual against a fixed multiple of the known measurement sigma and "
            "then demands m consecutive violations, because one tail draw is not a fault. Move "
            "the threshold down and the detection delay falls while the false-alarm rate climbs - "
            "that trade is the entire design decision, and both numbers come back so it can be "
            "made with evidence instead of taste.";
        op.params = {
            ParamSpec::enumeration("fault", "Injected fault", {"none", "stuck", "drift", "dropout"},
                                   "drift"),
            ParamSpec::scalar("onset", "Fault onset", "s", 0.0, 20.0, 1.0),
            ParamSpec::scalar("drift_rate", "Drift rate", "rad/s", -5.0, 5.0, 0.05),
            ParamSpec::scalar("dropout_probability", "Dropout probability per sample", "", 0.0,
                              1.0, 0.6),
            ParamSpec::scalar("threshold_sigma", "Residual threshold", "sigma", 0.5, 12.0, 4.0),
            ParamSpec::integer("consecutive", "Confirmation samples", "", 1, 100, 3),
            ParamSpec::scalar("noise_sigma", "Measurement sigma", "rad", 1e-9, 1.0, 0.004),
            ParamSpec::scalar("duration", "Record length", "s", 0.2, 20.0, 3.0),
            ParamSpec::scalar("sample_rate", "Sampling rate", "Hz", 10.0, 5000.0, 200.0),
            ParamSpec::integer("seed", "Random seed", "", kSeedMin, kSeedMax, 23),
        };
        op.outputs = {
            OutputSpec::make("traces", "series_set",
                             "Command, measurement, residual, thresholds and the alarm"),
            OutputSpec::make("summary", "table", "The detection record"),
            OutputSpec::make("detected", "bool", "Whether the fault was caught"),
            OutputSpec::make("detection_delay", "scalar", "Delay from onset to alarm", "s"),
            OutputSpec::make("false_alarm_rate", "scalar", "Alarms per pre-fault sample"),
            OutputSpec::make("residual_limit", "scalar", "Threshold that was applied", "rad"),
            OutputSpec::make("verdict", "text", "The delay against the false-alarm rate"),
        };
        d.ops.push_back(std::move(op));
    }
    return d;
}

json::Value SensingModelsModule::invoke(std::string_view op, const json::Value& args) const {
    if (op == "encoder") {
        return op_encoder(args);
    }
    if (op == "noise_models") {
        return op_noise_models(args);
    }
    if (op == "conditioning_chain") {
        return op_conditioning_chain(args);
    }
    if (op == "bayes_fusion") {
        return op_bayes_fusion(args);
    }
    if (op == "fault_detection") {
        return op_fault_detection(args);
    }
    unknown_op(name(), op);
}

}  // namespace yaskawa::study
