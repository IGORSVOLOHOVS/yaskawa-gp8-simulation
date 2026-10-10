#ifndef YASKAWA_STUDY_SENSING_MODELS_HPP
#define YASKAWA_STUDY_SENSING_MODELS_HPP

// Course 3884 "Robotic Systems Design", Sessions 27-28:
// information (sensing) devices of robotic systems, signal conditioning and
// filtering. Course 3286 (probability, random variables, the normal
// distribution) is the secondary course: this is where its distributions stop
// being abstract and become the error of a measurement on the GP8.
//
// Everything here is a *model* of a device on the robot of study/gp8_model.hpp:
// the encoder resolution is bits plus that joint's gear ratio, the noise is
// added to a signal the robot actually produces, and the fusion is Bayes' rule
// written out rather than a library call.
//
// Determinism is a hard requirement of this layer: every stochastic op takes an
// integer `seed` and draws from SeededRng below, which owns its own uniform,
// Gaussian and Poisson transforms so that the byte stream does not depend on
// the standard library's distribution implementations.

#include "study/study_module.hpp"

#include <cmath>
#include <cstdint>
#include <numbers>
#include <random>
#include <string_view>
#include <vector>

namespace yaskawa::study {

namespace sensing {

// A seeded std::mt19937 plus the three transforms the study layer needs.
// No std::random_device, no time, no std::*_distribution: the same seed gives
// the same bytes on every platform and in every build of this project.
class SeededRng {
public:
    explicit SeededRng(int seed) noexcept
        : engine_(static_cast<std::uint32_t>(static_cast<std::uint64_t>(
                      static_cast<std::int64_t>(seed)) & 0xFFFFFFFFULL)) {}

    // Open interval (0, 1): never exactly 0, so std::log() is always safe.
    [[nodiscard]] double uniform01() noexcept {
        return (static_cast<double>(engine_()) + 0.5) / 4294967296.0;
    }

    [[nodiscard]] double uniform(double low, double high) noexcept {
        return low + (high - low) * uniform01();
    }

    // Box-Muller, Cartesian form. The second deviate of each pair is cached,
    // which is why the draw order is part of the reproducible stream.
    [[nodiscard]] double gaussian() noexcept {
        if (has_spare_) {
            has_spare_ = false;
            return spare_;
        }
        const double u1 = uniform01();
        const double u2 = uniform01();
        const double radius = std::sqrt(-2.0 * std::log(u1));
        const double angle = 2.0 * std::numbers::pi * u2;
        spare_ = radius * std::sin(angle);
        has_spare_ = true;
        return radius * std::cos(angle);
    }

    [[nodiscard]] double gaussian(double mean, double sigma) noexcept {
        return mean + sigma * gaussian();
    }

    // Knuth's product-of-uniforms method. Exact for the lambda range the UI
    // exposes (<= 500), which is all a count-type sensor model needs.
    [[nodiscard]] int poisson(double lambda) noexcept {
        if (!(lambda > 0.0)) {
            return 0;
        }
        const double limit = std::exp(-lambda);
        int k = 0;
        double product = 1.0;
        do {
            ++k;
            product *= uniform01();
        } while (product > limit && k < 1000000);
        return k - 1;
    }

    // Uniform index in [0, n), for a tournament or a shuffled split.
    [[nodiscard]] std::size_t index(std::size_t n) noexcept {
        if (n == 0) {
            return 0;
        }
        const auto picked = static_cast<std::size_t>(uniform01() * static_cast<double>(n));
        return (picked >= n) ? (n - 1) : picked;
    }

private:
    std::mt19937 engine_;
    double spare_ = 0.0;
    bool has_spare_ = false;
};

// Sample moments of a finite record, computed in one pass over the data with
// the two-pass variance so that a large mean does not eat the precision.
struct SampleStats {
    double mean = 0.0;
    double variance = 0.0;  // unbiased (n-1) when n > 1
    double stddev = 0.0;
    double minimum = 0.0;
    double maximum = 0.0;
    double rms = 0.0;
    double skewness = 0.0;
    std::size_t count = 0;
};

[[nodiscard]] SampleStats summarise(const std::vector<double>& samples) noexcept;

// One encoder reading: the nearest multiple of the least significant increment.
[[nodiscard]] double quantise(double value, double step) noexcept;

// Smallest joint increment an n-bit encoder behind a gearbox can resolve:
//     delta = 2 pi / (2^bits * gear_ratio)
[[nodiscard]] double joint_increment(int bits, double gear_ratio) noexcept;

// Scalar Kalman / recursive-Bayes update of a Gaussian belief by a Gaussian
// measurement. Returns the posterior; `gain` is the Kalman gain that was used.
struct GaussianBelief {
    double mean = 0.0;
    double variance = 1.0;
};

[[nodiscard]] GaussianBelief bayes_update(const GaussianBelief& prior, double measurement,
                                          double measurement_variance, double& gain) noexcept;

}  // namespace sensing

class SensingModelsModule final : public StudyModule {
public:
    [[nodiscard]] std::string_view name() const noexcept override { return "sensing_models"; }
    [[nodiscard]] ModuleDescription describe() const override;
    [[nodiscard]] json::Value invoke(std::string_view op, const json::Value& args) const override;
};

}  // namespace yaskawa::study

#endif  // YASKAWA_STUDY_SENSING_MODELS_HPP
