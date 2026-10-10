#ifndef YASKAWA_STUDY_DIGITAL_CONTROL_HPP
#define YASKAWA_STUDY_DIGITAL_CONTROL_HPP

// Course 3884 "Robot Control and Feedback Systems" (M-408-01), sessions 25-26,
// the bridge from continuous control to the DSP course.
//
// The plant is the same GP8 joint control_system.hpp derives; what changes is
// that the controller now runs on a YRC1000micro at a finite rate, reads a
// finite-resolution encoder and writes a finite-word-length command. Every op
// here measures what that costs: the pole map from the s plane into the z
// plane, the phase margin the half-sample hold eats, the sample rate at which
// the loop stops being stable at all, and the position ripple the encoder
// quantum leaves against the GP8's 20 micrometre repeatability.
//
// Coefficient convention: DESCENDING powers of z, so {1, -1.8, 0.81} means
// z^2 - 1.8 z + 0.81.

#include "study/control_system.hpp"
#include "study/study_module.hpp"

#include <complex>
#include <cstddef>
#include <string>
#include <string_view>
#include <vector>

namespace yaskawa::study {

namespace digital {

using control::Complex;
using control::Poly;
using control::TransferFunction;

class DiscreteTransferFunction {
public:
    DiscreteTransferFunction();
    DiscreteTransferFunction(Poly numerator, Poly denominator, double sample_period);

    [[nodiscard]] const Poly& numerator() const noexcept { return num_; }
    [[nodiscard]] const Poly& denominator() const noexcept { return den_; }
    [[nodiscard]] double sample_period() const noexcept { return sample_period_; }
    [[nodiscard]] std::size_t order() const noexcept { return den_.size() - 1; }

    [[nodiscard]] std::vector<Complex> poles() const;
    [[nodiscard]] std::vector<Complex> zeros() const;
    // Every pole strictly inside the unit circle: the z-domain stability test.
    [[nodiscard]] bool is_stable() const;
    [[nodiscard]] double spectral_radius() const;

    [[nodiscard]] Complex evaluate(const Complex& z) const;
    [[nodiscard]] double dc_gain() const;  // z = 1

    // The difference equation run forward, which is what a controller does.
    [[nodiscard]] std::vector<double> response_to(const std::vector<double>& input) const;
    [[nodiscard]] std::vector<double> step_response(std::size_t steps,
                                                    double amplitude = 1.0) const;
    [[nodiscard]] std::string difference_equation() const;
    [[nodiscard]] std::string to_string() const;

    // Unity-feedback closed loop in the z domain.
    [[nodiscard]] DiscreteTransferFunction closed_loop() const;

private:
    Poly num_;
    Poly den_;
    double sample_period_ = 1.0;
};

enum class Method { ZeroOrderHold, Bilinear, ForwardEuler, BackwardEuler, MatchedPoles };

[[nodiscard]] Method method_from_string(std::string_view name);
[[nodiscard]] std::string method_name(Method method);
[[nodiscard]] std::string method_substitution(Method method, double sample_period);

[[nodiscard]] DiscreteTransferFunction discretise(const TransferFunction& continuous,
                                                  double sample_period, Method method);

// A real polynomial from a conjugate-closed root set.
[[nodiscard]] Poly poly_from_roots(const std::vector<Complex>& roots);

// One sampled-data run of the real loop: the continuous plant integrated with
// Runge-Kutta between samples, the controller executed once per sample with the
// command held. Nonlinear on purpose - saturation, integral windup and
// quantisation cannot be seen in a transfer function.
struct SampledLoopOptions {
    double sample_period = 0.004;
    double duration = 4.0;
    double setpoint = 1.0;
    double kp = 8.0;
    double ki = 4.0;
    double kd = 1.0;
    double derivative_filter = 0.01;   // seconds, 0 disables the filter
    bool anti_windup = true;
    double command_limit = 1.0;        // in command units, 1.0 = rated torque
    double measurement_quantum = 0.0;  // encoder resolution at the joint [rad]
    double command_quantum = 0.0;      // controller word length, command units
    double disturbance = 0.0;          // command-unit load applied as a step
    double disturbance_time = 0.0;
};

struct SampledLoopRun {
    std::vector<double> t;
    std::vector<double> y;          // joint angle [rad]
    std::vector<double> u;          // command actually written, after limits
    std::vector<double> integral;   // the integrator state, so windup is visible
    double peak_command = 0.0;
    double peak_integral = 0.0;
    double final_value = 0.0;
    double saturated_fraction = 0.0;
};

[[nodiscard]] SampledLoopRun simulate_sampled_loop(const TransferFunction& plant,
                                                   const SampledLoopOptions& options);

}  // namespace digital

class DigitalControlModule final : public StudyModule {
public:
    [[nodiscard]] std::string_view name() const noexcept override { return "digital_control"; }
    [[nodiscard]] ModuleDescription describe() const override;
    [[nodiscard]] json::Value invoke(std::string_view op, const json::Value& args) const override;
};

}  // namespace yaskawa::study

#endif  // YASKAWA_STUDY_DIGITAL_CONTROL_HPP
