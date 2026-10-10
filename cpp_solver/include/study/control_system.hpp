#ifndef YASKAWA_STUDY_CONTROL_SYSTEM_HPP
#define YASKAWA_STUDY_CONTROL_SYSTEM_HPP

// Course 3884 "Robot Control and Feedback Systems" (M-408-01), sessions 1-24.
//
// The plant is not a textbook example: it is one selectable joint of the GP8,
// with the joint-side inertia evaluated from the link geometry of
// study/gp8_model.hpp at a selectable arm configuration, the reflected rotor
// inertia of that axis' gearbox, its viscous coefficient, and its Coulomb
// torque linearised at a stated reference velocity. Everything above the plant
// - block algebra, stability, the root locus, PID and its tuning rules - is
// built on the small transfer-function core in namespace `control` below, so
// there is exactly one polynomial multiply and one integrator in the project.
//
// Coefficient convention everywhere: `Poly` is DESCENDING powers, so
//     {2, 3, 4}  means  2 s^2 + 3 s + 4.
//
// Type vocabulary note (contract section 3): `root_locus` reports one
// complex_set per locus branch. That list-of-complex_set is declared as
// `complex_set_set`, following the same idiom the contract already uses for
// `mat4_set` (a list of mat4).

#include "study/study_module.hpp"

#include <Eigen/Dense>

#include <complex>
#include <cstddef>
#include <string>
#include <string_view>
#include <vector>

namespace yaskawa::study {

namespace control {

// ---------------------------------------------------------------------------
// Polynomials in descending powers
// ---------------------------------------------------------------------------

using Poly = std::vector<double>;
using Complex = std::complex<double>;

// Drops leading (highest-power) coefficients that are numerically zero, so the
// degree a polynomial reports is the degree it actually has.
[[nodiscard]] Poly poly_trim(Poly p);

[[nodiscard]] Poly poly_multiply(const Poly& a, const Poly& b);
[[nodiscard]] Poly poly_add(const Poly& a, const Poly& b);
[[nodiscard]] Poly poly_subtract(const Poly& a, const Poly& b);
[[nodiscard]] Poly poly_scale(const Poly& a, double k);
[[nodiscard]] Poly poly_power(const Poly& a, std::size_t exponent);
[[nodiscard]] Complex poly_eval(const Poly& a, const Complex& s);
[[nodiscard]] std::string poly_to_string(const Poly& a, std::string_view variable = "s");

// Roots of a polynomial as the eigenvalues of its companion matrix.
[[nodiscard]] std::vector<Complex> poly_roots(const Poly& a);

// Characteristic polynomial of A and, as a by-product, the matrices of
// adj(sI - A) = sum_k M_k s^(n-k). Faddeev-LeVerrier, used by every
// state-space -> transfer-function conversion in the study layer.
struct CharacteristicPoly {
    Poly coefficients;                     // monic, descending, length n + 1
    std::vector<Eigen::MatrixXd> adjugate; // M_1 .. M_n, index 0 is M_1
};

[[nodiscard]] CharacteristicPoly characteristic_poly(const Eigen::MatrixXd& A);

// e^(A) by scaling and squaring with a truncated Taylor series. Self-contained
// so the build never needs unsupported/Eigen/MatrixFunctions.
[[nodiscard]] Eigen::MatrixXd matrix_exponential(const Eigen::MatrixXd& A);

// ---------------------------------------------------------------------------
// TransferFunction - the reusable core
// ---------------------------------------------------------------------------

struct StateSpace {
    Eigen::MatrixXd A;      // n x n, controller canonical form
    Eigen::VectorXd B;      // n x 1
    Eigen::RowVectorXd C;   // 1 x n
    double D = 0.0;
};

struct Response {
    std::vector<double> t;
    std::vector<double> y;
};

class TransferFunction {
public:
    TransferFunction();                              // 1 / 1
    TransferFunction(Poly numerator, Poly denominator);

    [[nodiscard]] static TransferFunction gain(double k);
    [[nodiscard]] static TransferFunction integrator(double k);
    [[nodiscard]] static TransferFunction differentiator(double k);
    [[nodiscard]] static TransferFunction first_order_lag(double k, double tau);
    [[nodiscard]] static TransferFunction second_order(double k, double zeta, double omega_n);
    [[nodiscard]] static TransferFunction lead_lag(double k, double lead_tau, double lag_tau);
    // First-order Pade stand-in for e^(-tau s): the course's dead-time link has
    // no rational form, and pretending otherwise is the one lie this module
    // refuses to tell silently.
    [[nodiscard]] static TransferFunction pade_delay(double dead_time);

    [[nodiscard]] const Poly& numerator() const noexcept { return num_; }
    [[nodiscard]] const Poly& denominator() const noexcept { return den_; }
    [[nodiscard]] std::size_t order() const noexcept { return den_.size() - 1; }
    [[nodiscard]] std::size_t numerator_degree() const noexcept { return num_.size() - 1; }
    [[nodiscard]] bool is_proper() const noexcept { return num_.size() <= den_.size(); }
    [[nodiscard]] bool is_strictly_proper() const noexcept { return num_.size() < den_.size(); }

    [[nodiscard]] Complex evaluate(const Complex& s) const;
    [[nodiscard]] Complex frequency_response(double omega) const;
    [[nodiscard]] double dc_gain() const;  // s = 0; +/-inf when a pole sits there

    [[nodiscard]] std::vector<Complex> poles() const;
    [[nodiscard]] std::vector<Complex> zeros() const;
    [[nodiscard]] bool is_stable() const;          // every pole strictly in the LHP
    [[nodiscard]] int rhp_pole_count() const;      // poles with Re > tolerance
    [[nodiscard]] int imaginary_axis_pole_count() const;

    [[nodiscard]] TransferFunction scaled(double k) const;
    [[nodiscard]] TransferFunction series(const TransferFunction& other) const;
    [[nodiscard]] TransferFunction parallel(const TransferFunction& other) const;
    // G / (1 +/- G H); `negative` selects the minus sign in the loop, i.e. the
    // plus sign in the denominator.
    [[nodiscard]] TransferFunction feedback(const TransferFunction& h, bool negative) const;
    // Unity-feedback closed loop of this transfer function as the loop gain.
    [[nodiscard]] TransferFunction closed_loop() const;

    [[nodiscard]] StateSpace state_space() const;
    [[nodiscard]] Response step_response(double duration, std::size_t samples,
                                         double amplitude = 1.0) const;
    [[nodiscard]] Response impulse_response(double duration, std::size_t samples) const;

    [[nodiscard]] std::string to_string(std::string_view variable = "s") const;

private:
    [[nodiscard]] Response simulate(double duration, std::size_t samples, double input_level,
                                    bool from_impulse) const;

    Poly num_;
    Poly den_;
};

// ---------------------------------------------------------------------------
// Time-domain specification (sessions 5 and 14)
// ---------------------------------------------------------------------------

struct TransientSpec {
    double rise_time = 0.0;            // 10 % -> 90 % of the final value [s]
    double peak_time = 0.0;            // [s]
    double peak_value = 0.0;
    double overshoot_percent = 0.0;
    double settling_time = 0.0;        // [s], last exit from the band
    double settling_band = 0.02;       // fraction of the final value
    double steady_state_value = 0.0;
    double steady_state_error = 0.0;
    bool settled = false;
    bool rise_measured = false;
};

[[nodiscard]] TransientSpec transient_spec(const Response& response, double reference,
                                           double final_value, double band);

// ---------------------------------------------------------------------------
// Frequency domain (sessions 7 and 11)
// ---------------------------------------------------------------------------

struct BodeData {
    std::vector<double> omega;
    std::vector<double> magnitude_db;
    std::vector<double> phase_deg;   // unwrapped, so a margin search is sane
};

[[nodiscard]] BodeData bode(const TransferFunction& loop, double omega_min, double omega_max,
                            std::size_t points);

struct Margins {
    double gain_margin_db = 0.0;
    double gain_margin_linear = 0.0;
    double phase_crossover_omega = 0.0;   // where the phase hits -180 deg
    double phase_margin_deg = 0.0;
    double gain_crossover_omega = 0.0;    // where the magnitude hits 0 dB
    bool has_gain_margin = false;
    bool has_phase_margin = false;
};

[[nodiscard]] Margins margins_from_bode(const BodeData& data);
[[nodiscard]] Margins loop_margins(const TransferFunction& loop, double omega_min,
                                   double omega_max, std::size_t points);

// ---------------------------------------------------------------------------
// Stability (sessions 10, 11, 12)
// ---------------------------------------------------------------------------

struct RouthArray {
    std::vector<std::vector<double>> rows;   // first column is rows[i][0]
    std::vector<std::string> labels;         // "s^n" .. "s^0"
    int sign_changes = 0;                    // = number of RHP roots
    bool stable = false;
    bool epsilon_used = false;               // a zero first element was perturbed
    bool auxiliary_used = false;             // a whole row vanished: marginal case
    std::string note;
};

[[nodiscard]] RouthArray routh_array(const Poly& characteristic);

struct NyquistResult {
    std::vector<Complex> contour;      // L(s) along the Nyquist D-contour
    std::vector<double> omega;         // the positive-frequency part only
    int encirclements = 0;             // clockwise encirclements of -1
    int open_loop_rhp_poles = 0;       // P
    int closed_loop_rhp_poles = 0;     // Z = N + P
    bool stable = false;
    std::string note;
};

[[nodiscard]] NyquistResult nyquist(const TransferFunction& loop, double omega_min,
                                    double omega_max, std::size_t points);

// ---------------------------------------------------------------------------
// The GP8 joint plant (sessions 2 and 9)
// ---------------------------------------------------------------------------

struct JointPlant {
    std::size_t joint = 1;              // 0-based index into GP8_LINKS
    double link_inertia = 0.0;          // [kg m^2] joint side, this configuration
    double payload_inertia = 0.0;       // [kg m^2] contribution of the payload
    double reflected_inertia = 0.0;     // [kg m^2] n^2 J_rotor
    double total_inertia = 0.0;         // [kg m^2]
    double gear_ratio = 0.0;
    double viscous = 0.0;               // [N m s / rad]
    double coulomb = 0.0;               // [N m]
    double coulomb_equivalent = 0.0;    // [N m s / rad] at omega_reference
    double damping = 0.0;               // viscous + coulomb_equivalent
    double omega_reference = 0.0;       // [rad/s] where Coulomb was linearised
    double drive_torque = 0.0;          // [N m] joint torque at command u = 1
    double dc_gain = 0.0;               // K   = drive_torque / damping
    double time_constant = 0.0;         // T   = total_inertia / damping
    double corner_frequency = 0.0;      // 1/T [rad/s]
    TransferFunction tf;                // K / (s (T s + 1)), angle per command
};

// Joint-side inertia seen by `joint` with the arm at `q` and `payload_kg` at
// the flange: the composite-rigid-body diagonal term, summed over every link
// outboard of that axis.
[[nodiscard]] double joint_side_inertia(std::size_t joint,
                                        const Eigen::Matrix<double, 6, 1>& q,
                                        double payload_kg);

[[nodiscard]] JointPlant build_joint_plant(std::size_t joint,
                                           const Eigen::Matrix<double, 6, 1>& q,
                                           double payload_kg, double velocity_fraction,
                                           double torque_fraction);

// PID with a first-order filter on the derivative term:
//     C(s) = Kp + Ki/s + Kd s / (tau s + 1)
[[nodiscard]] TransferFunction pid_controller(double kp, double ki, double kd,
                                              double derivative_tau);

}  // namespace control

class ControlSystemModule final : public StudyModule {
public:
    [[nodiscard]] std::string_view name() const noexcept override { return "control_system"; }
    [[nodiscard]] ModuleDescription describe() const override;
    [[nodiscard]] json::Value invoke(std::string_view op, const json::Value& args) const override;
};

}  // namespace yaskawa::study

#endif  // YASKAWA_STUDY_CONTROL_SYSTEM_HPP
