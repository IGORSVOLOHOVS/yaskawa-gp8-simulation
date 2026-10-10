#ifndef YASKAWA_STUDY_DYNAMICS_HPP
#define YASKAWA_STUDY_DYNAMICS_HPP

// Course 3883 "Robotics Modelling" (M-407-01),
// Block 4 "Dynamics: energy methods, the Lagrangian formulation, the
// Newton-Euler recursion and the critical comparison of the two".
//
// The whole point of this module is that the two formulations are written out
// independently and then shown to agree:
//
//   Lagrangian (closed form)   tau = M(q) qdd + C(q,qd) qd + g(q) + f(qd)
//       M(q) from the link Jacobians, C(q,qd) from the Christoffel symbols of
//       analytic dM/dq, g(q) from the gradient of the potential energy.
//
//   Newton-Euler (recursive)   an outward velocity/acceleration pass followed
//       by an inward force/moment pass, O(n) in the number of links, with
//       gravity injected as a base acceleration of -g.
//
// Both read the *same* rigid-body data from study/gp8_model.hpp (and fold an
// optional flange payload into link 6 the same way), so a disagreement is a
// bug in one of the algorithms and nothing else. src/test_study_dynamics.cpp
// asserts the agreement to 1e-8 on random states.
//
// Model honesty: the masses, centres of mass and inertia tensors in
// gp8_model.hpp are integrated from the Yaskawa CAD meshes by
// tools/compute_link_mass_properties.py, with one effective density solved so
// the seven links sum to the published 32 kg of the GP8 datasheet column (a
// figure Yaskawa's manual disputes at 35 kg; see GP8_ROBOT_MASS_KG) - that
// geometry is real, the
// uniform-density assumption behind the split between the links is not. The
// gear ratios, rotor inertias and friction coefficients remain ESTIMATED;
// Yaskawa publishes none of them. Every number this module returns is a number
// about that model.

#include "study/gp8_model.hpp"
#include "study/study_module.hpp"

#include <Eigen/Dense>

#include <array>
#include <cstddef>
#include <string_view>

namespace yaskawa::study {

namespace dyn {

using Vector6 = Eigen::Matrix<double, 6, 1>;
using Matrix6 = Eigen::Matrix<double, 6, 6>;

// One rigid body of the chain, expressed in its own DH frame i.
struct Body {
    double mass = 0.0;                                 // [kg]
    Eigen::Vector3d com{Eigen::Vector3d::Zero()};      // [m], in frame i
    Eigen::Matrix3d inertia{Eigen::Matrix3d::Zero()};  // [kg m^2] about com, frame i axes
};

using BodyArray = std::array<Body, GP8_DOF>;

// The six links of GP8_LINKS, with a point payload at the flange origin folded
// into link 6 as one composite rigid body (mass added, centre of mass moved,
// inertia carried over with the parallel-axis shift). Both formulations call
// this, so they are always arguing about the same robot.
[[nodiscard]] BodyArray bodies_with_payload(double payload_mass) noexcept;

// Joint-side inertia the six motors add through their gearboxes, n^2 J_rotor.
[[nodiscard]] Vector6 rotor_inertia_diagonal() noexcept;

// Rigid-body inertia matrix, M = sum_i (m_i Jv_i^T Jv_i + Jw_i^T R_i I_i R_i^T Jw_i).
[[nodiscard]] Matrix6 rigid_mass_matrix(const Vector6& q, double payload_mass) noexcept;

// The same plus diag(n^2 J_rotor): what the joints actually have to accelerate.
[[nodiscard]] Matrix6 mass_matrix(const Vector6& q, double payload_mass) noexcept;

// dM/dq_k for k = 0..5, analytic (no finite differences anywhere in this file).
[[nodiscard]] std::array<Matrix6, GP8_DOF> mass_matrix_gradient(const Vector6& q,
                                                                double payload_mass) noexcept;

// C(q, qd) from the Christoffel symbols of the first kind,
//   C_ij = sum_k 0.5 (dM_ij/dq_k + dM_ik/dq_j - dM_jk/dq_i) qd_k.
[[nodiscard]] Matrix6 coriolis_matrix(const Vector6& q, const Vector6& qd,
                                      double payload_mass) noexcept;

// Mdot = sum_k (dM/dq_k) qd_k. Used for the passivity identity
// qd^T (Mdot - 2C) qd = 0, which is a free correctness check on C.
[[nodiscard]] Matrix6 mass_matrix_rate(const Vector6& q, const Vector6& qd,
                                       double payload_mass) noexcept;

// g(q) = -sum_i m_i Jv_i^T g_vec, the holding torque against gravity.
[[nodiscard]] Vector6 gravity_torque(const Vector6& q, double payload_mass) noexcept;

// The same with an arbitrary gravity vector, so a test can switch gravity off.
[[nodiscard]] Vector6 gravity_torque(const Vector6& q, double payload_mass,
                                     const Eigen::Vector3d& gravity) noexcept;

// Viscous plus Coulomb friction, joint side: b_i qd_i + tau_c,i sign(qd_i).
[[nodiscard]] Vector6 friction_torque(const Vector6& qd) noexcept;

// Closed-form inverse dynamics from the energy method.
[[nodiscard]] Vector6 inverse_dynamics_lagrangian(const Vector6& q, const Vector6& qd,
                                                  const Vector6& qdd, double payload_mass,
                                                  bool include_friction) noexcept;

// Recursive inverse dynamics: outward kinematics pass, inward wrench pass.
[[nodiscard]] Vector6 inverse_dynamics_newton_euler(const Vector6& q, const Vector6& qd,
                                                    const Vector6& qdd, double payload_mass,
                                                    bool include_friction) noexcept;

struct ForwardResult {
    Vector6 qddot{Vector6::Zero()};
    double residual = 0.0;          // || M qdd - (tau - C qd - g - f) ||
    double condition_number = 0.0;  // of M, from its symmetric eigenvalues
    bool positive_definite = false;
};

// M qdd = tau - C(q,qd) qd - g(q) - f(qd), solved by LDLT on the symmetric M.
[[nodiscard]] ForwardResult forward_dynamics(const Vector6& q, const Vector6& qd,
                                             const Vector6& tau, double payload_mass,
                                             bool include_friction) noexcept;

struct EnergyTerms {
    double kinetic = 0.0;    // 0.5 qd^T M(q) qd, rotors included
    double potential = 0.0;  // sum_i m_i g h_i, datum at the base frame z = 0
    double total = 0.0;
    Vector6 kinetic_per_joint{Vector6::Zero()};    // 0.5 qd_i (M qd)_i, sums to kinetic
    Vector6 potential_per_link{Vector6::Zero()};
    Eigen::Vector3d centre_of_mass{Eigen::Vector3d::Zero()};
    double total_mass = 0.0;
};

[[nodiscard]] EnergyTerms energy(const Vector6& q, const Vector6& qd, double payload_mass) noexcept;

// Symmetric eigenvalues of M, ascending. Public because both the mass_matrix op
// and the forward_dynamics conditioning report need them.
[[nodiscard]] Vector6 symmetric_eigenvalues(const Matrix6& M) noexcept;

}  // namespace dyn

class DynamicsModule final : public StudyModule {
public:
    [[nodiscard]] std::string_view name() const noexcept override { return "dynamics"; }
    [[nodiscard]] ModuleDescription describe() const override;
    [[nodiscard]] json::Value invoke(std::string_view op, const json::Value& args) const override;
};

}  // namespace yaskawa::study

#endif  // YASKAWA_STUDY_DYNAMICS_HPP
