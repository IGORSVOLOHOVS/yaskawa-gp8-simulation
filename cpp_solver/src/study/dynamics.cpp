#include "study/dynamics.hpp"

#include <Eigen/Eigenvalues>

#include <algorithm>
#include <chrono>
#include <cmath>
#include <limits>
#include <string>
#include <utility>
#include <vector>

namespace yaskawa::study {

namespace {

using dyn::Body;
using dyn::BodyArray;
using dyn::Matrix6;
using dyn::Vector6;

constexpr double kMaxPayload = GP8_PAYLOAD_KG;  // 8 kg, the datasheet rating
constexpr double kVelocityGuard = 50.0;         // [rad/s], far outside any axis rating
constexpr double kAccelGuard = 500.0;           // [rad/s^2]
constexpr double kTorqueGuard = 5000.0;         // [N m]
constexpr int kTimingRepetitions = 200;         // enough to resolve one call on a 1 us clock

[[nodiscard]] Eigen::Matrix3d skew(const Eigen::Vector3d& v) noexcept {
    Eigen::Matrix3d S;
    S <<  0.0, -v.z(),  v.y(),
        v.z(),    0.0, -v.x(),
       -v.y(),  v.x(),    0.0;
    return S;
}

[[nodiscard]] double sign_of(double v) noexcept {
    return (v > 0.0) ? 1.0 : ((v < 0.0) ? -1.0 : 0.0);
}

// The geometry every closed-form term is built from: the six link frames, the
// axis of each joint and a point on it, all in the base frame.
struct Chain {
    std::array<Eigen::Isometry3d, GP8_DOF> T;  // T_0_i, i = 1..6
    std::array<Eigen::Vector3d, GP8_DOF> z;    // z_{i-1}, the axis of joint i
    std::array<Eigen::Vector3d, GP8_DOF> o;    // o_{i-1}, a point on that axis
};

[[nodiscard]] Chain chain_of(const Vector6& q) noexcept {
    Chain c;
    c.T = link_frames(q);
    c.z[0] = Eigen::Vector3d::UnitZ();
    c.o[0] = Eigen::Vector3d::Zero();
    for (std::size_t i = 1; i < GP8_DOF; ++i) {
        c.z[i] = c.T[i - 1].linear().col(2);
        c.o[i] = c.T[i - 1].translation();
    }
    return c;
}

// Per-link Jacobians of the centre of mass (Jv) and of the link frame (Jw),
// both in the base frame. Column j is zero for j > i: a joint beyond link i
// cannot move link i.
struct LinkJacobians {
    std::array<Eigen::Matrix<double, 3, 6>, GP8_DOF> Jv;
    std::array<Eigen::Matrix<double, 3, 6>, GP8_DOF> Jw;
    std::array<Eigen::Vector3d, GP8_DOF> p_com;        // com of link i in the base frame
    std::array<Eigen::Matrix3d, GP8_DOF> inertia_base; // R_i I_i R_i^T
};

[[nodiscard]] LinkJacobians link_jacobians(const Chain& c, const BodyArray& bodies) noexcept {
    LinkJacobians out;
    for (std::size_t i = 0; i < GP8_DOF; ++i) {
        out.p_com[i] = c.T[i] * bodies[i].com;
        const Eigen::Matrix3d R = c.T[i].linear();
        out.inertia_base[i] = R * bodies[i].inertia * R.transpose();
        out.Jv[i].setZero();
        out.Jw[i].setZero();
        for (std::size_t j = 0; j <= i; ++j) {
            const auto col = static_cast<Eigen::Index>(j);
            out.Jw[i].col(col) = c.z[j];
            out.Jv[i].col(col) = c.z[j].cross(out.p_com[i] - c.o[j]);
        }
    }
    return out;
}

// d(Jv_i)/dq_k, d(Jw_i)/dq_k and d(R_i I_i R_i^T)/dq_k, all analytic. For a
// chain of revolute joints every derivative is a cross product with z_k:
//   dz_j/dq_k    = z_k x z_j            (k < j)
//   do_j/dq_k    = z_k x (o_j - o_k)    (k < j)
//   dp_ci/dq_k   = z_k x (p_ci - o_k)   (k <= i)
//   dR_i/dq_k    = skew(z_k) R_i        (k <= i)
struct LinkJacobianDerivative {
    Eigen::Matrix<double, 3, 6> dJv{Eigen::Matrix<double, 3, 6>::Zero()};
    Eigen::Matrix<double, 3, 6> dJw{Eigen::Matrix<double, 3, 6>::Zero()};
    Eigen::Matrix3d dInertia{Eigen::Matrix3d::Zero()};
};

[[nodiscard]] LinkJacobianDerivative jacobian_derivative(const Chain& c, const LinkJacobians& jac,
                                                         std::size_t link,
                                                         std::size_t k) noexcept {
    LinkJacobianDerivative d;
    if (k > link) {
        return d;  // joint k+1 is distal to link i: it moves nothing of link i
    }
    const Eigen::Vector3d dp_com = c.z[k].cross(jac.p_com[link] - c.o[k]);
    for (std::size_t j = 0; j <= link; ++j) {
        const auto col = static_cast<Eigen::Index>(j);
        const Eigen::Vector3d dz = (k < j) ? Eigen::Vector3d(c.z[k].cross(c.z[j]))
                                           : Eigen::Vector3d::Zero();
        const Eigen::Vector3d doj = (k < j) ? Eigen::Vector3d(c.z[k].cross(c.o[j] - c.o[k]))
                                            : Eigen::Vector3d::Zero();
        d.dJw.col(col) = dz;
        d.dJv.col(col) = dz.cross(jac.p_com[link] - c.o[j]) + c.z[j].cross(dp_com - doj);
    }
    const Eigen::Matrix3d S = skew(c.z[k]);
    d.dInertia = S * jac.inertia_base[link] - jac.inertia_base[link] * S;
    return d;
}

[[nodiscard]] Matrix6 assemble_mass(const LinkJacobians& jac, const BodyArray& bodies) noexcept {
    Matrix6 M = Matrix6::Zero();
    for (std::size_t i = 0; i < GP8_DOF; ++i) {
        M.noalias() += bodies[i].mass * jac.Jv[i].transpose() * jac.Jv[i];
        M.noalias() += jac.Jw[i].transpose() * jac.inertia_base[i] * jac.Jw[i];
    }
    // M is symmetric by construction; symmetrise to kill the last bit of
    // floating-point asymmetry so LDLT and the eigen solver get a clean matrix.
    return 0.5 * (M + M.transpose());
}

}  // namespace

// ---------------------------------------------------------------------------
// dyn:: the physics
// ---------------------------------------------------------------------------

namespace dyn {

BodyArray bodies_with_payload(double payload_mass) noexcept {
    BodyArray bodies;
    for (std::size_t i = 0; i < GP8_DOF; ++i) {
        bodies[i].mass = GP8_LINKS[i].mass;
        bodies[i].com = link_com(i);
        bodies[i].inertia = link_inertia(i);
    }
    if (payload_mass <= 0.0) {
        return bodies;
    }
    // The payload is a point mass bolted to the flange, whose origin is the DH
    // frame 6 origin (a6 = d6 = 0). Composite body: add the mass, move the
    // centre of mass, and carry both inertias to the new centre of mass with
    // the parallel-axis theorem. A point mass has no inertia of its own.
    Body& wrist = bodies[GP8_DOF - 1];
    const double m_link = wrist.mass;
    const Eigen::Vector3d c_link = wrist.com;
    const Eigen::Vector3d c_load = Eigen::Vector3d::Zero();
    const double m_total = m_link + payload_mass;
    const Eigen::Vector3d c_total = (m_link * c_link + payload_mass * c_load) / m_total;

    const Eigen::Vector3d d_link = c_link - c_total;
    const Eigen::Vector3d d_load = c_load - c_total;
    Eigen::Matrix3d I = wrist.inertia;
    I += m_link * (d_link.squaredNorm() * Eigen::Matrix3d::Identity() -
                   d_link * d_link.transpose());
    I += payload_mass * (d_load.squaredNorm() * Eigen::Matrix3d::Identity() -
                         d_load * d_load.transpose());

    wrist.mass = m_total;
    wrist.com = c_total;
    wrist.inertia = I;
    return bodies;
}

Vector6 rotor_inertia_diagonal() noexcept {
    Vector6 out;
    for (std::size_t i = 0; i < GP8_DOF; ++i) {
        out(static_cast<Eigen::Index>(i)) = reflected_rotor_inertia(i);
    }
    return out;
}

Matrix6 rigid_mass_matrix(const Vector6& q, double payload_mass) noexcept {
    const BodyArray bodies = bodies_with_payload(payload_mass);
    const Chain c = chain_of(q);
    return assemble_mass(link_jacobians(c, bodies), bodies);
}

Matrix6 mass_matrix(const Vector6& q, double payload_mass) noexcept {
    Matrix6 M = rigid_mass_matrix(q, payload_mass);
    M.diagonal() += rotor_inertia_diagonal();
    return M;
}

std::array<Matrix6, GP8_DOF> mass_matrix_gradient(const Vector6& q,
                                                  double payload_mass) noexcept {
    const BodyArray bodies = bodies_with_payload(payload_mass);
    const Chain c = chain_of(q);
    const LinkJacobians jac = link_jacobians(c, bodies);

    std::array<Matrix6, GP8_DOF> gradient;
    for (std::size_t k = 0; k < GP8_DOF; ++k) {
        Matrix6 dM = Matrix6::Zero();
        for (std::size_t i = k; i < GP8_DOF; ++i) {  // links proximal to joint k+1 do not move
            const LinkJacobianDerivative d = jacobian_derivative(c, jac, i, k);
            dM.noalias() += bodies[i].mass * (d.dJv.transpose() * jac.Jv[i] +
                                              jac.Jv[i].transpose() * d.dJv);
            dM.noalias() += d.dJw.transpose() * jac.inertia_base[i] * jac.Jw[i];
            dM.noalias() += jac.Jw[i].transpose() * d.dInertia * jac.Jw[i];
            dM.noalias() += jac.Jw[i].transpose() * jac.inertia_base[i] * d.dJw;
        }
        gradient[k] = 0.5 * (dM + dM.transpose());  // dM/dq_k is symmetric too
    }
    return gradient;
}

Matrix6 coriolis_matrix(const Vector6& q, const Vector6& qd, double payload_mass) noexcept {
    const std::array<Matrix6, GP8_DOF> dM = mass_matrix_gradient(q, payload_mass);
    Matrix6 C = Matrix6::Zero();
    for (Eigen::Index i = 0; i < 6; ++i) {
        for (Eigen::Index j = 0; j < 6; ++j) {
            double sum = 0.0;
            for (Eigen::Index k = 0; k < 6; ++k) {
                const double christoffel =
                    0.5 * (dM[static_cast<std::size_t>(k)](i, j) +
                           dM[static_cast<std::size_t>(j)](i, k) -
                           dM[static_cast<std::size_t>(i)](j, k));
                sum += christoffel * qd(k);
            }
            C(i, j) = sum;
        }
    }
    return C;
}

Matrix6 mass_matrix_rate(const Vector6& q, const Vector6& qd, double payload_mass) noexcept {
    const std::array<Matrix6, GP8_DOF> dM = mass_matrix_gradient(q, payload_mass);
    Matrix6 Mdot = Matrix6::Zero();
    for (std::size_t k = 0; k < GP8_DOF; ++k) {
        Mdot.noalias() += qd(static_cast<Eigen::Index>(k)) * dM[k];
    }
    return Mdot;
}

Vector6 gravity_torque(const Vector6& q, double payload_mass,
                       const Eigen::Vector3d& gravity) noexcept {
    const BodyArray bodies = bodies_with_payload(payload_mass);
    const Chain c = chain_of(q);
    const LinkJacobians jac = link_jacobians(c, bodies);
    Vector6 g = Vector6::Zero();
    for (std::size_t i = 0; i < GP8_DOF; ++i) {
        g.noalias() -= bodies[i].mass * jac.Jv[i].transpose() * gravity;
    }
    return g;
}

Vector6 gravity_torque(const Vector6& q, double payload_mass) noexcept {
    return gravity_torque(q, payload_mass, gravity_vector());
}

Vector6 friction_torque(const Vector6& qd) noexcept {
    Vector6 out;
    for (std::size_t i = 0; i < GP8_DOF; ++i) {
        const auto index = static_cast<Eigen::Index>(i);
        out(index) = GP8_LINKS[i].viscous_friction * qd(index) +
                     GP8_LINKS[i].coulomb_friction * sign_of(qd(index));
    }
    return out;
}

Vector6 inverse_dynamics_lagrangian(const Vector6& q, const Vector6& qd, const Vector6& qdd,
                                    double payload_mass, bool include_friction) noexcept {
    const Matrix6 M = mass_matrix(q, payload_mass);
    const Matrix6 C = coriolis_matrix(q, qd, payload_mass);
    Vector6 tau = M * qdd + C * qd + gravity_torque(q, payload_mass);
    if (include_friction) {
        tau += friction_torque(qd);
    }
    return tau;
}

Vector6 inverse_dynamics_newton_euler(const Vector6& q, const Vector6& qd, const Vector6& qdd,
                                      double payload_mass, bool include_friction) noexcept {
    const BodyArray bodies = bodies_with_payload(payload_mass);

    // Per-link constants of the recursion, all in frame i.
    std::array<Eigen::Matrix3d, GP8_DOF> R;   // frame i -> frame i-1
    std::array<Eigen::Vector3d, GP8_DOF> p;   // o_i in frame i-1
    std::array<Eigen::Vector3d, GP8_DOF> axis;  // z_{i-1} in frame i
    std::array<Eigen::Vector3d, GP8_DOF> r_in;  // o_{i-1} -> o_i, in frame i
    for (std::size_t i = 0; i < GP8_DOF; ++i) {
        const Eigen::Isometry3d A = dh_transform(GP8_DH[i], q(static_cast<Eigen::Index>(i)));
        R[i] = A.linear();
        p[i] = A.translation();
        axis[i] = R[i].transpose() * Eigen::Vector3d::UnitZ();
        r_in[i] = R[i].transpose() * p[i];
    }

    // Outward pass: angular velocity, angular acceleration and the linear
    // acceleration of each centre of mass. Gravity enters once, as a base
    // acceleration of -g, which is why no weight term appears below.
    std::array<Eigen::Vector3d, GP8_DOF> w;
    std::array<Eigen::Vector3d, GP8_DOF> wd;
    std::array<Eigen::Vector3d, GP8_DOF> a_com;
    Eigen::Vector3d w_prev = Eigen::Vector3d::Zero();
    Eigen::Vector3d wd_prev = Eigen::Vector3d::Zero();
    Eigen::Vector3d a_prev = -gravity_vector();  // frame 0 = the base frame
    for (std::size_t i = 0; i < GP8_DOF; ++i) {
        const auto index = static_cast<Eigen::Index>(i);
        const Eigen::Matrix3d Rt = R[i].transpose();
        w[i] = Rt * w_prev + qd(index) * axis[i];
        wd[i] = Rt * wd_prev + qdd(index) * axis[i] + w[i].cross(qd(index) * axis[i]);
        const Eigen::Vector3d a_origin =
            Rt * a_prev + wd[i].cross(r_in[i]) + w[i].cross(w[i].cross(r_in[i]));
        a_com[i] = a_origin + wd[i].cross(bodies[i].com) +
                   w[i].cross(w[i].cross(bodies[i].com));
        w_prev = w[i];
        wd_prev = wd[i];
        a_prev = a_origin;
    }

    // Inward pass: the wrench each link has to receive from the one before it.
    Vector6 tau = Vector6::Zero();
    Eigen::Vector3d f_next = Eigen::Vector3d::Zero();  // no tool wrench at the flange
    Eigen::Vector3d n_next = Eigen::Vector3d::Zero();
    for (std::size_t step = GP8_DOF; step > 0; --step) {
        const std::size_t i = step - 1;
        const auto index = static_cast<Eigen::Index>(i);
        const Eigen::Vector3d F = bodies[i].mass * a_com[i];
        const Eigen::Vector3d N =
            bodies[i].inertia * wd[i] + w[i].cross(bodies[i].inertia * w[i]);

        const bool last = (i + 1 == GP8_DOF);
        const Eigen::Matrix3d R_next = last ? Eigen::Matrix3d::Identity() : R[i + 1];
        const Eigen::Vector3d r_next = last ? Eigen::Vector3d::Zero() : p[i + 1];
        const Eigen::Vector3d f_from_next = R_next * f_next;

        const Eigen::Vector3d f = f_from_next + F;
        const Eigen::Vector3d n = N + R_next * n_next + bodies[i].com.cross(F) +
                                  r_next.cross(f_from_next);

        // n is the moment about origin i, but joint i turns about z_{i-1}, a
        // line through origin i-1: in standard DH the two origins are a_i and
        // d_i apart. Transfer the moment to a point on the axis before
        // projecting onto it, or the a_i lever arm of the link force is lost
        // (on the L-axis, with a_2 = 0.345 m, that is 15 N m of gravity).
        const Eigen::Vector3d n_on_axis = n + r_in[i].cross(f);
        tau(index) = n_on_axis.dot(axis[i]) + reflected_rotor_inertia(i) * qdd(index);
        f_next = f;
        n_next = n;
    }
    if (include_friction) {
        tau += friction_torque(qd);
    }
    return tau;
}

Vector6 symmetric_eigenvalues(const Matrix6& M) noexcept {
    Eigen::SelfAdjointEigenSolver<Matrix6> solver(0.5 * (M + M.transpose()));
    if (solver.info() != Eigen::Success) {
        return Vector6::Zero();
    }
    return solver.eigenvalues();
}

ForwardResult forward_dynamics(const Vector6& q, const Vector6& qd, const Vector6& tau,
                               double payload_mass, bool include_friction) noexcept {
    const Matrix6 M = mass_matrix(q, payload_mass);
    Vector6 rhs = tau - coriolis_matrix(q, qd, payload_mass) * qd - gravity_torque(q, payload_mass);
    if (include_friction) {
        rhs -= friction_torque(qd);
    }
    ForwardResult out;
    const Eigen::LDLT<Matrix6> ldlt(M);
    out.qddot = ldlt.solve(rhs);
    out.residual = (M * out.qddot - rhs).norm();
    const Vector6 eigenvalues = symmetric_eigenvalues(M);
    out.positive_definite = eigenvalues.minCoeff() > 0.0;
    out.condition_number = (eigenvalues.minCoeff() > 0.0)
                               ? eigenvalues.maxCoeff() / eigenvalues.minCoeff()
                               : std::numeric_limits<double>::infinity();
    return out;
}

EnergyTerms energy(const Vector6& q, const Vector6& qd, double payload_mass) noexcept {
    const BodyArray bodies = bodies_with_payload(payload_mass);
    const Chain c = chain_of(q);
    const LinkJacobians jac = link_jacobians(c, bodies);
    const Eigen::Vector3d g = gravity_vector();

    Matrix6 M = assemble_mass(jac, bodies);
    M.diagonal() += rotor_inertia_diagonal();

    EnergyTerms out;
    const Vector6 momentum = M * qd;
    out.kinetic_per_joint = 0.5 * qd.cwiseProduct(momentum);
    out.kinetic = out.kinetic_per_joint.sum();

    Eigen::Vector3d weighted_com = Eigen::Vector3d::Zero();
    for (std::size_t i = 0; i < GP8_DOF; ++i) {
        const double v = -bodies[i].mass * g.dot(jac.p_com[i]);
        out.potential_per_link(static_cast<Eigen::Index>(i)) = v;
        out.potential += v;
        out.total_mass += bodies[i].mass;
        weighted_com += bodies[i].mass * jac.p_com[i];
    }
    out.centre_of_mass = weighted_com / out.total_mass;
    out.total = out.kinetic + out.potential;
    return out;
}

}  // namespace dyn

// ---------------------------------------------------------------------------
// Self-description
// ---------------------------------------------------------------------------

namespace {

[[nodiscard]] Vector6 home_pose() noexcept {
    Vector6 q;
    q << 0.0, 0.3, -0.4, 0.0, 0.6, 0.0;
    return q;
}

[[nodiscard]] ParamSpec param_q() {
    return ParamSpec::vec6("q", "Joint vector S, L, U, R, B, T", "rad", -widest_joint_range(),
                           widest_joint_range(), home_pose());
}

[[nodiscard]] ParamSpec param_qdot(double default_value) {
    Vector6 qd = Vector6::Constant(default_value);
    return ParamSpec::vec6("qdot", "Joint velocities", "rad/s", -kVelocityGuard, kVelocityGuard,
                           qd);
}

[[nodiscard]] ParamSpec param_payload() {
    return ParamSpec::scalar("payload", "Payload mass at the flange", "kg", 0.0, kMaxPayload, 0.0);
}

[[nodiscard]] ParamSpec param_friction() {
    return ParamSpec::boolean("include_friction", "Include viscous and Coulomb friction", true);
}

}  // namespace

ModuleDescription DynamicsModule::describe() const {
    ModuleDescription d;
    d.name = "dynamics";
    d.title = "Rigid-Body Dynamics: Lagrangian and Newton-Euler";
    d.course = CourseRef{3883, "M-407-01", "Robotics Modelling"};
    d.topics = {"Block 4 - Energy methods, Lagrangian formulation, Newton-Euler recursion"};
    d.source = "cpp_solver/include/study/dynamics.hpp";
    d.summary =
        "The equations of motion of the GP8 in standard form, M(q) qdd + C(q,qd) qd + g(q) + "
        "f(qd) = tau, computed twice by two independent methods that are then shown to agree.";

    {
        OpSpec op;
        op.name = "mass_matrix";
        op.title = "Inertia matrix M(q)";
        op.formula =
            "M(q) = \\sum_{i=1}^{6} \\left( m_i J_{v_i}^T J_{v_i} + J_{\\omega_i}^T R_i I_i R_i^T "
            "J_{\\omega_i} \\right) + \\operatorname{diag}(n_i^2 J_{r_i})";
        op.explain =
            "The inertia matrix says how hard each joint is to accelerate and how strongly the "
            "joints are coupled: the off-diagonal entry M_ij is the torque joint i feels when "
            "joint j accelerates. It is built from the link Jacobians, so it changes with the "
            "pose but never with the speed. Two things are worth watching. The gearbox term is "
            "shown separately: each motor's rotor inertia is multiplied by the square of its gear "
            "ratio, and on the wrist axes that reflected inertia dominates the link itself, which "
            "is why a geared robot feels so much stiffer than its arm weighs. The condition "
            "number is the ratio of the largest to the smallest eigenvalue; a large value means "
            "one direction of joint space is far heavier than another, and a computed-torque "
            "controller will struggle to use one gain for both.";
        op.params = {param_q(), param_payload()};
        op.outputs = {
            OutputSpec::make("M", "matrix", "Inertia matrix including the rotors", "kg m^2"),
            OutputSpec::make("M_rigid", "matrix", "Links only, gearboxes excluded", "kg m^2"),
            OutputSpec::make("rotor_inertia", "vec6", "n^2 J_rotor added to the diagonal",
                             "kg m^2"),
            OutputSpec::make("rotor_share", "table", "How much of M_ii the gearbox contributes"),
            OutputSpec::make("eigenvalues", "vec6", "Eigenvalues of M, ascending", "kg m^2"),
            OutputSpec::make("condition_number", "scalar", "lambda_max / lambda_min of M"),
            OutputSpec::make("symmetry_error", "scalar", "|| M - M^T ||_F, must stay at zero"),
            OutputSpec::make("positive_definite", "bool", "True when every eigenvalue is positive"),
            OutputSpec::make("note", "text", "What this pose's conditioning means"),
        };
        d.ops.push_back(std::move(op));
    }
    {
        OpSpec op;
        op.name = "gravity_torque";
        op.title = "Gravity term g(q)";
        op.formula =
            "g(q) = \\frac{\\partial V}{\\partial q} = -\\sum_{i=1}^{6} m_i J_{v_i}^T(q) \\, "
            "\\mathbf{g}, \\qquad V(q) = -\\sum_i m_i \\mathbf{g}^T p_{c_i}(q)";
        op.explain =
            "This is the torque each joint must hold just to stop the arm falling, with nothing "
            "moving at all. It is the gradient of the potential energy, so it depends only on the "
            "pose and on the payload. Joints 2 and 3 (L and U) carry almost all of it, because "
            "they are the ones whose axes are horizontal and therefore have the whole arm hanging "
            "off a lever; joints 1 and 6 see nothing, because their axes are parallel to gravity "
            "in this pose. The fraction of each axis's rated torque is shown so the limit is a "
            "number and not a feeling: a static hold that already eats 60 percent of the limit "
            "has very little left for acceleration.";
        op.params = {param_q(), param_payload()};
        op.outputs = {
            OutputSpec::make("tau_gravity", "vec6", "Holding torque per joint", "N m"),
            OutputSpec::make("per_joint", "table", "Torque, rated limit and the fraction used"),
            OutputSpec::make("max_fraction", "scalar", "Worst joint's share of its rated torque"),
            OutputSpec::make("binding_joint", "text", "The axis closest to its torque limit"),
            OutputSpec::make("within_limits", "bool", "True when no axis exceeds its rating"),
            OutputSpec::make("payload_torque", "vec6", "The payload's share of the torque", "N m"),
            OutputSpec::make("centre_of_mass", "vec3", "Whole-arm centre of mass", "m"),
            OutputSpec::make("potential_energy", "scalar", "V(q), datum at the base", "J"),
        };
        d.ops.push_back(std::move(op));
    }
    {
        OpSpec op;
        op.name = "coriolis_torque";
        op.title = "Coriolis and centrifugal term C(q,qd) qd";
        op.formula =
            "C_{ij}(q,\\dot q) = \\sum_{k=1}^{6} \\frac{1}{2} \\left( \\frac{\\partial M_{ij}}"
            "{\\partial q_k} + \\frac{\\partial M_{ik}}{\\partial q_j} - \\frac{\\partial M_{jk}}"
            "{\\partial q_i} \\right) \\dot q_k";
        op.explain =
            "These are the velocity-dependent coupling torques: the centrifugal ones, which go "
            "with the square of a single joint's speed, and the Coriolis ones, which go with the "
            "product of two. They are quadratic in velocity, so doubling the whole motion "
            "quadruples this term while gravity does not move at all - that contrast is the "
            "exam question. The Christoffel symbols come from analytic derivatives of M, not from "
            "finite differences. The passivity residual qd^T (Mdot - 2C) qd is identically zero "
            "for a correct C, so it is reported as a running proof rather than a claim.";
        op.params = {param_q(), param_qdot(0.5), param_payload()};
        op.outputs = {
            OutputSpec::make("C", "matrix", "Coriolis matrix C(q,qd)", "kg m^2/s"),
            OutputSpec::make("tau_coriolis", "vec6", "C(q,qd) qd", "N m"),
            OutputSpec::make("per_joint", "table", "Coriolis torque against the gravity torque"),
            OutputSpec::make("Mdot", "matrix", "dM/dt along this velocity", "kg m^2/s"),
            OutputSpec::make("passivity_residual", "scalar", "qd^T (Mdot - 2C) qd, exactly zero"),
            OutputSpec::make("quadratic_check", "scalar",
                             "|| C(q,2qd) 2qd - 4 C(q,qd) qd ||, zero because the term is "
                             "quadratic in speed"),
            OutputSpec::make("speed_norm", "scalar", "|| qd ||", "rad/s"),
        };
        d.ops.push_back(std::move(op));
    }
    {
        OpSpec op;
        op.name = "inverse_dynamics";
        op.title = "Inverse dynamics, Lagrangian against Newton-Euler";
        op.formula =
            "\\tau = M(q)\\ddot q + C(q,\\dot q)\\dot q + g(q) + f(\\dot q), \\qquad f_i = b_i "
            "\\dot q_i + \\tau_{c,i} \\operatorname{sign}(\\dot q_i)";
        op.explain =
            "Torques out of a motion in: this is what a computed-torque controller evaluates every "
            "millisecond. Both formulations are implemented in full and independently. The "
            "Lagrangian one assembles M, C and g as matrices and multiplies them out; the "
            "Newton-Euler one runs an outward pass for the velocities and accelerations of every "
            "link and an inward pass for the forces and moments, never forming a matrix at all. "
            "They return the same torques to within floating-point noise, and the measured "
            "microseconds of each show why the recursion - O(n) against the closed form's "
            "O(n^2) assembly - is the one that goes into real-time code.";
        op.params = {
            param_q(),
            param_qdot(0.5),
            ParamSpec::vec6("qddot", "Joint accelerations", "rad/s^2", -kAccelGuard, kAccelGuard,
                            Vector6::Constant(1.0)),
            param_payload(),
            ParamSpec::enumeration("formulation", "Which formulation to report",
                                   {"lagrangian", "newton_euler"}, "lagrangian"),
            param_friction(),
        };
        op.outputs = {
            OutputSpec::make("tau", "vec6", "Joint torque from the selected formulation", "N m"),
            OutputSpec::make("tau_lagrangian", "vec6", "Closed-form result", "N m"),
            OutputSpec::make("tau_newton_euler", "vec6", "Recursive result", "N m"),
            OutputSpec::make("max_disagreement", "scalar", "max |tau_L - tau_NE|", "N m"),
            OutputSpec::make("terms", "table", "Inertial, Coriolis, gravity and friction per joint"),
            OutputSpec::make("us_lagrangian", "scalar", "Mean time of one closed-form call", "us"),
            OutputSpec::make("us_newton_euler", "scalar", "Mean time of one recursive call", "us"),
            OutputSpec::make("speedup", "scalar", "Closed-form time divided by recursive time"),
            OutputSpec::make("torque_limits", "table", "Torque against each axis's rated limit"),
            OutputSpec::make("within_limits", "bool", "True when no axis exceeds its rating"),
            OutputSpec::make("note", "text", "What the comparison shows at this state"),
        };
        d.ops.push_back(std::move(op));
    }
    {
        OpSpec op;
        op.name = "forward_dynamics";
        op.title = "Forward dynamics: torques in, accelerations out";
        op.formula =
            "\\ddot q = M(q)^{-1} \\left( \\tau - C(q,\\dot q)\\dot q - g(q) - f(\\dot q) "
            "\\right)";
        op.explain =
            "The simulation direction: given what the motors are doing, what does the arm do? "
            "M(q) is symmetric and positive definite for every reachable pose, so the linear "
            "system is solved by an LDLT factorisation rather than by forming an inverse - "
            "cheaper, and numerically much better behaved. The residual || M qdd - rhs || is "
            "returned so the solve is auditable; it sits at the level of double-precision "
            "rounding. Feed the torque that inverse_dynamics produced for a given acceleration "
            "back in here and the acceleration comes out again, which is the round trip the test "
            "suite asserts to 1e-8.";
        op.params = {
            param_q(),
            param_qdot(0.5),
            ParamSpec::vec6("tau", "Applied joint torques", "N m", -kTorqueGuard, kTorqueGuard,
                            Vector6::Zero()),
            param_payload(),
            param_friction(),
        };
        op.outputs = {
            OutputSpec::make("qddot", "vec6", "Resulting joint accelerations", "rad/s^2"),
            OutputSpec::make("solve_method", "text", "The factorisation that was used"),
            OutputSpec::make("residual", "scalar", "|| M qdd - (tau - C qd - g - f) ||", "N m"),
            OutputSpec::make("condition_number", "scalar", "lambda_max / lambda_min of M"),
            OutputSpec::make("positive_definite", "bool", "True when M is positive definite"),
            OutputSpec::make("per_joint", "table", "Torque budget and the acceleration it buys"),
            OutputSpec::make("roundtrip_error", "scalar",
                             "max |tau - inverse_dynamics(qdd)|, the inversion check", "N m"),
        };
        d.ops.push_back(std::move(op));
    }
    {
        OpSpec op;
        op.name = "energy";
        op.title = "Kinetic and potential energy";
        op.formula =
            "T = \\tfrac{1}{2} \\dot q^T M(q) \\dot q, \\qquad V = -\\sum_i m_i \\mathbf{g}^T "
            "p_{c_i}(q), \\qquad \\mathcal{L} = T - V";
        op.explain =
            "The energy method is where the Lagrangian formulation starts: write down T and V, "
            "and the equations of motion follow by differentiation. Here they are evaluated "
            "directly so the bookkeeping is visible - the kinetic energy split per joint (the "
            "shares sum to the total because T is a quadratic form in qd), the potential energy "
            "per link against the base-frame datum, and the whole-arm centre of mass with its "
            "height. With no torque and no friction, T + V is constant, so integrating the "
            "torque-free arm and watching the total is a direct test of the dynamics: that is "
            "exactly what the test suite does over a 0.5 s fall.";
        op.params = {param_q(), param_qdot(1.0), param_payload()};
        op.outputs = {
            OutputSpec::make("kinetic", "scalar", "T", "J"),
            OutputSpec::make("potential", "scalar", "V, datum at the base frame", "J"),
            OutputSpec::make("total", "scalar", "T + V", "J"),
            OutputSpec::make("lagrangian", "scalar", "L = T - V", "J"),
            OutputSpec::make("kinetic_per_joint", "vec6", "Per-joint share of T", "J"),
            OutputSpec::make("potential_per_link", "vec6", "Per-link share of V", "J"),
            OutputSpec::make("per_link", "table", "Mass, centre-of-mass height and energy share"),
            OutputSpec::make("centre_of_mass", "vec3", "Whole-arm centre of mass", "m"),
            OutputSpec::make("com_height", "scalar", "Height of that centre of mass", "m"),
            OutputSpec::make("total_mass", "scalar", "Moving mass, payload included", "kg"),
        };
        d.ops.push_back(std::move(op));
    }

    return d;
}

// ---------------------------------------------------------------------------
// Ops
// ---------------------------------------------------------------------------

namespace {

struct State {
    Vector6 q{Vector6::Zero()};
    Vector6 qd{Vector6::Zero()};
    Vector6 qdd{Vector6::Zero()};
    double payload = 0.0;
};

[[nodiscard]] Vector6 read_joints(const json::Value& args) {
    return require_vec6(args, "q", -widest_joint_range(), widest_joint_range());
}

[[nodiscard]] double read_payload(const json::Value& args) {
    return optional_scalar(args, "payload", 0.0, 0.0, kMaxPayload);
}

[[nodiscard]] json::Value op_mass_matrix(const json::Value& args) {
    const Vector6 q = read_joints(args);
    const double payload = read_payload(args);

    const Matrix6 M_rigid = dyn::rigid_mass_matrix(q, payload);
    const Matrix6 M = dyn::mass_matrix(q, payload);
    const Vector6 rotor = dyn::rotor_inertia_diagonal();
    const Vector6 eigenvalues = dyn::symmetric_eigenvalues(M);
    const double lambda_min = eigenvalues.minCoeff();
    const double lambda_max = eigenvalues.maxCoeff();
    const double condition = (lambda_min > 0.0) ? lambda_max / lambda_min
                                                : std::numeric_limits<double>::infinity();

    std::vector<std::vector<json::Value>> rows;
    rows.reserve(GP8_DOF);
    for (std::size_t i = 0; i < GP8_DOF; ++i) {
        const auto index = static_cast<Eigen::Index>(i);
        const double link_part = M_rigid(index, index);
        const double total = M(index, index);
        rows.push_back({
            json::Value(GP8_AXIS_NAMES[i]),
            json::Value(link_part),
            json::Value(rotor(index)),
            json::Value(total),
            json::Value(rotor(index) / total),
            json::Value(GP8_LINKS[i].gear_ratio),
        });
    }

    const double asymmetry = (M - M.transpose()).norm();
    std::string note;
    if (condition > 1.0e4) {
        note =
            "Badly conditioned pose: the heaviest direction of joint space is more than ten "
            "thousand times heavier than the lightest, so one controller gain cannot suit both.";
    } else if (condition > 1.0e3) {
        note =
            "Moderately conditioned: a factor of a thousand between the heaviest and lightest "
            "directions, which is normal for a geared arm because the wrist rotors dominate "
            "their own axes.";
    } else {
        note = "Well conditioned pose: the eigenvalues of M are within three orders of magnitude.";
    }

    json::Value out = json::Value::object();
    out.set("M", json::from_matrix(M));
    out.set("M_rigid", json::from_matrix(M_rigid));
    out.set("rotor_inertia", json::from_vec6(rotor));
    out.set("rotor_share",
            json::from_table({"axis", "link_kgm2", "rotor_kgm2", "total_kgm2", "rotor_fraction",
                              "gear_ratio"},
                             rows));
    out.set("eigenvalues", json::from_vec6(eigenvalues));
    out.set("condition_number", json::Value(condition));
    out.set("symmetry_error", json::Value(asymmetry));
    out.set("positive_definite", json::Value(lambda_min > 0.0));
    out.set("note", json::Value(note));
    return out;
}

[[nodiscard]] json::Value op_gravity_torque(const json::Value& args) {
    const Vector6 q = read_joints(args);
    const double payload = read_payload(args);

    const Vector6 tau = dyn::gravity_torque(q, payload);
    const Vector6 tau_unloaded = dyn::gravity_torque(q, 0.0);
    const Vector6 payload_share = tau - tau_unloaded;
    const dyn::EnergyTerms terms = dyn::energy(q, Vector6::Zero(), payload);

    std::vector<std::vector<json::Value>> rows;
    rows.reserve(GP8_DOF);
    double worst_fraction = 0.0;
    std::size_t worst_joint = 0;
    bool within_limits = true;
    for (std::size_t i = 0; i < GP8_DOF; ++i) {
        const auto index = static_cast<Eigen::Index>(i);
        const double limit = GP8_LINKS[i].max_torque;
        const double fraction = std::abs(tau(index)) / limit;
        if (fraction > worst_fraction) {
            worst_fraction = fraction;
            worst_joint = i;
        }
        within_limits = within_limits && (fraction <= 1.0);
        rows.push_back({
            json::Value(GP8_AXIS_NAMES[i]),
            json::Value(tau(index)),
            json::Value(payload_share(index)),
            json::Value(limit),
            json::Value(fraction),
            json::Value(fraction <= 1.0),
        });
    }

    json::Value out = json::Value::object();
    out.set("tau_gravity", json::from_vec6(tau));
    out.set("per_joint", json::from_table({"axis", "torque_Nm", "payload_Nm", "limit_Nm",
                                           "fraction_of_limit", "ok"},
                                          rows));
    out.set("max_fraction", json::Value(worst_fraction));
    out.set("binding_joint",
            json::Value(std::string("axis ") + GP8_AXIS_NAMES[worst_joint] + " (joint " +
                        std::to_string(worst_joint + 1) + ") carries the largest share of its " +
                        "rated torque at this pose"));
    out.set("within_limits", json::Value(within_limits));
    out.set("payload_torque", json::from_vec6(payload_share));
    out.set("centre_of_mass", json::from_vec3(terms.centre_of_mass));
    out.set("potential_energy", json::Value(terms.potential));
    return out;
}

[[nodiscard]] json::Value op_coriolis_torque(const json::Value& args) {
    const Vector6 q = read_joints(args);
    const Vector6 qd = require_vec6(args, "qdot", -kVelocityGuard, kVelocityGuard);
    const double payload = read_payload(args);

    const Matrix6 C = dyn::coriolis_matrix(q, qd, payload);
    const Matrix6 Mdot = dyn::mass_matrix_rate(q, qd, payload);
    const Vector6 tau = C * qd;
    const Vector6 tau_gravity = dyn::gravity_torque(q, payload);

    const Matrix6 C_double = dyn::coriolis_matrix(q, 2.0 * qd, payload);
    const double quadratic_check = (C_double * (2.0 * qd) - 4.0 * tau).norm();
    const double passivity = qd.dot((Mdot - 2.0 * C) * qd);

    std::vector<std::vector<json::Value>> rows;
    rows.reserve(GP8_DOF);
    for (std::size_t i = 0; i < GP8_DOF; ++i) {
        const auto index = static_cast<Eigen::Index>(i);
        rows.push_back({
            json::Value(GP8_AXIS_NAMES[i]),
            json::Value(qd(index)),
            json::Value(tau(index)),
            json::Value(tau_gravity(index)),
            json::Value(std::abs(tau_gravity(index)) > 1e-12
                            ? std::abs(tau(index)) / std::abs(tau_gravity(index))
                            : 0.0),
        });
    }

    json::Value out = json::Value::object();
    out.set("C", json::from_matrix(C));
    out.set("tau_coriolis", json::from_vec6(tau));
    out.set("per_joint",
            json::from_table({"axis", "qdot_rad_s", "coriolis_Nm", "gravity_Nm",
                              "coriolis_over_gravity"},
                             rows));
    out.set("Mdot", json::from_matrix(Mdot));
    out.set("passivity_residual", json::Value(passivity));
    out.set("quadratic_check", json::Value(quadratic_check));
    out.set("speed_norm", json::Value(qd.norm()));
    return out;
}

[[nodiscard]] json::Value op_inverse_dynamics(const json::Value& args) {
    State s;
    s.q = read_joints(args);
    s.qd = require_vec6(args, "qdot", -kVelocityGuard, kVelocityGuard);
    s.qdd = require_vec6(args, "qddot", -kAccelGuard, kAccelGuard);
    s.payload = read_payload(args);
    const std::string formulation =
        optional_enum(args, "formulation", "lagrangian", {"lagrangian", "newton_euler"});
    const bool friction = optional_bool(args, "include_friction", true);

    const Vector6 tau_lagrangian =
        dyn::inverse_dynamics_lagrangian(s.q, s.qd, s.qdd, s.payload, friction);
    const Vector6 tau_newton =
        dyn::inverse_dynamics_newton_euler(s.q, s.qd, s.qdd, s.payload, friction);

    // Timing: the same call repeated, so one clock tick does not become the
    // whole measurement. The accumulator stops the optimiser removing the call.
    double sink = 0.0;
    const auto clock_start_l = std::chrono::steady_clock::now();
    for (int rep = 0; rep < kTimingRepetitions; ++rep) {
        sink += dyn::inverse_dynamics_lagrangian(s.q, s.qd, s.qdd, s.payload, friction)(0);
    }
    const auto clock_mid = std::chrono::steady_clock::now();
    for (int rep = 0; rep < kTimingRepetitions; ++rep) {
        sink += dyn::inverse_dynamics_newton_euler(s.q, s.qd, s.qdd, s.payload, friction)(0);
    }
    const auto clock_end = std::chrono::steady_clock::now();
    const double reps = static_cast<double>(kTimingRepetitions);
    const double us_lagrangian =
        std::chrono::duration<double, std::micro>(clock_mid - clock_start_l).count() / reps;
    const double us_newton =
        std::chrono::duration<double, std::micro>(clock_end - clock_mid).count() / reps;

    const Matrix6 M = dyn::mass_matrix(s.q, s.payload);
    const Matrix6 C = dyn::coriolis_matrix(s.q, s.qd, s.payload);
    const Vector6 inertial = M * s.qdd;
    const Vector6 coriolis = C * s.qd;
    const Vector6 gravity = dyn::gravity_torque(s.q, s.payload);
    const Vector6 friction_term = friction ? dyn::friction_torque(s.qd) : Vector6::Zero();

    std::vector<std::vector<json::Value>> term_rows;
    term_rows.reserve(GP8_DOF);
    std::vector<std::vector<json::Value>> limit_rows;
    limit_rows.reserve(GP8_DOF);
    bool within_limits = true;
    const Vector6& tau = (formulation == "lagrangian") ? tau_lagrangian : tau_newton;
    for (std::size_t i = 0; i < GP8_DOF; ++i) {
        const auto index = static_cast<Eigen::Index>(i);
        term_rows.push_back({
            json::Value(GP8_AXIS_NAMES[i]),
            json::Value(inertial(index)),
            json::Value(coriolis(index)),
            json::Value(gravity(index)),
            json::Value(friction_term(index)),
            json::Value(tau(index)),
        });
        const double limit = GP8_LINKS[i].max_torque;
        const bool ok = std::abs(tau(index)) <= limit;
        within_limits = within_limits && ok;
        limit_rows.push_back({
            json::Value(GP8_AXIS_NAMES[i]),
            json::Value(tau(index)),
            json::Value(limit),
            json::Value(std::abs(tau(index)) / limit),
            json::Value(ok),
        });
    }

    const double disagreement = (tau_lagrangian - tau_newton).cwiseAbs().maxCoeff();
    std::string note =
        "The two formulations agree to " + json::number_to_string(disagreement) +
        " N m, which is floating-point noise on torques of this size, and they share no code "
        "beyond the rigid-body data. ";
    if (us_newton < us_lagrangian) {
        note += "The recursion is the faster of the two here, by a factor of " +
                json::number_to_string(us_lagrangian / us_newton) +
                ": it never assembles M or C, which is why real-time computed torque is written "
                "that way.";
    } else {
        note +=
            "On this machine the closed form measured no slower than the recursion at six "
            "degrees of freedom; the recursion's O(n) advantage shows as the chain gets longer, "
            "and the measured microseconds are reported rather than assumed.";
    }
    if (sink == std::numeric_limits<double>::infinity()) {
        note += " ";  // the accumulator is only here to keep the timing loops alive
    }

    json::Value out = json::Value::object();
    out.set("tau", json::from_vec6(tau));
    out.set("tau_lagrangian", json::from_vec6(tau_lagrangian));
    out.set("tau_newton_euler", json::from_vec6(tau_newton));
    out.set("max_disagreement", json::Value(disagreement));
    out.set("terms", json::from_table({"axis", "inertial_Nm", "coriolis_Nm", "gravity_Nm",
                                       "friction_Nm", "total_Nm"},
                                      term_rows));
    out.set("us_lagrangian", json::Value(us_lagrangian));
    out.set("us_newton_euler", json::Value(us_newton));
    out.set("speedup", json::Value(us_lagrangian / us_newton));
    out.set("torque_limits",
            json::from_table({"axis", "torque_Nm", "limit_Nm", "fraction_of_limit", "ok"},
                             limit_rows));
    out.set("within_limits", json::Value(within_limits));
    out.set("formulation", json::Value(formulation));
    out.set("note", json::Value(note));
    return out;
}

[[nodiscard]] json::Value op_forward_dynamics(const json::Value& args) {
    const Vector6 q = read_joints(args);
    const Vector6 qd = require_vec6(args, "qdot", -kVelocityGuard, kVelocityGuard);
    const Vector6 tau = require_vec6(args, "tau", -kTorqueGuard, kTorqueGuard);
    const double payload = read_payload(args);
    const bool friction = optional_bool(args, "include_friction", true);

    const dyn::ForwardResult result = dyn::forward_dynamics(q, qd, tau, payload, friction);
    const Vector6 tau_back =
        dyn::inverse_dynamics_lagrangian(q, qd, result.qddot, payload, friction);
    const double roundtrip = (tau - tau_back).cwiseAbs().maxCoeff();

    const Matrix6 M = dyn::mass_matrix(q, payload);
    std::vector<std::vector<json::Value>> rows;
    rows.reserve(GP8_DOF);
    for (std::size_t i = 0; i < GP8_DOF; ++i) {
        const auto index = static_cast<Eigen::Index>(i);
        rows.push_back({
            json::Value(GP8_AXIS_NAMES[i]),
            json::Value(tau(index)),
            json::Value(result.qddot(index)),
            json::Value(M(index, index)),
            json::Value(GP8_LINKS[i].max_torque),
            json::Value(std::abs(tau(index)) / GP8_LINKS[i].max_torque),
        });
    }

    json::Value out = json::Value::object();
    out.set("qddot", json::from_vec6(result.qddot));
    out.set("solve_method",
            json::Value("LDLT (Cholesky with a diagonal) of the symmetric positive-definite M(q); "
                        "no explicit inverse is ever formed"));
    out.set("residual", json::Value(result.residual));
    out.set("condition_number", json::Value(result.condition_number));
    out.set("positive_definite", json::Value(result.positive_definite));
    out.set("per_joint", json::from_table({"axis", "tau_Nm", "qddot_rad_s2", "M_ii_kgm2",
                                           "limit_Nm", "fraction_of_limit"},
                                          rows));
    out.set("roundtrip_error", json::Value(roundtrip));
    return out;
}

[[nodiscard]] json::Value op_energy(const json::Value& args) {
    const Vector6 q = read_joints(args);
    const Vector6 qd = require_vec6(args, "qdot", -kVelocityGuard, kVelocityGuard);
    const double payload = read_payload(args);

    const dyn::EnergyTerms terms = dyn::energy(q, qd, payload);
    const dyn::BodyArray bodies = dyn::bodies_with_payload(payload);
    const auto frames = link_frames(q);

    std::vector<std::vector<json::Value>> rows;
    rows.reserve(GP8_DOF);
    for (std::size_t i = 0; i < GP8_DOF; ++i) {
        const auto index = static_cast<Eigen::Index>(i);
        const Eigen::Vector3d p_com = frames[i] * bodies[i].com;
        rows.push_back({
            json::Value(GP8_AXIS_NAMES[i]),
            json::Value(bodies[i].mass),
            json::Value(p_com.z()),
            json::Value(terms.potential_per_link(index)),
            json::Value(terms.kinetic_per_joint(index)),
        });
    }

    json::Value out = json::Value::object();
    out.set("kinetic", json::Value(terms.kinetic));
    out.set("potential", json::Value(terms.potential));
    out.set("total", json::Value(terms.total));
    out.set("lagrangian", json::Value(terms.kinetic - terms.potential));
    out.set("kinetic_per_joint", json::from_vec6(terms.kinetic_per_joint));
    out.set("potential_per_link", json::from_vec6(terms.potential_per_link));
    out.set("per_link",
            json::from_table({"axis", "mass_kg", "com_height_m", "potential_J", "kinetic_J"},
                             rows));
    out.set("centre_of_mass", json::from_vec3(terms.centre_of_mass));
    out.set("com_height", json::Value(terms.centre_of_mass.z()));
    out.set("total_mass", json::Value(terms.total_mass));
    return out;
}

}  // namespace

json::Value DynamicsModule::invoke(std::string_view op, const json::Value& args) const {
    if (op == "mass_matrix") {
        return op_mass_matrix(args);
    }
    if (op == "gravity_torque") {
        return op_gravity_torque(args);
    }
    if (op == "coriolis_torque") {
        return op_coriolis_torque(args);
    }
    if (op == "inverse_dynamics") {
        return op_inverse_dynamics(args);
    }
    if (op == "forward_dynamics") {
        return op_forward_dynamics(args);
    }
    if (op == "energy") {
        return op_energy(args);
    }
    unknown_op(name(), op);
}

}  // namespace yaskawa::study
