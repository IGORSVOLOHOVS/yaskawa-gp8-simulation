#include "study/jacobian_statics.hpp"

#include "study/dh_kinematics.hpp"

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <limits>
#include <numbers>
#include <string>
#include <utility>
#include <vector>

namespace yaskawa::study {

namespace {

constexpr double kPi = std::numbers::pi;

// Rates and wrenches are not angles, so these guards only reject nonsense (a
// UI sending a pixel coordinate), never a student probing a large value.
constexpr double kRateGuard = 1000.0;    // [rad/s]
constexpr double kWrenchGuard = 10000.0; // [N] and [N m]
constexpr double kTorqueGuard = 10000.0; // [N m]

// Below this singular value the matrix is treated as rank deficient. It is
// about 1e-9 of the largest singular value the GP8 ever shows, which is of
// order 1, so it is a numerical zero and not a physical threshold.
constexpr double kRankTolerance = 1e-9;

[[nodiscard]] jac::JointVector read_q(const json::Value& args) {
    return require_vec6(args, "q", -widest_joint_range(), widest_joint_range());
}

[[nodiscard]] std::size_t matrix_rank(const Eigen::Matrix<double, 6, 1>& sv) noexcept {
    std::size_t rank = 0;
    for (Eigen::Index i = 0; i < 6; ++i) {
        if (sv(i) > kRankTolerance) {
            ++rank;
        }
    }
    return rank;
}

[[nodiscard]] json::Value joint_rate_table(const jac::JointVector& qdot, bool& any_exceeded) {
    std::vector<std::vector<json::Value>> rows;
    rows.reserve(GP8_DOF);
    any_exceeded = false;
    for (std::size_t i = 0; i < GP8_DOF; ++i) {
        const double rate = qdot(static_cast<Eigen::Index>(i));
        const double limit = joint_max_velocity(i);
        const bool exceeded = std::abs(rate) > limit;
        any_exceeded = any_exceeded || exceeded;
        rows.push_back({
            json::Value(GP8_AXIS_NAMES[i]),
            json::Value(rate),
            json::Value(limit),
            json::Value(limit > 0.0 ? std::abs(rate) / limit : 0.0),
            json::Value(exceeded),
        });
    }
    return json::from_table({"axis", "qdot_rad_s", "max_vel_rad_s", "fraction_of_limit",
                             "exceeds_limit"},
                            rows);
}

// ---------------------------------------------------------------------------
// Ops
// ---------------------------------------------------------------------------

[[nodiscard]] json::Value op_geometric_jacobian(const json::Value& args) {
    const jac::JointVector q = read_q(args);
    const jac::Jacobian J = jac::geometric_jacobian(q);
    const jac::Jacobian J_numeric = jac::numeric_jacobian(q);
    const Eigen::Matrix<double, 6, 1> sv = jac::singular_values(J);

    const std::array<Eigen::Isometry3d, GP8_DOF> frames = link_frames(q);
    const Eigen::Vector3d p_e = frames[GP8_DOF - 1].translation();

    std::vector<std::vector<json::Value>> rows;
    rows.reserve(GP8_DOF);
    Eigen::Vector3d z_prev(0.0, 0.0, 1.0);
    Eigen::Vector3d o_prev = Eigen::Vector3d::Zero();
    for (std::size_t i = 0; i < GP8_DOF; ++i) {
        rows.push_back({
            json::Value(GP8_AXIS_NAMES[i]),
            json::Value(z_prev.x()),
            json::Value(z_prev.y()),
            json::Value(z_prev.z()),
            json::Value((p_e - o_prev).norm()),
            json::Value(J.block<3, 1>(0, static_cast<Eigen::Index>(i)).norm()),
            json::Value(J.block<3, 1>(3, static_cast<Eigen::Index>(i)).norm()),
        });
        z_prev = frames[i].linear().col(2);
        o_prev = frames[i].translation();
    }

    const yaskawa::YaskawaKinematics engine;
    const Eigen::Matrix<double, 6, 6> J_engine = engine.computeJacobian(q);

    json::Value out = json::Value::object();
    out.set("J", json::from_matrix(J));
    out.set("J_linear", json::from_matrix(J.block<3, 6>(0, 0)));
    out.set("J_angular", json::from_matrix(J.block<3, 6>(3, 0)));
    out.set("columns",
            json::from_table({"axis", "z_x", "z_y", "z_z", "moment_arm_m", "linear_gain",
                              "angular_gain"},
                             rows));
    out.set("determinant", json::Value(J.determinant()));
    out.set("rank", json::Value(static_cast<int>(matrix_rank(sv))));
    out.set("singular_values", json::from_vector(sv));
    out.set("numeric_difference_error", json::Value((J - J_numeric).norm()));
    out.set("engine_difference_error", json::Value((J - J_engine).norm()));
    out.set("tool_position", json::from_vec3(p_e));
    out.set("verdict",
            json::Value(std::string(
                "The top three rows turn joint rates into tool translation, the bottom three "
                "into tool rotation. numeric_difference_error is the analytic matrix against a "
                "central difference of the forward kinematics; engine_difference_error is the "
                "same matrix against the forward-difference Jacobian in yaskawa_kinematics.cpp, "
                "which is only first-order accurate and so disagrees around 1e-6.")));
    return out;
}

[[nodiscard]] json::Value op_velocity_propagation(const json::Value& args) {
    const jac::JointVector q = read_q(args);
    const jac::JointVector qdot = require_vec6(args, "qdot", -kRateGuard, kRateGuard);

    const jac::Jacobian J = jac::geometric_jacobian(q);
    const jac::Twist twist = J * qdot;

    // The link-by-link recursion of the lecture, run alongside the matrix
    // product: omega_i = omega_{i-1} + z_{i-1} qdot_i,
    //          v_i     = v_{i-1} + omega_i x (o_i - o_{i-1}).
    const std::array<Eigen::Isometry3d, GP8_DOF> frames = link_frames(q);
    Eigen::Vector3d omega = Eigen::Vector3d::Zero();
    Eigen::Vector3d v = Eigen::Vector3d::Zero();
    Eigen::Vector3d z_prev(0.0, 0.0, 1.0);
    Eigen::Vector3d o_prev = Eigen::Vector3d::Zero();

    std::vector<std::vector<json::Value>> recursion_rows;
    recursion_rows.reserve(GP8_DOF);
    for (std::size_t i = 0; i < GP8_DOF; ++i) {
        const Eigen::Vector3d o_i = frames[i].translation();
        omega += z_prev * qdot(static_cast<Eigen::Index>(i));
        v += omega.cross(o_i - o_prev);
        recursion_rows.push_back({
            json::Value(GP8_AXIS_NAMES[i]),
            json::Value(v.x()),
            json::Value(v.y()),
            json::Value(v.z()),
            json::Value(omega.norm()),
            json::Value(v.norm()),
        });
        z_prev = frames[i].linear().col(2);
        o_prev = o_i;
    }

    jac::Twist propagated;
    propagated.head<3>() = v;
    propagated.tail<3>() = omega;

    // Per-joint contribution: column i scaled by that joint's rate.
    std::vector<std::vector<json::Value>> share_rows;
    share_rows.reserve(GP8_DOF);
    const double total_linear = twist.head<3>().norm();
    std::size_t dominant = 0;
    double dominant_share = -1.0;
    for (std::size_t i = 0; i < GP8_DOF; ++i) {
        const Eigen::Index col = static_cast<Eigen::Index>(i);
        const double rate = qdot(col);
        const Eigen::Vector3d linear = J.block<3, 1>(0, col) * rate;
        const Eigen::Vector3d angular = J.block<3, 1>(3, col) * rate;
        const double linear_norm = linear.norm();
        if (linear_norm > dominant_share) {
            dominant_share = linear_norm;
            dominant = i;
        }
        share_rows.push_back({
            json::Value(GP8_AXIS_NAMES[i]),
            json::Value(rate),
            json::Value(linear.x()),
            json::Value(linear.y()),
            json::Value(linear.z()),
            json::Value(linear_norm),
            json::Value(angular.norm()),
            json::Value(total_linear > 0.0 ? linear_norm / total_linear : 0.0),
        });
    }

    bool exceeded = false;
    json::Value limits = joint_rate_table(qdot, exceeded);

    std::string verdict = "Axis ";
    verdict += GP8_AXIS_NAMES[dominant];
    verdict += " dominates the tool translation at this configuration, contributing ";
    verdict += json::number_to_string(dominant_share);
    verdict += " m/s of the ";
    verdict += json::number_to_string(total_linear);
    verdict += " m/s total. ";
    verdict += exceeded ? "At least one axis is commanded past its rated speed, so the robot "
                          "would clamp this motion."
                        : "Every axis is inside its rated speed.";

    json::Value out = json::Value::object();
    out.set("twist", json::from_vec6(twist));
    out.set("linear_velocity", json::from_vec3(Eigen::Vector3d(twist.head<3>())));
    out.set("angular_velocity", json::from_vec3(Eigen::Vector3d(twist.tail<3>())));
    out.set("contributions",
            json::from_table({"axis", "qdot_rad_s", "v_x", "v_y", "v_z", "v_norm_m_s",
                              "omega_norm_rad_s", "share_of_linear"},
                             share_rows));
    out.set("recursion",
            json::from_table({"axis", "v_x", "v_y", "v_z", "omega_norm_rad_s", "v_norm_m_s"},
                             recursion_rows));
    out.set("recursion_residual", json::Value((propagated - twist).norm()));
    out.set("speed_limits", std::move(limits));
    out.set("exceeds_speed_limit", json::Value(exceeded));
    out.set("dominant_axis", json::Value(GP8_AXIS_NAMES[dominant]));
    out.set("verdict", json::Value(verdict));
    return out;
}

[[nodiscard]] json::Value op_inverse_velocity(const json::Value& args) {
    const jac::JointVector q = read_q(args);
    const jac::Twist twist = require_vec6(args, "twist", -kRateGuard, kRateGuard);
    const double lambda = optional_scalar(args, "damping", 0.01, 0.0, 10.0);

    const jac::Jacobian J = jac::geometric_jacobian(q);
    const jac::DampedSolution damped = jac::damped_least_squares(J, twist, lambda);
    const jac::DampedSolution undamped = jac::damped_least_squares(J, twist, 0.0);
    const Eigen::Matrix<double, 6, 1> sv = jac::singular_values(J);
    const double sigma_min = sv(5);
    const double sigma_max = sv(0);

    // The trade-off made explicit: the same twist solved at several damping
    // factors, so the student sees joint rate bought with tracking error.
    static constexpr std::array<double, 5> kSweep = {0.0, 0.001, 0.01, 0.05, 0.2};
    std::vector<std::vector<json::Value>> sweep_rows;
    sweep_rows.reserve(kSweep.size());
    for (const double candidate : kSweep) {
        const jac::DampedSolution s = jac::damped_least_squares(J, twist, candidate);
        sweep_rows.push_back({
            json::Value(candidate),
            json::Value(s.qdot.norm()),
            json::Value(s.residual_norm),
            json::Value(s.qdot.cwiseAbs().maxCoeff()),
        });
    }

    bool exceeded = false;
    json::Value limits = joint_rate_table(damped.qdot, exceeded);

    std::string verdict =
        "Damping trades tracking accuracy for bounded joint rates. residual_twist is exactly "
        "what the damping gave up: at lambda = 0 it is zero away from a singularity, and it "
        "grows as lambda grows. ";
    if (sigma_min <= lambda) {
        verdict +=
            "The smallest singular value is already below lambda, so this configuration is "
            "inside the damped region and the residual is unavoidable with this lambda.";
    } else {
        verdict +=
            "The smallest singular value is still above lambda, so the damping is costing "
            "accuracy without being needed for conditioning yet.";
    }
    if (exceeded) {
        verdict += " At least one axis is still commanded past its rated speed: raise lambda.";
    }

    json::Value out = json::Value::object();
    out.set("qdot", json::from_vec6(damped.qdot));
    out.set("achieved_twist", json::from_vec6(damped.achieved));
    out.set("residual_twist", json::from_vec6(damped.residual));
    out.set("residual_norm", json::Value(damped.residual_norm));
    out.set("qdot_norm", json::Value(damped.qdot.norm()));
    out.set("undamped_qdot_norm", json::Value(undamped.qdot.norm()));
    out.set("undamped_residual_norm", json::Value(undamped.residual_norm));
    out.set("damping", json::Value(lambda));
    out.set("damping_tradeoff",
            json::from_table({"lambda", "qdot_norm_rad_s", "residual_twist_norm",
                              "max_abs_qdot_rad_s"},
                             sweep_rows));
    out.set("singular_values", json::from_vector(sv));
    out.set("sigma_min", json::Value(sigma_min));
    out.set("condition_number",
            json::Value(sigma_min > kRankTolerance ? sigma_max / sigma_min
                                                   : std::numeric_limits<double>::infinity()));
    out.set("speed_limits", std::move(limits));
    out.set("exceeds_speed_limit", json::Value(exceeded));
    out.set("verdict", json::Value(verdict));
    return out;
}

[[nodiscard]] json::Value op_force_torque_duality(const json::Value& args) {
    const jac::JointVector q = read_q(args);
    const jac::Wrench wrench = require_vec6(args, "wrench", -kWrenchGuard, kWrenchGuard);
    const jac::JointVector tau_in =
        optional_vec6(args, "tau", jac::JointVector::Zero(), -kTorqueGuard, kTorqueGuard);

    const jac::Jacobian J = jac::geometric_jacobian(q);
    const jac::JointVector tau = J.transpose() * wrench;

    // The reverse direction: which tool wrench would a measured joint-torque
    // vector correspond to. J^T F = tau solved for F.
    const Eigen::ColPivHouseholderQR<jac::Jacobian> qr(J.transpose());
    const jac::Wrench wrench_from_tau = qr.solve(tau_in);
    const jac::JointVector tau_round_trip = J.transpose() * wrench_from_tau;
    const jac::Wrench wrench_round_trip = qr.solve(tau);

    std::vector<std::vector<json::Value>> rows;
    rows.reserve(GP8_DOF);
    bool overloaded = false;
    std::size_t worst = 0;
    double worst_fraction = -1.0;
    for (std::size_t i = 0; i < GP8_DOF; ++i) {
        const double value = tau(static_cast<Eigen::Index>(i));
        const double budget = GP8_LINKS[i].max_torque;
        const double fraction = budget > 0.0 ? std::abs(value) / budget : 0.0;
        const bool over = fraction > 1.0;
        overloaded = overloaded || over;
        if (fraction > worst_fraction) {
            worst_fraction = fraction;
            worst = i;
        }
        rows.push_back({
            json::Value(GP8_AXIS_NAMES[i]),
            json::Value(value),
            json::Value(budget),
            json::Value(fraction),
            json::Value(over),
        });
    }

    std::string verdict =
        "Kinematics and statics are the same matrix read in opposite directions: v = J qdot maps "
        "joint rates forward to the tool, and tau = J^T F maps a tool wrench backward to the "
        "joints. That is the duality, and it holds because virtual work is frame independent: "
        "F . v = F . (J qdot) = (J^T F) . qdot = tau . qdot for every qdot. Column i of J is "
        "therefore both the tool velocity joint i alone would cause and the lever arm through "
        "which a tool force loads joint i. Axis ";
    verdict += GP8_AXIS_NAMES[worst];
    verdict += " carries the largest fraction of its torque budget here (";
    verdict += json::number_to_string(worst_fraction);
    verdict += " of the URDF effort limit). ";
    verdict += overloaded ? "At least one axis is over its limit: this wrench cannot be held."
                          : "Every axis is inside its limit.";

    json::Value out = json::Value::object();
    out.set("tau", json::from_vec6(tau));
    out.set("joint_torques",
            json::from_table({"axis", "tau_Nm", "max_torque_Nm", "fraction_of_limit", "overloaded"},
                             rows));
    out.set("wrench_from_tau", json::from_vec6(wrench_from_tau));
    out.set("tau_round_trip", json::from_vec6(tau_round_trip));
    out.set("duality_residual", json::Value((tau_round_trip - tau_in).norm()));
    out.set("wrench_round_trip", json::from_vec6(wrench_round_trip));
    out.set("wrench_residual", json::Value((wrench_round_trip - wrench).norm()));
    out.set("virtual_work_check", json::Value(std::abs(wrench.dot(J * jac::JointVector::Ones()) -
                                                       tau.dot(jac::JointVector::Ones()))));
    out.set("J_transpose", json::from_matrix(Eigen::Matrix<double, 6, 6>(J.transpose())));
    out.set("overloaded", json::Value(overloaded));
    out.set("verdict", json::Value(verdict));
    return out;
}

[[nodiscard]] json::Value op_manipulability(const json::Value& args) {
    const jac::JointVector q = read_q(args);
    const jac::Jacobian J = jac::geometric_jacobian(q);
    const Eigen::Matrix<double, 6, 1> sv = jac::singular_values(J);
    const double sigma_min = sv(5);
    const double sigma_max = sv(0);
    const std::size_t rank = matrix_rank(sv);
    const jac::VelocityEllipsoid ellipsoid = jac::velocity_ellipsoid(J);
    const dhk::SingularityVerdict verdict = dhk::classify_singularity(q, 1e-3);

    std::vector<std::vector<json::Value>> axis_rows;
    axis_rows.reserve(3);
    for (std::size_t i = 0; i < 3; ++i) {
        axis_rows.push_back({
            json::Value(static_cast<int>(i + 1)),
            json::Value(ellipsoid.radii(static_cast<Eigen::Index>(i))),
            json::Value(ellipsoid.axes[i].x()),
            json::Value(ellipsoid.axes[i].y()),
            json::Value(ellipsoid.axes[i].z()),
        });
    }

    std::string text =
        "manipulability is sqrt(det(J J^T)), the product of the six singular values and the "
        "volume of the velocity ellipsoid: it falls to zero exactly at a singularity. "
        "distance_to_singularity is the smallest singular value, which is the honest distance "
        "measure, while the condition number is the eccentricity of the ellipsoid and blows up "
        "first. The three axes and radii below are the translational ellipsoid at the flange: a "
        "long axis is a direction the arm moves easily, a short one a direction it barely moves. ";
    text += verdict.text;

    json::Value out = json::Value::object();
    out.set("singular_values", json::from_vector(sv));
    out.set("condition_number",
            json::Value(sigma_min > kRankTolerance ? sigma_max / sigma_min
                                                   : std::numeric_limits<double>::infinity()));
    out.set("manipulability", json::Value(jac::manipulability(J)));
    out.set("determinant", json::Value(J.determinant()));
    out.set("rank", json::Value(static_cast<int>(rank)));
    out.set("distance_to_singularity", json::Value(sigma_min));
    out.set("ellipsoid_radii", json::from_vec3(ellipsoid.radii));
    out.set("ellipsoid_axis_1", json::from_vec3(ellipsoid.axes[0]));
    out.set("ellipsoid_axis_2", json::from_vec3(ellipsoid.axes[1]));
    out.set("ellipsoid_axis_3", json::from_vec3(ellipsoid.axes[2]));
    out.set("ellipsoid_volume", json::Value(ellipsoid.volume));
    out.set("ellipsoid",
            json::from_table({"axis_index", "radius_m_s", "dir_x", "dir_y", "dir_z"}, axis_rows));
    out.set("ellipsoid_center", json::from_vec3(link_frames(q)[GP8_DOF - 1].translation()));
    out.set("singularity_type", json::Value(verdict.type));
    out.set("verdict", json::Value(text));
    return out;
}

}  // namespace

// ---------------------------------------------------------------------------
// jac:: maps
// ---------------------------------------------------------------------------

namespace jac {

Jacobian geometric_jacobian(const JointVector& q) noexcept {
    const std::array<Eigen::Isometry3d, GP8_DOF> frames = link_frames(q);
    // The flange correction is a pure rotation with no translation, so the DH
    // frame-6 origin is the flange origin and frame 6 and the flange share one
    // angular velocity. The chain Jacobian is therefore the flange Jacobian.
    const Eigen::Vector3d p_e = frames[GP8_DOF - 1].translation();

    Jacobian J = Jacobian::Zero();
    Eigen::Vector3d z_prev(0.0, 0.0, 1.0);
    Eigen::Vector3d o_prev = Eigen::Vector3d::Zero();
    for (std::size_t i = 0; i < GP8_DOF; ++i) {
        const Eigen::Index col = static_cast<Eigen::Index>(i);
        J.block<3, 1>(0, col) = z_prev.cross(p_e - o_prev);
        J.block<3, 1>(3, col) = z_prev;
        z_prev = frames[i].linear().col(2);
        o_prev = frames[i].translation();
    }
    return J;
}

Eigen::Vector3d rotation_difference(const Eigen::Matrix3d& after,
                                    const Eigen::Matrix3d& before) noexcept {
    const Eigen::AngleAxisd aa(Eigen::Matrix3d(after * before.transpose()));
    return aa.angle() * aa.axis();
}

Jacobian numeric_jacobian(const JointVector& q, double step) noexcept {
    Jacobian J = Jacobian::Zero();
    for (Eigen::Index i = 0; i < 6; ++i) {
        JointVector q_plus = q;
        JointVector q_minus = q;
        q_plus(i) += step;
        q_minus(i) -= step;
        const Eigen::Isometry3d T_plus = forward_kinematics_dh(q_plus);
        const Eigen::Isometry3d T_minus = forward_kinematics_dh(q_minus);
        J.block<3, 1>(0, i) = (T_plus.translation() - T_minus.translation()) / (2.0 * step);
        J.block<3, 1>(3, i) =
            rotation_difference(Eigen::Matrix3d(T_plus.linear()),
                                Eigen::Matrix3d(T_minus.linear())) /
            (2.0 * step);
    }
    return J;
}

DampedSolution damped_least_squares(const Jacobian& J, const Twist& twist,
                                    double lambda) noexcept {
    DampedSolution out;
    const Eigen::Matrix<double, 6, 6> JJt =
        J * J.transpose() + (lambda * lambda) * Eigen::Matrix<double, 6, 6>::Identity();
    if (lambda > 0.0) {
        out.qdot = J.transpose() * JJt.ldlt().solve(twist);
    } else {
        // lambda = 0 is the plain pseudo-inverse; an SVD solve is the only
        // form that stays finite when J is rank deficient.
        Eigen::JacobiSVD<Jacobian> svd(J, Eigen::ComputeFullU | Eigen::ComputeFullV);
        svd.setThreshold(1e-12);
        out.qdot = svd.solve(twist);
    }
    out.achieved = J * out.qdot;
    out.residual = twist - out.achieved;
    out.residual_norm = out.residual.norm();
    return out;
}

Eigen::Matrix<double, 6, 1> singular_values(const Jacobian& J) noexcept {
    const Eigen::JacobiSVD<Jacobian> svd(J);
    return svd.singularValues();
}

double manipulability(const Jacobian& J) noexcept {
    const Eigen::Matrix<double, 6, 1> sv = singular_values(J);
    double product = 1.0;
    for (Eigen::Index i = 0; i < 6; ++i) {
        product *= sv(i);
    }
    return product;
}

VelocityEllipsoid velocity_ellipsoid(const Jacobian& J) noexcept {
    VelocityEllipsoid out;
    const Eigen::Matrix<double, 3, 6> Jv = J.block<3, 6>(0, 0);
    const Eigen::Matrix3d A = Jv * Jv.transpose();
    const Eigen::SelfAdjointEigenSolver<Eigen::Matrix3d> solver(A);
    // Eigen returns eigenvalues ascending; the panel wants the long axis first.
    for (std::size_t i = 0; i < 3; ++i) {
        const Eigen::Index source = static_cast<Eigen::Index>(2 - i);
        const double eigenvalue = solver.eigenvalues()(source);
        out.radii(static_cast<Eigen::Index>(i)) = std::sqrt(std::max(0.0, eigenvalue));
        out.axes[i] = solver.eigenvectors().col(source);
    }
    out.volume = (4.0 / 3.0) * kPi * out.radii(0) * out.radii(1) * out.radii(2);
    return out;
}

}  // namespace jac

// ---------------------------------------------------------------------------
// Self-description
// ---------------------------------------------------------------------------

ModuleDescription JacobianStaticsModule::describe() const {
    ModuleDescription d;
    d.name = "jacobian_statics";
    d.title = "The Jacobian: Differential Kinematics and Statics";
    d.course = CourseRef{3883, "M-407-01", "Robotics Modelling"};
    d.topics = {"Block 3 · Differential Kinematics and Statics"};
    d.source = "cpp_solver/include/study/jacobian_statics.hpp";
    d.summary =
        "Builds the 6x6 geometric Jacobian of the GP8 from its link frames and reads it in every "
        "direction the course asks for: forward velocity, damped inverse velocity, the J^T force "
        "duality, and the singular values, condition number and velocity ellipsoid that measure "
        "how close the arm is to losing a degree of freedom.";

    const Eigen::Matrix<double, 6, 1> home = Eigen::Matrix<double, 6, 1>::Zero();
    Eigen::Matrix<double, 6, 1> posed;
    posed << 0.0, 0.3, 0.6, 0.0, 0.8, 0.0;
    Eigen::Matrix<double, 6, 1> rate;
    rate << 0.0, 1.0, 0.0, 0.0, 0.0, 0.0;
    Eigen::Matrix<double, 6, 1> twist;
    twist << 0.1, 0.0, 0.0, 0.0, 0.0, 0.0;
    Eigen::Matrix<double, 6, 1> wrench;
    wrench << 10.0, 0.0, 0.0, 0.0, 0.0, 0.0;

    const double joint_span = widest_joint_range();

    {
        OpSpec op;
        op.name = "geometric_jacobian";
        op.title = "The 6x6 geometric Jacobian";
        op.formula =
            "J(q) = \\begin{bmatrix} J_v \\\\ J_\\omega \\end{bmatrix},\\quad "
            "J_i = \\begin{bmatrix} z_{i-1} \\times (p_e - o_{i-1}) \\\\ z_{i-1} \\end{bmatrix}";
        op.explain =
            "For an all-revolute arm every column of the Jacobian is built from two things you "
            "can point at on the robot: the axis of that joint, z_{i-1}, and the vector from that "
            "joint to the tool, p_e - o_{i-1}. The cross product is the tool velocity one radian "
            "per second of that joint alone would cause; the axis itself is the tool angular "
            "velocity. Stack the six columns and you have the whole differential model. The op "
            "also reports the difference against a central-difference Jacobian of the forward "
            "kinematics: if your frame assignment is wrong, that number is where it shows.";
        op.params = {
            ParamSpec::vec6("q", "Joint vector", "rad", -joint_span, joint_span, posed),
        };
        op.outputs = {
            OutputSpec::make("J", "matrix", "Geometric Jacobian, 6x6"),
            OutputSpec::make("J_linear", "matrix", "Linear half J_v, 3x6"),
            OutputSpec::make("J_angular", "matrix", "Angular half J_omega, 3x6"),
            OutputSpec::make("columns", "table", "Per-column axis, moment arm and gains"),
            OutputSpec::make("determinant", "scalar", "det J"),
            OutputSpec::make("rank", "int", "Rank of J"),
            OutputSpec::make("singular_values", "vec6", "Singular values, descending"),
            OutputSpec::make("numeric_difference_error", "scalar",
                             "|| J_analytic - J_central_difference ||"),
            OutputSpec::make("engine_difference_error", "scalar",
                             "|| J_analytic - YaskawaKinematics::computeJacobian ||"),
            OutputSpec::make("tool_position", "vec3", "Flange origin", "m"),
            OutputSpec::make("verdict", "text", "How to read the two halves"),
        };
        d.ops.push_back(std::move(op));
    }

    {
        OpSpec op;
        op.name = "velocity_propagation";
        op.title = "Joint rates to the tool twist";
        op.formula =
            "\\begin{bmatrix} v_e \\\\ \\omega_e \\end{bmatrix} = J(q)\\,\\dot q,\\quad "
            "\\omega_i = \\omega_{i-1} + z_{i-1}\\dot q_i,\\quad "
            "v_i = v_{i-1} + \\omega_i \\times (o_i - o_{i-1})";
        op.explain =
            "The matrix product and the link-by-link recursion are the same statement, and this "
            "op runs both so you can see that. The recursion is how the lecture derives it: carry "
            "an angular velocity and a linear velocity outwards, adding one joint rate about "
            "z_{i-1} and one rigid-body term omega x r at every link. recursion_residual is the "
            "difference between the two routes and should sit at machine precision. The "
            "contributions table splits the answer per joint, so the axis that actually moves the "
            "tool is obvious instead of inferred, and every rate is checked against the GP8 rated "
            "speed for that axis.";
        op.params = {
            ParamSpec::vec6("q", "Joint vector", "rad", -joint_span, joint_span, posed),
            ParamSpec::vec6("qdot", "Joint rates", "rad/s", -20.0, 20.0, rate),
        };
        op.outputs = {
            OutputSpec::make("twist", "vec6", "Tool twist [v; omega]"),
            OutputSpec::make("linear_velocity", "vec3", "Tool linear velocity", "m/s"),
            OutputSpec::make("angular_velocity", "vec3", "Tool angular velocity", "rad/s"),
            OutputSpec::make("contributions", "table", "Per-joint contribution to the twist"),
            OutputSpec::make("recursion", "table", "Link-by-link propagation, frame by frame"),
            OutputSpec::make("recursion_residual", "scalar", "Recursion minus J qdot"),
            OutputSpec::make("speed_limits", "table", "Each rate against its rated speed"),
            OutputSpec::make("exceeds_speed_limit", "bool", "Any axis over its rated speed"),
            OutputSpec::make("dominant_axis", "text", "Axis contributing the most translation"),
            OutputSpec::make("verdict", "text", "Which axis dominates, and whether it is legal"),
        };
        d.ops.push_back(std::move(op));
    }

    {
        OpSpec op;
        op.name = "inverse_velocity";
        op.title = "Tool twist to joint rates, damped least squares";
        op.formula =
            "\\dot q = J^T\\left(J J^T + \\lambda^2 I\\right)^{-1} "
            "\\begin{bmatrix} v_d \\\\ \\omega_d \\end{bmatrix}";
        op.explain =
            "Inverting the Jacobian directly explodes near a singularity: a finite tool velocity "
            "asks for infinite joint rates. Damped least squares adds lambda^2 I before "
            "inverting, which bounds the joint rates by giving up some of the commanded twist. "
            "The part given up is residual_twist, and it is reported rather than hidden, because "
            "the whole point of lambda is that it is a trade and you have to choose it. The "
            "damping_tradeoff table runs the same twist at five values of lambda so the shape of "
            "that trade is visible, and sigma_min tells you whether damping is needed here at "
            "all.";
        op.params = {
            ParamSpec::vec6("q", "Joint vector", "rad", -joint_span, joint_span, posed),
            ParamSpec::vec6("twist", "Desired tool twist [v; omega]", "m/s, rad/s", -5.0, 5.0,
                            twist),
            ParamSpec::scalar("damping", "Damping factor lambda", "", 0.0, 1.0, 0.01),
        };
        op.outputs = {
            OutputSpec::make("qdot", "vec6", "Joint rates", "rad/s"),
            OutputSpec::make("achieved_twist", "vec6", "J qdot, what the robot would actually do"),
            OutputSpec::make("residual_twist", "vec6", "Desired minus achieved: the damping cost"),
            OutputSpec::make("residual_norm", "scalar", "Norm of the residual twist"),
            OutputSpec::make("qdot_norm", "scalar", "Norm of the joint rates", "rad/s"),
            OutputSpec::make("undamped_qdot_norm", "scalar", "Pseudo-inverse rate norm", "rad/s"),
            OutputSpec::make("undamped_residual_norm", "scalar", "Pseudo-inverse residual"),
            OutputSpec::make("damping_tradeoff", "table", "Rate norm and residual against lambda"),
            OutputSpec::make("singular_values", "vec6", "Singular values of J"),
            OutputSpec::make("sigma_min", "scalar", "Smallest singular value"),
            OutputSpec::make("condition_number", "scalar", "sigma_max / sigma_min"),
            OutputSpec::make("speed_limits", "table", "Each rate against its rated speed"),
            OutputSpec::make("exceeds_speed_limit", "bool", "Any axis over its rated speed"),
            OutputSpec::make("verdict", "text", "Whether this lambda is earning its cost"),
        };
        d.ops.push_back(std::move(op));
    }

    {
        OpSpec op;
        op.name = "force_torque_duality";
        op.title = "Force-torque duality through J transpose";
        op.formula =
            "\\tau = J^T(q)\\,\\mathcal{F},\\qquad "
            "\\mathcal{F} = J^{-T}(q)\\,\\tau,\\qquad "
            "\\mathcal{F}^T v = \\tau^T \\dot q";
        op.explain =
            "Velocity and force run through the same matrix in opposite directions. J maps joint "
            "rates forward to a tool twist; J transpose maps a tool wrench backward to joint "
            "torques. This is not a coincidence and it is not an approximation: virtual work is "
            "the same number computed in either space, so F . (J qdot) = (J^T F) . qdot for every "
            "qdot, which forces tau = J^T F. The practical reading is that column i of J is both "
            "the tool velocity joint i alone would cause and the lever arm through which a tool "
            "force loads joint i. This op runs both directions and the round trip between them, "
            "and checks every torque against the URDF effort limit of that axis.";
        op.params = {
            ParamSpec::vec6("q", "Joint vector", "rad", -joint_span, joint_span, posed),
            ParamSpec::vec6("wrench", "Tool wrench [f; m]", "N, N m", -500.0, 500.0, wrench),
            ParamSpec::vec6("tau", "Joint torques for the reverse map", "N m", -200.0, 200.0,
                            home),
        };
        op.outputs = {
            OutputSpec::make("tau", "vec6", "Joint torques J^T F", "N m"),
            OutputSpec::make("joint_torques", "table", "Each torque against its effort limit"),
            OutputSpec::make("wrench_from_tau", "vec6", "Tool wrench equivalent to the input tau"),
            OutputSpec::make("tau_round_trip", "vec6", "J^T applied to wrench_from_tau"),
            OutputSpec::make("duality_residual", "scalar", "Round-trip error of the reverse map"),
            OutputSpec::make("wrench_round_trip", "vec6", "Reverse map applied to tau"),
            OutputSpec::make("wrench_residual", "scalar", "Round-trip error of the forward map"),
            OutputSpec::make("virtual_work_check", "scalar", "F . (J qdot) - tau . qdot"),
            OutputSpec::make("J_transpose", "matrix", "J^T, the matrix doing the mapping"),
            OutputSpec::make("overloaded", "bool", "Any axis over its effort limit"),
            OutputSpec::make("verdict", "text", "The duality stated plainly"),
        };
        d.ops.push_back(std::move(op));
    }

    {
        OpSpec op;
        op.name = "manipulability";
        op.title = "Singular values, conditioning and the velocity ellipsoid";
        op.formula =
            "w(q) = \\sqrt{\\det\\left(J J^T\\right)} = \\prod_{i=1}^{6}\\sigma_i,\\qquad "
            "\\kappa(J) = \\frac{\\sigma_1}{\\sigma_6}";
        op.explain =
            "The singular values of J are the gains of the arm along six orthogonal directions, "
            "so they answer how well it moves, not just whether it can. Their product is "
            "Yoshikawa's manipulability measure, the volume of the velocity ellipsoid, which hits "
            "zero exactly at a singularity. Their ratio is the condition number, which blows up "
            "well before the determinant reaches zero and is therefore the earlier warning. The "
            "smallest singular value is the honest distance to singularity. The three axes and "
            "radii are the translational ellipsoid drawn at the flange: a long axis is a "
            "direction the tool moves easily, a short one a direction it hardly moves, and the "
            "ellipsoid flattening into a disc is what a singularity looks like.";
        op.params = {
            ParamSpec::vec6("q", "Joint vector", "rad", -joint_span, joint_span, posed),
        };
        op.outputs = {
            OutputSpec::make("singular_values", "vec6", "Singular values, descending"),
            OutputSpec::make("condition_number", "scalar", "sigma_max / sigma_min"),
            OutputSpec::make("manipulability", "scalar", "sqrt(det(J J^T))"),
            OutputSpec::make("determinant", "scalar", "det J"),
            OutputSpec::make("rank", "int", "Rank of J"),
            OutputSpec::make("distance_to_singularity", "scalar", "Smallest singular value"),
            OutputSpec::make("ellipsoid_radii", "vec3", "Principal radii", "m/s"),
            OutputSpec::make("ellipsoid_axis_1", "vec3", "Longest principal axis"),
            OutputSpec::make("ellipsoid_axis_2", "vec3", "Middle principal axis"),
            OutputSpec::make("ellipsoid_axis_3", "vec3", "Shortest principal axis"),
            OutputSpec::make("ellipsoid_volume", "scalar", "Volume of the translational ellipsoid"),
            OutputSpec::make("ellipsoid", "table", "Radii and axes as a table"),
            OutputSpec::make("ellipsoid_center", "vec3", "Where to draw it: the flange", "m"),
            OutputSpec::make("singularity_type", "text", "wrist, elbow, shoulder or none"),
            OutputSpec::make("verdict", "text", "How to read these numbers"),
        };
        d.ops.push_back(std::move(op));
    }

    return d;
}

json::Value JacobianStaticsModule::invoke(std::string_view op, const json::Value& args) const {
    if (op == "geometric_jacobian") {
        return op_geometric_jacobian(args);
    }
    if (op == "velocity_propagation") {
        return op_velocity_propagation(args);
    }
    if (op == "inverse_velocity") {
        return op_inverse_velocity(args);
    }
    if (op == "force_torque_duality") {
        return op_force_torque_duality(args);
    }
    if (op == "manipulability") {
        return op_manipulability(args);
    }
    unknown_op(name(), op);
}

}  // namespace yaskawa::study
