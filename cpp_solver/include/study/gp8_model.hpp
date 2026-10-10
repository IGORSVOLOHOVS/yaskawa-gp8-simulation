#ifndef YASKAWA_STUDY_GP8_MODEL_HPP
#define YASKAWA_STUDY_GP8_MODEL_HPP

// The single source of truth for the physical Yaskawa Motoman GP8
// (contract rule 5: every study module describes the *same* robot).
//
// Provenance of every number below
//  - DATASHEET: Yaskawa Motoman GP8 product datasheet / instruction manual
//    (payload 8 kg, horizontal reach 727 mm, repeatability +/-0.02 mm,
//    controller YRC1000micro, robot mass 34 kg, per-axis ranges and speeds).
//    Ranges and speeds are not repeated here: they are read from
//    GP8_JOINT_LIMITS in yaskawa_kinematics.hpp, which already encodes them.
//  - URDF: src/yaskawa_workcell_description/urdf/gp8_macro.xacro (link offsets,
//    joint axes, effort limits).
//  - ESTIMATED: not published by Yaskawa. Link masses, centres of mass,
//    inertia tensors, gear ratios, rotor inertias and friction coefficients are
//    engineering estimates from the link geometry and from comparable 8 kg
//    6-axis arms. They are physically plausible and internally consistent (the
//    six link masses sum to 23.6 kg, leaving 10.4 kg of the 34 kg datasheet
//    mass in the non-moving base), but they are NOT measured values: a
//    dynamics module must report them as a model, not as the robot.
//
// Denavit-Hartenberg convention: STANDARD (distal) DH,
//     A_i = Rot_z(theta_i) * Trans_z(d_i) * Trans_x(a_i) * Rot_x(alpha_i),
//     theta_i = q_i + theta_offset_i,
// with z_{i-1} along the axis of joint i. The table was derived from the link
// geometry in yaskawa_kinematics.cpp and gp8_macro.xacro (identical numbers in
// both) and reproduces that forward kinematics exactly, up to one constant
// right-multiplied frame rotation - see GP8_FLANGE_CORRECTION below.

#include "yaskawa_kinematics.hpp"

#include <Eigen/Dense>
#include <Eigen/Geometry>

#include <array>
#include <cmath>
#include <cstddef>
#include <numbers>

namespace yaskawa::study {

constexpr std::size_t GP8_DOF = 6;

// ---------------------------------------------------------------------------
// Datasheet ratings
// ---------------------------------------------------------------------------

constexpr double GP8_PAYLOAD_KG = 8.0;            // DATASHEET: rated payload
constexpr double GP8_HORIZONTAL_REACH_M = 0.727;  // DATASHEET: max horizontal reach
constexpr double GP8_REPEATABILITY_M = 20.0e-6;   // DATASHEET: +/-0.02 mm
constexpr double GP8_ROBOT_MASS_KG = 34.0;        // DATASHEET: robot mass
constexpr double GP8_GRAVITY_MPS2 = 9.80665;      // standard gravity

// Base-frame gravity vector: the robot stands upright on the floor.
constexpr std::array<double, 3> GP8_GRAVITY_VECTOR = {0.0, 0.0, -GP8_GRAVITY_MPS2};

[[nodiscard]] inline Eigen::Vector3d gravity_vector() noexcept {
    return Eigen::Vector3d(GP8_GRAVITY_VECTOR[0], GP8_GRAVITY_VECTOR[1], GP8_GRAVITY_VECTOR[2]);
}

// ---------------------------------------------------------------------------
// Standard Denavit-Hartenberg table
// ---------------------------------------------------------------------------

struct DHParams {
    double a;             // link length [m], along x_i
    double alpha;         // link twist [rad], about x_i
    double d;             // link offset [m], along z_{i-1}
    double theta_offset;  // added to q_i [rad], so that q = 0 is the URDF home
};

// Derivation, all in the base frame at q = 0 (axes read off the URDF):
//   joint 1 axis: z through (0, 0, 0)          -> d1 = 0.330, a1 = 0.040
//   joint 2 axis: y through (0.040, 0, 0.330)  -> alpha1 = -pi/2
//   joint 3 axis: y through (0.040, 0, 0.675)  -> a2 = 0.345, alpha2 = 0
//   joint 4 axis: x through (0.380, 0, 0.715)  -> a3 = 0.040, alpha3 = -pi/2
//   joint 5 axis: y through (0.380, 0, 0.715)  -> d4 = 0.340, alpha4 = +pi/2
//   joint 6 axis: x through (0.380, 0, 0.715)  -> alpha5 = -pi/2
// The wrist is spherical: axes 4, 5 and 6 intersect at (0.380, 0, 0.715), so
// a4 = a5 = a6 = d5 = d6 = 0 and the whole 0.340 m forearm shows up as d4.
// theta_offset2 = -pi/2 turns x_1 (base x) into x_2 (base z) so that the
// standard-DH chain lands on the URDF home pose at q = 0.
constexpr std::array<DHParams, GP8_DOF> GP8_DH = {{
    //   a [m]  alpha [rad]                 d [m]   theta_offset [rad]
    {0.040, -std::numbers::pi / 2.0, 0.330, 0.0},                       // 1 S-axis
    {0.345, 0.0, 0.000, -std::numbers::pi / 2.0},                       // 2 L-axis
    {0.040, -std::numbers::pi / 2.0, 0.000, 0.0},                       // 3 U-axis
    {0.000, std::numbers::pi / 2.0, 0.340, 0.0},                        // 4 R-axis
    {0.000, -std::numbers::pi / 2.0, 0.000, 0.0},                       // 5 B-axis
    {0.000, 0.0, 0.000, 0.0},                                           // 6 T-axis
}};

// Axis names as Yaskawa labels them, for tables in the UI.
constexpr std::array<const char*, GP8_DOF> GP8_AXIS_NAMES = {"S", "L", "U", "R", "B", "T"};

// DH frame 6 and the URDF flange frame share an origin but not an orientation:
// DH puts x_6 along the base z and z_6 along the base x at q = 0, while the
// URDF flange is identity-aligned with the base. This constant rotation,
// right-multiplied onto the DH chain, reconciles the two exactly:
//     T_flange(q) = A_1 ... A_6 * GP8_FLANGE_CORRECTION
//                 = YaskawaKinematics::forwardKinematics(q).
// It is a pure permutation/reflection pair (symmetric, its own inverse).
constexpr std::array<double, 9> GP8_FLANGE_CORRECTION = {
    0.0, 0.0, 1.0,
    0.0, -1.0, 0.0,
    1.0, 0.0, 0.0,
};

[[nodiscard]] inline Eigen::Isometry3d flange_correction() noexcept {
    Eigen::Matrix3d R;
    R << GP8_FLANGE_CORRECTION[0], GP8_FLANGE_CORRECTION[1], GP8_FLANGE_CORRECTION[2],
         GP8_FLANGE_CORRECTION[3], GP8_FLANGE_CORRECTION[4], GP8_FLANGE_CORRECTION[5],
         GP8_FLANGE_CORRECTION[6], GP8_FLANGE_CORRECTION[7], GP8_FLANGE_CORRECTION[8];
    Eigen::Isometry3d T = Eigen::Isometry3d::Identity();
    T.linear() = R;
    return T;
}

// One standard-DH link transform. `q` is the joint value; the table's
// theta_offset is added here so callers never apply it twice.
[[nodiscard]] inline Eigen::Isometry3d dh_transform(const DHParams& dh, double q) noexcept {
    const double theta = q + dh.theta_offset;
    const double ct = std::cos(theta);
    const double st = std::sin(theta);
    const double ca = std::cos(dh.alpha);
    const double sa = std::sin(dh.alpha);

    Eigen::Isometry3d T = Eigen::Isometry3d::Identity();
    T.matrix() << ct, -st * ca,  st * sa, dh.a * ct,
                  st,  ct * ca, -ct * sa, dh.a * st,
                 0.0,       sa,       ca,      dh.d,
                 0.0,      0.0,      0.0,       1.0;
    return T;
}

// T_0_i for i = 1..6, i.e. every link frame relative to the base.
[[nodiscard]] inline std::array<Eigen::Isometry3d, GP8_DOF> link_frames(
    const Eigen::Matrix<double, GP8_DOF, 1>& q) noexcept {
    std::array<Eigen::Isometry3d, GP8_DOF> frames;
    Eigen::Isometry3d T = Eigen::Isometry3d::Identity();
    for (std::size_t i = 0; i < GP8_DOF; ++i) {
        T = T * dh_transform(GP8_DH[i], q[static_cast<Eigen::Index>(i)]);
        frames[i] = T;
    }
    return frames;
}

// The DH chain through joint 6, without the flange correction.
[[nodiscard]] inline Eigen::Isometry3d dh_chain(const Eigen::Matrix<double, GP8_DOF, 1>& q) noexcept {
    return link_frames(q)[GP8_DOF - 1];
}

// The flange pose, identical to YaskawaKinematics::forwardKinematics(q).
[[nodiscard]] inline Eigen::Isometry3d forward_kinematics_dh(
    const Eigen::Matrix<double, GP8_DOF, 1>& q) noexcept {
    return dh_chain(q) * flange_correction();
}

// ---------------------------------------------------------------------------
// Joint limits and speeds - read from GP8_JOINT_LIMITS, never duplicated
// ---------------------------------------------------------------------------

[[nodiscard]] constexpr double joint_min(std::size_t joint) noexcept {
    return GP8_JOINT_LIMITS[joint].min_angle;
}

[[nodiscard]] constexpr double joint_max(std::size_t joint) noexcept {
    return GP8_JOINT_LIMITS[joint].max_angle;
}

[[nodiscard]] constexpr double joint_max_velocity(std::size_t joint) noexcept {
    return GP8_JOINT_LIMITS[joint].max_vel;
}

// Widest single-axis range in the machine (T-axis, +/-360 deg). The UI uses it
// as the slider span of a vec6 joint control; per-joint clamping still happens
// through joint_min/joint_max.
[[nodiscard]] constexpr double widest_joint_range() noexcept {
    double widest = 0.0;
    for (std::size_t i = 0; i < GP8_DOF; ++i) {
        const double bound = (-joint_min(i) > joint_max(i)) ? -joint_min(i) : joint_max(i);
        if (bound > widest) {
            widest = bound;
        }
    }
    return widest;
}

// ---------------------------------------------------------------------------
// Rigid-body and drive-train parameters, one struct per moving link
// ---------------------------------------------------------------------------

struct LinkParams {
    const char* axis;                   // Yaskawa axis letter
    double mass;                        // [kg]           ESTIMATED
    std::array<double, 3> com;          // [m] in DH frame i, ESTIMATED
    std::array<double, 9> inertia;      // [kg m^2] about the com, row-major,
                                        //          DH frame i axes, ESTIMATED
    double gear_ratio;                  // motor rev / joint rev, ESTIMATED
    double rotor_inertia;               // [kg m^2] motor side, ESTIMATED
    double viscous_friction;            // [N m s / rad] joint side, ESTIMATED
    double coulomb_friction;            // [N m] joint side, ESTIMATED
    double max_torque;                  // [N m] joint side, URDF effort limit
};

// Inertias are the box/rod/disc approximations of the link geometry that the DH
// table implies: a 0.20 x 0.20 x 0.25 turret (S), a 0.345 m arm of 0.15 m
// section (L), a 0.34 m forearm of 0.12 m section (U) and a compact three-axis
// wrist (R, B, T). They are ESTIMATED, as the comment at the top of the file
// states, and are what the dynamics and control modules are expected to tune.
constexpr std::array<LinkParams, GP8_DOF> GP8_LINKS = {{
    {
        "S", 8.0,
        {0.000, 0.000, -0.050},
        {0.06833, 0.0, 0.0,
         0.0, 0.06833, 0.0,
         0.0, 0.0, 0.05333},
        141.0, 1.50e-4, 1.20, 4.00, 100.0,
    },
    {
        "L", 6.5,
        {-0.1725, 0.000, 0.000},
        {0.02438, 0.0, 0.0,
         0.0, 0.07663, 0.0,
         0.0, 0.0, 0.07663},
        161.0, 1.50e-4, 1.20, 4.00, 100.0,
    },
    {
        "U", 5.0,
        {0.000, 0.000, 0.150},
        {0.05417, 0.0, 0.0,
         0.0, 0.05417, 0.0,
         0.0, 0.0, 0.01200},
        141.0, 1.00e-4, 0.80, 3.00, 100.0,
    },
    {
        "R", 2.2,
        {0.000, 0.000, -0.040},
        {0.00447, 0.0, 0.0,
         0.0, 0.00447, 0.0,
         0.0, 0.0, 0.00367},
        81.0, 5.00e-5, 0.35, 1.20, 50.0,
    },
    {
        "B", 1.4,
        {0.000, 0.000, 0.010},
        {0.00149, 0.0, 0.0,
         0.0, 0.00149, 0.0,
         0.0, 0.0, 0.00149},
        81.0, 5.00e-5, 0.35, 1.20, 50.0,
    },
    {
        "T", 0.5,
        {0.000, 0.000, 0.000},
        {0.00011, 0.0, 0.0,
         0.0, 0.00011, 0.0,
         0.0, 0.0, 0.00040},
        51.0, 3.00e-5, 0.20, 0.80, 30.0,
    },
}};

[[nodiscard]] constexpr double total_moving_mass() noexcept {
    double sum = 0.0;
    for (std::size_t i = 0; i < GP8_DOF; ++i) {
        sum += GP8_LINKS[i].mass;
    }
    return sum;
}

static_assert(total_moving_mass() < GP8_ROBOT_MASS_KG,
              "the six moving links cannot outweigh the whole robot");

[[nodiscard]] inline Eigen::Vector3d link_com(std::size_t joint) noexcept {
    const auto& c = GP8_LINKS[joint].com;
    return Eigen::Vector3d(c[0], c[1], c[2]);
}

[[nodiscard]] inline Eigen::Matrix3d link_inertia(std::size_t joint) noexcept {
    const auto& I = GP8_LINKS[joint].inertia;
    Eigen::Matrix3d out;
    out << I[0], I[1], I[2],
           I[3], I[4], I[5],
           I[6], I[7], I[8];
    return out;
}

// Joint-side inertia the motor adds through the gearbox: n^2 * J_rotor.
[[nodiscard]] inline double reflected_rotor_inertia(std::size_t joint) noexcept {
    const double n = GP8_LINKS[joint].gear_ratio;
    return n * n * GP8_LINKS[joint].rotor_inertia;
}

}  // namespace yaskawa::study

#endif  // YASKAWA_STUDY_GP8_MODEL_HPP
