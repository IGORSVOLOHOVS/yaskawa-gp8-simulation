#include "study/spatial_math.hpp"

#include "study/gp8_model.hpp"

#include <algorithm>
#include <cmath>
#include <numbers>
#include <string>
#include <utility>
#include <vector>

namespace yaskawa::study {

namespace {

constexpr double kPi = std::numbers::pi;

// Angles wrap, so validation only guards against nonsense (a UI sending a
// pixel coordinate), not against a student typing 7 rad on purpose.
constexpr double kAngleGuard = 100.0;
constexpr double kLengthGuard = 100.0;  // [m]

// The threshold below which the RPY inverse map is degenerate. cos(pitch)
// smaller than this leaves roll and yaw coupled to within double precision.
constexpr double kGimbalThreshold = 1e-9;

constexpr std::size_t kMaxComposeTransforms = 32;

[[nodiscard]] Eigen::Matrix3d rot_x(double angle) noexcept {
    const double c = std::cos(angle);
    const double s = std::sin(angle);
    Eigen::Matrix3d R;
    R << 1.0, 0.0, 0.0,
         0.0,   c,  -s,
         0.0,   s,   c;
    return R;
}

[[nodiscard]] Eigen::Matrix3d rot_y(double angle) noexcept {
    const double c = std::cos(angle);
    const double s = std::sin(angle);
    Eigen::Matrix3d R;
    R <<   c, 0.0,   s,
         0.0, 1.0, 0.0,
          -s, 0.0,   c;
    return R;
}

[[nodiscard]] Eigen::Matrix3d rot_z(double angle) noexcept {
    const double c = std::cos(angle);
    const double s = std::sin(angle);
    Eigen::Matrix3d R;
    R <<   c,  -s, 0.0,
           s,   c, 0.0,
         0.0, 0.0, 1.0;
    return R;
}

[[nodiscard]] json::Value quaternion_table(const Eigen::Matrix3d& R) {
    const Eigen::Quaterniond q(R);
    return json::from_table({"w", "x", "y", "z"},
                            std::vector<std::vector<double>>{{q.w(), q.x(), q.y(), q.z()}});
}

[[nodiscard]] Eigen::Vector3d read_triple(const json::Value& value, const std::string& label) {
    if (!value.is_array() || value.size() != 3) {
        throw StudyError("parameter '" + label + "' must be an array of 3 numbers, received " +
                         std::to_string(value.size()) + " elements");
    }
    Eigen::Vector3d out;
    for (std::size_t i = 0; i < 3; ++i) {
        const json::Value& element = value[i];
        if (!element.is_number() || !std::isfinite(element.as_double())) {
            throw StudyError("parameter '" + label + "' element " + std::to_string(i) +
                             " must be a finite number");
        }
        out[static_cast<Eigen::Index>(i)] = element.as_double();
    }
    return out;
}

struct RpyAndTranslation {
    Eigen::Vector3d rpy{Eigen::Vector3d::Zero()};
    Eigen::Vector3d p{Eigen::Vector3d::Zero()};
};

// `compose` accepts either shape, because the UI table and a hand-written
// request do not agree on one:
//   [{"rpy": [r, p, y], "p": [x, y, z]}, ...]                 (list of frames)
//   [[r, p, y, x, y, z], ...]                                 (table rows)
//   {"columns": [...], "rows": [[r, p, y, x, y, z], ...]}     (a `table` value)
[[nodiscard]] std::vector<RpyAndTranslation> read_transform_list(const json::Value& args) {
    const json::Value& value = require_present(args, "transforms");

    const json::Value* rows = nullptr;
    if (value.is_array()) {
        rows = &value;
    } else if (value.is_object() && value["rows"].is_array()) {
        rows = &value["rows"];
    } else {
        throw StudyError(
            "parameter 'transforms' must be a list of {rpy, p} objects, a list of 6-element rows, "
            "or a table with a 'rows' array");
    }

    if (rows->size() == 0) {
        throw StudyError("parameter 'transforms' needs at least 1 transform, received 0");
    }
    if (rows->size() > kMaxComposeTransforms) {
        throw StudyError("parameter 'transforms' accepts at most " +
                         std::to_string(kMaxComposeTransforms) + " transforms, received " +
                         std::to_string(rows->size()));
    }

    std::vector<RpyAndTranslation> out;
    out.reserve(rows->size());
    for (std::size_t i = 0; i < rows->size(); ++i) {
        const json::Value& entry = (*rows)[i];
        const std::string label = "transforms[" + std::to_string(i) + "]";
        RpyAndTranslation item;
        if (entry.is_object()) {
            if (!entry.contains("rpy") || !entry.contains("p")) {
                throw StudyError("parameter '" + label + "' must carry both 'rpy' and 'p'");
            }
            item.rpy = read_triple(entry["rpy"], label + ".rpy");
            item.p = read_triple(entry["p"], label + ".p");
        } else if (entry.is_array() && entry.size() == 6) {
            for (std::size_t k = 0; k < 6; ++k) {
                const json::Value& cell = entry[k];
                if (!cell.is_number() || !std::isfinite(cell.as_double())) {
                    throw StudyError("parameter '" + label + "' element " + std::to_string(k) +
                                     " must be a finite number");
                }
                const double v = cell.as_double();
                if (k < 3) {
                    item.rpy[static_cast<Eigen::Index>(k)] = v;
                } else {
                    item.p[static_cast<Eigen::Index>(k - 3)] = v;
                }
            }
        } else {
            throw StudyError("parameter '" + label +
                             "' must be either {rpy, p} or a row of 6 numbers");
        }
        out.push_back(item);
    }
    return out;
}

[[nodiscard]] json::Value identity_mat3_json() {
    return json::from_matrix(Eigen::Matrix3d::Identity());
}

}  // namespace

// ---------------------------------------------------------------------------
// spatial:: maps
// ---------------------------------------------------------------------------

namespace spatial {

Eigen::Matrix3d rotation_from_rpy(const Eigen::Vector3d& rpy) noexcept {
    // Fixed-axis X then Y then Z: pre-multiplication, so Rz * Ry * Rx.
    return rot_z(rpy.z()) * rot_y(rpy.y()) * rot_x(rpy.x());
}

Eigen::Matrix3d rotation_from_euler_zyz(const Eigen::Vector3d& angles) noexcept {
    return rot_z(angles.x()) * rot_y(angles.y()) * rot_z(angles.z());
}

Eigen::Matrix3d rotation_from_euler_zyx(const Eigen::Vector3d& angles) noexcept {
    return rot_z(angles.x()) * rot_y(angles.y()) * rot_x(angles.z());
}

RpyBranches rpy_from_rotation(const Eigen::Matrix3d& R) noexcept {
    RpyBranches out;
    const double cos_pitch = std::hypot(R(0, 0), R(1, 0));
    out.gimbal_margin = cos_pitch;

    if (cos_pitch < kGimbalThreshold) {
        // Degenerate: only (roll - yaw) for pitch = +pi/2, or (roll + yaw) for
        // pitch = -pi/2, is observable. Pin yaw to zero and say so.
        out.gimbal_lock = true;
        const bool pitch_positive = (R(2, 0) < 0.0);
        const double pitch = pitch_positive ? (kPi / 2.0) : (-kPi / 2.0);
        const double roll = pitch_positive ? std::atan2(R(0, 1), R(0, 2))
                                           : std::atan2(-R(0, 1), -R(0, 2));
        out.primary = Eigen::Vector3d(roll, pitch, 0.0);
        out.alternate = out.primary;
        return out;
    }

    const double pitch_1 = std::atan2(-R(2, 0), cos_pitch);
    const double yaw_1 = std::atan2(R(1, 0), R(0, 0));
    const double roll_1 = std::atan2(R(2, 1), R(2, 2));
    out.primary = Eigen::Vector3d(roll_1, pitch_1, yaw_1);

    const double pitch_2 = std::atan2(-R(2, 0), -cos_pitch);
    const double yaw_2 = std::atan2(-R(1, 0), -R(0, 0));
    const double roll_2 = std::atan2(-R(2, 1), -R(2, 2));
    out.alternate = Eigen::Vector3d(roll_2, pitch_2, yaw_2);
    return out;
}

double orthonormality_error(const Eigen::Matrix3d& R) noexcept {
    return (R.transpose() * R - Eigen::Matrix3d::Identity()).norm();
}

Eigen::Isometry3d transform_from_rpy_p(const Eigen::Vector3d& rpy,
                                       const Eigen::Vector3d& p) noexcept {
    Eigen::Isometry3d T = Eigen::Isometry3d::Identity();
    T.linear() = rotation_from_rpy(rpy);
    T.translation() = p;
    return T;
}

}  // namespace spatial

// ---------------------------------------------------------------------------
// Self-description
// ---------------------------------------------------------------------------

ModuleDescription SpatialMathModule::describe() const {
    ModuleDescription d;
    d.name = "spatial_math";
    d.title = "Position, Orientation and Rotation Matrices";
    d.course = CourseRef{3883, "M-407-01", "Robotics Modelling"};
    // The separator is a UTF-8 middle dot (U+00B7), as the contract's example
    // shows. It passes through dump() untouched: JSON text is UTF-8.
    d.topics ={"Block 1 · Spatial Descriptions and Transformations"};
    d.source = "cpp_solver/include/study/spatial_math.hpp";
    d.summary =
        "Builds and inverts rotation matrices, composes homogeneous transforms and places every "
        "link frame of the GP8, reporting the orthonormality drift of every rotation it produces.";

    const Eigen::Vector3d zero3 = Eigen::Vector3d::Zero();
    const Eigen::Matrix<double, 6, 1> zero6 = Eigen::Matrix<double, 6, 1>::Zero();

    {
        OpSpec op;
        op.name = "rotation_from_rpy";
        op.title = "Roll-pitch-yaw to rotation matrix";
        op.formula =
            "R_{XYZ}(\\phi,\\theta,\\psi) = R_z(\\psi)\\,R_y(\\theta)\\,R_x(\\phi)";
        op.explain =
            "Roll-pitch-yaw rotates about the FIXED base axes: first roll about x, then pitch "
            "about y, then yaw about z. Because each rotation is about an axis of the original "
            "frame, the matrices pre-multiply, which is why the product reads right-to-left as "
            "Rz Ry Rx. The same matrix is also the moving-axis Z-Y-X Euler result with the angles "
            "reversed - that equivalence is the classic exam question. The quaternion and the "
            "axis-angle pair describe the identical rotation with fewer numbers and no gimbal "
            "lock.";
        op.params = {
            ParamSpec::vec3("rpy", "Roll-pitch-yaw", "rad", -kPi, kPi, zero3),
        };
        op.outputs = {
            OutputSpec::make("R", "mat3", "Rotation matrix"),
            OutputSpec::make("quaternion", "table", "Equivalent unit quaternion (w, x, y, z)"),
            OutputSpec::make("axis", "vec3", "Equivalent rotation axis"),
            OutputSpec::make("angle", "scalar", "Equivalent rotation angle", "rad"),
            OutputSpec::make("det_R", "scalar", "det(R), must stay 1"),
            OutputSpec::make("orthonormality_error", "scalar", "|| R^T R - I ||_F"),
        };
        d.ops.push_back(std::move(op));
    }
    {
        OpSpec op;
        op.name = "rotation_from_euler";
        op.title = "Euler angles to rotation matrix";
        op.formula =
            "R_{ZYZ}(\\alpha,\\beta,\\gamma) = R_z(\\alpha) R_y(\\beta) R_z(\\gamma), \\qquad "
            "R_{ZYX}(\\alpha,\\beta,\\gamma) = R_z(\\alpha) R_y(\\beta) R_x(\\gamma)";
        op.explain =
            "Euler angles rotate about the MOVING axes of the frame being rotated, so the "
            "matrices post-multiply and the product reads left-to-right. ZYZ is the convention "
            "most robotics texts use for a spherical wrist; ZYX is the one that coincides with "
            "fixed-axis roll-pitch-yaw once the angle order is reversed. Pick the wrong "
            "convention and the arm points somewhere else, which is why the answer carries the "
            "convention that was applied.";
        op.params = {
            ParamSpec::vec3("angles", "Euler angles (first, second, third rotation)", "rad", -kPi,
                            kPi, zero3),
            ParamSpec::enumeration("convention", "Euler convention", {"zyz", "zyx"}, "zyz"),
        };
        op.outputs = {
            OutputSpec::make("R", "mat3", "Rotation matrix"),
            OutputSpec::make("convention", "text", "Convention that was applied"),
            OutputSpec::make("note", "text", "What the convention means for the multiplication order"),
            OutputSpec::make("rpy_equivalent", "vec3", "Fixed-axis roll-pitch-yaw of the same R", "rad"),
            OutputSpec::make("det_R", "scalar", "det(R), must stay 1"),
            OutputSpec::make("orthonormality_error", "scalar", "|| R^T R - I ||_F"),
        };
        d.ops.push_back(std::move(op));
    }
    {
        OpSpec op;
        op.name = "rpy_from_rotation";
        op.title = "Rotation matrix to roll-pitch-yaw";
        op.formula =
            "\\theta = \\operatorname{atan2}\\left(-r_{31}, \\pm\\sqrt{r_{11}^2 + r_{21}^2}"
            "\\right), \\quad \\psi = \\operatorname{atan2}\\left(\\frac{r_{21}}{\\cos\\theta}, "
            "\\frac{r_{11}}{\\cos\\theta}\\right), \\quad \\phi = \\operatorname{atan2}"
            "\\left(\\frac{r_{32}}{\\cos\\theta}, \\frac{r_{33}}{\\cos\\theta}\\right)";
        op.explain =
            "The inverse map is two-valued: the plus and the minus square root give two "
            "roll-pitch-yaw triples that produce exactly the same matrix, so both branches are "
            "returned rather than one being silently preferred. When cos(pitch) approaches zero "
            "the frame is in gimbal lock: roll and yaw stop being separately observable and only "
            "their sum or difference survives. The margin output is |cos(pitch)|, so the student "
            "can watch the conditioning collapse instead of being surprised by a jump.";
        op.params = {
            ParamSpec::structured("R", "Rotation matrix", "mat3", "", identity_mat3_json()),
        };
        op.outputs = {
            OutputSpec::make("rpy", "vec3", "Branch 1, pitch in [-pi/2, pi/2]", "rad"),
            OutputSpec::make("rpy_alternate", "vec3", "Branch 2, the other pre-image", "rad"),
            OutputSpec::make("gimbal_margin", "scalar", "|cos(pitch)|, 0 at gimbal lock"),
            OutputSpec::make("gimbal_lock", "bool", "True when the branches collapse"),
            OutputSpec::make("det_R", "scalar", "det(R) of the matrix that was supplied"),
            OutputSpec::make("orthonormality_error", "scalar", "|| R^T R - I ||_F of the input"),
            OutputSpec::make("note", "text", "Verdict on the conditioning of this inverse"),
        };
        d.ops.push_back(std::move(op));
    }
    {
        OpSpec op;
        op.name = "compose";
        op.title = "Compose homogeneous transforms";
        op.formula =
            "T = T_1 T_2 \\cdots T_n,\\quad T_i = \\begin{bmatrix} R_i & p_i \\\\ 0 & 1 "
            "\\end{bmatrix},\\quad T^{-1} = \\begin{bmatrix} R^T & -R^T p \\\\ 0 & 1 "
            "\\end{bmatrix}";
        op.explain =
            "A homogeneous transform packs a rotation and a translation into one 4x4 matrix so "
            "that chaining frames becomes plain matrix multiplication - that is the whole reason "
            "robot kinematics is written this way. The order matters: T_1 T_2 reads as "
            "'frame 2 expressed in frame 1'. The intermediate products are returned as well, "
            "because they are the frames a 3D view has to draw. The inverse is built from R^T and "
            "-R^T p, never by a general 4x4 matrix inversion.";
        op.params = {
            ParamSpec::structured(
                "transforms", "Chain of transforms (roll, pitch, yaw, x, y, z per row)", "table",
                "",
                json::from_table({"roll", "pitch", "yaw", "x", "y", "z"},
                                 std::vector<std::vector<double>>{{0.0, 0.0, 0.0, 0.04, 0.0, 0.33},
                                                                  {0.0, 0.0, 0.0, 0.0, 0.0, 0.345}})),
        };
        op.outputs = {
            OutputSpec::make("T", "mat4", "Product of the chain"),
            OutputSpec::make("partials", "mat4_set", "Running products T_1 ... T_1..T_n"),
            OutputSpec::make("T_inverse", "mat4", "Inverse of the product"),
            OutputSpec::make("position", "vec3", "Translation of the product", "m"),
            OutputSpec::make("rpy", "vec3", "Roll-pitch-yaw of the product", "rad"),
            OutputSpec::make("count", "int", "Number of transforms composed"),
            OutputSpec::make("inverse_residual", "scalar", "|| T T^{-1} - I ||_F"),
            OutputSpec::make("orthonormality_error", "scalar", "|| R^T R - I ||_F of the product"),
        };
        d.ops.push_back(std::move(op));
    }
    {
        OpSpec op;
        op.name = "transform_point";
        op.title = "Map a point into another frame";
        op.formula =
            "{}^{A}p = {}^{A}_{B}R \\, {}^{B}p + {}^{A}p_{B_{org}}, \\qquad "
            "\\begin{bmatrix} {}^{A}p \\\\ 1 \\end{bmatrix} = {}^{A}_{B}T "
            "\\begin{bmatrix} {}^{B}p \\\\ 1 \\end{bmatrix}";
        op.explain =
            "A point is rotated and then translated, and the homogeneous form does both in one "
            "multiplication by appending a 1 to the point. The step table is the arithmetic "
            "written out: each row is one component of the answer, each column one term of the "
            "dot product plus the translation. The last row is the homogeneous one; it stays 1, "
            "which is exactly what distinguishes a point from a direction vector (which would "
            "carry 0 there and so ignore the translation).";
        op.params = {
            ParamSpec::vec3("rpy", "Roll-pitch-yaw of the frame", "rad", -kPi, kPi, zero3),
            ParamSpec::vec3("p", "Origin of the frame", "m", -2.0, 2.0,
                            Eigen::Vector3d(0.04, 0.0, 0.33)),
            ParamSpec::vec3("point", "Point in the source frame", "m", -2.0, 2.0,
                            Eigen::Vector3d(0.1, 0.0, 0.0)),
        };
        op.outputs = {
            OutputSpec::make("p_out", "vec3", "Point in the target frame", "m"),
            OutputSpec::make("steps", "table", "Homogeneous arithmetic, one row per component"),
            OutputSpec::make("T", "mat4", "Transform that was applied"),
            OutputSpec::make("p_homogeneous", "table", "The 4-vector that came out"),
        };
        d.ops.push_back(std::move(op));
    }
    {
        OpSpec op;
        op.name = "frame_of_joint";
        op.title = "Link frames of the GP8 from the DH table";
        op.formula =
            "{}^{0}_{i}T = \\prod_{k=1}^{i} A_k(q_k), \\qquad A_k = R_z(\\theta_k) T_z(d_k) "
            "T_x(a_k) R_x(\\alpha_k), \\quad \\theta_k = q_k + \\theta_{k,0}";
        op.explain =
            "This is the forward kinematics of the real machine, one standard Denavit-Hartenberg "
            "link at a time. Each A_k screws along and about z, then along and about x, and four "
            "numbers per joint are enough to describe any serial arm. The frames come back as "
            "4x4 matrices and their origins as points, which is what the 3D view draws. The "
            "flange frame is the DH chain times one constant rotation, because the DH frame 6 and "
            "the URDF flange axes differ by a fixed re-labelling of x, y and z.";
        op.params = {
            ParamSpec::vec6("q", "Joint vector S, L, U, R, B, T", "rad", -widest_joint_range(),
                            widest_joint_range(), zero6),
        };
        op.outputs = {
            OutputSpec::make("frames", "mat4_set", "T_0_1 ... T_0_6, base-relative link frames"),
            OutputSpec::make("origins", "points", "Origin of every frame, base included", "m"),
            OutputSpec::make("T_flange", "mat4", "Flange pose, equal to the engine's FK"),
            OutputSpec::make("flange_position", "vec3", "Flange origin", "m"),
            OutputSpec::make("dh", "table", "The DH table with the theta actually used"),
            OutputSpec::make("limits", "table", "Per-joint limit check"),
            OutputSpec::make("within_limits", "bool", "True when every joint is inside its range"),
            OutputSpec::make("orthonormality_error", "scalar", "|| R^T R - I ||_F of the flange"),
        };
        d.ops.push_back(std::move(op));
    }

    return d;
}

// ---------------------------------------------------------------------------
// Ops
// ---------------------------------------------------------------------------

namespace {

[[nodiscard]] json::Value op_rotation_from_rpy(const json::Value& args) {
    const Eigen::Vector3d rpy = require_vec3(args, "rpy", -kAngleGuard, kAngleGuard);
    const Eigen::Matrix3d R = spatial::rotation_from_rpy(rpy);
    const Eigen::AngleAxisd axis_angle(R);

    json::Value out = json::Value::object();
    out.set("R", json::from_matrix(R));
    out.set("quaternion", quaternion_table(R));
    out.set("axis", json::from_vec3(axis_angle.axis()));
    out.set("angle", json::Value(axis_angle.angle()));
    out.set("det_R", json::Value(R.determinant()));
    out.set("orthonormality_error", json::Value(spatial::orthonormality_error(R)));
    return out;
}

[[nodiscard]] json::Value op_rotation_from_euler(const json::Value& args) {
    const Eigen::Vector3d angles = require_vec3(args, "angles", -kAngleGuard, kAngleGuard);
    const std::string convention = optional_enum(args, "convention", "zyz", {"zyz", "zyx"});

    Eigen::Matrix3d R;
    std::string note;
    if (convention == "zyz") {
        R = spatial::rotation_from_euler_zyz(angles);
        note =
            "ZYZ moving-axis Euler angles: R = Rz(a1) Ry(a2) Rz(a3), each rotation about an axis "
            "of the frame produced by the previous one. Degenerate when a2 = 0 or a2 = pi.";
    } else {
        R = spatial::rotation_from_euler_zyx(angles);
        note =
            "ZYX moving-axis Euler angles: R = Rz(a1) Ry(a2) Rx(a3). This is the same matrix as "
            "fixed-axis roll-pitch-yaw with roll = a3, pitch = a2, yaw = a1.";
    }

    json::Value out = json::Value::object();
    out.set("R", json::from_matrix(R));
    out.set("convention", json::Value(convention));
    out.set("note", json::Value(note));
    out.set("rpy_equivalent", json::from_vec3(spatial::rpy_from_rotation(R).primary));
    out.set("det_R", json::Value(R.determinant()));
    out.set("orthonormality_error", json::Value(spatial::orthonormality_error(R)));
    return out;
}

[[nodiscard]] json::Value op_rpy_from_rotation(const json::Value& args) {
    const Eigen::MatrixXd supplied = require_matrix(args, "R", 3, 3);
    const Eigen::Matrix3d R = supplied;
    const spatial::RpyBranches branches = spatial::rpy_from_rotation(R);
    const double drift = spatial::orthonormality_error(R);

    std::string note;
    if (branches.gimbal_lock) {
        note =
            "Gimbal lock: cos(pitch) is numerically zero, so roll and yaw are no longer "
            "separately observable. Yaw was pinned to 0 and the whole rotation about the "
            "collapsed axis was put into roll; both branches are therefore identical.";
    } else if (branches.gimbal_margin < 1e-3) {
        note =
            "Close to gimbal lock: |cos(pitch)| is below 1e-3, so roll and yaw are badly "
            "conditioned and a small change in R will swing them a long way.";
    } else {
        note =
            "Well conditioned: the two branches are genuinely different pre-images of the same "
            "rotation, and branch 1 is the one with pitch inside [-pi/2, pi/2].";
    }
    if (drift > 1e-9) {
        note += " The matrix supplied is not orthonormal to 1e-9; the angles below describe the "
                "closest interpretation of it, not a true rotation.";
    }

    json::Value out = json::Value::object();
    out.set("rpy", json::from_vec3(branches.primary));
    out.set("rpy_alternate", json::from_vec3(branches.alternate));
    out.set("gimbal_margin", json::Value(branches.gimbal_margin));
    out.set("gimbal_lock", json::Value(branches.gimbal_lock));
    out.set("det_R", json::Value(R.determinant()));
    out.set("orthonormality_error", json::Value(drift));
    out.set("note", json::Value(note));
    return out;
}

[[nodiscard]] json::Value op_compose(const json::Value& args) {
    const std::vector<RpyAndTranslation> chain = read_transform_list(args);

    std::vector<Eigen::Isometry3d> partials;
    partials.reserve(chain.size());

    Eigen::Isometry3d product = Eigen::Isometry3d::Identity();
    for (const auto& item : chain) {
        product = product * spatial::transform_from_rpy_p(item.rpy, item.p);
        partials.push_back(product);
    }

    Eigen::Isometry3d inverse = Eigen::Isometry3d::Identity();
    inverse.linear() = product.linear().transpose();
    inverse.translation() = -product.linear().transpose() * product.translation();

    const Eigen::Matrix4d residual =
        (product * inverse).matrix() - Eigen::Matrix4d::Identity();
    const Eigen::Matrix3d R = product.linear();

    json::Value out = json::Value::object();
    out.set("T", json::from_isometry(product));
    out.set("partials", json::from_isometry_list(partials));
    out.set("T_inverse", json::from_isometry(inverse));
    out.set("position", json::from_vec3(product.translation()));
    out.set("rpy", json::from_vec3(spatial::rpy_from_rotation(R).primary));
    out.set("count", json::Value(static_cast<int>(chain.size())));
    out.set("inverse_residual", json::Value(residual.norm()));
    out.set("orthonormality_error", json::Value(spatial::orthonormality_error(R)));
    return out;
}

[[nodiscard]] json::Value op_transform_point(const json::Value& args) {
    const Eigen::Vector3d rpy = require_vec3(args, "rpy", -kAngleGuard, kAngleGuard);
    const Eigen::Vector3d p = require_vec3(args, "p", -kLengthGuard, kLengthGuard);
    const Eigen::Vector3d point = require_vec3(args, "point", -kLengthGuard, kLengthGuard);

    const Eigen::Isometry3d T = spatial::transform_from_rpy_p(rpy, p);
    const Eigen::Matrix3d R = T.linear();
    const Eigen::Vector3d result = R * point + p;

    static const char* kComponents[3] = {"x", "y", "z"};
    std::vector<std::vector<json::Value>> rows;
    rows.reserve(4);
    for (Eigen::Index r = 0; r < 3; ++r) {
        rows.push_back({
            json::Value(kComponents[r]),
            json::Value(R(r, 0) * point.x()),
            json::Value(R(r, 1) * point.y()),
            json::Value(R(r, 2) * point.z()),
            json::Value(p[r]),
            json::Value(result[r]),
        });
    }
    // The homogeneous row: 0 0 0 1 keeps a point a point.
    rows.push_back({json::Value("w"), json::Value(0.0), json::Value(0.0), json::Value(0.0),
                    json::Value(1.0), json::Value(1.0)});

    json::Value out = json::Value::object();
    out.set("p_out", json::from_vec3(result));
    out.set("steps", json::from_table({"component", "R_i1 * x", "R_i2 * y", "R_i3 * z",
                                       "translation", "result"},
                                      rows));
    out.set("T", json::from_isometry(T));
    out.set("p_homogeneous",
            json::from_table({"x", "y", "z", "w"},
                             std::vector<std::vector<double>>{
                                 {result.x(), result.y(), result.z(), 1.0}}));
    return out;
}

[[nodiscard]] json::Value op_frame_of_joint(const json::Value& args) {
    const Eigen::Matrix<double, 6, 1> q =
        require_vec6(args, "q", -widest_joint_range(), widest_joint_range());

    const std::array<Eigen::Isometry3d, GP8_DOF> frames = link_frames(q);
    const Eigen::Isometry3d flange = frames[GP8_DOF - 1] * flange_correction();

    std::vector<Eigen::Isometry3d> frame_list(frames.begin(), frames.end());

    std::vector<Eigen::Vector3d> origins;
    origins.reserve(GP8_DOF + 2);
    origins.push_back(Eigen::Vector3d::Zero());  // the base, so the view has a root
    for (const auto& frame : frames) {
        origins.push_back(frame.translation());
    }
    origins.push_back(flange.translation());

    std::vector<std::vector<json::Value>> dh_rows;
    dh_rows.reserve(GP8_DOF);
    std::vector<std::vector<json::Value>> limit_rows;
    limit_rows.reserve(GP8_DOF);
    bool within_limits = true;

    for (std::size_t i = 0; i < GP8_DOF; ++i) {
        const DHParams& dh = GP8_DH[i];
        const double joint = q[static_cast<Eigen::Index>(i)];
        dh_rows.push_back({
            json::Value(GP8_AXIS_NAMES[i]),
            json::Value(dh.a),
            json::Value(dh.alpha),
            json::Value(dh.d),
            json::Value(dh.theta_offset),
            json::Value(joint + dh.theta_offset),
        });
        const bool ok = joint >= joint_min(i) && joint <= joint_max(i);
        within_limits = within_limits && ok;
        limit_rows.push_back({
            json::Value(GP8_AXIS_NAMES[i]),
            json::Value(joint_min(i)),
            json::Value(joint),
            json::Value(joint_max(i)),
            json::Value(ok),
        });
    }

    json::Value out = json::Value::object();
    out.set("frames", json::from_isometry_list(frame_list));
    out.set("origins", json::from_points(origins));
    out.set("T_flange", json::from_isometry(flange));
    out.set("flange_position", json::from_vec3(flange.translation()));
    out.set("dh", json::from_table({"axis", "a_m", "alpha_rad", "d_m", "theta_offset_rad",
                                    "theta_rad"},
                                   dh_rows));
    out.set("limits",
            json::from_table({"axis", "min_rad", "value_rad", "max_rad", "ok"}, limit_rows));
    out.set("within_limits", json::Value(within_limits));
    out.set("orthonormality_error",
            json::Value(spatial::orthonormality_error(Eigen::Matrix3d(flange.linear()))));
    return out;
}

}  // namespace

json::Value SpatialMathModule::invoke(std::string_view op, const json::Value& args) const {
    if (op == "rotation_from_rpy") {
        return op_rotation_from_rpy(args);
    }
    if (op == "rotation_from_euler") {
        return op_rotation_from_euler(args);
    }
    if (op == "rpy_from_rotation") {
        return op_rpy_from_rotation(args);
    }
    if (op == "compose") {
        return op_compose(args);
    }
    if (op == "transform_point") {
        return op_transform_point(args);
    }
    if (op == "frame_of_joint") {
        return op_frame_of_joint(args);
    }
    unknown_op(name(), op);
}

}  // namespace yaskawa::study
