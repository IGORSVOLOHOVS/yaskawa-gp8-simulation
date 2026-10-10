#include "study/dh_kinematics.hpp"

#include "study/jacobian_statics.hpp"
#include "study/spatial_math.hpp"

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <limits>
#include <numbers>
#include <random>
#include <string>
#include <utility>
#include <vector>

namespace yaskawa::study {

namespace {

constexpr double kPi = std::numbers::pi;

// Angles wrap, so validation only guards against nonsense (a UI sending a
// pixel coordinate), not against a student typing 7 rad on purpose.
constexpr double kAngleGuard = 100.0;
constexpr double kLengthGuard = 10.0;  // [m], far outside any GP8 workspace

// Two joint vectors closer than this are the same branch, not two branches.
constexpr double kBranchDistinctness = 1e-9;

// |sin(q5)| below this leaves q4 and q6 coupled to within double precision,
// so the wrist flip collapses to a single solution.
constexpr double kWristDegenerate = 1e-8;

constexpr int kMaxWorkspaceSamples = 20000;
constexpr int kMaxScanSamples = 2001;
constexpr int kMaxIkIterations = 500;

[[nodiscard]] double wrap_pi(double angle) noexcept {
    double shifted = std::fmod(angle + kPi, 2.0 * kPi);
    if (shifted < 0.0) {
        shifted += 2.0 * kPi;
    }
    return shifted - kPi;
}

[[nodiscard]] dhk::JointVector read_q(const json::Value& args, std::string_view key = "q") {
    return require_vec6(args, key, -widest_joint_range(), widest_joint_range());
}

[[nodiscard]] dhk::JointVector optional_q(const json::Value& args, std::string_view key) {
    return optional_vec6(args, key, dhk::JointVector::Zero(), -widest_joint_range(),
                         widest_joint_range());
}

// Maximum horizontal distance the wrist centre can reach, derived from the DH
// table itself rather than quoted: max(u^2 + w^2) = a2^2 + a3^2 + d4^2 + |(M, N)|.
[[nodiscard]] double max_horizontal_reach() noexcept {
    const double a1 = GP8_DH[0].a;
    const double a2 = GP8_DH[1].a;
    const double a3 = GP8_DH[2].a;
    const double d4 = GP8_DH[3].d;
    const double amplitude = std::hypot(2.0 * a2 * a3, 2.0 * a2 * d4);
    return a1 + std::sqrt(a2 * a2 + a3 * a3 + d4 * d4 + amplitude);
}

[[nodiscard]] json::Value limits_table(const dhk::JointVector& q, bool& within) {
    std::vector<std::vector<json::Value>> rows;
    rows.reserve(GP8_DOF);
    within = true;
    for (std::size_t i = 0; i < GP8_DOF; ++i) {
        const double value = q(static_cast<Eigen::Index>(i));
        const bool ok = value >= joint_min(i) && value <= joint_max(i);
        within = within && ok;
        rows.push_back({
            json::Value(static_cast<int>(i + 1)),
            json::Value(GP8_AXIS_NAMES[i]),
            json::Value(joint_min(i)),
            json::Value(value),
            json::Value(joint_max(i)),
            json::Value(joint_max_velocity(i)),
            json::Value(ok),
        });
    }
    return json::from_table(
        {"i", "axis", "q_min_rad", "q_rad", "q_max_rad", "max_vel_rad_s", "within_limits"}, rows);
}

[[nodiscard]] Eigen::Isometry3d pose_from_args(const json::Value& args) {
    const Eigen::Vector3d p = require_vec3(args, "p", -kLengthGuard, kLengthGuard);
    const Eigen::Vector3d rpy = require_vec3(args, "rpy", -kAngleGuard, kAngleGuard);
    return spatial::transform_from_rpy_p(rpy, p);
}

// ---------------------------------------------------------------------------
// Ops
// ---------------------------------------------------------------------------

[[nodiscard]] json::Value op_dh_table(const json::Value& args) {
    const dhk::JointVector q = optional_q(args, "q");

    std::vector<std::vector<json::Value>> dh_rows;
    dh_rows.reserve(GP8_DOF);
    for (std::size_t i = 0; i < GP8_DOF; ++i) {
        const DHParams& dh = GP8_DH[i];
        const double joint = q(static_cast<Eigen::Index>(i));
        dh_rows.push_back({
            json::Value(static_cast<int>(i + 1)),
            json::Value(GP8_AXIS_NAMES[i]),
            json::Value(dh.a),
            json::Value(dh.alpha),
            json::Value(dh.d),
            json::Value(dh.theta_offset),
            json::Value(joint + dh.theta_offset),
        });
    }

    bool within = false;
    json::Value limits = limits_table(q, within);

    const double a2 = GP8_DH[1].a;
    const double a3 = GP8_DH[2].a;
    const double d4 = GP8_DH[3].d;
    const bool spherical =
        GP8_DH[3].a == 0.0 && GP8_DH[4].a == 0.0 && GP8_DH[5].a == 0.0 &&
        GP8_DH[4].d == 0.0 && GP8_DH[5].d == 0.0;

    json::Value out = json::Value::object();
    out.set("dh", json::from_table({"i", "axis", "a_m", "alpha_rad", "d_m", "theta_offset_rad",
                                    "theta_rad"},
                                   dh_rows));
    out.set("limits", std::move(limits));
    out.set("within_limits", json::Value(within));
    out.set("dof", json::Value(static_cast<int>(GP8_DOF)));
    out.set("spherical_wrist", json::Value(spherical));
    out.set("max_horizontal_reach_m", json::Value(max_horizontal_reach()));
    out.set("datasheet_reach_m", json::Value(GP8_HORIZONTAL_REACH_M));
    out.set("shoulder_offset_m", json::Value(GP8_DH[0].a));
    out.set("base_height_m", json::Value(GP8_DH[0].d));
    out.set("upper_arm_m", json::Value(a2));
    out.set("forearm_m", json::Value(std::hypot(a3, d4)));
    out.set("T_flange", json::from_isometry(forward_kinematics_dh(q)));
    out.set("convention",
            json::Value(std::string(
                "STANDARD (distal) Denavit-Hartenberg: A_i = Rot_z(theta_i) Trans_z(d_i) "
                "Trans_x(a_i) Rot_x(alpha_i), with theta_i = q_i + theta_offset_i and z_{i-1} "
                "along the axis of joint i. The modified (proximal, Craig) convention would order "
                "the same four factors as Rot_x(alpha_{i-1}) Trans_x(a_{i-1}) Rot_z(theta_i) "
                "Trans_z(d_i) and shift the a and alpha columns up one row; the table below is "
                "only correct read as standard DH. The GP8 wrist is spherical (a4 = a5 = a6 = "
                "d5 = d6 = 0), so the whole 0.340 m forearm appears as d4 and the flange origin "
                "coincides with the wrist centre, which is exactly why the closed-form inverse "
                "in inverse_kinematics exists.")));
    return out;
}

[[nodiscard]] json::Value op_forward_kinematics(const json::Value& args) {
    const dhk::JointVector q = read_q(args);

    const std::array<Eigen::Isometry3d, GP8_DOF> A = dhk::joint_transforms(q);
    const std::array<Eigen::Isometry3d, GP8_DOF> frames = link_frames(q);
    const Eigen::Isometry3d T_tool = forward_kinematics_dh(q);

    std::vector<Eigen::Isometry3d> frame_list(frames.begin(), frames.end());
    std::vector<Eigen::Isometry3d> a_list(A.begin(), A.end());

    std::vector<Eigen::Vector3d> origins;
    origins.reserve(GP8_DOF + 1);
    origins.push_back(Eigen::Vector3d::Zero());  // the base, so the view has a root
    for (const auto& frame : frames) {
        origins.push_back(frame.translation());
    }

    std::vector<std::vector<json::Value>> origin_rows;
    origin_rows.reserve(GP8_DOF);
    for (std::size_t i = 0; i < GP8_DOF; ++i) {
        const Eigen::Vector3d o = frames[i].translation();
        const Eigen::Vector3d z = frames[i].linear().col(2);
        origin_rows.push_back({
            json::Value(static_cast<int>(i + 1)),
            json::Value(GP8_AXIS_NAMES[i]),
            json::Value(o.x()),
            json::Value(o.y()),
            json::Value(o.z()),
            json::Value(z.x()),
            json::Value(z.y()),
            json::Value(z.z()),
        });
    }

    const yaskawa::YaskawaKinematics engine;
    const Eigen::Isometry3d T_engine = engine.forwardKinematics(q);

    bool within = false;
    json::Value limits = limits_table(q, within);

    const Eigen::Matrix3d R = T_tool.linear();

    json::Value out = json::Value::object();
    out.set("T_tool", json::from_isometry(T_tool));
    out.set("position", json::from_vec3(T_tool.translation()));
    out.set("rpy", json::from_vec3(spatial::rpy_from_rotation(R).primary));
    out.set("frames", json::from_isometry_list(frame_list));
    out.set("A", json::from_isometry_list(a_list));
    out.set("origins", json::from_points(origins));
    out.set("frame_table",
            json::from_table({"i", "axis", "o_x", "o_y", "o_z", "z_x", "z_y", "z_z"}, origin_rows));
    out.set("wrist_center", json::from_vec3(dhk::wrist_center(q)));
    out.set("limits", std::move(limits));
    out.set("within_limits", json::Value(within));
    out.set("engine_position_error",
            json::Value((T_tool.translation() - T_engine.translation()).norm()));
    out.set("engine_orientation_error",
            json::Value(dhk::orientation_error(R, Eigen::Matrix3d(T_engine.linear()))));
    out.set("orthonormality_error", json::Value(spatial::orthonormality_error(R)));
    out.set("verdict",
            json::Value(std::string(
                "frames holds T_0_i for i = 1..6 and A holds the individual link transforms, so "
                "the chain is readable one factor at a time: T_0_i = T_0_{i-1} A_i, and the tool "
                "pose is A_1 ... A_6 times the constant flange correction. engine_position_error "
                "and engine_orientation_error compare this DH product against the hand-written "
                "chain in yaskawa_kinematics.cpp; they are the proof that the table and the "
                "solver describe one robot, and anything above 1e-9 is a frame-assignment bug.")));
    return out;
}

[[nodiscard]] json::Value op_inverse_kinematics(const json::Value& args) {
    const Eigen::Isometry3d target = pose_from_args(args);
    const std::string method =
        optional_enum(args, "method", "both", {"geometric", "algebraic", "both"});
    const dhk::JointVector seed = optional_q(args, "q_seed");
    const int max_iters = optional_int(args, "max_iters", 100, 1, kMaxIkIterations);
    const double pos_tol = optional_scalar(args, "pos_tol", 1e-5, 1e-12, 0.1);
    const double rot_tol = optional_scalar(args, "rot_tol", 1e-5, 1e-12, 0.1);

    const bool want_geometric = (method != "algebraic");
    const bool want_numeric = (method != "geometric");

    json::Value out = json::Value::object();
    out.set("method", json::Value(method));
    out.set("target", json::from_isometry(target));

    dhk::IkGeometric geometric;
    if (want_geometric) {
        geometric = dhk::solve_ik_geometric(target);

        std::vector<std::vector<json::Value>> rows;
        rows.reserve(geometric.branches.size());
        Eigen::MatrixXd solutions(static_cast<Eigen::Index>(geometric.branches.size()), 6);
        double worst_position = 0.0;
        double worst_orientation = 0.0;
        for (std::size_t b = 0; b < geometric.branches.size(); ++b) {
            const dhk::IkBranch& branch = geometric.branches[b];
            std::vector<json::Value> row;
            row.reserve(13);
            row.emplace_back(static_cast<int>(b + 1));
            row.emplace_back(branch.shoulder > 0 ? "front" : "back");
            row.emplace_back(branch.elbow > 0 ? "up" : "down");
            row.emplace_back(branch.wrist > 0 ? "no-flip" : "flip");
            for (Eigen::Index i = 0; i < 6; ++i) {
                row.emplace_back(branch.q(i));
                solutions(static_cast<Eigen::Index>(b), i) = branch.q(i);
            }
            row.emplace_back(branch.position_error);
            row.emplace_back(branch.orientation_error);
            row.emplace_back(branch.within_limits);
            rows.push_back(std::move(row));
            worst_position = std::max(worst_position, branch.position_error);
            worst_orientation = std::max(worst_orientation, branch.orientation_error);
        }

        out.set("reachable", json::Value(geometric.reachable));
        out.set("solution_count", json::Value(static_cast<int>(geometric.branches.size())));
        out.set("within_limits_count", json::Value(static_cast<int>(geometric.within_limits)));
        out.set("branches",
                json::from_table({"branch", "shoulder", "elbow", "wrist", "q1", "q2", "q3", "q4",
                                  "q5", "q6", "position_error_m", "orientation_error_rad",
                                  "within_limits"},
                                 rows));
        out.set("q_solutions", json::from_matrix(solutions));
        out.set("worst_position_error", json::Value(worst_position));
        out.set("worst_orientation_error", json::Value(worst_orientation));
        out.set("required_cosine", json::Value(geometric.required_cosine));
        out.set("available_amplitude", json::Value(geometric.available_amplitude));
        out.set("geometric_note", json::Value(geometric.note));
    }

    if (want_numeric) {
        const yaskawa::YaskawaKinematics engine;
        dhk::JointVector q_numeric = dhk::JointVector::Zero();
        const bool converged =
            engine.inverseKinematics(target, seed, q_numeric, pos_tol, rot_tol, max_iters);

        // The engine does not report its iteration count, and forking the
        // solver to add one would duplicate it. It is deterministic, so the
        // smallest iteration budget that still converges IS the iteration
        // count, and that is what this loop measures.
        int iterations = max_iters;
        if (converged) {
            for (int budget = 1; budget <= max_iters; ++budget) {
                dhk::JointVector probe = dhk::JointVector::Zero();
                if (engine.inverseKinematics(target, seed, probe, pos_tol, rot_tol, budget)) {
                    iterations = budget;
                    q_numeric = probe;
                    break;
                }
            }
        }

        const Eigen::Isometry3d reached = forward_kinematics_dh(q_numeric);
        const double position_error = (reached.translation() - target.translation()).norm();
        const double rotation_error = dhk::orientation_error(Eigen::Matrix3d(reached.linear()),
                                                             Eigen::Matrix3d(target.linear()));

        int nearest = 0;
        double nearest_distance = std::numeric_limits<double>::infinity();
        for (std::size_t b = 0; b < geometric.branches.size(); ++b) {
            const double distance = (geometric.branches[b].q - q_numeric).norm();
            if (distance < nearest_distance) {
                nearest_distance = distance;
                nearest = static_cast<int>(b + 1);
            }
        }

        bool within = false;
        json::Value limits = limits_table(q_numeric, within);

        out.set("numeric_q", json::from_vec6(q_numeric));
        out.set("numeric_converged", json::Value(converged));
        out.set("numeric_iterations", json::Value(iterations));
        out.set("numeric_position_error", json::Value(position_error));
        out.set("numeric_orientation_error", json::Value(rotation_error));
        out.set("numeric_limits", std::move(limits));
        out.set("numeric_within_limits", json::Value(within));
        out.set("numeric_nearest_branch", json::Value(nearest));
        out.set("numeric_branch_distance",
                json::Value(std::isfinite(nearest_distance) ? nearest_distance : 0.0));
        if (!want_geometric) {
            out.set("reachable", json::Value(converged));
            out.set("solution_count", json::Value(converged ? 1 : 0));
        }
    }

    std::string verdict;
    if (want_geometric) {
        verdict = geometric.note;
        verdict +=
            " Every branch carries its own forward-kinematics round trip, so a branch that does "
            "not actually reach the target says so in position_error_m instead of looking like an "
            "answer.";
    }
    if (want_numeric) {
        verdict +=
            want_geometric
                ? " Levenberg-Marquardt is seeded from q_seed and therefore lands on exactly one "
                  "branch - whichever is nearest the seed - which is the practical difference "
                  "between the two methods: the closed form enumerates, the numeric method "
                  "chooses."
                : "Levenberg-Marquardt from yaskawa_kinematics.cpp, seeded from q_seed. It "
                  "returns one branch, the one nearest the seed, and the iteration count is the "
                  "smallest budget that still converges.";
    }
    out.set("verdict", json::Value(verdict));
    return out;
}

[[nodiscard]] json::Value op_workspace_sample(const json::Value& args) {
    const int samples = optional_int(args, "samples", 2000, 50, kMaxWorkspaceSamples);
    const std::string slice =
        optional_enum(args, "slice", "full_cloud", {"full_cloud", "xz_plane", "xy_plane"});
    const double height = optional_scalar(args, "height", 0.675, -0.5, 1.5);

    std::vector<Eigen::Vector3d> points;
    points.reserve(static_cast<std::size_t>(samples));
    std::string note;

    dhk::JointVector q = dhk::JointVector::Zero();

    if (slice == "full_cloud") {
        // A fixed seed, so the same request always returns the same cloud and
        // a reach number read off it is reproducible.
        std::mt19937 generator(20260101U);
        std::uniform_real_distribution<double> unit(0.0, 1.0);
        for (int s = 0; s < samples; ++s) {
            for (std::size_t j = 0; j < 3; ++j) {
                const double low = joint_min(j);
                const double high = joint_max(j);
                q(static_cast<Eigen::Index>(j)) = low + unit(generator) * (high - low);
            }
            points.push_back(dhk::wrist_center(q));
        }
        note =
            "A uniform random sample of q1, q2 and q3 inside the GP8 joint limits. Only those "
            "three move the flange: the wrist is spherical with d6 = 0, so q4, q5 and q6 change "
            "the tool orientation and leave its position exactly where it was.";
    } else if (slice == "xz_plane") {
        const int side = std::max(4, static_cast<int>(std::sqrt(static_cast<double>(samples))));
        for (int i = 0; i < side; ++i) {
            const double t2 = static_cast<double>(i) / static_cast<double>(side - 1);
            q(1) = joint_min(1) + t2 * (joint_max(1) - joint_min(1));
            for (int k = 0; k < side; ++k) {
                const double t3 = static_cast<double>(k) / static_cast<double>(side - 1);
                q(2) = joint_min(2) + t3 * (joint_max(2) - joint_min(2));
                q(0) = 0.0;
                points.push_back(dhk::wrist_center(q));
            }
        }
        note =
            "q1 pinned to zero and a regular grid over q2 and q3, so every point has y = 0 and "
            "the cloud is the cross-section of the envelope. This is the slice the datasheet "
            "envelope drawing shows.";
    } else {
        // Scan the shoulder-elbow pair for configurations that land on the
        // requested height, then spin the result about the base axis.
        constexpr int kScanSide = 160;
        const double tolerance = 0.01;  // [m] half-thickness of the slab
        double radius_min = std::numeric_limits<double>::infinity();
        double radius_max = -std::numeric_limits<double>::infinity();
        int in_slab = 0;
        for (int i = 0; i < kScanSide; ++i) {
            const double t2 = static_cast<double>(i) / static_cast<double>(kScanSide - 1);
            q(1) = joint_min(1) + t2 * (joint_max(1) - joint_min(1));
            for (int k = 0; k < kScanSide; ++k) {
                const double t3 = static_cast<double>(k) / static_cast<double>(kScanSide - 1);
                q(2) = joint_min(2) + t3 * (joint_max(2) - joint_min(2));
                q(0) = 0.0;
                const Eigen::Vector3d p = dhk::wrist_center(q);
                if (std::abs(p.z() - height) <= tolerance) {
                    ++in_slab;
                    const double radius = std::hypot(p.x(), p.y());
                    radius_min = std::min(radius_min, radius);
                    radius_max = std::max(radius_max, radius);
                }
            }
        }
        if (in_slab == 0) {
            note = "No configuration of q2 and q3 puts the flange within 10 mm of z = " +
                   json::number_to_string(height) +
                   " m, so that height is outside the reachable envelope and the slice is empty.";
        } else {
            const int angle_steps =
                std::max(8, static_cast<int>(std::sqrt(static_cast<double>(samples))));
            const int radial_steps = std::max(2, samples / angle_steps);
            for (int a = 0; a < angle_steps; ++a) {
                const double ta = static_cast<double>(a) / static_cast<double>(angle_steps - 1);
                const double angle = joint_min(0) + ta * (joint_max(0) - joint_min(0));
                for (int r = 0; r < radial_steps; ++r) {
                    const double tr =
                        (radial_steps == 1) ? 0.0
                                            : static_cast<double>(r) /
                                                  static_cast<double>(radial_steps - 1);
                    const double radius = radius_min + tr * (radius_max - radius_min);
                    points.emplace_back(radius * std::cos(angle), radius * std::sin(angle),
                                        height);
                }
            }
            note =
                "The shoulder-elbow pair was scanned for configurations within 10 mm of z = " +
                json::number_to_string(height) +
                " m, giving a reachable radius band of [" + json::number_to_string(radius_min) +
                ", " + json::number_to_string(radius_max) +
                "] m, which is then swept over the q1 range. The slice is a sector rather than a "
                "full annulus because q1 stops at +/-2.967 rad, not +/-pi.";
        }
    }

    double max_radius = 0.0;
    double max_distance = 0.0;
    double min_z = std::numeric_limits<double>::infinity();
    double max_z = -std::numeric_limits<double>::infinity();
    double min_radius = std::numeric_limits<double>::infinity();
    for (const auto& p : points) {
        const double radius = std::hypot(p.x(), p.y());
        max_radius = std::max(max_radius, radius);
        min_radius = std::min(min_radius, radius);
        max_distance = std::max(max_distance, p.norm());
        min_z = std::min(min_z, p.z());
        max_z = std::max(max_z, p.z());
    }
    if (points.empty()) {
        min_z = 0.0;
        max_z = 0.0;
        min_radius = 0.0;
    }

    json::Value out = json::Value::object();
    out.set("points", json::from_points(points));
    out.set("point_count", json::Value(static_cast<int>(points.size())));
    out.set("slice", json::Value(slice));
    out.set("max_horizontal_reach_m", json::Value(max_radius));
    out.set("min_horizontal_reach_m", json::Value(min_radius));
    out.set("max_distance_from_base_m", json::Value(max_distance));
    out.set("min_z_m", json::Value(min_z));
    out.set("max_z_m", json::Value(max_z));
    out.set("theoretical_reach_m", json::Value(max_horizontal_reach()));
    out.set("datasheet_reach_m", json::Value(GP8_HORIZONTAL_REACH_M));
    out.set("extremes",
            json::from_table({"quantity", "value_m"},
                             std::vector<std::vector<json::Value>>{
                                 {json::Value("max horizontal radius"), json::Value(max_radius)},
                                 {json::Value("min horizontal radius"), json::Value(min_radius)},
                                 {json::Value("max distance from base"),
                                  json::Value(max_distance)},
                                 {json::Value("lowest z"), json::Value(min_z)},
                                 {json::Value("highest z"), json::Value(max_z)},
                                 {json::Value("DH table maximum"),
                                  json::Value(max_horizontal_reach())},
                             }));
    out.set("verdict", json::Value(note));
    return out;
}

[[nodiscard]] json::Value op_singularity_scan(const json::Value& args) {
    const dhk::JointVector base = optional_q(args, "q");
    const int joint = optional_int(args, "joint", 5, 1, static_cast<int>(GP8_DOF));
    const int samples = optional_int(args, "samples", 361, 11, kMaxScanSamples);
    const double threshold = optional_scalar(args, "threshold", 0.02, 1e-9, 1.0);

    const std::size_t index = static_cast<std::size_t>(joint - 1);
    const double low = joint_min(index);
    const double high = joint_max(index);

    std::vector<double> angles;
    std::vector<double> determinants;
    std::vector<double> sigma_mins;
    angles.reserve(static_cast<std::size_t>(samples));
    determinants.reserve(static_cast<std::size_t>(samples));
    sigma_mins.reserve(static_cast<std::size_t>(samples));

    std::vector<std::vector<json::Value>> crossings;
    double best_sigma = std::numeric_limits<double>::infinity();
    double best_angle = low;

    dhk::JointVector q = base;
    for (int s = 0; s < samples; ++s) {
        const double t = static_cast<double>(s) / static_cast<double>(samples - 1);
        const double angle = low + t * (high - low);
        q(static_cast<Eigen::Index>(index)) = angle;

        const jac::Jacobian J = jac::geometric_jacobian(q);
        const double determinant = J.determinant();
        const double sigma_min = jac::singular_values(J)(5);

        angles.push_back(angle);
        determinants.push_back(determinant);
        sigma_mins.push_back(sigma_min);

        if (sigma_min < best_sigma) {
            best_sigma = sigma_min;
            best_angle = angle;
        }
        if (sigma_min < threshold) {
            const dhk::SingularityVerdict verdict = dhk::classify_singularity(q, threshold);
            crossings.push_back({
                json::Value(angle),
                json::Value(determinant),
                json::Value(sigma_min),
                json::Value(verdict.type),
                json::Value(verdict.wrist_measure),
                json::Value(verdict.elbow_measure),
                json::Value(verdict.shoulder_measure),
            });
        }
    }

    q(static_cast<Eigen::Index>(index)) = best_angle;
    const dhk::SingularityVerdict worst = dhk::classify_singularity(q, threshold);

    std::string text = "Sweeping axis ";
    text += GP8_AXIS_NAMES[index];
    text += " across its full range [" + json::number_to_string(low) + ", " +
            json::number_to_string(high) + "] rad, the smallest singular value bottoms out at " +
            json::number_to_string(best_sigma) + " at q" + std::to_string(joint) + " = " +
            json::number_to_string(best_angle) + " rad. ";
    if (crossings.empty()) {
        text +=
            "It never drops below the threshold, so this sweep stays away from a singularity. "
            "The determinant is the wrong instrument to watch for this: it is a product of six "
            "gains and shrinks for reasons that have nothing to do with rank, which is why the "
            "smallest singular value is plotted beside it. ";
    } else {
        text += "It crosses the threshold at " + std::to_string(crossings.size()) +
                " of the " + std::to_string(samples) +
                " sampled angles, and the determinant collapses with it. ";
    }
    text += worst.text;

    json::Value out = json::Value::object();
    out.set("determinant_series", json::from_series("det J(q)", angles, determinants));
    out.set("sigma_min_series", json::from_series("smallest singular value", angles, sigma_mins));
    out.set("joint", json::Value(joint));
    out.set("axis", json::Value(GP8_AXIS_NAMES[index]));
    out.set("threshold", json::Value(threshold));
    out.set("singular_angles",
            json::from_table({"q_rad", "det_J", "sigma_min", "type", "wrist_measure",
                              "elbow_measure", "shoulder_measure"},
                             crossings));
    out.set("singular_angle_count", json::Value(static_cast<int>(crossings.size())));
    out.set("worst_angle_rad", json::Value(best_angle));
    out.set("worst_sigma_min", json::Value(best_sigma));
    out.set("singularity_type", json::Value(worst.type));
    out.set("wrist_measure", json::Value(worst.wrist_measure));
    out.set("elbow_measure", json::Value(worst.elbow_measure));
    out.set("shoulder_measure", json::Value(worst.shoulder_measure));
    out.set("verdict", json::Value(text));
    return out;
}

}  // namespace

// ---------------------------------------------------------------------------
// dhk:: maps
// ---------------------------------------------------------------------------

namespace dhk {

std::array<Eigen::Isometry3d, GP8_DOF> joint_transforms(const JointVector& q) noexcept {
    std::array<Eigen::Isometry3d, GP8_DOF> out;
    for (std::size_t i = 0; i < GP8_DOF; ++i) {
        out[i] = dh_transform(GP8_DH[i], q(static_cast<Eigen::Index>(i)));
    }
    return out;
}

Eigen::Vector3d wrist_center(const JointVector& q) noexcept {
    // a4 = a5 = a6 = d5 = d6 = 0, so frames 4, 5 and 6 share one origin and it
    // is also the flange origin: the flange correction is a pure rotation.
    return link_frames(q)[3].translation();
}

double orientation_error(const Eigen::Matrix3d& A, const Eigen::Matrix3d& B) noexcept {
    const Eigen::AngleAxisd aa(Eigen::Matrix3d(A * B.transpose()));
    return std::abs(aa.angle());
}

IkGeometric solve_ik_geometric(const Eigen::Isometry3d& target) {
    IkGeometric out;

    // The flange correction is symmetric and its own inverse, so right
    // multiplying by it again undoes it and leaves the pure DH-chain target.
    const Eigen::Isometry3d target_dh = target * flange_correction();
    const Eigen::Vector3d p = target_dh.translation();
    const Eigen::Matrix3d R_target = target_dh.linear();

    const double a1 = GP8_DH[0].a;
    const double d1 = GP8_DH[0].d;
    const double a2 = GP8_DH[1].a;
    const double a3 = GP8_DH[2].a;
    const double d4 = GP8_DH[3].d;

    // u^2 + w^2 = a2^2 + a3^2 + d4^2 + M cos(q3) + N sin(q3), so the elbow
    // angle is the solution of an amplitude-phase equation and the two acos
    // branches are the two elbow configurations.
    const double M = 2.0 * a2 * a3;
    const double N = -2.0 * a2 * d4;
    const double amplitude = std::hypot(M, N);
    const double phase = std::atan2(N, M);
    out.available_amplitude = amplitude;

    const double planar_radius = std::hypot(p.x(), p.y());
    const bool on_shoulder_axis = planar_radius < 1e-9;
    const double base_angle = on_shoulder_axis ? 0.0 : std::atan2(p.y(), p.x());

    bool too_far = false;
    bool too_close = false;

    for (int shoulder = 1; shoulder >= -1; shoulder -= 2) {
        if (on_shoulder_axis && shoulder < 0) {
            continue;  // both shoulder branches coincide on the axis itself
        }
        const double r = static_cast<double>(shoulder) * planar_radius;
        const double q1 = (shoulder > 0) ? base_angle : wrap_pi(base_angle + kPi);
        const double u = r - a1;
        const double w = p.z() - d1;
        const double K = u * u + w * w - a2 * a2 - a3 * a3 - d4 * d4;
        if (shoulder > 0) {
            out.required_cosine = std::abs(K);
        }
        if (std::abs(K) > amplitude) {
            too_far = too_far || (K > amplitude && shoulder > 0);
            too_close = too_close || (K < -amplitude && shoulder > 0);
            continue;
        }
        const double spread = std::acos(std::clamp(K / amplitude, -1.0, 1.0));

        for (int elbow = 1; elbow >= -1; elbow -= 2) {
            const double q3 = wrap_pi(phase + static_cast<double>(elbow) * spread);
            const double P = a2 + a3 * std::cos(q3) - d4 * std::sin(q3);
            const double Q = a3 * std::sin(q3) + d4 * std::cos(q3);
            const double denominator = P * P + Q * Q;
            if (denominator < 1e-12) {
                continue;  // the arm would have folded onto its own shoulder
            }
            const double q2 = std::atan2((P * u - Q * w) / denominator,
                                         (Q * u + P * w) / denominator);

            JointVector arm = JointVector::Zero();
            arm(0) = q1;
            arm(1) = q2;
            arm(2) = q3;
            const Eigen::Matrix3d R_0_3 = link_frames(arm)[2].linear();
            const Eigen::Matrix3d R_3_6 = R_0_3.transpose() * R_target;

            // R_3_6 = [ c4c5c6 - s4s6 ... ] with third column (-c4 s5, -s4 s5,
            // c5) and third row (s5 c6, -s5 s6, c5): a ZYZ-shaped wrist.
            const double sin_q5 = std::hypot(R_3_6(0, 2), R_3_6(1, 2));
            const bool degenerate = sin_q5 < kWristDegenerate;

            for (int wrist = 1; wrist >= -1; wrist -= 2) {
                if (degenerate && wrist < 0) {
                    continue;  // the flip produces the same joint vector
                }
                double q4 = 0.0;
                double q5 = 0.0;
                double q6 = 0.0;
                if (degenerate) {
                    // Only q4 +/- q6 is observable. Pin q4 to zero and say so
                    // through the wrist measure the caller can read back.
                    if (R_3_6(2, 2) >= 0.0) {
                        q5 = 0.0;
                        q6 = std::atan2(R_3_6(1, 0), R_3_6(0, 0));
                    } else {
                        q5 = kPi;
                        q6 = -std::atan2(-R_3_6(0, 1), -R_3_6(0, 0));
                    }
                } else {
                    const double s5 = static_cast<double>(wrist) * sin_q5;
                    q5 = std::atan2(s5, R_3_6(2, 2));
                    q4 = std::atan2(-R_3_6(1, 2) / s5, -R_3_6(0, 2) / s5);
                    q6 = std::atan2(-R_3_6(2, 1) / s5, R_3_6(2, 0) / s5);
                }

                IkBranch branch;
                branch.q << q1, q2, q3, wrap_pi(q4), wrap_pi(q5), wrap_pi(q6);
                branch.shoulder = shoulder;
                branch.elbow = elbow;
                branch.wrist = wrist;

                const Eigen::Isometry3d reached = forward_kinematics_dh(branch.q);
                branch.position_error = (reached.translation() - target.translation()).norm();
                branch.orientation_error = orientation_error(Eigen::Matrix3d(reached.linear()),
                                                             Eigen::Matrix3d(target.linear()));

                branch.within_limits = true;
                for (std::size_t i = 0; i < GP8_DOF; ++i) {
                    const double value = branch.q(static_cast<Eigen::Index>(i));
                    if (value < joint_min(i) || value > joint_max(i)) {
                        branch.within_limits = false;
                        if (branch.first_violated_joint == GP8_DOF) {
                            branch.first_violated_joint = i;
                        }
                    }
                }

                bool duplicate = false;
                for (const auto& existing : out.branches) {
                    if ((existing.q - branch.q).cwiseAbs().maxCoeff() < kBranchDistinctness) {
                        duplicate = true;
                        break;
                    }
                }
                if (!duplicate) {
                    if (branch.within_limits) {
                        ++out.within_limits;
                    }
                    out.branches.push_back(branch);
                }
            }
        }
    }

    out.reachable = !out.branches.empty();
    if (out.reachable) {
        out.note = "The closed form found " + std::to_string(out.branches.size()) +
                   " distinct solution branches (shoulder front/back x elbow up/down x wrist "
                   "flip), " + std::to_string(out.within_limits) +
                   " of which lie inside the GP8 joint limits.";
    } else if (too_far) {
        out.note =
            "Unreachable: the target is beyond the outer envelope. The law-of-cosines term |K| = " +
            json::number_to_string(out.required_cosine) + " exceeds the available amplitude " +
            json::number_to_string(out.available_amplitude) +
            ", which means no elbow angle puts the wrist centre that far from the shoulder. No "
            "joint vector is returned, because there is none.";
    } else if (too_close) {
        out.note =
            "Unreachable: the target is inside the hollow around the base. The law-of-cosines "
            "term K = -" + json::number_to_string(out.required_cosine) +
            " is below -" + json::number_to_string(out.available_amplitude) +
            ", so the arm cannot fold tightly enough to put the wrist centre there.";
    } else {
        out.note =
            "Unreachable: the shoulder-elbow geometry degenerates at this target, so no "
            "closed-form branch exists. No joint vector is returned.";
    }
    return out;
}

SingularityVerdict classify_singularity(const JointVector& q, double threshold) noexcept {
    const double a3 = GP8_DH[2].a;
    const double d4 = GP8_DH[3].d;

    SingularityVerdict out;
    out.wrist_measure = std::abs(std::sin(q(4)));
    out.elbow_measure = std::abs(a3 * std::sin(q(2)) + d4 * std::cos(q(2)));
    const Eigen::Vector3d centre = wrist_center(q);
    out.shoulder_measure = std::hypot(centre.x(), centre.y());

    // Scale each measure onto a comparable 0..1 range so "which one is closest"
    // is a fair question: the wrist measure is already a sine, the elbow one is
    // a length up to |(a3, d4)|, the shoulder one a radius up to the full reach.
    const double elbow_scale = std::hypot(a3, d4);
    const double shoulder_scale = GP8_HORIZONTAL_REACH_M;
    const double wrist_scaled = out.wrist_measure;
    const double elbow_scaled = (elbow_scale > 0.0) ? out.elbow_measure / elbow_scale : 1.0;
    const double shoulder_scaled =
        (shoulder_scale > 0.0) ? out.shoulder_measure / shoulder_scale : 1.0;

    const double smallest = std::min({wrist_scaled, elbow_scaled, shoulder_scaled});
    if (smallest > threshold) {
        out.type = "none";
        out.text =
            "No singularity within the threshold: |sin(q5)| = " +
            json::number_to_string(out.wrist_measure) + ", the elbow measure |a3 sin(q3) + d4 "
            "cos(q3)| = " + json::number_to_string(out.elbow_measure) +
            " m, and the wrist centre sits " + json::number_to_string(out.shoulder_measure) +
            " m off the q1 axis, so all three degeneracies are some way off.";
        return out;
    }

    if (smallest == wrist_scaled) {
        out.type = "wrist";
        out.text =
            "WRIST singularity: q5 is at zero (or at pi), which lines axis 4 up with axis 6. The "
            "two joints then produce the same tool rotation, the Jacobian loses a row, and any "
            "commanded rotation about the lost direction asks for unbounded q4 and q6 in "
            "opposite directions. |sin(q5)| = " + json::number_to_string(out.wrist_measure) + ".";
    } else if (smallest == elbow_scaled) {
        out.type = "elbow";
        out.text =
            "ELBOW singularity: a3 sin(q3) + d4 cos(q3) has reached zero, which is exactly the "
            "stretched (or fully folded) arm, where the wrist centre sits at an extremum of its "
            "distance from the shoulder. Radial motion away from the shoulder is no longer "
            "available at any joint rate. Measure = " +
            json::number_to_string(out.elbow_measure) + " m.";
    } else {
        out.type = "shoulder";
        out.text =
            "SHOULDER singularity: the wrist centre has arrived on the axis of joint 1, only " +
            json::number_to_string(out.shoulder_measure) +
            " m off it, so turning the base no longer moves the tool and q1 is undetermined. The "
            "closed-form inverse reports this as the two shoulder branches collapsing into one.";
    }
    return out;
}

}  // namespace dhk

// ---------------------------------------------------------------------------
// Self-description
// ---------------------------------------------------------------------------

ModuleDescription DhKinematicsModule::describe() const {
    ModuleDescription d;
    d.name = "dh_kinematics";
    d.title = "DH Convention, Forward and Inverse Kinematics";
    d.course = CourseRef{3883, "M-407-01", "Robotics Modelling"};
    d.topics = {
        "Block 2 · Kinematic Chains: the DH Convention, Forward and Inverse Kinematics",
        "Course 3286 · Linear Algebra: matrix rank, the 2x2 solve for q2, amplitude conditions",
    };
    d.source = "cpp_solver/include/study/dh_kinematics.hpp";
    d.summary =
        "Reads the GP8 standard-DH table out loud, builds the forward kinematics as a visible "
        "product of link transforms, inverts it both in closed form (every branch at once) and "
        "numerically, samples the reachable workspace, and sweeps a joint to find where the arm "
        "loses a degree of freedom.";

    const Eigen::Matrix<double, 6, 1> home = Eigen::Matrix<double, 6, 1>::Zero();
    Eigen::Matrix<double, 6, 1> posed;
    posed << 0.0, 0.3, 0.6, 0.0, 0.8, 0.0;
    const double joint_span = widest_joint_range();
    const Eigen::Vector3d default_p(0.45, 0.0, 0.60);
    const Eigen::Vector3d default_rpy(0.0, 0.5, 0.0);

    {
        OpSpec op;
        op.name = "dh_table";
        op.title = "The GP8 standard-DH table";
        op.formula =
            "A_i = \\mathrm{Rot}_z(\\theta_i)\\,\\mathrm{Trans}_z(d_i)\\,"
            "\\mathrm{Trans}_x(a_i)\\,\\mathrm{Rot}_x(\\alpha_i),\\quad "
            "\\theta_i = q_i + \\theta_{0,i}";
        op.explain =
            "Four numbers per joint describe the whole arm: a_i and alpha_i are the length and "
            "twist of the rigid link, d_i and theta_i the offset and rotation at the joint. This "
            "is the STANDARD (distal) convention, where z_{i-1} is the axis of joint i; the "
            "modified Craig convention writes the same four factors in a different order and "
            "shifts the a and alpha columns one row, which is the single most common way to get "
            "an exam answer wrong. Read the GP8 off this table: a 0.330 m base, a 0.040 m "
            "shoulder offset, a 0.345 m upper arm, and a 0.340 m forearm that appears as d4 "
            "because the wrist is spherical. The theta_offset column exists so that q = 0 is the "
            "URDF home pose instead of an arbitrary DH zero.";
        op.params = {
            ParamSpec::vec6("q", "Joint vector", "rad", -joint_span, joint_span, home),
        };
        op.outputs = {
            OutputSpec::make("dh", "table", "i, a, alpha, d, theta_offset, theta"),
            OutputSpec::make("limits", "table", "Per-joint range and rated speed"),
            OutputSpec::make("within_limits", "bool", "Whether q is inside every range"),
            OutputSpec::make("dof", "int", "Degrees of freedom"),
            OutputSpec::make("spherical_wrist", "bool", "Whether axes 4, 5, 6 intersect"),
            OutputSpec::make("max_horizontal_reach_m", "scalar", "Reach implied by the table", "m"),
            OutputSpec::make("datasheet_reach_m", "scalar", "Reach Yaskawa publishes", "m"),
            OutputSpec::make("shoulder_offset_m", "scalar", "a1", "m"),
            OutputSpec::make("base_height_m", "scalar", "d1", "m"),
            OutputSpec::make("upper_arm_m", "scalar", "a2", "m"),
            OutputSpec::make("forearm_m", "scalar", "|(a3, d4)|", "m"),
            OutputSpec::make("T_flange", "mat4", "Tool pose at this q"),
            OutputSpec::make("convention", "text", "Standard versus modified DH, stated"),
        };
        d.ops.push_back(std::move(op));
    }

    {
        OpSpec op;
        op.name = "forward_kinematics";
        op.title = "Forward kinematics as a chain product";
        op.formula =
            "{}^{0}T_{6}(q) = A_1(q_1)\\,A_2(q_2)\\cdots A_6(q_6),\\qquad "
            "{}^{0}T_{\\text{flange}} = {}^{0}T_{6}\\,C";
        op.explain =
            "Forward kinematics is one matrix product and nothing more: multiply the six link "
            "transforms in order and read the tool pose off the result. This op returns the "
            "factors as well as the product, so you can stop after A_1 A_2 A_3 and see where the "
            "wrist centre has got to, which is how you check a hand derivation. The constant C at "
            "the end reconciles the DH frame 6 with the URDF flange frame: they share an origin "
            "but not an orientation. The two engine_* errors compare this product against the "
            "hand-written chain already in the repository, which is the only honest way to claim "
            "the table is right.";
        op.params = {
            ParamSpec::vec6("q", "Joint vector", "rad", -joint_span, joint_span, posed),
        };
        op.outputs = {
            OutputSpec::make("T_tool", "mat4", "Flange pose"),
            OutputSpec::make("position", "vec3", "Flange position", "m"),
            OutputSpec::make("rpy", "vec3", "Flange roll-pitch-yaw", "rad"),
            OutputSpec::make("frames", "mat4_set", "T_0_i for i = 1..6"),
            OutputSpec::make("A", "mat4_set", "The six link transforms A_i"),
            OutputSpec::make("origins", "points", "Base and link-frame origins, for the 3D view"),
            OutputSpec::make("frame_table", "table", "Each frame origin and its z axis"),
            OutputSpec::make("wrist_center", "vec3", "Centre of the spherical wrist", "m"),
            OutputSpec::make("limits", "table", "Per-joint range check"),
            OutputSpec::make("within_limits", "bool", "Whether q is inside every range"),
            OutputSpec::make("engine_position_error", "scalar", "Against the existing FK", "m"),
            OutputSpec::make("engine_orientation_error", "scalar", "Against the existing FK",
                             "rad"),
            OutputSpec::make("orthonormality_error", "scalar", "Drift of the tool rotation"),
            OutputSpec::make("verdict", "text", "How to read the factors"),
        };
        d.ops.push_back(std::move(op));
    }

    {
        OpSpec op;
        op.name = "inverse_kinematics";
        op.title = "Inverse kinematics: closed form and Levenberg-Marquardt";
        op.formula =
            "p_w = p_e,\\quad "
            "u^2 + w^2 = a_2^2 + a_3^2 + d_4^2 + 2a_2\\left(a_3\\cos q_3 - d_4 \\sin q_3\\right),"
            "\\qquad {}^{3}R_{6} = {}^{0}R_{3}^{T}\\,{}^{0}R_{6}";
        op.explain =
            "The GP8 wrist is spherical and d6 = 0, so the flange origin IS the wrist centre. "
            "That single fact splits the problem in half: the position depends only on q1, q2 and "
            "q3, and once those are fixed the remaining rotation is a three-angle wrist problem. "
            "The position half reduces to one amplitude equation in q3 - which is where "
            "reachability lives, since |K| must not exceed |(M, N)| - then a 2x2 linear solve for "
            "sin q2 and cos q2, and an atan2 for q1. Each stage has two roots, so a reachable "
            "pose has up to eight branches: shoulder front or back, elbow up or down, wrist "
            "flipped or not. The geometric method returns all of them with their joint-limit "
            "status; the algebraic method is the Levenberg-Marquardt solver already in "
            "yaskawa_kinematics.cpp, which converges to the one branch nearest its seed. Every "
            "returned solution is pushed back through forward kinematics, so a wrong answer "
            "cannot hide.";
        op.params = {
            ParamSpec::vec3("p", "Target position", "m", -1.5, 1.5, default_p),
            ParamSpec::vec3("rpy", "Target roll-pitch-yaw", "rad", -kPi, kPi, default_rpy),
            ParamSpec::enumeration("method", "Method", {"geometric", "algebraic", "both"}, "both"),
            ParamSpec::vec6("q_seed", "Seed for the numeric solver", "rad", -joint_span,
                            joint_span, home),
            ParamSpec::integer("max_iters", "Maximum numeric iterations", "", 1,
                               kMaxIkIterations, 100),
            ParamSpec::scalar("pos_tol", "Position tolerance", "m", 1e-9, 1e-2, 1e-5),
            ParamSpec::scalar("rot_tol", "Rotation tolerance", "rad", 1e-9, 1e-2, 1e-5),
        };
        op.outputs = {
            OutputSpec::make("reachable", "bool", "Whether any solution exists"),
            OutputSpec::make("solution_count", "int", "Distinct branches found"),
            OutputSpec::make("within_limits_count", "int", "Branches the robot can hold"),
            OutputSpec::make("branches", "table", "Every branch, its errors and its limit status"),
            OutputSpec::make("q_solutions", "matrix", "One joint vector per row, for ghost arms"),
            OutputSpec::make("worst_position_error", "scalar", "Worst branch round trip", "m"),
            OutputSpec::make("worst_orientation_error", "scalar", "Worst branch round trip", "rad"),
            OutputSpec::make("required_cosine", "scalar", "|K| in the reachability test"),
            OutputSpec::make("available_amplitude", "scalar", "The bound |K| must respect"),
            OutputSpec::make("geometric_note", "text", "What the closed form found, or why not"),
            OutputSpec::make("numeric_q", "vec6", "Levenberg-Marquardt solution", "rad"),
            OutputSpec::make("numeric_converged", "bool", "Whether it met the tolerances"),
            OutputSpec::make("numeric_iterations", "int", "Iterations it actually needed"),
            OutputSpec::make("numeric_position_error", "scalar", "Numeric round trip", "m"),
            OutputSpec::make("numeric_orientation_error", "scalar", "Numeric round trip", "rad"),
            OutputSpec::make("numeric_limits", "table", "Numeric solution against the limits"),
            OutputSpec::make("numeric_within_limits", "bool", "Numeric solution is holdable"),
            OutputSpec::make("numeric_nearest_branch", "int", "Which branch it landed on"),
            OutputSpec::make("numeric_branch_distance", "scalar", "Distance to that branch", "rad"),
            OutputSpec::make("target", "mat4", "The target pose, as built from p and rpy"),
            OutputSpec::make("verdict", "text", "Enumeration versus convergence"),
        };
        d.ops.push_back(std::move(op));
    }

    {
        OpSpec op;
        op.name = "workspace_sample";
        op.title = "Reachable workspace";
        op.formula =
            "\\mathcal{W} = \\left\\{\\,p_w(q_1, q_2, q_3) \\;:\\; "
            "q_i^{min} \\le q_i \\le q_i^{max} \\,\\right\\}";
        op.explain =
            "The workspace is the image of the joint box under forward kinematics, and for this "
            "robot only three joints contribute: the wrist is spherical with d6 = 0, so q4, q5 "
            "and q6 rotate the tool without moving it. Sampling the three shoulder-elbow joints "
            "therefore maps the whole reachable set. The cloud is worth more than the number: the "
            "outer envelope is where the elbow is stretched, the hollow near the base is where "
            "the arm cannot fold tightly enough, and the sector gap exists because q1 stops at "
            "+/-2.967 rad rather than +/-pi. Compare max_horizontal_reach_m against the 0.727 m "
            "Yaskawa publishes: the table was derived independently of the datasheet and lands on "
            "it.";
        op.params = {
            ParamSpec::integer("samples", "Sample count", "", 50, kMaxWorkspaceSamples, 2000),
            ParamSpec::enumeration("slice", "Slice", {"full_cloud", "xz_plane", "xy_plane"},
                                   "full_cloud"),
            ParamSpec::scalar("height", "Height of the XY slice", "m", -0.5, 1.5, 0.675),
        };
        op.outputs = {
            OutputSpec::make("points", "points", "Reachable flange positions", "m"),
            OutputSpec::make("point_count", "int", "How many points came back"),
            OutputSpec::make("slice", "text", "Which slice was sampled"),
            OutputSpec::make("max_horizontal_reach_m", "scalar", "Largest radius found", "m"),
            OutputSpec::make("min_horizontal_reach_m", "scalar", "Smallest radius found", "m"),
            OutputSpec::make("max_distance_from_base_m", "scalar", "Largest |p| found", "m"),
            OutputSpec::make("min_z_m", "scalar", "Lowest point reached", "m"),
            OutputSpec::make("max_z_m", "scalar", "Highest point reached", "m"),
            OutputSpec::make("theoretical_reach_m", "scalar", "Reach implied by the DH table", "m"),
            OutputSpec::make("datasheet_reach_m", "scalar", "Reach Yaskawa publishes", "m"),
            OutputSpec::make("extremes", "table", "The extremes actually found"),
            OutputSpec::make("verdict", "text", "What this slice is and how it was built"),
        };
        d.ops.push_back(std::move(op));
    }

    {
        OpSpec op;
        op.name = "singularity_scan";
        op.title = "Sweep one joint and watch the arm lose a degree of freedom";
        op.formula =
            "\\det J(q) = \\prod_{i=1}^{6}\\sigma_i(q),\\qquad "
            "\\sigma_6(q) \\to 0 \\iff \\mathrm{rank}\\,J < 6";
        op.explain =
            "A singularity is a configuration where the Jacobian drops rank, so some direction of "
            "tool motion stops being available at any joint rate. Sweeping one joint across its "
            "whole range and plotting both the determinant and the smallest singular value shows "
            "the difference between the two instruments: the determinant is a product of six "
            "gains and shrinks for reasons that have nothing to do with rank, while the smallest "
            "singular value is the actual distance to rank loss. The GP8 has all three classical "
            "degeneracies, and the op names which one it found rather than only printing a small "
            "number: the WRIST singularity at sin(q5) = 0, where axes 4 and 6 align; the ELBOW "
            "singularity where a3 sin(q3) + d4 cos(q3) = 0 and the arm is stretched; and the "
            "SHOULDER singularity where the wrist centre arrives on the axis of joint 1.";
        op.params = {
            ParamSpec::vec6("q", "Base configuration", "rad", -joint_span, joint_span, posed),
            ParamSpec::integer("joint", "Joint to sweep", "", 1, static_cast<int>(GP8_DOF), 5),
            ParamSpec::integer("samples", "Samples across the range", "", 11, kMaxScanSamples,
                               361),
            ParamSpec::scalar("threshold", "Singularity threshold", "", 1e-9, 1.0, 0.02),
        };
        op.outputs = {
            OutputSpec::make("determinant_series", "series", "det J against the swept angle"),
            OutputSpec::make("sigma_min_series", "series", "Smallest singular value against it"),
            OutputSpec::make("joint", "int", "Which joint was swept"),
            OutputSpec::make("axis", "text", "Its Yaskawa axis letter"),
            OutputSpec::make("threshold", "scalar", "The threshold applied"),
            OutputSpec::make("singular_angles", "table", "Angles below the threshold, classified"),
            OutputSpec::make("singular_angle_count", "int", "How many samples crossed it"),
            OutputSpec::make("worst_angle_rad", "scalar", "Angle of the worst conditioning", "rad"),
            OutputSpec::make("worst_sigma_min", "scalar", "Smallest singular value reached"),
            OutputSpec::make("singularity_type", "text", "wrist, elbow, shoulder or none"),
            OutputSpec::make("wrist_measure", "scalar", "|sin(q5)|"),
            OutputSpec::make("elbow_measure", "scalar", "|a3 sin(q3) + d4 cos(q3)|", "m"),
            OutputSpec::make("shoulder_measure", "scalar", "Wrist-centre offset from the q1 axis",
                             "m"),
            OutputSpec::make("verdict", "text", "Which singularity this is, named"),
        };
        d.ops.push_back(std::move(op));
    }

    return d;
}

json::Value DhKinematicsModule::invoke(std::string_view op, const json::Value& args) const {
    if (op == "dh_table") {
        return op_dh_table(args);
    }
    if (op == "forward_kinematics") {
        return op_forward_kinematics(args);
    }
    if (op == "inverse_kinematics") {
        return op_inverse_kinematics(args);
    }
    if (op == "workspace_sample") {
        return op_workspace_sample(args);
    }
    if (op == "singularity_scan") {
        return op_singularity_scan(args);
    }
    unknown_op(name(), op);
}

}  // namespace yaskawa::study
