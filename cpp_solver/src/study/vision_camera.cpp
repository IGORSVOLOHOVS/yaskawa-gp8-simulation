#include "study/vision_camera.hpp"

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
constexpr int kSeedMin = 0;
constexpr int kSeedMax = 1000000;
constexpr std::size_t kMaxTargets = 64;

// The round-trip tolerance this module promises: a noiseless projection fed
// back through pose_estimate reproduces its own pixels to better than this.
constexpr double kPoseRoundTripTolerancePx = 0.05;

const std::vector<std::string> kMountings = {"forward_tool", "flange_aligned", "tilted_30"};

[[nodiscard]] Eigen::Matrix<double, 6, 1> default_configuration() noexcept {
    return Eigen::Matrix<double, 6, 1>::Zero();
}

[[nodiscard]] std::vector<Eigen::Vector3d> default_targets() {
    // Five fixture points ahead of the q = 0 flange plus one deliberately
    // outside the field of view, so the panel always shows both verdicts.
    return {
        Eigen::Vector3d(0.90, 0.00, 0.735), Eigen::Vector3d(0.85, 0.08, 0.780),
        Eigen::Vector3d(0.80, -0.10, 0.700), Eigen::Vector3d(1.05, 0.06, 0.700),
        Eigen::Vector3d(0.70, 0.00, 0.790), Eigen::Vector3d(0.60, 0.40, 0.700),
    };
}

[[nodiscard]] json::Value targets_default_table() {
    const std::vector<Eigen::Vector3d> points = default_targets();
    std::vector<std::vector<double>> rows;
    rows.reserve(points.size());
    for (const auto& p : points) {
        rows.push_back({p.x(), p.y(), p.z()});
    }
    return json::from_table({"x", "y", "z"}, rows);
}

// Accepts [[x,y,z], ...] or a `table` value with a "rows" array, which is what
// the UI's editable table sends back.
[[nodiscard]] std::vector<Eigen::Vector3d> read_targets(const json::Value& args) {
    if (is_absent(args, "targets")) {
        return default_targets();
    }
    const json::Value& value = args["targets"];
    const json::Value* rows = nullptr;
    if (value.is_array()) {
        rows = &value;
    } else if (value.is_object() && value["rows"].is_array()) {
        rows = &value["rows"];
    } else {
        throw StudyError(
            "parameter 'targets' must be a list of [x, y, z] rows or a table with a 'rows' array");
    }
    if (rows->size() == 0) {
        throw StudyError("parameter 'targets' needs at least 1 point, received 0");
    }
    if (rows->size() > kMaxTargets) {
        throw StudyError("parameter 'targets' accepts at most " + std::to_string(kMaxTargets) +
                         " points, received " + std::to_string(rows->size()));
    }
    std::vector<Eigen::Vector3d> out;
    out.reserve(rows->size());
    for (std::size_t i = 0; i < rows->size(); ++i) {
        const json::Value& row = (*rows)[i];
        if (!row.is_array() || row.size() != 3) {
            throw StudyError("parameter 'targets' row " + std::to_string(i) +
                             " must hold 3 numbers");
        }
        Eigen::Vector3d p;
        for (std::size_t k = 0; k < 3; ++k) {
            const json::Value& cell = row[k];
            if (!cell.is_number() || !std::isfinite(cell.as_double()) ||
                std::abs(cell.as_double()) > 10.0) {
                throw StudyError("parameter 'targets' element [" + std::to_string(i) + "][" +
                                 std::to_string(k) + "] must be a finite coordinate within 10 m");
            }
            p[static_cast<Eigen::Index>(k)] = cell.as_double();
        }
        out.push_back(p);
    }
    return out;
}

struct CameraSetup {
    vision::Intrinsics K;
    Eigen::Matrix<double, 6, 1> q = Eigen::Matrix<double, 6, 1>::Zero();
    Eigen::Isometry3d flange_to_camera = Eigen::Isometry3d::Identity();
    Eigen::Isometry3d base_to_flange = Eigen::Isometry3d::Identity();
    Eigen::Isometry3d base_to_camera = Eigen::Isometry3d::Identity();
    Eigen::Isometry3d camera_to_base = Eigen::Isometry3d::Identity();
    std::string mounting;
};

[[nodiscard]] CameraSetup read_camera(const json::Value& args) {
    CameraSetup setup;
    const double focal = optional_scalar(args, "focal_length", 8.0, 1.0, 200.0);
    const double sensor_width = optional_scalar(args, "sensor_width", 6.4, 0.5, 60.0);
    const double sensor_height = optional_scalar(args, "sensor_height", 4.8, 0.5, 60.0);
    const int width = optional_int(args, "width_px", 1280, 64, 4096);
    const int height = optional_int(args, "height_px", 960, 64, 4096);
    setup.K = vision::intrinsics_from_sensor(focal, sensor_width, sensor_height, width, height);
    setup.mounting = optional_enum(args, "mounting", "forward_tool", kMountings);
    setup.q = optional_vec6(args, "q", default_configuration(), -widest_joint_range(),
                            widest_joint_range());
    setup.flange_to_camera = vision::hand_eye_transform(setup.mounting);
    setup.base_to_flange = forward_kinematics_dh(setup.q);
    setup.base_to_camera = setup.base_to_flange * setup.flange_to_camera;
    setup.camera_to_base = setup.base_to_camera.inverse();
    return setup;
}

[[nodiscard]] std::vector<ParamSpec> camera_params() {
    return {
        ParamSpec::scalar("focal_length", "Lens focal length", "mm", 1.0, 200.0, 8.0),
        ParamSpec::scalar("sensor_width", "Sensor width", "mm", 0.5, 60.0, 6.4),
        ParamSpec::scalar("sensor_height", "Sensor height", "mm", 0.5, 60.0, 4.8),
        ParamSpec::integer("width_px", "Image width", "px", 64, 4096, 1280),
        ParamSpec::integer("height_px", "Image height", "px", 64, 4096, 960),
        ParamSpec::enumeration("mounting", "Hand-eye mounting", kMountings, "forward_tool"),
        ParamSpec::vec6("q", "Joint configuration", "rad", -widest_joint_range(),
                        widest_joint_range(), default_configuration()),
    };
}

[[nodiscard]] json::Value intrinsics_table(const vision::Intrinsics& K) {
    return json::from_table({"fx_px", "fy_px", "cx_px", "cy_px", "width_px", "height_px"},
                            std::vector<std::vector<double>>{{K.fx, K.fy, K.cx, K.cy,
                                                              static_cast<double>(K.width),
                                                              static_cast<double>(K.height)}});
}

// ---------------------------------------------------------------------------
// camera_model
// ---------------------------------------------------------------------------

[[nodiscard]] json::Value op_camera_model(const json::Value& args) {
    const CameraSetup setup = read_camera(args);
    const double near_plane = optional_scalar(args, "near_plane", 0.08, 0.005, 5.0);
    const double far_plane = optional_scalar(args, "far_plane", 0.80, 0.01, 20.0);
    if (far_plane <= near_plane) {
        throw StudyError("parameter 'far_plane' = " + json::number_to_string(far_plane) +
                         " must exceed 'near_plane' = " + json::number_to_string(near_plane));
    }

    const Eigen::Matrix3d K = setup.K.matrix();
    const Eigen::Matrix3d R_cb = setup.camera_to_base.linear();
    const Eigen::Vector3d t_cb = setup.camera_to_base.translation();

    Eigen::Matrix<double, 3, 4> extrinsic;
    extrinsic.block<3, 3>(0, 0) = R_cb;
    extrinsic.block<3, 1>(0, 3) = t_cb;
    const Eigen::Matrix<double, 3, 4> projection = K * extrinsic;

    // Frustum: the optical centre plus the four corner rays at both planes,
    // all expressed in the base frame so the 3D view can draw the cone.
    const std::array<std::pair<double, double>, 4> corners = {{
        {0.0, 0.0},
        {static_cast<double>(setup.K.width), 0.0},
        {static_cast<double>(setup.K.width), static_cast<double>(setup.K.height)},
        {0.0, static_cast<double>(setup.K.height)},
    }};
    std::vector<Eigen::Vector3d> frustum;
    frustum.reserve(9);
    frustum.push_back(setup.base_to_camera.translation());
    for (const double plane : {near_plane, far_plane}) {
        for (const auto& corner : corners) {
            const Eigen::Vector3d ray((corner.first - setup.K.cx) / setup.K.fx,
                                      (corner.second - setup.K.cy) / setup.K.fy, 1.0);
            frustum.push_back(setup.base_to_camera * (plane * ray));
        }
    }

    const double fov_h = 2.0 * std::atan(0.5 * static_cast<double>(setup.K.width) / setup.K.fx);
    const double fov_v = 2.0 * std::atan(0.5 * static_cast<double>(setup.K.height) / setup.K.fy);

    const std::vector<std::vector<json::Value>> chain_rows = {
        {json::Value("0_T_e  flange from the DH chain"),
         json::Value(setup.base_to_flange.translation().x()),
         json::Value(setup.base_to_flange.translation().y()),
         json::Value(setup.base_to_flange.translation().z())},
        {json::Value("e_T_c  hand-eye (" + setup.mounting + ")"),
         json::Value(setup.flange_to_camera.translation().x()),
         json::Value(setup.flange_to_camera.translation().y()),
         json::Value(setup.flange_to_camera.translation().z())},
        {json::Value("0_T_c  camera in the base frame"),
         json::Value(setup.base_to_camera.translation().x()),
         json::Value(setup.base_to_camera.translation().y()),
         json::Value(setup.base_to_camera.translation().z())},
    };

    json::Value out = json::Value::object();
    out.set("K", json::from_matrix(K));
    out.set("intrinsics", intrinsics_table(setup.K));
    out.set("T_flange_camera", json::from_isometry(setup.flange_to_camera));
    out.set("T_base_flange", json::from_isometry(setup.base_to_flange));
    out.set("T_base_camera", json::from_isometry(setup.base_to_camera));
    out.set("extrinsic", json::from_matrix(extrinsic));
    out.set("projection", json::from_matrix(projection));
    out.set("frustum", json::from_points(frustum));
    out.set("chain", json::from_table({"factor", "x", "y", "z"}, chain_rows));
    out.set("fov_horizontal", json::Value(fov_h));
    out.set("fov_vertical", json::Value(fov_v));
    out.set("fov_horizontal_deg", json::Value(fov_h * 180.0 / kPi));
    out.set("fov_vertical_deg", json::Value(fov_v * 180.0 / kPi));
    out.set("mounting", json::Value(setup.mounting));
    out.set("optical_axis_in_base",
            json::from_vec3(Eigen::Vector3d(setup.base_to_camera.linear().col(2))));
    out.set("note",
            json::Value("The extrinsic transform is the inverse of 0_T_c = 0_T_e e_T_c, so it is "
                        "the DH chain of this robot and the hand-eye transform - never a "
                        "hand-typed camera pose. Reorder those two factors and the predicted "
                        "pixel moves, which is exactly exam question Q1(a)."));
    return out;
}

// ---------------------------------------------------------------------------
// project_targets
// ---------------------------------------------------------------------------

struct TargetProjection {
    Eigen::Vector3d point_base{Eigen::Vector3d::Zero()};
    Eigen::Vector3d point_camera{Eigen::Vector3d::Zero()};
    vision::Projection ideal;
    vision::Projection distorted;
    bool occluded = false;
};

[[nodiscard]] std::vector<TargetProjection> project_all(const CameraSetup& setup,
                                                        const std::vector<Eigen::Vector3d>& targets,
                                                        double k1, double k2,
                                                        double occlusion_radius) {
    std::vector<TargetProjection> out;
    out.reserve(targets.size());
    for (const auto& p : targets) {
        TargetProjection item;
        item.point_base = p;
        item.point_camera = setup.camera_to_base * p;
        item.ideal = vision::project(setup.K, item.point_camera);
        item.distorted = vision::project_distorted(setup.K, item.point_camera, k1, k2);
        out.push_back(item);
    }
    // A target is occluded when a nearer visible target lands within the
    // occlusion radius of it in the image: the blob in front hides it.
    for (std::size_t i = 0; i < out.size(); ++i) {
        if (!(out[i].ideal.in_front && out[i].ideal.in_image)) {
            continue;
        }
        for (std::size_t j = 0; j < out.size(); ++j) {
            if (i == j || !(out[j].ideal.in_front && out[j].ideal.in_image)) {
                continue;
            }
            if (out[j].ideal.depth >= out[i].ideal.depth) {
                continue;
            }
            const double du = out[i].ideal.u - out[j].ideal.u;
            const double dv = out[i].ideal.v - out[j].ideal.v;
            if (std::sqrt(du * du + dv * dv) < occlusion_radius) {
                out[i].occluded = true;
                break;
            }
        }
    }
    return out;
}

[[nodiscard]] json::Value op_project_targets(const json::Value& args) {
    const CameraSetup setup = read_camera(args);
    const std::vector<Eigen::Vector3d> targets = read_targets(args);
    const double k1 = optional_scalar(args, "k1", -0.18, -1.0, 1.0);
    const double k2 = optional_scalar(args, "k2", 0.04, -1.0, 1.0);
    const double occlusion_radius = optional_scalar(args, "occlusion_radius", 18.0, 0.0, 500.0);

    const std::vector<TargetProjection> projected =
        project_all(setup, targets, k1, k2, occlusion_radius);

    std::vector<std::vector<json::Value>> rows;
    rows.reserve(projected.size());
    std::vector<Eigen::Vector3d> visible_points;
    std::size_t in_fov = 0;
    std::size_t occlusion_free = 0;
    double max_distortion_shift = 0.0;

    for (std::size_t i = 0; i < projected.size(); ++i) {
        const TargetProjection& t = projected[i];
        const bool visible = t.ideal.in_front && t.ideal.in_image;
        if (visible) {
            ++in_fov;
            visible_points.push_back(t.point_base);
            if (!t.occluded) {
                ++occlusion_free;
            }
        }
        const double shift =
            std::hypot(t.distorted.u - t.ideal.u, t.distorted.v - t.ideal.v);
        max_distortion_shift = std::max(max_distortion_shift, visible ? shift : 0.0);
        rows.push_back({json::Value(static_cast<double>(i)), json::Value(t.point_base.x()),
                        json::Value(t.point_base.y()), json::Value(t.point_base.z()),
                        json::Value(t.ideal.depth), json::Value(t.ideal.u),
                        json::Value(t.ideal.v), json::Value(t.distorted.u),
                        json::Value(t.distorted.v), json::Value(shift), json::Value(visible),
                        json::Value(t.occluded)});
    }

    json::Value out = json::Value::object();
    out.set("pixels",
            json::from_table({"index", "x_base", "y_base", "z_base", "depth_m", "u_ideal",
                              "v_ideal", "u_distorted", "v_distorted", "distortion_shift_px",
                              "in_fov", "occluded"},
                             rows));
    out.set("visible_points", json::from_points(visible_points));
    out.set("target_count", json::Value(static_cast<double>(targets.size())));
    out.set("in_fov_count", json::Value(static_cast<double>(in_fov)));
    out.set("occlusion_free_count", json::Value(static_cast<double>(occlusion_free)));
    out.set("max_distortion_shift_px", json::Value(max_distortion_shift));
    out.set("k1", json::Value(k1));
    out.set("k2", json::Value(k2));
    out.set("intrinsics", intrinsics_table(setup.K));
    out.set("T_base_camera", json::from_isometry(setup.base_to_camera));
    out.set("note",
            json::Value("Distortion is applied in normalised coordinates before the intrinsics "
                        "scale them, which is why the shift grows toward the image corners and "
                        "vanishes at the principal point. The undistorted pixels are the ones a "
                        "pose estimator must be fed; the distorted ones are what the sensor "
                        "actually reports."));
    return out;
}

// ---------------------------------------------------------------------------
// detect
// ---------------------------------------------------------------------------

struct Candidate {
    double u = 0.0;
    double v = 0.0;
    double edge_energy = 0.0;
    double shape_match = 0.0;
    double score = 0.0;
    bool is_target = false;
    std::size_t truth_index = 0;
};

// Greedy non-maximum suppression over a score-sorted candidate list.
[[nodiscard]] std::vector<std::size_t> suppress(const std::vector<Candidate>& candidates,
                                                const std::vector<std::size_t>& order,
                                                double radius) {
    std::vector<std::size_t> kept;
    kept.reserve(order.size());
    for (const std::size_t index : order) {
        bool blocked = false;
        for (const std::size_t other : kept) {
            const double du = candidates[index].u - candidates[other].u;
            const double dv = candidates[index].v - candidates[other].v;
            if (std::sqrt(du * du + dv * dv) < radius) {
                blocked = true;
                break;
            }
        }
        if (!blocked) {
            kept.push_back(index);
        }
    }
    return kept;
}

struct PrPoint {
    double precision = 1.0;
    double recall = 0.0;
    std::size_t true_positives = 0;
    std::size_t false_positives = 0;
};

[[nodiscard]] PrPoint evaluate(const std::vector<Candidate>& candidates,
                               const std::vector<std::size_t>& score_order, double threshold,
                               double nms_radius, std::size_t truth_count,
                               std::vector<std::size_t>* kept_out) {
    std::vector<std::size_t> filtered;
    filtered.reserve(score_order.size());
    for (const std::size_t index : score_order) {
        if (candidates[index].score >= threshold) {
            filtered.push_back(index);
        }
    }
    const std::vector<std::size_t> kept = suppress(candidates, filtered, nms_radius);
    if (kept_out != nullptr) {
        *kept_out = kept;
    }

    std::vector<bool> matched(truth_count, false);
    PrPoint point;
    for (const std::size_t index : kept) {
        const Candidate& c = candidates[index];
        if (c.is_target && c.truth_index < truth_count && !matched[c.truth_index]) {
            matched[c.truth_index] = true;
            ++point.true_positives;
        } else {
            ++point.false_positives;
        }
    }
    const std::size_t reported = point.true_positives + point.false_positives;
    point.precision = (reported > 0) ? static_cast<double>(point.true_positives) /
                                           static_cast<double>(reported)
                                     : 1.0;
    point.recall = (truth_count > 0) ? static_cast<double>(point.true_positives) /
                                           static_cast<double>(truth_count)
                                     : 0.0;
    return point;
}

[[nodiscard]] json::Value op_detect(const json::Value& args) {
    const CameraSetup setup = read_camera(args);
    const std::vector<Eigen::Vector3d> targets = read_targets(args);
    const double threshold = optional_scalar(args, "threshold", 0.55, 0.0, 1.0);
    const int clutter_count = optional_int(args, "clutter", 14, 0, 400);
    const double pixel_noise = optional_scalar(args, "pixel_noise", 1.5, 0.0, 50.0);
    const double nms_radius = optional_scalar(args, "nms_radius", 24.0, 1.0, 500.0);
    const double match_radius = optional_scalar(args, "match_radius", 12.0, 1.0, 500.0);
    const int seed = optional_int(args, "seed", 17, kSeedMin, kSeedMax);

    const std::vector<TargetProjection> projected = project_all(setup, targets, 0.0, 0.0, 0.0);
    sensing::SeededRng rng(seed);

    // Two explainable features per candidate. Edge energy is a real geometric
    // quantity - a nearer blob subtends more pixels and so carries more edge -
    // and shape match is the template score. Clutter draws both from lower
    // ranges that still OVERLAP the target ranges, which is what makes the
    // precision-recall trade real rather than decorative.
    constexpr double kReferenceDepth = 0.45;
    std::vector<Candidate> candidates;
    candidates.reserve(targets.size() + static_cast<std::size_t>(clutter_count));
    std::size_t truth_count = 0;

    for (std::size_t i = 0; i < projected.size(); ++i) {
        const TargetProjection& t = projected[i];
        if (!(t.ideal.in_front && t.ideal.in_image)) {
            continue;
        }
        Candidate c;
        c.u = t.ideal.u + pixel_noise * rng.gaussian();
        c.v = t.ideal.v + pixel_noise * rng.gaussian();
        c.edge_energy = kReferenceDepth / (kReferenceDepth + t.ideal.depth);
        c.shape_match = 0.80 + 0.20 * rng.uniform01();
        c.score = 0.5 * c.edge_energy + 0.5 * c.shape_match;
        c.is_target = true;
        c.truth_index = truth_count;
        candidates.push_back(c);
        ++truth_count;
    }

    for (int k = 0; k < clutter_count; ++k) {
        Candidate c;
        c.u = rng.uniform(0.0, static_cast<double>(setup.K.width));
        c.v = rng.uniform(0.0, static_cast<double>(setup.K.height));
        c.edge_energy = 0.15 + 0.50 * rng.uniform01();
        c.shape_match = 0.20 + 0.55 * rng.uniform01();
        c.score = 0.5 * c.edge_energy + 0.5 * c.shape_match;
        c.is_target = false;
        candidates.push_back(c);
    }

    // A clutter blob that happens to land on a target is a true positive: that
    // is how a real evaluation scores it, and pretending otherwise would
    // flatter the detector.
    for (auto& c : candidates) {
        if (c.is_target) {
            continue;
        }
        std::size_t truth_index = 0;
        for (std::size_t i = 0; i < projected.size(); ++i) {
            const TargetProjection& t = projected[i];
            if (!(t.ideal.in_front && t.ideal.in_image)) {
                continue;
            }
            if (std::hypot(c.u - t.ideal.u, c.v - t.ideal.v) < match_radius) {
                c.is_target = true;
                c.truth_index = truth_index;
                break;
            }
            ++truth_index;
        }
    }

    std::vector<std::size_t> score_order(candidates.size());
    for (std::size_t i = 0; i < candidates.size(); ++i) {
        score_order[i] = i;
    }
    std::stable_sort(score_order.begin(), score_order.end(),
                     [&candidates](std::size_t a, std::size_t b) {
                         return candidates[a].score > candidates[b].score;
                     });

    std::vector<std::size_t> kept;
    const PrPoint at_threshold =
        evaluate(candidates, score_order, threshold, nms_radius, truth_count, &kept);

    std::vector<std::vector<json::Value>> detection_rows;
    detection_rows.reserve(kept.size());
    for (const std::size_t index : kept) {
        const Candidate& c = candidates[index];
        detection_rows.push_back({json::Value(c.u), json::Value(c.v),
                                  json::Value(c.edge_energy), json::Value(c.shape_match),
                                  json::Value(c.score), json::Value(c.is_target)});
    }

    // The full sweep, which is the point of the panel: move the threshold and
    // watch precision trade against recall on identical input.
    const int sweep_steps = 51;
    std::vector<double> thresholds;
    std::vector<double> precisions;
    std::vector<double> recalls;
    std::vector<double> f1_scores;
    thresholds.reserve(static_cast<std::size_t>(sweep_steps));
    precisions.reserve(static_cast<std::size_t>(sweep_steps));
    recalls.reserve(static_cast<std::size_t>(sweep_steps));
    f1_scores.reserve(static_cast<std::size_t>(sweep_steps));
    double best_f1 = -1.0;
    double best_f1_threshold = 0.0;
    for (int s = 0; s < sweep_steps; ++s) {
        const double value = static_cast<double>(s) / static_cast<double>(sweep_steps - 1);
        const PrPoint point =
            evaluate(candidates, score_order, value, nms_radius, truth_count, nullptr);
        thresholds.push_back(value);
        precisions.push_back(point.precision);
        recalls.push_back(point.recall);
        const double denominator = point.precision + point.recall;
        const double f1 = (denominator > 0.0) ? 2.0 * point.precision * point.recall / denominator
                                              : 0.0;
        f1_scores.push_back(f1);
        if (f1 > best_f1) {
            best_f1 = f1;
            best_f1_threshold = value;
        }
    }

    json::Value sweep = json::Value::array();
    sweep.push_back(json::from_series("precision", thresholds, precisions));
    sweep.push_back(json::from_series("recall", thresholds, recalls));
    sweep.push_back(json::from_series("F1", thresholds, f1_scores));

    json::Value out = json::Value::object();
    out.set("detections",
            json::from_table({"u_px", "v_px", "edge_energy", "shape_match", "confidence",
                              "is_true_target"},
                             detection_rows));
    out.set("pr_curve", json::from_series("precision against recall", recalls, precisions));
    out.set("threshold_sweep", std::move(sweep));
    out.set("threshold", json::Value(threshold));
    out.set("precision", json::Value(at_threshold.precision));
    out.set("recall", json::Value(at_threshold.recall));
    out.set("true_positives", json::Value(static_cast<double>(at_threshold.true_positives)));
    out.set("false_positives", json::Value(static_cast<double>(at_threshold.false_positives)));
    out.set("ground_truth_count", json::Value(static_cast<double>(truth_count)));
    out.set("candidate_count", json::Value(static_cast<double>(candidates.size())));
    out.set("detection_count", json::Value(static_cast<double>(kept.size())));
    out.set("best_f1", json::Value(best_f1));
    out.set("best_f1_threshold", json::Value(best_f1_threshold));
    out.set("seed", json::Value(seed));
    out.set("note",
            json::Value("Nothing here is learned: the confidence is half the geometric edge "
                        "energy plus half the template match, so every number on a detection can "
                        "be traced to the two features that produced it. Raise the threshold and "
                        "precision climbs while recall falls - the sweep shows the whole trade, "
                        "and the F1 maximum names the threshold a cell would actually ship."));
    return out;
}

// ---------------------------------------------------------------------------
// pose_estimate
// ---------------------------------------------------------------------------

// Four corners of a square planar marker, in the marker's own frame.
[[nodiscard]] std::array<Eigen::Vector3d, 4> marker_corners(double side) noexcept {
    const double h = 0.5 * side;
    return {Eigen::Vector3d(-h, -h, 0.0), Eigen::Vector3d(h, -h, 0.0), Eigen::Vector3d(h, h, 0.0),
            Eigen::Vector3d(-h, h, 0.0)};
}

[[nodiscard]] Eigen::Matrix3d orthonormalise(const Eigen::Matrix3d& M) {
    const Eigen::JacobiSVD<Eigen::Matrix3d> svd(M, Eigen::ComputeFullU | Eigen::ComputeFullV);
    Eigen::Matrix3d R = svd.matrixU() * svd.matrixV().transpose();
    if (R.determinant() < 0.0) {
        Eigen::Matrix3d flip = Eigen::Matrix3d::Identity();
        flip(2, 2) = -1.0;
        R = svd.matrixU() * flip * svd.matrixV().transpose();
    }
    return R;
}

// Planar DLT homography from four or more coplanar correspondences, decomposed
// with the known intrinsics into an initial camera-from-marker pose.
[[nodiscard]] Eigen::Isometry3d homography_pose(const vision::Intrinsics& K,
                                                const std::array<Eigen::Vector3d, 4>& object,
                                                const std::array<Eigen::Vector2d, 4>& pixels) {
    Eigen::Matrix<double, 8, 9> A = Eigen::Matrix<double, 8, 9>::Zero();
    for (int i = 0; i < 4; ++i) {
        const double X = object[static_cast<std::size_t>(i)].x();
        const double Y = object[static_cast<std::size_t>(i)].y();
        const double u = pixels[static_cast<std::size_t>(i)].x();
        const double v = pixels[static_cast<std::size_t>(i)].y();
        A.row(2 * i) << -X, -Y, -1.0, 0.0, 0.0, 0.0, u * X, u * Y, u;
        A.row(2 * i + 1) << 0.0, 0.0, 0.0, -X, -Y, -1.0, v * X, v * Y, v;
    }
    const Eigen::JacobiSVD<Eigen::Matrix<double, 8, 9>> svd(A, Eigen::ComputeFullV);
    const Eigen::Matrix<double, 9, 1> h = svd.matrixV().col(8);
    Eigen::Matrix3d H;
    H << h(0), h(1), h(2), h(3), h(4), h(5), h(6), h(7), h(8);

    const Eigen::Matrix3d B = K.matrix().inverse() * H;
    const double scale = 2.0 / (B.col(0).norm() + B.col(1).norm());
    Eigen::Vector3d r1 = scale * B.col(0);
    Eigen::Vector3d r2 = scale * B.col(1);
    Eigen::Vector3d t = scale * B.col(2);
    if (t.z() < 0.0) {
        r1 = -r1;
        r2 = -r2;
        t = -t;
    }
    Eigen::Matrix3d R;
    R.col(0) = r1;
    R.col(1) = r2;
    R.col(2) = r1.cross(r2);

    Eigen::Isometry3d pose = Eigen::Isometry3d::Identity();
    pose.linear() = orthonormalise(R);
    pose.translation() = t;
    return pose;
}

[[nodiscard]] Eigen::Isometry3d apply_increment(const Eigen::Isometry3d& pose,
                                                const Eigen::Matrix<double, 6, 1>& delta) {
    const Eigen::Vector3d omega = delta.tail<3>();
    const double angle = omega.norm();
    Eigen::Matrix3d dR = Eigen::Matrix3d::Identity();
    if (angle > 1e-14) {
        dR = Eigen::AngleAxisd(angle, omega / angle).toRotationMatrix();
    }
    Eigen::Isometry3d out = Eigen::Isometry3d::Identity();
    out.linear() = orthonormalise(dR * pose.linear());
    out.translation() = pose.translation() + delta.head<3>();
    return out;
}

[[nodiscard]] Eigen::VectorXd reprojection_residual(const vision::Intrinsics& K,
                                                    const Eigen::Isometry3d& pose,
                                                    const std::array<Eigen::Vector3d, 4>& object,
                                                    const std::array<Eigen::Vector2d, 4>& pixels) {
    Eigen::VectorXd residual(8);
    for (int i = 0; i < 4; ++i) {
        const Eigen::Vector3d p_cam = pose * object[static_cast<std::size_t>(i)];
        const vision::Projection projected = vision::project(K, p_cam);
        residual(2 * i) = projected.u - pixels[static_cast<std::size_t>(i)].x();
        residual(2 * i + 1) = projected.v - pixels[static_cast<std::size_t>(i)].y();
    }
    return residual;
}

struct PnpResult {
    Eigen::Isometry3d pose = Eigen::Isometry3d::Identity();
    double reprojection_rms = 0.0;
    int iterations = 0;
};

// Homography initialisation followed by Gauss-Newton on the reprojection
// residual. The Jacobian is central-differenced on the six pose increments,
// which is honest and costs twelve projections per iteration.
[[nodiscard]] PnpResult solve_pnp(const vision::Intrinsics& K,
                                  const std::array<Eigen::Vector3d, 4>& object,
                                  const std::array<Eigen::Vector2d, 4>& pixels,
                                  int max_iterations) {
    PnpResult result;
    result.pose = homography_pose(K, object, pixels);
    Eigen::VectorXd residual = reprojection_residual(K, result.pose, object, pixels);
    const double step = 1e-7;
    for (int iteration = 0; iteration < max_iterations; ++iteration) {
        Eigen::Matrix<double, 8, 6> J;
        for (int c = 0; c < 6; ++c) {
            Eigen::Matrix<double, 6, 1> delta = Eigen::Matrix<double, 6, 1>::Zero();
            delta(c) = step;
            const Eigen::VectorXd plus =
                reprojection_residual(K, apply_increment(result.pose, delta), object, pixels);
            delta(c) = -step;
            const Eigen::VectorXd minus =
                reprojection_residual(K, apply_increment(result.pose, delta), object, pixels);
            J.col(c) = (plus - minus) / (2.0 * step);
        }
        const Eigen::Matrix<double, 6, 6> H =
            J.transpose() * J + 1e-12 * Eigen::Matrix<double, 6, 6>::Identity();
        const Eigen::Matrix<double, 6, 1> delta = H.ldlt().solve(-J.transpose() * residual);
        const Eigen::Isometry3d candidate = apply_increment(result.pose, delta);
        const Eigen::VectorXd candidate_residual =
            reprojection_residual(K, candidate, object, pixels);
        result.iterations = iteration + 1;
        if (candidate_residual.norm() >= residual.norm()) {
            break;
        }
        result.pose = candidate;
        residual = candidate_residual;
        if (delta.norm() < 1e-12) {
            break;
        }
    }
    result.reprojection_rms = residual.norm() / std::sqrt(8.0);
    return result;
}

[[nodiscard]] json::Value op_pose_estimate(const json::Value& args) {
    const CameraSetup setup = read_camera(args);
    const double side = optional_scalar(args, "marker_side", 0.06, 0.005, 1.0);
    const Eigen::Vector3d position =
        optional_vec3(args, "marker_position", Eigen::Vector3d(0.90, 0.0, 0.735), -5.0, 5.0);
    const Eigen::Vector3d rpy =
        optional_vec3(args, "marker_rpy", Eigen::Vector3d(0.0, -kPi / 2.0, 0.0), -kPi, kPi);
    const double pixel_noise = optional_scalar(args, "pixel_noise", 0.0, 0.0, 20.0);
    const double sweep_max_noise = optional_scalar(args, "sweep_max_noise", 3.0, 0.0, 20.0);
    const int trials = optional_int(args, "trials", 24, 1, 500);
    const int seed = optional_int(args, "seed", 5, kSeedMin, kSeedMax);

    Eigen::Isometry3d marker_in_base = Eigen::Isometry3d::Identity();
    marker_in_base.linear() =
        (Eigen::AngleAxisd(rpy.z(), Eigen::Vector3d::UnitZ()) *
         Eigen::AngleAxisd(rpy.y(), Eigen::Vector3d::UnitY()) *
         Eigen::AngleAxisd(rpy.x(), Eigen::Vector3d::UnitX()))
            .toRotationMatrix();
    marker_in_base.translation() = position;

    const Eigen::Isometry3d truth_camera_marker = setup.camera_to_base * marker_in_base;
    const std::array<Eigen::Vector3d, 4> object = marker_corners(side);

    std::array<Eigen::Vector2d, 4> ideal_pixels;
    for (std::size_t i = 0; i < 4; ++i) {
        const Eigen::Vector3d p_cam = truth_camera_marker * object[i];
        if (p_cam.z() <= 1e-4) {
            throw StudyError(
                "the marker is not in front of the camera at this configuration: corner " +
                std::to_string(i) + " has a camera-frame depth of " +
                json::number_to_string(p_cam.z()) + " m");
        }
        const vision::Projection projected = vision::project(setup.K, p_cam);
        ideal_pixels[i] = Eigen::Vector2d(projected.u, projected.v);
    }

    const auto estimate_with_noise = [&](double sigma, int run_seed) {
        sensing::SeededRng rng(run_seed);
        std::array<Eigen::Vector2d, 4> noisy = ideal_pixels;
        for (auto& pixel : noisy) {
            pixel.x() += sigma * rng.gaussian();
            pixel.y() += sigma * rng.gaussian();
        }
        return solve_pnp(setup.K, object, noisy, 30);
    };

    const PnpResult main_result = estimate_with_noise(pixel_noise, seed);
    const Eigen::Isometry3d estimated_base_marker = setup.base_to_camera * main_result.pose;

    const double translation_error =
        (main_result.pose.translation() - truth_camera_marker.translation()).norm();
    const Eigen::Matrix3d rotation_difference =
        main_result.pose.linear().transpose() * truth_camera_marker.linear();
    const double rotation_error = Eigen::AngleAxisd(rotation_difference).angle();

    // Sensitivity: the same geometry re-estimated over a seeded noise sweep, so
    // the student sees how a pixel of jitter turns into millimetres of pose.
    const int sweep_points = 7;
    std::vector<double> noise_levels;
    std::vector<double> translation_errors;
    std::vector<double> rotation_errors;
    std::vector<double> reprojection_errors;
    noise_levels.reserve(static_cast<std::size_t>(sweep_points));
    translation_errors.reserve(static_cast<std::size_t>(sweep_points));
    rotation_errors.reserve(static_cast<std::size_t>(sweep_points));
    reprojection_errors.reserve(static_cast<std::size_t>(sweep_points));
    for (int s = 0; s < sweep_points; ++s) {
        const double sigma =
            sweep_max_noise * static_cast<double>(s) / static_cast<double>(sweep_points - 1);
        double translation_sum = 0.0;
        double rotation_sum = 0.0;
        double reprojection_sum = 0.0;
        for (int trial = 0; trial < trials; ++trial) {
            const PnpResult run = estimate_with_noise(sigma, seed + 1000 * s + trial);
            translation_sum +=
                (run.pose.translation() - truth_camera_marker.translation()).norm();
            const Eigen::Matrix3d difference =
                run.pose.linear().transpose() * truth_camera_marker.linear();
            rotation_sum += std::abs(Eigen::AngleAxisd(difference).angle());
            reprojection_sum += run.reprojection_rms;
        }
        const auto count = static_cast<double>(trials);
        noise_levels.push_back(sigma);
        translation_errors.push_back(translation_sum / count);
        rotation_errors.push_back(rotation_sum / count);
        reprojection_errors.push_back(reprojection_sum / count);
    }

    json::Value sensitivity = json::Value::array();
    sensitivity.push_back(
        json::from_series("mean translation error [m]", noise_levels, translation_errors));
    sensitivity.push_back(
        json::from_series("mean rotation error [rad]", noise_levels, rotation_errors));
    sensitivity.push_back(
        json::from_series("mean reprojection RMS [px]", noise_levels, reprojection_errors));

    std::vector<std::vector<json::Value>> corner_rows;
    corner_rows.reserve(4);
    for (std::size_t i = 0; i < 4; ++i) {
        const Eigen::Vector3d p_cam = main_result.pose * object[i];
        const vision::Projection projected = vision::project(setup.K, p_cam);
        corner_rows.push_back({json::Value(static_cast<double>(i)),
                               json::Value(ideal_pixels[i].x()), json::Value(ideal_pixels[i].y()),
                               json::Value(projected.u), json::Value(projected.v),
                               json::Value(std::hypot(projected.u - ideal_pixels[i].x(),
                                                      projected.v - ideal_pixels[i].y()))});
    }

    json::Value out = json::Value::object();
    out.set("T_camera_marker", json::from_isometry(main_result.pose));
    out.set("T_camera_marker_truth", json::from_isometry(truth_camera_marker));
    out.set("T_base_marker", json::from_isometry(estimated_base_marker));
    out.set("corners",
            json::from_table({"corner", "u_observed", "v_observed", "u_reprojected",
                              "v_reprojected", "error_px"},
                             corner_rows));
    out.set("sensitivity", std::move(sensitivity));
    out.set("reprojection_rms_px", json::Value(main_result.reprojection_rms));
    out.set("round_trip_tolerance_px", json::Value(kPoseRoundTripTolerancePx));
    out.set("round_trip_within_tolerance",
            json::Value(main_result.reprojection_rms <= kPoseRoundTripTolerancePx));
    out.set("translation_error_m", json::Value(translation_error));
    out.set("rotation_error_rad", json::Value(rotation_error));
    out.set("iterations", json::Value(static_cast<double>(main_result.iterations)));
    out.set("marker_side", json::Value(side));
    out.set("pixel_noise", json::Value(pixel_noise));
    out.set("seed", json::Value(seed));
    out.set("note",
            json::Value("Four coplanar corners fix a homography, the known intrinsics turn that "
                        "homography into a pose, and Gauss-Newton then minimises the pixel "
                        "residual itself. With zero pixel noise the loop closes to better than " +
                        json::number_to_string(kPoseRoundTripTolerancePx) +
                        " px RMS, which is the tolerance this op promises; the sensitivity sweep "
                        "is how fast that promise degrades with real jitter."));
    return out;
}

// ---------------------------------------------------------------------------
// visual_servo
// ---------------------------------------------------------------------------

struct ServoRun {
    std::vector<double> error_norm;
    std::vector<double> error_u;
    std::vector<double> error_v;
    std::vector<Eigen::Matrix<double, 6, 1>> joints;
    std::vector<Eigen::Vector3d> tool_path;
    bool diverged = false;
    bool lost_target = false;
    bool clamped = false;
    double convergence_time = std::numeric_limits<double>::quiet_NaN();
};

[[nodiscard]] ServoRun run_servo(const CameraSetup& setup, const Eigen::Vector3d& target,
                                 double desired_u, double desired_v, double gain, double dt,
                                 int steps, double pixel_noise, double tolerance, double damping,
                                 int seed) {
    ServoRun run;
    const auto n = static_cast<std::size_t>(steps);
    run.error_norm.reserve(n + 1);
    run.error_u.reserve(n + 1);
    run.error_v.reserve(n + 1);
    run.joints.reserve(n + 1);
    run.tool_path.reserve(n + 1);

    sensing::SeededRng rng(seed);
    Eigen::Matrix<double, 6, 1> q = setup.q;
    double initial_error = 0.0;

    for (std::size_t k = 0; k <= n; ++k) {
        const Eigen::Isometry3d base_camera =
            forward_kinematics_dh(q) * setup.flange_to_camera;
        const Eigen::Vector3d p_cam = base_camera.inverse() * target;
        if (p_cam.z() <= 1e-4) {
            run.lost_target = true;
            run.diverged = true;
            break;
        }
        const vision::Projection projected = vision::project(setup.K, p_cam);
        const double u = projected.u + pixel_noise * rng.gaussian();
        const double v = projected.v + pixel_noise * rng.gaussian();
        const Eigen::Vector2d error(u - desired_u, v - desired_v);

        run.error_u.push_back(error.x());
        run.error_v.push_back(error.y());
        run.error_norm.push_back(error.norm());
        run.joints.push_back(q);
        run.tool_path.push_back(forward_kinematics_dh(q).translation());

        if (k == 0) {
            initial_error = error.norm();
        }
        if (!std::isfinite(error.norm()) || error.norm() > 50.0 * (initial_error + 1.0)) {
            run.diverged = true;
            break;
        }
        if (std::isnan(run.convergence_time) && error.norm() <= tolerance) {
            run.convergence_time = dt * static_cast<double>(k);
        }
        if (k == n) {
            break;
        }

        // Resolved-rate visual servoing: the image Jacobian maps a camera
        // twist to feature velocity, the camera Jacobian maps joint rates to
        // that twist, and the damped pseudo-inverse goes back the other way.
        const Eigen::Matrix<double, 2, 6> L = vision::image_jacobian(setup.K, p_cam);
        const Eigen::Matrix<double, 6, 6> V =
            vision::camera_jacobian(q, setup.flange_to_camera);
        const Eigen::Matrix<double, 2, 6> feature_jacobian = L * V;
        const Eigen::Matrix<double, 6, 2> inverse =
            vision::damped_pseudo_inverse(feature_jacobian, damping);
        const Eigen::Matrix<double, 6, 1> qdot = -gain * (inverse * error);
        if (!qdot.allFinite()) {
            run.diverged = true;
            break;
        }
        q += qdot * dt;
        for (std::size_t j = 0; j < GP8_DOF; ++j) {
            const auto index = static_cast<Eigen::Index>(j);
            const double clamped = std::clamp(q(index), joint_min(j), joint_max(j));
            if (clamped != q(index)) {
                run.clamped = true;
            }
            q(index) = clamped;
        }
    }

    if (!run.error_norm.empty()) {
        const double first = run.error_norm.front();
        const double last = run.error_norm.back();
        if (!std::isfinite(last) || last > first) {
            run.diverged = true;
        }
    }
    return run;
}

[[nodiscard]] json::Value op_visual_servo(const json::Value& args) {
    const CameraSetup setup = read_camera(args);
    const Eigen::Vector3d target =
        optional_vec3(args, "target", Eigen::Vector3d(0.90, 0.07, 0.80), -5.0, 5.0);
    const double dt = optional_scalar(args, "dt", 0.02, 0.001, 0.2);
    const int steps = optional_int(args, "steps", 120, 2, 2000);
    const double pixel_noise = optional_scalar(args, "pixel_noise", 0.0, 0.0, 20.0);
    const double tolerance = optional_scalar(args, "tolerance", 1.0, 1e-4, 100.0);
    const double damping = optional_scalar(args, "damping", 1e-6, 0.0, 1.0);
    const int seed = optional_int(args, "seed", 13, kSeedMin, kSeedMax);
    const double desired_u =
        optional_scalar(args, "desired_u", 0.5 * static_cast<double>(setup.K.width), 0.0,
                        static_cast<double>(setup.K.width));
    const double desired_v =
        optional_scalar(args, "desired_v", 0.5 * static_cast<double>(setup.K.height), 0.0,
                        static_cast<double>(setup.K.height));

    // Euler integration of edot = -lambda e is stable only while lambda dt < 2;
    // the sweep measures where this nonlinear loop really gives up, which is
    // the number the panel and the test both quote.
    const double theoretical_limit = 2.0 / dt;
    double stable_limit = 0.0;
    double unstable_gain = std::numeric_limits<double>::quiet_NaN();
    std::vector<double> sweep_gains;
    std::vector<double> sweep_final_errors;
    const int probe_steps = std::min(steps, 60);
    for (double probe = 0.5; probe <= 6.0 * theoretical_limit; probe *= 1.15) {
        const ServoRun run = run_servo(setup, target, desired_u, desired_v, probe, dt,
                                       probe_steps, 0.0, tolerance, damping, seed);
        const double final_error =
            run.error_norm.empty() ? std::numeric_limits<double>::infinity()
                                   : run.error_norm.back();
        sweep_gains.push_back(probe);
        sweep_final_errors.push_back(std::isfinite(final_error) ? final_error : 1e9);
        if (run.diverged) {
            if (std::isnan(unstable_gain)) {
                unstable_gain = probe;
            }
        } else if (std::isnan(unstable_gain)) {
            stable_limit = probe;
        }
    }
    if (std::isnan(unstable_gain)) {
        unstable_gain = 6.0 * theoretical_limit;
    }

    const double gain = optional_scalar(args, "gain", 0.25 * theoretical_limit, 0.001, 2000.0);
    const ServoRun run = run_servo(setup, target, desired_u, desired_v, gain, dt, steps,
                                   pixel_noise, tolerance, damping, seed);

    std::vector<double> time;
    time.reserve(run.error_norm.size());
    for (std::size_t k = 0; k < run.error_norm.size(); ++k) {
        time.push_back(dt * static_cast<double>(k));
    }

    json::Value image_error = json::Value::array();
    image_error.push_back(json::from_series("|e| [px]", time, run.error_norm));
    image_error.push_back(json::from_series("e_u [px]", time, run.error_u));
    image_error.push_back(json::from_series("e_v [px]", time, run.error_v));

    json::Value joint_trajectory = json::Value::array();
    for (std::size_t j = 0; j < GP8_DOF; ++j) {
        std::vector<double> values;
        values.reserve(run.joints.size());
        for (const auto& q : run.joints) {
            values.push_back(q(static_cast<Eigen::Index>(j)));
        }
        joint_trajectory.push_back(
            json::from_series(std::string(GP8_AXIS_NAMES[j]) + " [rad]", time, values));
    }

    bool monotone = true;
    for (std::size_t k = 1; k < run.error_norm.size(); ++k) {
        if (run.error_norm[k] > run.error_norm[k - 1] * (1.0 + 1e-9) + 1e-12) {
            monotone = false;
            break;
        }
    }

    json::Value out = json::Value::object();
    out.set("image_error", std::move(image_error));
    out.set("joint_trajectory", std::move(joint_trajectory));
    out.set("tool_path", json::from_points(run.tool_path));
    out.set("gain_sweep",
            json::from_series("final |e| after " + std::to_string(probe_steps) + " steps",
                              sweep_gains, sweep_final_errors));
    out.set("gain", json::Value(gain));
    out.set("dt", json::Value(dt));
    out.set("initial_error_px",
            json::Value(run.error_norm.empty() ? 0.0 : run.error_norm.front()));
    out.set("final_error_px", json::Value(run.error_norm.empty() ? 0.0 : run.error_norm.back()));
    out.set("convergence_time", json::Value(run.convergence_time));
    out.set("converged", json::Value(!std::isnan(run.convergence_time)));
    out.set("error_monotonically_decreasing", json::Value(monotone));
    out.set("diverged", json::Value(run.diverged));
    out.set("lost_target", json::Value(run.lost_target));
    out.set("hit_joint_limit", json::Value(run.clamped));
    out.set("stable_gain_limit", json::Value(stable_limit));
    out.set("unstable_gain", json::Value(unstable_gain));
    out.set("theoretical_gain_limit", json::Value(theoretical_limit));
    out.set("steps", json::Value(steps));
    out.set("seed", json::Value(seed));
    out.set("note",
            json::Value("This is genuinely closed loop: every step reprojects the target through "
                        "the current joint configuration, so the image error the controller sees "
                        "is produced by the motion it commanded. Euler integration of "
                        "edot = -lambda e is stable only while lambda dt < 2, which is " +
                        json::number_to_string(theoretical_limit) +
                        " here; the measured onset of divergence is " +
                        json::number_to_string(unstable_gain) +
                        ", which does not land exactly on that bound because the interaction "
                        "matrix is rebuilt from the new depth at every step."));
    return out;
}

// ---------------------------------------------------------------------------
// hand_eye_calibration
// ---------------------------------------------------------------------------

[[nodiscard]] Eigen::Vector3d rotation_log(const Eigen::Matrix3d& R) {
    const Eigen::AngleAxisd aa(orthonormalise(R));
    return aa.axis() * aa.angle();
}

// (M^T M)^{-1/2} M^T, the Park-Martin closed form for A X = X B rotations.
[[nodiscard]] Eigen::Matrix3d park_martin_rotation(const Eigen::Matrix3d& M) {
    const Eigen::Matrix3d MtM = M.transpose() * M;
    Eigen::SelfAdjointEigenSolver<Eigen::Matrix3d> solver(MtM);
    Eigen::Vector3d inverse_root = Eigen::Vector3d::Zero();
    for (int i = 0; i < 3; ++i) {
        const double lambda = solver.eigenvalues()(i);
        inverse_root(i) = (lambda > 1e-14) ? 1.0 / std::sqrt(lambda) : 0.0;
    }
    const Eigen::Matrix3d root_inverse =
        solver.eigenvectors() * inverse_root.asDiagonal() * solver.eigenvectors().transpose();
    return orthonormalise(root_inverse * M.transpose());
}

struct HandEyeSolution {
    Eigen::Isometry3d X = Eigen::Isometry3d::Identity();
    Eigen::Isometry3d Z = Eigen::Isometry3d::Identity();
    double residual = 0.0;
    std::size_t pairs = 0;
};

[[nodiscard]] HandEyeSolution solve_hand_eye(const std::vector<Eigen::Isometry3d>& base_flange,
                                             const std::vector<Eigen::Isometry3d>& camera_target) {
    HandEyeSolution solution;
    const std::size_t n = base_flange.size();

    Eigen::Matrix3d M = Eigen::Matrix3d::Zero();
    std::vector<Eigen::Matrix3d> rotation_a;
    std::vector<Eigen::Vector3d> translation_a;
    std::vector<Eigen::Vector3d> beta_list;
    for (std::size_t i = 0; i < n; ++i) {
        for (std::size_t j = i + 1; j < n; ++j) {
            const Eigen::Isometry3d A = base_flange[j].inverse() * base_flange[i];
            const Eigen::Isometry3d B = camera_target[j] * camera_target[i].inverse();
            const Eigen::Vector3d alpha = rotation_log(A.linear());
            const Eigen::Vector3d beta = rotation_log(B.linear());
            if (alpha.norm() < 1e-6 || beta.norm() < 1e-6) {
                continue;  // a pair with no relative rotation carries no information
            }
            M += beta * alpha.transpose();
            rotation_a.push_back(A.linear());
            translation_a.push_back(A.translation());
            beta_list.push_back(B.translation());
            ++solution.pairs;
        }
    }
    if (solution.pairs < 2) {
        throw StudyError("hand-eye calibration needs at least two pose pairs with distinct "
                         "orientations; this pose set supplies " +
                         std::to_string(solution.pairs));
    }

    solution.X.linear() = park_martin_rotation(M);

    // (R_A - I) t_X = R_X t_B - t_A, stacked over every usable pair.
    const auto rows = static_cast<Eigen::Index>(3 * solution.pairs);
    Eigen::MatrixXd C(rows, 3);
    Eigen::VectorXd d(rows);
    for (std::size_t k = 0; k < solution.pairs; ++k) {
        const auto row = static_cast<Eigen::Index>(3 * k);
        C.block<3, 3>(row, 0) = rotation_a[k] - Eigen::Matrix3d::Identity();
        d.segment<3>(row) = solution.X.linear() * beta_list[k] - translation_a[k];
    }
    solution.X.translation() =
        C.colPivHouseholderQr().solve(d);

    // Z = 0_T_e X c_T_t is the same fixture pose from every robot pose, so its
    // spread over the poses IS the calibration residual.
    Eigen::Vector3d mean_translation = Eigen::Vector3d::Zero();
    Eigen::Matrix3d mean_rotation = Eigen::Matrix3d::Zero();
    std::vector<Eigen::Isometry3d> estimates;
    estimates.reserve(n);
    for (std::size_t i = 0; i < n; ++i) {
        const Eigen::Isometry3d Z = base_flange[i] * solution.X * camera_target[i];
        estimates.push_back(Z);
        mean_translation += Z.translation();
        mean_rotation += Z.linear();
    }
    solution.Z.translation() = mean_translation / static_cast<double>(n);
    solution.Z.linear() = orthonormalise(mean_rotation / static_cast<double>(n));

    double sum = 0.0;
    for (const auto& Z : estimates) {
        sum += (Z.matrix() - solution.Z.matrix()).norm();
    }
    solution.residual = sum / static_cast<double>(n);
    return solution;
}

[[nodiscard]] json::Value op_hand_eye_calibration(const json::Value& args) {
    const std::string mounting = optional_enum(args, "mounting", "forward_tool", kMountings);
    const int poses = optional_int(args, "poses", 10, 3, 30);
    const double rotation_sigma = optional_scalar(args, "rotation_sigma", 0.002, 0.0, 0.2);
    const double translation_sigma = optional_scalar(args, "translation_sigma", 0.001, 0.0, 0.1);
    const Eigen::Vector3d fixture =
        optional_vec3(args, "fixture", Eigen::Vector3d(0.90, 0.0, 0.70), -5.0, 5.0);
    const int seed = optional_int(args, "seed", 29, kSeedMin, kSeedMax);

    const Eigen::Isometry3d truth = vision::hand_eye_transform(mounting);
    Eigen::Isometry3d fixture_pose = Eigen::Isometry3d::Identity();
    fixture_pose.translation() = fixture;

    // Robot poses: a seeded sweep over a safe fraction of every joint's travel,
    // because a hand-eye solution needs genuinely different orientations.
    sensing::SeededRng rng(seed);
    const auto n = static_cast<std::size_t>(poses);
    std::vector<Eigen::Isometry3d> base_flange;
    std::vector<Eigen::Isometry3d> camera_target;
    std::vector<Eigen::Matrix<double, 6, 1>> configurations;
    base_flange.reserve(n);
    camera_target.reserve(n);
    configurations.reserve(n);

    for (std::size_t i = 0; i < n; ++i) {
        Eigen::Matrix<double, 6, 1> q;
        for (std::size_t j = 0; j < GP8_DOF; ++j) {
            const double span = 0.35 * std::min(-joint_min(j), joint_max(j));
            q(static_cast<Eigen::Index>(j)) = rng.uniform(-span, span);
        }
        const Eigen::Isometry3d flange = forward_kinematics_dh(q);
        const Eigen::Isometry3d camera = flange * truth;
        Eigen::Isometry3d observation = camera.inverse() * fixture_pose;

        // Measurement noise on the observed pose, as a camera really reports it.
        if (rotation_sigma > 0.0) {
            const Eigen::Vector3d axis(rng.gaussian(), rng.gaussian(), rng.gaussian());
            const double norm = axis.norm();
            if (norm > 1e-12) {
                observation.linear() =
                    Eigen::AngleAxisd(rotation_sigma * rng.gaussian(), axis / norm)
                        .toRotationMatrix() *
                    observation.linear();
            }
        }
        observation.translation() += translation_sigma * Eigen::Vector3d(
                                                             rng.gaussian(), rng.gaussian(),
                                                             rng.gaussian());

        configurations.push_back(q);
        base_flange.push_back(flange);
        camera_target.push_back(observation);
    }

    const HandEyeSolution solution = solve_hand_eye(base_flange, camera_target);

    // How the residual falls with the number of poses used: the whole argument
    // for taking more than the minimum.
    std::vector<double> pose_counts;
    std::vector<double> residuals;
    std::vector<double> translation_errors;
    std::vector<double> rotation_errors;
    for (std::size_t m = 3; m <= n; ++m) {
        const std::vector<Eigen::Isometry3d> flanges(base_flange.begin(),
                                                     base_flange.begin() +
                                                         static_cast<std::ptrdiff_t>(m));
        const std::vector<Eigen::Isometry3d> observations(
            camera_target.begin(), camera_target.begin() + static_cast<std::ptrdiff_t>(m));
        const HandEyeSolution partial = solve_hand_eye(flanges, observations);
        pose_counts.push_back(static_cast<double>(m));
        residuals.push_back(partial.residual);
        translation_errors.push_back(
            (partial.X.translation() - truth.translation()).norm());
        rotation_errors.push_back(std::abs(
            Eigen::AngleAxisd(partial.X.linear().transpose() * truth.linear()).angle()));
    }

    json::Value convergence = json::Value::array();
    convergence.push_back(json::from_series("residual [-]", pose_counts, residuals));
    convergence.push_back(
        json::from_series("translation error [m]", pose_counts, translation_errors));
    convergence.push_back(json::from_series("rotation error [rad]", pose_counts, rotation_errors));

    std::vector<std::vector<json::Value>> pose_rows;
    pose_rows.reserve(n);
    for (std::size_t i = 0; i < n; ++i) {
        const Eigen::Isometry3d Z = base_flange[i] * solution.X * camera_target[i];
        pose_rows.push_back({json::Value(static_cast<double>(i)),
                             json::Value(base_flange[i].translation().x()),
                             json::Value(base_flange[i].translation().y()),
                             json::Value(base_flange[i].translation().z()),
                             json::Value(camera_target[i].translation().norm()),
                             json::Value((Z.translation() - solution.Z.translation()).norm())});
    }

    const double translation_error = (solution.X.translation() - truth.translation()).norm();
    const double rotation_error = std::abs(
        Eigen::AngleAxisd(solution.X.linear().transpose() * truth.linear()).angle());

    json::Value out = json::Value::object();
    out.set("T_flange_camera", json::from_isometry(solution.X));
    out.set("T_flange_camera_truth", json::from_isometry(truth));
    out.set("T_base_fixture", json::from_isometry(solution.Z));
    out.set("poses", json::from_table({"pose", "flange_x", "flange_y", "flange_z",
                                       "observed_range_m", "fixture_scatter_m"},
                                      pose_rows));
    out.set("convergence", std::move(convergence));
    out.set("residual", json::Value(solution.residual));
    out.set("translation_error_m", json::Value(translation_error));
    out.set("rotation_error_rad", json::Value(rotation_error));
    out.set("pose_count", json::Value(poses));
    out.set("pair_count", json::Value(static_cast<double>(solution.pairs)));
    out.set("mounting", json::Value(mounting));
    out.set("seed", json::Value(seed));
    out.set("note",
            json::Value("Every pose gives 0_T_e X c_T_t = Z with X and Z both unknown, so two "
                        "poses give A X = X B and the rotation follows in closed form from "
                        "(M^T M)^{-1/2} M^T with M = sum beta alpha^T, Park and Martin's result. "
                        "The translation is then a plain least-squares solve of "
                        "(R_A - I) t_X = R_X t_B - t_A. Two poses are the minimum and are never "
                        "enough: the convergence curve is the reason to teach ten."));
    return out;
}

}  // namespace

// ---------------------------------------------------------------------------
// vision:: free functions
// ---------------------------------------------------------------------------

namespace vision {

Eigen::Matrix3d Intrinsics::matrix() const noexcept {
    Eigen::Matrix3d K = Eigen::Matrix3d::Identity();
    K(0, 0) = fx;
    K(1, 1) = fy;
    K(0, 2) = cx;
    K(1, 2) = cy;
    return K;
}

Intrinsics intrinsics_from_sensor(double focal_mm, double sensor_width_mm,
                                  double sensor_height_mm, int width_px,
                                  int height_px) noexcept {
    Intrinsics K;
    K.width = width_px;
    K.height = height_px;
    K.fx = focal_mm / sensor_width_mm * static_cast<double>(width_px);
    K.fy = focal_mm / sensor_height_mm * static_cast<double>(height_px);
    K.cx = 0.5 * static_cast<double>(width_px);
    K.cy = 0.5 * static_cast<double>(height_px);
    return K;
}

Eigen::Isometry3d hand_eye_transform(std::string_view mounting) noexcept {
    // Camera axes expressed in the flange frame. "forward_tool" points the
    // optical axis along the flange x axis, which at q = 0 looks horizontally
    // out of the cell; the image u axis runs along -y and v runs along -z, the
    // usual right-down-forward camera convention.
    Eigen::Matrix3d forward;
    forward << 0.0, 0.0, 1.0,
              -1.0, 0.0, 0.0,
               0.0, -1.0, 0.0;

    Eigen::Isometry3d T = Eigen::Isometry3d::Identity();
    if (mounting == "flange_aligned") {
        T.translation() = Eigen::Vector3d(0.0, 0.0, 0.060);
        return T;
    }
    if (mounting == "tilted_30") {
        T.linear() = forward *
                     Eigen::AngleAxisd(30.0 * std::numbers::pi / 180.0, Eigen::Vector3d::UnitX())
                         .toRotationMatrix();
        T.translation() = Eigen::Vector3d(0.060, 0.0, 0.020);
        return T;
    }
    T.linear() = forward;
    T.translation() = Eigen::Vector3d(0.060, 0.0, 0.020);
    return T;
}

Projection project(const Intrinsics& K, const Eigen::Vector3d& p_cam) noexcept {
    Projection out;
    out.depth = p_cam.z();
    out.in_front = p_cam.z() > 1e-9;
    if (!out.in_front) {
        return out;
    }
    out.u = K.fx * (p_cam.x() / p_cam.z()) + K.cx;
    out.v = K.fy * (p_cam.y() / p_cam.z()) + K.cy;
    out.in_image = out.u >= 0.0 && out.u <= static_cast<double>(K.width) && out.v >= 0.0 &&
                   out.v <= static_cast<double>(K.height);
    return out;
}

Projection project_distorted(const Intrinsics& K, const Eigen::Vector3d& p_cam, double k1,
                             double k2) noexcept {
    Projection out;
    out.depth = p_cam.z();
    out.in_front = p_cam.z() > 1e-9;
    if (!out.in_front) {
        return out;
    }
    const double x = p_cam.x() / p_cam.z();
    const double y = p_cam.y() / p_cam.z();
    const double r2 = x * x + y * y;
    const double scale = 1.0 + k1 * r2 + k2 * r2 * r2;
    out.u = K.fx * (x * scale) + K.cx;
    out.v = K.fy * (y * scale) + K.cy;
    out.in_image = out.u >= 0.0 && out.u <= static_cast<double>(K.width) && out.v >= 0.0 &&
                   out.v <= static_cast<double>(K.height);
    return out;
}

Eigen::Matrix<double, 6, 6> camera_jacobian(const Eigen::Matrix<double, 6, 1>& q,
                                            const Eigen::Isometry3d& flange_to_camera) noexcept {
    const std::array<Eigen::Isometry3d, GP8_DOF> frames = link_frames(q);
    const Eigen::Isometry3d base_camera = forward_kinematics_dh(q) * flange_to_camera;
    const Eigen::Vector3d p_camera = base_camera.translation();
    const Eigen::Matrix3d R_cb = base_camera.linear().transpose();

    Eigen::Matrix<double, 6, 6> J = Eigen::Matrix<double, 6, 6>::Zero();
    for (std::size_t i = 0; i < GP8_DOF; ++i) {
        const Eigen::Vector3d axis =
            (i == 0) ? Eigen::Vector3d::UnitZ() : Eigen::Vector3d(frames[i - 1].linear().col(2));
        const Eigen::Vector3d origin = (i == 0) ? Eigen::Vector3d(0.0, 0.0, 0.0)
                                                : Eigen::Vector3d(frames[i - 1].translation());
        const auto column = static_cast<Eigen::Index>(i);
        J.block<3, 1>(0, column) = R_cb * axis.cross(p_camera - origin);
        J.block<3, 1>(3, column) = R_cb * axis;
    }
    return J;
}

Eigen::Matrix<double, 2, 6> image_jacobian(const Intrinsics& K,
                                           const Eigen::Vector3d& p_cam) noexcept {
    const double Z = p_cam.z();
    const double x = p_cam.x() / Z;
    const double y = p_cam.y() / Z;
    Eigen::Matrix<double, 2, 6> L;
    L(0, 0) = -K.fx / Z;
    L(0, 1) = 0.0;
    L(0, 2) = K.fx * x / Z;
    L(0, 3) = K.fx * x * y;
    L(0, 4) = -K.fx * (1.0 + x * x);
    L(0, 5) = K.fx * y;
    L(1, 0) = 0.0;
    L(1, 1) = -K.fy / Z;
    L(1, 2) = K.fy * y / Z;
    L(1, 3) = K.fy * (1.0 + y * y);
    L(1, 4) = -K.fy * x * y;
    L(1, 5) = -K.fy * x;
    return L;
}

Eigen::Matrix<double, 6, 2> damped_pseudo_inverse(const Eigen::Matrix<double, 2, 6>& J,
                                                  double damping) noexcept {
    const Eigen::Matrix2d gram =
        J * J.transpose() + damping * damping * Eigen::Matrix2d::Identity();
    return J.transpose() * gram.inverse();
}

}  // namespace vision

// ---------------------------------------------------------------------------
// Self-description
// ---------------------------------------------------------------------------

ModuleDescription VisionCameraModule::describe() const {
    ModuleDescription d;
    d.name = "vision_camera";
    d.title = "Eye-in-Hand AI Camera: Projection, Detection and Visual Servoing";
    d.course = CourseRef{3884, "M-408-01", "Robotic Systems Design"};
    d.topics = {
        "Session 27 · Information (sensing) devices of robotic systems",
        "Course 2953 · Applied AI: perception, detection and its evaluation",
        "Course 3883 · Exam Q1(a): 0_T_p = 0_T_e e_T_c c_T_p",
    };
    d.source = "cpp_solver/include/study/vision_camera.hpp";
    d.summary =
        "Models the camera on the GP8 flange rather than reading one: intrinsics from a real "
        "lens and sensor, extrinsics from the DH chain times the hand-eye transform, radial "
        "distortion, an explainable detector with its full precision-recall trade, homography "
        "plus Gauss-Newton pose estimation, a closed-loop resolved-rate visual servo with the "
        "gain at which it destabilises, and a Park-Martin hand-eye calibration.";

    const std::vector<ParamSpec> shared = camera_params();

    {
        OpSpec op;
        op.name = "camera_model";
        op.title = "Pinhole camera on the flange: intrinsics, extrinsics and the view cone";
        op.formula =
            "K = \\begin{bmatrix} f_x & 0 & c_x \\\\ 0 & f_y & c_y \\\\ 0 & 0 & 1 "
            "\\end{bmatrix}, \\quad f_x = \\frac{f\\,W_{px}}{W_{mm}}, \\qquad "
            "{}^{0}T_c = {}^{0}T_e \\, {}^{e}T_c, \\qquad P = K \\begin{bmatrix} "
            "{}^{c}R_0 & {}^{c}t_0 \\end{bmatrix}";
        op.explain =
            "The intrinsic matrix is nothing but the lens and the sensor written as pixels: the "
            "focal length in millimetres divided by the sensor size and multiplied by the pixel "
            "count. The extrinsic part is where the robot enters - the camera pose is the DH "
            "chain of this GP8 times the hand-eye transform, so moving a joint moves the view "
            "cone, and swapping those two factors moves the predicted pixel, which is exactly "
            "what exam question Q1(a) asks you to explain. The frustum corners come back in the "
            "base frame so the 3D view can draw what the camera can actually see.";
        op.params = shared;
        op.params.push_back(ParamSpec::scalar("near_plane", "Frustum near plane", "m", 0.005, 5.0,
                                              0.08));
        op.params.push_back(
            ParamSpec::scalar("far_plane", "Frustum far plane", "m", 0.01, 20.0, 0.80));
        op.outputs = {
            OutputSpec::make("K", "mat3", "Intrinsic matrix", "px"),
            OutputSpec::make("intrinsics", "table", "fx, fy, cx, cy and the image size"),
            OutputSpec::make("T_base_camera", "mat4", "Camera pose in the base frame"),
            OutputSpec::make("T_flange_camera", "mat4", "Hand-eye transform that was used"),
            OutputSpec::make("extrinsic", "matrix", "[c_R_0 | c_t_0], world to camera"),
            OutputSpec::make("projection", "matrix", "P = K [R | t], the 3x4 projection"),
            OutputSpec::make("frustum", "points", "Optical centre plus the eight plane corners"),
            OutputSpec::make("chain", "table", "The three factors of the hand-eye chain"),
            OutputSpec::make("fov_horizontal_deg", "scalar", "Horizontal field of view", "deg"),
            OutputSpec::make("fov_vertical_deg", "scalar", "Vertical field of view", "deg"),
            OutputSpec::make("note", "text", "Why the multiplication order matters"),
        };
        d.ops.push_back(std::move(op));
    }
    {
        OpSpec op;
        op.name = "project_targets";
        op.title = "Project workcell targets into the image, with lens distortion";
        op.formula =
            "{}^{c}p = {}^{c}T_0 \\, {}^{0}p, \\quad x = \\frac{X}{Z},\\; y = \\frac{Y}{Z}, "
            "\\quad r^2 = x^2 + y^2, \\quad \\begin{bmatrix} u \\\\ v \\end{bmatrix} = "
            "\\begin{bmatrix} f_x x (1 + k_1 r^2 + k_2 r^4) + c_x \\\\ f_y y (1 + k_1 r^2 + "
            "k_2 r^4) + c_y \\end{bmatrix}";
        op.explain =
            "Projection is a division, and that division is where the information is lost: two "
            "points on the same ray land on the same pixel, which is why the depth column and "
            "the occlusion flag matter more than the pixel pair. A target counts as visible only "
            "when it is in front of the lens AND inside the image - the pinhole equations happily "
            "return a finite pixel for a point behind the camera, and that is the classic silent "
            "bug. Distortion is applied in normalised coordinates before the intrinsics scale "
            "them, so the shift grows toward the corners and vanishes at the principal point.";
        op.params = shared;
        op.params.push_back(ParamSpec::structured("targets", "Target points in the workcell",
                                                  "table", "m", targets_default_table()));
        op.params.push_back(
            ParamSpec::scalar("k1", "Radial distortion k1", "", -1.0, 1.0, -0.18));
        op.params.push_back(ParamSpec::scalar("k2", "Radial distortion k2", "", -1.0, 1.0, 0.04));
        op.params.push_back(ParamSpec::scalar("occlusion_radius", "Occlusion radius", "px", 0.0,
                                             500.0, 18.0));
        op.outputs = {
            OutputSpec::make("pixels", "table",
                             "Per target: depth, ideal and distorted pixel, visibility"),
            OutputSpec::make("visible_points", "points", "The targets inside the field of view"),
            OutputSpec::make("in_fov_count", "int", "How many targets are visible"),
            OutputSpec::make("occlusion_free_count", "int", "Visible and not hidden by another"),
            OutputSpec::make("max_distortion_shift_px", "scalar", "Largest distortion shift",
                             "px"),
            OutputSpec::make("note", "text", "Why the undistorted pixels are the usable ones"),
        };
        d.ops.push_back(std::move(op));
    }
    {
        OpSpec op;
        op.name = "detect";
        op.title = "Explainable detector: confidence, NMS, precision and recall";
        op.formula =
            "s = \\tfrac{1}{2} E + \\tfrac{1}{2} M, \\quad E = \\frac{d_{ref}}{d_{ref} + Z}, "
            "\\qquad P = \\frac{TP}{TP + FP}, \\quad R = \\frac{TP}{TP + FN}, \\quad "
            "F_1 = \\frac{2PR}{P + R}";
        op.explain =
            "Nothing here is learned, and that is deliberate: the confidence is half a geometric "
            "edge energy (a nearer blob subtends more pixels, so it carries more edge) and half a "
            "template match score, so every number on a detection traces back to the two features "
            "that produced it. Non-maximum suppression then removes the duplicates a sliding "
            "detector always produces, keeping the highest score in each neighbourhood. Move the "
            "threshold and watch precision climb while recall falls on identical input - the full "
            "sweep comes back so that trade is a curve and not an anecdote, and the F1 maximum "
            "names the threshold a real cell would ship.";
        op.params = shared;
        op.params.push_back(ParamSpec::structured("targets", "Target points in the workcell",
                                                  "table", "m", targets_default_table()));
        op.params.push_back(
            ParamSpec::scalar("threshold", "Confidence threshold", "", 0.0, 1.0, 0.55));
        op.params.push_back(ParamSpec::integer("clutter", "Clutter detections", "", 0, 400, 14));
        op.params.push_back(
            ParamSpec::scalar("pixel_noise", "Pixel noise sigma", "px", 0.0, 50.0, 1.5));
        op.params.push_back(
            ParamSpec::scalar("nms_radius", "Suppression radius", "px", 1.0, 500.0, 24.0));
        op.params.push_back(
            ParamSpec::scalar("match_radius", "Ground-truth match radius", "px", 1.0, 500.0,
                              12.0));
        op.params.push_back(ParamSpec::integer("seed", "Random seed", "", kSeedMin, kSeedMax, 17));
        op.outputs = {
            OutputSpec::make("detections", "table", "Kept detections with both features"),
            OutputSpec::make("pr_curve", "series", "Precision against recall"),
            OutputSpec::make("threshold_sweep", "series_set",
                             "Precision, recall and F1 against the threshold"),
            OutputSpec::make("precision", "scalar", "Precision at the chosen threshold"),
            OutputSpec::make("recall", "scalar", "Recall at the chosen threshold"),
            OutputSpec::make("best_f1_threshold", "scalar", "Threshold maximising F1"),
            OutputSpec::make("ground_truth_count", "int", "Visible targets to be found"),
            OutputSpec::make("note", "text", "What makes this detector explainable"),
        };
        d.ops.push_back(std::move(op));
    }
    {
        OpSpec op;
        op.name = "pose_estimate";
        op.title = "Pose of a planar marker from its projected corners";
        op.formula =
            "s \\begin{bmatrix} u \\\\ v \\\\ 1 \\end{bmatrix} = H \\begin{bmatrix} X \\\\ Y "
            "\\\\ 1 \\end{bmatrix}, \\quad [r_1\\, r_2\\, t] = \\lambda K^{-1} H, \\quad "
            "r_3 = r_1 \\times r_2, \\qquad \\min_{T} \\sum_i \\lVert \\pi(T \\, p_i) - "
            "\\tilde{u}_i \\rVert^2";
        op.explain =
            "Four coplanar corners determine a homography, and the known intrinsics turn that "
            "homography straight into a pose: the first two columns of K^-1 H are the first two "
            "rotation columns up to a common scale, and their cross product supplies the third. "
            "That initial guess is then refined by Gauss-Newton on the pixel residual itself, "
            "which is the quantity you actually measured. With zero pixel noise the whole loop "
            "closes to better than the stated tolerance; the sensitivity sweep shows how fast a "
            "pixel of jitter becomes millimetres of pose, and it degrades as the marker gets "
            "smaller or further away.";
        op.params = shared;
        op.params.push_back(
            ParamSpec::scalar("marker_side", "Marker side length", "m", 0.005, 1.0, 0.06));
        op.params.push_back(ParamSpec::vec3("marker_position", "Marker position in the base frame",
                                            "m", -5.0, 5.0, Eigen::Vector3d(0.90, 0.0, 0.735)));
        op.params.push_back(ParamSpec::vec3("marker_rpy", "Marker roll-pitch-yaw", "rad", -kPi,
                                            kPi, Eigen::Vector3d(0.0, -kPi / 2.0, 0.0)));
        op.params.push_back(
            ParamSpec::scalar("pixel_noise", "Pixel noise sigma", "px", 0.0, 20.0, 0.0));
        op.params.push_back(
            ParamSpec::scalar("sweep_max_noise", "Sensitivity sweep maximum", "px", 0.0, 20.0,
                              3.0));
        op.params.push_back(ParamSpec::integer("trials", "Trials per sweep point", "", 1, 500, 24));
        op.params.push_back(ParamSpec::integer("seed", "Random seed", "", kSeedMin, kSeedMax, 5));
        op.outputs = {
            OutputSpec::make("T_camera_marker", "mat4", "Estimated marker pose in the camera"),
            OutputSpec::make("T_base_marker", "mat4", "The same pose mapped into the base frame"),
            OutputSpec::make("corners", "table", "Observed against reprojected corners"),
            OutputSpec::make("sensitivity", "series_set", "Error against pixel noise"),
            OutputSpec::make("reprojection_rms_px", "scalar", "RMS reprojection error", "px"),
            OutputSpec::make("round_trip_tolerance_px", "scalar", "Tolerance this op promises",
                             "px"),
            OutputSpec::make("translation_error_m", "scalar", "Translation error", "m"),
            OutputSpec::make("rotation_error_rad", "scalar", "Rotation error", "rad"),
            OutputSpec::make("note", "text", "What the tolerance means"),
        };
        d.ops.push_back(std::move(op));
    }
    {
        OpSpec op;
        op.name = "visual_servo";
        op.title = "Resolved-rate visual servoing: the camera drives the joints";
        op.formula =
            "e = s - s^{*}, \\qquad \\dot{s} = L({}^{c}p)\\, {}^{c}V = L\\,{}^{c}J(q)\\,"
            "\\dot{q}, \\qquad \\dot{q} = -\\lambda\\,(L\\,{}^{c}J)^{\\dagger} e, \\qquad "
            "\\text{stable while } \\lambda T < 2";
        op.explain =
            "This is the op where the camera genuinely drives the robot: the image error is "
            "mapped to a camera twist by the interaction matrix, the camera twist is mapped to "
            "joint rates by the camera Jacobian of this GP8's DH chain, and the damped "
            "pseudo-inverse goes back the other way. Every step reprojects the target through "
            "the configuration the previous step commanded, so the loop is closed and not "
            "replayed. With a small gain the error decays geometrically; raise the gain and the "
            "Euler integration of edot = -lambda e loses stability past lambda T = 2. The "
            "measured onset of divergence is reported alongside that bound rather than assumed "
            "equal to it, because the interaction matrix is rebuilt from the new depth at every "
            "step and the real loop is not the linear one the bound describes.";
        op.params = shared;
        op.params.push_back(ParamSpec::vec3("target", "Target point in the base frame", "m", -5.0,
                                            5.0, Eigen::Vector3d(0.90, 0.07, 0.80)));
        op.params.push_back(ParamSpec::scalar("gain", "Servo gain lambda", "1/s", 0.001, 2000.0,
                                              25.0));
        op.params.push_back(ParamSpec::scalar("dt", "Control period", "s", 0.001, 0.2, 0.02));
        op.params.push_back(ParamSpec::integer("steps", "Control steps", "", 2, 2000, 120));
        op.params.push_back(
            ParamSpec::scalar("tolerance", "Convergence tolerance", "px", 1e-4, 100.0, 1.0));
        op.params.push_back(
            ParamSpec::scalar("pixel_noise", "Pixel noise sigma", "px", 0.0, 20.0, 0.0));
        op.params.push_back(ParamSpec::scalar("damping", "Pseudo-inverse damping", "", 0.0, 1.0,
                                              1e-6));
        op.params.push_back(ParamSpec::integer("seed", "Random seed", "", kSeedMin, kSeedMax, 13));
        op.outputs = {
            OutputSpec::make("image_error", "series_set", "|e|, e_u and e_v against time"),
            OutputSpec::make("joint_trajectory", "series_set", "Every axis against time"),
            OutputSpec::make("tool_path", "points", "Flange path the loop produced"),
            OutputSpec::make("gain_sweep", "series", "Final error against gain"),
            OutputSpec::make("convergence_time", "scalar", "Time to reach the tolerance", "s"),
            OutputSpec::make("stable_gain_limit", "scalar", "Largest gain that still converges",
                             "1/s"),
            OutputSpec::make("unstable_gain", "scalar", "Measured onset of divergence", "1/s"),
            OutputSpec::make("theoretical_gain_limit", "scalar", "2 / T, the Euler bound", "1/s"),
            OutputSpec::make("error_monotonically_decreasing", "bool",
                             "Whether |e| fell at every step"),
            OutputSpec::make("note", "text", "Why the measured limit is below the bound"),
        };
        d.ops.push_back(std::move(op));
    }
    {
        OpSpec op;
        op.name = "hand_eye_calibration";
        op.title = "Solve the unknown camera-to-flange transform from robot poses";
        op.formula =
            "{}^{0}T_{e_i} X \\, {}^{c}T_{t_i} = Z \\;\\Rightarrow\\; A_{ij} X = X B_{ij}, "
            "\\qquad R_X = (M^{T} M)^{-1/2} M^{T},\\; M = \\sum \\beta_{ij}\\alpha_{ij}^{T}, "
            "\\qquad (R_A - I)\\,t_X = R_X t_B - t_A";
        op.explain =
            "The hand-eye transform is the one frame nobody can measure with a ruler, so it has "
            "to be solved for. Each robot pose gives flange times X times observation equals the "
            "same fixture pose Z, with X and Z both unknown; differencing two poses eliminates Z "
            "and leaves the classic A X = X B. The rotation then follows in closed form from the "
            "logarithms of the relative rotations - Park and Martin's (M^T M)^-1/2 M^T - and the "
            "translation is a plain least-squares solve. Two poses are the algebraic minimum and "
            "are never enough in practice: the convergence curve is exactly why a calibration "
            "procedure asks for ten.";
        op.params = {
            ParamSpec::enumeration("mounting", "True mounting to recover", kMountings,
                                   "forward_tool"),
            ParamSpec::integer("poses", "Number of robot poses", "", 3, 30, 10),
            ParamSpec::scalar("rotation_sigma", "Observation rotation noise", "rad", 0.0, 0.2,
                              0.002),
            ParamSpec::scalar("translation_sigma", "Observation translation noise", "m", 0.0, 0.1,
                              0.001),
            ParamSpec::vec3("fixture", "Calibration fixture position", "m", -5.0, 5.0,
                            Eigen::Vector3d(0.90, 0.0, 0.70)),
            ParamSpec::integer("seed", "Random seed", "", kSeedMin, kSeedMax, 29),
        };
        op.outputs = {
            OutputSpec::make("T_flange_camera", "mat4", "Solved hand-eye transform"),
            OutputSpec::make("T_flange_camera_truth", "mat4", "The transform that was simulated"),
            OutputSpec::make("T_base_fixture", "mat4", "Recovered fixture pose Z"),
            OutputSpec::make("poses", "table", "Per-pose flange position and fixture scatter"),
            OutputSpec::make("convergence", "series_set", "Residual and error against pose count"),
            OutputSpec::make("residual", "scalar", "Mean Frobenius spread of Z"),
            OutputSpec::make("translation_error_m", "scalar", "Error against the truth", "m"),
            OutputSpec::make("rotation_error_rad", "scalar", "Error against the truth", "rad"),
            OutputSpec::make("note", "text", "Why two poses are never enough"),
        };
        d.ops.push_back(std::move(op));
    }
    return d;
}

json::Value VisionCameraModule::invoke(std::string_view op, const json::Value& args) const {
    if (op == "camera_model") {
        return op_camera_model(args);
    }
    if (op == "project_targets") {
        return op_project_targets(args);
    }
    if (op == "detect") {
        return op_detect(args);
    }
    if (op == "pose_estimate") {
        return op_pose_estimate(args);
    }
    if (op == "visual_servo") {
        return op_visual_servo(args);
    }
    if (op == "hand_eye_calibration") {
        return op_hand_eye_calibration(args);
    }
    unknown_op(name(), op);
}

}  // namespace yaskawa::study
