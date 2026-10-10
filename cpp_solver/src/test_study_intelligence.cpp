// Test suite for the intelligence half of the study layer: sensing_models,
// vision_camera and learning_models. Plain asserts in the style of
// src/test_study_modules.cpp - there is no gtest in this project and there
// will not be one.
//
// Every check asserts a property, never a printed string:
//   - the same seed gives bit-identical output and a different seed does not
//   - a fitted noise parameter converges to the true one as n grows
//   - the fused posterior beats both sensors it was built from
//   - a projected point's pose round-trips inside the stated pixel tolerance
//   - the reported frustum agrees with the reported visibility
//   - visual servoing decays for a stable gain and diverges above the one it
//     reports as unstable
//   - the network beats a stated trivial baseline, and the GA never regresses
//   - responsible_ai actually refuses a motion past a joint limit

#include "study/gp8_model.hpp"
#include "study/json.hpp"
#include "study/learning_models.hpp"
#include "study/sensing_models.hpp"
#include "study/study_module.hpp"
#include "study/vision_camera.hpp"

#include <Eigen/Dense>
#include <Eigen/Geometry>

#include <algorithm>
#include <cmath>
#include <iostream>
#include <string>
#include <vector>

using yaskawa::study::json::Value;
namespace json = yaskawa::study::json;
namespace study = yaskawa::study;

namespace {

int g_checks = 0;
int g_failures = 0;
std::vector<std::string> g_failed_names;

void check(bool ok, const std::string& what) {
    ++g_checks;
    if (ok) {
        std::cout << "[PASS] " << what << "\n";
    } else {
        ++g_failures;
        g_failed_names.push_back(what);
        std::cout << "[FAIL] " << what << "\n";
    }
}

void check_lt(double actual, double bound, const std::string& what) {
    const bool ok = std::isfinite(actual) && actual < bound;
    if (!ok) {
        std::cout << "       expected < " << bound << ", got " << actual << "\n";
    }
    check(ok, what);
}

// Every call goes through here, so a thrown StudyError is a failed check with
// its message rather than a crashed suite.
[[nodiscard]] Value call(const study::StudyModule& module, const char* op, const Value& args) {
    try {
        return module.invoke(op, args);
    } catch (const std::exception& error) {
        std::cout << "       " << module.name() << "." << op << " threw: " << error.what()
                  << "\n";
        check(false, std::string(module.name()) + "." + op + " must not throw on valid input");
        return Value::object();
    }
}

[[nodiscard]] Value with_seed(const Value& base, int seed) {
    Value args = base;
    args.set("seed", Value(seed));
    return args;
}

// The highest-value property in the whole suite: same seed, same bytes.
void expect_seed_determinism(const study::StudyModule& module, const char* op, const Value& base) {
    const std::string label = std::string(module.name()) + "." + op;
    const Value first = call(module, op, with_seed(base, 101));
    const Value again = call(module, op, with_seed(base, 101));
    const Value other = call(module, op, with_seed(base, 202));
    const std::string first_text = json::dump(first);
    const std::string again_text = json::dump(again);
    const std::string other_text = json::dump(other);
    check(first_text == again_text, label + ": seed 101 twice is bit identical");
    check(first_text != other_text, label + ": seed 202 differs from seed 101");
}

void expect_unknown_op(const study::StudyModule& module) {
    bool threw = false;
    try {
        const Value ignored = module.invoke("no_such_op", Value::object());
        (void)ignored;
    } catch (const study::StudyError&) {
        threw = true;
    } catch (...) {
        threw = false;
    }
    check(threw, std::string(module.name()) + " rejects an unknown op with StudyError");
}

// describe() has to be complete enough for the browser to build the panel.
void expect_describe_is_usable(const study::StudyModule& module) {
    const study::ModuleDescription d = module.describe();
    const std::string label = std::string(module.name());
    check(d.name == module.name(), label + ": describe() names the module");
    check(!d.title.empty() && !d.summary.empty() && !d.source.empty() && !d.topics.empty(),
          label + ": describe() carries a title, summary, source and topics");
    check(d.course.id != 0 && !d.course.code.empty(), label + ": describe() names its course");
    check(!d.ops.empty(), label + ": describe() lists at least one op");

    bool ops_complete = true;
    for (const study::OpSpec& op : d.ops) {
        if (op.name.empty() || op.title.empty() || op.formula.empty() || op.explain.empty() ||
            op.outputs.empty()) {
            ops_complete = false;
            std::cout << "       incomplete op: " << op.name << "\n";
        }
        for (const study::ParamSpec& p : op.params) {
            if (p.name.empty() || p.type.empty() || p.label.empty() ||
                p.default_value.is_null()) {
                ops_complete = false;
                std::cout << "       incomplete param " << p.name << " of op " << op.name << "\n";
            }
            if (p.type == "enum" && p.options.empty()) {
                ops_complete = false;
            }
        }
        for (const study::OutputSpec& o : op.outputs) {
            if (o.name.empty() || o.type.empty() || o.label.empty()) {
                ops_complete = false;
            }
        }
    }
    check(ops_complete, label + ": every op has a formula, an explanation, typed params with "
                                "defaults and typed outputs");

    // Every advertised op must answer with its documented defaults alone.
    bool all_ops_answer = true;
    for (const study::OpSpec& op : d.ops) {
        try {
            const Value result = module.invoke(op.name, Value::object());
            if (!result.is_object() || result.empty()) {
                all_ops_answer = false;
                std::cout << "       op " << op.name << " returned an empty result\n";
            }
        } catch (const std::exception& error) {
            all_ops_answer = false;
            std::cout << "       op " << op.name << " threw on its own defaults: " << error.what()
                      << "\n";
        }
    }
    check(all_ops_answer, label + ": every advertised op answers with its own defaults");
}

// ---------------------------------------------------------------------------
// sensing_models
// ---------------------------------------------------------------------------

void test_sensing_models() {
    std::cout << "\n--- sensing_models ---\n";
    const study::SensingModelsModule module;
    expect_describe_is_usable(module);
    expect_unknown_op(module);

    // Encoder: the quantisation error has to match the delta^2/12 theory, and
    // the increment has to come from the gear ratio in gp8_model.hpp.
    const Value encoder = call(module, "encoder", Value::object());
    const double increment = encoder["joint_increment"].as_double();
    const double expected_increment =
        2.0 * std::numbers::pi / (std::ldexp(1.0, 17) * study::GP8_LINKS[1].gear_ratio);
    check(std::abs(increment - expected_increment) < 1e-18,
          "encoder: the increment is 2 pi / (2^bits x gear ratio) of the L axis");
    const double measured_rms = encoder["position_rms_error"].as_double();
    const double theoretical_rms = encoder["theoretical_position_rms"].as_double();
    check(measured_rms > 0.4 * theoretical_rms && measured_rms < 2.0 * theoretical_rms,
          "encoder: the measured quantisation RMS matches delta / sqrt(12) within a factor of 2");
    check(encoder["samples_outside_half_increment"].as_double() == 0.0,
          "encoder: no sample leaves the +/- half increment band");
    const Value finer = call(module, "encoder", Value::object({{"bits", Value(20)}}));
    check_lt(finer["joint_increment"].as_double(), increment,
             "encoder: more bits resolve a smaller increment");

    // Noise: the fit has to converge as the sample count grows. Averaged over
    // several seeds, because one seed is an anecdote.
    const auto mean_sigma_error = [&module](int samples) {
        double sum = 0.0;
        const int seeds = 5;
        for (int s = 0; s < seeds; ++s) {
            const Value result =
                call(module, "noise_models",
                     Value::object({{"samples", Value(samples)}, {"seed", Value(900 + s)}}));
            sum += result["sigma_abs_error"].as_double();
        }
        return sum / static_cast<double>(seeds);
    };
    const double error_small = mean_sigma_error(200);
    const double error_large = mean_sigma_error(20000);
    std::cout << "       fitted sigma error: " << error_small << " at n=200, " << error_large
              << " at n=20000\n";
    check_lt(error_large, error_small,
             "noise_models: the fitted sigma converges to the true sigma as n grows");
    check_lt(error_large, 0.2 * error_small,
             "noise_models: 100x the samples cuts the sigma error by well over half");

    const Value poisson = call(module, "noise_models",
                               Value::object({{"distribution", Value("poisson")},
                                              {"samples", Value(40000)},
                                              {"lambda", Value(6.0)}}));
    check(poisson["noise_skewness"].as_double() > 0.2,
          "noise_models: Poisson noise is visibly right skewed at small lambda");
    const Value uniform = call(module, "noise_models",
                               Value::object({{"distribution", Value("uniform")},
                                              {"samples", Value(40000)}}));
    check(std::abs(uniform["noise_skewness"].as_double()) < 0.1,
          "noise_models: uniform noise is symmetric");
    const Value gaussian = call(module, "noise_models",
                                Value::object({{"samples", Value(100000)}}));
    const double outside = gaussian["three_sigma_fraction_outside"].as_double();
    check(outside < 0.01, "noise_models: the +/- 3 sigma band holds over 99 % of normal samples");

    // Conditioning: the chain must actually clean the signal, and the alias
    // frequency must be the folded one.
    const Value chain = call(module, "conditioning_chain", Value::object());
    check_lt(chain["final_rms_deviation"].as_double(), chain["raw_rms_deviation"].as_double(),
             "conditioning_chain: the output deviates from the ideal less than the raw input");
    check(chain["interference_will_alias"].as_bool(),
          "conditioning_chain: 150 Hz interference sampled at 200 Hz is flagged as aliasing");
    check(std::abs(chain["aliased_interference_frequency"].as_double() - 50.0) < 1e-9,
          "conditioning_chain: 150 Hz folds to 50 Hz at a 200 Hz sampling rate");
    check_lt(chain["antialias_attenuation_at_interference"].as_double(), 0.2,
             "conditioning_chain: the anti-alias stage attenuates the interference");

    // Bayesian fusion: the posterior must be tighter than either sensor, and
    // over many seeds the fused estimate must beat both of them in RMS.
    const Value fusion = call(module, "bayes_fusion", Value::object());
    check(fusion["variance_below_best_sensor"].as_bool(),
          "bayes_fusion: the posterior variance is below both sensor variances");
    check_lt(fusion["final_variance"].as_double(), fusion["encoder_variance"].as_double(),
             "bayes_fusion: the posterior beats the encoder variance");
    check_lt(fusion["final_variance"].as_double(), fusion["vision_variance"].as_double(),
             "bayes_fusion: the posterior beats the vision variance");

    double fused_square = 0.0;
    double encoder_square = 0.0;
    double vision_square = 0.0;
    bool variance_monotone = true;
    const int fusion_seeds = 150;
    for (int s = 0; s < fusion_seeds; ++s) {
        const Value run = call(module, "bayes_fusion",
                               Value::object({{"encoder_sigma", Value(0.03)},
                                              {"vision_sigma", Value(0.04)},
                                              {"seed", Value(1000 + s)}}));
        const double fused = run["error_fused"].as_double();
        const double encoder_only = run["error_encoder_only"].as_double();
        const double vision_only = run["error_vision_only"].as_double();
        fused_square += fused * fused;
        encoder_square += encoder_only * encoder_only;
        vision_square += vision_only * vision_only;
        if (s == 0) {
            const Value& curves = run["curves"];
            const Value& variance = curves[static_cast<std::size_t>(0)]["y"];
            for (std::size_t k = 1; k < variance.size(); ++k) {
                if (variance[k].as_double() > variance[k - 1].as_double()) {
                    variance_monotone = false;
                }
            }
        }
    }
    const auto runs = static_cast<double>(fusion_seeds);
    const double fused_rms = std::sqrt(fused_square / runs);
    const double encoder_rms = std::sqrt(encoder_square / runs);
    const double vision_rms = std::sqrt(vision_square / runs);
    std::cout << "       RMS over " << fusion_seeds << " seeds: fused " << fused_rms
              << ", encoder only " << encoder_rms << ", vision only " << vision_rms << "\n";
    check(variance_monotone, "bayes_fusion: the posterior variance never grows");
    check_lt(fused_rms, encoder_rms, "bayes_fusion: the fused estimate beats the encoder alone");
    check_lt(fused_rms, vision_rms, "bayes_fusion: the fused estimate beats the camera alone");

    // Fault detection: each fault must be caught, and tightening the threshold
    // must shorten the delay.
    for (const char* fault : {"stuck", "drift", "dropout"}) {
        const Value run = call(module, "fault_detection",
                               Value::object({{"fault", Value(fault)}, {"seed", Value(77)}}));
        check(run["detected"].as_bool(),
              std::string("fault_detection: the ") + fault + " fault is detected");
        check(run["detection_delay"].as_double() >= 0.0,
              std::string("fault_detection: the ") + fault + " delay is measured after onset");
        check(run["false_alarm_rate"].as_double() < 0.05,
              std::string("fault_detection: the ") + fault +
                  " run keeps the false-alarm rate under 5 %");
    }
    const Value clean = call(module, "fault_detection",
                             Value::object({{"fault", Value("none")}, {"seed", Value(77)}}));
    check(!clean.is_null() && clean["alarms_after_onset"].as_double() == 0.0,
          "fault_detection: a healthy sensor raises no alarm at 4 sigma");
    const Value tight = call(module, "fault_detection",
                             Value::object({{"threshold_sigma", Value(2.0)}, {"seed", Value(77)}}));
    const Value loose = call(module, "fault_detection",
                             Value::object({{"threshold_sigma", Value(8.0)}, {"seed", Value(77)}}));
    check(tight["detection_delay"].as_double() <= loose["detection_delay"].as_double(),
          "fault_detection: a tighter threshold detects no later than a loose one");
    check(tight["false_alarm_rate"].as_double() >= loose["false_alarm_rate"].as_double(),
          "fault_detection: and it pays for that with no fewer false alarms");

    expect_seed_determinism(module, "noise_models", Value::object({{"samples", Value(500)}}));
    expect_seed_determinism(module, "conditioning_chain", Value::object());
    expect_seed_determinism(module, "bayes_fusion", Value::object());
    expect_seed_determinism(module, "fault_detection", Value::object());
}

// ---------------------------------------------------------------------------
// vision_camera
// ---------------------------------------------------------------------------

[[nodiscard]] std::vector<Eigen::Vector3d> read_points(const Value& value) {
    std::vector<Eigen::Vector3d> out;
    out.reserve(value.size());
    for (std::size_t i = 0; i < value.size(); ++i) {
        const Value& row = value[i];
        out.emplace_back(row[static_cast<std::size_t>(0)].as_double(),
                         row[static_cast<std::size_t>(1)].as_double(),
                         row[static_cast<std::size_t>(2)].as_double());
    }
    return out;
}

// The four side planes of the reported frustum, each oriented so that a point
// inside the cone evaluates positive.
[[nodiscard]] bool inside_frustum(const std::vector<Eigen::Vector3d>& frustum,
                                  const Eigen::Vector3d& point) {
    if (frustum.size() != 9) {
        return false;
    }
    const Eigen::Vector3d apex = frustum[0];
    Eigen::Vector3d interior = Eigen::Vector3d::Zero();
    for (std::size_t i = 5; i < 9; ++i) {
        interior += frustum[i];
    }
    interior *= 0.25;
    for (std::size_t i = 0; i < 4; ++i) {
        const Eigen::Vector3d a = frustum[1 + i] - apex;
        const Eigen::Vector3d b = frustum[1 + ((i + 1) % 4)] - apex;
        Eigen::Vector3d normal = a.cross(b);
        if (normal.dot(interior - apex) < 0.0) {
            normal = -normal;
        }
        if (normal.dot(point - apex) < 0.0) {
            return false;
        }
    }
    return true;
}

void test_vision_camera() {
    std::cout << "\n--- vision_camera ---\n";
    const study::VisionCameraModule module;
    expect_describe_is_usable(module);
    expect_unknown_op(module);

    // Intrinsics and the hand-eye chain.
    const Value camera = call(module, "camera_model", Value::object());
    const Value& intrinsics = camera["intrinsics"]["rows"][static_cast<std::size_t>(0)];
    const double fx = intrinsics[static_cast<std::size_t>(0)].as_double();
    check(std::abs(fx - 8.0 / 6.4 * 1280.0) < 1e-9,
          "camera_model: fx is the focal length over the sensor width times the pixel count");
    const Eigen::Matrix<double, 6, 1> home = Eigen::Matrix<double, 6, 1>::Zero();
    const Eigen::Isometry3d expected_camera =
        study::forward_kinematics_dh(home) * study::vision::hand_eye_transform("forward_tool");
    const Value& T = camera["T_base_camera"];
    double chain_error = 0.0;
    for (std::size_t r = 0; r < 4; ++r) {
        for (std::size_t c = 0; c < 4; ++c) {
            chain_error = std::max(chain_error,
                                   std::abs(T[r][c].as_double() -
                                            expected_camera.matrix()(static_cast<Eigen::Index>(r),
                                                                     static_cast<Eigen::Index>(c))));
        }
    }
    check_lt(chain_error, 1e-12,
             "camera_model: the extrinsic pose is exactly the DH chain times the hand-eye "
             "transform");

    // The frustum must agree with the visibility the module reports.
    const std::vector<Eigen::Vector3d> frustum = read_points(camera["frustum"]);
    check(frustum.size() == 9, "camera_model: the frustum is the apex plus eight plane corners");
    const Value projected = call(module, "project_targets", Value::object());
    const Value& pixel_rows = projected["pixels"]["rows"];
    bool visible_inside = true;
    bool hidden_outside = true;
    std::size_t visible_checked = 0;
    std::size_t hidden_checked = 0;
    for (std::size_t i = 0; i < pixel_rows.size(); ++i) {
        const Value& row = pixel_rows[i];
        const Eigen::Vector3d point(row[static_cast<std::size_t>(1)].as_double(),
                                    row[static_cast<std::size_t>(2)].as_double(),
                                    row[static_cast<std::size_t>(3)].as_double());
        const bool in_fov = row[static_cast<std::size_t>(10)].as_bool();
        const bool inside = inside_frustum(frustum, point);
        if (in_fov) {
            ++visible_checked;
            visible_inside = visible_inside && inside;
        } else {
            ++hidden_checked;
            hidden_outside = hidden_outside && !inside;
        }
    }
    check(visible_checked > 0 && visible_inside,
          "camera_model: every target reported visible lies inside the reported frustum");
    check(hidden_checked > 0 && hidden_outside,
          "camera_model: every target reported invisible lies outside it");
    check(projected["in_fov_count"].as_double() >= projected["occlusion_free_count"].as_double(),
          "project_targets: the occlusion-free count cannot exceed the in-view count");
    check(projected["max_distortion_shift_px"].as_double() > 0.5,
          "project_targets: the distorted pixels really differ from the undistorted ones");

    // Pose estimation round trip, at the tolerance the op itself publishes.
    const Value pose = call(module, "pose_estimate", Value::object());
    const double tolerance = pose["round_trip_tolerance_px"].as_double();
    const double reprojection = pose["reprojection_rms_px"].as_double();
    std::cout << "       pose round trip: " << reprojection << " px RMS against a published "
              << tolerance << " px tolerance\n";
    check_lt(reprojection, tolerance,
             "pose_estimate: a noiseless projection round-trips inside the published tolerance");
    check(pose["round_trip_within_tolerance"].as_bool(),
          "pose_estimate: and the op says so itself");
    check_lt(pose["translation_error_m"].as_double(), 1e-6,
             "pose_estimate: the recovered translation matches the truth to a micrometre");
    check_lt(pose["rotation_error_rad"].as_double(), 1e-6,
             "pose_estimate: the recovered rotation matches the truth to a microradian");
    const Value& sensitivity = pose["sensitivity"][static_cast<std::size_t>(0)]["y"];
    check(sensitivity[sensitivity.size() - 1].as_double() >
              sensitivity[static_cast<std::size_t>(0)].as_double(),
          "pose_estimate: pose error grows with pixel noise");

    // Detection: precision against recall, on identical input.
    const Value detect = call(module, "detect", Value::object());
    check(detect["ground_truth_count"].as_double() > 0.0,
          "detect: there is something to find in the default scene");
    const Value& sweep = detect["threshold_sweep"];
    const Value& precision = sweep[static_cast<std::size_t>(0)]["y"];
    const Value& recall = sweep[static_cast<std::size_t>(1)]["y"];
    bool recall_non_increasing = true;
    for (std::size_t i = 1; i < recall.size(); ++i) {
        if (recall[i].as_double() > recall[i - 1].as_double() + 1e-12) {
            recall_non_increasing = false;
        }
    }
    check(recall_non_increasing, "detect: recall never rises as the threshold rises");
    check(precision[precision.size() - 1].as_double() >=
              precision[static_cast<std::size_t>(0)].as_double(),
          "detect: precision at the top of the sweep is no worse than at the bottom");
    check(recall[static_cast<std::size_t>(0)].as_double() >
              recall[recall.size() - 1].as_double(),
          "detect: and recall pays for it - the trade is real, not nominal");
    check(detect["detection_count"].as_double() <= detect["candidate_count"].as_double(),
          "detect: suppression removes candidates, it never invents them");

    // Visual servoing: decay below the gain it reports, divergence above it.
    const Value servo = call(module, "visual_servo", Value::object());
    const double stable_limit = servo["stable_gain_limit"].as_double();
    const double unstable_gain = servo["unstable_gain"].as_double();
    const double theoretical = servo["theoretical_gain_limit"].as_double();
    std::cout << "       visual servo: stable up to " << stable_limit << " 1/s, diverges at "
              << unstable_gain << " 1/s, Euler bound 2/T = " << theoretical << " 1/s\n";
    check(stable_limit > 0.0 && unstable_gain > stable_limit,
          "visual_servo: the reported stability limit is below the reported onset of divergence");
    check(servo["converged"].as_bool(),
          "visual_servo: the default gain drives the image error inside the tolerance");
    check_lt(servo["final_error_px"].as_double(), 0.05 * servo["initial_error_px"].as_double(),
             "visual_servo: the default gain removes at least 95 % of the image error");

    const Value calm = call(module, "visual_servo",
                            Value::object({{"gain", Value(0.2 * stable_limit)}}));
    check(calm["error_monotonically_decreasing"].as_bool(),
          "visual_servo: a gain well inside the limit reduces the image error at every step");
    check(!calm["diverged"].as_bool(), "visual_servo: and it does not diverge");
    const Value wild = call(module, "visual_servo",
                            Value::object({{"gain", Value(1.5 * unstable_gain)}}));
    check(wild["diverged"].as_bool(),
          "visual_servo: a gain above the reported onset diverges");
    check(!wild["error_monotonically_decreasing"].as_bool(),
          "visual_servo: and its image error does not decay monotonically");

    // Hand-eye calibration: exact without noise, and better with more poses.
    const Value calibration = call(module, "hand_eye_calibration",
                                   Value::object({{"rotation_sigma", Value(0.0)},
                                                  {"translation_sigma", Value(0.0)},
                                                  {"poses", Value(8)}}));
    check_lt(calibration["translation_error_m"].as_double(), 1e-9,
             "hand_eye_calibration: noiseless observations recover the translation exactly");
    check_lt(calibration["rotation_error_rad"].as_double(), 1e-9,
             "hand_eye_calibration: and the rotation exactly");
    check_lt(calibration["residual"].as_double(), 1e-9,
             "hand_eye_calibration: the fixture pose Z is identical from every robot pose");
    const Value noisy = call(module, "hand_eye_calibration", Value::object({{"poses", Value(20)}}));
    const Value& convergence = noisy["convergence"][static_cast<std::size_t>(1)]["y"];
    check(convergence.size() > 4 &&
              convergence[convergence.size() - 1].as_double() <
                  convergence[static_cast<std::size_t>(0)].as_double(),
          "hand_eye_calibration: the translation error falls as more poses are used");

    expect_seed_determinism(module, "detect", Value::object());
    expect_seed_determinism(module, "pose_estimate",
                            Value::object({{"pixel_noise", Value(1.0)}, {"trials", Value(4)}}));
    expect_seed_determinism(module, "visual_servo",
                            Value::object({{"pixel_noise", Value(0.5)}, {"steps", Value(40)}}));
    expect_seed_determinism(module, "hand_eye_calibration", Value::object({{"poses", Value(6)}}));
}

// ---------------------------------------------------------------------------
// learning_models
// ---------------------------------------------------------------------------

void test_learning_models() {
    std::cout << "\n--- learning_models ---\n";
    const study::LearningModelsModule module;
    expect_describe_is_usable(module);
    expect_unknown_op(module);

    // The network must learn, and must beat a stated trivial baseline.
    const Value net = call(module, "neural_network", Value::object());
    const double initial_loss = net["initial_loss"].as_double();
    const double final_loss = net["final_loss"].as_double();
    const double final_error = net["final_error"].as_double();
    const double baseline_error = net["baseline_error"].as_double();
    std::cout << "       network: loss " << initial_loss << " -> " << final_loss
              << ", held-out error " << final_error << " rad against a " << baseline_error
              << " rad predict-the-mean baseline\n";
    check_lt(final_loss, initial_loss, "neural_network: the training loss decreases");
    check_lt(final_error, baseline_error,
             "neural_network: the held-out error beats the predict-the-mean baseline");
    check(net["beats_baseline"].as_bool(), "neural_network: and the op says so itself");
    check(net["improvement_factor"].as_double() > 3.0,
          "neural_network: it beats that baseline by more than a factor of three");
    const Value longer = call(module, "neural_network", Value::object({{"epochs", Value(900)}}));
    check_lt(longer["final_loss"].as_double(), final_loss,
             "neural_network: more epochs reduce the training loss further");

    // The GA cannot regress while the elite is carried forward.
    const Value ga = call(module, "genetic_algorithm", Value::object());
    check(ga["best_fitness_monotone"].as_bool(),
          "genetic_algorithm: the best-so-far fitness is monotonically non-decreasing");
    const Value& best_curve = ga["best_fitness_history"]["y"];
    bool curve_monotone = true;
    for (std::size_t i = 1; i < best_curve.size(); ++i) {
        if (best_curve[i].as_double() < best_curve[i - 1].as_double()) {
            curve_monotone = false;
        }
    }
    check(curve_monotone, "genetic_algorithm: the returned curve itself never steps down");
    check(best_curve[best_curve.size() - 1].as_double() >
              best_curve[static_cast<std::size_t>(0)].as_double(),
          "genetic_algorithm: the search actually improves on its initial population");
    check_lt(ga["diversity_final"].as_double(), ga["diversity_initial"].as_double(),
             "genetic_algorithm: selection pressure collapses the gene diversity");
    check(ga["best_fitness"].as_double() > 0.0 && ga["best_fitness"].as_double() <= 1.0,
          "genetic_algorithm: the fitness stays in its bounded range");
    const Value frozen = call(module, "genetic_algorithm",
                              Value::object({{"mutation_rate", Value(0.0)},
                                             {"crossover_rate", Value(0.0)}}));
    check_lt(frozen["diversity_final"].as_double(), 1e-12,
             "genetic_algorithm: with no mutation and no crossover the population clones itself");

    // The tree must be interpretable and beat the majority-class baseline.
    const Value tree = call(module, "decision_tree", Value::object());
    check(tree["test_accuracy"].as_double() > tree["majority_baseline_accuracy"].as_double(),
          "decision_tree: the held-out accuracy beats always predicting the majority class");
    check(tree["node_count"].as_double() > 1.0, "decision_tree: the tree actually split");
    check(tree["root_entropy"].as_double() > 0.3,
          "decision_tree: the dataset is not already pure, so there is something to learn");
    const Value& gains = tree["information_gain"]["y"];
    bool gains_positive = gains.size() > 0;
    for (std::size_t i = 0; i < gains.size(); ++i) {
        if (!(gains[i].as_double() > 0.0)) {
            gains_positive = false;
        }
    }
    check(gains_positive, "decision_tree: every split has a strictly positive information gain");
    check(study::learning::binary_entropy(5, 10) > 0.999 &&
              study::learning::binary_entropy(0, 10) == 0.0,
          "decision_tree: the entropy is 1 bit at a 50/50 split and 0 bits on a pure set");

    // Agents: identical world, comparable scores, a readable trace.
    const Value agents = call(module, "agent_policy", Value::object());
    const Value& scores = agents["scores"]["rows"];
    check(scores.size() == 3, "agent_policy: all three architectures are scored");
    double reflex_score = 0.0;
    double model_score = 0.0;
    double goal_score = 0.0;
    double reflex_failures = -1.0;
    for (std::size_t i = 0; i < scores.size(); ++i) {
        const Value& row = scores[i];
        const std::string name = row[static_cast<std::size_t>(0)].as_string();
        const double score = row[static_cast<std::size_t>(4)].as_double();
        if (name == "simple_reflex") {
            reflex_score = score;
            reflex_failures = row[static_cast<std::size_t>(2)].as_double();
        } else if (name == "model_based") {
            model_score = score;
        } else if (name == "goal_based_utility") {
            goal_score = score;
        }
    }
    check(reflex_failures > 0.0,
          "agent_policy: the stateless reflex agent fails grips it cannot learn from");
    check(model_score > reflex_score,
          "agent_policy: keeping internal state beats the reflex agent on the same tray");
    check(goal_score >= model_score,
          "agent_policy: adding a goal and a utility does not make it worse");
    check(agents["decision_trace"]["rows"].size() > 0,
          "agent_policy: every decision is traced with its percept, state and reason");

    // Responsible AI: a motion past a joint limit must be refused, overrides
    // or no overrides.
    Eigen::Matrix<double, 6, 1> illegal = Eigen::Matrix<double, 6, 1>::Zero();
    illegal(0) = study::joint_max(0) + 0.5;
    const Value rejected =
        call(module, "responsible_ai",
             Value::object({{"q_proposed", json::from_vec6(illegal)}, {"duration", Value(5.0)}}));
    check(!rejected["accepted"].as_bool(),
          "responsible_ai: a motion past the S-axis travel limit is rejected");
    check(rejected["non_overridable_failures"].as_double() >= 1.0,
          "responsible_ai: and the failure is recorded as non-overridable");
    check(rejected["rejected_checks"].as_string().find("travel") != std::string::npos,
          "responsible_ai: the record names the travel check that failed");
    const Value forced = call(module, "responsible_ai",
                              Value::object({{"q_proposed", json::from_vec6(illegal)},
                                             {"duration", Value(5.0)},
                                             {"allow_overrides", Value(true)}}));
    check(!forced["accepted"].as_bool(),
          "responsible_ai: no override can wave a hard limit through");

    Eigen::Matrix<double, 6, 1> legal = Eigen::Matrix<double, 6, 1>::Zero();
    legal(1) = 0.4;
    legal(2) = -0.3;
    const Value accepted =
        call(module, "responsible_ai",
             Value::object({{"q_proposed", json::from_vec6(legal)}, {"duration", Value(8.0)}}));
    check(accepted["accepted"].as_bool(),
          "responsible_ai: a slow motion well inside every limit is accepted");
    check(accepted["non_overridable_failures"].as_double() == 0.0 &&
              accepted["overridable_failures"].as_double() == 0.0,
          "responsible_ai: with no failures in either category");

    const Value fast = call(module, "responsible_ai",
                            Value::object({{"q_proposed", json::from_vec6(legal)},
                                           {"duration", Value(0.4)}}));
    check(!fast["accepted"].as_bool(),
          "responsible_ai: the same pose reached too fast breaches the collaborative speed");
    const Value fast_with_override =
        call(module, "responsible_ai", Value::object({{"q_proposed", json::from_vec6(legal)},
                                                      {"duration", Value(0.4)},
                                                      {"allow_overrides", Value(true)}}));
    check(fast_with_override["accepted"].as_bool(),
          "responsible_ai: a cell policy, unlike a hard limit, can be overridden on the record");

    const Value overloaded = call(module, "responsible_ai",
                                  Value::object({{"payload", Value(12.0)},
                                                 {"duration", Value(8.0)}}));
    check(!overloaded["accepted"].as_bool() &&
              overloaded["non_overridable_failures"].as_double() >= 1.0,
          "responsible_ai: 12 kg on an 8 kg robot is a non-overridable rejection");

    expect_seed_determinism(module, "neural_network",
                            Value::object({{"epochs", Value(40)}, {"samples", Value(120)}}));
    expect_seed_determinism(module, "genetic_algorithm",
                            Value::object({{"generations", Value(6)}, {"population", Value(12)}}));
    expect_seed_determinism(module, "decision_tree", Value::object({{"samples", Value(200)}}));
    expect_seed_determinism(module, "agent_policy", Value::object());
}

// ---------------------------------------------------------------------------
// The shared seeded random source every stochastic op draws from
// ---------------------------------------------------------------------------

void test_seeded_rng() {
    std::cout << "\n--- seeded rng ---\n";
    study::sensing::SeededRng a(12345);
    study::sensing::SeededRng b(12345);
    study::sensing::SeededRng c(12346);
    bool identical = true;
    bool differs = false;
    for (int i = 0; i < 64; ++i) {
        const double x = a.uniform01();
        const double y = b.uniform01();
        const double z = c.uniform01();
        identical = identical && (x == y);
        differs = differs || (x != z);
        if (!(x > 0.0 && x < 1.0)) {
            identical = false;
        }
    }
    check(identical, "SeededRng: the same seed replays the same stream inside the open unit "
                     "interval");
    check(differs, "SeededRng: a different seed produces a different stream");

    study::sensing::SeededRng normal(7);
    std::vector<double> draws;
    draws.reserve(200000);
    for (int i = 0; i < 200000; ++i) {
        draws.push_back(normal.gaussian());
    }
    const study::sensing::SampleStats stats = study::sensing::summarise(draws);
    check(std::abs(stats.mean) < 0.01 && std::abs(stats.stddev - 1.0) < 0.01,
          "SeededRng: the Box-Muller transform is standard normal to two decimals");
    check(std::abs(stats.skewness) < 0.03, "SeededRng: and it is symmetric");

    study::sensing::SeededRng counts(9);
    double sum = 0.0;
    const int poisson_draws = 200000;
    for (int i = 0; i < poisson_draws; ++i) {
        sum += static_cast<double>(counts.poisson(4.0));
    }
    check(std::abs(sum / static_cast<double>(poisson_draws) - 4.0) < 0.05,
          "SeededRng: the Poisson sampler has the mean it was asked for");
}

}  // namespace

int main() {
    std::cout << "====================================================\n";
    std::cout << "   RUNNING YASKAWA STUDY INTELLIGENCE TEST SUITE    \n";
    std::cout << "====================================================\n";

    test_seeded_rng();
    test_sensing_models();
    test_vision_camera();
    test_learning_models();

    std::cout << "====================================================\n";
    std::cout << "checks run: " << g_checks << ", failed: " << g_failures << "\n";
    if (g_failures != 0) {
        for (const auto& name : g_failed_names) {
            std::cout << "FAILED: " << name << "\n";
        }
        std::cout << "STUDY INTELLIGENCE TESTS FAILED\n";
        return 1;
    }
    std::cout << "          ALL TESTS PASSED                          \n";
    std::cout << "====================================================\n";
    return 0;
}
