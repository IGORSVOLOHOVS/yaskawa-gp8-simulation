#ifndef YASKAWA_STUDY_VISION_CAMERA_HPP
#define YASKAWA_STUDY_VISION_CAMERA_HPP

// Course 3884 "Robotic Systems Design", Session 27 (information devices), and
// course 2953 "Intelligent Systems 1" (applied AI / perception). Exam question
// Q1(a) of course 3883 is the same chain: 0_T_p = 0_T_e * e_T_c * c_T_p.
//
// The eye-in-hand AI camera on the GP8 flange. There is no camera and no image
// library in this layer, and there must not be: the GEOMETRY and the DETECTION
// are modelled here in C++ so they can be tested without hardware, and the
// browser draws the result. Everything is referred to the single robot of
// study/gp8_model.hpp - the extrinsic transform is the DH chain times the
// hand-eye transform, never a hand-typed pose.
//
// Every stochastic op takes an integer `seed` and draws from
// sensing::SeededRng, so a panel and a test reproduce bit for bit.

#include "study/sensing_models.hpp"
#include "study/study_module.hpp"

#include <Eigen/Dense>
#include <Eigen/Geometry>

#include <string>
#include <string_view>
#include <vector>

namespace yaskawa::study {

namespace vision {

// Pinhole intrinsics in pixels, derived from a physical lens and sensor.
struct Intrinsics {
    double fx = 0.0;
    double fy = 0.0;
    double cx = 0.0;
    double cy = 0.0;
    int width = 0;
    int height = 0;

    [[nodiscard]] Eigen::Matrix3d matrix() const noexcept;
};

// fx = f[mm] / sensor_width[mm] * width[px]; the same for fy.
[[nodiscard]] Intrinsics intrinsics_from_sensor(double focal_mm, double sensor_width_mm,
                                                double sensor_height_mm, int width_px,
                                                int height_px) noexcept;

// The hand-eye transform e_T_c, flange frame to camera frame, for the three
// mountings the panel offers. Unknown names are rejected before use by
// require_enum, so this never has to guess.
[[nodiscard]] Eigen::Isometry3d hand_eye_transform(std::string_view mounting) noexcept;

// One projected point. `in_front` is false behind the lens, where the pinhole
// equations still produce a finite pixel and would silently lie.
struct Projection {
    double u = 0.0;
    double v = 0.0;
    double depth = 0.0;
    bool in_front = false;
    bool in_image = false;
};

[[nodiscard]] Projection project(const Intrinsics& K, const Eigen::Vector3d& p_cam) noexcept;

// Brown-Conrady radial distortion with two coefficients, applied in normalised
// image coordinates before the intrinsics scale them to pixels.
[[nodiscard]] Projection project_distorted(const Intrinsics& K, const Eigen::Vector3d& p_cam,
                                           double k1, double k2) noexcept;

// Geometric Jacobian of the CAMERA frame, expressed in the camera frame:
// [v; omega]_camera = J(q) qdot. Built from the DH link frames of
// study/gp8_model.hpp, so it is the same robot the rest of the layer uses.
[[nodiscard]] Eigen::Matrix<double, 6, 6> camera_jacobian(
    const Eigen::Matrix<double, 6, 1>& q, const Eigen::Isometry3d& flange_to_camera) noexcept;

// Interaction matrix of one image point, in pixels, for a camera-frame twist:
//     sdot = L(p_cam) [v; omega]
[[nodiscard]] Eigen::Matrix<double, 2, 6> image_jacobian(const Intrinsics& K,
                                                         const Eigen::Vector3d& p_cam) noexcept;

// Damped least-squares pseudo-inverse, so a rank-deficient image Jacobian
// bends the loop instead of exploding it.
[[nodiscard]] Eigen::Matrix<double, 6, 2> damped_pseudo_inverse(
    const Eigen::Matrix<double, 2, 6>& J, double damping) noexcept;

}  // namespace vision

class VisionCameraModule final : public StudyModule {
public:
    [[nodiscard]] std::string_view name() const noexcept override { return "vision_camera"; }
    [[nodiscard]] ModuleDescription describe() const override;
    [[nodiscard]] json::Value invoke(std::string_view op, const json::Value& args) const override;
};

}  // namespace yaskawa::study

#endif  // YASKAWA_STUDY_VISION_CAMERA_HPP
