#include "yaskawa_trajectory.hpp"
#include <cmath>

namespace yaskawa {

std::vector<TrajectoryPoint> QuinticTrajectoryPlanner::planJointTrajectory(
    const Eigen::Matrix<double, DOF, 1>& q_start,
    const Eigen::Matrix<double, DOF, 1>& q_end,
    double duration,
    int num_samples
) const {
    std::vector<TrajectoryPoint> trajectory;
    if (duration <= 0.0 || num_samples < 2) return trajectory;

    trajectory.reserve(num_samples);
    double dt = duration / (num_samples - 1);

    for (int i = 0; i < num_samples; ++i) {
        double t = i * dt;
        double s = t / duration; // normalized time [0, 1]

        double s3 = s * s * s;
        double s4 = s3 * s;
        double s5 = s4 * s;

        double poly_pos = 10.0 * s3 - 15.0 * s4 + 6.0 * s5;
        double poly_vel = (30.0 * s * s - 60.0 * s3 + 30.0 * s4) / duration;
        double poly_acc = (60.0 * s - 180.0 * s * s + 120.0 * s3) / (duration * duration);

        TrajectoryPoint pt;
        pt.time = t;
        pt.position = q_start + poly_pos * (q_end - q_start);
        pt.velocity = poly_vel * (q_end - q_start);
        pt.acceleration = poly_acc * (q_end - q_start);

        trajectory.push_back(pt);
    }

    return trajectory;
}

Generator<TrajectoryPoint> QuinticTrajectoryPlanner::generateLazyJointTrajectory(
    Eigen::Matrix<double, DOF, 1> q_start,
    Eigen::Matrix<double, DOF, 1> q_end,
    double duration,
    int num_samples
) const {
    if (duration <= 0.0 || num_samples < 2) co_return;

    double dt = duration / (num_samples - 1);

    for (int i = 0; i < num_samples; ++i) {
        double t = i * dt;
        double s = t / duration;

        double s3 = s * s * s;
        double s4 = s3 * s;
        double s5 = s4 * s;

        double poly_pos = 10.0 * s3 - 15.0 * s4 + 6.0 * s5;
        double poly_vel = (30.0 * s * s - 60.0 * s3 + 30.0 * s4) / duration;
        double poly_acc = (60.0 * s - 180.0 * s * s + 120.0 * s3) / (duration * duration);

        TrajectoryPoint pt;
        pt.time = t;
        pt.position = q_start + poly_pos * (q_end - q_start);
        pt.velocity = poly_vel * (q_end - q_start);
        pt.acceleration = poly_acc * (q_end - q_start);

        co_yield pt;
    }
}

} // namespace yaskawa
