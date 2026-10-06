#ifndef YASKAWA_TRAJECTORY_HPP
#define YASKAWA_TRAJECTORY_HPP

#include "yaskawa_kinematics.hpp"
#include "coroutine_generator.hpp"
#include <vector>

namespace yaskawa {

struct TrajectoryPoint {
    double time;
    Eigen::Matrix<double, DOF, 1> position;
    Eigen::Matrix<double, DOF, 1> velocity;
    Eigen::Matrix<double, DOF, 1> acceleration;
};

class QuinticTrajectoryPlanner {
public:
    QuinticTrajectoryPlanner() = default;

    // Standard trajectory generation (pre-allocated vector)
    std::vector<TrajectoryPoint> planJointTrajectory(
        const Eigen::Matrix<double, DOF, 1>& q_start,
        const Eigen::Matrix<double, DOF, 1>& q_end,
        double duration,
        int num_samples = 100
    ) const;

    // C++20/C++23 Coroutine Lazy Evaluation Trajectory Generator
    // Evaluates trajectory points on-demand with zero vector allocations!
    Generator<TrajectoryPoint> generateLazyJointTrajectory(
        Eigen::Matrix<double, DOF, 1> q_start,
        Eigen::Matrix<double, DOF, 1> q_end,
        double duration,
        int num_samples = 100
    ) const;
};

} // namespace yaskawa

#endif // YASKAWA_TRAJECTORY_HPP
