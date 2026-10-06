#include "yaskawa_kinematics.hpp"

namespace yaskawa {

// Thread-local pre-allocated workspace buffer for zero-allocation IK and Jacobian calculation
static thread_local IKWorkspaceBuffer tls_ik_ws;

Eigen::Isometry3d YaskawaKinematics::forwardKinematics(const Eigen::Matrix<double, DOF, 1>& q) const noexcept {
    // Pre-calculate sine and cosine cache for maximum speed
    std::array<double, DOF> sin_q, cos_q;
    for (size_t i = 0; i < DOF; ++i) {
        sin_q[i] = std::sin(q[i]);
        cos_q[i] = std::cos(q[i]);
    }
    return forwardKinematicsCached(sin_q, cos_q);
}

Eigen::Isometry3d YaskawaKinematics::forwardKinematicsCached(
    const std::array<double, DOF>& sin_q,
    const std::array<double, DOF>& cos_q
) const noexcept {
    Eigen::Isometry3d T = Eigen::Isometry3d::Identity();

    // Joint 1 (S): Translation (0, 0, 0.33), Rotation Z(q[0])
    T.translate(Eigen::Vector3d(0.0, 0.0, 0.33));
    Eigen::Matrix3d R1;
    R1 << cos_q[0], -sin_q[0], 0.0,
          sin_q[0],  cos_q[0], 0.0,
          0.0,       0.0,      1.0;
    T.rotate(R1);

    // Joint 2 (L): Translation (0.04, 0.0, 0.0), Rotation Y(q[1])
    T.translate(Eigen::Vector3d(0.04, 0.0, 0.0));
    Eigen::Matrix3d R2;
    R2 <<  cos_q[1], 0.0, sin_q[1],
           0.0,      1.0, 0.0,
          -sin_q[1], 0.0, cos_q[1];
    T.rotate(R2);

    // Joint 3 (U): Translation (0.0, 0.0, 0.345), Rotation Y(q[2])
    T.translate(Eigen::Vector3d(0.0, 0.0, 0.345));
    Eigen::Matrix3d R3;
    R3 <<  cos_q[2], 0.0, sin_q[2],
           0.0,      1.0, 0.0,
          -sin_q[2], 0.0, cos_q[2];
    T.rotate(R3);

    // Joint 4 (R): Translation (0.34, 0.0, 0.04), Rotation X(q[3])
    T.translate(Eigen::Vector3d(0.34, 0.0, 0.04));
    Eigen::Matrix3d R4;
    R4 << 1.0, 0.0,      0.0,
          0.0, cos_q[3], -sin_q[3],
          0.0, sin_q[3],  cos_q[3];
    T.rotate(R4);

    // Joint 5 (B): Translation (0.15, 0.0, 0.0), Rotation Y(q[4])
    T.translate(Eigen::Vector3d(0.15, 0.0, 0.0));
    Eigen::Matrix3d R5;
    R5 <<  cos_q[4], 0.0, sin_q[4],
           0.0,      1.0, 0.0,
          -sin_q[4], 0.0, cos_q[4];
    T.rotate(R5);

    // Joint 6 (T): Translation (0.0, 0.0, 0.0), Rotation X(q[5])
    Eigen::Matrix3d R6;
    R6 << 1.0, 0.0,      0.0,
          0.0, cos_q[5], -sin_q[5],
          0.0, sin_q[5],  cos_q[5];
    T.rotate(R6);

    return T;
}

Eigen::Matrix<double, 6, DOF> YaskawaKinematics::computeJacobian(const Eigen::Matrix<double, DOF, 1>& q) const noexcept {
    const double eps = 1e-6;

    Eigen::Isometry3d T0 = forwardKinematics(q);
    Eigen::Vector3d p0 = T0.translation();
    Eigen::Matrix3d R0 = T0.rotation();

    for (size_t i = 0; i < DOF; ++i) {
        tls_ik_ws.q_plus = q;
        tls_ik_ws.q_plus[i] += eps;

        tls_ik_ws.T_plus = forwardKinematics(tls_ik_ws.q_plus);
        Eigen::Vector3d p_plus = tls_ik_ws.T_plus.translation();
        Eigen::Matrix3d R_plus = tls_ik_ws.T_plus.rotation();

        Eigen::Vector3d dp = (p_plus - p0) / eps;
        Eigen::Matrix3d dR = (R_plus - R0) / eps;
        Eigen::Matrix3d omega_skew = dR * R0.transpose();
        Eigen::Vector3d w(omega_skew(2, 1), omega_skew(0, 2), omega_skew(1, 0));

        tls_ik_ws.J.block<3, 1>(0, i) = dp;
        tls_ik_ws.J.block<3, 1>(3, i) = w;
    }

    return tls_ik_ws.J;
}

bool YaskawaKinematics::inverseKinematics(
    const Eigen::Isometry3d& target_pose,
    const Eigen::Matrix<double, DOF, 1>& q_init,
    Eigen::Matrix<double, DOF, 1>& q_out,
    double pos_tol,
    double rot_tol,
    int max_iters
) const noexcept {
    // ZERO HEAP ALLOCATION - uses thread-local pre-allocated workspace tls_ik_ws
    q_out = clampJoints(q_init);
    double lambda = 1e-3;

    const Eigen::Vector3d& p_target = target_pose.translation();
    const Eigen::Matrix3d& R_target = target_pose.rotation();

    for (int iter = 0; iter < max_iters; ++iter) {
        tls_ik_ws.T_curr = forwardKinematics(q_out);
        Eigen::Vector3d pos_err = p_target - tls_ik_ws.T_curr.translation();
        Eigen::Matrix3d R_err = R_target * tls_ik_ws.T_curr.rotation().transpose();
        Eigen::AngleAxisd aa(R_err);
        Eigen::Vector3d rot_err = aa.angle() * aa.axis();

        if (pos_err.norm() < pos_tol && rot_err.norm() < rot_tol) {
            return true;
        }

        tls_ik_ws.error.block<3, 1>(0, 0) = pos_err;
        tls_ik_ws.error.block<3, 1>(3, 0) = rot_err;

        tls_ik_ws.J = computeJacobian(q_out);
        tls_ik_ws.JJt = tls_ik_ws.J * tls_ik_ws.J.transpose() + (lambda * lambda) * Eigen::Matrix<double, 6, 6>::Identity();
        tls_ik_ws.dq = tls_ik_ws.J.transpose() * tls_ik_ws.JJt.ldlt().solve(tls_ik_ws.error);

        q_out = clampJoints(q_out + tls_ik_ws.dq);
    }

    Eigen::Isometry3d T_final = forwardKinematics(q_out);
    Eigen::Vector3d pos_err = p_target - T_final.translation();
    Eigen::Matrix3d R_err = target_pose.rotation() * T_final.rotation().transpose();
    Eigen::AngleAxisd aa(R_err);

    return (pos_err.norm() < pos_tol * 2.0 && std::abs(aa.angle()) < rot_tol * 2.0);
}

} // namespace yaskawa
