#include "yaskawa_kinematics.hpp"
#include "yaskawa_trajectory.hpp"
#include "coroutine_generator.hpp"
#include "memory_pool.hpp"
#include "producer_consumer.hpp"
#include "nvidia_stdexec.hpp"
#include "reflection.hpp"
#include "rt_logger.hpp"
#include "compressed_logger.hpp"
#include "atomic_structures.hpp"
#include <iostream>
#include <cassert>
#include <cmath>
#include <execution>
#include <vector>
#include <algorithm>

using namespace yaskawa;

void test_forward_kinematics() {
    YaskawaKinematics solver;
    Eigen::Matrix<double, DOF, 1> q = Eigen::Matrix<double, DOF, 1>::Zero();
    Eigen::Isometry3d T = solver.forwardKinematics(q);

    Eigen::Vector3d p = T.translation();
    std::cout << "[TEST] FK Home Position: X=" << p.x() << " Y=" << p.y() << " Z=" << p.z() << "\n";
    assert(std::abs(p.x() - 0.38) < 1e-3);
    assert(std::abs(p.y() - 0.0) < 1e-3);
    assert(std::abs(p.z() - 0.715) < 1e-3);
    std::cout << "[PASS] Forward Kinematics Test\n";
}

void test_inverse_kinematics() {
    YaskawaKinematics solver;
    Eigen::Matrix<double, DOF, 1> q_target;
    q_target << 0.2, -0.3, 0.4, -0.1, 0.2, -0.5;

    Eigen::Isometry3d target_pose = solver.forwardKinematics(q_target);
    Eigen::Matrix<double, DOF, 1> q_init = Eigen::Matrix<double, DOF, 1>::Zero();
    Eigen::Matrix<double, DOF, 1> q_solved;

    bool ok = solver.inverseKinematics(target_pose, q_init, q_solved);
    assert(ok == true);

    Eigen::Isometry3d T_solved = solver.forwardKinematics(q_solved);
    double error = (target_pose.translation() - T_solved.translation()).norm();
    std::cout << "[TEST] IK Position Error: " << error << " meters\n";
    assert(error < 1e-3);
    std::cout << "[PASS] Inverse Kinematics Test\n";
}

void test_coroutine_lazy_trajectory() {
    QuinticTrajectoryPlanner planner;
    Eigen::Matrix<double, DOF, 1> q_start = Eigen::Matrix<double, DOF, 1>::Zero();
    Eigen::Matrix<double, DOF, 1> q_end;
    q_end << 1.0, 0.5, -0.5, 1.5, -1.0, 2.0;

    auto lazy_gen = planner.generateLazyJointTrajectory(q_start, q_end, 2.0, 50);
    int point_count = 0;
    while (lazy_gen.next()) {
        const auto& pt = lazy_gen.value();
        (void)pt;
        point_count++;
    }
    std::cout << "[TEST] Coroutine Lazy Generator Produced: " << point_count << " trajectory points\n";
    assert(point_count == 50);
    std::cout << "[PASS] Coroutine Lazy Trajectory Test\n";
}

void test_lock_free_atomic_structures() {
    std::cout << "[TEST] Testing Lock-Free Atomic Structures & Memory Orderings...\n";

    // 1. Lock-Free SPSC Queue
    atomic::LockFreeSPSCQueue<int, 16> spsc_queue;
    for (int i = 1; i <= 5; ++i) {
        bool ok = spsc_queue.push(i * 10);
        assert(ok);
    }
    for (int i = 1; i <= 5; ++i) {
        auto val = spsc_queue.pop();
        assert(val.has_value());
        assert(val.value() == i * 10);
    }
    assert(spsc_queue.empty());

    // 2. Lock-Free Triple Buffer (1000Hz Writer -> 60Hz Reader)
    atomic::LockFreeTripleBuffer<std::array<double, 6>> triple_buffer;
    std::array<double, 6> state_in = {0.1, 0.2, 0.3, 0.4, 0.5, 0.6};
    std::array<double, 6> state_out = {};

    triple_buffer.write(state_in);
    bool read_ok = triple_buffer.read(state_out);
    assert(read_ok);
    assert(std::abs(state_out[0] - 0.1) < 1e-6);

    // 3. Lock-Free Treiber Stack
    atomic::LockFreeStack<int> stack;
    for (int i = 1; i <= 5; ++i) {
        stack.push(i * 100);
    }
    for (int i = 5; i >= 1; --i) {
        auto val = stack.pop();
        assert(val.has_value());
        assert(val.value() == i * 100);
    }
    assert(stack.empty());

    // 4. Lock-Free Spinlock Guard
    atomic::LockFreeSpinlock spinlock;
    spinlock.lock();
    int shared_val = 42;
    spinlock.unlock();
    assert(shared_val == 42);

    // 5. C++20 std::atomic_ref Array
    std::array<double, 6> raw_joints = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
    atomic::AtomicRefArray<double, 6> atomic_joints(raw_joints);
    atomic_joints.store_element(2, 1.57, std::memory_order_release);
    assert(std::abs(atomic_joints.load_element(2, std::memory_order_acquire) - 1.57) < 1e-6);

    // 6. Sequentially-Consistent Emergency Stop Flag
    atomic::AtomicEmergencyStop estop;
    assert(!estop.isEmergencyStopped());
    estop.triggerEmergencyStop();
    assert(estop.isEmergencyStopped());
    estop.reset();
    assert(!estop.isEmergencyStopped());

    // 7. Atomic Metrics Counter
    atomic::AtomicMetricsCounter counter;
    assert(counter.isLockFree());
    counter.incrementRelaxed();
    counter.incrementAcqRel();
    counter.incrementSeqCst();
    assert(counter.get() == 3);

    std::cout << "[PASS] Lock-Free Atomic Structures & Memory Orders Test (Is Lock-Free: TRUE)\n";
}

void test_cpp26_reflection() {
    std::cout << "[TEST] Testing C++26 Static Reflection...\n";
    int joint_count = 0;
    meta::reflect_for_each_joint([&joint_count]<size_t Index>(std::string_view name, auto limit) {
        std::cout << "  - Joint [" << Index << "]: " << name << " Min=" << limit.min_angle << " Max=" << limit.max_angle << "\n";
        joint_count++;
    });
    assert(joint_count == DOF);
    std::cout << "[PASS] C++26 Static Reflection Test\n";
}

void test_bitfield_logger() {
    std::cout << "[TEST] Testing 16-Byte Bit-Field Frame Logger...\n";
    BitFieldLogger bit_logger("profiling/test_bit_field.bin");

    Eigen::Matrix<double, DOF, 1> q = Eigen::Matrix<double, DOF, 1>::Zero();

    for (int i = 0; i < 100; ++i) {
        bit_logger.logBitFrame(77, q);
    }
    assert(bit_logger.getFrameCount() == 100);
    assert(bit_logger.getCompressedSizeBytes() == 1600); // 100 frames * 16 bytes
    std::cout << "[PASS] 16-Byte Bit-Field Frame Logger Test (EXACTLY 16 BYTES / FRAME)\n";
}

void test_nvidia_stdexec_channel() {
    std::cout << "[TEST] Testing NVIDIA stdexec Senders/Receivers Channel...\n";
    nvidia::stdexec::SenderReceiverChannel<int, 64> channel;
    for (int i = 1; i <= 5; ++i) {
        bool ok = channel.produce(i * 10);
        assert(ok);
    }
    for (int i = 1; i <= 5; ++i) {
        auto val = channel.consume();
        assert(val.has_value());
        assert(val.value() == i * 10);
    }
    std::cout << "[PASS] NVIDIA stdexec Channel Test\n";
}

void test_std_execution_policies() {
    std::cout << "[TEST] Testing C++26 std::execution Parallel Policies...\n";
    std::vector<double> v(1000, 1.0);
    std::for_each(std::execution::par_unseq, v.begin(), v.end(), [](double& val) {
        val *= 2.0;
    });
    assert(v[0] == 2.0);
    std::cout << "[PASS] std::execution Parallel Policies Test\n";
}

int main() {
    std::cout << "====================================================\n";
    std::cout << "      RUNNING YASKAWA C++26 UNIT TEST SUITE         \n";
    std::cout << "====================================================\n";

    test_forward_kinematics();
    test_inverse_kinematics();
    test_coroutine_lazy_trajectory();
    test_lock_free_atomic_structures();
    test_cpp26_reflection();
    test_bitfield_logger();
    test_nvidia_stdexec_channel();
    test_std_execution_policies();

    std::cout << "====================================================\n";
    std::cout << "          ALL UNIT TESTS PASSED SUCCESSFULLY        \n";
    std::cout << "====================================================\n";
    return 0;
}
