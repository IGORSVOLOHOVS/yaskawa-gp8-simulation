#include "yaskawa_kinematics.hpp"
#include "yaskawa_trajectory.hpp"
#include "memory_pool.hpp"
#include "producer_consumer.hpp"
#include "thread_pool.hpp"
#include "nvidia_stdexec.hpp"
#include "reflection.hpp"
#include "rt_logger.hpp"
#include "compressed_logger.hpp"
#include "atomic_structures.hpp"
#include "robot_physical_tree.hpp"
#include <chrono>
#include <iostream>
#include <iomanip>
#include <vector>
#include <numeric>
#include <random>
#include <execution>
#include <algorithm>
#include <unordered_map>

#if __has_include(<flat_map>)
#include <flat_map>
#define HAS_FLAT_MAP 1
#else
#include <map>
#define HAS_FLAT_MAP 0
#endif

using namespace yaskawa;

int main() {
    std::cout << "=================================================================\n";
    std::cout << "    YASKAWA GP8 C++26 REAL-TIME SOLVER & ATOMICS BENCHMARK       \n";
    std::cout << "=================================================================\n\n";

    YaskawaKinematics solver;
    QuinticTrajectoryPlanner planner;
    ThreadPoolScheduler thread_pool;
    BitFieldLogger bit_logger("profiling/ultra_compact_trace.bin");

    std::cout << "System CPU Cores Active in Thread Pool: " << thread_pool.workerCount() << "\n";
    std::cout << "Reflected Type Name: " << meta::reflect_type_name<YaskawaKinematics>() << "\n\n";

    // -------------------------------------------------------------------------
    // Benchmark 1: Forward Kinematics (FK) Micro-Benchmark
    // -------------------------------------------------------------------------
    const int fk_iterations = 1000000;
    Eigen::Matrix<double, DOF, 1> q_test;
    q_test << 0.1, -0.2, 0.3, -0.4, 0.5, -0.6;

    auto fk_start = std::chrono::high_resolution_clock::now();
    volatile double dummy_sum = 0.0;
    for (int i = 0; i < fk_iterations; ++i) {
        q_test[0] = 0.1 + (i % 100) * 0.001;
        Eigen::Isometry3d T = solver.forwardKinematics(q_test);
        dummy_sum += T.translation().x();
    }
    auto fk_end = std::chrono::high_resolution_clock::now();
    std::chrono::duration<double, std::micro> fk_duration = fk_end - fk_start;
    double fk_avg_us = fk_duration.count() / fk_iterations;
    double fk_ops_per_sec = fk_iterations / (fk_duration.count() / 1e6);

    Eigen::Isometry3d T_sample = solver.forwardKinematics(q_test);
    bit_logger.logBitFrame(1, q_test);

    std::cout << "1. FORWARD KINEMATICS (FK) SPEED:\n";
    std::cout << "   - Total Iterations : " << fk_iterations << "\n";
    std::cout << "   - Total Time       : " << std::fixed << std::setprecision(3) << fk_duration.count() / 1000.0 << " ms\n";
    std::cout << "   - Average Latency  : " << std::setprecision(1) << (fk_avg_us * 1000.0) << " ns (" << fk_avg_us << " us)\n";
    std::cout << "   - Throughput       : " << std::setprecision(0) << fk_ops_per_sec << " FK calls / sec\n\n";

    // -------------------------------------------------------------------------
    // Benchmark 2: Lock-Free Atomic Structures & Memory Orders Throughput
    // -------------------------------------------------------------------------
    const int atomic_iterations = 10000000;
    atomic::AtomicMetricsCounter counter;

    auto atomic_start = std::chrono::high_resolution_clock::now();
    for (int i = 0; i < atomic_iterations; ++i) {
        counter.incrementRelaxed();
    }
    auto atomic_end = std::chrono::high_resolution_clock::now();
    std::chrono::duration<double, std::milli> atomic_ms = atomic_end - atomic_start;
    double atomic_ops_per_sec = atomic_iterations / (atomic_ms.count() / 1000.0);

    // Benchmark 2b: Lock-Free SPSC Queue Throughput
    atomic::LockFreeSPSCQueue<int, 4096> spsc_bench;
    const int spsc_iterations = 5000000;
    auto spsc_start = std::chrono::high_resolution_clock::now();
    for (int i = 0; i < spsc_iterations; ++i) {
        spsc_bench.push(i);
        spsc_bench.pop();
    }
    auto spsc_end = std::chrono::high_resolution_clock::now();
    std::chrono::duration<double, std::milli> spsc_ms = spsc_end - spsc_start;
    double spsc_ops_per_sec = (spsc_iterations * 2) / (spsc_ms.count() / 1000.0);

    // Benchmark 2c: Lock-Free Triple Buffer (Writer -> Reader)
    atomic::LockFreeTripleBuffer<std::array<double, 6>> tb_bench;
    std::array<double, 6> dummy_state = {0.1, 0.2, 0.3, 0.4, 0.5, 0.6};
    std::array<double, 6> out_state;
    const int tb_iterations = 5000000;
    auto tb_start = std::chrono::high_resolution_clock::now();
    for (int i = 0; i < tb_iterations; ++i) {
        tb_bench.write(dummy_state);
        tb_bench.read(out_state);
    }
    auto tb_end = std::chrono::high_resolution_clock::now();
    std::chrono::duration<double, std::milli> tb_ms = tb_end - tb_start;
    double tb_ops_per_sec = (tb_iterations * 2) / (tb_ms.count() / 1000.0);

    std::cout << "2. LOCK-FREE ATOMIC STRUCTURES & MEMORY ORDERS:\n";
    std::cout << "   - Atomic Increment (std::memory_order_relaxed) : " << std::setprecision(0) << atomic_ops_per_sec << " ops / sec\n";
    std::cout << "   - Lock-Free SPSC Queue Throughput              : " << std::setprecision(0) << spsc_ops_per_sec << " push+pop / sec\n";
    std::cout << "   - Lock-Free Triple Buffer Throughput          : " << std::setprecision(0) << tb_ops_per_sec << " write+read / sec\n";
    std::cout << "   - Hardware Lock-Free Guarantee                 : " << (counter.isLockFree() ? "TRUE (Lock-Free)" : "FALSE") << "\n\n";

    // -------------------------------------------------------------------------
    // Benchmark 3: Parallel std::execution::par_unseq IK Solving
    // -------------------------------------------------------------------------
    const int ik_total_samples = 10000;
    std::vector<Eigen::Matrix<double, DOF, 1>> targets(ik_total_samples);
    std::mt19937 rng(42);
    std::uniform_real_distribution<double> dist_s(-2.0, 2.0);
    std::uniform_real_distribution<double> dist_l(-0.5, 1.5);
    std::uniform_real_distribution<double> dist_u(-0.5, 2.0);

    for (int i = 0; i < ik_total_samples; ++i) {
        targets[i] << dist_s(rng), dist_l(rng), dist_u(rng), 0.0, 0.0, 0.0;
    }

    std::atomic<int> success_count(0);
    auto ik_parallel_start = std::chrono::high_resolution_clock::now();

    std::for_each(std::execution::par_unseq, targets.begin(), targets.end(), [&solver, &success_count](const auto& q_tgt) {
        Eigen::Isometry3d target_pose = solver.forwardKinematics(q_tgt);
        Eigen::Matrix<double, DOF, 1> q_init = Eigen::Matrix<double, DOF, 1>::Zero();
        Eigen::Matrix<double, DOF, 1> q_solved;
        if (solver.inverseKinematics(target_pose, q_init, q_solved)) {
            success_count.fetch_add(1, std::memory_order_relaxed);
        }
    });

    auto ik_parallel_end = std::chrono::high_resolution_clock::now();
    std::chrono::duration<double, std::milli> ik_wall_ms = ik_parallel_end - ik_parallel_start;
    double ik_parallel_rate = (100.0 * success_count.load()) / ik_total_samples;
    double ik_parallel_ops_per_sec = (ik_total_samples) / (ik_wall_ms.count() / 1000.0);

    std::cout << "3. PARALLEL IK (C++26 std::execution::par_unseq Policy):\n";
    std::cout << "   - Total Poses Solved       : " << ik_total_samples << "\n";
    std::cout << "   - Parallel Wall-Clock Time : " << std::setprecision(2) << ik_wall_ms.count() << " ms\n";
    std::cout << "   - Overall IK Success Rate  : " << std::setprecision(1) << ik_parallel_rate << " %\n";
    std::cout << "   - Parallel Throughput      : " << std::setprecision(0) << ik_parallel_ops_per_sec << " IK solutions / sec\n\n";

    // -------------------------------------------------------------------------
    // Benchmark 4: C++23 std::flat_map vs std::unordered_map Component Lookup
    // -------------------------------------------------------------------------
    size_t total_comps = yaskawa::physical::get_total_component_count();
#if HAS_FLAT_MAP
    std::flat_map<std::string, const physical::ComponentSpec*> map_index;
#else
    std::map<std::string, const physical::ComponentSpec*> map_index;
#endif
    std::unordered_map<std::string, const physical::ComponentSpec*> unord_index;
    unord_index.reserve(total_comps);

    for (size_t i = 0; i < total_comps; ++i) {
        const auto* comp = yaskawa::physical::get_component_at(i);
        map_index[comp->id] = comp;
        unord_index[comp->id] = comp;
    }

    const int map_lookup_cycles = 10000;
    auto map_start = std::chrono::high_resolution_clock::now();
    size_t map_found = 0;
    for (int cycle = 0; cycle < map_lookup_cycles; ++cycle) {
        for (size_t i = 0; i < total_comps; ++i) {
            const auto* comp = yaskawa::physical::get_component_at(i);
            auto it = map_index.find(comp->id);
            if (it != map_index.end()) {
                map_found++;
            }
        }
    }
    auto map_end = std::chrono::high_resolution_clock::now();
    std::chrono::duration<double, std::milli> map_ms = map_end - map_start;
    double map_ops_per_sec = (map_lookup_cycles * total_comps) / (map_ms.count() / 1000.0);

    auto unord_start = std::chrono::high_resolution_clock::now();
    size_t unord_found = 0;
    for (int cycle = 0; cycle < map_lookup_cycles; ++cycle) {
        for (size_t i = 0; i < total_comps; ++i) {
            const auto* comp = yaskawa::physical::get_component_at(i);
            auto it = unord_index.find(comp->id);
            if (it != unord_index.end()) {
                unord_found++;
            }
        }
    }
    auto unord_end = std::chrono::high_resolution_clock::now();
    std::chrono::duration<double, std::milli> unord_ms = unord_end - unord_start;
    double unord_ops_per_sec = (map_lookup_cycles * total_comps) / (unord_ms.count() / 1000.0);

#if HAS_FLAT_MAP
    std::cout << "4. C++23 std::flat_map vs std::unordered_map LOOKUP THROUGHPUT:\n";
    std::cout << "   - Components Indexed       : " << total_comps << "\n";
    std::cout << "   - std::flat_map Time       : " << std::fixed << std::setprecision(2) << map_ms.count() << " ms (" << std::setprecision(0) << map_ops_per_sec << " lookups/sec)\n";
    std::cout << "   - std::unordered_map Time  : " << std::setprecision(2) << unord_ms.count() << " ms (" << std::setprecision(0) << unord_ops_per_sec << " lookups/sec)\n";
    std::cout << "   - Cache Locality Advantage : std::flat_map contiguous keys & values layout\n\n";
#else
    std::cout << "4. Associative std::map vs std::unordered_map LOOKUP THROUGHPUT:\n";
    std::cout << "   - Components Indexed       : " << total_comps << "\n";
    std::cout << "   - std::map Time            : " << std::fixed << std::setprecision(2) << map_ms.count() << " ms (" << std::setprecision(0) << map_ops_per_sec << " lookups/sec)\n";
    std::cout << "   - std::unordered_map Time  : " << std::setprecision(2) << unord_ms.count() << " ms (" << std::setprecision(0) << unord_ops_per_sec << " lookups/sec)\n";
    std::cout << "   - Container Layout         : std::unordered_map hash buckets vs red-black tree\n\n";
#endif

    std::cout << "=================================================================\n";
    std::cout << "      C++26 REAL-TIME ENGINE BENCHMARK COMPLETED SUCCESSFULLY    \n";
    std::cout << "=================================================================\n";

    return 0;
}
