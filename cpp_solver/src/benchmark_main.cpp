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
#include "portable_print.hpp"
#include <chrono>
#include <vector>
#include <numeric>
#include <random>
#include <execution>
#include <algorithm>
#include <ranges>
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
    std::println("=================================================================");
    std::println("    YASKAWA GP8 C++26 REAL-TIME SOLVER & ATOMICS BENCHMARK       ");
    std::println("=================================================================\n");

    YaskawaKinematics solver;
    QuinticTrajectoryPlanner planner;
    ThreadPoolScheduler thread_pool;
    BitFieldLogger bit_logger("profiling/ultra_compact_trace.bin");

    std::println("System CPU Cores Active in Thread Pool: {}", thread_pool.workerCount());
    std::println("Reflected Type Name: {}\n", meta::reflect_type_name<YaskawaKinematics>());

    // -------------------------------------------------------------------------
    // Benchmark 1: Forward Kinematics (FK) Micro-Benchmark
    // -------------------------------------------------------------------------
    const int fk_iterations = 1000000;
    Eigen::Matrix<double, DOF, 1> q_test;
    q_test << 0.1, -0.2, 0.3, -0.4, 0.5, -0.6;

    auto fk_start = std::chrono::high_resolution_clock::now();
    volatile double dummy_sum = 0.0;
    for (int i : std::views::iota(0, fk_iterations)) {
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

    std::println("1. FORWARD KINEMATICS (FK) SPEED:");
    std::println("   - Total Iterations : {}", fk_iterations);
    std::println("   - Total Time       : {:.3f} ms", fk_duration.count() / 1000.0);
    std::println("   - Average Latency  : {:.1f} ns ({:.3f} us)", fk_avg_us * 1000.0, fk_avg_us);
    std::println("   - Throughput       : {:.0f} FK calls / sec\n", fk_ops_per_sec);

    // -------------------------------------------------------------------------
    // Benchmark 2: Lock-Free Atomic Structures & Memory Orders Throughput
    // -------------------------------------------------------------------------
    const int atomic_iterations = 10000000;
    atomic::AtomicMetricsCounter counter;

    auto atomic_start = std::chrono::high_resolution_clock::now();
    for (int i : std::views::iota(0, atomic_iterations)) {
        counter.incrementRelaxed();
    }
    auto atomic_end = std::chrono::high_resolution_clock::now();
    std::chrono::duration<double, std::milli> atomic_ms = atomic_end - atomic_start;
    double atomic_ops_per_sec = atomic_iterations / (atomic_ms.count() / 1000.0);

    // Benchmark 2b: Lock-Free SPSC Queue Throughput
    atomic::LockFreeSPSCQueue<int, 4096> spsc_bench;
    const int spsc_iterations = 5000000;
    auto spsc_start = std::chrono::high_resolution_clock::now();
    for (int i : std::views::iota(0, spsc_iterations)) {
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
    for (int i : std::views::iota(0, tb_iterations)) {
        tb_bench.write(dummy_state);
        tb_bench.read(out_state);
    }
    auto tb_end = std::chrono::high_resolution_clock::now();
    std::chrono::duration<double, std::milli> tb_ms = tb_end - tb_start;
    double tb_ops_per_sec = (tb_iterations * 2) / (tb_ms.count() / 1000.0);

    std::println("2. LOCK-FREE ATOMIC STRUCTURES & MEMORY ORDERS:");
    std::println("   - Atomic Increment (std::memory_order_relaxed) : {:.0f} ops / sec", atomic_ops_per_sec);
    std::println("   - Lock-Free SPSC Queue Throughput              : {:.0f} push+pop / sec", spsc_ops_per_sec);
    std::println("   - Lock-Free Triple Buffer Throughput          : {:.0f} write+read / sec", tb_ops_per_sec);
    std::println("   - Hardware Lock-Free Guarantee                 : {}\n", counter.isLockFree() ? "TRUE (Lock-Free)" : "FALSE");

    // -------------------------------------------------------------------------
    // Benchmark 3: Parallel std::execution::par_unseq IK Solving
    // -------------------------------------------------------------------------
    const int ik_total_samples = 10000;
    std::vector<Eigen::Matrix<double, DOF, 1>> targets(ik_total_samples);
    std::mt19937 rng(42);
    std::uniform_real_distribution<double> dist_s(-2.0, 2.0);
    std::uniform_real_distribution<double> dist_l(-0.5, 1.5);
    std::uniform_real_distribution<double> dist_u(-0.5, 2.0);

    for (int i : std::views::iota(0, ik_total_samples)) {
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

    std::println("3. PARALLEL IK (C++26 std::execution::par_unseq Policy):");
    std::println("   - Total Poses Solved       : {}", ik_total_samples);
    std::println("   - Parallel Wall-Clock Time : {:.2f} ms", ik_wall_ms.count());
    std::println("   - Overall IK Success Rate  : {:.1f} %", ik_parallel_rate);
    std::println("   - Parallel Throughput      : {:.0f} IK solutions / sec\n", ik_parallel_ops_per_sec);

    // -------------------------------------------------------------------------
    // Benchmark 4: C++23 std::flat_map vs std::unordered_map Component Lookup
    // -------------------------------------------------------------------------
    size_t total_comps = yaskawa::physical::get_total_component_count();
#if HAS_FLAT_MAP
    std::flat_map<std::string, physical::ComponentPtr> map_index;
#else
    std::map<std::string, physical::ComponentPtr> map_index;
#endif
    std::unordered_map<std::string, physical::ComponentPtr> unord_index;
    unord_index.reserve(total_comps);

    for (size_t i : std::views::iota(size_t{0}, total_comps)) {
        auto comp = yaskawa::physical::get_component_at(i);
        map_index[comp->id] = comp;
        unord_index[comp->id] = comp;
    }

    const int map_lookup_cycles = 10000;
    auto map_start = std::chrono::high_resolution_clock::now();
    size_t map_found = 0;
    for (int cycle : std::views::iota(0, map_lookup_cycles)) {
        for (size_t i : std::views::iota(size_t{0}, total_comps)) {
            auto comp = yaskawa::physical::get_component_at(i);
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
    for (int cycle : std::views::iota(0, map_lookup_cycles)) {
        for (size_t i : std::views::iota(size_t{0}, total_comps)) {
            auto comp = yaskawa::physical::get_component_at(i);
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
    std::println("4. C++23 std::flat_map vs std::unordered_map LOOKUP THROUGHPUT:");
    std::println("   - Components Indexed       : {}", total_comps);
    std::println("   - std::flat_map Time       : {:.2f} ms ({:.0f} lookups/sec)", map_ms.count(), map_ops_per_sec);
    std::println("   - std::unordered_map Time  : {:.2f} ms ({:.0f} lookups/sec)", unord_ms.count(), unord_ops_per_sec);
    std::println("   - Cache Locality Advantage : std::flat_map contiguous keys & values layout\n");
#else
    std::println("4. Associative std::map vs std::unordered_map LOOKUP THROUGHPUT:");
    std::println("   - Components Indexed       : {}", total_comps);
    std::println("   - std::map Time            : {:.2f} ms ({:.0f} lookups/sec)", map_ms.count(), map_ops_per_sec);
    std::println("   - std::unordered_map Time  : {:.2f} ms ({:.0f} lookups/sec)", unord_ms.count(), unord_ops_per_sec);
    std::println("   - Container Layout         : std::unordered_map hash buckets vs red-black tree\n");
#endif

    std::println("=================================================================");
    std::println("      C++26 REAL-TIME ENGINE BENCHMARK COMPLETED SUCCESSFULLY    ");
    std::println("=================================================================");

    return 0;
}
