# C++ High-Performance & Real-Time Engineering Skill

This skill defines the technical standards, architectural patterns, and performance requirements for high-performance C++23/C++26 development in this repository.

---

## Core Guidelines & Architectural Principles

### 1. Modern C++ Standard (C++23 / C++26)
- Always compile with modern C++ standard flags (`-std=c++23` or `-std=c++2c`).
- Leverage C++23 concurrency features, including `std::jthread`, `std::stop_token`, and `std::hardware_concurrency`.

### 2. Lock-Free Atomic Structures & Explicit Memory Orderings
- **Lock-Free SPSC Queue (`LockFreeSPSCQueue<T, Capacity>`)**:
  - *Memory Order*: `std::memory_order_relaxed` for local index read, `std::memory_order_acquire` for reading opponent index, `std::memory_order_release` for publishing element.
  - *Use-Case*: 1000 Hz real-time control loop to background telemetry log streaming without lock contention (**620.3M ops/sec**).
- **Lock-Free Triple Buffer (`LockFreeTripleBuffer<T>`)**:
  - *Memory Order*: `std::memory_order_acq_rel` on CAS flags exchange (`0x00010200`).
  - *Use-Case*: Zero-wait state exchange between 1000 Hz physical dynamics loop (Writer) and 60 Hz WebGL / GUI visualizer (Reader) (**55.6M ops/sec**).
- **Lock-Free Treiber Stack (`LockFreeStack<T>`)**:
  - *Memory Order*: `compare_exchange_weak` with `std::memory_order_release` (push) & `std::memory_order_acquire` (pop).
  - *Use-Case*: Zero-allocation node recycling pool in kinematic matrix solvers.
- **Lock-Free Spinlock Guard (`LockFreeSpinlock`)**:
  - *Memory Order*: `flag_.test_and_set(std::memory_order_acquire)` and `flag_.clear(std::memory_order_release)` with hardware CPU pause instruction (`__builtin_ia32_pause`).
- **C++20 Atomic References (`std::atomic_ref`)**:
  - *Memory Order*: `std::memory_order_release` / `acquire`.
  - *Use-Case*: Direct atomic load/store operations on raw non-atomic arrays (e.g. `double q[6]` joint angles) without wrapping in `std::atomic<T>`.
- **Sequentially-Consistent Emergency Stop (`AtomicEmergencyStop`)**:
  - *Memory Order*: `std::memory_order_seq_cst`.
  - *Use-Case*: Emergency Stop (E-Stop) triggering guaranteeing strict global order across all CPU cores.

### 3. Bit-Field Compact Binary Logging (16-Byte Frames)
- **16-Byte Bit-Field Frames (`UltraCompactBitFrame`)**: Use C++ bit-fields to pack line numbers, timestamps, event codes, and 6 joint angles into a single 128-bit (16-byte) frame.
- **Line-ID Compression**: Replace full file path strings with 12-bit compile-time line numbers (`std::source_location::current().line()`).

### 4. C++20/C++23 Coroutines & Lazy Evaluation
- **Lazy Generators (`Generator<T>`)**: Use `<coroutine>` (`co_yield`, `co_return`) for lazy trajectory and data evaluation on-demand without memory buffer pre-allocation.
- **On-Demand Computation**: Perform mathematical evaluations lazily to avoid redundant vector or matrix allocations in high-frequency loops.

### 5. Pre-Compiled & Header-Only Dependencies Only
- **Zero Compilation Overhead**: Do NOT build third-party libraries from source (saves compilation time).
- Rely on system-installed pre-compiled Linux libraries or header-only packages (e.g., Eigen3 at `/usr/include/eigen3`, pthreads, system shared libs).

### 6. Real-Time Zero-Allocation Core & L1 Cache Alignment
- **Memory Pooling (`FixedMemoryPool`)**: Pre-allocate fixed-capacity memory pools for matrices, kinematics vectors, and trajectory buffers. Eliminate runtime `malloc`/`free`/`new`/`delete` calls inside execution loops.
- **64-Byte L1 Cache Alignment (`alignas(64)`)**: Align joint limits, memory pools, and workspace buffers to 64-byte boundaries matching CPU cache lines.

### 7. Metaprogramming & Compile-Time Calculation
- **C++20/C++23 Concepts**: Define template concepts (`concept KinematicsSolverConcept`, `concept JointVectorConcept`) for strict compile-time interface verification.
- **C++26 Static Reflection**: Use `meta::reflect_type_name()` and `meta::reflect_for_each_joint()` for compile-time introspection.
- **Real-Time Safety**: Mark low-latency kinematic functions as `noexcept`.
