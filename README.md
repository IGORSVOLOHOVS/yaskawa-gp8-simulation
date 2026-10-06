# Yaskawa GP8 Kinematics & Simulation Engine (`waam-manipulator`)

High-performance C++26 Yaskawa GP8 robot kinematics solver, coroutine lazy trajectory planner, WebGL 3D simulator, and benchmarking engine.

This repository complies with revision **4.0.0** of [BEST_REQUIREMENTS-v4.0.0.md](file:///home/igors/Downloads/BEST_REQUIREMENTS-v4.0.0.md).

---

## Quick Start

Initialize environment and run tests:
```bash
./robot init
./robot build release
./robot test
```

Run performance benchmark and export callgrind / gprof bottleneck traces:
```bash
./robot benchmark
```

Launch the interactive 3D Web Dashboard:
```bash
./robot run
```

---

## CLI Commands (`./robot`)

| Command | Description |
| :--- | :--- |
| `./robot init` | Initialize environment and verify build toolchain |
| `./robot build [profile]` | Build C++26 solver (`release`, `debug`, `profile`, `pgo`) |
| `./robot test` | Run C++26 unit test suite |
| `./robot benchmark` | Run high-resolution benchmark & export bottleneck profiles |
| `./robot run [web\|ros]` | Launch Web 3D Dashboard or ROS 2 MoveIt scene |
| `./robot install` | Install binaries |
| `./robot uninstall` | Remove installed binaries |
| `./robot clean` | Clean build and profiling outputs |
| `./robot info` | Display project metadata |
