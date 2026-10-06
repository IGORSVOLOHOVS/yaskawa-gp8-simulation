# User Manual

## Overview
The Yaskawa GP8 Kinematics & Simulation Engine provides microsecond-level forward and inverse kinematics calculations, coroutine lazy trajectory planning, and a WebGL 3D visualization dashboard.

## Running the Web Dashboard
Launch the standalone web dashboard:
```bash
./robot run
```
Open `http://localhost:8080` in your web browser to interact with joint controls, end-effector targets, and real-time trajectory visualization.

## Running Benchmarks
To run high-resolution microsecond benchmarks and generate Callgrind / gprof bottleneck traces:
```bash
./robot benchmark
```
Bottleneck traces will be saved in `profiling/profile_report.txt` and `profiling/callgrind.out`.
