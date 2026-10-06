# Yaskawa GP8 Robot Project Guidelines & Rules

Compliance with **BEST_REQUIREMENTS-v4.0.0.md** (§7).

## Commands
- `./robot init` : Initialize environment & verify C++23 dev tools
- `./robot build [release|debug|profile|pgo]` : Build C++23 solver with selected profile
- `./robot test` : Run C++23 unit test suite
- `./robot benchmark` : Run performance benchmark and save callgrind/gprof trace files to `profiling/`
- `./robot run [web|ros]` : Launch 3D Web Dashboard or ROS 2 MoveIt scene
- `./robot debug` : Build and debug target with ASan

## C++ Standards & Skill
Refer to [.claude/skills/cpp-optimization/SKILL.md](file:///home/igors/Projects/Robot/.claude/skills/cpp-optimization/SKILL.md) for full C++23 metaprogramming, memory pool, lock-free queue, and profiling rules.
