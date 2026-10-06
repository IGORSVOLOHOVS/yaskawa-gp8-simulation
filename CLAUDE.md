# Yaskawa GP8 Kinematics & Simulation Engine (`waam-manipulator`) Guidelines

This repository complies with revision **4.0.0** of `BEST_REQUIREMENTS-v4.0.0.md` (§7).

---

## 1. CLI Commands (`./robot`)

The `./robot` bash script is the primary execution interface for all lifecycle tasks:

- `./robot init` : Verify environment and required C++ build toolchain (`g++`, `cmake`, `eigen3`).
- `./robot build [release|debug|profile|pgo]` : Build C++26 solver with the selected profile.
- `./robot test` : Set git hooks path and run the unit test suite.
- `./robot benchmark` : Execute performance benchmarks and export callgrind / gprof bottleneck traces to `profiling/`.
- `./robot run [web|ros]` : Launch interactive 3D WebGL Dashboard (`web`) or ROS 2 MoveIt scene (`ros`).
- `./robot install` : Install compiled executables to `install/bin/`.
- `./robot uninstall` : Remove installed binaries from `install/bin/`.
- `./robot clean` : Clean build, profiling, and install artifacts.
- `./robot info` : Output project metadata and installation status.
- `./robot publish` : Prepare release assets for version publication.

---

## 2. Branch & Git Workflow

- **Long-lived Branches**:
  - `release`: Production release branch. Accepts merges from `test` only.
  - `test`: Integration testing branch.
- **Feature Branch Naming**:
  - Format: `wmp-<issue>/<type>/<slug>` (e.g., `wmp-7/feat/kinematics-solver`).
  - Allowed types: `feat`, `fix`, `docs`, `refactor`, `test`, `build`, `ci`, `chore`, `hotfix`, `experiment`.
- **Commit Format**:
  - Conventional Commits: `<type>(<scope>): <description>` (e.g., `feat(core): implement LM IK solver`).
  - Always sign off commits: `git commit -s`.
  - All commit messages, documentation, and issues must be in **English**.

---

## 3. Secret Keys & Security (R-02)

- Local API keys are specified in `.claude/.env` (`GITHUB_TOKEN=...`).
- `.claude/.env` is ignored by git (`.gitignore`) and MUST NOT be pushed to remote repositories.
- Use `.claude/.env.example` as a template for setting up local credentials.

---

## 4. Roles & Review Policy

- **Author / Developer**: Robot Developer (AI Assistant / Contributor).
- **Reviewer**: `igorsvolohovs` (Repository Owner / Reviewer).
- Merges into `release` require approval from a name in `$Reviewers`.
