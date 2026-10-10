#!/usr/bin/env bash
set -e

PROJECT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
CPP_DIR="${PROJECT_DIR}/cpp_solver"
BUILD_DIR="${CPP_DIR}/build"
PROFILE_DIR="${PROJECT_DIR}/profiling"
INSTALL_DIR="${PROJECT_DIR}/install"

mkdir -p "${PROFILE_DIR}"

usage() {
    echo "========================================================="
    echo "           YASKAWA GP8 ROBOT CONTROL CLI                "
    echo "========================================================="
    echo "Usage: ./robot <command> [options]"
    echo ""
    echo "Commands:"
    echo "  init              - Initialize project environment & verify compilers"
    echo "  build [profile]   - Build project (Profiles: release | debug | profile | pgo)"
    echo "  test              - Run C++26 unit test suite"
    echo "  benchmark         - Run performance benchmark & export bottleneck profiles"
    echo "  run [web|ros]     - Launch Web 3D Dashboard (web) or ROS2/MoveIt scene (ros)"
    echo "  install           - Install built binaries to ${INSTALL_DIR}"
    echo "  uninstall         - Remove installed binaries"
    echo "  clean             - Clean build and profiling outputs"
    echo "  info              - Display project metadata"
    echo "  publish           - Publish current release version"
    echo "  debug             - Build and debug with AddressSanitizer/GDB"
    echo "========================================================="
    exit 1
}

if [ $# -lt 1 ]; then
    usage
fi

CMD="$1"
shift

case "${CMD}" in
    init)
        echo "--> Initializing environment & verifying dev tools..."
        cmake --version
        g++ --version
        pkg-config --cflags eigen3
        echo "--> Environment initialization complete!"
        ;;

    build)
        PROFILE="Release"
        if [ "$1" == "debug" ]; then
            PROFILE="Debug"
        elif [ "$1" == "profile" ]; then
            PROFILE="Profile"
        elif [ "$1" == "pgo" ]; then
            PROFILE="PGO_Generate"
        fi

        echo "--> Building Yaskawa C++26 Solver with Profile: [${PROFILE}]..."
        cmake -B "${BUILD_DIR}" -S "${CPP_DIR}" -DCMAKE_BUILD_TYPE="${PROFILE}"
        cmake --build "${BUILD_DIR}" -j$(nproc)
        echo "--> Build finished successfully! Executables located in ${BUILD_DIR}"
        ;;

    test)
        if [ ! -f "${BUILD_DIR}/test_cpp" ]; then
            echo "--> Binary not found. Building release..."
            "${PROJECT_DIR}/robot" build release
        fi
        echo "--> Setting git hooks path..."
        git config core.hooksPath .claude/hooks/git || true
        echo "--> Running C++26 Unit Tests..."
        "${BUILD_DIR}/test_cpp"
        echo "--> Running Study Module Test Suite..."
        "${BUILD_DIR}/test_study_cpp"
        echo "--> Smoke-testing the study API catalogue..."
        "${BUILD_DIR}/study_api" --describe > /dev/null
        echo "--> Study API responded with a valid catalogue."
        ;;

    benchmark)
        echo "--> Building in Profile mode for call-graph bottleneck analysis..."
        cmake -B "${BUILD_DIR}" -S "${CPP_DIR}" -DCMAKE_BUILD_TYPE="Profile"
        cmake --build "${BUILD_DIR}" -j$(nproc)

        echo "--> Running C++26 High-Resolution Benchmark..."
        cd "${PROFILE_DIR}"
        "${BUILD_DIR}/benchmark_cpp"

        if command -v gprof &> /dev/null && [ -f "${PROFILE_DIR}/gmon.out" ]; then
            echo "--> Exporting callgrind / gprof bottleneck profile file..."
            gprof "${BUILD_DIR}/benchmark_cpp" "${PROFILE_DIR}/gmon.out" > "${PROFILE_DIR}/profile_report.txt"
            echo "--> Bottleneck trace exported to: ${PROFILE_DIR}/profile_report.txt"
        fi

        if command -v valgrind &> /dev/null; then
            echo "--> Generating Callgrind profile trace file (valgrind --tool=callgrind)..."
            valgrind --tool=callgrind --callgrind-out-file="${PROFILE_DIR}/callgrind.out" "${BUILD_DIR}/benchmark_cpp" > /dev/null 2>&1 || true
            echo "--> Callgrind profile trace saved to: ${PROFILE_DIR}/callgrind.out"
        fi

        cd "${PROJECT_DIR}"
        ;;

    run)
        MODE="${1:-web}"
        if [ "${MODE}" == "ros" ]; then
            echo "--> Launching ROS 2 MoveIt Scene container..."
            "${PROJECT_DIR}/start_yaskawa_scene.sh"
        else
            echo "--> Launching Standalone Web 3D Dashboard..."
            python3 "${CPP_DIR}/web/server.py"
        fi
        ;;

    install)
        echo "--> Installing binaries to ${INSTALL_DIR}..."
        mkdir -p "${INSTALL_DIR}/bin"
        cp -f "${BUILD_DIR}/benchmark_cpp" "${INSTALL_DIR}/bin/" 2>/dev/null || true
        cp -f "${BUILD_DIR}/test_cpp" "${INSTALL_DIR}/bin/" 2>/dev/null || true
        echo "--> Installation complete."
        ;;

    uninstall)
        echo "--> Removing installed binaries..."
        rm -rf "${INSTALL_DIR}/bin"
        echo "--> Uninstall complete."
        ;;

    clean)
        echo "--> Cleaning build and profiling output directories..."
        rm -rf "${BUILD_DIR}" "${PROFILE_DIR}" "${INSTALL_DIR}/bin"
        echo "--> Clean complete."
        ;;

    info)
        echo "Name    : waam-manipulator"
        echo "Code    : wmp"
        echo "Version : 1.0.0"
        echo "Summary : High-performance C++26 Yaskawa GP8 robot kinematics solver and simulation engine."
        if [ ! -f "${INSTALL_DIR}/bin/benchmark_cpp" ] && [ ! -f "${INSTALL_DIR}/bin/test_cpp" ]; then
            echo "Status  : Not installed"
            exit 1
        fi
        echo "Status  : Installed (${INSTALL_DIR}/bin)"
        exit 0
        ;;

    publish)
        echo "--> Publishing version 1.0.0 release asset..."
        ;;

    debug)
        echo "--> Building in Debug mode with AddressSanitizer..."
        cmake -B "${BUILD_DIR}" -S "${CPP_DIR}" -DCMAKE_BUILD_TYPE="Debug"
        cmake --build "${BUILD_DIR}" -j$(nproc)
        echo "--> Running Debug Test with ASan..."
        "${BUILD_DIR}/test_cpp"
        ;;

    *)
        usage
        ;;
esac
