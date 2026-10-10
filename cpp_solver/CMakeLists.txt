cmake_minimum_required(VERSION 3.14)
project(yaskawa_cpp_solver LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 23)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

if(NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE "Release" CACHE STRING "Choose build type: Release Debug Profile PGO_Generate PGO_Use" FORCE)
endif()

message(STATUS "Yaskawa C++23 Build Profile: ${CMAKE_BUILD_TYPE}")

find_package(Eigen3 3.3 REQUIRED NO_MODULE)
find_package(Threads REQUIRED)

include_directories(
    ${CMAKE_CURRENT_SOURCE_DIR}/include
    ${EIGEN3_INCLUDE_DIR}
)

add_library(yaskawa_kinematics STATIC
    src/yaskawa_kinematics.cpp
    src/yaskawa_trajectory.cpp
    src/robot_physical_tree.cpp
)
target_link_libraries(yaskawa_kinematics PUBLIC Eigen3::Eigen Threads::Threads)

# Study layer (docs/STUDY_MODULE_CONTRACT.md): self-describing modules behind a
# JSON-lines protocol. It reuses the kinematics engine, never forks it.
add_library(yaskawa_study STATIC
    src/study/module_registry.cpp
    src/study/spatial_math.cpp
    src/study/dh_kinematics.cpp
    src/study/jacobian_statics.cpp
    src/study/dynamics.cpp
    src/study/trajectory_profiles.cpp
    src/study/control_system.cpp
    src/study/digital_control.cpp
    src/study/dsp_system.cpp
    src/study/sensing_models.cpp
    src/study/vision_camera.cpp
    src/study/learning_models.cpp
    src/study/robot_specification.cpp
    src/robot_physical_tree.cpp
)
target_link_libraries(yaskawa_study PUBLIC yaskawa_kinematics Eigen3::Eigen Threads::Threads)

add_executable(benchmark_cpp src/benchmark_main.cpp)
target_link_libraries(benchmark_cpp PRIVATE yaskawa_kinematics Eigen3::Eigen Threads::Threads)

add_executable(test_cpp src/test_kinematics.cpp)
target_link_libraries(test_cpp PRIVATE yaskawa_kinematics Eigen3::Eigen Threads::Threads)

add_executable(study_api src/study_api_main.cpp)
target_link_libraries(study_api PRIVATE yaskawa_study Eigen3::Eigen Threads::Threads)

add_executable(test_study_cpp src/test_study_modules.cpp)
target_link_libraries(test_study_cpp PRIVATE yaskawa_study Eigen3::Eigen Threads::Threads)

# One test executable per topic area, so a failure names its own subject.
add_executable(test_study_kinematics src/test_study_kinematics.cpp)
target_link_libraries(test_study_kinematics PRIVATE yaskawa_study Eigen3::Eigen Threads::Threads)
add_executable(test_study_dynamics src/test_study_dynamics.cpp)
target_link_libraries(test_study_dynamics PRIVATE yaskawa_study Eigen3::Eigen Threads::Threads)
add_executable(test_study_control src/test_study_control.cpp)
target_link_libraries(test_study_control PRIVATE yaskawa_study Eigen3::Eigen Threads::Threads)
add_executable(test_study_dsp src/test_study_dsp.cpp)
target_link_libraries(test_study_dsp PRIVATE yaskawa_study Eigen3::Eigen Threads::Threads)
add_executable(test_study_intelligence src/test_study_intelligence.cpp)
target_link_libraries(test_study_intelligence PRIVATE yaskawa_study Eigen3::Eigen Threads::Threads)
add_executable(test_study_specification src/test_study_specification.cpp)
target_link_libraries(test_study_specification PRIVATE yaskawa_study Eigen3::Eigen Threads::Threads)

# Profile-Specific Compilation Flags
if(CMAKE_BUILD_TYPE STREQUAL "Release")
    set(RELEASE_OPT_FLAGS
        -O3
        -march=native
        -mtune=native
        -ffast-math
        -funroll-loops
        -fomit-frame-pointer
        -flto=auto
        -fno-signed-zeros
        -fno-trapping-math
        -finline-functions
        -fstrict-aliasing
        -DNDEBUG
    )
    target_compile_options(yaskawa_kinematics PRIVATE ${RELEASE_OPT_FLAGS})
    target_compile_options(benchmark_cpp PRIVATE ${RELEASE_OPT_FLAGS})
    target_compile_options(test_cpp PRIVATE ${RELEASE_OPT_FLAGS})
    target_link_options(benchmark_cpp PRIVATE -flto=auto)
    target_link_options(test_cpp PRIVATE -flto=auto)

    # The study layer keeps the same optimisation level but drops every flag that
    # trades numerical truth for speed: it reports orthonormality drift, gimbal-lock
    # margins, stability margins and filter deviations, and those numbers must mean
    # what they say.
    set(STUDY_OPT_FLAGS
        -O3
        -march=native
        -mtune=native
        -funroll-loops
        -fomit-frame-pointer
        -finline-functions
        -fstrict-aliasing
        -DNDEBUG
    )
    foreach(study_target yaskawa_study study_api test_study_cpp test_study_kinematics
                         test_study_dynamics test_study_control test_study_dsp
                         test_study_intelligence test_study_specification)
        target_compile_options(${study_target} PRIVATE ${STUDY_OPT_FLAGS})
    endforeach()
elseif(CMAKE_BUILD_TYPE STREQUAL "Debug")
    target_compile_options(yaskawa_kinematics PRIVATE -O0 -g -fsanitize=address)
    target_compile_options(benchmark_cpp PRIVATE -O0 -g -fsanitize=address)
    target_compile_options(test_cpp PRIVATE -O0 -g -fsanitize=address)
    target_link_libraries(benchmark_cpp PRIVATE -fsanitize=address)
    target_link_libraries(test_cpp PRIVATE -fsanitize=address)
    target_compile_options(yaskawa_study PRIVATE -O0 -g -fsanitize=address)
    target_compile_options(study_api PRIVATE -O0 -g -fsanitize=address)
    target_compile_options(test_study_cpp PRIVATE -O0 -g -fsanitize=address)
    target_compile_options(test_study_kinematics PRIVATE -O0 -g -fsanitize=address)
    target_compile_options(test_study_dynamics PRIVATE -O0 -g -fsanitize=address)
    target_compile_options(test_study_control PRIVATE -O0 -g -fsanitize=address)
    target_compile_options(test_study_dsp PRIVATE -O0 -g -fsanitize=address)
    target_compile_options(test_study_intelligence PRIVATE -O0 -g -fsanitize=address)
    target_compile_options(test_study_specification PRIVATE -O0 -g -fsanitize=address)
    target_link_libraries(test_study_kinematics PRIVATE -fsanitize=address)
    target_link_libraries(test_study_dynamics PRIVATE -fsanitize=address)
    target_link_libraries(test_study_control PRIVATE -fsanitize=address)
    target_link_libraries(test_study_dsp PRIVATE -fsanitize=address)
    target_link_libraries(test_study_intelligence PRIVATE -fsanitize=address)
    target_link_libraries(test_study_specification PRIVATE -fsanitize=address)
    target_link_libraries(study_api PRIVATE -fsanitize=address)
    target_link_libraries(test_study_cpp PRIVATE -fsanitize=address)
elseif(CMAKE_BUILD_TYPE STREQUAL "Profile")
    target_compile_options(yaskawa_kinematics PRIVATE -O3 -pg -g)
    target_compile_options(benchmark_cpp PRIVATE -O3 -pg -g)
    target_compile_options(test_cpp PRIVATE -O3 -pg -g)
    target_link_libraries(benchmark_cpp PRIVATE -pg)
    target_link_libraries(test_cpp PRIVATE -pg)
    target_compile_options(yaskawa_study PRIVATE -O3 -pg -g -march=native)
    target_compile_options(study_api PRIVATE -O3 -pg -g -march=native)
    target_compile_options(test_study_cpp PRIVATE -O3 -pg -g -march=native)
    target_compile_options(test_study_kinematics PRIVATE -O3 -pg -g -march=native)
    target_compile_options(test_study_dynamics PRIVATE -O3 -pg -g -march=native)
    target_compile_options(test_study_control PRIVATE -O3 -pg -g -march=native)
    target_compile_options(test_study_dsp PRIVATE -O3 -pg -g -march=native)
    target_compile_options(test_study_intelligence PRIVATE -O3 -pg -g -march=native)
    target_compile_options(test_study_specification PRIVATE -O3 -pg -g -march=native)
    target_link_libraries(test_study_kinematics PRIVATE -pg)
    target_link_libraries(test_study_dynamics PRIVATE -pg)
    target_link_libraries(test_study_control PRIVATE -pg)
    target_link_libraries(test_study_dsp PRIVATE -pg)
    target_link_libraries(test_study_intelligence PRIVATE -pg)
    target_link_libraries(test_study_specification PRIVATE -pg)
    target_link_libraries(study_api PRIVATE -pg)
    target_link_libraries(test_study_cpp PRIVATE -pg)
elseif(CMAKE_BUILD_TYPE STREQUAL "PGO_Generate")
    target_compile_options(yaskawa_kinematics PRIVATE -O3 -fprofile-generate)
    target_compile_options(benchmark_cpp PRIVATE -O3 -fprofile-generate)
    target_compile_options(test_cpp PRIVATE -O3 -fprofile-generate)
    target_link_libraries(benchmark_cpp PRIVATE -fprofile-generate)
    target_link_libraries(test_cpp PRIVATE -fprofile-generate)
    target_compile_options(yaskawa_study PRIVATE -O3 -fprofile-generate)
    target_compile_options(study_api PRIVATE -O3 -fprofile-generate)
    target_compile_options(test_study_cpp PRIVATE -O3 -fprofile-generate)
    target_link_libraries(study_api PRIVATE -fprofile-generate)
    target_link_libraries(test_study_cpp PRIVATE -fprofile-generate)
elseif(CMAKE_BUILD_TYPE STREQUAL "PGO_Use")
    target_compile_options(yaskawa_kinematics PRIVATE -O3 -fprofile-use)
    target_compile_options(benchmark_cpp PRIVATE -O3 -fprofile-use)
    target_compile_options(test_cpp PRIVATE -O3 -fprofile-use)
    target_link_libraries(benchmark_cpp PRIVATE -fprofile-use)
    target_link_libraries(test_cpp PRIVATE -fprofile-use)
    target_compile_options(yaskawa_study PRIVATE -O3 -fprofile-use)
    target_compile_options(study_api PRIVATE -O3 -fprofile-use)
    target_compile_options(test_study_cpp PRIVATE -O3 -fprofile-use)
    target_link_libraries(study_api PRIVATE -fprofile-use)
    target_link_libraries(test_study_cpp PRIVATE -fprofile-use)
endif()

message(STATUS "Yaskawa C++23 Real-Time Solver configured successfully!")
