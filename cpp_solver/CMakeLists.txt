cmake_minimum_required(VERSION 3.14)
project(yaskawa_cpp_solver LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 23)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

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
)
target_link_libraries(yaskawa_kinematics PUBLIC Eigen3::Eigen Threads::Threads)

add_executable(benchmark_cpp src/benchmark_main.cpp)
target_link_libraries(benchmark_cpp PRIVATE yaskawa_kinematics Eigen3::Eigen Threads::Threads)

add_executable(test_cpp src/test_kinematics.cpp)
target_link_libraries(test_cpp PRIVATE yaskawa_kinematics Eigen3::Eigen Threads::Threads)

# Profile-Specific Compilation Flags
if(CMAKE_BUILD_TYPE STREQUAL "Release")
    target_compile_options(yaskawa_kinematics PRIVATE -O3 -march=native -ffast-math)
    target_compile_options(benchmark_cpp PRIVATE -O3 -march=native -ffast-math)
    target_compile_options(test_cpp PRIVATE -O3 -march=native -ffast-math)
elseif(CMAKE_BUILD_TYPE STREQUAL "Debug")
    target_compile_options(yaskawa_kinematics PRIVATE -O0 -g -fsanitize=address)
    target_compile_options(benchmark_cpp PRIVATE -O0 -g -fsanitize=address)
    target_compile_options(test_cpp PRIVATE -O0 -g -fsanitize=address)
    target_link_libraries(benchmark_cpp PRIVATE -fsanitize=address)
    target_link_libraries(test_cpp PRIVATE -fsanitize=address)
elseif(CMAKE_BUILD_TYPE STREQUAL "Profile")
    target_compile_options(yaskawa_kinematics PRIVATE -O3 -pg -g -march=native)
    target_compile_options(benchmark_cpp PRIVATE -O3 -pg -g -march=native)
    target_compile_options(test_cpp PRIVATE -O3 -pg -g -march=native)
    target_link_libraries(benchmark_cpp PRIVATE -pg)
    target_link_libraries(test_cpp PRIVATE -pg)
elseif(CMAKE_BUILD_TYPE STREQUAL "PGO_Generate")
    target_compile_options(yaskawa_kinematics PRIVATE -O3 -fprofile-generate)
    target_compile_options(benchmark_cpp PRIVATE -O3 -fprofile-generate)
    target_compile_options(test_cpp PRIVATE -O3 -fprofile-generate)
    target_link_libraries(benchmark_cpp PRIVATE -fprofile-generate)
    target_link_libraries(test_cpp PRIVATE -fprofile-generate)
elseif(CMAKE_BUILD_TYPE STREQUAL "PGO_Use")
    target_compile_options(yaskawa_kinematics PRIVATE -O3 -fprofile-use)
    target_compile_options(benchmark_cpp PRIVATE -O3 -fprofile-use)
    target_compile_options(test_cpp PRIVATE -O3 -fprofile-use)
    target_link_libraries(benchmark_cpp PRIVATE -fprofile-use)
    target_link_libraries(test_cpp PRIVATE -fprofile-use)
endif()

message(STATUS "Yaskawa C++23 Real-Time Solver configured successfully!")
