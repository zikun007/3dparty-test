cmake_minimum_required(VERSION 3.16)
project(3dparty-test LANGUAGES CXX)

# ======================================================================
# 0. Default build type (when user doesn't specify -DCMAKE_BUILD_TYPE=…)
# ======================================================================
if(NOT CMAKE_BUILD_TYPE AND NOT CMAKE_CONFIGURATION_TYPES)
    set(CMAKE_BUILD_TYPE "Release" CACHE STRING "Build type" FORCE)
    message(STATUS "Build type not specified — defaulting to Release")
endif()
set_property(CACHE CMAKE_BUILD_TYPE PROPERTY STRINGS "Debug" "Release" "RelWithDebInfo" "MinSizeRel")

# ======================================================================
# 1. Output directory structure
#    cmake -B build -DCMAKE_BUILD_TYPE=Debug
#    cmake -B build -DCMAKE_BUILD_TYPE=Release
#
#    → build/<build-type>/bin/   executables
#    → build/<build-type>/lib/   static / shared libraries
# ======================================================================
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}/${CMAKE_BUILD_TYPE}/bin")
set(CMAKE_ARCHIVE_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}/${CMAKE_BUILD_TYPE}/lib")
set(CMAKE_LIBRARY_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}/${CMAKE_BUILD_TYPE}/lib")

# ======================================================================
# 2. User-configurable build options
# ======================================================================
option(USE_EIGEN_VECTORIZATION "Enable Eigen SIMD (NEON/SSE). ON for ARMv8-A NEON targets such as Cortex-A55" ON)
option(USE_EIGEN_PARALLEL      "Enable Eigen internal OpenMP parallelization"    OFF)

# ======================================================================
# 3. Compiler settings
# ======================================================================
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

if(CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang")
    add_compile_options(-Wall -Wextra)
endif()

# ======================================================================
# 4. Trimmed third-party libraries
# ======================================================================
set(EIGEN3_ENABLE_VECTORIZATION ${USE_EIGEN_VECTORIZATION} CACHE BOOL
    "Enable Eigen SIMD vectorization" FORCE)
set(EIGEN3_ENABLE_PARALLEL ${USE_EIGEN_PARALLEL} CACHE BOOL
    "Enable Eigen internal parallelization/OpenMP" FORCE)
add_subdirectory(thirdparty/eigen3)

# ======================================================================
# 5. nav_core library
# ======================================================================
add_library(nav_core STATIC
    nav_core/systime.cpp
)
add_library(nav_core::nav_core ALIAS nav_core)

target_include_directories(nav_core PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}>
)
target_link_libraries(nav_core PUBLIC Eigen3::Eigen)

include(CTest)

if(BUILD_TESTING)
    add_subdirectory(test)
endif()
