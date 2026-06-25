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
#    Turn OFF for embedded / constrained toolchains.
# ======================================================================
option(USE_CCACHE              "Enable ccache for faster recompilation"           OFF)
option(USE_UNITY_BUILD         "Merge translation units (fewer compiler invocations)" OFF)
option(USE_EIGEN_VECTORIZATION "Enable Eigen SIMD (NEON/SSE). OFF if compiler ICE" OFF)
option(BUILD_SPDLOG            "Build spdlog test + library (needs pthreads)"     OFF)
option(SPDLOG_COMPILED_LIB     "Build spdlog as compiled lib (needs pthreads)"    OFF)

# ======================================================================
# 3. Compilation speed — ccache
# ======================================================================
if(USE_CCACHE)
    find_program(CCACHE_FOUND ccache)
    if(CCACHE_FOUND)
        set(CMAKE_CXX_COMPILER_LAUNCHER "${CCACHE_FOUND}")
        message(STATUS "ccache: found and enabled (${CCACHE_FOUND})")
    else()
        message(STATUS "ccache: not found — install 'ccache' for faster recompilation")
    endif()
endif()

# ======================================================================
# 4. Compilation speed — Unity builds
# ======================================================================
if(USE_UNITY_BUILD)
    set(CMAKE_UNITY_BUILD ON)
    set(CMAKE_UNITY_BUILD_BATCH_SIZE 8)
    message(STATUS "Unity build: ON (batch size 8)")
endif()

# ======================================================================
# 5. Compiler settings
# ======================================================================
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

if(CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang")
    add_compile_options(-Wall -Wextra)
endif()

# Debug: -Og (optimise for debugging) compiles faster than default -O0
set(CMAKE_CXX_FLAGS_DEBUG "-Og -g" CACHE STRING
    "Flags used by the CXX compiler during Debug builds" FORCE)

# Release: full optimisation
set(CMAKE_CXX_FLAGS_RELEASE "-O3 -DNDEBUG" CACHE STRING
    "Flags used by the CXX compiler during Release builds" FORCE)

# ======================================================================
# 6. Trimmed third-party libraries
# ======================================================================
add_subdirectory(thirdparty/eigen3)

if(NOT USE_EIGEN_VECTORIZATION)
    target_compile_definitions(Eigen3 INTERFACE EIGEN_DONT_VECTORIZE)
    message(STATUS "Eigen: SIMD vectorization DISABLED (USE_EIGEN_VECTORIZATION=OFF)")
endif()

if(BUILD_SPDLOG)
    add_subdirectory(thirdparty/spdlog)
endif()

# ======================================================================
# 7. Test executables
# ======================================================================
# Eigen test — always built (header-only, no OS dependencies)
add_executable(test_eigen3 app/test_eigen3.cpp)
target_link_libraries(test_eigen3 PRIVATE Eigen3::Eigen)

# spdlog test — only when spdlog is enabled
if(BUILD_SPDLOG)
    add_executable(test_spdlog app/test_spdlog.cpp)
    target_link_libraries(test_spdlog PRIVATE spdlog::spdlog)
endif()

# ======================================================================
# 8. CTest
# ======================================================================
enable_testing()
add_test(NAME Eigen3Smoke COMMAND test_eigen3)
set_tests_properties(Eigen3Smoke PROPERTIES
    PASS_REGULAR_EXPRESSION "169 / 169 passed"
)

if(BUILD_SPDLOG)
    add_test(NAME SpdlogSmoke COMMAND test_spdlog)
endif()
