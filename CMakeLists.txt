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
# 2. Compilation speed — ccache
#    Install: sudo apt install ccache
# ======================================================================
find_program(CCACHE_FOUND ccache)
if(CCACHE_FOUND)
    set(CMAKE_CXX_COMPILER_LAUNCHER "${CCACHE_FOUND}")
    message(STATUS "ccache: found and enabled (${CCACHE_FOUND})")
else()
    message(STATUS "ccache: not found — install 'ccache' for faster recompilation")
endif()

# ======================================================================
# 3. Compilation speed — Unity builds (merge TUs, reduce header re-parsing)
#    CMake ≥ 3.16 required
# ======================================================================
set(CMAKE_UNITY_BUILD ON)
set(CMAKE_UNITY_BUILD_BATCH_SIZE 8)

# ======================================================================
# 4. Compilation speed — spdlog compiled library
#    Avoids re-instantiating spdlog templates in every translation unit.
#    SET BEFORE add_subdirectory(thirdparty/spdlog) !
# ======================================================================
set(SPDLOG_COMPILED_LIB ON CACHE BOOL "Build spdlog as compiled library (faster builds)")

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
set(CMAKE_CXX_FLAGS_DEBUG "-Og -g" CACHE STRING "Flags used by the CXX compiler during Debug builds" FORCE)

# Release: full optimisation
set(CMAKE_CXX_FLAGS_RELEASE "-O3 -DNDEBUG" CACHE STRING "Flags used by the CXX compiler during Release builds" FORCE)

# ======================================================================
# 6. Trimmed third-party libraries
# ======================================================================
add_subdirectory(thirdparty/eigen3)
add_subdirectory(thirdparty/spdlog)

# ======================================================================
# 7. Test executables
# ======================================================================
add_executable(test_eigen3 app/test_eigen3.cpp)
target_link_libraries(test_eigen3 PRIVATE Eigen3::Eigen)

add_executable(test_spdlog app/test_spdlog.cpp)
target_link_libraries(test_spdlog PRIVATE spdlog::spdlog)

# ======================================================================
# 8. CTest
# ======================================================================
enable_testing()
add_test(NAME Eigen3Smoke COMMAND test_eigen3)
add_test(NAME SpdlogSmoke COMMAND test_spdlog)
set_tests_properties(Eigen3Smoke PROPERTIES
    PASS_REGULAR_EXPRESSION "169 / 169 passed"
)
