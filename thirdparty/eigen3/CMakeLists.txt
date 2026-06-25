# Eigen3 — trimmed Dense-only, header-only library
#
# Usage from parent project:
#   add_subdirectory(thirdparty/eigen3)
#   target_link_libraries(your_target PRIVATE Eigen3::Eigen)

cmake_minimum_required(VERSION 3.10)

# -------------------------------------------------------------------
# Sanity check: ensure we can find the Core header
# -------------------------------------------------------------------
if(NOT EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/Eigen/Core")
    message(FATAL_ERROR
        "Eigen/Core not found under ${CMAKE_CURRENT_SOURCE_DIR}.\n"
        "The trimmed Eigen3 directory appears to be incomplete.")
endif()

# -------------------------------------------------------------------
# Interface library
# -------------------------------------------------------------------
add_library(Eigen3 INTERFACE)
add_library(Eigen3::Eigen ALIAS Eigen3)

target_include_directories(Eigen3 INTERFACE
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}>
    $<INSTALL_INTERFACE:${CMAKE_INSTALL_INCLUDEDIR}/eigen3>
)

# -------------------------------------------------------------------
# Compile-feature hints for consumers.
# Eigen 3.4+ prefers C++14 but degrades gracefully to C++11.
# -------------------------------------------------------------------
target_compile_features(Eigen3 INTERFACE cxx_std_11)

# Suppress pedantic warnings that Eigen's template-heavy headers trigger.
# These apply to all consumers via the interface library, but only when
# the compiler is GNU/Clang.
if(CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang")
    target_compile_options(Eigen3 INTERFACE
        $<$<CXX_COMPILER_ID:GNU>:-Wno-pedantic>
        $<$<CXX_COMPILER_ID:Clang,AppleClang>:-Wno-pedantic>
    )
endif()

# -------------------------------------------------------------------
# Install rules (optional — only used when the parent project runs
# `install()` with this subdirectory).
# -------------------------------------------------------------------
include(GNUInstallDirs)
install(DIRECTORY Eigen
    DESTINATION "${CMAKE_INSTALL_INCLUDEDIR}/eigen3"
    COMPONENT Devel
    FILES_MATCHING PATTERN "*.h" PATTERN "Eigen" EXCLUDE
    PATTERN "src" EXCLUDE
)
install(DIRECTORY Eigen/
    DESTINATION "${CMAKE_INSTALL_INCLUDEDIR}/eigen3/Eigen"
    COMPONENT Devel
    FILES_MATCHING
    PATTERN "Eigen"   # top-level module headers (Core, Dense, etc.)
)
install(DIRECTORY Eigen/src
    DESTINATION "${CMAKE_INSTALL_INCLUDEDIR}/eigen3/Eigen"
    COMPONENT Devel
)
install(TARGETS Eigen3
    EXPORT Eigen3Targets
    COMPONENT Devel
)
install(EXPORT Eigen3Targets
    NAMESPACE Eigen3::
    DESTINATION "${CMAKE_INSTALL_LIBDIR}/cmake/Eigen3"
    COMPONENT Devel
)

message(STATUS "Eigen3 (trimmed Dense-only): ${CMAKE_CURRENT_SOURCE_DIR}")
