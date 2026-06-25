# spdlog — trimmed embedded version (header-only by default)
#
# Usage from parent project:
#   add_subdirectory(thirdparty/spdlog)
#   target_link_libraries(your_target PRIVATE spdlog::spdlog)
#
# To build as compiled library (faster compile, smaller binary):
#   set(SPDLOG_COMPILED_LIB ON)
#   add_subdirectory(thirdparty/spdlog)

cmake_minimum_required(VERSION 3.10)

# Guard against multiple inclusion
if(TARGET spdlog::spdlog)
    return()
endif()

# -------------------------------------------------------------------
# Sanity check
# -------------------------------------------------------------------
if(NOT EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/include/spdlog/spdlog.h")
    message(FATAL_ERROR
        "spdlog headers not found at ${CMAKE_CURRENT_SOURCE_DIR}/include\n"
        "The trimmed spdlog directory appears to be incomplete.")
endif()

# -------------------------------------------------------------------
# Build mode: header-only (default) or compiled library
# -------------------------------------------------------------------
option(SPDLOG_COMPILED_LIB "Build spdlog as a compiled library" OFF)

# -------------------------------------------------------------------
# Compile-time feature options (all default OFF for max compatibility)
# -------------------------------------------------------------------
option(SPDLOG_NO_EXCEPTIONS   "Compile without exception support" OFF)
option(SPDLOG_NO_THREAD_ID    "Skip thread ID queries (saves syscall per log)" OFF)
option(SPDLOG_NO_ATOMIC_LEVELS "Use non-atomic log levels (no concurrent level changes)" OFF)
option(SPDLOG_NO_TLS          "Disable thread-local storage" OFF)
option(SPDLOG_CLOCK_COARSE    "Use CLOCK_REALTIME_COARSE on Linux (faster, coarser)" OFF)
option(SPDLOG_PREVENT_CHILD_FD "Prevent child FD inheritance" OFF)
option(SPDLOG_NO_SOURCE_LOC   "Omit __FILE__/__LINE__ from log calls" OFF)
option(SPDLOG_DISABLE_DEFAULT_LOGGER "Skip creating default logger at init" OFF)

# -------------------------------------------------------------------
# Interface library (always available for header-only mode)
# -------------------------------------------------------------------
add_library(spdlog_headers INTERFACE)
add_library(spdlog::spdlog_headers ALIAS spdlog_headers)

target_include_directories(spdlog_headers INTERFACE
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:${CMAKE_INSTALL_INCLUDEDIR}>
)

target_compile_features(spdlog_headers INTERFACE cxx_std_11)

# -------------------------------------------------------------------
# Apply feature options as compile definitions on the interface
# -------------------------------------------------------------------
function(spdlog_add_opt opt def)
    if(${opt})
        target_compile_definitions(spdlog_headers INTERFACE ${def})
    endif()
endfunction()

spdlog_add_opt(SPDLOG_NO_EXCEPTIONS    SPDLOG_NO_EXCEPTIONS)
spdlog_add_opt(SPDLOG_NO_THREAD_ID     SPDLOG_NO_THREAD_ID)
spdlog_add_opt(SPDLOG_NO_ATOMIC_LEVELS SPDLOG_NO_ATOMIC_LEVELS)
spdlog_add_opt(SPDLOG_NO_TLS           SPDLOG_NO_TLS)
spdlog_add_opt(SPDLOG_CLOCK_COARSE     SPDLOG_CLOCK_COARSE)
spdlog_add_opt(SPDLOG_PREVENT_CHILD_FD SPDLOG_PREVENT_CHILD_FD)
spdlog_add_opt(SPDLOG_NO_SOURCE_LOC    SPDLOG_NO_SOURCE_LOC)
spdlog_add_opt(SPDLOG_DISABLE_DEFAULT_LOGGER SPDLOG_DISABLE_DEFAULT_LOGGER)

# -------------------------------------------------------------------
# Compiled library mode (optional, reduces per-TU recompilation)
# -------------------------------------------------------------------
if(SPDLOG_COMPILED_LIB)
    set(_mode "compiled")
    add_library(spdlog STATIC
        src/spdlog.cpp
        src/stdout_sinks.cpp
        src/file_sinks.cpp
    )
    add_library(spdlog::spdlog ALIAS spdlog)

    target_include_directories(spdlog
        PUBLIC  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
        PRIVATE $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/src>
    )
    target_compile_definitions(spdlog PRIVATE SPDLOG_COMPILED_LIB)
    target_link_libraries(spdlog PUBLIC spdlog_headers)

    # Threads needed for mutex and periodic flush
    find_package(Threads REQUIRED)
    target_link_libraries(spdlog PUBLIC Threads::Threads)

    # Install rules for compiled lib
    install(TARGETS spdlog EXPORT spdlogTargets
        ARCHIVE DESTINATION "${CMAKE_INSTALL_LIBDIR}"
        COMPONENT Devel
    )
else()
    set(_mode "header-only")
    # Header-only mode: create an ALIAS for convenience
    add_library(spdlog INTERFACE)
    add_library(spdlog::spdlog ALIAS spdlog)
    target_link_libraries(spdlog INTERFACE spdlog_headers)

    # spdlog needs Threads for mutex even in header-only mode
    find_package(Threads REQUIRED)
    target_link_libraries(spdlog INTERFACE Threads::Threads)
endif()

# -------------------------------------------------------------------
# Install rules
# -------------------------------------------------------------------
install(DIRECTORY include/
    DESTINATION "${CMAKE_INSTALL_INCLUDEDIR}"
    COMPONENT Devel
)
install(TARGETS spdlog_headers EXPORT spdlogTargets COMPONENT Devel)
install(EXPORT spdlogTargets
    NAMESPACE spdlog::
    DESTINATION "${CMAKE_INSTALL_LIBDIR}/cmake/spdlog"
    COMPONENT Devel
)

message(STATUS "spdlog (trimmed embedded): ${CMAKE_CURRENT_SOURCE_DIR}  [${_mode}]")
