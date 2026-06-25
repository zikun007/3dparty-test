cmake_minimum_required(VERSION 3.10)
project(3dparty-test LANGUAGES CXX)

# -------------------------------------------------------------------
# Compiler settings (apply to this project only)
# -------------------------------------------------------------------
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

if(CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang")
    add_compile_options(-Wall -Wextra)
endif()

# -------------------------------------------------------------------
# Trimmed Eigen3 — add_subdirectory style
# -------------------------------------------------------------------
add_subdirectory(thirdparty/eigen3)
add_subdirectory(thirdparty/spdlog)

# -------------------------------------------------------------------
# Test executables
# -------------------------------------------------------------------
add_executable(test_eigen3 app/test_eigen3.cpp)
target_link_libraries(test_eigen3 PRIVATE Eigen3::Eigen)

add_executable(test_spdlog app/test_spdlog.cpp)
target_link_libraries(test_spdlog PRIVATE spdlog::spdlog)

# -------------------------------------------------------------------
# CTest
# -------------------------------------------------------------------
enable_testing()
add_test(NAME Eigen3Smoke COMMAND test_eigen3)
add_test(NAME SpdlogSmoke COMMAND test_spdlog)
set_tests_properties(Eigen3Smoke PROPERTIES
    PASS_REGULAR_EXPRESSION "169 / 169 passed"
)
