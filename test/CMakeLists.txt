add_executable(test_eigen3 test_eigen3.cpp)
target_link_libraries(test_eigen3 PRIVATE Eigen3::Eigen)

add_executable(test_systime test_systime.cpp)
target_link_libraries(test_systime PRIVATE nav_core::nav_core)

add_executable(test_syscoord test_syscoord.cpp)
target_link_libraries(test_syscoord PRIVATE nav_core::nav_core)

add_test(NAME Eigen3Smoke COMMAND test_eigen3)
set_tests_properties(Eigen3Smoke PROPERTIES
    PASS_REGULAR_EXPRESSION "eigen3: 41 / 41 passed"
)

add_test(NAME SysTimeSmoke COMMAND test_systime)
set_tests_properties(SysTimeSmoke PROPERTIES
    PASS_REGULAR_EXPRESSION "systime: 10 / 10 passed"
)

add_test(NAME SysCoordSmoke COMMAND test_syscoord)
set_tests_properties(SysCoordSmoke PROPERTIES
    PASS_REGULAR_EXPRESSION "syscoord: 12 / 12 passed"
)
