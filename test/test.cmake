find_program(LUA_EXECUTABLE NAMES lua lua5.4 REQUIRED)

# Resolve project root regardless of where this file is included from
get_filename_component(RMCS_NAVIGATION_ROOT "${CMAKE_CURRENT_LIST_DIR}/.." ABSOLUTE)

set(RMCS_NAVIGATION_LUA_TESTS
    action
    clock
    fsm
    map
    order
    scheduler
    toggle_record
    record
)

foreach(test_name IN LISTS RMCS_NAVIGATION_LUA_TESTS)
    add_test(
        NAME rmcs-navigation.lua.${test_name}
        COMMAND ${LUA_EXECUTABLE} ${RMCS_NAVIGATION_ROOT}/test/lua/${test_name}.lua
        WORKING_DIRECTORY ${RMCS_NAVIGATION_ROOT}
    )
endforeach()

find_package(Threads REQUIRED)
add_executable(record_session_test
    ${RMCS_NAVIGATION_ROOT}/test/cxx/record_session.cc
    ${RMCS_NAVIGATION_ROOT}/src/cxx/record_session.cc
)
target_compile_features(record_session_test PRIVATE cxx_std_20)
target_include_directories(record_session_test PRIVATE ${RMCS_NAVIGATION_ROOT}/src)
target_link_libraries(record_session_test PRIVATE Threads::Threads)
add_test(NAME rmcs-navigation.record_session COMMAND record_session_test)
set_tests_properties(rmcs-navigation.record_session PROPERTIES TIMEOUT 25)

find_package(Python3 COMPONENTS Interpreter REQUIRED)
add_test(NAME rmcs-navigation.record_control
    COMMAND ${Python3_EXECUTABLE} ${RMCS_NAVIGATION_ROOT}/test/scripts/record_control.py
)
set_tests_properties(rmcs-navigation.record_control PROPERTIES TIMEOUT 25 SKIP_RETURN_CODE 77)

add_test(NAME rmcs-navigation.record_shutdown
    COMMAND ${Python3_EXECUTABLE} ${RMCS_NAVIGATION_ROOT}/test/scripts/record_session.py
)
set_tests_properties(rmcs-navigation.record_shutdown PROPERTIES TIMEOUT 40)

# scanning 依赖 ROS/sol2，仅在主包 BUILD_TESTING 路径编译；
# CI 的 `cmake -S test`（project: rmcs-navigation-test）不进入此分支。
if(NOT CMAKE_PROJECT_NAME STREQUAL "rmcs-navigation-test")
    add_executable(scanning ${RMCS_NAVIGATION_ROOT}/test/cxx/scanning.cc)
    target_compile_features(scanning PRIVATE cxx_std_23)
    target_include_directories(
        scanning
        SYSTEM PRIVATE
            ${LUA_INCLUDE_DIR}
            ${visualization_msgs_INCLUDE_DIRS}
    )
    target_link_libraries(
        scanning
        PRIVATE
            ${LUA_LIBRARIES}
            sol2::sol2
            rclcpp::rclcpp
            ${visualization_msgs_LIBRARIES}
    )
endif()
