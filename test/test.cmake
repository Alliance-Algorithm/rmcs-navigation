find_program(LUA_EXECUTABLE NAMES lua lua5.4 REQUIRED)

# Resolve project root regardless of where this file is included from
get_filename_component(RMCS_NAVIGATION_ROOT "${CMAKE_CURRENT_LIST_DIR}/.." ABSOLUTE)

set(RMCS_NAVIGATION_LUA_TESTS
    clock
    fsm
    scheduler
)

foreach(test_name IN LISTS RMCS_NAVIGATION_LUA_TESTS)
    add_test(
        NAME rmcs-navigation.lua.${test_name}
        COMMAND ${LUA_EXECUTABLE} ${RMCS_NAVIGATION_ROOT}/test/lua/${test_name}.lua
        WORKING_DIRECTORY ${RMCS_NAVIGATION_ROOT}
    )
endforeach()
