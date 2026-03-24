cmake_minimum_required(VERSION 3.28)
project(rmcs-navigation)

set(CMAKE_EXPORT_COMPILE_COMMANDS ON)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_STANDARD 23)
set(CMAKE_BUILD_TYPE "Debug")

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
    add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(SOURCE_DIR "${CMAKE_CURRENT_SOURCE_DIR}/../src")
include_directories(${SOURCE_DIR})

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_gtest REQUIRED)
find_package(yaml-cpp REQUIRED)
find_package(rmcs_msgs REQUIRED)

ament_add_gtest(fsm_test fsm.cc)

ament_add_gtest(
    plan_box_test
    plan_box.cc
    ${SOURCE_DIR}/component/decision/plan.cc
)
target_link_libraries(
    plan_box_test
    yaml-cpp::yaml-cpp
)
ament_target_dependencies(
    plan_box_test
    rmcs_msgs
)
