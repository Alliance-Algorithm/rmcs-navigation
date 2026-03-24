cmake_minimum_required(VERSION 3.30)

set(CMAKE_EXPORT_COMPILE_COMMANDS ON)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_STANDARD 23)

set(CMAKE_POLICY_DEFAULT_CMP0167 OLD)
set(CMAKE_POLICY_DEFAULT_CMP0144 NEW)

project(rmcs-navigation)

if(NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE "Release" CACHE STRING "Build type" FORCE)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
    add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(Eigen3 REQUIRED)
find_package(yaml-cpp REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(nav2_msgs REQUIRED)
find_package(PCL REQUIRED)

find_package(geometry_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(rmcs_msgs REQUIRED)
find_package(ament_index_cpp REQUIRED)
find_package(tf2_ros REQUIRED)

find_package(rmcs_executor REQUIRED)

# RMCS Component
add_library(${PROJECT_NAME} SHARED)
target_sources(
    ${PROJECT_NAME}
    PRIVATE
        src/component/component.cc
        src/component/decision/plan.cc
)
target_include_directories(
    ${PROJECT_NAME}
    PRIVATE
        ${PROJECT_SOURCE_DIR}/src/
        ${rmcs_executor_INCLUDE_DIRS}
        ${geometry_msgs_INCLUDE_DIRS}
        ${rmcs_msgs_INCLUDE_DIRS}
        ${std_srvs_INCLUDE_DIRS}
        ${ament_index_cpp_INCLUDE_DIRS}
        ${tf2_ros_INCLUDE_DIRS}
        ${rclcpp_action_INCLUDE_DIRS}
        ${nav2_msgs_INCLUDE_DIRS}
)
target_link_libraries(
    ${PROJECT_NAME}
    rclcpp::rclcpp
    ament_index_cpp::ament_index_cpp
    yaml-cpp::yaml-cpp
    Eigen3::Eigen
    ${rmcs_executor_LIBRARIES}
    ${geometry_msgs_LIBRARIES}
    ${std_srvs_LIBRARIES}
    ${rmcs_msgs_LIBRARIES}
    ${tf2_ros_LIBRARIES}
    ${rclcpp_action_LIBRARIES}
    ${nav2_msgs_LIBRARIES}
)
find_package(pluginlib REQUIRED)
pluginlib_export_plugin_description_file(
    rmcs_executor plugins.xml
)

include(cmake/version.cmake)
include(cmake/export.cmake)
