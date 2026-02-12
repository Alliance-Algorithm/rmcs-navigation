cmake_minimum_required(VERSION 4.0.2)

set(CMAKE_EXPORT_COMPILE_COMMANDS ON)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_STANDARD 23)

cmake_policy(SET CMP0167 OLD)
cmake_policy(SET CMP0144 NEW)

project(rmcs-navigation)

if(NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE "Release" CACHE STRING "Build type" FORCE)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
    add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(Eigen3 REQUIRED)
find_package(PCL REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)

file(GLOB_RECURSE PROJECT_SOURCE
    CONFIGURE_DEPENDS
    ${PROJECT_SOURCE_DIR}/src/*.cpp
    ${PROJECT_SOURCE_DIR}/src/*.cxx
    ${PROJECT_SOURCE_DIR}/src/*.cc
)
file(GLOB_RECURSE PROJECT_MODULE
    CONFIGURE_DEPENDS
    ${PROJECT_SOURCE_DIR}/src/*.cppm
    ${PROJECT_SOURCE_DIR}/src/*.cxxm
    ${PROJECT_SOURCE_DIR}/src/*.ccm
)
include_directories(
    src
    ${sensor_msgs_INCLUDE_DIRS}
)

add_executable(
    app
    ${PROJECT_SOURCE}
)
target_sources(
    app
    PUBLIC FILE_SET CXX_MODULES FILES
        ${PROJECT_MODULE}
)
target_link_libraries(
    app
    rclcpp::rclcpp
    ${sensor_msgs_LIBRARIES}
)


include(cmake/version.cmake)
include(cmake/export.cmake)
