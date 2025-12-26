cmake_minimum_required(VERSION 3.20)
project(rmcs-navigation)

set(CMAKE_EXPORT_COMPILE_COMMANDS ON)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_STANDARD 23)

if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE "Release" CACHE STRING "Build type" FORCE)
endif()


if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(Eigen3 REQUIRED)

file(GLOB_RECURSE PROJECT_SOURCE
    CONFIGURE_DEPENDS
    ${PROJECT_SOURCE_DIR}/src/*.cpp
    ${PROJECT_SOURCE_DIR}/src/*.cxx
    ${PROJECT_SOURCE_DIR}/src/*.cc
)

add_executable(
    app
    ${PROJECT_SOURCE}
)

include(cmake/version.cmake)
include(cmake/export.cmake)
