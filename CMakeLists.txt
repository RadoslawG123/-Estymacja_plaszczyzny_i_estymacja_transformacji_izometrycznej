cmake_minimum_required(VERSION 3.16)
project(GeometryEstimation VERSION 1.0 LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

if(NOT CMAKE_BUILD_TYPE AND NOT CMAKE_CONFIGURATION_TYPES)
    set(CMAKE_BUILD_TYPE Release CACHE STRING "Build type" FORCE)
endif()

# Eigen: use the system installation if there is one, otherwise download it.
find_package(Eigen3 3.3 QUIET NO_MODULE)
if(NOT TARGET Eigen3::Eigen)
    message(STATUS "Eigen3 not found, downloading 3.4.0")
    include(FetchContent)
    FetchContent_Declare(eigen
        URL https://gitlab.com/libeigen/eigen/-/archive/3.4.0/eigen-3.4.0.tar.gz
        URL_HASH SHA256=8586084f71f9bde545ee7fa6d00288b264a2b7ac3607b974e54d13e7162c1c72
        DOWNLOAD_EXTRACT_TIMESTAMP TRUE
        # header-only, we don't want Eigen's own CMake project (tests, docs...)
        SOURCE_SUBDIR do_not_build
    )
    FetchContent_MakeAvailable(eigen)
    add_library(Eigen3::Eigen INTERFACE IMPORTED)
    target_include_directories(Eigen3::Eigen INTERFACE ${eigen_SOURCE_DIR})
endif()

add_executable(estimator
    src/main.cpp
)
target_include_directories(estimator PRIVATE include)
target_link_libraries(estimator PRIVATE Eigen3::Eigen)

if(MSVC)
    target_compile_options(estimator PRIVATE /W4 /utf-8)
else()
    target_compile_options(estimator PRIVATE -Wall -Wextra -Wpedantic)
endif()
