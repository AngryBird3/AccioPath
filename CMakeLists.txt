cmake_minimum_required(VERSION 3.16)
project(AccioPath LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_VERBOSE_MAKEFILE ON)

# Eigen (try config mode first)
find_package(Eigen3 CONFIG QUIET)
if(NOT TARGET Eigen3::Eigen)
    message(STATUS "Eigen3::Eigen not found — falling back to include path.")
    include_directories(/usr/local/include/eigen3)
endif()

# Global include directory
include_directories(${PROJECT_SOURCE_DIR}/include)
include_directories(/usr/local/include/eigen3)

include(FetchContent)
FetchContent_Declare(
        googletest
        GIT_REPOSITORY https://github.com/google/googletest.git
        GIT_TAG v1.15.2    # ✅ add explicit stable version
)
# Prevent overriding our compiler flags on old CMake versions
set(gtest_force_shared_crt ON CACHE BOOL "" FORCE)
FetchContent_MakeAvailable(googletest)

add_subdirectory(src)
add_subdirectory(unit_test)

add_executable(AccioPath main.cpp)
target_link_libraries(AccioPath PRIVATE AccioPath_lib)

# Link Eigen if available
if(TARGET Eigen3::Eigen)
    target_link_libraries(AccioPath_lib PRIVATE Eigen3::Eigen)
endif()

# ------------------------------------------------------------------------------
# Enable testing
# ------------------------------------------------------------------------------
enable_testing()
