cmake_minimum_required(VERSION 3.16)
project(AccioPath)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_VERBOSE_MAKEFILE ON)
# Add include directory
include_directories(${PROJECT_SOURCE_DIR}/include)
include_directories(/usr/local/include/eigen3)

add_subdirectory(src)

add_executable(AccioPath main.cpp)

# Link your library to the executable
target_link_libraries(AccioPath PRIVATE AccioPath_lib)

# Eigen
find_package(Eigen3 CONFIG REQUIRED)
if (TARGET Eigen3::Eigen)
    target_link_libraries(AccioPath_lib PRIVATE Eigen3::Eigen)
else()
    # Fallback for older Eigen versions or if find_package fails
    include_directories(/usr/local/include/eigen3) # Or the path noted from brew install
endif()
