# src/CMakeLists.txt
file(GLOB SRC_FILES *.cpp)

add_library(AccioPath_lib ${SRC_FILES})

target_include_directories(AccioPath_lib PUBLIC
        ${PROJECT_SOURCE_DIR}/include
)

# Link Eigen if available
if(TARGET Eigen3::Eigen)
    target_link_libraries(AccioPath_lib PRIVATE Eigen3::Eigen)
endif()
