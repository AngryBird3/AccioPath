# Collect all cpp files in src/
file(GLOB SRC_FILES ${CMAKE_CURRENT_SOURCE_DIR}/*.cpp)

# Create a static (or shared) library
add_library(AccioPath_lib ${SRC_FILES})

# Optionally specify include directories for this library
target_include_directories(AccioPath_lib PUBLIC ${PROJECT_SOURCE_DIR}/include)
