# collect test sources
file(GLOB TEST_SOURCES *.cpp)

add_executable(runUnitTests ${TEST_SOURCES})

target_link_libraries(runUnitTests PRIVATE
        GTest::gtest_main
        AccioPath_lib
)

target_include_directories(runUnitTests PRIVATE
        ${PROJECT_SOURCE_DIR}/include
)

include(GoogleTest)
gtest_discover_tests(runUnitTests
        WORKING_DIRECTORY ${PROJECT_DIR}
        DISCOVERY_TIMEOUT 30
)

