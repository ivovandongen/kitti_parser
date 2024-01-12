include(${PROJECT_SOURCE_DIR}/deps/googletest.cmake)

# Find all source files
file(GLOB_RECURSE SRC_FILES
        RELATIVE ${CMAKE_CURRENT_SOURCE_DIR}
        "${CMAKE_CURRENT_SOURCE_DIR}/*.cpp")


# Fixtures Dir
set(FIXTURES_DIR ${CMAKE_SOURCE_DIR}/test/fixtures)

# Create the test executable
add_executable(kitti_parser_tests
        ${CMAKE_CURRENT_SOURCE_DIR}/test.hpp
        ${CMAKE_CURRENT_SOURCE_DIR}/test.cpp
        ${CMAKE_CURRENT_SOURCE_DIR}/main.cpp

        ${SRC_FILES}
)
target_include_directories(kitti_parser_tests PRIVATE ${CMAKE_CURRENT_SOURCE_DIR})
target_link_libraries(kitti_parser_tests PUBLIC gtest gmock kitti_parser)
target_compile_definitions(kitti_parser_tests PRIVATE FIXTURES_DIR=\"${FIXTURES_DIR}\")

# Add a CTest entry
add_test(NAME kitti_parser_tests COMMAND kitti_parser_tests)