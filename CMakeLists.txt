cmake_minimum_required(VERSION 3.11)

project(kitti_parser)
set(CMAKE_CXX_STANDARD 17)

include(FetchContent)
set(FETCHCONTENT_QUIET FALSE)

# Include libraries
find_package(OpenCV 4 REQUIRED core plot videoio ximgproc highgui)
include(${PROJECT_SOURCE_DIR}/deps/yaml-cpp.cmake)

# Create the core library (excludes the main file)
add_library(kitti_parser STATIC
    ${PROJECT_SOURCE_DIR}/src/kitti_parser/Parser.cpp
    ${PROJECT_SOURCE_DIR}/src/kitti_parser/util/Loader.cpp
    ${PROJECT_SOURCE_DIR}/src/kitti_parser/util/timestamp.hpp
    ${PROJECT_SOURCE_DIR}/src/kitti_parser/util/timestamp.cpp
)
target_link_libraries(kitti_parser PUBLIC yaml-cpp opencv_core opencv_imgproc opencv_imgcodecs)
target_include_directories(kitti_parser PUBLIC ${PROJECT_SOURCE_DIR}/src)

# Create the main executable, and link libraries
add_executable(main_text ${PROJECT_SOURCE_DIR}/src/main_text.cpp)
target_link_libraries(main_text kitti_parser opencv_highgui)

# Create the main executable, and link libraries
add_executable(main_image ${PROJECT_SOURCE_DIR}/src/main_image.cpp)
target_link_libraries(main_image kitti_parser opencv_highgui)

enable_testing()
add_subdirectory(test)
