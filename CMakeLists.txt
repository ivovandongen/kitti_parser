cmake_minimum_required(VERSION 3.11)

project(kitti_parser)
set(CMAKE_CXX_STANDARD 17)

# Include libraries
find_package(Boost REQUIRED COMPONENTS system filesystem thread date_time)
find_package(OpenCV 4 REQUIRED core plot videoio ximgproc highgui)
include(${PROJECT_SOURCE_DIR}/deps/yaml-cpp.cmake)

# Create the core library (excludes the main file)
add_library(kitti_parser STATIC
    src/kitti_parser/Parser.cpp
    src/kitti_parser/util/Loader.cpp
)
target_link_libraries(kitti_parser PUBLIC yaml-cpp opencv_core opencv_imgproc opencv_imgcodecs Boost::boost Boost::filesystem)
target_include_directories(kitti_parser PUBLIC src)

# Create the main executable, and link libraries
add_executable(main_text src/main_text.cpp)
target_link_libraries(main_text kitti_parser opencv_highgui)

# Create the main executable, and link libraries
add_executable(main_image src/main_image.cpp)
target_link_libraries(main_image kitti_parser opencv_highgui)
