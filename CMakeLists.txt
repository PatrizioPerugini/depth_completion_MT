cmake_minimum_required(VERSION 3.3.1)

project(img_completion)
find_package(OpenCV REQUIRED)
include_directories(${OpenCV_INCLUDE_DIRS} include)

add_library(img_completion src/img_completion.cpp)

add_executable(main src/main.cpp)
#dd_executable(utils src/utils.cpp)

target_link_libraries(main img_completion ${OpenCV_LIBRARIES})