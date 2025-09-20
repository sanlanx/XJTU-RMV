cmake_minimum_required(VERSION 3.10.0)
project(task2 VERSION 0.1.0 LANGUAGES C CXX)

add_executable(task2_main src/main.cpp)
add_executable(task2_catch src/catchLight.cpp)

find_package(OpenCV REQUIRED)
include_directories(${OpenCV_INCLUDE_DIRS})
target_link_libraries(task2_main ${OpenCV_LIBS})
target_link_libraries(task2_catch ${OpenCV_LIBS})