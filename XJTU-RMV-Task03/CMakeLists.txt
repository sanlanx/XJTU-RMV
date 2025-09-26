cmake_minimum_required(VERSION 3.10.0)
project(task VERSION 0.1.0 LANGUAGES C CXX)

add_executable(task src/main.cpp)

# 查找必需的包
find_package(Eigen3 REQUIRED)
find_package(Ceres REQUIRED)
find_package(OpenCV REQUIRED)

# 使用传统方式链接库
target_include_directories(task PRIVATE
    ${EIGEN3_INCLUDE_DIRS}
    ${CERES_INCLUDE_DIRS}
    ${OpenCV_INCLUDE_DIRS}
)

target_link_libraries(task
    ${CERES_LIBRARIES}
    ${EIGEN3_LIBRARIES}
    ${OpenCV_LIBS}
)