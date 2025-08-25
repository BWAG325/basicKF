cmake_minimum_required(VERSION 3.30)
project(basicKF)

set(CMAKE_CXX_STANDARD 20)

find_package(Eigen3 REQUIRED)
find_package(OpenCV REQUIRED)

include_directories(${EIGEN3_INCLUDE_DIRS})
# 现代CMake推荐使用target_include_directories
file(GLOB SOURCES "src/*.cpp")  # 只收集.cpp文件
file(GLOB HEADERS "include/*.h")  # 头文件

add_executable(Test
        ${SOURCES}
)
target_include_directories(Test PRIVATE
        ${OpenCV_INCLUDE_DIRS}
        include
)

target_link_libraries(Test PRIVATE ${OpenCV_LIBS})
