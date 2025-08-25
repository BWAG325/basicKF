cmake_minimum_required(VERSION 3.30)
project(basicKF)

set(CMAKE_CXX_STANDARD 20)

find_package(Eigen3 REQUIRED)
find_package(OpenCV REQUIRED)

include_directories(${EIGEN3_INCLUDE_DIRS})
# 注意要使用递归去寻找文件
file(GLOB_RECURSE SOURCES "src/*.cpp")  # 只收集.cpp文件
file(GLOB_RECURSE HEADERS "include/*.h")  # 头文件

add_executable(${PROJECT_NAME}
        ${SOURCES}
)
target_include_directories(${PROJECT_NAME} PUBLIC
        ${OpenCV_INCLUDE_DIRS}
        ${EIGEN3_INCLUDE_DIRS}
        include
)

target_link_libraries(${PROJECT_NAME} PRIVATE ${OpenCV_LIBS})
