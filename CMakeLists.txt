cmake_minimum_required(VERSION 3.30)
project(basicKF)

set(CMAKE_CXX_STANDARD 20)

find_package(Eigen3 REQUIRED)

include_directories(${EIGEN3_INCLUDE_DIRS})

INCLUDE_DIRECTORIES(include)
INCLUDE_DIRECTORIES(src)
AUX_SOURCE_DIRECTORY(src gSOURCE_FILES_)

add_executable(${PROJECT_NAME}
        ${gSOURCE_FILES_}
)
