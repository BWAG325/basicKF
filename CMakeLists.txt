cmake_minimum_required(VERSION 3.22)
project(basicKF LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

if (NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE RelWithDebInfo CACHE STRING "" FORCE)
endif()

# 把工程根目录编进程序：相对路径统一相对工程根解析，CLion / 命令行行为一致
add_compile_definitions(SIM_PROJECT_SOURCE_DIR="${PROJECT_SOURCE_DIR}")

find_package(Eigen3 3.3 REQUIRED NO_MODULE)
find_package(yaml-cpp REQUIRED)
find_package(OpenCV REQUIRED)

if (TARGET yaml-cpp::yaml-cpp)
    set(YAML_CPP_TARGET yaml-cpp::yaml-cpp)
else()
    set(YAML_CPP_TARGET yaml-cpp)
endif()

add_library(kf_core INTERFACE)
target_include_directories(kf_core INTERFACE include)
target_link_libraries(kf_core INTERFACE Eigen3::Eigen)

add_library(sim_core STATIC
    src/config.cpp
    src/simulator.cpp
    src/motion/preset.cpp
    src/sensing/sensor.cpp
)
target_include_directories(sim_core PUBLIC include)
target_link_libraries(sim_core PUBLIC Eigen3::Eigen PRIVATE ${YAML_CPP_TARGET})
target_compile_options(sim_core PRIVATE -Wall -Wextra -Wpedantic)

add_executable(sim_demo apps/sim_demo.cpp)
target_link_libraries(sim_demo PRIVATE sim_core ${OpenCV_LIBS})
target_include_directories(sim_demo PRIVATE ${OpenCV_INCLUDE_DIRS})
target_compile_options(sim_demo PRIVATE -Wall -Wextra -Wpedantic)

add_executable(kf_demo apps/kf_demo.cpp)
target_link_libraries(kf_demo PRIVATE sim_core kf_core ${OpenCV_LIBS})
target_include_directories(kf_demo PRIVATE ${OpenCV_INCLUDE_DIRS})
target_compile_options(kf_demo PRIVATE -Wall -Wextra -Wpedantic)

add_executable(sim_contract_test tests/sim_contract_test.cpp)
target_link_libraries(sim_contract_test PRIVATE sim_core)
target_compile_options(sim_contract_test PRIVATE -Wall -Wextra -Wpedantic)

enable_testing()
add_test(NAME sim_contract COMMAND sim_contract_test)
