cmake_minimum_required(VERSION 3.22)
project(basicKF LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

if (NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE RelWithDebInfo CACHE STRING "" FORCE)
endif()

find_package(Eigen3 3.3 REQUIRED NO_MODULE)
find_package(yaml-cpp REQUIRED)

if (TARGET yaml-cpp::yaml-cpp)
    set(YAML_CPP_TARGET yaml-cpp::yaml-cpp)
else()
    set(YAML_CPP_TARGET yaml-cpp)
endif()

add_library(sim_core STATIC
    src/scene.cpp
    src/sensor.cpp
    src/config.cpp
)
target_include_directories(sim_core PUBLIC include)
target_link_libraries(sim_core PUBLIC Eigen3::Eigen PRIVATE ${YAML_CPP_TARGET})
target_compile_options(sim_core PRIVATE -Wall -Wextra -Wpedantic)

add_executable(sim_demo apps/sim_demo.cpp)
target_link_libraries(sim_demo PRIVATE sim_core)
target_compile_options(sim_demo PRIVATE -Wall -Wextra -Wpedantic)

add_executable(sim_contract_test tests/sim_contract_test.cpp)
target_link_libraries(sim_contract_test PRIVATE sim_core)
target_compile_definitions(sim_contract_test PRIVATE SIM_SOURCE_DIR="${CMAKE_SOURCE_DIR}")
target_compile_options(sim_contract_test PRIVATE -Wall -Wextra -Wpedantic)

enable_testing()
add_test(NAME sim_contract COMMAND sim_contract_test)
