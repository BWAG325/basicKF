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

add_executable(kf_future apps/kf_future.cpp)
target_link_libraries(kf_future PRIVATE sim_core kf_core ${YAML_CPP_TARGET} ${OpenCV_LIBS})
target_include_directories(kf_future PRIVATE ${OpenCV_INCLUDE_DIRS})
target_compile_options(kf_future PRIVATE -Wall -Wextra -Wpedantic)

add_executable(kf_robust apps/kf_robust.cpp)
target_link_libraries(kf_robust PRIVATE sim_core kf_core ${YAML_CPP_TARGET} ${OpenCV_LIBS})
target_include_directories(kf_robust PRIVATE ${OpenCV_INCLUDE_DIRS})
target_compile_options(kf_robust PRIVATE -Wall -Wextra -Wpedantic)

add_executable(sim_demo apps/sim_demo.cpp)
target_link_libraries(sim_demo PRIVATE sim_core ${OpenCV_LIBS})
target_include_directories(sim_demo PRIVATE ${OpenCV_INCLUDE_DIRS})
target_compile_options(sim_demo PRIVATE -Wall -Wextra -Wpedantic)

add_executable(kf_demo apps/kf_demo.cpp)
target_link_libraries(kf_demo PRIVATE sim_core kf_core ${OpenCV_LIBS})
target_include_directories(kf_demo PRIVATE ${OpenCV_INCLUDE_DIRS})
target_compile_options(kf_demo PRIVATE -Wall -Wextra -Wpedantic)

add_executable(kf_compare apps/kf_compare.cpp)
target_link_libraries(kf_compare PRIVATE sim_core kf_core ${YAML_CPP_TARGET} ${OpenCV_LIBS})
target_include_directories(kf_compare PRIVATE ${OpenCV_INCLUDE_DIRS})
target_compile_options(kf_compare PRIVATE -Wall -Wextra -Wpedantic)

add_executable(kf_tune apps/kf_tune.cpp)
target_link_libraries(kf_tune PRIVATE sim_core kf_core ${YAML_CPP_TARGET} ${OpenCV_LIBS})
target_include_directories(kf_tune PRIVATE ${OpenCV_INCLUDE_DIRS})
target_compile_options(kf_tune PRIVATE -Wall -Wextra -Wpedantic)

add_executable(imm_demo apps/imm_demo.cpp)
target_link_libraries(imm_demo PRIVATE sim_core kf_core ${OpenCV_LIBS})
target_include_directories(imm_demo PRIVATE ${OpenCV_INCLUDE_DIRS})
target_compile_options(imm_demo PRIVATE -Wall -Wextra -Wpedantic)

add_executable(imm_test tests/imm_test.cpp)
target_link_libraries(imm_test PRIVATE kf_core)
target_compile_options(imm_test PRIVATE -Wall -Wextra -Wpedantic)

add_executable(ca_test tests/ca_test.cpp)
target_link_libraries(ca_test PRIVATE kf_core)
target_compile_options(ca_test PRIVATE -Wall -Wextra -Wpedantic)

add_executable(cj_test tests/cj_test.cpp)
target_link_libraries(cj_test PRIVATE kf_core)
target_compile_options(cj_test PRIVATE -Wall -Wextra -Wpedantic)

add_executable(ct_spin_test tests/ct_spin_test.cpp)
target_link_libraries(ct_spin_test PRIVATE kf_core)
target_compile_options(ct_spin_test PRIVATE -Wall -Wextra -Wpedantic)

add_executable(sim_contract_test tests/sim_contract_test.cpp)
target_link_libraries(sim_contract_test PRIVATE sim_core)
target_compile_options(sim_contract_test PRIVATE -Wall -Wextra -Wpedantic)

enable_testing()
add_test(NAME sim_contract COMMAND sim_contract_test)
add_test(NAME imm COMMAND imm_test)
add_test(NAME ca COMMAND ca_test)
add_test(NAME cj COMMAND cj_test)
add_test(NAME ct_spin COMMAND ct_spin_test)
