cmake_minimum_required(VERSION 3.20)

# 自包含示例包：复制本目录到任意位置（需已含 3rd/、lib/、include/），在同版本 GCC + Boost 环境下即可配置构建。
project(force_res_example_sdk LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

set(EIGEN3_INCLUDE_DIR "${CMAKE_CURRENT_SOURCE_DIR}/3rd/eigen")
if(NOT EXISTS "${EIGEN3_INCLUDE_DIR}/Eigen/Core")
  message(
    FATAL_ERROR
      "缺少 Eigen: \"${EIGEN3_INCLUDE_DIR}/Eigen/Core\"。\n"
      "请在完整工程根目录执行: ./example/sync_from_repo.sh\n"
      "或手动将 Eigen 源码树复制到 example/3rd/eigen/。")
endif()
if(NOT EXISTS "${EIGEN3_INCLUDE_DIR}/Eigen/src/Core/util/Macros.h")
  message(FATAL_ERROR "Eigen 不完整，请补全 example/3rd/eigen/ 后再 cmake。")
endif()

find_package(Boost REQUIRED)
find_package(Threads REQUIRED)

set(FORCE_RES_LIB "${CMAKE_CURRENT_SOURCE_DIR}/lib/libforce_res.a")
if(NOT EXISTS "${FORCE_RES_LIB}")
  message(
    FATAL_ERROR
      "未找到 ${FORCE_RES_LIB}。\n"
      "请先在仓库根目录构建主静态库，再执行: ./example/sync_from_repo.sh")
endif()

add_subdirectory(demos)
