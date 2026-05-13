/**
 * 仅用于 example/demos：替代原 rokae_header/log_util.hpp 中的打印宏，使 demo 在「只 -I
 * example/include + Eigen」的客户式布局下仍能编译。不随 SDK 交付给客户。
 */
#pragma once

/** 自包含 demo 用圆周率（替代原 KDL::PI，不依赖 KDL 头） */
#ifndef FORCE_RES_EXAMPLE_PI
#define FORCE_RES_EXAMPLE_PI 3.14159265358979323846
#endif

#include <Eigen/Dense>
#include <iterator>
#include <sstream>
#include <string>

#include <spdlog/sinks/stdout_color_sinks.h>
#include <spdlog/spdlog.h>

namespace {
struct DemoSpdlogInit {
    DemoSpdlogInit() {
        if (spdlog::default_logger() == nullptr) {
            spdlog::set_default_logger(spdlog::stdout_color_mt("force_res_demos"));
        }
    }
};
const DemoSpdlogInit k_demo_spdlog_init{};
}  // namespace

#define LOG_ERROR(...) SPDLOG_ERROR(__VA_ARGS__)
#define LOG_INFO(...) SPDLOG_INFO(__VA_ARGS__)

#define SPD_EIGEN_MATRIX(MAT)                                                                                \
    do {                                                                                                     \
        std::stringstream ss;                                                                                \
        ss << (MAT);                                                                                         \
        spdlog::info("{} =\n{}", #MAT, ss.str());                                                           \
    } while (0)

#define SPD_CONTAINER(TAG, CONTAINER)                                                                        \
    do {                                                                                                     \
        std::ostringstream oss;                                                                              \
        auto it = std::begin(CONTAINER);                                                                     \
        auto end = std::end(CONTAINER);                                                                      \
        if (it != end) {                                                                                     \
            oss << *it;                                                                                      \
            ++it;                                                                                            \
        }                                                                                                    \
        for (; it != end; ++it) {                                                                            \
            oss << ", " << *it;                                                                              \
        }                                                                                                    \
        spdlog::info("{} {}", TAG, oss.str());                                                               \
    } while (0)
