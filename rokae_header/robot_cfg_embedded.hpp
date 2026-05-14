/**
 * Copyright(C) 2024 Rokae Technology Co., Ltd.
 * All Rights Reserved.
 *
 * @file robot_cfg_embedded.hpp
 * @brief 构建期内嵌机型 JSON 查询声明（实现由 CMake 生成的 embedded_robot_cfgs.cpp 提供）
 */
#ifndef ROKAE_HEADER_ROBOT_CFG_EMBEDDED_HPP
#define ROKAE_HEADER_ROBOT_CFG_EMBEDDED_HPP

#include <string_view>

namespace RokaeApi {
namespace RobotCfgEmbedded {

/** 按机型目录名查找内嵌 cfg 正文；未找到返回空 string_view */
std::string_view GetEmbeddedRobotCfgJson(std::string_view model_name);

}  // namespace RobotCfgEmbedded
}  // namespace RokaeApi

#endif
