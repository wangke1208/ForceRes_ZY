/**
 * Copyright(C) 2024 Rokae Technology Co., Ltd.
 * All Rights Reserved.
 *
 * @file robot_cfg_loader.hpp
 * @brief 从 JSON 文本填充 RobotConfiguration；机型名 L/R 解析；可选开发路径读盘
 */
#ifndef ROKAE_HEADER_ROBOT_CFG_LOADER_HPP
#define ROKAE_HEADER_ROBOT_CFG_LOADER_HPP

#include <string>
#include <string_view>

#include "rokae_header/data_structure_define.hpp"
#include "rokae_header/robot_cfg_embedded.hpp"
#include "rokae_header/robot_config.hpp"

namespace RokaeApi {

/**
 * 从机型名字符串解析左右手构型（匹配 `_数字` 或 `_数字.数字` 后的独立 L/R，再遇分隔符 `-`/`_`）。
 * 禁止 naive 子串搜索，避免命中如 W4C**L** 等字段。
 */
int ParseMechUnitFromModelName(std::string_view model_name, Model::MechUnitType& out_type);

/** 解析 JSON 填入 out_cfg（会先按 CONFIGURATION.AXES_NUMBER Resize）。当前 ARC 固定关节模板仅支持 7 轴。 */
int LoadRobotConfigurationFromJsonString(std::string_view json_text, Model::RobotConfiguration& out_cfg);

/**
 * 获取机型 cfg JSON：若编译定义 FORCE_RES_ROBOT_CFG_DEV_PATH 且环境变量 FORCE_RES_ROBOT_CFG_DEV_PATH
 * 指向目录，则优先读取 <root>/<model>/<model>.cfg；否则使用内嵌表。
 */
int ResolveRobotCfgJson(const std::string& model_name, std::string& out_json);

}  // namespace RokaeApi

#endif
