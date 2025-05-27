/**
 * Copyright(C) 2024 Rokae Technology Co., Ltd.
 * All Rights Reserved.
 *
 * Information in this file is the intellectual property of Rokae Technology Co., Ltd,
 * And may contains trade secrets that must be stored and viewed confidentially.
 *
 * @file: version.hpp
 * @author: wangke
 * @date:
 * @brief: 版本记录
 */

#ifndef ROKAE_HEADER_VERSION_HPP_
#define ROKAE_HEADER_VERSION_HPP_
namespace RokaeApi {
const char* const VERSION = "v1.0.4";

// v1.0版本：添加基本轴空间拖动功能，无保护功能。
// v1.0.1版本：添加笛卡尔和轴空间阻抗
// v1.0.2版本：完善笛卡尔&轴空间阻抗、修复编译报错
// v1.0.3版本: 1.修复机器人机型限制bug 2.修复科氏力求解错误 3.修复重力矢量设置错误
//             4.修复惯量参数读取错误bug 5.新增LOG输出容器和Eigen数组的宏 6.修复根据负载调节增益接口失效
//             7.新增6轴机器人接口 8.新增用户下发指令接口
// v1.0.4版本: 1.新增运动学逆解接口 2.新增逆解接口test文件
}  // namespace RokaeApi

#endif
