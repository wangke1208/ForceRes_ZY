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
const char* const VERSION = "v1.0.2";

// v1.0版本：添加基本轴空间拖动和基础阻抗功能，无保护功能。
// v1.0.1版本：删除掉力控软限位保护
// v1.0.2版本：添加用户自定义旋转方向设置接口

}  // namespace RokaeApi

#endif
