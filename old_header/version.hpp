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

#pragma once

#define VERSION "v1.2.4"

// v1.0版本：添加基本轴空间拖动功能，无保护功能。
// v1.1版本: 1.适配SR5机型 2.增加开启拖动时力矩偏差检查 3.增加传感器零点标定接口 4.增加按照负载信息调整力控参数
// v1.2版本: 1.新增软限位保护功能 2.修复多次开启drag_config会导致力控参数降低的bug 3.修改一下测试文件
// v1.2.1版本: 新增传感器单轴标定接口
// v1.2.2版本：新增力控增益调节接口
// v1.2.2.T版本：特殊临时版本，新增编码器位置反馈保护。
// v1.2.3版本：针对编码器跳变问题，给出代码规避方案，同时增加sr3的机型适配
// v1.2.4版本：新增关节速度反馈输出

