/**
 * Copyright(C) 2024 Rokae Technology Co., Ltd.
 * All Rights Reserved.
 *
 * Information in this file is the intellectual property of Rokae Technology Co., Ltd,
 * And may contains trade secrets that must be stored and viewed confidentially.
 *
 * @file: force_protect.hpp
 * @author: wangke
 * @date: 2025/3/27
 * @brief: 力控保护模块
 */

#ifndef ROKAE_HEADER_FORCE_PROTECT_HPP
#define ROKAE_HEADER_FORCE_PROTECT_HPP

#include <vector>

#include "rokae_header/data_structure_define.hpp"

namespace RokaeApi {
namespace Protect {

class ForceProtect {
   public:
    /**
     * @brief 构造函数，初始化力控保护对象
     *
     * @param axis_num 轴的数量
     * @param fc_params_inner 指向内部力控参数结构体的指针
     */
    ForceProtect(unsigned int axis_num, Control::FcParamsInner* fc_params_inner);

    /**
     * @brief 默认析构函数
     */
    ~ForceProtect() = default;

    /**
     * @brief 力矩模型偏差保护
     *
     * 该函数用于检查传感器反馈力矩与模型计算力矩之间的偏差是否超过预设阈值。
     * 如果超过阈值，则返回错误码，表示力矩偏差过大；否则返回正常码。
     *
     * @param sensor_feedback_trq 传感器反馈的力矩值数组
     * @param model_trq 模型计算得到的力矩值数组
     * @return 错误码
     */
    int TrqErrorProtect(const std::vector<double>& sensor_feedback_trq, const std::vector<double>& model_trq);

    /**
     * @brief 判断关节位置是否在力控区域内
     *
     * 该函数通过比较输入的关节位置与关节上下限范围，判断关节是否处于力控区域内。
     *
     * @param q_in 输入的关节位置数组
     * @param joint_limit_upper 软限位上限
     * @param joint_limit_lower 软限位下限
     * @return 如果关节位置在力控区域内返回 true，否则返回 false
     */
    bool IsInForceControlArea(const JntArray& q_in, const std::vector<double>& joint_limit_upper,
                              const std::vector<double>& joint_limit_lower);

   private:
    unsigned int m_axis_num;
    Control::FcParamsInner* m_fc_params_inner_ptr;
};

}  // namespace Protect
}  // namespace RokaeApi

#endif  // ROKAE_HEADER_FORCE_PROTECT_HPP
