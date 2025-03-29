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

#include "rokae_header/data_structure_define.hpp"

namespace RokaeApi {
namespace Protect {
class ForceProtect {
   public:
    ForceProtect(){};
    ForceProtect(unsigned int axis_num){};
    ~ForceProtect(){};
    //力矩模型偏差保护
    int TrqErrorProtect(const std::vector<double>& sensor_feedback_trq, const std::vector<double>& model_trq,
                        const std::vector<double> trq_error_threshold);
    bool IsInForceControlArea(const JntArray& q_in, const std::vector<double>& joint_limit_upper,
                                            const std::vector<double>& joint_limit_lower);

   private:
    unsigned int m_axis_num;
};
}  // namespace Protect

}  // namespace RokaeApi

#endif