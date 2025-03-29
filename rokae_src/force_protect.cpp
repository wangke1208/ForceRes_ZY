/**
 * Copyright(C) 2024 Rokae Technology Co., Ltd.
 * All Rights Reserved.
 *
 * Information in this file is the intellectual property of Rokae Technology Co., Ltd,
 * And may contains trade secrets that must be stored and viewed confidentially.
 *
 * @file: force_protect.cpp
 * @author: wangke
 * @date: 2025/3/27
 * @brief: 力控保护模块
 */

#include "rokae_header/force_protect.hpp"

namespace RokaeApi {
namespace Protect {
ForceProtect::ForceProtect(unsigned int axis_num)
    : m_axis_num(axis_num){};

int ForceProtect::TrqErrorProtect(const std::vector<double>& sensor_feedback_trq, const std::vector<double>& model_trq,
                                  const std::vector<double> trq_error_threshold) {
    if (sensor_feedback_trq.size() != m_axis_num || model_trq.size() != m_axis_num || trq_error_threshold.size() != m_axis_num) {
        return SIZE_ERROR;
    }
    for (unsigned int i = 0; i < m_axis_num; i++) {
        if (std::abs(sensor_feedback_trq[i] - model_trq[i]) > trq_error_threshold[i]) {
            return EXCESSIVE_TORQUE_ERROR;
        }
    }
}

bool ForceProtect::IsInForceControlArea(const JntArray& q_in, const std::vector<double>& joint_limit_upper, const std::vector<double>& joint_limit_lower) {
    for(unsigned int i = 0 ; i< m_axis_num ; i++){
        if((q_in(i) < (joint_limit_lower[i] + 9.5*KDL::deg2rad))
        or (q_in(i) > (joint_limit_upper[i] - 9.5*KDL::deg2rad))){
            return true;
        }
    }
    return false;
}
};  // namespace Protect
}  // namespace RokaeApi