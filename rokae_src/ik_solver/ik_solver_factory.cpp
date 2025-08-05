/**
 * Copyright(C) 2024 Rokae Technology Co., Ltd.
 * All Rights Reserved.
 *
 * Information in this file is the intellectual property of Rokae Technology Co., Ltd,
 * And may contains trade secrets that must be stored and viewed confidentially.
 *
 * @file: ik_solver_factory.cpp
 * @author: wangke
 * @date: 2025/8/4
 * @brief: 逆运动学求解器工厂类
 */

#include "rokae_header/ik_solver/ik_solver_factory.hpp"

namespace RokaeApi {
namespace Model {

IKSolverFactory::IKSolverFactory(const KDL::Chain& chain, const Model::ModelParams& model_param)
    : m_chain(chain), m_model_param(model_param) {}

std::unique_ptr<IKSolverBase> IKSolverFactory::CreateIkSolverPos() {
    if (m_model_param.axis_num != 7) {
        return nullptr;
    }
    if (m_model_param.is_wrist_cross) {
        return std::make_unique<inverse_kinematics_cross_solver>(m_chain, m_model_param);
    } else {
        return std::make_unique<inverse_kinematics_solver>(m_chain, m_model_param);
    }
}

}  // namespace Model
}  // namespace RokaeApi


