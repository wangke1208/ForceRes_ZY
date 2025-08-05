/**
 * Copyright(C) 2024 Rokae Technology Co., Ltd.
 * All Rights Reserved.
 *
 * Information in this file is the intellectual property of Rokae Technology Co., Ltd,
 * And may contains trade secrets that must be stored and viewed confidentially.
 *
 * @file: ik_solver_base.cpp
 * @author: wangke
 * @date: 2025/8/4
 * @brief: 逆运动学求解器基类
 */

#include "rokae_header/ik_solver/ik_solver_base.hpp"

namespace RokaeApi {
namespace Model {

IKSolverBase::IKSolverBase(const KDL::Chain& chain, const Model::ModelParams& model_param)
    : m_chain(chain), m_model_param(model_param), m_fkpos_ptr(std::make_shared<KDL::ChainFkSolverPos_recursive>(m_chain)) {}


}  // namespace Model
}  // namespace RokaeApi

