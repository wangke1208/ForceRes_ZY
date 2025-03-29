/**
 * Copyright(C) 2024 Rokae Technology Co., Ltd.
 * All Rights Reserved.
 *
 * Information in this file is the intellectual property of Rokae Technology Co., Ltd,
 * And may contains trade secrets that must be stored and viewed confidentially.
 *
 * @file: fc_data_computation.cpp
 * @author: wangke
 * @date: 2025/3/27
 * @brief: 力控数据流
 */

#include "rokae_header/fc_data_computation.hpp"

namespace RokaeApi {
namespace Control {

FcStatusTracker::FcStatusTracker(InitRobot* init_robot_ptr) : m_jnt_num(init_robot_ptr->GetChain().getNrOfJoints()) {
    m_dynamic_solver = new Model::DynamicSolver(init_robot_ptr->GetChain(), init_robot_ptr->GetGravity());
}

FcStatusTracker::~FcStatusTracker() { delete m_dynamic_solver; }

void FcStatusTracker::SetLoad(const LoadInertia& load) {m_load = load; }

}  // namespace Control
}  // namespace RokaeApi