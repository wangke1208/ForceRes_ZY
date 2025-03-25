/**
 * Copyright(C) 2024 Rokae Technology Co., Ltd.
 * All Rights Reserved.
 *
 * Information in this file is the intellectual property of Rokae Technology Co., Ltd,
 * And may contains trade secrets that must be stored and viewed confidentially.
 *
 * @file: fc_status_tracker.cpp
 * @author: wangke
 * @date: 2024/4/25
 * @brief: 力控计算模块
 */

#include "../rokae_header/fc_status_tracker.hpp"

#define FC m_fc_status_info
namespace RokaeApi {
namespace Control {

FcStatusTracker::FcStatusTracker(InitRobot* init_robot_ptr, FcStatusInfo* fc_status_info)
    : m_jnt_num(init_robot_ptr->GetChain().getNrOfJoints()), m_fc_status_info(fc_status_info) {
    m_jnt_vel_filter.resize(m_jnt_num);
    m_period = 0.001;  //这样写不好，暂时先这样，后续调到Initialize或ForceControl
    m_dynamic_solver = new Model::DynamicSolver(init_robot_ptr->GetChain(), init_robot_ptr->GetGravity());
    for (unsigned int i = 0; i < m_jnt_num; i++) {
        m_jnt_vel_filter[i] = new Lowpassfilter(m_period, 2);
        m_jnt_vel_filter[i]->CalcFilterPara(100.0);
    }
}

FcStatusTracker::~FcStatusTracker() {
    for (unsigned int i = 0; i < m_jnt_num; i++) {
        delete m_jnt_vel_filter[i];
    }
    delete m_dynamic_solver;
}

void FcStatusTracker::UpdateDateStream() {
    KDL::Subtract(FC->jnt_pos_command, FC->jnt_pos_measure, FC->jnt_pos_following_error);
    //滤波
    for(unsigned int i = 0 ; i < m_jnt_num ; i++){
        FC->jnt_vel_measure_filter(i) = m_jnt_vel_filter[i] -> Update(FC->jnt_vel_measure(i));
    }
    FC->jnt_trq_gra_command = m_dynamic_solver->GetGravity(m_load, FC->jnt_pos_command);
    FC->jnt_trq_gra_measure = m_dynamic_solver->GetGravity(m_load, FC->jnt_pos_measure);
    m_dynamic_solver->JntToMass(m_load, FC->jnt_pos_measure, FC->jnt_inertia_matrix_measure);
}

void FcStatusTracker::SetLoad(const LoadInertia& load) {
    m_load.m = load.m;
    m_load.mx = load.mx;
    m_load.m_cog = load.m_cog;
    for (unsigned int i = 0; i < 6; i++) {
        m_load.inertia[i] = load.inertia[i];
    }
}
}  // namespace Control
}  // namespace RokaeApi