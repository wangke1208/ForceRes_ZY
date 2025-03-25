/**
 * Copyright(C) 2024 Rokae Technology Co., Ltd.
 * All Rights Reserved.
 *
 * Information in this file is the intellectual property of Rokae Technology Co., Ltd,
 * And may contains trade secrets that must be stored and viewed confidentially.
 *
 * @file: force_planner.cpp
 * @author: wangke
 * @date: 2024/6/17
 * @brief: 力控相关功能计算模块
 */

#include "../rokae_header/force_planner.hpp"

namespace RokaeApi {
namespace Control {

ForcePlanner::ForcePlanner(InitRobot* init_robot_ptr, FcStatusInfo* fc_status_info)
    : m_jnt_num(init_robot_ptr->GetChain().getNrOfJoints()), m_fc_status_info(fc_status_info) {
    m_joint_range_min.resize(m_jnt_num);
    m_joint_range_max.resize(m_jnt_num);
    //给一个默认值，防止出现意外
    m_joint_range_min = {-175*KDL::deg2rad, -160*KDL::deg2rad, -170*KDL::deg2rad, -175*KDL::deg2rad, -175*KDL::deg2rad, -175*KDL::deg2rad};
    m_joint_range_max = {175*KDL::deg2rad, 150*KDL::deg2rad, 140*KDL::deg2rad, 175*KDL::deg2rad, 175*KDL::deg2rad, 175*KDL::deg2rad};

    m_q_feedback.resize(m_jnt_num);
    m_q_dot_feedback.resize(m_jnt_num);
    m_lower_bound.resize(m_jnt_num);
    m_upper_bound.resize(m_jnt_num);
    m_lower_monitor_bound.resize(m_jnt_num);
    m_upper_monitor_bound.resize(m_jnt_num);
    m_protect_torque.resize(m_jnt_num);

    m_pre_protect_angle = 5 * KDL::deg2rad;
    m_jnt_pos_safety_threshold = 10 * KDL::deg2rad;

    // SR系列软限位刚度和阻尼调整(目前只有SR机型，后续可在调整)
    m_protect_force_stiffness.assign(m_jnt_num, 500.0);
    m_protect_force_damping.assign(m_jnt_num, 20.0f);

    for (unsigned int i = 0; i < m_jnt_num; i++) {
        m_lower_bound(i) = m_joint_range_min[i];
        m_upper_bound(i) = m_joint_range_max[i];
        m_lower_monitor_bound(i) = m_lower_bound(i) + m_jnt_pos_safety_threshold;
        m_upper_monitor_bound(i) = m_upper_bound(i) - m_jnt_pos_safety_threshold;
    }
}

ForcePlanner::~ForcePlanner() {}

void ForcePlanner::ChangeLimitTemp(const std::vector<double>& limit_lower, const std::vector<double>& limit_upper) {
    for (unsigned int i = 0; i < m_jnt_num; i++) {
        m_lower_monitor_bound(i) = limit_lower[i];
        m_upper_monitor_bound(i) = limit_upper[i];
    }
}

int ForcePlanner::SetSoftLimit(const std::vector<double> joint_range_min, const std::vector<double> joint_range_max) {
    //与force_control.cpp不同，这个接口输入的是弧度值

    for (unsigned int i = 0; i < m_jnt_num; i++) {
        m_joint_range_min[i] = joint_range_min[i];
        m_joint_range_max[i] = joint_range_max[i];
        //更新力控相关软限位范围
        m_lower_bound(i) = m_joint_range_min[i];
        m_upper_bound(i) = m_joint_range_max[i];
        m_lower_monitor_bound(i) = m_lower_bound(i) + m_jnt_pos_safety_threshold;
        m_upper_monitor_bound(i) = m_upper_bound(i) - m_jnt_pos_safety_threshold;
    }
    return SOLVE_NOERROR;
}

const KDL::JntArray& ForcePlanner::JointLimitUpdate(std::vector<double> trq_comp_coef) {
    //位置、速度反馈赋值
    m_q_feedback = m_fc_status_info->jnt_pos_measure;
    m_q_dot_feedback = m_fc_status_info->jnt_vel_measure;

    //软限位保护力计算
    for (unsigned int i = 0; i < m_jnt_num; i++) {
        //1.当前关节角度大于软限位上限-10°
        if (m_q_feedback(i) > m_upper_monitor_bound(i)) {
            m_protect_torque(i) = -m_protect_force_stiffness[i] * std::min(m_q_feedback(i) - m_upper_monitor_bound(i), m_jnt_pos_safety_threshold) -
                                  m_protect_force_damping[i] * m_q_dot_feedback(i);
            trq_comp_coef[i] = 0.0;
        }
        //2. 当前关节角度小于软限位下限+10°
        else if (m_q_feedback(i) < m_lower_monitor_bound(i)) {
            m_protect_torque(i) = m_protect_force_stiffness[i] * std::min(m_lower_monitor_bound(i) - m_q_feedback(i), m_jnt_pos_safety_threshold) -
                                  m_protect_force_damping[i] * m_q_dot_feedback(i);
            trq_comp_coef[i] = 0.0;
        }
        //3. 当前关节角度大于软限位上限-5°        
         else if (m_q_feedback(i) > (m_upper_monitor_bound(i) - m_pre_protect_angle)) {
            m_protect_torque(i) = 0.0;
            trq_comp_coef[i] = (m_upper_monitor_bound(i) - m_q_feedback(i)) / m_pre_protect_angle;
            if (trq_comp_coef[i] < 0.1) {
                trq_comp_coef[i] = 0.1;
            }
        } 
        //4. 当前关节角度小于软限位下限+5°        
        else if (m_q_feedback(i) < (m_lower_monitor_bound(i) + m_pre_protect_angle)) {
            m_protect_torque(i) = 0.0;
            trq_comp_coef[i] = (m_q_feedback(i) - m_lower_monitor_bound(i)) / m_pre_protect_angle;
            if (trq_comp_coef[i] < 0.1) {
                trq_comp_coef[i] = 0.1;
            }
        } 
        //5. 当前关节角度在软限位10度范围以内              
        else {
            m_protect_torque(i) = 0.0;
            trq_comp_coef[i] = 1.0;
        }
        // cout << "m_joint_range_min = " <<m_joint_range_min[i]/PI*180<<endl;
        //  cout << "m_joint_range_max = " <<m_joint_range_max[i]/PI*180<<endl;
        //  cout << "m_q_feedback = " <<m_q_feedback(i)/PI*180<<endl;
        //  cout << "m_q_dot_feedback = " <<m_q_dot_feedback(i)/PI*180<<endl;
        //  cout << "m_protect_torque = " <<m_protect_torque(i)<<endl;

    }
    return m_protect_torque;
}

}  // namespace Control

}  // namespace RokaeApi
