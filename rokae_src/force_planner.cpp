/**
 * Copyright(C) 2024 Rokae Technology Co., Ltd.
 * All Rights Reserved.
 *
 * Information in this file is the intellectual property of Rokae Technology Co., Ltd,
 * And may contains trade secrets that must be stored and viewed confidentially.
 *
 * @file: force_planner.cpp
 * @author: wangke
 * @date: 2025/3/29
 * @brief: 力控相关功能计算模块
 */

#include "rokae_header/force_planner.hpp"

#define FC m_fc_status_ptr

namespace RokaeApi {
namespace Control {

ForcePlanner::ForcePlanner(InitRobot* init_robot_ptr, Control::FcStatusInner* fc_status_ptr,
                           Control::FcParamsInner* fc_params_inner_ptr)
    : m_init_robot_ptr(init_robot_ptr), m_fc_status_ptr(fc_status_ptr), m_fc_params_inner_ptr(fc_params_inner_ptr) {
    // 初始化成员变量或执行其他必要的操作
    m_jnt_num = m_init_robot_ptr->GetJntNum();
    //初始化
    // 初始化关节和笛卡尔的刚度、阻尼向量
    m_jnt_stiff.resize(m_jnt_num, 0.0);
    m_jnt_damp.resize(m_jnt_num, 0.0);
    m_cart_stiff.resize(6, 100.0);
    m_cart_damp.resize(6, 10.0);

    // 关节阻抗力
    m_function_jnt_imp_trq.resize(m_jnt_num);
    m_function_jnt_imp_stiff_trq.resize(m_jnt_num);
    m_function_jnt_imp_damp_trq.resize(m_jnt_num);

    // 笛卡尔阻抗力
    m_function_cart_imp_trq.resize(6);
    m_function_cart_imp_stiff_trq.resize(6);
    m_function_cart_imp_damp_trq.resize(6);

    //其他
    m_function_imp_trq.resize(m_jnt_num);      //最终输出的阻抗力
    m_function_trq_ref.resize(m_jnt_num);      //最终力矩指令
    m_function_jnt_gravity.resize(m_jnt_num);  //重力补偿
    m_function_jnt_zero_trq.resize(m_jnt_num);
    m_function_jnt_zero_trq.data.setZero();

    // 初始化关节软限位边界相关向量
    m_lower_bound.resize(m_jnt_num, -180);
    m_upper_bound.resize(m_jnt_num, 180);
    m_lower_monitor_bound.resize(m_jnt_num, -180);
    m_upper_monitor_bound.resize(m_jnt_num, 180);

    // 初始化软限位保护力相关向量
    m_protect_force_damp.resize(m_jnt_num);
    m_protect_force_stiff.resize(m_jnt_num);
    m_function_jnt_limit_trq.resize(m_jnt_num);
    fc_params_inner_ptr->m_function_params.GetParams("soft_limit_stiff", m_protect_force_stiff);
    fc_params_inner_ptr->m_function_params.GetParams("soft_limit_damp", m_protect_force_damp);

    // 初始化角度阈值常量
    m_pre_protect_angle = 5 * KDL::deg2rad;
    m_jnt_pos_safety_threshold = 10 * KDL::deg2rad;
}

const KDL::JntArray ForcePlanner::ForcePlannerUpdata() {
    // 1.阻抗力
    if (FC->drag_type == Control::DragType::DRAG_JOINT) {
        JointImpedanceUpdate(m_function_imp_trq);
    } else {
        CartImpedanceUpdate(m_function_imp_trq);
    }
    // 2.关节保护力
    JointLimitProtectUpdate(m_function_jnt_limit_trq);
    // 3.动力学补偿
    m_function_jnt_gravity = FC->jnt_gravity_trq_measure;
    // 4.合力
    for (unsigned int i = 0; i < m_jnt_num; i++) {
        m_function_trq_ref(i) = m_function_jnt_gravity(i) + m_function_imp_trq(i) + m_function_jnt_limit_trq(i);
    }
    return m_function_trq_ref;
}

void ForcePlanner::JointImpedanceUpdate(KDL::JntArray& function_imp_trq) {
    // //1.系数更新
    // m_fc_params_inner_ptr->m_function_params.GetParams("joint_stiff", m_jnt_stiff);
    // m_fc_params_inner_ptr->m_function_params.GetParams("joint_damp", m_jnt_damp);

    // //2.系数平滑（暂时不做）

    // //3.阻抗力计算(TODO:速度未滤波)
    // for (unsigned int i = 0; i < m_jnt_num; i++) {
    //     m_function_jnt_imp_stiff_trq(i) = m_jnt_stiff[i] * FC->jnt_pos_following_error(i);
    //     m_function_jnt_imp_damp_trq(i) = - (m_jnt_damp[i]) * (FC->jnt_vel_measire_filtered(i) - FC->jnt_vel_command);
    //     m_function_jnt_imp_trq(i) = m_function_jnt_imp_stiff_trq(i) + m_function_jnt_imp_damp_trq(i);
    // }

    // return m_function_jnt_imp_trq;

    //纯轴空间拖动，刚度和阻尼都是0，阻抗力直接为0即可
    function_imp_trq.data.setZero();
}

void ForcePlanner::CartImpedanceUpdate(KDL::JntArray& function_imp_trq) {
    //参数更新
    m_fc_params_inner_ptr->m_function_params.GetParams("cart_stiff", m_cart_stiff);
    m_fc_params_inner_ptr->m_function_params.GetParams("cart_damp", m_cart_damp);

    for (unsigned int i = 0; i < 6; i++) {
        m_function_cart_imp_stiff_trq(i) = m_jnt_stiff[i] * FC->cart_pos_following_error_tcp_in_fcframe[i];
        m_function_cart_imp_damp_trq(i) = m_jnt_damp[i] * (FC->cart_vel_following_error_tcp_in_fcframe[i]);
        function_imp_trq(i) = m_function_cart_imp_stiff_trq(i) + m_function_cart_imp_damp_trq(i);
    }
}

void ForcePlanner::JointLimitProtectUpdate(KDL::JntArray& protect_torque) {
    //软限位保护力计算
    for (unsigned int i = 0; i < m_jnt_num; i++) {
        // 1.当前关节角度大于软限位上限-10°
        if (FC->jnt_pos_measure(i) > m_upper_monitor_bound[i]) {
            protect_torque(i) = -m_protect_force_stiff[i] *
                                    std::min(FC->jnt_pos_measure(i) - m_upper_monitor_bound[i], m_jnt_pos_safety_threshold) -
                                m_protect_force_damp[i] * FC->jnt_vel_measure(i);
        }
        // 2. 当前关节角度小于软限位下限+10°
        else if (FC->jnt_pos_measure(i) < m_lower_monitor_bound[i]) {
            protect_torque(i) = m_protect_force_stiff[i] *
                                    std::min(m_lower_monitor_bound[i] - FC->jnt_pos_measure(i), m_jnt_pos_safety_threshold) -
                                m_protect_force_damp[i] * FC->jnt_vel_measure(i);
        }
        // 3. 当前关节角度大于软限位上限-5°
        else if (FC->jnt_pos_measure(i) > (m_upper_monitor_bound[i] - m_pre_protect_angle)) {
            protect_torque(i) = 0.0;

        }
        // 4. 当前关节角度小于软限位下限+5°
        else if (FC->jnt_pos_measure(i) < (m_lower_monitor_bound[i] + m_pre_protect_angle)) {
            protect_torque(i) = 0.0;

        }
        // 5. 当前关节角度在软限位10度范围以内
        else {
            protect_torque(i) = 0.0;
        }
    }
}

void ForcePlanner::SetSoftLimit(const std::vector<double>& joint_range_min_input,
                                const std::vector<double>& joint_range_max_input) {
    if (joint_range_min_input.size() != m_jnt_num || joint_range_max_input.size() != m_jnt_num) {
        // TODO:加日志
        return;
    }

    for (unsigned int i = 0; i < m_jnt_num; i++) {
        //更新力控相关软限位范围
        m_lower_bound[i] = joint_range_min_input[i];
        m_upper_bound[i] = joint_range_max_input[i];
        m_lower_monitor_bound[i] = m_lower_bound[i] + m_jnt_pos_safety_threshold;
        m_upper_monitor_bound[i] = m_upper_bound[i] - m_jnt_pos_safety_threshold;
    }
    return;
}
}  // namespace Control
}  // namespace RokaeApi