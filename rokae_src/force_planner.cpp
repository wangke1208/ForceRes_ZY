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

// --------------------- 构造函数 ---------------------
ForcePlanner::ForcePlanner(InitRobot* init_robot_ptr, FcStatusInner* fc_status_ptr, FcParamsInner* fc_params_inner_ptr)
    : m_init_robot_ptr(init_robot_ptr), m_fc_status_ptr(fc_status_ptr), m_fc_params_inner_ptr(fc_params_inner_ptr) {
    m_jnt_num = m_init_robot_ptr->GetJntNum();

    // 初始化关节和笛卡尔的刚度、阻尼向量
    m_jnt_stiff.resize(m_jnt_num, 0.0);
    m_jnt_damp.resize(m_jnt_num, 0.0);
    m_cart_stiff.resize(6, 100.0);
    m_cart_damp.resize(6, 10.0);
    m_null_stiff.resize(1, 100.0);

    // 笛卡尔阻抗力
    m_function_cart_imp_trq.Zero();
    m_function_cart_imp_stiff_trq.resize(6);
    m_function_cart_imp_damp_trq.resize(6);
    m_function_cart_imp_joint_trq.resize(m_jnt_num);
    m_function_cart_imp_trq_in_base = Eigen::Matrix<double, 6, 1>::Zero();
    m_function_cart_imp_trq_in_base_wrench.Zero();
    m_function_null_space_trq.resize(m_jnt_num);
    m_function_null_space_trq_final.resize(m_jnt_num);
    m_matrix_temp = Eigen::MatrixXd::Identity(7, 7);
    //关节阻抗力
    m_function_jnt_imp_damp_trq.resize(m_jnt_num);
    m_function_jnt_imp_stiff_trq.resize(m_jnt_num);

    // 其他
    m_function_imp_trq.resize(m_jnt_num);      // 最终输出的阻抗力
    m_function_trq_ref.resize(m_jnt_num);      // 最终力矩指令
    m_function_jnt_gravity.resize(m_jnt_num);  // 重力补偿
    m_function_jnt_zero_trq.resize(m_jnt_num);
    m_function_jnt_zero_trq.data.setZero();

    // 初始化关节软限位边界相关向量
    m_lower_bound.resize(m_jnt_num, -180 * KDL::deg2rad);
    m_upper_bound.resize(m_jnt_num, 180 * KDL::deg2rad);
    m_lower_monitor_bound.resize(m_jnt_num, -180 * KDL::deg2rad);
    m_upper_monitor_bound.resize(m_jnt_num, 180 * KDL::deg2rad);

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
 
 // --------------------- 析构函数 ---------------------
 ForcePlanner::~ForcePlanner() {
     // 无特殊析构操作
 }
 
 // --------------------- 力控更新 ---------------------
 void ForcePlanner::ForcePlannerUpdata() {
     // 1.阻抗力
     if (FC->drag_type == Control::DragType::DRAG_JOINT || FC->drag_type == Control::DragType::IMPEDANCE_JOINT) {
         JointImpedanceUpdate(m_function_imp_trq);
     } else {
         CartImpedanceUpdate(m_function_imp_trq);
     }
     // 2.关节保护力
     JointLimitProtectUpdate(m_function_jnt_limit_trq);
     // 3.动力学补偿
     m_function_jnt_gravity = FC->jnt_gravity_trq_measure;
     // 4.惯量
     // 5.合力
     for (unsigned int i = 0; i < m_jnt_num; i++) {
         FC->jnt_trq_final_cmd(i) = m_function_jnt_gravity(i) + m_function_imp_trq(i) + m_function_jnt_limit_trq(i);
     }
     return;
 }

 void ForcePlanner::ForcePlannerUpdataUser(const KDL::JntArray& trq_cmd_by_user) {
     // 1.用户设定的力矩
     for (unsigned int i = 0; i < m_jnt_num; i++) {
         FC->jnt_trq_final_cmd(i) = trq_cmd_by_user(i);
     }
     return;
 }

 // --------------------- 阻抗力更新 ---------------------
 void ForcePlanner::JointImpedanceUpdate(KDL::JntArray& function_imp_trq) {
     if (FC->drag_type == Control::DragType::IMPEDANCE_JOINT) {
         for (unsigned int i = 0; i < m_jnt_num; i++) {
             m_function_jnt_imp_stiff_trq(i) = m_jnt_stiff[i] * FC->jnt_pos_following_error(i);
             m_function_jnt_imp_damp_trq(i) = m_jnt_damp[i] * (FC->jnt_vel_command(i) - FC->jnt_vel_measure(i));
             function_imp_trq.data(i) = m_function_jnt_imp_stiff_trq(i) + m_function_jnt_imp_damp_trq(i);
         }
     } else {
         // 纯轴空间拖动，刚度和阻尼都是0，阻抗力直接为0即可
         function_imp_trq.data.setZero();
     }
 }

 void ForcePlanner::CartImpedanceUpdate(KDL::JntArray& function_imp_trq) {
     for (unsigned int i = 0; i < 6; i++) {
         //都是相对于力控坐标系
         m_function_cart_imp_stiff_trq(i) = m_cart_stiff[i] * FC->cart_pos_following_error_tcp_in_fcframe[i];
         m_function_cart_imp_damp_trq(i) = m_cart_damp[i] * FC->cart_vel_following_error_tcp_in_fcframe[i];
         m_function_cart_imp_trq(i) = m_function_cart_imp_stiff_trq(i) + m_function_cart_imp_damp_trq(i);
     }
     //转换到基坐标系下
     m_function_cart_imp_trq_in_base_wrench = FC->fc_frame.M.Inverse() * m_function_cart_imp_trq;
     for (unsigned int i = 0; i < 6; i++) {
         m_function_cart_imp_trq_in_base(i) = m_function_cart_imp_trq_in_base_wrench(i);
     }
     function_imp_trq.data = FC->jac_trans_measure_tcp_in_base * m_function_cart_imp_trq_in_base;
     //零空间阻抗(TODO)
     m_function_null_space_trq.data = m_null_stiff[0] * (FC->cart_pos_jnt_command.data - FC->jnt_pos_measure.data) -
                                      (1.4 * std::sqrt(m_null_stiff[0])) * FC->jnt_vel_measure.data;

     //求伪逆
     J_pinv =
         FC->jac_trans_measure_flan_in_base * (FC->jac_measure_flan_in_base.data * FC->jac_trans_measure_flan_in_base).inverse();

     m_function_null_space_trq_final.data =
         (m_matrix_temp - J_pinv * FC->jac_measure_flan_in_base.data) * m_function_null_space_trq.data;
     //总阻抗力矩
     function_imp_trq.data = function_imp_trq.data + m_function_null_space_trq_final.data;

 }
 
 // --------------------- 关节保护力更新 ---------------------
 void ForcePlanner::JointLimitProtectUpdate(KDL::JntArray& protect_torque) {
     for (unsigned int i = 0; i < m_jnt_num; i++) {
         if (FC->jnt_pos_measure(i) > m_upper_monitor_bound[i]) {
             protect_torque(i) = -m_protect_force_stiff[i] *
                                     std::min(FC->jnt_pos_measure(i) - m_upper_monitor_bound[i], m_jnt_pos_safety_threshold) -
                                 m_protect_force_damp[i] * FC->jnt_vel_measure(i);
         }
         else if (FC->jnt_pos_measure(i) < m_lower_monitor_bound[i]) {
             protect_torque(i) = m_protect_force_stiff[i] *
                                     std::min(m_lower_monitor_bound[i] - FC->jnt_pos_measure(i), m_jnt_pos_safety_threshold) -
                                 m_protect_force_damp[i] * FC->jnt_vel_measure(i);
         }
         else if (FC->jnt_pos_measure(i) > (m_upper_monitor_bound[i] - m_pre_protect_angle)) {
             protect_torque(i) = 0.0;
         }
         else if (FC->jnt_pos_measure(i) < (m_lower_monitor_bound[i] + m_pre_protect_angle)) {
             protect_torque(i) = 0.0;
         }
         else {
             protect_torque(i) = 0.0;
         }
     }
 }
 
 // --------------------- 软限位设置 ---------------------
 void ForcePlanner::SetSoftLimit(const std::vector<double>& joint_range_min_input,
                                 const std::vector<double>& joint_range_max_input) {
     if (joint_range_min_input.size() != m_jnt_num || joint_range_max_input.size() != m_jnt_num) {
         return;
     }
     for (unsigned int i = 0; i < m_jnt_num; i++) {
         //弧度
         m_lower_bound[i] = joint_range_min_input[i];
         m_upper_bound[i] = joint_range_max_input[i];
         m_lower_monitor_bound[i] = m_lower_bound[i] + m_jnt_pos_safety_threshold;
         m_upper_monitor_bound[i] = m_upper_bound[i] - m_jnt_pos_safety_threshold;
     }
     return;
 }

 void ForcePlanner::UpdateParams() {
     m_fc_params_inner_ptr->m_function_params.GetParams("cart_stiff", m_cart_stiff);
     m_fc_params_inner_ptr->m_function_params.GetParams("cart_damp", m_cart_damp);
     m_fc_params_inner_ptr->m_function_params.GetParams("null_stiff", m_null_stiff);
     m_fc_params_inner_ptr->m_function_params.GetParams("joint_stiff", m_jnt_stiff);
     m_fc_params_inner_ptr->m_function_params.GetParams("joint_damp", m_jnt_damp);
 }
 }  // namespace Control
 }  // namespace RokaeApi
 