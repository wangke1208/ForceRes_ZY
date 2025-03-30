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
ForcePlanner::ForcePlanner(InitRobot* init_robot_ptr, Control::FcStatusInner* fc_status_ptr,
                           Control::FcParamsInner* fc_params_inner_ptr)
    : m_init_robot_ptr(init_robot_ptr), m_fc_status_ptr(fc_status_ptr), m_fc_params_inner_ptr(fc_params_inner_ptr) {
    // 初始化成员变量或执行其他必要的操作
    m_jnt_num = m_init_robot_ptr->GetJntNum();
    //初始化
    m_jnt_stiff.resize(m_jnt_num, 0.0);
    m_jnt_damp.resize(m_jnt_num, 0.0);
    m_function_jnt_imp_trq.resize(m_jnt_num);  //
    m_function_jnt_imp_stiff_trq.resize(m_jnt_num);  //
    m_function_jnt_imp_damp_trq.resize(m_jnt_num);  //
    m_function_jnt_zero_trq.resize(m_jnt_num);
    m_function_jnt_zero_trq.data.setZero(); 
    m_function_imp_trq.resize(m_jnt_num);  //
}

const KDL::JntArray ForcePlanner::ForcePlannerUpdata(){
     //1.阻抗力
    if(FC->drag_type == Control::DragType::DRAG_JOINT){
        m_function_imp_trq = JointImpedanceUpdate();
    }else{
        m_function_imp_trq = CartImpedanceUpdate();
    }
     //2.关节保护力
     m_function_jnt_limit_trq = JointLimitProtectUpdate();
     //3.动力学补偿
     m_function_jnt_gravity = FC->jnt_gravity_trq_measure;
     //4.合力
     for(unsigned int i = 0; i < m_jnt_num; i++){
        m_function_jnt_trq_all(i) = m_function_jnt_gravity(i) + m_function_imp_trq(i) + m_function_jnt_limit_trq(i);
     }
     return m_function_jnt_trq_all;
}

const KDL::JntArray ForcePlanner::JointImpedanceUpdate(){
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
    return m_function_jnt_zero_trq;
}

const KDL::JntArray ForcePlanner::CartImpedanceUpdate(){

    return m_function_jnt_zero_trq;

}

const KDL::JntArray ForcePlanner::JointLimitProtectUpdate(){

    return m_function_jnt_zero_trq;

}
}