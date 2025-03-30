/**
 * Copyright(C) 2024 Rokae Technology Co., Ltd.
 * All Rights Reserved.
 *
 * Information in this file is the intellectual property of Rokae Technology Co., Ltd,
 * And may contains trade secrets that must be stored and viewed confidentially.
 *
 * @file: force_planner.hpp
 * @author: wangke
 * @date: 2024/4/25
 * @brief: 力控计算模块
 */

#ifndef ROKAE_HEADER_FORCE_PLANNER_HPP_
#define ROKAE_HEADER_FORCE_PLANNER_HPP_

#include "rokae_header/fc_data_computation.hpp"

namespace RokaeApi {
class ForcePlanner {
   public:
    ForcePlanner();
    ~ForcePlanner();
    ForcePlanner(InitRobot* init_robot_ptr, Control::FcStatusInner* fc_status_ptr, Control::FcParamsInner* fc_params_inner_ptr);
    const KDL::JntArray ForcePlannerUpdata();
    const KDL::JntArray JointImpedanceUpdate();
    const KDL::JntArray ForcePlanner::CartImpedanceUpdate();
    const KDL::JntArray ForcePlanner::JointLimitProtectUpdate();

   private:
    unsigned int m_jnt_num;
    InitRobot* m_init_robot_ptr;
    Control::FcStatusInner* m_fc_status_ptr;
    Control::FcParamsInner* m_fc_params_inner_ptr;

    //控制参数
    std::vector<double> m_jnt_stiff;
    std::vector<double> m_jnt_damp;

    //功能力相关变量
    KDL::JntArray m_function_jnt_trq_all;  //合力
    KDL::JntArray m_function_jnt_gravity;  //重力
    KDL::JntArray m_function_imp_trq;  //阻抗力
    KDL::JntArray m_function_jnt_imp_trq;  //关节阻抗力
    KDL::JntArray m_function_jnt_imp_stiff_trq;  //关节阻抗力
    KDL::JntArray m_function_jnt_imp_damp_trq;  //关节阻抗力

    KDL::JntArray m_function_cart_imp_trq;  //笛卡尔阻抗力
    KDL::JntArray m_function_cart_imp_trq;  //笛卡尔阻抗力
    KDL::JntArray m_function_cart_imp_trq;  //笛卡尔阻抗力

    KDL::JntArray m_function_jnt_limit_trq;  //关节限位力

    KDL::JntArray m_function_jnt_zero_trq;  //空数据


};

}  // namespace RokaeApi

#endif
