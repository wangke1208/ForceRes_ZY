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
namespace Control {

class ForcePlanner {
   public:
    ForcePlanner();
    ~ForcePlanner();
    ForcePlanner(InitRobot* init_robot_ptr, Control::FcStatusInner* fc_status_ptr, Control::FcParamsInner* fc_params_inner_ptr);
    const KDL::JntArray ForcePlannerUpdata();
    void ForcePlanner::JointImpedanceUpdate(KDL::JntArray& function_imp_trq);
    void ForcePlanner::CartImpedanceUpdate(KDL::JntArray& function_imp_trq);
    void ForcePlanner::JointLimitProtectUpdate(KDL::JntArray& protect_torque);
    void ForcePlanner::SetSoftLimit(const std::vector<double>& joint_range_min_input,
                                    const std::vector<double>& joint_range_max_input);

   private:
    unsigned int m_jnt_num;
    InitRobot* m_init_robot_ptr;
    Control::FcStatusInner* m_fc_status_ptr;
    Control::FcParamsInner* m_fc_params_inner_ptr;

    //控制参数
    std::vector<double> m_jnt_stiff;
    std::vector<double> m_jnt_damp;
    std::vector<double> m_cart_stiff;
    std::vector<double> m_cart_damp;
    //关节限位
    std::vector<double> m_lower_bound;
    std::vector<double> m_upper_bound;
    std::vector<double> m_lower_monitor_bound;
    std::vector<double> m_upper_monitor_bound;
    std::vector<double> m_protect_force_damp;  //这两个参数基本不允许更改，所以在初始化的时候进行赋值
    std::vector<double> m_protect_force_stiff;
    double m_pre_protect_angle;  //预保护角度
    double m_jnt_pos_safety_threshold;

    //功能力相关变量
    KDL::JntArray m_function_trq_ref;            //合力
    KDL::JntArray m_function_jnt_gravity;        //重力
    KDL::JntArray m_function_imp_trq;            //阻抗力
    KDL::JntArray m_function_jnt_imp_trq;        //关节阻抗力
    KDL::JntArray m_function_jnt_imp_stiff_trq;  //关节阻抗力
    KDL::JntArray m_function_jnt_imp_damp_trq;   //关节阻抗力

    KDL::JntArray m_function_cart_imp_trq;        //笛卡尔阻抗力
    KDL::JntArray m_function_cart_imp_stiff_trq;  //笛卡尔阻抗力
    KDL::JntArray m_function_cart_imp_damp_trq;   //笛卡尔阻抗力

    KDL::JntArray m_function_jnt_limit_trq;  //关节限位力

    KDL::JntArray m_function_jnt_zero_trq;  //空数据
};

}  // namespace Control
}  // namespace RokaeApi
#endif
