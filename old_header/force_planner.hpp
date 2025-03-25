/**
 * Copyright(C) 2024 Rokae Technology Co., Ltd.
 * All Rights Reserved.
 *
 * Information in this file is the intellectual property of Rokae Technology Co., Ltd,
 * And may contains trade secrets that must be stored and viewed confidentially.
 *
 * @file: force_planner.hpp
 * @author: wangke
 * @date: 2024/6/17
 * @brief: 力控相关功能计算模块
 */
#ifndef FORCE_PLANNER_H
#define FORCE_PLANNER_H

#include "data_structure_define.hpp"
#include "initialize.hpp"

using namespace RokaeApi::Model;

namespace RokaeApi {
namespace Control {
class ForcePlanner {
   public:
    ForcePlanner(InitRobot* init_robot_ptr, FcStatusInfo* fc_status_info);

    ~ForcePlanner();

    /**
     * @brief 临时修改力控软限位(弧度)
     * @param[in] limit_lower: 软限位下限(弧度)
     * @param[in] limit_upper: 软限位上限(弧度)

     * @param return：无
     *
     */
    void ChangeLimitTemp(const std::vector<double>& limit_lower, const std::vector<double>& limit_upper);

    /**
     * @brief 软限位设置接口(弧度)
     * @param[in] joint_range_min: 软限位下限(弧度)
     * @param[in] joint_range_max: 软限位上限(弧度)

     * @param return：0:软限位设置成功
     *
     */
    int SetSoftLimit(const std::vector<double> joint_range_min, const std::vector<double> joint_range_max);

    /**
     * @brief 软限位保护力计算接口
     * @param[out] trq_comp_coef: 保护力矩补偿系数

     * @param return：软限位保护力矩
     *
     */
    const KDL::JntArray& JointLimitUpdate(std::vector<double> trq_comp_coef);

   private:
    FcStatusInfo* m_fc_status_info;

    std::vector<double> m_joint_range_min;  //软限位下限
    std::vector<double> m_joint_range_max;  //软限位上限
    std::vector<double> m_protect_force_stiffness;  //软限位保护力矩刚度
    std::vector<double> m_protect_force_damping;    //软限位保护力矩阻尼

    KDL::JntArray m_q_feedback;             //关节位置反馈
    KDL::JntArray m_q_dot_feedback;         //关节速度反馈
    KDL::JntArray m_lower_bound;
    KDL::JntArray m_upper_bound;
    KDL::JntArray m_lower_monitor_bound;
    KDL::JntArray m_upper_monitor_bound;
    KDL::JntArray m_protect_torque;
    double m_jnt_pos_safety_threshold;
    double m_pre_protect_angle;  //预保护角度，在此范围内力控拖动增益线性下降

    unsigned int m_jnt_num;
};

}  // namespace Control
}  // namespace RokaeApi

#endif
