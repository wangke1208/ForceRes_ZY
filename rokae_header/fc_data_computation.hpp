/**
 * Copyright(C) 2024 Rokae Technology Co., Ltd.
 * All Rights Reserved.
 *
 * Information in this file is the intellectual property of Rokae Technology Co., Ltd,
 * And may contains trade secrets that must be stored and viewed confidentially.
 *
 * @file: fc_data_computation.hpp
 * @author: wangke
 * @date: 2025/3/27
 * @brief: 力控数据流计算
 */
#ifndef ROKAE_HEADER_FC_DATA_COMPUTATION_HPP_
#define ROKAE_HEADER_FC_DATA_COMPUTATION_HPP_

#include "data_structure_define.hpp"
#include "dynamic_solver.hpp"
#include "initialize.hpp"

using namespace RokaeApi::Model;

namespace RokaeApi {
namespace Control {
class FcStatusTracker {
   public:
    FcStatusTracker(InitRobot* init_robot_ptr, FcStatusInner* fc_status_ptr);

    ~FcStatusTracker();

    /**
     * @brief 力控模块数据流计算
     *
     * @return 无
     */
    int FcStatusUpdata();

    /**
     * @brief 设置负载
     *
     * @param[in] load:负载信息
     * @return 无
     */
    void SetLoad(const LoadInertia& load);
    //对旋转数据进行解缠绕处理 ，避免角度在正负 2π 附近出现跳跃不连续的情况
    void UnwarpRPY(const KDL::Vector& data_last, KDL::Vector& data);


   private:
    DynamicSolver* m_dynamic_solver;
    FcStatusInner* m_fc_status_info;
    KDL::ChainFkSolverPos_recursive* m_fkpos_ptr;

    double m_period;
    //一些变量
    unsigned int m_jnt_num;
    LoadInertia m_load;
    KDL::Frame m_tool_in_flan;
    bool m_if_first_in;
    bool m_is_rot_angle_outof_range;
    KDL::Vector m_orient_delta_d;
    KDL::Vector m_orient_delta_d_last;
};

}  // namespace Control
}  // namespace RokaeApi

#endif
