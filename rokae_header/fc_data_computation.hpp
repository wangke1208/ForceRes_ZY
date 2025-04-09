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
#include "fc_params.hpp"
#include "initialize.hpp"

using namespace RokaeApi::Model;

namespace RokaeApi {
namespace Control {
class FcStatusTracker {
   public:
    FcStatusTracker(InitRobot* init_robot_ptr, FcStatusInner* fc_status_ptr, FcParamsInner* fc_params_inner);

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
    void SetLoad(const RokaeLoad& load);
    //对旋转数据进行解缠绕处理 ，避免角度在正负 2π 附近出现跳跃不连续的情况
    void UnwarpRPY(const KDL::Vector& data_last, KDL::Vector& data);
    int FcStatusUpdataCart();     //笛卡尔空间数据更新
    int FcStatusUpdataJoint();    //轴空间数据更新
    int FcStatusUpdataCommon();   //更新基础部分，用来外部获取

    const KDL::Frame& GetBaseInFcFrame() { return m_fc_frame; }
    int SetFcFrameType(const FcFrameType& fc_frame_type);
    void SetFrameToolInFlan(const KDL::Frame& frame_tool_in_flan) { m_tool_in_flan = frame_tool_in_flan; }
    void ResetCalStatus();
    void SetGravity(const Vector& gravity) { m_dynamic_solver->SetGravity(gravity); }
    void SetBaseFrame(const KDL::Frame& base_frame);

   private:
    DynamicSolver* m_dynamic_solver;
    FcStatusInner* m_fc_status_info;
    FcParamsInner* m_fc_params_inner_ptr;
    KDL::ChainFkSolverPos_recursive* m_fkpos_ptr;
    KDL::ChainJntToJacSolver* m_jac_solver;

    double m_period;
    //一些变量
    unsigned int m_jnt_num;
    RokaeLoad m_load;
    bool m_if_first_in;
    bool m_is_rot_angle_outof_range;
    KDL::Vector m_orient_delta_d;
    KDL::Vector m_orient_delta_d_last;
    std::vector<double> m_cart_stiffness;
    //坐标系
    KDL::Frame m_tool_in_flan;
    KDL::Frame m_fc_frame;  //力控坐标系，拖动情况下，力控坐标系均为工具坐标系
    FcFrameType m_fc_frame_type;
    //动态基坐标系更改
    std::vector<KDL::Frame> m_base_frame_buffer;
    std::atomic<int> m_frame_active{0};  // 当前激活缓冲区索引（0 或 1）
};

}  // namespace Control
}  // namespace RokaeApi

#endif
