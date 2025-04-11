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
    /**
     * @brief 构造函数
     *
     * @param[in] init_robot_ptr     初始化机器人指针
     * @param[in] fc_status_ptr      力控状态数据指针
     * @param[in] fc_params_inner    力控参数配置指针
     */
    FcStatusTracker(InitRobot* init_robot_ptr, FcStatusInner* fc_status_ptr, FcParamsInner* fc_params_inner);

    /**
     * @brief 析构函数
     */
    ~FcStatusTracker();

    /**
     * @brief 力控主数据更新流程
     *
     * @return 0 成功，非0 失败
     */
    int FcStatusUpdata();

    /**
     * @brief 关节空间数据流更新
     *
     * @return 0 成功，非0 失败
     */
    int FcStatusUpdataJoint();

    /**
     * @brief 笛卡尔空间数据流更新
     *
     * @return 0 成功，非0 失败
     */
    int FcStatusUpdataCart();

    /**
     * @brief 基础信息数据流更新
     *
     * @return 0 成功，非0 失败
     */
    int FcStatusUpdataCommon();

    /**
     * @brief 设置负载信息
     *
     * @param[in] load 负载数据
     */
    void SetLoad(const RokaeLoad& load);

    /**
     * @brief 设置重力加速度方向
     *
     * @param[in] gravity 重力向量
     */
    void SetGravity(const Vector& gravity) { m_dynamic_solver->SetGravity(gravity); }

    /**
     * @brief 设置工具在法兰上的相对位姿
     *
     * @param[in] frame_tool_in_flan 工具相对法兰位姿
     */
    void SetFrameToolInFlan(const KDL::Frame& frame_tool_in_flan) { m_tool_in_flan = frame_tool_in_flan; }

    /**
     * @brief 设置基坐标系
     *
     * @param[in] base_frame 基坐标系
     */
    void SetBaseFrame(const KDL::Frame& base_frame);

    /**
     * @brief 设置力控坐标系类型
     *
     * @param[in] fc_frame_type 坐标系类型
     *
     * @return 0 成功，非0 失败
     */
    int SetFcFrameType(const FcFrameType& fc_frame_type);

    /**
     * @brief 获取当前力控坐标系（base_in_FC）
     *
     * @return 基坐标系下的力控坐标系
     */
    const KDL::Frame& GetBaseInFcFrame() { return m_fc_frame; }

    /**
     * @brief 重置内部计算状态
     */
    void ResetCalStatus();

    /**
     * @brief 对欧拉角数据进行解缠绕处理，避免 ±2π 跳变
     *
     * @param[in]  data_last 上一周期欧拉角
     * @param[out] data      当前周期欧拉角
     */
    void UnwarpRPY(const KDL::Vector& data_last, KDL::Vector& data);

   private:
    DynamicSolver* m_dynamic_solver;
    KDL::ChainFkSolverPos_recursive* m_fkpos_ptr;
    KDL::ChainJntToJacSolver* m_jac_solver;

    FcStatusInner* m_fc_status_info;
    FcParamsInner* m_fc_params_inner_ptr;

    double m_period;  //暂时没用到，控制周期
    unsigned int m_jnt_num;
    RokaeLoad m_load;
    std::vector<double> m_cart_stiffness;

    bool m_if_first_in;
    bool m_is_rot_angle_outof_range;
    KDL::Vector m_orient_delta_d;
    KDL::Vector m_orient_delta_d_last;

    KDL::Frame m_tool_in_flan;                    // 工具在法兰坐标系下位姿
    KDL::Frame m_fc_frame;                        // 力控坐标系（默认为工具坐标系）
    FcFrameType m_fc_frame_type;                  // 力控坐标系类型
    std::vector<KDL::Frame> m_base_frame_buffer;  // 用于双缓冲更新
    std::atomic<int> m_frame_active{0};           // 当前激活帧索引（0或1）
};

}  // namespace Control
}  // namespace RokaeApi

#endif  // ROKAE_HEADER_FC_DATA_COMPUTATION_HPP_
