/**
 * Copyright(C) 2024 Rokae Technology Co., Ltd.
 * All Rights Reserved.
 *
 * Information in this file is the intellectual property of Rokae Technology Co., Ltd,
 * And may contains trade secrets that must be stored and viewed confidentially.
 *
 * @file: initialize.hpp
 * @author: wangke
 * @date: 2025/3/29
 * @brief: 机器人初始化模块，负责创建机器人模型及配置
 */

#ifndef ROKAE_HEADER_INITIALIZE_H
#define ROKAE_HEADER_INITIALIZE_H

#include "3rd/kdl/chain.hpp"
#include "rokae_header/data_structure_convert.hpp"
#include "rokae_header/robot_config.hpp"


namespace RokaeApi {

/**
 * @class InitRobot
 * @brief 机器人初始化类，负责根据机器人类型和配置初始化模型和相关参数
 */
class InitRobot {
   public:
    /**
     * @brief 构造函数，初始化机器人类型
     * @param robot_type 机器人类型
     */
    explicit InitRobot(const Model::MechUnitType& robot_type);

    /**
     * @brief 析构函数
     */
    ~InitRobot() = default;

    /**
     * @brief 创建机器人模型
     * @param[out] model_param 初始化用到的相关模型参数，包括关节类型、RD参数等
     * @param[out] control_param 初始化用到的控制参数
     * @return 返回错误码
     */
    int CreateModels();

    /**
     * @brief 根据RD参数构建机器人链
     * @param[in] model_params_input 初始化用到的相关参数
     * @param[out] chain 构建的机器人链
     */
    void MakeChain_By_RobDim(const Model::ModelParams& model_params_input, KDL::Chain& chain);

    /**
     * @brief 获取重力矢量
     * @return 返回当前重力矢量
     */
    const KDL::Vector& GetGravity() const { return m_gravity; }

    /**
     * @brief 获取机器人链
     * @return 返回机器人链
     */
    const KDL::Chain& GetChain() const { return m_chain; }

    /**
     * @brief 获取机器人配置
     * @return 返回机器人配置
     */
    const Model::RobotConfiguration& GetRobotConfiguration() const { return m_robot_config; }

    /**
     * @brief 获取关节数量
     * @return 返回关节数量
     */
    unsigned int GetJntNum() const { return m_jnt_num; }

    /**
     * @brief 获取模型参数
     * @return 返回模型参数
     */
    const Model::ModelParams& GetModelParams() const { return m_model_param; }

    /**
     * @brief 获取控制参数
     * @return 返回控制参数
     */
    const Control::ControlParams& GetControlParams() const { return m_control_param; }

    /**
     * @brief 获取机械参数
     * @return 返回机械参数
     */
    const Model::MechanicalParams& GetMechanicalParams() const { return m_mechanical_params; }

    /**
     * @brief 设置重力加速度方向
     * @param gravity_in 重力加速度方向
     */
    void SetGravity(const KDL::Vector& gravity_in) { m_gravity = gravity_in; }

   private:
    Model::MechUnitType m_robot_type;             ///< 机器人类型
    Model::RobotConfiguration m_robot_config;     ///< 从配置文件读取的机器人配置
    unsigned int m_jnt_num;                       ///< 关节数量
    Model::ModelParams m_model_param;             ///< 机器人模型参数
    Control::ControlParams m_control_param;       ///< 控制参数
    Model::MechanicalParams m_mechanical_params;  ///< 机械参数
    KDL::Chain m_chain;                           ///< 机器人链
    KDL::Vector m_gravity;                        ///< 重力矢量
};

}  // namespace RokaeApi

#endif
