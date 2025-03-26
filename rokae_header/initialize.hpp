
#include <../3rd/kdl/chain.hpp>
#include <fstream>
#include <iostream>

#include "./robot_config.hpp"
#include "data_structure_convert.hpp"

#ifndef INITIALIZE_H
#define INITIALIZE_H

using namespace KDL;
using namespace std;

namespace RokaeApi {

class InitRobot {
   public:
    InitRobot(const Model::MechUnitType robot_type);
    ~InitRobot(){};

    // 创建模型
    /**
     * @brief 创建机器人模型
     *
     * @param[out] model_param:初始化用到的相关参数，包括关节类型、RD参数等
     * @param[out] control_param:初始化用到的控制参数
     *
     * @return
     */
    int CreateModels();

    // 通过RD参数构建模型
    /**
     * @brief 通过RD参数构建模型
     *
     * @param[in] model_params_input:初始化用到的相关参数
     * @param[out] chain:构建的chain
     *
     * @return
     */
    void MakeChain_By_RobDim(const Model::ModelParams& model_params_input, KDL::Chain& chain);

    // 获取重力矢量
    /**
     * @brief 获取重力矢量
     *
     * @param[out] m_gravity:重力矢量
     *
     * @return
     */
    const KDL::Vector& GetGravity() { return m_gravity; }

    const KDL::Chain& GetChain() { return m_chain; }

    const Model::RobotConfiguration& GetRobotConfiguration() { return m_robot_config; }

    const unsigned int& GetJntNum() { return m_jnt_num; }

   private:
    Model::MechUnitType m_robot_type;
    Model::RobotConfiguration m_robot_config;     //从cfg读取的数据
    unsigned int m_jnt_num;

    Model::ModelParams m_model_param;             //转换后的模型数据
    Control::ControlParams m_control_param;       //转换后的控制数据
    Model::MechanicalParams m_mechanical_params;  //转换后的机械数据
    KDL::Chain m_chain;
    KDL::Vector m_gravity;

};

}  // namespace RokaeApi

#endif
