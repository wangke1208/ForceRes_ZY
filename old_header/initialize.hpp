
#include <../3rd/kdl/chain.hpp>
#include <fstream>
#include <iostream>

#include "data_structure_define.hpp"
#include "rokae_config.hpp"

#ifndef INITIALIZE_H
#define INITIALIZE_H

using namespace KDL;
using namespace std;
using namespace RokaeApi::Model;
using namespace RokaeApi::Control;

namespace RokaeApi {

class InitRobot {
   public:
    InitRobot(const MechUnitType robot_type);
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

    // 读取对应cfg中模型参数
    /**
     * @brief 读取对应cfg中模型参数
     *
     * @param[out] model_params:初始化用到的相关参数，包括关节类型、RD参数等
     * @param[out] control_params:初始化用到的控制参数
     * @param[out] mechanical_params:初始化用到的机械参数
     * @return
     */
    int ReadParams(ModelParams& model_params, ControlParams& control_params, MechanicalParams& mechanical_params);

    // 赋值从SR4-C的cfg中读取的参数
    /**
     * @brief 赋值从CFG中读取的参数
     *
     * @param[out] model_params_output:输出初始化用到的相关参数，包括关节类型、RD参数等
     *
     * @return
     */
    int ReadModelParamsSR4(Model::ModelParams& model_params_output);

    // 赋值从SR4-C的cfg中读取的控制参数
    /**
     * @brief 赋值从CFG中读取的控制参数
     *
     * @param[out] control_params_output:输出初始化用到的控制相关参数
     *
     * @return
     */
    int ReadControlParamsSR4(ControlParams& control_params_output);

    // 赋值从SR4-C的cfg中读取的机械参数
    /**
     * @brief 赋值从CFG中读取的机械参数
     *
     * @param[out] control_params_output:输出初始化用到的机械相关参数
     *
     * @return
     */
    int ReadMechanicalParamsSR4(Model::MechanicalParams& mechaincal_params_output);

    // 赋值从SR5-C的cfg中读取的参数
    /**
     * @brief 赋值从CFG中读取的参数
     *
     * @param[out] model_params_output:输出初始化用到的相关参数，包括关节类型、RD参数等
     *
     * @return
     */
    int ReadModelParamsSR5(Model::ModelParams& model_params_output);

    // 赋值从SR5-C的cfg中读取的控制参数
    /**
     * @brief 赋值从CFG中读取的控制参数
     *
     * @param[out] control_params_output:输出初始化用到的控制相关参数
     *
     * @return
     */
    int ReadControlParamsSR5(ControlParams& control_params_output);

    // 赋值从SR5-C的cfg中读取的机械参数
    /**
     * @brief 赋值从CFG中读取的机械参数
     *
     * @param[out] control_params_output:输出初始化用到的机械相关参数
     *
     * @return
     */
    int ReadMechanicalParamsSR5(Model::MechanicalParams& mechaincal_params_output);

    // 赋值从SR3-C的cfg中读取的参数
    /**
     * @brief 赋值从CFG中读取的参数
     *
     * @param[out] model_params_output:输出初始化用到的相关参数，包括关节类型、RD参数等
     *
     * @return
     */
    int ReadModelParamsSR3(Model::ModelParams& model_params_output);

    // 赋值从SR3-C的cfg中读取的控制参数
    /**
     * @brief 赋值从CFG中读取的控制参数
     *
     * @param[out] control_params_output:输出初始化用到的控制相关参数
     *
     * @return
     */
    int ReadControlParamsSR3(ControlParams& control_params_output);

    // 赋值从SR3-C的cfg中读取的机械参数
    /**
     * @brief 赋值从CFG中读取的机械参数
     *
     * @param[out] control_params_output:输出初始化用到的机械相关参数
     *
     * @return
     */
    int ReadMechanicalParamsSR3(Model::MechanicalParams& mechaincal_params_output);

    // 通过RD参数构建模型
    /**
     * @brief 通过RD参数构建模型
     *
     * @param[in] model_params_input:初始化用到的相关参数
     * @param[out] chain:构建的chain
     *
     * @return
     */
    void MakeChain_By_RobDim(const ModelParams& model_params_input, KDL::Chain& chain);

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

    const MechanicalParams& GetMechanicalParams() { return m_mechanical_params; }
    const ControlParams& GetControlParams() { return m_control_param; }
    const ModelParams& GetModelParams() { return m_model_param; }

    const unsigned int GetJntNum() { return m_jnt_num; }

   private:
    unsigned int m_jnt_num;
    KDL::Vector m_gravity;
    Model::MechUnitType m_robot_type;
    string m_path_cfg;  // cfg存放的路径
    KDL::Chain m_chain;
    ModelParams m_model_param;
    ControlParams m_control_param;
    MechanicalParams m_mechanical_params;
};

}  // namespace RokaeApi

#endif
