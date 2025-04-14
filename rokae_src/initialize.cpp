/**
 * Copyright(C) 2024 Rokae Technology Co., Ltd.
 * All Rights Reserved.
 *
 * Information in this file is the intellectual property of Rokae Technology
 * Co., Ltd, And may contains trade secrets that must be stored and viewed
 * confidentially.
 *
 * @file: initialize.cpp
 * @author: wangke
 * @date: 2025/3/25
 * @brief: 初始化模块
 */

#include "rokae_header/initialize.hpp"

namespace RokaeApi {

InitRobot::InitRobot(const Model::MechUnitType& robot_type)
    : m_robot_type(robot_type),
      m_robot_config(robot_type),
      m_jnt_num(m_robot_config.model_config_params.AXIS_NUM),
      m_model_param(m_jnt_num + 1),
      m_control_param(m_jnt_num),
      m_mechanical_params(m_jnt_num) {
    m_gravity = KDL::Vector(0, 0, -9.81);  //用来构建动力学解算器的重力矩
    m_chain = KDL::Chain();
}

int InitRobot::CreateModels() {
    int res = SOLVE_NOERROR;
    //机型参数转换
    res = ConfigurationToRobotParams(m_robot_config, m_model_param, m_mechanical_params, m_control_param);
    if (res != SOLVE_NOERROR) {
        return res;
    }
    //根据RD参数构建模型
    MakeChain_By_RobDim(m_model_param, m_chain);

    return SOLVE_NOERROR;
}

void InitRobot::MakeChain_By_RobDim(const Model::ModelParams& model_params_input, KDL::Chain& chain) {
    double rob_dim_x;
    double rob_dim_y;
    double rob_dim_z;
    auto dim_ptr = &(model_params_input.rob_dimensions.L01x) - 1;

    for (unsigned int i = 0; i < model_params_input.joint_type.size(); i++) {
        rob_dim_x = *(++dim_ptr);
        rob_dim_y = *(++dim_ptr);
        rob_dim_z = *(++dim_ptr);
        KDL::Rotation rot_orien;
        switch (model_params_input.coor_orient[i].rot_axis) {
        case Model::ModelParams::ROT_X:
            rot_orien.DoRotX(model_params_input.coor_orient[i].rot_angle);
            break;
        case Model::ModelParams::ROT_Y:
            rot_orien.DoRotY(model_params_input.coor_orient[i].rot_angle);
            break;
        case Model::ModelParams::ROT_Z:
            rot_orien.DoRotZ(model_params_input.coor_orient[i].rot_angle);
            break;
        default:
            break;
        }
        //纯连杆求解器
        KDL::Vector cog_in_tip = rot_orien.Inverse() * (KDL::Vector(model_params_input.link_inertia[i].centroid[0],
                                                                    model_params_input.link_inertia[i].centroid[1],
                                                                    model_params_input.link_inertia[i].centroid[2]) -
                                                        KDL::Vector(rob_dim_x, rob_dim_y, rob_dim_z));
        chain.addSegment(KDL::Segment(
            "Segment " + std::to_string(i), KDL::Joint("Joint " + std::to_string(i), model_params_input.joint_type[i]),
            KDL::Frame(rot_orien, KDL::Vector(rob_dim_x, rob_dim_y, rob_dim_z)),
            KDL::RigidBodyInertia(
                model_params_input.link_inertia[i].mass, cog_in_tip,
                KDL::RotationalInertia(
                    model_params_input.link_inertia[i].moment_link[0], model_params_input.link_inertia[i].moment_link[1],
                    model_params_input.link_inertia[i].moment_link[2], model_params_input.link_inertia[i].moment_link[3],
                    model_params_input.link_inertia[i].moment_link[4], model_params_input.link_inertia[i].moment_link[5]))));
    }
}
}  // namespace RokaeApi
