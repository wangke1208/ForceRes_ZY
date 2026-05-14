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

#include <stdexcept>

#include "rokae_header/robot_cfg_loader.hpp"

namespace RokaeApi {

namespace {

KDL::Rotation MakeBaseRotationFromXyzDeg(const std::array<double, 3>& base_rotation_xyz_deg) {
    return KDL::Rotation::RotZ(base_rotation_xyz_deg[2] * DEG_TO_RAD) *
           KDL::Rotation::RotY(base_rotation_xyz_deg[1] * DEG_TO_RAD) *
           KDL::Rotation::RotX(base_rotation_xyz_deg[0] * DEG_TO_RAD);
}

}  // namespace

InitRobot::InitRobot(Model::RobotConfiguration&& robot_configuration, const Model::MechUnitType& robot_type,
                     const std::array<double, 3>& base_rotation_xyz_deg)
    : m_robot_type(robot_type),
      m_robot_config(std::move(robot_configuration)),
      m_jnt_num(m_robot_config.model_config_params.AXIS_NUM),
      m_model_param(m_jnt_num + 1),
      m_control_param(m_jnt_num),
      m_mechanical_params(m_jnt_num),
      m_chain(),
      m_base_R_world_from_base(MakeBaseRotationFromXyzDeg(base_rotation_xyz_deg)),
      m_gravity() {
    m_gravity = m_base_R_world_from_base.Inverse() * KDL::Vector(0.0, 0.0, -9.81);
}

InitRobot::InitRobot(const std::string& model_name)
    : InitRobot(model_name, std::array<double, 3>{0.0, 0.0, 0.0}) {}

InitRobot::InitRobot(const std::string& model_name, const std::array<double, 3>& base_rotation_xyz_deg)
    : m_robot_type(Model::MechUnitType::AR5C_L),
      m_robot_config(DEFAULT_AXIS),
      m_jnt_num(DEFAULT_AXIS),
      m_model_param(DEFAULT_AXIS + 1),
      m_control_param(DEFAULT_AXIS),
      m_mechanical_params(DEFAULT_AXIS),
      m_chain(),
      m_base_R_world_from_base(MakeBaseRotationFromXyzDeg(base_rotation_xyz_deg)),
      m_gravity() {
    Model::MechUnitType mt{};
    int e = ParseMechUnitFromModelName(model_name, mt);
    if (e != SOLVE_NOERROR) {
        throw std::runtime_error("ParseMechUnitFromModelName failed");
    }
    std::string json;
    e = ResolveRobotCfgJson(model_name, json);
    if (e != SOLVE_NOERROR) {
        throw std::runtime_error("ResolveRobotCfgJson failed");
    }
    Model::RobotConfiguration cfg(DEFAULT_AXIS);
    e = LoadRobotConfigurationFromJsonString(json, cfg);
    if (e != SOLVE_NOERROR) {
        throw std::runtime_error("LoadRobotConfigurationFromJsonString failed");
    }
    m_robot_type = mt;
    m_robot_config = std::move(cfg);
    m_jnt_num = m_robot_config.model_config_params.AXIS_NUM;
    m_model_param = Model::ModelParams(m_jnt_num + 1);
    m_control_param = Control::ControlParams(m_jnt_num);
    m_mechanical_params = Model::MechanicalParams(m_jnt_num);
    m_base_R_world_from_base = MakeBaseRotationFromXyzDeg(base_rotation_xyz_deg);
    m_gravity = m_base_R_world_from_base.Inverse() * KDL::Vector(0.0, 0.0, -9.81);
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
