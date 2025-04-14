/**
 * Copyright(C) 2024 Rokae Technology Co., Ltd.
 * All Rights Reserved.
 *
 * Information in this file is the intellectual property of Rokae Technology Co., Ltd,
 * And may contains trade secrets that must be stored and viewed confidentially.
 *
 * @file: data_structure_convert.hpp
 * @author: wangke
 * @date:
 * @brief: 数据类型转化相关的一些工具函数
 */

#ifndef ROKAE_HEADER_DATA_STRUCTURE_CONVERT_H_
#define ROKAE_HEADER_DATA_STRUCTURE_CONVERT_H_

#include <3rd/kdl/jntarray.hpp>
#include <3rd/kdl/rigidbodyinertia.hpp>
#include <algorithm>
#include <iostream>
#include <vector>

#include "rokae_header/robot_config.hpp"


namespace RokaeApi {
// 定义角度转换因子
constexpr double DEG_TO_RAD = PI / 180.0;
constexpr double RAD_TO_DEG = 180.0 / PI;
inline KDL::RigidBodyInertia GetKDLloadFromRokaeLoad(const Model::RokaeLoadInertia& in) {
    return KDL::RigidBodyInertia(in.mass, in.GetCOG(),
                                 KDL::RotationalInertia(in.m_inertia[0], in.m_inertia[1], in.m_inertia[2], in.m_inertia[3],
                                                        in.m_inertia[4], in.m_inertia[5]));
}

inline int VectorToJntArray(const std::vector<double>& in, KDL::JntArray& out) {
    if (in.size() != out.rows()) return -1;
    for (unsigned int i = 0; i < in.size(); i++) {
        out(i) = in[i];
    }
    return 0;
}

inline KDL::JntArray& VectorToJntArray(const std::vector<double>& in) {
    KDL::JntArray out(in.size());
    for (unsigned int i = 0; i < in.size(); i++) {
        out(i) = in[i];
    }
    return out;
}

inline int JntArrayToVector(const KDL::JntArray& in, std::vector<double>& out) {
    if (out.size() != in.rows()) return -1;
    for (unsigned int i = 0; i < in.rows(); i++) {
        out[i] = in(i);
    }
    return 0;
}

inline std::vector<double>& JntArrayToVector(const KDL::JntArray& in) {
    std::vector<double> out(in.rows());
    for (unsigned int i = 0; i < in.rows(); i++) {
        out[i] = in(i);
    }
    return out;
}

// 向量与Wrench转换函数
inline void FCVectorXdToWrench(const Eigen::VectorXd& in, KDL::Wrench& out) {
    for (unsigned int i = 0; i < 3; i++) {
        out.force(i) = in[i];
        out.torque(i) = in[i + 3];
    }
}

// 数组与KDL::Frame转换函数
inline int ArrayToKdlFrame(const std::array<double, 6>& in, KDL::Frame& out) {
    //对欧拉角进行限制
    if (in[3] > 180 || in[3] < -180 || in[4] > 180 || in[4] < -180 || in[5] > 180 || in[5] < -180) {
        return ERROR_EULER_PARAMS;
    }
    KDL::Vector pos_temp(in[0], in[1], in[2]);
    out.p = pos_temp;
    out.M = KDL::Rotation::RPY(in[3] * DEG_TO_RAD, in[4] * DEG_TO_RAD, in[5] * DEG_TO_RAD);
    return SOLVE_NOERROR;
}

// 向量与RobDimensions转换函数
inline void VectorToRD(const std::vector<double>& in, Model::ModelParams::RobDimensions& out) {
    int joint_num = in.size() / 3;
    std::array<double*, 24> out_pointers = {&out.L01x, &out.L01y, &out.L01z, &out.L12x, &out.L12y, &out.L12z,
                                            &out.L23x, &out.L23y, &out.L23z, &out.L34x, &out.L34y, &out.L34z,
                                            &out.L45x, &out.L45y, &out.L45z, &out.L56x, &out.L56y, &out.L56z,
                                            &out.L67x, &out.L67y, &out.L67z, &out.L78x, &out.L78y, &out.L78z};
    // Note:目前固定RD参数长度为24，也就是最大只支持7轴机器人
    for (int i = 0; i < std::min(joint_num * 3, static_cast<int>(out_pointers.size())); ++i) {
        *out_pointers[i] = in[i] / 1000.0;
    }
}

// 配置参数转换函数
inline int ConfigurationToRobotParams(const Model::RobotConfiguration& in, Model::ModelParams& model_out,
                                      Model::MechanicalParams& mec_out, Control::ControlParams& control_out) {
    //判断参数是否可转换(TODO:先简单判断下，后续再优化)
    if (in.model_config_params.AXIS_NUM != model_out.axis_num ||
        in.mechanical_config_params.ENCODER_OFFESET.size() != mec_out.encoder_offset.size() ||
        in.control_config_params.CTRL_BANDWIDTH_SERVO_EXEC.size() != control_out.m_gain_params.joint_gain_kp.size())
        return ERROR_SIZE_WRONG;

    // 1.0模型参数
    for (unsigned int i = 0; i < model_out.axis_num + 1; i++) {
        model_out.joint_type[i] = static_cast<KDL::Joint::JointType>(in.model_config_params.JOINT_TYPE[i]);
        model_out.coor_orient[i].rot_axis = static_cast<Model::ModelParams::Rot_Axis>(in.model_config_params.ROT_AXIS[i]);
        model_out.coor_orient[i].rot_angle = in.model_config_params.ROT_ANGLE[i];
    }
    // 1.1惯量参数
    model_out.link_inertia.at(0).mass = 0.0;
    model_out.link_inertia.at(0).centroid.assign(3, 0.0);
    model_out.link_inertia.at(0).moment.assign(6, 0.0);
    model_out.link_inertia.at(0).moment_link.assign(6, 0.0);
    for (unsigned int i = 1; i < model_out.axis_num + 1; i++) {
        unsigned int t = i - 1;
        model_out.link_inertia[i].mass = in.model_config_params.LINK_MASS[t];
        for (unsigned int j = 0; j < 3; j++) {
            model_out.link_inertia[i].centroid[j] = in.model_config_params.LINK_CENTROID[3 * t + j];
        }
        for (unsigned int k = 0; k < 6; k++) {
            model_out.link_inertia[i].moment[k] = in.model_config_params.LINK_MOMENT_OF_INERTIA[3 * i + k];
            model_out.link_inertia[i].moment_link[k] = in.model_config_params.LINK_MOMENT_OF_INERTIA_LOW[3 * i + k];
        }
    }
    // 1.2 RD参数
    VectorToRD(in.model_config_params.ROBOT_DIMENSIONS, model_out.rob_dimensions);

    // 1.3 其他模型参数
    for (unsigned int i = 0; i < model_out.axis_num; i++) {
        model_out.joint_range_min[i] = in.model_config_params.JOINT_RANGE_MIN_CUSTOMIZE[i];
        model_out.joint_range_max[i] = in.model_config_params.JOINT_RANGE_MAX_CUSTOMIZE[i];
        model_out.joint_range_min_new[i] = in.model_config_params.JOINT_RANGE_MIN_NEW[i];
        model_out.joint_range_max_new[i] = in.model_config_params.JOINT_RANGE_MAX_NEW[i];
    }
    model_out.axis_num = in.model_config_params.AXIS_NUM;
    model_out.max_load = in.model_config_params.MAX_LOAD;
    model_out.max_load_tcp_length = in.model_config_params.MAX_LOAD_TCP_LENGTH;

    // 2.机械参数
    for (unsigned int i = 0; i < mec_out.encoder_offset.size(); i++) {
        mec_out.encoder_offset[i] = in.mechanical_config_params.ENCODER_OFFESET[i];
        mec_out.encoder_resolution[i] = in.mechanical_config_params.ENCODER_RESOLUTION[i];
        mec_out.decel_ratio_high[i] = in.mechanical_config_params.REDUCTION_RATIO_NUMERATOR[i];
        mec_out.decel_ratio_low[i] = in.mechanical_config_params.REDUCTION_RATIO_DENOMINATOR[i];
        mec_out.analog2trq_high[i] = in.mechanical_config_params.SENSOR_ANALOG_TO_TORQUE_HIGH[i];
        mec_out.analog2trq_low[i] = in.mechanical_config_params.SENSOR_ANALOG_TO_TORQUE_LOW[i];
        mec_out.analog_bias[i] = in.mechanical_config_params.SENSOR_BIAS[i];
        mec_out.sensor_amplify[i] = in.mechanical_config_params.SENSOR_AMPLIFY[i];
        mec_out.rated_torque[i] = in.mechanical_config_params.RATED_TORQUE[i];
    }

    // 3.控制参数&保护参数
    for (unsigned int i = 0; i < control_out.m_gain_params.joint_gain_kp.size(); i++) {
        control_out.m_gain_params.joint_gain_kp[i] = in.control_config_params.CTRL_BANDWIDTH_SERVO_EXEC[i];
        control_out.m_gain_params.joint_damp_zeta[i] = in.control_config_params.CTRL_ZETA_SERVO_EXEC[i];
        control_out.m_gain_params.friction_cof_servo[i] = in.control_config_params.FRICTION_COF_DRAG[i];
        control_out.m_protect_params.max_mode_switch_trq[i] = in.protect_config_params.SWITCH_THRESHOLD_OF_TORQUE_CONTROL[i];
    }
    return SOLVE_NOERROR;
}

}  // namespace RokaeApi
#endif