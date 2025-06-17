/**
 * Copyright(C) 2024 Rokae Technology Co., Ltd.
 * All Rights Reserved.
 *
 * Information in this file is the intellectual property of Rokae Technology
 * Co., Ltd, And may contains trade secrets that must be stored and viewed
 * confidentially.
 *
 * @file: robot_config.hpp
 * @author: wangke
 * @date:
 * @brief: 机型文件配置参数
 */

#ifndef ROKAE_HEADER_ROBOT_CONFIG_HPP
#define ROKAE_HEADER_ROBOT_CONFIG_HPP

#include <string>
#include <vector>

#include "rokae_header/data_structure_define.hpp"


namespace RokaeApi {
namespace Model {
// 常量定义
const unsigned int SIX_AXIS_ROBOT = 6;
const unsigned int SEVEN_AXIS_ROBOT = 7;
const unsigned int DEFAULT_CART_PARAMS_SIZE = 6;
const unsigned int DEFAULT_CART_GAIN_SIZE = 3;
const unsigned int SIZE_LINK_INERITA = 6;
const unsigned int SIZE_LINK_CENTROID = 3;
const unsigned int SIZE_LINK_RD = 3;
const unsigned int DEFAULT_SENSOR_FIX_SIZE = 9;
struct ControlConfigParams {
    std::vector<double> CTRL_BANDWIDTH_SERVO_EXEC;  // 拖动带宽
    std::vector<double> CTRL_ZETA_SERVO_EXEC;       // 拖动阻尼比
    std::vector<double> FRICTION_COF_DRAG;          // 拖动摩擦力系数
    std::vector<double> ROTATION_STIFFNESS_OF_TRANSLATION_DRAGGING;
    std::vector<double> ROTATION_DAMPING_OF_TRANSLATION_DRAGGING;
    std::vector<double> TRANSLATION_STIFFNESS_OF_ROTATION_DRAGGING;
    std::vector<double> TRANSLATION_DAMPING_OF_ROTATION_DRAGGING;
    std::vector<double> JOINT_IMPEDANCE_CTRL_DAMP_ZETA;
    std::vector<double> CART_IMPEDANCE_CTRL_DAMP_ZETA;
    std::vector<double> SENSOR_BIAS_DYNAMIC;
    std::vector<double> POS_FIX_PARMAS;
    std::vector<double> NEG_FIX_PARMAS;
    std::vector<bool> IS_SUPPORT_SENSOR_FIX;
    ControlConfigParams(unsigned int jnt_num = DEFAULT_AXIS);
    void Resize(unsigned int jnt_num);
};

struct MechanicalConfigParams {
    std::vector<int> ENCODER_OFFESET;
    std::vector<int> ENCODER_RESOLUTION;
    std::vector<double> REDUCTION_RATIO_NUMERATOR;
    std::vector<double> REDUCTION_RATIO_DENOMINATOR;
    std::vector<double> SENSOR_ANALOG_TO_TORQUE_HIGH;
    std::vector<double> SENSOR_ANALOG_TO_TORQUE_LOW;
    std::vector<double> SENSOR_BIAS;
    std::vector<double> SENSOR_AMPLIFY;
    std::vector<double> RATED_TORQUE;

    MechanicalConfigParams(unsigned int jnt_num = DEFAULT_AXIS);
    void Resize(unsigned int jnt_num);
};

struct ProtectConfigParams {
    std::vector<double> SWITCH_THRESHOLD_OF_TORQUE_CONTROL;

    ProtectConfigParams(unsigned int jnt_num = DEFAULT_AXIS);
    void Resize(unsigned int jnt_num);
};

struct ModelConfigParams {
    std::string ROBOT_NAME;
    unsigned int AXIS_NUM;
    double MAX_LOAD;
    double MAX_LOAD_TCP_LENGTH;
    std::vector<int> JOINT_TYPE;
    std::vector<int> ROT_AXIS;
    std::vector<double> ROT_ANGLE;
    std::vector<double> LINK_MASS;
    std::vector<double> LINK_CENTROID;
    std::vector<double> LINK_MOMENT_OF_INERTIA;
    std::vector<double> LINK_MOMENT_OF_INERTIA_LOW;
    std::vector<double> ROBOT_DIMENSIONS;
    std::vector<double> JOINT_RANGE_MIN_CUSTOMIZE;
    std::vector<double> JOINT_RANGE_MAX_CUSTOMIZE;
    std::vector<double> JOINT_RANGE_MIN_NEW;
    std::vector<double> JOINT_RANGE_MAX_NEW;

    ModelConfigParams(unsigned int jnt_num = DEFAULT_AXIS);
    void Resize(unsigned int jnt_num);
};

struct RobotConfiguration {
    ControlConfigParams control_config_params;
    MechanicalConfigParams mechanical_config_params;
    ProtectConfigParams protect_config_params;
    ModelConfigParams model_config_params;

    RobotConfiguration(unsigned int jnt_num = DEFAULT_AXIS);
    RobotConfiguration(MechUnitType robot_type);
    void Resize(unsigned int jnt_num);
};
}  // namespace Model
}  // namespace RokaeApi

#endif  // ROBOT_CONFIG_HPP