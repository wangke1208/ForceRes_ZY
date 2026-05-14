/**
 * Copyright(C) 2024 Rokae Technology Co., Ltd.
 * All Rights Reserved.
 *
 * Information in this file is the intellectual property of Rokae Technology
 * Co., Ltd, And may contains trade secrets that must be stored and viewed
 * confidentially.
 *
 * @file: robot_config.cpp
 * @author: wangke
 * @date:
 * @brief: 机型文件配置参数
 */

#include "rokae_header/robot_config.hpp"

namespace RokaeApi {
namespace Model {

ControlConfigParams::ControlConfigParams(unsigned int jnt_num)
    : CTRL_BANDWIDTH_SERVO_EXEC(jnt_num, 0.0),
      CTRL_ZETA_SERVO_EXEC(jnt_num, 0.0),
      FRICTION_COF_DRAG(jnt_num, 0.0),
      ROTATION_STIFFNESS_OF_TRANSLATION_DRAGGING(DEFAULT_CART_GAIN_SIZE, 0.0),
      ROTATION_DAMPING_OF_TRANSLATION_DRAGGING(DEFAULT_CART_GAIN_SIZE, 0.0),
      TRANSLATION_STIFFNESS_OF_ROTATION_DRAGGING(DEFAULT_CART_GAIN_SIZE, 0.0),
      TRANSLATION_DAMPING_OF_ROTATION_DRAGGING(DEFAULT_CART_GAIN_SIZE, 0.0),
      FRICTION_COF_IMPED(jnt_num, 0.0),
      CTRL_BANDWIDTH_SERVO_EXEC_IMPE(jnt_num, 0.0),
      JOINT_IMPEDANCE_CTRL_DAMP_ZETA(jnt_num, 0.0),
      CART_IMPEDANCE_CTRL_DAMP_ZETA(DEFAULT_CART_PARAMS_SIZE, 0.0){}

void ControlConfigParams::Resize(unsigned int jnt_num) {
    CTRL_BANDWIDTH_SERVO_EXEC.resize(jnt_num, 0.0);
    CTRL_ZETA_SERVO_EXEC.resize(jnt_num, 0.0);
    FRICTION_COF_DRAG.resize(jnt_num, 0.0);
    ROTATION_STIFFNESS_OF_TRANSLATION_DRAGGING.resize(DEFAULT_CART_GAIN_SIZE, 0.0);
    ROTATION_DAMPING_OF_TRANSLATION_DRAGGING.resize(DEFAULT_CART_GAIN_SIZE, 0.0);
    TRANSLATION_STIFFNESS_OF_ROTATION_DRAGGING.resize(DEFAULT_CART_GAIN_SIZE, 0.0);
    TRANSLATION_DAMPING_OF_ROTATION_DRAGGING.resize(DEFAULT_CART_GAIN_SIZE, 0.0);
    FRICTION_COF_IMPED.resize(jnt_num, 0.0);
    CTRL_BANDWIDTH_SERVO_EXEC_IMPE.resize(jnt_num, 0.0);
    JOINT_IMPEDANCE_CTRL_DAMP_ZETA.resize(jnt_num, 0.0);
    CART_IMPEDANCE_CTRL_DAMP_ZETA.resize(DEFAULT_CART_PARAMS_SIZE, 0.0);
}

MechanicalConfigParams::MechanicalConfigParams(unsigned int jnt_num)
    : ENCODER_OFFESET(jnt_num, 0),
      ENCODER_RESOLUTION(jnt_num, 0),
      REDUCTION_RATIO_NUMERATOR(jnt_num, 0.0),
      REDUCTION_RATIO_DENOMINATOR(jnt_num, 0.0),
      SENSOR_ANALOG_TO_TORQUE_HIGH(jnt_num, 0.0),
      SENSOR_ANALOG_TO_TORQUE_LOW(jnt_num, 0.0),
      SENSOR_BIAS(jnt_num, 0.0),
      SENSOR_AMPLIFY(jnt_num, 0.0),
      RATED_TORQUE(jnt_num, 0.0) {}

void MechanicalConfigParams::Resize(unsigned int jnt_num) {
    ENCODER_OFFESET.resize(jnt_num, 0);
    ENCODER_RESOLUTION.resize(jnt_num, 0);
    REDUCTION_RATIO_NUMERATOR.resize(jnt_num, 0.0);
    REDUCTION_RATIO_DENOMINATOR.resize(jnt_num, 0.0);
    SENSOR_ANALOG_TO_TORQUE_HIGH.resize(jnt_num, 0.0);
    SENSOR_ANALOG_TO_TORQUE_LOW.resize(jnt_num, 0.0);
    SENSOR_BIAS.resize(jnt_num, 0.0);
    SENSOR_AMPLIFY.resize(jnt_num, 0.0);
    RATED_TORQUE.resize(jnt_num, 0.0);
}

ProtectConfigParams::ProtectConfigParams(unsigned int jnt_num)
    : SWITCH_THRESHOLD_OF_TORQUE_CONTROL(jnt_num, 0.0) {}

void ProtectConfigParams::Resize(unsigned int jnt_num) { SWITCH_THRESHOLD_OF_TORQUE_CONTROL.resize(jnt_num, 0.0); }

ModelConfigParams::ModelConfigParams(unsigned int jnt_num)
    : AXIS_NUM(jnt_num),
      MAX_LOAD(0.0),
      MAX_LOAD_TCP_LENGTH(0.3),
      IS_WRITST_CROSS(false),
      JOINT_TYPE(jnt_num, 0),
      ROT_AXIS(jnt_num, 0),
      ROT_ANGLE(jnt_num, 0.0),
      LINK_MASS(jnt_num, 0.0),
      LINK_CENTROID(jnt_num, 0.0),
      LINK_MOMENT_OF_INERTIA(jnt_num, 0.0),
      LINK_MOMENT_OF_INERTIA_LOW(jnt_num, 0.0),
      ROBOT_DIMENSIONS((1 + jnt_num) * SIZE_LINK_RD, 0.0),
      JOINT_RANGE_MIN_CUSTOMIZE(jnt_num, 0.0),
      JOINT_RANGE_MAX_CUSTOMIZE(jnt_num, 0.0),
      JOINT_RANGE_MIN_NEW(jnt_num, 0.0),
      JOINT_RANGE_MAX_NEW(jnt_num, 0.0) {
    ROBOT_NAME = "UNKNOWN";
}

void ModelConfigParams::Resize(unsigned int jnt_num) {
    ROBOT_NAME = "UNKNOWN";
    AXIS_NUM = jnt_num;
    MAX_LOAD = 0.0;
    MAX_LOAD_TCP_LENGTH = 0.3;
    IS_WRITST_CROSS = false;
    JOINT_TYPE.resize(jnt_num + 1, 0);
    ROT_AXIS.resize(jnt_num + 1, 0);
    ROT_ANGLE.resize(jnt_num + 1, 0.0);
    LINK_MASS.resize(jnt_num, 0.0);
    LINK_CENTROID.resize(SIZE_LINK_CENTROID * (jnt_num + 1), 0.0);
    LINK_MOMENT_OF_INERTIA.resize(SIZE_LINK_INERITA * (jnt_num + 1), 0.0);
    LINK_MOMENT_OF_INERTIA_LOW.resize(SIZE_LINK_INERITA * (jnt_num + 1), 0.0);
    ROBOT_DIMENSIONS.resize((1 + jnt_num) * SIZE_LINK_RD, 0.0);
    JOINT_RANGE_MIN_CUSTOMIZE.resize(jnt_num, 0.0);
    JOINT_RANGE_MAX_CUSTOMIZE.resize(jnt_num, 0.0);
    JOINT_RANGE_MIN_NEW.resize(jnt_num, 0.0);
    JOINT_RANGE_MAX_NEW.resize(jnt_num, 0.0);
}

RobotConfiguration::RobotConfiguration(unsigned int jnt_num)
    : control_config_params(jnt_num),
      mechanical_config_params(jnt_num),
      protect_config_params(jnt_num),
      model_config_params(jnt_num) {}

void RobotConfiguration::Resize(unsigned int jnt_num) {
    control_config_params.Resize(jnt_num);
    mechanical_config_params.Resize(jnt_num);
    protect_config_params.Resize(jnt_num);
    model_config_params.Resize(jnt_num);
}
}  // namespace Model
}  // namespace RokaeApi