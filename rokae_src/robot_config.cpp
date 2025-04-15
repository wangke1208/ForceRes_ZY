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
    : CTRL_BANDWIDTH_SERVO_EXEC(jnt_num, 0.0), CTRL_ZETA_SERVO_EXEC(jnt_num, 0.0), FRICTION_COF_DRAG(jnt_num, 0.0) {}

void ControlConfigParams::Resize(unsigned int jnt_num) {
    CTRL_BANDWIDTH_SERVO_EXEC.resize(jnt_num, 0.0);
    CTRL_ZETA_SERVO_EXEC.resize(jnt_num, 0.0);
    FRICTION_COF_DRAG.resize(jnt_num, 0.0);
    ROTATION_STIFFNESS_OF_TRANSLATION_DRAGGING.resize(DEFAULT_CART_PARAMS_SIZE, 0.0);
    ROTATION_DAMPING_OF_TRANSLATION_DRAGGING.resize(DEFAULT_CART_PARAMS_SIZE, 0.0);
    TRANSLATION_STIFFNESS_OF_ROTATION_DRAGGING.resize(DEFAULT_CART_PARAMS_SIZE, 0.0);
    TRANSLATION_DAMPING_OF_ROTATION_DRAGGING.resize(DEFAULT_CART_PARAMS_SIZE, 0.0);
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

RobotConfiguration::RobotConfiguration(MechUnitType robot_type) {
    switch (robot_type) {
    case MechUnitType::DEFALUT_SENVEN_AXIS:
        this->Resize(SEVEN_AXIS_ROBOT);
        // ModelConfigParams
        model_config_params.ROBOT_NAME = "xMatePro7";
        model_config_params.AXIS_NUM = 7;
        model_config_params.MAX_LOAD = 7;
        model_config_params.MAX_LOAD_TCP_LENGTH = 0.3;

        model_config_params.JOINT_TYPE = {8, 3, 2, 3, 2, 3, 2, 3};
        model_config_params.ROT_AXIS = {0, 0, 0, 0, 0, 0, 0};
        model_config_params.ROT_ANGLE = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};

        model_config_params.LINK_MASS = {5.179, 4.566, 3.163, 2.990, 2.501, 2.192, 0.961};

        model_config_params.LINK_CENTROID = {-0.042,  -13.288, 327.892, -0.020,  27.853, 103.134, -0.333,
                                             -11.578, -63.455, -0.278,  19.667,  88.966, 0.178,   -12.921,
                                             -57.504, -0.062,  13.079,  114.262, 0.574,  16.762,  -51.921};

        model_config_params.LINK_MOMENT_OF_INERTIA = {
            53972.936, 51289.371,   8130842.419, -37.314,   37.357,     -4638.570,   68949.385, 8182756.988, 11550.531,
            12.027,    -55.853,     -11570.623,  22329.438, 21179.904,  1405507.522, 14.054,    19.681,      -2019.482,
            29059.862, 1427298.713, 5269.376,    -12.758,   45.389,     -4638.908,   19660.795, 17807.651,   468417.825,
            -34.156,   21.141,      -1551.880,   17043.207, 479915.258, 3121.779,    -2.744,    -9.340,      -2964.904,
            2275.892,  1503.751,    465395.271,  1.551,     5.105,      -282.405};

        model_config_params.LINK_MOMENT_OF_INERTIA_LOW = {
            53972.936, 51289.371, 13773.619,  -37.314,   37.357,    -4638.570, 68949.385, 65688.188, 11550.531,
            12.027,    -55.853,   -11570.623, 22329.438, 21179.904, 5307.522,  14.054,    19.681,    -2019.482,
            29059.862, 27098.713, 5269.376,   -12.758,   45.389,    -4638.908, 19660.795, 17807.651, 4577.825,
            -34.156,   21.141,    -1551.880,  17043.207, 16075.258, 3121.779,  -2.744,    -9.340,    -2964.904,
            2275.892,  1503.751,  1555.271,   1.551,     5.105,     -282.405};

        model_config_params.ROBOT_DIMENSIONS = {0.00, 0.00, 0.00,   0.00, 0.00, 404.00, 0.00, 0.00, 437.50, 0.00, 0.00, 0.00,
                                                0.00, 0.00, 412.50, 0.00, 0.00, 0.00,   0.00, 0.00, 275.50, 0.00, 0.00, 0.00};

        model_config_params.JOINT_RANGE_MIN_CUSTOMIZE = {-165, -115, -165, -115, -165, -115, -355};
        model_config_params.JOINT_RANGE_MAX_CUSTOMIZE = {165, 115, 165, 115, 165, 115, 355};
        model_config_params.JOINT_RANGE_MIN_NEW = {-170, -120, -170, -120, -170, -120, -360};
        model_config_params.JOINT_RANGE_MAX_NEW = {170, 120, 170, 120, 170, 120, 360};

        // control_config_params
        control_config_params.CTRL_BANDWIDTH_SERVO_EXEC = {30.0, 30.0, 30.0, 30.0, 30.0, 30.0, 40.0};
        control_config_params.CTRL_ZETA_SERVO_EXEC = {0.7, 0.7, 0.7, 0.7, 0.7, 0.7, 0.7};
        control_config_params.FRICTION_COF_DRAG = {0.8, 0.8, 0.8, 0.8, 0.8, 0.8, 0.8};
        control_config_params.ROTATION_STIFFNESS_OF_TRANSLATION_DRAGGING = {300, 300, 300};
        control_config_params.ROTATION_DAMPING_OF_TRANSLATION_DRAGGING = {5.0, 5.0, 5.0};
        control_config_params.TRANSLATION_STIFFNESS_OF_ROTATION_DRAGGING = {2000, 2000, 2000};
        control_config_params.TRANSLATION_DAMPING_OF_ROTATION_DRAGGING = {10.0, 10.0, 10.0};

        // mechanical_config_params
        mechanical_config_params.ENCODER_OFFESET = {0, 0, 0, 0, 0, 0};
        mechanical_config_params.ENCODER_RESOLUTION = {262144, 262144, 262144, 262144, 262144, 262144, 262144};
        mechanical_config_params.REDUCTION_RATIO_NUMERATOR = {160, 160, 100, 100, 100, 100, 100};
        mechanical_config_params.REDUCTION_RATIO_DENOMINATOR = {1, 1, 1, 1, 1, 1};
        mechanical_config_params.SENSOR_ANALOG_TO_TORQUE_HIGH = {210, 210, 110, 110, 60, 60, 60};
        mechanical_config_params.SENSOR_ANALOG_TO_TORQUE_LOW = {2.25, 2.25, 2.25, 2.25, 2.25, 2.25, 2.25};
        mechanical_config_params.SENSOR_BIAS = {2000, 2000, 2000, 2000, 2000, 2000, 2000};
        mechanical_config_params.SENSOR_AMPLIFY = {1, 1, 1, 1, 1, 1, 1};
        mechanical_config_params.RATED_TORQUE = {1.5, 1.5, 0.939, 0.939, 0.415, 0.415, 0.415};

        // protect_config_params
        protect_config_params.SWITCH_THRESHOLD_OF_TORQUE_CONTROL = {25.0, 25.0, 20.0, 25.0, 15.0, 15.0, 10.0};
        break;
    default:
    // 其他未定义机型，直接报错
        throw std::invalid_argument("Unsupported robot type provided to RobotConfiguration constructor.");
    }
}

void RobotConfiguration::Resize(unsigned int jnt_num) {
    control_config_params.Resize(jnt_num);
    mechanical_config_params.Resize(jnt_num);
    protect_config_params.Resize(jnt_num);
    model_config_params.Resize(jnt_num);
}
}  // namespace Model
}  // namespace RokaeApi