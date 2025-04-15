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
        this->Resize(SIX_AXIS_ROBOT);
        // ModelConfigParams
        model_config_params.ROBOT_NAME = "XMS3-R580-W4G3B1C";
        model_config_params.AXIS_NUM = 6;
        model_config_params.MAX_LOAD = 4.5;
        model_config_params.MAX_LOAD_TCP_LENGTH = 0.3;

        model_config_params.JOINT_TYPE = {8, 3, 2, 2, 3, 2, 3};
        model_config_params.ROT_AXIS = {0, 0, 3, 0, 0, 0, 3};
        model_config_params.ROT_ANGLE = {0.0, 0.0, PI, 0.0, 0.0, 0.0, PI};

        model_config_params.LINK_MASS = {2.52, 3.268, 1.662, 1.882, 1.507, 0.634};

        model_config_params.LINK_CENTROID = {0.022,  -8.728,  225.433, 19.368, -68.585, 146.373, 24.115, 12.803, 39.802,
                                             -0.025, -10.923, -71.267, 0.003,  29.173,  7.019,   0.073,  0.079,  -44.853};

        model_config_params.LINK_MOMENT_OF_INERTIA = {
            21369.338, 20930.725, 822620.658, 0.113,    -3.834,     -1763.079,  63295.898, 881968.783, 9116.219,
            524.974,   9061.836,  -171.482,   6774.086, 825626.587, 3518.156,   -575.854,  1770.774,   -902.096,
            13319.852, 12519.892, 231678.281, -1.85,    0.867,      -1438.94,   5032.625,  231504.418, 3944.803,
            -6.075,    -1.293,    -331.648,   997.016,  1002.53,    229337.948, -1.376,    0.897,      1.581};

        model_config_params.LINK_MOMENT_OF_INERTIA_LOW = {
            21369.338, 20930.725, 4182.258, 0.113,    -3.834,   -1763.079, 63295.898, 63530.383, 9116.219,
            524.974,   9061.836,  -171.482, 6774.086, 7188.187, 3518.156,  -575.854,  1770.774,  -902.096,
            13319.852, 12519.892, 2788.281, -1.85,    0.867,    -1438.94,  5032.625,  2614.418,  3944.803,
            -6.075,    -1.293,    -331.648, 997.016,  1002.53,  447.948,   -1.376,    0.897,     1.581};

        model_config_params.ROBOT_DIMENSIONS = {0.0,   0.0, 0.0,    0.0, 0.0, 313.0, 50.0, 0.0,   290.0, 50.0, 0.0,
                                                290.0, 0.0, -136.0, 0.0, 0.0, 0.0,   0.0,  103.5, 0.0,   0.0,  0.0};

        model_config_params.JOINT_RANGE_MIN_CUSTOMIZE = {-360, -155, -175, -360, -360, -360};
        model_config_params.JOINT_RANGE_MAX_CUSTOMIZE = {360, 140, 135, 360, 360, 360};
        model_config_params.JOINT_RANGE_MIN_NEW = {-360, -155, -175, -360, -360, -360};
        model_config_params.JOINT_RANGE_MAX_NEW = {360, 140, 135, 360, 360, 360};

        // control_config_params
        control_config_params.CTRL_BANDWIDTH_SERVO_EXEC = {32.0, 32.0, 32.0, 30.0, 20.0, 35.0};
        control_config_params.CTRL_ZETA_SERVO_EXEC = {0.7, 0.7, 0.7, 0.7, 0.4, 0.4};
        control_config_params.FRICTION_COF_DRAG = {0.5, 0.4, 0.4, 0.5, 0.5, 0.8};
        control_config_params.ROTATION_STIFFNESS_OF_TRANSLATION_DRAGGING = {300, 300, 300};
        control_config_params.ROTATION_DAMPING_OF_TRANSLATION_DRAGGING = {5.0, 5.0, 5.0};
        control_config_params.TRANSLATION_STIFFNESS_OF_ROTATION_DRAGGING = {2000, 2000, 2000};
        control_config_params.TRANSLATION_DAMPING_OF_ROTATION_DRAGGING = {10.0, 10.0, 10.0};

        // mechanical_config_params
        mechanical_config_params.ENCODER_OFFESET = {0, 0, 0, 0, 0, 0};
        mechanical_config_params.ENCODER_RESOLUTION = {262144, 262144, 262144, 262144, 262144, 262144};
        mechanical_config_params.REDUCTION_RATIO_NUMERATOR = {120, -120, -120, 100, -100, 100};
        mechanical_config_params.REDUCTION_RATIO_DENOMINATOR = {1, 1, 1, 1, 1, 1};
        mechanical_config_params.SENSOR_ANALOG_TO_TORQUE_HIGH = {60, 60, 60, 25, 25, 25};
        mechanical_config_params.SENSOR_ANALOG_TO_TORQUE_LOW = {2.25, -2.25, -2.25, 2.25, -2.25, 2.25};
        mechanical_config_params.SENSOR_BIAS = {2500, 2500, 2500, 2500, 2500, 2500};
        mechanical_config_params.SENSOR_AMPLIFY = {1, 1, 1, 1, 1, 1};
        mechanical_config_params.RATED_TORQUE = {0.55, 0.55, 0.55, 0.2, 0.2, 0.2};

        // protect_config_params
        protect_config_params.SWITCH_THRESHOLD_OF_TORQUE_CONTROL = {15.0, 15.0, 15.0, 10.0, 10.0, 10.0};
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