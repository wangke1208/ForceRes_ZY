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
      CART_IMPEDANCE_CTRL_DAMP_ZETA(DEFAULT_CART_PARAMS_SIZE, 0.0),
      SENSOR_BIAS_DYNAMIC(jnt_num, 2500),
      POS_FIX_PARMAS(DEFAULT_SENSOR_FIX_SIZE, 0.0),
      NEG_FIX_PARMAS(DEFAULT_SENSOR_FIX_SIZE, 0.0),
      IS_SUPPORT_SENSOR_FIX(jnt_num, false) {}

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
    SENSOR_BIAS_DYNAMIC.resize(jnt_num, 2500);
    POS_FIX_PARMAS.resize(DEFAULT_SENSOR_FIX_SIZE * jnt_num, 0.0);
    NEG_FIX_PARMAS.resize(DEFAULT_SENSOR_FIX_SIZE * jnt_num, 0.0);
    IS_SUPPORT_SENSOR_FIX.resize(jnt_num, false);
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

RobotConfiguration::RobotConfiguration(MechUnitType robot_type) {
    switch (robot_type) {
    case MechUnitType::AR5_L:
        this->Resize(SEVEN_AXIS_ROBOT);
        // ModelConfigParams
        model_config_params.ROBOT_NAME = "AR5_L";
        model_config_params.AXIS_NUM = 7;
        model_config_params.MAX_LOAD = 5;
        model_config_params.MAX_LOAD_TCP_LENGTH = 0.3;
        model_config_params.IS_WRITST_CROSS = false;

        model_config_params.JOINT_TYPE = {8, 3, 2, 3, 2, 3, 2, 3};
        model_config_params.ROT_AXIS = {0, 0, 0, 0, 0, 0, 0, 0};
        model_config_params.ROT_ANGLE = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};

        model_config_params.LINK_MASS = {2.400, 2.260, 1.850, 1.190, 0.950, 1.060, 0.180};

        model_config_params.LINK_CENTROID = {0.026805,  1.174921,   171.179958, 0.010382,  -3.330598, 100.666568, -7.574792,
                                             1.848523,  -17.501298, 7.943048,   -5.043471, 70.013368, 0.012128,   1.731178,
                                             -2.652562, -0.053371,  -2.338131,  76.153958, -0.158025, 0.036244,   -17.928215};

        model_config_params.LINK_MOMENT_OF_INERTIA = {
            3979.647236102, 3153.608616,  3579.792654, 3.028196641,  0.352045521, 24.41954637, 9984.556133,
            8642.427123,    3979.094239,  0.113661929, -0.493471216, 709.4387148, 5149.01341,  4558.395976,
            2498.658938,    7.046503262,  335.2268369, 53.13042131,  3585.14617,  2888.200439, 1733.465859,
            -50.09269845,   -166.4539576, 413.9160045, 943.702398,   714.8274519, 867.2998326, 0.569137902,
            0.595997585,    9.192853216,  2307.592731, 1956.768415,  1033.9388,   -0.66612469, 0.715815857,
            187.1128396,    99.75647412,  96.7123724,  148.2758075,  1.386868756, 0.247604245, 0.357437425};

        model_config_params.LINK_MOMENT_OF_INERTIA_LOW = {
            3979.647236102, 3153.608616,  3579.792654, 3.028196641,  0.352045521, 24.41954637, 9984.556133,
            8642.427123,    3979.094239,  0.113661929, -0.493471216, 709.4387148, 5149.01341,  4558.395976,
            2498.658938,    7.046503262,  335.2268369, 53.13042131,  3585.14617,  2888.200439, 1733.465859,
            -50.09269845,   -166.4539576, 413.9160045, 943.702398,   714.8274519, 867.2998326, 0.569137902,
            0.595997585,    9.192853216,  2307.592731, 1956.768415,  1033.9388,   -0.66612469, 0.715815857,
            187.1128396,    99.75647412,  96.7123724,  148.2758075,  1.386868756, 0.247604245, 0.357437425};

        model_config_params.ROBOT_DIMENSIONS = {0.00,  0.00, 0.00,   0.00, 0.00, 174.50, 0.00, 0.00, 305.00, -10.00, 0.00, 0.00,
                                                10.00, 0.00, 179.00, 0.00, 0.00, 0.00,   0.00, 0.00, 166.00, 0.00,   0.00, 0.00};

        model_config_params.JOINT_RANGE_MIN_CUSTOMIZE = {-178, -120, -178, -145, -178, -110, -180};
        model_config_params.JOINT_RANGE_MAX_CUSTOMIZE = {178, 120, 178, 80, 178, 110, 180};
        model_config_params.JOINT_RANGE_MIN_NEW = {-178, -120, -178, -145, -178, -110, -180};
        model_config_params.JOINT_RANGE_MAX_NEW = {178, 120, 178, 80, 178, 110, 180};

        // control_config_params
        control_config_params.CTRL_BANDWIDTH_SERVO_EXEC = {25.0, 25.0, 30.0, 25.0, 40.0, 40.0, 40.0};
        control_config_params.CTRL_ZETA_SERVO_EXEC = {0.3, 0.3, 0.3, 0.3, 0.3, 0.3, 0.3};
        control_config_params.FRICTION_COF_DRAG = {0.7, 0.5, 0.5, 0.5, 0.7, 0.7, 0.7};
        control_config_params.ROTATION_STIFFNESS_OF_TRANSLATION_DRAGGING = {300, 300, 300};
        control_config_params.ROTATION_DAMPING_OF_TRANSLATION_DRAGGING = {5.0, 5.0, 5.0};
        control_config_params.TRANSLATION_STIFFNESS_OF_ROTATION_DRAGGING = {2000, 2000, 2000};
        control_config_params.TRANSLATION_DAMPING_OF_ROTATION_DRAGGING = {10.0, 10.0, 10.0};
        //阻抗部分
        control_config_params.FRICTION_COF_IMPED = {0.4, 0.4, 0.4, 0.4, 0.4, 0.4, 0.4};
        control_config_params.CTRL_BANDWIDTH_SERVO_EXEC_IMPE = {20.0, 20.0, 25.0, 25.0, 30.0, 30.0, 30.0};
        control_config_params.JOINT_IMPEDANCE_CTRL_DAMP_ZETA = {0.8, 0.8, 0.7, 0.5, 0.3, 0.3, 0.3};
        control_config_params.CART_IMPEDANCE_CTRL_DAMP_ZETA = {0.3, 0.3, 0.3, 0.7, 0.7, 0.7};

        //传感器动态拟合参数
        control_config_params.SENSOR_BIAS_DYNAMIC = {2500, 2500, 2500, 2500, 2500, 2500, 2500};
        control_config_params.POS_FIX_PARMAS = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
                                                0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
                                                0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};
        control_config_params.NEG_FIX_PARMAS = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
                                                0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
                                                0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};
        control_config_params.IS_SUPPORT_SENSOR_FIX = {false, false, false, false, false, false, false};

        // mechanical_config_params
        mechanical_config_params.ENCODER_OFFESET = {0, 0, 0, 0, 0, 0, 0};
        mechanical_config_params.ENCODER_RESOLUTION = {262144, 262144, 262144, 262144, 262144, 262144, 262144};
        mechanical_config_params.REDUCTION_RATIO_NUMERATOR = {120, 120, 100, 100, 100, 100, 100};
        mechanical_config_params.REDUCTION_RATIO_DENOMINATOR = {1, 1, 1, 1, 1, 1, 1};
        mechanical_config_params.SENSOR_ANALOG_TO_TORQUE_HIGH = {110, 110, 70, 70, 25, 25, 25};
        mechanical_config_params.SENSOR_ANALOG_TO_TORQUE_LOW = {2.25, 2.25, 2.25, 2.25, 2.25, 2.25, 2.25};
        mechanical_config_params.SENSOR_BIAS = {2500, 2500, 2500, 2500, 2500, 2500, 2500};
        mechanical_config_params.SENSOR_AMPLIFY = {1, 1, 1, 1, 1, 1, 1};
        mechanical_config_params.RATED_TORQUE = {1, 1, 0.45, 0.45, 0.2, 0.2, 0.2};

        // protect_config_params
        protect_config_params.SWITCH_THRESHOLD_OF_TORQUE_CONTROL = {25.0, 25.0, 20.0, 25.0, 15.0, 15.0, 10.0};
        break;
    case MechUnitType::AR5_R:
        this->Resize(SEVEN_AXIS_ROBOT);
        // ModelConfigParams
        model_config_params.ROBOT_NAME = "AR5_R";
        model_config_params.AXIS_NUM = 7;
        model_config_params.MAX_LOAD = 5;
        model_config_params.MAX_LOAD_TCP_LENGTH = 0.3;
        model_config_params.IS_WRITST_CROSS = false;

        model_config_params.JOINT_TYPE = {8, 3, 2, 3, 2, 3, 2, 3};
        model_config_params.ROT_AXIS = {0, 0, 0, 0, 0, 0, 0, 0};
        model_config_params.ROT_ANGLE = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};

        model_config_params.LINK_MASS = {2.4, 2.3, 1.85, 1.19, 0.95, 1.06, 0.18};

        model_config_params.LINK_CENTROID = {-0.0,   2.017,  171.128, -0.1,   5.578, 97.429, 7.655,  0.272, -17.2, -7.999, 5.054,
                                             71.124, -0.036, 1.054,   -2.771, 0.068, 2.374,  77.402, 0.157, 0.024, -17.978};

        model_config_params.LINK_MOMENT_OF_INERTIA = {
            3934.551, 3181.715, 968335.329, 0.061, -0.148,  3.435,  11219.968, 974421.161, 4377.736, 5.472,  -10.067,  -1215.6,
            4715.829, 4264.119, 328262.061, 2.278, 310.436, 15.123, 3646.441,  328947.698, 1748.173, 45.488, -164.301, -422.053,
            878.735,  707.542,  117797.46,  0.963, -0.201,  -1.249, 2350.862,  119001.526, 1046.598, -0.215, -0.841,   -190.476,
            100.464,  97.086,   117148.968, 1.386, -0.246,  -0.436};

        model_config_params.LINK_MOMENT_OF_INERTIA_LOW = {
            3934.551, 3181.715, 3535.329, 0.061,    -0.148, 3.435,   11219.968, 9621.161, 4377.736, 5.472,    -10.067,
            -1215.6,  4715.829, 4264.119, 2262.061, 2.278,  310.436, 15.123,    3646.441, 2947.698, 1748.173, 45.488,
            -164.301, -422.053, 878.735,  707.542,  797.46, 0.963,   -0.201,    -1.249,   2350.862, 2001.526, 1046.598,
            -0.215,   -0.841,   -190.476, 100.464,  97.086, 148.968, 1.386,     -0.246,   -0.436};

        model_config_params.ROBOT_DIMENSIONS = {0.0,   0.0, 0.0,   0.0, 0.0, 174.5, 0.0, 0.0, 305.0, 10.0, 0.0, 0.0,
                                                -10.0, 0.0, 179.0, 0.0, 0.0, 0.0,   0.0, 0.0, 166.0, 0.0,  0.0, 0.0};

        model_config_params.JOINT_RANGE_MIN_CUSTOMIZE = {-178, -120, -178, -60, -178, -110, -180};
        model_config_params.JOINT_RANGE_MAX_CUSTOMIZE = {178, 120, 178, 145, 178, 110, 180};
        model_config_params.JOINT_RANGE_MIN_NEW = {-178, -120, -178, -60, -178, -110, -180};
        model_config_params.JOINT_RANGE_MAX_NEW = {178, 120, 178, 145, 178, 110, 180};

        // control_config_params
        control_config_params.CTRL_BANDWIDTH_SERVO_EXEC = {25.0, 25.0, 30.0, 25.0, 40.0, 40.0, 40.0};
        control_config_params.CTRL_ZETA_SERVO_EXEC = {0.3, 0.3, 0.3, 0.3, 0.3, 0.3, 0.3};
        control_config_params.FRICTION_COF_DRAG = {0.7, 0.5, 0.5, 0.5, 0.7, 0.7, 0.7};
        control_config_params.ROTATION_STIFFNESS_OF_TRANSLATION_DRAGGING = {300, 300, 300};
        control_config_params.ROTATION_DAMPING_OF_TRANSLATION_DRAGGING = {5.0, 5.0, 5.0};
        control_config_params.TRANSLATION_STIFFNESS_OF_ROTATION_DRAGGING = {2000, 2000, 2000};
        control_config_params.TRANSLATION_DAMPING_OF_ROTATION_DRAGGING = {10.0, 10.0, 10.0};

        //阻抗部分
        control_config_params.FRICTION_COF_IMPED = {0.4, 0.4, 0.4, 0.4, 0.4, 0.4, 0.4};
        control_config_params.CTRL_BANDWIDTH_SERVO_EXEC_IMPE = {20.0, 20.0, 25.0, 25.0, 30.0, 30.0, 30.0};
        control_config_params.JOINT_IMPEDANCE_CTRL_DAMP_ZETA = {0.8, 0.8, 0.7, 0.5, 0.3, 0.3, 0.3};
        control_config_params.CART_IMPEDANCE_CTRL_DAMP_ZETA = {0.3, 0.3, 0.3, 0.7, 0.7, 0.7};

        //传感器动态拟合参数
        control_config_params.SENSOR_BIAS_DYNAMIC = {2500, 2500, 2500, 2500, 2500, 2500, 2500};
        control_config_params.POS_FIX_PARMAS = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
                                                0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
                                                0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};
        control_config_params.NEG_FIX_PARMAS = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
                                                0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
                                                0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};
        control_config_params.IS_SUPPORT_SENSOR_FIX = {false, false, false, false, false, false, false};

        // mechanical_config_params
        mechanical_config_params.ENCODER_OFFESET = {0, 0, 0, 0, 0, 0, 0};
        mechanical_config_params.ENCODER_RESOLUTION = {262144, 262144, 262144, 262144, 262144, 262144, 262144};
        mechanical_config_params.REDUCTION_RATIO_NUMERATOR = {120, 120, 100, 100, 100, 100, 100};
        mechanical_config_params.REDUCTION_RATIO_DENOMINATOR = {1, 1, 1, 1, 1, 1, 1};
        mechanical_config_params.SENSOR_ANALOG_TO_TORQUE_HIGH = {110, 110, 70, 70, 25, 25, 25};
        mechanical_config_params.SENSOR_ANALOG_TO_TORQUE_LOW = {2.25, 2.25, 2.25, 2.25, 2.25, 2.25, 2.25};
        mechanical_config_params.SENSOR_BIAS = {2500, 2500, 2500, 2500, 2500, 2500, 2500};
        mechanical_config_params.SENSOR_AMPLIFY = {1, 1, 1, 1, 1, 1, 1};
        mechanical_config_params.RATED_TORQUE = {1, 1, 0.45, 0.45, 0.2, 0.2, 0.2};

        // protect_config_params
        protect_config_params.SWITCH_THRESHOLD_OF_TORQUE_CONTROL = {25.0, 25.0, 20.0, 25.0, 15.0, 15.0, 10.0};
        break;
    case MechUnitType::AR5C_L:
        this->Resize(SEVEN_AXIS_ROBOT);
        // ModelConfigParams
        model_config_params.ROBOT_NAME = "AR5C_L";
        model_config_params.AXIS_NUM = 7;
        model_config_params.MAX_LOAD = 5;
        model_config_params.MAX_LOAD_TCP_LENGTH = 0.3;
        model_config_params.IS_WRITST_CROSS = true;

        model_config_params.JOINT_TYPE = {8, 3, 2, 3, 2, 3, 2, 1};
        model_config_params.ROT_AXIS = {0, 0, 0, 0, 0, 0, 0, 0};
        model_config_params.ROT_ANGLE = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};

        model_config_params.LINK_MASS = {2.400, 2.300, 1.850, 1.190, 1.270, 0.910, 0.300};

        model_config_params.LINK_CENTROID = {0.027,   1.175,   171.180, 0.010,  -3.331, 100.667, 7.575,
                                             1.849,   -17.501, -7.943,  -5.043, 70.013, 0.523,   -0.103,
                                             -83.612, 3.719,   0.000,   -0.066, 1.297,  -0.124,  48.647};

        model_config_params.LINK_MOMENT_OF_INERTIA = {
            3979.647, 3153.609, 968379.793, 3.028,  0.352,   24.420, 9984.556, 973442.427, 3979.094, 0.114,   -0.493,   709.439,
            5149.013, 4558.396, 328498.659, 7.047,  335.227, 53.130, 3585.146, 328888.200, 1733.466, -50.093, -166.454, 413.916,
            2216.236, 1702.209, 118526.713, 28.726, 45.240,  41.024, 596.605,  117711.286, 736.939,  -5.942,  0.841,    0.397,
            381.764,  632.012,  117428.751, -1.569, -24.088, -0.211};

        model_config_params.LINK_MOMENT_OF_INERTIA_LOW = {
            3979.647, 3153.609, 3579.793, 3.028,    0.352,    24.420,  9984.556, 8642.427, 3979.094, 0.114,    -0.493,
            709.439,  5149.013, 4558.396, 2498.659, 7.047,    335.227, 53.130,   3585.146, 2888.200, 1733.466, -50.093,
            -166.454, 413.916,  2216.236, 1702.209, 1526.713, 28.726,  45.240,   41.024,   596.605,  711.286,  736.939,
            -5.942,   0.841,    0.397,    381.764,  632.012,  428.751, -1.569,   -24.088,  -0.211};

        model_config_params.ROBOT_DIMENSIONS = {0.00,   0.00, 0.00,   0.00, 0.00, 174.50, 0.00, 0.00, 305.00, 10.00, 0.00, 0.00,
                                                -10.00, 0.00, 264.00, 0.00, 0.00, 0.00,   0.00, 0.00, 0.00,   0.00,  0.00, 86.00};

        model_config_params.JOINT_RANGE_MIN_CUSTOMIZE = {-178, -120, -178, -60, -178, -60, -60};
        model_config_params.JOINT_RANGE_MAX_CUSTOMIZE = {178, 120, 178, 145, 178, 60, 60};
        model_config_params.JOINT_RANGE_MIN_NEW = {-178, -120, -178, -60, -178, -60, -60};
        model_config_params.JOINT_RANGE_MAX_NEW = {178, 120, 178, 145, 178, 60, 60};

        // control_config_params
        control_config_params.CTRL_BANDWIDTH_SERVO_EXEC = {25.0, 25.0, 30.0, 25.0, 40.0, 45.0, 45.0};
        control_config_params.CTRL_ZETA_SERVO_EXEC = {0.3, 0.3, 0.3, 0.3, 0.3, 0.3, 0.3};
        control_config_params.FRICTION_COF_DRAG = {0.7, 0.5, 0.7, 0.5, 0.8, 0.9, 0.9};
        control_config_params.ROTATION_STIFFNESS_OF_TRANSLATION_DRAGGING = {300, 300, 300};
        control_config_params.ROTATION_DAMPING_OF_TRANSLATION_DRAGGING = {5.0, 5.0, 5.0};
        control_config_params.TRANSLATION_STIFFNESS_OF_ROTATION_DRAGGING = {2000, 2000, 2000};
        control_config_params.TRANSLATION_DAMPING_OF_ROTATION_DRAGGING = {10.0, 10.0, 10.0};

        //阻抗部分
        control_config_params.FRICTION_COF_IMPED = {0.4, 0.4, 0.4, 0.4, 0.4, 0.4, 0.4};
        control_config_params.CTRL_BANDWIDTH_SERVO_EXEC_IMPE = {20.0, 20.0, 25.0, 25.0, 30.0, 30.0, 30.0};
        control_config_params.JOINT_IMPEDANCE_CTRL_DAMP_ZETA = {0.8, 0.8, 0.7, 0.5, 0.3, 0.3, 0.3};
        control_config_params.CART_IMPEDANCE_CTRL_DAMP_ZETA = {0.3, 0.3, 0.3, 0.7, 0.7, 0.7};

        //传感器动态拟合参数
        control_config_params.SENSOR_BIAS_DYNAMIC = {2500, 2500, 2500, 2500, 2500, 2500, 2500};
        control_config_params.POS_FIX_PARMAS = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
                                                0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
                                                0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};
        control_config_params.NEG_FIX_PARMAS = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
                                                0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
                                                0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};
        control_config_params.IS_SUPPORT_SENSOR_FIX = {false, false, false, false, false, false, false};

        // mechanical_config_params
        mechanical_config_params.ENCODER_OFFESET = {0, 0, 0, 0, 0, 0, 0};
        mechanical_config_params.ENCODER_RESOLUTION = {262144, 262144, 262144, 262144, 262144, 262144, 262144};
        mechanical_config_params.REDUCTION_RATIO_NUMERATOR = {120, -120, 100, -100, 100, 100, 100};
        mechanical_config_params.REDUCTION_RATIO_DENOMINATOR = {1, 1, 1, 1, 1, 1, 1};
        mechanical_config_params.SENSOR_ANALOG_TO_TORQUE_HIGH = {110, 110, 70, 70, 25, 25, 25};
        mechanical_config_params.SENSOR_ANALOG_TO_TORQUE_LOW = {2.25, -2.25, 2.25, -2.25, 2.25, 2.25, 2.25};
        mechanical_config_params.SENSOR_BIAS = {2500, 2500, 2500, 2500, 2500, 2500, 2500};
        mechanical_config_params.SENSOR_AMPLIFY = {1, 1, 1, 1, 1, 1, 1};
        mechanical_config_params.RATED_TORQUE = {1, 1, 0.45, 0.45, 0.2, 0.2, 0.2};

        // protect_config_params
        protect_config_params.SWITCH_THRESHOLD_OF_TORQUE_CONTROL = {25.0, 25.0, 20.0, 25.0, 15.0, 15.0, 10.0};
        break;
    case MechUnitType::AR5C_R:
        this->Resize(SEVEN_AXIS_ROBOT);
        // ModelConfigParams
        model_config_params.ROBOT_NAME = "AR5C_R";
        model_config_params.AXIS_NUM = 7;
        model_config_params.MAX_LOAD = 5;
        model_config_params.MAX_LOAD_TCP_LENGTH = 0.3;
        model_config_params.IS_WRITST_CROSS = true;

        model_config_params.JOINT_TYPE = {8, 3, 2, 3, 2, 3, 2, 1};
        model_config_params.ROT_AXIS = {0, 0, 0, 0, 0, 0, 0, 0};
        model_config_params.ROT_ANGLE = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};

        model_config_params.LINK_MASS = {2.400, 2.300, 1.850, 1.190, 1.270, 0.910, 0.300};

        model_config_params.LINK_CENTROID = {-0.000, 2.017, 171.128, -0.100,  5.578, 97.429, 7.655, 0.272, -17.200, -7.999, 5.054,
                                             71.124, 0.969, 2.116,   -81.973, 3.722, -0.000, 0.066, 1.294, 0.124,   48.742};

        model_config_params.LINK_MOMENT_OF_INERTIA = {
            3934.551, 3181.715,   968335.329, 0.061,    -0.148,     3.435,      11219.968, 974421.161, 4377.736,
            5.472,    -10.067,    -1215.600,  4715.829, 4264.119,   328262.061, 2.278,     310.436,    15.123,
            3646.441, 328947.698, 1748.173,   45.488,   -164.301,   -422.053,   2579.023,  1914.327,   118769.446,
            -57.580,  67.443,     -30.172,    596.626,  117711.220, 736.875,    5.942,     -0.841,     0.397,
            382.763,  633.005,    117428.830, 1.569,    -24.126,    0.207};

        model_config_params.LINK_MOMENT_OF_INERTIA_LOW = {
            3934.551,  3181.715, 3535.329, 0.061,    -0.148,   3.435,   11219.968, 9621.161, 4377.736, 5.472,    -10.067,
            -1215.600, 4715.829, 4264.119, 2262.061, 2.278,    310.436, 15.123,    3646.441, 2947.698, 1748.173, 45.488,
            -164.301,  -422.053, 2579.023, 1914.327, 1769.446, -57.580, 67.443,    -30.172,  596.626,  711.220,  736.875,
            5.942,     -0.841,   0.397,    382.763,  633.005,  428.830, 1.569,     -24.126,  0.207};

        model_config_params.ROBOT_DIMENSIONS = {0.00,   0.00, 0.00,   0.00, 0.00, 174.50, 0.00, 0.00, 305.00, 10.00, 0.00, 0.00,
                                                -10.00, 0.00, 264.00, 0.00, 0.00, 0.00,   0.00, 0.00, 0.00,   0.00,  0.00, 86.00};

        model_config_params.JOINT_RANGE_MIN_CUSTOMIZE = {-178, -120, -178, -60, -178, -60, -60};
        model_config_params.JOINT_RANGE_MAX_CUSTOMIZE = {178, 120, 178, 145, 178, 60, 60};
        model_config_params.JOINT_RANGE_MIN_NEW = {-178, -120, -178, -60, -178, -60, -60};
        model_config_params.JOINT_RANGE_MAX_NEW = {178, 120, 178, 145, 178, 60, 60};

        // control_config_params
        control_config_params.CTRL_BANDWIDTH_SERVO_EXEC = {25.0, 25.0, 30.0, 25.0, 40.0, 45.0, 45.0};
        control_config_params.CTRL_ZETA_SERVO_EXEC = {0.3, 0.3, 0.3, 0.3, 0.3, 0.3, 0.3};
        control_config_params.FRICTION_COF_DRAG = {0.7, 0.5, 0.5, 0.5, 0.7, 0.7, 0.7};
        control_config_params.ROTATION_STIFFNESS_OF_TRANSLATION_DRAGGING = {300, 300, 300};
        control_config_params.ROTATION_DAMPING_OF_TRANSLATION_DRAGGING = {5.0, 5.0, 5.0};
        control_config_params.TRANSLATION_STIFFNESS_OF_ROTATION_DRAGGING = {2000, 2000, 2000};
        control_config_params.TRANSLATION_DAMPING_OF_ROTATION_DRAGGING = {10.0, 10.0, 10.0};

        //阻抗部分
        control_config_params.FRICTION_COF_IMPED = {0.4, 0.4, 0.4, 0.4, 0.4, 0.4, 0.4};
        control_config_params.CTRL_BANDWIDTH_SERVO_EXEC_IMPE = {20.0, 20.0, 25.0, 25.0, 30.0, 30.0, 30.0};
        control_config_params.JOINT_IMPEDANCE_CTRL_DAMP_ZETA = {0.8, 0.8, 0.7, 0.5, 0.3, 0.3, 0.3};
        control_config_params.CART_IMPEDANCE_CTRL_DAMP_ZETA = {0.3, 0.3, 0.3, 0.7, 0.7, 0.7};

        //传感器动态拟合参数
        control_config_params.SENSOR_BIAS_DYNAMIC = {2500, 2500, 2500, 2500, 2500, 2500, 2500};
        control_config_params.POS_FIX_PARMAS = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
                                                0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
                                                0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};
        control_config_params.NEG_FIX_PARMAS = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
                                                0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
                                                0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};
        control_config_params.IS_SUPPORT_SENSOR_FIX = {false, false, false, false, false, false, false};

        // mechanical_config_params
        mechanical_config_params.ENCODER_OFFESET = {0, 0, 0, 0, 0, 0, 0};
        mechanical_config_params.ENCODER_RESOLUTION = {262144, 262144, 262144, 262144, 262144, 262144, 262144};
        mechanical_config_params.REDUCTION_RATIO_NUMERATOR = {120, 120, 100, 100, 100, -100, 100};
        mechanical_config_params.REDUCTION_RATIO_DENOMINATOR = {1, 1, 1, 1, 1, 1, 1};
        mechanical_config_params.SENSOR_ANALOG_TO_TORQUE_HIGH = {110, 110, 70, 70, 25, 25, 25};
        mechanical_config_params.SENSOR_ANALOG_TO_TORQUE_LOW = {2.25, 2.25, 2.25, 2.25, 2.25, -2.25, 2.25};
        mechanical_config_params.SENSOR_BIAS = {2500, 2500, 2500, 2500, 2500, 2500, 2500};
        mechanical_config_params.SENSOR_AMPLIFY = {1, 1, 1, 1, 1, 1, 1};
        mechanical_config_params.RATED_TORQUE = {1, 1, 0.45, 0.45, 0.2, 0.2, 0.2};

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