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
    case MechUnitType::AR5_L:
        this->Resize(SEVEN_AXIS_ROBOT);
        // ModelConfigParams
        model_config_params.ROBOT_NAME = "AR5_L";
        model_config_params.AXIS_NUM = 7;
        model_config_params.MAX_LOAD = 5;
        model_config_params.MAX_LOAD_TCP_LENGTH = 0.3;

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
        control_config_params.CTRL_BANDWIDTH_SERVO_EXEC = {30.0, 30.0, 30.0, 30.0, 40.0, 40.0, 40.0};
        control_config_params.CTRL_ZETA_SERVO_EXEC = {0.7, 0.7, 0.7, 0.7, 0.7, 0.7, 0.7};
        control_config_params.FRICTION_COF_DRAG = {0.9, 0.9, 0.9, 0.9, 0.9, 0.9, 0.9};
        control_config_params.ROTATION_STIFFNESS_OF_TRANSLATION_DRAGGING = {300, 300, 300};
        control_config_params.ROTATION_DAMPING_OF_TRANSLATION_DRAGGING = {5.0, 5.0, 5.0};
        control_config_params.TRANSLATION_STIFFNESS_OF_ROTATION_DRAGGING = {2000, 2000, 2000};
        control_config_params.TRANSLATION_DAMPING_OF_ROTATION_DRAGGING = {10.0, 10.0, 10.0};
        control_config_params.JOINT_IMPEDANCE_CTRL_DAMP_ZETA = {0.8, 0.8, 0.7, 0.5, 0.3, 0.3, 0.3};
        control_config_params.CART_IMPEDANCE_CTRL_DAMP_ZETA = {0.5, 0.5, 0.5, 0.3, 0.3, 0.3};

        // mechanical_config_params
        mechanical_config_params.ENCODER_OFFESET = {0, 0, 0, 0, 0, 0, 0};
        mechanical_config_params.ENCODER_RESOLUTION = {262144, 262144, 262144, 262144, 262144, 262144, 262144};
        mechanical_config_params.REDUCTION_RATIO_NUMERATOR = {120, 120, 100, 100, 100, 100, 100};
        mechanical_config_params.REDUCTION_RATIO_DENOMINATOR = {1, 1, 1, 1, 1, 1, 1};
        mechanical_config_params.SENSOR_ANALOG_TO_TORQUE_HIGH = {110, 110, 70, 70, 25, 25, 25};
        mechanical_config_params.SENSOR_ANALOG_TO_TORQUE_LOW = {2.25, 2.25, 2.25, 2.25, 2.25, 2.25, 2.25};
        mechanical_config_params.SENSOR_BIAS = {2500, 2500, 2500, 2500, 2500, 2500, 25000};
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

        model_config_params.JOINT_TYPE = {8, 3, 2, 3, 2, 3, 2, 3};
        model_config_params.ROT_AXIS = {0, 0, 0, 0, 0, 0, 0, 0};
        model_config_params.ROT_ANGLE = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};

        model_config_params.LINK_MASS = {2.400, 2.260, 1.850, 1.190, 0.950, 1.060, 0.180};

        model_config_params.LINK_CENTROID = {0.026805,  1.174921,   171.179958, 0.010382,  -3.330598, 100.666568, 7.574792,
                                             1.848523,  -17.501298, -7.943048,  -5.043471, 70.013368, 0.012128,   1.731178,
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

        model_config_params.ROBOT_DIMENSIONS = {0.00,   0.00, 0.00,   0.00, 0.00, 174.50, 0.00, 0.00, 305.00, 10.00, 0.00, 0.00,
                                                -10.00, 0.00, 179.00, 0.00, 0.00, 0.00,   0.00, 0.00, 166.00, 0.00,  0.00, 0.00};

        model_config_params.JOINT_RANGE_MIN_CUSTOMIZE = {-178, -120, -178, -80, -178, -110, -180};
        model_config_params.JOINT_RANGE_MAX_CUSTOMIZE = {178, 120, 178, 145, 178, 110, 180};
        model_config_params.JOINT_RANGE_MIN_NEW = {-178, -120, -178, -80, -178, -110, -180};
        model_config_params.JOINT_RANGE_MAX_NEW = {178, 120, 178, 145, 178, 110, 180};

        // control_config_params
        control_config_params.CTRL_BANDWIDTH_SERVO_EXEC = {30.0, 30.0, 30.0, 30.0, 40.0, 40.0, 40.0};
        control_config_params.CTRL_ZETA_SERVO_EXEC = {0.7, 0.7, 0.7, 0.7, 0.7, 0.7, 0.7};
        control_config_params.FRICTION_COF_DRAG = {0.9, 0.9, 0.9, 0.9, 0.9, 0.9, 0.9};
        control_config_params.ROTATION_STIFFNESS_OF_TRANSLATION_DRAGGING = {300, 300, 300};
        control_config_params.ROTATION_DAMPING_OF_TRANSLATION_DRAGGING = {5.0, 5.0, 5.0};
        control_config_params.TRANSLATION_STIFFNESS_OF_ROTATION_DRAGGING = {2000, 2000, 2000};
        control_config_params.TRANSLATION_DAMPING_OF_ROTATION_DRAGGING = {10.0, 10.0, 10.0};
        control_config_params.JOINT_IMPEDANCE_CTRL_DAMP_ZETA = {0.8, 0.8, 0.7, 0.5, 0.3, 0.3, 0.3};
        control_config_params.CART_IMPEDANCE_CTRL_DAMP_ZETA = {0.5, 0.5, 0.5, 0.3, 0.3, 0.3};

        // mechanical_config_params
        mechanical_config_params.ENCODER_OFFESET = {0, 0, 0, 0, 0, 0, 0};
        mechanical_config_params.ENCODER_RESOLUTION = {262144, 262144, 262144, 262144, 262144, 262144, 262144};
        mechanical_config_params.REDUCTION_RATIO_NUMERATOR = {120, 120, 100, 100, 100, 100, 100};
        mechanical_config_params.REDUCTION_RATIO_DENOMINATOR = {1, 1, 1, 1, 1, 1, 1};
        mechanical_config_params.SENSOR_ANALOG_TO_TORQUE_HIGH = {110, 110, 70, 70, 25, 25, 25};
        mechanical_config_params.SENSOR_ANALOG_TO_TORQUE_LOW = {2.25, 2.25, 2.25, 2.25, 2.25, 2.25, 2.25};
        mechanical_config_params.SENSOR_BIAS = {2500, 2500, 2500, 2500, 2500, 2500, 25000};
        mechanical_config_params.SENSOR_AMPLIFY = {1, 1, 1, 1, 1, 1, 1};
        mechanical_config_params.RATED_TORQUE = {1, 1, 0.45, 0.45, 0.2, 0.2, 0.2};

        // protect_config_params
        protect_config_params.SWITCH_THRESHOLD_OF_TORQUE_CONTROL = {25.0, 25.0, 20.0, 25.0, 15.0, 15.0, 10.0};
        break;
    case MechUnitType::DEFALUT_SIX_AXIS:
        this->Resize(SEVEN_AXIS_ROBOT);
        // ModelConfigParams
        model_config_params.ROBOT_NAME = "XMC7-R850-W7G3B1C";
        model_config_params.AXIS_NUM = 6;
        model_config_params.MAX_LOAD = 6;
        model_config_params.MAX_LOAD_TCP_LENGTH = 0.3;

        model_config_params.JOINT_TYPE = {8, 3, 2, 2, 3, 2, 3};
        model_config_params.ROT_AXIS = {0, 0, 3, 0, 0, 0, 3};
        model_config_params.ROT_ANGLE = {0.0, 0.0, PI, 0.0, 0.0, 0.0, PI};

        model_config_params.LINK_MASS = {5.489, 7.281, 2.872, 2.823, 2.34, 1.142};

        model_config_params.LINK_CENTROID = {-0.006, 35.426, 173.361, -0.006, 155.018, 199.572, -0.012, -44.249, 48.945,
                                             0.024,  18.331, -86.517, -0.043, -31.099, 14.133,  0.001,  -0.201,  -49.302};

        model_config_params.LINK_MOMENT_OF_INERTIA = {
            49303.867, 37221.789, 4358386.66, -10.899,   5.122,      13262.003, 342183.385, 4667499.02, 25164.737,
            -2.429,    -59.164,   7466.562,   20899.594, 1102806.17, 12136.455, -1.925,     0.242,      6293.742,
            31668.234, 30304.755, 574049.264, -0.223,    -0.437,     4367.878,  9533.629,   574175.545, 6920.849,
            -2.821,    8.24,      997.174,    2177.431,  2188.3,     569611.22, -0.178,     -0.155,     -5.703};

        model_config_params.LINK_MOMENT_OF_INERTIA_LOW = {
            49303.867, 37221.789, 28236.662, -10.899,   5.122,     13262.003, 342183.385, 337349.017, 25164.737,
            -2.429,    -59.16,    7466.562,  20899.594, 13076.166, 12136.455, -1.925,     0.242,      6293.742,
            31668.234, 30304.755, 5689.264,  -0.223,    -0.437,    4367.878,  9533.629,   5815.545,   6920.849,
            -2.821,    8.24,      997.174,   2177.431,  2188.3,    1251.22,   -0.178,     -0.155,     -5.703};

        model_config_params.ROBOT_DIMENSIONS = {0.0,   0.0, 0.0,   0.0, 0.0, 241.5, 0.0,   0.0, 490.0, 0.0, 0.0,
                                                360.0, 0.0, 150.0, 0.0, 0.0, 0.0,   127.0, 0.0, 0.0,   0.0};

        model_config_params.JOINT_RANGE_MIN_CUSTOMIZE = {-360, -360, -360, -360, -360, -360};
        model_config_params.JOINT_RANGE_MAX_CUSTOMIZE = {360, 360, 360, 360, 360, 360};
        model_config_params.JOINT_RANGE_MIN_NEW = {-360, -360, -360, -360, -360, -360};
        model_config_params.JOINT_RANGE_MAX_NEW = {360, 360, 360, 360, 360, 360};

        // control_config_params
        control_config_params.CTRL_BANDWIDTH_SERVO_EXEC = {25.0, 25.0, 25.0, 20.0, 20.0, 30.0};
        control_config_params.CTRL_ZETA_SERVO_EXEC = {0.5, 0.5, 0.5, 0.5, 0.5, 0.5};
        control_config_params.FRICTION_COF_DRAG = {0.9, 0.9, 0.9, 0.8, 0.8, 0.8};
        control_config_params.ROTATION_STIFFNESS_OF_TRANSLATION_DRAGGING = {300, 300, 300};
        control_config_params.ROTATION_DAMPING_OF_TRANSLATION_DRAGGING = {5.0, 5.0, 5.0};
        control_config_params.TRANSLATION_STIFFNESS_OF_ROTATION_DRAGGING = {2000, 2000, 2000};
        control_config_params.TRANSLATION_DAMPING_OF_ROTATION_DRAGGING = {10.0, 10.0, 10.0};
        control_config_params.JOINT_IMPEDANCE_CTRL_DAMP_ZETA = {1.0, 0.7, 0.5, 0.5, 0.5, 0.3};
        control_config_params.CART_IMPEDANCE_CTRL_DAMP_ZETA = {1.1, 1.1, 1.1, 0.4, 0.4, 0.4};

        // mechanical_config_params
        mechanical_config_params.ENCODER_OFFESET = {0, 0, 0, 0, 0, 0};
        mechanical_config_params.ENCODER_RESOLUTION = {262144, 262144, 262144, 262144, 262144, 262144, 262144};
        mechanical_config_params.REDUCTION_RATIO_NUMERATOR = {100, 100, 100, 100, 100, 100};
        mechanical_config_params.REDUCTION_RATIO_DENOMINATOR = {1, 1, 1, 1, 1, 1};
        mechanical_config_params.SENSOR_ANALOG_TO_TORQUE_HIGH = {210, 210, 110, 60, 60, 60};
        mechanical_config_params.SENSOR_ANALOG_TO_TORQUE_LOW = {2.25, 2.25, 2.25, 2.25, 2.25, 2.25};
        mechanical_config_params.SENSOR_BIAS = {2500, 2500, 2500, 2500, 2500, 2500};
        mechanical_config_params.SENSOR_AMPLIFY = {1, 1, 1, 1, 1, 1};
        mechanical_config_params.RATED_TORQUE = {1.8, 1.8, 0.63, 0.55, 0.55, 0.55};

        // protect_config_params
        protect_config_params.SWITCH_THRESHOLD_OF_TORQUE_CONTROL = {25.0, 25.0, 15.0, 10.0, 10.0, 10.0};
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