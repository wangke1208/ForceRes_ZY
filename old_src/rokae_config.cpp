/**
 * Copyright(C) 2024 Rokae Technology Co., Ltd.
 * All Rights Reserved.
 *
 * Information in this file is the intellectual property of Rokae Technology Co., Ltd,
 * And may contains trade secrets that must be stored and viewed confidentially.
 *
 * @file: rokae_config.hpp
 * @author: wangke
 * @date:
 * @brief: 机型文件配置参数
 */

#include "../rokae_header/rokae_config.hpp"

namespace RokaeApi {
namespace XMS3 {

// ModelParams
const string ROBOT_NAME = "XMS3-R580-W4G3B1C";
const unsigned int AXIS_NUM = 6;
const double MAX_LOAD = 4.5;  // kg
const std::vector<double> LINK_MASS = {2.52, 3.268, 1.662, 1.882, 1.507, 0.634};
const std::vector<double> LINK_CENTROID = {0.022,  -8.728,  225.433, 19.368, -68.585, 146.373, 24.115, 12.803, 39.802,
                                           -0.025, -10.923, -71.267, 0.003,  29.173,  7.019,   0.073,  0.079,  -44.853};  //质心(mm)
const std::vector<double> LINK_MOMENT_OF_INERTIA = {
    21369.338, 20930.725,  822620.658, 0.113,    -3.834,   -1763.079, 63295.898, 881968.783, 9116.219,   524.974, 9061.836, -171.482,
    6774.086,  825626.587, 3518.156,   -575.854, 1770.774, -902.096,  13319.852, 12519.892,  231678.281, -1.85,   0.867,    -1438.94,
    5032.625,  231504.418, 3944.803,   -6.075,   -1.293,   -331.648,  997.016,   1002.53,    229337.948, -1.376,  0.897,    1.581};  //总惯量(kg.mm**2)
const std::vector<double> LINK_MOMENT_OF_INERTIA_LOW = {
    21369.338, 20930.725, 4182.258, 0.113,    -3.834,   -1763.079, 63295.898, 63530.383, 9116.219, 524.974, 9061.836, -171.482,
    6774.086,  7188.187,  3518.156, -575.854, 1770.774, -902.096,  13319.852, 12519.892, 2788.281, -1.85,   0.867,    -1438.94,
    5032.625,  2614.418,  3944.803, -6.075,   -1.293,   -331.648,  997.016,   1002.53,   447.948,  -1.376,  0.897,    1.581};  //纯连杆惯量(kg.mm**2)
const std::vector<double> ROBOT_DIMENSIONS = {0.0,   0.0, 0.0,    0.0, 0.0, 313.0, 50.0,  0.0, 290.0, 50.0, 0.0,
                                              290.0, 0.0, -136.0, 0.0, 0.0, 0.0,   103.5, 0.0, 0.0,   0.0};

const std::vector<double> JOINT_RANGE_MIN_CUSTOMIZE = {-360, -155, -175, -360, -360, -360};
const std::vector<double> JOINT_RANGE_MAX_CUSTOMIZE = {360, 140, 135, 360, 360, 360};
const std::vector<double> JOINT_RANGE_MIN_NEW = {-360, -155, -175, -360, -360, -360};
const std::vector<double> JOINT_RANGE_MAX_NEW = {360, 140, 135, 360, 360, 360};

// ControlParams
const std::vector<double> CTRL_BANDWIDTH_SERVO_EXEC = {32.0, 32.0, 32.0, 30.0, 20.0, 35.0};  //拖动带宽
const std::vector<double> CTRL_ZETA_SERVO_EXEC = {0.7, 0.7, 0.7, 0.7, 0.4, 0.4};             // 拖动阻尼比
const std::vector<double> FRICTION_COF_DRAG = {0.5, 0.4, 0.4, 0.5, 0.5, 0.8};                //拖动摩擦力系数

// MechanicalParams
const std::vector<int> ENCODER_OFFESET = {0, 0, 0, 0, 0, 0};
const std::vector<int> ENCODER_RESOLUTION = {262144, 262144, 262144, 262144, 262144, 262144};
const std::vector<double> REDUCTION_RATIO_NUMERATOR = {120, -120, -120, 100, -100, 100};
const std::vector<double> REDUCTION_RATIO_DENOMINATOR = {1, 1, 1, 1, 1, 1};
const std::vector<double> SENSOR_ANALOG_TO_TORQUE_HIGH = {60, 60, 60, 25, 25, 25};
const std::vector<double> SENSOR_ANALOG_TO_TORQUE_LOW = {2.25, -2.25, -2.25, 2.25, -2.25, 2.25};
const std::vector<double> SENSOR_BIAS = {2500, 2500, 2500, 2500, 2500, 2500};
const std::vector<double> SENSOR_AMPLIFY = {1, 1, 1, 1, 1, 1};
const std::vector<double> RATED_TORQUE = {0.55, 0.55, 0.55, 0.2, 0.2, 0.2};

// ProtectParams
const std::vector<double> SWITCH_THRESHOLD_OF_TORQUE_CONTROL = {15.0, 15.0, 15.0, 10.0, 10.0, 10.0};

}  // namespace XMS3
namespace XMS4 {

// ModelParams
const string ROBOT_NAME = "XMS4-R800-B4G1A4C";
const unsigned int AXIS_NUM = 6;
const double MAX_LOAD = 4;  // kg
const std::vector<double> LINK_MASS = {3.167175036, 4.29096783, 2.018198648, 1.882223023, 1.507185922, 0.634034296};
const std::vector<double> LINK_CENTROID = {0.011136,  -7.992137, 235.842012, 16.666103, -69.300915, 184.67127, 24.900208, 9.985986, 83.739181,
                                           -0.025487, -10.92259, -71.267109, 0.002911,  29.173125,  7.018623,  0.072631,  0.079047, -44.852927};  //质心(mm)
const std::vector<double> LINK_MOMENT_OF_INERTIA = {
    31167.987, 30712.71,   1575088.602, -0.648,   -4.273,   -2236.558, 140425.151, 1709613.182, 12647.754,  390.088, 14948.192, -2552.767,
    21038.419, 840115.068, 4293.575,    -483.905, 4026.647, -1704.899, 13319.852,  12519.892,   231678.281, -1.85,   0.867,     -1438.94,
    5032.625,  231504.418, 3944.803,    -6.075,   -1.293,   -331.648,  997.016,    1002.53,     229337.948, -1.376,  0.897,     1.581};  //总惯量(kg.mm**2)
const std::vector<double> LINK_MOMENT_OF_INERTIA_LOW = {
    31167.987, 30712.71,  5877.402, -0.648,   -4.273,   -2236.558, 140425.151, 140401.982, 12647.754, 390.088, 14948.192, -2552.767,
    21038.419, 21676.668, 4293.575, -483.905, 4026.647, -1704.899, 13319.852,  12519.892,  2788.281,  -1.85,   0.867,     -1438.94,
    5032.625,  2614.418,  3944.803, -6.075,   -1.293,   -331.648,  997.016,    1002.53,    447.948,   -1.376,  0.897,     1.581};  //纯连杆惯量(kg.mm**2)
const std::vector<double> ROBOT_DIMENSIONS = {0.0, 0.0, 0.0, 0.0, 0.0, 328.0, 50.0, 0.0, 400.0, 50.0, 0, 400.0, 0.0, -136, 0.0, 0.0, 0.0, 103.5, 0.0, 0.0, 0.0};
const std::vector<double> JOINT_RANGE_MIN_CUSTOMIZE = {-175, -160, -170, -175, -175, -175};
const std::vector<double> JOINT_RANGE_MAX_CUSTOMIZE = {175, 150, 140, 175, 175, 175};
const std::vector<double> JOINT_RANGE_MIN_NEW = {-360, -160, -170, -360, -360, -360};
const std::vector<double> JOINT_RANGE_MAX_NEW = {360, 150, 140, 360, 360, 360};
// ControlParams
const std::vector<double> CTRL_BANDWIDTH_SERVO_EXEC = {20.0, 30.0, 30.0, 25.0, 15.0, 35.0};  //拖动带宽
const std::vector<double> CTRL_ZETA_SERVO_EXEC = {0.7, 0.7, 0.7, 0.5, 0.4, 0.5};             // 拖动阻尼比
const std::vector<double> FRICTION_COF_DRAG = {0.7, 0.7, 0.7, 0.7, 0.7, 0.7};                //拖动摩擦力系数

// MechanicalParams
const std::vector<int> ENCODER_OFFESET = {0, 0, 0, 0, 0, 0};
const std::vector<int> ENCODER_RESOLUTION = {262144, 262144, 262144, 262144, 262144, 262144};
const std::vector<double> REDUCTION_RATIO_NUMERATOR = {120, -120, -120, 100, -100, 100};
const std::vector<double> REDUCTION_RATIO_DENOMINATOR = {1, 1, 1, 1, 1, 1};
const std::vector<double> SENSOR_ANALOG_TO_TORQUE_HIGH = {110, 110, 60, 25, 25, 25};
const std::vector<double> SENSOR_ANALOG_TO_TORQUE_LOW = {2.25, -2.25, -2.25, 2.25, -2.25, 2.25};
const std::vector<double> SENSOR_BIAS = {2500, 2500, 2500, 2500, 2500, 2500};
const std::vector<double> SENSOR_AMPLIFY = {1, 1, 1, 1, 1, 1};
const std::vector<double> RATED_TORQUE = {0.63, 0.63, 0.55, 0.2, 0.2, 0.2};

// ProtectParams
const std::vector<double> SWITCH_THRESHOLD_OF_TORQUE_CONTROL = {25.0, 25.0, 25.0, 25.0, 25.0, 25.0};

}  // namespace XMS4

namespace XMS5 {

// ModelParams
const string ROBOT_NAME = "XMS5-R800-B4G1A4C";
const unsigned int AXIS_NUM = 6;
const double MAX_LOAD = 5;  // kg
const std::vector<double> LINK_MASS = {3.167, 4.291, 2.018, 2.005, 1.507, 0.634};
const std::vector<double> LINK_CENTROID = {0.011, -7.992, 235.842, 16.666, -69.301, 184.671, 24.9,  9.986, 83.739,
                                           0.066, -8.515, -68.865, 0.003,  29.173,  7.019,   0.073, 0.079, -44.853};  //质心(mm)
const std::vector<double> LINK_MOMENT_OF_INERTIA = {
    31167.987, 30712.71,   1575088.602, -0.648,   -4.273,   -2236.558, 140425.151, 1709613.182, 12647.754,  390.088, 14948.192, -2552.767,
    21038.419, 840115.068, 4293.575,    -483.905, 4026.647, -1704.899, 13251.186,  12556.84,    231634.848, 0.608,   2.782,     -1157.307,
    5032.625,  231504.418, 3944.803,    -6.075,   -1.293,   -331.648,  997.016,    1002.53,     229337.948, -1.376,  0.897,     1.581};  //总惯量(kg.mm**2)
const std::vector<double> LINK_MOMENT_OF_INERTIA_LOW = {
    31167.987, 30712.71,  5877.402, -0.648,   -4.273,   -2236.558, 140425.151, 140401.982, 12647.754, 390.088, 14948.192, -2552.767,
    21038.419, 21676.668, 4293.575, -483.905, 4026.647, -1704.899, 13251.186,  12556.84,   2744.848,  0.608,   2.782,     -1157.307,
    5032.625,  2614.418,  3944.803, -6.075,   -1.293,   -331.648,  997.016,    1002.53,    447.948,   -1.376,  0.897,     1.581};  //纯连杆惯量(kg.mm**2)
const std::vector<double> ROBOT_DIMENSIONS = {0.0,   0.0, 0.0,    0.0, 0.0, 328.0, 50.0,  0.0, 400.0, 50.0, 0.0,
                                              400.0, 0.0, -136.0, 0.0, 0.0, 0.0,   103.5, 0.0, 0.0,   0.0};

const std::vector<double> JOINT_RANGE_MIN_CUSTOMIZE = {-175, -160, -170, -175, -175, -175};
const std::vector<double> JOINT_RANGE_MAX_CUSTOMIZE = {175, 150, 140, 175, 175, 175};
const std::vector<double> JOINT_RANGE_MIN_NEW = {-360, -160, -170, -360, -360, -360};
const std::vector<double> JOINT_RANGE_MAX_NEW = {360, 150, 140, 360, 360, 360};

// ControlParams
const std::vector<double> CTRL_BANDWIDTH_SERVO_EXEC = {20.0, 30.0, 30.0, 20.0, 15.0, 25.0};  //拖动带宽
const std::vector<double> CTRL_ZETA_SERVO_EXEC = {0.7, 0.7, 0.7, 0.5, 0.4, 0.5};             // 拖动阻尼比
const std::vector<double> FRICTION_COF_DRAG = {0.7, 0.7, 0.7, 0.7, 0.7, 0.7};                //拖动摩擦力系数

// MechanicalParams
const std::vector<int> ENCODER_OFFESET = {0, 0, 0, 0, 0, 0};
const std::vector<int> ENCODER_RESOLUTION = {262144, 262144, 262144, 262144, 262144, 262144};
const std::vector<double> REDUCTION_RATIO_NUMERATOR = {120, -120, -120, 100, -100, 100};
const std::vector<double> REDUCTION_RATIO_DENOMINATOR = {1, 1, 1, 1, 1, 1};
const std::vector<double> SENSOR_ANALOG_TO_TORQUE_HIGH = {110, 110, 60, 25, 25, 25};
const std::vector<double> SENSOR_ANALOG_TO_TORQUE_LOW = {2.25, -2.25, -2.25, 2.25, -2.25, 2.25};
const std::vector<double> SENSOR_BIAS = {2500, 2500, 2500, 2500, 2500, 2500};
const std::vector<double> SENSOR_AMPLIFY = {1, 1, 1, 1, 1, 1};
const std::vector<double> RATED_TORQUE = {0.63, 0.63, 0.55, 0.2, 0.2, 0.2};

// ProtectParams
const std::vector<double> SWITCH_THRESHOLD_OF_TORQUE_CONTROL = {25.0, 25.0, 25.0, 25.0, 25.0, 25.0};

}  // namespace XMS5

   }  // namespace RokaeApi