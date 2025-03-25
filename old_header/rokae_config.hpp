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


#ifndef ROKAE_CONFIG_H
#define ROKAE_CONFIG_H

#include <string>
#include <vector>
using namespace std;

namespace RokaeApi {
namespace XMS4 {
// ModelParams
extern const string ROBOT_NAME;
extern const unsigned int AXIS_NUM;
extern const double MAX_LOAD;        

extern const std::vector<double> LINK_MASS;               // kg
extern const std::vector<double> LINK_CENTROID;               //质心(mm)
extern const std::vector<double> LINK_MOMENT_OF_INERTIA;      //总惯量(kg.mm**2)
extern const std::vector<double> LINK_MOMENT_OF_INERTIA_LOW;  //纯连杆惯量(kg.mm**2)
extern const std::vector<double> ROBOT_DIMENSIONS;
extern const std::vector<double> JOINT_RANGE_MIN_CUSTOMIZE;
extern const std::vector<double> JOINT_RANGE_MAX_CUSTOMIZE;
extern const std::vector<double> JOINT_RANGE_MIN_NEW;
extern const std::vector<double> JOINT_RANGE_MAX_NEW;

// ControlParams
extern const std::vector<double> CTRL_BANDWIDTH_SERVO_EXEC;  //拖动带宽
extern const std::vector<double> CTRL_ZETA_SERVO_EXEC;       // 拖动阻尼比
extern const std::vector<double> FRICTION_COF_DRAG;          //拖动摩擦力系数

// MechanicalParams
extern const std::vector<int> ENCODER_OFFESET;
extern const std::vector<int> ENCODER_RESOLUTION;
extern const std::vector<double> REDUCTION_RATIO_NUMERATOR;
extern const std::vector<double> REDUCTION_RATIO_DENOMINATOR;
extern const std::vector<double> SENSOR_ANALOG_TO_TORQUE_HIGH;
extern const std::vector<double> SENSOR_ANALOG_TO_TORQUE_LOW;
extern const std::vector<double> SENSOR_BIAS;
extern const std::vector<double> SENSOR_AMPLIFY;
extern const std::vector<double> RATED_TORQUE;

// ProtectParams
extern const std::vector<double> SWITCH_THRESHOLD_OF_TORQUE_CONTROL;

}  // namespace XMS4

namespace XMS5 {
// ModelParams
extern const string ROBOT_NAME;
extern const unsigned int AXIS_NUM;
extern const double MAX_LOAD;        

extern const std::vector<double> LINK_MASS;                  // kg
extern const std::vector<double> LINK_CENTROID;              //质心(mm)
extern const std::vector<double> LINK_MOMENT_OF_INERTIA;     //总惯量(kg.mm**2)
extern const std::vector<double> LINK_MOMENT_OF_INERTIA_LOW; //纯连杆惯量(kg.mm**2)
extern const std::vector<double> ROBOT_DIMENSIONS;
extern const std::vector<double> JOINT_RANGE_MIN_CUSTOMIZE;  //软限位
extern const std::vector<double> JOINT_RANGE_MAX_CUSTOMIZE;
extern const std::vector<double> JOINT_RANGE_MIN_NEW;        //硬限位
extern const std::vector<double> JOINT_RANGE_MAX_NEW;
// ControlParams
extern const std::vector<double> CTRL_BANDWIDTH_SERVO_EXEC;  //拖动带宽
extern const std::vector<double> CTRL_ZETA_SERVO_EXEC;       // 拖动阻尼比
extern const std::vector<double> FRICTION_COF_DRAG;          //拖动摩擦力系数

// MechanicalParams
extern const std::vector<int> ENCODER_OFFESET;
extern const std::vector<int> ENCODER_RESOLUTION;
extern const std::vector<double> REDUCTION_RATIO_NUMERATOR;
extern const std::vector<double> REDUCTION_RATIO_DENOMINATOR;
extern const std::vector<double> SENSOR_ANALOG_TO_TORQUE_HIGH;
extern const std::vector<double> SENSOR_ANALOG_TO_TORQUE_LOW;
extern const std::vector<double> SENSOR_BIAS;
extern const std::vector<double> SENSOR_AMPLIFY;
extern const std::vector<double> RATED_TORQUE;

// ProtectParams
extern const std::vector<double> SWITCH_THRESHOLD_OF_TORQUE_CONTROL;

}  // namespace XMS4

namespace XMS3 {
// ModelParams
extern const string ROBOT_NAME;
extern const unsigned int AXIS_NUM;
extern const double MAX_LOAD;

extern const std::vector<double> LINK_MASS;                   // kg
extern const std::vector<double> LINK_CENTROID;               //质心(mm)
extern const std::vector<double> LINK_MOMENT_OF_INERTIA;      //总惯量(kg.mm**2)
extern const std::vector<double> LINK_MOMENT_OF_INERTIA_LOW;  //纯连杆惯量(kg.mm**2)
extern const std::vector<double> ROBOT_DIMENSIONS;
extern const std::vector<double> JOINT_RANGE_MIN_CUSTOMIZE;  //软限位
extern const std::vector<double> JOINT_RANGE_MAX_CUSTOMIZE;
extern const std::vector<double> JOINT_RANGE_MIN_NEW;  //硬限位
extern const std::vector<double> JOINT_RANGE_MAX_NEW;
// ControlParams
extern const std::vector<double> CTRL_BANDWIDTH_SERVO_EXEC;  //拖动带宽
extern const std::vector<double> CTRL_ZETA_SERVO_EXEC;       // 拖动阻尼比
extern const std::vector<double> FRICTION_COF_DRAG;          //拖动摩擦力系数

// MechanicalParams
extern const std::vector<int> ENCODER_OFFESET;
extern const std::vector<int> ENCODER_RESOLUTION;
extern const std::vector<double> REDUCTION_RATIO_NUMERATOR;
extern const std::vector<double> REDUCTION_RATIO_DENOMINATOR;
extern const std::vector<double> SENSOR_ANALOG_TO_TORQUE_HIGH;
extern const std::vector<double> SENSOR_ANALOG_TO_TORQUE_LOW;
extern const std::vector<double> SENSOR_BIAS;
extern const std::vector<double> SENSOR_AMPLIFY;
extern const std::vector<double> RATED_TORQUE;

// ProtectParams
extern const std::vector<double> SWITCH_THRESHOLD_OF_TORQUE_CONTROL;

}  // namespace XMS3
}  // namespace RokaeApi

#endif