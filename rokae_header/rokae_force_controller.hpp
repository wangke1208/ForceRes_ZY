/**
 * Copyright(C) 2024 Rokae Technology Co., Ltd.
 * All Rights Reserved.
 *
 * Information in this file is the intellectual property of Rokae Technology Co., Ltd,
 * And may contains trade secrets that must be stored and viewed confidentially.
 *
 * @file: rokae_force_controller.hpp
 * @author: wangke
 * @date: 2025/3/31
 * @brief:力控对外接口
 */

#include "data_structure_define.hpp"
namespace RokaeApi {

// 1.初始化部分
int Rokae_Force_Init(const Model::MechUnitType robot_type);
int Rokae_Force_Deinit();

// 2.拖动部分
int Rokae_Force_Config_Drag(const LoadParams& load, const std::vector<double>& gravity_vector,
                            const std::vector<int32_t>& pos_encoder_from_servo, const std::vector<int8_t>& servo_mode_from_servo,
                            const std::vector<int16_t>& analog_ch1, const std::vector<int16_t>& analog_ch2,
                            const Control::DragType& drag_type);

int Rokae_Force_Calculate(const std::vector<int8_t>& servo_mode_from_servo, const std::vector<int16_t>& pdo_analog_ch1,
                          const std::vector<int16_t>& pdo_analog_ch2, const std::vector<int16_t>& trq_encoder_from_servo,
                          const std::vector<int>& pos_encoder_from_servo, const std::vector<int>& vel_encoder_from_servo,
                          std::vector<int16_t>& trq_cmd_to_servo, std::vector<int16_t>& fc_trq_feedforward_to_servo,
                          std::vector<int16_t>& fc_kp_to_servo, std::vector<int16_t>& fc_kd_to_servo,
                          std::vector<int16_t>& fc_edb_cof_to_servo, std::vector<int16_t>& fc_edb_o_to_servo,
                          std::vector<int16_t>& fc_fric_cof_to_servo, std::vector<int16_t>& fc_jnt_inertia_to_servo);

int Rokae_Force_StopDrag(const std::vector<int8_t>& param_0x6061);

// 3.参数设置部分
int Rokae_SetSensorLinearity(const std::vector<double>& analog2trq_low);                           //传感器线性度
int Rokae_SetSensorBias(const std::vector<double>& analog_bias);                                   //传感器零点
int Rokae_SetEncoderOffset(const std::vector<int32_t>& encoder_offset);                            //编码器零点
int Rokae_SetControlGain(const std::vector<double>& kp_set, const std::vector<double>& fric_set);  //带宽&摩擦力补偿
int Rokae_SetSoftLimit(const std::vector<double>& joint_range_min_input, const std::vector<double>& joint_range_max_input);
int Rokae_SetLoad(const LoadParams& load);

int Rokae_SetGravityDirection(const std::vector<double>& gravity_vector);

// 4.运动学部分
int Rokae_GetCartPositon(const LoadParams& load, std::vector<double>& joint_position, CartPos& cart_position);

// 5.动力学部分
int Rokae_GetDynamicTrq(const LoadParams& load, const std::vector<double>& jnt_pos, const std::vector<double>& jnt_vel,
                        const std::vector<double>& jnt_acc, std::vector<double>& trq_gravity, KDL::JntArray& trq_coriolis,
                        std::vector<double>& trq_inertial, std::vector<double>& trq_all);
int Rokae_GetJac(const LoadParams& load, const std::vector<double>& jnt_pos, Rokae_Jac& jac);
int Rokae_GetInertia_Matrix(const LoadParams& load, const std::vector<double>& jnt_pos, Rokae_Inertia& inertia_matrix);



// 6.状态计算
int Rokae_GetJointPositon(const std::vector<int>& pos_encoder, std::vector<double>& joint_position);
int Rokae_GetJointVel(const std::vector<int>& vel_encoder, std::vector<double>& joint_vel);
int Rokae_GetJointTrq(const std::vector<int16_t>& pdo_analog_ch1, const std::vector<int16_t>& pdo_analog_ch2,
                      std::vector<double>& joint_trq);
int Rokae_GetTcpForce(const std::vector<double>& joint_trq, Rokae_Wrench& tcp_force);

}  // namespace RokaeApi