/**
 * Copyright(C) 2024 Rokae Technology Co., Ltd.
 * All Rights Reserved.
 *
 * Information in this file is the intellectual property of Rokae Technology Co., Ltd,
 * And may contains trade secrets that must be stored and viewed confidentially.
 *
 * @file: Servo_Fc_convert.hpp
 * @author: wangke
 * @date: 2025/3/27
 * @brief: 伺服力控转换接口
 */

#ifndef SERVO_FC_CONVERT_H
#define SERVO_FC_CONVERT_H

#include "rokae_header/data_structure_define.hpp"

namespace RokaeApi {

class Axis_Convert {
   public:
    Axis_Convert(){};
    ~Axis_Convert(){};
    Axis_Convert(unsigned int axis_num, const Model::MechanicalParams& mec_params_input){};

    int GetEncoderValue(const std::vector<double>& jnt_pos_rad, std::vector<int>& encoder_value);
    int GetAxisPos(const std::vector<int>& encoder_value, std::vector<double>& jnt_pos_rad);
    int SetEncoderBias(const std::vector<int>& encoder_bias_set);
    int GetVelRegValueForServo(const std::vector<double>& axis_vel_rad, std::vector<int16_t>& vel_reg_value);
    int GetAxisVel(const std::vector<int>& encoder_vel_value, std::vector<double>& jnt_vel_rad);

    int GetCobotTrq(const std::vector<int16_t>& analog_ch1, const std::vector<int16_t>& analog_ch2,
                    std::vector<double>& jnt_sensor_feedback);
    int SetSensorBias(const std::vector<double>& sensor_bias_set);
    int SetSensorLinearity(const std::vector<double>& analog_low_set);

   private:
    //电机相关转换
    unsigned m_axis_num;                                // 轴数
    std::vector<int> m_motorside_encoder_offset;        // 关节端编码器偏移量
    std::vector<int> m_motorside_encoder_resolution;    // 关节端编码器分辨率
    std::vector<double> m_motorside_reduce_retio;       //关节端编码器减速比
    std::vector<double> m_motorside_reduce_retio_high;  //关节端编码器减速比
    std::vector<double> m_motorside_reduce_retio_low;   //关节端编码器减速比
    std::vector<double> m_rated_torque;                 // 连杆端编码器偏移量

    std::vector<double> m_jnt_to_encoder_scale;  //关节弧度到编码器的值
    std::vector<double> m_encoder_to_jnt_scale;  //编码器到关节弧度

    //传感器相关转换
    std::vector<double> m_analog2trq_high;  //传感器量程
    std::vector<double> m_analog2trq_low;   //传感器线性度
    std::vector<double> m_analog2trq;       //模拟量到力矩的转换比
    std::vector<double> m_analog_bias;      //传感器偏移量(零点)
    std::vector<double> m_sensor_amplify;   //放大系数(目前均默认为1)
    std::vector<double> m_sensor_trq;       //转换后的力矩值
};

}  // namespace RokaeApi
#endif