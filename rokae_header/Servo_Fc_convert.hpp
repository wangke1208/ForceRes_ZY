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

/**
 * @class Axis_Convert
 * @brief 轴转换类，负责伺服电机和力控模块之间的数据转换。
 */
class Axis_Convert {
    const int SENSOR_ZERO_POINT_MAX = 3750;  ///< 传感器零点最大值
    const int SENSOR_ZERO_POINT_MIN = 1250;  ///< 传感器零点最小值

   public:
    /**
     * @brief 默认构造函数
     */
    Axis_Convert(){};

    /**
     * @brief 析构函数
     */
    virtual ~Axis_Convert(){};

    /**
     * @brief 带参数的构造函数
     * @param [in] axis_num 轴数
     * @param [in] mec_params_input 机械参数输入
     */
    Axis_Convert(unsigned int axis_num, const Model::MechanicalParams& mec_params_input);

    /**
     * @brief 根据关节角度获取编码器值
     * @param [in] jnt_pos_rad 输入的关节位置（单位：弧度）
     * @param [out] encoder_value 输出的编码器值
     * @return 错误码
     */
    int GetEncoderValue(const std::vector<double>& jnt_pos_rad, std::vector<int>& encoder_value);

    /**
     * @brief 根据编码器值获取关节位置
     * @param [in] encoder_value 输入的编码器值
     * @param [out] jnt_pos_rad 输出的关节位置（单位：弧度）
     * @return 错误码
     */
    int GetAxisPos(const std::vector<int>& encoder_value, std::vector<double>& jnt_pos_rad);

    /**
     * @brief 根据编码器值获取关节位置（使用KDL的JntArray）
     * @param [in] encoder_value 输入的编码器值
     * @param [out] jnt_pos_rad 输出的关节位置（单位：弧度）
     * @return 错误码
     */
    int GetAxisPos(const std::vector<int>& encoder_value, KDL::JntArray& jnt_pos_rad);

    /**
     * @brief 设置编码器偏移量
     * @param [in] encoder_bias_set 编码器偏移量集合
     * @return 错误码
     */
    int SetEncoderBias(const std::vector<int>& encoder_bias_set);

    /**
     * @brief 根据关节速度获取伺服电机的速度编码器值
     * @param [in] axis_vel_rad 输入的关节速度（单位：弧度/秒）
     * @param [out] vel_reg_value 伺服电机的速度编码器值
     * @return 错误码
     */
    int GetVelRegValueForServo(const std::vector<double>& axis_vel_rad, std::vector<int16_t>& vel_reg_value);

    /**
     * @brief 根据编码器的速度值获取关节速度
     * @param [in] encoder_vel_value 输入的编码器速度值
     * @param [out] jnt_vel_rad 输出的关节速度（单位：弧度/秒）
     * @return 错误码
     */
    int GetAxisVel(const std::vector<int>& encoder_vel_value, std::vector<double>& jnt_vel_rad);

    /**
     * @brief 根据编码器的速度值获取关节速度（使用KDL的JntArray）
     * @param [in] encoder_vel_value 输入的编码器速度值
     * @param [out] jnt_vel_rad 输出的关节速度（单位：弧度/秒）
     * @return 错误码
     */
    int GetAxisVel(const std::vector<int>& encoder_vel_value, KDL::JntArray& jnt_vel_rad);

    /**
     * @brief 获取机器人关节的传感器力矩
     * @param [in] analog_ch1 模拟通道1的输入值
     * @param [in] analog_ch2 模拟通道2的输入值
     * @param [out] jnt_sensor_feedback 输出的关节传感器反馈力矩
     * @return 错误码
     */
    int GetCobotTrq(const std::vector<int16_t>& analog_ch1, const std::vector<int16_t>& analog_ch2,
                    std::vector<double>& jnt_sensor_feedback);

    /**
     * @brief 获取机器人关节的传感器力矩（使用KDL的JntArray）
     * @param [in] analog_ch1 模拟通道1的输入值
     * @param [in] analog_ch2 模拟通道2的输入值
     * @param [out] jnt_sensor_feedback 输出的关节传感器反馈力矩
     * @return 错误码
     */
    int GetCobotTrq(const std::vector<int16_t>& analog_ch1, const std::vector<int16_t>& analog_ch2,
                    KDL::JntArray& jnt_sensor_feedback);

    /**
     * @brief 设置传感器的零点偏移量
     * @param [in] sensor_bias_set 传感器零点偏移量集合
     * @return 错误码
     */
    int SetSensorBias(const std::vector<double>& sensor_bias_set);

    /**
     * @brief 设置传感器的线性度
     * @param [in] analog_low_set 传感器低值的集合
     * @return 错误码
     */
    int SetSensorLinearity(const std::vector<double>& analog_low_set);

    /**
     * @brief 获取传感器的偏移量
     * @param [in] trq_gra_jntarray 输入的力矩引导关节数组
     * @param [in] analog_average 输入的模拟平均值
     * @param [out] analog_bias 输出的传感器偏移量
     * @return 错误码
     */
    int GetAnalogBias(const KDL::JntArray& trq_gra_jntarray, const std::vector<double>& analog_average,
                      std::vector<double>& analog_bias);

   protected:
    unsigned m_axis_num;  ///< 轴数

    // 电机相关转换
    std::vector<int> m_motorside_encoder_offset;        ///< 关节端编码器偏移量
    std::vector<int> m_motorside_encoder_resolution;    ///< 关节端编码器分辨率
    std::vector<double> m_motorside_reduce_ratio;       ///< 关节端编码器减速比
    std::vector<double> m_motorside_reduce_retio_high;  ///< 关节端编码器减速比高
    std::vector<double> m_motorside_reduce_retio_low;   ///< 关节端编码器减速比低
    std::vector<double> m_rated_torque;                 ///< 额定转矩

    std::vector<double> m_jnt_to_encoder_scale;  ///< 关节弧度到编码器的转换比
    std::vector<double> m_encoder_to_jnt_scale;  ///< 编码器到关节弧度的转换比

    // 传感器相关转换
    std::vector<double> m_analog2trq_high;  ///< 传感器量程
    std::vector<double> m_analog2trq_low;   ///< 传感器线性度
    std::vector<double> m_analog2trq;       ///< 模拟量到力矩的转换比
    std::vector<double> m_analog_bias;      ///< 传感器偏移量（零点）
    std::vector<double> m_sensor_amplify;   ///< 放大系数（目前默认为1）
    std::vector<double> m_sensor_trq;       ///< 转换后的力矩值
};

/**
 * @class Servo_Fc_Convert
 * @brief 伺服力控转换类，继承自Axis_Convert，负责伺服数据和力控模块数据之间的转换。
 */
class Servo_Fc_Convert : public Axis_Convert {
   public:
    /**
     * @brief 默认构造函数
     */
    Servo_Fc_Convert() : Axis_Convert(){};

    /**
     * @brief 析构函数
     */
    ~Servo_Fc_Convert() override{};

    /**
     * @brief 带参数的构造函数
     * @param [in] axis_num 轴数
     * @param [in] mec_params_input 机械参数输入
     */
    Servo_Fc_Convert(unsigned int axis_num, const Model::MechanicalParams& mec_params_input)
        : Axis_Convert(axis_num, mec_params_input){};

    /**
     * @brief 将伺服数据转换为FcInner数据
     * @param [in] pdo_mode_operation_0x6061 伺服操作模式PDO数据
     * @param [in] pdo_analog_ch1_0x2401 模拟通道1的PDO数据
     * @param [in] pdo_analog_ch2_0x2402 模拟通道2的PDO数据
     * @param [in] pdo_trq_feedback_0x2406 力矩反馈PDO数据
     * @param [in] pdo_pos_feedback_0x6064 位置反馈PDO数据
     * @param [in] pdo_vel_feedback_0x606C 速度反馈PDO数据
     * @param [out] servo_data_fcinner 输出的FcInner数据
     * @return 错误码
     */
    int ServoData2FcInner(const std::vector<int8_t>& pdo_mode_operation_0x6061, const std::vector<int16_t>& pdo_analog_ch1_0x2401,
                          const std::vector<int16_t>& pdo_analog_ch2_0x2402, const std::vector<int16_t>& pdo_trq_feedback_0x2406,
                          const std::vector<int32_t>& pdo_pos_feedback_0x6064,
                          const std::vector<int32_t>& pdo_vel_feedback_0x606C, Control::Servo_To_FcInner& servo_data_fcinner);

    /**
     * @brief 将FcInner数据转换为伺服数据
     * @param [in] fc_status_inner 输入的FcStatusInner数据
     * @param [in] fc_params_inner 可选的FcParamsInner数据
     * @param [out] fc_inner_to_servo 输出的伺服数据
     */
    void FcData2ServoData(const Control::FcStatusInner& fc_status_inner, const Control::FcParamsInner* fc_params_inner,
                          Control::FcInner_To_Servo& fc_inner_to_servo);

   private:
    std::vector<int16_t> m_zero_feedforward_trq;  ///< 前馈力矩(力矩模式前馈目前给0)
};

}  // namespace RokaeApi

#endif