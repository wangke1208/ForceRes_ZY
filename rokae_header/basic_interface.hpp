/**
 * Copyright(C) 2024 Rokae Technology Co., Ltd.
 * All Rights Reserved.
 *
 * Information in this file is the intellectual property of Rokae Technology Co., Ltd,
 * And may contains trade secrets that must be stored and viewed confidentially.
 *
 * @file: basic_interface.hpp
 * @author: wangke
 * @date:
 * @brief: 对外接口文件
 */

#ifndef ROKAE_HEADER_BASIC_INTERFACE_HPP_
#define ROKAE_HEADER_BASIC_INTERFACE_HPP_

#include "force_control.hpp"
#include <memory>



namespace RokaeApi {
namespace BasicInterface {

/**
 * @brief 初始化整个系统，包括参数模块、力控模块等
 * @param robot_type 机器人的类型
 * @return 初始化结果，参考 SolverRes 枚举
 */
int InitInterface(const Model::MechUnitType& robot_type);

/**
 * @brief 配置拖动相关参数
 * @param[in] pos_encoder_from_servo 来自伺服的编码器位置数据PDO_0x6064
 * @param[in] servo_mode_from_servo 来自伺服的伺服模式数据PDO_0x6061
 * @param[in] analog_ch1 传感器通道1的数据PDO_0x2401
 * @param[in] analog_ch2 传感器通道2的数据PDO_0x2402
 * @param[in] drag_type 拖动类型(轴空间、笛卡尔仅平移、笛卡尔仅旋转、笛卡尔自由)
 * @return 错误码，参考 SolverRes 枚举
 */
int DragConfig(const std::vector<int32_t>& pos_encoder_from_servo, const std::vector<int8_t>& servo_mode_from_servo,
               const std::vector<int16_t>& analog_ch1, const std::vector<int16_t>& analog_ch2, const DragType& drag_type);

/**
 * @brief 更新力控数据
 * @param[in] servo_mode_from_servo 来自伺服的伺服模式数据(PDO_0x6061)
 * @param[in] pdo_analog_ch1 PDO模拟信号通道1的数据(PDO_0x2401)
 * @param[in] pdo_analog_ch2 PDO模拟信号通道2的数据(PDO_0x2402)
 * @param[in] trq_encoder_from_servo 来自伺服的扭矩编码器数据(PDO_0x2406)
 * @param[in] pos_encoder_from_servo 来自伺服的位置编码器数据(PDO_0x6064)
 * @param[in] vel_encoder_from_servo 来自伺服的速度编码器数据(PDO_0x606C)
 *
 * @param[out] fc_trq_cmd_to_servo 伺服侧扭矩指令(PDO_0x6071)
 * @param[out] fc_trq_feedforward_to_servo 伺服侧力控扭矩前馈(PDO_0x60B2)
 * @param[out] fc_kp_to_servo 伺服侧关节力控环带宽(PDO_0x2201)
 * @param[out] fc_kd_to_servo 伺服侧关节力控环阻尼比(PDO_0x2202)
 * @param[out] fc_edb_cof_to_servo 伺服侧扭矩反馈系数(PDO_0x2203)
 * @param[out] fc_edb_o_to_servo 伺服侧扭矩反馈偏置(PDO_2204)
 * @param[out] fc_fric_cof_to_servo 伺服侧电机侧摩擦补偿系数(PDO_0x2205)
 * @param[out] fc_jnt_inertia_to_servo 伺服侧关节惯量(PDO_0x2206)
 * @return 错误码，参考 SolverRes 枚举
 */
int FcUpdate(const std::vector<int8_t>& servo_mode_from_servo, const std::vector<int16_t>& pdo_analog_ch1,
             const std::vector<int16_t>& pdo_analog_ch2, const std::vector<int16_t>& trq_encoder_from_servo,
             const std::vector<int>& pos_encoder_from_servo, const std::vector<int>& vel_encoder_from_servo,
             std::vector<int16_t>& fc_trq_cmd_to_servo, std::vector<int16_t>& fc_trq_feedforward_to_servo,
             std::vector<int16_t>& fc_kp_to_servo, std::vector<int16_t>& fc_kd_to_servo,
             std::vector<int16_t>& fc_edb_cof_to_servo, std::vector<int16_t>& fc_edb_o_to_servo,
             std::vector<int16_t>& fc_fric_cof_to_servo, std::vector<int16_t>& fc_jnt_inertia_to_servo);

/**
 * @brief 停止力控功能
 * @param[in] servo_mode 伺服模式数据
 * @return 错误码
 */
int FcStop(const std::vector<int8_t>& servo_mode);

/**
 * @brief 设置传感器线性度
 * @param[in] servo_mode 伺服模式数据
 * @param[in] analog2trq_low 传感器线性度
 * @return 错误码
 */
int SetSensorLinearity(const std::vector<int8_t>& servo_mode, const std::vector<double> analog2trq_low);

/**
 * @brief 设置传感器零点
 * @param[in] servo_mode 伺服模式数据
 * @param[in] analog_bias 传感器零点
 * @return
 */
int SetSensorBias(const std::vector<int8_t>& servo_mode, const std::vector<double> analog_bias);

/**
 * @brief 设置编码器零点
 * @param[in] servo_mode 伺服模式数据
 * @param[in] encoder_offset 编码器偏移值
 * @return
 */
int SetEncoderOffset(const std::vector<int8_t>& servo_mode, const std::vector<int32_t>& encoder_offset);

/**
 * @brief 设置关节软限位
 * @param[in] servo_mode 伺服模式数据
 * @param[in] joint_range_min 关节最小范围
 * @param[in] joint_range_max 关节最大范围
 * @return
 */
int SetSoftLimit(const std::vector<int8_t>& servo_mode, const std::vector<double>& joint_range_min,
                 const std::vector<double>& joint_range_max);

/**
 * @brief 设置最大扭矩误差阈值
 * @param[in] servo_mode 伺服模式数据
 * @param[in] m_max_trq_error_threshold 最大扭矩误差阈值
 * @return
 */
int SetMaxTrqErrorThreshold(const std::vector<int8_t>& servo_mode, const std::vector<double>& m_max_trq_error_threshold);

/**
 * @brief 设置负载限制
 * @param[in] servo_mode 伺服模式数据
 * @param[in] max_load_mass 最大负载质量
 * @param[in] max_load_tcp_length 最大负载 TCP 长度
 * @return
 */
int SetLoadLimit(const std::vector<int8_t>& servo_mode, const double& max_load_mass, const double& max_load_tcp_length);

/**
 * @brief 设置力控负载
 * @param[in] servo_mode 伺服模式数据
 * @param[in] load 力控负载信息
 * @return
 */
int SetFcLoad(const std::vector<int8_t>& servo_mode, const RokaeLoad& load);

/**
 * @brief 力控增益滑条系数
 * @param[in] servo_mode 伺服模式数据
 * @param[in] kp_gain_set KP
 * @return
 */
int SetKpGain(const std::vector<int8_t>& servo_mode, const std::vector<double>& kp_gain_set);

/**
 * @brief 设置摩擦增益系数
 * @param[in] servo_mode 伺服模式数据
 * @param[in] fric_gain_set 摩擦增益系数
 * @return
 */
int SetFricGain(const std::vector<int8_t>& servo_mode, const std::vector<double>& fric_gain_set);

/**
 * @brief 校准扭矩传感器
 * @param[in] pos_encoder_feedback 输入：编码器反馈的位置数据
 * @param[in] load_input 输入：负载信息
 * @param[in] analog_array_ch1 输入：传感器通道1的数据数组，每个数组包含200个元素
 * @param[in] analog_array_ch2 输入：传感器通道2的数据数组，每个数组包含200个元素
 * @param[out] sensor_bias 输出：计算得到的传感器零点
 * @return 输出：成功返回 SOLVE_NOERROR，失败返回相应的错误码
 */
int CalibrateTrqSensor(const std::vector<int8_t>& servo_mode, const std::vector<int32_t>& pos_encoder_feedback,
                       const RokaeLoad& load_input, const std::vector<std::array<int16_t, ANALOG_DATA_COUNT>>& analog_array_ch1,
                       const std::vector<std::array<int16_t, ANALOG_DATA_COUNT>>& analog_array_ch2,
                       std::vector<double>& sensor_bias);

/**
 * @brief 获取关节位置
 * @param[in] encoder_value 编码器值
 * @param[out] jnt_pos_rad 关节位置（弧度）
 * @return
 */
int GetAxisPos(const std::vector<int>& encoder_value, std::vector<double>& jnt_pos_rad);

/**
 * @brief 获取关节速度
 * @param[in] encoder_vel_value 编码器速度值
 * @param[out] jnt_vel_rad 关节速度（弧度/秒）
 * @return
 */
int GetAxisVel(const std::vector<int>& encoder_vel_value, std::vector<double>& jnt_vel_rad);

/**
 * @brief 获取协作机器人扭矩
 * @param[in] analog_ch1 模拟通道 1 的数据
 * @param[in] analog_ch2 模拟通道 2 的数据
 * @param[out] jnt_trq_feedback 关节扭矩反馈
 * @return
 */
int GetCobotTrq(const std::vector<int16_t>& analog_ch1, const std::vector<int16_t>& analog_ch2,
                std::vector<double>& jnt_trq_feedback);

/**
 * @brief 获取 TCP 力和扭矩
 * @param[in] load 负载信息
 * @param[in] jnt_pos 关节位置
 * @param[in] jnt_trq_feedback 关节扭矩反馈
 * @param[in] jnt_gra_trq 关节重力扭矩
 * @param[out] ext_force 外部力和扭矩
 */
void GetTcpWrench(const RokaeLoadPose& load, const std::vector<double>& jnt_pos, const std::vector<double>& jnt_trq_feedback,
                  const std::vector<double>& jnt_gra_trq, std::array<double, 6>& ext_force);

/**
 * @brief 获取重力扭矩
 * @param[in] load_params 负载参数
 * @param[in] q 关节位置
 * @return[out] 重力矩
 */
const KDL::JntArray& GetGraTorque(const RokaeLoadInertia& load_params, const KDL::JntArray& q);

/**
 * @brief 获取惯性扭矩
 * @param[in] load_params 负载参数
 * @param[in] q 关节位置
 * @param[in] ddq 关节加速度
 * @return 惯性力矩
 */
const KDL::JntArray& GetInertTorque(const RokaeLoadInertia& load_params, const KDL::JntArray& q, const KDL::JntArray& ddq);

/**
 * @brief 获取科里奥利扭矩
 * @param[in] load_params 负载参数
 * @param[in] q 关节位置
 * @param[in] dq 关节速度
 * @return 科氏力矩
 */
const KDL::JntArray& GetColioTorque(const RokaeLoadInertia& load_params, const KDL::JntArray& q, const KDL::JntArray& dq);

/**
 * @brief 获取 TCP 位置
 * @param[in] load 负载信息
 * @param[in] jnt_pos 关节位置
 * @param[out] tcp_pos TCP 位置
 */
void GetTcpPos(const RokaeLoad& load, const KDL::JntArray& jnt_pos, KDL::Frame& tcp_pos);

/**
 * @brief 从关节位置计算质量矩阵
 * @param[in] load_params 负载参数
 * @param[in] q 关节位置
 * @param[out] H 关节空间惯性矩阵
 */
void JntToMass(const RokaeLoadInertia& load_params, const KDL::JntArray& q, KDL::JntSpaceInertiaMatrix& H);

/**
 * @brief 获取 TCP 雅可比矩阵
 * @param[in] load 负载信息
 * @param[in] q 关节位置
 * @param[out] jacobian TCP 雅可比矩阵
 */
void GetTcpJacobian(const RokaeLoad& load, const KDL::JntArray& q, KDL::Jacobian& jacobian);

/**
 * @brief 判断伺服是否处于位置模式
 * @param[in] servo_mode 伺服模式数据
 * @return true 表示处于位置模式，false 表示不处于位置模式
 */
bool IsInPositionMode(const std::vector<int8_t>& servo_mode);

}  // namespace BasicInterface

}  // namespace RokaeApi

#endif