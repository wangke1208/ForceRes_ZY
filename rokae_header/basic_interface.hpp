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

#include <memory>

#include "rokae_header/data_structure_convert.hpp"
#include "rokae_header/force_control.hpp"
#include "rokae_header/inverse_kinematics_solver.hpp"
#include "rokae_header/robot_config.hpp"
#include "rokae_header/version.hpp"

namespace RokaeApi {
namespace BasicInterface {

/**
 * @brief 初始化整个系统，包括参数模块、力控模块等
 * @param robot_type 机器人的类型
 * @return 初始化结果，参考 SolverRes 枚举
 */
int InitInterface(const Model::MechUnitType& robot_type);

/**
 * @brief 清空初始化数据，释放空间
 * @return 错误码
 */

void DeinitInterface();

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
               const std::vector<int16_t>& analog_ch1, const std::vector<int16_t>& analog_ch2, const Control::DragType& drag_type,
               const bool& is_command_by_user);

/**
 * @brief 更新力控数据
 * @param[in] servo_mode_from_servo 来自伺服的伺服模式数据(PDO_0x6061)
 * @param[in] pdo_analog_ch1 PDO模拟信号通道1的数据(PDO_0x2401)
 * @param[in] pdo_analog_ch2 PDO模拟信号通道2的数据(PDO_0x2402)
 * @param[in] trq_encoder_from_servo 来自伺服的扭矩编码器数据(PDO_0x2406)
 * @param[in] pos_encoder_from_servo 来自伺服的位置编码器数据(PDO_0x6064)
 * @param[in] vel_encoder_from_servo 来自伺服的速度编码器数据(PDO_0x606C)
 * @param [in] jnt_pos_cmd_from_user  用户输入的关节阻抗位置指令(仅关节阻抗生效)
 * @param [in] cart_pos_cmd_from_user 用户输入的笛卡尔阻抗位置指令(仅笛卡尔阻抗生效)
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
             const std::vector<double>& jnt_pos_cmd_from_user, const std::array<double, 6>& cart_pos_cmd_from_user,
             const std::vector<double>& jnt_trq_cmd_from_user, std::vector<int16_t>& fc_trq_cmd_to_servo,
             std::vector<int16_t>& fc_trq_feedforward_to_servo, std::vector<int16_t>& fc_kp_to_servo,
             std::vector<int16_t>& fc_kd_to_servo, std::vector<int16_t>& fc_edb_cof_to_servo,
             std::vector<int16_t>& fc_edb_o_to_servo, std::vector<int16_t>& fc_fric_cof_to_servo,
             std::vector<int16_t>& fc_jnt_inertia_to_servo);

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
int SetSensorLinearity(const std::vector<int8_t>& servo_mode, const std::vector<double>& analog2trq_low);

/**
 * @brief 设置传感器零点
 * @param[in] servo_mode 伺服模式数据
 * @param[in] analog_bias 传感器零点
 * @return
 */
int SetSensorBias(const std::vector<int8_t>& servo_mode, const std::vector<double>& analog_bias);

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
 * @brief 设置关节阻抗刚度
 * @param[in] servo_mode 伺服模式数据
 * @param[in] joint_stiffness 关节阻抗刚度
 * @return
 */
int SetJointImpedance(const std::vector<int8_t>& servo_mode, const std::vector<double>& joint_stiffness);

/**
 * @brief 设置笛卡尔阻抗刚度
 * @param[in] servo_mode 伺服模式数据
 * @param[in] cart_stiffness 笛卡尔阻抗刚度
 * @return
 */
int SetCartImpedance(const std::vector<int8_t>& servo_mode, const std::array<double, 6>& cart_stiffness);

/**
 * @brief 设置基坐标系和重力矢量
 * @param[in] base_poseture 基坐标系方向(x,y,z,a,b,c)
 * @return
 */
int SetBaseFrameAndGravity(const std::array<double, 6>& base_poseture);

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
                       const RokaeLoad& load_input, const std::vector<std::array<int16_t, 200>>& analog_array_ch1,
                       const std::vector<std::array<int16_t, 200>>& analog_array_ch2, std::vector<double>& sensor_bias);
/**
 * @brief 获取算法库版本
 * @return 输出：算法库版本号
 */
const char* GetVersion();

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
void GetTcpWrench(const RokaeLoad& load, const std::vector<double>& jnt_pos, const std::vector<double>& jnt_trq_feedback,
                  const std::vector<double>& jnt_gra_trq, std::array<double, 6>& ext_force);

/**
 * @brief 获取重力扭矩
 * @param[in] load_params 负载参数
 * @param[in] q 关节位置，使用 std::vector<double> 表示
 * @param[out] trq_gravity 重力矩，使用 std::vector<double> 存储
 * @return 错误码，参考 SolverRes 枚举
 */
int GetGraTorque(const RokaeLoad& load_params, const std::vector<double>& q, std::vector<double>& trq_gravity);

/**
 * @brief 获取惯性扭矩
 * @param[in] load_params 负载参数
 * @param[in] q 关节位置，使用 std::vector<double> 表示
 * @param[in] ddq 关节加速度，使用 std::vector<double> 表示
 * @param[out] trq_inertia 惯性力矩，使用 std::vector<double> 存储
 * @return 错误码，参考 SolverRes 枚举
 */
int GetInertTorque(const RokaeLoad& load_params, const std::vector<double>& q, const std::vector<double>& ddq,
                   std::vector<double>& trq_inertia);

/**
 * @brief 获取科里奥利扭矩
 * @param[in] load_params 负载参数
 * @param[in] q 关节位置，使用 std::vector<double> 表示
 * @param[in] dq 关节速度，使用 std::vector<double> 表示
 * @param[out] trq_coriolis 科氏力矩，使用 std::vector<double> 存储
 * @return 错误码，参考 SolverRes 枚举
 */
int GetCoriolisTorque(const RokaeLoad& load_params, const std::vector<double>& q, const std::vector<double>& dq,
                      std::vector<double>& trq_coriolis);

/**
 * @brief 获取动力学全力矩
 * @param[in] load_params 负载参数
 * @param[in] q 关节位置，使用 std::vector<double> 表示
 * @param[in] dq 关节速度，使用 std::vector<double> 表示
 * @param[in] ddq 关节加速度，使用 std::vector<double> 表示
 * @param[out] trq_total 动力学全力矩，使用 std::vector<double> 存储
 * @return 错误码，参考 SolverRes 枚举
 */
int GetTotalTorque(const RokaeLoad& load_params, const std::vector<double>& q, const std::vector<double>& dq,
                   const std::vector<double>& ddq, std::vector<double>& trq_total);

/**
 * @brief 获取 TCP 位置
 * @param[in] load 负载信息
 * @param[in] jnt_pos 关节位置，使用 std::vector<double> 表示
 * @param[out] tcp_pos TCP 位置，使用 std::array<double, 6> 存储
 * @return 错误码，参考 SolverRes 枚举
 */
int GetTcpPos(const RokaeLoad& load, const std::vector<double>& jnt_pos, std::array<double, 6>& tcp_pos);

/**
 * @brief 获取运动学逆解
 * @param[in] curJnt_origin 机器人初始关节角度
 * @param[in] target_Flan 目标frame和psi信息
 * @param[out] OutJointPose 目标位置对应的关节角度
 * @return 错误码，参考 IkSolveRes 枚举
 */
int GetJointPos(const std::vector<double>& curJnt_origin, const GeneralizedFrame& target_Flan, std::vector<double>& OutJointPose);

/**
 * @brief 获取运动学逆解
 * @param[in] curJntPose 机器人当前关节角度
 * @param[out] psi 当前臂角
 * @return 错误码，参考 IkSolveRes 枚举
 */
int GetCurPsi(const std::vector<double>& curJntPose, double& psi);

/**
 * @brief 从关节位置计算质量矩阵
 * @param[in] load_params 负载参数
 * @param[in] jnt_pos 关节位置
 * @param[out] mass_matrix 关节空间惯性矩阵
 * @return 错误码，参考 SolverRes 枚举
 */
int GetMassMatrix(const RokaeLoad& load_params, const std::vector<double>& jnt_pos, Eigen::MatrixXd& mass_matrix);

/**
 * @brief 获取 TCP 雅可比矩阵
 * @param[in] load 负载信息
 * @param[in] jnt_pos 关节位置
 * @param[out] jacobian TCP 雅可比矩阵
 * @return 错误码，参考 SolverRes 枚举
 */
int GetTcpJacobian(const RokaeLoad& load, const std::vector<double>& jnt_pos, Eigen::Matrix<double, 6, Eigen::Dynamic>& jacobian);
//*******************************获取实时内部状态*********************************/
/**
 * @brief 获取当前关节位置
 * @param[out] jnt_pos_rad 用于存储当前关节位置的向量，单位为角度
 * @return 错误码，参考 SolverRes 枚举
 */
int GetAxisPosCurrent(std::vector<double>& jnt_pos_rad);

/**
 * @brief 获取当前关节速度
 * @param[out] jnt_vel_rad 用于存储当前关节速度的向量，单位为角度/秒
 * @return 错误码，参考 SolverRes 枚举
 */
int GetAxisVelCurrent(std::vector<double>& jnt_vel_rad);

/**
 * @brief 获取当前协作机器人关节扭矩反馈
 * @param[out] jnt_trq_feedback 用于存储当前关节扭矩反馈的向量
 * @return 错误码，参考 SolverRes 枚举
 */
int GetCobotTrqCurrent(std::vector<double>& jnt_trq_feedback);

/**
 * @brief 获取当前 TCP（工具中心点）的力和扭矩
 * @param[out] ext_force 用于存储当前 TCP 力和扭矩的数组，长度为 6
 * @return 错误码，参考 SolverRes 枚举
 */
int GetTcpWrenchCurrent(std::array<double, 6>& ext_force);

/**
 * @brief 获取当前 TCP（工具中心点）的位置
 * @param[out] tcp_pos 用于存储当前 TCP 位置的数组，长度为 6
 * @return 错误码，参考 SolverRes 枚举
 */
int GetTcpPosCurrent(std::array<double, 6>& tcp_pos);

/**
 * @brief 获取当前的重力扭矩、科里奥利力扭矩和惯性矩阵
 * @param[out] trq_gravity 用于存储当前重力扭矩的向量
 * @param[out] trq_coriolis 用于存储当前科里奥利力扭矩的向量
 * @param[out] mass_matrix 用于存储当前惯性矩阵的 Eigen 矩阵
 * @return 错误码，参考 SolverRes 枚举
 */
int GetDynamicTorqueCurrent(std::vector<double>& trq_gravity, std::vector<double>& trq_coriolis, Eigen::MatrixXd& mass_matrix);

/**
 * @brief 获取当前的 TCP 雅可比矩阵
 * @param[out] jacobian 用于存储当前 TCP 雅可比矩阵的 Eigen 矩阵
 * @return 错误码，参考 SolverRes 枚举
 */
int GetJacobianCurrent(Eigen::Matrix<double, 6, Eigen::Dynamic>& jacobian);

/**
 * @brief 判断伺服是否处于位置模式
 * @param[in] servo_mode 伺服模式数据
 * @return true 表示处于位置模式，false 表示不处于位置模式
 */
bool IsInPositionMode(const std::vector<int8_t>& servo_mode);

}  // namespace BasicInterface

}  // namespace RokaeApi

#endif