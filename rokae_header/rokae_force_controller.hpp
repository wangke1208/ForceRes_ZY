/**
 * Copyright(C) 2024 Rokae Technology Co., Ltd.
 * All Rights Reserved.
 *
 * Information in this file is the intellectual property of Rokae Technology Co., Ltd,
 * And may contains trade secrets that must be stored and viewed confidentially.
 *
 * @file: rokae_force_controller.hpp
 * @date: 2025/4/11
 * @brief: 力控对外接口（单头）：仅标准库 + Eigen，不 #include 本仓库其它 rokae_header。
 *         SDK 客户与本仓库示例均包含本文件即可，与 libforce_res.a 链接；实现见 rokae_src/rokae_force_controller.cpp。
 */

#ifndef ROKAE_HEADER_ROKAE_FORCE_CONTROLLER_HPP_
#define ROKAE_HEADER_ROKAE_FORCE_CONTROLLER_HPP_

#include <Eigen/Dense>
#include <array>
#include <cstdint>
#include <vector>

namespace RokaeApi {
namespace External {
// ================== 数据结构定义 ==================

enum External_MechUnitType { AR5_L, AR5_R, AR5C_L, AR5C_R };

enum External_DragType {
    DRAG_JOINT,       //轴空间拖动（目前只支持这一种）
    DRAG_CART_TRANS,  //笛卡尔空间仅平移
    DRAG_CART_ROT,    //笛卡尔空间仅旋转
    IMPEDANCE_JOINT,  //关节阻抗
    IMPEDANCE_CART    //笛卡尔阻抗
};
struct External_RokaeLoad {
    //坐标系参数
    std::array<double, 3> position_offset{0.0, 0.0, 0.0};  //相对于法兰坐标系中心的偏移量，单位:m
    std::array<double, 3> posture_rpy{0.0, 0.0, 0.0};      //相对于法兰坐标系的旋转欧拉角(RPY)，单位:度
    //动力学参数
    double mass{0.0};                                             // 质量，单位kg
    std::array<double, 3> centroid{0.0, 0.0, 0.0};                // 质心，单位m
    std::array<double, 6> inertia{0.0, 0.0, 0.0, 0.0, 0.0, 0.0};  // 惯性矩阵，单位kg·m² Ixx Iyy Izz Ixy Ixz Iyz
};
// ================== 初始化整体模块 ==================

/**
 * @brief 初始化力控模块
 * @param [in] robot_type 机器人类型，参考枚举External_MechUnitType
 * @return 错误码，成功返回SOLVE_NOERROR，失败返回相应的错误码
 */
int RokaeForce_Init(const External_MechUnitType& robot_type);

/**
 * @brief 反初始化力控模块，释放相关资源
 * 此函数用于在完成力控操作后，释放初始化过程中分配的资源，
 * 重置相关状态，以便后续再次初始化或结束程序。
 */
void RokaeForce_Deinit();

// ================== 力控算法接口 ==================

// 3.1 拖动配置
/**
 * @brief 配置拖动功能
 * @param [in] PDO_0x6064 相关的PDO_0x6064数据向量
 * @param [in] PDO_0x6061 相关的PDO_0x6061数据向量
 * @param [in] PDO_0x2401 相关的PDO_0x2401数据向量
 * @param [in] PDO_0x2402 相关的PDO_0x2402数据向量
 * @param [in] drag_type 拖动类型
 * @param [in] is_command_by_user 力矩指令是否由用户下发
 * @return 错误码
 */
int RokaeForce_DragConfig(const std::vector<int32_t>& PDO_0x6064, const std::vector<int8_t>& PDO_0x6061,
                          const std::vector<int16_t>& PDO_0x2401, const std::vector<int16_t>& PDO_0x2402,
                          const External_DragType& drag_type, const bool& is_command_by_user);

/**
 * @brief 力控更新接口，用于更新力控相关的数据和状态
 * @param [in] PDO_0x6061 相关的PDO_0x6061数据向量
 * @param [in] PDO_0x2401 相关的PDO_0x2401数据向量
 * @param [in] PDO_0x2402 相关的PDO_0x2402数据向量
 * @param [in] PDO_0x2406 相关的PDO_0x2406数据向量
 * @param [in] PDO_0x6064 相关的PDO_0x6064数据向量
 * @param [in] PDO_0x606C 相关的PDO_0x606C数据向量
 * @param [in] jnt_pos_cmd_from_user  用户输入的关节阻抗位置指令(仅关节阻抗生效)
 * @param [in] cart_pos_cmd_from_user 用户输入的笛卡尔阻抗位置指令(仅笛卡尔阻抗生效)
 * @param [in] jnt_trq_cmd_from_user  用户输入的关节力矩指令
 * @param [out] PDO_0x6071 输出的PDO_0x6071数据向量
 * @param [out] PDO_0x60B2 输出的PDO_0x60B2数据向量
 * @param [out] PDO_0x2201 输出的PDO_0x2201数据向量
 * @param [out] PDO_0x2202 输出的PDO_0x2202数据向量
 * @param [out] PDO_0x2203 输出的PDO_0x2203数据向量
 * @param [out] PDO_0x2204 输出的PDO_0x2204数据向量
 * @param [out] PDO_0x2205 输出的PDO_0x2205数据向量
 * @param [out] PDO_0x2206 输出的PDO_0x2206数据向量
 * @return 错误码
 */
int RokaeForce_FcUpdate(const std::vector<int8_t>& PDO_0x6061, const std::vector<int16_t>& PDO_0x2401,
                        const std::vector<int16_t>& PDO_0x2402, const std::vector<int16_t>& PDO_0x2406,
                        const std::vector<int32_t>& PDO_0x6064, const std::vector<int32_t>& PDO_0x606C,
                        const std::vector<double>& jnt_pos_cmd_from_user, const std::array<double, 6>& cart_pos_cmd_from_user,
                        const std::vector<double>& jnt_trq_cmd_from_user, std::vector<int16_t>& PDO_0x6071,
                        std::vector<int16_t>& PDO_0x60B2, std::vector<int16_t>& PDO_0x2201, std::vector<int16_t>& PDO_0x2202,
                        std::vector<int16_t>& PDO_0x2203, std::vector<int16_t>& PDO_0x2204, std::vector<int16_t>& PDO_0x2205,
                        std::vector<int16_t>& PDO_0x2206);

/**
 * @brief 停止力控，重置内部状态
 * @param [in] PDO_0x6061 相关的PDO_0x6061数据向量
 * @return 错误码
 */
int RokaeForce_FcStop(const std::vector<int8_t>& PDO_0x6061);
// ================== 参数设置接口(不允许实时设置) ==================

/**
 * @brief 设置传感器线性度
 * @param[in] PDO_0x6061
 * @param[in] analog2trq_low 传感器线性度
 * @return 错误码
 */
int RokaeForce_SetSensorLinearity(const std::vector<int8_t>& PDO_0x6061, const std::vector<double>& analog2trq_low);

/**
 * @brief 设置传感器零点
 * @param[in] PDO_0x6061
 * @param[in] analog_bias 传感器零点
 * @return
 */
int RokaeForce_SetSensorBias(const std::vector<int8_t>& PDO_0x6061, const std::vector<double>& analog_bias);

/**
 * @brief 设置编码器零点
 * @param[in] PDO_0x6061
 * @param[in] encoder_offset 编码器零点
 * @return
 */
int RokaeForce_SetEncoderOffset(const std::vector<int8_t>& PDO_0x6061, const std::vector<int32_t>& encoder_offset);

/**
 * @brief 设置传感器动态补偿系数
 * @param[in] servo_mode 伺服模式
 * @param[in] dynamic_sensor_bias_baseline 传感器基准电压
 * @param[in] pos_sensor_fix_params 正向拟合参数
 * @param[in] neg_sensor_fix_params 负向拟合参数
 * @return
 */
int RokaeForce_SetSensorFixParams(const std::vector<int8_t>& servo_mode, const std::vector<double>& dynamic_sensor_bias_baseline,
                                  const std::vector<double>& pos_sensor_fix_params,
                                  const std::vector<double>& neg_sensor_fix_params);

/**
 * @brief 设置传感器动态补偿开关
 * @param[in] servo_mode 伺服模式
 * @param[in] is_support_sensor_fix 是否允许开启传感器动态补偿
 * @return
 */
int RokaeForce_SetSensorDynamicFixSwitch(const std::vector<int8_t>& servo_mode, const std::vector<bool>& is_support_sensor_fix);

/**
 * @brief 设置关节软限位
 * @param[in] PDO_0x6061
 * @param[in] joint_range_min 关节最小范围
 * @param[in] joint_range_max 关节最大范围
 * @return
 */
int RokaeForce_SetSoftLimit(const std::vector<int8_t>& PDO_0x6061, const std::vector<double>& joint_range_min,
                            const std::vector<double>& joint_range_max);

/**
 * @brief 设置力控负载
 * @param[in] PDO_0x6061
 * @param[in] load 力控负载信息
 * @return
 */
int RokaeForce_SetFcLoad(const std::vector<int8_t>& PDO_0x6061, const External_RokaeLoad& load);

/**
 * @brief 力控增益滑条系数
 * @param[in] PDO_0x6061
 * @param[in] kp_gain_set 拖动增益系数
 * @return
 */
int RokaeForce_SetKpGain(const std::vector<int8_t>& PDO_0x6061, const std::vector<double>& kp_gain_set);

/**
 * @brief 设置摩擦增益系数
 * @param[in] PDO_0x6061
 * @param[in] fric_gain_set 摩擦增益系数
 * @return
 */
int RokaeForce_SetFricGain(const std::vector<int8_t>& PDO_0x6061, const std::vector<double>& fric_gain_set);

/**
 * @brief 设置关节阻抗刚度
 * @param[in] PDO_0x6061
 * @param[in] cartesian_impedance 关节阻抗刚度
 * @return
 */
int RokaeForce_SetJointImpedance(const std::vector<int8_t>& PDO_0x6061, const std::vector<double>& joint_impedance_stiffness);

/**
 * @brief 设置笛卡尔阻抗刚度
 * @param[in] PDO_0x6061
 * @param[in] cartesian_impedance_stiffness 笛卡尔阻抗刚度
 * @return
 */
int RokaeForce_SetCartesianImpedance(const std::vector<int8_t>& PDO_0x6061,
                                     const std::array<double, 6>& cartesian_impedance_stiffness);
// ================== 参数设置接口(允许实时设置) ==================

/**
 * @brief 设置基坐标系和重力矢量
 * @param[in] base_poseture 基坐标系方向(x,y,z,a,b,c)单位：x、y、z m，a、b、c 度
 * @return
 */
int RokaeForce_SetBaseFrameAndGravity(const std::array<double, 6>& base_poseture);

// ================== 单独计算接口(可自行输入数据) ==================
/**
 * @brief 获取关节位置
 * @param[in] PDO_0x6064 PDO_0x6064
 * @param[out] jnt_pos_rad 关节位置（弧度）
 * @return
 */
int RokaeForce_GetAxisPos(const std::vector<int>& PDO_0x6064, std::vector<double>& jnt_pos_rad);

/**
 * @brief 获取关节速度
 * @param[in] PDO_0x606C PDO_0x606C
 * @param[out] jnt_vel_rad 关节速度（弧度/秒）
 * @return
 */
int RokaeForce_GetAxisVel(const std::vector<int>& PDO_0x606C, std::vector<double>& jnt_vel_rad);

/**
 * @brief 获取协作机器人传感器反馈扭矩
 * @param[in] PDO_0x2401 PDO_0x2401
 * @param[in] PDO_0x2402 PDO_0x2402
 * @param[out] jnt_trq_feedback 关节扭矩反馈
 * @return
 */
int RokaeForce_GetCobotTrq(const std::vector<int16_t>& PDO_0x2401, const std::vector<int16_t>& PDO_0x2402,
                           std::vector<double>& jnt_trq_feedback);

/**
 * @brief 获取 TCP 力和扭矩
 * @param[in] load 负载信息
 * @param[in] jnt_pos 关节位置(rad/s)
 * @param[in] jnt_trq_feedback 关节扭矩反馈(N.m)
 * @param[in] jnt_gra_trq 关节重力扭矩(N.m)
 * @param[out] ext_force 外部力和扭矩(N & N.m)
 */
void RokaeForce_GetTcpWrench(const External_RokaeLoad& load, const std::vector<double>& jnt_pos,
                             const std::vector<double>& jnt_trq_feedback, const std::vector<double>& jnt_gra_trq,
                             std::array<double, 6>& ext_force);

/**
 * @brief 获取重力扭矩
 * @param[in] load 负载参数
 * @param[in] jnt_pos 关节位置
 * @param[out] trq_gravity 重力矩
 * @return 错误码
 */
int RokaeForce_GetGraTorque(const External_RokaeLoad& load, const std::vector<double>& jnt_pos, std::vector<double>& trq_gravity);

/**
 * @brief 获取惯性扭矩
 * @param[in] load 负载参数
 * @param[in] jnt_pos 关节位置
 * @param[in] jnt_acc 关节加速度
 * @param[out] trq_inertia 惯性力矩
 * @return 错误码
 */
int RokaeForce_GetInertTorque(const External_RokaeLoad& load, const std::vector<double>& jnt_pos,
                              const std::vector<double>& jnt_acc, std::vector<double>& trq_inertia);

/**
 * @brief 获取科里奥利扭矩
 * @param[in] load 负载参数
 * @param[in] jnt_pos 关节位置
 * @param[in] jnt_vel 关节速度
 * @param[out] trq_coriolis 科氏力矩
 * @return 错误码
 */
int RokaeForce_GetCoriolisTorque(const External_RokaeLoad& load, const std::vector<double>& jnt_pos,
                                 const std::vector<double>& jnt_vel, std::vector<double>& trq_coriolis);

/**
 * @brief 获取动力学全力矩
 * @param[in] load 负载参数
 * @param[in] jnt_pos 关节位置
 * @param[in] jnt_vel 关节速度
 * @param[in] jnt_acc 关节加速度
 * @param[out] trq_total 动力学全力矩
 * @return 错误码
 */
int RokaeForce_GetTotalTorque(const External_RokaeLoad& load, const std::vector<double>& jnt_pos,
                              const std::vector<double>& jnt_vel, const std::vector<double>& jnt_acc,
                              std::vector<double>& trq_total);

/**
 * @brief 获取TCP位姿
 * @param[in] CartPos 目标笛卡尔空间位姿
 * @param[in] psi 目标臂角
 * @param[in] q_init 机器人初始关节角度
 * @param[out] q_out 目标关节角度
 * @return 错误码
 */
int RokaeForce_GetJointPos(const std::array<double, 16>& CartPos, const double& psi, const std::vector<double>& q_init,
                           std::vector<double>& q_out);

/**
 * @brief 获取当前臂角
 * @param[in] curJntPose 机器人当前关节角度
 * @param[out] psi 当前臂角
 * @return 错误码，参考 IkSolveRes 枚举
 */
int RokaeForce_GetCurPsi(const std::vector<double>& curJntPose, double& psi);

/**
 * @brief 获取Flan位姿
 * @param[in] jnt_pos 关节位置
 * @param[out] flanTobase_pos 法兰到基座的变换矩阵
 * @return 错误码
 */
int RokaeForce_GetFlanPos(const std::vector<double>& jnt_pos, std::array<double, 16>& flanTobase_pos);
/**
 * @brief 获取TCP位姿
 * @param[in] load 负载信息
 * @param[in] jnt_pos 关节位置
 * @param[out] toolTobase_pos 工具到基座的变换矩阵
 * @param[out] tcp_pos TCP位姿
 * @return 错误码
 */
int RokaeForce_GetTcpPos(const External_RokaeLoad& load, const std::vector<double>& jnt_pos,
                         std::array<double, 16>& toolTobase_pos, std::array<double, 6>& tcp_pos);

/**
 * @brief 从关节位置计算质量矩阵
 * @param[in] load 负载参数
 * @param[in] jnt_pos 关节位置
 * @param[out] mass_matrix 关节空间惯性矩阵
 * @return 错误码
 */
int RokaeForce_GetMassMatrix(const External_RokaeLoad& load, const std::vector<double>& jnt_pos, Eigen::MatrixXd& mass_matrix);

/**
 * @brief 获取 TCP 雅可比矩阵
 * @param[in] load 负载信息
 * @param[in] jnt_pos 关节位置
 * @param[out] jacobian TCP 雅可比矩阵
 * @return 错误码
 */
int RokaeForce_GetTcpJacobian(const External_RokaeLoad& load, const std::vector<double>& jnt_pos,
                              Eigen::Matrix<double, 6, Eigen::Dynamic>& jacobian);

// ================== 获取拖动状态下的实时数据 ==================

/**
 * @brief 获取当前关节位置
 * @param[out] jnt_pos 用于存储当前关节位置的向量，单位为角度
 * @return 错误码
 */
int RokaeForce_GetAxisPosCurrent(std::vector<double>& jnt_pos);

/**
 * @brief 获取当前关节速度
 * @param[out] jnt_vel 用于存储当前关节速度的向量，单位为角度/秒
 * @return 错误码
 */
int RokaeForce_GetAxisVelCurrent(std::vector<double>& jnt_vel);

/**
 * @brief 获取当前协作机器人关节扭矩反馈
 * @param[out] jnt_trq_feedback 关节扭矩反馈
 * @return 错误码
 */
int RokaeForce_GetCobotTrqCurrent(std::vector<double>& jnt_trq_feedback);

/**
 * @brief 获取当前 TCP（工具中心点）的力和扭矩
 * @param[out] ext_force xyz方向力，rxryrz方向扭矩，单位为N和N.m
 * @return 错误码
 */
int RokaeForce_GetTcpWrenchCurrent(std::array<double, 6>& ext_force);

/**
 * @brief 获取当前 TCP（工具中心点）的位置
 * @param[out] tcp_pos xyz方向位置，RPY欧拉角旋转角度，单位为m和度
 * @return 错误码
 */
int RokaeForce_GetTcpPosCurrent(std::array<double, 6>& tcp_pos);

/**
 * @brief 获取当前的重力扭矩、科里奥利力扭矩和惯性矩阵
 * @param[out] trq_gravity 重力矩
 * @param[out] trq_coriolis 科里奥利力扭矩
 * @param[out] mass_matrix 惯性矩阵
 * @return 错误码
 */
int RokaeForce_GetDynamicTorqueCurrent(std::vector<double>& trq_gravity, std::vector<double>& trq_coriolis,
                                       Eigen::MatrixXd& mass_matrix);

/**
 * @brief 获取当前的 TCP 雅可比矩阵
 * @param[out] jacobian 用于存储当前 TCP 雅可比矩阵的 Eigen 矩阵
 * @return 错误码
 */
int RokaeForce_GetJacobianCurrent(Eigen::Matrix<double, 6, Eigen::Dynamic>& jacobian);

// ================== 其他功能 ==================
/**
 * @brief 校准扭矩传感器
 * @param[in] PDO_0x6061 PDO_0x6061
 * @param[in] PDO_0x6064 PDO_0x6064
 * @param[in] load_input 输入：负载信息
 * @param[in] PDO_0x2401 PDO_0x2401数据构成的数组，每个数组包含200个元素
 * @param[in] PDO_0x2402 PDO_0x2402数据构成的数组，每个数组包含200个元素
 * @param[out] sensor_bias 输出：计算得到的传感器零点
 * @return 输出：成功返回 SOLVE_NOERROR，失败返回相应的错误码
 */
int RokaeForce_CalibrateTrqSensor(const std::vector<int8_t>& PDO_0x6061, const std::vector<int32_t>& PDO_0x6064,
                                  const External_RokaeLoad& load_input, const std::vector<std::array<int16_t, 200>>& PDO_0x2401,
                                  const std::vector<std::array<int16_t, 200>>& PDO_0x2402, std::vector<double>& sensor_bias);
}  // namespace External

}  // namespace RokaeApi
#endif
