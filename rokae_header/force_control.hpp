/**
 * Copyright(C) 2024 Rokae Technology Co., Ltd.
 * All Rights Reserved.
 *
 * Information in this file is the intellectual property of Rokae Technology Co., Ltd,
 * And may contains trade secrets that must be stored and viewed confidentially.
 *
 * @file: force_control.hpp
 * @author: wangke
 * @date: 2024/4/25
 * @brief: 力控计算模块
 */

#ifndef FORCE_CONTROL_H
#define FORCE_CONTROL_H

// 包含必要的头文件
#include "Servo_Fc_convert.hpp"
#include "data_structure_define.hpp"
#include "dynamic_solver.hpp"
#include "fc_data_computation.hpp"
#include "force_planner.hpp"
#include "force_protect.hpp"
#include "initialize.hpp"

namespace RokaeApi {
namespace Control {
/**
 * @class ForceControl
 * @brief 力控模块的计算类，负责力控相关的计算和参数管理
 *
 */
static constexpr int POSITION_MODE = 8;
static constexpr int TORQUE_MODE = 10;
static constexpr int ANALOG_DATA_COUNT = 200;

class ForceControl {
   private:
    // 常量定义
    static constexpr double DEFAULT_KP_GAIN = 1.0;
    static constexpr double DEFAULT_FRIC_GAIN = 0.5;

   public:
    //构造函数
    ForceControl(InitRobot* init_robot_ptr);

    ~ForceControl();

    // 力控功能函数
    /**
     * @brief 初始化力控模块
     *
     * 此函数用于初始化力控模块，初始化一些内部参数，包括编码器零点、传感器零点、传感器线性度、
     * 关节限位、拖动力矩限制、伺服控制参数
     *
     * @return 成功返回 SOLVE_NOERROR，失败返回相应的错误码
     */
    int Fcinit();

    /**
     * @brief 拖动功能设置
     *
     * 此函数用于配置拖动功能，主要设置一下拖动类型、内部的参数以及判断当前状态是否允许开启拖动
     *
     * @param[in] pos_encoder_from_servo 来自伺服的编码器位置数据PDO_0x6064
     * @param[in] servo_mode_from_servo 来自伺服的伺服模式数据PDO_0x6061
     * @param[in] analog_ch1 传感器通道1的数据PDO_0x2401
     * @param[in] analog_ch2 传感器通道2的数据PDO_0x2402
     * @param[in] drag_type 拖动类型(轴空间、笛卡尔仅平移、笛卡尔仅旋转、笛卡尔自由)
     *
     * @return 成功返回 SOLVE_NOERROR，失败返回相应的错误码
     */
    int DragConfig(const std::vector<int32_t>& pos_encoder_from_servo, const std::vector<int8_t>& servo_mode_from_servo,
                   const std::vector<int16_t>& analog_ch1, const std::vector<int16_t>& analog_ch2, const DragType& drag_type);

    /**
     * @brief 设置拖动指令
     *
     * 此函数用于更新并设置拖动位置和速度指令
     *
     * @param[in] servo_data_fc_inner 输入：转换后的力控内部数据
     */
    void SetFcCommand(const Servo_To_FcInner& servo_data_fc_inner);

    /**
     * @brief 力控主更新函数
     *
     * 主要用于计算最终的发送到伺服的指令
     *
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
     *
     * @return 成功返回 SOLVE_NOERROR，失败返回相应的错误码
     */
    int FcUpdate(const std::vector<int8_t>& servo_mode_from_servo, const std::vector<int16_t>& pdo_analog_ch1,
                 const std::vector<int16_t>& pdo_analog_ch2, const std::vector<int16_t>& trq_encoder_from_servo,
                 const std::vector<int>& pos_encoder_from_servo, const std::vector<int>& vel_encoder_from_servo,
                 std::vector<int16_t>& fc_trq_cmd_to_servo, std::vector<int16_t>& fc_trq_feedforward_to_servo,
                 std::vector<int16_t>& fc_kp_to_servo, std::vector<int16_t>& fc_kd_to_servo,
                 std::vector<int16_t>& fc_edb_cof_to_servo, std::vector<int16_t>& fc_edb_o_to_servo,
                 std::vector<int16_t>& fc_fric_cof_to_servo, std::vector<int16_t>& fc_jnt_inertia_to_servo);

    // 参数设置函数
    /**
     * @brief 设置传感器线性度
     *
     * 此函数用于设置传感器的线性度参数。
     *
     * @param[in] analog2trq_low 输入：传感器线性度
     * @return 输出：成功返回 SOLVE_NOERROR，失败返回相应的错误码
     */
    int SetSensorLinearity(const std::vector<double>& analog2trq_low);

    /**
     * @brief 设置传感器零点
     *
     * 此函数用于设置传感器的零点参数。
     *
     * @param[in] analog_bias 输入：传感器偏置
     * @return 输出：成功返回 SOLVE_NOERROR，失败返回相应的错误码
     */
    int SetSensorBias(const std::vector<double>& analog_bias);

    /**
     * @brief 设置机械零点
     *
     * 此函数用于设置关节位置编码器的偏移参数。
     *
     * @param[in] encoder_offset 输入：编码器的偏移参数
     * @return 输出：成功返回 SOLVE_NOERROR，失败返回相应的错误码
     */
    int SetEncoderOffset(const std::vector<int32_t>& encoder_offset);

    /**
     * @brief 设置软限位
     *
     * 此函数用于设置关节的软限位参数。(软限位要小于硬限位，同时负限位小于0，正限位大于0)
     *
     * @param[in] joint_range_min 输入：关节的最小限位参数
     * @param[in] joint_range_max 输入：关节的最大限位参数
     * @return 输出：成功返回 SOLVE_NOERROR，失败返回相应的错误码
     */
    int SetSoftLimit(const std::vector<double>& joint_range_min, const std::vector<double>& joint_range_max);

    /**
     * @brief 设置最大扭矩误差阈值
     *
     * 此函数用于设置允许开启拖动最大扭矩误差的阈值参数。
     *
     * @param[in] m_max_trq_error_threshold 输入：允许的最大扭矩误差的阈值
     * @return 输出：成功返回 SOLVE_NOERROR，失败返回相应的错误码
     */
    int SetMaxTrqErrorThreshold(const std::vector<double>& m_max_trq_error_threshold);

    /**
     * @brief 设置阻尼比增益
     *
     * 此函数用于设置阻尼比增益参数。(暂时不允许外部调用)
     *
     * @param[in] zeta_set 输入：阻尼比增益设置值
     * @return 输出：成功返回 SOLVE_NOERROR，失败返回相应的错误码
     */
    int SetZetaGain(const std::vector<double>& zeta_set);

    /**
     * @brief 设置阻抗增益
     *
     * 该接口暂不开放，用于根据拖动类型设置阻抗增益。
     *
     * @param[in] drag_type 输入：拖动类型
     * @return 输出：成功返回 SOLVE_NOERROR，失败返回相应的错误码
     */
    int SetImpedenceGain(const DragType& drag_type);  //该接口先不开放

    /**
     * @brief 设置负载限制
     *
     * 此函数用于设置负载的最大质量和最大TCP长度限制。
     *
     * @param[in] max_load_mass 输入：最大负载质量
     * @param[in] max_load_tcp_length 输入：最大负载TCP长度
     * @return 输出：成功返回 SOLVE_NOERROR，失败返回相应的错误码
     */
    int SetLoadLimit(const double& max_load_mass, const double& max_load_tcp_length);

    /**
     * @brief 设置力控负载
     *
     * 此函数用于设置力控模块的负载信息。
     *
     * @param[in] load 输入：力控负载信息
     * @return 输出：成功返回 SOLVE_NOERROR，失败返回相应的错误码
     */
    int SetFcLoad(const RokaeLoad& load);

    /**
     * @brief 设置拖动增益滑条系数
     *
     * 此函数用于设置比例增益参数。
     *
     * @param[in] kp_gain_set 输入：比例增益设置值
     * @return 输出：成功返回 SOLVE_NOERROR，失败返回相应的错误码
     */
    int SetKpGain(const std::vector<double>& kp_gain_set);

    /**
     * @brief 设置拖动摩擦力补偿滑条系数
     *
     * 此函数用于设置摩擦增益参数。
     *
     * @param[in] fric_gain_set 输入：摩擦增益设置值
     * @return 输出：成功返回 SOLVE_NOERROR，失败返回相应的错误码
     */
    int SetFricGain(const std::vector<double>& fric_gain_set);

    /**
     * @brief 根据负载重置发给伺服的带宽
     *
     * 此函数用于根据负载信息重置比例增益。
     *
     * @param[in] load 输入：负载信息
     * @return 输出：成功返回 SOLVE_NOERROR，失败返回相应的错误码
     */
    int ResetKpByLoad(const RokaeLoad& load);

    /**
     * @brief 根据负载重置发给伺服的摩擦力补偿
     *
     * 此函数用于根据负载信息重置摩擦增益。
     *
     * @param[in] load 输入：负载信息
     * @return 输出：成功返回 SOLVE_NOERROR，失败返回相应的错误码
     */
    int ResetFricByLoad(const RokaeLoad& load);

    /**
     * @brief 重置力控内部状态
     *
     * 用于当Drag_Config或Fc_Update返回错误时调用。调用后需要，重新设置各种增益以及负载参数，零点和传感器相关参数不用重新设置
     *
     * @return 输出：成功返回 SOLVE_NOERROR，失败返回相应的错误码
     */
    int ResetFcStatus();

    /**
     * @brief 重置内部状态接口
     *
     * 重置内部状态
     *
     * @return 输出：成功返回 SOLVE_NOERROR，失败返回相应的错误码
     */
    void FcStatusRefresh();

    // 额外功能函数
    /**
     * @brief 力矩传感器零点标定
     *
     * 此函数用于校准扭矩传感器，通过传入编码器反馈位置、负载信息以及模拟信号通道数据，计算并输出传感器的偏置值。
     *
     * @param[in] pos_encoder_feedback 输入：编码器反馈的位置数据
     * @param[in] load_input 输入：负载信息
     * @param[in] analog_array_ch1 输入：传感器通道1的数据数组，每个数组包含200个元素
     * @param[in] analog_array_ch2 输入：传感器通道2的数据数组，每个数组包含200个元素
     * @param[out] sensor_bias 输出：计算得到的传感器零点
     * @return 输出：成功返回 SOLVE_NOERROR，失败返回相应的错误码
     */
    int CalibrateTrqSensor(const std::vector<int32_t>& pos_encoder_feedback, const RokaeLoad& load_input,
                           const std::vector<std::array<int16_t, 200>>& analog_array_ch1,
                           const std::vector<std::array<int16_t, 200>>& analog_array_ch2, std::vector<double>& sensor_bias);

    //一些获取状态的接口
    const bool& GetDragStatus() { return m_enable_drag; };  // 获取是否允许拖动

   private:
    // 内部计算变量
    unsigned int m_jnt_num;  // 关节数
    KDL::Chain m_chain;      // chain
    RokaeLoad m_load;        // 负载信息
    DragType m_drag_type;    // 拖动类型

    std::vector<double> m_joint_range_min_inner;  // 软限位负
    std::vector<double> m_joint_range_max_inner;  // 软限位正
    std::vector<double> m_kp_gain_set;            // kp增益(滑条系数)
    std::vector<double> m_fri_gain_set;           // fri增益(滑条系数)
    std::vector<double> m_kp_set_by_load;         // 实际发给伺服的kp增益
    std::vector<double> m_fri_set_by_load;        // 实际发给伺服的fri增益
    std::vector<double> m_load_mass_limit;        // 最大质量限制
    std::vector<double> m_load_tcp_length_limit;  // 最大TCP长度限制

    // 内部状态标志位
    bool m_enable_drag;
    bool m_is_first_drag;

    // 内部数据流
    Servo_To_FcInner m_servo_data_fc_inner;
    FcInner_To_Servo m_fc_inner_servo_data;
    FcStatusInner m_fc_status_inner;
    FcParamsInner* m_fc_params_inner_ptr;

    // 求解器
    InitRobot* m_init_robot_ptr;
    Protect::ForceProtect* m_force_protect_ptr;
    DynamicSolver* m_dynamicsolver_ptr;
    KDL::ChainFkSolverPos_recursive* m_fkpos_ptr;
    FcStatusTracker* m_fc_status_tracker_ptr;
    Servo_Fc_Convert* m_servo_fc_convert_ptr;
    Control::ForcePlanner* m_force_planner_ptr;
};

}  // namespace Control
}  // namespace RokaeApi

#endif