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

#include "Servo_Fc_convert.hpp"
#include "data_structure_define.hpp"
#include "dynamic_solver.hpp"
#include "fc_data_computation.hpp"
#include "force_protect.hpp"
#include "initialize.hpp"
#include "force_planner.hpp"
namespace RokaeApi {
namespace Control {
#define FORCECONTROL_INSTANCE ForceControl::GetInstance()  // 定义一个宏，用于获取单例实例
class ForceControl {
   private:
    static constexpr int ANALOG_DATA_COUNT = 200;
    static constexpr int POSITION_MODE = 8;
    static constexpr int TORQUE_MODE = 10;

   private:
    static ForceControl* force_control_instance_ptr;  // 静态成员变量，指向唯一的实例
    ForceControl(InitRobot* init_robot_ptr);

   public:
    ~ForceControl();
    // 获取实例的静态方法
    static ForceControl* GetInstance();
    //初始化单例实例
    static void InitInStance(InitRobot* init_robot_ptr);
    // 释放单例实例
    static void ReleaseInstance();
    // 防止拷贝构造和拷贝赋值
    ForceControl(const ForceControl&) = delete;
    ForceControl& operator=(const ForceControl&) = delete;

   public:
    int Fcinit();

    int DragConfig(const std::vector<int32_t>& pos_encoder_from_servo, const std::vector<int8_t>& servo_mode_from_servo,
                   const std::vector<int16_t>& analog_ch1, const std::vector<int16_t>& analog_ch2, const DragType& drag_type);
    int FcUpdate(const std::vector<int8_t>& servo_mode_from_servo, const std::vector<int16_t>& pdo_analog_ch1,
                 const std::vector<int16_t>& pdo_analog_ch2, const std::vector<int16_t>& trq_encoder_from_servo,
                 const std::vector<int>& pos_encoder_from_servo, const std::vector<int>& vel_encoder_from_servo,
                 std::vector<int16_t>& trq_cmd_to_servo, std::vector<int16_t>& fc_trq_feedforward_to_servo,
                 std::vector<int16_t>& fc_kp_to_servo, std::vector<int16_t>& fc_kd_to_servo,
                 std::vector<int16_t>& fc_edb_cof_to_servo, std::vector<int16_t>& fc_edb_o_to_servo,
                 std::vector<int16_t>& fc_fric_cof_to_servo, std::vector<int16_t>& fc_jnt_inertia_to_servo);
    void SetFcCommand(const Servo_To_FcInner& servo_data_fc_inner);
    int StopDrag(const int8_t param_0x6061[6]);

    int SetSensorLinearity(const std::vector<double> analog2trq_low);
    int SetSensorBias(const std::vector<double> analog_bias);
    int SetEncoderOffset(const std::vector<int32_t> encoder_offset);
    int SetSoftLimit(const std::vector<double> joint_range_min, const std::vector<double> joint_range_max);
    int SetMaxTrqErrorThreshold(const std::vector<double> m_max_trq_error_threshold);

    int SetZetaGain(const std::vector<double> zeta_set);
    int SetImpedenceGain(const DragType& drag_type);  //该接口先不开放
    int SetLoadLimit(const double& max_load_mass, const double& max_load_tcp_length);
    int SetFcLoad(const RokaeLoad& load);
    int SetKpGain(const std::vector<double>& kp_gain_set);
    int SetFricGain(const std::vector<double>& fric_gain_set);
    int ResetKpByLoad(const RokaeLoad& load);
    int ResetFricByLoad(const RokaeLoad& load);

    //额外功能
    int CalibrateTrqSensor(const std::vector<int32_t>& pos_encoder_feedback, const RokaeLoad& load_input,
                           const std::vector<std::array<int16_t, 200>>& analog_array_ch1,
                           const std::vector<std::array<int16_t, 200>>& analog_array_ch2, std::vector<double>& sensor_bias);

   private:
    // ForceControl内部计算的变量
    // 1.可变参数部分(配置文件中存在，但可变的)
    // 1.1机械部分
    std::vector<int32_t> m_encoder_offset_inner;  //电机编码器零点
    std::vector<double> m_analog_bias_inner;      //传感器零点
    std::vector<double> m_analog2trq_low;         //传感器线性度

    // 1.2模型参数
    std::vector<double> m_joint_range_min_inner;
    std::vector<double> m_joint_range_max_inner;
    std::vector<double> m_joint_range_min_new_inner;
    std::vector<double> m_joint_range_max_new_inner;

    // 1.3控制参数
    std::vector<double> m_joint_gain_kp_inner;
    std::vector<double> m_joint_damp_zeta_inner;
    std::vector<double> m_friction_cof_servo_inner;

    // 1.4保护参数
    std::vector<double> m_trq_error_threshold_inner;

    // 2.功能力计算
    KDL::JntArray m_ref_trq;               //指令力矩

    // 3.传感器相关
    std::vector<int16_t> m_analog_ch1;      //通道1电压
    std::vector<int16_t> m_analog_ch2;      //通道2电压
    std::vector<int16_t> m_analog_average;  //双通道电压平均值
    std::vector<double> m_sensor_trq;       //传感器反馈力矩

    // 4.模型基础信息及负载
    InitRobot* m_init_robot_ptr;
    unsigned int m_jnt_num;
    KDL::Chain m_chain;
    RokaeLoad m_load;
    std::vector<double> m_jnt_current_pos;

    // 5.其他参数
    std::vector<double> m_trq_error;  //重力矩与传感器反馈之差
    bool m_enable_drag;
    DragType m_drag_type;
    bool m_is_first_drag;
    std::vector<int32_t> m_zero_vector;
    KDL::JntArray m_zero_jnt;

    // 6.内部数据流
    Servo_To_FcInner m_servo_data_fc_inner;
    FcInner_To_Servo m_fc_inner_servo_data;
    FcStatusInner m_fc_status_inner;
    FcParamsInner* m_fc_params_inner_ptr;    

    // 8.一些求解器
    Protect::ForceProtect* m_force_protect_ptr;
    DynamicSolver* m_dynamicsolver_ptr;
    KDL::ChainFkSolverPos_recursive* m_fkpos_ptr;
    FcStatusTracker* m_fc_status_tracker_ptr;
    Servo_Fc_Convert* m_servo_fc_convert_ptr;
    Control::ForcePlanner* m_force_planner_ptr;

    //9.一些只在forcecontrol内部用的成员变量
    std::vector<double> m_kp_gain_set;
    std::vector<double> m_kp_set_by_load;
    std::vector<double> m_fri_gain_set;
    std::vector<double> m_fri_set_by_load;
    std::vector<double> m_load_mass_limit;
    std::vector<double> m_load_tcp_length_limit;


};

}  // namespace Control
}  // namespace RokaeApi

#endif