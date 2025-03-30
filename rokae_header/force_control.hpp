/**
 * Copyright(C) 2024 Rokae Technology Co., Ltd.
 * All Rights Reserved.
 *
 * Information in this file is the intellectual property of Rokae Technology Co., Ltd,
 * And may contains trade secrets that must be stored and viewed confidentially.
 *
 * @file: forcecontrol.hpp
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

namespace RokaeApi {
namespace Control {

class ForceControl {
   public:
    ForceControl(InitRobot* init_robot_ptr);
    ~ForceControl();

    /**
     * @brief 力控初始化接口，转化一些从cfg中读取到的参数，并赋值到成员变量中
     *
     * @return 返回0表示运行成功，不成功则返回错误码
     */
    int Fcinit();

    /**
     * @brief 拖动配置接口，负责判断一下当前伺服模式，同时根据传感器信息判断是否存在双通道电压差过大(该功能暂时未实现)
     *
     * @param[in] param_0x6061:伺服模式,8代表位置模式,10代表力矩模式
     * @param[in] param_0x2401:传感器通道1电压，单位mv
     * @param[in] param_0x2402:传感器通道2电压，单位mv
     * @param[in] param_0x6064:位置编码器反馈
     * @param[in] param_0x606C:速度编码器反馈
     * @param[in] param_0x2406:传感器扭矩反馈
     * @param[in] load_params:负载信息
     * @param[in] drag_type:拖动模式，目前只支持轴空间拖动
     *
     * @param[out] sensor_trq_feedback:传感器反馈计算值
     * @param[out] trq_ref:力矩指令值
     * @param[out] trq_error:力矩误差
     *
     * @return 返回0表示配置成功，不成功则返回错误码
     */
    // int DragConfig(const int8_t param_0x6061[6], const int16_t param_0x2401[6], const int16_t param_0x2402[6],
    //                const int32_t param_0x6064[6], const int32_t param_0x606C[6], const int16_t param_0x2406[6],
    //                const LoadInertia& load_params, DragType drag_type, double sensor_trq_feedback[6], double trq_ref[6],
    //                double trq_error[6]);

    /**
     * @brief 力控计算总接口，包括数据读取、数据转换、数据计算以及下发，每个控制周期调用一次
     *
     * @param[in] param_0x6064:位置编码器反馈
     * @param[in] param_0x606C:速度编码器反馈
     * @param[in] param_0x2406:关节扭矩反馈(暂时未用到)
     * @param[in] param_0x6061:伺服模式,8代表位置模式,10代表力矩模式
     *
     * @param[out] param_0x6071:关节扭矩指令
     * @param[out] param_0x60B2:扭矩前馈
     * @param[out] param_0x2201:关节力控环带宽
     * @param[out] param_0x2202:关节力控环阻尼比
     * @param[out] param_0x2203:扭矩反馈系数
     * @param[out] param_0x2204:扭矩反馈偏置
     * @param[out] param_0x2205:电机侧摩擦补偿系数
     * @param[out] param_0x2206:关节惯量
     *
     * @return 返回0表示计算成功，不成功则返回错误码
     */
    int FcUpdate(const int32_t param_0x6064[6], const int32_t param_0x606C[6], const int16_t param_0x2406[6],
                 const int8_t param_0x6061[6], int16_t param_0x6071[6], int16_t param_0x60B2[6], int16_t param_0x2201[6],
                 int16_t param_0x2202[6], int16_t param_0x2203[6], int16_t param_0x2204[6], int16_t param_0x2205[6],
                 int16_t param_0x2206[6], double jnt_vel_rad[6]);

    /**
     * @brief 数据转换接口，将从pdo读取到的数据转化为内部数据
     *
     * @param[in] param_0x6064:位置编码器反馈
     * @param[in] param_0x606C:速度编码器反馈
     * @param[in] param_0x2406:关节扭矩反馈(暂时未用到)
     * @param[in] param_0x6061:伺服模式,8代表位置模式,10代表力矩模式
     *
     *
     * @return 返回0表示转换成功，不成功则返回错误码
     */
    int DataTransform(const int32_t param_0x6064[6], const int32_t param_0x606C[6], const int16_t param_0x2406[6],
                      const int8_t param_0x6061[6]);

    /**
     * @brief 关节位置计算接口，将位置编码器的值转化为关节角度值，单位弧度
     *
     * @param[in] encoder_value:位置编码器反馈

     * @return 无
     */
    void GetAxisPos(const std::vector<int32_t>& encoder_value);

    /**
     * @brief 关节速度计算接口，将速度编码器的值转化为关节速度值，单位弧度/s
     *
     * @param[in] encoder_vel_value:速度编码器反馈

     * @return 无
     */
    void GetJntVel(const std::vector<int32_t>& encoder_vel_value);

    /**
     * @brief 传感器力矩计算接口，单位Nm
     *
     * @param[in] analog_average:传感器双通道电压平均值，单位mv

     * @return 无
     */
    void GetSensorTorque(const std::vector<int16_t>& analog_average);

    /**
     * @brief 力控数据流接受、计算、更新接口，由FcUpdate接口调用
     *
     * @param[in] param_0x6064:位置编码器反馈
     * @param[in] param_0x606C:速度编码器反馈
     * @param[in] param_0x2406:关节扭矩反馈(暂时未用到)
     * @param[in] param_0x6061:伺服模式,8代表位置模式,10代表力矩模式

     * @return 返回0表示计算，不成功则返回错误码
     */
    int UpdateFcStatus(const int32_t param_0x6064[6], const int32_t param_0x606C[6], const int16_t param_0x2406[6],
                       const int8_t param_0x6061[6]);

    /**
     * @brief 根据力控模式设置位置指令，拖动位置指令为反馈。
     *
     * @return 无
     */
    void SetFcCommand(const Servo_To_FcInner& servo_data_fc_inner);

    /**
     * @brief 力控模块相关指令计算，包括期望力、阻抗力、搜索运动、虚拟器、保护力、动力学补偿等，目前拖动只进行动力学补偿
     *
     * @return 无
     */
    void ForcePlanner();

    /**
     * @brief Fc指令下发到伺服的接口
     *
     * @return 无
     */
    void UpDateFcToServo();

    /**
     * @brief 数据转化接口，将vector数据转化为数组下发到伺服(C语言无法识别vector)
     *
     * @param[out] param_0x6071:关节扭矩指令
     * @param[out] param_0x60B2:扭矩前馈
     * @param[out] param_0x2201:关节力控环带宽
     * @param[out] param_0x2202:关节力控环阻尼比
     * @param[out] param_0x2203:扭矩反馈系数
     * @param[out] param_0x2204:扭矩反馈偏置
     * @param[out] param_0x2205:电机侧摩擦补偿系数
     * @param[out] param_0x2206:关节惯量
     *
     * @return 无
     */
    void ServoDataVectorToArray(int16_t param_0x6071[6], int16_t param_0x60B2[6], int16_t param_0x2201[6],
                                int16_t param_0x2202[6], int16_t param_0x2203[6], int16_t param_0x2204[6],
                                int16_t param_0x2205[6], int16_t param_0x2206[6]);

    /**
     * @brief 关闭拖动接口，拖动的关闭不依靠该接口实现，该接口只是进行内部状态的更改，以及当前模式的判断，只允许在位置模式调用
     *
     * @param[in] param_0x6061:伺服模式,8代表位置模式,10代表力矩模式
     *
     * @return 关闭成功返回0，失败返回错误码
     */
    int StopDrag(const int8_t param_0x6061[6]);

    /**
     * @brief 设置传感器线性度接口
     * @param[in] analog2trq_low: 传感器线性度
     *
     * @param return：0:设置成功；
     *                其他:设置失败，返回错误码
     */
    int SetSensorLinearity(const std::vector<double> analog2trq_low);

    /**
     * @brief 设置传感器偏置接口
     * @param[in] analog_bias: 传感器偏置
     *
     * @param return：0:设置成功；
     *                其他:设置失败，返回错误码
     */
    int SetSensorBias(const std::vector<double> analog_bias);

    /**
     * @brief 设置位置编码器偏置接口
     * @param[in] encoder_offset: 位置编码器偏置
     *
     * @param return：0:设置成功；
     *                其他:设置失败，返回错误码
     */
    int SetEncoderOffset(const std::vector<int32_t> encoder_offset);

    /**
     * @brief 设置摩擦力补偿系数，范围[0~1]
     * @param[in] fric_set: 摩擦力补偿系数
     *
     * @param return：0:设置成功；
     *                其他:设置失败，返回错误码
     */
    int SetFricGain(const std::vector<double> fric_set);

    /**
     * @brief 设置力控增益系数，范围[0~1]
     * @param[in] kp_set: 力控增益系数
     *
     * @param return：0:设置成功；
     *                其他:设置失败，返回错误码
     */
    int SetKpGain(const std::vector<double> kp_set);

    /**
     * @brief 设置软限位(输入的是角度，内部会自动转化为弧度)
     * @param[in] joint_range_min: 软限位下限
     * @param[in] joint_range_max: 软限位上限
     *
     * @param return：0:设置成功；
     *                其他:设置失败，返回错误码
     */
    int SetSoftLimit(const std::vector<double> joint_range_min, const std::vector<double> joint_range_max);

    /**
     * @brief 检查开启拖动前力矩偏差，若偏差过大则不允许切换到力矩模式
     * @param[out] sensor_trq_feedback: 传感器力矩反馈
     * @param[out] m_trq_ref: 力矩指令
     * @param[out] trq_error: 力矩偏差
     *
     * @param return：0:允许开启力控；
     *                其他:力矩偏差过大，不允许开启力控
     */
    int SwichToTrqMode(double sensor_trq_feedback[6], double trq_ref[6], double trq_error[6]);

    /**
     * @brief 检查开启拖动前机械臂位置是否处于力控保护区，力控保护区是超过软限位10°以内
     * @param[in] q_in: 当前关节位置
     * @param[in] joint_limit_upper: 软限位上限
     * @param[in] joint_limit_lower: 软限位下限
     *
     * @param return：true:不处在软限位范围内，不允许开启拖动
     *                false:处在软限位范围内，允许开启拖动
     */
    bool IsInForceControlArea(const JntArray& q_in, const std::vector<double>& joint_limit_upper,
                              const std::vector<double>& joint_limit_lower);

    /**
     * @brief 传感器零点标定接口
     * @param[in] param_0x6064: 位置编码器值
     * @param[in] load_params_in: 负载参数
     * @param[in] analog_array_ch1: param_0x2401 传感器通道1采集200次组成2维数组
     * @param[in] analog_array_ch2: param_0x2402 传感器通道2采集200次组成2维数组
     *
     * @param[out] sensor_bias: 传感器零点
     *
     * @param return：0:零点标定成功
     *
     */
    int CalibrateTrqSensor(const int32_t param_0x6064[6], const LoadInertia& load_params_in,
                           const int16_t analog_array_ch1[6][200], const int16_t analog_array_ch2[6][200], double sensor_bias[6]);

    /**
     * @brief 单轴传感器零点标定接口
     * @param[in] param_0x6064: 位置编码器值
     * @param[in] load_params_in: 负载参数
     * @param[in] analog_array_ch1: param_0x2401 标定轴的传感器通道1采集200次组成1维数组
     * @param[in] analog_array_ch2: param_0x2402 标定轴的传感器通道2采集200次组成1维数组
     * @param[in] axis_num: 标定的轴号
     *
     * @param[out] sensor_bias_axis: 传感器零点
     *
     * @param return：0:零点标定成功
     *
     */
    int CalibrateTrqSensorAxis(const int32_t param_0x6064[6], const LoadInertia& load_params_in,
                               const int16_t analog_array_ch1[200], const int16_t analog_array_ch2[200],
                               const unsigned int axis_num, double sensor_bias_axis[6]);

    /**
     * @brief 根据负载信息设置Kp参数
     *
     * @param[in] load:负载信息
     *
     * @return 无返回值
     */
    void ResetKpByLoad(const LoadInertia& load);

    /**
     * @brief 根据负载信息设置Fric Gain参数
     *
     * @param[in] load:负载信息
     *
     * @return 无返回值
     */
    void ResetFricByLoad(const LoadInertia& load);

   public:
    int DragConfig(const std::vector<int32_t>& pos_encoder_from_servo, const std::vector<int8_t>& servo_mode_from_servo,
                   const std::vector<int16_t>& analog_ch1, const std::vector<int16_t>& analog_ch2, const DragType& drag_type);
    int FcUpdate(const std::vector<int8_t>& servo_mode_from_servo, const std::vector<int16_t>& pdo_analog_ch1,
                 const std::vector<int16_t>& pdo_analog_ch2, const std::vector<int16_t>& trq_encoder_from_servo,
                 const std::vector<int>& pos_encoder_from_servo, const std::vector<int>& vel_encoder_from_servo,
                 std::vector<int16_t>& trq_cmd_to_servo, std::vector<int16_t>& fc_trq_feedforward_to_servo,
                 std::vector<int16_t>& fc_kp_to_servo, std::vector<int16_t>& fc_kd_to_servo,
                 std::vector<int16_t>& fc_edb_cof_to_servo, std::vector<int16_t>& fc_edb_o_to_servo,
                 std::vector<int16_t>& fc_fric_cof_to_servo, std::vector<int16_t>& fc_jnt_inertia_to_servo);

   private:
    // ForceControl内部计算的变量
    // 1.可变参数部分(配置文件中存在，但可变的)
    // 1.1机械部分
    std::vector<int32_t> m_encoder_offset_inner;  //电机编码器零点
    std::vector<double> m_analog_bias_inner;      //传感器零点
    std::vector<double> m_decel_ratio_low_inner;  //传感器线性度

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
    KDL::JntArray m_ref_trq_desire;        //期望力
    KDL::JntArray m_ref_trq_overlay;       //搜索力
    KDL::JntArray m_ref_trq_impedance;     //阻抗力
    KDL::JntArray m_ref_trq_virtual_wall;  //虚拟墙
    KDL::JntArray m_ref_trq_joint_limit;   //关节限位保护力
    KDL::JntArray m_ref_trq_dyn;           //静动力学(仅重力)
    KDL::JntArray m_ref_trq;               //指令力矩
    std::vector<double> m_trq_comp_coef;   //软限位保护力
    std::vector<double> m_joint_inertia;   //关节惯量


    // 3.增益调节相关
    std::vector<double> m_kp_set_gain;
    std::vector<double> m_fric_set_gain;

    // 4.传感器相关
    std::vector<int16_t> m_analog_ch1;      //通道1电压
    std::vector<int16_t> m_analog_ch2;      //通道2电压
    std::vector<int16_t> m_analog_average;  //双通道电压平均值
    std::vector<double> m_sensor_trq;       //传感器反馈力矩

    // 5.模型基础信息及负载
    InitRobot* m_init_robot_ptr;
    unsigned int m_jnt_num;
    KDL::Chain m_chain;
    LoadInertia m_load;
    std::vector<double> m_jnt_current_pos;
    // std::vector<double> m_jnt_vel_abs;
    // std::vector<double> m_jnt_vel_real;

    // 6.其他参数
    std::vector<double> m_trq_error;  //重力矩与传感器反馈之差
    bool m_enable_drag;
    DragType m_drag_type;
    bool m_is_first_drag;

    // 7.内部数据流
    Servo_To_FcInner m_servo_data_fc_inner;
    FcStatusInner m_fc_status_inner;
    FcParamsInner* m_fc_params_inner_ptr;

    // 8.一些求解器
    Protect::ForceProtect* m_force_protect_ptr;
    Axis_Convert* m_axis_convert_ptr;
    DynamicSolver* m_dynamicsolver_ptr;
    KDL::ChainFkSolverPos_recursive* m_fkpos_ptr;
    FcStatusTracker* m_fc_status_tracker_ptr;
    Servo_Fc_Convert* m_servo_fc_convert_ptr;
};

}  // namespace Control
}  // namespace RokaeApi

#endif