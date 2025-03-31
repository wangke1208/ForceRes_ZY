/**
 * Copyright(C) 2024 Rokae Technology Co., Ltd.
 * All Rights Reserved.
 *
 * Information in this file is the intellectual property of Rokae Technology Co., Ltd,
 * And may contains trade secrets that must be stored and viewed confidentially.
 *
 * @file: forcecontrol.cpp
 * @author: wangke
 * @date: 2024/4/25
 * @brief: 力控计算模块
 */
#include <algorithm>
#include <cstddef>
#include <iterator>

#include "rokae_header/force_control.hpp"


using namespace std;

namespace RokaeApi {
namespace Control {
ForceControl::ForceControl(InitRobot* init_robot_ptr)
    : m_init_robot_ptr(init_robot_ptr),
      m_jnt_num(m_init_robot_ptr->GetJntNum()),
      m_chain(init_robot_ptr->GetChain()),
      m_fc_status_inner(init_robot_ptr->GetJntNum()),
      m_servo_data_fc_inner(init_robot_ptr->GetJntNum()) {
    //初始化一些求解器
    m_fc_params_inner_ptr = new FcParamsInner(m_jnt_num);

    m_force_protect_ptr = new Protect::ForceProtect(m_jnt_num, m_fc_params_inner_ptr);
    m_axis_convert_ptr = new Axis_Convert(m_jnt_num, m_init_robot_ptr->GetMechanicalParams());
    m_dynamicsolver_ptr = new DynamicSolver(m_chain, m_init_robot_ptr->GetGravity());
    m_fkpos_ptr = new KDL::ChainFkSolverPos_recursive(m_chain);
    m_fc_status_tracker_ptr = new FcStatusTracker(m_init_robot_ptr, &m_fc_status_inner, m_fc_params_inner_ptr);
    m_servo_fc_convert_ptr = new Servo_Fc_Convert(m_jnt_num);
    m_force_planner_ptr = new Control::ForcePlanner(m_init_robot_ptr, &m_fc_status_inner, m_fc_params_inner_ptr);
    //初始化信息
    m_load.SetZero();
    m_drag_type = DragType::DRAG_JOINT;
    m_enable_drag = false;
    m_is_first_drag = true;
    m_servo_data_fc_inner.Resize(m_jnt_num);

    //初始化参数
    m_encoder_offset_inner.resize(m_jnt_num);  
    m_analog_bias_inner.resize(m_jnt_num);      
    m_decel_ratio_low_inner.resize(m_jnt_num);

    m_joint_range_min_inner.resize(m_jnt_num);
    m_joint_range_max_inner.resize(m_jnt_num);
    m_joint_range_min_new_inner.resize(m_jnt_num);
    m_joint_range_max_new_inner.resize(m_jnt_num);

    m_joint_gain_kp_inner.resize(m_jnt_num);
    m_joint_damp_zeta_inner.resize(m_jnt_num);
    m_friction_cof_servo_inner.resize(m_jnt_num);

    m_ref_trq.resize(m_jnt_num);

    m_kp_set_gain.resize(m_jnt_num);
    m_fric_set_gain.resize(m_jnt_num);

    m_analog_ch1.resize(m_jnt_num);
    m_analog_ch2.resize(m_jnt_num);
    m_analog_average.resize(m_jnt_num);
    m_sensor_trq.resize(m_jnt_num);

    m_jnt_current_pos.resize(m_jnt_num);
    // m_jnt_vel_abs.resize(m_jnt_num);
    // m_jnt_vel_real.resize(m_jnt_num);

    m_trq_error.resize(m_jnt_num);
}

ForceControl::~ForceControl() {
    delete m_force_protect_ptr;
    delete m_axis_convert_ptr;
    delete m_dynamicsolver_ptr;
    delete m_fc_status_tracker_ptr;
    delete m_servo_fc_convert_ptr;
}

int ForceControl::Fcinit() {
    //可变参数赋值:TODO 后续也像Fc内部参数一样进行优化
    for (unsigned int i = 0; i < m_jnt_num; i++) {
        //可变参数--机械
        m_encoder_offset_inner[i] = m_init_robot_ptr->GetMechanicalParams().encoder_offset[i];  
        m_analog_bias_inner[i] = m_init_robot_ptr->GetMechanicalParams().analog_bias[i];      
        m_decel_ratio_low_inner[i] = m_init_robot_ptr->GetMechanicalParams().decel_ratio_low[i];    
    
        //可变参数--模型
        m_joint_range_min_inner[i] = m_init_robot_ptr->GetModelParams().joint_range_min[i];
        m_joint_range_max_inner[i] = m_init_robot_ptr->GetModelParams().joint_range_max[i];
        m_joint_range_min_new_inner[i] = m_init_robot_ptr->GetModelParams().joint_range_min_new[i];
        m_joint_range_max_new_inner[i] = m_init_robot_ptr->GetModelParams().joint_range_max_new[i];
    }

    // Fc内部可调参数
    m_fc_params_inner_ptr->m_function_params.SetParam("joint_servo_kp",
                                                  m_init_robot_ptr->GetControlParams().m_gain_params.joint_gain_kp);
    m_fc_params_inner_ptr->m_function_params.SetParam("joint_servo_dmap_kv",
                                                  m_init_robot_ptr->GetControlParams().m_gain_params.joint_damp_zeta);
    m_fc_params_inner_ptr->m_function_params.SetParam("joint_servo_friction",
                                                  m_init_robot_ptr->GetControlParams().m_gain_params.friction_cof_servo);
    m_fc_params_inner_ptr->m_protect_params.SetParam("max_mode_switch_trq",
                                                 m_init_robot_ptr->GetControlParams().m_protect_params.max_mode_switch_trq);

    //设置默认软限位
    m_force_planner_ptr->SetSoftLimit(m_joint_range_min_inner, m_joint_range_max_inner);

    return SOLVE_NOERROR;
}

void ForceControl::SetFcCommand(const Servo_To_FcInner& servo_data_fc_inner) {
    // 1.计算当前关节位置
    m_axis_convert_ptr->GetAxisPos(servo_data_fc_inner.pos_feedback, m_fc_status_inner.jnt_pos_measure);
    // 2.更新拖动类型
    m_fc_status_inner.drag_type = m_drag_type;

    //计算实际位置
    switch (m_drag_type) {
    case DragType::DRAG_JOINT:
        if (m_is_first_drag) {
            m_fc_status_inner.jnt_pos_command = m_fc_status_inner.jnt_pos_measure;
            m_fc_status_inner.jnt_vel_command.data.setZero();  //速度指令给0
            m_is_first_drag = false;
        }
        m_fc_status_inner.cart_pos_jnt_command = m_fc_status_inner.jnt_pos_measure;
        m_fkpos_ptr->JntToCart(m_fc_status_inner.cart_pos_jnt_command, m_fc_status_inner.cart_pos_command_flan_in_base);

        break;

    case DragType::DRAG_CART_TRANS:
    case DragType::DRAG_CART_ROT:
    case DragType::DRAG_CART_FREE:
        if (m_is_first_drag) {
            m_fc_status_inner.cart_pos_jnt_command = m_fc_status_inner.jnt_pos_measure;
            m_fkpos_ptr->JntToCart(m_fc_status_inner.cart_pos_jnt_command, m_fc_status_inner.cart_pos_command_flan_in_base);
            m_fc_status_inner.jnt_vel_command.data.setZero();
            m_is_first_drag = false;
        }
        m_fc_status_inner.jnt_pos_command = m_fc_status_inner.jnt_pos_measure;

        break;
    }
}

int ForceControl::FcUpdate(const std::vector<int8_t>& servo_mode_from_servo, const std::vector<int16_t>& pdo_analog_ch1,
                           const std::vector<int16_t>& pdo_analog_ch2, const std::vector<int16_t>& trq_encoder_from_servo,
                           const std::vector<int>& pos_encoder_from_servo, const std::vector<int>& vel_encoder_from_servo,
                           std::vector<int16_t>& trq_cmd_to_servo, std::vector<int16_t>& fc_trq_feedforward_to_servo,
                           std::vector<int16_t>& fc_kp_to_servo, std::vector<int16_t>& fc_kd_to_servo,
                           std::vector<int16_t>& fc_edb_cof_to_servo, std::vector<int16_t>& fc_edb_o_to_servo,
                           std::vector<int16_t>& fc_fric_cof_to_servo, std::vector<int16_t>& fc_jnt_inertia_to_servo) {
    // 1.判断是否进行了drag_config
    if (!m_enable_drag) {
        return ERROR_DRAG_ENABLE;
    }
    // 2.读取伺服数据并转换为Fc内部变量
    m_servo_fc_convert_ptr->ServoData2FcInner(servo_mode_from_servo, pdo_analog_ch1, pdo_analog_ch2, trq_encoder_from_servo,
                                              pos_encoder_from_servo, vel_encoder_from_servo, m_servo_data_fc_inner);

    // 3.更新指令和反馈
    SetFcCommand(m_servo_data_fc_inner);

    // 3.力控数据流计算
    m_fc_status_tracker_ptr->FcStatusUpdata();
    // 4.力控模块功能力计算
    m_ref_trq = m_force_planner_ptr->ForcePlannerUpdata();
    // 5.将Fc内部数据转换为下发给伺服数据
    
    // 6.判断当前伺服模式是否为力矩模式，处于力矩模式允许下发力控相关指令
}

int ForceControl::DragConfig(const std::vector<int32_t>& pos_encoder_from_servo, const std::vector<int8_t>& servo_mode_from_servo,
                             const std::vector<int16_t>& analog_ch1, const std::vector<int16_t>& analog_ch2, const DragType& drag_type) {
    //0.初始化标志位
    int res = SOLVE_NOERROR;
    m_enable_drag = false;
    m_is_first_drag = true;
    // 1.判断是否允许进行Drag设置
    // 1.1 处于位置模式下
    if (std::any_of(pos_encoder_from_servo.cbegin(), pos_encoder_from_servo.cend(),
                    [](int8_t servo_type) { return servo_type != 8; })) {
        return SERVO_MODE_ERROR;
    }
    // 1.2 计算当前位置
    std::vector<double> jnt_pos_rad_temp(m_jnt_num);
    m_axis_convert_ptr->GetAxisPos(pos_encoder_from_servo, jnt_pos_rad_temp);

    // 1.3 力矩偏差在合理范围
    std::vector<double> sensor_trq_temp(m_jnt_num);
    auto model_trq_temp = m_dynamicsolver_ptr->GetGraTorque(m_load, VectorToJntArray(jnt_pos_rad_temp));
    m_axis_convert_ptr->GetCobotTrq(analog_ch1, analog_ch2, sensor_trq_temp);
    res = m_force_protect_ptr->TrqErrorProtect(sensor_trq_temp, JntArrayToVector(model_trq_temp));
    if (res != SOLVE_NOERROR) return res;

    // 1.4 当前机器人位置不处在软限位保护范围内
    if (m_force_protect_ptr->IsInForceControlArea(VectorToJntArray(jnt_pos_rad_temp), m_joint_range_max_inner,
                                                  m_joint_range_min_inner)) {
        return STARTDRAG_POS_OVER_LIMIT;
    }

    // 2.设置拖动模式
    m_drag_type = drag_type;

    // 3.设置负载参数
    m_fc_status_tracker_ptr->SetLoad(m_load);

    // 4.更新标志位
    m_enable_drag = true;
    
    return SOLVE_NOERROR;
}

//*******************************数据更新相关接口部分*****************************
int ForceControl::FcUpdate(const int32_t param_0x6064[6], const int32_t param_0x606C[6], const int16_t param_0x2406[6],
                           const int8_t param_0x6061[6], int16_t param_0x6071[6], int16_t param_0x60B2[6],
                           int16_t param_0x2201[6], int16_t param_0x2202[6], int16_t param_0x2203[6], int16_t param_0x2204[6],
                           int16_t param_0x2205[6], int16_t param_0x2206[6], double jnt_vel_rad[6]) {
    // 0.m_enable_drag==true之后才能进行FcUpdate
    if (m_enable_drag == false) {
        return ERROR_DRAG_ENABLE;
    }

    // 1.读取伺服数据，并更新Fcstatus
    UpdateFcStatus(param_0x6064, param_0x606C, param_0x2406, param_0x6061);

    // 1.1新增，只要drag之后，就更新速度反馈
    for (unsigned int i = 0; i < m_jnt_num; i++) {
        jnt_vel_rad[i] = m_fc_status.jnt_vel_measure(i);
    }
    // 2.判断伺服模式，伺服为力矩模式才允许更新力闭环参数
    for (unsigned int i = 0; i < m_jnt_num; i++) {
        if (m_servo_to_fc->mode_operation_0x6061[i] != 10) {
            if (m_servo_to_fc->mode_operation_0x6061[i] == 0) {
                //临时版本：针对伺服模式突变为0的情况，不做任何处理，仍计算力矩
            } else {
                return SERVO_MODE_ERROR;
            }
        }
    }

    // 3.更新力矩闭环相关参数
    UpDateFcToServo();

    // 4.数据转化，下发到伺服
    ServoDataVectorToArray(param_0x6071, param_0x60B2, param_0x2201, param_0x2202, param_0x2203, param_0x2204, param_0x2205,
                           param_0x2206);

    return SOLVE_NOERROR;
}

int ForceControl::DataTransform(const int32_t param_0x6064[6], const int32_t param_0x606C[6], const int16_t param_0x2406[6],
                                const int8_t param_0x6061[6]) {
    std::copy(param_0x6064, param_0x6064 + 6, std::begin(m_servo_to_fc->pos_feedback_0x6064));  //此处加6不是加5
    std::copy(param_0x606C, param_0x606C + 6, std::begin(m_servo_to_fc->vel_feedback_0x606C));
    std::copy(param_0x2406, param_0x2406 + 6, std::begin(m_servo_to_fc->trq_feedback_0x2406));
    std::copy(param_0x6061, param_0x6061 + 6, std::begin(m_servo_to_fc->mode_operation_0x6061));

    return SOLVE_NOERROR;
}

int ForceControl::UpdateFcStatus(const int32_t param_0x6064[6], const int32_t param_0x606C[6], const int16_t param_0x2406[6],
                                 const int8_t param_0x6061[6]) {
    // 1.从伺服读取数据，并进行数据类型转换，同时进行模式判断，非力矩模式返回错误码，不允许继续下发
    DataTransform(param_0x6064, param_0x606C, param_0x2406, param_0x6061);

    // 2.根据伺服数据(编码器值等)计算位置、速度、传感器力矩等
    GetAxisPos(m_servo_to_fc->pos_feedback_0x6064);
    GetJntVel(m_servo_to_fc->vel_feedback_0x606C);  //暂时没用到

    // GetSensorTrq() 暂不开发

    // 3.更新位置指令，拖动模式将反馈当作指令
    SetFcCommand();

    // 4.根据更新的fcstatus更新力控模块数据流（包括各种动力学计算）
    m_fc_status_tracker->UpdateDateStream();

    // 5.计算更新力控相关指令
    ForcePlanner();

    return SOLVE_NOERROR;
}



void ForceControl::ForcePlanner() {
    // 1.期望力

    // 2.搜索运动

    // 3.阻抗运动

    // 4.虚拟墙

    // 5.关节限位
    m_ref_trq_joint_limit = m_force_planner_ptr->JointLimitUpdate(m_trq_comp_coef);  //暂时没用到m_trq_comp_coef

    // 6.动力学补偿
    m_ref_trq_dyn = m_fc_status.jnt_trq_gra_measure;

    // 7.关节期望扭矩（total）
    for (unsigned int i = 0; i < m_jnt_num; i++) {
        m_ref_trq(i) = m_ref_trq_dyn(i) + m_ref_trq_joint_limit(i);

        // 8.惯量计算（放到plan里计算，但是不计入m_ref_trq）
        m_joint_inertia[i] = m_fc_status.jnt_inertia_matrix_measure(i, i);
    }

    // 9.前馈计算(力矩模式下发前馈力矩为0)
    std::fill(m_trq_feedforward.begin(), m_trq_feedforward.end(), 0);

    // 10.数据结构转换
    JntArrayToVector(m_ref_trq, m_ref_trq_vec);
}

void ForceControl::UpDateFcToServo() {
    //整合到m_fc_to_servo
    for (unsigned int i = 0; i < m_jnt_num; i++) {
        m_fc_to_servo.trq_cmd[i] = (int16_t)(m_ref_trq_vec[i] * KDL::sign(m_decel_ratio[i]) / 1000.0 * 32768.0);
        //前馈力矩为0
        m_fc_to_servo.trq_feedforward[i] = (int16_t)(m_trq_feedforward[i] * 1000 / (m_rated_torque[i] * m_decel_ratio[i]));
        m_fc_to_servo.k_p[i] = (int16_t)(m_joint_gain_kp[i] * 100);
        m_fc_to_servo.k_d[i] = (int16_t)(m_joint_damp_zeta[i] * 100);
        m_fc_to_servo.edb_cof[i] = (int16_t)(2.25 / fabs(m_analog2trq_low[i]) * 100.0);
        m_fc_to_servo.edb_cof[i] = (m_fc_to_servo.edb_cof[i] < 90) ? 90 : m_fc_to_servo.edb_cof[i];
        m_fc_to_servo.edb_o[i] = (int16_t)((m_analog_bias[i] - 2500) / 1000.0 / 2.25 * m_analog2trq_high[i] * 100);
        m_fc_to_servo.fric_cof[i] = (int16_t)(m_friction_cof_servo[i] * 100);
        m_fc_to_servo.jnt_inertia[i] = (int16_t)(m_joint_inertia[i] * 100);
    }
}

void ForceControl::ServoDataVectorToArray(int16_t param_0x6071[6], int16_t param_0x60B2[6], int16_t param_0x2201[6],
                                          int16_t param_0x2202[6], int16_t param_0x2203[6], int16_t param_0x2204[6],
                                          int16_t param_0x2205[6], int16_t param_0x2206[6]) {
    std::copy(m_fc_to_servo.trq_cmd.begin(), m_fc_to_servo.trq_cmd.end(), param_0x6071);
    std::copy(m_fc_to_servo.trq_feedforward.begin(), m_fc_to_servo.trq_feedforward.end(), param_0x60B2);
    std::copy(m_fc_to_servo.k_p.begin(), m_fc_to_servo.k_p.end(), param_0x2201);
    std::copy(m_fc_to_servo.k_d.begin(), m_fc_to_servo.k_d.end(), param_0x2202);
    std::copy(m_fc_to_servo.edb_cof.begin(), m_fc_to_servo.edb_cof.end(), param_0x2203);
    std::copy(m_fc_to_servo.edb_o.begin(), m_fc_to_servo.edb_o.end(), param_0x2204);
    std::copy(m_fc_to_servo.fric_cof.begin(), m_fc_to_servo.fric_cof.end(), param_0x2205);
    std::copy(m_fc_to_servo.jnt_inertia.begin(), m_fc_to_servo.jnt_inertia.end(), param_0x2206);
}
//************************************************************************************************

//**************************************编码器数据转化接口******************************************

void ForceControl::GetAxisPos(const std::vector<int32_t>& encoder_value) {
    //临时针对中秒抖动问题加一个保护，编码器突然跳变到0附近，则不更新位置(只针对力矩模式下)
    for (unsigned i = 0; i < m_jnt_num; i++) {
        if (m_servo_to_fc->mode_operation_0x6061[i] == 8) {
            m_jnt_pos[i] = ((encoder_value[i] - m_encoder_offset[i]) * m_encoder_to_jnt_scale[i] / m_decel_ratio[i]);
        } else if (abs(encoder_value[i]) > 10) {
            m_jnt_pos[i] = ((encoder_value[i] - m_encoder_offset[i]) * m_encoder_to_jnt_scale[i] / m_decel_ratio[i]);
        }
    }
    //数据转换
    VectorToJntArray(m_jnt_pos, m_fc_status.jnt_pos_measure);
}

void ForceControl::GetJntVel(const std::vector<int32_t>& encoder_vel_value) {
    for (unsigned int i = 0; i < m_jnt_num; i++) {
        m_jnt_vel_abs[i] = fabs((encoder_vel_value[i] * PI * 2 / 60 / m_decel_ratio[i]));
        m_jnt_vel_real[i] = (encoder_vel_value[i] * PI * 2 / 60 / m_decel_ratio[i]);
    }

    VectorToJntArray(m_jnt_vel_abs, m_fc_status.jnt_vel_measure_abs);
    VectorToJntArray(m_jnt_vel_real, m_fc_status.jnt_vel_measure);
}

void ForceControl::GetSensorTorque(const std::vector<int16_t>& analog_average) {
    for (unsigned i = 0; i < m_jnt_num; i++) {
        //单位：弧度
        m_sensor_trq[i] = (double(analog_average[i]) - m_analog_bias[i]) / 1000.0 * m_analog2trq[i] / m_sensor_amplify[i];
    }

    //数据转换
    VectorToJntArray(m_sensor_trq, m_fc_status.jnt_trq_sensor_measure);
}
//************************************************************************************************

//*******************************内部控制参数计算、配置及状态切换接口部分*****************************

int ForceControl::DragConfig(const int8_t param_0x6061[6], const int16_t param_0x2401[6], const int16_t param_0x2402[6],
                             const int32_t param_0x6064[6], const int32_t param_0x606C[6], const int16_t param_0x2406[6],
                             const LoadInertia& load_params, DragType drag_type, double sensor_trq_feedback[6], double trq_ref[6],
                             double trq_error[6]) {
    int res_drag_config = SOLVE_NOERROR;
    // 1.读取模式相关变量
    m_fc_status.drag_type = drag_type;
    m_fc_status.force_type = ForceType::DRAG;

    for (unsigned int i = 0; i < m_jnt_num; i++) {
        // 1.伺服模式 :TDOO这部分写的不好，有时间再改
        if (param_0x6061[i] == 10) {
            m_fc_status.servo_type[i] = ServoType::FORCE_CONTROL;
            res_drag_config = SERVO_MODE_ERROR;
            return res_drag_config;
        } else if (param_0x6061[i] == 8) {
            m_fc_status.servo_type[i] = ServoType::POSTION_CONTROL;
        } else {
            m_fc_status.servo_type[i] = ServoType::UNKNOWN;
            res_drag_config = SERVO_MODE_ERROR;
            return res_drag_config;
        }

        // 2.传感器平均值计算
        m_analog_ch1[i] = param_0x2401[i];
        m_analog_ch2[i] = param_0x2402[i];
        m_analog_average[i] = (m_analog_ch1[i] + m_analog_ch2[i]) / 2;
    }

    // 2.设置负载
    m_load = load_params;

    //判断负载参数是否合理
    if (m_load.m > m_model_params.max_load) {  //工具质量限制，小于最大负载
        return LOAD_PARAMS_ERROR;
    } else if (m_load.m_cog.Norm() > 0.3) {  //工具TCP长度限制
        return LOAD_PARAMS_ERROR;
    }

    m_fc_status_tracker->SetLoad(m_load);

    // 3.根据负载设置力控参数
    ResetKpByLoad(m_load);
    ResetFricByLoad(m_load);

    // 4.更新fcstatus
    UpdateFcStatus(param_0x6064, param_0x606C, param_0x2406, param_0x6061);

    // 5.判断当前位置是否允许开启拖动
    if (IsInForceControlArea(m_fc_status.jnt_pos_measure, m_joint_range_max_inner, m_joint_range_min_inner) == true) {
        return STARTDRAG_POS_OVER_LIMIT;
    }

    // 6.判断力矩偏差是否允许开启拖动
    GetSensorTorque(m_analog_average);
    res_drag_config = SwichToTrqMode(sensor_trq_feedback, trq_ref, trq_error);
    if (res_drag_config != SOLVE_NOERROR) {
        return res_drag_config;
    }

    // 7.重置控制参数（暂时不需要）

    // 8.更新标志位
    m_enable_drag = true;

    return res_drag_config;
}

void ForceControl::ResetKpByLoad(const LoadInertia& load) {
    //调节范围为0~配置文件最大带宽
    //负载比例系数
    double load_scale = load.m_cog.Norm() * load.m / (0.3 * m_model_params.max_load);
    for (unsigned int i = 0; i < m_jnt_num; i++) {
        m_joint_gain_kp[i] = m_control_params.m_gain_params.joint_gain_kp[i] * m_kp_set_gain[i];
        m_joint_gain_kp[i] = m_joint_gain_kp[i] * (1 - load_scale);

        //幅值限制,SR系列不会超过该阈值
        if (m_joint_gain_kp[i] < 5) {
            m_joint_gain_kp[i] = 5;
        }
        if (m_joint_gain_kp[i] > 50) {
            m_joint_gain_kp[i] = 50;
        }
    }
    //临时对2、3轴进行调整
    if (m_joint_gain_kp[1] < 25) {
        m_joint_gain_kp[1] = 25;
    }
    if (m_joint_gain_kp[2] < 25) {
        m_joint_gain_kp[2] = 25;
    }
}

void ForceControl::ResetFricByLoad(const LoadInertia& load) {
    //调节范围为0~1
    double load_fric_scale = load.m / m_model_params.max_load;

    for (unsigned int i = 0; i < m_jnt_num; i++) {
        if (m_fric_set_gain[i] >= 0.5 && m_fric_set_gain[i] <= 1) {
            m_friction_cof_servo[i] =
                ((m_fric_set_gain[i] - 0.5) / 0.5) * (1 - m_control_params.m_gain_params.friction_cof_servo[i]) +
                m_control_params.m_gain_params.friction_cof_servo[i];
        } else if (m_fric_set_gain[i] >= 0) {
            m_friction_cof_servo[i] = m_friction_cof_servo[i] - ((0.5 - m_fric_set_gain[i]) / 0.5) * m_friction_cof_servo[i];
        }

        // 6轴暂时不响应负载变化
        m_friction_cof_servo[i] = m_friction_cof_servo[i] - load_fric_scale * m_friction_cof_servo[i] / 2;
        //幅值限制,SR系列不会超过该阈值
        if (m_friction_cof_servo[i] < 0) {
            m_friction_cof_servo[i] = 0;
        }

        if (m_friction_cof_servo[i] > 0.9) {
            m_friction_cof_servo[i] = 0.9;
        }

        // if(m_friction_cof_servo[i] < 0.3){m_friction_cof_servo[i] = 0.3;}
        // if(m_friction_cof_servo[5] < 0.4){m_friction_cof_servo[5] = 0.4;}
    }
    //临时改动，2、3轴不能低于0.15
    if (m_friction_cof_servo[1] < 0.15) {
        m_friction_cof_servo[1] = 0.15;
    }
    if (m_friction_cof_servo[2] < 0.15) {
        m_friction_cof_servo[2] = 0.15;
    }
}

int ForceControl::StopDrag(const int8_t param_0x6061[6]) {
    for (unsigned int i = 0; i < m_jnt_num; i++) {
        if (param_0x6061[i] != 8) {
            return SERVO_MODE_ERROR;
        }
    }
    m_enable_drag = false;
    m_is_first_drag = true;
    return SOLVE_NOERROR;
}
//************************************************************************************************

//*******************************外部设置参数接口部分*****************************
int ForceControl::SetSensorLinearity(const std::vector<double> analog2trq_low) {
    if (m_analog2trq_low.size() != analog2trq_low.size()) {
        return SIZE_ERROR;
    }
    m_analog2trq_low = analog2trq_low;
    for (unsigned int i = 0; i < m_jnt_num; i++) {
        m_analog2trq[i] = m_analog2trq_high[i] / m_analog2trq_low[i];
    }

    return SOLVE_NOERROR;
}

int ForceControl::SetSensorBias(const std::vector<double> analog_bias) {
    if (m_analog_bias.size() != analog_bias.size()) {
        return SIZE_ERROR;
    }
    m_analog_bias = analog_bias;
    return SOLVE_NOERROR;
}

int ForceControl::SetEncoderOffset(const std::vector<int32_t> encoder_offset) {
    if (m_encoder_offset.size() != encoder_offset.size()) {
        return SIZE_ERROR;
    }
    m_encoder_offset = encoder_offset;
    return SOLVE_NOERROR;
}

int ForceControl::SetKpGain(const std::vector<double> kp_set) {
    if (m_kp_set_gain.size() != kp_set.size()) {
        return SIZE_ERROR;
    }
    for (unsigned int i = 0; i < kp_set.size(); i++) {
        if (kp_set[i] < 0 or kp_set[i] > 1) {
            return GAIN_VALUE_ERROR;
        }
    }
    m_kp_set_gain = kp_set;
    return SOLVE_NOERROR;
}

int ForceControl::SetFricGain(const std::vector<double> fric_set) {
    if (m_fric_set_gain.size() != fric_set.size()) {
        return SIZE_ERROR;
    }
    for (unsigned int i = 0; i < fric_set.size(); i++) {
        if (fric_set[i] < 0 or fric_set[i] > 1) {
            return GAIN_VALUE_ERROR;
        }
    }
    m_fric_set_gain = fric_set;
    return SOLVE_NOERROR;
}

int ForceControl::SetSoftLimit(const std::vector<double> joint_range_min, const std::vector<double> joint_range_max) {
    int res = SOLVE_NOERROR;

    //长度检查
    if (joint_range_min.size() != m_joint_range_min.size()) {
        return SIZE_ERROR;
    }
    if (joint_range_max.size() != m_joint_range_max.size()) {
        return SIZE_ERROR;
    }

    //数据有效性检查
    for (unsigned int i = 0; i < m_jnt_num; i++) {
        if ((joint_range_min[i] < m_joint_range_min_new[i]) or (joint_range_max[i] > m_joint_range_max_new[i])) {
            return SOFT_LIMIT_PARAMS_ERROR;
        }
    }

    //更新软限位成员变量
    for (unsigned int j = 0; j < m_jnt_num; j++) {
        m_joint_range_min[j] = joint_range_min[j] / 180 * PI;
        m_joint_range_max[j] = joint_range_max[j] / 180 * PI;
    }
    //设置软限位
    res = m_force_planner_ptr->SetSoftLimit(m_joint_range_min, m_joint_range_max);

    return res;
}

int ForceControl::CalibrateTrqSensor(const int32_t param_0x6064[6], const LoadInertia& load_params_in,
                                     const int16_t analog_array_ch1[6][200], const int16_t analog_array_ch2[6][200],
                                     double sensor_bias[6]) {
    std::vector<double> analog_read;
    std::vector<int32_t> pos_encoder_input;
    KDL::JntArray trq_gra_jntarray;
    KDL::JntArray q_in_jntarray;

    trq_gra_jntarray.resize(m_jnt_num);
    analog_read.resize(m_jnt_num);
    pos_encoder_input.resize(m_jnt_num);
    q_in_jntarray.resize(m_jnt_num);

    //数据转换&计算关节角度
    std::copy(param_0x6064, param_0x6064 + 6, std::begin(pos_encoder_input));
    for (unsigned int i = 0; i < m_jnt_num; i++) {
        q_in_jntarray(i) = double((pos_encoder_input[i] - m_encoder_offset[i]) * m_encoder_to_jnt_scale[i] / m_decel_ratio[i]);
    }

    //计算当前位置模型力矩
    trq_gra_jntarray = m_fc_dynamic_solver->GetGraTorque(load_params_in, q_in_jntarray);

    //计算200次读取传感器电压的平均值
    for (unsigned int i = 0; i < m_jnt_num; i++) {
        for (unsigned int j = 0; j < 200; j++) {
            analog_read[i] += double((analog_array_ch1[i][j] + analog_array_ch2[i][j]) / 2);
        }
        analog_read[i] = analog_read[i] / 200;
    }

    //计算传感器零点
    for (unsigned int i = 0; i < m_jnt_num; i++) {
        sensor_bias[i] =
            analog_read[i] - (trq_gra_jntarray(i) * m_sensor_amplify[i] * 1000 * m_analog2trq_low[i]) / m_analog2trq_high[i];
        //传感器零点一般不会超过2.5V±50%的误差，如果超过，就意味着传感器失效或负载信息错误。
        if ((sensor_bias[i] > 3750) or (sensor_bias[i] < 1250)) {
            return SENSOR_BIAS_ERROR;
        }
    }

    return SOLVE_NOERROR;
}

int ForceControl::CalibrateTrqSensorAxis(const int32_t param_0x6064[6], const LoadInertia& load_params_in,
                                         const int16_t analog_array_ch1[200], const int16_t analog_array_ch2[200],
                                         const unsigned int axis_num, double sensor_bias_axis[6]) {
    if (axis_num > m_jnt_num or axis_num < 1) {
        return AXIS_NUM_ERROR;
    }
    double analog_read = 0;
    std::vector<int32_t> pos_encoder_input;
    KDL::JntArray trq_gra_jntarray;
    KDL::JntArray q_in_jntarray;

    trq_gra_jntarray.resize(m_jnt_num);
    pos_encoder_input.resize(m_jnt_num);
    q_in_jntarray.resize(m_jnt_num);

    //数据转换&计算关节角度
    std::copy(param_0x6064, param_0x6064 + 6, std::begin(pos_encoder_input));
    for (unsigned int i = 0; i < m_jnt_num; i++) {
        q_in_jntarray(i) = double((pos_encoder_input[i] - m_encoder_offset[i]) * m_encoder_to_jnt_scale[i] / m_decel_ratio[i]);
    }

    //计算当前位置模型力矩
    trq_gra_jntarray = m_fc_dynamic_solver->GetGraTorque(load_params_in, q_in_jntarray);

    //计算200次读取传感器电压的平均值
    for (unsigned int i = 0; i < 200; i++) {
        analog_read += double((analog_array_ch1[i] + analog_array_ch2[i]) / 2);
    }
    analog_read = analog_read / 200;

    //先给非标定轴的sensor_bias赋值
    for (unsigned int i = 0; i < m_jnt_num; i++) {
        sensor_bias_axis[i] = m_analog_bias[i];
    }

    //计算特定轴传感器零点
    sensor_bias_axis[axis_num - 1] =
        analog_read - (trq_gra_jntarray(axis_num - 1) * m_sensor_amplify[axis_num - 1] * 1000 * m_analog2trq_low[axis_num - 1]) /
                          m_analog2trq_high[axis_num - 1];

    //传感器零点一般不会超过2.5V±50%的误差，如果超过，就意味着传感器失效或负载信息错误。
    if ((sensor_bias_axis[axis_num - 1] > 3750) or (sensor_bias_axis[axis_num - 1] < 1250)) {
        return SENSOR_BIAS_ERROR;
    }

    return SOLVE_NOERROR;
}
//************************************************************************************************

//**************************************保护接口部分***********************************************
int ForceControl::SwichToTrqMode(double sensor_trq_feedback[6], double trq_ref[6], double trq_error[6]) {
    bool if_error_inside = true;  //如果if_error_inside最后输出为1，则表明误差小于阈值，若等于0，则误差大于阈值
    for (unsigned int i = 0; i < m_jnt_num; i++) {
        sensor_trq_feedback[i] = m_sensor_trq[i];
        trq_ref[i] = m_ref_trq_vec[i];
        trq_error[i] = fabs(sensor_trq_feedback[i] - trq_ref[i]);
        if (trq_error[i] > m_trq_error_max[i]) {
            if_error_inside = false;  //只要有一个轴超过阈值，if_error_inside就为false
        }
    }
    if (if_error_inside == true) {
        return SOLVE_NOERROR;
    } else {
        return EXCESSIVE_TORQUE_ERROR;
    }
}

bool ForceControl::IsInForceControlArea(const JntArray& q_in, const std::vector<double>& joint_limit_upper,
                                        const std::vector<double>& joint_limit_lower) {
    for (unsigned int i = 0; i < m_jnt_num; i++) {
        if ((q_in(i) < (joint_limit_lower[i] + 9.5 * KDL::deg2rad)) or (q_in(i) > (joint_limit_upper[i] - 9.5 * KDL::deg2rad))) {
            return true;
        }
    }
    return false;
}
//************************************************************************************************

}  // namespace Control
}  // namespace RokaeApi