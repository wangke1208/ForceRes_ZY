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
// 构造函数
ForceControl::ForceControl(InitRobot* init_robot_ptr)
    : m_init_robot_ptr(init_robot_ptr),
      m_jnt_num(init_robot_ptr->GetJntNum()),
      m_chain(init_robot_ptr->GetChain()),
      m_fc_status_inner(init_robot_ptr->GetJntNum()),
      m_fc_status_outer(init_robot_ptr->GetJntNum()),
      m_servo_data_fc_inner(init_robot_ptr->GetJntNum()),
      m_fc_inner_servo_data(init_robot_ptr->GetJntNum()) {
    // 初始化一些求解器
    m_fc_params_inner_ptr = new FcParamsInner(m_jnt_num);
    m_fkpos_ptr = new KDL::ChainFkSolverPos_recursive(m_chain);
    m_dynamicsolver_ptr = new DynamicSolver(m_chain, m_init_robot_ptr->GetGravity());
    m_force_protect_ptr = new Protect::ForceProtect(m_jnt_num, m_fc_params_inner_ptr);
    m_servo_fc_convert_ptr = new Servo_Fc_Convert(m_jnt_num, m_init_robot_ptr->GetMechanicalParams());
    m_fc_status_tracker_ptr = new FcStatusTracker(m_init_robot_ptr, &m_fc_status_inner, m_fc_params_inner_ptr);
    m_force_planner_ptr = new Control::ForcePlanner(m_init_robot_ptr, &m_fc_status_inner, m_fc_params_inner_ptr);

    // 初始化负载信息
    m_load.SetZero();

    // 初始化拖动类型
    m_drag_type = DragType::DRAG_JOINT;

    // 初始化拖动标志位
    m_enable_drag = false;
    m_is_first_drag = true;

    // 调整内部数据结构大小

    // 初始化参数
    m_servo_data_fc_inner.Resize(m_jnt_num);
    m_joint_range_min_inner.resize(m_jnt_num);
    m_joint_range_max_inner.resize(m_jnt_num);
    m_kp_gain_set.resize(m_jnt_num, DEFAULT_KP_GAIN);
    m_fri_gain_set.resize(m_jnt_num, DEFAULT_FRIC_GAIN);
    m_kp_set_by_load.resize(m_jnt_num);
    m_fri_set_by_load.resize(m_jnt_num);
    m_load_mass_limit.resize(1);
    m_load_tcp_length_limit.resize(1);
}

ForceControl::~ForceControl() {
    delete m_force_protect_ptr;
    delete m_dynamicsolver_ptr;
    delete m_fkpos_ptr;
    delete m_fc_status_tracker_ptr;
    delete m_servo_fc_convert_ptr;
    delete m_force_planner_ptr;
}

int ForceControl::Fcinit() {
    // 初始化一些参数
    // 1.编码器零点
    // 2.传感器零点
    // 3.传感器线性度
    // 4.关节限位
    // 5.拖动力矩限制
    // 5.伺服kp
    // 6.伺服摩擦力补偿
    // 7.伺服阻尼比
    std::vector<std::function<int()>> func_calls = {
        [this]() { return SetEncoderOffset(m_init_robot_ptr->GetMechanicalParams().encoder_offset); },
        [this]() { return SetSensorBias(m_init_robot_ptr->GetMechanicalParams().analog_bias); },
        [this]() { return SetSensorLinearity(m_init_robot_ptr->GetMechanicalParams().analog2trq_low); },
        [this]() {
            return SetSoftLimit(m_init_robot_ptr->GetModelParams().joint_range_min,
                                m_init_robot_ptr->GetModelParams().joint_range_max);
        },
        [this]() { return SetMaxTrqErrorThreshold(m_init_robot_ptr->GetControlParams().m_protect_params.max_mode_switch_trq); },
        [this]() { return SetKpGain(m_kp_gain_set); },
        [this]() { return SetFricGain(m_fri_gain_set); },
        [this]() { return ResetKpByLoad(m_load); },
        [this]() { return ResetFricByLoad(m_load); },
        [this]() { return SetZetaGain(m_init_robot_ptr->GetControlParams().m_gain_params.joint_damp_zeta); },
        [this]() { return SetImpedenceGain(DragType::DRAG_JOINT); },
        [this]() {
            return SetLoadLimit(m_init_robot_ptr->GetModelParams().max_load,
                                m_init_robot_ptr->GetModelParams().max_load_tcp_length);
        }};

    for (const auto& call : func_calls) {
        if (call() != SOLVE_NOERROR) {
            return INIT_ERROR;
        }
    }
    return SOLVE_NOERROR;
}

int ForceControl::DragConfig(const std::vector<int32_t>& pos_encoder_from_servo, const std::vector<int8_t>& servo_mode_from_servo,
                             const std::vector<int16_t>& analog_ch1, const std::vector<int16_t>& analog_ch2,
                             const DragType& drag_type) {
    // 0.初始化标志位
    int res = SOLVE_NOERROR;
    m_enable_drag = false;
    m_is_first_drag = true;

    // 1.判断是否允许进行Drag设置
    // 1.1 处于位置模式下
    if (std::any_of(servo_mode_from_servo.cbegin(), servo_mode_from_servo.cend(),
                    [](int8_t servo_type) { return servo_type != POSITION_MODE; })) {
        return SERVO_MODE_ERROR;
    }

    // 1.2 计算当前位置
    std::vector<double> jnt_pos_rad_temp(m_jnt_num);
    m_servo_fc_convert_ptr->GetAxisPos(pos_encoder_from_servo, jnt_pos_rad_temp);

    // 1.3 力矩偏差在合理范围
    std::vector<double> sensor_trq_temp(m_jnt_num);
    auto model_trq_temp = m_dynamicsolver_ptr->GetGraTorque(m_load.m_rokae_load_inertia, VectorToJntArray(jnt_pos_rad_temp));
    m_servo_fc_convert_ptr->GetCobotTrq(analog_ch1, analog_ch2, sensor_trq_temp);
    res = m_force_protect_ptr->TrqErrorProtect(sensor_trq_temp, JntArrayToVector(model_trq_temp));
    if (res != SOLVE_NOERROR) {
        return res;
    }

    // 1.4 当前机器人位置不处在软限位保护范围内
    if (m_force_protect_ptr->IsInForceControlArea(VectorToJntArray(jnt_pos_rad_temp), m_joint_range_max_inner,
                                                  m_joint_range_min_inner)) {
        return STARTDRAG_POS_OVER_LIMIT;
    }

    // 2.设置拖动模式
    if (drag_type < 0 || drag_type > 3) {
        return DRAGTYPE_ERROR;
    }
    m_drag_type = drag_type;

    // 3.设置数据流计算负载参数(同一放到外部接口设置)
    // m_fc_status_tracker_ptr->SetLoad(m_load);

    // 4.设置拖动相关增益(不在每次开启拖动都设置了，通过接口设置即可)

    // 5.设置阻抗相关增益(该接口暂不开放，内部参数固定设置)
    SetImpedenceGain(m_drag_type);

    // 6.更新标志位
    m_enable_drag = true;

    return SOLVE_NOERROR;
}

void ForceControl::SetFcCommand(const Servo_To_FcInner& servo_data_fc_inner) {
    // 1.计算当前关节位置&速度&传感器反馈
    m_servo_fc_convert_ptr->GetAxisPos(servo_data_fc_inner.pos_feedback, m_fc_status_inner.jnt_pos_measure);
    m_servo_fc_convert_ptr->GetAxisVel(servo_data_fc_inner.vel_feedback, m_fc_status_inner.jnt_vel_measure);
    m_servo_fc_convert_ptr->GetCobotTrq(servo_data_fc_inner.analog_ch1, servo_data_fc_inner.analog_ch2,
                                        m_fc_status_inner.jnt_trq_sensor_measure);

    // 2.赋值拖动类型
    m_fc_status_inner.drag_type = m_drag_type;

    // 计算实际位置
    switch (m_drag_type) {
    case DragType::DRAG_JOINT:
        if (m_is_first_drag) {
            m_fc_status_inner.jnt_pos_command = m_fc_status_inner.jnt_pos_measure;
            m_fc_status_inner.jnt_vel_command.data.setZero();  // 速度指令给0
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
                           std::vector<int16_t>& fc_trq_cmd_to_servo, std::vector<int16_t>& fc_trq_feedforward_to_servo,
                           std::vector<int16_t>& fc_kp_to_servo, std::vector<int16_t>& fc_kd_to_servo,
                           std::vector<int16_t>& fc_edb_cof_to_servo, std::vector<int16_t>& fc_edb_o_to_servo,
                           std::vector<int16_t>& fc_fric_cof_to_servo, std::vector<int16_t>& fc_jnt_inertia_to_servo) {
    int res = SOLVE_NOERROR;

    // 1.判断是否进行了drag_config
    if (!m_enable_drag) {
        return ERROR_DRAG_ENABLE;
    }

    // 1.1 当前非力矩模式不允许调用本接口(双重保护,避免drag_config后又置为位置模式)
    if (std::any_of(servo_mode_from_servo.cbegin(), servo_mode_from_servo.cend(),
                    [](int8_t servo_type) { return servo_type != TORQUE_MODE; })) {
        return SERVO_MODE_ERROR;
    }

    // 2.读取伺服数据并转换为Fc内部变量
    res = m_servo_fc_convert_ptr->ServoData2FcInner(servo_mode_from_servo, pdo_analog_ch1, pdo_analog_ch2, trq_encoder_from_servo,
                                                    pos_encoder_from_servo, vel_encoder_from_servo, m_servo_data_fc_inner);
    if (res != SOLVE_NOERROR) {
        return res;
    }

    // 3.更新指令和反馈
    SetFcCommand(m_servo_data_fc_inner);

    // 3.力控数据流计算
    res = m_fc_status_tracker_ptr->FcStatusUpdata();
    if (res != SOLVE_NOERROR) {
        return res;
    }

    // 4.力控模块功能力计算
    m_force_planner_ptr->ForcePlannerUpdata();

    // 5.根据负载参数更新下发给伺服的增益
    res = (ResetKpByLoad(m_load) && ResetFricByLoad(m_load));
    if (res != SOLVE_NOERROR) {
        return res;
    }

    // 6.将Fc内部数据转换为下发给伺服数据
    m_servo_fc_convert_ptr->FcData2ServoData(m_fc_status_inner, m_fc_params_inner_ptr, m_fc_inner_servo_data);
    std::copy(m_fc_inner_servo_data.trq_cmd.cbegin(), m_fc_inner_servo_data.trq_cmd.cend(), fc_trq_cmd_to_servo.begin());
    std::copy(m_fc_inner_servo_data.trq_feedforward.cbegin(), m_fc_inner_servo_data.trq_feedforward.cend(),
              fc_trq_feedforward_to_servo.begin());
    std::copy(m_fc_inner_servo_data.k_p.cbegin(), m_fc_inner_servo_data.k_p.cend(), fc_kp_to_servo.begin());
    std::copy(m_fc_inner_servo_data.k_d.cbegin(), m_fc_inner_servo_data.k_d.cend(), fc_kd_to_servo.begin());
    std::copy(m_fc_inner_servo_data.edb_cof.cbegin(), m_fc_inner_servo_data.edb_cof.cend(), fc_edb_cof_to_servo.begin());
    std::copy(m_fc_inner_servo_data.edb_o.cbegin(), m_fc_inner_servo_data.edb_o.cend(), fc_edb_o_to_servo.begin());
    std::copy(m_fc_inner_servo_data.fric_cof.cbegin(), m_fc_inner_servo_data.fric_cof.cend(), fc_fric_cof_to_servo.begin());
    std::copy(m_fc_inner_servo_data.jnt_inertia.cbegin(), m_fc_inner_servo_data.jnt_inertia.cend(),
              fc_jnt_inertia_to_servo.begin());

    // 7.外部数据copy
    FcStatusCopy(m_fc_status_inner);

    return SOLVE_NOERROR;
}

int ForceControl::SetSensorLinearity(const std::vector<double>& analog2trq_low) {
    if (m_jnt_num != analog2trq_low.size()) {
        return SIZE_ERROR;
    }
    m_fc_params_inner_ptr->m_hardware_params.SetParam("analog2trq_low", m_init_robot_ptr->GetMechanicalParams().analog2trq_low);
    m_servo_fc_convert_ptr->SetSensorLinearity(analog2trq_low);

    return SOLVE_NOERROR;
}

int ForceControl::SetSensorBias(const std::vector<double>& analog_bias) {
    if (m_jnt_num != analog_bias.size()) {
        return SIZE_ERROR;
    }
    m_fc_params_inner_ptr->m_hardware_params.SetParam("analog_bias", m_init_robot_ptr->GetMechanicalParams().analog_bias);
    m_servo_fc_convert_ptr->SetSensorBias(analog_bias);
    return SOLVE_NOERROR;
}

int ForceControl::SetEncoderOffset(const std::vector<int32_t>& encoder_offset) {
    if (m_jnt_num != encoder_offset.size()) {
        return SIZE_ERROR;
    }
    std::vector<double> encoder_offset_temp(encoder_offset.begin(), encoder_offset.end());
    m_fc_params_inner_ptr->m_hardware_params.SetParam("encoder_offset", (encoder_offset_temp));
    m_servo_fc_convert_ptr->SetEncoderBias(encoder_offset);
    return SOLVE_NOERROR;
}

int ForceControl::SetSoftLimit(const std::vector<double>& joint_range_min, const std::vector<double>& joint_range_max) {
    int res = SOLVE_NOERROR;

    // 长度检查
    if (joint_range_min.size() != m_jnt_num) {
        return SIZE_ERROR;
    }
    if (joint_range_max.size() != m_jnt_num) {
        return SIZE_ERROR;
    }

    // 数据有效性检查
    for (unsigned int i = 0; i < m_jnt_num; ++i) {
        // 负软限位小于0，正软限位大于0，或者软限位超出机械硬限位
        if (joint_range_min[i] < 0 || joint_range_max[i] > 0 || 
            joint_range_min[i] < m_init_robot_ptr->GetModelParams().joint_range_min_new[i] ||
            joint_range_max[i] > m_init_robot_ptr->GetModelParams().joint_range_max_new[i]) {
            return SOFT_LIMIT_PARAMS_ERROR;
        }
    }

    // 更新内部软限位成员变量
    for (unsigned int j = 0; j < m_jnt_num; j++) {
        m_joint_range_min_inner[j] = joint_range_min[j] / 180 * PI;
        m_joint_range_max_inner[j] = joint_range_max[j] / 180 * PI;
    }
    m_fc_params_inner_ptr->m_hardware_params.SetParam("joint_angle_limit_min", m_joint_range_min_inner);
    m_fc_params_inner_ptr->m_hardware_params.SetParam("joint_angle_limit_max", m_joint_range_max_inner);

    // 设置软限位
    m_force_planner_ptr->SetSoftLimit(m_joint_range_min_inner, m_joint_range_max_inner);

    return SOLVE_NOERROR;
}

int ForceControl::SetMaxTrqErrorThreshold(const std::vector<double>& m_max_trq_error_threshold) {
    if (m_jnt_num != m_max_trq_error_threshold.size()) {
        return SIZE_ERROR;
    }
    m_fc_params_inner_ptr->m_protect_params.SetParam("max_mode_switch_trq", m_max_trq_error_threshold);
    return SOLVE_NOERROR;
}

int ForceControl::SetZetaGain(const std::vector<double>& zeta_set) {
    if (m_jnt_num != zeta_set.size()) {
        return SIZE_ERROR;
    }
    for (unsigned int i = 0; i < zeta_set.size(); i++) {
        if (zeta_set[i] < 0 or zeta_set[i] > 1.5) {
            return GAIN_VALUE_ERROR;
        }
    }
    m_fc_params_inner_ptr->m_function_params.SetParam("joint_servo_dmap_kv", zeta_set);

    return SOLVE_NOERROR;
}

int ForceControl::SetImpedenceGain(const DragType& drag_type) {
    if (m_drag_type != drag_type) {
        return DRAGTYPE_ERROR;
    }

    // 1.先设置默认拖动参数
    m_fc_params_inner_ptr->m_function_params.SetFreeDragParams();

    // 2.根据拖动类型设置参数(暂时不支持外部设置参数，在配置文件中写死)
    switch (drag_type) {
    case DragType::DRAG_CART_ROT:
        m_fc_params_inner_ptr->m_function_params.SetRotParams(
            m_init_robot_ptr->GetControlParams().m_gain_params.rot_drag_trans_stiff,
            m_init_robot_ptr->GetControlParams().m_gain_params.rot_drag_trans_damp);
        return SOLVE_NOERROR;
    case DragType::DRAG_CART_TRANS:
        m_fc_params_inner_ptr->m_function_params.SetTransParams(
            m_init_robot_ptr->GetControlParams().m_gain_params.trans_drag_rot_stiff,
            m_init_robot_ptr->GetControlParams().m_gain_params.trans_drag_rot_damp);
        return SOLVE_NOERROR;
    default:
        return SOLVE_NOERROR;
    }
}

int ForceControl::SetLoadLimit(const double& max_load_mass, const double& max_load_tcp_length) {
    if (max_load_mass <= 0 || max_load_tcp_length <= 0) {
        return LOAD_LIMIT_PARAMS_ERROR;
    }
    m_load_mass_limit[0] = max_load_mass;
    m_load_tcp_length_limit[0] = max_load_tcp_length;

    m_fc_params_inner_ptr->m_protect_params.SetParam("max_load_mass", m_load_mass_limit);
    m_fc_params_inner_ptr->m_protect_params.SetParam("max_load_tcp_length", m_load_tcp_length_limit);
    return SOLVE_NOERROR;
}

int ForceControl::SetFcLoad(const RokaeLoad& load) {
    // 判断负载参数是否合理
    if (load.m_rokae_load_inertia.mass >
        m_fc_params_inner_ptr->m_protect_params.m_params["max_load_mass"].at(0)) {  // 工具质量限制，小于最大负载
        return LOAD_PARAMS_ERROR;
    } else if (load.m_rokae_load_inertia.GetCOG().Norm() > 0.3) {  // 工具TCP长度限制
        return LOAD_PARAMS_ERROR;
    }

    KDL::Vector rpy_rad = load.m_rokae_load_pose.eulerangles / 180.0 * PI;

    // forcecontrol内部负载设置
    m_load.SetRokaeLoadInertia(load.m_rokae_load_inertia);
    m_load.m_rokae_load_pose.SetSpatiaPos(load.m_rokae_load_pose.spatiapos);
    m_load.m_rokae_load_pose.SetRotAngle(rpy_rad);

    // fc_status_tracker数据流计算参数设置
    m_fc_status_tracker_ptr->SetLoad(m_load);
    return SOLVE_NOERROR;
}

int ForceControl::SetKpGain(const std::vector<double>& kp_gain_set) {
    if (m_jnt_num != kp_gain_set.size()) {
        return SIZE_ERROR;
    }
    for (unsigned int i = 0; i < kp_gain_set.size(); i++) {
        if (kp_gain_set[i] < 0 or kp_gain_set[i] > 1) {
            return GAIN_VALUE_ERROR;
        }
    }
    m_fc_params_inner_ptr->m_function_params.SetParam("kp_gain_set", kp_gain_set);
    std::copy(kp_gain_set.cbegin(), kp_gain_set.cend(), m_kp_gain_set.begin());
    return SOLVE_NOERROR;
}

int ForceControl::SetFricGain(const std::vector<double>& fric_gain_set) {
    if (m_jnt_num != fric_gain_set.size()) {
        return SIZE_ERROR;
    }
    for (unsigned int i = 0; i < fric_gain_set.size(); i++) {
        if (fric_gain_set[i] < 0 or fric_gain_set[i] > 1) {
            return GAIN_VALUE_ERROR;
        }
    }
    m_fc_params_inner_ptr->m_function_params.SetParam("fri_gain_set", fric_gain_set);
    std::copy(fric_gain_set.cbegin(), fric_gain_set.cend(), m_fri_gain_set.begin());
    return SOLVE_NOERROR;
}

int ForceControl::ResetKpByLoad(const RokaeLoad& load) {
    double load_scale = load.m_rokae_load_inertia.GetCOG().Norm() * load.m_rokae_load_inertia.mass /
                        (m_load_tcp_length_limit[0] * m_load_mass_limit[0]);
    for (unsigned int i = 0; i < m_jnt_num; i++) {
        m_kp_set_by_load[i] = m_init_robot_ptr->GetControlParams().m_gain_params.joint_gain_kp[i] * m_kp_gain_set[i];
        m_kp_set_by_load[i] = m_kp_set_by_load[i] * (1 - load_scale);
    }
    // 更新到FcParams中
    m_fc_params_inner_ptr->m_function_params.SetParam("joint_servo_kp", m_kp_set_by_load);
    return SOLVE_NOERROR;
}

int ForceControl::ResetFricByLoad(const RokaeLoad& load) {
    // 调节范围为0~1
    double load_fric_scale = load.m_rokae_load_inertia.mass / m_load_mass_limit[0];

    for (unsigned int i = 0; i < m_jnt_num; i++) {
        if (m_fri_gain_set[i] >= 0.5 && m_fri_gain_set[i] <= 1) {
            m_fri_set_by_load[i] = ((m_fri_gain_set[i] - 0.5) / 0.5) *
                                       (1 - m_init_robot_ptr->GetControlParams().m_gain_params.friction_cof_servo[i]) +
                                   m_init_robot_ptr->GetControlParams().m_gain_params.friction_cof_servo[i];
        } else if (m_fri_gain_set[i] >= 0) {
            m_fri_set_by_load[i] = m_fri_set_by_load[i] - ((0.5 - m_fri_gain_set[i]) / 0.5) * m_fri_set_by_load[i];
        }

        m_fri_set_by_load[i] = m_fri_set_by_load[i] - load_fric_scale * m_fri_set_by_load[i] / 2;
    }

    m_fc_params_inner_ptr->m_function_params.SetParam("joint_servo_friction", m_fri_set_by_load);
    return SOLVE_NOERROR;
}

void ForceControl::SetGravity(const Vector& gravity) {
    m_dynamicsolver_ptr->SetGravity(gravity);
    m_fc_status_tracker_ptr->SetGravity(gravity);
}

int ForceControl::ResetFcStatus() {
    // for (unsigned int i = 0; i < m_jnt_num; i++) {
    //     if (servo_mode[i] != POSITION_MODE) {
    //         return SERVO_MODE_ERROR;
    //     }
    // }

    // 1.重置内部状态参数
    m_enable_drag = false;
    m_is_first_drag = true;

    // 2.重置拖动类型
    m_drag_type = DragType::DRAG_JOINT;

    // 3.重置负载信息
    m_load.SetZero();
    SetFcLoad(m_load);

    // 4.重置力控增益
    m_kp_gain_set.assign(m_jnt_num, DEFAULT_KP_GAIN);
    m_fri_gain_set.assign(m_jnt_num, DEFAULT_FRIC_GAIN);
    SetKpGain(m_kp_gain_set);
    SetFricGain(m_fri_gain_set);
    ResetKpByLoad(m_load);
    ResetFricByLoad(m_load);

    // 5.重置阻抗增益
    SetImpedenceGain(m_drag_type);

    return SOLVE_NOERROR;
}

void ForceControl::FcStatusRefresh() {
    // 重置内部状态参数
    m_enable_drag = false;
    m_is_first_drag = true;
    m_fc_status_tracker_ptr->ResetCalStatus();
    return;
}

int ForceControl::CalibrateTrqSensor(const std::vector<int32_t>& pos_encoder_feedback, const RokaeLoad& load_input,
                                     const std::vector<std::array<int16_t, ANALOG_DATA_COUNT>>& analog_array_ch1,
                                     const std::vector<std::array<int16_t, ANALOG_DATA_COUNT>>& analog_array_ch2,
                                     std::vector<double>& sensor_bias) {
    // 长度检查
    if (pos_encoder_feedback.size() != m_jnt_num || analog_array_ch1.size() != m_jnt_num ||
        analog_array_ch2.size() != m_jnt_num) {
        return SIZE_ERROR;
    }

    KDL::JntArray trq_gra_jntarray;
    KDL::JntArray q_in_jntarray;
    std::vector<double> analog_average(m_jnt_num);
    trq_gra_jntarray.resize(m_jnt_num);
    q_in_jntarray.resize(m_jnt_num);

    // 1.计算当前关节角度
    m_servo_fc_convert_ptr->GetAxisPos(pos_encoder_feedback, q_in_jntarray);

    // 2.计算当前模型重力矩
    trq_gra_jntarray = m_dynamicsolver_ptr->GetGraTorque(load_input.m_rokae_load_inertia, q_in_jntarray);

    // 3.提取200次电压数据的平均值
    for (unsigned int i = 0; i < m_jnt_num; i++) {
        for (unsigned int j = 0; j < ANALOG_DATA_COUNT; j++) {
            analog_average[i] += double((analog_array_ch1[i][j] + analog_array_ch2[i][j]) / 2);
        }
        analog_average[i] = analog_average[i] / ANALOG_DATA_COUNT;
    }

    // 4.计算传感器零点
    int res = m_servo_fc_convert_ptr->GetAnalogBias(trq_gra_jntarray, analog_average, sensor_bias);

    return res;
}

//外部获取接口
const FcStatusInner& ForceControl::GetFcStatusCopy() {
    std::lock_guard<std::mutex> lock(m_mutex);
    return m_fc_status_outer;
}
void ForceControl::FcStatusCopy(const FcStatusInner& fc_status_in) {
    std::lock_guard<std::mutex> lock(m_mutex);
    m_fc_status_outer = fc_status_in;
}

}  // namespace Control
}  // namespace RokaeApi