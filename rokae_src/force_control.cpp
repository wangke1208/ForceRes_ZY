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
ForceControl* ForceControl::force_control_instance_ptr = nullptr;
ForceControl::ForceControl(InitRobot* init_robot_ptr)
    : m_init_robot_ptr(init_robot_ptr),
      m_jnt_num(m_init_robot_ptr->GetJntNum()),
      m_chain(init_robot_ptr->GetChain()),
      m_fc_status_inner(init_robot_ptr->GetJntNum()),
      m_servo_data_fc_inner(init_robot_ptr->GetJntNum()),
      m_fc_inner_servo_data(init_robot_ptr->GetJntNum()) {
    //初始化一些求解器
    m_fc_params_inner_ptr = new FcParamsInner(m_jnt_num);

    m_force_protect_ptr = new Protect::ForceProtect(m_jnt_num, m_fc_params_inner_ptr);
    m_dynamicsolver_ptr = new DynamicSolver(m_chain, m_init_robot_ptr->GetGravity());
    m_fkpos_ptr = new KDL::ChainFkSolverPos_recursive(m_chain);
    m_fc_status_tracker_ptr = new FcStatusTracker(m_init_robot_ptr, &m_fc_status_inner, m_fc_params_inner_ptr);
    m_servo_fc_convert_ptr = new Servo_Fc_Convert(m_jnt_num, m_init_robot_ptr->GetMechanicalParams());
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
    m_analog2trq_low.resize(m_jnt_num);

    m_joint_range_min_inner.resize(m_jnt_num);
    m_joint_range_max_inner.resize(m_jnt_num);
    m_joint_range_min_new_inner.resize(m_jnt_num);
    m_joint_range_max_new_inner.resize(m_jnt_num);

    m_joint_gain_kp_inner.resize(m_jnt_num);
    m_joint_damp_zeta_inner.resize(m_jnt_num);
    m_friction_cof_servo_inner.resize(m_jnt_num);

    m_ref_trq.resize(m_jnt_num);

    m_analog_ch1.resize(m_jnt_num);
    m_analog_ch2.resize(m_jnt_num);
    m_analog_average.resize(m_jnt_num);
    m_sensor_trq.resize(m_jnt_num);

    m_jnt_current_pos.resize(m_jnt_num);
    // m_jnt_vel_abs.resize(m_jnt_num);
    // m_jnt_vel_real.resize(m_jnt_num);

    m_trq_error.resize(m_jnt_num);
    m_zero_vector.resize(m_jnt_num, 0);

    m_kp_gain_set.resize(m_jnt_num, 1.0);
    m_fri_gain_set.resize(m_jnt_num, 0.5);
    m_kp_set_by_load.resize(m_jnt_num);
    m_fri_set_by_load.resize(m_jnt_num);
    m_load_mass_limit.resize(1);
    m_load_tcp_length_limit.resize(1);
}

ForceControl::~ForceControl() {
    delete m_force_protect_ptr;
    delete m_dynamicsolver_ptr;
    delete m_fc_status_tracker_ptr;
    delete m_servo_fc_convert_ptr;
}

void ForceControl::InitInStance(InitRobot* init_robot_ptr) {
    if (force_control_instance_ptr == nullptr) {
        force_control_instance_ptr = new ForceControl(init_robot_ptr);
    }
}

void ForceControl::ReleaseInstance() {
    if (force_control_instance_ptr) {
        delete force_control_instance_ptr;
        force_control_instance_ptr = nullptr;
    }
}
ForceControl* ForceControl::GetInstance() { return force_control_instance_ptr; }
int ForceControl::Fcinit() {
    //初始化一些参数
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

void ForceControl::SetFcCommand(const Servo_To_FcInner& servo_data_fc_inner) {
    // 1.计算当前关节位置
    m_servo_fc_convert_ptr->GetAxisPos(servo_data_fc_inner.pos_feedback, m_fc_status_inner.jnt_pos_measure);
    // 2.赋值拖动类型
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
    // 1.1当前非力矩模式不允许调用本接口(双重保护,避免drag_config后又置为位置模式)
    if (std::any_of(servo_mode_from_servo.cbegin(), servo_mode_from_servo.cend(),
                    [](int8_t servo_type) { return servo_type != 10; })) {
        return SERVO_MODE_ERROR;
    }
    // 2.读取伺服数据并转换为Fc内部变量
    m_servo_fc_convert_ptr->ServoData2FcInner(servo_mode_from_servo, pdo_analog_ch1, pdo_analog_ch2, trq_encoder_from_servo,
                                              pos_encoder_from_servo, vel_encoder_from_servo, m_servo_data_fc_inner);

    // 3.更新指令和反馈
    SetFcCommand(m_servo_data_fc_inner);

    // 3.力控数据流计算
    m_fc_status_tracker_ptr->FcStatusUpdata();

    // 4.力控模块功能力计算
    m_force_planner_ptr->ForcePlannerUpdata();

    // 5.根据负载参数更新下发给伺服的增益
    ResetKpByLoad(m_load);
    ResetFricByLoad(m_load);

    // 6.将Fc内部数据转换为下发给伺服数据
    m_servo_fc_convert_ptr->FcData2ServoData(m_fc_status_inner, m_fc_params_inner_ptr, m_fc_inner_servo_data);
    std::copy(m_fc_inner_servo_data.trq_cmd.cbegin(), m_fc_inner_servo_data.trq_cmd.cend(), trq_cmd_to_servo.begin());
    std::copy(m_fc_inner_servo_data.trq_feedforward.cbegin(), m_fc_inner_servo_data.trq_feedforward.cend(),
              fc_trq_feedforward_to_servo.begin());
    std::copy(m_fc_inner_servo_data.k_p.cbegin(), m_fc_inner_servo_data.k_p.cend(), fc_kp_to_servo.begin());
    std::copy(m_fc_inner_servo_data.k_d.cbegin(), m_fc_inner_servo_data.k_d.cend(), fc_kd_to_servo.begin());
    std::copy(m_fc_inner_servo_data.edb_cof.cbegin(), m_fc_inner_servo_data.edb_cof.cend(), fc_edb_cof_to_servo.begin());
    std::copy(m_fc_inner_servo_data.edb_o.cbegin(), m_fc_inner_servo_data.edb_o.cend(), fc_edb_o_to_servo.begin());
    std::copy(m_fc_inner_servo_data.fric_cof.cbegin(), m_fc_inner_servo_data.fric_cof.cend(), fc_fric_cof_to_servo.begin());
    std::copy(m_fc_inner_servo_data.jnt_inertia.cbegin(), m_fc_inner_servo_data.jnt_inertia.cend(),
              fc_jnt_inertia_to_servo.begin());
}

int ForceControl::DragConfig(const std::vector<int32_t>& pos_encoder_from_servo, const std::vector<int8_t>& servo_mode_from_servo,
                             const std::vector<int16_t>& analog_ch1, const std::vector<int16_t>& analog_ch2, const DragType& drag_type) {
    //0.初始化标志位
    int res = SOLVE_NOERROR;
    m_enable_drag = false;
    m_is_first_drag = true;
    // 1.判断是否允许进行Drag设置
    // 1.1 处于位置模式下
    if (std::any_of(servo_mode_from_servo.cbegin(), servo_mode_from_servo.cend(),
                    [](int8_t servo_type) { return servo_type != 8; })) {
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
    m_drag_type = drag_type;

    // 3.设置数据流计算负载参数(同一放到外部接口设置)
    // m_fc_status_tracker_ptr->SetLoad(m_load);

    // 4.设置拖动相关增益(不在每次开启拖动都设置了，通过接口设置即可)

    // 5.设置阻抗相关增益(该接口暂不开放，内部参数固定设置)
    SetImpedenceGain(m_drag_type);

    // 4.更新标志位
    m_enable_drag = true;

    return SOLVE_NOERROR;
}

int ForceControl::SetFcLoad(const RokaeLoad& load) {
    //判断负载参数是否合理
    if (load.m_rokae_load_inertia.mass >
        m_fc_params_inner_ptr->m_protect_params.m_params["max_load_mass"].at(0)) {  //工具质量限制，小于最大负载
        return LOAD_PARAMS_ERROR;
    } else if (load.m_rokae_load_inertia.GetCOG().Norm() > 0.3) {  //工具TCP长度限制
        return LOAD_PARAMS_ERROR;
    }

    KDL::Vector rpy_rad = load.m_rokae_load_pose.eulerangles / 180.0 * PI;

    // forcecontrol内部负载设置
    m_load.SetRokaeLoadInertia(load.m_rokae_load_inertia);
    m_load.m_rokae_load_pose.SetSpatiaPos(load.m_rokae_load_pose.spatiapos);
    m_load.m_rokae_load_pose.SetRotAngle(rpy_rad);

    // fc_status_tracker数据流计算参数设置
    m_fc_status_tracker_ptr->SetLoad(m_load);
}

int ForceControl::SetSensorLinearity(const std::vector<double> analog2trq_low) {
    if (m_jnt_num != analog2trq_low.size()) {
        return SIZE_ERROR;
    }
    m_fc_params_inner_ptr->m_hardware_params.SetParam("analog2trq_low", m_init_robot_ptr->GetMechanicalParams().analog2trq_low);
    m_servo_fc_convert_ptr->SetSensorLinearity(analog2trq_low);

    return SOLVE_NOERROR;
}

int ForceControl::SetSensorBias(const std::vector<double> analog_bias) {
    if (m_jnt_num != analog_bias.size()) {
        return SIZE_ERROR;
    }
    m_fc_params_inner_ptr->m_hardware_params.SetParam("analog_bias", m_init_robot_ptr->GetMechanicalParams().analog_bias);
    m_servo_fc_convert_ptr->SetSensorBias(analog_bias);
    return SOLVE_NOERROR;
}

int ForceControl::SetEncoderOffset(const std::vector<int32_t> encoder_offset) {
    if (m_jnt_num != encoder_offset.size()) {
        return SIZE_ERROR;
    }
    std::vector<double> encoder_offset_temp(encoder_offset.begin(), encoder_offset.end());
    m_fc_params_inner_ptr->m_hardware_params.SetParam("encoder_offset", (encoder_offset_temp));
    m_servo_fc_convert_ptr->SetEncoderBias(encoder_offset);
    return SOLVE_NOERROR;
}

int ForceControl::SetSoftLimit(const std::vector<double> joint_range_min, const std::vector<double> joint_range_max) {
    int res = SOLVE_NOERROR;

    //长度检查
    if (joint_range_min.size() != m_jnt_num) {
        return SIZE_ERROR;
    }
    if (joint_range_max.size() != m_jnt_num) {
        return SIZE_ERROR;
    }

    //数据有效性检查
    for (unsigned int i = 0; i < m_jnt_num; i++) {
        if ((joint_range_min[i] < m_init_robot_ptr->GetModelParams().joint_range_min_new[i]) or
            (joint_range_max[i] > m_init_robot_ptr->GetModelParams().joint_range_max_new[i])) {
            return SOFT_LIMIT_PARAMS_ERROR;
        }
    }

    //更新软限位成员变量
    std::vector<double> m_joint_range_max_temp(m_jnt_num);
    std::vector<double> m_joint_range_min_temp(m_jnt_num);

    for (unsigned int j = 0; j < m_jnt_num; j++) {
        m_joint_range_min_temp[j] = joint_range_min[j] / 180 * PI;
        m_joint_range_max_temp[j] = joint_range_max[j] / 180 * PI;
    }
    m_fc_params_inner_ptr->m_hardware_params.SetParam("joint_angle_limit_min", m_joint_range_min_temp);
    m_fc_params_inner_ptr->m_hardware_params.SetParam("joint_angle_limit_max", m_joint_range_max_temp);
    //设置软限位
    m_force_planner_ptr->SetSoftLimit(joint_range_min, joint_range_max);

    return SOLVE_NOERROR;
}
int ForceControl::SetMaxTrqErrorThreshold(const std::vector<double> m_max_trq_error_threshold) {
    if (m_jnt_num != m_max_trq_error_threshold.size()) {
        return SIZE_ERROR;
    }
    m_fc_params_inner_ptr->m_protect_params.SetParam("max_mode_switch_trq", m_max_trq_error_threshold);
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

int ForceControl::SetZetaGain(const std::vector<double> zeta_set) {
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
int ForceControl::ResetKpByLoad(const RokaeLoad& load) {
    double load_scale = load.m_rokae_load_inertia.GetCOG().Norm() * load.m_rokae_load_inertia.mass /
                        (m_load_tcp_length_limit[0] * m_load_mass_limit[0]);
    for (unsigned int i = 0; i < m_jnt_num; i++) {
        m_kp_set_by_load[i] = m_init_robot_ptr->GetControlParams().m_gain_params.joint_gain_kp[i] * m_kp_gain_set[i];
        m_kp_set_by_load[i] = m_kp_set_by_load[i] * (1 - load_scale);
    }
    //更新到FcParams中
    m_fc_params_inner_ptr->m_function_params.SetParam("joint_servo_kp", m_kp_set_by_load);
    return SOLVE_NOERROR;
}

int ForceControl::ResetFricByLoad(const RokaeLoad& load) {
    //调节范围为0~1
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
//************************************************************************************************

//*******************************内部控制参数计算、配置及状态切换接口部分*****************************

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