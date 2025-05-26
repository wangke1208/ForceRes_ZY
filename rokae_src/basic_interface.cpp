/**
 * Copyright(C) 2024 Rokae Technology Co., Ltd.
 * All Rights Reserved.
 *
 * Information in this file is the intellectual property of Rokae Technology Co., Ltd,
 * And may contains trade secrets that must be stored and viewed confidentially.
 *
 * @file: basic_interface.cpp
 * @author: wangke
 * @date: 2025/4/7
 * @brief: 对外接口
 */

#include "rokae_header/basic_interface.hpp"
using namespace RokaeApi::Control;
namespace RokaeApi {

namespace BasicInterface {
//定义
std::shared_ptr<Control::ForceControl> forcecontrol_ptr;
std::shared_ptr<InitRobot> initrobot_ptr;
std::shared_ptr<Axis_Convert> axisconvert_ptr;
std::shared_ptr<DynamicSolver> dynamicsolver_ptr;
std::shared_ptr<inverse_kinematics_solver> inverse_kinematics_solver_ptr;

unsigned int jnt_num;  // 关节数
bool is_initialized = false;  // 初始化标志

KDL::JntArray jnt_pos_kdl;
KDL::JntArray jnt_ext_trq;
KDL::Wrench tcp_wrench;
KDL::Vector gravity_vector;  //重力矢量
KDL::Frame frame_base_in_world;  // 基坐标系在世界坐标系下的位置

//计算运动学&动力学的临时变量
KDL::JntArray q_inverse_out_temp;
KDL::JntArray q_inverse_in_temp;
KDL::JntArray q_temp;
KDL::JntArray qd_temp;
KDL::JntArray qdd_temp;
KDL::JntArray trq_total_temp;
KDL::JntArray trq_gravity_temp;
KDL::JntArray trq_coriolis_temp;
KDL::JntArray trq_inertia_temp;
KDL::JntArray trq_ext_temp;
KDL::Frame tcp_frame_temp;
KDL::Jacobian jacobian_temp;
KDL::JntSpaceInertiaMatrix inertia_matrix_temp;

int InitInterface(const Model::MechUnitType& robot_type) {
    if (is_initialized) return ERROR_ALREADY_INIT;

    // 0.初始化日志模块
    LogConfig conf_log;
    conf_log.level = "trace";
    conf_log.path = "../log/Rokae_LOG.log";
    conf_log.size = 10 * 1024 * 1024;  // 10MB
    conf_log.count = 10;               // 最多保留100个文件
    INITLOG(conf_log);
    // 1.初始化参数模块
    try {
        initrobot_ptr = std::make_shared<InitRobot>(robot_type);
        auto res_initialize = initrobot_ptr->CreateModels();
        if (res_initialize != SOLVE_NOERROR) {
            return res_initialize;
        }
    } catch (const std::exception& e) {
        std::cerr << "Failed to initialize InitRobot: " << e.what() << std::endl;
        return ERROR_ROBOTTYPE;
    }
    // 2.初始化力控模块
    forcecontrol_ptr = std::make_shared<Control::ForceControl>(initrobot_ptr.get());
    auto res_forcecontrol = forcecontrol_ptr->Fcinit();
    if (res_forcecontrol != SOLVE_NOERROR) {
        return res_forcecontrol;
    }

    // 3.初始化其他模块(用来计算的)
    axisconvert_ptr = std::make_shared<Axis_Convert>(initrobot_ptr->GetJntNum(), initrobot_ptr.get()->GetMechanicalParams());
    dynamicsolver_ptr = std::make_shared<DynamicSolver>(initrobot_ptr->GetChain(), initrobot_ptr->GetGravity());
    inverse_kinematics_solver_ptr =
        std::make_shared<inverse_kinematics_solver>(initrobot_ptr->GetChain(), initrobot_ptr->GetModelParams());
    // 4.初始化参数
    jnt_num = initrobot_ptr->GetJntNum();
    jnt_pos_kdl.resize(jnt_num);
    jnt_ext_trq.resize(jnt_num);
    tcp_wrench.Zero();
    frame_base_in_world.Identity();
    gravity_vector = initrobot_ptr->GetGravity();
    q_inverse_out_temp.resize(jnt_num);
    q_inverse_in_temp.resize(jnt_num);
    q_temp.resize(jnt_num);
    qd_temp.resize(jnt_num);
    qdd_temp.resize(jnt_num);
    trq_total_temp.resize(jnt_num);
    trq_gravity_temp.resize(jnt_num);
    trq_coriolis_temp.resize(jnt_num);
    trq_inertia_temp.resize(jnt_num);

    tcp_frame_temp.Identity();
    jacobian_temp.resize(jnt_num);
    inertia_matrix_temp.resize(jnt_num);

    is_initialized = true;
    return SOLVE_NOERROR;
}

//********************************力控算法接口******************************************
int DragConfig(const std::vector<int32_t>& pos_encoder_from_servo, const std::vector<int8_t>& servo_mode_from_servo,
               const std::vector<int16_t>& analog_ch1, const std::vector<int16_t>& analog_ch2, const DragType& drag_type,
               const bool& is_command_by_user) {
    auto res = forcecontrol_ptr->DragConfig(pos_encoder_from_servo, servo_mode_from_servo, analog_ch1, analog_ch2, drag_type,
                                            is_command_by_user);
    if (res != SOLVE_NOERROR) {
        forcecontrol_ptr->FcStatusRefresh();
        return res;
    }
    return res;
}

int FcUpdate(const std::vector<int8_t>& servo_mode_from_servo, const std::vector<int16_t>& pdo_analog_ch1,
             const std::vector<int16_t>& pdo_analog_ch2, const std::vector<int16_t>& trq_encoder_from_servo,
             const std::vector<int>& pos_encoder_from_servo, const std::vector<int>& vel_encoder_from_servo,
             const std::vector<double>& jnt_pos_cmd_from_user, const std::array<double, 6>& cart_pos_cmd_from_user,
             const std::vector<double>& jnt_trq_cmd_from_user, std::vector<int16_t>& fc_trq_cmd_to_servo,
             std::vector<int16_t>& fc_trq_feedforward_to_servo, std::vector<int16_t>& fc_kp_to_servo,
             std::vector<int16_t>& fc_kd_to_servo, std::vector<int16_t>& fc_edb_cof_to_servo,
             std::vector<int16_t>& fc_edb_o_to_servo, std::vector<int16_t>& fc_fric_cof_to_servo,
             std::vector<int16_t>& fc_jnt_inertia_to_servo) {
    auto res = forcecontrol_ptr->FcUpdate(servo_mode_from_servo, pdo_analog_ch1, pdo_analog_ch2, trq_encoder_from_servo,
                                          pos_encoder_from_servo, vel_encoder_from_servo, jnt_pos_cmd_from_user,
                                          cart_pos_cmd_from_user, jnt_trq_cmd_from_user, fc_trq_cmd_to_servo,
                                          fc_trq_feedforward_to_servo, fc_kp_to_servo, fc_kd_to_servo, fc_edb_cof_to_servo,
                                          fc_edb_o_to_servo, fc_fric_cof_to_servo, fc_jnt_inertia_to_servo);
    if (res != SOLVE_NOERROR) {
        forcecontrol_ptr->FcStatusRefresh();
        return res;
    }
    return SOLVE_NOERROR;
}

int FcStop(const std::vector<int8_t>& servo_mode) {
    for (unsigned int i = 0; i < jnt_num; i++) {
        if (servo_mode[i] != POSITION_MODE) {
            return ERROR_SERVO_MODE;
        }
    }
    forcecontrol_ptr->FcStatusRefresh();
    return SOLVE_NOERROR;
}

//********************************力控参数设置接口******************************************
int SetSensorLinearity(const std::vector<int8_t>& servo_mode, const std::vector<double>& analog2trq_low) {
    //判断伺服模式是否处于位置模式
    if (IsInPositionMode(servo_mode) != true) {
        return ERROR_SERVO_MODE;
    }
    //判断是否进行了DragConfig
    if (forcecontrol_ptr->GetDragStatus() == true) {
        return ERROR_DRAG_STATUS;
    }
    //设置线性度
    return (forcecontrol_ptr->SetSensorLinearity(analog2trq_low) && axisconvert_ptr->SetSensorLinearity(analog2trq_low));
}

int SetSensorBias(const std::vector<int8_t>& servo_mode, const std::vector<double>& analog_bias) {
    //判断伺服模式是否处于位置模式
    if (IsInPositionMode(servo_mode) != true) {
        return ERROR_SERVO_MODE;
    }
    //判断是否进行了DragConfig
    if (forcecontrol_ptr->GetDragStatus() == true) {
        return ERROR_DRAG_STATUS;
    }
    //设置传感器零点
    auto res1 = forcecontrol_ptr->SetSensorBias(analog_bias);
    auto res2 = axisconvert_ptr->SetSensorBias(analog_bias);
    if (res1 != SOLVE_NOERROR || res2 != SOLVE_NOERROR) {
        return res1;
    }
    return SOLVE_NOERROR;
}

int SetEncoderOffset(const std::vector<int8_t>& servo_mode, const std::vector<int32_t>& encoder_offset) {
    //判断伺服模式是否处于位置模式
    if (IsInPositionMode(servo_mode) != true) {
        return ERROR_SERVO_MODE;
    }
    //判断是否进行了DragConfig
    if (forcecontrol_ptr->GetDragStatus() == true) {
        return ERROR_DRAG_STATUS;
    }
    //设置编码器零点
    auto res1 = forcecontrol_ptr->SetEncoderOffset(encoder_offset);
    auto res2 = axisconvert_ptr->SetEncoderBias(encoder_offset);
    if (res1 != SOLVE_NOERROR || res2 != SOLVE_NOERROR) {
        return res1;
    }
    return SOLVE_NOERROR;
}

int SetSoftLimit(const std::vector<int8_t>& servo_mode, const std::vector<double>& joint_range_min,
                 const std::vector<double>& joint_range_max) {
    //判断伺服模式是否处于位置模式
    if (IsInPositionMode(servo_mode) != true) {
        return ERROR_SERVO_MODE;
    }
    //判断是否进行了DragConfig
    if (forcecontrol_ptr->GetDragStatus() == true) {
        return ERROR_DRAG_STATUS;
    }
    //设置软限位
    return forcecontrol_ptr->SetSoftLimit(joint_range_min, joint_range_max);
}

int SetMaxTrqErrorThreshold(const std::vector<int8_t>& servo_mode, const std::vector<double>& m_max_trq_error_threshold) {
    //判断伺服模式是否处于位置模式
    if (IsInPositionMode(servo_mode) != true) {
        return ERROR_SERVO_MODE;
    }
    //判断是否进行了DragConfig
    if (forcecontrol_ptr->GetDragStatus() == true) {
        return ERROR_DRAG_STATUS;
    }
    //设置最大扭矩误差阈值
    return forcecontrol_ptr->SetMaxTrqErrorThreshold(m_max_trq_error_threshold);
}

int SetLoadLimit(const std::vector<int8_t>& servo_mode, const double& max_load_mass, const double& max_load_tcp_length) {
    //判断伺服模式是否处于位置模式
    if (IsInPositionMode(servo_mode) != true) {
        return ERROR_SERVO_MODE;
    }
    //判断是否进行了DragConfig
    if (forcecontrol_ptr->GetDragStatus() == true) {
        return ERROR_DRAG_STATUS;
    }
    //设置负载限制
    return forcecontrol_ptr->SetLoadLimit(max_load_mass, max_load_tcp_length);
}

int SetFcLoad(const std::vector<int8_t>& servo_mode, const RokaeLoad& load) {
    //判断伺服模式是否处于位置模式
    if (IsInPositionMode(servo_mode) != true) {
        return ERROR_SERVO_MODE;
    }
    //判断是否进行了DragConfig
    if (forcecontrol_ptr->GetDragStatus() == true) {
        return ERROR_DRAG_STATUS;
    }
    //设置力控负载
    return forcecontrol_ptr->SetFcLoad(load);
}

int SetKpGain(const std::vector<int8_t>& servo_mode, const std::vector<double>& kp_gain_set) {
    //判断伺服模式是否处于位置模式
    if (IsInPositionMode(servo_mode) != true) {
        return ERROR_SERVO_MODE;
    }
    //判断是否进行了DragConfig
    if (forcecontrol_ptr->GetDragStatus() == true) {
        return ERROR_DRAG_STATUS;
    }
    //设置kp滑条系数
    return forcecontrol_ptr->SetKpGain(kp_gain_set);
}

int SetFricGain(const std::vector<int8_t>& servo_mode, const std::vector<double>& fric_gain_set) {
    //判断伺服模式是否处于位置模式
    if (IsInPositionMode(servo_mode) != true) {
        return ERROR_SERVO_MODE;
    }
    //判断是否进行了DragConfig
    if (forcecontrol_ptr->GetDragStatus() == true) {
        return ERROR_DRAG_STATUS;
    }
    //设置摩擦滑条系数
    return forcecontrol_ptr->SetFricGain(fric_gain_set);
}

int SetJointImpedance(const std::vector<int8_t>& servo_mode, const std::vector<double>& joint_stiffness) {
    //判断伺服模式是否处于位置模式
    if (IsInPositionMode(servo_mode) != true) {
        return ERROR_SERVO_MODE;
    }
    //判断是否进行了DragConfig
    if (forcecontrol_ptr->GetDragStatus() == true) {
        return ERROR_DRAG_STATUS;
    }
    //设置摩擦滑条系数
    return forcecontrol_ptr->SetJointImpedance(joint_stiffness);
}

int SetCartImpedance(const std::vector<int8_t>& servo_mode, const std::array<double, 6>& cart_stiffness) {
    //判断伺服模式是否处于位置模式
    if (IsInPositionMode(servo_mode) != true) {
        return ERROR_SERVO_MODE;
    }
    //判断是否进行了DragConfig
    if (forcecontrol_ptr->GetDragStatus() == true) {
        return ERROR_DRAG_STATUS;
    }
    //设置笛卡尔刚度
    return forcecontrol_ptr->SetCartImpedance(cart_stiffness);
}

int SetBaseFrameAndGravity(const std::array<double, 6>& base_poseture) {
    if (ArrayToKdlFrame(base_poseture, frame_base_in_world) != SOLVE_NOERROR) {
        return ERROR_EULER_PARAMS;
    }
    //根据基坐标系方向确定重力矢量方向
    gravity_vector = frame_base_in_world.M.Inverse() * KDL::Vector(0, 0, -9.81);
    //更新初始化接口(没有实际用处，但是保持一致)
    initrobot_ptr->SetGravity(gravity_vector);
    //更新非实时接口
    dynamicsolver_ptr->SetGravity(gravity_vector);
    //更新实时接口
    forcecontrol_ptr->SetBaseFrameAndGravity(frame_base_in_world, gravity_vector);
    return SOLVE_NOERROR;
}

//*******************************功能接口*********************************/
int CalibrateTrqSensor(const std::vector<int8_t>& servo_mode, const std::vector<int32_t>& pos_encoder_feedback,
                       const RokaeLoad& load_input, const std::vector<std::array<int16_t, 200>>& analog_array_ch1,
                       const std::vector<std::array<int16_t, 200>>& analog_array_ch2, std::vector<double>& sensor_bias) {
    //判断伺服模式是否处于位置模式
    if (IsInPositionMode(servo_mode) != true) {
        return ERROR_SERVO_MODE;
    }
    //判断是否进行了DragConfig
    if (forcecontrol_ptr->GetDragStatus() == true) {
        return ERROR_DRAG_STATUS;
    }
    //进行校准
    return forcecontrol_ptr->CalibrateTrqSensor(pos_encoder_feedback, load_input, analog_array_ch1, analog_array_ch2,
                                                sensor_bias);
}

//*******************************关节状态*********************************/
int GetAxisPos(const std::vector<int>& encoder_value, std::vector<double>& jnt_pos_rad) {
    if (encoder_value.size() != jnt_num || jnt_pos_rad.size() != jnt_num) {
        return ERROR_SIZE_WRONG;
    }
    return axisconvert_ptr->GetAxisPos(encoder_value, jnt_pos_rad);
}

int GetAxisVel(const std::vector<int>& encoder_vel_value, std::vector<double>& jnt_vel_rad) {
    if (jnt_vel_rad.size() != jnt_num || encoder_vel_value.size() != jnt_num) {
        return ERROR_SIZE_WRONG;
    }
    return axisconvert_ptr->GetAxisVel(encoder_vel_value, jnt_vel_rad);
}

int GetCobotTrq(const std::vector<int16_t>& analog_ch1, const std::vector<int16_t>& analog_ch2,
                std::vector<double>& jnt_trq_feedback) {
    if (jnt_trq_feedback.size() != jnt_num || analog_ch1.size() != jnt_num || analog_ch2.size() != jnt_num) {
        return ERROR_SIZE_WRONG;
    }
    return axisconvert_ptr->GetCobotTrq(analog_ch1, analog_ch2, jnt_trq_feedback);
}

void GetTcpWrench(const RokaeLoad& load, const std::vector<double>& jnt_pos, const std::vector<double>& jnt_trq_feedback,
                  const std::vector<double>& jnt_gra_trq, std::array<double, 6>& ext_force) {
    for (unsigned int i = 0; i < jnt_num; i++) {
        jnt_pos_kdl(i) = jnt_pos[i];
        jnt_ext_trq(i) = jnt_gra_trq[i] - jnt_trq_feedback[i];
    }
    tcp_wrench.Zero();
    dynamicsolver_ptr->GetWrench(load.m_rokae_load_pose, jnt_pos_kdl, jnt_ext_trq, tcp_wrench);
    for (unsigned int i = 0; i < 3; i++) {
        ext_force[i] = tcp_wrench.force(i);
        ext_force[i + 3] = tcp_wrench.torque(i);
    }
}

//*******************************模型接口*********************************/
int GetGraTorque(const RokaeLoad& load_params, const std::vector<double>& q, std::vector<double>& trq_gravity) {
    if (q.size() != jnt_num || trq_gravity.size() != jnt_num) {
        return ERROR_SIZE_WRONG;
    }
    VectorToJntArray(q, q_temp);
    trq_gravity_temp = dynamicsolver_ptr->GetGraTorque(load_params.m_rokae_load_inertia, q_temp);
    JntArrayToVector(trq_gravity_temp, trq_gravity);
    return SOLVE_NOERROR;
}

int GetInertTorque(const RokaeLoad& load_params, const std::vector<double>& q, const std::vector<double>& ddq,
                   std::vector<double>& trq_inertia) {
    if (q.size() != jnt_num || ddq.size() != jnt_num || trq_inertia.size() != jnt_num) {
        return ERROR_SIZE_WRONG;
    }
    VectorToJntArray(q, q_temp);
    VectorToJntArray(ddq, qd_temp);
    trq_inertia_temp = dynamicsolver_ptr->GetInertTorque(load_params.m_rokae_load_inertia, q_temp, qd_temp);
    JntArrayToVector(trq_inertia_temp, trq_inertia);
    return SOLVE_NOERROR;
}

int GetCoriolisTorque(const RokaeLoad& load_params, const std::vector<double>& q, const std::vector<double>& dq,
                      std::vector<double>& trq_coriolis) {
    if (q.size() != jnt_num || dq.size() != jnt_num || trq_coriolis.size() != jnt_num) {
        return ERROR_SIZE_WRONG;
    }
    VectorToJntArray(q, q_temp);
    VectorToJntArray(dq, qd_temp);
    trq_coriolis_temp = dynamicsolver_ptr->GetColioTorque(load_params.m_rokae_load_inertia, q_temp, qd_temp);
    JntArrayToVector(trq_coriolis_temp, trq_coriolis);
    return SOLVE_NOERROR;
}

int GetTotalTorque(const RokaeLoad& load_params, const std::vector<double>& q, const std::vector<double>& dq,
                   const std::vector<double>& ddq, std::vector<double>& trq_total) {
    if (q.size() != jnt_num || dq.size() != jnt_num || ddq.size() != jnt_num || trq_total.size() != jnt_num) {
        return ERROR_SIZE_WRONG;
    }
    VectorToJntArray(q, q_temp);
    VectorToJntArray(dq, qd_temp);
    VectorToJntArray(ddq, qdd_temp);
    trq_total_temp = dynamicsolver_ptr->GetTotalTorque(load_params.m_rokae_load_inertia, q_temp, qd_temp, qdd_temp);
    JntArrayToVector(trq_total_temp, trq_total);
    return SOLVE_NOERROR;
}

int GetTcpPos(const RokaeLoad& load, const std::vector<double>& jnt_pos, std::array<double, 6>& tcp_pos) {
    if (jnt_pos.size() != jnt_num) {
        return ERROR_SIZE_WRONG;
    }
    VectorToJntArray(jnt_pos, q_temp);
    dynamicsolver_ptr->GetTcpPos(load, q_temp, tcp_frame_temp);
    std::copy(tcp_frame_temp.p.data, tcp_frame_temp.p.data + 3, tcp_pos.begin());
    tcp_frame_temp.M.GetRPY(tcp_pos[3], tcp_pos[4], tcp_pos[5]);
    //弧度转角度
    for (unsigned int i = 0; i < 3; i++) {
        tcp_pos[i + 3] = tcp_pos[i + 3] * RAD_TO_DEG;
    }
    return SOLVE_NOERROR;
}

int GetJointPos(const std::vector<double>& curJnt_origin, const GeneralizedFrame& target_Flan,
                std::vector<double>& OutJointPose) {
    if (curJnt_origin.size() != jnt_num || OutJointPose.size() != jnt_num) {
        return ERROR_SIZE_WRONG;
    }
    VectorToJntArray(curJnt_origin, q_inverse_in_temp);
    int res = inverse_kinematics_solver_ptr->CartToJnt(q_inverse_in_temp, target_Flan, q_inverse_out_temp);
    if (res != SOLVE_NOERROR) {
        return res;
    }
    JntArrayToVector(q_inverse_out_temp, OutJointPose);
    return SOLVE_NOERROR;
}

int GetMassMatrix(const RokaeLoad& load_params, const std::vector<double>& jnt_pos, Eigen::MatrixXd& mass_matrix) {
    if (jnt_pos.size() != jnt_num) {
        return ERROR_SIZE_WRONG;
    }
    VectorToJntArray(jnt_pos, q_temp);
    dynamicsolver_ptr->JntToMass(load_params.m_rokae_load_inertia, q_temp, inertia_matrix_temp);
    mass_matrix = inertia_matrix_temp.data;
    return SOLVE_NOERROR;
}

int GetTcpJacobian(const RokaeLoad& load, const std::vector<double>& jnt_pos,
                   Eigen::Matrix<double, 6, Eigen::Dynamic>& jacobian) {
    if (jnt_pos.size() != jnt_num) {
        return ERROR_SIZE_WRONG;
    }
    VectorToJntArray(jnt_pos, q_temp);
    dynamicsolver_ptr->GetTcpJacobian(load, q_temp, jacobian_temp);
    jacobian = jacobian_temp.data;
    return SOLVE_NOERROR;
}

//*******************************获取实时内部状态*********************************/
int GetAxisPosCurrent(std::vector<double>& jnt_pos_rad) {
    if (jnt_pos_rad.size() != jnt_num) {
        return ERROR_SIZE_WRONG;
    }

    std::copy_n(forcecontrol_ptr->GetFcStatusCopy().jnt_pos_measure.data.data(),
                forcecontrol_ptr->GetFcStatusCopy().jnt_pos_measure.rows(), jnt_pos_rad.begin());

    //弧度转角度
    std::transform(jnt_pos_rad.begin(), jnt_pos_rad.end(), jnt_pos_rad.begin(), [](double rad) { return rad * RAD_TO_DEG; });

    return SOLVE_NOERROR;
}

int GetAxisVelCurrent(std::vector<double>& jnt_vel_rad) {
    if (jnt_vel_rad.size() != jnt_num) {
        return ERROR_SIZE_WRONG;
    }

    std::copy_n(forcecontrol_ptr->GetFcStatusCopy().jnt_vel_measure.data.data(),
                forcecontrol_ptr->GetFcStatusCopy().jnt_vel_measure.rows(), jnt_vel_rad.begin());
    //弧度转角度
    std::transform(jnt_vel_rad.begin(), jnt_vel_rad.end(), jnt_vel_rad.begin(), [](double rad) { return rad * RAD_TO_DEG; });

    return SOLVE_NOERROR;
}

int GetCobotTrqCurrent(std::vector<double>& jnt_trq_feedback) {
    if (jnt_trq_feedback.size() != jnt_num) {
        return ERROR_SIZE_WRONG;
    }

    std::copy_n(forcecontrol_ptr->GetFcStatusCopy().jnt_trq_sensor_measure.data.data(),
                forcecontrol_ptr->GetFcStatusCopy().jnt_trq_sensor_measure.rows(), jnt_trq_feedback.begin());
    return SOLVE_NOERROR;
}

int GetTcpWrenchCurrent(std::array<double, 6>& ext_force) {
    for (unsigned int i = 0; i < 3; i++) {
        ext_force[i] = forcecontrol_ptr->GetFcStatusCopy().tcp_wrench.force[i];
        ext_force[i + 3] = forcecontrol_ptr->GetFcStatusCopy().tcp_wrench.torque[i];
    }
    return SOLVE_NOERROR;
}

int GetTcpPosCurrent(std::array<double, 6>& tcp_pos) {
    std::copy_n(forcecontrol_ptr->GetFcStatusCopy().cart_pos_measure_tcp_in_base.p.data, 3, tcp_pos.begin());
    forcecontrol_ptr->GetFcStatusCopy().cart_pos_measure_tcp_in_base.M.GetRPY(tcp_pos[3], tcp_pos[4], tcp_pos[5]);
    //弧度转角度
    for (unsigned int i = 0; i < 3; i++) {
        tcp_pos[i + 3] = tcp_pos[i + 3] * RAD_TO_DEG;
    }
    return SOLVE_NOERROR;
}

int GetDynamicTorqueCurrent(std::vector<double>& trq_gravity, std::vector<double>& trq_coriolis, Eigen::MatrixXd& mass_matrix) {
    if (mass_matrix.rows() != jnt_num || mass_matrix.cols() != jnt_num || trq_gravity.size() != jnt_num ||
        trq_coriolis.size() != jnt_num) {
        return ERROR_SIZE_WRONG;
    }
    std::copy_n(forcecontrol_ptr->GetFcStatusCopy().jnt_gravity_trq_measure.data.data(),
                forcecontrol_ptr->GetFcStatusCopy().jnt_gravity_trq_measure.rows(), trq_gravity.begin());
    std::copy_n(forcecontrol_ptr->GetFcStatusCopy().jnt_corlios_trq_measure.data.data(),
                forcecontrol_ptr->GetFcStatusCopy().jnt_corlios_trq_measure.rows(), trq_coriolis.begin());
    mass_matrix = forcecontrol_ptr->GetFcStatusCopy().jnt_inertia_matrix_measure.data;
    return SOLVE_NOERROR;
}

int GetJacobianCurrent(Eigen::Matrix<double, 6, Eigen::Dynamic>& jacobian) {
    if (jacobian.rows() != 6 || jacobian.cols() != jnt_num) {
        return ERROR_SIZE_WRONG;
    }
    jacobian = forcecontrol_ptr->GetFcStatusCopy().jac_measure_tcp_in_base.data;
    return SOLVE_NOERROR;
}

//*******************************其他接口*********************************/
bool IsInPositionMode(const std::vector<int8_t>& servo_mode) {
    if (servo_mode.size() != jnt_num) {
        return false;
    }
    for (unsigned int i = 0; i < jnt_num; i++) {
        if (servo_mode[i] != POSITION_MODE) {
            return false;
        }
    }
    return true;
}

const char* GetVersion() { return VERSION; }

//*****************************Deinit接口*******************************/
void DeinitInterface() {
    // 清空智能指针，释放资源
    forcecontrol_ptr.reset();
    initrobot_ptr.reset();
    axisconvert_ptr.reset();
    dynamicsolver_ptr.reset();

    // 重置变量
    jnt_num = DEFAULT_AXIS;
    is_initialized = false;

    tcp_wrench = KDL::Wrench::Zero();
    gravity_vector = KDL::Vector::Zero();
    frame_base_in_world = KDL::Frame::Identity();
    tcp_frame_temp = KDL::Frame::Identity();

    jnt_pos_kdl = KDL::JntArray(DEFAULT_AXIS);
    jnt_ext_trq = KDL::JntArray(DEFAULT_AXIS);
    q_inverse_temp = KDL::JntArray(DEFAULT_AXIS);
    q_temp = KDL::JntArray(DEFAULT_AXIS);
    qd_temp = KDL::JntArray(DEFAULT_AXIS);
    qdd_temp = KDL::JntArray(DEFAULT_AXIS);
    trq_total_temp = KDL::JntArray(DEFAULT_AXIS);
    trq_gravity_temp = KDL::JntArray(DEFAULT_AXIS);
    trq_coriolis_temp = KDL::JntArray(DEFAULT_AXIS);
    trq_inertia_temp = KDL::JntArray(DEFAULT_AXIS);
    trq_ext_temp = KDL::JntArray(DEFAULT_AXIS);

    jacobian_temp.resize(DEFAULT_AXIS);
    inertia_matrix_temp.resize(DEFAULT_AXIS);
    jacobian_temp.data.setZero();
    inertia_matrix_temp.data.setZero();
}

}  // namespace BasicInterface
}  // namespace RokaeApi
