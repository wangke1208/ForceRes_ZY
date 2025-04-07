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

unsigned int jnt_num;  // 关节数
KDL::JntArray jnt_pos_kdl;
KDL::JntArray jnt_ext_trq;
KDL::Wrench tcp_wrench;
RokaeLoad m_load;
int InitInterface(const Model::MechUnitType& robot_type) {
    // 1.初始化参数模块
    try {
        initrobot_ptr = std::make_shared<InitRobot>(robot_type);
        auto res_initialize = initrobot_ptr->CreateModels();
        if (res_initialize != SOLVE_NOERROR) {
            return res_initialize;
        }
    } catch (const std::exception& e) {
        std::cerr << "Failed to initialize InitRobot: " << e.what() << std::endl;
        return ROBOT_INITIALIZE_ERROR;
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

    // 4.初始化参数
    jnt_num = initrobot_ptr->GetJntNum();
    jnt_pos_kdl.resize(jnt_num);
    jnt_ext_trq.resize(jnt_num);
    tcp_wrench.Zero();
    m_load.SetZero();

    return SOLVE_NOERROR;
}

//********************************力控算法接口******************************************
int DragConfig(const std::vector<int32_t>& pos_encoder_from_servo, const std::vector<int8_t>& servo_mode_from_servo,
               const std::vector<int16_t>& analog_ch1, const std::vector<int16_t>& analog_ch2, const DragType& drag_type) {
    auto res = forcecontrol_ptr->DragConfig(pos_encoder_from_servo, servo_mode_from_servo, analog_ch1, analog_ch2, drag_type);
    if (res != SOLVE_NOERROR) {
        forcecontrol_ptr->FcStatusRefresh();
        return res;
    }
    return res;
}

int FcUpdate(const std::vector<int8_t>& servo_mode_from_servo, const std::vector<int16_t>& pdo_analog_ch1,
             const std::vector<int16_t>& pdo_analog_ch2, const std::vector<int16_t>& trq_encoder_from_servo,
             const std::vector<int>& pos_encoder_from_servo, const std::vector<int>& vel_encoder_from_servo,
             std::vector<int16_t>& fc_trq_cmd_to_servo, std::vector<int16_t>& fc_trq_feedforward_to_servo,
             std::vector<int16_t>& fc_kp_to_servo, std::vector<int16_t>& fc_kd_to_servo,
             std::vector<int16_t>& fc_edb_cof_to_servo, std::vector<int16_t>& fc_edb_o_to_servo,
             std::vector<int16_t>& fc_fric_cof_to_servo, std::vector<int16_t>& fc_jnt_inertia_to_servo) {
    auto res = forcecontrol_ptr->FcUpdate(servo_mode_from_servo, pdo_analog_ch1, pdo_analog_ch2, trq_encoder_from_servo,
                                          pos_encoder_from_servo, vel_encoder_from_servo, fc_trq_cmd_to_servo,
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
            return SERVO_MODE_ERROR;
        }
    }
    forcecontrol_ptr->FcStatusRefresh();
    return SOLVE_NOERROR;
}

//********************************力控参数设置接口******************************************
int SetSensorLinearity(const std::vector<int8_t>& servo_mode, const std::vector<double> analog2trq_low) {
    //判断伺服模式是否处于位置模式
    if (IsInPositionMode(servo_mode) != true) {
        return SERVO_MODE_ERROR;
    }
    //判断是否进行了DragConfig
    if (forcecontrol_ptr->GetDragStatus() != true) {
        return DRAG_STATUS_ERROR;
    }
    //设置线性度
    return (forcecontrol_ptr->SetSensorLinearity(analog2trq_low) && axisconvert_ptr->SetSensorLinearity(analog2trq_low));
}

int SetSensorBias(const std::vector<int8_t>& servo_mode, const std::vector<double> analog_bias) {
    //判断伺服模式是否处于位置模式
    if (IsInPositionMode(servo_mode) != true) {
        return SERVO_MODE_ERROR;
    }
    //判断是否进行了DragConfig
    if (forcecontrol_ptr->GetDragStatus() != true) {
        return DRAG_STATUS_ERROR;
    }
    //设置传感器零点
    return (forcecontrol_ptr->SetSensorBias(analog_bias) && axisconvert_ptr->SetSensorBias(analog_bias));
}

int SetEncoderOffset(const std::vector<int8_t>& servo_mode, const std::vector<int32_t>& encoder_offset) {
    //判断伺服模式是否处于位置模式
    if (IsInPositionMode(servo_mode) != true) {
        return SERVO_MODE_ERROR;
    }
    //判断是否进行了DragConfig
    if (forcecontrol_ptr->GetDragStatus() != true) {
        return DRAG_STATUS_ERROR;
    }
    //设置编码器零点
    return (forcecontrol_ptr->SetEncoderOffset(encoder_offset) && axisconvert_ptr->SetEncoderBias(encoder_offset));
}

int SetSoftLimit(const std::vector<int8_t>& servo_mode, const std::vector<double>& joint_range_min,
                 const std::vector<double>& joint_range_max) {
    //判断伺服模式是否处于位置模式
    if (IsInPositionMode(servo_mode) != true) {
        return SERVO_MODE_ERROR;
    }
    //判断是否进行了DragConfig
    if (forcecontrol_ptr->GetDragStatus() != true) {
        return DRAG_STATUS_ERROR;
    }
    //设置软限位
    return forcecontrol_ptr->SetSoftLimit(joint_range_min, joint_range_max);
}

int SetMaxTrqErrorThreshold(const std::vector<int8_t>& servo_mode, const std::vector<double>& m_max_trq_error_threshold) {
    //判断伺服模式是否处于位置模式
    if (IsInPositionMode(servo_mode) != true) {
        return SERVO_MODE_ERROR;
    }
    //判断是否进行了DragConfig
    if (forcecontrol_ptr->GetDragStatus() != true) {
        return DRAG_STATUS_ERROR;
    }
    //设置最大扭矩误差阈值
    return forcecontrol_ptr->SetMaxTrqErrorThreshold(m_max_trq_error_threshold);
}

int SetLoadLimit(const std::vector<int8_t>& servo_mode, const double& max_load_mass, const double& max_load_tcp_length) {
    //判断伺服模式是否处于位置模式
    if (IsInPositionMode(servo_mode) != true) {
        return SERVO_MODE_ERROR;
    }
    //判断是否进行了DragConfig
    if (forcecontrol_ptr->GetDragStatus() != true) {
        return DRAG_STATUS_ERROR;
    }
    //设置负载限制
    return forcecontrol_ptr->SetLoadLimit(max_load_mass, max_load_tcp_length);
}

int SetFcLoad(const std::vector<int8_t>& servo_mode, const RokaeLoad& load) {
    //判断伺服模式是否处于位置模式
    if (IsInPositionMode(servo_mode) != true) {
        return SERVO_MODE_ERROR;
    }
    //判断是否进行了DragConfig
    if (forcecontrol_ptr->GetDragStatus() != true) {
        return DRAG_STATUS_ERROR;
    }
    //设置力控负载
    return forcecontrol_ptr->SetFcLoad(load);
}

int SetKpGain(const std::vector<int8_t>& servo_mode, const std::vector<double>& kp_gain_set) {
    //判断伺服模式是否处于位置模式
    if (IsInPositionMode(servo_mode) != true) {
        return SERVO_MODE_ERROR;
    }
    //判断是否进行了DragConfig
    if (forcecontrol_ptr->GetDragStatus() != true) {
        return DRAG_STATUS_ERROR;
    }
    //设置kp滑条系数
    return forcecontrol_ptr->SetKpGain(kp_gain_set);
}

int SetFricGain(const std::vector<int8_t>& servo_mode, const std::vector<double>& fric_gain_set) {
    //判断伺服模式是否处于位置模式
    if (IsInPositionMode(servo_mode) != true) {
        return SERVO_MODE_ERROR;
    }
    //判断是否进行了DragConfig
    if (forcecontrol_ptr->GetDragStatus() != true) {
        return DRAG_STATUS_ERROR;
    }
    //设置摩擦滑条系数
    return forcecontrol_ptr->SetFricGain(fric_gain_set);
}

//*******************************功能接口*********************************/
int CalibrateTrqSensor(const std::vector<int8_t>& servo_mode, const std::vector<int32_t>& pos_encoder_feedback,
                       const RokaeLoad& load_input, const std::vector<std::array<int16_t, ANALOG_DATA_COUNT>>& analog_array_ch1,
                       const std::vector<std::array<int16_t, ANALOG_DATA_COUNT>>& analog_array_ch2,
                       std::vector<double>& sensor_bias) {
    //判断伺服模式是否处于位置模式
    if (IsInPositionMode(servo_mode) != true) {
        return SERVO_MODE_ERROR;
    }
    //判断是否进行了DragConfig
    if (forcecontrol_ptr->GetDragStatus() != true) {
        return DRAG_STATUS_ERROR;
    }
    //进行校准
    return forcecontrol_ptr->CalibrateTrqSensor(pos_encoder_feedback, load_input, analog_array_ch1, analog_array_ch2,
                                                sensor_bias);
}

//*******************************关节状态*********************************/
int GetAxisPos(const std::vector<int>& encoder_value, std::vector<double>& jnt_pos_rad) {
    if (encoder_value.size() != jnt_num || jnt_pos_rad.size() != jnt_num) {
        return SIZE_ERROR;
    }
    return axisconvert_ptr->GetAxisPos(encoder_value, jnt_pos_rad);
}

int GetAxisVel(const std::vector<int>& encoder_vel_value, std::vector<double>& jnt_vel_rad) {
    if (jnt_vel_rad.size() != jnt_num || encoder_vel_value.size() != jnt_num) {
        return SIZE_ERROR;
    }
    return axisconvert_ptr->GetAxisVel(encoder_vel_value, jnt_vel_rad);
}

int GetCobotTrq(const std::vector<int16_t>& analog_ch1, const std::vector<int16_t>& analog_ch2,
                std::vector<double>& jnt_trq_feedback) {
    if (jnt_trq_feedback.size() != jnt_num || analog_ch1.size() != jnt_num || analog_ch2.size() != jnt_num) {
        return SIZE_ERROR;
    }
    return axisconvert_ptr->GetCobotTrq(analog_ch1, analog_ch2, jnt_trq_feedback);
}

void GetTcpWrench(const RokaeLoadPose& load, const std::vector<double>& jnt_pos, const std::vector<double>& jnt_trq_feedback,
                  const std::vector<double>& jnt_gra_trq, std::array<double, 6>& ext_force) {
    for (unsigned int i = 0; i < jnt_num; i++) {
        jnt_pos_kdl(i) = jnt_pos[i];
        jnt_ext_trq(i) = jnt_gra_trq[i] - jnt_trq_feedback[i];
    }
    dynamicsolver_ptr->GetWrench(load, jnt_pos_kdl, jnt_ext_trq, tcp_wrench);
    for (unsigned int i = 0; i < 3; i++) {
        ext_force[i] = tcp_wrench.force(i);
        ext_force[i + 3] = tcp_wrench.torque(i);
    }
}

//*******************************模型接口*********************************/
const KDL::JntArray& GetGraTorque(const RokaeLoadInertia& load_params, const KDL::JntArray& q) {
    return dynamicsolver_ptr->GetGraTorque(load_params, q);
}

const KDL::JntArray& GetInertTorque(const RokaeLoadInertia& load_params, const KDL::JntArray& q, const KDL::JntArray& ddq) {
    return dynamicsolver_ptr->GetInertTorque(load_params, q, ddq);
}
const KDL::JntArray& GetColioTorque(const RokaeLoadInertia& load_params, const KDL::JntArray& q, const KDL::JntArray& dq) {
    return dynamicsolver_ptr->GetColioTorque(load_params, q, dq);
}
void GetTcpPos(const RokaeLoad& load, const KDL::JntArray& jnt_pos, KDL::Frame& tcp_pos) {
    dynamicsolver_ptr->GetTcpPos(load, jnt_pos, tcp_pos);
}
void JntToMass(const RokaeLoadInertia& load_params, const KDL::JntArray& q, KDL::JntSpaceInertiaMatrix& H) {
    dynamicsolver_ptr->JntToMass(load_params, q, H);
}
void GetTcpJacobian(const RokaeLoad& load, const KDL::JntArray& q, KDL::Jacobian& jacobian) {
    dynamicsolver_ptr->GetTcpJacobian(load, q, jacobian);
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

}  // namespace BasicInterface
}  // namespace RokaeApi
