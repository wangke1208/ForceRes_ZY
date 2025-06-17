/**
 * Copyright(C) 2024 Rokae Technology Co., Ltd.
 * All Rights Reserved.
 *
 * Information in this file is the intellectual property of Rokae Technology Co., Ltd,
 * And may contains trade secrets that must be stored and viewed confidentially.
 *
 * @file: rokae_force_controller.cpp
 * @author: wangke
 * @date: 2025/4/11
 * @brief:力控对外接口
 */
#include "rokae_header/rokae_force_controller.hpp"

namespace RokaeApi {
namespace External {
// ================== 外部数据结构与内部数据结构的转换接口 ==================
inline Model::MechUnitType MechUnitTypeConvert(const External_MechUnitType& external_type) {
    if (external_type < 0 || external_type > 2) {
        throw std::invalid_argument("Invalid external type: value is unkown");
    }
    return static_cast<Model::MechUnitType>(external_type);
}

inline Control::DragType DragTypeConvert(const External_DragType& external_type) {
    if (external_type < 0 || external_type > 5) {
        throw std::invalid_argument("Invalid drag type: value out of expected range [0, 3].");
    }
    return static_cast<Control::DragType>(external_type);
}

inline RokaeLoad RokaeLoadConvert(const External_RokaeLoad& external_load) {
    // 转换动力学参数
    RokaeLoadInertia inertia(external_load.mass,
                             KDL::Vector(external_load.centroid[0], external_load.centroid[1], external_load.centroid[2]),
                             external_load.inertia);

    // 转换位姿参数（角度转弧度）
    RokaeLoadPose pose(external_load.position_offset[0], external_load.position_offset[1], external_load.position_offset[2],
                       external_load.posture_rpy[0] * PI / 180.0, external_load.posture_rpy[1] * PI / 180.0,
                       external_load.posture_rpy[2] * PI / 180.0);

    return RokaeLoad(inertia, pose);
}

// ================== 初始化模块接口 ==================

int RokaeForce_Init(const External_MechUnitType& robot_type) {
    try {
        return BasicInterface::InitInterface(MechUnitTypeConvert(robot_type));
    } catch (const std::exception& e) {
        std::cerr << "[RokaeForce_Init] Exception: " << e.what() << std::endl;
        return ERROR_ROBOTTYPE;  // 统一错误码
    }
}

void RokaeForce_Deinit() { BasicInterface::DeinitInterface(); }

// ================== 力控算法接口 ==================

// 3.1 拖动配置
int RokaeForce_DragConfig(const std::vector<int32_t>& PDO_0x6064, const std::vector<int8_t>& PDO_0x6061,
                          const std::vector<int16_t>& PDO_0x2401, const std::vector<int16_t>& PDO_0x2402,
                          const External_DragType& drag_type, const bool& is_command_by_user) {
    try {
        auto drag_type_in = DragTypeConvert(drag_type);
        return BasicInterface::DragConfig(PDO_0x6064, PDO_0x6061, PDO_0x2401, PDO_0x2402, drag_type_in, is_command_by_user);

    } catch (const std::exception& e) {
        std::cerr << "[RokaeForce_DragConfig] Exception: " << e.what() << std::endl;
        return ERROR_DRAGTYPE;  // 统一错误码
    }
}

int RokaeForce_FcUpdate(const std::vector<int8_t>& PDO_0x6061, const std::vector<int16_t>& PDO_0x2401,
                        const std::vector<int16_t>& PDO_0x2402, const std::vector<int16_t>& PDO_0x2406,
                        const std::vector<int32_t>& PDO_0x6064, const std::vector<int32_t>& PDO_0x606C,
                        const std::vector<double>& jnt_pos_cmd_from_user, const std::array<double, 6>& cart_pos_cmd_from_user,
                        const std::vector<double>& jnt_trq_cmd_from_user, std::vector<int16_t>& PDO_0x6071,
                        std::vector<int16_t>& PDO_0x60B2, std::vector<int16_t>& PDO_0x2201, std::vector<int16_t>& PDO_0x2202,
                        std::vector<int16_t>& PDO_0x2203, std::vector<int16_t>& PDO_0x2204, std::vector<int16_t>& PDO_0x2205,
                        std::vector<int16_t>& PDO_0x2206) {
    return BasicInterface::FcUpdate(PDO_0x6061, PDO_0x2401, PDO_0x2402, PDO_0x2406, PDO_0x6064, PDO_0x606C, jnt_pos_cmd_from_user,
                                    cart_pos_cmd_from_user, jnt_trq_cmd_from_user, PDO_0x6071, PDO_0x60B2, PDO_0x2201, PDO_0x2202,
                                    PDO_0x2203, PDO_0x2204, PDO_0x2205, PDO_0x2206);
}

int RokaeForce_FcStop(const std::vector<int8_t>& PDO_0x6061) { return BasicInterface::FcStop(PDO_0x6061); }

// ================== 参数设置接口(不允许实时设置) ==================

int RokaeForce_SetSensorLinearity(const std::vector<int8_t>& PDO_0x6061, const std::vector<double>& analog2trq_low) {
    return BasicInterface::SetSensorLinearity(PDO_0x6061, analog2trq_low);
}

int RokaeForce_SetSensorBias(const std::vector<int8_t>& PDO_0x6061, const std::vector<double>& analog_bias) {
    return BasicInterface::SetSensorBias(PDO_0x6061, analog_bias);
}

int RokaeForce_SetEncoderOffset(const std::vector<int8_t>& PDO_0x6061, const std::vector<int32_t>& encoder_offset) {
    return BasicInterface::SetEncoderOffset(PDO_0x6061, encoder_offset);
}

int RokaeForce_SetSensorFixParams(const std::vector<int8_t>& servo_mode, const std::vector<double>& dynamic_sensor_bias_baseline,
                                  const std::vector<double>& pos_sensor_fix_params,
                                  const std::vector<double>& neg_sensor_fix_params) {
    return BasicInterface::SetSensorFixParams(servo_mode, dynamic_sensor_bias_baseline, pos_sensor_fix_params,
                                              neg_sensor_fix_params);
}

int RokaeForce_SetSoftLimit(const std::vector<int8_t>& PDO_0x6061, const std::vector<double>& joint_range_min,
                            const std::vector<double>& joint_range_max) {
    return BasicInterface::SetSoftLimit(PDO_0x6061, joint_range_min, joint_range_max);
}

int RokaeForce_SetFcLoad(const std::vector<int8_t>& PDO_0x6061, const External_RokaeLoad& load) {
    return BasicInterface::SetFcLoad(PDO_0x6061, RokaeLoadConvert(load));
}

int RokaeForce_SetKpGain(const std::vector<int8_t>& PDO_0x6061, const std::vector<double>& kp_gain_set) {
    return BasicInterface::SetKpGain(PDO_0x6061, kp_gain_set);
}

int RokaeForce_SetFricGain(const std::vector<int8_t>& PDO_0x6061, const std::vector<double>& fric_gain_set) {
    return BasicInterface::SetFricGain(PDO_0x6061, fric_gain_set);
}
int RokaeForce_SetJointImpedance(const std::vector<int8_t>& PDO_0x6061, const std::vector<double>& joint_impedance) {
    return BasicInterface::SetJointImpedance(PDO_0x6061, joint_impedance);
};
int RokaeForce_SetCartesianImpedance(const std::vector<int8_t>& PDO_0x6061, const std::array<double, 6>& cartesian_impedance) {
    return BasicInterface::SetCartImpedance(PDO_0x6061, cartesian_impedance);
};

// ================== 参数设置接口(允许实时设置) ==================

int RokaeForce_SetBaseFrameAndGravity(const std::array<double, 6>& base_poseture) {
    return BasicInterface::SetBaseFrameAndGravity(base_poseture);
}

// ================== 单独计算接口(可自行输入数据) ==================

int RokaeForce_GetAxisPos(const std::vector<int>& PDO_0x6064, std::vector<double>& jnt_pos_rad) {
    return BasicInterface::GetAxisPos(PDO_0x6064, jnt_pos_rad);
}

int RokaeForce_GetAxisVel(const std::vector<int>& PDO_0x606C, std::vector<double>& jnt_vel_rad) {
    return BasicInterface::GetAxisVel(PDO_0x606C, jnt_vel_rad);
}

int RokaeForce_GetCobotTrq(const std::vector<int16_t>& PDO_0x2401, const std::vector<int16_t>& PDO_0x2402,
                std::vector<double>& jnt_trq_feedback) {
    return BasicInterface::GetCobotTrq(PDO_0x2401, PDO_0x2402, jnt_trq_feedback);
}

void RokaeForce_GetTcpWrench(const External_RokaeLoad& load, const std::vector<double>& jnt_pos, const std::vector<double>& jnt_trq_feedback,
                  const std::vector<double>& jnt_gra_trq, std::array<double, 6>& ext_force) {
    BasicInterface::GetTcpWrench(RokaeLoadConvert(load), jnt_pos, jnt_trq_feedback, jnt_gra_trq, ext_force);
}

int RokaeForce_GetGraTorque(const External_RokaeLoad& load, const std::vector<double>& jnt_pos, std::vector<double>& trq_gravity) {
    return BasicInterface::GetGraTorque(RokaeLoadConvert(load), jnt_pos, trq_gravity);
}

int RokaeForce_GetInertTorque(const External_RokaeLoad& load, const std::vector<double>& jnt_pos, const std::vector<double>& jnt_acc,
                   std::vector<double>& trq_inertia) {
    return BasicInterface::GetInertTorque(RokaeLoadConvert(load), jnt_pos, jnt_acc, trq_inertia);
}

int RokaeForce_GetCoriolisTorque(const External_RokaeLoad& load, const std::vector<double>& jnt_pos, const std::vector<double>& jnt_vel,
                      std::vector<double>& trq_coriolis) {
    return BasicInterface::GetCoriolisTorque(RokaeLoadConvert(load), jnt_pos, jnt_vel, trq_coriolis);
}

int RokaeForce_GetTotalTorque(const External_RokaeLoad& load, const std::vector<double>& jnt_pos, const std::vector<double>& jnt_vel,
                   const std::vector<double>& jnt_acc, std::vector<double>& trq_total) {
    return BasicInterface::GetTotalTorque(RokaeLoadConvert(load), jnt_pos, jnt_vel, jnt_acc, trq_total);
}

int RokaeForce_GetJointPos(const std::array<double, 16>& CartPos, const double& psi, const std::vector<double>& q_init,
                           std::vector<double>& q_out) {
    KDL::Frame frame = KDL::Frame::Identity();
    ConvertFrameArray(frame, CartPos);
    GeneralizedFrame GF(frame, psi);
    return BasicInterface::GetJointPos(q_init, GF, q_out);
}

int RokaeForce_GetCurPsi(const std::vector<double>& curJntPose, double& psi) {
    return BasicInterface::GetCurPsi(curJntPose, psi);
}

int RokaeForce_GetFlanPos(const std::vector<double>& jnt_pos,
                         std::array<double, 16>& flanTobase_pos) {

    return BasicInterface::GetFlanPos(jnt_pos, flanTobase_pos);
}
int RokaeForce_GetTcpPos(const External_RokaeLoad& load, const std::vector<double>& jnt_pos,
                         std::array<double, 16>& toolTobase_pos, std::array<double, 6>& tcp_pos) {
    return BasicInterface::GetTcpPos(RokaeLoadConvert(load), jnt_pos, toolTobase_pos, tcp_pos);
}

int RokaeForce_GetMassMatrix(const External_RokaeLoad& load, const std::vector<double>& jnt_pos, Eigen::MatrixXd& mass_matrix) {
    return BasicInterface::GetMassMatrix(RokaeLoadConvert(load), jnt_pos, mass_matrix);
}

int RokaeForce_GetTcpJacobian(const External_RokaeLoad& load, const std::vector<double>& jnt_pos,
                              Eigen::Matrix<double, 6, Eigen::Dynamic>& jacobian) {
    return BasicInterface::GetTcpJacobian(RokaeLoadConvert(load), jnt_pos, jacobian);
}

// ================== 获取拖动状态下的实时数据 ==================

int RokaeForce_GetAxisPosCurrent(std::vector<double>& jnt_pos_rad) { return BasicInterface::GetAxisPosCurrent(jnt_pos_rad); }

int RokaeForce_GetAxisVelCurrent(std::vector<double>& jnt_vel_rad) { return BasicInterface::GetAxisVelCurrent(jnt_vel_rad); }

int RokaeForce_GetCobotTrqCurrent(std::vector<double>& jnt_trq_feedback) {
    return BasicInterface::GetCobotTrqCurrent(jnt_trq_feedback);
}

int RokaeForce_GetTcpWrenchCurrent(std::array<double, 6>& ext_force) { return BasicInterface::GetTcpWrenchCurrent(ext_force); }

int RokaeForce_GetTcpPosCurrent(std::array<double, 6>& tcp_pos) { return BasicInterface::GetTcpPosCurrent(tcp_pos); }

int RokaeForce_GetDynamicTorqueCurrent(std::vector<double>& trq_gravity, std::vector<double>& trq_coriolis,
                                       Eigen::MatrixXd& mass_matrix) {
    return BasicInterface::GetDynamicTorqueCurrent(trq_gravity, trq_coriolis, mass_matrix);
}

int RokaeForce_GetJacobianCurrent(Eigen::Matrix<double, 6, Eigen::Dynamic>& jacobian) {
    return BasicInterface::GetJacobianCurrent(jacobian);
}

// ================== 其他功能 ==================

int RokaeForce_CalibrateTrqSensor(const std::vector<int8_t>& PDO_0x6061, const std::vector<int32_t>& PDO_0x606C,
                                  const External_RokaeLoad& load_input, const std::vector<std::array<int16_t, 200>>& PDO_0x2401,
                                  const std::vector<std::array<int16_t, 200>>& PDO_0x2402, std::vector<double>& sensor_bias) {
    return BasicInterface::CalibrateTrqSensor(PDO_0x6061, PDO_0x606C, RokaeLoadConvert(load_input), PDO_0x2401, PDO_0x2402,
                                              sensor_bias);
}
}  // namespace External

}  // namespace RokaeApi