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

#include "rokae_header/basic_interface.hpp"
#include "rokae_header/data_structure_define.hpp"
#include "rokae_header/rokae_force_controller.hpp"
namespace RokaeApi {
namespace External {
// ================== 外部数据结构与内部数据结构的转换接口 ==================
inline Model::MechUnitType MechUnitTypeConvert(const External_MechUnitType& external_type) {
    if (external_type != 1) {
        throw std::invalid_argument("Invalid external type: value is unkown");
    }
    return static_cast<Model::MechUnitType>(external_type);
}

inline Control::DragType DragTypeConvert(const External_DragType& external_type) {
    if (external_type < 0 || external_type > 3) {
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

// ================== 初始化整体模块 ==================

int RokaeForce_Init(const External_MechUnitType& robot_type) {
    try {
        return BasicInterface::InitInterface(MechUnitTypeConvert(robot_type));
    } catch (const std::exception& e) {
        std::cerr << "[RokaeForce_Init] Exception: " << e.what() << std::endl;
        return ERROR_ROBOTTYPE;  // 统一错误码
    }
}

void RokaeForce_DeinitInterface() { BasicInterface::DeinitInterface(); }

// ================== 力控算法接口 ==================

// 3.1 拖动配置
int RokaeForce_DragConfig(const std::vector<int32_t>& PDO_0x6064, const std::vector<int8_t>& PDO_0x6061,
                          const std::vector<int16_t>& PDO_0x2401, const std::vector<int16_t>& PDO_0x2402,
                          const External_DragType& drag_type) {
    try {
        auto drag_type_in = DragTypeConvert(drag_type);
        return BasicInterface::DragConfig(PDO_0x6064, PDO_0x6061, PDO_0x2401, PDO_0x2402, drag_type_in);

    } catch (const std::exception& e) {
        std::cerr << "[RokaeForce_DragConfig] Exception: " << e.what() << std::endl;
        return ERROR_DRAGTYPE;  // 统一错误码
    }
}

int RokaeForce_FcUpdate(const std::vector<int8_t>& PDO_0x6061, const std::vector<int16_t>& PDO_0x2401,
                        const std::vector<int16_t>& PDO_0x2402, const std::vector<int16_t>& PDO_0x2406,
                        const std::vector<int32_t>& PDO_0x6064, const std::vector<int32_t>& PDO_0x606C,
                        std::vector<int16_t>& PDO_0x6071, std::vector<int16_t>& PDO_0x60B2, std::vector<int16_t>& PDO_0x2201,
                        std::vector<int16_t>& PDO_0x2202, std::vector<int16_t>& PDO_0x2203, std::vector<int16_t>& PDO_0x2204,
                        std::vector<int16_t>& PDO_0x2205, std::vector<int16_t>& PDO_0x2206);

int RokaeForce_FcStop(const std::vector<int8_t>& PDO_0x6061);
// ================== 参数设置接口(不允许实时设置) ==================

int RokaeForce_SetSensorLinearity(const std::vector<int8_t>& PDO_0x6061, const std::vector<double>& analog2trq_low);

int RokaeForce_SetSensorBias(const std::vector<int8_t>& PDO_0x6061, const std::vector<double>& analog_bias);

int RokaeForce_SetEncoderOffset(const std::vector<int8_t>& PDO_0x6061, const std::vector<int32_t>& encoder_offset);

int RokaeForce_SetSoftLimit(const std::vector<int8_t>& PDO_0x6061, const std::vector<double>& joint_range_min,
                            const std::vector<double>& joint_range_max);

int RokaeForce_SetFcLoad(const std::vector<int8_t>& PDO_0x6061, const External_RokaeLoad& load);

int RokaeForce_SetKpGain(const std::vector<int8_t>& PDO_0x6061, const std::vector<double>& kp_gain_set);

int RokaeForce_SetFricGain(const std::vector<int8_t>& PDO_0x6061, const std::vector<double>& fric_gain_set);
// ================== 参数设置接口(允许实时设置) ==================

int SetBaseFrameAndGravity(const std::array<double, 6>& base_poseture);

// ================== 单独计算接口(可自行输入数据) ==================

int GetAxisPos(const std::vector<int>& PDO_0x6064, std::vector<double>& jnt_pos_rad);

int GetAxisVel(const std::vector<int>& PDO_0x606C, std::vector<double>& jnt_vel_rad);

int GetCobotTrq(const std::vector<int16_t>& PDO_0x2401, const std::vector<int16_t>& PDO_0x2402,
                std::vector<double>& jnt_trq_feedback);

void GetTcpWrench(const External_RokaeLoad& load, const std::vector<double>& jnt_pos, const std::vector<double>& jnt_trq_feedback,
                  const std::vector<double>& jnt_gra_trq, std::array<double, 6>& ext_force);

int GetGraTorque(const External_RokaeLoad& load, const std::vector<double>& jnt_pos, std::vector<double>& trq_gravity);

int GetInertTorque(const External_RokaeLoad& load, const std::vector<double>& jnt_pos, const std::vector<double>& jnt_acc,
                   std::vector<double>& trq_inertia);

int GetCoriolisTorque(const External_RokaeLoad& load, const std::vector<double>& jnt_pos, const std::vector<double>& jnt_vel,
                      std::vector<double>& trq_coriolis);

int GetTotalTorque(const External_RokaeLoad& load, const std::vector<double>& jnt_pos, const std::vector<double>& jnt_vel,
                   const std::vector<double>& jnt_acc, std::vector<double>& trq_total);

int GetTcpPos(const External_RokaeLoad& load, const std::vector<double>& jnt_pos, std::array<double, 6>& tcp_pos);

int GetMassMatrix(const External_RokaeLoad& load, const std::vector<double>& jnt_pos, Eigen::MatrixXd& mass_matrix);

int GetTcpJacobian(const External_RokaeLoad& load, const std::vector<double>& jnt_pos,
                   Eigen::Matrix<double, 6, Eigen::Dynamic>& jacobian);

// ================== 获取拖动状态下的实时数据 ==================

int GetAxisPosCurrent(std::vector<double>& jnt_pos_rad);

int GetAxisVelCurrent(std::vector<double>& jnt_vel_rad);

int GetCobotTrqCurrent(std::vector<double>& jnt_trq_feedback);

int GetTcpWrenchCurrent(std::array<double, 6>& ext_force);

int GetTcpPosCurrent(std::array<double, 6>& tcp_pos);

int GetDynamicTorqueCurrent(std::vector<double>& trq_gravity, std::vector<double>& trq_coriolis, Eigen::MatrixXd& mass_matrix);

int GetJacobianCurrent(Eigen::Matrix<double, 6, Eigen::Dynamic>& jacobian);

// ================== 其他功能 ==================

int CalibrateTrqSensor(const std::vector<int8_t>& PDO_0x6061, const std::vector<int32_t>& PDO_0x606C,
                       const External_RokaeLoad& load_input, const std::vector<std::array<int16_t, 200>>& PDO_0x2401,
                       const std::vector<std::array<int16_t, 200>>& PDO_0x2402, std::vector<double>& sensor_bias);
}  // namespace External

}  // namespace RokaeApi