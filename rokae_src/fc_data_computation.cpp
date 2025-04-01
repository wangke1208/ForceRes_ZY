/**
 * Copyright(C) 2024 Rokae Technology Co., Ltd.
 * All Rights Reserved.
 *
 * Information in this file is the intellectual property of Rokae Technology Co., Ltd,
 * And may contains trade secrets that must be stored and viewed confidentially.
 *
 * @file: fc_data_computation.cpp
 * @author: wangke
 * @date: 2025/3/27
 * @brief: 力控数据流
 */

#include "rokae_header/fc_data_computation.hpp"
#define FC m_fc_status_info
namespace RokaeApi {
namespace Control {

FcStatusTracker::FcStatusTracker(InitRobot* init_robot_ptr, FcStatusInner* fc_status_ptr, FcParamsInner* fc_params_inner)
    : m_jnt_num(init_robot_ptr->GetChain().getNrOfJoints()),
      m_fc_status_info(fc_status_ptr),
      m_fc_params_inner_ptr(fc_params_inner) {
    m_dynamic_solver = new Model::DynamicSolver(init_robot_ptr->GetChain(), init_robot_ptr->GetGravity());
    m_fkpos_ptr = new KDL::ChainFkSolverPos_recursive(init_robot_ptr->GetChain());
    m_jac_solver = new KDL::ChainJntToJacSolver(init_robot_ptr->GetChain());
    m_tool_in_flan = KDL::Frame::Identity();
    m_fc_frame = KDL::Frame::Identity();
    m_if_first_in = true;
    m_fc_frame_type = FcFrameType::FCFRAME_TOOL;
    m_is_rot_angle_outof_range = false;
    KDL::SetToZero(m_orient_delta_d);
    KDL::SetToZero(m_orient_delta_d_last);
    m_cart_stiffness.resize(6, 100.0);
}

FcStatusTracker::~FcStatusTracker() {
    delete m_dynamic_solver;
    delete m_fkpos_ptr;
    delete m_jac_solver;
}

int FcStatusTracker::FcStatusUpdata() {
    switch (FC->drag_type) {
    case Control::DragType::DRAG_JOINT:
        FcStatusUpdataJoint();
        return FcStatusUpdataDynamic();
    case Control::DragType::DRAG_CART_TRANS:
    case Control::DragType::DRAG_CART_ROT:
    case Control::DragType::DRAG_CART_FREE:
        int res = FcStatusUpdataCart();
        if (!res) {
            return res;
        } else {
            return FcStatusUpdataDynamic();
        }
    default:
        return DRAGTYPE_ERROR;
    }
}

int FcStatusTracker::FcStatusUpdataJoint() {
    //关节跟踪误差
    KDL::Subtract(FC->jnt_pos_command, FC->jnt_pos_measure, FC->jnt_pos_following_error);
    return SOLVE_NOERROR;
}
int FcStatusTracker::FcStatusUpdataCart() {
    // 1.笛卡尔反馈(flan_in_base)
    m_fkpos_ptr->JntToCart(FC->jnt_pos_measure, FC->cart_pos_measure_flan_in_base);
    // 2.笛卡尔反馈(tcp_in_base)
    FC->cart_pos_measure_tcp_in_base = FC->cart_pos_measure_flan_in_base * m_tool_in_flan;

    // 3.笛卡尔指令(tcp_in_base)
    FC->cart_pos_command_tcp_in_base = FC->cart_pos_command_flan_in_base * m_tool_in_flan;

    // 4.笛卡尔位置偏差(指令相对于测量的偏移)
    FC->cart_pos_following_error_flan_in_base_pos = FC->cart_pos_command_flan_in_base.p - FC->cart_pos_measure_flan_in_base.p;
    FC->cart_pos_following_error_tcp_in_base_pos = FC->cart_pos_command_tcp_in_base.p - FC->cart_pos_measure_tcp_in_base.p;

    // 5.笛卡尔姿态偏差(指令相对于测量的旋转)
    FC->cart_tcp_rot_between_command_and_measure =
        FC->cart_pos_measure_tcp_in_base.M.Inverse() * FC->cart_pos_command_tcp_in_base.M;
    if (!m_is_rot_angle_outof_range) {
        m_orient_delta_d = FC->cart_tcp_rot_between_command_and_measure.GetRot();  //
        if (m_if_first_in == true) {
            m_orient_delta_d_last = m_orient_delta_d;
            m_if_first_in = false;
        }
    }
    //解缠绕
    UnwarpRPY(m_orient_delta_d_last, m_orient_delta_d);
    m_orient_delta_d_last = m_orient_delta_d;
    //旋转分量范围检查，如果任意一个分量接近 ±π，则认为旋转角度超出范围
    m_fc_params_inner_ptr->m_function_params.GetParams("cart_stiff", m_cart_stiffness);
    while (m_cart_stiffness[3] > EPSILON4 or m_cart_stiffness[4] > EPSILON4 or m_cart_stiffness[5] > EPSILON4) {
        if (fabs(m_orient_delta_d[0]) >= (PI - EPSILON1) || fabs(m_orient_delta_d[1]) >= (PI - EPSILON1) ||
            fabs(m_orient_delta_d[2]) >= (PI - EPSILON1)) {
            m_is_rot_angle_outof_range = true;
            return ERROR_RPY_CAL;
        }
        break;
    }
    //将姿态偏差转到基坐标系下
    if (!m_is_rot_angle_outof_range) {
        m_orient_delta_d = FC->cart_pos_command_tcp_in_base.M * m_orient_delta_d;
        FC->cart_pos_following_error_tcp_in_base.rot = m_orient_delta_d;
    }

    //6.基坐标系下的总偏差（平移+旋转）
    FC->cart_pos_following_error_tcp_in_base.vel = FC->cart_pos_following_error_tcp_in_base_pos;

    // 7.计算雅可比以及速度相关
    m_jac_solver->JntToJac(FC->jnt_pos_measure, FC->jac_measure_flan_in_base);
    m_jac_solver->JntToJac(FC->jnt_pos_command, FC->jac_command_flan_in_base);

    KDL::MultiplyJacobian(FC->jac_measure_flan_in_base, FC->jnt_vel_measure, FC->cart_vel_measure_flan_in_base);
    KDL::MultiplyJacobian(FC->jac_command_flan_in_base, FC->jnt_vel_command, FC->cart_vel_command_flan_in_base);

    KDL::changeRefPoint(FC->jac_measure_flan_in_base, FC->cart_pos_measure_flan_in_base.M * m_tool_in_flan.p,
                        FC->jac_measure_tcp_in_base);
    KDL::MultiplyJacobian(FC->jac_measure_tcp_in_base, FC->jnt_vel_measure, FC->cart_vel_measure_tcp_in_base);
    KDL::MultiplyJacobian(FC->jac_command_tcp_in_base, FC->jnt_vel_command, FC->cart_vel_command_tcp_in_base);
    FC->cart_vel_command_tcp_in_fcframe = m_fc_frame.M * FC->cart_vel_command_tcp_in_base;
    FC->cart_vel_measure_tcp_in_fcframe = m_fc_frame.M * FC->cart_vel_measure_tcp_in_base;

    // 8.计算力控坐标系
    switch (m_fc_frame_type) {
    case FcFrameType::FCFRAME_TOOL:
        m_fc_frame =
            m_tool_in_flan.Inverse() * FC->cart_pos_measure_flan_in_base.Inverse() * FC->cart_pos_command_flan_in_base.Inverse();
        break;
    //其他暂时不考虑，先均给到TCP
    default:
        m_fc_frame =
            m_tool_in_flan.Inverse() * FC->cart_pos_measure_flan_in_base.Inverse() * FC->cart_pos_command_flan_in_base.Inverse();
        break;
    }

    // 9.将总偏差转到力控坐标系下

    FC->cart_pos_following_error_tcp_in_fcframe = m_fc_frame.M * FC->cart_pos_following_error_tcp_in_base;
    FC->cart_vel_following_error_tcp_in_fcframe = FC->cart_vel_command_tcp_in_fcframe - FC->cart_vel_measure_tcp_in_fcframe;

    return SOLVE_NOERROR;
}

int FcStatusTracker::FcStatusUpdataDynamic() {
    //动力学计算部分
    // 1.重力矩
    FC->jnt_gravity_trq_measure = m_dynamic_solver->GetGraTorque(m_load, FC->jnt_pos_measure);
    // 2.惯量
    m_dynamic_solver->JntToMass(m_load, FC->jnt_pos_measure, FC->jnt_inertia_matrix_measure);
    for(unsigned int i = 0; i < m_jnt_num; i++){
            m_joint_inertia[i] = m_fc_status.jnt_inertia_matrix_measure(i, i);

    }
    // 3.科式力(不在这里计算)
    return SOLVE_NOERROR;
}

void FcStatusTracker::SetLoad(const LoadInertia& load) {m_load = load; }

void FcStatusTracker::UnwarpRPY(const KDL::Vector& data_last, KDL::Vector& data) {
    for (unsigned int i = 0; i < 3; i++) {
        if (data[i] - data_last[i] > 6) {
            data[i] -= 2 * PI;
        } else if (data[i] - data_last[i] < -6) {
            data[i] += 2 * PI;
        }
    }
}

}  // namespace Control
}  // namespace RokaeApi