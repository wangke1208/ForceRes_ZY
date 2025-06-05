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
    m_jnt_sensor_ext_torque.resize(m_jnt_num);
    m_cart_stiffness.resize(6, 100.0);
    m_load.SetZero();
    m_base_frame_buffer.resize(2);
    m_base_frame_buffer[0] = KDL::Frame::Identity();
    m_base_frame_buffer[1] = KDL::Frame::Identity();
    m_frame_active.store(0, std::memory_order_relaxed);
}

FcStatusTracker::~FcStatusTracker() {
    delete m_dynamic_solver;
    delete m_fkpos_ptr;
    delete m_jac_solver;
}

int FcStatusTracker::FcStatusUpdata() {
    //基础计算更新
    FcStatusUpdataCommon();
    switch (FC->drag_type) {
    case Control::DragType::DRAG_JOINT:
    case Control::DragType::IMPEDANCE_JOINT:
        return FcStatusUpdataJoint();
    case Control::DragType::DRAG_CART_TRANS:
    case Control::DragType::DRAG_CART_ROT:
    case Control::DragType::DRAG_CART_FREE:
    case Control::DragType::IMPEDANCE_CART:
        return FcStatusUpdataCart();

    default:
        return ERROR_DRAGTYPE;
    }
}

int FcStatusTracker::FcStatusUpdataJoint() {
    //关节跟踪误差
    KDL::Subtract(FC->jnt_pos_command, FC->jnt_pos_measure, FC->jnt_pos_following_error);
    return SOLVE_NOERROR;
}
int FcStatusTracker::FcStatusUpdataCart() {
    // 1.笛卡尔指令(tcp_in_base)
    if (FC->drag_type != Control::DragType::IMPEDANCE_CART) {
        //非笛卡尔阻抗，需要更新cart_pos_following_error_tcp_in_base_pos
        FC->cart_pos_command_tcp_in_base = FC->cart_pos_command_flan_in_base * m_tool_in_flan;
    }
    // 2.笛卡尔位置偏差(指令相对于测量的偏移)
    FC->cart_pos_following_error_flan_in_base_pos = FC->cart_pos_command_flan_in_base.p - FC->cart_pos_measure_flan_in_base.p;
    FC->cart_pos_following_error_tcp_in_base_pos = FC->cart_pos_command_tcp_in_base.p - FC->cart_pos_measure_tcp_in_base.p;

    // 3.笛卡尔姿态偏差(指令相对于测量的旋转)
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

    // 4.基坐标系下的总偏差（平移+旋转）
    FC->cart_pos_following_error_tcp_in_base.vel = FC->cart_pos_following_error_tcp_in_base_pos;

    // 5.计算雅可比以及速度相关
    m_jac_solver->JntToJac(FC->jnt_pos_command, FC->jac_command_flan_in_base);

    KDL::MultiplyJacobian(FC->jac_measure_flan_in_base, FC->jnt_vel_measure, FC->cart_vel_measure_flan_in_base);
    KDL::MultiplyJacobian(FC->jac_command_flan_in_base, FC->jnt_vel_command, FC->cart_vel_command_flan_in_base);

    KDL::changeRefPoint(FC->jac_measure_flan_in_base, FC->cart_pos_measure_flan_in_base.M * m_tool_in_flan.p,
                        FC->jac_measure_tcp_in_base);
    KDL::MultiplyJacobian(FC->jac_measure_tcp_in_base, FC->jnt_vel_measure, FC->cart_vel_measure_tcp_in_base);
    KDL::MultiplyJacobian(FC->jac_command_tcp_in_base, FC->jnt_vel_command, FC->cart_vel_command_tcp_in_base);
    FC->jac_trans_measure_tcp_in_base = FC->jac_measure_tcp_in_base.data.transpose();

    // 6.计算力控坐标系(base_in_fcframe)
    switch (m_fc_frame_type) {
    case FcFrameType::FCFRAME_TOOL:
        m_fc_frame = m_tool_in_flan.Inverse() * FC->cart_pos_measure_flan_in_base.Inverse();
        break;
    //其他暂时不考虑，先均给到TCP
    default:
        m_fc_frame = m_tool_in_flan.Inverse() * FC->cart_pos_measure_flan_in_base.Inverse();
        break;
    }
    FC->fc_frame = m_fc_frame;

    // 9.将总偏差转到力控坐标系下
    FC->cart_vel_command_tcp_in_fcframe = m_fc_frame.M * FC->cart_vel_command_tcp_in_base;
    FC->cart_vel_measure_tcp_in_fcframe = m_fc_frame.M * FC->cart_vel_measure_tcp_in_base;

    FC->cart_pos_following_error_tcp_in_fcframe = m_fc_frame.M * FC->cart_pos_following_error_tcp_in_base;
    FC->cart_vel_following_error_tcp_in_fcframe = FC->cart_vel_command_tcp_in_fcframe - FC->cart_vel_measure_tcp_in_fcframe;

    return SOLVE_NOERROR;
}

int FcStatusTracker::FcStatusUpdataCommon() {
    //运动学计算反馈部分
    // 1.笛卡尔反馈(flan_in_base)
    m_fkpos_ptr->JntToCart(FC->jnt_pos_measure, FC->cart_pos_measure_flan_in_base);
    // 2.笛卡尔反馈(tcp_in_base)
    FC->cart_pos_measure_tcp_in_base = FC->cart_pos_measure_flan_in_base * m_tool_in_flan;
    // 3.更新base_in_world
    FC->base_in_flan = m_base_frame_buffer[m_frame_active.load(std::memory_order_acquire)];

    //动力学计算部分
    // 1.重力矩
    FC->jnt_gravity_trq_measure = m_dynamic_solver->GetGraTorque(m_load.m_rokae_load_inertia, FC->jnt_pos_measure);
    // 2.惯量
    m_dynamic_solver->JntToMass(m_load.m_rokae_load_inertia, FC->jnt_pos_measure, FC->jnt_inertia_matrix_measure);
    for (unsigned int i = 0; i < m_jnt_num; i++) {
        FC->jnt_inertia(i) = FC->jnt_inertia_matrix_measure(i, i);
    }
    // 3.科式力
    FC->jnt_corlios_trq_measure =
        m_dynamic_solver->GetColioTorque(m_load.m_rokae_load_inertia, FC->jnt_pos_measure, FC->jnt_vel_measure);

    //雅可比计算部分
    m_dynamic_solver->GetFlanJacobian(FC->jnt_pos_measure, FC->jac_measure_flan_in_base);
    FC->jac_trans_measure_flan_in_base = FC->jac_measure_flan_in_base.data.transpose();
    m_dynamic_solver->GetJacobianTransInverse(FC->jac_measure_flan_in_base, FC->jac_trans_inv_measure_flan_in_base);

    //计算可操作度
    FC->mani_measure = m_dynamic_solver->GetManipulate(FC->jac_measure_flan_in_base);

    //计算外力部分
    KDL::Subtract(FC->jnt_gravity_trq_measure, FC->jnt_trq_sensor_measure, m_jnt_sensor_ext_torque);

    //根据可操作度进行Wrench计算
    if(FC->mani_measure < EPSILON5){
        KDL::SetToZero(FC->flan_wrench);
    }else{
        FCVectorXdToWrench(FC->jac_trans_inv_measure_flan_in_base * m_jnt_sensor_ext_torque.data, FC->flan_wrench);
    }

    //转换到TCP末端的Wrench
    FC->tcp_wrench = FC->flan_wrench.RefPoint(m_tool_in_flan.p);
    return SOLVE_NOERROR;
}

void FcStatusTracker::SetLoad(const RokaeLoad& load) {
    //设置负载信息
    m_load = load;
    //更新tool_in_flan坐标系
    m_tool_in_flan = load.m_rokae_load_pose.GetKDLFrame();
    FC->tool_in_flan = m_tool_in_flan;
    return;
}

int FcStatusTracker::SetFcFrameType(const FcFrameType& fc_frame_type) {
    if (fc_frame_type > 3 || fc_frame_type < 0) {
        return ERROR_FC_FRAME_TYPE;
    }
    //设置力控坐标系
    m_fc_frame_type = fc_frame_type;
    return SOLVE_NOERROR;
}

void FcStatusTracker::SetBaseFrame(const KDL::Frame& base_frame) {
    int current = m_frame_active.load(std::memory_order_relaxed);
    int next = 1 - current;
    m_base_frame_buffer[next] = base_frame;
    m_frame_active.store(next, std::memory_order_release);
}

void FcStatusTracker::UnwarpRPY(const KDL::Vector& data_last, KDL::Vector& data) {
    for (unsigned int i = 0; i < 3; i++) {
        if (data[i] - data_last[i] > 6) {
            data[i] -= 2 * PI;
        } else if (data[i] - data_last[i] < -6) {
            data[i] += 2 * PI;
        }
    }
}

void FcStatusTracker::ResetCalStatus() { m_is_rot_angle_outof_range = false; }

void FcStatusTracker::UpdateParams() { m_fc_params_inner_ptr->m_function_params.GetParams("cart_stiff", m_cart_stiffness); }

}  // namespace Control
}  // namespace RokaeApi