/**
 * Copyright(C) 2024 Rokae Technology Co., Ltd.
 * All Rights Reserved.
 *
 * Information in this file is the intellectual property of Rokae Technology Co., Ltd,
 * And may contains trade secrets that must be stored and viewed confidentially.
 *
 * @file: dynamic_solver.cpp
 * @author: wangke
 * @date: 2025/4/7
 * @brief: 动力学求解接口
 */
#include "rokae_header/dynamic_solver.hpp"

namespace RokaeApi {
namespace Model {

DynamicSolver::DynamicSolver(const KDL::Chain& chain, const KDL::Vector& gravity)
    : m_chain(chain), m_gravity(gravity), m_joint_num(chain.getNrOfJoints()) {

    m_chain_dyn_params = new KDL::ChainDynParam(m_chain, m_gravity);
    m_chain_dyn_solver = new KDL::ChainIdSolver_RNE(m_chain, m_gravity);
    m_jnt_to_jac_solver = new KDL::ChainJntToJacSolver(m_chain);
    m_fkpos_ptr = new KDL::ChainFkSolverPos_recursive(m_chain);
    m_svd_ptr = new Eigen::JacobiSVD<Eigen::MatrixXd>(m_jacobian_trans, Eigen::ComputeFullU | Eigen::ComputeFullV);

    m_zeros_jntarry.resize(m_joint_num);
    m_trq_gravity.resize(m_joint_num);
    m_trq_coriolis.resize(m_joint_num);
    m_trq_inertia.resize(m_joint_num);
    m_trq_total.resize(m_joint_num);
    KDL::SetToZero(m_zeros_jntarry);

    m_jacobian.resize(m_joint_num);
    m_jacobian_trans.resize(m_joint_num, 6);

    m_singular_num = KDL::min(m_joint_num, 6);
    m_singular_values.resize(m_singular_num);
    m_singular_values_inv_mat = Eigen::MatrixXd::Zero(6, m_joint_num);
    m_tolerance = EPSILON15;
    m_manipulate = 0.0;

    m_jacobian.resize(m_joint_num);
    m_jacobian_trans.resize(m_joint_num, 6);

    //对外输出需要用到的变量
    m_jac_measure_flan_in_base_out.resize(m_joint_num);
    m_jac_measure_tcp_in_base_out.resize(m_joint_num);
    m_jacobian_trans_inv_out.resize(6, m_joint_num);
    m_cart_pos_measure_flan_in_base_out.Identity();
    m_tool_in_flan.Identity();
    temp.Identity();
    m_flan_wrench_out.Zero();
}

DynamicSolver::~DynamicSolver() {
    delete m_chain_dyn_params;
    delete m_chain_dyn_solver;
    delete m_jnt_to_jac_solver;
    delete m_fkpos_ptr;
    delete m_svd_ptr;
}

// -------------------- 运动学 --------------------

void DynamicSolver::GetTcpPos(const RokaeLoad& load, const KDL::JntArray& jnt_pos, KDL::Frame& tcp_pos) {
    m_fkpos_ptr->JntToCart(jnt_pos, temp);
    tcp_pos = temp * load.m_rokae_load_pose.GetKDLFrame();
}

// -------------------- 动力学 --------------------

void DynamicSolver::JntToMass(const RokaeLoadInertia& load_params, const KDL::JntArray& q, KDL::JntSpaceInertiaMatrix& H) {
    m_chain_dyn_params->JntToMass(GetKDLloadFromRokaeLoad(load_params), q, H);
}

const KDL::JntArray& DynamicSolver::GetTotalTorque(const RokaeLoadInertia& load_params,
                                                   const KDL::JntArray& q,
                                                   const KDL::JntArray& dq,
                                                   const KDL::JntArray& ddq) {
    m_load_temp_all.Zero();
    m_load_temp_all = GetKDLloadFromRokaeLoad(load_params);
    m_chain_dyn_solver->CartToJnt(q, dq, ddq, m_load_temp_all, m_trq_total);
    return m_trq_total;
}

const KDL::JntArray& DynamicSolver::GetGraTorque(const RokaeLoadInertia& load_params, const KDL::JntArray& q) {
    m_load_temp.Zero();
    m_load_temp = GetKDLloadFromRokaeLoad(load_params);
    KDL::SetToZero(m_trq_gravity);
    m_chain_dyn_solver->CartToJnt(q, m_zeros_jntarry, m_zeros_jntarry, m_load_temp, m_trq_gravity);
    return m_trq_gravity;
}

const KDL::JntArray& DynamicSolver::GetInertTorque(const RokaeLoadInertia& load_params,
                                                   const KDL::JntArray& q,
                                                   const KDL::JntArray& ddq) {
    m_load_temp.Zero();
    m_load_temp = GetKDLloadFromRokaeLoad(load_params);
    KDL::SetToZero(m_trq_gravity);
    KDL::SetToZero(m_trq_inertia);

    m_chain_dyn_solver->CartToJnt(q, m_zeros_jntarry, m_zeros_jntarry, m_load_temp, m_trq_gravity);
    m_chain_dyn_solver->CartToJnt(q, m_zeros_jntarry, ddq, m_load_temp, m_trq_inertia);
    KDL::Subtract(m_trq_inertia, m_trq_gravity, m_trq_inertia);

    return m_trq_inertia;
}

const KDL::JntArray& DynamicSolver::GetColioTorque(const RokaeLoadInertia& load_params,
                                                   const KDL::JntArray& q,
                                                   const KDL::JntArray& dq) {
    m_load_temp.Zero();
    m_load_temp = GetKDLloadFromRokaeLoad(load_params);
    KDL::SetToZero(m_trq_gravity);
    KDL::SetToZero(m_trq_coriolis);

    m_chain_dyn_solver->CartToJnt(q, m_zeros_jntarry, m_zeros_jntarry, m_load_temp, m_trq_gravity);
    m_chain_dyn_solver->CartToJnt(q, dq, m_zeros_jntarry, m_load_temp, m_trq_inertia);
    KDL::Subtract(m_trq_coriolis, m_trq_gravity, m_trq_coriolis);

    return m_trq_coriolis;
}

// -------------------- 雅可比矩阵 --------------------

void DynamicSolver::GetFlanJacobian(const KDL::JntArray& q, KDL::Jacobian& jacobian) {
    if (q.data.size() != m_joint_num) return;
    m_jnt_to_jac_solver->JntToJac(q, jacobian);
}

void DynamicSolver::GetTcpJacobian(const RokaeLoad& load, const KDL::JntArray& q, KDL::Jacobian& jacobian) {
    if (q.data.size() != m_joint_num) {
        // TODO::日志
        return;
    }
    KDL::Frame temp;
    temp.Identity();

    m_fkpos_ptr->JntToCart(q, temp);
    m_jnt_to_jac_solver->JntToJac(q, jacobian);
    KDL::changeRefPoint(jacobian, temp.M * m_tool_in_flan.p, jacobian);
}

void DynamicSolver::GetJacobianTrans(const KDL::Jacobian& jacobian, Jacobian_trans& jacobian_trans) {
    jacobian_trans = jacobian.data.transpose();
}

void DynamicSolver::GetJacobianTransInverse(KDL::Jacobian& jacobian, Jacobian_trans_inv& jacobian_trans_inv) {
    //基于奇异值分解求解矩阵的逆
    m_jacobian_trans = jacobian.data.transpose();
    m_svd_ptr->compute(m_jacobian_trans, Eigen::ComputeFullU | Eigen::ComputeFullV);
    m_singular_values = m_svd_ptr->singularValues();

    for (unsigned int i = 0; i < m_singular_num; ++i) {
        m_singular_values_inv_mat(i, i) = m_singular_values(i) > m_tolerance ? 1.0 / m_singular_values(i) : 0;
    }

    jacobian_trans_inv = m_svd_ptr->matrixV() * m_singular_values_inv_mat * m_svd_ptr->matrixU().transpose();
}

// -------------------- 外力与其他 --------------------

void DynamicSolver::GetWrench(const RokaeLoadPose& load,
                              const KDL::JntArray& jnt_pos,
                              const KDL::JntArray& jnt_ext_trq,
                              KDL::Wrench& tcp_wrench) {
    //求解雅可比
    m_jnt_to_jac_solver->JntToJac(jnt_ext_trq, m_jac_measure_flan_in_base_out);
    //计算可操作度
    double mani = GetManipulate(m_jac_measure_flan_in_base_out);
    if (mani < EPSILON5) {
        KDL::SetToZero(m_jac_measure_flan_in_base_out);
        return;
    }
    //求解frame flan_in_base
    m_fkpos_ptr->JntToCart(jnt_pos, m_cart_pos_measure_flan_in_base_out);
    //求解frame tcp_in_flan
    m_tool_in_flan = load.GetKDLFrame();
    //求解法兰雅可比转置的逆
    GetJacobianTransInverse(m_jac_measure_flan_in_base_out, m_jacobian_trans_inv_out);
    //计算flan的wrench
    FCVectorXdToWrench(m_jacobian_trans_inv_out * jnt_ext_trq.data, m_flan_wrench_out);
    //转换到TCP末端的Wrench
    tcp_wrench = m_flan_wrench_out.RefPoint(m_tool_in_flan.p);
}

double DynamicSolver::GetManipulate(const KDL::Jacobian& jacobian) {
    m_manipulate = sqrt((jacobian.data * jacobian.data.transpose()).determinant());
    return m_manipulate;
}

void DynamicSolver::SetGravity(const KDL::Vector& gravity) {
    m_chain_dyn_params->SetGravity(gravity);
    m_chain_dyn_solver->SetGravity(gravity);
}

}  // namespace Model
}  // namespace RokaeApi
