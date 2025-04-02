/**
 * Copyright(C) 2024 Rokae Technology Co., Ltd.
 * All Rights Reserved.
 *
 * Information in this file is the intellectual property of Rokae Technology Co., Ltd,
 * And may contains trade secrets that must be stored and viewed confidentially.
 *
 * @file: dynamic_solver.cpp
 * @author: wangke
 * @date: 2024/4/25
 * @brief: 动力学解算模块(纯连杆)
 */
#include "rokae_header/dynamic_solver.hpp"

namespace RokaeApi {
namespace Model {
DynamicSolver::DynamicSolver(const KDL::Chain& chain, const KDL::Vector& gravity) : m_chain(chain), m_gravity(gravity) {
    m_jonit_num = m_chain.getNrOfJoints();

    m_chain_dyn_params = new KDL::ChainDynParam(m_chain, m_gravity);
    m_chain_dyn_solver = new KDL::ChainIdSolver_RNE(m_chain, m_gravity);
    m_jnt_to_jac_solver = new KDL::ChainJntToJacSolver(m_chain);
    m_svd_ptr = new Eigen::JacobiSVD<Eigen::MatrixXd>(m_jacobian_trans, Eigen::ComputeFullU | Eigen::ComputeFullV);

    m_zeros_jntarry.resize(m_jonit_num);
    m_trq_gravity.resize(m_jonit_num);
    m_trq_coriolis.resize(m_jonit_num);
    m_trq_inertia.resize(m_jonit_num);
    m_trq_total.resize(m_jonit_num);
    KDL::SetToZero(m_zeros_jntarry);

    //雅可比相关
    m_manipulate = 0.0;
    m_singular_num = KDL::min(m_jonit_num, 6);
    m_singular_values.resize(m_singular_num);
    m_singular_values_inv_mat = Eigen::MatrixXd::Zero(6, m_jonit_num);
    m_tolerance = EPSILON15;

    m_jacobian.resize(m_jonit_num);
    m_jacobian_trans.resize(m_jonit_num, 6);
}

DynamicSolver::~DynamicSolver() {
    delete m_chain_dyn_params;
    delete m_chain_dyn_solver;
    delete m_jnt_to_jac_solver;
    delete m_svd_ptr;
}

void DynamicSolver::JntToMass(const RokaeLoadInertia& load_params, const KDL::JntArray& q, KDL::JntSpaceInertiaMatrix& H) {
    m_chain_dyn_params->JntToMass(GetKDLloadFromRokaeLoad(load_params), q, H);
}

const KDL::JntArray& DynamicSolver::GetGraTorque(const RokaeLoadInertia& load_params, const KDL::JntArray& q) {
    m_load_temp.Zero();
    m_load_temp = GetKDLloadFromRokaeLoad(load_params);
    KDL::SetToZero(m_trq_gravity);
    int res = m_chain_dyn_solver->CartToJnt(q, m_zeros_jntarry, m_zeros_jntarry, m_load_temp, m_trq_gravity);

    return m_trq_gravity;
}

const KDL::JntArray& DynamicSolver::GetInertTorque(const RokaeLoadInertia& load_params, const KDL::JntArray& q,const KDL::JntArray& ddq) {
    m_load_temp.Zero();
    m_load_temp = GetKDLloadFromRokaeLoad(load_params);
    KDL::SetToZero(m_trq_gravity);
    KDL::SetToZero(m_trq_inertia);
    m_chain_dyn_solver->CartToJnt(q, m_zeros_jntarry, m_zeros_jntarry, m_load_temp, m_trq_gravity);
    m_chain_dyn_solver->CartToJnt(q, m_zeros_jntarry, ddq, m_load_temp, m_trq_inertia);
    KDL::Subtract(m_trq_inertia, m_trq_gravity, m_trq_inertia);
    return m_trq_inertia;
}

const KDL::JntArray& DynamicSolver::GetColioTorque(const RokaeLoadInertia& load_params, const KDL::JntArray& q,const KDL::JntArray& dq){
    m_load_temp.Zero();
    m_load_temp = GetKDLloadFromRokaeLoad(load_params);
    KDL::SetToZero(m_trq_gravity);
    KDL::SetToZero(m_trq_coriolis);
    m_chain_dyn_solver->CartToJnt(q, m_zeros_jntarry, m_zeros_jntarry, m_load_temp, m_trq_gravity);
    m_chain_dyn_solver->CartToJnt(q, dq, m_zeros_jntarry, m_load_temp, m_trq_inertia);
    KDL::Subtract(m_trq_coriolis, m_trq_gravity, m_trq_coriolis);
    return m_trq_coriolis;
}

void DynamicSolver::GetJacobian(const KDL::JntArray& q, KDL::Jacobian& jacobian) {
    if (q.data.size() != m_jonit_num) {
        // TODO::日志
        return;
    }
    m_jnt_to_jac_solver->JntToJac(q, jacobian);
}

void DynamicSolver::GetJacobianTrans(const KDL::Jacobian& jacobian, Jacobian_trans& jacobian_trans) {
    jacobian_trans = jacobian.data.transpose();
}

void DynamicSolver::GetJacobianTransInverse(KDL::Jacobian& jacobian, Jacobian_trans_inv& jacobian_trans_inv) {
    //基于奇异值分解求解矩阵的逆
    m_jacobian_trans = jacobian.data.transpose();
    m_svd_ptr->compute(m_jacobian_trans, Eigen::ComputeFullU | Eigen::ComputeFullV);
    m_singular_values = m_svd_ptr->singularValues();
    for (unsigned int i = 0; i < m_singular_num; i++) {
        m_singular_values_inv_mat(i, i) = m_singular_values(i) > m_tolerance ? 1.0 / m_singular_values(i) : 0;
    }
    jacobian_trans_inv = (m_svd_ptr->matrixV()) * (m_singular_values_inv_mat) * (m_svd_ptr->matrixU().transpose());
}

double DynamicSolver::GetManipulate(const KDL::Jacobian& jacobian) {
    m_manipulate = sqrt((jacobian.data * jacobian.data.transpose()).determinant());
    return m_manipulate;
}
}  // namespace Model
}  // namespace RokaeApi