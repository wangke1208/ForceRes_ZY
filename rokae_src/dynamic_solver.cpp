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
    m_zeros_jntarry.resize(m_jonit_num);
    m_trq_gravity.resize(m_jonit_num);
    m_trq_coriolis.resize(m_jonit_num);
    m_trq_inertia.resize(m_jonit_num);
    m_trq_total.resize(m_jonit_num);
    KDL::SetToZero(m_zeros_jntarry);
}

DynamicSolver::~DynamicSolver() {
    delete m_chain_dyn_params;
    delete m_chain_dyn_solver;
}

void DynamicSolver::JntToMass(const LoadInertia& load_params, const KDL::JntArray& q, KDL::JntSpaceInertiaMatrix& H) {
    m_chain_dyn_params->JntToMass(GetKDLloadFromRokaeLoad(load_params), q, H);
}

const KDL::JntArray& DynamicSolver::GetGravity(const LoadInertia& load_params, const KDL::JntArray& q) {
    m_load_temp.Zero();
    m_load_temp = GetKDLloadFromRokaeLoad(load_params);
    KDL::SetToZero(m_trq_gravity);
    int res = m_chain_dyn_solver->CartToJnt(q, m_zeros_jntarry, m_zeros_jntarry, m_load_temp, m_trq_gravity);

    return m_trq_gravity;
}

}  // namespace Model
}  // namespace RokaeApi