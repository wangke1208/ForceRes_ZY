/**
 * Copyright(C) 2024 Rokae Technology Co., Ltd.
 * All Rights Reserved.
 *
 * Information in this file is the intellectual property of Rokae Technology Co., Ltd,
 * And may contains trade secrets that must be stored and viewed confidentially.
 *
 * @file: dynamic_solver.hpp
 * @author: wangke
 * @date: 2024/4/25
 * @brief: 动力学解算模块
 */
#ifndef DYNAMIC_SOLVER_H
#define DYNAMIC_SOLVER_H

#include <../3rd/kdl/chain.hpp>
#include <../3rd/kdl/chaindynparam.hpp>
#include <../3rd/kdl/chainfksolverpos_recursive.hpp>
#include <../3rd/kdl/chainidsolver_recursive_newton_euler.hpp>
#include <../3rd/kdl/jntarray.hpp>
#include <../3rd/kdl/jntspaceinertiamatrix.hpp>

#include "data_structure_convert.hpp"
#include "data_structure_define.hpp"


using namespace KDL;
namespace RokaeApi {
namespace Model {

class DynamicSolver {
   public:
    DynamicSolver(const KDL::Chain& chain, const KDL::Vector& gravity);
    ~DynamicSolver();

    /**
     * @brief 计算连杆惯量
     *
     * @param[in] load_params:负载信息（质量、质心、一阶矩、二阶矩）
     * @param[in] q:关节角度(弧度)
     * @param[out] H:惯量阵
     *
     * @return 无
     */
    void JntToMass(const LoadInertia& load_params, const KDL::JntArray& q, KDL::JntSpaceInertiaMatrix& H);

    /**
     * @brief 计算动力学总力矩，，包括重力、科式力、惯性力
     *
     * @param[in] load_params:初始化用到的相关参数，包括关节类型、RD参数等
     * @param[in] q:关节角度(弧度)
     * @param[in] dq:关节角速度
     * @param[in] ddq:关节角加速度
     *
     * @return 动力学总力矩(KDL::JntArray)
     */
    const KDL::JntArray& GetTotalTorque(const LoadInertia& load_params, const KDL::JntArray& q, const KDL::JntArray& dq, const KDL::JntArray& ddq);

    /**
     * @brief 计算重力矩
     *
     * @param[in] load_params:初始化用到的相关参数，包括关节类型、RD参数等
     * @param[in] q:关节角度(弧度)
     *
     * @return 重力矩(KDL::JntArray)
     */
    const KDL::JntArray& GetGravity(const LoadInertia& load_params, const KDL::JntArray& q);

   private:
    KDL::Chain m_chain;
    KDL::ChainDynParam* m_chain_dyn_params;
    KDL::ChainIdSolver_RNE* m_chain_dyn_solver;

   private:
    unsigned int m_jonit_num;
    KDL::RigidBodyInertia m_load_temp;

    KDL::Vector m_gravity;  //重力矢量，默认-9.81
    
    KDL::JntArray m_zeros_jntarry;  //零
    KDL::JntArray m_trq_gravity;   //重力矩
    KDL::JntArray m_trq_inertia;   //惯性力矩
    KDL::JntArray m_trq_coriolis;  //科式力矩
    KDL::JntArray m_trq_total;     //全力矩
};

}  // namespace Model
}  // namespace RokaeApi

#endif