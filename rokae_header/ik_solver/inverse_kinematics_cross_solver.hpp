/**
 * Copyright(C) 2024 Rokae Technology Co., Ltd.
 * All Rights Reserved.
 *
 * Information in this file is the intellectual property of Rokae Technology Co., Ltd,
 * And may contains trade secrets that must be stored and viewed confidentially.
 *
 * @file: inverse_kinematics_cross_solver.hpp
 * @author: wangke
 * @date: 2025/8/4
 * @brief: 十字手腕逆运动学求解接口
 */

#ifndef ROKAE_HEADER_IK_SOLVER_INVERSE_KINEMATICS_CROSS_SOLVER_HPP
#define ROKAE_HEADER_IK_SOLVER_INVERSE_KINEMATICS_CROSS_SOLVER_HPP

#include "rokae_header/ik_solver/ik_solver_base.hpp"

namespace RokaeApi {
namespace Model {
class inverse_kinematics_cross_solver: public IKSolverBase {
   public:
    inverse_kinematics_cross_solver(const KDL::Chain& chain, const ModelParams& model_param);

    ~inverse_kinematics_cross_solver() override = default;

    IkSolveRes CartToJnt(const KDL::JntArray& curJnt_origin, const GeneralizedFrame& target_Flan,
                         KDL::JntArray& OutJointPose) override;

    bool IsSingular(const KDL::JntArray& q);
    bool Solve_CurPsi_Conf(const KDL::JntArray& cur_Jnt, const KDL::Frame& cur_Flan, Conf_xMate& conf_xmate, double& psi);
    int Compute_Q4_ABC(const KDL::Frame& tar_Flan, const Conf_xMate& conf_xmate, Q4_ABC& q4_abc);
    void Solve_Jnt(const KDL::JntArray& curJntPose, const Conf_xMate& conf_xmate, const double& targ_Psi, const Q4_ABC& q4_abc,
                   KDL::JntArray& q);
    bool GetCurPsi(const KDL::JntArray& curJntPose, double& psi) override;

   private:
    ModelParams::RobDimensions m_rob_dim;
    std::vector<double> m_max_joint, m_min_joint;
    unsigned int m_joint_num;

    double m_d_bs, m_d_se, m_d_ew, m_d_wt;
    KDL::Vector Lbs0, Lse3, Lew4, Lwt7;
    double m_q4_offset;
    double m_d_mm;        //肘部偏置
    double m_length_max;  //最大可达长度

    KDL::Vector xend_origin;
    KDL::Rotation R7_0_offset;
    KDL::Vector xw_origin;
    KDL::Vector xsw_origin;
    double d_xsw_origin;
};



}}
#endif