/**
 * Copyright(C) 2024 Rokae Technology Co., Ltd.
 * All Rights Reserved.
 *
 * Information in this file is the intellectual property of Rokae Technology Co., Ltd,
 * And may contains trade secrets that must be stored and viewed confidentially.
 *
 * @file: inverse_kinematics_solver.hpp
 * @author: wangke
 * @date: 2025/5/26
 * @brief: 逆运动学求解接口
 */

#ifndef ROKAE_HEADER_INVERSE_KINEMATICS_SOLVER_H
#define ROKAE_HEADER_INVERSE_KINEMATICS_SOLVER_H

#include <3rd/kdl/chain.hpp>
#include <3rd/kdl/chainfksolverpos_recursive.hpp>

#include "rokae_header/data_structure_define.hpp"
#include "rokae_header/initialize.hpp"


namespace RokaeApi {
namespace Model {
struct Q4_ABC {
    double a_q2, b_q2, c_q2, a_q6, b_q6, c_q6;
    double an_q1, bn_q1, cn_q1, ad_q1, bd_q1, cd_q1;
    double an_q3, bn_q3, cn_q3, ad_q3, bd_q3, cd_q3;
    double an_q5, bn_q5, cn_q5, ad_q5, bd_q5, cd_q5;
    double an_q7, bn_q7, cn_q7, ad_q7, bd_q7, cd_q7;
    double q4;
    // double a00_q13, b00_q13, c00_q13, a02_q13, b02_q13, c02_q13, a10_q13, b10_q13, c10_q13, a12_q13, b12_q13, c12_q13;
    // double a00_q57, b00_q57, c00_q57, a01_q57, b01_q57, c01_q57, a10_q57, b10_q57, c10_q57, a11_q57, b11_q57, c11_q57;
};

struct Conf_xMate {
    int cf2;  // 2轴区位, 1->theta2>=0; -1->theta2<0.
    int cf4;
    int cf6;
    int cf4_offset;
    double q2_0;
};

class inverse_kinematics_solver {
   public:
    inverse_kinematics_solver(const KDL::Chain& chain, const ModelParams& model_param);
    ~inverse_kinematics_solver();

    IkSolveRes CartToJnt(const KDL::JntArray& curJnt_origin, const GeneralizedFrame& target_Flan, KDL::JntArray& OutJointPose);

    bool IsSingular(const KDL::JntArray& q);
    bool Solve_CurPsi_Conf(const KDL::JntArray& cur_Jnt, const KDL::Frame& cur_Flan, Conf_xMate& conf_xmate, double& psi);
    int Compute_Q4_ABC(const KDL::Frame& tar_Flan, const Conf_xMate& conf_xmate, Q4_ABC& q4_abc);
    void Solve_Jnt(const KDL::JntArray& curJntPose, const Conf_xMate& conf_xmate, const double& targ_Psi, const Q4_ABC& q4_abc,
                   KDL::JntArray& q);
    //一些小接口
    inline double ArcTangentSolver(double an, double bn, double cn, double ad, double bd, double cd, double psi) {
        return std::atan2(an * std::sin(psi) + bn * std::cos(psi) + cn, ad * std::sin(psi) + bd * std::cos(psi) + cd);
    }

    inline double ArcConsineSolver(double a, double b, double c, double psi) {
        double cos_field_angle = a * std::sin(psi) + b * std::cos(psi) + c;
        if (cos_field_angle >= 1.0) {
            cos_field_angle = 1.0;
        } else if (cos_field_angle <= -1.0) {
            cos_field_angle = -1.0;
        }
        return std::acos(cos_field_angle);
    }

   private:
    KDL::Chain m_chain;
    ModelParams m_model_param;
    ModelParams::RobDimensions m_rob_dim;
    std::vector<double> m_max_joint, m_min_joint;
    unsigned int m_joint_num;
    KDL::ChainFkSolverPos_recursive* m_fkpos_ptr;

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
}  // namespace Model
}  // namespace RokaeApi

#endif // ROKAE_HEADER_INVERSE_KINEMATICS_SOLVER_H