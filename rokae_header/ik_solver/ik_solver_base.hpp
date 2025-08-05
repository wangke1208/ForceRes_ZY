/**
 * Copyright(C) 2024 Rokae Technology Co., Ltd.
 * All Rights Reserved.
 *
 * Information in this file is the intellectual property of Rokae Technology
 * Co., Ltd, And may contains trade secrets that must be stored and viewed
 * confidentially.
 *
 * @file: ik_solver_base.hpp
 * @author: wangke
 * @date: 2025/8/4
 * @brief: 逆运动学求解器基类
 */

#ifndef ROKAE_HEADER_IK_SOLVER_IK_SOLVER_BASE_HPP
#define ROKAE_HEADER_IK_SOLVER_IK_SOLVER_BASE_HPP

#include <3rd/kdl/chain.hpp>
#include <3rd/kdl/chainfksolverpos_recursive.hpp>

#include "rokae_header/data_structure_define.hpp"
#include "rokae_header/initialize.hpp"
#include "rokae_header/log_util.hpp"
namespace RokaeApi {
namespace Model {
struct Q4_ABC {
    double a_q2, b_q2, c_q2, a_q6, b_q6, c_q6;
    double an_q1, bn_q1, cn_q1, ad_q1, bd_q1, cd_q1;
    double an_q3, bn_q3, cn_q3, ad_q3, bd_q3, cd_q3;
    double an_q5, bn_q5, cn_q5, ad_q5, bd_q5, cd_q5;
    double an_q7, bn_q7, cn_q7, ad_q7, bd_q7, cd_q7;
    double q4;
};

struct Conf_xMate {
    int cf2;  // 2轴区位, 1->theta2>=0; -1->theta2<0.
    int cf4;
    int cf6;
    int cf6_cos;
    int cf4_offset;
    double q2_0;
};
class IKSolverBase {

   public:
    IKSolverBase(const KDL::Chain& chain, const Model::ModelParams& model_param);
    virtual ~IKSolverBase() = default; // 显式默认析构
    virtual IkSolveRes CartToJnt(const KDL::JntArray& curJnt_origin, const GeneralizedFrame& target_Flan,
                                 KDL::JntArray& OutJointPose) = 0;
    virtual bool GetCurPsi(const KDL::JntArray& curJntPose, double& psi) = 0;
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

    inline double ArcSineSolver(double a, double b, double c, double psi){
        double sin_field_angle = a* std::sin(psi) + b * std::cos(psi) + c;
        if(sin_field_angle >= 1.0){
            sin_field_angle = 1.0;
        }else if(sin_field_angle<=-1.0){
            sin_field_angle = -1.0;
        }
        return std::asin(sin_field_angle);
    }
   protected:
    KDL::Chain m_chain;
    Model::ModelParams m_model_param;
    std::shared_ptr<KDL::ChainFkSolverPos_recursive> m_fkpos_ptr;

};

}  // namespace Model
}  // namespace RokaeApi

#endif