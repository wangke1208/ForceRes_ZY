/**
 * Copyright(C) 2024 Rokae Technology Co., Ltd.
 * All Rights Reserved.
 *
 * Information in this file is the intellectual property of Rokae Technology Co., Ltd,
 * And may contains trade secrets that must be stored and viewed confidentially.
 *
 * @file: force_planner.hpp
 * @author: wangke
 * @date: 2024/4/25
 * @brief: 力控计算模块
 */

#ifndef ROKAE_HEADER_FORCE_PLANNER_HPP_
#define ROKAE_HEADER_FORCE_PLANNER_HPP_

#include <algorithm>
#include <cmath>

#include "rokae_header/fc_data_computation.hpp"

namespace RokaeApi {
namespace Control {

class ForcePlanner {
   public:
   EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    /**
     * @brief 默认构造函数
     *
     * @return 无
     */
    ForcePlanner();

    /**
     * @brief 析构函数
     *
     * @return 无
     */
    ~ForcePlanner();

    /**
     * @brief 带参数构造函数
     *
     * @param[in] init_robot_ptr      初始化机器人指针
     * @param[in] fc_status_ptr       内部力控状态数据指针
     * @param[in] fc_params_inner_ptr 内部力控参数指针
     *
     * @return 无
     */
    ForcePlanner(InitRobot* init_robot_ptr, FcStatusInner* fc_status_ptr, FcParamsInner* fc_params_inner_ptr);

    /**
     * @brief 更新功能力计算（包括重力和阻抗）
     *
     * @return 无
     */
    void ForcePlannerUpdata();

    /**
     * @brief 更新功能力计算（用户设定）
     * @param[in] trq_cmd_by_user 用户设定的关节力矩指令
     * @return 无
     */
    void ForcePlannerUpdataUser(const KDL::JntArray& trq_cmd_by_user);
   
    /**
     * @brief 更新关节阻抗力计算
     *
     * @param[out] function_imp_trq 输出关节阻抗力
     *
     * @return 无
     */
    void JointImpedanceUpdate(KDL::JntArray& function_imp_trq);

    /**
     * @brief 更新笛卡尔阻抗力计算
     *
     * @param[out] function_imp_trq 输出笛卡尔阻抗力
     *
     * @return 无
     */
    void CartImpedanceUpdate(KDL::JntArray& function_imp_trq);

    /**
     * @brief 更新内部参数
     *
     */
    void UpdateParams();

   private:
    unsigned int m_jnt_num;  ///< 机器人关节数量

    InitRobot* m_init_robot_ptr;           ///< 初始化机器人指针
    FcStatusInner* m_fc_status_ptr;        ///< 内部力控状态数据指针
    FcParamsInner* m_fc_params_inner_ptr;  ///< 内部力控参数指针

    // 控制参数
    std::vector<double> m_jnt_stiff;   ///< 关节刚度参数
    std::vector<double> m_jnt_damp;    ///< 关节阻尼参数
    std::vector<double> m_cart_stiff;  ///< 笛卡尔刚度参数
    std::vector<double> m_cart_damp;   ///< 笛卡尔阻尼参数
    std::vector<double> m_null_stiff;  //零空间刚度

    // 功能力相关变量
    KDL::JntArray m_function_trq_ref;             ///< 参考合力
    KDL::JntArray m_function_jnt_gravity;         ///< 重力补偿
    KDL::JntArray m_function_imp_trq;             ///< 关节阻抗力
    KDL::JntArray m_function_jnt_imp_damp_trq;    ///< 关节空间阻尼阻抗力
    KDL::JntArray m_function_jnt_imp_stiff_trq;   ///< 关节空间刚度阻抗力

    KDL::Wrench m_function_cart_imp_trq;          ///< 笛卡尔阻抗力
    KDL::JntArray m_function_cart_imp_joint_trq;  ///< 笛卡尔阻抗力折算到末端
    KDL::JntArray m_function_cart_imp_stiff_trq;  ///< 笛卡尔刚度阻抗力
    KDL::JntArray m_function_cart_imp_damp_trq;   ///< 笛卡尔阻尼阻抗力
    KDL::JntArray m_function_jnt_zero_trq;        ///< 空数据（零力数据）
    KDL::JntArray m_function_null_space_trq;      //零空间虚拟阻抗力矩
    KDL::JntArray m_function_null_space_trq_final;  //投影后的零空间阻抗力矩

    Eigen::MatrixXd J_pinv;
    Eigen::Matrix<double, 6, 1> m_function_cart_imp_trq_in_base;
    Eigen::MatrixXd m_matrix_temp;
    KDL::Wrench m_function_cart_imp_trq_in_base_wrench;
};

}  // namespace Control
}  // namespace RokaeApi

#endif
