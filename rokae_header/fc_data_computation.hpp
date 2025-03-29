/**
 * Copyright(C) 2024 Rokae Technology Co., Ltd.
 * All Rights Reserved.
 *
 * Information in this file is the intellectual property of Rokae Technology Co., Ltd,
 * And may contains trade secrets that must be stored and viewed confidentially.
 *
 * @file: fc_data_computation.hpp
 * @author: wangke
 * @date: 2025/3/27
 * @brief: 力控数据流计算
 */
#ifndef ROKAE_HEADER_FC_DATA_COMPUTATION_HPP_
#define ROKAE_HEADER_FC_DATA_COMPUTATION_HPP_

#include "data_structure_define.hpp"
#include "dynamic_solver.hpp"
#include "initialize.hpp"

using namespace RokaeApi::Model;

namespace RokaeApi {
namespace Control {
class FcStatusTracker {
   public:
    FcStatusTracker(InitRobot* init_robot_ptr);

    ~FcStatusTracker();

    /**
     * @brief 力控模块数据流计算
     *
     * @return 无
     */
    void UpdateDateStream();

    /**
     * @brief 设置负载
     *
     * @param[in] load:负载信息
     * @return 无
     */
    void SetLoad(const LoadInertia& load);


   private:
    DynamicSolver* m_dynamic_solver;
    FcStatusInfo* m_fc_status_info;
    double m_period;
    //一些变量
    unsigned int m_jnt_num;
    LoadInertia m_load;
};

}  // namespace Control
}  // namespace RokaeApi

#endif
