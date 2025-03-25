/**
 * Copyright(C) 2024 Rokae Technology Co., Ltd.
 * All Rights Reserved.
 *
 * Information in this file is the intellectual property of Rokae Technology Co., Ltd,
 * And may contains trade secrets that must be stored and viewed confidentially.
 *
 * @file: fc_status_tracker.hpp
 * @author: wangke
 * @date: 2024/4/25
 * @brief: 力控数据流计算
 */
#ifndef FC_STATUS_TRACKER_H
#define FC_STATUS_TRACKER_H

#include "data_structure_define.hpp"
#include "dynamic_solver.hpp"
#include "initialize.hpp"
#include "irr_low_pass_filter.hpp"

using namespace RokaeApi::Model;

namespace RokaeApi {
namespace Control {
class FcStatusTracker {
   public:
    FcStatusTracker(InitRobot* init_robot_ptr, FcStatusInfo* fc_status_info);

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
    std::vector<Lowpassfilter*> m_jnt_vel_filter;
    double m_period;
    //一些变量
    unsigned int m_jnt_num;
    LoadInertia m_load;
};

}  // namespace Control
}  // namespace RokaeApi

#endif
