/**
 * Copyright(C) 2024 Rokae Technology Co., Ltd.
 * All Rights Reserved.
 *
 * Information in this file is the intellectual property of Rokae Technology Co., Ltd,
 * And may contains trade secrets that must be stored and viewed confidentially.
 *
 * @file: rokae_force_controller_C.cpp
 * @author: wangke
 * @date: 2024/5/15
 * @brief:力控对外接口
 */

#include "../rokae_header/rokae_force_controller_C.h"
#include "../rokae_header/data_structure_convert.hpp"
#include "../rokae_header/data_structure_define.hpp"
#include "../rokae_header/dynamic_solver.hpp"
#include "../rokae_header/fc_status_tracker.hpp"
#include "../rokae_header/forcecontrol.hpp"
#include "../rokae_header/initialize.hpp"
#include "../rokae_header/version.hpp"
namespace RokaeApi {

RokaeApi::InitRobot* RokaeApi_InitSolver;
RokaeApi::Control::ForceControl* RokaeApi_ForceSolver;

int Rokae_Force_Init(const MechUnitType_C robot_type) {
    int res_init = SOLVE_NOERROR;
    //数据类型转换
    MechUnitType m_type;
    if (robot_type == SR3_C) {
        m_type = MechUnitType::SR3_C;
    } else if (robot_type == SR4_C) {
        m_type = MechUnitType::SR4_C;
    } else if (robot_type == SR5_C) {
        m_type = MechUnitType::SR5_C;
    } else {
        res_init = ROBOTTYPE_ERROR;
        return res_init;
    }

    //构建模型
    if (RokaeApi_InitSolver == nullptr) {
        RokaeApi_InitSolver = new RokaeApi::InitRobot(m_type);
    }
    res_init = RokaeApi_InitSolver->CreateModels();
    if (res_init != SOLVE_NOERROR) {
        return res_init;
    }

    //构建力矩模块求解器
    if (RokaeApi_ForceSolver == nullptr) {
        RokaeApi_ForceSolver = new RokaeApi::Control::ForceControl(RokaeApi_InitSolver);
    }
    res_init = RokaeApi_ForceSolver->Fcinit();
    if (res_init != SOLVE_NOERROR) {
        return res_init;
    }
    return res_init;
}

int Rokae_Force_StartDrag(const int8_t param_0x6061[6], const int16_t param_0x2401[6], const int16_t param_0x2402[6], const int32_t param_0x6064[6],
                          const int32_t param_0x606C[6], const int16_t param_0x2406[6], const LoadInertia_C* load_params, DragType_C drag_type,
                          double trq_1[6], double trq_2[6], double trq_error[6]) {
    int res_StartDrag = SOLVE_NOERROR;
    //数据转换
    DragType m_drag_type;
    LoadInertia m_load;
    switch (drag_type) {
    case DRAG_JOINT:
        m_drag_type = DragType::DRAG_JOINT;
        break;
    default:
        return DRAGTYPE_ERROR;
    }
    m_load.m = load_params->m;
    KDL::Vector cog_temp(load_params->m_cog.m_x, load_params->m_cog.m_y, load_params->m_cog.m_z);
    m_load.SetCOG(cog_temp);
    m_load.SetInertia(load_params->inertia[0], load_params->inertia[1], load_params->inertia[2], load_params->inertia[3], load_params->inertia[4],
                      load_params->inertia[5]);

    //接口部分
    res_StartDrag = RokaeApi_ForceSolver->DragConfig(param_0x6061, param_0x2401, param_0x2402, param_0x6064,param_0x606C, param_0x2406, m_load, m_drag_type, trq_1, trq_2, trq_error);
    return res_StartDrag;
}

int Rokae_Force_Calculate(const int32_t param_0x6064[6], const int32_t param_0x606C[6], const int16_t param_0x2406[6], const int8_t param_0x6061[6],
                          int16_t param_0x6071[6], int16_t param_0x60B2[6], int16_t param_0x2201[6], int16_t param_0x2202[6], int16_t param_0x2203[6],
                          int16_t param_0x2204[6], int16_t param_0x2205[6], int16_t param_0x2206[6], double jnt_vel_rad[6]) {
    int res_calculate = SOLVE_NOERROR;
    res_calculate = RokaeApi_ForceSolver->FcUpdate(param_0x6064, param_0x606C, param_0x2406, param_0x6061, param_0x6071, param_0x60B2, param_0x2201,
                                                   param_0x2202, param_0x2203, param_0x2204, param_0x2205, param_0x2206, jnt_vel_rad);

    return res_calculate;
}

int Rokae_Force_StopDrag(const int8_t param_0x6061[6]) {
    int res_StopDrag = SOLVE_NOERROR;
    res_StopDrag = RokaeApi_ForceSolver->StopDrag(param_0x6061);
    return res_StopDrag;
}

int Rokae_Force_Deinit() {
    if (RokaeApi_InitSolver != nullptr) {
        delete RokaeApi_InitSolver;
        RokaeApi_InitSolver = nullptr;
    }
    if (RokaeApi_ForceSolver != nullptr) {
        delete RokaeApi_ForceSolver;
        RokaeApi_ForceSolver = nullptr;
    }
    return SOLVE_NOERROR;
}

int Rokae_CalibrateTrqSensor(const int32_t param_0x6064[6], const LoadInertia_C* load_params, const int16_t param_0x2401_array[6][200], const int16_t param_0x2402_array[6][200], double sensor_bias[6]) {
    int res_CalibrateTrqSensor = SOLVE_NOERROR;

    LoadInertia load_input;
    load_input.m = load_params->m;
    KDL::Vector cog_temp(load_params->m_cog.m_x, load_params->m_cog.m_y, load_params->m_cog.m_z);
    load_input.SetCOG(cog_temp);
    load_input.SetInertia(load_params->inertia[0], load_params->inertia[1], load_params->inertia[2], load_params->inertia[3], load_params->inertia[4],
                      load_params->inertia[5]);

    res_CalibrateTrqSensor = RokaeApi_ForceSolver ->CalibrateTrqSensor(param_0x6064, load_input, param_0x2401_array, param_0x2402_array, sensor_bias);

    return res_CalibrateTrqSensor;
}

int Rokae_CalibrateTrqSensorAxis(const int32_t param_0x6064[6], const LoadInertia_C* load_params, const int16_t param_0x2401_array_axis[200],
                                 const int16_t param_0x2402_array_axis[200], const unsigned int axis_num, double sensor_bias[6]) {
    int res_CalibrateTrqSensorAxis = SOLVE_NOERROR;

    LoadInertia load_input;
    load_input.m = load_params->m;
    KDL::Vector cog_temp(load_params->m_cog.m_x, load_params->m_cog.m_y, load_params->m_cog.m_z);
    load_input.SetCOG(cog_temp);
    load_input.SetInertia(load_params->inertia[0], load_params->inertia[1], load_params->inertia[2], load_params->inertia[3], load_params->inertia[4],
                          load_params->inertia[5]);

    res_CalibrateTrqSensorAxis =
        RokaeApi_ForceSolver->CalibrateTrqSensorAxis(param_0x6064, load_input, param_0x2401_array_axis, param_0x2402_array_axis, axis_num, sensor_bias);

    return res_CalibrateTrqSensorAxis;
}

int Rokae_SetSensorLinearity(const double analog2trq_low[6]) {
    std::vector<double> input(6);
    std::copy(analog2trq_low, analog2trq_low + 6, std::begin(input));  //此处加6不是加5
    return RokaeApi_ForceSolver->SetSensorLinearity(input);
}

int Rokae_SetSensorBias(const double analog_bias[6]) {
    std::vector<double> input(6);
    std::copy(analog_bias, analog_bias + 6, std::begin(input));  //此处加6不是加5
    return RokaeApi_ForceSolver->SetSensorBias(input);
}

int Rokae_SetEncoderOffset(const int32_t encoder_offset[6]) {
    std::vector<int32_t> input(6);
    std::copy(encoder_offset, encoder_offset + 6, std::begin(input));  //此处加6不是加5
    return RokaeApi_ForceSolver->SetEncoderOffset(input);
}

int Rokae_SetControlGain(const double kp_set[6], const double fric_set[6]) {
    std::vector<double> input_kp(6);
    std::vector<double> input_fric(6);

    std::copy(kp_set, kp_set + 6, std::begin(input_kp));  //此处加6不是加5
    std::copy(fric_set, fric_set + 6, std::begin(input_fric));  //此处加6不是加5

    int res1 = RokaeApi_ForceSolver->SetKpGain(input_kp);
    if(res1 != SOLVE_NOERROR){
        return res1;
    }

    int res2 = RokaeApi_ForceSolver->SetFricGain(input_fric);
    if(res2 != SOLVE_NOERROR){
        return res2;
    }
    return SOLVE_NOERROR;

}

int Rokae_SetFricGain(const double fric_set[6]) {
    std::vector<double> input(6);
    std::copy(fric_set, fric_set + 6, std::begin(input));  //此处加6不是加5
    return RokaeApi_ForceSolver->SetFricGain(input);
}

int Rokae_SetSoftLimit(const double joint_range_min_input[6], const double joint_range_max_input[6]){
    std::vector<double> joint_range_min(6);
    std::vector<double> joint_range_max(6);

    std::copy(joint_range_min_input, joint_range_min_input + 6, std::begin(joint_range_min));  //此处加6不是加5
    std::copy(joint_range_max_input, joint_range_max_input + 6, std::begin(joint_range_max));  //此处加6不是加5

    return RokaeApi_ForceSolver->SetSoftLimit(joint_range_min, joint_range_max);
}

const char* Rokae_GetVersion(){
    return VERSION;
}
};  // namespace RokaeApi
