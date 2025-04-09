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


 namespace RokaeApi {
 
 RokaeApi::InitRobot* RokaeApi_InitSolver;
 RokaeApi::Control::ForceControl* RokaeApi_ForceSolver;
 
 int Rokae_Force_Init(const MechUnitType robot_type) {
     int res_init = SOLVE_NOERROR;
     //1.判断当前机器人类型是否合理

     //2.构建模型

 
     //3.初始化求解器(力控模块)

 }
 
 int Rokae_Force_Config_Drag(const std::vector<int32_t>& pos_encoder_from_servo, const std::vector<int8_t>& servo_mode_from_servo,
                            const std::vector<int16_t>& analog_ch1, const std::vector<int16_t>& analog_ch2, const DragType& drag_type) {
     int res_StartDrag = SOLVE_NOERROR;
     //数据转换
     DragType m_drag_type;
     LoadInertia m_load;
     switch (drag_type) {
     case DRAG_JOINT:
         m_drag_type = DragType::DRAG_JOINT;
         break;
     default:
         return ERROR_DRAGTYPE;
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
 
 int Rokae_Force_Calculate(const std::vector<int8_t>& servo_mode_from_servo, const std::vector<int16_t>& pdo_analog_ch1,
    const std::vector<int16_t>& pdo_analog_ch2, const std::vector<int16_t>& trq_encoder_from_servo,
    const std::vector<int>& pos_encoder_from_servo, const std::vector<int>& vel_encoder_from_servo,
    std::vector<int16_t>& trq_cmd_to_servo, std::vector<int16_t>& fc_trq_feedforward_to_servo,
    std::vector<int16_t>& fc_kp_to_servo, std::vector<int16_t>& fc_kd_to_servo,
    std::vector<int16_t>& fc_edb_cof_to_servo, std::vector<int16_t>& fc_edb_o_to_servo,
    std::vector<int16_t>& fc_fric_cof_to_servo, std::vector<int16_t>& fc_jnt_inertia_to_servo) {
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

 int Rokae_GetDynamicTrq(const std::vector<double>& jnt_pos, const std::vector<double>& jnt_vel, const std::vector<double>& jnt_acc,
 KDL::JntArray& trq_gravity, KDL::JntArray& trq_coriolis, KDL::JntArray& trq_inertial){

    return 0;
}

int Rokae_GetJntPos(const std::vector<double>& jnt_pos, const std::vector<double>& jnt_vel, const std::vector<double>& jnt_acc,
    KDL::JntArray& trq_gravity, KDL::JntArray& trq_coriolis, KDL::JntArray& trq_inertial){
   
       return 0;
}

 const char* Rokae_GetVersion(){
     return VERSION;
 }
 };  // namespace RokaeApi
 