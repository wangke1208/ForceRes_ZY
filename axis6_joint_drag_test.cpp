#include "rokae_header/rokae_force_controller.hpp"
#include <crtdbg.h>

using namespace RokaeApi;
using namespace RokaeApi::External;

int main() {
    _CrtSetDbgFlag(_CRTDBG_ALLOC_MEM_DF | _CRTDBG_LEAK_CHECK_DF);
    SetConsoleOutputCP(CP_UTF8);
    // ---------------------------模型初始化部分-----------------------
    int res = 0;

    // 1.Deinit，防止有内存残留
    RokaeForce_Deinit();

    // 2.建立机器人模型(6轴机器人)
    auto mechUnitType = External_MechUnitType::DEFALUT_SIX_AXIS;
    res = RokaeForce_Init(mechUnitType);
    if (res != 0) {
        LOG_ERROR("机器人初始化失败,错误码为 {}", res);
        return -1;
    } else {
        LOG_INFO("机器人初始化成功!");
    }

    // ---------------------------基础参数初始化部分-----------------------
    // 1.设置编码器零点
    std::vector<int8_t> PDO_0x6061 = {8, 8, 8, 8, 8, 8};
    std::vector<int32_t> encoder_offset = {0, 0, 0, 0, 0, 0};
    res = RokaeForce_SetEncoderOffset(PDO_0x6061, encoder_offset);
    if (res != 0) {
        LOG_ERROR("编码器零点设置失败,错误码为 {}", res);
        return -1;
    } else {
        LOG_INFO("编码器零点设置成功，当前零点值为：{},{},{},{},{},{}", encoder_offset[0], encoder_offset[1], encoder_offset[2],
                 encoder_offset[3], encoder_offset[4], encoder_offset[5]);
    }

    // 2.设置传感器线性度
    PDO_0x6061 = {8, 8, 8, 8, 8, 8};
    std::vector<double> sensor_linearity = {2.25, 2.25, 2.25, 2.25, 2.25, 2.25};
    res = RokaeForce_SetSensorLinearity(PDO_0x6061, sensor_linearity);
    if (res != 0) {
        LOG_ERROR("传感器线性度设置失败,错误码为 {}", res);
        return -1;
    } else {
        LOG_INFO("传感器线性度设置成功，当前线性度值为：{},{},{},{},{},{}", sensor_linearity[0], sensor_linearity[1],
                 sensor_linearity[2], sensor_linearity[3], sensor_linearity[4], sensor_linearity[5]);
    }

    // 3.设置基坐标系&重力矩
    // std::array<double, 6> base_frame = {0.011,-0.032,0.021, 12.0, -44.0, 123.0};
    std::array<double, 6> base_frame = {0,0,0,0,0,0};
    res = RokaeForce_SetBaseFrameAndGravity(base_frame);
    if (res != 0) {
        LOG_ERROR("基坐标系设置失败,错误码为: = {}", res);
        return -1;
    } else {
        LOG_INFO("基坐标系设置成功,当前基坐标系参数: = {},{},{},{},{},{}", base_frame[0], base_frame[1], base_frame[2],
                 base_frame[3], base_frame[4], base_frame[5]);
    }

    // 4.设置负载参数
    PDO_0x6061 = {8, 8, 8, 8, 8, 8};
    External_RokaeLoad load_input;
    //动力学参数
    // load_input.centroid = {0.015,0.03,0.045};
    // load_input.inertia = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
    // load_input.mass = 4.3;

    //坐标系参数
    // load_input.position_offset = {0.034, 0.022, 0.013};
    // load_input.posture_rpy = {55.0, 43.0, 12.0};
    // load_input.position_offset = {0,0,0};
    // load_input.posture_rpy = {0,0,0};
    res = RokaeForce_SetFcLoad(PDO_0x6061, load_input);
    if (res != 0) {
        LOG_ERROR("负载参数设置失败,错误码为 {}", res);
        return -1;
    } else {
        LOG_INFO("负载参数设置成功，负载质量为：{}kg", load_input.mass);
    }

    // 4.传感器零点标定
    PDO_0x6061 = {8, 8, 8, 8, 8, 8};
    std::vector<int32_t> PDO_0x6064 = {233, 2184730, -6553690, 31, -4369070, 19};
    //传感器数据
    std::vector<std::array<int16_t, 200>> PDO_0x2401_array(6);
    std::vector<std::array<int16_t, 200>> PDO_0x2402_array(6);
    // 1轴
    std::fill(PDO_0x2401_array[0].begin(), PDO_0x2401_array[0].end(), 2631);
    std::fill(PDO_0x2402_array[0].begin(), PDO_0x2402_array[0].end(), 2499);
    // 2轴
    std::fill(PDO_0x2401_array[1].begin(), PDO_0x2401_array[1].end(), 2208);
    std::fill(PDO_0x2402_array[1].begin(), PDO_0x2402_array[1].end(), 2026);
    // 3轴
    std::fill(PDO_0x2401_array[2].begin(), PDO_0x2401_array[2].end(), 2558);
    std::fill(PDO_0x2402_array[2].begin(), PDO_0x2402_array[2].end(), 2317);
    // 4轴
    std::fill(PDO_0x2401_array[3].begin(), PDO_0x2401_array[3].end(), 2441);
    std::fill(PDO_0x2402_array[3].begin(), PDO_0x2402_array[3].end(), 2365);
    // 5轴
    std::fill(PDO_0x2401_array[4].begin(), PDO_0x2401_array[4].end(), 2768);
    std::fill(PDO_0x2402_array[4].begin(), PDO_0x2402_array[4].end(), 2340);
    // 6轴
    std::fill(PDO_0x2401_array[5].begin(), PDO_0x2401_array[5].end(), 2593);
    std::fill(PDO_0x2402_array[5].begin(), PDO_0x2402_array[5].end(), 2579);

    std::vector<double> sensor_bias(6);
    res = RokaeForce_CalibrateTrqSensor(PDO_0x6061, PDO_0x6064, load_input, PDO_0x2401_array, PDO_0x2402_array, sensor_bias);
    if (res != 0) {
        LOG_ERROR("传感器零点标定失败,错误码为 {}", res);
        return -1;
    } else {
        LOG_INFO("传感器零点标定成功，当前传感器零点标定值为：{},{},{},{},{},{}", sensor_bias[0], sensor_bias[1], sensor_bias[2],
                 sensor_bias[3], sensor_bias[4], sensor_bias[5]);
    }

    // 5.传感器零点设置
    PDO_0x6061 = {8, 8, 8, 8, 8, 8};
    sensor_bias = {2564.2,2626.646180867353,2060.97530135018,2563.497266136032,2553.609376772012,2582.710005244681};
    res = RokaeForce_SetSensorBias(PDO_0x6061, sensor_bias);
    if (res != 0) {
        LOG_ERROR("传感器零点设置失败,错误码为 {}", res);
        return -1;
    } else {
        LOG_INFO("传感器零点设置成功，当前传感器零点值为：{},{},{},{},{},{}", sensor_bias[0], sensor_bias[1], sensor_bias[2],
                 sensor_bias[3], sensor_bias[4], sensor_bias[5]);
    }

    // 6.设置力控软限位
    PDO_0x6061 = {8, 8, 8, 8, 8, 8};
    std::vector<double> soft_limit_low = {-360, -360, -360, -360, -360, -360};
    std::vector<double> soft_limit_high = {360, 360, 360, 360, 360, 360};

    res = RokaeForce_SetSoftLimit(PDO_0x6061, soft_limit_low, soft_limit_high);
    if (res != 0) {
        LOG_ERROR("力控软限位设置失败,错误码为 {}", res);
        return -1;
    } else {
        LOG_INFO("力控软限位设置成功，当前软限位下限为：{},{},{},{},{},{}", soft_limit_low[0], soft_limit_low[1],
                 soft_limit_low[2], soft_limit_low[3], soft_limit_low[4], soft_limit_low[5]);
        LOG_INFO("力控软限位设置成功，当前软限位上限为：{},{},{},{},{},{}", soft_limit_high[0], soft_limit_high[1],
                 soft_limit_high[2], soft_limit_high[3], soft_limit_high[4], soft_limit_high[5]);
    }

    // 7.设置力控增益接口(可选)
    PDO_0x6061 = {8, 8, 8, 8, 8, 8};
    std::vector<double> kp_gain_set = {1.0, 1.0, 1.0, 1.0, 1.0, 1.0};
    res = RokaeForce_SetKpGain(PDO_0x6061, kp_gain_set);
    if (res != 0) {
        LOG_ERROR("力控增益设置失败,错误码为 {}", res);
        return -1;
    } else {
        LOG_INFO("力控增益设置成功，当前kp增益值为：{},{},{},{},{},{}", kp_gain_set[0], kp_gain_set[1], kp_gain_set[2],
                 kp_gain_set[3], kp_gain_set[4], kp_gain_set[5]);
    }

    // 8.设置摩擦力增益接口(可选)
    std::vector<double> fric_gain_set = {0.5, 0.5, 0.5, 0.5, 0.5, 0.5};

    res = RokaeForce_SetFricGain(PDO_0x6061, fric_gain_set);
    if (res != 0) {
        LOG_ERROR("摩擦力增益设置失败,错误码为 {}", res);
        return -1;
    } else {
        LOG_INFO("摩擦力增益设置成功，当前摩擦力增益值为：{},{},{},{},{},{}", fric_gain_set[0], fric_gain_set[1],
                 fric_gain_set[2], fric_gain_set[3], fric_gain_set[4], fric_gain_set[5]);
    }

    // ================== 单独计算接口测试 ==================
    // 1.获取关节位置

    PDO_0x6064[1] = 3850040;
    std::vector<double> jnt_pos_rad(6);
    RokaeForce_GetAxisPos(PDO_0x6064, jnt_pos_rad);
    SPD_CONTAINER("关节位置", jnt_pos_rad);

    // 2.获取关节速度
    std::vector<int32_t> PDO_0x606C = {-253, 0, 0, 0, 0, 0};
    std::vector<double> jnt_vel_rad(6);
    RokaeForce_GetAxisVel(PDO_0x606C, jnt_vel_rad);
    SPD_CONTAINER("关节速度", jnt_vel_rad);

    // 3.获取关节力矩
    std::vector<int16_t> PDO_0x2401 = {2612, 2199, 2500, 2500, 2500, 2500};
    std::vector<int16_t> PDO_0x2402 = {2537, 1999, 2500, 2500, 2500, 2500};
    std::vector<double> jnt_trq_feedback(6);
    RokaeForce_GetCobotTrq(PDO_0x2401, PDO_0x2402, jnt_trq_feedback);
    SPD_CONTAINER("关节扭矩", jnt_trq_feedback);

    //4.获取TCP位置
    jnt_pos_rad = {0,0.523599,-1.5708,0,-1.0472,0};
    std::array<double, 6> tcp_pos;
    RokaeForce_GetTcpPos(load_input, jnt_pos_rad,tcp_pos);
    SPD_CONTAINER("TCP位置为:", tcp_pos);

    //5.动力学部分
    std::vector<double> trq_temp(6);
    jnt_pos_rad = {-0.264569, 0.310776, -0.964682, -0.520078, -1.1335, -1.5777};
    // jnt_pos_rad = {0, 0.310776, 0, 0, 0, 0};

    jnt_vel_rad = {0.111505,-0.130979,0.406573,0.219191,0.477721,0.664934};
    std::vector<double> jnt_acc_rad = {-0.03066,0.0360146,-0.111793,-0.0602698,-0.131356,0.182833};
    // jnt_pos_rad = {0, -0.102647, 0, 0, 0, 0};
    // jnt_vel_rad = {0.0,-0.235562, 0, 0, 0, 0};
    // std::vector<double> jnt_acc_rad = {0.0,0.261003, 0, 0, 0, 0};
    
    RokaeForce_GetGraTorque(load_input, jnt_pos_rad, trq_temp);
    SPD_CONTAINER("重力矩为:", trq_temp);
    RokaeForce_GetInertTorque(load_input, jnt_pos_rad, jnt_acc_rad, trq_temp);
    SPD_CONTAINER("惯性力矩:", trq_temp);
    RokaeForce_GetCoriolisTorque(load_input, jnt_pos_rad,jnt_vel_rad,trq_temp);
    SPD_CONTAINER("科氏力矩:", trq_temp);
    RokaeForce_GetTotalTorque(load_input, jnt_pos_rad, jnt_vel_rad, jnt_acc_rad, trq_temp);
    SPD_CONTAINER("全力矩:", trq_temp);

    //6.雅可比矩阵
    Eigen::Matrix<double, 6, Eigen::Dynamic> jacobian;
    RokaeForce_GetTcpJacobian(load_input, jnt_pos_rad, jacobian);
    SPD_EIGEN_MATRIX(jacobian);

    //7.质量阵
    Eigen::MatrixXd mass_matrix(6,6);
    RokaeForce_GetMassMatrix(load_input, jnt_pos_rad, mass_matrix);
    SPD_EIGEN_MATRIX(mass_matrix);



    // ---------------------------力控算法部分-----------------------
    // 1.设置力控模式，配置力控内部参数
    PDO_0x6064 = {0, 2184530, -6553600, 0, -4369070, 0};
    PDO_0x6061 = {8, 8, 8, 8, 8, 8};
    PDO_0x2401 = {2650, 2213, 2540, 2368, 2718, 2575};
    PDO_0x2402 = {2492, 2011, 2284, 2429, 2291, 2583};
    External_DragType drag_type = External_DragType::DRAG_JOINT;

    res = RokaeForce_DragConfig(PDO_0x6064, PDO_0x6061, PDO_0x2401, PDO_0x2402, drag_type);
    if (res != 0) {
        LOG_ERROR("力控配置出错,错误码为 {}", res);
        return -1;
    } else {
        LOG_INFO("力控配置成功");
    }

    // 2.力控指令更新接口(每周期调用)
    PDO_0x6061 = {10, 10, 10, 10, 10, 10};
    PDO_0x2401 = {2650, 2213, 2540, 2368, 2718, 2575};
    PDO_0x2402 = {2492, 2011, 2284, 2429, 2291, 2583};
    std::vector<int16_t> PDO_0x2406 = {4000, 3500, 3000, 2500, 2000, 1500};
    PDO_0x6064 = {0, 2184530, -6553600, 0, -4369070, 0};
    PDO_0x606C = {0, 0, 0, 0, 0, 0};
    //输出参数
    std::vector<int16_t> PDO_0x6071(7);
    std::vector<int16_t> PDO_0x60B2(7);
    std::vector<int16_t> PDO_0x2201(7);
    std::vector<int16_t> PDO_0x2202(7);
    std::vector<int16_t> PDO_0x2203(7);
    std::vector<int16_t> PDO_0x2204(7);
    std::vector<int16_t> PDO_0x2205(7);
    std::vector<int16_t> PDO_0x2206(7);
    std::vector<double> jnt_pos_cmd_from_user(6, 0.0);
    std::array<double, 6> cart_pos_cmd_from_user = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
    res = RokaeForce_FcUpdate(PDO_0x6061, PDO_0x2401, PDO_0x2402, PDO_0x2406, PDO_0x6064, PDO_0x606C, jnt_pos_cmd_from_user,
                              cart_pos_cmd_from_user, PDO_0x6071, PDO_0x60B2, PDO_0x2201, PDO_0x2202, PDO_0x2203, PDO_0x2204,
                              PDO_0x2205, PDO_0x2206);
    if (res != 0) {
        LOG_ERROR("力控指令更新出错,不允许下发给伺服，错误码为 {}", res);
        return -1;
    } else {
        LOG_INFO("力控指令更新成功，允许下发伺服");
        LOG_INFO("PDO_0x6071 = {},{},{},{},{},{},{}", PDO_0x6071[0], PDO_0x6071[1], PDO_0x6071[2], PDO_0x6071[3], PDO_0x6071[4],
                 PDO_0x6071[5], PDO_0x6071[6]);
        LOG_INFO("PDO_0x60B2 = {},{},{},{},{},{},{}", PDO_0x60B2[0], PDO_0x60B2[1], PDO_0x60B2[2], PDO_0x60B2[3], PDO_0x60B2[4],
                 PDO_0x60B2[5], PDO_0x60B2[6]);
        LOG_INFO("PDO_0x2201 = {},{},{},{},{},{},{}", PDO_0x2201[0], PDO_0x2201[1], PDO_0x2201[2], PDO_0x2201[3], PDO_0x2201[4],
                 PDO_0x2201[5], PDO_0x2201[6]);
        LOG_INFO("PDO_0x2202 = {},{},{},{},{},{},{}", PDO_0x2202[0], PDO_0x2202[1], PDO_0x2202[2], PDO_0x2202[3], PDO_0x2202[4],
                 PDO_0x2202[5], PDO_0x2202[6]);
        LOG_INFO("PDO_0x2203 = {},{},{},{},{},{},{}", PDO_0x2203[0], PDO_0x2203[1], PDO_0x2203[2], PDO_0x2203[3], PDO_0x2203[4],
                 PDO_0x2203[5], PDO_0x2203[6]);
        LOG_INFO("PDO_0x2204 = {},{},{},{},{},{},{}", PDO_0x2204[0], PDO_0x2204[1], PDO_0x2204[2], PDO_0x2204[3], PDO_0x2204[4],
                 PDO_0x2204[5], PDO_0x2204[6]);
        LOG_INFO("PDO_0x2205 = {},{},{},{},{},{},{}", PDO_0x2205[0], PDO_0x2205[1], PDO_0x2205[2], PDO_0x2205[3], PDO_0x2205[4],
                 PDO_0x2205[5], PDO_0x2205[6]);
        LOG_INFO("PDO_0x2206 = {},{},{},{},{},{},{}", PDO_0x2206[0], PDO_0x2206[1], PDO_0x2206[2], PDO_0x2206[3], PDO_0x2206[4],
                 PDO_0x2206[5], PDO_0x2206[6]);
    }

    // 3. 力控内部状态获取
    std::vector<double> current_jnt_pos(6);
    RokaeForce_GetAxisPosCurrent(current_jnt_pos);
    LOG_INFO("获取当前关节位置为：{},{},{},{},{},{}", current_jnt_pos[0], current_jnt_pos[1], current_jnt_pos[2],
             current_jnt_pos[3], current_jnt_pos[4], current_jnt_pos[5]);

    std::vector<double> current_jnt_vel(6);
    RokaeForce_GetAxisVelCurrent(current_jnt_vel);
    LOG_INFO("获取当前关节速度为：{},{},{},{},{},{}", current_jnt_vel[0], current_jnt_vel[1], current_jnt_vel[2],
             current_jnt_vel[3], current_jnt_vel[4], current_jnt_vel[5]);

    std::vector<double> current_jnt_trq(6);
    RokaeForce_GetCobotTrqCurrent(current_jnt_trq);
    LOG_INFO("获取当前关节力矩为：{},{},{},{},{},{}", current_jnt_trq[0], current_jnt_trq[1], current_jnt_trq[2],
             current_jnt_trq[3], current_jnt_trq[4], current_jnt_trq[5]);

    std::array<double, 6> ext_wrench;
    RokaeForce_GetTcpWrenchCurrent(ext_wrench);
    LOG_INFO("获取当前TCP wrench为: {},{},{},{},{},{}", ext_wrench[0], ext_wrench[1], ext_wrench[2], ext_wrench[3], ext_wrench[4],
             ext_wrench[5]);

    RokaeForce_GetTcpPosCurrent(tcp_pos);
    LOG_INFO("获取当前TCP位置为: {},{},{},{},{},{}", tcp_pos[0], tcp_pos[1], tcp_pos[2], tcp_pos[3], tcp_pos[4], tcp_pos[5]);

    std::vector<double> trq_gravity(6);
    std::vector<double> trq_coriolis(6);
    //Eigen::MatrixXd mass_matrix(7, 7);
    RokaeForce_GetDynamicTorqueCurrent(trq_gravity, trq_coriolis, mass_matrix);
    LOG_INFO("获取当前重力力矩为: {},{},{},{},{},{}", trq_gravity[0], trq_gravity[1], trq_gravity[2], trq_gravity[3],
             trq_gravity[4], trq_gravity[5]);
    LOG_INFO("获取当前科里奥利力矩为: {},{},{},{},{},{}", trq_coriolis[0], trq_coriolis[1], trq_coriolis[2], trq_coriolis[3],
             trq_coriolis[4], trq_coriolis[5]);
    LOG_INFO("获取当前质量矩阵为: {},{},{},{},{},{}", mass_matrix(0, 0), mass_matrix(1, 1), mass_matrix(2, 2),
             mass_matrix(3, 3), mass_matrix(4, 4), mass_matrix(5, 5));

    //Eigen::Matrix<double, 6, Eigen::Dynamic> jacobian(6, 7);
    RokaeForce_GetJacobianCurrent(jacobian);

    PDO_0x6061 = {8, 8, 8, 8, 8, 8};
    RokaeForce_FcStop(PDO_0x6061);

    RokaeForce_Deinit();

    return 0;
}