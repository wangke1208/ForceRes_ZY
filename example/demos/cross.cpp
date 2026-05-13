#include "rokae_force_controller.hpp"
#include "client_demo_macros.h"

#ifdef _WIN32
#include <crtdbg.h>
#include <windows.h>
#endif

using namespace RokaeApi;
using namespace RokaeApi::External;

int main() {
#ifdef _WIN32
    // _CrtSetDbgFlag(_CRTDBG_ALLOC_MEM_DF | _CRTDBG_LEAK_CHECK_DF);
    SetConsoleOutputCP(CP_UTF8);
#endif

    // ---------------------------模型初始化部分-----------------------
    int res = 0;

    // 1.Deinit，防止有内存残留
    RokaeForce_Deinit();

    // 2.建立机器人模型(右臂)
    auto mechUnitType = External_MechUnitType::AR5C_L;
    res = RokaeForce_Init(mechUnitType);
    if (res != 0) {
        LOG_ERROR("机器人初始化失败,错误码为 {}", res);
        return -1;
    } else {
        LOG_INFO("机器人初始化成功!");
    }

    // ---------------------------基础参数初始化部分-----------------------
    // 1.设置编码器零点
    std::vector<int8_t> PDO_0x6061 = {8, 8, 8, 8, 8, 8, 8};
    std::vector<int32_t> encoder_offset = {6954651, -20169, 42141, -209178, -33577, 383115, -38823};
    res = RokaeForce_SetEncoderOffset(PDO_0x6061, encoder_offset);
    if (res != 0) {
        LOG_ERROR("编码器零点设置失败,错误码为 {}", res);
        return -1;
    } else {
        SPD_CONTAINER("编码器零点设置成功，当前的零点值为: ", encoder_offset);
    }

    // 2.设置传感器线性度
    PDO_0x6061 = {8, 8, 8, 8, 8, 8, 8};
    std::vector<double> sensor_linearity = {
        2.25, -1.927968023138513, 2.075085513169817, -2.21382966128082, 1.682116478706906, 1.982771985912998, 1.827657484070191};
    res = RokaeForce_SetSensorLinearity(PDO_0x6061, sensor_linearity);
    if (res != 0) {
        LOG_ERROR("传感器线性度设置失败,错误码为 {}", res);
        return -1;
    } else {
        SPD_CONTAINER("传感器线性度设置成功，当前线性度值为：", sensor_linearity);
    }

    // 3.设置基坐标系&重力矩
    // std::array<double, 6> base_frame = {0.011,-0.032,0.021, 12.0, -44.0, 123.0};
    std::array<double, 6> base_frame = {0,0,0,0,0,0};
    res = RokaeForce_SetBaseFrameAndGravity(base_frame);
    if (res != 0) {
        LOG_ERROR("基坐标系设置失败,错误码为: = {}", res);
        return -1;
    } else {
        SPD_CONTAINER("基坐标系设置成功,当前基坐标系参数: ", base_frame);
    }

    // 4.设置负载参数
    PDO_0x6061 = {8, 8, 8, 8, 8, 8, 8};
    External_RokaeLoad load_input;  //默认无负载

    res = RokaeForce_SetFcLoad(PDO_0x6061, load_input);
    if (res != 0) {
        LOG_ERROR("负载参数设置失败,错误码为 {}", res);
        return -1;
    } else {
        LOG_INFO("负载参数设置成功 - 负载质量: {}", load_input.mass);
        SPD_CONTAINER("负载质心: ", load_input.centroid);
        SPD_CONTAINER("负载惯量: ", load_input.inertia);
        SPD_CONTAINER("负载位置偏移: ", load_input.position_offset);
        SPD_CONTAINER("负载姿态偏移: ", load_input.posture_rpy);
    }

    // 4.2传感器零点标定
    std::vector<int32_t> PDO_0x6064 = {-1, -2, 3, -195406, 2, 204086, -2};
    //传感器数据
    std::vector<std::array<int16_t, 200>> PDO_0x2401_array(7);
    std::vector<std::array<int16_t, 200>> PDO_0x2402_array(7);

    // 1轴
    std::fill(PDO_0x2401_array[0].begin(), PDO_0x2401_array[0].end(), 2653);
    std::fill(PDO_0x2402_array[0].begin(), PDO_0x2402_array[0].end(), 2653);
    // 2轴
    std::fill(PDO_0x2401_array[1].begin(), PDO_0x2401_array[1].end(), 2589);
    std::fill(PDO_0x2402_array[1].begin(), PDO_0x2402_array[1].end(), 2589);
    // 3轴
    std::fill(PDO_0x2401_array[2].begin(), PDO_0x2401_array[2].end(), 2433);
    std::fill(PDO_0x2402_array[2].begin(), PDO_0x2402_array[2].end(), 2433);
    // 4轴
    std::fill(PDO_0x2401_array[3].begin(), PDO_0x2401_array[3].end(), 2331);
    std::fill(PDO_0x2402_array[3].begin(), PDO_0x2402_array[3].end(), 2331);
    // 5轴
    std::fill(PDO_0x2401_array[4].begin(), PDO_0x2401_array[4].end(), 2719);
    std::fill(PDO_0x2402_array[4].begin(), PDO_0x2402_array[4].end(), 2719);
    // 6轴
    std::fill(PDO_0x2401_array[5].begin(), PDO_0x2401_array[5].end(), 2534);
    std::fill(PDO_0x2402_array[5].begin(), PDO_0x2402_array[5].end(), 2534);
    // 7轴
    std::fill(PDO_0x2401_array[6].begin(), PDO_0x2401_array[6].end(), 2541);
    std::fill(PDO_0x2402_array[6].begin(), PDO_0x2402_array[6].end(), 2541);

    std::vector<double> sensor_bias(7);
    res = RokaeForce_CalibrateTrqSensor(PDO_0x6061, PDO_0x6064, load_input, PDO_0x2401_array, PDO_0x2402_array, sensor_bias);
    if (res != 0) {
        LOG_ERROR("传感器零点标定失败,错误码为 {}", res);
        return -1;
    } else {
        SPD_CONTAINER("传感器零点标定成功，当前传感器零点标定值为: ", sensor_bias);
    }

    // 5.传感器零点设置
    PDO_0x6061 = {8, 8, 8, 8, 8, 8, 8};

    // sensor_bias = {2567.51,           2590.161743900649, 2450.625,         2317.805136389301,
    //                2710.955000130881, 2528.288872827914, 2472.054999998878};
    sensor_bias = {2557, 2551, 2455, 2329, 2704, 2531, 2474};
    res = RokaeForce_SetSensorBias(PDO_0x6061, sensor_bias);
    if (res != 0) {
        LOG_ERROR("传感器零点设置失败,错误码为 {}", res);
        return -1;
    } else {
        SPD_CONTAINER("传感器零点设置成功，当前传感器零点值为: ", sensor_bias);
    }

    // 6.设置力控软限位
    PDO_0x6061 = {8, 8, 8, 8, 8, 8, 8};
    std::vector<double> soft_limit_low = {-178, -120, -178, -80, -178, -110, -180};
    std::vector<double> soft_limit_high = {178, 120, 178, 145, 178, 110, 180};

    //res = RokaeForce_SetSoftLimit(PDO_0x6061, soft_limit_low, soft_limit_high);
    if (res != 0) {
        LOG_ERROR("力控软限位设置失败,错误码为 {}", res);
        return -1;
    } else {
        SPD_CONTAINER("力控软限位设置成功，当前软限位上限为：", soft_limit_high);
        SPD_CONTAINER("力控软限位设置成功，当前软限位下限为：", soft_limit_low);
    }

    // 7.设置力控增益接口(可选)
    PDO_0x6061 = {8, 8, 8, 8, 8, 8, 8};
    std::vector<double> kp_gain_set = {1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0};
    res = RokaeForce_SetKpGain(PDO_0x6061, kp_gain_set);
    if (res != 0) {
        LOG_ERROR("力控增益设置失败,错误码为 {}", res);
        return -1;
    } else {
        SPD_CONTAINER("力控增益设置成功，当前kp增益值为：", kp_gain_set);
    }

    // 8.设置摩擦力增益接口(可选)
    std::vector<double> fric_gain_set = {0.5, 0.5, 0.5, 0.5, 0.5, 0.5, 0.5};

    res = RokaeForce_SetFricGain(PDO_0x6061, fric_gain_set);
    if (res != 0) {
        LOG_ERROR("摩擦力增益设置失败,错误码为 {}", res);
        return -1;
    } else {
        SPD_CONTAINER("摩擦力增益设置成功，当前摩擦力增益值为：", fric_gain_set);
    }

    // ================== 单独计算接口测试 ==================
    // 1.获取关节位置

    PDO_0x6064 = {6949648, 4472474, 42722, 4417213, -33679, 5121324, -43168};
    std::vector<double> jnt_pos_rad(7);
    RokaeForce_GetAxisPos(PDO_0x6064, jnt_pos_rad);
    SPD_CONTAINER("关节位置", jnt_pos_rad);

    // 2.获取关节速度
    std::vector<int32_t> PDO_0x606C = {1071, -80, -648, -20, -6, 560, 0};
    std::vector<double> jnt_vel_rad(7);
    RokaeForce_GetAxisVel(PDO_0x606C, jnt_vel_rad);
    SPD_CONTAINER("关节速度", jnt_vel_rad);

    // 3.获取关节力矩
    std::vector<int16_t> PDO_0x2401 = {2624, 2336, 2377, 2216, 2679, 2532, 2537};
    std::vector<int16_t> PDO_0x2402 = {2624, 2336, 2377, 2216, 2679, 2532, 2537};
    std::vector<double> jnt_trq_feedback(7);
    RokaeForce_GetCobotTrq(PDO_0x2401, PDO_0x2402, jnt_trq_feedback);
    SPD_CONTAINER("关节扭矩", jnt_trq_feedback);

    // 4.1获取法兰位置
    double deg2rad = FORCE_RES_EXAMPLE_PI / 180.0;
    jnt_pos_rad = {7.520*deg2rad, 2.843*deg2rad,-19.656*deg2rad, -13.860*deg2rad,
                   30.490*deg2rad, -18.125*deg2rad,  -13.978*deg2rad};

    std::array<double, 16> flanTobase_pos;
    RokaeForce_GetFlanPos(jnt_pos_rad, flanTobase_pos);
    SPD_CONTAINER("flanTobase_pos为:", flanTobase_pos);

    // 4.2获取TCP位置

    std::array<double, 6> tcp_pos;
    std::array<double, 16> toolTobase_pos;
    RokaeForce_GetTcpPos(load_input, jnt_pos_rad, toolTobase_pos, tcp_pos);
    SPD_CONTAINER("toolTobase_pos为:", toolTobase_pos);
    SPD_CONTAINER("TCP位置为:", tcp_pos);

    //4.逆解接口
    //4.1臂角求解
    double cur_psi = 0.0;
    jnt_pos_rad = {7.520*deg2rad, 2.843*deg2rad,-19.656*deg2rad, -13.860*deg2rad,
                   30.490*deg2rad, -18.125*deg2rad,  -13.978*deg2rad};
    int res_psi = RokaeForce_GetCurPsi(jnt_pos_rad, cur_psi);
    if (res_psi == 0) {
        LOG_INFO("求解臂角成功，当前的psi值为: {}", cur_psi);
    } else {
        LOG_ERROR("求解臂角失败,错误码为: {}", res_psi);
    }

    //4.2 逆解接口
    std::array<double, 16> flan_pos_target = flanTobase_pos;
    double psi_tar = cur_psi;
    std::vector<double> q_out(7);
    int res_inverse = RokaeForce_GetJointPos(flan_pos_target, psi_tar, jnt_pos_rad, q_out);
    if (res_inverse != 0) {
        LOG_ERROR("逆解失败,错误码为：{}", res_inverse);
    } else {
        SPD_CONTAINER("逆解成功,逆解结果为 :", q_out);
    }

    // 5.动力学部分
    std::vector<double> trq_temp(7);
    jnt_pos_rad = {1.553482913072114, -2.071310550762818, -0.776741456536057, 0.517827637690704,
                   1.760613968148395, 1.329090936739475,  -2.848052007298875};

    jnt_vel_rad = {0.113490939688671, -0.151321252918228, -0.056745469844335, 0.037830313229557,
                   0.128623064980494, 0.097097803955863,  -0.208066722762563};
    std::vector<double> jnt_acc_rad = {-0.779291293186507, 1.039055057582010,  0.389645646593254, -0.259763764395502,
                                       -0.883196798944708, -0.666726995281790, 1.428700704175263};

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
    jnt_pos_rad = {0.000000000000000, 0.523598775598299, 0.000000000000000, 1.047197551196598,
                   0.000000000000000, 1.570796326794897, 0.000000000000000};
    RokaeForce_GetTcpJacobian(load_input, jnt_pos_rad, jacobian);
    SPD_EIGEN_MATRIX(jacobian);

    //7.质量阵
    Eigen::MatrixXd mass_matrix(6,6);
    RokaeForce_GetMassMatrix(load_input, jnt_pos_rad, mass_matrix);
    SPD_EIGEN_MATRIX(mass_matrix);

    // 8.0.给定负载参数
    External_RokaeLoad load_input_real;
    //动力学参数
    load_input_real.centroid = {0.02, -0.03, 0.025};
    load_input_real.inertia = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
    load_input_real.mass = 3;

    //坐标系参数
    load_input_real.position_offset = {0.02, -0.01, 0.015};
    load_input_real.posture_rpy = {60, 30, -90};

    // 8.1.获取TCP位置(带负载)
    jnt_pos_rad = {-0.000000000000000, 0.355592129828050, 0.000000000000000, 0.711184259656100,
                   0.000000000000000,  1.066776389484150, 0.000000000000000};
    RokaeForce_GetTcpPos(load_input_real, jnt_pos_rad, toolTobase_pos, tcp_pos);
    SPD_CONTAINER("TCP位置为:", tcp_pos);

    // 8.2.动力学部分(带负载)
    jnt_pos_rad = {0.555828337827665, -1.684328296447471, 1.094813392690856, -0.370552225218444,
                   1.802231277198794, 1.515895466802724,  -0.555044567319444};

    jnt_vel_rad = {0.094718929897795, -0.287027060296350, 0.186567589192627, -0.063145953265197,
                   0.307118954517094, 0.258324354266715,  -0.094585367251252};
    jnt_acc_rad = {-0.121886983175194, 0.369354494470286,  -0.240080421405686, 0.081257988783463,
                   -0.395209309083206, -0.332419045023257, 0.121715111004872};

    RokaeForce_GetGraTorque(load_input_real, jnt_pos_rad, trq_temp);
    SPD_CONTAINER("重力矩为:", trq_temp);
    RokaeForce_GetInertTorque(load_input_real, jnt_pos_rad, jnt_acc_rad, trq_temp);
    SPD_CONTAINER("惯性力矩:", trq_temp);
    RokaeForce_GetCoriolisTorque(load_input_real, jnt_pos_rad, jnt_vel_rad, trq_temp);
    SPD_CONTAINER("科氏力矩:", trq_temp);
    RokaeForce_GetTotalTorque(load_input_real, jnt_pos_rad, jnt_vel_rad, jnt_acc_rad, trq_temp);
    SPD_CONTAINER("全力矩:", trq_temp);

    // 8.3.雅可比矩阵(带负载)
    jnt_pos_rad = {0.575958653158129, -1.745329251994330, 1.134464013796314, -0.383972435438752,
                   1.867502299633933, 1.570796326794897,  -0.575146497002030};
    RokaeForce_GetTcpJacobian(load_input_real, jnt_pos_rad, jacobian);
    SPD_EIGEN_MATRIX(jacobian);

    // 8.4.质量阵(带负载)
    RokaeForce_GetMassMatrix(load_input_real, jnt_pos_rad, mass_matrix);
    SPD_EIGEN_MATRIX(mass_matrix);

    // ---------------------------力控算法部分-----------------------
    // 1.设置力控模式，配置力控内部参数
    PDO_0x6064 = {6465032, 1243857,422208, 6773478, 153135, 3241717, -1397951};
    PDO_0x6061 = {8, 8, 8, 8, 8, 8};
    PDO_0x2401 = {2529, 2312, 2443, 1865, 2548, 2581, 2384};
    PDO_0x2402 = {2622, 2224, 2504, 2359, 2817, 2164, 2593};
    External_DragType drag_type = External_DragType::DRAG_CART_ROT;
    bool is_command_by_user = false;  //指令不由用户发送
    res = RokaeForce_DragConfig(PDO_0x6064, PDO_0x6061, PDO_0x2401, PDO_0x2402, drag_type, is_command_by_user);
    if (res != 0) {
        LOG_ERROR("力控配置出错,错误码为 {}", res);
        return -1;
    } else {
        LOG_INFO("力控配置成功");
    }

    // 2.力控指令更新接口(每周期调用)
    PDO_0x6061 = {10, 10, 10, 10, 10, 10, 10};
    PDO_0x2401 = {2529, 2312, 2443, 1865, 2548, 2581, 2384};
    PDO_0x2402 = {2622, 2224, 2504, 2359, 2817, 2164, 2593};
    std::vector<int16_t> PDO_0x2406 = {10, 0, 0, 0, 0, 0, 0};
    PDO_0x6064 = {6465032, 1243857,422208, 6773478, 153135, 3241717, -1397951};
    PDO_0x606C = {0, 186, 1, -344, 4, 527, 0};
    //输出参数
    std::vector<int16_t> PDO_0x6071(7);
    std::vector<int16_t> PDO_0x60B2(7);
    std::vector<int16_t> PDO_0x2201(7);
    std::vector<int16_t> PDO_0x2202(7);
    std::vector<int16_t> PDO_0x2203(7);
    std::vector<int16_t> PDO_0x2204(7);
    std::vector<int16_t> PDO_0x2205(7);
    std::vector<int16_t> PDO_0x2206(7);
    std::vector<double> jnt_pos_cmd_from_user(7, 0.0);
    std::array<double, 6> cart_pos_cmd_from_user = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
    std::vector<double> jnt_trq_cmd_from_user(7, 0.0);
    res = RokaeForce_FcUpdate(PDO_0x6061, PDO_0x2401, PDO_0x2402, PDO_0x2406, PDO_0x6064, PDO_0x606C, jnt_pos_cmd_from_user,
                              cart_pos_cmd_from_user, jnt_trq_cmd_from_user, PDO_0x6071, PDO_0x60B2, PDO_0x2201, PDO_0x2202,
                              PDO_0x2203, PDO_0x2204, PDO_0x2205, PDO_0x2206);
    if (res != 0) {
        LOG_ERROR("力控指令更新出错,不允许下发给伺服，错误码为 {}", res);
        return -1;
    } else {
        LOG_INFO("力控指令更新成功，允许下发伺服");
        SPD_CONTAINER("PDO_0x6071 = ", PDO_0x6071);
        SPD_CONTAINER("PDO_0x60B2 = ", PDO_0x60B2);
        SPD_CONTAINER("PDO_0x2201 = ", PDO_0x2201);
        SPD_CONTAINER("PDO_0x2202 = ", PDO_0x2202);
        SPD_CONTAINER("PDO_0x2203 = ", PDO_0x2203);
        SPD_CONTAINER("PDO_0x2204 = ", PDO_0x2204);
        SPD_CONTAINER("PDO_0x2205 = ", PDO_0x2205);
        SPD_CONTAINER("PDO_0x2206 = ", PDO_0x2206);
    }

    //第二次
    PDO_0x6061 = {10, 10, 10, 10, 10, 10, 10};
    PDO_0x2401 = {2615, 2269, 2469, 2231, 2601, 2587, 2485};
    PDO_0x2402 = {2615, 2269, 2469, 2231, 2601, 2587, 2485};
    PDO_0x2406 = {10, 0, 0, 0, 0, 0, 0};
    PDO_0x6064 = {6533603, 4603417, 51454, 4432620, -35216, 4981731, -373550};
    PDO_0x606C = {-591, 0, 30, -8, -4, 8, -469};
    res = RokaeForce_FcUpdate(PDO_0x6061, PDO_0x2401, PDO_0x2402, PDO_0x2406, PDO_0x6064, PDO_0x606C, jnt_pos_cmd_from_user,
                              cart_pos_cmd_from_user, jnt_trq_cmd_from_user, PDO_0x6071, PDO_0x60B2, PDO_0x2201, PDO_0x2202,
                              PDO_0x2203, PDO_0x2204, PDO_0x2205, PDO_0x2206);
    if (res != 0) {
        LOG_ERROR("力控指令更新出错,不允许下发给伺服，错误码为 {}", res);
        return -1;
    } else {
        LOG_INFO("力控指令更新成功，允许下发伺服");
        SPD_CONTAINER("PDO_0x6071 = ", PDO_0x6071);
        SPD_CONTAINER("PDO_0x60B2 = ", PDO_0x60B2);
        SPD_CONTAINER("PDO_0x2201 = ", PDO_0x2201);
        SPD_CONTAINER("PDO_0x2202 = ", PDO_0x2202);
        SPD_CONTAINER("PDO_0x2203 = ", PDO_0x2203);
        SPD_CONTAINER("PDO_0x2204 = ", PDO_0x2204);
        SPD_CONTAINER("PDO_0x2205 = ", PDO_0x2205);
        SPD_CONTAINER("PDO_0x2206 = ", PDO_0x2206);
    }
    // 3. 力控内部状态获取
    std::vector<double> current_jnt_pos(7);
    RokaeForce_GetAxisPosCurrent(current_jnt_pos);
    SPD_CONTAINER("获取当前关节位置为：", current_jnt_pos);

    std::vector<double> current_jnt_vel(7);
    RokaeForce_GetAxisVelCurrent(current_jnt_vel);
    SPD_CONTAINER("获取当前关节速度为：", current_jnt_vel);

    std::vector<double> current_jnt_trq(7);
    RokaeForce_GetCobotTrqCurrent(current_jnt_trq);
    SPD_CONTAINER("获取当前关节力矩为：", current_jnt_trq);

    std::array<double, 6> ext_wrench;
    RokaeForce_GetTcpWrenchCurrent(ext_wrench);
    SPD_CONTAINER("获取当前TCP wrench为：", ext_wrench);

    RokaeForce_GetTcpPosCurrent(tcp_pos);
    SPD_CONTAINER("获取当前TCP位置为：", tcp_pos);

    std::vector<double> trq_gravity(7);
    std::vector<double> trq_coriolis(7);
    RokaeForce_GetDynamicTorqueCurrent(trq_gravity, trq_coriolis, mass_matrix);
    SPD_CONTAINER("获取当前重力力矩为：", trq_gravity);
    SPD_CONTAINER("获取当前科氏力矩为：", trq_coriolis);
    SPD_EIGEN_MATRIX(mass_matrix);


    RokaeForce_GetJacobianCurrent(jacobian);

    PDO_0x6061 = {8, 8, 8, 8, 8, 8, 8};
    RokaeForce_FcStop(PDO_0x6061);

    // //用户输入力矩指令测试
    // // 1.测试不进行Config是否能下发力矩指令
    // res = RokaeForce_FcUpdate(PDO_0x6061, PDO_0x2401, PDO_0x2402, PDO_0x2406, PDO_0x6064, PDO_0x606C, jnt_pos_cmd_from_user,
    //                           cart_pos_cmd_from_user, jnt_trq_cmd_from_user, PDO_0x6071, PDO_0x60B2, PDO_0x2201, PDO_0x2202,
    //                           PDO_0x2203, PDO_0x2204, PDO_0x2205, PDO_0x2206);
    // LOG_INFO("res = {}", res);

    // // 2.Config
    // drag_type = External_DragType::DRAG_JOINT;
    // is_command_by_user = true;  //指令不由用户发送
    // res = RokaeForce_DragConfig(PDO_0x6064, PDO_0x6061, PDO_0x2401, PDO_0x2402, drag_type, is_command_by_user);

    // // 3. Fcupdate
    // PDO_0x6061 = {10, 10, 10, 10, 10, 10, 10};
    // jnt_trq_cmd_from_user = {0.0, 20.0, 10.0, 5.0, 7.0, 3.0, 1.1};
    // res = RokaeForce_FcUpdate(PDO_0x6061, PDO_0x2401, PDO_0x2402, PDO_0x2406, PDO_0x6064, PDO_0x606C, jnt_pos_cmd_from_user,
    //                           cart_pos_cmd_from_user, jnt_trq_cmd_from_user, PDO_0x6071, PDO_0x60B2, PDO_0x2201, PDO_0x2202,
    //                           PDO_0x2203, PDO_0x2204, PDO_0x2205, PDO_0x2206);
    // if (res != 0) {
    //     LOG_ERROR("力控指令更新出错,不允许下发给伺服，错误码为 {}", res);
    //     return -1;
    // } else {
    //     LOG_INFO("力控指令更新成功，允许下发伺服");
    //     SPD_CONTAINER("PDO_0x6071 = ", PDO_0x6071);
    //     SPD_CONTAINER("PDO_0x60B2 = ", PDO_0x60B2);
    //     SPD_CONTAINER("PDO_0x2201 = ", PDO_0x2201);
    //     SPD_CONTAINER("PDO_0x2202 = ", PDO_0x2202);
    //     SPD_CONTAINER("PDO_0x2203 = ", PDO_0x2203);
    //     SPD_CONTAINER("PDO_0x2204 = ", PDO_0x2204);
    //     SPD_CONTAINER("PDO_0x2205 = ", PDO_0x2205);
    //     SPD_CONTAINER("PDO_0x2206 = ", PDO_0x2206);
    // }
    RokaeForce_Deinit();

    return 0;
}