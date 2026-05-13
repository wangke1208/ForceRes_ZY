#include "rokae_force_controller_public.hpp"
#include "client_demo_macros.h"

#ifdef _WIN32
#include <windows.h>
#endif

using namespace RokaeApi;
using namespace RokaeApi::External;
int main() {
#ifdef _WIN32
    SetConsoleOutputCP(CP_UTF8);
#endif
    // ---------------------------模型初始化部分-----------------------
    int res = 0;

    // 1.Deinit，防止有内存残留
    RokaeForce_Deinit();

    // 2.建立机器人模型(7轴机器人)
    auto mechUnitType = External_MechUnitType::AR5_R;
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
        LOG_INFO("编码器零点设置成功");
    }

    // 2.设置传感器线性度
    PDO_0x6061 = {8, 8, 8, 8, 8, 8, 8};
    std::vector<double> sensor_linearity = {2.25, 2.2, 2.35, 2.25, 2.25, 2.25, 2.26};
    res = RokaeForce_SetSensorLinearity(PDO_0x6061, sensor_linearity);
    if (res != 0) {
        LOG_ERROR("传感器线性度设置失败,错误码为 {}", res);
        return -1;
    } else {
        LOG_INFO("传感器线性度设置成功");
    }

    // 3.设置基坐标系&重力矩
    std::array<double, 6> base_frame = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
    res = RokaeForce_SetBaseFrameAndGravity(base_frame);
    if (res != 0) {
        LOG_ERROR("基坐标系设置失败,错误码为: = {}", res);
        return -1;
    } else {
        LOG_INFO("基坐标系设置成功");
    }

    // 4.设置负载参数
    PDO_0x6061 = {8, 8, 8, 8, 8, 8, 8};
    External_RokaeLoad load_input;
    //动力学参数
    load_input.centroid = {0.0, 0.0, 0.0};
    load_input.inertia = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
    load_input.mass = 0;
    //坐标系参数
    load_input.position_offset = {0.0, 0.0, 0.0};
    load_input.posture_rpy = {0.0, 0.0, 0.0};
    res = RokaeForce_SetFcLoad(PDO_0x6061, load_input);
    if (res != 0) {
        LOG_ERROR("负载参数设置失败,错误码为 {}", res);
        return -1;
    } else {
        LOG_INFO("负载参数设置成功");
    }

    // 4.1传感器动态补偿参数设置
    PDO_0x6061 = {8, 8, 8, 8, 8, 8, 8};
    std::vector<double> dynamic_sensor_bias_baseline(7);
    std::vector<double> pos_sensor_fix_params(63);
    std::vector<double> neg_sensor_fix_params(63);
    dynamic_sensor_bias_baseline = {2567, 2586, 2463, 2312, 2694, 2525, 2496};
    pos_sensor_fix_params = {162.807960, 0.053426, -0.234833, -21.077730, 0.657647, 0.121429,  22.092383, 4.016613, 2.160198,
                             30.658550,  0.564124, -1.218808, 5.110712,   1.734754, 1.035753,  23.597628, 4.054341, 2.236284,
                             87.224323,  0.021120, -0.059520, 2.478285,   2.226488, -1.130773, 12.373376, 3.966912, -0.383316,
                             18.936390,  1.707876, -1.213262, 11.643932,  1.949929, 1.593634,  4.628516,  4.105484, 0.062468,
                             14.806343,  0.322175, -1.483700, 2.057463,   2.105045, 0.425788,  4.343226,  4.145373, 0.796019,
                             13.968772,  0.703201, -1.518742, 14.847307,  1.652572, 1.231401,  -1.790922, 3.231630, 1.104126,
                             0.0,        0.0,      0.0,       0.0,        0.0,      0.0,       0.0,       0.0,      0.0};

    //负向参数没更新，暂时用不到
    neg_sensor_fix_params = {61.085351,  0.029700, -0.194760, -7.125775,  0.964782,  0.343541,  21.976220,  3.996282, 2.201233,
                             112.464210, 0.166941, 3.193622,  -25.576201, -0.165867, -0.101843, -22.756813, 3.969568, -7.229926,
                             29.634980,  0.011015, 1.396799,  3.680441,   1.690203,  -1.402636, 12.765942,  3.965537, -0.375712,
                             30.069241,  0.128446, 0.996031,  18.870360,  1.326491,  -0.449290, 3.779639,   4.105744, 0.172469,
                             432.314970, 0.075883, 0.204734,  186.035734, 0.196947,  -2.747579, 3.366651,   4.160138, 0.800932,
                             21.497044,  0.527410, 1.816453,  12.389369,  1.381289,  0.786413,  0.774258,   5.115738, -1.139267,
                             0.0,        0.0,      0.0,       0.0,        0.0,       0.0,       0.0,        0.0,      0.0};
    res = RokaeForce_SetSensorFixParams(PDO_0x6061, dynamic_sensor_bias_baseline, pos_sensor_fix_params, neg_sensor_fix_params);
    if (res != 0) {
        LOG_ERROR("传感器动态补偿参数设置失败,错误码为 {}", res);
        return -1;
    } else {
        LOG_INFO("传感器动态补偿参数设置失败成功");
    }

    // 4.2传感器零点标定
    PDO_0x6061 = {8, 8, 8, 8, 8, 8, 8};
    std::vector<int32_t> PDO_0x6064 = {20000, 20000, 20000, 20000, 20000, 20000, 20000};
    //传感器数据
    std::vector<std::array<int16_t, 200>> PDO_0x2401_array(7);
    std::vector<std::array<int16_t, 200>> PDO_0x2402_array(7);
    for (auto &element : PDO_0x2401_array) {
        std::fill(element.begin(), element.end(), 2500);
    }
    for (auto &element : PDO_0x2402_array) {
        std::fill(element.begin(), element.end(), 2500);
    }
    std::vector<double> sensor_bias(7);
    res = RokaeForce_CalibrateTrqSensor(PDO_0x6061, PDO_0x6064, load_input, PDO_0x2401_array, PDO_0x2402_array, sensor_bias);
    if (res != 0) {
        LOG_ERROR("传感器零点标定失败,错误码为 {}", res);
        return -1;
    } else {
        LOG_INFO("传感器零点标定成功");
    }

    // 5.传感器零点设置
    PDO_0x6061 = {8, 8, 8, 8, 8, 8, 8};
    sensor_bias = {2567, 2590, 2450, 2317, 2710, 2528, 2472};
    res = RokaeForce_SetSensorBias(PDO_0x6061, sensor_bias);
    if (res != 0) {
        LOG_ERROR("传感器零点设置失败,错误码为 {}", res);
        return -1;
    } else {
        LOG_INFO("传感器零点设置成功");
    }

    // 6.设置力控软限位
    PDO_0x6061 = {8, 8, 8, 8, 8, 8, 8};
    std::vector<double> soft_limit_low = {-178, -120, -178, -80, -178, -110, -180};
    std::vector<double> soft_limit_high = {178, 120, 178, 145, 178, 110, 180};

    res = RokaeForce_SetSoftLimit(PDO_0x6061, soft_limit_low, soft_limit_high);
    if (res != 0) {
        LOG_ERROR("力控软限位设置失败,错误码为 {}", res);
        return -1;
    } else {
        LOG_INFO("力控软限位设置成功");
    }

    // 7.设置力控增益接口(阻抗不支持改变增益，调用该接口无效)

    // 8.设置摩擦力增益接口(阻抗不支持改变摩擦力增益，调用该接口无效)

    // 9.设置笛卡尔空间阻抗刚度
    std::array<double,6> cart_stiffness = {3000, 3000, 3000, 300, 300, 300};
    res = RokaeForce_SetCartesianImpedance(PDO_0x6061,cart_stiffness);
    if (res != 0) {
        LOG_ERROR("笛卡尔阻抗刚度设置失败,错误码为 {}", res);
        return -1;
    } else {
        LOG_INFO("笛卡尔阻抗刚度设置成功");
    }


    // ---------------------------力控算法部分-----------------------
    // 1.设置力控模式，配置力控内部参数
    PDO_0x6064 = {0, 0, 0, 0, 0, 0, 0};
    PDO_0x6061 = {8, 8, 8, 8, 8, 8, 8};
    std::vector<int16_t> PDO_0x2401 = {2500, 2500, 2500, 2500, 2500, 2500, 2500};
    std::vector<int16_t> PDO_0x2402 = {2500, 2500, 2500, 2500, 2500, 2500, 2500};
    External_DragType drag_type = External_DragType::IMPEDANCE_CART;
    bool is_command_by_user = false;
    res = RokaeForce_DragConfig(PDO_0x6064, PDO_0x6061, PDO_0x2401, PDO_0x2402, drag_type, is_command_by_user);
    if (res != 0) {
        LOG_ERROR("力控配置出错,错误码为 {}", res);
        return -1;
    } else {
        LOG_INFO("力控配置成功");
    }

    // 2.力控指令更新接口(每周期调用)
    //这些pdo一定是每周期在动态改变的，这里为了方便演示，就不做修改了
    PDO_0x6061 = {10, 10, 10, 10, 10, 10, 10};
    PDO_0x2401 = {2500, 2500, 2500, 2500, 2500, 2500, 2500};
    PDO_0x2402 = {2500, 2500, 2500, 2500, 2500, 2500, 2500};
    std::vector<int16_t> PDO_0x2406 = {4000, 3500, 3000, 2500, 2000, 1500, 1500};
    PDO_0x6064 = {2264206, 2998976, -4816443, 5852990, -630057, 5115646, 827439};
    std::vector<int32_t> PDO_0x606C = {0, 0, 0, 0, 0, 0, 0};
    //输出参数(需要发送给伺服)
    std::vector<int16_t> PDO_0x6071(7);
    std::vector<int16_t> PDO_0x60B2(7);
    std::vector<int16_t> PDO_0x2201(7);
    std::vector<int16_t> PDO_0x2202(7);
    std::vector<int16_t> PDO_0x2203(7);
    std::vector<int16_t> PDO_0x2204(7);
    std::vector<int16_t> PDO_0x2205(7);
    std::vector<int16_t> PDO_0x2206(7);

    bool init = true;
    double continue_time = 5;
    double step_time = 0.5;
    double time = 0;
    double kRadius = 0.2;
    double angle = 0.0;
    double delta_z = 0.0;
    std::vector<double> jnt_pos_init(7);           //弧度
    std::array<double, 16U> cart_frame_init;       //变换矩阵
    std::array<double, 6U> cart_pos_init;          // TCP位姿

    std::vector<double> jnt_pos_cmd_zero(7);  //轴空间关节指令给默认值，不参与计算
    std::array<double, 6> cart_pos_cmd;       //笛卡尔空间指令
    std::vector<double> jnt_trq_cmd_from_user(7);  //力矩指令
    jnt_trq_cmd_from_user = {0, 0, 0, 0, 0, 0, 0};
    while (time < continue_time) {
        time += step_time;
        if (init) {
            RokaeForce_GetAxisPos(PDO_0x6064, jnt_pos_init);  //这里假设反馈值不变，实际上肯定是变的
            RokaeForce_GetTcpPos(load_input, jnt_pos_init, cart_frame_init, cart_pos_init);
            cart_pos_cmd = cart_pos_init;
            init = false;
        }

        angle = FORCE_RES_EXAMPLE_PI / 4 * (1 - std::cos(FORCE_RES_EXAMPLE_PI / 2 * time));
        delta_z = kRadius * (std::cos(angle) - 1);
        cart_pos_cmd[2] = cart_pos_init[2] + delta_z;

        res = RokaeForce_FcUpdate(PDO_0x6061, PDO_0x2401, PDO_0x2402, PDO_0x2406, PDO_0x6064, PDO_0x606C, jnt_pos_cmd_zero,
                                  cart_pos_cmd, jnt_trq_cmd_from_user, PDO_0x6071, PDO_0x60B2, PDO_0x2201, PDO_0x2202, PDO_0x2203,
                                  PDO_0x2204, PDO_0x2205, PDO_0x2206);
        if (res != 0) {
            LOG_ERROR("力控指令更新出错,不允许下发给伺服，错误码为 {}", res);
            return -1;
        } else {
            LOG_INFO("力控指令更新成功");
            LOG_INFO("PDO_0x6071 = {},{},{},{},{},{},{}", PDO_0x6071[0], PDO_0x6071[1], PDO_0x6071[2], PDO_0x6071[3],
                     PDO_0x6071[4], PDO_0x6071[5], PDO_0x6071[6]);
            LOG_INFO("PDO_0x60B2 = {},{},{},{},{},{},{}", PDO_0x60B2[0], PDO_0x60B2[1], PDO_0x60B2[2], PDO_0x60B2[3],
                     PDO_0x60B2[4], PDO_0x60B2[5], PDO_0x60B2[6]);
            LOG_INFO("PDO_0x2201 = {},{},{},{},{},{},{}", PDO_0x2201[0], PDO_0x2201[1], PDO_0x2201[2], PDO_0x2201[3],
                     PDO_0x2201[4], PDO_0x2201[5], PDO_0x2201[6]);
            LOG_INFO("PDO_0x2202 = {},{},{},{},{},{},{}", PDO_0x2202[0], PDO_0x2202[1], PDO_0x2202[2], PDO_0x2202[3],
                     PDO_0x2202[4], PDO_0x2202[5], PDO_0x2202[6]);
            LOG_INFO("PDO_0x2203 = {},{},{},{},{},{},{}", PDO_0x2203[0], PDO_0x2203[1], PDO_0x2203[2], PDO_0x2203[3],
                     PDO_0x2203[4], PDO_0x2203[5], PDO_0x2203[6]);
            LOG_INFO("PDO_0x2204 = {},{},{},{},{},{},{}", PDO_0x2204[0], PDO_0x2204[1], PDO_0x2204[2], PDO_0x2204[3],
                     PDO_0x2204[4], PDO_0x2204[5], PDO_0x2204[6]);
            LOG_INFO("PDO_0x2205 = {},{},{},{},{},{},{}", PDO_0x2205[0], PDO_0x2205[1], PDO_0x2205[2], PDO_0x2205[3],
                     PDO_0x2205[4], PDO_0x2205[5], PDO_0x2205[6]);
            LOG_INFO("PDO_0x2206 = {},{},{},{},{},{},{}", PDO_0x2206[0], PDO_0x2206[1], PDO_0x2206[2], PDO_0x2206[3],
                     PDO_0x2206[4], PDO_0x2206[5], PDO_0x2206[6]);
        }
    }

    // 3.结束阻抗，切换到位置模式

    // 4.内部状态重置
    PDO_0x6061 = {8, 8, 8, 8, 8, 8, 8};
    res = RokaeForce_FcStop(PDO_0x6061);    
    if (res != 0) {
        LOG_ERROR("FcStop出错,错误码为 {}", res);
        return -1;
    }

    // 5.结束任务，Deinit
    RokaeForce_Deinit();

    return 0;
}