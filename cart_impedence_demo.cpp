#include "rokae_header/rokae_force_controller.hpp"

using namespace RokaeApi;
using namespace RokaeApi::External;
int main() {
    // ---------------------------模型初始化部分-----------------------
    int res = 0;

    // 1.Deinit，防止有内存残留
    RokaeForce_Deinit();

    // 2.建立机器人模型(7轴机器人)
    auto mechUnitType = External_MechUnitType::DEFALUT_SENVEN_AXIS;
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
    std::vector<int32_t> encoder_offset = {0, 0, 0, 0, 0, 0, 0};
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
    load_input.mass = 7;
    //坐标系参数
    load_input.position_offset = {0.0, 0.0, 0.0};
    load_input.posture_rpy = {180.0, 0.0, 0.0};
    res = RokaeForce_SetFcLoad(PDO_0x6061, load_input);
    if (res != 0) {
        LOG_ERROR("负载参数设置失败,错误码为 {}", res);
        return -1;
    } else {
        LOG_INFO("负载参数设置成功");
    }

    // 4.传感器零点标定
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
    res = RokaeForce_SetSensorBias(PDO_0x6061, sensor_bias);
    if (res != 0) {
        LOG_ERROR("传感器零点设置失败,错误码为 {}", res);
        return -1;
    } else {
        LOG_INFO("传感器零点设置成功");
    }

    // 6.设置力控软限位
    PDO_0x6061 = {8, 8, 8, 8, 8, 8, 8};
    std::vector<double> soft_limit_low = {-165, -115, -165, -115, -165, -115, -355};
    std::vector<double> soft_limit_high = {165, 115, 165, 115, 165, 115, 355};

    res = RokaeForce_SetSoftLimit(PDO_0x6061, soft_limit_low, soft_limit_high);
    if (res != 0) {
        LOG_ERROR("力控软限位设置失败,错误码为 {}", res);
        return -1;
    } else {
        LOG_INFO("力控软限位设置成功");
    }

    // 7.设置力控增益接口(可选)
    PDO_0x6061 = {8, 8, 8, 8, 8, 8, 8};
    std::vector<double> kp_gain_set = {1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0};
    res = RokaeForce_SetKpGain(PDO_0x6061, kp_gain_set);
    if (res != 0) {
        LOG_ERROR("力控增益设置失败,错误码为 {}", res);
        return -1;
    } else {
        LOG_INFO("力控增益设置成功");
    }

    // 8.设置摩擦力增益接口(可选)
    std::vector<double> fric_gain_set = {0.5, 0.5, 0.5, 0.5, 0.5, 0.5, 0.5};

    res = RokaeForce_SetFricGain(PDO_0x6061, fric_gain_set);
    if (res != 0) {
        LOG_ERROR("摩擦力增益设置失败,错误码为 {}", res);
        return -1;
    } else {
        LOG_INFO("摩擦力增益设置成功");
    }

    // 9.设置轴空间阻抗刚度
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

    res = RokaeForce_DragConfig(PDO_0x6064, PDO_0x6061, PDO_0x2401, PDO_0x2402, drag_type);
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
    PDO_0x6064 = {0, 0, 0, 0, 0, 0, 0};
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
    double step_time = 0.001;
    double time = 0;
    double kRadius = 0.2;
    double angle = 0.0;
    double delta_z = 0.0;
    std::vector<double> jnt_pos_init(7);           //弧度
    std::array<double,6> cart_pos_init;
    std::vector<double> jnt_pos_cmd_zero(7);  //轴空间关节指令给默认值，不参与计算
    std::array<double,6> cart_pos_cmd; //笛卡尔空间指令

    while (time < continue_time) {
        time += step_time;
        if (init) {
            RokaeForce_GetAxisPos(PDO_0x6064, jnt_pos_init);  //这里假设反馈值不变，实际上肯定是变的
            RokaeForce_GetTcpPos(load_input,jnt_pos_init,cart_pos_init);
            cart_pos_cmd = cart_pos_init;
            init = false;
        }

        angle = KDL::PI / 4 * (1 - std::cos(KDL::PI / 2 * time));
        delta_z = kRadius * (std::cos(angle) - 1);
        cart_pos_cmd[2] = cart_pos_init[2] + delta_z;

        res = RokaeForce_FcUpdate(PDO_0x6061, PDO_0x2401, PDO_0x2402, PDO_0x2406, PDO_0x6064, PDO_0x606C, jnt_pos_cmd_zero,cart_pos_cmd, PDO_0x6071,
                                  PDO_0x60B2, PDO_0x2201, PDO_0x2202, PDO_0x2203, PDO_0x2204, PDO_0x2205, PDO_0x2206);
        if (res != 0) {
            LOG_ERROR("力控指令更新出错,不允许下发给伺服，错误码为 {}", res);
            return -1;
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