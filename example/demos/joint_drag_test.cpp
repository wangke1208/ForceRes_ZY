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
    // 内存泄漏检查
    // _CrtSetDbgFlag(_CRTDBG_ALLOC_MEM_DF | _CRTDBG_LEAK_CHECK_DF);
    SetConsoleOutputCP(CP_UTF8);
#endif

    // ---------------------------模型初始化部分-----------------------
    int res = 0;

    // 1.Deinit，防止有内存残留
    RokaeForce_Deinit();

    // 2.建立机器人模型(左臂)
    std::array<double,3> base_rot = {-90,0.0,0.0}; //基座标系旋转角度
    res = RokaeForce_InitByModelName("AR5-5_0.8L-W4C1C5-ZY2",base_rot);
    if (res != 0) {
        LOG_ERROR("机器人初始化失败,错误码为 {}", res);
        return -1;
    } else {
        LOG_INFO("机器人初始化成功!");
    }

    // ---------------------------基础参数初始化部分-----------------------
    // 1.设置编码器零点
    std::vector<int8_t> PDO_0x6061 = {8, 8, 8, 8, 8, 8, 8};
    std::vector<int32_t> encoder_offset = {33598300, 58215107, 143610739, 74245115, 164768513, 108187571, 160597961};
    res = RokaeForce_SetEncoderOffset(PDO_0x6061, encoder_offset);
    if (res != 0) {
        LOG_ERROR("编码器零点设置失败,错误码为 {}", res);
        return -1;
    } else {
        SPD_CONTAINER("编码器零点设置成功，当前的零点值为: ", encoder_offset);
    }

    // 2.设置传感器线性度
    PDO_0x6061 = {8, 8, 8, 8, 8, 8, 8};
    std::vector<double> sensor_linearity = {2.111447, 2.025197, 2.241651, -2.163994, 1.668150, 2.269543, -2.287792};
    res = RokaeForce_SetSensorLinearity(PDO_0x6061, sensor_linearity);
    if (res != 0) {
        LOG_ERROR("传感器线性度设置失败,错误码为 {}", res);
        return -1;
    } else {
        SPD_CONTAINER("传感器线性度设置成功，当前线性度值为：", sensor_linearity);
    }

    // 4.设置负载参数
    PDO_0x6061 = {8, 8, 8, 8, 8, 8, 8};
    External_RokaeLoad load_input;
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
    PDO_0x6061 = {8, 8, 8, 8, 8, 8, 8};
    std::vector<int32_t> PDO_0x6064 = {33598299, 60836546, 143610735, 69876043, 164768509, 108187583, 160591964};
    //传感器数据
    std::vector<std::array<int16_t, 200>> PDO_0x2401_array(7);
    std::vector<std::array<int16_t, 200>> PDO_0x2402_array(7);
    // 1轴
    std::fill(PDO_0x2401_array[0].begin(), PDO_0x2401_array[0].end(), 2078);
    std::fill(PDO_0x2402_array[0].begin(), PDO_0x2402_array[0].end(), 2078);
    // 2轴
    std::fill(PDO_0x2401_array[1].begin(), PDO_0x2401_array[1].end(), 2358);
    std::fill(PDO_0x2402_array[1].begin(), PDO_0x2402_array[1].end(), 2358);
    // 3轴
    std::fill(PDO_0x2401_array[2].begin(), PDO_0x2401_array[2].end(), 2379);
    std::fill(PDO_0x2402_array[2].begin(), PDO_0x2402_array[2].end(), 2379);
    // 4轴
    std::fill(PDO_0x2401_array[3].begin(), PDO_0x2401_array[3].end(), 2517);
    std::fill(PDO_0x2402_array[3].begin(), PDO_0x2402_array[3].end(), 2517);
    // 5轴
    std::fill(PDO_0x2401_array[4].begin(), PDO_0x2401_array[4].end(), 2462);
    std::fill(PDO_0x2402_array[4].begin(), PDO_0x2402_array[4].end(), 2462);
    // 6轴
    std::fill(PDO_0x2401_array[5].begin(), PDO_0x2401_array[5].end(), 2476);
    std::fill(PDO_0x2402_array[5].begin(), PDO_0x2402_array[5].end(), 2476);
    // 7轴
    std::fill(PDO_0x2401_array[6].begin(), PDO_0x2401_array[6].end(), 2429);
    std::fill(PDO_0x2402_array[6].begin(), PDO_0x2402_array[6].end(), 2429);

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
    res = RokaeForce_SetSensorBias(PDO_0x6061, sensor_bias);
    if (res != 0) {
        LOG_ERROR("传感器零点设置失败,错误码为 {}", res);
        return -1;
    } else {
        SPD_CONTAINER("传感器零点设置成功，当前传感器零点值为: ", sensor_bias);
    }

    // 6.设置力控软限位
    PDO_0x6061 = {8, 8, 8, 8, 8, 8, 8};
    std::vector<double> soft_limit_low = {-178, -120, -178, -60, -178, -50, -50};
    std::vector<double> soft_limit_high = {178, 120, 178, 145, 178, 50, 50};

    res = RokaeForce_SetSoftLimit(PDO_0x6061, soft_limit_low, soft_limit_high);
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

    PDO_0x6064 = {33598299, 60836546, 143610735, 69876043, 164768509, 108187583, 160591964};
    std::vector<double> jnt_pos_rad(7);
    RokaeForce_GetAxisPos(PDO_0x6064, jnt_pos_rad);
    SPD_CONTAINER_JNT_POS_DEG("关节位置", jnt_pos_rad);

    // 2.获取关节速度
    std::vector<int32_t> PDO_0x606C = {1071, -80, -648, -20, -6, 560, 0};
    std::vector<double> jnt_vel_rad(7);
    RokaeForce_GetAxisVel(PDO_0x606C, jnt_vel_rad);
    SPD_CONTAINER("关节速度", jnt_vel_rad);

    // 3.获取关节力矩

    std::vector<int16_t> PDO_0x2401 = {2074, 2360, 2380, 2516, 2461, 2481, 2444};
    std::vector<int16_t> PDO_0x2402 = {2074, 2360, 2380, 2516, 2461, 2481, 2444};
    std::vector<double> jnt_trq_feedback(7);
    RokaeForce_GetCobotTrq(PDO_0x2401, PDO_0x2402, jnt_trq_feedback);
    SPD_CONTAINER("关节扭矩", jnt_trq_feedback);

    // 4获取法兰位置
    std::array<double, 16> flanTobase_pos;
    RokaeForce_GetFlanPos(jnt_pos_rad, flanTobase_pos);
    SPD_CONTAINER("flanTobase_pos为:", flanTobase_pos);


    // 5.动力学部分
    std::vector<double> trq_temp(7);
    jnt_pos_rad = {0.000000199630197, 0.523318383019684,-0.000000718668708,1.047853091488638,-0.000001197781180,0.000469383809724,-0.000000239556236};

    jnt_vel_rad = {-0.000000008452743,-0.022158350460699,0.000000030429875,0.051702763080558,0.000000050716459,0.036930496756152,0.000000010143292};
    std::vector<double> jnt_acc_rad = {-0.000000497657886,-1.304579790970024,0.000001791568390,3.044016294075726,0.000002985947316,2.174294509151885,0.000000597189463};

    RokaeForce_GetGraTorque(load_input, jnt_pos_rad, trq_temp);
    SPD_CONTAINER("重力矩为:", trq_temp);
    RokaeForce_GetInertTorque(load_input, jnt_pos_rad, jnt_acc_rad, trq_temp);
    SPD_CONTAINER("惯性力矩:", trq_temp);
    RokaeForce_GetCoriolisTorque(load_input, jnt_pos_rad,jnt_vel_rad,trq_temp);
    SPD_CONTAINER("科氏力矩:", trq_temp);
    RokaeForce_GetTotalTorque(load_input, jnt_pos_rad, jnt_vel_rad, jnt_acc_rad, trq_temp);
    SPD_CONTAINER("全力矩:", trq_temp);


    // ---------------------------力控算法部分-----------------------
    // 1.设置力控模式，配置力控内部参数
    PDO_0x6064 = {33598299, 60836546, 143610735, 69876043, 164768509, 108187583, 160591964};
    PDO_0x6061 = {8, 8, 8, 8, 8, 8};
    PDO_0x2401 = {2074, 2360, 2380, 2516, 2461, 2481, 2444};
    PDO_0x2402 = {2074, 2360, 2380, 2516, 2461, 2481, 2444};
    External_DragType drag_type = External_DragType::DRAG_JOINT;
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
    PDO_0x2401 = {2044, 2292, 2369, 2450, 2448, 2484, 2409};
    PDO_0x2402 = {2044, 2292, 2369, 2450, 2448, 2484, 2409};
    std::vector<int16_t> PDO_0x2406 = {-55, -7, -56, -34, 83, -47, 54};
    PDO_0x6064 = {35007178, 61295024, 141552186, 69364231, 166472404, 108796144, 160906260};
    PDO_0x606C = {89, 341, 16, -278, 2, -19, 118};
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

    PDO_0x6061 = {8, 8, 8, 8, 8, 8, 8};
    RokaeForce_FcStop(PDO_0x6061);

    RokaeForce_Deinit();

    return 0;
}