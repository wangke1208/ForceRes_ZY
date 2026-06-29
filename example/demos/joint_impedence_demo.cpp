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
    std::vector<int32_t> encoder_offset = {6954651, -20169, 42141, -209178, -33577, 383115, -38823};
    res = RokaeForce_SetEncoderOffset(PDO_0x6061, encoder_offset);
    if (res != 0) {
        LOG_ERROR("编码器零点设置失败,错误码为 {}", res);
        return -1;
    } else {
        LOG_INFO("编码器零点设置成功");
    }

    // 2.设置用户自定义旋转方向（须在传感器零点标定/设置之前；建议编码器零点设置后立即调用）
    PDO_0x6061 = {8, 8, 8, 8, 8, 8, 8};
    std::vector<bool> is_direction_right = {true, true, true, true, true, true, true};
    res = RokaeForce_SetDirectionCoef(PDO_0x6061, is_direction_right);
    if (res != 0) {
        LOG_ERROR("用户自定义旋转方向设置失败,错误码为 {}", res);
        return -1;
    } else {
        LOG_INFO("用户自定义旋转方向设置成功");
    }

    // 3.设置传感器线性度
    PDO_0x6061 = {8, 8, 8, 8, 8, 8, 8};
    std::vector<double> sensor_linearity = {2.111447, 2.025197, 2.241651, -2.163994, 1.668150, 2.269543, -2.287792};
    res = RokaeForce_SetSensorLinearity(PDO_0x6061, sensor_linearity);
    if (res != 0) {
        LOG_ERROR("传感器线性度设置失败,错误码为 {}", res);
        return -1;
    } else {
        LOG_INFO("传感器线性度设置成功");
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

    // 5.传感器零点标定
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

    // 6.传感器零点设置
    PDO_0x6061 = {8, 8, 8, 8, 8, 8, 8};
    res = RokaeForce_SetSensorBias(PDO_0x6061, sensor_bias);
    if (res != 0) {
        LOG_ERROR("传感器零点设置失败,错误码为 {}", res);
        return -1;
    } else {
        LOG_INFO("传感器零点设置成功");
    }

    // 7.设置力控软限位
    PDO_0x6061 = {8, 8, 8, 8, 8, 8, 8};
    std::vector<double> soft_limit_low = {-178, -120, -178, -60, -178, -50, -50};
    std::vector<double> soft_limit_high = {178, 120, 178, 145, 178, 50, 50};

    res = RokaeForce_SetSoftLimit(PDO_0x6061, soft_limit_low, soft_limit_high);
    if (res != 0) {
        LOG_ERROR("力控软限位设置失败,错误码为 {}", res);
        return -1;
    } else {
        LOG_INFO("力控软限位设置成功");
    }

    // 8.设置力控增益接口(阻抗不支持改变增益，调用该接口无效)

    // 9.设置摩擦力增益接口(阻抗不支持改变摩擦力增益，调用该接口无效)

    // 10.设置关节阻抗刚度
    std::vector<double> joint_stiffness = {2000, 2000, 1500, 1500, 500, 500, 500};
    res = RokaeForce_SetJointImpedance(PDO_0x6061,joint_stiffness);
    if (res != 0) {
        LOG_ERROR("关节阻抗刚度设置失败,错误码为 {}", res);
        return -1;
    } else {
        LOG_INFO("关节阻抗刚度设置成功");
    }


    // ---------------------------力控算法部分-----------------------
    // 1.设置力控模式，配置力控内部参数
    PDO_0x6064 = {0, 0, 0, 0, 0, 0, 0};
    PDO_0x6061 = {8, 8, 8, 8, 8, 8, 8};
    std::vector<int16_t> PDO_0x2401 = {2500, 2500, 2500, 2500, 2500, 2500, 2500};
    std::vector<int16_t> PDO_0x2402 = {2500, 2500, 2500, 2500, 2500, 2500, 2500};
    External_DragType drag_type = External_DragType::IMPEDANCE_JOINT;

    bool is_command_by_user = false;   //这里一定要给false，关节阻抗暂时用不到这个，这个是纯由客户去进行力矩指令的计算与下发
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
    double step_time = 0.001;
    double time = 0;
    double angle = 0.0;
    std::vector<double> jnt_pos_init(7);           //弧度
    std::vector<double> jnt_pos_cmd_from_user(7);  //轴空间关节指令
    std::vector<double> jnt_trq_cmd_from_user(7);
    jnt_trq_cmd_from_user = {5, 5, 5, 5, 5, 5, 5};   //随便给一个值就行，不起作用
    std::array<double, 6> cart_cmd_zero{0,0,0,0,0,0}; //笛卡尔空间指令随便给个值即可，不参与计算
    while (time < continue_time) {
        time += step_time;
        if (init) {
            RokaeForce_GetAxisPos(PDO_0x6064, jnt_pos_init);
            init = false;
        }

        jnt_pos_cmd_from_user = jnt_pos_init;  //对于纯关节阻抗不叠加运动的，位置指令永远下发初始值即可，且必须这样下发

        res = RokaeForce_FcUpdate(PDO_0x6061, PDO_0x2401, PDO_0x2402, PDO_0x2406, PDO_0x6064, PDO_0x606C, jnt_pos_cmd_from_user,
                                  cart_cmd_zero, jnt_trq_cmd_from_user, PDO_0x6071, PDO_0x60B2, PDO_0x2201, PDO_0x2202,
                                  PDO_0x2203, PDO_0x2204, PDO_0x2205, PDO_0x2206);
        if (res != 0) {
            LOG_ERROR("力控指令更新出错,不允许下发给伺服，错误码为 {}", res);
            //如果出现异常，返回非0值，请立刻下电切模式，并调用 RokaeForce_FcStop 和 RokaeForce_Deinit
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