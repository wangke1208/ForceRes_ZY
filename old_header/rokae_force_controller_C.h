#ifndef ROKAE_FORCE_CONTROLLER_C_H
#define ROKAE_FORCE_CONTROLLER_C_H
#include <stdint.h>


#ifdef __cplusplus
extern "C" {
namespace RokaeApi {

struct Vector3D_C {
    double m_x, m_y, m_z;
};
struct LoadInertia_C {
    double m;           //质量
    Vector3D_C m_cog;   // 质⼼
    double inertia[6];  // ⼆阶矩阵,ix,iy,iz,ixy,ixz,iyz
};
enum MechUnitType_C {
    SR3_C,
    SR4_C,  // XMS4-R800-B4G1A4C
    SR5_C,
};
enum DragType_C {
    DRAG_JOINT,       //轴空间拖动（⽬前只⽀持这⼀种）
    DRAG_CART_TRANS,  //笛卡尔空间仅平移
    DRAG_CART_ROT,    //笛卡尔空间仅旋转
    DRAG_CART_FREE    //笛卡尔⾃由
};

/**
 * @brief 初始化模块，用来初始化机器人模型以及一些参数
 *
 * @param[in] robot_type: 机器人类型(当前机器型号，默认为SR4-C)
 *
 * @return 0:表示初始化成功；
 *         其他:初始化失败，返回相应错误码
 */
int Rokae_Force_Init(const MechUnitType_C robot_type);

/**
 * @brief 拖动配置接口，负责判断一下当前伺服模式，同时判断当前传感器状态是否允许开启拖动
 *
 * @param[in] param_0x6061:PDO参数0x6061
 * @param[in] param_0x2401:PDO参数0x2401
 * @param[in] param_0x2402:PDO参数0x2402
 * @param[in] param_0x6064:PDO参数0x6064
 * @param[in] param_0x606C:PDO参数0x606C
 * @param[in] param_0x2406:PDO参数0x2406
 * @param[in] load_params:负载信息
 * @param[in] drag_type:拖动模式，目前只支持轴空间拖动
 *
 * @param[out] trq_1:力矩值1
 * @param[out] trq_2:力矩值2
 * @param[out] trq_error:力矩误差
 * 
 * @return 返回0表示配置成功，不成功则返回错误码
 */
int Rokae_Force_StartDrag(const int8_t param_0x6061[6], const int16_t param_0x2401[6], const int16_t param_0x2402[6], const int32_t param_0x6064[6],
                          const int32_t param_0x606C[6], const int16_t param_0x2406[6], const LoadInertia_C* load_params, DragType_C drag_type,
                          double trq_1[6], double trq_2[6], double trq_error[6]);

/**
 * @brief 拖动力矩指令计算接口
 *
 * @param[in] param_0x6064:PDO 0x6064
 * @param[in] param_0x606C:PDO 0x606C
 * @param[in] param_0x2406:PDO 0x2406
 * @param[in] param_0x6061:PDO 0x6061
 *
 * @param[out] param_0x6071:PDO 0x6071
 * @param[out] param_0x60B2:PDO 0x60B2
 * @param[out] param_0x2201:PDO 0x2201
 * @param[out] param_0x2202:PDO 0x2202
 * @param[out] param_0x2203:PDO 0x2203
 * @param[out] param_0x2204:PDO 0x2204
 * @param[out] param_0x2205:PDO 0x2205
 * @param[out] param_0x2206:PDO 0x2206
 *
 * @return 返回0表示计算成功，不成功则返回错误码
 */
int Rokae_Force_Calculate(const int32_t param_0x6064[6], const int32_t param_0x606C[6], const int16_t param_0x2406[6], const int8_t param_0x6061[6],
                          int16_t param_0x6071[6], int16_t param_0x60B2[6], int16_t param_0x2201[6], int16_t param_0x2202[6], int16_t param_0x2203[6],
                          int16_t param_0x2204[6], int16_t param_0x2205[6], int16_t param_0x2206[6], double jnt_vel_rad[6]);

/**
 * @brief 关闭拖动接口
 *
 * @param[in] param_0x6061:PDO 0x6061
 *
 * @return 关闭成功返回0，失败返回错误码
 */
int Rokae_Force_StopDrag(const int8_t param_0x6061[6]);

/**
 * @brief Deinit接口，清理相关指针
 *
 * @return 成功调用返回0
 */
int Rokae_Force_Deinit();

/**
 * @brief 传感器零点标定接口
 * @param[in] param_0x6064: PDO 0x6064
 * @param[in] load_params_in: 负载参数
 * @param[in] param_0x2401_array: PDO 0x2401采集200次组成2维数组
 * @param[in] param_0x2402_array: PDO 0x2402采集200次组成2维数组
 * 
 * @param[out] sensor_bias: 传感器零点
 *
 * @param return：0:零点标定成功
 *                
 */
int Rokae_CalibrateTrqSensor(const int32_t param_0x6064[6], const LoadInertia_C* load_params, const int16_t param_0x2401_array[6][200], const int16_t param_0x2402_array[6][200], double sensor_bias[6]);

/**
 * @brief 单轴传感器零点标定接口
 * @param[in] param_0x6064: 位置编码器值
 * @param[in] load_params: 负载参数
 * @param[in] param_0x2401_array_axis: param_0x2401 标定轴的传感器通道1采集200次组成1维数组
 * @param[in] param_0x2402_array_axis: param_0x2402 标定轴的传感器通道2采集200次组成1维数组
 * @param[in] axis_num: 标定的轴号
 * 
 * @param[out] sensor_bias: 传感器零点
 *
 * @param return：0:零点标定成功
 *
 */
int Rokae_CalibrateTrqSensorAxis(const int32_t param_0x6064[6], const LoadInertia_C* load_params, const int16_t param_0x2401_array_axis[200],
                                 const int16_t param_0x2402_array_axis[200], const unsigned int axis_num, double sensor_bias[6]);

/**
 * @brief 设置传感器线性度接口
 * @param[in] analog2trq_low: 传感器线性度
 *
 * @param return：0:设置成功；
 *                其他:设置失败，返回错误码
 */
int Rokae_SetSensorLinearity(const double analog2trq_low[6]);

/**
 * @brief 设置传感器偏置接口
 * @param[in] analog_bias: 传感器偏置
 *
 * @param return：0:设置成功；
 *                其他:设置失败，返回错误码
 */
int Rokae_SetSensorBias(const double analog_bias[6]);

/**
 * @brief 设置位置编码器偏置接口
 * @param[in] encoder_offset: 位置编码器偏置
 *
 * @param return：0:设置成功；
 *                其他:设置失败，返回错误码
 */
int Rokae_SetEncoderOffset(const int32_t encoder_offset[6]);

/**
 * @brief 设置力控系数，范围[0~1]
 * @param[in]  kp_set: 增益补偿系数
 * @param[in] fric_set: 摩擦力补偿系数
 *
 * @param return：0:设置成功；
 *                其他:设置失败，返回错误码
 */
int Rokae_SetControlGain(const double kp_set[6], const double fric_set[6]);

/**
 * @brief 设置软限位(输入的是角度)
 * @param[in] joint_range_min_input: 软限位下限
 * @param[in] joint_range_max_input: 软限位上限
 *
 * @param return：0:设置成功；
 *                其他:设置失败，返回错误码
 */
int Rokae_SetSoftLimit(const double joint_range_min_input[6], const double joint_range_max_input[6]);

/**
 * @brief 获取当前版本号
 *
 * @param return:版本号
 *
 */
const char* Rokae_GetVersion();

}  // namespace RokaeApi
}
#endif
#endif
