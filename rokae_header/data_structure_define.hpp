/**
 * Copyright(C) 2024 Rokae Technology Co., Ltd.
 * All Rights Reserved.
 *
 * Information in this file is the intellectual property of Rokae Technology Co., Ltd,
 * And may contains trade secrets that must be stored and viewed confidentially.
 *
 * @file: data_structure_define.hpp
 * @author: wangke
 * @date:
 * @brief: 和数据结构定义
 */

#ifndef ROKAE_HEADER_DATA_STRUCTURE_DEFINE_H
#define ROKAE_HEADER_DATA_STRUCTURE_DEFINE_H

#include <3rd/kdl/chain.hpp>
#include <3rd/kdl/chaindynparam.hpp>
#include <3rd/kdl/jntarray.hpp>
#include <3rd/kdl/jntspaceinertiamatrix.hpp>
#include <3rd/kdl/joint.hpp>
#include <vector>
#include <array>

#include "rokae_header/fc_params.hpp"

using namespace KDL;

namespace RokaeApi {
template <typename T>
void ResizeVector(std::vector<T>& vec, unsigned int size, T default_value = T()) {
    vec.resize(size, default_value);
}

// 常量定义
constexpr unsigned int DEFAULT_AXIS = 6U;
constexpr unsigned int DEFAULT_SEGMENT = 7U;
constexpr double PI = 3.1415926535;

typedef Eigen::Matrix<double, Eigen::Dynamic, 6> Jacobian_trans;
typedef Eigen::Matrix<double, Eigen::Dynamic, 6> Jacobian_inv;
typedef Eigen::Matrix<double, 6, Eigen::Dynamic> Jacobian_trans_inv;

//精度
const double EPSILON16 = 0.000000000000001;
const double EPSILON15 = 0.00000000000001;
const double EPSILON14 = 0.0000000000001;
const double EPSILON13 = 0.000000000001;
const double EPSILON12 = 0.00000000001;
const double EPSILON11 = 0.0000000001;
const double EPSILON10 = 0.0000000001;
const double EPSILON9 = 0.000000001;
const double EPSILON8 = 0.00000001;
const double EPSILON7 = 0.0000001;
const double EPSILON6 = 0.000001;
const double EPSILON5 = 0.00001;
const double EPSILON4 = 0.0001;
const double EPSILON3 = 0.001;
const double EPSILON2 = 0.01;
const double EPSILON1 = 0.1;

enum SolverRes {
    SOLVE_NOERROR = 0,
    ERROR_FC_FRAME_TYPE = -1,
    ERROR_DRAG_STATUS = -2,
    ERROR_LOAD_LIMIT_PARAMS = -3,
    ERROR_EULER_PARAMS = -4,
    ERROR_SENSOR_LINERALITY_SET = -5,
    ERROR_GAIN_VALUE_SET = -6,
    ERROR_RPY_CAL = -7,
    ERROR_DRAG_START_POS = -8,
    ERROR_SOFT_LIMIT_PARAMS = -9,
    ERROR_LOAD_PARAMS = -10,
    ERROR_SENSOR_BIAS = -11,
    ERROR_EXCESSIVE_TORQUE = -12,
    ERROR_DRAG_ENABLE = -13,
    ERROR_SIZE_WRONG = -14,
    ERROR_ROBOTTYPE = -15,
    ERROR_DRAGTYPE = -16,
    ERROR_SERVO_MODE = -17,
    ERROR_ALREADY_INIT = -18
};

namespace Model {
enum MechUnitType {
    UNKNOWN,
    DEFALUT_SENVEN_AXIS,  // XMS3-R580-W4G3B1C
};

struct ModelParams {
    struct Link_Inertial {
        double mass = 0.0;                                                 // 质量,单位kg
        std::vector<double> centroid = {0.0, 0.0, 0.0};                    // 质心,单位mm
        std::vector<double> moment = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0};       // 总惯量，单位kg.mm^2
        std::vector<double> moment_link = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0};  // 连杆惯量，单位kg.mm^2

        // 重置所有成员为零
        void SetZero() {
            mass = 0.0;
            centroid.assign(3, 0.0);
            moment.assign(6, 0.0);
            moment_link.assign(6, 0.0);
        }
    };

    enum Rot_Axis { ROT_NONE, ROT_X, ROT_Y, ROT_Z };

    struct Coord_Orientation {
        Rot_Axis rot_axis;  // 旋转轴
        double rot_angle;   // 旋转角度
    };

    struct RobDimensions {
        double L01x = 0.0, L01y = 0.0, L01z = 0.0;  // 0,1 轴沿着base轴坐标系x, y, z方向的距离
        double L12x = 0.0, L12y = 0.0, L12z = 0.0;  // 1,2 轴沿着base轴坐标系x, y, z方向的距离
        double L23x = 0.0, L23y = 0.0, L23z = 0.0;  // 2,3 轴沿着base轴坐标系x, y, z方向的距离
        double L34x = 0.0, L34y = 0.0, L34z = 0.0;  // 3,4 轴沿着base轴坐标系x, y, z方向的距离
        double L45x = 0.0, L45y = 0.0, L45z = 0.0;  // 4,5 轴沿着base轴坐标系x, y, z方向的距离
        double L56x = 0.0, L56y = 0.0, L56z = 0.0;  // 5,6 轴沿着base轴坐标系x, y, z方向的距离
        double L67x = 0.0, L67y = 0.0, L67z = 0.0;  // 6,7 轴沿着base轴坐标系x, y, z方向的距离
        double L78x = 0.0, L78y = 0.0, L78z = 0.0;  // 7,8 轴沿着base轴坐标系x, y, z方向的距离

        // 默认构造函数
        RobDimensions() = default;

        // 使用std::vector初始化成员
        RobDimensions(std::vector<double>& rd) {
            if (rd.size() >= 24) {
                L01x = rd[0];
                L01y = rd[1];
                L01z = rd[2];
                L12x = rd[3];
                L12y = rd[4];
                L12z = rd[5];
                L23x = rd[6];
                L23y = rd[7];
                L23z = rd[8];
                L34x = rd[9];
                L34y = rd[10];
                L34z = rd[11];
                L45x = rd[12];
                L45y = rd[13];
                L45z = rd[14];
                L56x = rd[15];
                L56y = rd[16];
                L56z = rd[17];
                L67x = rd[18];
                L67y = rd[19];
                L67z = rd[20];
                L78x = rd[21];
                L78y = rd[22];
                L78z = rd[23];
            }
        }
    };

    unsigned int axis_num;                          // 轴数量
    std::vector<Link_Inertial> link_inertia;        // 连杆惯量
    std::vector<KDL::Joint::JointType> joint_type;  // 关节类型
    std::vector<Coord_Orientation> coor_orient;     // 坐标系
    RobDimensions rob_dimensions;                   // 机器人尺寸
    std::vector<double> joint_range_min;            // 软限位
    std::vector<double> joint_range_max;            // 软限位
    std::vector<double> joint_range_min_new;        // 硬限位
    std::vector<double> joint_range_max_new;        // 硬限位
    double max_load;                                // 最大负载
    double max_load_tcp_length;                     // 最大负载末端执行器长度

    // 构造函数
    ModelParams(unsigned int segments_cnt) { this->Resize(segments_cnt); }

    // Resize方法，调整各成员数组的大小
    void Resize(unsigned int segments_cnt) {
        axis_num = segments_cnt - 1;
        joint_type.resize(segments_cnt);
        coor_orient.resize(segments_cnt);
        link_inertia.resize(segments_cnt);
        joint_range_min.resize(segments_cnt - 1);
        joint_range_max.resize(segments_cnt - 1);
        joint_range_min_new.resize(segments_cnt - 1);
        joint_range_max_new.resize(segments_cnt - 1);
        max_load = 0.0;
        max_load_tcp_length = 0.3;
    }
};

struct MechanicalParams {
    // 使用初始化列表，直接构造
    std::vector<int32_t> encoder_offset;
    std::vector<int> encoder_resolution;
    std::vector<double> decel_ratio_high;
    std::vector<double> decel_ratio_low;
    std::vector<double> analog2trq_high;
    std::vector<double> analog2trq_low;
    std::vector<double> analog_bias;
    std::vector<double> sensor_amplify;
    std::vector<double> rated_torque;

    // 构造函数初始化
    MechanicalParams(unsigned int jnt_num)
        : encoder_offset(jnt_num),
          encoder_resolution(jnt_num),
          decel_ratio_high(jnt_num),
          decel_ratio_low(jnt_num),
          analog2trq_high(jnt_num),
          analog2trq_low(jnt_num),
          analog_bias(jnt_num),
          sensor_amplify(jnt_num),
          rated_torque(jnt_num) {}

    template <typename T>
    void SetMechStatusInfo(T& dest, const T& src) {
        dest = src;
    }

    // 拷贝赋值函数优化
    MechanicalParams& operator=(const MechanicalParams& mec_param) {
        SetMechStatusInfo(encoder_offset, mec_param.encoder_offset);
        SetMechStatusInfo(encoder_resolution, mec_param.encoder_resolution);
        SetMechStatusInfo(decel_ratio_high, mec_param.decel_ratio_high);
        SetMechStatusInfo(decel_ratio_low, mec_param.decel_ratio_low);
        SetMechStatusInfo(analog2trq_high, mec_param.analog2trq_high);
        SetMechStatusInfo(analog2trq_low, mec_param.analog2trq_low);
        SetMechStatusInfo(analog_bias, mec_param.analog_bias);
        SetMechStatusInfo(sensor_amplify, mec_param.sensor_amplify);
        SetMechStatusInfo(rated_torque, mec_param.rated_torque);
        return *this;
    }
};

struct RokaeLoadInertia {
    double mass{0.0};                   // 质量，单位kg
    KDL::Vector m_cog{0.0, 0.0, 0.0};   // 质心，单位m
    KDL::Vector mx{0.0, 0.0, 0.0};      // 一阶矩，单位kg·m
    std::array<double, 6> m_inertia{0.0, 0.0, 0.0, 0.0, 0.0, 0.0};  // 二阶惯性矩阵，单位kg·m²

    // 默认构造
    RokaeLoadInertia() = default;

    // 质量 + 质心构造
    RokaeLoadInertia(double mass, const KDL::Vector& cog, const std::array<double, 6> inertia = {0, 0, 0, 0, 0, 0})
        : mass(mass), m_cog(cog), mx(mass * cog), m_inertia(inertia) {}

    // 设置质量
    void SetMass(double mass_val) noexcept {
        mass = mass_val;
        mx = mass * m_cog;
    }

    // 设置质心
    void SetCOG(const KDL::Vector& cog) noexcept {
        m_cog = cog;
        mx = mass * cog;
    }

    // 设置惯性矩阵
    void SetInertia(double ix, double iy, double iz, double ixy = 0.0, double ixz = 0.0, double iyz = 0.0) noexcept {
        m_inertia = {ix, iy, iz, ixy, ixz, iyz};
    }

    // 清零
    void SetZero() noexcept {
        mass = 0.0;
        m_cog = KDL::Vector::Zero();
        mx = KDL::Vector::Zero();
        m_inertia.fill(0.0);
    }

    // Getters
    const double& GetMass() const noexcept { return mass; }
    const KDL::Vector& GetCOG() const noexcept { return m_cog; }
    const std::array<double, 6>& GetInertia() const noexcept { return m_inertia; }

    // 拷贝赋值
    RokaeLoadInertia& operator=(const RokaeLoadInertia& other) noexcept {
        if (this != &other) {
            mass = other.mass;
            m_cog = other.m_cog;
            mx = other.mx;
            m_inertia = other.m_inertia;
        }
        return *this;
    }
};

struct RokaeLoadPose {
    KDL::Vector spatiapos{0.0, 0.0, 0.0};    // 空间位置
    KDL::Vector eulerangles{0.0, 0.0, 0.0};  // 旋转欧拉角(弧度)

    // 默认构造函数
    RokaeLoadPose() = default;

    // 带参数的构造函数
    RokaeLoadPose(double x, double y, double z, double a, double b, double c) : spatiapos(x, y, z), eulerangles(a, b, c) {}

    // 获取KDL的Frame对象
    KDL::Frame GetKDLFrame() const {
        // 使用欧拉角构造旋转矩阵，并返回一个Frame对象
        KDL::Rotation rot = KDL::Rotation::RPY(eulerangles.x(), eulerangles.y(), eulerangles.z());
        return KDL::Frame(rot, spatiapos);  // 直接返回构造的Frame对象
    }

    // 设置空间位置
    void SetSpatiaPos(const KDL::Vector& pos) { spatiapos = pos; }

    // 设置旋转欧拉角
    void SetRotAngle(const KDL::Vector& rot) { eulerangles = rot; }

    // 重置为零
    void SetZero() {
        spatiapos.Zero();
        eulerangles.Zero();
    }

    // 拷贝赋值
    RokaeLoadPose& operator=(const RokaeLoadPose& load_input) = default;
};

struct RokaeLoad {
    RokaeLoadInertia m_rokae_load_inertia;  // 动力学信息
    RokaeLoadPose m_rokae_load_pose;        // 位姿信息

    // 默认构造函数，直接初始化成员变量
    RokaeLoad() = default;
    RokaeLoad(const RokaeLoadInertia& load_inertia, const RokaeLoadPose& load_pose)
        : m_rokae_load_inertia(load_inertia), m_rokae_load_pose(load_pose) {}

    // 设置零值
    void SetZero() {
        m_rokae_load_inertia.SetZero();
        m_rokae_load_pose.SetZero();
    }

    // 设置动力学信息
    void SetRokaeLoadInertia(const RokaeLoadInertia& load_inertia) { m_rokae_load_inertia = load_inertia; }

    // 设置位姿信息
    void SetRokaeLoadPose(const RokaeLoadPose& load_pose) { m_rokae_load_pose = load_pose; }

    // 获取动力学信息
    const RokaeLoadInertia& GetRokaeLoadInertia() const { return m_rokae_load_inertia; }

    // 获取位姿信息
    const RokaeLoadPose& GetRokaeLoadPose() const { return m_rokae_load_pose; }

    // 拷贝赋值
    RokaeLoad& operator=(const RokaeLoad& load_input) = default;
};

}  // namespace Model
namespace Control {

enum DragType {
    DRAG_JOINT,       //轴空间拖动（目前只支持这一种）
    DRAG_CART_TRANS,  //笛卡尔空间仅平移
    DRAG_CART_ROT,    //笛卡尔空间仅旋转
    DRAG_CART_FREE    //笛卡尔自由
};

enum FcFrameType {
    FCFRAME_WORLD,  //世界坐标系
    FCFRAME_TOOL,   //工具坐标系
    FCFRAME_FLAN,   //法兰坐标系
    FCFRAME_BASE    //基坐标系
};

struct ProtectParams {
    unsigned int m_jnt_num;
    // 轴空间
    std::vector<double> max_joint_stiff;
    std::vector<double> max_joint_damp;
    std::vector<double> max_joint_stiff_trq;
    std::vector<double> max_joint_damp_trq;
    // 保护功能
    std::vector<double> max_mode_switch_trq;
    std::vector<double> soft_joint_limit_stiff;
    std::vector<double> soft_joint_limit_damp;

    // 使用 member initializer list 直接初始化
    ProtectParams(unsigned int jnt_num = 6)
        : m_jnt_num(jnt_num),
          max_joint_stiff(jnt_num, 300.0),
          max_joint_damp(jnt_num, 10.0),
          max_joint_stiff_trq(jnt_num, 60.0),
          max_joint_damp_trq(jnt_num, 20.0),
          max_mode_switch_trq(jnt_num, 30.0),
          soft_joint_limit_stiff(jnt_num, 1000.0),
          soft_joint_limit_damp(jnt_num, 10.0) {}
};

struct GainParams {
    unsigned int m_jnt_num;
    // 轴空间
    std::vector<double> joint_gain_kp;
    std::vector<double> joint_damp_zeta;
    std::vector<double> friction_cof_servo;
    std::vector<double> trans_drag_rot_stiff;
    std::vector<double> trans_drag_rot_damp;
    std::vector<double> rot_drag_trans_stiff;
    std::vector<double> rot_drag_trans_damp;

    GainParams(unsigned int jnt_num = 6)
        : m_jnt_num(jnt_num),
          joint_gain_kp(jnt_num, 1.0),
          joint_damp_zeta(jnt_num, 0.707),
          friction_cof_servo(jnt_num, 0.6),
          trans_drag_rot_stiff(6, 300.0),
          trans_drag_rot_damp(6, 5.0),
          rot_drag_trans_stiff(6, 2000),
          rot_drag_trans_damp(6, 10.0) {}
};

struct ControlParams {
    ProtectParams m_protect_params;
    GainParams m_gain_params;

    ControlParams(unsigned int jnt_num) : m_protect_params(jnt_num), m_gain_params(jnt_num) {}

    // Resize 用于重新初始化
    void Resize(unsigned int jnt_num) {
        m_protect_params = ProtectParams(jnt_num);
        m_gain_params = GainParams(jnt_num);
    }
};

struct Servo_To_FcInner {
    std::vector<int32_t> pos_feedback;
    std::vector<int32_t> vel_feedback;
    std::vector<int16_t> trq_feedback;
    std::vector<int16_t> analog_ch1;
    std::vector<int16_t> analog_ch2;
    std::vector<int8_t> mode_operation;

    Servo_To_FcInner(unsigned int jnt_num) { Resize(jnt_num); }

    void Resize(unsigned int jnt_num) {
        ResizeVector(pos_feedback, jnt_num);
        ResizeVector(vel_feedback, jnt_num);
        ResizeVector(trq_feedback, jnt_num);
        ResizeVector(analog_ch1, jnt_num);
        ResizeVector(analog_ch2, jnt_num);
        ResizeVector(mode_operation, jnt_num);
    }
};

struct FcInner_To_Servo {
    std::vector<int16_t> trq_cmd;
    std::vector<int16_t> trq_feedforward;
    std::vector<int16_t> k_p;
    std::vector<int16_t> k_d;
    std::vector<int16_t> k_p_reset_by_load;
    std::vector<int16_t> k_d_reset_by_load;
    std::vector<int16_t> edb_cof;
    std::vector<int16_t> edb_o;
    std::vector<int16_t> fric_cof;
    std::vector<int16_t> jnt_inertia;

    FcInner_To_Servo(unsigned int jnt_num) { Resize(jnt_num); }

    void Resize(unsigned int jnt_num) {
        ResizeVector(trq_cmd, jnt_num);
        ResizeVector(trq_feedforward, jnt_num);
        ResizeVector(k_p, jnt_num, static_cast<int16_t>(2000));  // Custom default for k_p
        ResizeVector(k_d, jnt_num, static_cast<int16_t>(70));    // Custom default for k_d
        ResizeVector(k_p_reset_by_load, jnt_num, static_cast<int16_t>(2000));
        ResizeVector(k_d_reset_by_load, jnt_num, static_cast<int16_t>(70));
        ResizeVector(edb_cof, jnt_num, static_cast<int16_t>(100));
        ResizeVector(edb_o, jnt_num, static_cast<int16_t>(0));
        ResizeVector(fric_cof, jnt_num, static_cast<int16_t>(10));
        ResizeVector(jnt_inertia, jnt_num, static_cast<int16_t>(700));

        // Apply specific initialization for 6 joints
        if (jnt_num == 6) {
            jnt_inertia = {600, 500, 400, 300, 200, 100};
        }
    }

    void SetZero() {
        std::fill(trq_cmd.begin(), trq_cmd.end(), 0);
        std::fill(trq_feedforward.begin(), trq_feedforward.end(), 0);
        std::fill(k_p.begin(), k_p.end(), 0);
        std::fill(k_d.begin(), k_d.end(), 0);
        std::fill(k_p_reset_by_load.begin(), k_p_reset_by_load.end(), 0);
        std::fill(k_d_reset_by_load.begin(), k_d_reset_by_load.end(), 0);
        std::fill(edb_cof.begin(), edb_cof.end(), 0);
        std::fill(edb_o.begin(), edb_o.end(), 0);
        std::fill(fric_cof.begin(), fric_cof.end(), 0);
        std::fill(jnt_inertia.begin(), jnt_inertia.end(), 0);
    }
};

struct FcStatusInner {
    //拖动类型
    DragType drag_type;
    //关节指令
    KDL::JntArray jnt_pos_command;
    KDL::JntArray jnt_vel_command;
    KDL::JntArray jnt_acc_command;

    //笛卡尔指令
    KDL::Frame cart_pos_command_flan_in_base;  //旋转指令flan_in_base
    KDL::Frame cart_pos_command_tcp_in_base;   //旋转指令tcp_in_base
    KDL::Frame base_in_flan;
    KDL::JntArray cart_pos_jnt_command;
    KDL::Twist cart_pos_following_error_tcp_in_base;     //位置+旋转误差 tcp_in_base
    KDL::Twist cart_pos_following_error_tcp_in_fcframe;  //位置+旋转误差 tcp_in_frame
    KDL::Twist cart_vel_following_error_tcp_in_fcframe;  //速度误差 tcp_in_fcframe

    KDL::Twist cart_vel_command_flan_in_base;
    KDL::Twist cart_vel_command_tcp_in_base;
    KDL::Twist cart_vel_command_tcp_in_fcframe;

    //动力学指令
    KDL::JntArray jnt_trq_gra_command;

    //关节反馈
    KDL::JntArray jnt_pos_measure;
    KDL::JntArray jnt_vel_measure;
    KDL::JntArray jnt_acc_measure;
    KDL::JntArray jnt_trq_sensor_measure;  //根据传感器双通道计算的传感器反馈
    KDL::JntArray jnt_pos_following_error;

    //动力学反馈
    KDL::JntArray jnt_ineria_trq_measure;
    KDL::JntArray jnt_inertia;
    KDL::JntArray jnt_corlios_trq_measure;
    KDL::JntArray jnt_gravity_trq_measure;
    KDL::JntSpaceInertiaMatrix jnt_inertia_matrix_measure;

    //笛卡尔反馈
    KDL::Frame cart_pos_measure_flan_in_base;
    KDL::Frame cart_pos_measure_tcp_in_base;
    KDL::Vector cart_pos_following_error_flan_in_base_pos;
    KDL::Vector cart_pos_following_error_tcp_in_base_pos;
    KDL::Rotation cart_tcp_rot_between_command_and_measure;
    KDL::Twist cart_vel_measure_flan_in_base;
    KDL::Twist cart_vel_measure_tcp_in_base;
    KDL::Twist cart_vel_measure_tcp_in_fcframe;

    //反馈力雅可比
    KDL::Jacobian jac_measure_flan_in_base;
    KDL::Jacobian jac_measure_tcp_in_base;
    Jacobian_trans jac_trans_measure_flan_in_base;
    Jacobian_trans jac_trans_measure_tcp_in_base;
    Jacobian_inv jac_inv_measure_flan_in_base;
    Jacobian_trans_inv jac_trans_inv_measure_flan_in_base;
    double mani_measure;

    //外部力wrench
    KDL::Wrench flan_wrench;
    KDL::Wrench tcp_wrench;

    //指令力雅可比
    KDL::Jacobian jac_command_flan_in_base;
    KDL::Jacobian jac_command_tcp_in_base;

    //力矩指令
    KDL::JntArray jnt_trq_final_cmd;

    FcStatusInner(unsigned int jnt_num)
        : drag_type(DRAG_JOINT),
          jnt_pos_command(jnt_num),
          jnt_vel_command(jnt_num),
          jnt_acc_command(jnt_num),
          cart_pos_command_flan_in_base(KDL::Frame::Identity()),
          cart_pos_command_tcp_in_base(KDL::Frame::Identity()),
          base_in_flan(KDL::Frame::Identity()),
          cart_pos_jnt_command(jnt_num),
          cart_pos_following_error_tcp_in_base(KDL::Twist::Zero()),
          cart_pos_following_error_tcp_in_fcframe(KDL::Twist::Zero()),
          cart_vel_following_error_tcp_in_fcframe(KDL::Twist::Zero()),
          cart_vel_command_flan_in_base(KDL::Twist::Zero()),
          cart_vel_command_tcp_in_base(KDL::Twist::Zero()),
          cart_vel_command_tcp_in_fcframe(KDL::Twist::Zero()),
          jnt_trq_gra_command(jnt_num),
          jnt_pos_measure(jnt_num),
          jnt_vel_measure(jnt_num),
          jnt_acc_measure(jnt_num),
          jnt_trq_sensor_measure(jnt_num),
          jnt_pos_following_error(jnt_num),
          jnt_ineria_trq_measure(jnt_num),
          jnt_inertia(jnt_num),
          jnt_corlios_trq_measure(jnt_num),
          jnt_gravity_trq_measure(jnt_num),
          jnt_inertia_matrix_measure(jnt_num),
          cart_pos_measure_flan_in_base(KDL::Frame::Identity()),
          cart_pos_measure_tcp_in_base(KDL::Frame::Identity()),
          cart_pos_following_error_flan_in_base_pos(KDL::Vector::Zero()),
          cart_pos_following_error_tcp_in_base_pos(KDL::Vector::Zero()),
          cart_tcp_rot_between_command_and_measure(KDL::Rotation::Identity()),
          cart_vel_measure_flan_in_base(KDL::Twist::Zero()),
          cart_vel_measure_tcp_in_base(KDL::Twist::Zero()),
          cart_vel_measure_tcp_in_fcframe(KDL::Twist::Zero()),
          jac_measure_flan_in_base(jnt_num),
          jac_measure_tcp_in_base(jnt_num),
          jac_trans_measure_flan_in_base(jnt_num, 6),
          jac_trans_measure_tcp_in_base(jnt_num, 6),
          jac_inv_measure_flan_in_base(jnt_num, 6),
          jac_trans_inv_measure_flan_in_base(6, jnt_num),
          flan_wrench(KDL::Wrench::Zero()),
          tcp_wrench(KDL::Wrench::Zero()),
          jac_command_flan_in_base(jnt_num),
          jac_command_tcp_in_base(jnt_num),
          mani_measure(0.0),
          jnt_trq_final_cmd(jnt_num) {}

#define SET_FC_STATUS_INFO(name) this->name = fc_status_inner.name
    FcStatusInner& operator=(const FcStatusInner fc_status_inner) {
        SET_FC_STATUS_INFO(drag_type);
        SET_FC_STATUS_INFO(jnt_pos_command);
        SET_FC_STATUS_INFO(jnt_vel_command);
        SET_FC_STATUS_INFO(jnt_acc_command);
        SET_FC_STATUS_INFO(cart_pos_command_flan_in_base);
        SET_FC_STATUS_INFO(cart_pos_command_tcp_in_base);
        SET_FC_STATUS_INFO(base_in_flan);
        SET_FC_STATUS_INFO(cart_pos_jnt_command);
        SET_FC_STATUS_INFO(cart_pos_following_error_tcp_in_base);
        SET_FC_STATUS_INFO(cart_pos_following_error_tcp_in_fcframe);
        SET_FC_STATUS_INFO(cart_vel_following_error_tcp_in_fcframe);
        SET_FC_STATUS_INFO(cart_vel_command_flan_in_base);
        SET_FC_STATUS_INFO(cart_vel_command_tcp_in_base);
        SET_FC_STATUS_INFO(cart_vel_command_tcp_in_fcframe);
        SET_FC_STATUS_INFO(jnt_trq_gra_command);
        SET_FC_STATUS_INFO(jnt_pos_measure);
        SET_FC_STATUS_INFO(jnt_vel_measure);
        SET_FC_STATUS_INFO(jnt_acc_measure);
        SET_FC_STATUS_INFO(jnt_trq_sensor_measure);
        SET_FC_STATUS_INFO(jnt_pos_following_error);
        SET_FC_STATUS_INFO(jnt_ineria_trq_measure);
        SET_FC_STATUS_INFO(jnt_inertia);
        SET_FC_STATUS_INFO(jnt_corlios_trq_measure);
        SET_FC_STATUS_INFO(jnt_gravity_trq_measure);
        SET_FC_STATUS_INFO(jnt_inertia_matrix_measure);
        SET_FC_STATUS_INFO(cart_pos_measure_flan_in_base);
        SET_FC_STATUS_INFO(cart_pos_measure_tcp_in_base);
        SET_FC_STATUS_INFO(cart_pos_following_error_flan_in_base_pos);
        SET_FC_STATUS_INFO(cart_pos_following_error_tcp_in_base_pos);
        SET_FC_STATUS_INFO(cart_tcp_rot_between_command_and_measure);
        SET_FC_STATUS_INFO(cart_vel_measure_flan_in_base);
        SET_FC_STATUS_INFO(cart_vel_measure_tcp_in_base);
        SET_FC_STATUS_INFO(cart_vel_measure_tcp_in_fcframe);
        SET_FC_STATUS_INFO(jac_measure_flan_in_base);
        SET_FC_STATUS_INFO(jac_measure_tcp_in_base);
        SET_FC_STATUS_INFO(jac_trans_measure_flan_in_base);
        SET_FC_STATUS_INFO(jac_trans_measure_tcp_in_base);
        SET_FC_STATUS_INFO(jac_inv_measure_flan_in_base);
        SET_FC_STATUS_INFO(jac_trans_inv_measure_flan_in_base);
        SET_FC_STATUS_INFO(flan_wrench),
        SET_FC_STATUS_INFO(tcp_wrench),
        SET_FC_STATUS_INFO(jac_command_flan_in_base),
        SET_FC_STATUS_INFO(jac_command_tcp_in_base),
        SET_FC_STATUS_INFO(mani_measure);
        SET_FC_STATUS_INFO(jnt_trq_final_cmd);
        return *this;
    }
};

}  // namespace Control

}  // namespace RokaeApi

#endif
