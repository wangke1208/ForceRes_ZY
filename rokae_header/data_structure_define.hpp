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

#ifndef DATA_STRUCTURE_DEFINE_H
#define DATA_STRUCTURE_DEFINE_H
#include <../3rd/kdl/chain.hpp>
#include <../3rd/kdl/chaindynparam.hpp>
#include <../3rd/kdl/jntarray.hpp>
#include <../3rd/kdl/jntspaceinertiamatrix.hpp>
#include <../3rd/kdl/joint.hpp>
#include <vector>

#include "fc_params.hpp"

//一些宏定义
#define DEFAULT_AXIS 6U
#define DEFAULT_SEGMENT 7U

#define PI 3.1415926535

using namespace KDL;

namespace RokaeApi {

typedef Eigen::Matrix<double, Eigen::Dynamic, 6> Jacobian_trans;
typedef Eigen::Matrix<double, Eigen::Dynamic, 6> Jacobian_inv;
typedef Eigen::Matrix<double, 6, Eigen::Dynamic> Jacobian_trans_inv;

//精度
const float EPSILON16 = 0.000000000000001;
const float EPSILON15 = 0.00000000000001;
const float EPSILON14 = 0.0000000000001;
const float EPSILON13 = 0.000000000001;
const float EPSILON12 = 0.00000000001;
const float EPSILON11 = 0.0000000001;
const float EPSILON = 0.0000000001;
const float EPSILON9 = 0.000000001;
const float EPSILON8 = 0.00000001;
const float EPSILON7 = 0.0000001;
const float EPSILON6 = 0.000001;
const float EPSILON5 = 0.00001;
const float EPSILON4 = 0.0001;
const float EPSILON3 = 0.001;
const float EPSILON2 = 0.01;
const float EPSILON1 = 0.1;

enum SolverRes {
    SOLVE_NOERROR = 0,
    ROBOTTYPE_ERROR = -1,
    DRAGTYPE_ERROR = -2,
    SERVO_MODE_ERROR = -3,
    SIZE_ERROR = -4,
    ERROR_DRAG_ENABLE = -5,
    EXCESSIVE_TORQUE_ERROR = -6,
    SENSOR_BIAS_ERROR = -7,
    LOAD_PARAMS_ERROR = -8,
    SOFT_LIMIT_PARAMS_ERROR = -9,
    STARTDRAG_POS_OVER_LIMIT = -10,
    AXIS_NUM_ERROR = -11,
    GAIN_VALUE_ERROR = -12,
    SENSOR_LINERALITY_ERROR = -13,
    ERROR_RPY_CAL = -14,
    INIT_ERROR = -15
};

enum ServoMode { SERVO_MODE_POS = 8, SERVO_MODE_TORQUE = 10 };

namespace Model {
enum MechUnitType {
    UNKNOWN,
    SR3_C,  // XMS3-R580-W4G3B1C
};
struct ModelParams {
    struct Link_Inertial {
        double mass;            //质量,单位kg
        std::vector<double> centroid;     //质心,单位mm
        std::vector<double> moment;       //总惯量，单位kg.mm^2
        std::vector<double> moment_link;  //连杆惯量，单位kg.mm^2

        Link_Inertial() : mass(0.0), centroid(3, 0.0), moment(6, 0.0), moment_link(6, 0.0){};
        void SetZero() {
            this->mass = 0.0;
            this->centroid.assign(3, 0.0);
            this->moment.assign(6, 0.0);
            this->moment_link.assign(6, 0.0);
        }
    };

    enum Rot_Axis { ROT_NONE, ROT_X, ROT_Y, ROT_Z };

    struct Coord_Orientation {
        Rot_Axis rot_axis;
        double rot_angle;
    };

    struct RobDimensions {
        double L01x;  // z 0,1 轴沿着base轴坐标系x方向的距离
        double L01y;  // z 0,1 轴沿着base轴坐标系x方向的距离
        double L01z;  // z 0,1 轴沿着base轴坐标系x方向的距离
        double L12x;  // z 1,2 轴沿着base轴坐标系x方向的距离
        double L12y;  // z 1,2 轴沿着base轴坐标系x方向的距离
        double L12z;  // z 1,2 轴沿着base轴坐标系x方向的距离
        double L23x;  // z 2,3 轴沿着base轴坐标系x方向的距离
        double L23y;  // z 2,3 轴沿着base轴坐标系x方向的距离
        double L23z;  // z 2,3 轴沿着base轴坐标系x方向的距离
        double L34x;  // z 3,4 轴沿着base轴坐标系x方向的距离
        double L34y;  // z 3,4 轴沿着base轴坐标系x方向的距离
        double L34z;  // z 3,4 轴沿着base轴坐标系x方向的距离
        double L45x;  // z 4,5 轴沿着base轴坐标系x方向的距离
        double L45y;  // z 4,5 轴沿着base轴坐标系x方向的距离
        double L45z;  // z 4,5 轴沿着base轴坐标系x方向的距离
        double L56x;  // z 5,6 轴沿着base轴坐标系x方向的距离
        double L56y;  // z 5,6 轴沿着base轴坐标系x方向的距离
        double L56z;  // z 5,6 轴沿着base轴坐标系x方向的距离
        double L67x;  // z 6,7 轴沿着base轴坐标系x方向的距离
        double L67y;  // z 6,7 轴沿着base轴坐标系x方向的距离
        double L67z;  // z 6,7 轴沿着base轴坐标系x方向的距离
        double L78x;  // z 7,8 轴沿着base轴坐标系x方向的距离
        double L78y;  // z 7,8 轴沿着base轴坐标系x方向的距离
        double L78z;  // z 7,8 轴沿着base轴坐标系x方向的距离

        // 添加RobDimensions的默认构造函数
        RobDimensions() {}
        RobDimensions(std::vector<double> rd) {
            int joint_num_temp = rd.size() / 3;
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
            if (joint_num_temp >= 4) {
                L45x = rd[12];
                L45y = rd[13];
                L45z = rd[14];
                if (joint_num_temp >= 5) {
                    L56x = rd[15];
                    L56y = rd[16];
                    L56z = rd[17];
                    if (joint_num_temp >= 6) {
                        L67x = rd[18];
                        L67y = rd[19];
                        L67z = rd[20];
                        if (joint_num_temp >= 7) {
                            L78x = rd[21];
                            L78y = rd[22];
                            L78z = rd[23];
                        }
                    }
                }
            }
        }
    };
    unsigned int axis_num;
    std::vector<Link_Inertial> link_inertia;
    std::vector<KDL::Joint::JointType> joint_type;
    std::vector<Coord_Orientation> coor_orient;
    RobDimensions rob_dimensions;
    std::vector<double> joint_range_min;  //软限位
    std::vector<double> joint_range_max;
    std::vector<double> joint_range_min_new;  //硬限位
    std::vector<double> joint_range_max_new;
    double max_load;
    // ModelParams(){this->Resize(DEFAULT_SEGMENT);};
    ModelParams(unsigned int segments_cnt) { this->Resize(segments_cnt); };

    void Resize(unsigned int segments_cnt) {
        max_load = 0.0;
        axis_num = segments_cnt - 1;
        joint_type.resize(segments_cnt);
        coor_orient.resize(segments_cnt);
        link_inertia.resize(segments_cnt);
        joint_range_min.resize(segments_cnt - 1);
        joint_range_max.resize(segments_cnt - 1);
        joint_range_min_new.resize(segments_cnt - 1);
        joint_range_max_new.resize(segments_cnt - 1);
    }
};

struct MechanicalParams {
    //位置编码器
    std::vector<int32_t> encoder_offset;
    std::vector<int> encoder_resolution;
    //减速比
    std::vector<double> decel_ratio_high;
    std::vector<double> decel_ratio_low;
    //传感器
    std::vector<double> analog2trq_high;
    std::vector<double> analog2trq_low;
    std::vector<double> analog_bias;
    std::vector<double> sensor_amplify;
    //电机
    std::vector<double> rated_torque;
    // MechanicalParams() { this->Resize(6); }  //默认6轴构造
    MechanicalParams(unsigned int jnt_num) { Resize(jnt_num); }

    void Resize(unsigned int jnt_num) {
        encoder_offset.resize(jnt_num);
        encoder_resolution.resize(jnt_num);
        decel_ratio_high.resize(jnt_num);
        decel_ratio_low.resize(jnt_num);

        analog2trq_high.resize(jnt_num);
        analog2trq_low.resize(jnt_num);
        analog_bias.resize(jnt_num);
        sensor_amplify.resize(jnt_num);
        rated_torque.resize(jnt_num);
    }

#define SET_MECH_STATUS_INFO(name) this->name = mec_param.name
    MechanicalParams& operator=(const MechanicalParams mec_param) {
        SET_MECH_STATUS_INFO(encoder_offset);
        SET_MECH_STATUS_INFO(encoder_resolution);
        SET_MECH_STATUS_INFO(decel_ratio_high);
        SET_MECH_STATUS_INFO(decel_ratio_low);
        SET_MECH_STATUS_INFO(analog2trq_high);
        SET_MECH_STATUS_INFO(analog2trq_low);
        SET_MECH_STATUS_INFO(analog_bias);
        SET_MECH_STATUS_INFO(sensor_amplify);
        SET_MECH_STATUS_INFO(rated_torque);
        return *this;
    }
};

enum RobotType {
    ROBT_INVALID_ROBOT,    // 0
    ROBT_STANDARD_6_AXES,  // 1
    ROBT_UR_6_AXES,        // 2
    ROBT_XMATE_6_AXES,     // 3
    ROBT_XMATE_7_AXES,     // 4
    ROBT_RS_4_AXES,        // 5
    ROBT_SCARA_4_AXES,     // 6
    ROBT_XD_3_AXES,        // 7
    ROBT_PCB_3_AXES,       // 8
    ROBT_PCB_4_AXES,       // 9
    ROBOT_XMATE_CR_6_AXES  // 10  注意SR机型和CR机型使用的是相同的字段
};

struct LoadInertia {
    double m;              //质量，单位kg
    KDL::Vector m_cog;  // 质心,单位m
    KDL::Vector mx;     // 一阶矩阵 单位kg.m
    double inertia[6];  // 二阶矩阵,ix,iy,iz,ixy,ixz,iyz,单位kg.m^2
    LoadInertia() : m(0.0), m_cog(0.0, 0.0, 0.0), mx(0.0, 0.0, 0.0) {
        for (unsigned int i = 0; i < 6; ++i) {
            inertia[i] = 0.0;
        }
    }

    LoadInertia(const double& mass, const KDL::Vector& cog) : m(mass), m_cog(cog) {
        mx = m * cog;
        for (int i = 0; i < 6; i++) {
            inertia[i] = 0.0;
        }
    }

    KDL::Vector GetCOG() const { return m_cog; }

    void GetCOG(double& x, double& y, double& z) const {
        x = m_cog.x();
        y = m_cog.y();
        z = m_cog.z();
    }

    void SetCOG(const KDL::Vector& cog) {
        this->m_cog = cog;
        this->mx = this->m * cog;
    }

    void SetInertia(double ix, double iy, double iz, double ixy = 0, double ixz = 0, double iyz = 0) {
        inertia[0] = ix;
        inertia[1] = iy;
        inertia[2] = iz;
        inertia[3] = ixy;
        inertia[4] = ixz;
        inertia[5] = iyz;
    }

    void SetZero() {
        this->m = 0.0;
        for (unsigned int i = 0; i < 3; ++i) {
            this->m_cog(i) = 0.0;
            this->mx(i) = 0.0;
        }
        for (unsigned int i = 0; i < 6; ++i) {
            this->inertia[i] = 0.0;
        }
    }
    LoadInertia& operator=(const LoadInertia load_input) {
        this->SetCOG(load_input.GetCOG());
        this->m = load_input.m;
        this->SetInertia(load_input.inertia[0], load_input.inertia[1], load_input.inertia[2], load_input.inertia[3], load_input.inertia[4],
                         load_input.inertia[5]);
        return *this;
    }
};

struct Vector3D {
    double m_x, m_y, m_z;

    // 构造函数
    Vector3D(double x = 0.0, double y = 0.0, double z = 0.0) : m_x(x), m_y(y), m_z(z) {}

    // 其他可能的成员函数，如加法、减法、点乘等（这里省略）
    void SetToZero() {
        m_x = 0.0;
        m_y = 0.0;
        m_z = 0.0;
    }
};

struct LoadInertia_C {
    double m;
    Vector3D m_cog;     // 质心
    Vector3D mx;        // 一阶矩阵
    double inertia[6];  // 二阶矩阵,ix,iy,iz,ixy,ixz,iyz
    LoadInertia_C() : m(0.0) {
        m_cog.SetToZero();
        mx.SetToZero();
        for (unsigned int i = 0; i < 6; ++i) {
            inertia[i] = 0.0;
        }
    }

    LoadInertia_C(double mass, Vector3D cog) : m(mass) {
        m_cog.m_x = cog.m_x;
        m_cog.m_y = cog.m_y;
        m_cog.m_z = cog.m_z;
        mx.m_x = m_cog.m_x * m;
        mx.m_y = m_cog.m_y * m;
        mx.m_z = m_cog.m_z * m;
        for (int i = 0; i < 6; i++) {
            inertia[i] = 0.0;
        }
    }

    Vector3D GetCOG() const { return m_cog; }

    void SetCOG(const Vector3D cog) {
        m_cog.m_x = cog.m_x;
        m_cog.m_y = cog.m_y;
        m_cog.m_z = cog.m_z;
    }

    void SetInertia(double ix, double iy, double iz, double ixy = 0, double ixz = 0, double iyz = 0) {
        inertia[0] = ix;
        inertia[1] = iy;
        inertia[2] = iz;
        inertia[3] = ixy;
        inertia[4] = ixz;
        inertia[5] = iyz;
    }

    void SetZero() {
        m = 0.0;
        m_cog.SetToZero();
        mx.SetToZero();
        for (unsigned int i = 0; i < 6; ++i) {
            inertia[i] = 0.0;
        }
    }
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

enum ServoType {
    POSTION_CONTROL,  //位置模式
    FORCE_CONTROL,    //力矩模式
    UNKNOWN
};

struct ProtectParams {
    unsigned int m_jnt_num;
    //轴空间
    std::vector<double> max_joint_stiff;
    std::vector<double> max_joint_damp;
    std::vector<double> max_joint_stiff_trq;
    std::vector<double> max_joint_damp_trq;
    //笛卡尔空间(暂时忽略)

    //保护功能
    std::vector<double> max_mode_switch_trq;
    std::vector<double> soft_joint_limit_stiff;
    std::vector<double> soft_joint_limit_damp;
    //以下参数暂不开放
    // std::vector<double> virtual_wall_stiff;
    // std::vector<double> prevent_mini_distance;

    ProtectParams(unsigned int jnt_num = 6) : m_jnt_num(jnt_num){};  //默认为6轴
    ~ProtectParams(){};
    void InitProtectParams(unsigned int size) {
        m_jnt_num = size;
        max_joint_stiff.resize(m_jnt_num, 300.0);
        max_joint_damp.resize(m_jnt_num, 10.0);
        max_joint_stiff_trq.resize(m_jnt_num, 60.0);
        max_joint_damp_trq.resize(m_jnt_num, 20.0);
        max_mode_switch_trq.resize(m_jnt_num, 30.0);
        soft_joint_limit_stiff.resize(m_jnt_num, 1000.0);
        soft_joint_limit_damp.resize(m_jnt_num, 10.0);
    }
};

struct GainParams {
    unsigned int m_jnt_num;
    //轴空间
    std::vector<double> joint_gain_kp;
    std::vector<double> joint_damp_zeta;
    std::vector<double> friction_cof_servo;
    //笛卡尔空间暂不开放
    //阻抗暂不开放

    GainParams(unsigned int jnt_num = 6) : m_jnt_num(jnt_num){};  //默认为6轴
    ~GainParams(){};
    void InitGainParams(unsigned int size) {
        m_jnt_num = size;
        joint_gain_kp.resize(m_jnt_num, 1.0);
        joint_damp_zeta.resize(m_jnt_num, 0.707);
        friction_cof_servo.resize(m_jnt_num, 0.6);
    }
};

struct ControlParams {
    ProtectParams m_protect_params;
    GainParams m_gain_params;
    // ControlParams() { this->Resize(6); }
    ControlParams(unsigned int jnt_num) {
        m_protect_params.InitProtectParams(jnt_num);
        m_gain_params.InitGainParams(jnt_num);
    }
    void Resize(unsigned int jnt_num) {
        m_protect_params.InitProtectParams(jnt_num);
        m_gain_params.InitGainParams(jnt_num);
    }
    ~ControlParams(){};
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
        pos_feedback.resize(jnt_num, 0);
        vel_feedback.resize(jnt_num, 0);
        trq_feedback.resize(jnt_num, 0);
        analog_ch1.resize(jnt_num, 0);
        analog_ch2.resize(jnt_num, 0);
        mode_operation.resize(jnt_num, 0);
    }
};

struct FcInner_To_Servo {
    std::vector<int16_t> trq_cmd;          //关节扭矩指令
    std::vector<int16_t> trq_feedforward;  //力矩前馈
    std::vector<int16_t> k_p;              //关节力矩环带宽
    std::vector<int16_t> k_d;              //关节阻尼比
    std::vector<int16_t> edb_cof;          //传感器线性度
    std::vector<int16_t> edb_o;            //传感器偏置
    std::vector<int16_t> fric_cof;         //摩擦力补偿系数
    std::vector<int16_t> jnt_inertia;      //关节惯量

    FcInner_To_Servo(unsigned int jnt_num) { Resize(jnt_num); }

    void Resize(unsigned int jnt_num) {
        trq_cmd.resize(jnt_num, 0);
        trq_feedforward.resize(jnt_num, 0);
        k_p.resize(jnt_num, 2000);
        k_d.resize(jnt_num, 70);
        edb_cof.resize(jnt_num, 100);
        edb_o.resize(jnt_num, 0);
        fric_cof.resize(jnt_num, 10);
        jnt_inertia.resize(jnt_num, 700);
        //对惯量做初值保护
        if (jnt_num == 6) {
            jnt_inertia = {600, 500, 400, 300, 200, 100};
        }
    }

    void SetZero() {
        std::fill(trq_cmd.begin(), trq_cmd.end(), 0);
        std::fill(trq_feedforward.begin(), trq_feedforward.end(), 0);
        std::fill(k_p.begin(), k_p.end(), 0);
        std::fill(k_d.begin(), k_d.end(), 0);
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
    KDL::JntArray cart_pos_jnt_command;
    KDL::Twist cart_pos_following_error_tcp_in_base;  //位置+旋转误差 tcp_in_base
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
          mani_measure(0.0),
          jnt_trq_final_cmd(jnt_num){}

#define SET_FC_STATUS_INFO(name) this->name = fc_status_inner.name
    FcStatusInner& operator=(const FcStatusInner fc_status_inner) {
        SET_FC_STATUS_INFO(drag_type);
        SET_FC_STATUS_INFO(jnt_pos_command);
        SET_FC_STATUS_INFO(jnt_vel_command);
        SET_FC_STATUS_INFO(jnt_acc_command);
        SET_FC_STATUS_INFO(cart_pos_command_flan_in_base);
        SET_FC_STATUS_INFO(cart_pos_command_tcp_in_base);
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
        SET_FC_STATUS_INFO(mani_measure);
        SET_FC_STATUS_INFO(jnt_trq_final_cmd);
        return *this;
    }
};

}  // namespace Control

}  // namespace RokaeApi

#endif
