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


//一些宏定义
#define DEFAULT_AXIS 6U
#define DEFAULT_SEGMENT 7U

#define PI 3.1415926535

using namespace KDL;

namespace RokaeApi {

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
    GAIN_VALUE_ERROR = -12
};

namespace Model {
enum MechUnitType {  ///< TODO 把robot_model　和　lircos/mechanical_unit中的定义干掉
    // INDUSTRYROBOT,        //机器人,这个是为了保留对以前代码的支持，lircos的robot用到了
    // SIX_AXIS_SERIES = 1,  //传统六轴串联，XB
    // UR,                   // UR构型机器人
    // FOUR_AXIS_SERIES,     //标准四轴串联
    // SCARA,                // scara
    // DELTA,                // delta
    // TRACK,                //导轨
    // POSITIONER,           //变位机	one/two axis
    // CONVEYOR,             //传送带	linear,indexed,circle...
    // XMATE_7,              // xMate 7轴
    // XMATE_6,              // xMate 6轴
    // RS,                   //第一轴为移动轴的scara类机械臂
    // XS,
    // CB_SCARA_3,      //精雕机三轴
    // PCB_THREE_AXIS,  // PCB三轴
    // PCB_FOUR_AXIS,   // PCB四轴
    // XMATECR_6,       // xMateCR 6轴
    //新增
    SR3_C,  // XMS3-R580-W4G3B1C
    SR4_C,  // XMS4-R800-B4G1A4C
    SR5_C
};
struct ModelParams {
    struct Link_Inertial {
        double mass;            //质量,单位kg
        double centroid[3];     //质心,单位mm
        double moment[6];       //总惯量，单位kg.mm^2
        double moment_link[6];  //连杆惯量，单位kg.mm^2
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
        }
    };
    unsigned int axis_num;
    std::vector<Link_Inertial> link_inertia;
    std::vector<KDL::Joint::JointType> joint_type;
    std::vector<Coord_Orientation> coor_orient;
    MechUnitType mech_type;
    RobDimensions rob_dimensions;
    std::vector<double> joint_range_min;  //软限位
    std::vector<double> joint_range_max;
    std::vector<double> joint_range_min_new;  //硬限位
    std::vector<double> joint_range_max_new;

    double max_load;
    ModelParams(unsigned int segments_cnt) : joint_type(segments_cnt), coor_orient(segments_cnt), link_inertia(segments_cnt){};

    void Resize(unsigned int segments_cnt) {
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
    MechanicalParams(unsigned int jnt_num) { Risize(jnt_num + 1); }

    void Risize(unsigned int jnt_num) {
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

enum ServoType {
    POSTION_CONTROL,  //位置模式
    FORCE_CONTROL,    //力矩模式
    UNKNOWN
};

enum ForceType {
    DRAG,      //拖动(暂时只有拖动)
    IMPEDANCE  //阻抗
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

struct FcStatusInfo {
    //关节指令
    KDL::JntArray jnt_pos_command;
    KDL::JntArray jnt_vel_command;
    KDL::JntArray jnt_trq_gra_command;
    //暂时不添加
    // KDL::JntArray jnt_trq_desire;
    // KDL::JntArray jnt_acc_command;

    //笛卡尔指令（暂不添加）

    //关节反馈
    KDL::JntArray jnt_pos_measure;
    KDL::JntArray jnt_vel_measure;
    KDL::JntArray jnt_vel_measure_abs;
    KDL::JntArray jnt_trq_gra_measure;
    KDL::JntArray jnt_trq_sensor_measure;//根据传感器双通道计算的传感器反馈
    KDL::JntArray jnt_pos_following_error;
    KDL::JntSpaceInertiaMatrix jnt_inertia_matrix_measure;
    // KDL::JntArray jnt_acc_measure;

    //滤波后
    KDL::JntArray jnt_vel_measure_filter;

    //笛卡尔反馈（暂不添加）

    //一些状态标志
    DragType drag_type;
    ForceType force_type;
    std::vector<ServoType> servo_type;

    FcStatusInfo(unsigned int jnt_num) {
        jnt_pos_command.resize(jnt_num);
        jnt_vel_command.resize(jnt_num);
        jnt_trq_gra_command.resize(jnt_num);
        jnt_pos_measure.resize(jnt_num);
        jnt_vel_measure.resize(jnt_num);
        jnt_vel_measure_abs.resize(jnt_num);
        jnt_vel_measure_filter.resize(jnt_num);
        jnt_trq_gra_measure.resize(jnt_num);
        jnt_trq_sensor_measure.resize(jnt_num);
        jnt_pos_following_error.resize(jnt_num);
        jnt_inertia_matrix_measure.resize(jnt_num);
        servo_type.resize(jnt_num);
        drag_type = DragType::DRAG_JOINT;
        force_type = ForceType::DRAG;
    }

#define SET_FC_STATUS_INFO(name) this->name = fc_status_info.name
    FcStatusInfo& operator=(const FcStatusInfo fc_status_info) {
        SET_FC_STATUS_INFO(jnt_pos_command);
        SET_FC_STATUS_INFO(jnt_vel_command);
        SET_FC_STATUS_INFO(jnt_pos_measure);
        SET_FC_STATUS_INFO(jnt_vel_measure);
        SET_FC_STATUS_INFO(jnt_vel_measure_abs);
        SET_FC_STATUS_INFO(jnt_vel_measure_filter);
        SET_FC_STATUS_INFO(jnt_trq_sensor_measure);
        SET_FC_STATUS_INFO(jnt_inertia_matrix_measure);
        SET_FC_STATUS_INFO(drag_type);
        SET_FC_STATUS_INFO(servo_type);
        return *this;
    }
};

struct ServoToFc {
    std::vector<int32_t> pos_feedback_0x6064;
    std::vector<int32_t> vel_feedback_0x606C;
    std::vector<int16_t> trq_feedback_0x2406;
    std::vector<int16_t> analog_ch1_0x2401;
    std::vector<int16_t> analog_ch2_0x2402;
    std::vector<int8_t> mode_operation_0x6061;

    ServoToFc(unsigned int jnt_num) { Resize(jnt_num); }

    void Resize(unsigned int jnt_num) {
        pos_feedback_0x6064.resize(jnt_num, 0);
        vel_feedback_0x606C.resize(jnt_num, 0);
        trq_feedback_0x2406.resize(jnt_num, 0);
        analog_ch1_0x2401.resize(jnt_num, 0);
        analog_ch2_0x2402.resize(jnt_num, 0);
        mode_operation_0x6061.resize(jnt_num, 0);
    }
};

struct FcToServo {
    std::vector<int16_t> trq_cmd;          //关节扭矩指令
    std::vector<int16_t> trq_feedforward;  //力矩前馈
    std::vector<int16_t> k_p;              //关节力矩环带宽
    std::vector<int16_t> k_d;              //关节阻尼比
    std::vector<int16_t> edb_cof;          //传感器线性度
    std::vector<int16_t> edb_o;            //传感器偏置
    std::vector<int16_t> fric_cof;         //摩擦力补偿系数
    std::vector<int16_t> jnt_inertia;      //关节惯量

    FcToServo(unsigned int jnt_num) { Resize(jnt_num); }

    void Resize(unsigned int jnt_num) {
        trq_cmd.resize(jnt_num, 0);
        trq_feedforward.resize(jnt_num,0);
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
        std::fill(trq_feedforward.begin(),trq_feedforward.end(),0);
        std::fill(k_p.begin(), k_p.end(), 0);
        std::fill(k_d.begin(), k_d.end(), 0);
        std::fill(edb_cof.begin(), edb_cof.end(), 0);
        std::fill(edb_o.begin(), edb_o.end(), 0);
        std::fill(fric_cof.begin(), fric_cof.end(), 0);
        std::fill(jnt_inertia.begin(), jnt_inertia.end(), 0);
    }
};
}  // namespace Control

}  // namespace RokaeApi

#endif
