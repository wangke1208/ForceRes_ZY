/**
 * Copyright(C) 2024 Rokae Technology Co., Ltd.
 * All Rights Reserved.
 *
 * Information in this file is the intellectual property of Rokae Technology Co., Ltd,
 * And may contains trade secrets that must be stored and viewed confidentially.
 *
 * @file: initialize.cpp
 * @author: wangke
 * @date: 2024/4/25
 * @brief: 初始化模块
 */

#include "../rokae_header/initialize.hpp"

using namespace RokaeApi::XMS4;
using namespace RokaeApi::XMS5;
using namespace RokaeApi::XMS3;
namespace RokaeApi {

InitRobot::InitRobot(const Model::MechUnitType robot_type)
    : m_robot_type(robot_type), m_model_param(DEFAULT_SEGMENT), m_control_param(DEFAULT_AXIS), m_mechanical_params(DEFAULT_AXIS) {
    m_gravity = KDL::Vector(0, 0, -9.81);  //用来构建动力学解算器的重力矩
    m_chain = KDL::Chain();
    m_jnt_num = 6;  //默认为6轴机器人
}

int InitRobot::CreateModels() {
    //根据机器类型，进行参数初始化

    switch (m_robot_type) {
    case SR3_C:
        m_jnt_num = XMS3::AXIS_NUM;
        break;
    //机型为SR4_C
    case SR4_C:
        m_jnt_num = XMS4::AXIS_NUM;
        break;
    case SR5_C:
        m_jnt_num = XMS5::AXIS_NUM;
        break;
    default:
        return ROBOTTYPE_ERROR;
    }

    //读取参数&模型构建
    if (ReadParams(m_model_param, m_control_param, m_mechanical_params) != SOLVE_NOERROR) {
        return ROBOTTYPE_ERROR;
    };

    MakeChain_By_RobDim(m_model_param, m_chain);

    return SOLVE_NOERROR;
}

int InitRobot::ReadParams(ModelParams& model_params, ControlParams& control_params, MechanicalParams& mechanical_params) {
    //读取参数部分
    switch (m_robot_type) {
    case SR3_C:
        ReadModelParamsSR3(model_params);
        ReadControlParamsSR3(control_params);
        ReadMechanicalParamsSR3(mechanical_params);
        break;
    case SR4_C:
        ReadModelParamsSR4(model_params);
        ReadControlParamsSR4(control_params);
        ReadMechanicalParamsSR4(mechanical_params);
        break;
    case SR5_C:
        ReadModelParamsSR5(model_params);
        ReadControlParamsSR5(control_params);
        ReadMechanicalParamsSR5(mechanical_params);
        break;

    default:
        return ROBOTTYPE_ERROR;
    }

    return SOLVE_NOERROR;
}

void InitRobot::MakeChain_By_RobDim(const ModelParams& model_params_input, KDL::Chain& chain) {
    double rob_dim_x;
    double rob_dim_y;
    double rob_dim_z;
    auto dim_ptr = &(model_params_input.rob_dimensions.L01x) - 1;

    for (unsigned int i = 0; i < model_params_input.joint_type.size(); i++) {
        rob_dim_x = *(++dim_ptr);
        rob_dim_y = *(++dim_ptr);
        rob_dim_z = *(++dim_ptr);
        KDL::Rotation rot_orien;
        switch (model_params_input.coor_orient[i].rot_axis) {
        case ModelParams::ROT_X:
            rot_orien.DoRotX(model_params_input.coor_orient[i].rot_angle);
            break;
        case ModelParams::ROT_Y:
            rot_orien.DoRotY(model_params_input.coor_orient[i].rot_angle);
            break;
        case ModelParams::ROT_Z:
            rot_orien.DoRotZ(model_params_input.coor_orient[i].rot_angle);
            break;
        default:
            break;
        }
        //
        // KDL::Vector cog_in_tip =
        //     rot_orien.Inverse() * (KDL::Vector(model_params_input.link_inertia[i].centroid[0], model_params_input.link_inertia[i].centroid[1],
        //                                        model_params_input.link_inertia[i].centroid[2]) -
        //                            KDL::Vector(rob_dim_x, rob_dim_y, rob_dim_z));
        // chain.addSegment(KDL::Segment(
        //     "Segment " + std::to_string(i), KDL::Joint("Joint " + std::to_string(i), model_params_input.joint_type[i]),
        //     KDL::Frame(rot_orien, KDL::Vector(rob_dim_x, rob_dim_y, rob_dim_z)),
        //     KDL::RigidBodyInertia(model_params_input.link_inertia[i].mass, cog_in_tip,
        //                           KDL::RotationalInertia(model_params_input.link_inertia[i].moment[0], model_params_input.link_inertia[i].moment[1],
        //                                                  model_params_input.link_inertia[i].moment[2], model_params_input.link_inertia[i].moment[3],
        //                                                  model_params_input.link_inertia[i].moment[4], model_params_input.link_inertia[i].moment[5]))));

        //纯连杆求解器
        KDL::Vector cog_in_tip =
            rot_orien.Inverse() * (KDL::Vector(model_params_input.link_inertia[i].centroid[0], model_params_input.link_inertia[i].centroid[1],
                                               model_params_input.link_inertia[i].centroid[2]) -
                                   KDL::Vector(rob_dim_x, rob_dim_y, rob_dim_z));
        chain.addSegment(
            KDL::Segment("Segment " + std::to_string(i), KDL::Joint("Joint " + std::to_string(i), model_params_input.joint_type[i]),
                         KDL::Frame(rot_orien, KDL::Vector(rob_dim_x, rob_dim_y, rob_dim_z)),
                         KDL::RigidBodyInertia(
                             model_params_input.link_inertia[i].mass, cog_in_tip,
                             KDL::RotationalInertia(model_params_input.link_inertia[i].moment_link[0], model_params_input.link_inertia[i].moment_link[1],
                                                    model_params_input.link_inertia[i].moment_link[2], model_params_input.link_inertia[i].moment_link[3],
                                                    model_params_input.link_inertia[i].moment_link[4], model_params_input.link_inertia[i].moment_link[5]))));
    }
}

int InitRobot::ReadModelParamsSR3(Model::ModelParams& model_params_output) {
    //机器类型
    if (XMS3::ROBOT_NAME == "XMS3-R580-W4G3B1C") {
        model_params_output.mech_type = MechUnitType::SR3_C;
    } else {
        return ROBOTTYPE_ERROR;
    }
    //机器轴数
    model_params_output.axis_num = XMS3::AXIS_NUM;

    //最大负载(kg)
    model_params_output.max_load = XMS3::MAX_LOAD;

    //软限位
    model_params_output.joint_range_max = XMS3::JOINT_RANGE_MAX_CUSTOMIZE;
    model_params_output.joint_range_min = XMS3::JOINT_RANGE_MIN_CUSTOMIZE;

    //硬限位
    model_params_output.joint_range_max_new = XMS3::JOINT_RANGE_MAX_NEW;
    model_params_output.joint_range_min_new = XMS3::JOINT_RANGE_MIN_NEW;

    //连杆相关参数

    //基座各参数赋值0
    model_params_output.link_inertia[0].mass = 0;
    model_params_output.link_inertia[0].centroid[0] = 0;
    model_params_output.link_inertia[0].centroid[1] = 0;
    model_params_output.link_inertia[0].centroid[2] = 0;
    model_params_output.link_inertia[0].moment[0] = 0;
    model_params_output.link_inertia[0].moment[1] = 0;
    model_params_output.link_inertia[0].moment[2] = 0;
    model_params_output.link_inertia[0].moment[3] = 0;
    model_params_output.link_inertia[0].moment[4] = 0;
    model_params_output.link_inertia[0].moment[5] = 0;
    model_params_output.link_inertia[0].moment_link[0] = 0;
    model_params_output.link_inertia[0].moment_link[1] = 0;
    model_params_output.link_inertia[0].moment_link[2] = 0;
    model_params_output.link_inertia[0].moment_link[3] = 0;
    model_params_output.link_inertia[0].moment_link[4] = 0;
    model_params_output.link_inertia[0].moment_link[5] = 0;

    //各轴参数赋值
    unsigned int k;
    for (unsigned int i = 1; i < m_jnt_num + 1; i++) {
        k = i - 1;
        model_params_output.link_inertia[i].mass = XMS3::LINK_MASS[k];
        model_params_output.link_inertia[i].centroid[0] = XMS3::LINK_CENTROID[k * 3 + 0] / 1000.0;
        model_params_output.link_inertia[i].centroid[1] = XMS3::LINK_CENTROID[k * 3 + 1] / 1000.0;
        model_params_output.link_inertia[i].centroid[2] = XMS3::LINK_CENTROID[k * 3 + 2] / 1000.0;
        model_params_output.link_inertia[i].moment[0] = XMS3::LINK_MOMENT_OF_INERTIA[k * 6 + 0] / 1000000.0;
        model_params_output.link_inertia[i].moment[1] = XMS3::LINK_MOMENT_OF_INERTIA[k * 6 + 1] / 1000000.0;
        model_params_output.link_inertia[i].moment[2] = XMS3::LINK_MOMENT_OF_INERTIA[k * 6 + 2] / 1000000.0;
        model_params_output.link_inertia[i].moment[3] = XMS3::LINK_MOMENT_OF_INERTIA[k * 6 + 3] / 1000000.0;
        model_params_output.link_inertia[i].moment[4] = XMS3::LINK_MOMENT_OF_INERTIA[k * 6 + 4] / 1000000.0;
        model_params_output.link_inertia[i].moment[5] = XMS3::LINK_MOMENT_OF_INERTIA[k * 6 + 5] / 1000000.0;
        model_params_output.link_inertia[i].moment_link[0] = XMS3::LINK_MOMENT_OF_INERTIA_LOW[k * 6 + 0] / 1000000.0;
        model_params_output.link_inertia[i].moment_link[1] = XMS3::LINK_MOMENT_OF_INERTIA_LOW[k * 6 + 1] / 1000000.0;
        model_params_output.link_inertia[i].moment_link[2] = XMS3::LINK_MOMENT_OF_INERTIA_LOW[k * 6 + 2] / 1000000.0;
        model_params_output.link_inertia[i].moment_link[3] = XMS3::LINK_MOMENT_OF_INERTIA_LOW[k * 6 + 3] / 1000000.0;
        model_params_output.link_inertia[i].moment_link[4] = XMS3::LINK_MOMENT_OF_INERTIA_LOW[k * 6 + 4] / 1000000.0;
        model_params_output.link_inertia[i].moment_link[5] = XMS3::LINK_MOMENT_OF_INERTIA_LOW[k * 6 + 5] / 1000000.0;
    }

    // Coord_Orientation
    model_params_output.joint_type[0] = KDL::Joint::JointType::None;
    model_params_output.joint_type[1] = KDL::Joint::JointType::RotZ;
    model_params_output.joint_type[2] = KDL::Joint::JointType::RotY;
    model_params_output.joint_type[3] = KDL::Joint::JointType::RotY;
    model_params_output.joint_type[4] = KDL::Joint::JointType::RotZ;
    model_params_output.joint_type[5] = KDL::Joint::JointType::RotY;
    model_params_output.joint_type[6] = KDL::Joint::JointType::RotZ;
    model_params_output.coor_orient[2].rot_axis = Model::ModelParams::ROT_Z;
    model_params_output.coor_orient[2].rot_angle = PI;
    model_params_output.coor_orient[6].rot_axis = Model::ModelParams::ROT_Z;
    model_params_output.coor_orient[6].rot_angle = PI;

    // rob_dimension数据结构转换

    model_params_output.rob_dimensions.L01x = XMS3::ROBOT_DIMENSIONS[0] / 1000.0;
    model_params_output.rob_dimensions.L01y = XMS3::ROBOT_DIMENSIONS[1] / 1000.0;
    model_params_output.rob_dimensions.L01z = XMS3::ROBOT_DIMENSIONS[2] / 1000.0;
    model_params_output.rob_dimensions.L12x = XMS3::ROBOT_DIMENSIONS[3] / 1000.0;
    model_params_output.rob_dimensions.L12y = XMS3::ROBOT_DIMENSIONS[4] / 1000.0;
    model_params_output.rob_dimensions.L12z = XMS3::ROBOT_DIMENSIONS[5] / 1000.0;
    model_params_output.rob_dimensions.L23x = XMS3::ROBOT_DIMENSIONS[6] / 1000.0;
    model_params_output.rob_dimensions.L23y = XMS3::ROBOT_DIMENSIONS[7] / 1000.0;
    model_params_output.rob_dimensions.L23z = XMS3::ROBOT_DIMENSIONS[8] / 1000.0;
    model_params_output.rob_dimensions.L34x = XMS3::ROBOT_DIMENSIONS[9] / 1000.0;
    model_params_output.rob_dimensions.L34y = XMS3::ROBOT_DIMENSIONS[10] / 1000.0;
    model_params_output.rob_dimensions.L34z = XMS3::ROBOT_DIMENSIONS[11] / 1000.0;
    model_params_output.rob_dimensions.L45x = XMS3::ROBOT_DIMENSIONS[12] / 1000.0;
    model_params_output.rob_dimensions.L45y = XMS3::ROBOT_DIMENSIONS[13] / 1000.0;
    model_params_output.rob_dimensions.L45z = XMS3::ROBOT_DIMENSIONS[14] / 1000.0;
    if (model_params_output.axis_num >= 6) {
        //其他轴暂时不加
        model_params_output.rob_dimensions.L56x = XMS3::ROBOT_DIMENSIONS[15] / 1000.0;
        model_params_output.rob_dimensions.L56y = XMS3::ROBOT_DIMENSIONS[16] / 1000.0;
        model_params_output.rob_dimensions.L56z = XMS3::ROBOT_DIMENSIONS[17] / 1000.0;
        model_params_output.rob_dimensions.L67x = XMS3::ROBOT_DIMENSIONS[18] / 1000.0;
        model_params_output.rob_dimensions.L67y = XMS3::ROBOT_DIMENSIONS[19] / 1000.0;
        model_params_output.rob_dimensions.L67z = XMS3::ROBOT_DIMENSIONS[20] / 1000.0;
    }

    return SOLVE_NOERROR;
}

int InitRobot::ReadControlParamsSR3(Control::ControlParams& control_params_output) {
    //控制参数：
    // 1.拖动带宽(Kp,CTRL_BANDWIDTH_SERVO_EXEC)
    control_params_output.m_gain_params.joint_gain_kp = XMS3::CTRL_BANDWIDTH_SERVO_EXEC;

    // 2.拖动阻尼比(zeta,CTRL_ZETA_SERVO_EXEC)
    control_params_output.m_gain_params.joint_damp_zeta = XMS3::CTRL_ZETA_SERVO_EXEC;

    // 3.拖动摩擦力系数(fri_coef,FRICTION_COF_DRAG)
    control_params_output.m_gain_params.friction_cof_servo = XMS3::FRICTION_COF_DRAG;

    //保护参数：
    control_params_output.m_protect_params.max_mode_switch_trq = XMS3::SWITCH_THRESHOLD_OF_TORQUE_CONTROL;

    return SOLVE_NOERROR;
}

int InitRobot::ReadMechanicalParamsSR3(Model::MechanicalParams& mechaincal_params_output) {
    // 1.encoder_offset
    mechaincal_params_output.encoder_offset = XMS3::ENCODER_OFFESET;

    // 2.encoder_resolution
    mechaincal_params_output.encoder_resolution = XMS3::ENCODER_RESOLUTION;

    // 3.m_decel_ratio_high
    mechaincal_params_output.decel_ratio_high = XMS3::REDUCTION_RATIO_NUMERATOR;

    // 4.m_decel_ratio_low
    mechaincal_params_output.decel_ratio_low = XMS3::REDUCTION_RATIO_DENOMINATOR;

    // 5.analog2trq_high
    mechaincal_params_output.analog2trq_high = XMS3::SENSOR_ANALOG_TO_TORQUE_HIGH;

    // 6.analog2trq_low
    mechaincal_params_output.analog2trq_low = XMS3::SENSOR_ANALOG_TO_TORQUE_LOW;

    // 7.analog_bias
    mechaincal_params_output.analog_bias = XMS3::SENSOR_BIAS;

    // 8.sensor_amplify
    mechaincal_params_output.sensor_amplify = XMS3::SENSOR_AMPLIFY;

    // 9.rated_torque
    mechaincal_params_output.rated_torque = XMS3::RATED_TORQUE;

    return SOLVE_NOERROR;
}

int InitRobot::ReadModelParamsSR4(Model::ModelParams& model_params_output) {
    //机器类型
    if (XMS4::ROBOT_NAME == "XMS4-R800-B4G1A4C") {
        model_params_output.mech_type = MechUnitType::SR4_C;
    } else {
        return ROBOTTYPE_ERROR;
    }
    //机器轴数
    model_params_output.axis_num = XMS4::AXIS_NUM;

    //最大负载(kg)
    model_params_output.max_load = XMS4::MAX_LOAD;

    //软限位
    model_params_output.joint_range_max = XMS4::JOINT_RANGE_MAX_CUSTOMIZE;
    model_params_output.joint_range_min = XMS4::JOINT_RANGE_MIN_CUSTOMIZE;
    
    //硬限位
    model_params_output.joint_range_max_new = XMS4::JOINT_RANGE_MAX_NEW;
    model_params_output.joint_range_min_new = XMS4::JOINT_RANGE_MIN_NEW;

    //连杆相关参数

    //基座各参数赋值0
    model_params_output.link_inertia[0].mass = 0;
    model_params_output.link_inertia[0].centroid[0] = 0;
    model_params_output.link_inertia[0].centroid[1] = 0;
    model_params_output.link_inertia[0].centroid[2] = 0;
    model_params_output.link_inertia[0].moment[0] = 0;
    model_params_output.link_inertia[0].moment[1] = 0;
    model_params_output.link_inertia[0].moment[2] = 0;
    model_params_output.link_inertia[0].moment[3] = 0;
    model_params_output.link_inertia[0].moment[4] = 0;
    model_params_output.link_inertia[0].moment[5] = 0;
    model_params_output.link_inertia[0].moment_link[0] = 0;
    model_params_output.link_inertia[0].moment_link[1] = 0;
    model_params_output.link_inertia[0].moment_link[2] = 0;
    model_params_output.link_inertia[0].moment_link[3] = 0;
    model_params_output.link_inertia[0].moment_link[4] = 0;
    model_params_output.link_inertia[0].moment_link[5] = 0;

    //各轴参数赋值
    unsigned int k;
    for (unsigned int i = 1; i < m_jnt_num + 1; i++) {
        k = i - 1;
        model_params_output.link_inertia[i].mass = XMS4::LINK_MASS[k];
        model_params_output.link_inertia[i].centroid[0] = XMS4::LINK_CENTROID[k * 3 + 0] / 1000.0;
        model_params_output.link_inertia[i].centroid[1] = XMS4::LINK_CENTROID[k * 3 + 1] / 1000.0;
        model_params_output.link_inertia[i].centroid[2] = XMS4::LINK_CENTROID[k * 3 + 2] / 1000.0;
        model_params_output.link_inertia[i].moment[0] = XMS4::LINK_MOMENT_OF_INERTIA[k * 6 + 0] / 1000000.0;
        model_params_output.link_inertia[i].moment[1] = XMS4::LINK_MOMENT_OF_INERTIA[k * 6 + 1] / 1000000.0;
        model_params_output.link_inertia[i].moment[2] = XMS4::LINK_MOMENT_OF_INERTIA[k * 6 + 2] / 1000000.0;
        model_params_output.link_inertia[i].moment[3] = XMS4::LINK_MOMENT_OF_INERTIA[k * 6 + 3] / 1000000.0;
        model_params_output.link_inertia[i].moment[4] = XMS4::LINK_MOMENT_OF_INERTIA[k * 6 + 4] / 1000000.0;
        model_params_output.link_inertia[i].moment[5] = XMS4::LINK_MOMENT_OF_INERTIA[k * 6 + 5] / 1000000.0;
        model_params_output.link_inertia[i].moment_link[0] = XMS4::LINK_MOMENT_OF_INERTIA_LOW[k * 6 + 0] / 1000000.0;
        model_params_output.link_inertia[i].moment_link[1] = XMS4::LINK_MOMENT_OF_INERTIA_LOW[k * 6 + 1] / 1000000.0;
        model_params_output.link_inertia[i].moment_link[2] = XMS4::LINK_MOMENT_OF_INERTIA_LOW[k * 6 + 2] / 1000000.0;
        model_params_output.link_inertia[i].moment_link[3] = XMS4::LINK_MOMENT_OF_INERTIA_LOW[k * 6 + 3] / 1000000.0;
        model_params_output.link_inertia[i].moment_link[4] = XMS4::LINK_MOMENT_OF_INERTIA_LOW[k * 6 + 4] / 1000000.0;
        model_params_output.link_inertia[i].moment_link[5] = XMS4::LINK_MOMENT_OF_INERTIA_LOW[k * 6 + 5] / 1000000.0;
    }

    // Coord_Orientation
    model_params_output.joint_type[0] = KDL::Joint::JointType::None;
    model_params_output.joint_type[1] = KDL::Joint::JointType::RotZ;
    model_params_output.joint_type[2] = KDL::Joint::JointType::RotY;
    model_params_output.joint_type[3] = KDL::Joint::JointType::RotY;
    model_params_output.joint_type[4] = KDL::Joint::JointType::RotZ;
    model_params_output.joint_type[5] = KDL::Joint::JointType::RotY;
    model_params_output.joint_type[6] = KDL::Joint::JointType::RotZ;
    model_params_output.coor_orient[2].rot_axis = Model::ModelParams::ROT_Z;
    model_params_output.coor_orient[2].rot_angle = PI;
    model_params_output.coor_orient[6].rot_axis = Model::ModelParams::ROT_Z;
    model_params_output.coor_orient[6].rot_angle = PI;

    // rob_dimension数据结构转换

    model_params_output.rob_dimensions.L01x = XMS4::ROBOT_DIMENSIONS[0] / 1000.0;
    model_params_output.rob_dimensions.L01y = XMS4::ROBOT_DIMENSIONS[1] / 1000.0;
    model_params_output.rob_dimensions.L01z = XMS4::ROBOT_DIMENSIONS[2] / 1000.0;
    model_params_output.rob_dimensions.L12x = XMS4::ROBOT_DIMENSIONS[3] / 1000.0;
    model_params_output.rob_dimensions.L12y = XMS4::ROBOT_DIMENSIONS[4] / 1000.0;
    model_params_output.rob_dimensions.L12z = XMS4::ROBOT_DIMENSIONS[5] / 1000.0;
    model_params_output.rob_dimensions.L23x = XMS4::ROBOT_DIMENSIONS[6] / 1000.0;
    model_params_output.rob_dimensions.L23y = XMS4::ROBOT_DIMENSIONS[7] / 1000.0;
    model_params_output.rob_dimensions.L23z = XMS4::ROBOT_DIMENSIONS[8] / 1000.0;
    model_params_output.rob_dimensions.L34x = XMS4::ROBOT_DIMENSIONS[9] / 1000.0;
    model_params_output.rob_dimensions.L34y = XMS4::ROBOT_DIMENSIONS[10] / 1000.0;
    model_params_output.rob_dimensions.L34z = XMS4::ROBOT_DIMENSIONS[11] / 1000.0;
    model_params_output.rob_dimensions.L45x = XMS4::ROBOT_DIMENSIONS[12] / 1000.0;
    model_params_output.rob_dimensions.L45y = XMS4::ROBOT_DIMENSIONS[13] / 1000.0;
    model_params_output.rob_dimensions.L45z = XMS4::ROBOT_DIMENSIONS[14] / 1000.0;
    if (model_params_output.axis_num >= 6) {
        //其他轴暂时不加
        model_params_output.rob_dimensions.L56x = XMS4::ROBOT_DIMENSIONS[15] / 1000.0;
        model_params_output.rob_dimensions.L56y = XMS4::ROBOT_DIMENSIONS[16] / 1000.0;
        model_params_output.rob_dimensions.L56z = XMS4::ROBOT_DIMENSIONS[17] / 1000.0;
        model_params_output.rob_dimensions.L67x = XMS4::ROBOT_DIMENSIONS[18] / 1000.0;
        model_params_output.rob_dimensions.L67y = XMS4::ROBOT_DIMENSIONS[19] / 1000.0;
        model_params_output.rob_dimensions.L67z = XMS4::ROBOT_DIMENSIONS[20] / 1000.0;
    }

    return SOLVE_NOERROR;
}

int InitRobot::ReadControlParamsSR4(Control::ControlParams& control_params_output) {
    //控制参数：
    // 1.拖动带宽(Kp,CTRL_BANDWIDTH_SERVO_EXEC)
    control_params_output.m_gain_params.joint_gain_kp = XMS4::CTRL_BANDWIDTH_SERVO_EXEC;

    // 2.拖动阻尼比(zeta,CTRL_ZETA_SERVO_EXEC)
    control_params_output.m_gain_params.joint_damp_zeta = XMS4::CTRL_ZETA_SERVO_EXEC;

    // 3.拖动摩擦力系数(fri_coef,FRICTION_COF_DRAG)
    control_params_output.m_gain_params.friction_cof_servo = XMS4::FRICTION_COF_DRAG;

    //保护参数：
    control_params_output.m_protect_params.max_mode_switch_trq = XMS4::SWITCH_THRESHOLD_OF_TORQUE_CONTROL;

    return SOLVE_NOERROR;
}

int InitRobot::ReadMechanicalParamsSR4(Model::MechanicalParams& mechaincal_params_output) {
    // 1.encoder_offset
    mechaincal_params_output.encoder_offset = XMS4::ENCODER_OFFESET;

    // 2.encoder_resolution
    mechaincal_params_output.encoder_resolution = XMS4::ENCODER_RESOLUTION;

    // 3.m_decel_ratio_high
    mechaincal_params_output.decel_ratio_high = XMS4::REDUCTION_RATIO_NUMERATOR;

    // 4.m_decel_ratio_low
    mechaincal_params_output.decel_ratio_low = XMS4::REDUCTION_RATIO_DENOMINATOR;

    // 5.analog2trq_high
    mechaincal_params_output.analog2trq_high = XMS4::SENSOR_ANALOG_TO_TORQUE_HIGH;

    // 6.analog2trq_low
    mechaincal_params_output.analog2trq_low = XMS4::SENSOR_ANALOG_TO_TORQUE_LOW;

    // 7.analog_bias
    mechaincal_params_output.analog_bias = XMS4::SENSOR_BIAS;

    // 8.sensor_amplify
    mechaincal_params_output.sensor_amplify = XMS4::SENSOR_AMPLIFY;

    // 9.rated_torque
    mechaincal_params_output.rated_torque = XMS4::RATED_TORQUE;

    return SOLVE_NOERROR;
}

int InitRobot::ReadModelParamsSR5(Model::ModelParams& model_params_output) {
    //机器类型
    if (XMS5::ROBOT_NAME == "XMS5-R800-B4G1A4C") {
        model_params_output.mech_type = MechUnitType::SR5_C;
    } else {
        return ROBOTTYPE_ERROR;
    }
    //机器轴数
    model_params_output.axis_num = XMS5::AXIS_NUM;

    //最大负载(kg)
    model_params_output.max_load = XMS5::MAX_LOAD;

    //软限位
    model_params_output.joint_range_max = XMS5::JOINT_RANGE_MAX_CUSTOMIZE;
    model_params_output.joint_range_min = XMS5::JOINT_RANGE_MIN_CUSTOMIZE;
    
    //硬限位
    model_params_output.joint_range_max_new = XMS5::JOINT_RANGE_MAX_NEW;
    model_params_output.joint_range_min_new = XMS5::JOINT_RANGE_MIN_NEW;
    //连杆相关参数

    //基座各参数赋值0
    model_params_output.link_inertia[0].mass = 0;
    model_params_output.link_inertia[0].centroid[0] = 0;
    model_params_output.link_inertia[0].centroid[1] = 0;
    model_params_output.link_inertia[0].centroid[2] = 0;
    model_params_output.link_inertia[0].moment[0] = 0;
    model_params_output.link_inertia[0].moment[1] = 0;
    model_params_output.link_inertia[0].moment[2] = 0;
    model_params_output.link_inertia[0].moment[3] = 0;
    model_params_output.link_inertia[0].moment[4] = 0;
    model_params_output.link_inertia[0].moment[5] = 0;
    model_params_output.link_inertia[0].moment_link[0] = 0;
    model_params_output.link_inertia[0].moment_link[1] = 0;
    model_params_output.link_inertia[0].moment_link[2] = 0;
    model_params_output.link_inertia[0].moment_link[3] = 0;
    model_params_output.link_inertia[0].moment_link[4] = 0;
    model_params_output.link_inertia[0].moment_link[5] = 0;

    //各轴参数赋值
    unsigned int k;
    for (unsigned int i = 1; i < m_jnt_num + 1; i++) {
        k = i - 1;
        model_params_output.link_inertia[i].mass = XMS5::LINK_MASS[k];
        model_params_output.link_inertia[i].centroid[0] = XMS5::LINK_CENTROID[k * 3 + 0] / 1000.0;
        model_params_output.link_inertia[i].centroid[1] = XMS5::LINK_CENTROID[k * 3 + 1] / 1000.0;
        model_params_output.link_inertia[i].centroid[2] = XMS5::LINK_CENTROID[k * 3 + 2] / 1000.0;
        model_params_output.link_inertia[i].moment[0] = XMS5::LINK_MOMENT_OF_INERTIA[k * 6 + 0] / 1000000.0;
        model_params_output.link_inertia[i].moment[1] = XMS5::LINK_MOMENT_OF_INERTIA[k * 6 + 1] / 1000000.0;
        model_params_output.link_inertia[i].moment[2] = XMS5::LINK_MOMENT_OF_INERTIA[k * 6 + 2] / 1000000.0;
        model_params_output.link_inertia[i].moment[3] = XMS5::LINK_MOMENT_OF_INERTIA[k * 6 + 3] / 1000000.0;
        model_params_output.link_inertia[i].moment[4] = XMS5::LINK_MOMENT_OF_INERTIA[k * 6 + 4] / 1000000.0;
        model_params_output.link_inertia[i].moment[5] = XMS5::LINK_MOMENT_OF_INERTIA[k * 6 + 5] / 1000000.0;
        model_params_output.link_inertia[i].moment_link[0] = XMS5::LINK_MOMENT_OF_INERTIA_LOW[k * 6 + 0] / 1000000.0;
        model_params_output.link_inertia[i].moment_link[1] = XMS5::LINK_MOMENT_OF_INERTIA_LOW[k * 6 + 1] / 1000000.0;
        model_params_output.link_inertia[i].moment_link[2] = XMS5::LINK_MOMENT_OF_INERTIA_LOW[k * 6 + 2] / 1000000.0;
        model_params_output.link_inertia[i].moment_link[3] = XMS5::LINK_MOMENT_OF_INERTIA_LOW[k * 6 + 3] / 1000000.0;
        model_params_output.link_inertia[i].moment_link[4] = XMS5::LINK_MOMENT_OF_INERTIA_LOW[k * 6 + 4] / 1000000.0;
        model_params_output.link_inertia[i].moment_link[5] = XMS5::LINK_MOMENT_OF_INERTIA_LOW[k * 6 + 5] / 1000000.0;
    }

    // Coord_Orientation
    model_params_output.joint_type[0] = KDL::Joint::JointType::None;
    model_params_output.joint_type[1] = KDL::Joint::JointType::RotZ;
    model_params_output.joint_type[2] = KDL::Joint::JointType::RotY;
    model_params_output.joint_type[3] = KDL::Joint::JointType::RotY;
    model_params_output.joint_type[4] = KDL::Joint::JointType::RotZ;
    model_params_output.joint_type[5] = KDL::Joint::JointType::RotY;
    model_params_output.joint_type[6] = KDL::Joint::JointType::RotZ;
    model_params_output.coor_orient[2].rot_axis = Model::ModelParams::ROT_Z;
    model_params_output.coor_orient[2].rot_angle = PI;
    model_params_output.coor_orient[6].rot_axis = Model::ModelParams::ROT_Z;
    model_params_output.coor_orient[6].rot_angle = PI;

    // rob_dimension数据结构转换

    model_params_output.rob_dimensions.L01x = XMS5::ROBOT_DIMENSIONS[0] / 1000.0;
    model_params_output.rob_dimensions.L01y = XMS5::ROBOT_DIMENSIONS[1] / 1000.0;
    model_params_output.rob_dimensions.L01z = XMS5::ROBOT_DIMENSIONS[2] / 1000.0;
    model_params_output.rob_dimensions.L12x = XMS5::ROBOT_DIMENSIONS[3] / 1000.0;
    model_params_output.rob_dimensions.L12y = XMS5::ROBOT_DIMENSIONS[4] / 1000.0;
    model_params_output.rob_dimensions.L12z = XMS5::ROBOT_DIMENSIONS[5] / 1000.0;
    model_params_output.rob_dimensions.L23x = XMS5::ROBOT_DIMENSIONS[6] / 1000.0;
    model_params_output.rob_dimensions.L23y = XMS5::ROBOT_DIMENSIONS[7] / 1000.0;
    model_params_output.rob_dimensions.L23z = XMS5::ROBOT_DIMENSIONS[8] / 1000.0;
    model_params_output.rob_dimensions.L34x = XMS5::ROBOT_DIMENSIONS[9] / 1000.0;
    model_params_output.rob_dimensions.L34y = XMS5::ROBOT_DIMENSIONS[10] / 1000.0;
    model_params_output.rob_dimensions.L34z = XMS5::ROBOT_DIMENSIONS[11] / 1000.0;
    model_params_output.rob_dimensions.L45x = XMS5::ROBOT_DIMENSIONS[12] / 1000.0;
    model_params_output.rob_dimensions.L45y = XMS5::ROBOT_DIMENSIONS[13] / 1000.0;
    model_params_output.rob_dimensions.L45z = XMS5::ROBOT_DIMENSIONS[14] / 1000.0;
    if (model_params_output.axis_num >= 6) {
        //其他轴暂时不加
        model_params_output.rob_dimensions.L56x = XMS5::ROBOT_DIMENSIONS[15] / 1000.0;
        model_params_output.rob_dimensions.L56y = XMS5::ROBOT_DIMENSIONS[16] / 1000.0;
        model_params_output.rob_dimensions.L56z = XMS5::ROBOT_DIMENSIONS[17] / 1000.0;
        model_params_output.rob_dimensions.L67x = XMS5::ROBOT_DIMENSIONS[18] / 1000.0;
        model_params_output.rob_dimensions.L67y = XMS5::ROBOT_DIMENSIONS[19] / 1000.0;
        model_params_output.rob_dimensions.L67z = XMS5::ROBOT_DIMENSIONS[20] / 1000.0;
    }

    return SOLVE_NOERROR;
}

int InitRobot::ReadControlParamsSR5(Control::ControlParams& control_params_output) {
    //控制参数：
    // 1.拖动带宽(Kp,CTRL_BANDWIDTH_SERVO_EXEC)
    control_params_output.m_gain_params.joint_gain_kp = XMS5::CTRL_BANDWIDTH_SERVO_EXEC;

    // 2.拖动阻尼比(zeta,CTRL_ZETA_SERVO_EXEC)
    control_params_output.m_gain_params.joint_damp_zeta = XMS5::CTRL_ZETA_SERVO_EXEC;

    // 3.拖动摩擦力系数(fri_coef,FRICTION_COF_DRAG)
    control_params_output.m_gain_params.friction_cof_servo = XMS5::FRICTION_COF_DRAG;

    //保护参数：
    control_params_output.m_protect_params.max_mode_switch_trq = XMS5::SWITCH_THRESHOLD_OF_TORQUE_CONTROL;

    return SOLVE_NOERROR;
}

int InitRobot::ReadMechanicalParamsSR5(Model::MechanicalParams& mechaincal_params_output) {
    // 1.encoder_offset
    mechaincal_params_output.encoder_offset = XMS5::ENCODER_OFFESET;

    // 2.encoder_resolution
    mechaincal_params_output.encoder_resolution = XMS5::ENCODER_RESOLUTION;

    // 3.m_decel_ratio_high
    mechaincal_params_output.decel_ratio_high = XMS5::REDUCTION_RATIO_NUMERATOR;

    // 4.m_decel_ratio_low
    mechaincal_params_output.decel_ratio_low = XMS5::REDUCTION_RATIO_DENOMINATOR;

    // 5.analog2trq_high
    mechaincal_params_output.analog2trq_high = XMS5::SENSOR_ANALOG_TO_TORQUE_HIGH;

    // 6.analog2trq_low
    mechaincal_params_output.analog2trq_low = XMS5::SENSOR_ANALOG_TO_TORQUE_LOW;

    // 7.analog_bias
    mechaincal_params_output.analog_bias = XMS5::SENSOR_BIAS;

    // 8.sensor_amplify
    mechaincal_params_output.sensor_amplify = XMS5::SENSOR_AMPLIFY;

    // 9.rated_torque
    mechaincal_params_output.rated_torque = XMS5::RATED_TORQUE;

    return SOLVE_NOERROR;
}

}  // namespace RokaeApi
