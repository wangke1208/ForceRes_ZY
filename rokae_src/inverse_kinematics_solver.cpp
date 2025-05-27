/**
 * Copyright(C) 2024 Rokae Technology Co., Ltd.
 * All Rights Reserved.
 *
 * Information in this file is the intellectual property of Rokae Technology Co., Ltd,
 * And may contains trade secrets that must be stored and viewed confidentially.
 *
 * @file: inverse_kinematics_solver.cpp
 * @author: wangke
 * @date: 2025/5/26
 * @brief: 逆运动学求解接口
 */

#include "inverse_kinematics_solver.hpp"

using namespace std;
namespace RokaeApi {
namespace Model {
inverse_kinematics_solver::inverse_kinematics_solver(const KDL::Chain& chain, const ModelParams& model_param)
    : m_chain(chain), m_model_param(model_param), m_joint_num(chain.getNrOfJoints()) {
    m_max_joint.resize(m_joint_num);  //初始化最大角度
    m_min_joint.resize(m_joint_num);  //初始化最小角度

    //模型参数赋值
    m_max_joint = model_param.joint_range_max_new;
    m_min_joint = model_param.joint_range_min_new;
    m_rob_dim = model_param.rob_dimensions;

    m_fkpos_ptr = new KDL::ChainFkSolverPos_recursive(m_chain);

    m_length_max = m_rob_dim.L12z + m_rob_dim.L23z + m_rob_dim.L45z + m_rob_dim.L67z;

    m_d_mm = m_rob_dim.L34x;  //肘部偏移
    m_d_bs = m_rob_dim.L12z;
    m_d_se = std::sqrt(m_rob_dim.L23z * m_rob_dim.L23z + m_d_mm * m_d_mm);
    m_d_ew = std::sqrt(m_rob_dim.L45z * m_rob_dim.L45z + m_d_mm * m_d_mm);
    m_d_wt = m_rob_dim.L67z;

    Lbs0(2) = m_d_bs;  //  Lbs0 = (0,0, 0.0, m_d_bs)
    Lse3(0) = m_d_mm;
    Lse3(1) = -m_rob_dim.L23z;  //  Lse3 = (m_d_mm, -m_rob_dim.L23z, 0.0)
    Lew4(0) = -m_d_mm;
    Lew4(2) = m_rob_dim.L45z;  //  Lew4 = (-m_d_mm, 0.0, m_rob_dim.L45z)
    Lwt7(2) = m_d_wt;          //  Lwt7 = (0,0, 0.0, m_d_wt)

    //初始角度
    xend_origin = KDL::Vector(0, 0, m_rob_dim.L12z + m_rob_dim.L23z + m_rob_dim.L45z + m_rob_dim.L67z);
    R7_0_offset = Rotation::RotX(-PI / 2) * Rotation::RotX(PI / 2) * Rotation::RotX(-PI / 2) * Rotation::RotX(PI / 2) *
                  Rotation::RotX(-PI / 2) * Rotation::RotX(PI / 2);
    xw_origin = xend_origin - R7_0_offset * Lwt7;
    xsw_origin = xw_origin - Lbs0;
    d_xsw_origin = xsw_origin.Norm();
    m_q4_offset = std::acos((d_xsw_origin * d_xsw_origin - m_d_se * m_d_se - m_d_ew * m_d_ew) / (2 * m_d_se * m_d_ew));  //初始偏置角度
};

inverse_kinematics_solver::~inverse_kinematics_solver(){
    delete m_fkpos_ptr;
}

IkSolveRes inverse_kinematics_solver::CartToJnt(const KDL::JntArray& curJnt_origin, const GeneralizedFrame& target_Flan,
                                                KDL::JntArray& OutJointPose) {
    double xabs = fabs(target_Flan.frame.p.x());
    double yabs = fabs(target_Flan.frame.p.y());
    double zabs = fabs(target_Flan.frame.p.z());
    if (xabs * xabs + yabs * yabs + zabs * zabs > m_length_max * m_length_max + EPSILON3) {
        return IkSolveRes::TARGET_OUT_OF_RANGE;
    }

    //===========前处理，将初始角度转换到[-pi,pi] ===========//
    if (curJnt_origin.rows() != 7) {
        return IkSolveRes::INPUT_DATA_ERROR;
    }

    if (IsSingular(curJnt_origin)) {
        return IkSolveRes::INPUT_DATA_SINGULAR;
    }

    KDL::JntArray curJntPose(7);
    int axis_scope[7] = {0};
    for (int i = 0; i < 7; i++) {
        curJntPose(i) = curJnt_origin(i);
        if (curJnt_origin(i) > KDL::PI) {
            axis_scope[i] = ceil((curJnt_origin(i) - KDL::PI) / (2 * KDL::PI));
            curJntPose(i) -= axis_scope[i] * 2 * KDL::PI;
        } else if (curJnt_origin(i) < -KDL::PI) {
            axis_scope[i] = floor((curJnt_origin(i) + KDL::PI) / (2 * KDL::PI));
            curJntPose(i) -= axis_scope[i] * 2 * KDL::PI;
        }
    }

    //===========Begin of The Solver Core===========//
    KDL::Frame curFlanPose;
    m_fkpos_ptr->JntToCart(curJntPose, curFlanPose);

    Conf_xMate conf_xmate;
    double psi;
    if (false == Solve_CurPsi_Conf(curJntPose, curFlanPose, conf_xmate, psi)) {
        return IkSolveRes::OTHER_ERROR;
    }

    Q4_ABC q4_abc;
    int res = Compute_Q4_ABC(target_Flan.frame, conf_xmate, q4_abc);
    if (res != 0) {
        return IkSolveRes::OTHER_ERROR;
    }

    KDL::JntArray OutJointPose_temp(7);
    Solve_Jnt(curJntPose, conf_xmate, target_Flan.psi, q4_abc, OutJointPose_temp);
    //===========End of The Solver Core===========//
    bool has_nan = false;
    for (int i = 0; i < 7; ++i) {
        if (true == std::isnan(OutJointPose_temp(i))) {
            has_nan = true;
            break;
        }
    }

    if (has_nan == true) {
        double a, b, c, d;
        target_Flan.frame.M.GetQuaternion(a, b, c, d);
        return IkSolveRes::OTHER_ERROR;
    }

    //围绕2*PI为周期,将角度进行后处理并输出
    for (int i = 0; i < 7; ++i) {
        double tempQ = 0;
        tempQ = fabs(OutJointPose_temp(i) - curJntPose(i));
        if (curJnt_origin(i) > KDL::PI) {
            if (tempQ < 0.5 * KDL::PI) {
                OutJointPose(i) = OutJointPose_temp(i) + axis_scope[i] * 2 * KDL::PI;
            } else {
                //如 InitJointPoseTemp=-160°和result=160°, 之间只相差40°符合, 不是320°
                if ((OutJointPose_temp(i) - curJntPose(i)) > KDL::PI) {
                    OutJointPose(i) = OutJointPose_temp(i) + (axis_scope[i] - 1) * 2 * KDL::PI;
                } else if ((OutJointPose_temp(i) - curJntPose(i)) < -KDL::PI) {
                    OutJointPose(i) = OutJointPose_temp(i) + (axis_scope[i] + 1) * 2 * KDL::PI;
                } else {
                    return IkSolveRes::TOO_LARGE_STEP;
                }
            }
        } else if (curJnt_origin(i) < -KDL::PI) {
            if (tempQ < 0.5 * KDL::PI) {
                OutJointPose(i) = OutJointPose_temp(i) + axis_scope[i] * 2 * KDL::PI;
            } else {
                if ((OutJointPose_temp(i) - curJntPose(i)) > KDL::PI) {
                    OutJointPose(i) = OutJointPose_temp(i) + (axis_scope[i] - 1) * 2 * KDL::PI;
                } else if ((OutJointPose_temp(i) - curJntPose(i)) < -KDL::PI) {
                    OutJointPose(i) = OutJointPose_temp(i) + (axis_scope[i] + 1) * 2 * KDL::PI;
                } else {
                    return IkSolveRes::TOO_LARGE_STEP;
                }
            }
        } else {
            if (tempQ < 0.5 * KDL::PI) {
                OutJointPose(i) = OutJointPose_temp(i);
            } else {
                if ((OutJointPose_temp(i) - curJntPose(i)) > KDL::PI) {
                    OutJointPose(i) = OutJointPose_temp(i) - 2 * KDL::PI;
                } else if ((OutJointPose_temp(i) - curJntPose(i)) < -KDL::PI) {
                    OutJointPose(i) = OutJointPose_temp(i) + 2 * KDL::PI;
                } else {
                    double a, b, c, d;
                    curFlanPose.M.GetQuaternion(a, b, c, d);
                    return IkSolveRes::TOO_LARGE_STEP;
                }
            }
        }
    }
    //检查得到的最近解是否在各轴运动范围内
    for (unsigned int i = 0; i < 7; i++) {
        if ((OutJointPose(i) < m_min_joint[i] + EPSILON8) || (OutJointPose(i) > m_max_joint[i] - EPSILON8)) {
            KDL::JntArray temp(7);
            temp.data << axis_scope[0], axis_scope[1], axis_scope[2], axis_scope[3], axis_scope[4], axis_scope[5], axis_scope[6];
            return IkSolveRes::JOINT_OVER_LIMIT;
        }
    }
    return IkSolveRes::SUCCESS;
}

bool inverse_kinematics_solver::IsSingular(const KDL::JntArray& q) {
    //如果二轴、四轴或六轴处于奇异位置
    // if(KDL::Equal(q(1), 0.0, KDL::PI/360.0) or KDL::Equal(q(3), 0.0, KDL::PI/360.0) or KDL::Equal(q(5), 0.0, KDL::PI/360.0)){
    if ((KDL::Equal(q(1), 0.0, KDL::PI / 360.0) && KDL::Equal(std::cos(q(2)), 0.0, KDL::PI / 360.0)) ||
        KDL::Equal(q(3), 0.0, KDL::PI / 360.0) ||
        (KDL::Equal(q(5), 0.0, KDL::PI / 360.0) && KDL::Equal(std::cos(q(4)), 0.0, KDL::PI / 360.0)) ||
        (KDL::Equal(q(1), 0.0, KDL::PI / 360.0) && KDL::Equal(q(5), 0.0, KDL::PI / 360.0))) {
        return true;
    }
    // KDL::Frame wrist;
    // m_fkpos->JntToCart(q, wrist, 5);
    // // 如果腕心点在基坐标系的Z轴上
    // if(KDL::Equal(wrist.p.x(), 0.0, EPSILON4) and KDL::Equal(wrist.p.y(), 0.0, EPSILON4)){
    //     return true;
    // }
    return false;
}

bool inverse_kinematics_solver::Solve_CurPsi_Conf(const KDL::JntArray& cur_Jnt, const KDL::Frame& cur_Flan,
                                                  Conf_xMate& conf_xmate, double& psi) {
    conf_xmate.cf2 = sign(cur_Jnt(1));
    conf_xmate.cf6 = sign(cur_Jnt(5));

    // 4轴的conf
    conf_xmate.cf4 = sign(cur_Jnt(3));  //判断4轴是否大于0
    if (m_d_mm > 0) {
        conf_xmate.cf4_offset = sign(cur_Jnt(3) - m_q4_offset);
    } else {
        conf_xmate.cf4_offset = sign(cur_Jnt(3) + m_q4_offset);
    }

    //初始位置姿态
    Rotation Rd70_temp = cur_Flan.M;
    KDL::Vector Xd70_temp = cur_Flan.p;
    KDL::Vector Xsw0_temp = Xd70_temp - Lbs0 - Rd70_temp * Lwt7;
    KDL::Vector Usw0_temp = Xsw0_temp / Xsw0_temp.Norm();

    /*
     * =========== 计算q1_0,q2_0和R3_0 ===========
     */
    double q1_0[2], q2_0[2];
    //==== 计算q2_0 ====
    double a = -(Lse3(0) + std::cos(cur_Jnt(3)) * Lew4(0) + std::sin(cur_Jnt(3)) * Lew4(2));
    double b = -(Lse3(1) + std::sin(cur_Jnt(3)) * Lew4(0) - std::cos(cur_Jnt(3)) * Lew4(2));
    double rou = std::sqrt(a * a + b * b);
    double div = Xsw0_temp(2) / rou;
    // acos边界保护
    if (fabs(div) > (1.0 - EPSILON15)) {
        div = sign(div);
    }
    // atan2边界保护
    if (fabs(a) < EPSILON15 && fabs(b) < EPSILON15) {
        return false;
    }
    q2_0[0] = std::atan2(a, b) + std::acos(div);  // DLOG(ERROR)<< " theta1= "<<std::atan2(a, b)<< " theta2= "<<std::acos(div);
    q2_0[1] = std::atan2(a, b) - std::acos(div);
    int index_q20 = 2;  //两组解若相等, 只取一组, 否则==2
    if (q2_0[0] == q2_0[1]) {
        index_q20 = 1;
    }
    for (int i = 0; i < index_q20; i++) {
        double temp = -a * std::cos(q2_0[i]) + b * std::sin(q2_0[i]);
        if (fabs(Xsw0_temp(0)) < EPSILON6 && fabs(Xsw0_temp(1)) < EPSILON6) {
            q1_0[i] = 0.0;
        } else {
            q1_0[i] = std::atan2(sign(temp) * Xsw0_temp(1), sign(temp) * Xsw0_temp(0));
        }
    }

    //恒定取肘关节在上面的那个参考构型
    int k = (fabs(q2_0[0]) < fabs(q2_0[1])) ? 0 : 1;
    //==== 计算R30_0 ====
    Rotation R30_0_temp = Rotation::RotZ(q1_0[k]) * Rotation::RotX(-PI / 2) * Rotation::RotZ(q2_0[k]) * Rotation::RotX(PI / 2) *
                          Rotation::RotX(-PI / 2);

    //计算当前psi
    Rotation R30_temp = Rotation::RotZ(cur_Jnt(0)) * Rotation::RotX(-PI / 2) * Rotation::RotZ(cur_Jnt(1)) *
                        Rotation::RotX(PI / 2) * Rotation::RotZ(cur_Jnt(2)) * Rotation::RotX(-PI / 2);
    KDL::Vector Xse = R30_temp * Lse3;  //当前肘位置向量
    KDL::Vector Use = Xse / Xse.Norm();
    KDL::Vector Vn_sew = Use * Usw0_temp;  //当前臂平面法向量
    KDL::Vector Un_sew = Vn_sew / Vn_sew.Norm();

    KDL::Vector Xse_0 = R30_0_temp * Lse3;  //零臂角位时的肘位置向量
    KDL::Vector Use_0 = Xse_0 / Xse_0.Norm();
    KDL::Vector Vn_sew_0 = Use_0 * Usw0_temp;  //零臂角位时的臂平面法向量

    //肘部奇异 (机器人完全伸展) 时, Vn_sew_0.Norm()=0
    if (Vn_sew_0.Norm() < EPSILON8) {
        // continue; //肘部奇异
    }
    KDL::Vector Un_sew_0 = Vn_sew_0 / Vn_sew_0.Norm();
    double U_dot = dot(Un_sew_0, Un_sew);

    if (fabs(fabs(U_dot) - 1.0) <= EPSILON10) {  //数值精度保护
        U_dot = sign(U_dot);
    }
    psi = sign(dot(Un_sew_0 * Un_sew, Xsw0_temp)) * std::acos(U_dot);
    conf_xmate.q2_0 = q2_0[k];
    return true;
}

int inverse_kinematics_solver::Compute_Q4_ABC(const KDL::Frame& tar_Flan, const Conf_xMate& conf_xmate, Q4_ABC& q4_abc) {
    //目标位置姿态
    KDL::Rotation Rd70 = tar_Flan.M;
    KDL::Vector Xd70 = tar_Flan.p;
    KDL::Vector Xsw0 = Xd70 - Lbs0 - Rd70 * Lwt7;
    KDL::Vector Usw0 = Xsw0 / Xsw0.Norm();
    /*
     * =====================计算肘角度q4 =====================
     */
    //计算未考虑偏置的关节角
    double C4 = (pow(Xsw0.Norm(), 2) - m_d_se * m_d_se - m_d_ew * m_d_ew) / (2 * m_d_se * m_d_ew);

    // TODO:ZY 奇异判断是否需要改
    if (fabs(fabs(C4) - 1.0) < EPSILON12) {
        return -8;  // elbow singularity,return zero
    }

    //保持q4区间不变
    //计算实际的q4(TODO:是不是存在边界效应? )
    if (m_d_mm > 0) {
        // case1:正偏置 m_d_mm>0
        q4_abc.q4 = conf_xmate.cf4_offset * std::acos(C4) + m_q4_offset;
    } else {
        // case2:负偏置 m_d_mm<0
        q4_abc.q4 = conf_xmate.cf4_offset * std::acos(C4) - m_q4_offset;
    }

    if (q4_abc.q4 > m_max_joint[3] || q4_abc.q4 < m_min_joint[3]) {
        //超出四轴软限位范围
        return -7;
    }

    //==== 计算q2_0 ====
    double a = -(Lse3(0) + std::cos(q4_abc.q4) * Lew4(0) + std::sin(q4_abc.q4) * Lew4(2));
    double b = -(Lse3(1) + std::sin(q4_abc.q4) * Lew4(0) - std::cos(q4_abc.q4) * Lew4(2));
    double rou = std::sqrt(a * a + b * b);
    double div = Xsw0(2) / rou;
    double temp1 = std::atan2(a, b) + std::acos(div);
    double temp2 = std::atan2(a, b) - std::acos(div);
    double q2_0;

    if (conf_xmate.q2_0 < -10) {
        q2_0 = temp1;
    } else if (conf_xmate.q2_0 > 10) {
        q2_0 = temp2;
    } else {
        q2_0 = (fabs(temp1 - conf_xmate.q2_0) < fabs(temp2 - conf_xmate.q2_0)) ? temp1 : temp2;
    }

    double temp = -a * std::cos(q2_0) + b * std::sin(q2_0);
    double q1_0 = std::atan2(sign(temp) * Xsw0(1), sign(temp) * Xsw0(0));
    KDL::Rotation R30_0 = KDL::Rotation::RotZ(q1_0) * KDL::Rotation::RotX(-KDL::PI / 2) * KDL::Rotation::RotZ(q2_0) *
                          KDL::Rotation::RotX(KDL::PI / 2) * KDL::Rotation::RotX(-KDL::PI / 2);

    //==== compute A, B, C matrixes ====
    //注: 下面的矩阵不都是旋转矩阵, 但矩阵运算规则与旋转矩阵通用,故借用Rotation类
    // 注意不能使用Rotx等空间变换操作, 求逆不等于转置
    KDL::Rotation Usw0_X = KDL::Rotation(0.0, -Usw0(2), Usw0(1), Usw0(2), 0.0, -Usw0(0), -Usw0(1), Usw0(0), 0.0);
    KDL::Rotation As = Usw0_X * R30_0;
    KDL::Rotation Bs_neg = Usw0_X * Usw0_X * R30_0;
    KDL::Rotation Bs = MatrixNeg(Bs_neg);
    KDL::Rotation Usw_m =
        KDL::Rotation(Usw0(0) * Usw0(0), Usw0(0) * Usw0(1), Usw0(0) * Usw0(2), Usw0(1) * Usw0(0), Usw0(1) * Usw0(1),
                      Usw0(1) * Usw0(2), Usw0(2) * Usw0(0), Usw0(2) * Usw0(1), Usw0(2) * Usw0(2));
    KDL::Rotation Cs = Usw_m * R30_0;

    KDL::Rotation R43 = KDL::Rotation::RotZ(q4_abc.q4) * KDL::Rotation::RotX(KDL::PI / 2);

    KDL::Rotation As_t = MatrixTrans(As);
    KDL::Rotation Bs_t = MatrixTrans(Bs);
    KDL::Rotation Cs_t = MatrixTrans(Cs);
    KDL::Rotation R43_t = MatrixTrans(R43);

    KDL::Rotation Aw = R43_t * As_t * Rd70;
    KDL::Rotation Bw = R43_t * Bs_t * Rd70;
    KDL::Rotation Cw = R43_t * Cs_t * Rd70;

    q4_abc.an_q1 = -As(1, 1) * conf_xmate.cf2;
    q4_abc.bn_q1 = -Bs(1, 1) * conf_xmate.cf2;
    q4_abc.cn_q1 = -Cs(1, 1) * conf_xmate.cf2;
    q4_abc.ad_q1 = -As(0, 1) * conf_xmate.cf2;
    q4_abc.bd_q1 = -Bs(0, 1) * conf_xmate.cf2;
    q4_abc.cd_q1 = -Cs(0, 1) * conf_xmate.cf2;

    q4_abc.a_q2 = -As(2, 1);
    q4_abc.b_q2 = -Bs(2, 1);
    q4_abc.c_q2 = -Cs(2, 1);

    q4_abc.an_q3 = As(2, 2) * conf_xmate.cf2;
    q4_abc.bn_q3 = Bs(2, 2) * conf_xmate.cf2;
    q4_abc.cn_q3 = Cs(2, 2) * conf_xmate.cf2;
    q4_abc.ad_q3 = -As(2, 0) * conf_xmate.cf2;
    q4_abc.bd_q3 = -Bs(2, 0) * conf_xmate.cf2;
    q4_abc.cd_q3 = -Cs(2, 0) * conf_xmate.cf2;

    q4_abc.an_q5 = Aw(1, 2) * conf_xmate.cf6;
    q4_abc.bn_q5 = Bw(1, 2) * conf_xmate.cf6;
    q4_abc.cn_q5 = Cw(1, 2) * conf_xmate.cf6;
    q4_abc.ad_q5 = Aw(0, 2) * conf_xmate.cf6;
    q4_abc.bd_q5 = Bw(0, 2) * conf_xmate.cf6;
    q4_abc.cd_q5 = Cw(0, 2) * conf_xmate.cf6;

    q4_abc.a_q6 = Aw(2, 2);
    q4_abc.b_q6 = Bw(2, 2);
    q4_abc.c_q6 = Cw(2, 2);

    q4_abc.an_q7 = Aw(2, 1) * conf_xmate.cf6;
    q4_abc.bn_q7 = Bw(2, 1) * conf_xmate.cf6;
    q4_abc.cn_q7 = Cw(2, 1) * conf_xmate.cf6;
    q4_abc.ad_q7 = -Aw(2, 0) * conf_xmate.cf6;
    q4_abc.bd_q7 = -Bw(2, 0) * conf_xmate.cf6;
    q4_abc.cd_q7 = -Cw(2, 0) * conf_xmate.cf6;

    return 0;
}

void inverse_kinematics_solver::Solve_Jnt(const KDL::JntArray& curJntPose, const Conf_xMate& conf_xmate, const double& targ_Psi,
                                          const Q4_ABC& q4_abc, KDL::JntArray& q) {
    double q1_s = ArcConsineSolver(q4_abc.a_q2, q4_abc.b_q2, q4_abc.c_q2, targ_Psi);

    if (fabs(q1_s - 0.0) < 1e-7) {
        // LogE << "二轴算法奇异";
        q(0) = curJntPose(0);
        q(1) = q1_s * conf_xmate.cf2;
        q(2) = curJntPose(2);
    } else {
        double q0_s =
            ArcTangentSolver(q4_abc.an_q1, q4_abc.bn_q1, q4_abc.cn_q1, q4_abc.ad_q1, q4_abc.bd_q1, q4_abc.cd_q1, targ_Psi);

        if (fabs(q0_s - curJntPose(0)) > 1.5) {
            q(0) = ArcTangentSolver(-q4_abc.an_q1, -q4_abc.bn_q1, -q4_abc.cn_q1, -q4_abc.ad_q1, -q4_abc.bd_q1, -q4_abc.cd_q1,
                                    targ_Psi);

            q(1) = -ArcConsineSolver(q4_abc.a_q2, q4_abc.b_q2, q4_abc.c_q2, targ_Psi) * conf_xmate.cf2;

            q(2) = ArcTangentSolver(-q4_abc.an_q3, -q4_abc.bn_q3, -q4_abc.cn_q3, -q4_abc.ad_q3, -q4_abc.bd_q3, -q4_abc.cd_q3,
                                    targ_Psi);
        } else {
            q(0) = ArcTangentSolver(q4_abc.an_q1, q4_abc.bn_q1, q4_abc.cn_q1, q4_abc.ad_q1, q4_abc.bd_q1, q4_abc.cd_q1, targ_Psi);

            q(1) = ArcConsineSolver(q4_abc.a_q2, q4_abc.b_q2, q4_abc.c_q2, targ_Psi) * conf_xmate.cf2;

            q(2) = ArcTangentSolver(q4_abc.an_q3, q4_abc.bn_q3, q4_abc.cn_q3, q4_abc.ad_q3, q4_abc.bd_q3, q4_abc.cd_q3, targ_Psi);
        }
    }

    q(3) = q4_abc.q4;

    double q5_s = ArcConsineSolver(q4_abc.a_q6, q4_abc.b_q6, q4_abc.c_q6, targ_Psi);

    if (fabs(q5_s - 0.0) < 1e-7) {
        // LogE<<"六轴算法奇异";
        q(4) = curJntPose(4);
        q(5) = q5_s * conf_xmate.cf6;
        q(6) = curJntPose(6);
    } else {
        double q4_s =
            ArcTangentSolver(q4_abc.an_q5, q4_abc.bn_q5, q4_abc.cn_q5, q4_abc.ad_q5, q4_abc.bd_q5, q4_abc.cd_q5, targ_Psi);
        if (fabs(q4_s - curJntPose(4)) > 1.5) {
            q(4) = ArcTangentSolver(-q4_abc.an_q5, -q4_abc.bn_q5, -q4_abc.cn_q5, -q4_abc.ad_q5, -q4_abc.bd_q5, -q4_abc.cd_q5,
                                    targ_Psi);

            q(5) = -ArcConsineSolver(q4_abc.a_q6, q4_abc.b_q6, q4_abc.c_q6, targ_Psi) * conf_xmate.cf6;

            q(6) = ArcTangentSolver(-q4_abc.an_q7, -q4_abc.bn_q7, -q4_abc.cn_q7, -q4_abc.ad_q7, -q4_abc.bd_q7, -q4_abc.cd_q7,
                                    targ_Psi);
        } else {
            q(4) = ArcTangentSolver(q4_abc.an_q5, q4_abc.bn_q5, q4_abc.cn_q5, q4_abc.ad_q5, q4_abc.bd_q5, q4_abc.cd_q5, targ_Psi);

            q(5) = ArcConsineSolver(q4_abc.a_q6, q4_abc.b_q6, q4_abc.c_q6, targ_Psi) * conf_xmate.cf6;

            q(6) = ArcTangentSolver(q4_abc.an_q7, q4_abc.bn_q7, q4_abc.cn_q7, q4_abc.ad_q7, q4_abc.bd_q7, q4_abc.cd_q7, targ_Psi);
        }
    }
}

}  // namespace Model
}  // namespace RokaeApi