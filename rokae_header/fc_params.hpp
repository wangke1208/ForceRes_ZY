/**
 * Copyright(C) 2024 Rokae Technology Co., Ltd.
 * All Rights Reserved.
 *
 * Information in this file is the intellectual property of Rokae Technology Co., Ltd,
 * And may contains trade secrets that must be stored and viewed confidentially.
 *
 * @file: fc_params.hpp
 * @author: wangke
 * @date: 2025/3/30
 * @brief: 力控参数
 */

#ifndef ROKAE_HEADER_FC_PARAMS_HPP_
#define ROKAE_HEADER_FC_PARAMS_HPP_

#include <unordered_map>
#include <string>
#include <vector>

#define ADD_PARAM_VECTOR(name, default_value, value_size)      \
    std::vector<double> data##name(value_size, default_value); \
    InsertParams(#name, data##name);
namespace RokaeApi {
namespace Control {
struct FcParamsInnerBase {
    std::unordered_map<std::string, std::vector<double>> m_params;
    unsigned int m_jnt_num;
    FcParamsInnerBase(unsigned int jnt_num = 7) : m_jnt_num(jnt_num){};
    virtual ~FcParamsInnerBase(){};
    virtual void InitParams(unsigned int jnt_num){};
    void InsertParams(const std::string& name, const std::vector<double>& param) {
        m_params.insert(std::pair<std::string, std::vector<double>>(name, param));
    };
    inline bool GetParams(const std::string& name, std::vector<double>& param) const {
        auto it = m_params.find(name);
        if (it == m_params.end()) {
            //参数不存在
            return false;
        }
        param = it->second;
        return true;
    }

    inline bool SetParam(const std::string& name, const std::vector<double>& param) {
        auto it = m_params.find(name);
        if (it == m_params.end()) {
            //参数不存在
            return false;
        }
        if (param.size() != it->second.size()) {
            //参数长度不一致
            return false;
        }
        it->second = param;
        return true;
    }

    inline bool SetParam(const std::string& name, const double& param, unsigned int index) {
        auto it = m_params.find(name);
        if (it == m_params.end()) {
            //参数不存在
            return false;
        }
        if (index >= it->second.size()) {
            //参数长度不一致
            return false;
        }
        it->second[index] = param;
        return true;
    }

    inline bool SetZero(const std::string& name) {
        auto it = m_params.find(name);
        if (it == m_params.end()) {
            //参数不存在
            return false;
        }
        std::fill(it->second.begin(), it->second.end(), 0.0);
        return true;
    }
    virtual void ResetParamsToDefault(){};
    FcParamsInnerBase& operator=(const FcParamsInnerBase& other) {
        if (this != &other) {
            m_params = other.m_params;
            m_jnt_num = other.m_jnt_num;
        }
        return *this;
    }
};

struct ProtectParamsInner : public FcParamsInnerBase {
    ProtectParamsInner(unsigned int jnt_num = 7) : FcParamsInnerBase(jnt_num){};
    virtual ~ProtectParamsInner(){};
    void InitParams(unsigned int jnt_num) override {
        m_jnt_num = jnt_num;
        ADD_PARAM_VECTOR(max_joint_stiff, 300.0, jnt_num);
        ADD_PARAM_VECTOR(max_joint_damp, 10.0, jnt_num);
        ADD_PARAM_VECTOR(max_joint_stiff_trq, 60.0, jnt_num);
        ADD_PARAM_VECTOR(max_joint_damp_trq, 20.0, jnt_num);
        ADD_PARAM_VECTOR(max_cart_stiff, 300.0, 6);
        ADD_PARAM_VECTOR(max_cart_damp, 300.0, 6);
        ADD_PARAM_VECTOR(max_cart_stiff_trq, 60.0, 6);
        ADD_PARAM_VECTOR(max_cart_damp_trq, 20.0, 6);
        ADD_PARAM_VECTOR(max_null_stiff, 100.0, 1);
        ADD_PARAM_VECTOR(max_mode_switch_trq, 30.0, jnt_num);
    }
    void ResetParamsToDefault() override;
};
struct FunctionParamsInner : public FcParamsInnerBase {
    FunctionParamsInner(unsigned int jnt_num = 7) : FcParamsInnerBase(jnt_num){};
    virtual ~FunctionParamsInner(){};
    void InitParams(unsigned int jnt_num) override {
        m_jnt_num = jnt_num;
        ADD_PARAM_VECTOR(joint_servo_kp, 1.0, jnt_num);
        ADD_PARAM_VECTOR(joint_servo_dmap_kv, 0.0, jnt_num);
        ADD_PARAM_VECTOR(joint_servo_friction, 0.6, jnt_num)
        ADD_PARAM_VECTOR(joint_stiff, 100.0, jnt_num);
        ADD_PARAM_VECTOR(joint_damp, 10.0, jnt_num);
        ADD_PARAM_VECTOR(cart_stiff, 100.0, 6);
        ADD_PARAM_VECTOR(cart_damp, 10.0, 6);
        ADD_PARAM_VECTOR(null_stiff, 100.0, 1);
        ADD_PARAM_VECTOR(null_damp, 10.0, 1);
        ADD_PARAM_VECTOR(joint_damp_zeta, 0.707, jnt_num);
        ADD_PARAM_VECTOR(cart_damp_zeta, 0.707, 6);
    }
    void ResetParamsToDefault() override {
        std::fill(m_params["joint_servo_kp"].begin(), m_params["joint_servo_kp"].end(), 1.0);
        std::fill(m_params["joint_servo_dmap_kv"].begin(), m_params["joint_servo_dmap_kv"].end(), 0.0);
        std::fill(m_params["joint_servo_friction"].begin(), m_params["joint_servo_friction"].end(), 0.6);
        std::fill(m_params["joint_stiff"].begin(), m_params["joint_stiff"].end(), 100.0);
        std::fill(m_params["joint_damp"].begin(), m_params["joint_damp"].end(), 10.0);
        std::fill(m_params["cart_stiff"].begin(), m_params["cart_stiff"].end(), 100.0);
        std::fill(m_params["cart_damp"].begin(), m_params["cart_damp"].end(), 10.0);
        std::fill(m_params["null_stiff"].begin(), m_params["null_stiff"].end(), 100.0);
        std::fill(m_params["null_damp"].begin(), m_params["null_damp"].end(), 10.0);
        std::fill(m_params["joint_damp_zeta"].begin(), m_params["joint_damp_zeta"].end(), 0.707);
        std::fill(m_params["cart_damp_zeta"].begin(), m_params["cart_damp_zeta"].end(), 0.707);
    }
    void SetFreeDragParams() {
        SetZero("joint_stiff");
        SetZero("joint_damp");
        SetZero("cart_stiff");
        SetZero("cart_damp");
    }

    bool SetTransParams(const std::vector<double>& trans_stiff, const std::vector<double>& trans_damp) {
        if (trans_stiff.size() != 3 || trans_damp.size() != 3) {
            return false;
        }
        return FixedX(trans_stiff[0], trans_damp[0]) && FixedY(trans_stiff[1], trans_damp[1]) &&
               FixedZ(trans_stiff[2], trans_damp[2]);
    }

    bool SetRotParams(const std::vector<double>& rot_stiff, const std::vector<double>& rot_damp) {
        if (rot_stiff.size() != 3 || rot_damp.size() != 3) {
            return false;
        }
        return FixedA(rot_stiff[0], rot_damp[0]) && FixedB(rot_stiff[1], rot_damp[1]) && FixedC(rot_stiff[2], rot_damp[2]);
    }
    bool FixedX(double stiff, double damp) { return SetParam("cart_stiff", stiff, 0) && SetParam("cart_damp", damp, 0); }
    bool FixedY(double stiff, double damp) { return SetParam("cart_stiff", stiff, 1) && SetParam("cart_damp", damp, 1); }
    bool FixedZ(double stiff, double damp) { return SetParam("cart_stiff", stiff, 2) && SetParam("cart_damp", damp, 2); }
    bool FixedA(double stiff, double damp) { return SetParam("cart_stiff", stiff, 3) && SetParam("cart_damp", damp, 3); }
    bool FixedB(double stiff, double damp) { return SetParam("cart_stiff", stiff, 4) && SetParam("cart_damp", damp, 4); }
    bool FixedC(double stiff, double damp) { return SetParam("cart_stiff", stiff, 5) && SetParam("cart_damp", damp, 5); }
};
struct FcParamsInner {
    ProtectParamsInner m_protect_params;
    FunctionParamsInner m_function_params;
    FcParamsInner(unsigned int jnt_num) : m_protect_params(jnt_num), m_function_params(jnt_num){
        m_protect_params.InitParams(jnt_num);
        m_function_params.InitParams(jnt_num);
    };
    ~FcParamsInner(){};
};
}
}
#endif