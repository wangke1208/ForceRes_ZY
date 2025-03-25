/**
 * Copyright(C) 2024 Rokae Technology Co., Ltd.
 * All Rights Reserved.
 *
 * Information in this file is the intellectual property of Rokae Technology Co., Ltd,
 * And may contains trade secrets that must be stored and viewed confidentially.
 *
 * @file: data_structure_convert.hpp
 * @author: wangke
 * @date:
 * @brief: 数据类型转化相关的一些工具函数
 */

#ifndef DATA_STRUCTURE_CONVERT_H
#define DATA_STRUCTURE_CONVERT_H
#include <iostream>
#include <../3rd/kdl/jntarray.hpp>
#include <../3rd/kdl/rigidbodyinertia.hpp>
#include <vector>

#include "data_structure_define.hpp"

using namespace RokaeApi::Model;

namespace RokaeApi {

inline KDL::RigidBodyInertia GetKDLloadFromRokaeLoad(const LoadInertia& in) {
    return KDL::RigidBodyInertia(in.m, in.GetCOG(),
                                 KDL::RotationalInertia(in.inertia[0], in.inertia[1], in.inertia[2], in.inertia[3], in.inertia[4], in.inertia[5]));
}

inline int VectorToJntArray(const std::vector<double>& in, KDL::JntArray& out) {
    if (in.size() != out.rows()) return -1;
    for (unsigned int i = 0; i < in.size(); i++) {
        out(i) = in[i];
    }
    return 0;
}

inline int JntArrayToVector(const KDL::JntArray& in, std::vector<double>& out) {
    if (out.size() != in.rows()) return -1;
    for (unsigned int i = 0; i < in.rows(); i++) {
        out[i] = in(i);
    }
    return 0;
}
}  // namespace RokaeApi
#endif