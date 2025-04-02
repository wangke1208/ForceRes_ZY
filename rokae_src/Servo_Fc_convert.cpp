/**
 * Copyright(C) 2024 Rokae Technology Co., Ltd.
 * All Rights Reserved.
 *
 * Information in this file is the intellectual property of Rokae Technology Co., Ltd,
 * And may contains trade secrets that must be stored and viewed confidentially.
 *
 * @file: Servo_Fc_convert.cpp
 * @author: wangke
 * @date: 2025/3/27
 * @brief: 伺服力控转换接口
 */

#include "../rokae_header/Servo_Fc_convert.hpp"

namespace RokaeApi {
/****************************************轴转换类************************************ */
Axis_Convert::Axis_Convert(unsigned int axis_num, const Model::MechanicalParams& mec_params_input) {
    //初始化
    m_axis_num = axis_num;
    m_motorside_encoder_offset.resize(m_axis_num);
    m_motorside_encoder_resolution.resize(m_axis_num);
    m_motorside_reduce_retio.resize(axis_num);
    m_motorside_reduce_retio_high.resize(axis_num);
    m_motorside_reduce_retio_low.resize(axis_num);
    m_rated_torque.resize(m_axis_num);
    m_analog_bias.resize(m_axis_num);
    m_analog2trq_high.resize(m_axis_num);
    m_analog2trq_low.resize(m_axis_num);
    m_analog2trq.resize(m_axis_num);
    m_sensor_amplify.resize(m_axis_num);
    m_sensor_trq.resize(m_axis_num);
    m_jnt_to_encoder_scale.resize(m_axis_num);
    m_encoder_to_jnt_scale.resize(m_axis_num);

    //赋初值
    std::copy(mec_params_input.encoder_offset.cbegin(), mec_params_input.encoder_offset.cend(),
              m_motorside_encoder_offset.begin());
    std::copy(mec_params_input.encoder_resolution.cbegin(), mec_params_input.encoder_resolution.cend(),
              m_motorside_encoder_resolution.begin());
    std::copy(mec_params_input.decel_ratio_high.cbegin(), mec_params_input.decel_ratio_high.cend(),
              m_motorside_reduce_retio_high.begin());
    std::copy(mec_params_input.decel_ratio_low.cbegin(), mec_params_input.decel_ratio_low.cend(),
              m_motorside_reduce_retio_low.begin());
    std::copy(mec_params_input.rated_torque.cbegin(), mec_params_input.rated_torque.cend(), m_rated_torque.begin());
    std::copy(mec_params_input.analog_bias.cbegin(), mec_params_input.analog_bias.cend(), m_analog_bias.begin());
    std::copy(mec_params_input.analog2trq_high.cbegin(), mec_params_input.analog2trq_high.cend(), m_analog2trq_high.begin());
    std::copy(mec_params_input.analog2trq_low.cbegin(), mec_params_input.analog2trq_low.cend(), m_analog2trq_low.begin());
    std::copy(mec_params_input.sensor_amplify.cbegin(), mec_params_input.sensor_amplify.cend(), m_sensor_amplify.begin());

    // std::transform(m_motorside_reduce_retio_high.begin(), m_motorside_reduce_retio_high.end(),
    //                m_motorside_reduce_retio_low.begin(), m_motorside_reduce_retio.begin(),
    //                [](double high, double low) { return high / low; });

    for (unsigned int i = 0; i < m_axis_num; i++) {
        m_motorside_reduce_retio[i] = m_motorside_reduce_retio_high[i] / m_motorside_reduce_retio_low[i];
        m_jnt_to_encoder_scale[i] = static_cast<double>(m_motorside_encoder_resolution[i]) / (2.0 * PI);
        m_encoder_to_jnt_scale[i] = 2.0 * PI / m_motorside_encoder_resolution[i];
        m_analog2trq[i] = m_analog2trq_high[i] / m_analog2trq_low[i];
    }
}

int Axis_Convert::SetEncoderBias(const std::vector<int>& encoder_bias_set) {
    if(encoder_bias_set.size()!= m_axis_num) {
        return SIZE_ERROR; 
    }
    for(unsigned int i = 0; i < m_axis_num; i++) {
        m_motorside_encoder_offset[i] = encoder_bias_set[i]; 
    }
    return SOLVE_NOERROR;
}

/****************************************电机相关************************************ */

int Axis_Convert::GetEncoderValue(const std::vector<double>& jnt_pos_rad, std::vector<int>& encoder_value) {
    int res = SOLVE_NOERROR;
    for (uint32_t i = 0; i < m_axis_num; i++) {
        encoder_value[i] =
            jnt_pos_rad[i] * m_motorside_reduce_retio[i] * m_jnt_to_encoder_scale[i] + m_motorside_encoder_offset[i];
    }
    return res;
}

int Axis_Convert::GetAxisPos(const std::vector<int>& encoder_value, std::vector<double>& jnt_pos_rad) {
    if (encoder_value.size() != m_axis_num) {
        return SIZE_ERROR;
    }
    //临时针对中秒抖动问题加一个保护，编码器突然跳变到0附近，则不更新位置(只针对力矩模式下)
    for (unsigned i = 0; i < m_axis_num; i++) {
        jnt_pos_rad[i] =
            ((encoder_value[i] - m_motorside_encoder_offset[i]) * m_encoder_to_jnt_scale[i] / m_motorside_reduce_retio[i]);
    }
    return SOLVE_NOERROR;
}

int Axis_Convert::GetAxisPos(const std::vector<int>& encoder_value, KDL::JntArray& jnt_pos_rad) {
    if (encoder_value.size() != m_axis_num) {
        return SIZE_ERROR;
    }
    //临时针对中秒抖动问题加一个保护，编码器突然跳变到0附近，则不更新位置(只针对力矩模式下)
    for (unsigned i = 0; i < m_axis_num; i++) {
        jnt_pos_rad(i) =
            ((encoder_value[i] - m_motorside_encoder_offset[i]) * m_encoder_to_jnt_scale[i] / m_motorside_reduce_retio[i]);
    }
    return SOLVE_NOERROR;
}

int Axis_Convert::GetVelRegValueForServo(const std::vector<double>& axis_vel_rad, std::vector<int16_t>& vel_reg_value) {
    if (axis_vel_rad.size() != m_axis_num) {
        return SIZE_ERROR;
    }

    for (uint32_t i = 0; i < m_axis_num; ++i) {
        vel_reg_value[i] = static_cast<int16_t>(axis_vel_rad[i] * m_motorside_reduce_retio[i] * 30 / PI);
    }
    return SOLVE_NOERROR;
}

int Axis_Convert::GetAxisVel(const std::vector<int>& encoder_vel_value, std::vector<double>& jnt_vel_rad) {
    if (jnt_vel_rad.size() != m_axis_num) {
        return SIZE_ERROR;
    }

    for (unsigned int i = 0; i < m_axis_num; i++) {
        jnt_vel_rad[i] = (encoder_vel_value[i] * PI * 2 / 60 / m_motorside_reduce_retio[i]);
    }
    return SOLVE_NOERROR;
}

/****************************************传感器相关************************************ */
int Axis_Convert::GetCobotTrq(const std::vector<int16_t>& analog_ch1, const std::vector<int16_t>& analog_ch2,
                              std::vector<double>& jnt_sensor_feedback) {
    for (uint32_t i = 0; i < m_axis_num; i++) {
        jnt_sensor_feedback[i] =
            ((analog_ch1[i] + analog_ch2[i]) / 2 - m_analog_bias[i]) / 1000.0 * m_analog2trq[i] / m_sensor_amplify[i];
    }

    return SOLVE_NOERROR;
}

int Axis_Convert::SetSensorBias(const std::vector<double>& sensor_bias_set) {
    if (sensor_bias_set.size() != m_axis_num) {
        return SIZE_ERROR;
    }
    for (unsigned int i = 0; i < m_axis_num; i++) {
        if (sensor_bias_set[i] <= 0.0 || sensor_bias_set[i] >= 5000.0) {
            return SENSOR_BIAS_ERROR;
        }
    }
    std::copy(sensor_bias_set.cbegin(), sensor_bias_set.cend(), m_analog_bias.begin());
    return SOLVE_NOERROR;
}

int Axis_Convert::SetSensorLinearity(const std::vector<double>& analog_low_set) {
    if (analog_low_set.size() != m_axis_num) {
        return SIZE_ERROR;
    }
    for (unsigned int i = 0; i < m_axis_num; i++) {
        if (analog_low_set[i] < 1.75 || analog_low_set[i] > 2.75) {
            return SENSOR_LINERALITY_ERROR;
        }
    }
    std::copy(analog_low_set.cbegin(), analog_low_set.cend(), m_analog2trq_low.begin());

    //设置线性度后需要重新设置m_analog2trq
    std::transform(m_analog2trq_high.cbegin(), m_analog2trq_high.cend(), m_analog2trq_low.cbegin(), m_analog2trq.begin(),
                   [](double high, double low) { return high / low; });

    return SOLVE_NOERROR;
}
int Axis_Convert::GetAnalogBias(const KDL::JntArray& trq_gra_jntarray, const std::vector<double>& analog_average,
                                std::vector<double>& analog_bias) {
    if (trq_gra_jntarray.rows() != m_axis_num || analog_average.size() != m_axis_num || analog_bias.size() != m_axis_num) {
        return SIZE_ERROR;
    }
    for (unsigned int i = 0; i < m_axis_num; i++) {
        analog_bias[i] =
            analog_average[i] - (trq_gra_jntarray(i) * m_sensor_amplify[i] * 1000 * m_analog2trq_low[i]) / m_analog2trq_high[i];
        //传感器零点一般不会超过2.5V±50%的误差，如果超过，就意味着传感器失效或负载信息错误。
        if ((analog_bias[i] > 3750) or (analog_bias[i] < 1250)) {
            //辨识失败均返回0
            analog_bias.assign(m_axis_num, 0.0);
            return SENSOR_BIAS_ERROR;
        }
    }
    return SOLVE_NOERROR;
}

/****************************************伺服数据和Fc数据转换类************************************ */
Servo_Fc_Convert::Servo_Fc_Convert(unsigned int axis_num, const Model::MechanicalParams& mec_params_input)
    : Axis_Convert(axis_num, mec_params_input), m_zero_feedforward_trq(axis_num, 0){};

int Servo_Fc_Convert::ServoData2FcInner(const std::vector<int8_t>& pdo_mode_operation_0x6061,
                                        const std::vector<int16_t>& pdo_analog_ch1_0x2401,
                                        const std::vector<int16_t>& pdo_analog_ch2_0x2402,
                                        const std::vector<int16_t>& pdo_trq_feedback_0x2406,
                                        const std::vector<int32_t>& pdo_pos_feedback_0x6064,
                                        const std::vector<int32_t>& pdo_vel_feedback_0x606C,
                                        Control::Servo_To_FcInner& servo_data_fcinner) {
    //检查输入数据是否符合要求
    if (pdo_mode_operation_0x6061.size() != m_axis_num || pdo_analog_ch1_0x2401.size() != m_axis_num ||
        pdo_analog_ch2_0x2402.size() != m_axis_num || pdo_trq_feedback_0x2406.size() != m_axis_num ||
        pdo_pos_feedback_0x6064.size() != m_axis_num || pdo_vel_feedback_0x606C.size() != m_axis_num) {
        return SIZE_ERROR;
    }
    //将伺服的PDO数据转换为FcInner数据
    std::copy(pdo_mode_operation_0x6061.cbegin(), pdo_mode_operation_0x6061.cend(), servo_data_fcinner.mode_operation.begin());
    std::copy(pdo_analog_ch1_0x2401.cbegin(), pdo_analog_ch1_0x2401.cend(), servo_data_fcinner.analog_ch1.begin());
    std::copy(pdo_analog_ch2_0x2402.cbegin(), pdo_analog_ch2_0x2402.cend(), servo_data_fcinner.analog_ch2.begin());
    std::copy(pdo_trq_feedback_0x2406.cbegin(), pdo_trq_feedback_0x2406.cend(), servo_data_fcinner.trq_feedback.begin());
    std::copy(pdo_pos_feedback_0x6064.cbegin(), pdo_pos_feedback_0x6064.cend(), servo_data_fcinner.pos_feedback.begin());
    std::copy(pdo_vel_feedback_0x606C.cbegin(), pdo_vel_feedback_0x606C.cend(), servo_data_fcinner.vel_feedback.begin());

    return SOLVE_NOERROR;
}

void Servo_Fc_Convert::FcData2ServoData(const Control::FcStatusInner& fc_status_inner,
                                        const Control::FcParamsInner* fc_params_inner,
                                        Control::FcInner_To_Servo& fc_inner_servo_data) {
    //整合到m_fc_to_servo
    for (unsigned int i = 0; i < m_axis_num; i++) {
        fc_inner_servo_data.trq_cmd[i] =
            (int16_t)(fc_status_inner.jnt_trq_final_cmd(i) * KDL::sign(m_motorside_reduce_retio[i]) / 1000.0 * 32768.0);
        //前馈力矩为0
        fc_inner_servo_data.trq_feedforward[i] =
            (int16_t)(m_zero_feedforward_trq[i] * 1000 / (m_rated_torque[i] * m_motorside_reduce_retio[i]));
        fc_inner_servo_data.k_p[i] = (int16_t)(fc_params_inner->m_function_params.m_params.at("joint_servo_kp")[i] * 100);
        fc_inner_servo_data.k_d[i] = (int16_t)(fc_params_inner->m_function_params.m_params.at("joint_servo_dmap_kv")[i] * 100);
        fc_inner_servo_data.edb_cof[i] = (int16_t)(2.25 / fabs(m_analog2trq_low[i]) * 100.0);
        fc_inner_servo_data.edb_cof[i] = (fc_inner_servo_data.edb_cof[i] < 90) ? 90 : fc_inner_servo_data.edb_cof[i];
        fc_inner_servo_data.edb_o[i] = (int16_t)((m_analog_bias[i] - 2500) / 1000.0 / 2.25 * m_analog2trq_high[i] * 100);
        fc_inner_servo_data.fric_cof[i] =
            (int16_t)(fc_params_inner->m_function_params.m_params.at("joint_servo_friction")[i] * 100);
        fc_inner_servo_data.jnt_inertia[i] = (int16_t)(fc_status_inner.jnt_inertia(i) * 100);
    }
    return;
}

}  // namespace RokaeApi