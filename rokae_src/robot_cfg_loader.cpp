/**
 * Copyright(C) 2024 Rokae Technology Co., Ltd.
 * All Rights Reserved.
 *
 * @file robot_cfg_loader.cpp
 * @brief robot_cfg JSON → RobotConfiguration
 */

#include "rokae_header/robot_cfg_loader.hpp"

#include <cstdlib>
#include <fstream>
#include <regex>
#include <sstream>
#include <string>

#include <boost/property_tree/json_parser.hpp>
#include <boost/property_tree/ptree.hpp>

namespace RokaeApi {
namespace {

namespace pt = boost::property_tree;

void FillDoubleArray(const pt::ptree& parent, const char* key, std::vector<double>& out, size_t expected) {
    out.clear();
    const auto& ch = parent.get_child(key);
    for (const auto& item : ch) {
        out.push_back(item.second.get_value<double>());
    }
    if (out.size() != expected) {
        throw pt::ptree_error("array length mismatch");
    }
}

std::vector<double> ReadDoubleArray(const pt::ptree& parent, const char* key) {
    std::vector<double> v;
    const auto& ch = parent.get_child(key);
    for (const auto& item : ch) {
        v.push_back(item.second.get_value<double>());
    }
    return v;
}

void FillIntArray(const pt::ptree& parent, const char* key, std::vector<int>& out, size_t expected) {
    out.clear();
    const auto& ch = parent.get_child(key);
    for (const auto& item : ch) {
        out.push_back(item.second.get_value<int>());
    }
    if (out.size() != expected) {
        throw pt::ptree_error("array length mismatch");
    }
}

void ApplyArcSevenAxisTemplate(Model::RobotConfiguration& cfg) {
    const unsigned int n = cfg.model_config_params.AXIS_NUM;
    if (n != Model::SEVEN_AXIS_ROBOT) {
        throw pt::ptree_error("only 7-axis ARC templates are supported");
    }
    cfg.model_config_params.IS_WRITST_CROSS = true;
    cfg.model_config_params.JOINT_TYPE = {8, 3, 2, 3, 2, 3, 2, 1};
    cfg.model_config_params.ROT_AXIS.assign(n + 1, 0);
    cfg.model_config_params.ROT_ANGLE.assign(n + 1, 0.0);
    cfg.mechanical_config_params.ENCODER_OFFESET.assign(n, 0);
}

}  // namespace

int ParseMechUnitFromModelName(std::string_view model_name, Model::MechUnitType& out_type) {
    static const std::regex kHandednessRe(R"(_[0-9]+(?:\.[0-9]+)?([LR])(?:[-_]|$))");
    const std::string s(model_name);
    std::smatch m;
    if (!std::regex_search(s, m, kHandednessRe)) {
        return ERROR_ROBOT_CFG_PARSE;
    }
    const char h = m[1].str()[0];
    if (h == 'L') {
        out_type = Model::MechUnitType::AR5C_L;
    } else {
        out_type = Model::MechUnitType::AR5C_R;
    }
    return SOLVE_NOERROR;
}

int LoadRobotConfigurationFromJsonString(std::string_view json_text, Model::RobotConfiguration& out_cfg) {
    try {
        pt::ptree root;
        const std::string json_owned(json_text);
        std::istringstream iss(json_owned);
        pt::read_json(iss, root);

        const auto& conf = root.get_child("CONFIGURATION");
        const unsigned int axes = conf.get<unsigned int>("AXES_NUMBER");
        if (axes != Model::SEVEN_AXIS_ROBOT) {
            return ERROR_ROBOT_CFG_PARSE;
        }
        out_cfg.Resize(axes);

        const double payload = conf.get<double>("PAYLOAD");
        out_cfg.model_config_params.MAX_LOAD = payload;

        if (auto name_opt = root.get_optional<std::string>("ROBOT_NAME")) {
            out_cfg.model_config_params.ROBOT_NAME = *name_opt;
        } else {
            out_cfg.model_config_params.ROBOT_NAME = "UNKNOWN";
        }

        const auto& fc_common = root.get_child("FORCE_CONTROL").get_child("COMMON");
        out_cfg.model_config_params.MAX_LOAD_TCP_LENGTH = fc_common.get<double>("MAX_TCP_LENGTH");

        const auto& kin = root.get_child("KINEMATICS");
        FillDoubleArray(kin, "ROBOT_DIMENSIONS", out_cfg.model_config_params.ROBOT_DIMENSIONS, (axes + 1U) * Model::SIZE_LINK_RD);
        FillDoubleArray(kin, "JOINT_RANGE_MIN_CUSTOMIZE", out_cfg.model_config_params.JOINT_RANGE_MIN_CUSTOMIZE, axes);
        FillDoubleArray(kin, "JOINT_RANGE_MAX_CUSTOMIZE", out_cfg.model_config_params.JOINT_RANGE_MAX_CUSTOMIZE, axes);
        FillDoubleArray(kin, "JOINT_RANGE_MIN_NEW", out_cfg.model_config_params.JOINT_RANGE_MIN_NEW, axes);
        FillDoubleArray(kin, "JOINT_RANGE_MAX_NEW", out_cfg.model_config_params.JOINT_RANGE_MAX_NEW, axes);

        const auto& dyn = root.get_child("DYNAMICS");
        FillDoubleArray(dyn, "LINK_MASS", out_cfg.model_config_params.LINK_MASS, axes);

        {
            std::vector<double> lc = ReadDoubleArray(dyn, "LINK_CENTROID");
            if (lc.size() != axes * Model::SIZE_LINK_CENTROID) {
                throw pt::ptree_error("LINK_CENTROID length");
            }
            for (unsigned i = 0; i < lc.size(); ++i) {
                out_cfg.model_config_params.LINK_CENTROID[i] = lc[i];
            }
        }
        {
            std::vector<double> li = ReadDoubleArray(dyn, "LINK_MOMENT_OF_INERTIA");
            if (li.size() != axes * Model::SIZE_LINK_INERITA) {
                throw pt::ptree_error("LINK_MOMENT_OF_INERTIA length");
            }
            for (unsigned i = 0; i < li.size(); ++i) {
                out_cfg.model_config_params.LINK_MOMENT_OF_INERTIA[i] = li[i];
            }
        }
        {
            std::vector<double> lil = ReadDoubleArray(dyn, "LINK_MOMENT_OF_INERTIA_LOW");
            if (lil.size() != axes * Model::SIZE_LINK_INERITA) {
                throw pt::ptree_error("LINK_MOMENT_OF_INERTIA_LOW length");
            }
            for (unsigned i = 0; i < lil.size(); ++i) {
                out_cfg.model_config_params.LINK_MOMENT_OF_INERTIA_LOW[i] = lil[i];
            }
        }

        const auto& motor = root.get_child("MOTOR");
        FillIntArray(motor, "ENCODER_RESOLUTION", out_cfg.mechanical_config_params.ENCODER_RESOLUTION, axes);
        FillDoubleArray(motor, "RATED_TORQUE", out_cfg.mechanical_config_params.RATED_TORQUE, axes);

        const auto& trans = root.get_child("TRANSMISSION");
        FillDoubleArray(trans, "REDUCTION_RATIO_NUMERATOR", out_cfg.mechanical_config_params.REDUCTION_RATIO_NUMERATOR, axes);
        FillDoubleArray(trans, "REDUCTION_RATIO_DENOMINATOR", out_cfg.mechanical_config_params.REDUCTION_RATIO_DENOMINATOR, axes);

        const auto& js = root.get_child("JOINT_SENSOR");
        FillDoubleArray(js, "SENSOR_ANALOG_TO_TORQUE_HIGH", out_cfg.mechanical_config_params.SENSOR_ANALOG_TO_TORQUE_HIGH, axes);
        FillDoubleArray(js, "SENSOR_ANALOG_TO_TORQUE_LOW", out_cfg.mechanical_config_params.SENSOR_ANALOG_TO_TORQUE_LOW, axes);
        FillDoubleArray(js, "SENSOR_BIAS", out_cfg.mechanical_config_params.SENSOR_BIAS, axes);
        FillDoubleArray(js, "SENSOR_AMPLIFY", out_cfg.mechanical_config_params.SENSOR_AMPLIFY, axes);

        FillDoubleArray(fc_common, "CTRL_BANDWIDTH_SERVO_EXEC", out_cfg.control_config_params.CTRL_BANDWIDTH_SERVO_EXEC, axes);
        FillDoubleArray(fc_common, "CTRL_ZETA_SERVO_EXEC", out_cfg.control_config_params.CTRL_ZETA_SERVO_EXEC, axes);
        FillDoubleArray(fc_common, "FRICTION_COF_DRAG", out_cfg.control_config_params.FRICTION_COF_DRAG, axes);
        FillDoubleArray(fc_common, "FRICTION_COF_IMPED", out_cfg.control_config_params.FRICTION_COF_IMPED, axes);
        FillDoubleArray(fc_common, "SWITCH_THRESHOLD_OF_TORQUE_CONTROL",
                        out_cfg.protect_config_params.SWITCH_THRESHOLD_OF_TORQUE_CONTROL, axes);

        const auto& drag = root.get_child("FORCE_CONTROL").get_child("DRAGGING");
        FillDoubleArray(drag, "ROTATION_STIFFNESS_OF_TRANSLATION_DRAGGING",
                        out_cfg.control_config_params.ROTATION_STIFFNESS_OF_TRANSLATION_DRAGGING, Model::DEFAULT_CART_GAIN_SIZE);
        FillDoubleArray(drag, "ROTATION_DAMPING_OF_TRANSLATION_DRAGGING",
                        out_cfg.control_config_params.ROTATION_DAMPING_OF_TRANSLATION_DRAGGING, Model::DEFAULT_CART_GAIN_SIZE);
        FillDoubleArray(drag, "TRANSLATION_STIFFNESS_OF_ROTATION_DRAGGING",
                        out_cfg.control_config_params.TRANSLATION_STIFFNESS_OF_ROTATION_DRAGGING, Model::DEFAULT_CART_GAIN_SIZE);
        FillDoubleArray(drag, "TRANSLATION_DAMPING_OF_ROTATION_DRAGGING",
                        out_cfg.control_config_params.TRANSLATION_DAMPING_OF_ROTATION_DRAGGING, Model::DEFAULT_CART_GAIN_SIZE);

        const auto& imped = root.get_child("FORCE_CONTROL").get_child("IMPEDANCE");
        FillDoubleArray(imped, "CTRL_BANDWIDTH_SERVO_EXEC_IMPE", out_cfg.control_config_params.CTRL_BANDWIDTH_SERVO_EXEC_IMPE, axes);
        FillDoubleArray(imped, "JOINT_CTRL_DAMP_ZETA", out_cfg.control_config_params.JOINT_IMPEDANCE_CTRL_DAMP_ZETA, axes);
        FillDoubleArray(imped, "CART_CTRL_DAMP_ZETA", out_cfg.control_config_params.CART_IMPEDANCE_CTRL_DAMP_ZETA,
                        Model::DEFAULT_CART_PARAMS_SIZE);

        ApplyArcSevenAxisTemplate(out_cfg);
        return SOLVE_NOERROR;
    } catch (const pt::ptree_error&) {
        return ERROR_ROBOT_CFG_PARSE;
    } catch (const std::exception&) {
        return ERROR_ROBOT_CFG_PARSE;
    }
}

int ResolveRobotCfgJson(const std::string& model_name, std::string& out_json) {
#ifdef FORCE_RES_ROBOT_CFG_DEV_PATH
    if (const char* dev_root = std::getenv("FORCE_RES_ROBOT_CFG_DEV_PATH")) {
        if (dev_root[0] != '\0') {
            std::string path = std::string(dev_root) + "/" + model_name + "/" + model_name + ".cfg";
            std::ifstream in(path, std::ios::in | std::ios::binary);
            if (in) {
                std::ostringstream buffer;
                buffer << in.rdbuf();
                out_json = buffer.str();
                if (!out_json.empty()) {
                    return SOLVE_NOERROR;
                }
            }
        }
    }
#endif
    const std::string_view embedded = RobotCfgEmbedded::GetEmbeddedRobotCfgJson(model_name);
    if (embedded.empty()) {
        return ERROR_ROBOT_CFG_MODEL_NOT_FOUND;
    }
    out_json.assign(embedded.begin(), embedded.end());
    return SOLVE_NOERROR;
}

}  // namespace RokaeApi
