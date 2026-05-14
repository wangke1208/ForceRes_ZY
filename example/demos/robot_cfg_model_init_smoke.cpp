/**
 * 冒烟：验证内嵌机型名初始化与「不存在机型」错误码（与主库 SolverRes 一致）。
 * 仅依赖 example/include 单头 + libforce_res.a，不访问仓库 robot_cfg 目录。
 */
#include "client_demo_macros.h"
#include "rokae_force_controller.hpp"

using namespace RokaeApi::External;

// 与 rokae_header/data_structure_define.hpp 中 SolverRes 保持一致（示例包不暴露该头）
constexpr int kSolveNoError = 0;
constexpr int kErrRobotCfgModelNotFound = -20;

int main() {
    RokaeForce_Deinit();

    int r = RokaeForce_InitByModelName("AR5-3_0.7R-W4C1C5-S2");
    if (r != kSolveNoError) {
        LOG_ERROR("smoke: 合法机型应初始化成功，错误码 {}", r);
        return 1;
    }
    RokaeForce_Deinit();

    // 命名符合 L/R 规则但仓库无此目录 → 内嵌表未命中
    r = RokaeForce_InitByModelName("AR5-5_0.7L-NOT_IN_REPO_FAKE");
    if (r != kErrRobotCfgModelNotFound) {
        LOG_ERROR("smoke: 期望不存在机型返回 {}，实际 {}", kErrRobotCfgModelNotFound, r);
        return 2;
    }

    LOG_INFO("robot_cfg_model_init_smoke: ok");
    return 0;
}
