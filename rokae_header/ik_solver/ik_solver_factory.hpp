#ifndef ROKAE_HEADER_IK_SOLVER_FACTORY_HPP
#define ROKAE_HEADER_IK_SOLVER_FACTORY_HPP

#include <memory>
#include "rokae_header/ik_solver/ik_solver_base.hpp"
#include "rokae_header/ik_solver/inverse_kinematics_solver.hpp"
#include "rokae_header/ik_solver/inverse_kinematics_cross_solver.hpp"

namespace RokaeApi {
namespace Model {

class IKSolverFactory {
public:
    IKSolverFactory(const KDL::Chain& chain, const Model::ModelParams& model_param);
    ~IKSolverFactory() = default;

    // 返回智能指针，调用者负责生命周期
    std::shared_ptr<IKSolverBase> CreateIkSolverPos();

private:
    KDL::Chain m_chain;
    Model::ModelParams m_model_param;
};

}  // namespace Model
}  // namespace RokaeApi

#endif
