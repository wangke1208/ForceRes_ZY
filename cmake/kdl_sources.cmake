# 仅编入力控实际用到的 KDL 子集（Chain + FK + Jac + DynParam + RNE 及依赖），
# 排除 Tree / Path / Trajectory / 各类 IK 与未用求解器，以缩小 libforce_res.a。
# 列表随 3rd/kdl 布局维护；若升级 KDL 后链接失败，按 undefined symbol 补源文件。

set(FORCE_RES_KDL_SOURCES
    "${CMAKE_SOURCE_DIR}/3rd/kdl/articulatedbodyinertia.cpp"
    "${CMAKE_SOURCE_DIR}/3rd/kdl/chain.cpp"
    "${CMAKE_SOURCE_DIR}/3rd/kdl/chaindynparam.cpp"
    "${CMAKE_SOURCE_DIR}/3rd/kdl/chainfksolverpos_recursive.cpp"
    "${CMAKE_SOURCE_DIR}/3rd/kdl/chainidsolver_recursive_newton_euler.cpp"
    "${CMAKE_SOURCE_DIR}/3rd/kdl/chainjnttojacsolver.cpp"
    "${CMAKE_SOURCE_DIR}/3rd/kdl/frameacc.cpp"
    "${CMAKE_SOURCE_DIR}/3rd/kdl/framevel.cpp"
    "${CMAKE_SOURCE_DIR}/3rd/kdl/frames.cpp"
    "${CMAKE_SOURCE_DIR}/3rd/kdl/frames_io.cpp"
    "${CMAKE_SOURCE_DIR}/3rd/kdl/jacobian.cpp"
    "${CMAKE_SOURCE_DIR}/3rd/kdl/jntarray.cpp"
    "${CMAKE_SOURCE_DIR}/3rd/kdl/jntarrayacc.cpp"
    "${CMAKE_SOURCE_DIR}/3rd/kdl/jntarrayvel.cpp"
    "${CMAKE_SOURCE_DIR}/3rd/kdl/jntspaceinertiamatrix.cpp"
    "${CMAKE_SOURCE_DIR}/3rd/kdl/joint.cpp"
    "${CMAKE_SOURCE_DIR}/3rd/kdl/kinfam_io.cpp"
    "${CMAKE_SOURCE_DIR}/3rd/kdl/rigidbodyinertia.cpp"
    "${CMAKE_SOURCE_DIR}/3rd/kdl/rotationalinertia.cpp"
    "${CMAKE_SOURCE_DIR}/3rd/kdl/rotational_interpolation.cpp"
    "${CMAKE_SOURCE_DIR}/3rd/kdl/rotational_interpolation_sa.cpp"
    "${CMAKE_SOURCE_DIR}/3rd/kdl/segment.cpp"
    "${CMAKE_SOURCE_DIR}/3rd/kdl/utilities/error_stack.cxx"
    "${CMAKE_SOURCE_DIR}/3rd/kdl/utilities/ldl_solver_eigen.cpp"
    "${CMAKE_SOURCE_DIR}/3rd/kdl/utilities/svd_eigen_HH.cpp"
    "${CMAKE_SOURCE_DIR}/3rd/kdl/utilities/svd_eigen_Macie.cpp"
    "${CMAKE_SOURCE_DIR}/3rd/kdl/utilities/svd_HH.cpp"
    "${CMAKE_SOURCE_DIR}/3rd/kdl/utilities/utility.cxx"
    "${CMAKE_SOURCE_DIR}/3rd/kdl/utilities/utility_io.cxx")
