/**
 * @file linearize_wip.cpp
 * @author Munzir Zafar
 * @date Feb 25, 2019
 * @brief Compute linearized dynamics of the WIP model krang
 * */

#ifndef KRANG_UTILS__LINEARIZE_WIP_HPP
#define KRANG_UTILS__LINEARIZE_WIP_HPP

#include <dart/dart.hpp>

#include "krang-utils/linearize_wip.hpp"

namespace linearize_wip {

// ==========================================================================
// Get the center of mass of body excluding wheels
Eigen::Vector3d GetBodyCOM(dart::dynamics::SkeletonPtr robot);

// ==========================================================================
// compute linearized dynamics for lqr as well as B matrix for Eso updates
void ComputeLinearizedDynamics(const dart::dynamics::SkeletonPtr robot,
                               const struct ParametersNotFoundInUrdf& params,
                               Eigen::MatrixXd& A, Eigen::MatrixXd& B);

struct ParametersNotFoundInUrdf {
  double rotor_inertia;
  double gear_ratio;
  double wheel_radius;
};

}  // namespace linearize_wip

#endif
