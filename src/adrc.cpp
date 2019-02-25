/**
 * @file adrc.cpp
 * @author Munzir Zafar
 * @date Sept 21, 2018
 * @brief This file contains helper functions pertaining to the adrc
 * implementation of balancing
 */

#include "krang-utils/adrc.hpp"
#include "krang-utils/lqr.hpp"

// ==========================================================================
//
void activeDisturbanceRejectionControl(
    // inputs
    const Eigen::MatrixXd& A, const Eigen::MatrixXd& B,
    const Eigen::MatrixXd& Q, const Eigen::MatrixXd& R, Eso* EthWheel,
    Eso* EthCOM, const Eigen::VectorXd& B_thWheel,
    const Eigen::VectorXd& B_thCOM, const Eigen::VectorXd& state,
    const Eigen::VectorXd& refState,  // refState is (thCOM, dthCOM, thWheels,
                                      // dthWheels, thSpin, dthSpin)
    const double& dt,
    // outputs
    Eigen::VectorXd& u_thWheel, Eigen::VectorXd& u_thCOM) {
  // LQR for controller gains
  Eigen::VectorXd F = Eigen::VectorXd::Zero(4);
  lqr(A, B, Q, R, F);

  // Observer Control Gains
  Eigen::VectorXd K = -F;
  Eigen::VectorXd error = state - refState;
  u_thCOM << K.topLeftCorner<2, 1>().dot(error.topLeftCorner<2, 1>());
  u_thWheel << K(2) * error(2) + K(3) * error(3);

  // Update Extended State Observer
  EthCOM->update(state(0), B_thCOM, u_thCOM, dt);
  EthWheel->update(state(2), B_thWheel, u_thWheel, dt);

  // Compensate disturbance
  u_thWheel(0) -= EthWheel->getState()(2) / B_thWheel(1);
  u_thCOM(0) -= EthCOM->getState()(2) / B_thCOM(1);
}

// ==========================================================================
// adrc with lqrHackRatios
void activeDisturbanceRejectionControl(
    // inputs
    const Eigen::MatrixXd& A, const Eigen::MatrixXd& B,
    const Eigen::MatrixXd& Q, const Eigen::MatrixXd& R,
    const Eigen::MatrixXd& lqrHackRatios, Eso* EthWheel, Eso* EthCOM,
    const Eigen::VectorXd& B_thWheel, const Eigen::VectorXd& B_thCOM,
    const Eigen::VectorXd& state,
    const Eigen::VectorXd& refState,  // refState is (thCOM, dthCOM, thWheels,
                                      // dthWheels, thSpin, dthSpin)
    const double& dt,
    // outputs
    Eigen::VectorXd& K, Eigen::VectorXd& u_thWheel, Eigen::VectorXd& u_thCOM) {
  // LQR for controller gains
  Eigen::VectorXd F = Eigen::VectorXd::Zero(4);
  lqr(A, B, Q, R, F);

  // Observer Control Gains
  K = lqrHackRatios * -F;
  Eigen::VectorXd error = state - refState;
  u_thCOM << K.topLeftCorner<2, 1>().dot(error.topLeftCorner<2, 1>());
  u_thWheel << K(2) * error(2) + K(3) * error(3);

  // Update Extended State Observer
  EthCOM->update(state(0), B_thCOM, u_thCOM, dt);
  EthWheel->update(state(2), B_thWheel, u_thWheel, dt);

  // Compensate disturbance
  u_thWheel(0) -= EthWheel->getState()(2) / B_thWheel(1);
  u_thCOM(0) -= EthCOM->getState()(2) / B_thCOM(1);
}
