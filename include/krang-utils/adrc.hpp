/**
 * @file helpers.h
 * @author Munzir
 * @date Sept 21th, 2018
 * @brief This file comtains some helper functions used for adrc balancing
 */

#include <dart/dart.hpp>
#include "eso.hpp"


using namespace dart;
using namespace dart::dynamics;

// perform ADRC
void activeDisturbanceRejectionControl(
  //inputs
  const Eigen::MatrixXd& A, const Eigen::MatrixXd& B, const Eigen::MatrixXd& Q, const Eigen::MatrixXd& R,
  Eso* EthWheel, Eso* EthCOM, const Eigen::VectorXd& B_thWheel, const Eigen::VectorXd& B_thCOM,
  const Eigen::VectorXd& state, const Eigen::VectorXd& refState, //refState is (thCOM, dthCOM, thWheels, dthWheels, thSpin, dthSpin)
  const double& dt,
  // outputs
  Eigen::VectorXd& u_thWheel, Eigen::VectorXd& u_thCOM);

// perform ADRC with lqr hacks
void activeDisturbanceRejectionControl(
  //inputs
  const Eigen::MatrixXd& A, const Eigen::MatrixXd& B, const Eigen::MatrixXd& Q, const Eigen::MatrixXd& R, const Eigen::MatrixXd& lqrHackRatios,
  Eso* EthWheel, Eso* EthCOM, const Eigen::VectorXd& B_thWheel, const Eigen::VectorXd& B_thCOM,
  const Eigen::VectorXd& state, const Eigen::VectorXd& refState, //refState is (thCOM, dthCOM, thWheels, dthWheels, thSpin, dthSpin)
  const double& dt,
  // outputs
  Eigen::VectorXd& K, Eigen::VectorXd& u_thWheel, Eigen::VectorXd& u_thCOM);
