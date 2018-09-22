/**
 * @file helpers.h
 * @author Munzir
 * @date Sept 21th, 2018
 * @brief This file comtains some helper functions used for adrc balancing
 */

#include <dart/dart.hpp>
#include "ESO.hpp"


using namespace dart;
using namespace dart::dynamics;

// read the costs Q and R
void readCosts(Eigen::MatrixXd& Q, Eigen::MatrixXd& R);

// get body com
Eigen::Vector3d getBodyCOM(SkeletonPtr robot);

// compute linearized dynamics
void computeLinearizedDynamics(const SkeletonPtr robot, \
  Eigen::MatrixXd& A, Eigen::MatrixXd& B, Eigen::VectorXd& B_thWheel, Eigen::VectorXd& B_thCOM);

// perform ADRC
void activeDisturbanceRejectionControl( 
  //inputs
  const Eigen::MatrixXd& A, const Eigen::MatrixXd& B, const Eigen::MatrixXd& Q, const Eigen::MatrixXd& R, 
  ESO* EthWheel, ESO* EthCOM, const Eigen::VectorXd& B_thWheel, const Eigen::VectorXd& B_thCOM, 
  const Eigen::VectorXd& state, const Eigen::VectorXd& refState, //refState is (thCOM, dthCOM, thWheels, dthWheels, thSpin, dthSpin) 
  const double& dt,
  // outputs
  Eigen::VectorXd& u_thWheel, Eigen::VectorXd& u_thCOM);