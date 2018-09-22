/**
 * @file adrc.cpp
 * @author Munzir Zafar
 * @date Sept 21, 2018
 * @brief This file contains helper functions pertaining to the adrc implementation of balancing
 */

#include "adrc.hpp"
#include "lqr.hpp"

using namespace std;
using namespace dart;
using namespace dart::dynamics;

// ==========================================================================
// Get the center of mass of body excluding wheels
Eigen::Vector3d getBodyCOM(SkeletonPtr robot) {
    double fullMass = robot->getMass();
    double wheelMass = robot->getBodyNode("LWheel")->getMass();
    return (fullMass*robot->getCOM() - wheelMass*robot->getBodyNode("LWheel")->getCOM() - wheelMass*robot->getBodyNode("RWheel")->getCOM())/(fullMass - 2*wheelMass);
}

// ==========================================================================
// compute linearized dynamics for lqr as well as B matrix for ESO updates
void computeLinearizedDynamics(const SkeletonPtr robot, \
  Eigen::MatrixXd& A, Eigen::MatrixXd& B, Eigen::VectorXd& B_thWheel, Eigen::VectorXd& B_thCOM) {
     // ********************* Extracting Required Parameters from DART URDF

    // Get Rot0, xyz0
    Eigen::Matrix<double, 4, 4> baseTf = robot->getBodyNode(0)->getTransform().matrix();
    double psi =  atan2(baseTf(0,0), -baseTf(1,0));
    Eigen::Matrix3d Rot0;
    Rot0 << cos(psi), sin(psi), 0,
            -sin(psi), cos(psi), 0,
            0, 0, 1;
    Eigen::Vector3d xyz0 = robot->getPositions().segment(3,3);

    // Wheeled Inverted Pendulum Parameters (symbols taken from the paper)
    double I_ra = 0;
    double gamma = 1.0;
    double g = 9.81;
    double c_w = 0.1;
    double r_w = 0.25;
    double m_w;
    double I_wa;
    double M_g;
    double l_g;
    double I_yy;
    double delta, c1, c2; // Intermediate Parameters

    // Our intermediate Variables
    double ixx, iyy, izz, ixy, ixz, iyz;
    Eigen::Vector3d COM;
    int nBodies;
    Eigen::Matrix3d iMat;
    Eigen::Matrix3d iBody;
    Eigen::Matrix3d rot;
    Eigen::Vector3d t;
    Eigen::Matrix3d tMat;
    dart::dynamics::BodyNodePtr b;
    dart::dynamics::Frame* baseFrame;
    double m;

    // Wheel Mass
    m_w = robot->getBodyNode("LWheel")->getMass();

    // Wheel inertia (axis)
    robot->getBodyNode("LWheel")->getMomentOfInertia(ixx, iyy, izz, ixy, ixz, iyz);
    I_wa = ixx;

    // Body Mass
    M_g = robot->getMass() - 2*m_w;

    // Distance to body COM
    COM = Rot0*(getBodyCOM(robot) - xyz0); COM(1) = 0;
    l_g = COM.norm();

    // Body inertia (axis)
    nBodies = robot->getNumBodyNodes();
    iBody = Eigen::Matrix3d::Zero();
    baseFrame = robot->getBodyNode("Base");
    for(int i=0; i<nBodies; i++){
      if(i==1 || i==2) continue; // Skip wheels
      b = robot->getBodyNode(i);
      b->getMomentOfInertia(ixx, iyy, izz, ixy, ixz, iyz);
      rot = b->getTransform(baseFrame).rotation();
      t = robot->getCOM(baseFrame) - b->getCOM(baseFrame) ; // Position vector from local COM to body COM expressed in base frame
      m = b->getMass();
      iMat << ixx, ixy, ixz, // Inertia tensor of the body around its CoM expressed in body frame
              ixy, iyy, iyz,
              ixz, iyz, izz;
      iMat = rot*iMat*rot.transpose(); // Inertia tensor of the body around its CoM expressed in base frame
      tMat << (t(1)*t(1)+t(2)*t(2)), (-t(0)*t(1)),          (-t(0)*t(2)),
              (-t(0)*t(1)),          (t(0)*t(0)+t(2)*t(2)), (-t(1)*t(2)),
              (-t(0)*t(2)),          (-t(1)*t(2)),          (t(0)*t(0)+t(1)*t(1));
      iMat = iMat + m*tMat; // Parallel Axis Theorem
      iBody += iMat;
    }
    I_yy = iBody(0, 0);

    // Intermediate Parameters
    delta = (M_g*l_g+I_yy+pow(gamma,2)*I_ra)*(M_g+m_w)*pow(r_w,2)+I_wa+I_ra*pow(gamma,2)-pow(M_g*r_w*l_g-I_ra*pow(gamma,2),2);
    c1 = (M_g+m_w)*pow(r_w,2)+I_wa+I_ra*pow(gamma,2)+M_g*r_w*l_g+I_ra*pow(gamma,2);
    c2 = M_g*r_w*l_g+M_g*pow(l_g,2)+I_yy;

    // ******************** Robot dynamics for LQR Gains
    // A << 0, 0, 1, 0, //thCOM
    //      0, 0, 0, 1, //thWheel
    //      ((M_g+m_w)*pow(r_w,2)+I_wa+I_ra*pow(gamma,2))*M_g*g*l_g/delta, 0, -c1*c_w/delta, c1*c_w/delta, //dthCOM
    //      (M_g*r_w*l_g-I_ra*pow(gamma,2))*M_g*g*l_g/delta, 0, c2*c_w/delta, -c2*c_w/delta; //dthWheel

    // B << 0, //thCOM
    //      0, //thWheel
    //      -c1/delta, //dthCOM
    //      c2/delta; //dthWheel


    A << 0, 1, 0, 0, //thCOM
         ((M_g+m_w)*pow(r_w,2)+I_wa+I_ra*pow(gamma,2))*M_g*g*l_g/delta, -c1*c_w/delta, 0, c1*c_w/delta, //dthCOM
         0, 0, 0, 1, //thWheel
         (M_g*r_w*l_g-I_ra*pow(gamma,2))*M_g*g*l_g/delta, c2*c_w/delta, 0, -c2*c_w/delta; //dthWheel

    B << 0, //thCOM
         -c1/delta, //dthCOM
         0, //thWheel
         c2/delta; //dthWheel

    // ********************** Observer Dynamics
    B_thWheel << 0,
           c2/delta,
           0;
    B_thCOM << 0,
           -c1/delta,
           0;
}

// ==========================================================================
//
void activeDisturbanceRejectionControl(
  //inputs
  const Eigen::MatrixXd& A, const Eigen::MatrixXd& B, const Eigen::MatrixXd& Q, const Eigen::MatrixXd& R,
  ESO* EthWheel, ESO* EthCOM, const Eigen::VectorXd& B_thWheel, const Eigen::VectorXd& B_thCOM,
  const Eigen::VectorXd& state, const Eigen::VectorXd& refState, //refState is (thCOM, dthCOM, thWheels, dthWheels, thSpin, dthSpin)
  const double& dt,
  // outputs
  Eigen::VectorXd& u_thWheel, Eigen::VectorXd& u_thCOM)
{
  // LQR for controller gains
  Eigen::VectorXd F = Eigen::VectorXd::Zero(4);
  lqr(A, B, Q, R, F);

  // Observer Control Gains
  Eigen::VectorXd K = -F;
  Eigen::VectorXd error = state - refState;
  u_thCOM << K.topLeftCorner<2,1>().dot(error.topLeftCorner<2,1>());
  u_thWheel << K(2)*error(2) + K(3)*error(3);

  // Update Extended State Observer
  EthCOM->update(state(0), B_thCOM, u_thCOM, dt);
  EthWheel->update(state(2), B_thWheel, u_thWheel, dt);

  // Compensate disturbance
  u_thWheel(0) -= EthWheel->getState()(2)/B_thWheel(1);
  u_thCOM(0)  -= EthCOM->getState()(2)/B_thCOM(1);
}
