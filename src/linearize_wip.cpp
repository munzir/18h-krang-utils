/**
 * @file linearize_wip.cpp
 * @author Munzir Zafar
 * @date Feb 25, 2019
 * @brief Compute linearized dynamics of the WIP model krang
 * */

#include <dart/dart.hpp>

#include "krang-utils/linearize_wip.hpp"

namespace linearize_wip {

// ==========================================================================
// Get the center of mass of body excluding wheels
Eigen::Vector3d GetBodyCOM(dart::dynamics::SkeletonPtr robot) {
  double fullMass = robot->getMass();
  double wheelMass = robot->getBodyNode("LWheel")->getMass();
  return (fullMass * robot->getCOM() -
          wheelMass * robot->getBodyNode("LWheel")->getCOM() -
          wheelMass * robot->getBodyNode("RWheel")->getCOM()) /
         (fullMass - 2 * wheelMass);
}

// ==========================================================================
// compute linearized dynamics for lqr as well as B matrix for Eso updates
void ComputeLinearizedDynamics(const dart::dynamics::SkeletonPtr robot,
const struct ParametersNotFoundInUrdf& params,
                               Eigen::MatrixXd& A, Eigen::MatrixXd& B
                               ) {
  // Wheeled Inverted Pendulum Parameters (symbols taken from the paper)
  double I_ra = params.rotor_inertia;
  double gamma = params.gear_ratio;
  double g = 9.81;
  double c_w = robot->getJoint("JLWheel")->getDampingCoefficient(0);
  double r_w = params.wheel_radius;
  double m_w;
  double I_wa;
  double M_g;
  double l_g;
  double I_yy;
  // Intermediate Parameters
  double delta, c1, c2;
  double a31, a33, a34, a41, a43, a44, b3, b4;

  // Wheel Mass
  m_w = robot->getBodyNode("LWheel")->getMass();

  // Wheel inertia (axis)
  double ixx, iyy, izz, ixy, ixz, iyz;
  robot->getBodyNode("LWheel")->getMomentOfInertia(ixx, iyy, izz, ixy, ixz,
                                                   iyz);
  I_wa = ixx;

  // Body Mass
  M_g = robot->getMass() - 2 * m_w;

  // Distance to body COM
  Eigen::Vector3d xyz0 = robot->getPositions().segment(3, 3);
  l_g = (GetBodyCOM(robot) - xyz0).norm();

  // Body inertia (axis)
  int nBodies = robot->getNumBodyNodes();
  dart::dynamics::Frame* baseFrame = robot->getBodyNode("Base");
  Eigen::Matrix3d iBody = Eigen::Matrix3d::Zero();
  Eigen::Matrix3d rot;
  Eigen::Matrix3d iMat;
  Eigen::Vector3d t;
  Eigen::Matrix3d tMat;
  dart::dynamics::BodyNodePtr b;
  double m;
  for (int i = 0; i < nBodies; i++) {
    if (i == 1 || i == 2) continue;  // Skip wheels
    b = robot->getBodyNode(i);
    b->getMomentOfInertia(ixx, iyy, izz, ixy, ixz, iyz);
    rot = b->getTransform(baseFrame).rotation();
    t = robot->getCOM(baseFrame) -
        b->getCOM(baseFrame);  // Position vector from local COM to body COM
                               // expressed in base frame
    m = b->getMass();
    iMat << ixx, ixy, ixz,  // Inertia tensor of the body around its CoM
                            // expressed in body frame
        ixy, iyy, iyz, ixz, iyz, izz;
    iMat = rot * iMat * rot.transpose();  // Inertia tensor of the body around
                                          // its CoM expressed in base frame
    tMat << (t(1) * t(1) + t(2) * t(2)), (-t(0) * t(1)), (-t(0) * t(2)),
        (-t(0) * t(1)), (t(0) * t(0) + t(2) * t(2)), (-t(1) * t(2)),
        (-t(0) * t(2)), (-t(1) * t(2)), (t(0) * t(0) + t(1) * t(1));
    iMat = iMat + m * tMat;  // Parallel Axis Theorem
    iBody += iMat;
  }
  I_yy = iBody(0, 0);

  // Intermediate Parameters
  delta =
      (M_g * l_g + I_yy + pow(gamma, 2) * I_ra) * (M_g + m_w) * pow(r_w, 2) +
      I_wa + I_ra * pow(gamma, 2) -
      pow(M_g * r_w * l_g - I_ra * pow(gamma, 2), 2);
  c1 = (M_g + m_w) * pow(r_w, 2) + I_wa +
       M_g * r_w * l_g;  // correction of paper. see scanned notes
  c2 = M_g * r_w * l_g + M_g * pow(l_g, 2) + I_yy;

  a31 = ((M_g + m_w) * pow(r_w, 2) + I_wa + I_ra * pow(gamma, 2)) * M_g * g *
        l_g / delta;
  a33 = -c1 * c_w / delta;
  a34 = c1 * c_w / delta;
  a41 = -(M_g * r_w * l_g - I_ra * pow(gamma, 2)) * M_g * g * l_g /
        delta;  // correction of paper. Negative sign added. see scanned notes
  a43 = c2 * c_w / delta;
  a44 = -c2 * c_w / delta;
  b3 = -c1 / delta;
  b4 = c2 / delta;

  // Linearized dynamics
  A << 0, 1, 0, 0,       // thCOM
      a31, a33, 0, a34,  // dthCOM
      0, 0, 0, 1,        // thWheel
      a41, a43, 0, a44;  // dthWheel

  B << 0,  // thCOM
      b3,  // dthCOM
      0,   // thWheel
      b4;  // dthWheel
}

}  // namespace linearize_wip
