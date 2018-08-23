// Author Akash Patel (apatel435@gatech.edu)
// Purpose: Helper source code file for generating poses that focuses on
// balancing and collision code

// Includes
#include <dart/dart.hpp>
#include <dart/utils/urdf/urdf.hpp>
#include <nlopt.hpp>

#include "balance.hpp"
#include "collision.hpp"
#include "convert_pose_formats.hpp"

// Namespaces
using namespace std;
using namespace dart::collision;
using namespace dart::common;
using namespace dart::dynamics;
using namespace dart::math;
using namespace dart::simulation;
using namespace dart::utils;

// Structs and functions for balancing Krang Model (Full)
struct comOptParams {
    SkeletonPtr robot;
    Eigen::Matrix<double, 24, 1> qInit;
};

double comOptFunc(const std::vector<double> &x, std::vector<double> &grad, void *my_func_data) {
    comOptParams* optParams = reinterpret_cast<comOptParams *>(my_func_data);
    Eigen::Matrix<double, 24, 1> q(x.data());

    if (!grad.empty()) {
        Eigen::Matrix<double, 24, 1> mGrad = q-optParams->qInit;
        Eigen::VectorXd::Map(&grad[0], mGrad.size()) = mGrad;
    }
    return (0.5*pow((q-optParams->qInit).norm(), 2));
}

double comConstraint(const std::vector<double> &x, std::vector<double> &grad, void *com_const_data) {
    comOptParams* optParams = reinterpret_cast<comOptParams *>(com_const_data);
    Eigen::Matrix<double, 24, 1> q(x.data());
    optParams->robot->setPositions(q);
    return (pow(optParams->robot->getCOM()(0)-optParams->robot->getPosition(3), 2) \
        + pow(optParams->robot->getCOM()(1)-optParams->robot->getPosition(4), 2));
}

double wheelAxisConstraint(const std::vector<double> &x, std::vector<double> &grad, void *wheelAxis_const_data) {
    comOptParams* optParams = reinterpret_cast<comOptParams *>(wheelAxis_const_data);
    Eigen::Matrix<double, 24, 1> q(x.data());
    optParams->robot->setPositions(q);
    return optParams->robot->getBodyNode(0)->getTransform().matrix()(2,0);
}

double headingConstraint(const std::vector<double> &x, std::vector<double> &grad, void *heading_const_data) {
    comOptParams* optParams = reinterpret_cast<comOptParams *>(heading_const_data);
    Eigen::Matrix<double, 24, 1> q(x.data());
    optParams->robot->setPositions(q);
    Eigen::Matrix<double, 4, 4> Tf = optParams->robot->getBodyNode(0)->getTransform().matrix();
    double heading = atan2(Tf(0,0), -Tf(1,0));
    optParams->robot->setPositions(optParams->qInit);
    Tf = optParams->robot->getBodyNode(0)->getTransform().matrix();
    double headingInit = atan2(Tf(0,0), -Tf(1,0));
    return heading-headingInit;
}

// Structs and functions for balancing Fixed Wheel Krang Model (Simple)
struct inequalityOptParams {
    Eigen::MatrixXd P;
    Eigen::VectorXd b;
};

double comSimpleOptFunc(const std::vector<double> &x, std::vector<double> &grad, void *my_func_data) {
    comOptParams* optParams = reinterpret_cast<comOptParams *>(my_func_data);
    Eigen::Matrix<double, 1, 1> q1(x.data());
    Eigen::Matrix<double, 18, 1> q;
    q << q1, optParams->qInit.tail(17);

    optParams->robot->setPositions(q);

    // ATTN: getCOM()(2) instead of (1) because urdf is aligned on z-axis
    return optParams->robot->getCOM()(2);
}

void qBaseConstraint(unsigned m, double *result, unsigned n, const double* x, double* grad, void* f_data) {
    inequalityOptParams* constParams = reinterpret_cast<inequalityOptParams *>(f_data);

    if (grad != NULL) {
        for(int i=0; i<m; i++) {
            for(int j=0; j<n; j++){
                grad[i*n+j] = constParams->P(i, j);
            }
        }
    }

    Eigen::Matrix<double, 1, 1> X;
    for(size_t i=0; i<n; i++) X(i) = x[i];

    Eigen::VectorXd mResult;
    mResult = constParams->P*X - constParams->b;
    for(size_t i=0; i<m; i++) {
        result[i] = mResult(i);
    }
}

// Balance and Collision Method
Eigen::MatrixXd balanceAndCollision(Eigen::MatrixXd inputPose, SkeletonPtr fullRobot, SkeletonPtr fixedWheelRobot, Eigen::MatrixXd(*balance)(SkeletonPtr robot, Eigen::MatrixXd unBalPose), double tolerance, bool collisionCheck) {
    Eigen::MatrixXd balPoseParams;

    try {
        if (balance == fixedWheelBalancePose) {
            balPoseParams = balance(fixedWheelRobot, inputPose);
        } else {
            balPoseParams = balance(fullRobot, inputPose);
        }
    } catch (exception& e) {
        throw runtime_error("Pose balancing failed!");
    }

    // Set position of full robot to the pose for transform and collision
    Eigen::MatrixXd balPoseParamsDart = munzirToDart(balPoseParams.transpose());
    fullRobot->setPositions(balPoseParamsDart);

    // Reset zero quantities that do no effect balancing (xCOM of robot)
    // TODO: Fix These methods
    //balPoseParams = resetZeroParams(balPoseParams);

    // If qBase is flipped, flip it back to positive angle (2*pi rotation)
    // TODO: Fix These methods
    //balPoseParams = resetQBase(balPoseParams);

    // Run it through collision check
    bool isCollision = false;
    if (collisionCheck == true) {
        isCollision = isColliding(fullRobot);
        // Throw exception if colliding
        if (isCollision) {
            throw runtime_error("Pose is in collision!");
        }
    }

    // Check the effect of balancing, throw exception if it fails
    // Get x center of mass
    double xCOM = fullRobot->getCOM()(0) - balPoseParams.coeff(2, 0);

    // Check for tolerance
    if (balance != doNotBalancePose && abs(xCOM) > tolerance) {
        throw runtime_error("Tolerance level of " + to_string(tolerance) + " not met!");
    }

    return balPoseParams.transpose();
}

// // Balancing Methods
Eigen::MatrixXd fullBalancePose(SkeletonPtr robot, Eigen::MatrixXd unBalPose) {

    const int dof = (const int) robot->getNumDofs();
    comOptParams optParams;
    optParams.robot = robot;
    optParams.qInit << unBalPose;
    nlopt::opt opt(nlopt::LN_COBYLA, dof);
    std::vector<double> unbalPoseParams(dof);
    double minf;
    opt.set_min_objective(comOptFunc, &optParams);
    opt.add_equality_constraint(comConstraint, &optParams, 1e-8);
    opt.add_equality_constraint(wheelAxisConstraint, &optParams, 1e-8);
    opt.add_equality_constraint(headingConstraint, &optParams, 1e-8);
    opt.set_xtol_rel(1e-4);
    opt.set_maxtime(10);
    opt.optimize(unbalPoseParams, minf);
    Eigen::Matrix<double, 24, 1> balPoseParams(unbalPoseParams.data());

    return balPoseParams;
}

// TODO: Need to make sure the input pose and output pose params are in the
// right format
Eigen::MatrixXd angleBalancePose(SkeletonPtr robot, Eigen::MatrixXd unBalPose) {

    // Set robot to unbalPose
    // and find the xcom error angle
    // correct the qBase to balance it and return the balPose

    Eigen::MatrixXd balPoseParams = unBalPose;

    Eigen::MatrixXd xyzVec = unBalPose.block(2, 0, 3, 1);
    Eigen::MatrixXd unBalPoseDart = munzirToDart(unBalPose.transpose());
    robot->setPositions(unBalPoseDart);

    Eigen::MatrixXd COM = robot->getCOM() - xyzVec;
    double comAngle = atan2(COM(0, 0), COM(2, 0));

    // Adjust qBase to bring COM on top of wheels
    balPoseParams(1, 0) -= comAngle;

    return balPoseParams;
}

// TODO Need to fix this one (packing/dimension issues)
Eigen::MatrixXd fixedWheelBalancePose(SkeletonPtr robot, Eigen::MatrixXd unBalPose) {
    comOptParams optParams;
    optParams.robot = robot;

    // TODO: need to unpackage unBalPose as well for 17 dof instead of input 24
    // dof
    optParams.qInit << unBalPose;

    // The specific algorithm to use (COBYLA: Constrained Optimization BY
    // Linear Approximations
    nlopt::opt opt(nlopt::LN_COBYLA, 1);
    vector<double> unbalPoseParams(1);
    double minf;

    // Minimize the objective function
    opt.set_min_objective(comSimpleOptFunc, &optParams);

    // Add the inequality constraint on qBase
    const vector<double> inequalityconstraintTol(2, 1e-3);
    inequalityOptParams inequalityconstraintParams;

    // Set P and b vectors
    // A way to represent the inequality constraint where qBase should be
    // between -pi/2 and pi/2 so that our COM is above the wheel not below
    // it
    Eigen::MatrixXd setP(2,1);
    Eigen::MatrixXd setb(2,1);

    setP(0,0) = 1;
    setP(1,0) = -1;
    setb(0,0) = M_PI/2;
    setb(1,0) = M_PI/2;

    inequalityconstraintParams.P = setP;
    inequalityconstraintParams.b = setb;

    opt.add_inequality_mconstraint(qBaseConstraint, &inequalityconstraintParams, inequalityconstraintTol);

    //Set relative tolerance on opt params
    opt.set_xtol_rel(1e-4);

    //Set max time allocation for optimizing
    opt.set_maxtime(10);

    opt.optimize(unbalPoseParams, minf);

    Eigen::Matrix<double, 18, 1> balPoseParams(unbalPoseParams.data());

    // TODO: Pack result in 24 dof from 18 dof for full robot
    double z = 0; //axis-angle1
    double a = 0; //axis-angle2
    double b = 0; //axis-angle3
    double c = 0; //x
    double d = 0; //y
    double e = 0; //z
    double f = 0; //qLWheel
    double g = 0; //qRWheel
    double j = 0; //qKinect

    return balPoseParams;
}

Eigen::MatrixXd doNotBalancePose(SkeletonPtr robot, Eigen::MatrixXd unbalPose) {
    return unbalPose;
}

// // Reset params that do not effect xCOM to zero
// TODO
Eigen::MatrixXd resetZeroParams(Eigen::MatrixXd inputPose) {
    // What we change: qWaist, qTorso, qLArm0, ... qLArm6, qRArm0, ..., qRArm6
    // i, j, l, m, n, o, p, q, r, s, t, u, v, w, x, y

    // Axis-angle changing is expected (this is where balancing kicks in)
    // qBase is thing changing due to balance but heading shouldnt
    // Need to change axis angle such that qBase is unchanged but heading
    // is zero
    //double a = 0; //axis-angle1
    //double b = 0; //axis-angle2
    //double c = 0; //axis-angle3

    // Finding qBase
    //Eigen::MatrixXd robotBaseTf = robot->getBodyNode(0)->getTransform().matrix();
    //double heading = atan2(robotBaseTf(0, 0), -robotBaseTf(1, 0));
    //double qBase = atan2(robotBaseTf(0,1)*cos(heading) + robotBaseTf(1,1)*sin(heading), robotBaseTf(2,1));

    //// Force heading to be zero
    //heading = 0;

    //// Calculating the axis angle representation of orientation from headingInit and qBaseInit:
    //// RotX(pi/2)*RotY(-pi/2+headingInit)*RotX(-qBaseInit)
    //Eigen::Transform<double, 3, Eigen::Affine> baseTf = Eigen::Transform<double, 3, Eigen::Affine>::Identity();
    //baseTf.prerotate(Eigen::AngleAxisd(-qBase,Eigen::Vector3d::UnitX())).prerotate(Eigen::AngleAxisd(-M_PI/2+heading,Eigen::Vector3d::UnitY())).prerotate(Eigen::AngleAxisd(M_PI/2, Eigen::Vector3d::UnitX()));
    //Eigen::AngleAxisd aa(baseTf.matrix().block<3,3>(0,0));

    //Eigen::MatrixXd aaAngleAxis = aa.angle()*aa.axis();
    //inputPose(0, 0) = aaAngleAxis(0, 0);
    //inputPose(1, 0) = aaAngleAxis(1, 0);
    //inputPose(2, 0) = aaAngleAxis(2, 0);

    //// What we keep same
    //double d = 0; //x
    //double e = 0; //y
    //double f = 0.264; //z
    //double g = 0; //qLWheel
    //double h = 0; //qRWheel
    //// If kinect changes it might effect balance leave it in
    ////double k = 0; //qKinect

    //inputPose(3, 0) = d;
    //inputPose(4, 0) = e;
    //inputPose(5, 0) = f;
    //inputPose(6, 0) = g;
    //inputPose(7, 0) = h;

    return inputPose;
}

// // Flip qBase if required
// TODO
Eigen::MatrixXd resetQBase(Eigen::MatrixXd inputPose) {
    //// Check for base angle constraint (has to be b/t -pi/2 and pi/2)
    //Eigen::MatrixXd robotBaseTf = robot->getBodyNode(0)->getTransform().matrix();
    //double heading = atan2(robotBaseTf(0, 0), -robotBaseTf(1, 0));
    //double qBase = atan2(robotBaseTf(0,1)*cos(heading) + robotBaseTf(1,1)*sin(heading), robotBaseTf(2,1));

    //if (qBase < 0) {
    //    qBase += M_PI;
    //} else if (qBase > M_PI) {
    //    qBase -= M_PI;
    //}

    //// Calculating the axis angle representation of orientation from headingInit and qBaseInit:
    //// RotX(pi/2)*RotY(-pi/2+headingInit)*RotX(-qBaseInit)
    //Eigen::Transform<double, 3, Eigen::Affine> baseTf = Eigen::Transform<double, 3, Eigen::Affine>::Identity();
    //baseTf.prerotate(Eigen::AngleAxisd(-qBase,Eigen::Vector3d::UnitX())).prerotate(Eigen::AngleAxisd(-M_PI/2+heading,Eigen::Vector3d::UnitY())).prerotate(Eigen::AngleAxisd(M_PI/2, Eigen::Vector3d::UnitX()));
    //Eigen::AngleAxisd aa(baseTf.matrix().block<3,3>(0,0));

    //Eigen::MatrixXd aaAngleAxis = aa.angle()*aa.axis();
    //inputPose(0, 0) = aaAngleAxis(0, 0);
    //inputPose(1, 0) = aaAngleAxis(1, 0);
    //inputPose(2, 0) = aaAngleAxis(2, 0);

    return inputPose;
}
