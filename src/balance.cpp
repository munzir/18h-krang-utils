// Author Akash Patel (apatel435@gatech.edu)
// Purpose: Helper source code file for generating poses that focuses on
// balancing and collision code

// Includes
#include <dart/dart.hpp>
#include <dart/utils/urdf/urdf.hpp>

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

// Balance and Collision Method
Eigen::MatrixXd balanceAndCollision(Eigen::MatrixXd inputPose, SkeletonPtr robot, double tolerance, bool collisionCheck) {
    Eigen::MatrixXd balPoseParams;

    try {
        balPoseParams = balancePose(robot, inputPose);
    } catch (exception& e) {
        throw runtime_error("Pose balancing failed!");
    }

    // Set position of full robot to the pose for transform and collision
    Eigen::MatrixXd balPoseParamsDart = munzirToDart(balPoseParams.transpose());
    robot->setPositions(balPoseParamsDart);

    // Run it through collision check
    bool isCollision = false;
    if (collisionCheck == true) {
        isCollision = isColliding(robot);
        // Throw exception if colliding
        if (isCollision) {
            throw runtime_error("Pose is in collision!");
        }
    }

    // Check the effect of balancing, throw exception if it fails
    // Get x center of mass
    double xCOM = robot->getCOM()(0) - balPoseParams.coeff(2, 0);

    // Check for tolerance
    if (abs(xCOM) > tolerance) {
        throw runtime_error("Tolerance level of " + to_string(tolerance) + " not met!");
    }

    return balPoseParams.transpose();
}

// TODO: Need to make sure the input pose and output pose params are in the
// right format
Eigen::MatrixXd balancePose(SkeletonPtr robot, Eigen::MatrixXd unBalPose) {

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
