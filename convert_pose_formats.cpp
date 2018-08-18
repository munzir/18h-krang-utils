// Author Akash Patel (apatel435@gatech.edu)
// Purpose: To convert between Munzir's set of coordinates and DART coordinates

// Munizr Coordinate Format
// heading, qBase, x, y, z, qLWheel, qRWheel, qWaist, qTorso,
// qLArm0, ... qLArm6, qRArm0, ..., qRArm6

// DART Coordinate format
// axis-angle1, axis-angle2, axis-angle3, x, y, z, qLWheel, qRWheel, qWaist, qTorso,
// qLArm0, ... qLArm6, qRArm0, ..., qRArm6

// Includes
#include "convert_pose_formats.hpp"

// Functions
Eigen::MatrixXd munzirToDart(Eigen::RowVectorXd munzirPose) {
    // Convert input

    // Find the pose in DART formats
    double headingInit = munzirPose(0);
    double qBaseInit = munzirPose(1);
    Eigen::Matrix<double, 21, 1> unchangedValues;
    unchangedValues << munzirPose.segment(2,21).transpose();

    // Calculating the axis angle representation of orientation from headingInit and qBaseInit:
    // RotX(pi/2)*RotY(-pi/2+headingInit)*RotX(-qBaseInit)
    Eigen::Transform<double, 3, Eigen::Affine> baseTf = Eigen::Transform<double, 3, Eigen::Affine>::Identity();
    baseTf.prerotate(Eigen::AngleAxisd(-qBaseInit,Eigen::Vector3d::UnitX())).prerotate(Eigen::AngleAxisd(-M_PI/2+headingInit,Eigen::Vector3d::UnitY())).prerotate(Eigen::AngleAxisd(M_PI/2, Eigen::Vector3d::UnitX()));
    Eigen::AngleAxisd aa(baseTf.matrix().block<3,3>(0,0));

    // Now compile this data into dartPoseParams
    Eigen::Matrix<double, 24, 1> dartPose;
    dartPose << aa.angle()*aa.axis(), unchangedValues;

    return dartPose;
}

Eigen::MatrixXd dartToMunzir(Eigen::RowVectorXd dartPose, SkeletonPtr robot) {
    // Find the pose in munzir format
    Eigen::Matrix<double, 21, 1> unchangedValues;
    unchangedValues << dartPose.segment(3,21).transpose();

    // Calculating the headingInit and qBase Init from the axis angle representation of orientation:
    robot->setPositions(dartPose);
    Eigen::MatrixXd baseTf = robot->getBodyNode(0)->getTransform().matrix();
    double headingInit = atan2(baseTf(0, 0), -baseTf(1, 0));
    double qBaseInit = atan2(baseTf(0,1)*cos(headingInit) + baseTf(1,1)*sin(headingInit), baseTf(2,1));
    // I thought the below would work but its not qbase is off by 1.57 rads (90
    // degs) the above expression for qBaseInit fixes that
    //double qBaseInit = atan2(baseTf(2,1), baseTf(2,2));

    // Now compile this data into munzir pose
    Eigen::Matrix<double, 23, 1> munzirPose;
    munzirPose << headingInit, qBaseInit, unchangedValues;

    return munzirPose;
}
