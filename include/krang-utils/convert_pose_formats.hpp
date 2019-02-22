// Author Akash Patel (apatel435@gatech.edu)
// Purpose: To convert between Munzir's set of coordinates and DART coordinates

// Munizr Coordinate Format
// heading, qBase, x, y, z, qLWheel, qRWheel, qWaist, qTorso,
// qLArm0, ... qLArm6, qRArm0, ..., qRArm6

// DART Coordinate format
// axis-angle1, axis-angle2, axis-angle3, x, y, z, qLWheel, qRWheel, qWaist, qTorso,
// qLArm0, ... qLArm6, qRArm0, ..., qRArm6

// Includes
#include <dart/dart.hpp>
#include <dart/utils/urdf/urdf.hpp>

// Namespaces
using namespace std;
using namespace dart::common;
using namespace dart::dynamics;
using namespace dart::math;
using namespace dart::utils;

// Function Prototypes
// // Conversions
Eigen::MatrixXd munzirToDart(Eigen::RowVectorXd munzirPose);
Eigen::MatrixXd dartToMunzir(Eigen::RowVectorXd dartPose, SkeletonPtr robot);
