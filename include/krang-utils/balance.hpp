// Author Akash Patel (apatel435@gatech.edu)
// Purpose: Helper source code file for generating poses that focuses on
// balancing and collision code

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
// // Balance and Collision Method
Eigen::MatrixXd balanceAndCollision(Eigen::MatrixXd inputPose, SkeletonPtr fullRobot, double tolerance, bool collisionCheck);

// // Balancing Methods
Eigen::MatrixXd balancePose(SkeletonPtr robot, Eigen::MatrixXd unBalPose);
