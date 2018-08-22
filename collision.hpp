// Author Akash Patel (apatel435@gatech.edu)
// Purpose: To provide a wrapper for collision checking in DART and some other
// collision related methods

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
// // Collision Checking
bool inFirstParentJointLimits(Eigen::MatrixXd inputpose);
bool isColliding(SkeletonPtr robot);

// // World creation for collision checking
SkeletonPtr createFloor();
