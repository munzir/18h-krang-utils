// Author Akash Patel (apatel435@gatech.edu)
// Purpose: Helper source code file for generating poses that focuses on
// balancing and collision code

// Includes
#include <dart/dart.hpp>
#include <dart/utils/urdf/urdf.hpp>
#include <nlopt.hpp>

// Namespaces
using namespace std;
using namespace dart::common;
using namespace dart::dynamics;
using namespace dart::math;
using namespace dart::utils;

// Structs and functions for balancing Krang Model (Full)
//struct comOptParams {};

double comOptFunc(const std::vector<double> &x, std::vector<double> &grad, void *my_func_data);
double comConstraint(const std::vector<double> &x, std::vector<double> &grad, void *com_const_data);
double wheelAxisConstraint(const std::vector<double> &x, std::vector<double> &grad, void *wheelAxis_const_data);
double headingConstraint(const std::vector<double> &x, std::vector<double> &grad, void *heading_const_data);

// Structs and functions for balancing Fixed Wheel Krang Model (Simple)
//struct inequalityOptParams {};

double comSimpleOptFunc(const std::vector<double> &x, std::vector<double> &grad, void *my_func_data);
void qBaseConstraint(unsigned m, double *result, unsigned n, const double* x, double* grad, void* f_data);

// Function Prototypes
// // Balance and Collision Method
Eigen::MatrixXd balanceAndCollision(Eigen::MatrixXd inputPose, SkeletonPtr fullRobot, SkeletonPtr fixedWheelRobot, Eigen::MatrixXd(*balance)(SkeletonPtr robot, Eigen::MatrixXd unBalPose), double tolerance, bool collisionCheck);

// // Balancing Methods
Eigen::MatrixXd fullBalancePose(SkeletonPtr robot, Eigen::MatrixXd unBalPose);
Eigen::MatrixXd angleBalancePose(SkeletonPtr robot, Eigen::MatrixXd unBalPose);
Eigen::MatrixXd fixedWheelBalancePose(SkeletonPtr robot, Eigen::MatrixXd unBalPose);
Eigen::MatrixXd doNotBalancePose(SkeletonPtr robot, Eigen::MatrixXd unBalPose);

// // Eliminate balancing artifacts in final pose (i.e. nonzero values that do
// not effect the xCOM value
Eigen::MatrixXd resetZeroParams(Eigen::MatrixXd inputPose);
// // Flip qBase if needed (balancing may give good results but flipped by 2*pi)
Eigen::MatrixXd resetQBase(Eigen::MatrixXd inputPose);
