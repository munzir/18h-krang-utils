// Author Akash Patel (apatel435@gatech.edu)
// Purpose: To provide a wrapper for collision checking in DART and some other
// collision related methods

// Includes
#include <dart/dart.hpp>
#include <dart/utils/urdf/urdf.hpp>

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

// Global Variables
double kpi = M_PI;
double kbendLimit = 2.0944;

// // Limits of each joint to prevent collision with the previous joint
// // Below values obtained from hardware init script on Krang: https://github.gatech.edu/WholeBodyControlAttempt1/Krang-Software/blob/master/project/krang/util/scripts/initd/krang
// Format for below vectors
// qWaist, qTorso, qLArm0, ... qLArm6, qRArm0, ..., qRArm6
// negative value (-) is clockwise from parent axis

// qBase limit numbers found by binary searching up to thousandth digit and checking for
// collision cases at qWaist = 0 for upperqBaseLimit and qWaist = 2 for
// lowerqBaseLimit
double lowerqBaseLimit = -1.764;
double upperqBaseLimit = 1.737;

double lowerJointLimit[] = {0, -1.57, -kpi, -kbendLimit, -kpi, -kbendLimit, -kpi, -kbendLimit, -kpi, -kpi, -kbendLimit, -kpi, -kbendLimit, -kpi, -kbendLimit, -kpi};
double upperJointLimit[] = {2.88, 1.57, kpi, kbendLimit, kpi, kbendLimit, kpi, kbendLimit, kpi, kpi, kbendLimit, kpi, kbendLimit, kpi, kbendLimit, kpi};

// Functions
// // First Parent Collision Checking
bool notInFirstParentJointLimits(Eigen::MatrixXd inputPose) {
    // Check for base angle constraintt
    double qBase = inputPose.coeff(1, 0);

    if (qBase < lowerqBaseLimit || qBase > upperqBaseLimit) {
        return true;
    }

    int startJoint = 7;
    for (int i = 0; i < sizeof(lowerJointLimit)/sizeof(lowerJointLimit[0]); i++) {
        // The rest of the angles
        if (inputPose.row(startJoint + i)(0, 0) < lowerJointLimit[i] || inputPose.row(startJoint + i)(0, 0) > upperJointLimit[i]) {
            return true;
        }
    }
    return false;
}

// // Collision Check
// TODO: Need to fix implementation
bool isColliding(SkeletonPtr robot) {
    WorldPtr world(new World);

    SkeletonPtr floor = createFloor();

    robot->enableSelfCollisionCheck();
    robot->disableAdjacentBodyCheck();

    world->addSkeleton(floor);
    world->addSkeleton(robot);

    auto constraintSolver = world->getConstraintSolver();
    auto group = constraintSolver->getCollisionGroup();
    auto& option = constraintSolver->getCollisionOption();
    auto bodyNodeFilter = std::make_shared<BodyNodeCollisionFilter>();
    option.collisionFilter = bodyNodeFilter;

    CollisionResult result;
    group->collide(option, &result);

    //bool inCollision = result.isCollision() || notInFirstParentJointLimits(dartToMunzir(robot->getPositions(), robot));
    bool inCollision = notInFirstParentJointLimits(dartToMunzir(robot->getPositions(), robot));

    return inCollision;
}

// // World creation for collision checking
SkeletonPtr createFloor() {
    SkeletonPtr floor = Skeleton::create();

    // Give the floor a body
    BodyNodePtr body = floor->createJointAndBodyNodePair<WeldJoint>(nullptr).second;

    // Give the body a shape
    double floor_width = 10;
    double floor_height = 0.05;
    std::shared_ptr<BoxShape> box(
          new BoxShape(Eigen::Vector3d(floor_width, floor_width, floor_height)));
    auto shapeNode
        = body->createShapeNodeWith<VisualAspect, CollisionAspect, DynamicsAspect>(box);
    shapeNode->getVisualAspect()->setColor(dart::Color::Blue());

    // Put the body into position
    Eigen::Isometry3d tf(Eigen::Isometry3d::Identity());
    tf.translation() = Eigen::Vector3d(0.0, 0.0, -floor_height / 2.0 - 0.012);
    body->getParentJoint()->setTransformFromParentBodyNode(tf);

    return floor;
}
