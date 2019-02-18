// Author Akash Patel (apatel435@gatech.edu)
//
// Purpose: Helper source code file for determining optimal gain matrix using lqr
// Returns true if successful and outputs caluclated gain matrix in input
// pointer

// Includes
#include <dart/dart.hpp>

// Function Prototypes

/**
 * @brief Computes the LQR gain matrix (usually denoted K) for a discrete time
 * infinite horizon problem.
 *
 * @param A State matrix of the underlying system
 * @param B Input matrix of the underlying system
 * @param Q Weight matrix penalizing the state
 * @param R Weight matrix penalizing the controls
 * @param N Weight matrix penalizing state / control pairs
 * @param K Pointer to the generated matrix (has to be a double/dynamic size
 * matrix!)
 */

// TODO
// // LQR Method (full)
//bool lqr(const Eigen::MatrixXd& A, const Eigen::MatrixXd& B, const Eigen::MatrixXd& Q, const Eigen::MatrixXd& R, const Eigen::MatrixXd& N, Eigen::MatrixXd& K);

// // LQR Method (without n)
bool lqr(const Eigen::MatrixXd& A, const Eigen::MatrixXd& B, const Eigen::MatrixXd& Q, const Eigen::MatrixXd& R, Eigen::VectorXd& K);
