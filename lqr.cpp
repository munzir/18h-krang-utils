// Author Munzir Zafar (mzafar7@gatech.edu)
//
// Purpose: Helper source code file for determining optimal gain matrix using lqr method (continuous time)
// LQR Solver for Continuous Time Infinite Horizon Problem
// Returns true if successful and outputs calculated gain matrix in input
// pointer

// Includes
#include <dart/dart.hpp>

#include "lqr.hpp"

// Namespaces
using namespace std;

// // Function Prototypes
// Balance Matrix for the hamiltonian matrix for numerical stability
void balance_matrix(const Eigen::MatrixXd &A, Eigen::MatrixXd &Aprime, Eigen::MatrixXd &D);

// ==========================================================================
// DGEES computes for an N-by-N real nonsymmetric matrix A, the
// eigenvalues, the real Schur form T, and, optionally, the matrix of
// Schur vectors Z.  This gives the Schur factorization A = Z*T*(Z**T).

// Optionally, it also orders the eigenvalues on the diagonal of the
// real Schur form so that selected eigenvalues are at the top left.
// The leading columns of Z then form an orthonormal basis for the
// invariant subspace corresponding to the selected eigenvalues.

// A matrix is in real Schur form if it is upper quasi-triangular with
// 1-by-1 and 2-by-2 blocks. 2-by-2 blocks will be standardized in the
// form
//         [  a  b  ]
//         [  c  a  ]

// where b*c < 0. The eigenvalues of such a block are a +- sqrt(bc).

typedef long int logical; // logical is the return type of select function to be passed to dgees_
typedef logical (selectFcn)(double *, double *); // function pointer type for select function
logical select(double *wr, double *wi) {         // the function to be passed as select function
    return (*wr < 0);
}

// C function declaration for the DGEES fortran function in lapack library
// see link: http://eigen.tuxfamily.org/index.php?title=Lapack
// for a short tutorial describing how to do this
extern "C" void dgees_(const char* JOBVS, const char* SORT, selectFcn* SELECT, const int* N, double* A, \
    const int* LDA, int* SDIM, double* WR, double* WI, double* VS, const int* LDVS, double* WORK, \
    const int* LWORK, logical* BWORK, int* INFO );

// Function
// // TODO
// // LQR Method (full)
//bool lqr(const Eigen::MatrixXd& A, const Eigen::MatrixXd& B, const Eigen::MatrixXd& Q, const Eigen::MatrixXd& R, const Eigen::MatrixXd& N, Eigen::MatrixXd& K) {
    //Eigen::MatrixXd n = Eigen::MatrixXd::Zero(a.rows(), b.cols());
    //return lqr(a, b, q, r, n, k);
    // Check n
    //if (N.rows() != B.rows() || N.cols() != B.cols()) {
    //    cout << "LQR Error: Fifth (N) matrix must be identically dimensioned with second (B) matrix!" << endl;
    //    return false;
    //}
//}

// // LQR method (without n)
bool lqr(const Eigen::MatrixXd& A, const Eigen::MatrixXd& B, const Eigen::MatrixXd& Q, const Eigen::MatrixXd& R, Eigen::VectorXd& K) {
    // Check dimensionality of input matrices

    // Check a
    if (A.rows() != A.cols()) {
        cout << "LQR Error: First (A) matrix must be square!" << endl;
        return false;
    }

    // Check b
    if (B.rows() != A.cols()) {
        cout << "LQR Error: First (A) and second (B) matrix must be conformal!" << endl;
        return false;
    }

    // Check q
    if (Q.rows() != Q.cols() || Q.rows() != A.cols()) {
        cout << "LQR Error: Third (Q) matrix must be square and conformal with the first (A) matrix!" << endl;
        return false;
    }

    // Check r
    if (R.rows() != R.cols() || R.cols() != B.cols()) {
        cout << "LQR Error: Fourth (R) matrix must be square and conformal with column dimension of second (B) matrix!" << endl;
        return false;
    }

    // Dimension
    int n, n2;
    n = A.rows();
    n2 = 2*n;

    Eigen::MatrixXd H(n2, n2);
    Eigen::MatrixXd M, dM, balM, sMat;
    Eigen::VectorXd D(n);
    Eigen::VectorXd s(n2);
    Eigen::MatrixXd X1(n,n), X2(n,n), L(n,n), U(n,n), Xa(n,n), Xb(n,n), X(n,n);
    Eigen::VectorXd gains(n);

    // ================== Pre-Schur
    // Hamiltonian
    H << A, -B*R.inverse()*B.transpose(),
         -Q, -A.transpose();

    // balance H
    M = H;
    dM = M.diagonal().asDiagonal();
    balance_matrix(M-dM, balM, sMat);
    for(int i=0; i<n2; i++) s(i) = log2(sMat(i,i));
    for(int i=0; i<n; i++) D(i) = round((-s(i) + s(i+n))/2.0);
    for(int i=0; i<n; i++) {
      s(i) = pow(2.0, D(i));
      s(i+n) = pow(2.0, -D(i));
    }
    D = s.head(n);
    H = s.asDiagonal()*H*s.asDiagonal().inverse();

    // =================================== Schur
    // Input Arguments for dgees_ call
    int LDA = H.outerStride();
    int LDVS = n2;

    // Outputs of dgees_ call
    int SDIM;               // Number of eigenvalues (after sorting) for which SELECT is true
    double WR[n2], WI[n2];  // Real and imaginary parts of the computed eigenvalues in the same order
                            // that they appear on the diagonal of the output Schur form T
    Eigen::MatrixXd Z(n2, n2); // Contains the orthogonal matrix Z of Schur vectors

    // The fixed part of the scratch space for dgees_ to do calculations
    logical BWORK[n2];

    // Determine the optimal work size (size of the variable-sized scratch space for dgees_ to do its calculations)
    double WORKDUMMY;
    int LWORK = -1;
    int INFO = 0;
    dgees_("V", "S", select, &n2, H.data(), &LDA, &SDIM, &WR[0], &WI[0], Z.data(), &LDVS, \
      &WORKDUMMY, &LWORK, &BWORK[0], &INFO );
    LWORK = int(WORKDUMMY) + 32;
    Eigen::VectorXd WORK(LWORK);

    // dgees call for real schur decomposition along with ordering
    dgees_("V", "S", select, &n2, H.data(), &LDA, &SDIM, &WR[0], &WI[0], Z.data(), &LDVS, \
      WORK.data(), &LWORK, &BWORK[0], &INFO );

    // ======================================== Post-Schur
    // finding solution X to riccati equation
    X1 = Z.topLeftCorner(n ,n);
    X2 = Z.bottomLeftCorner(n ,n);
    Eigen::PartialPivLU<Eigen::MatrixXd> lu(X1);
    L = Eigen::MatrixXd::Identity(n, n);
    L.block(0,0,n,n).triangularView<Eigen::StrictlyLower>() = lu.matrixLU();
    U = lu.matrixLU().triangularView<Eigen::Upper>();
    Xa = ((X2*U.inverse())*L.inverse())*lu.permutationP();
    Xb = (Xa + Xa.transpose())/2.0;
    X = D.asDiagonal()*Xb*D.asDiagonal();

    // Controller gains
    K << (R.inverse()*(B.transpose()*X)).transpose();
    //K << (X.transpose()*B)*R.inverse();

    

    return true;
}

// Balance Matrix for the hamiltonian matrix for numerical stability
void balance_matrix(const Eigen::MatrixXd &A, Eigen::MatrixXd &Aprime, Eigen::MatrixXd &D) {
    // https://arxiv.org/pdf/1401.5766.pdf (Algorithm #3)
    const int p = 2;
    double beta = 2; // Radix base (2?)
    Aprime = A;
    D = Eigen::MatrixXd::Identity(A.rows(), A.cols());
    bool converged = false;
    do {
        converged = true;
        for (Eigen::Index i = 0; i < A.rows(); ++i) {
            double c = Aprime.col(i).lpNorm<p>();
            double r = Aprime.row(i).lpNorm<p>();
            double s = pow(c, p) + pow(r, p);
            double f = 1;
            while (c < r / beta) {
                c *= beta;
                r /= beta;
                f *= beta;
            }
            while (c >= r*beta) {
                c /= beta;
                r *= beta;
                f /= beta;
            }
            if (pow(c, p) + pow(r, p) < 0.95*s) {
                converged = false;
                D(i, i) *= f;
                Aprime.col(i) *= f;
                Aprime.row(i) /= f;
            }
        }
    } while (!converged);
}
