#include <Eigen/Eigen>


class Eso {
public:
  Eso(const Eigen::VectorXd& X0, const Eigen::VectorXd& L_) : mL(L_), mX(X0) {
  	int n = X0.size();
  	mA = Eigen::MatrixXd::Zero(n, n);
  	for(int i=0; i<n-1; i++) mA(i, i+1) = 1;
  }

  Eigen::VectorXd update(const double& Z, const Eigen::VectorXd& B, const Eigen::VectorXd& u, const double& dt) {
  	mX += (mA*mX + mL*(Z - mX(0)) + B*u)*dt;
  	return mX;
  }

  Eigen::VectorXd getState() { return mX; }
private:
  Eigen::VectorXd mL;
  Eigen::VectorXd mX;
  Eigen::MatrixXd mA;
};
