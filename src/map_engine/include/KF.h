#include <Eigen/Dense>
#include <math.h>

using Eigen::MatrixXd;
using Eigen::VectorXd;

class KalmanFilter {
public:

  // state vector
  VectorXd x_;

  // state covariance matrix
  MatrixXd P_;

  // state transition matrix
  MatrixXd F_;

  // process covariance matrix
  MatrixXd Q_;

  // measurement matrix
  MatrixXd H_;

  // measurement covariance matrix
  MatrixXd R_;

  /**
   * Constructor
   */
  KalmanFilter();

  /**
   * Destructor
   */
  virtual ~KalmanFilter();

  /**
   * Init Initializes Kalman filter
   * @param x_in Initial state
   * @param P_in Initial state covariance
   * @param F_in Transition matrix
   * @param H_in Measurement matrix
   * @param R_in Measurement covariance matrix
   * @param Q_in Process covariance matrix
   */
  void Init(VectorXd &x_in, MatrixXd &P_in, MatrixXd &F_in,
      MatrixXd &H_in, MatrixXd &R_in, MatrixXd &Q_in)
  {
  	x_ = x_in;
	P_ = P_in;
	F_ = F_in;
	H_ = H_in;
	R_ = R_in;
	Q_ = Q_in;
  }

  /**
   * Prediction Predicts the state and the state covariance
   * using the process model
   * @param delta_T Time between k and k+1 in s
   */
  void Predict()
  {
  	x_ = F_ * x_;
  	MatrixXd Ft = F_.transpose();
  	P_ = F_ * P_ * Ft + Q_;
  }

  /**
   * Updates the state by using standard Kalman Filter equations
   * @param z The measurement at k+1
   */
  void Update(const VectorXd &z)
  {
  	// 使用转换矩阵计算K值，K值可以判断预测值和测量值在最终结果的比例
  	MatrixXd Ht = H_.transpose();
  	MatrixXd S = H_ * P_ * Ht + R_;
  	MatrixXd St = S.inverse();
  	MatrixXd K = P_ * Ht * St;
  	// 更新状态和协方差矩阵
  	VectorXd z_pred = H_ * x_;
  	VectorXd y = z - z_pred;
  	x_ = x_ + K * y;
  	long x_size = x_.size();
  	MatrixXd I = MatrixXd::Identity(x_size, x_size);
  	P_ = (I - K * H_) * P_;
  }

  /**
   * Updates the state by using Extended Kalman Filter equations
   * @param z The measurement at k+1
   */
  void UpdateEKF(const VectorXd &z)
  {
  	// 使用雅各比矩阵计算K值，K值可以判断预测值和测量值在最终结果的比例
	Tools tools;
	MatrixXd Hj = tools.CalculateJacobian(x_);
	MatrixXd Hjt = Hj.transpose();
	MatrixXd S = Hj * P_ * Hjt + R_;
	MatrixXd St = S.inverse();
	MatrixXd K = P_ * Hjt * St;
	// 把笛卡尔坐标转换成极坐标，转换成和测量值相同的格式
	VectorXd z_pred = VectorXd(3);
	z_pred[0] = sqrt(x_[0] * x_[0] + x_[1] * x_[1]);
	z_pred[1] = atan2(x_[1], x_[0]);
	if (z_pred[0] != 0) {
	z_pred[2] = (x_[0] * x_[2] + x_[1] * x_[3]) / z_pred[0];
	} else {
	z_pred[2] = 0;
	}
	// 更新状态和协方差矩阵
	VectorXd y = z - z_pred;
	// 注意：这个地方需要把角度范围缩放到-PI到PI之间
	while (y[1] > M_PI){
	y[1] -= 2 * M_PI;
	}
	while (y[1] < -M_PI){
	y[1] += 2 * M_PI;
	}
	x_ = x_ + K * y;
	long x_size = x_.size();
	MatrixXd I = MatrixXd::Identity(x_size, x_size);
	P_ = (I - K * Hj) * P_;
  }

};
