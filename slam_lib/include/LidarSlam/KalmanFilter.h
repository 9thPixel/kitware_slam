#ifndef KALMANFILTER_H
#define KALMANFILTER_H

#include <math.h>
#include <vector>

#include <Eigen/Dense>
#include <Eigen/Core>

// Fix for windows compilation
// M_PI is not define by include/math.h
#define M_PI       3.14159265358979323846

struct inputMeasure
{
  // Function H to convert a state X to one measure Z (Z = HX)
  Eigen::MatrixXd MeasureModel;

  // Covariance representing incertainty of measure
  Eigen::MatrixXd MeasureCovariance;

  // Current measure
  Eigen::MatrixXd Measure;
};

class KalmanFilter
{
public:
  // default constructor
  KalmanFilter();

  // Reset the class
  void ResetKalmanFilter();

  // Set current time of the algorithm and consequently update the motion model/prediction features
  void EstimateMotion(double time);

  // Prediction of the next state vector
  void Prediction();

  // Update of the state using the input measure (ex : a 3D rigid transform)
  void Update(inputMeasure& input);

  // Perform a complete iteration of Kalman Filter on new measures
  void KalmanIteration(std::vector<inputMeasure*>& Inputs, double t);

  // Set the maximum angle acceleration
  // used to compute the measures covariance matrix
  void SetMaxAngleAcceleration(double acc);

  // Set the maximum velocity acceleration
  // use to compute measures covariance matrix
  void SetMaxVelocityAcceleration(double acc);

  // return the state vector
  Eigen::MatrixXd GetState();

  // Initialize the state vector and the covariance
  void InitState(Eigen::Matrix<double, 12, 1> iniVector, Eigen::Matrix<double, 12, 12> iniCov);

  // return the size of the state
  int GetSizeState();

private:
  // Kalman Filter mode:
  // 0 : registration
  // 1 : registration + GPS velocity
  int mode;

  //------------------------------------------------------
  // STATE
  //------------------------------------------------------
  // State vector composed like this:
  // -rx, ry, rz
  // -tx, ty, tz
  // -drx/dt, dry/dt, drz/dt
  // -dtx/dt, dty/dt, dtz/dt
  Eigen::Matrix<double, 12, 1> StateEstimated;
  Eigen::Matrix<double, 12, 12> CovarianceEstimated;

  //------------------------------------------------------
  // MOTION MODEL
  //------------------------------------------------------
  // Function M to estimate the state with previous position X(t+1) = MX(t)
  Eigen::Matrix<double, 12, 12> MotionModel;

  // Covariance of motion model
  Eigen::Matrix<double, 12, 12> MotionCovariance;

  // Maximale acceleration endorsed by the vehicule
  // used to estimate motion model covariance
  double MaxAcceleration;
  double MaxAngleAcceleration;

  // delta time to compute current estimated motion from motion model
  double PreviousTime;
  double CurrentTime;
  double DeltaTime;

  //------------------------------------------------------
  // OTHER
  //------------------------------------------------------
  // indicate the number of observed measures
  unsigned int SizeState;
};

#endif // KALMANFILTER_H
