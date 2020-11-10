#ifndef KALMANFILTER_H
#define KALMANFILTER_H

#include <math.h>
#include <vector>
#include <iostream>

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
  void Reset();

  // Set current time of the algorithm and consequently update the motion model/prediction features
  void EstimateMotion(double delta_time);

  // Prediction of the next state vector
  void Prediction(double delta_time);

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
  void InitState(Eigen::MatrixXd iniVector, Eigen::MatrixXd iniCov);

  // return the size of the state
  int GetSizeState();

  //return the 6D parameters needed for SLAM registration (i.e RPYXYZ)
  Eigen::Matrix<double, 6, 1> Get6DState();
  //return the 6D parameters covariance needed for SLAM registration (i.e RPYXYZ)
  Eigen::Matrix<double, 6, 6> Get6DCovariance();

private:
  //------------------------------------------------------
  // STATE
  //------------------------------------------------------
  // State vector composed like this:
  // -rx, ry, rz
  // -tx, ty, tz
  // -drx/dt, dry/dt, drz/dt
  // -dtx/dt, dty/dt, dtz/dt
  // -drx2/dt2, dry2/dt2, drz2/dt2
  // -dtx2/dt2, dty2/dt2, dtz2/dt2
  Eigen::MatrixXd StateEstimated;
  Eigen::MatrixXd CovarianceEstimated;

  //------------------------------------------------------
  // MOTION MODEL
  //------------------------------------------------------
  // Function M to estimate the state with previous position X(t+1) = MX(t)
  Eigen::MatrixXd MotionModel;

  // Covariance of motion model
  Eigen::MatrixXd MotionCovariance;

  // Maximale acceleration endorsed by the vehicule
  // used to estimate motion model covariance
  double MaxAcceleration;
  double MaxAngleAcceleration;

  //------------------------------------------------------
  // OTHER
  //------------------------------------------------------
  // indicate the number of observed measures
  unsigned int SizeState;
};

#endif // KALMANFILTER_H
