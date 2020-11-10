#include "LidarSlam/KalmanFilter.h"

struct inputMeasure;

//-----------------------------------------------------------------------------
void KalmanFilter::Reset()
{
  // set the number of measures observed
  this->SizeState = 18;

  // Init motion Model diagonal
  this->MotionModel = Eigen::MatrixXd::Identity(this->SizeState, this->SizeState);

  // Init Motion model covariance
  this->MotionCovariance = Eigen::MatrixXd::Zero(this->SizeState, this->SizeState);

  // Init Estimator covariance
  this->CovarianceEstimated = Eigen::MatrixXd::Zero(this->SizeState, this->SizeState);

  // Fill state vector
  this->StateEstimated = Eigen::MatrixXd::Zero(this->SizeState, 1);

  // Set the maximale acceleration
  // Settle to 10 m.s-2 (= 1g). it is
  // the maximal acceleration that a "normal"
  // car can endorsed. Moreover, it the acceleration
  // of a falling drone. Seems a good limit
  // Reducing the maximal acceleration will
  // reduce the motion model covariance matrix
  // We can take an additional 20% error
  this->MaxAcceleration = 1.2 * 10.0;

  // Maximal acceleration settled to
  // 1080 degrees / s-2. To have an image
  // the maximale acceleration corresponds
  // to an none-mobile object than can goes
  // up to 3 rotations in one second (180
  // per min). This seems reasonable.
  // We can take an additional 20% error
  this->MaxAngleAcceleration = 1.2 * 1080.0 / 180.0 * M_PI;
}

//-----------------------------------------------------------------------------
void KalmanFilter::InitState(const Eigen::MatrixXd& iniState, const Eigen::MatrixXd& iniCov)
{
  this->StateEstimated = iniState;
  this->CovarianceEstimated = iniCov;
}

//-----------------------------------------------------------------------------
void KalmanFilter::EstimateMotion(double delta_time)
{
  // Update motion model matrix
  for (unsigned int i = 0; i <= 5; ++i)
    this->MotionModel(i, i + 6) = delta_time;

  // Update Motion model covariance matrix :
  // -Angle
  int fact = 1;
  for (unsigned int i = 0; i < 3; ++i)
    this->MotionCovariance(i, i) = std::pow(fact*0.5 * this->MaxAngleAcceleration * std::pow(delta_time, 2), 2);

  // -Position
  for (unsigned int i = 3; i < 6; ++i)
    this->MotionCovariance(i, i) = std::pow(fact*0.5 * this->MaxAcceleration * std::pow(delta_time, 2), 2);

  // -Angle speed
  for (unsigned int i = 6; i < 9; ++i)
    this->MotionCovariance(i, i) = std::pow(fact*this->MaxAngleAcceleration * delta_time, 2);

  // -Velocity
  for (unsigned int i = 9; i < 12; ++i)
    this->MotionCovariance(i, i) = std::pow(fact*this->MaxAcceleration * delta_time, 2);

  // -Acceleration
  for (unsigned int i = 12; i < 18; ++i)
    this->MotionCovariance(i, i) = std::pow(fact*this->MaxAcceleration, 2);
}

//-----------------------------------------------------------------------------
void KalmanFilter::Prediction(double delta_time)
{
  // Prediction using motion model and motion covariance
  // Build new motion model with current time
  EstimateMotion(delta_time);
  // State prediction
  this->StateEstimated = this->MotionModel * this->StateEstimated ;
  // Covariance prediction
  this->CovarianceEstimated = this->MotionModel * this->CovarianceEstimated * this->MotionModel.transpose() + this->MotionCovariance;
}

//-----------------------------------------------------------------------------
void KalmanFilter::Update(inputMeasure& input)
{
  // Update estimated state with one measure (various measures can update the result successively)
  // Compute Kalman gain (named K)
  Eigen::MatrixXd innovation = (input.MeasureModel * this->CovarianceEstimated * input.MeasureModel.transpose() + input.MeasureCovariance);
  Eigen::MatrixXd K = this->CovarianceEstimated * input.MeasureModel.transpose() * innovation.inverse();

  // Update the estimated state
  Eigen::MatrixXd error = input.Measure - input.MeasureModel * this->StateEstimated;
  this->StateEstimated = this->StateEstimated + (K * error);

  // Update the estimated covariance
  this->CovarianceEstimated = this->CovarianceEstimated - (K * input.MeasureModel * this->CovarianceEstimated);
}

//-----------------------------------------------------------------------------
void KalmanFilter::KalmanIteration(std::vector<inputMeasure*>& Inputs, double t)
{
  //Perform a complete iteration prediction + update with all available inputs
  // predict pose with motion model
  Prediction(t);
  for(int i = 0; i < Inputs.size(); ++i)
    Update(*Inputs[i]);
}

//-----------------------------------------------------------------------------
Eigen::MatrixXd KalmanFilter::GetState()
{
  return this->StateEstimated;
}

//-----------------------------------------------------------------------------
Eigen::Matrix<double, 6, 1> KalmanFilter::Get6DState()
{
  return this->StateEstimated.topLeftCorner(6, 1);
}

//-----------------------------------------------------------------------------
Eigen::Matrix<double, 6, 6> KalmanFilter::Get6DCovariance()
{
  return this->CovarianceEstimated.topLeftCorner(6, 6);
}

//-----------------------------------------------------------------------------
void KalmanFilter::SetMaxAngleAcceleration(double acc)
{
  this->MaxAngleAcceleration = acc * M_PI / 180.0;
}

//-----------------------------------------------------------------------------
void KalmanFilter::SetMaxVelocityAcceleration(double acc)
{
  this->MaxAcceleration = acc;
}

//-----------------------------------------------------------------------------
int KalmanFilter::GetSizeState()
{
  return this->StateEstimated.rows();
}
