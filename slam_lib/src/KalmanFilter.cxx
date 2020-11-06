#include "KalmanFilter.h"

#include <iostream>


//-----------------------------------------------------------------------------
KalmanFilter::KalmanFilter()
{
  this->ResetKalmanFilter();
}

//-----------------------------------------------------------------------------
void KalmanFilter::ResetKalmanFilter()
{
  // set the number of measures observed
  this->SizeState = 12;

  // Init motion Model diagonal
  this->MotionModel.setIdentity();

  // Init Motion model covariance
  this->MotionCovariance.setZero();

  // Init Estimator covariance
  this->CovarianceEstimated.setZero();

  // Fill state vector
  this->StateEstimated.setZero();

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
  // up to 3 rotation per secondes (180 per min)
  // in one second. This seems reasonable
  // We can take an additional 20% error
  this->MaxAngleAcceleration = 1.2 * 1080.0 / 180.0 * M_PI;

  this->PreviousTime = 0.0;
  this->CurrentTime = 0.0;
  this->DeltaTime = 0.0;
}

//-----------------------------------------------------------------------------
void KalmanFilter::InitState(Eigen::Matrix<double, 12, 1> iniState, Eigen::Matrix<double, 12, 12> iniCov)
{
  this->StateEstimated = iniState;
  this->CovarianceEstimated = iniCov;
}

//-----------------------------------------------------------------------------
void KalmanFilter::EstimateMotion(double time)
{
  // Update time
  this->PreviousTime = this->CurrentTime;
  this->CurrentTime = time;
  this->DeltaTime = this->CurrentTime - this->PreviousTime;

  // Update motion model matrix
  for (unsigned int i = 0; i <= 5; ++i)
  {
    this->MotionModel(i, i + 6) = this->DeltaTime;
  }

  // Update Motion model covariance matrix :
  // -Angle
  for (unsigned int i = 0; i < 3; ++i)
  {
    this->MotionCovariance(i, i) = std::pow(0.5 * this->MaxAngleAcceleration * std::pow(this->DeltaTime, 2), 2);
  }
  // -Position
  for (unsigned int i = 3; i < 6; ++i)
  {
    this->MotionCovariance(i, i) = std::pow(0.5 * this->MaxAcceleration * std::pow(this->DeltaTime, 2), 2);
  }
  // -Angle speed
  for (unsigned int i = 6; i < 9; ++i)
  {
    this->MotionCovariance(i, i) = std::pow(this->MaxAngleAcceleration * this->DeltaTime, 2);
  }
  // -Velocity
  for (unsigned int i = 9; i < 12; ++i)
  {
    this->MotionCovariance(i, i) = std::pow(this->MaxAcceleration * this->DeltaTime, 2);
  }
}

//-----------------------------------------------------------------------------
void KalmanFilter::Prediction()
{
  // Prediction using motion model and motion covariance
  // State prediction
  this->StateEstimated = this->MotionModel * this->StateEstimated ;
  // Covariance prediction
  this->CovarianceEstimated = this->MotionModel * this->CovarianceEstimated * this->MotionModel.transpose() + this->MotionCovariance;
}

//-----------------------------------------------------------------------------

void KalmanFilter::KalmanIteration(std::vector<inputMeasure*>& Inputs, double t)
{
  //Perform a complete iteration prediction + update with all available inputs
  //Update motion with current time
  EstimateMotion(t);
  // predict pose with motion model
  Prediction();
  for(int i = 0; i<Inputs.size(); ++i)
    Update(*Inputs[i]);
}

//-----------------------------------------------------------------------------
void KalmanFilter::Update(inputMeasure& input)
{
  // Update estimated state with one measure (various measures can update the result successively)
  // Compute Kalman gain (named K)
  Eigen::MatrixXd innovation = (input.MeasureModel * this->CovarianceEstimated * input.MeasureModel.transpose() + input.MeasureCovariance);
  Eigen::MatrixXd K = this->CovarianceEstimated * input.MeasureModel.transpose() * innovation.inverse();

  // Update the estimated state
  Eigen::MatrixXd errVector = input.Measure - input.MeasureModel * this->StateEstimated;
  this->StateEstimated += K * errVector;

  // Update the estimated covariance
  this->CovarianceEstimated -= K * input.MeasureModel * this->CovarianceEstimated;
}

//-----------------------------------------------------------------------------
Eigen::MatrixXd KalmanFilter::GetState()
{
  return this->StateEstimated;
}

//-----------------------------------------------------------------------------
void KalmanFilter::SetMaxAngleAcceleration(double acc)
{
  this->MaxAngleAcceleration = acc / 180.0 * M_PI;
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
