#include "LidarSlam/SensorConstraints.h"

namespace LidarSlam
{
namespace SensorConstraints
{

bool GetWheelAbsoluteConstraint(double lidarTime, double& weight, std::vector<WheelOdomMeasurement>& measures,
                                CeresTools::Residual& residual)
{
  static int prevIdx = 0;   // Index of measurements used for previous frame
                            // (interpolation between prevIdx and prevIdx + 1 values)
  static double firstDistance = measures[0].Distance; // interpolation used for the previous frame
  unsigned int currIdx = 0; // Index of measurements used for this frame
                            // (interpolation between currIdx and currIdx + 1 values)
  double currDistance = firstDistance; // interpolation used for the current frame

  // Check if odometry measurements were taken around Lidar Frame acquisition
  if (measures.front().Time < lidarTime && lidarTime < measures.back().Time)
  {
    // Reset if needed
    if (measures[prevIdx].Time > lidarTime)
      prevIdx = 0;

    currIdx = prevIdx;
    // Get index of last odometry measurement before LiDAR time
    while (currIdx + 1 < measures.size() && measures[currIdx + 1].Time <= lidarTime)
      currIdx++;

    // If we got to the end of measures
    // no measure corresponds to Lidar time
    if (currIdx + 1 == measures.size())
      return false;

    // Interpolate odometry measurement at LiDAR timestamp
    double rt = (lidarTime - measures[currIdx].Time) / (measures[currIdx + 1].Time - measures[currIdx].Time);
    currDistance = (1 - rt) * measures[currIdx].Distance + rt * measures[currIdx + 1].Distance;

    // Build odometry residual
    // if it is not first frame
    if (prevIdx > 0)
    {
      residual.Cost = CeresCostFunctions::OdometerDistanceResidual::Create(Eigen::Vector3d::Zero(), currDistance - firstDistance);
      residual.Robustifier = std::shared_ptr<ceres::ScaledLoss>(new ceres::ScaledLoss(NULL, weight, ceres::TAKE_OWNERSHIP));
      std::cout << "Adding odometry residual : " << currDistance - firstDistance << " m travelled since first frame." << std::endl;
    }
    else
    {
      firstDistance = currDistance;
      residual.Cost = CeresCostFunctions::OdometerDistanceResidual::Create(Eigen::Vector3d::Zero(), 0);
      residual.Robustifier = std::shared_ptr<ceres::ScaledLoss>(new ceres::ScaledLoss(NULL, 0., ceres::TAKE_OWNERSHIP));
      std::cout << "First frame => no odometry" << std::endl;
    }
  }
  else
    return false;

  prevIdx = currIdx;
  return true;
}

bool GetWheelOdomConstraint(double lidarTime, double& weight, std::vector<WheelOdomMeasurement>& measures,
                            Eigen::Isometry3d& previousTworld, CeresTools::Residual& residual)
{
  static int prevIdx = 0;   // Index of measurements used for previous frame
                            // (interpolation between prevIdx and prevIdx + 1 values)
  static double prevDistance = measures[0].Distance; // interpolation used for the previous frame
  unsigned int currIdx = 0; // Index of measurements used for this frame
                            // (interpolation between currIdx and currIdx + 1 values)
  double currDistance = 0.; // interpolation used for the current frame

  // Check if odometry measurements were taken around Lidar Frame acquisition
  if (measures.front().Time < lidarTime && lidarTime < measures.back().Time)
  {
    // Reset if needed
    if (measures[prevIdx].Time > lidarTime)
    {
      prevIdx = 0;
      prevDistance = measures[0].Distance;
    }

    currIdx = prevIdx;
    // Get index of last odometry measurement before LiDAR time
    while (currIdx + 1 < measures.size() && measures[currIdx + 1].Time <= lidarTime)
      currIdx++;

    // If we got to the end of measures
    // no measure corresponds to Lidar time
    if (currIdx + 1 == measures.size())
      return false;

    // Interpolate odometry measurement at LiDAR timestamp
    double rt = (lidarTime - measures[currIdx].Time) / (measures[currIdx + 1].Time - measures[currIdx].Time);
    currDistance = (1 - rt) * measures[currIdx].Distance + rt * measures[currIdx + 1].Distance;

    // Build odometry residual
    // if there is memory of a previous pose
    if (prevIdx > 0)
    {
      double distDiff = std::abs(currDistance - prevDistance);

      residual.Cost = CeresCostFunctions::OdometerDistanceResidual::Create(previousTworld.translation(), distDiff);
      residual.Robustifier = std::shared_ptr<ceres::ScaledLoss>(new ceres::ScaledLoss(NULL, weight, ceres::TAKE_OWNERSHIP));
      std::cout << "Adding odometry residual : " << distDiff << " m travelled since last frame." << std::endl;
    }
    else
    {
      residual.Cost = CeresCostFunctions::OdometerDistanceResidual::Create(previousTworld.translation(), 0);
      residual.Robustifier = std::shared_ptr<ceres::ScaledLoss>(new ceres::ScaledLoss(NULL, 0., ceres::TAKE_OWNERSHIP));
      std::cout << "First frame = no odometry" << std::endl;
    }
  }
  else
    return false;

  prevDistance = currDistance;
  prevIdx = currIdx;
  return true;
}

bool GetGravityConstraint(double lidarTime, double& weight, std::vector<GravityMeasurement>& measures,
                          Eigen::Vector3d& gravityRef, CeresTools::Residual& residual)
{
  static int prevIdx = 0;   // Index of measurements used for previous frame
                            // (measures from 0 to prevIdx)
  unsigned int currIdx = 0; // Index of measurement used for this frame
                            // (measures from 0 to prevIdx)

  if (measures.front().Time < lidarTime && lidarTime < measures.back().Time)
  {
    currIdx = prevIdx;
    // Get index of last IMU measurement before LiDAR time
    // The first IMU measurement was taken before Lidar frame (cf previous if)
    while (currIdx + 1 < measures.size() && measures[currIdx + 1].Time <= lidarTime)
      currIdx++;

    // If we got to the end of measures
    // no measure corresponds to Lidar time
    if (currIdx + 1 == measures.size())
      return false;   

    // Interpolate gravity measurement at LiDAR timestamp
    double rt = (lidarTime - measures[currIdx].Time) / (measures[currIdx + 1].Time - measures[currIdx].Time);
    Eigen::Vector3d gravityDirection = (1 - rt) * measures[currIdx].Acceleration.normalized() + rt * measures[currIdx + 1].Acceleration.normalized();
    if (gravityDirection.norm() > 1e-6)
      gravityDirection.normalize();
    else 
      return false;

    // Build gravity constraint
    residual.Cost = CeresCostFunctions::ImuGravityAlignmentResidual::Create(gravityRef, gravityDirection);
    residual.Robustifier = std::shared_ptr<ceres::ScaledLoss>(new ceres::ScaledLoss(NULL, weight, ceres::TAKE_OWNERSHIP));
    std::cout << "Adding IMU gravity residual." << std::endl;
  }
  else
    return false;

  prevIdx = currIdx;
  return true;
}

Eigen::Vector3d ComputeGravityRef(std::vector<GravityMeasurement>& measures, double deltaAngle)
{
  // Init histogram 2D (phi and theta)
  int NPhi = std::ceil(2 * M_PI / deltaAngle);
  int NTheta = std::ceil(M_PI / deltaAngle);
  std::vector<std::vector<std::vector<int>>> histogram(NPhi, std::vector<std::vector<int>>(NTheta));
  // Store acceleration vector indices in histogram
  for (unsigned int idxAcc = 0; idxAcc < measures.size(); ++idxAcc)
  {
    Eigen::Vector3d AccelDirection = measures[idxAcc].Acceleration.normalized();
    int idxPhi = ( std::atan2(AccelDirection.y(), AccelDirection.x()) + M_PI ) / deltaAngle;
    int idxTheta = ( std::acos(AccelDirection.z()) ) / deltaAngle;
    histogram[idxPhi][idxTheta].push_back(idxAcc);
  }

  // Get bin containing most points
  unsigned int bestPhi = 0;
  unsigned int bestTheta = 0;
  for (unsigned int idxPhi = 0; idxPhi < histogram.size(); ++ idxPhi)
  {
    for (unsigned int idxTheta = 0; idxTheta < histogram[0].size(); ++ idxTheta)
    {
      if (histogram[idxPhi][idxTheta].size() > histogram[bestPhi][bestTheta].size())
      {
        bestPhi = idxPhi;
        bestTheta = idxTheta;
      }
    }
  }

  // Compute mean of this bin 
  Eigen::Vector3d gravityRef = {0, 0, 0};
  for (int idxAcc : histogram[bestPhi][bestTheta])
    gravityRef += measures[idxAcc].Acceleration.normalized();
  gravityRef.normalize();

  return gravityRef;
}

} // end of SensorConstraints namespace
} // end of LidarSlam namespace