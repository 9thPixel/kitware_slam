#include "LidarSlam/CeresCostFunctions.h" // for residual structure + ceres

namespace LidarSlam
{
struct WheelOdomMeasurement
{
  double Time = 0.;
  double Distance = 0.;
};

struct GravityMeasurement
{
  double Time = 0.;
  Eigen::Vector3d Acceleration = {0., 0., 0.};
};

namespace SensorConstraints
{

// Wheel odometry constraint (unoriented)
bool GetWheelOdomConstraint(double lidarTime, double& weight, std::vector<WheelOdomMeasurement>& measures,
                            Eigen::Isometry3d& previousTworld, CeresTools::Residual& residual);

// Wheel absolute abscisse constraint (unoriented)
bool GetWheelAbsoluteConstraint(double lidarTime, double& weight, std::vector<WheelOdomMeasurement>& measures,
                                CeresTools::Residual& residual);

// IMU constraint (gravity)
bool GetGravityConstraint(double lidarTime, double& weight, std::vector<GravityMeasurement>& measures,
                          Eigen::Vector3d& gravityRef, CeresTools::Residual& residual);

Eigen::Vector3d ComputeGravityRef(std::vector<GravityMeasurement>& measures, double deltaAngle);

} // end of SensorConstraints namespace
} // end of LidarSlam namespace