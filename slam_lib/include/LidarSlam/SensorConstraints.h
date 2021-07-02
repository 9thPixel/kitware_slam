//==============================================================================
// Copyright 2018-2020 Kitware, Inc., Kitware SAS
// Author: Sanchez Julia (Kitware SAS)
//         Cadart Nicolas (Kitware SAS)
// Creation date: 2021-04-02
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.
//==============================================================================

#include <queue>
#include <shared_mutex>

#include "LidarSlam/CeresCostFunctions.h" // for residual structure + ceres
#include "LidarSlam/Utilities.h"

namespace LidarSlam
{
  
#define SetSensorMacro(name,type) void Set##name (type _arg) { this->name = _arg; }
#define GetSensorMacro(name,type) type Get##name () const { return this->name; }

namespace SensorConstraints
{

struct WheelOdomMeasurement
{
  double Time = 0.;
  double Distance = 0.;
};

struct GravityMeasurement
{
  double Time = 0.;
  Eigen::Vector3d Acceleration = Eigen::Vector3d::Zero();
};

template <typename T>
class SensorManager
{
public:
  //Setters/Getters
  GetSensorMacro(Weight, double)
  SetSensorMacro(Weight, double)

  GetSensorMacro(TimeOffset, double)
  SetSensorMacro(TimeOffset, double)

  GetSensorMacro(Measures, std::deque<T>)
  SetSensorMacro(Measures, const std::deque<T>&)

  GetSensorMacro(MaxNbMeasures, unsigned int)
  SetSensorMacro(MaxNbMeasures, unsigned int)

  GetSensorMacro(Residual, CeresTools::Residual)

  // Add one measure at a time in measures deque
  void AddMeasurement(const T& m)
  {
    std::unique_lock<std::shared_timed_mutex> lock(this->Mutex);
    this->Measures.emplace_back(m);
    if (this->Measures.size() > this->MaxNbMeasures)
      this->Measures.pop_front();
  }
  void Reset()
  {
    std::unique_lock<std::shared_timed_mutex> lock(this->Mutex);
    this->ResetResidual();
    this->Measures.clear();
    this->PrevIt = this->Measures.begin();
    this->TimeOffset = 0.;
  }

protected:
  void ResetResidual()
  {
    this->Residual.Cost.reset();
    this->Residual.Robustifier.reset();
  }

  // Check if sensor can be used in optimization
  // The weight must be not null and the measures vector must contain elements
  bool CanBeUsed() {std::shared_lock<std::shared_timed_mutex> lock(this->Mutex); return this->Weight > 1e-6 && !this->Measures.empty();}

  // Index of measurement used for previous frame
  typename std::deque<T>::iterator PrevIt;
  // Measures stored
  std::deque<T> Measures;
  // Weight to apply to sensor constraint
  double Weight = 0.;
  // Time offset to make external sensors/Lidar correspondance
  double TimeOffset = 0.;
  // Resulting residual
  CeresTools::Residual Residual;
  // Boolean to know in which state we are
  bool IsRef = false;
  // Max number of measures kept in memory
  unsigned int MaxNbMeasures = 1e4;
  // Mutex to protect members from residual computing +
  // external main thread data adding + manager resetting
  // The data could not be directly in a protected structure because
  // the computation applied to it is very specific to its elements construction
  // WARNING: it cannot perform two constraint computations at a time
  // NOTE: shared_timed_mutex should be replaced by shared_mutex
  // when upgrading to C++17
  mutable std::shared_timed_mutex Mutex;
};

class WheelOdometryManager : public SensorManager<WheelOdomMeasurement>
{
public:
  //Setters/Getters
  GetSensorMacro(RefPose, Eigen::Isometry3d)
  SetSensorMacro(RefPose, const Eigen::Isometry3d&)

  // Wheel odometry constraint (unoriented)
  // The relative spatial distance between two poses
  // corresponding to successive Lidar scans timestamps
  // is taken as constraint
  void ComputeWheelOdomConstraint(double lidarTime);
  // Wheel absolute abscisse constraint (unoriented)
  // The odometry value corresponding to first frame timestamp
  // is taken as reference.
  // The spatial distance between a pose corresponding to a new Lidar scan
  // and the reference pose is taken as constraint
  // This constraint can be used for straight motion (e.g. corridor, tunnel)
  void ComputeWheelAbsoluteConstraint(double lidarTime);

private:
  // Members used when using the relative distance with last estimated pose
  Eigen::Isometry3d RefPose = Eigen::Isometry3d::Identity();
  float RefDistance = 0.f;
};

class ImuManager : public SensorManager<GravityMeasurement>
{
public:
  //Setters/Getters
  GetSensorMacro(GravityRef, Eigen::Vector3d)
  SetSensorMacro(GravityRef, const Eigen::Vector3d&)

  // IMU constraint (gravity)
  void ComputeGravityConstraint(double lidarTime);
  // Compute Reference gravity vector from IMU measurements
  void ComputeGravityRef(double deltaAngle);

private:
  Eigen::Vector3d GravityRef = Eigen::Vector3d::Zero();
};

} // end of SensorConstraints namespace
} // end of LidarSlam namespace