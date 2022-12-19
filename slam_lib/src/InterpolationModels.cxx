//==============================================================================
// Copyright 2022 Kitware, Inc., Kitware SAS
// Authors: Arthur Bourbousson (Kitware SAS),
// Creation date: 2022-10-17
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

#include "LidarSlam/InterpolationModels.h"
#include <cmath>

namespace LidarSlam
{
namespace Interpolation
{
// ---------------------------------------------------------------------------
//   Translation Models
// ---------------------------------------------------------------------------

// -------------------------------- Linear -----------------------------------

/**
 * @brief Create linear interpolation class from 2 translations
 * @param vecState vector of state containing time and transformation for each frame
 */
Linear::Linear(const std::vector<LidarState> &vecState)
{
  this->RecomputeModel(vecState);
}

/**
 * @brief Make interpolation for t time
 *
 * @param t time
 *  If t0=t1 or Transf[0]=Transf[1], this returns Transf[0] to avoid numerical issues.
 * @return Translation transformation
 */
Eigen::Isometry3d Linear::operator()(double t) const
{
  double time = (t - Time[0]) / (Time[1] - Time[0]);

  if (Time[0] == Time[1] || Transf[0].isApprox(Transf[1]))
    return(Eigen::Translation3d(Transf[0]) * Eigen::Quaterniond::Identity());
  Eigen::Translation3d trans(Transf[0] + time * (Transf[1] - Transf[0]));

  return (trans * Eigen::Quaterniond::Identity());
}

void Linear::RecomputeModel(const std::vector<LidarState>& vecState)
{
  if (vecState.size() < 1)
  {
    PRINT_ERROR("No data for Linear interpolation, use null data");
    Time = {0., 1.};
    Transf = {Eigen::Vector3d::Zero(), Eigen::Vector3d::Zero()};
    return;
  }
  if (vecState.size() == 1)
    PRINT_WARNING("Only one data for Linear translation, perform constant interpolation");
  if (vecState.size() > 2)
    PRINT_WARNING("Linear interpolation has more than 2 transformations, only use the 2 last ones");
  const auto &stateStart = vecState.crbegin()[1];
  const auto &stateEnd = vecState.back();
  Time = {stateStart.Time, stateEnd.Time};
  Transf = {stateStart.Isometry.translation().matrix(), stateEnd.Isometry.translation().matrix()};
}

// -------------------------------- Spline -----------------------------------

Spline::Spline(const std::vector<LidarState> &vecState, unsigned int degree)
{
  this->RecomputeModel(vecState, degree);
}

Eigen::Isometry3d Spline::operator()(double t) const
{
  return (Eigen::Translation3d(this->SplineModel(ScaledValue(t))) * Eigen::Quaterniond::Identity());
}

void Spline::RecomputeModel(const std::vector<LidarState> &vecState)
{
  this->RecomputeModel(vecState, this->Degree);
}

/**
 * @brief Recompute a Spline model from new data and a certain degree
 *        Use Eigen module to generate Spline
 * @param vecState
 * @param degree
 */
void Spline::RecomputeModel(const std::vector<LidarState> &vecState, unsigned int degree)
{
  size_t size = std::max((int)vecState.size(), 2);
  Eigen::VectorXd timeVec(size);
  Eigen::Matrix<double, 3, Eigen::Dynamic> knotMat(3, size);

  // If not enough data or impossible degree, give up recomputation
  if (vecState.empty() || !degree)
  {
    PRINT_ERROR("No data for Spline interpolation, use null data");
    knotMat = Eigen::ArrayXXd::Zero(3, 2);
  }

  for (int i = 0; i < vecState.size(); ++i)
  {
    timeVec[i] = vecState[i].Time;
    knotMat.col(i) = vecState[i].Isometry.translation();
  }
  if (vecState.size() == 1)
  {
    PRINT_WARNING("Only one data for spline, compute constant translation");
    // Duplicate value to do a constant Spline
    timeVec[1] = timeVec[0] + 1;
    knotMat.block(0, 1, 3, 1) = knotMat.block(0, 0, 3, 1);
  }
  Degree = std::min((unsigned int)vecState.size() - 1, degree);
  MinTime = timeVec.minCoeff();
  MaxTime = timeVec.maxCoeff();
  // Need scaled values in Eigen model
  SplineModel = Eigen::SplineFitting<Spline3d>::Interpolate(knotMat, Degree, ScaledValues(timeVec));
}

double Spline::ScaledValue(double t) const {
  return (t - MinTime) / (MaxTime - MinTime);
}

// Scaled time values and transpose the time vector
Eigen::RowVectorXd Spline::ScaledValues(Eigen::VectorXd const &t_vec) const
{
  return t_vec.unaryExpr([this](double t) { return ScaledValue(t); }).transpose();
}

// ---------------------------------------------------------------------------
//   Rotation Models
// ---------------------------------------------------------------------------

// -------------------------------- Slerp ------------------------------------

/**
 * @brief Create Slerp interpolation from 2 rotations
 * @param vecState  vector of state containing time and transformation for each frame
 */
Slerp::Slerp(const std::vector<LidarState>& vecState)
{
  RecomputeModel(vecState);
}

/**
 * @brief Make interpolation for t time
 *
 * @param t time
 *  If t0=t1 or H0=H1, this returns Rot[0] to avoid numerical issues.
 * @return Rotation transformation
 */
Eigen::Isometry3d Slerp::operator()(double t) const
{
  double time = (t - Time[0]) / (Time[1] - Time[0]);

  if (Time[0] == Time[1] || Rot[0].isApprox(Rot[1]))
    return(Eigen::Translation3d::Identity() * Rot[0]);
  Eigen::Quaterniond rot(Rot[0].slerp(time, Rot[1]));
  return (Eigen::Translation3d::Identity() * rot);
}

void Slerp::RecomputeModel(const std::vector<LidarState> &vecState)
{
  if (vecState.empty())
  {
    PRINT_ERROR("No data for Slerp interpolation, compute identity rotation");
    Time = {0., 1.};
    Rot = {Eigen::Quaterniond::Identity(), Eigen::Quaterniond::Identity()};
    return;
  }
  if (vecState.size() == 1)
    PRINT_WARNING("Slerp has only one transformation, compute constant rotation");
  if (vecState.size() > 2)
    PRINT_WARNING("Slerp has more than 2 transformations, only use the 2 last ones");
  const auto &stateStart = vecState.crbegin()[1];
  const auto &stateEnd = vecState.back();
  Time = {stateStart.Time, stateEnd.Time};
  Rot = {Eigen::Quaterniond(stateStart.Isometry.linear()), Eigen::Quaterniond(stateEnd.Isometry.linear())};
}

// -------------------------------- N-Slerp ------------------------------------

NSlerp::NSlerp(const std::vector<LidarState> &vecState)
{
  this->RecomputeModel(vecState);
}

Eigen::Isometry3d NSlerp::operator()(double t) const
{
  std::pair<size_t, size_t> pairIndex{0, 1};

  // Find the 2 closest rotation to the time t
  if (this->VecTime.size() > 2)
    pairIndex = binarySearch(t);

  Eigen::Quaterniond rot;
  // Special case, the time is in our dataset
  if (pairIndex.first == pairIndex.second)
    rot = this->VecRot[pairIndex.first];
  else
  {
    // Interpolate the rotation with Slerp
    size_t inf = pairIndex.first, supp = pairIndex.second;
    double time = (t - this->VecTime[inf]) / (this->VecTime[supp] - this->VecTime[inf]);
    rot = this->VecRot[inf].slerp(time, this->VecRot[supp]);
  }
  return (Eigen::Translation3d::Identity() * rot);
}

void NSlerp::RecomputeModel(const std::vector<LidarState> &vecState)
{
  this->VecTime.clear();
  this->VecRot.clear();
  if (vecState.empty())
  {
    PRINT_ERROR("No data for NSlerp interpolation, compute identity rotation");
    this->VecTime.push_back(0.);
    this->VecTime.push_back(1.);
    this->VecRot.push_back(Eigen::Quaterniond::Identity());
    this->VecRot.push_back(Eigen::Quaterniond::Identity());
  }
  for (const auto &state : vecState)
  {
    this->VecTime.push_back(state.Time);
    this->VecRot.push_back(Eigen::Quaterniond(state.Isometry.linear()));
  }
  if (vecState.size() == 1)
  {
    PRINT_WARNING("Only one data for N-Slerp, compute constant rotation");
    this->VecTime.push_back(vecState[0].Time + 1.);
    this->VecRot.push_back(Eigen::Quaterniond(vecState[0].Isometry.linear()));
  }
}

// Search the closest inferior value and the superior value of t
// Complexity of log2(N)
std::pair<size_t, size_t> NSlerp::binarySearch(const double t) const
{
  size_t low = 0;
  size_t high = this->VecTime.size() - 1;
  if (t < this->VecTime[low])
    return std::pair<size_t, size_t>(low, low + 1);
  else if (t > this->VecTime[high])
    return std::pair<size_t, size_t>(high - 1, high);
  while (high - low > 1)
  {
    size_t middle = (high + low) / 2;
    // Compare if it is equal with respect to a precision
    if (std::abs(t - this->VecTime[middle]) < 1e-6)
      return (std::pair<size_t, size_t>(middle, middle));
    else if (t < this->VecTime[middle])
      high = middle;
    else
      low = middle;
  }
  return (std::pair<size_t, size_t>(low, high));
}

// ---------------------------------------------------------------------------
//   Trajectory Model
// ---------------------------------------------------------------------------

Trajectory::Trajectory(const std::vector<LidarState>& vecState, Model interpolationModel,
                        bool onlyNecessary): OnlyNecessary(onlyNecessary)
{
  SetModel(vecState, interpolationModel);
}

void Trajectory::SetOnlyNecessary(bool onlyNecessary)
{
  this->OnlyNecessary = onlyNecessary;
}

void Trajectory::SetModel(const std::vector<LidarState>& vecState, Model interpolationModel)
{
  // Determine number of data needed for Spline
  int nbrDataSpline = 0;
  if (this->OnlyNecessary)
  {
    if (interpolationModel == Model::LINEAR_SPLINE)
      nbrDataSpline = 2;
    else if (interpolationModel == Model::QUADRATIC_SPLINE)
      nbrDataSpline = 3;
    else if (interpolationModel == Model::CUBIC_SPLINE)
      nbrDataSpline = 4;
  }

  // Choose translation model
  if (interpolationModel == Model::LINEAR)
    this->TranslationPtr = CreateModel<Linear>(vecState, 2);
  if (interpolationModel == Model::LINEAR_SPLINE)
    this->TranslationPtr = CreateModel<Spline>(vecState, nbrDataSpline, 1);
  if (interpolationModel == Model::QUADRATIC_SPLINE)
    this->TranslationPtr = CreateModel<Spline>(vecState, nbrDataSpline, 2);
  if (interpolationModel == Model::CUBIC_SPLINE)
    this->TranslationPtr = CreateModel<Spline>(vecState, nbrDataSpline, 3);

  // Choose rotation model
  if (this->OnlyNecessary || vecState.size() <= 2)
    this->RotationPtr = CreateModel<Slerp>(vecState, 2);
  else
    this->RotationPtr = CreateModel<NSlerp>(vecState);
}

void Trajectory::RecomputeModel(const std::vector<LidarState> &vecState)
{
  this->TranslationPtr->RecomputeModel(vecState);
  this->RotationPtr->RecomputeModel(vecState);
}

Eigen::Isometry3d Trajectory::operator()(double t) const
{
  return ((*this->TranslationPtr)(t) * (*this->RotationPtr)(t));
}

// ---------------------------------------------------------------------------
//   Interpolation functions
// ---------------------------------------------------------------------------

Eigen::Isometry3d LinearInterpolation(const Eigen::Isometry3d& H0, const Eigen::Isometry3d& H1,
                                      double t, double t0, double t1)
{
  if(t0 == t1 || H0.isApprox(H1))
    return H1;
  const double time = (t - t0) / (t1 - t0);
  Eigen::Quaterniond rot(Eigen::Quaterniond(H0.linear()).slerp(time, Eigen::Quaterniond(H1.linear())));
  Eigen::Translation3d trans(H0.translation() + time * (H1.translation() - H0.translation()));
  return trans * rot;
}

}  // end of Interpolation namespace
}  // end of LidarSlam namespace