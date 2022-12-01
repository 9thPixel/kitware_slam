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

#pragma once

#include <vector>
#include <Eigen/Geometry>
#include <unsupported/Eigen/Splines>
#include "LidarSlam/State.h"

namespace LidarSlam
{
namespace Interpolation
{
// List models of interpolation for transformation matrices
enum Model
{
  LINEAR,             // Linear interpolation between 2 transformations
  QUADRATIC_SPLINE,   // Quadratic spline interpolation between N transformations (at least 3)
  CUBIC_SPLINE,       // Cubic spline interpolation between N transformations (at least 4)
};

/**
 * @brief IModel
 * Interface class with minimum required to be an transformation interpolation
 */
class IModel
{
  public:
    virtual Eigen::Isometry3d operator()(double t) const = 0;
    virtual void RecomputeModel(const std::vector<LidarState> &vecState) = 0;
};

// ---------------------------------------------------------------------------
//   Translation Models
// ---------------------------------------------------------------------------

/**
 * @brief Linear Class:
 * Linearly interpolate between 2 translations
 * @note if there is more than 2 data, use the 2 last ones
 */
class Linear : public IModel
{
  public:
    Linear() = delete;
    Linear(const std::vector<LidarState> &vecState);
    Eigen::Isometry3d operator()(double t) const override;
    void RecomputeModel(const std::vector<LidarState> &vecState) override;

  private:
    std::array<double, 2> Time;
    std::array<Eigen::Vector3d, 2> Transf;
};

// ---------------------------------------------------------------------------
/**
 * @brief Spline
 * Interpolate translations with a global polynomial spline of a certain degree (2, 3...)
 */
class Spline : public IModel
{
  // Spline model for spacial data
  typedef Eigen::Spline<double, 3> Spline3d;

  public:
    Spline() = delete;
    Spline(const std::vector<LidarState> &vecState, unsigned int degree);
    Eigen::Isometry3d operator()(double t) const override;
    void RecomputeModel(const std::vector<LidarState> &vecState) override;
    void RecomputeModel(const std::vector<LidarState> &vecState, unsigned int degree);

  private:
    //spline model
    Spline3d SplineModel;
    double MinTime, MaxTime;
    unsigned int Degree;

    // Functions to scaled values because eigen spline function need scaled values
    double ScaledValue(double t) const;
    Eigen::RowVectorXd ScaledValues(Eigen::VectorXd const &t_vec) const;
};

// ---------------------------------------------------------------------------
//   Rotation Models
// ---------------------------------------------------------------------------

/**
 * @brief Slerp
 * Interpolate 2 rotations with SLERP model
 * @note if there is more than 2 data, use the 2 last ones
 */
class Slerp : public IModel
{
  public:
    Slerp() = delete;
    Slerp(const std::vector<LidarState> &vecState);
    Eigen::Isometry3d operator()(double t) const override;
    void RecomputeModel(const std::vector<LidarState> &vecState) override;

  private:
    std::array<double,2> Time;
    std::array<Eigen::Quaterniond, 2> Rot;
};

/**
 * @brief NSlerp
 * Interpolate N rotations with a SLERP Model between every points
 * @warning The points need to be sorted in ascending order of time
 * @warning This class is work in progress, using a simple dichotomy algorithm to find points
 */
class NSlerp : public IModel
{
  public:
    NSlerp() = delete;
    NSlerp(const std::vector<LidarState> &vecState);
    Eigen::Isometry3d operator()(double t) const override;
    void RecomputeModel(const std::vector<LidarState> &vecState) override;

  private:
    std::vector<double> VecTime;
    std::vector<Eigen::Quaterniond> VecRot;
    // Binary search of the closest points to time t over VecTime
    std::pair<size_t, size_t> binarySearch(const double t) const;
};

// ---------------------------------------------------------------------------
//   Trajectory Model
// ---------------------------------------------------------------------------

/**
 * @brief Trajectory interpolation function
 *        Combine a rotation interpolation model with a translation interpolation model
 * @param vecState           data transformations and time
 * @param interpolationModel model of interpolation choosen
 */
  class Trajectory : public IModel
  {
    using ModelPtr = std::unique_ptr<IModel>;

    public:
      Trajectory() = delete;
      Trajectory(const std::vector<LidarState>& vecState, Model interpolationModel,
                  bool onlyNecessary = true);
      void SetOnlyNecessary(bool onlyNecessary);
      void SetModel(const std::vector<LidarState>& vecState, Model interpolationModel);
      void RecomputeModel(const std::vector<LidarState> &vecState) override;
      Eigen::Isometry3d operator()(double t) const override;

    private:
      // Option for model to take only the number of transformations required
      // using the last data in priority
      // ex: a cubic translation will use the last 4 and a SLERP will use only the last 2
      bool OnlyNecessary = true;
      // Separate transformation models into a translation model and a rotation model
      // Idea from Sommer_Efficient_Derivative_Computation_for_Cumulative_B-Splines_on_Lie_Groups paper
      ModelPtr TranslationPtr;
      ModelPtr RotationPtr;
  };

// ---------------------------------------------------------------------------
//   Interpolation utilities
// ---------------------------------------------------------------------------

/**
 * @brief Create an interpolation model
 * T : type of the model
 * vecState : data for interpolation
 * numberData : number data used (0 is all)
 */
template <typename T>
std::unique_ptr<T> CreateModel(const std::vector<LidarState>& vecState, int numberData = 0)
{
  // Cut to the number of data needed
  if (numberData && vecState.size() > numberData)
  {
    const std::vector<LidarState> necessaryData{vecState.end() - (1 + numberData), vecState.end() - 1};
    return (std::make_unique<T>(necessaryData));
  }
  return (std::make_unique<T>(vecState));
}

// Template for Spline case
template <typename T>
std::unique_ptr<T> CreateModel(const std::vector<LidarState>& vecState, int numberData, int degree)
{
  // Cut to the number of data needed
  if (numberData && vecState.size() > numberData)
  {
    const std::vector<LidarState> necessaryData{vecState.end() - (1 + numberData), vecState.end() - 1};
    return (std::make_unique<T>(necessaryData, degree));
  }
  return (std::make_unique<T>(vecState, degree));
}

/**
 * @brief Compute a one-time interpolation for transformation at time t
 * @param vecState Dataset use for interpolation
 * @param time Time where to interpolate
 * @param model Model of interpolation (see enum Model)
 * @param onlyNecessary Does the model only use the necessary number of points to compute the interpolation
 */
Eigen::Isometry3d ComputeTransfo(const std::vector<LidarState>& vecState, double time,
                                       Model model, bool onlyNecessary = true);

}  // end of Interpolation namespace
}  // end of LidarSlam namespace
