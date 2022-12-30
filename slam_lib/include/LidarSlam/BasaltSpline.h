
#pragma once

#include "LidarSlam/InterpolationModels.h"

#include <basalt/spline/rd_spline.h>
#include <basalt/spline/so3_spline.h>
#include <sophus/so3.hpp>

namespace LidarSlam
{
namespace Interpolation
{
// ---------------------------------------------------------------------------
//   Rotation Models
// ---------------------------------------------------------------------------

// ---------------------------- Rotation Spline --------------------------------

// Non-Uniform Catmull–Rom-Like Rotation Splines
// Using a spherical Bézier curves of "degree" cubic
// Using https://splines.readthedocs.io/en/latest/rotation/catmull-rom-non-uniform.html in C++

class RotSpline : public IModel
{
  using SO3d = Sophus::SO3<double>;

  public:
    RotSpline() = delete;
    RotSpline(const std::vector<PoseStamped>& vecPose)
    {
      // for (auto it = vecPose.begin(); it != vecPose.end(); ++it)
      // {
      //     VecKnots.push_back(SO3d{Eigen::Quaterniond{it->Pose.linear()}});
      //   // if (it != vecPose.begin())
      //   //TODO canonize the quaternions (see slines site)
      // }
    }
    Eigen::Isometry3d operator()(double t) const
    {
      return (Eigen::Isometry3d::Identity());
    }
    void RecomputeModel(const std::vector<PoseStamped> &vecPose)
    {
      (void)vecPose;
    }

  private:
    std::vector<SO3d> VecKnots;
    std::vector<std::pair<SO3d, SO3d>> VecCtrlPoints;
    std::vector<double> VecTime;
};

}  // end of Interpolation namespace
}  // end of LidarSlam namespace