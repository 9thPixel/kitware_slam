
#pragma once

#include "LidarSlam/InterpolationModels.h"

#include <basalt/spline/rd_spline.h>
#include <basalt/spline/so3_spline.h>
namespace LidarSlam
{
namespace Interpolation
{

// TODO check time conversion is correct everywhere
class BasaltSO3Quad : public IModel
{
  struct RotStamped
  {
    Eigen::Quaterniond Rot;
    double Time;
  };
  public:
    BasaltSO3Quad() = delete;
    BasaltSO3Quad(const std::vector<PoseStamped>& vecPose);
    Eigen::Isometry3d operator()(double t) const;
    void RecomputeModel(const std::vector<PoseStamped> &vecPose);

  private:
    const int64_t S_TO_NS = 1e9;
    basalt::So3Spline<2> RotSpline = {S_TO_NS};
};

}  // end of Interpolation namespace
}  // end of LidarSlam namespace