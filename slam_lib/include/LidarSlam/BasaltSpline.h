
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
// Using a spherical Bézier curves of cubic "degree"
// Using https://splines.readthedocs.io/en/latest/rotation/catmull-rom-non-uniform.html in C++
// Ctrl points : https://splines.readthedocs.io/en/latest/rotation/catmull-rom-non-uniform.html
// End points : https://splines.readthedocs.io/en/latest/rotation/end-conditions-natural.htm
class RotSpline : public IModel
{
  using SO3d = Sophus::SO3<double>;
  using Quaterniond = Eigen::Quaterniond;

  public:
    RotSpline() = delete;
    RotSpline(const std::vector<PoseStamped>& vecPose);

    // TODO handle invalid number of data (0 or 1)
    Eigen::Isometry3d operator()(double t) const;
    void RecomputeModel(const std::vector<PoseStamped> &vecPose);

  private:
    std::vector<Quaterniond> VecKnots;
    std::vector<std::pair<Quaterniond, Quaterniond>> VecCtrlPoints;
    std::vector<double> VecTime;

    // Tools methods

    // Convert the data from PoseStamped to Sophus rotation
    // Make sure the angle between rotations are less than 180 degrees
    void IntegrateData(const std::vector<PoseStamped>& vecPose);

    // Create the controls points for the index of the
    void CreateCtrlPts(void);

    // Make the current quaternion to an angle < 180deg to prev
    void Canonicalized(const Quaterniond &prev, Quaterniond &current);

    static Eigen::Quaterniond exp_map(const Eigen::Vector3d &tangent);

    static Eigen::Vector3d log_map(const Eigen::Quaterniond &quat);

    static Eigen::Quaterniond DeCasteljau(const Quaterniond& q0, const Quaterniond& q1,
                                          const Quaterniond& q2, const Quaterniond& q3,
                                          double t);
};

}  // end of Interpolation namespace
}  // end of LidarSlam namespace