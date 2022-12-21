
#include "LidarSlam/BasaltSpline.h"

namespace LidarSlam
{
namespace Interpolation
{

BasaltSO3Quad::BasaltSO3Quad(const std::vector<PoseStamped>& vecPose)
{
  this->RecomputeModel(vecPose);
}

Eigen::Isometry3d BasaltSO3Quad::operator()(double t) const
{
  int64_t i_t = t * 1e9;
  if (t < this->RotSpline.minTimeNs() || t > this->RotSpline.maxTimeNs())
  {
    PRINT_ERROR("Basalt is unable to extrapolate. Use other interpolation model instead")
    return (Eigen::Isometry3d::Identity());
  }
  Eigen::Quaterniond rot = this->RotSpline.evaluate(i_t).unit_quaternion();
  return (Eigen::Translation3d::Identity() * rot);
}

void BasaltSO3Quad::RecomputeModel(const std::vector<PoseStamped> &vecPose)
{
  if (vecPose.empty())
  {
    PRINT_ERROR("No data for Basalt rotation interpolation, use null data");
    Sophus::SO3<double> vecIdentity;
    this->RotSpline = {S_TO_NS};
    this->RotSpline.knotsPushBack(vecIdentity);
    this->RotSpline.knotsPushBack(vecIdentity);
    return;
  }
  if (vecPose.size() == 1)
  {
    PRINT_ERROR("Only one data for Basalt rotation interpolation, perform constant interpolation");
    this->RotSpline = {S_TO_NS, static_cast<int64_t>(vecPose[0].Time * S_TO_NS) - S_TO_NS / 2};
    this->RotSpline.knotsPushBack({vecPose[0].Pose.linear()});
    this->RotSpline.knotsPushBack({vecPose[0].Pose.linear()});
    return;
  }
  if (vecPose.size() > 3)
    PRINT_WARNING("Basalt interpolation has more than 3 rotations, only use the last ones");
  std::vector<RotStamped> vecRot;
  vecRot.push_back({Eigen::Quaterniond{(vecPose.end() - 3)->Pose.linear()}, (vecPose.end() - 3)->Time});
  vecRot.push_back({Eigen::Quaterniond{(vecPose.end() - 2)->Pose.linear()}, (vecPose.end() - 2)->Time});
  vecRot.push_back({Eigen::Quaterniond{(vecPose.end() - 1)->Pose.linear()}, (vecPose.end() - 1)->Time});

  double dt10 = vecRot[1].Time - vecRot[0].Time;
  double dt21 = vecRot[2].Time - vecRot[1].Time;
  
  // Uniform the 3 measures by linearly interpolate a measure
  // TODO test if uniformization works => print times after uniformization
  // TODO after it, delete struct and use Quaterniond vector instead
  if (dt21 > dt10)
  {
    double ratio = dt10 / dt21;
    vecRot[2].Rot = vecRot[1].Rot.slerp(ratio, vecRot[2].Rot);
    vecRot[2].Time = vecRot[1].Time + ratio * dt21;
  }
  else if (dt10 > dt21)
  {
    double ratio = 1 - dt21 / dt10;
    vecRot[0].Rot = vecRot[0].Rot.slerp(ratio, vecRot[1].Rot);
    vecRot[0].Time = vecRot[0].Time + ratio * dt10;
  }

  this->RotSpline = {static_cast<int64_t>(vecRot[0].Time)};
  this->RotSpline.knotsPushBack(Sophus::SO3d{vecRot[0].Rot});
  this->RotSpline.knotsPushBack(Sophus::SO3d{vecRot[1].Rot});
  this->RotSpline.knotsPushBack(Sophus::SO3d{vecRot[2].Rot});
}

}  // end of Interpolation namespace
}  // end of LidarSlam namespace