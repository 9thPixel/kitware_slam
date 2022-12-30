
#include "LidarSlam/BasaltSpline.h"
#include <cmath>

namespace LidarSlam
{
namespace Interpolation
{

RotSpline::RotSpline(const std::vector<PoseStamped>& vecPose)
{
  this->RecomputeModel(vecPose);
}

void RotSpline::RecomputeModel(const std::vector<PoseStamped> &vecPose)
{
  this->IntegrateData(vecPose);
  this->CreateCtrlPts();
}

// Spherical Bézier curve of "degree" 3 made with De Casteljau algorithm
// Using 2 knot points : (q0, q3) and 2 control points : (q1, q2)
Eigen::Quaterniond RotSpline::DeCasteljau(const Quaterniond& q0, const Quaterniond& q1,
                                          const Quaterniond& q2, const Quaterniond& q3,
                                          double t)
{
  // First iteration
  auto slerp_0_1 = q0.slerp(t, q1);
  auto slerp_1_2 = q1.slerp(t, q2);
  auto slerp_2_3 = q2.slerp(t, q3);

  // Second iteration
  auto slerp_0_2 = slerp_0_1.slerp(t, slerp_1_2);
  auto slerp_1_3 = slerp_1_2.slerp(t, slerp_2_3);

  // //*TEST
  // std::cout << "data for DeCasteljau algorithm : " << std::endl;
  // std::cout << "q0 : " << q0.coeffs().transpose() << std::endl;
  // std::cout << "slerp_0_1" << slerp_0_1.coeffs().transpose() << std::endl;
  // std::cout << "q1 : " << q1.coeffs().transpose() << std::endl;
  // std::cout << "slerp_1_2" << slerp_1_2.coeffs().transpose() << std::endl;
  // std::cout << "slerp_0_2" << slerp_0_2.coeffs().transpose() << std::endl;
  // std::cout << "q2 : " << q2.coeffs().transpose() << std::endl;
  // std::cout << "slerp_2_3" << slerp_2_3.coeffs().transpose() << std::endl;
  // std::cout << "q3 : " << q3.coeffs().transpose() << std::endl;
  // std::cout << "slerp_1_3" << slerp_1_3.coeffs().transpose() << std::endl;
  // std::cout << "t : " << t << std::endl;
  // std::cout << "result : " << slerp_0_2.slerp(t, slerp_1_3).coeffs().transpose() << std::endl;
  // //*TEST

  // Final iteration
  return slerp_0_2.slerp(t, slerp_1_3);
}

Eigen::Isometry3d RotSpline::operator()(double t) const
{
  // Handle the case of bad data for interpolation
  if (this->VecKnots.size() == 0)
    return Eigen::Isometry3d::Identity();
  else if (this->VecKnots.size() == 1)
    return Eigen::Translation3d::Identity() * this->VecKnots[0];
  else if (this->VecKnots.size() == 2)
  {
    double t_norm = (t - this->VecTime[0] / (this->VecTime[1] - this->VecTime[0]));
    return Eigen::Translation3d::Identity() * this->VecKnots[0].slerp(t_norm, this->VecKnots[1]);
  }

  auto pairInd = BinarySearch(t, this->VecTime);
  size_t firstInd = pairInd.first, secondInd = pairInd.second;
  Quaterniond rot;
  double t_norm = (t - this->VecTime[firstInd]) / (this->VecTime[secondInd] - this->VecTime[firstInd]);

  // else if (firstInd == 0 && t < this->VecTime.front())
  //   rot = this->VecKnots[0].slerp(t_norm, this->VecKnots[1]);
  // else if (secondInd == this->VecTime.size() - 1 && t > this->VecTime.back())
  //   rot = this->VecKnots[this->VecKnots.size() - 2].slerp(t_norm, this->VecKnots.back());
  if (firstInd == secondInd)
    rot = this->VecKnots[firstInd];
  else
    rot = DeCasteljau(this->VecKnots[firstInd], this->VecCtrlPoints[firstInd].second,
                      this->VecCtrlPoints[secondInd].first, this->VecKnots[secondInd],
                      t_norm);
  return (Eigen::Translation3d::Identity() * rot);
}

// ------------------ Tools Methods ------------------


void RotSpline::IntegrateData(const std::vector<PoseStamped>& vecPose)
{
  if (vecPose.empty())
  {
    PRINT_ERROR("No data for rotation Spline");
    return ;
  }
  else if (vecPose.size() == 1)
  {
    PRINT_ERROR("Only one data for rotation Spline");
    this->VecKnots.emplace_back(vecPose[0].Pose.linear());
    this->VecTime.push_back(vecPose[0].Time);
    return;
  }

  auto prevQuat = Quaterniond::Identity();
  for (auto it = vecPose.begin(); it != vecPose.end(); ++it)
  {
    Quaterniond quat{it->Pose.linear()};
    // canonilize the quaternions so the angle is always inferior to 180 degrees
    Canonicalized(prevQuat, quat);
    // std::cout << "dot product : " << prevQuat.dot(quat) << std::endl;
    prevQuat = quat;
    this->VecKnots.push_back(quat);
    this->VecTime.push_back(it->Time);
  }
  // std::cout << "knots nb : " << this->VecKnots.size() << " time nb : " << this->VecTime.size() << std::endl;
}

/**
 * Create Controls points from Knots points and time
 * Using https://splines.readthedocs.io/en/latest/rotation/catmull-rom-non-uniform.html
 * and   https://splines.readthedocs.io/en/latest/rotation/end-conditions-natural.html
 */
void RotSpline::CreateCtrlPts(void)
{
  if (this->VecKnots.size() <= 2)
    return;

  // Create Time diff array : ti+1 - ti : delta_i
  std::vector<double> vecDiffTime;
  for (size_t i = 0; i < this->VecTime.size() - 1; ++i)
    vecDiffTime.push_back(this->VecTime[i+1] - this->VecTime[i]);

  // Create rho array
  // Formula : rho_i = log_mat(qi+1 * qi^-1) / (ti+1 - ti)
  std::vector<SO3d::Tangent> vecRho;
  for (size_t i = 0; i < this->VecKnots.size() - 1; ++i)
  {
    auto rho = log_map(this->VecKnots[i + 1] * this->VecKnots[i].inverse()) / vecDiffTime[i];
    vecRho.emplace_back(rho);
  }

  // Create Omega array
  // Formula : omega_i = ((ti+1 - ti)rho_i_1 + (ti - ti_1)rho_i) / (ti+1 - ti_1)
  std::vector<SO3d::Tangent> vecOmega;
  for (size_t i = 1; i < this->VecKnots.size() - 1; ++i)
  {
    auto omega = (vecDiffTime[i] * vecRho[i-1] + vecDiffTime[i-1] * vecRho[i])
    / (this->VecTime[i+1] - this->VecTime[i-1]);
    vecOmega.push_back(omega);
  }

  // Create Control points
  // Formula : ctr_pt+ = exp(diffTime_i / 3 * omega_i) * qi
  // Formula : ctr_pt- = exp(diffTime_i_1 / 3 * omega_i)^-1 * qi
  for (size_t i = 1; i < this->VecKnots.size() - 1; ++i)
  {
    Quaterniond qi_plus  = exp_map(vecDiffTime[i] / 3. * vecOmega[i - 1]) * this->VecKnots[i];
    Quaterniond qi_minus = exp_map(vecDiffTime[i - 1] / 3. * vecOmega[i - 1]).inverse() * this->VecKnots[i];

    // Start Point : ctr_pt0+ = slerp(q0, ctrl_pt1-, 0.5)
    if (i == 1)
    {
      Quaterniond q0_plus = this->VecKnots.front().slerp(0.5, qi_minus);
      Canonicalized(Quaterniond::Identity(), q0_plus);
      this->VecCtrlPoints.emplace_back(Quaterniond::Identity(), q0_plus);
    }
    Canonicalized(this->VecKnots[i - 1], qi_minus);
    Canonicalized(this->VecKnots[i], qi_plus);
    this->VecCtrlPoints.emplace_back(qi_minus, qi_plus);

    // End Point : ctr_pt(N-1)- = slerp(q(N-1), ctrl_pt(N-2)+, 0.5)
    if (i == this->VecKnots.size() - 2)
    {
      Quaterniond qN_minus{this->VecKnots.back().slerp(0.5, qi_plus)};
      Canonicalized(this->VecKnots[i - 1], qN_minus);
      this->VecCtrlPoints.emplace_back(qN_minus, Quaterniond::Identity());
    }
  }

  // test control points
  std::cout << "TEST CONTROL POINTS " << std::endl;
  std::cout << "Nb ctrl points : " << this->VecCtrlPoints.size() << std::endl;
  for (int i = 0; i < this->VecCtrlPoints.size(); ++i)
  {
    printf("q%dminus : ", i);
    std::cout << this->VecCtrlPoints[i].first.coeffs().transpose() << std::endl;
    printf("q%d(t=%lf) : ", i, this->VecTime[i]);
    std::cout << this->VecKnots[i].coeffs().transpose() << std::endl;
    printf("q%dplus : ", i);
    std::cout << this->VecCtrlPoints[i].second.coeffs().transpose() << std::endl;
  }
  std::cout << std::endl << std::endl;
}

Eigen::Quaterniond RotSpline::exp_map(const Eigen::Vector3d &tangent)
{
  double norm = tangent.norm() / 2;
  double sin_norm = sin(norm) / norm / 2;

  // if (!SO3d::exp(tangent).matrix().isApprox(Eigen::Quaterniond(cos(norm), tangent(0) * sin_norm, tangent(1) * sin_norm, tangent(2) * sin_norm).matrix()))
  //   std::cout << "exp is false" << std::endl;
  // else
  //   std::cout << "exp is true" << std::endl;

  return (Eigen::Quaterniond(cos(norm), tangent(0) * sin_norm, tangent(1) * sin_norm, tangent(2) * sin_norm).normalized());
}

// Inspired of https://splines.readthedocs.io/en/latest/_modules/splines/quaternion.html#UnitQuaternion.log_map
Eigen::Vector3d RotSpline::log_map(const Eigen::Quaterniond &quat)
{

  if (std::abs(quat.norm() - 1.) > 1e-6)
    return (Eigen::Vector3d::Zero());
  Eigen::AngleAxis<double> angleAxis(quat);

  // if (!SO3d{quat}.log().isApprox(angleAxis.angle() * angleAxis.axis()))
  //   std::cout << "log is false" << std::endl;
  // else
  //   std::cout << "log is true" << std::endl;

  return (angleAxis.angle() * angleAxis.axis());
}


void RotSpline::Canonicalized(const Quaterniond &prev, Quaterniond &current)
{
  if (prev.dot(current) <  -1e-6)
    current.coeffs() = current.coeffs() * -1;
}


}  // end of Interpolation namespace
}  // end of LidarSlam namespace