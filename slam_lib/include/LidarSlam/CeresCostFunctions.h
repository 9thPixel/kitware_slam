//==============================================================================
// Copyright 2018-2020 Kitware, Inc., Kitware SAS
// Author: Guilbert Pierre (Kitware SAS)
//         Cadart Nicolas (Kitware SAS)
// Creation date: 2018-03-27
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

// CERES
#include <ceres/ceres.h>
// EIGEN
#include <Eigen/Geometry>

//------------------------------------------------------------------------------
/**
 * \brief Factory to ease the construction of the auto-diff residual objects
 *
 * Type: the residual functor type
 * ResidualSize: int, the size of the output residual block
 * ...: int(s), the size(s) of the input parameter block(s)
 */
#define RESIDUAL_FACTORY(Type, ResidualSize, ...) \
  template<typename ...Args> \
  static std::shared_ptr<ceres::CostFunction> Create(Args&& ...args) \
  {return std::make_shared<ceres::AutoDiffCostFunction<Type, ResidualSize, __VA_ARGS__>>(new Type(args...));}


namespace LidarSlam
{
namespace CeresTools
{
struct Residual
{
  std::shared_ptr<ceres::CostFunction> Cost;
  std::shared_ptr<ceres::LossFunction> Robustifier;
};
}
namespace CeresCostFunctions
{

namespace Utils
{
namespace
{
//------------------------------------------------------------------------------
/**
 * \brief Build rotation matrix from Euler angles.
 *
 * It estimates R using the Euler-Angle mapping between R^3 and SO(3) :
 *   R(rx, ry, rz) = Rz(rz) * Ry(ry) * Rx(rx)
 */
template <typename T>
Eigen::Matrix<T, 3, 3> RotationMatrixFromRPY(const T& rx, const T& ry, const T& rz)
{
  const T cx = ceres::cos(rx);  const T sx = ceres::sin(rx);
  const T cy = ceres::cos(ry);  const T sy = ceres::sin(ry);
  const T cz = ceres::cos(rz);  const T sz = ceres::sin(rz);

  Eigen::Matrix<T, 3, 3> R;
  R << cy*cz,  sx*sy*cz-cx*sz,  cx*sy*cz+sx*sz,
       cy*sz,  sx*sy*sz+cx*cz,  cx*sy*sz-sx*cz,
         -sy,           sx*cy,           cx*cy;
  return R;
}

//------------------------------------------------------------------------------
template <typename T>
Eigen::Transform<T, 3, Eigen::Isometry> XYZRPYtoIsometry(const T& x, const T& y, const T& z, const T& rx, const T& ry, const T& rz)
{
  const T cx = ceres::cos(rx);  const T sx = ceres::sin(rx);
  const T cy = ceres::cos(ry);  const T sy = ceres::sin(ry);
  const T cz = ceres::cos(rz);  const T sz = ceres::sin(rz);

  Eigen::Transform<T, 3, Eigen::Isometry> transform;
  transform.matrix() << cy*cz,  sx*sy*cz-cx*sz,  cx*sy*cz+sx*sz,      x,
                        cy*sz,  sx*sy*sz+cx*cz,  cx*sy*sz-sx*cz,      y,
                          -sy,           sx*cy,           cx*cy,      z,
                        T(0.),           T(0.),           T(0.),  T(1.);
  return transform;
}

//------------------------------------------------------------------------------
template <typename T>
Eigen::Matrix<T, 6, 1> IsometryToXYZRPY(const Eigen::Transform<T, 3, Eigen::Isometry>& transform)
{
  Eigen::Matrix<T, 3, 3> rot = transform.linear();
  Eigen::Matrix<T, 3, 1> rpy;
  rpy.x() = ceres::atan2(rot(2, 1), rot(2, 2));
  rpy.y() = -ceres::asin(rot(2, 0));
  rpy.z() = ceres::atan2(rot(1, 0), rot(0, 0));
  Eigen::Matrix<T, 6, 1> xyzrpy;
  xyzrpy << transform.translation(), rpy;
  return xyzrpy;
}

//------------------------------------------------------------------------------
template <typename T>
Eigen::Transform<T, 3, Eigen::Isometry> XYZQuatToIsometry(const T& x, const T& y, const T& z, const T& qx, const T& qy, const T& qz, const T& qw)
{
  return Eigen::Translation<T, 3>(x, y, z) * Eigen::Quaternion<T>(qw, qx, qy, qz).normalized();
}

//------------------------------------------------------------------------------
template <typename T>
Eigen::Matrix<T, 7, 1> IsometryToXYZQuat(const Eigen::Transform<T, 3, Eigen::Isometry>& transform)
{
  Eigen::Matrix<T, 7, 1> xyzQuat;
  xyzQuat.head(3) = transform.translation();
  Eigen::Quaternion<T> quat(transform.linear());
  xyzQuat.tail(4) << quat.x(), quat.y(), quat.z(), quat.w();
  xyzQuat.tail(4).normalize();
  return xyzQuat;
}

} // end of anonymous namespace
} // end of Utils namespace

//------------------------------------------------------------------------------
/**
 * \class MahalanobisDistanceAffineIsometryResidual
 * \brief Cost function to optimize the affine isometry transformation
 *        (rotation and translation) so that the mahalanobis distance
 *        between a source point and a target model is minimized.
 *
 * In case the user is interested in implementing a cost function:
 *     cost(x) = (Y - P)^t C^{-1} (Y - P)
 * where:
 *  - Y = (R X + T) is the source point X transformed by the isometry (R, T) to optimize
 *  - P is a point laying on the target model
 *  - C is the covariance matrix of the target model,
 * then A = C^{-1/2}, i.e the matrix A is the square root of the inverse of the
 * covariance, also known as the stiffness matrix.
 *
 * This function takes one 6D parameters block :
 *   - 3 first parameters to encode translation : X, Y, Z
 *   - 3 last parameters to encode rotation with euler angles : rX, rY, rZ
 *
 * It outputs a 3D residual block.
 */
struct MahalanobisDistanceAffineIsometryResidual
{
  MahalanobisDistanceAffineIsometryResidual(const Eigen::Matrix3d& argA,
                                            const Eigen::Vector3d& argP,
                                            const Eigen::Vector3d& argX)
    : A(argA)
    , P(argP)
    , X(argX)
  {}

  template <typename T>
  bool operator()(const T* const w, T* residual) const
  {
    using Matrix3T = Eigen::Matrix<T, 3, 3>;
    using Vector3T = Eigen::Matrix<T, 3, 1>;

    // Get translation part
    Eigen::Map<const Vector3T> trans(&w[0]);

    // Get rotation part, in a static way.
    // The idea is that all residual functions will need to evaluate those
    // sin/cos so we only compute them once each time the parameters change.
    static Matrix3T rot = Matrix3T::Identity();
    static T lastRot[3] = {T(-1.), T(-1.), T(-1.)};
    if (!std::equal(w + 3, w + 6, lastRot))
    {
      rot = Utils::RotationMatrixFromRPY(w[3], w[4], w[5]);
      std::copy(w + 3, w + 6, lastRot);
    }

    // Transform point with rotation and translation
    const Vector3T Y = rot * X + trans;

    // Compute residual
    Eigen::Map<Vector3T> residualVec(residual);
    residualVec = A * (Y - P);

    return true;
  }

  // Factory to ease the construction of the auto-diff residual object
  RESIDUAL_FACTORY(MahalanobisDistanceAffineIsometryResidual, 3, 6)

private:
  const Eigen::Matrix3d A;
  const Eigen::Vector3d P;
  const Eigen::Vector3d X;
};

//------------------------------------------------------------------------------
/**
 * \class MahalanobisDistanceInterpolatedMotionResidual
 * \brief Cost function to optimize the affine isometry transformations
 *        (R0, T0) and (R1, T1) at timestamps t0=0 and t1=1 so that the
 *        linearly interpolated transform (R, T) applied to source point X
 *        acquired at time t minimizes the mahalanobis distance.
 *
 * In case the user is interested in implementing a cost function:
 *     cost(x) = (Y - P)^t C^{-1} (Y - P)
 * where:
 *  - Y = (R X + T) is the source point X transformed by the interpolated isometry
 *     (R, T) = (R0^(1-t) * R1^t, (1 - t) T0 + t T1)
 *  - P is a point laying on the target model
 *  - C is the covariance matrix of the target model,
 * then, A = C^{-1/2}, i.e the matrix A is the square root of the inverse of the
 * covariance, also known as the stiffness matrix.
 *
 * This function takes two 6D parameters blocks :
 *  1) First isometry H0 :
 *   - 3 parameters (0, 1, 2) to encode translation T0 : X, Y, Z
 *   - 3 parameters (3, 4, 5) to encode rotation R0 with euler angles : rX, rY, rZ
 *  2) Second isometry H1 :
 *   - 3 parameters (6, 7, 8) to encode translation T1 : X, Y, Z
 *   - 3 parameters (9, 10, 11) to encode rotation R1 with euler angles : rX, rY, rZ
 *
 * It outputs a 3D residual block.
 */
struct MahalanobisDistanceInterpolatedMotionResidual
{
  MahalanobisDistanceInterpolatedMotionResidual(const Eigen::Matrix3d& argA,
                                                const Eigen::Vector3d& argP,
                                                const Eigen::Vector3d& argX,
                                                double argTime)
    : A(argA)
    , P(argP)
    , X(argX)
    , Time(argTime)
  {}

  template <typename T>
  bool operator()(const T* const w0, const T* const w1, T* residual) const
  {
    using Vector3T = Eigen::Matrix<T, 3, 1>;
    using Isometry3T = Eigen::Transform<T, 3, Eigen::Isometry>;
    using Translation3T = Eigen::Translation<T, 3>;
    using QuaternionT = Eigen::Quaternion<T>;

    // Create H0 / H1 transforms in static way.
    // The idea is that all residual functions will need to
    // evaluate those variables so we will only compute then
    // once each time the parameters values change
    static Isometry3T H0 = Isometry3T::Identity(), H1 = Isometry3T::Identity();
    static T lastW0[6] = {T(-1.), T(-1.), T(-1.), T(-1.), T(-1.), T(-1.)};
    static T lastW1[6] = {T(-1.), T(-1.), T(-1.), T(-1.), T(-1.), T(-1.)};

    // Update H0 if needed
    if (!std::equal(w0, w0 + 6, lastW0))
    {
      H0.linear() << Utils::RotationMatrixFromRPY(w0[3], w0[4], w0[5]);
      H0.translation() << w0[0], w0[1], w0[2];
      std::copy(w0, w0 + 6, lastW0);
    }

    // Update H1 if needed
    if (!std::equal(w1, w1 + 6, lastW1))
    {
      H1.linear() << Utils::RotationMatrixFromRPY(w1[3], w1[4], w1[5]);
      H1.translation() << w1[0], w1[1], w1[2];
      std::copy(w1, w1 + 6, lastW1);
    }

    // Compute the transform to apply to X depending on (R0, T0) and (R1, T1).
    // The applied isometry will be the linear interpolation between them :
    // (R, T) = (R0^(1-t) * R1^t, T0 + (T1 - T0)t
    Isometry3T H;
    // Avoid numerical issues
    if (H0.isApprox(H1))
      H = H0;
    else
    {
      Translation3T trans(H0.translation() + Time * (H1.translation() - H0.translation()));
      QuaternionT rot(Eigen::Quaterniond(H0.linear()).slerp(Time, Eigen::Quaterniond(H1.linear())));
      H = trans * rot;
    }

    // Transform point with rotation and translation
    const Vector3T Y = H.linear() * X + H.translation();

    // Compute residual
    Eigen::Map<Vector3T> residualVec(residual);
    residualVec = A * (Y - P);

    return true;
  }

  // Factory to ease the construction of the auto-diff residual object
  RESIDUAL_FACTORY(MahalanobisDistanceInterpolatedMotionResidual, 3, 6, 6)


private:
  const Eigen::Matrix3d A;
  const Eigen::Vector3d P;
  const Eigen::Vector3d X;
  const double Time;
};


//------------------------------------------------------------------------------
/**
 * \class OdometerDistanceResidual
 * \brief Cost function to optimize the translation of the affine isometry
 *        transformation (rotation and translation) so that the distance
 *        from a previous known pose corresponds to an external sensor odometry measure.
 *
 * This function takes one 6D parameters block :
 *   - 3 first parameters to encode translation : X, Y, Z
 *   - [unused] 3 last parameters to encode rotation with euler angles : rX, rY, rZ
 *
 * It outputs a 1D residual block.
 */
struct OdometerDistanceResidual
{
  OdometerDistanceResidual(const Eigen::Vector3d& refPos,
                           const Eigen::Isometry3d& calibration,
                           double distance)
    : RefPos(refPos)
    , Calibration(calibration)
    , Distance(distance)
  {}

  template <typename T>
  bool operator()(const T* const w, T* residual) const
  {
    using Vector3T = Eigen::Matrix<T, 3, 1>;
    using Isometry3T = Eigen::Transform<T, 3, Eigen::Isometry>;

    // Compute transform matrix of current pose from position and Euler angles (RPY convention)
    Isometry3T basePoseTransform = Utils::XYZRPYtoIsometry(w[0], w[1], w[2], w[3], w[4], w[5]);

    // Compute residual
    Isometry3T wheelPoseTransform = basePoseTransform * this->Calibration.cast<T>();
    *residual = (wheelPoseTransform.translation() - this->RefPos.cast<T>()).squaredNorm() - this->Distance * this->Distance;

    return true;
  }

  // Factory to ease the construction of the auto-diff residual object
  RESIDUAL_FACTORY(OdometerDistanceResidual, 1, 6)

private:
  const Eigen::Vector3d RefPos;
  const Eigen::Isometry3d Calibration;
  const double Distance;
};

//------------------------------------------------------------------------------
/**
 * \class OdometerTranslationResidual
 * \brief Cost function to optimize the affine isometry
 *        transformation (rotation and translation) so that the translation
 *        fits a wheel encoder measurement
 *
 * This function takes one 6D parameters block :
 *   - 3 first parameters to encode translation : X, Y, Z
 *   - [unused] 3 last parameters to encode rotation with euler angles : rX, rY, rZ
 *
 * It outputs a 3D residual block.
 */
struct OdometerTranslationResidual
{
  OdometerTranslationResidual(const Eigen::Vector3d& refPos,
                              const Eigen::Isometry3d& calibration,
                              const Eigen::Vector3d& translation)
    : RefPos(refPos)
    , Calibration(calibration)
    , Translation(translation)
  {}

  template <typename T>
  bool operator()(const T* const w, T* residual) const
  {
    using Vector3T = Eigen::Matrix<T, 3, 1>;
    using Isometry3T = Eigen::Transform<T, 3, Eigen::Isometry>;

    // Compute transform matrix of current pose from position and Euler angles (RPY convention)
    Isometry3T basePoseTransform = Utils::XYZRPYtoIsometry(w[0], w[1], w[2], w[3], w[4], w[5]);

    // Compute residual
    Isometry3T wheelPoseTransform = basePoseTransform * this->Calibration.cast<T>();
    Eigen::Map<Vector3T> residualVec(residual);
    residualVec = wheelPoseTransform.translation() - (this->RefPos + this->Translation).cast<T>();

    return true;
  }

  // Factory to ease the construction of the auto-diff residual object
  RESIDUAL_FACTORY(OdometerTranslationResidual, 3, 6)

private:
  const Eigen::Vector3d RefPos;
  const Eigen::Isometry3d Calibration;
  const Eigen::Vector3d Translation;
};

//------------------------------------------------------------------------------
/**
 * \class ImuGravityAlignmentResidual
 * \brief Cost function to optimize the orientation of the affine
 *        isometry transformation (rotation and translation) so
 *        that the gravity vector corresponds to a reference.
 *        This gravity vector is usually supplied by an IMU.
 *
 * This function takes one 6D parameters block :
 *   - [unused] 3 first parameters to encode translation : X, Y, Z
 *   - 3 last parameters to encode rotation with euler angles : rX, rY, rZ
 *
 * It outputs a 3D residual block.
 */
struct ImuGravityAlignmentResidual
{
  ImuGravityAlignmentResidual(const Eigen::Vector3d& refGravityDir,
                              const Eigen::Vector3d& currGravityDir)
    : ReferenceGravityDirection(refGravityDir)
    , CurrentGravityDirection(currGravityDir)
  {}

  template <typename T>
  bool operator()(const T* const w, T* residual) const
  {
    using Matrix3T = Eigen::Matrix<T, 3, 3>;
    using Vector3T = Eigen::Matrix<T, 3, 1>;

    // Get rotation part
    Matrix3T rot = Utils::RotationMatrixFromRPY(w[3], w[4], w[5]);

    // Compute residual
    Eigen::Map<Vector3T> residualVec(residual);
    residualVec = rot * CurrentGravityDirection - ReferenceGravityDirection;

    return true;
  }

  // Factory to ease the construction of the auto-diff residual object
  RESIDUAL_FACTORY(ImuGravityAlignmentResidual, 3, 6)

private:
  const Eigen::Vector3d ReferenceGravityDirection;
  const Eigen::Vector3d CurrentGravityDirection;
};

//------------------------------------------------------------------------------
/**
 * \class LandmarkPositionResidual
 * \brief Cost function to optimize the affine isometry transformation
 *        (rotation and translation) so that it is consistent
 *        with a landmark detection (knowing its absolute position)
 *
 * This function takes one 6D parameters block :
 *   - 3 parameters to encode translation : X, Y, Z
 *   - 3 last parameters to encode rotation with Euler angles : rX, rY, rZ
 *
 * It outputs a 3D residual block.
 */
struct LandmarkPositionResidual
{
  using Vector6d = Eigen::Matrix<double, 6, 1>;
  using Matrix6d = Eigen::Matrix<double, 6, 6>;

  LandmarkPositionResidual(const Eigen::Isometry3d& relativeTransform,
                           const Vector6d& absolutePose)
    : RelativeTransform(relativeTransform)
    , AbsolutePoseRef(absolutePose)
  {}

  template <typename T>
  bool operator()(const T* const w, T* residual) const
  {
    using Vector3T = Eigen::Matrix<T, 3, 1>;
    using Isometry3T = Eigen::Transform<T, 3, Eigen::Isometry>;

    // Get transformation, in a static way.
    // The idea is that all landmark residual functions will need to evaluate those
    // sin/cos so we only compute them once each time the parameters change.
    static Isometry3T currentPoseTransfo = Isometry3T::Identity();
    static T lastT[6] = {T(-1.)};
    if (!std::equal(w, w + 6, lastT))
    {
      currentPoseTransfo = Utils::XYZRPYtoIsometry(w[0], w[1], w[2], w[3], w[4], w[5]);
      std::copy(w, w + 6, lastT);
    }

    // Compute absolute pose
    Isometry3T tagTransform = currentPoseTransfo * this->RelativeTransform.cast<T>();

    // Compute residual
    Eigen::Map<Vector3T> residualVec(residual);
    residualVec = tagTransform.translation() - this->AbsolutePoseRef.head(3);

    return true;
  }

  // Factory to ease the construction of the auto-diff residual object
  RESIDUAL_FACTORY(LandmarkPositionResidual, 3, 6)

private:
  const Eigen::Isometry3d RelativeTransform;
  const Vector6d AbsolutePoseRef;
};

//------------------------------------------------------------------------------
/**
 * \class LandmarkResidual
 * \brief Cost function to optimize the affine isometry transformation
 *        (rotation and translation) so that the absolute pose is consistent
 *        with a landmark detection (knowing its absolute pose)
 *
 * This function takes one 6D parameters block :
 *   - 3 first parameters to encode translation : X, Y, Z
 *   - 3 last parameters to encode rotation with Euler angles : rX, rY, rZ
 *
 * It outputs a 6D residual block.
 */
struct LandmarkResidual
{
  using Vector6d = Eigen::Matrix<double, 6, 1>;
  using Matrix6d = Eigen::Matrix<double, 6, 6>;

  LandmarkResidual(const Eigen::Isometry3d& relativeTransform,
                   const Vector6d& absolutePose)
    : RelativeTransform(relativeTransform)
    , AbsolutePoseRef(absolutePose)
  {}

  template <typename T>
  bool operator()(const T* const w, T* residual) const
  {
    using Vector6T = Eigen::Matrix<T, 6, 1>;
    using Isometry3T = Eigen::Transform<T, 3, Eigen::Isometry>;

    // Get transformation, in a static way.
    // The idea is that all landmark residual functions will need to evaluate those
    // sin/cos so we only compute them once each time the parameters change.
    static Isometry3T currentPoseTransfo = Isometry3T::Identity();
    static T lastT[6] = {T(-1.)};
    if (!std::equal(w, w + 6, lastT))
    {
      currentPoseTransfo = Utils::XYZRPYtoIsometry(w[0], w[1], w[2], w[3], w[4], w[5]);
      std::copy(w, w + 6, lastT);
    }

    // Compute absolute pose
    Isometry3T tagTransform = currentPoseTransfo * this->RelativeTransform.cast<T>();
    Vector6T tagPose = Utils::IsometryToXYZRPY(tagTransform);

    // Compute residual
    Eigen::Map<Vector6T> residualVec(residual);
    residualVec = (tagPose - this->AbsolutePoseRef);

    return true;
  }

  // Factory to ease the construction of the auto-diff residual object
  RESIDUAL_FACTORY(LandmarkResidual, 6, 6)

private:
  const Eigen::Isometry3d RelativeTransform;
  const Vector6d AbsolutePoseRef;
};

//------------------------------------------------------------------------------
/**
 * \class ExternalPoseResidual
 * \brief Cost function to optimize the affine isometry transformation
 *        (rotation and translation) so that the absolute pose is consistent
 *        with an external pose measurement by other sensor.
 *
 * This function takes one 6D parameters block :
 *   - 3 first parameters to encode translation : X, Y, Z
 *   - 3 last parameters to encode rotation with Euler angles : rX, rY, rZ
 *
 * It outputs a 6D residual block.
 */
struct ExternalPoseResidual
{
  using Vector6d = Eigen::Matrix<double, 6, 1>;

  ExternalPoseResidual(const Vector6d& extPose, const Eigen::Isometry3d& prevPose)
    : ExternalRelativePose(extPose),
      PrevPose(prevPose)
  {}

  template <typename T>
  bool operator()(const T* const w, T* residual) const
  {
    using Vector6T = Eigen::Matrix<T, 6, 1>;
    using Isometry3T = Eigen::Transform<T, 3, Eigen::Isometry>;

    // Get transformation, in a static way.
    // The idea is that all landmark residual functions will need to evaluate those
    // sin/cos so we only compute them once each time the parameters change.
    static Isometry3T currentPoseTransfo = Isometry3T::Identity();
    static T lastT[6] = {T(-1.)};
    if (!std::equal(w, w + 6, lastT))
    {
      currentPoseTransfo = Utils::XYZRPYtoIsometry(w[0], w[1], w[2], w[3], w[4], w[5]);
      std::copy(w, w + 6, lastT);
    }

    Isometry3T Trel = this->PrevPose.cast<T>().inverse() * currentPoseTransfo;

    // Compute residual
    Eigen::Map<Vector6T> residualVec(residual);
    residualVec = (Utils::IsometryToXYZRPY(Trel) - ExternalRelativePose.cast<T>());

    return true;
  }

  // Factory to ease the construction of the auto-diff residual object
  RESIDUAL_FACTORY(ExternalPoseResidual, 6, 6)

private:
  const Vector6d ExternalRelativePose;
  const Eigen::Isometry3d PrevPose;
};

//------------------------------------------------------------------------------
/**
 * \class CameraResidual
 * \brief Cost function to optimize the affine isometry transformation
 *        (rotation and translation) so that successive frames from an RGB camera
 *        are consistent using pixel matches based contraints
 *
 * This function takes one 6D parameters block :
 *   - 3 first parameters to encode translation : X, Y, Z
 *   - 3 last parameters to encode rotation with Euler angles : rX, rY, rZ
 *
 * It outputs a 6D residual block.
 */
struct CameraResidual
{
  CameraResidual(const Eigen::Vector3f& pt, const Eigen::Vector2f& pix,
                 const Eigen::Isometry3d& prevPoseTransfo,
                 const Eigen::Isometry3d& ext, const Eigen::Matrix3f& in)
    : Point3D (pt),
      Pixel(pix),
      PrevPoseTransfo(prevPoseTransfo),
      Extrinsic(ext),
      Intrinsic(in)
  {}

  template <typename T>
  bool operator()(const T* const w, T* residual) const
  {
    using Vector2T = Eigen::Matrix<T, 2, 1>;
    using Isometry3T = Eigen::Transform<T, 3, Eigen::Isometry>;

    // Get transformation, in a static way.
    // The idea is that all landmark residual functions will need to evaluate those
    // sin/cos so we only compute them once each time the parameters change.
    static Isometry3T currentPoseTransfo = Isometry3T::Identity();
    static T lastT[6] = {T(-1.)};
    if (!std::equal(w, w + 6, lastT))
    {
      currentPoseTransfo = Utils::XYZRPYtoIsometry(w[0], w[1], w[2], w[3], w[4], w[5]);
      std::copy(w, w + 6, lastT);
    }

    // Compute residual : pixel to pixel match
    Eigen::Map<Vector2T> residualVec(residual);
    Isometry3T prevBaseToCamera = this->PrevPoseTransfo.inverse().cast<T>() * currentPoseTransfo * this->Extrinsic.cast<T>();
    residualVec = (this->Intrinsic.cast<T>() * (prevBaseToCamera * this->Point3D.cast<T>())).hnormalized()
                  - this->Pixel.cast<T>();

    // Residual lower than 1 pixel are not taken into account
    // because the 3D point can be in any place inside the pixel of the first image
    // The pixel of the second image is built from the center of the first pixel (optical flow)
    if (residualVec.norm() < static_cast<T>(1.f))
      residualVec.Zero();

    return true;
  }

  // Factory to ease the construction of the auto-diff residual object
  RESIDUAL_FACTORY(CameraResidual, 2, 6)

private:
  // 3D Point in last pose frame
  Eigen::Vector3f Point3D;
  // pixel that must contain the 3D point in new frame
  Eigen::Vector2f Pixel;
  // Last pose frame transform
  Eigen::Isometry3d PrevPoseTransfo;
  // Extrinsic calibration
  Eigen::Isometry3d Extrinsic;
  // Intrinsic calibration
  Eigen::Matrix3f Intrinsic;
};

//------------------------------------------------------------------------------
/**
 * \class CalibPosesResidual
 * \brief Residual corresponding to a comparison between 2 poses (pose1 and pose2)
 * representing RELATIVE motion of two frames rigidly linked with unknown calibration offset
 * In the same global referential frame : transform(absPose1) = transform(absPose2) * Calib
 * Therefore, using relative motions : residual = pose6D(Calib * transform(pose1) * Calib^-1) - pose2
 *
 * This function takes one 7D parameters block :
 *   - 3 first parameters to encode translation : X, Y, Z
 *   - 4 last parameters to encode rotation with quaternions : qX, qY, qZ, qW
 *
 * It outputs a 7D residual block.
 */
struct CalibPosesResidual
{
  CalibPosesResidual(const Eigen::Isometry3d& pose1,
                     const Eigen::Isometry3d& pose2)
                    : Pose1Isometry(pose1)
  {
    this->Pose2.head<3>() = pose2.translation();
    Eigen::Quaterniond quat(pose2.linear());
    this->Pose2.tail<4>() << quat.x(), quat.y(), quat.z(), quat.w();
  }

  template <typename T>
  bool operator()(const T* const w, T* residual) const
  {
    using Vector7T = Eigen::Matrix<T, 7, 1>;
    using Isometry3T = Eigen::Transform<T, 3, Eigen::Isometry>;

    Isometry3T calibIsometry = Utils::XYZQuatToIsometry(w[0], w[1], w[2], w[3], w[4], w[5], w[6]);
    // Transform pose1 to get pose2 equivalent
    Vector7T pose1Transformed = Utils::IsometryToXYZQuat(calibIsometry * this->Pose1Isometry.cast<T>() * calibIsometry.inverse());

    // Compute residual
    Eigen::Map<Vector7T> residualVec(residual);
    residualVec = pose1Transformed - this->Pose2.cast<T>();

    return true;
  }

  // Factory to ease the construction of the auto-diff residual object
  RESIDUAL_FACTORY(CalibPosesResidual, 7, 7)

private:
  // Poses corresponding to RELATIVE motions of two rigidly
  // linked frames that move together in a global frame
  const Eigen::Isometry3d Pose1Isometry;
  Eigen::Matrix<double, 7, 1> Pose2;
};

//------------------------------------------------------------------------------
/**
 * \class CalibTransResidual
 * \brief Residual corresponding to the translation norm constraint
 * This function takes one 7D parameters block :
 *   - 3 first parameters to encode translation : X, Y, Z
 *   - 4 last parameters to encode rotation with quaternions : qX, qY, qZ, qW
 *
 * It outputs a 1D residual block.
 */
struct CalibTransResidual
{
  CalibTransResidual(double norm)
                    : TransNorm(norm)
  {}

  template <typename T>
  bool operator()(const T* const w, T* residual) const
  {
    T sqNorm = w[0] * w[0] + w[1] * w[1] + w[2] * w[2];

    // Compute residual
    if (sqNorm > T(1e-8))
      *residual = ceres::sqrt(sqNorm) - T(this->TransNorm);
    else
      *residual = sqNorm - T(this->TransNorm * this->TransNorm);

    return true;
  }

  // Factory to ease the construction of the auto-diff residual object
  RESIDUAL_FACTORY(CalibTransResidual, 1, 7)

private:
  // Squared norm of the translation (lever arm)
  double TransNorm = -1.;
};

//------------------------------------------------------------------------------
/**
 * \class CalibGpsResidual
 * \brief Residual corresponding to a comparison between a pose and a position
 * representing RELATIVE motion of two frames rigidly linked with unknown calibration offset
 * In the same global referential frame : absPosition = (absPose * Calib).Position
 * Therefore, using relative motions : residual = (Calib^-1 * transform(pose) * Calib).Position - position
 *
 * This function takes one 7D parameters block :
 *   - 3 first parameters to encode translation : X, Y, Z
 *   - 4 last parameters to encode rotation with quaternions : qX, qY, qZ, qW
 *
 * It outputs a 7D residual block.
 */
struct CalibGpsResidual
{
  CalibGpsResidual(const Eigen::Isometry3d& pose, const Eigen::Vector3d& position)
                  : Pose1Isometry(pose), Pose2Position(position) {}

  template <typename T>
  bool operator()(const T* const w, T* residual) const
  {
    using Vector7T = Eigen::Matrix<T, 7, 1>;
    using Vector3T = Eigen::Matrix<T, 3, 1>;
    using Isometry3T = Eigen::Transform<T, 3, Eigen::Isometry>;

    Isometry3T calibIsometry = Utils::XYZQuatToIsometry(w[0], w[1], w[2], w[3], w[4], w[5], w[6]);
    // Transform pose1 to get position2 equivalent
    Vector3T positionFromPoseAndCalib = (calibIsometry.inverse() * this->Pose1Isometry.cast<T>() * calibIsometry).translation();

    // Compute residual
    Eigen::Map<Vector3T> residualVec(residual);
    residualVec = positionFromPoseAndCalib - this->Pose2Position.cast<T>();

    return true;
  }

  // Factory to ease the construction of the auto-diff residual object
  RESIDUAL_FACTORY(CalibGpsResidual, 3, 7)

private:
  // Poses corresponding to RELATIVE motions of two rigidly
  // linked frames that move together in a global frame
  const Eigen::Isometry3d Pose1Isometry;
  Eigen::Matrix<double, 3, 1> Pose2Position;
};

//------------------------------------------------------------------------------
/**
 * \class Transform
 * \brief Function that transforms a pose. This ceres function can be used to
 *        compute a Jacobian and derive a rotated covariance
 * This function takes one 6D parameters block :
 *   - 3 first parameters to encode translation : X, Y, Z
 *   - 3 last parameters to encode rotation with Euler angles : rX, rY, rZ
 *
 * It outputs a 6D residual block.
 */
struct Transform {
  Transform(const Eigen::Isometry3d& transformation): Transformation(transformation) {}

  template <typename T>
  bool operator()(const T* const poseEuler, T* residual) const
  {
    using Vector6T = Eigen::Matrix<T, 6, 1>;
    using Isometry3T = Eigen::Transform<T, 3, Eigen::Isometry>;

    // Compute transform matrix of current pose from Euler angles (RPY convention) and position
    Isometry3T transform = Utils::XYZRPYtoIsometry(poseEuler[0], poseEuler[1], poseEuler[2],
                                                   poseEuler[3], poseEuler[4], poseEuler[5]);

    // Transform pose
    Isometry3T newTransform = transform * this->Transformation.cast<T>();

    // Reconvert to 6D pose
    Eigen::Map<Vector6T> residualVec(residual);
    residualVec = Utils::IsometryToXYZRPY(newTransform);

    return true;
  }

  // Factory to ease the construction of the auto-diff residual object
  RESIDUAL_FACTORY(Transform, 6, 6)

private:
  // Transformation to apply to input pose
  const Eigen::Isometry3d Transformation;
};

//------------------------------------------------------------------------------
/**
 * \class ChangeRefFrame
 * \brief Function that changes the reference frame of a pose. This ceres function can be used to
 *        compute a Jacobian and derive a rotated covariance
 * This function takes one 6D parameters block :
 *   - 3 first parameters to encode translation : X, Y, Z
 *   - 3 last parameters to encode rotation with Euler angles : rX, rY, rZ
 *
 * It outputs a 6D residual block.
 */
struct ChangeRefFrame {
  ChangeRefFrame(const Eigen::Isometry3d& transformation): Transformation(transformation) {}

  template <typename T>
  bool operator()(const T* const poseEuler, T* residual) const
  {
    using Vector6T = Eigen::Matrix<T, 6, 1>;
    using Isometry3T = Eigen::Transform<T, 3, Eigen::Isometry>;

    // Compute transform matrix of current pose from Euler angles (RPY convention) and position
    Isometry3T transform = Utils::XYZRPYtoIsometry(poseEuler[0], poseEuler[1], poseEuler[2],
                                                   poseEuler[3], poseEuler[4], poseEuler[5]);
    // Transform pose
    Isometry3T ref = this->Transformation.cast<T>();
    Isometry3T newTransform = ref * transform;

    // Reconvert to 6D pose
    Eigen::Map<Vector6T> residualVec(residual);
    residualVec = Utils::IsometryToXYZRPY(newTransform);

    return true;
  }

  // Factory to ease the construction of the auto-diff residual object
  RESIDUAL_FACTORY(ChangeRefFrame, 6, 6)

private:
  // Transformation to change reference frame of the input pose
  const Eigen::Isometry3d Transformation;
};

} // end of namespace CeresCostFunctions

namespace CeresTools
{
//------------------------------------------------------------------------------
/*!
 * @brief Transform a covariance to change the reference frame or to transform the pose
 *        Theory : the variables of the pose X are associated with a covariance C
 *                 If we want to transform the pose, we apply the function f to X
 *                 The covariance associated to the new pose f(X) is JCJ^T (J being the Jacobian of the f function)
 *                 f is the combination of the conversion to Isometry matrix, the transformation and the conversion to 6D pose again.
 *                 i.e., if changeFrame = false : Pose(Isometry(X) * T), T being the transformation to apply to the pose
 *                 if changeFrame = true : Pose(T * Isometry(X)), T being the reference frame transformation to apply to the pose
 * @param[in] pose : pose associated to the covariance
 * @param[in] covariance : covariance matrix to recompute
 * @param[in] transformation : 3d isometry to apply (containing a 4x4 matrix)
 * @param[in] changeFrame : bool to decide in which order to perform the matrix multiplication
 */
inline Eigen::Matrix<double, 6, 6> RotateCovariance(Eigen::Matrix<double, 6, 1>& pose,
                                                    const Eigen::Matrix<double, 6, 6>& covariance,
                                                    const Eigen::Isometry3d& transformation,
                                                    bool changeFrame = false)
{
  // Get Jacobian of F function at pose
  // 1_ Apply function F to input pose
  // 1_1 Create F
  ceres::CostFunction* F;
  // Choose whether we want to transform current pose or to change its referential frame (multiplication order)
  if (changeFrame)
    F = new ceres::AutoDiffCostFunction<CeresCostFunctions::ChangeRefFrame, 6, 6>(new CeresCostFunctions::ChangeRefFrame(transformation));
  else
    F = new ceres::AutoDiffCostFunction<CeresCostFunctions::Transform, 6, 6>(new CeresCostFunctions::Transform(transformation));
  // 1_2 Apply F
  ceres::Problem problem;
  problem.AddResidualBlock(F, nullptr, pose.data());
  double cost = 0.0;
  ceres::CRSMatrix jacobian;
  problem.Evaluate(ceres::Problem::EvaluateOptions(), &cost, nullptr, nullptr, &jacobian);

  // 2_ Get Jacobian from F evaluation at pose
  Eigen::Matrix<double, 6, 6> J = Eigen::Matrix<double, 6, 6>::Zero();
  // convert CRSMatrix to Eigen matrix
  std::vector<double> values = jacobian.values;
  std::vector<int> rows = jacobian.rows;
  std::vector<int> cols = jacobian.cols;
  int nRows = jacobian.num_rows;
  for (int i = 0; i < nRows; ++i)
  {
    for (int j = rows[i]; j < rows[i+1]; ++j)
      J(i, cols[j]) = values[j];
  }
  // Apply Jacobian to covariance
  return J * covariance * J.transpose();
}
} // end of namespace CeresTools

} // end of LidarSlam namespace
