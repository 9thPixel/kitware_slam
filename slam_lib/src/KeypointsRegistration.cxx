//==============================================================================
// Copyright 2019-2020 Kitware, Inc., Kitware SAS
// Author: Cadart Nicolas (Kitware SAS)
// Creation date: 2020-10-16
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

#include "LidarSlam/KeypointsRegistration.h"
#include "LidarSlam/CeresCostFunctions.h"

namespace LidarSlam
{

//-----------------------------------------------------------------------------
KeypointsRegistration::KeypointsRegistration(const KeypointsRegistration::Parameters& params,
                                             const Eigen::Isometry3d& posePrior)
  : Params(params)
  , PosePrior(posePrior)
{
  // Convert isometry to 6D state vector : X, Y, Z, rX, rY, rZ
  this->PoseArray = Utils::IsometryToXYZRPY(posePrior);
}

//-----------------------------------------------------------------------------
KeypointsRegistration::MatchingResults KeypointsRegistration::BuildAndMatchResiduals(const PointCloud::Ptr& currPoints,
                                                                                     const KDTree& prevPoints,
                                                                                     Keypoint keypointType)
{
  // Call the correct point-to-neighborhood method
  auto BuildAndMatchSingleResidual = [&](const Point& currentPoint)
  {
    switch(keypointType)
    {
      case Keypoint::EDGE:
        return this->BuildLineMatch(prevPoints, currentPoint);
      case Keypoint::PLANE:
        return this->BuildPlaneMatch(prevPoints, currentPoint);
      case Keypoint::BLOB:
        return this->BuildBlobMatch(prevPoints, currentPoint);
      default:
        return MatchingResults::MatchStatus::UNKOWN;
    }
  };

  // Reset matching results
  MatchingResults matchingResults;
  matchingResults.Rejections.assign(currPoints->size(), MatchingResults::MatchStatus::UNKOWN);
  matchingResults.RejectionsHistogram.fill(0);

  // Loop over keypoints and try to build residuals
  if (!currPoints->empty() && prevPoints.GetInputCloud() && !prevPoints.GetInputCloud()->empty())
  {
    #pragma omp parallel for num_threads(this->Params.NbThreads) schedule(guided, 8)
    for (int ptIndex = 0; ptIndex < static_cast<int>(currPoints->size()); ++ptIndex)
    {
      const Point& currentPoint = currPoints->points[ptIndex];
      MatchingResults::MatchStatus rejectionIndex = BuildAndMatchSingleResidual(currentPoint);
      matchingResults.Rejections[ptIndex] = rejectionIndex;
      #pragma omp atomic
      matchingResults.RejectionsHistogram[rejectionIndex]++;
    }
  }

  return matchingResults;
}

//----------------------------------------------------------------------------
ceres::Solver::Summary KeypointsRegistration::Solve()
{
  ceres::Solver::Options options;
  options.max_num_iterations = this->Params.LMMaxIter;
  options.linear_solver_type = ceres::DENSE_QR;  // TODO : try also DENSE_NORMAL_CHOLESKY or SPARSE_NORMAL_CHOLESKY
  options.minimizer_progress_to_stdout = false;
  options.num_threads = this->Params.NbThreads;

  ceres::Solver::Summary summary;
  ceres::Solve(options, &(this->Problem), &summary);
  return summary;
}

//----------------------------------------------------------------------------
KeypointsRegistration::RegistrationError KeypointsRegistration::EstimateRegistrationError()
{
  RegistrationError err;

  // Covariance computation options
  ceres::Covariance::Options covOptions;
  covOptions.apply_loss_function = true;
  covOptions.algorithm_type = ceres::CovarianceAlgorithmType::DENSE_SVD;
  covOptions.null_space_rank = -1;
  covOptions.num_threads = this->Params.NbThreads;

  // Computation of the variance-covariance matrix
  ceres::Covariance covarianceSolver(covOptions);
  std::vector<std::pair<const double*, const double*>> covarianceBlocks;
  const double* paramBlock = this->PoseArray.data();
  covarianceBlocks.emplace_back(paramBlock, paramBlock);
  covarianceSolver.Compute(covarianceBlocks, &this->Problem);
  covarianceSolver.GetCovarianceBlock(paramBlock, paramBlock, err.Covariance.data());

  // Estimate max position/orientation errors and directions from covariance
  Eigen::SelfAdjointEigenSolver<Eigen::Matrix3d> eigPosition(err.Covariance.topLeftCorner<3, 3>());
  err.PositionError = std::sqrt(eigPosition.eigenvalues()(2));
  err.PositionErrorDirection = eigPosition.eigenvectors().col(2);
  Eigen::SelfAdjointEigenSolver<Eigen::Matrix3d> eigOrientation(err.Covariance.bottomRightCorner<3, 3>());
  err.OrientationError = Utils::Rad2Deg(std::sqrt(eigOrientation.eigenvalues()(2)));
  err.OrientationErrorDirection = eigOrientation.eigenvectors().col(2);

  return err;
}

//----------------------------------------------------------------------------
void KeypointsRegistration::AddIcpResidual(const Eigen::Matrix3d& A, const Eigen::Vector3d& P, const Eigen::Vector3d& X, double weight)
{
  // Create the point-to-line/plane/blob cost function
  using Residual = CeresCostFunctions::MahalanobisDistanceAffineIsometryResidual;
  ceres::CostFunction* costFunction = new ceres::AutoDiffCostFunction<Residual, 1, 6>(new Residual(A, P, X));

  // Use a robustifier to limit the contribution of an outlier match
  auto* robustifier = new ceres::TukeyLoss(std::sqrt(this->Params.SaturationDistance));
  // Weight the contribution of the given match by its reliability
  auto* loss = new ceres::ScaledLoss(robustifier, weight, ceres::TAKE_OWNERSHIP);

  // Add weighted constraint to the problem
  #pragma omp critical(addIcpResidual)
  this->Problem.AddResidualBlock(costFunction, loss, this->PoseArray.data());
}

//-----------------------------------------------------------------------------
KeypointsRegistration::MatchingResults::MatchStatus KeypointsRegistration::BuildLineMatch(const KDTree& kdtreePreviousEdges, const Point& p)
{
  // =====================================================
  // Transform the point using the current pose estimation

  // localPoint is the raw local position, on which we need to apply the transform to optimize.
  // worldPoint is the estimated position in world coodinates.
  Eigen::Vector3d localPoint = p.getVector3fMap().cast<double>();
  Eigen::Vector3d worldPoint = this->PosePrior * localPoint;

  // ===================================================
  // Get neighboring points in previous set of keypoints

  std::vector<int> knnIndices;
  std::vector<float> knnSqDist;
  if (this->Params.SingleEdgePerRing)
    this->GetPerRingLineNeighbors(kdtreePreviousEdges, worldPoint.data(), this->Params.LineDistanceNbrNeighbors, knnIndices, knnSqDist);
  else
    this->GetRansacLineNeighbors(kdtreePreviousEdges, worldPoint.data(), this->Params.LineDistanceNbrNeighbors, this->Params.MaxLineDistance, knnIndices, knnSqDist);

  // If not enough neighbors, abort
  unsigned int neighborhoodSize = knnIndices.size();
  if (neighborhoodSize < this->Params.MinimumLineNeighborRejection)
  {
    return MatchingResults::MatchStatus::NOT_ENOUGH_NEIGHBORS;
  }

  // If the nearest edges are too far from the current edge keypoint,
  // we skip this point.
  if (knnSqDist.back() > this->Params.MaxDistanceForICPMatching * this->Params.MaxDistanceForICPMatching)
  {
    return MatchingResults::MatchStatus::NEIGHBORS_TOO_FAR;
  }

  // Shortcut to keypoints cloud
  const PointCloud& previousEdgesPoints = *kdtreePreviousEdges.GetInputCloud();

  // =======================================================
  // Check if neighborhood is a good line candidate with PCA

  // Compute PCA to determine best line approximation of the neighborhood.
  // Thanks to the PCA we will check the shape of the neighborhood and keep it
  // if it is well distributed along a line.
  Eigen::Vector3d mean;
  Eigen::Vector3d eigVals;
  Eigen::Matrix3d eigVecs;
  Utils::ComputeMeanAndPCA(previousEdgesPoints, knnIndices, mean, eigVecs, eigVals);

  // If the first eigen value is significantly higher than the second one,
  // it means that the sourrounding points are distributed on an edge line.
  // Otherwise, discard this bad unstructured neighborhood.
  if (eigVals(2) < this->Params.LineDistancefactor * eigVals(1))
  {
    return MatchingResults::MatchStatus::BAD_PCA_STRUCTURE;
  }

  // =============================================
  // Compute point-to-line optimization parameters

  // n is the director vector of the line
  const Eigen::Vector3d& n = eigVecs.col(2);

  // A = (I-n*n.t).t * (I-n*n.t) = (I - n*n.t)^2
  // since (I-n*n.t) is a symmetric matrix
  // Then it comes A (I-n*n.t)^2 = (I-n*n.t) since
  // A is the matrix of a projection endomorphism
  Eigen::Matrix3d A = Eigen::Matrix3d::Identity() - n * n.transpose();

  // =========================
  // Check parameters validity

  // It would be the case if P1 = P2, for instance if the sensor has some dual
  // returns that hit the same point.
  if (!std::isfinite(A(0, 0)))
  {
    return MatchingResults::MatchStatus::INVALID_NUMERICAL;
  }

  // Evaluate the distance from the fitted line distribution of the neighborhood
  double meanSquaredDist = 0.;
  double squaredMaxDist = this->Params.MaxLineDistance * this->Params.MaxLineDistance;
  for (unsigned int nearestPointIndex: knnIndices)
  {
    const Point& pt = previousEdgesPoints[nearestPointIndex];
    Eigen::Vector3d Xtemp(pt.x, pt.y, pt.z);
    double squaredDist = (Xtemp - mean).transpose() * A * (Xtemp - mean);
    // CHECK invalidate all neighborhood even if only one point is bad?
    if (squaredDist > squaredMaxDist)
    {
      return MatchingResults::MatchStatus::MSE_TOO_LARGE;
    }
    meanSquaredDist += squaredDist;
  }
  meanSquaredDist /= static_cast<double>(neighborhoodSize);

  // ===========================================
  // Add valid parameters for later optimization

  // Quality score of the point-to-line match
  double fitQualityCoeff = 1.0 - std::sqrt(meanSquaredDist / squaredMaxDist);

  // Store the distance parameters values
  this->AddIcpResidual(A, mean, localPoint, fitQualityCoeff);

  return MatchingResults::MatchStatus::SUCCESS;
}

//-----------------------------------------------------------------------------
KeypointsRegistration::MatchingResults::MatchStatus KeypointsRegistration::BuildPlaneMatch(const KDTree& kdtreePreviousPlanes, const Point& p)
{
  // =====================================================
  // Transform the point using the current pose estimation

  // localPoint is the raw local position, on which we need to apply the transform to optimize.
  // worldPoint is the estimated position in world coodinates.
  Eigen::Vector3d localPoint = p.getVector3fMap().cast<double>();
  Eigen::Vector3d worldPoint = this->PosePrior * localPoint;

  // ===================================================
  // Get neighboring points in previous set of keypoints

  std::vector<int> knnIndices;
  std::vector<float> knnSqDist;
  unsigned int neighborhoodSize = kdtreePreviousPlanes.KnnSearch(worldPoint.data(), this->Params.PlaneDistanceNbrNeighbors, knnIndices, knnSqDist);

  // It means that there is not enough keypoints in the neighborhood
  if (neighborhoodSize < this->Params.PlaneDistanceNbrNeighbors)
  {
    return MatchingResults::MatchStatus::NOT_ENOUGH_NEIGHBORS;
  }

  // If the nearest planar points are too far from the current keypoint,
  // we skip this point.
  if (knnSqDist.back() > this->Params.MaxDistanceForICPMatching * this->Params.MaxDistanceForICPMatching)
  {
    return MatchingResults::MatchStatus::NEIGHBORS_TOO_FAR;
  }

  // Shortcut to keypoints cloud
  const PointCloud& previousPlanesPoints = *kdtreePreviousPlanes.GetInputCloud();

  // ========================================================
  // Check if neighborhood is a good plane candidate with PCA

  // Compute PCA to determine best plane approximation of the neighborhood.
  // Thanks to the PCA we will check the shape of the neighborhood and keep it
  // if it is well distributed along a plane.
  Eigen::Vector3d mean;
  Eigen::Vector3d eigVals;
  Eigen::Matrix3d eigVecs;
  Utils::ComputeMeanAndPCA(previousPlanesPoints, knnIndices, mean, eigVecs, eigVals);

  // If the second eigen value is close to the highest one and bigger than the
  // smallest one, it means that the points are distributed along a plane.
  // Otherwise, discard this bad unstructured neighborhood.
  if (this->Params.PlaneDistancefactor2 * eigVals(1) < eigVals(2) ||
      eigVals(1) < this->Params.PlaneDistancefactor1 * eigVals(0))
  {
    return MatchingResults::MatchStatus::BAD_PCA_STRUCTURE;
  }

  // ==============================================
  // Compute point-to-plane optimization parameters

  // n is the normal vector of the plane
  const Eigen::Vector3d& n = eigVecs.col(0);
  Eigen::Matrix3d A = n * n.transpose();

  // It would be the case if P1 = P2, P1 = P3 or P3 = P2, for instance if the
  // sensor has some dual returns that hit the same point.
  if (!std::isfinite(A(0, 0)))
  {
    return MatchingResults::MatchStatus::INVALID_NUMERICAL;
  }

  // Evaluate the distance from the fitted plane distribution of the neighborhood
  double meanSquaredDist = 0.;
  double squaredMaxDist = this->Params.MaxPlaneDistance * this->Params.MaxPlaneDistance;
  for (unsigned int nearestPointIndex: knnIndices)
  {
    const Point& pt = previousPlanesPoints[nearestPointIndex];
    Eigen::Vector3d Xtemp(pt.x, pt.y, pt.z);
    double squaredDist = (Xtemp - mean).transpose() * A * (Xtemp - mean);
    // CHECK invalidate all neighborhood even if only one point is bad?
    if (squaredDist > squaredMaxDist)
    {
      return MatchingResults::MatchStatus::MSE_TOO_LARGE;
    }
    meanSquaredDist += squaredDist;
  }
  meanSquaredDist /= static_cast<double>(neighborhoodSize);

  // ===========================================
  // Add valid parameters for later optimization

  // Quality score of the point-to-plane match
  double fitQualityCoeff = 1.0 - std::sqrt(meanSquaredDist / squaredMaxDist);

  // Store the distance parameters values
  this->AddIcpResidual(A, mean, localPoint, fitQualityCoeff);

  return MatchingResults::MatchStatus::SUCCESS;
}

//-----------------------------------------------------------------------------
KeypointsRegistration::MatchingResults::MatchStatus KeypointsRegistration::BuildBlobMatch(const KDTree& kdtreePreviousBlobs, const Point& p)
{
  // =====================================================
  // Transform the point using the current pose estimation

  // localPoint is the raw local position, on which we need to apply the transform to optimize.
  // worldPoint is the estimated position in world coodinates.
  Eigen::Vector3d localPoint = p.getVector3fMap().cast<double>();
  Eigen::Vector3d worldPoint = this->PosePrior * localPoint;

  // ===================================================
  // Get neighboring points in previous set of keypoints

  // double maxDist = this->MaxDistanceForICPMatching;  //< maximum distance between keypoints and its neighbors
  float maxDiameter = 4.;

  std::vector<int> knnIndices;
  std::vector<float> knnSqDist;
  unsigned int neighborhoodSize = kdtreePreviousBlobs.KnnSearch(worldPoint.data(), this->Params.BlobDistanceNbrNeighbors, knnIndices, knnSqDist);

  // It means that there is not enough keypoints in the neighborhood
  if (neighborhoodSize < this->Params.BlobDistanceNbrNeighbors)
  {
    return MatchingResults::MatchStatus::NOT_ENOUGH_NEIGHBORS;
  }

  // If the nearest blob points are too far from the current keypoint,
  // we skip this point.
  if (knnSqDist.back() > this->Params.MaxDistanceForICPMatching * this->Params.MaxDistanceForICPMatching)
  {
    return MatchingResults::MatchStatus::NEIGHBORS_TOO_FAR;
  }

  // Shortcut to keypoints cloud
  const PointCloud& previousBlobsPoints = *kdtreePreviousBlobs.GetInputCloud();

  // ======================================
  // Check the diameter of the neighborhood

  // If the diameter is too big, we don't want to keep this blob.
  // We must do that since the fitted ellipsoid assumes to encode the local
  // shape of the neighborhood.
  float squaredDiameter = 0.;
  for (unsigned int nearestPointIndexI: knnIndices)
  {
    const Point& ptI = previousBlobsPoints[nearestPointIndexI];
    for (unsigned int nearestPointIndexJ: knnIndices)
    {
      const Point& ptJ = previousBlobsPoints[nearestPointIndexJ];
      float squaredDistanceIJ = (ptI.getVector3fMap() - ptJ.getVector3fMap()).squaredNorm();
      squaredDiameter = std::max(squaredDiameter, squaredDistanceIJ);
    }
  }
  if (squaredDiameter > maxDiameter * maxDiameter)
  {
    return MatchingResults::MatchStatus::MSE_TOO_LARGE;
  }

  // ======================================================
  // Compute point-to-blob optimization parameters with PCA

  // Compute PCA to determine best ellipsoid approximation of the neighborhood.
  // Thanks to the PCA we will check the shape of the neighborhood and tune a
  // distance function adapted to the distribution (Mahalanobis distance).
  Eigen::Vector3d mean;
  Eigen::Vector3d eigVals;
  Eigen::Matrix3d eigVecs;
  Utils::ComputeMeanAndPCA(previousBlobsPoints, knnIndices, mean, eigVecs, eigVals);

  // TODO: check PCA structure
  // if (PCA shape isn't OK)
  // {
  //   return MatchingResults::MatchStatus::BAD_PCA_STRUCTURE;
  // }

  // The inverse of the covariance matrix encodes the mahalanobis distance.
  // Rescale the eigen values to preserve the shape of the mahalanobis distance,
  // but removing the variance values scaling.
  Eigen::Vector3d eigValsInv = eigVals.array().inverse();
  eigValsInv /= eigValsInv.maxCoeff();
  Eigen::Matrix3d A = eigVecs * eigValsInv.asDiagonal() * eigVecs.transpose();

  // Check the determinant of the matrix
  if (!std::isfinite(eigValsInv.prod()))
  {
    return MatchingResults::MatchStatus::INVALID_NUMERICAL;
  }

  // ===========================================
  // Add valid parameters for later optimization

  // Quality score of the point-to-blob match
  // The aim is to prevent wrong matching pulling the pointcloud in a bad direction.
  double fitQualityCoeff = 1.0;//1.0 - knnSqDist.back() / maxDist;

  // store the distance parameters values
  this->AddIcpResidual(A, mean, localPoint, fitQualityCoeff);

  return MatchingResults::MatchStatus::SUCCESS;
}

//-----------------------------------------------------------------------------
void KeypointsRegistration::GetPerRingLineNeighbors(const KDTree& kdtreePreviousEdges, const double pos[3], unsigned int knearest,
                                                    std::vector<int>& validKnnIndices, std::vector<float>& validKnnSqDist) const
{
  // Get nearest neighbors of the query point
  std::vector<int> knnIndices;
  std::vector<float> knnSqDist;
  unsigned int neighborhoodSize = kdtreePreviousEdges.KnnSearch(pos, knearest, knnIndices, knnSqDist);

  // If empty neighborhood, return
  if (neighborhoodSize == 0)
    return;

  // Shortcut to keypoints cloud
  const PointCloud& previousEdgesPoints = *kdtreePreviousEdges.GetInputCloud();

  // Take the closest point
  const Point& closest = previousEdgesPoints[knnIndices[0]];
  int closestLaserId = static_cast<int>(closest.laser_id);

  // Get number of scan lines of this neighborhood
  int laserIdMin = std::numeric_limits<int>::max();
  int laserIdMax = std::numeric_limits<int>::min();
  for (unsigned int k = 0; k < neighborhoodSize; ++k)
  {
    int scanLine = previousEdgesPoints[knnIndices[k]].laser_id;
    laserIdMin = std::min(laserIdMin, scanLine);
    laserIdMax = std::max(laserIdMax, scanLine);
  }
  int nLasers = laserIdMax - laserIdMin + 1;

  // Invalid all points that are on the same scan line than the closest one
  std::vector<uint8_t> idAlreadyTook(nLasers, 0);
  idAlreadyTook[closestLaserId - laserIdMin] = 1;

  // Invalid all points from scan lines that are too far from the closest one
  const int maxScanLineDiff = 4;  // TODO : add parameter to discard too far laser rings
  for (int laserId = laserIdMin; laserId <= laserIdMax; ++laserId)
  {
    if (std::abs(closestLaserId - laserId) > maxScanLineDiff)
      idAlreadyTook[laserId - laserIdMin] = 1;
  }

  // Make a selection among the neighborhood of the query point.
  // We can only take one edge per scan line.
  validKnnIndices.clear();
  validKnnSqDist.clear();
  for (unsigned int k = 0; k < neighborhoodSize; ++k)
  {
    int scanLine = previousEdgesPoints[knnIndices[k]].laser_id - laserIdMin;
    if (!idAlreadyTook[scanLine])
    {
      idAlreadyTook[scanLine] = 1;
      validKnnIndices.push_back(knnIndices[k]);
      validKnnSqDist.push_back(knnSqDist[k]);
    }
  }
}

//-----------------------------------------------------------------------------
void KeypointsRegistration::GetRansacLineNeighbors(const KDTree& kdtreePreviousEdges, const double pos[3], unsigned int knearest, double maxDistInlier,
                                                   std::vector<int>& validKnnIndices, std::vector<float>& validKnnSqDist) const
{
  // Get nearest neighbors of the query point
  std::vector<int> knnIndices;
  std::vector<float> knnSqDist;
  unsigned int neighborhoodSize = kdtreePreviousEdges.KnnSearch(pos, knearest, knnIndices, knnSqDist);

  // If empty neighborhood, return
  if (neighborhoodSize == 0)
    return;

  // Shortcut to keypoints cloud
  const PointCloud& previousEdgesPoints = *kdtreePreviousEdges.GetInputCloud();

  // To avoid square root when performing comparison
  const float squaredMaxDistInlier = maxDistInlier * maxDistInlier;

  // Take the closest point
  const Point& closest = previousEdgesPoints[knnIndices[0]];
  const auto P1 = closest.getVector3fMap();

  // Loop over neighbors of the neighborhood. For each of them, compute the line
  // between closest point and current point and compute the number of inliers
  // that fit this line.
  std::vector<std::vector<unsigned int>> inliersList;
  inliersList.reserve(neighborhoodSize - 1);
  for (unsigned int ptIndex = 1; ptIndex < neighborhoodSize; ++ptIndex)
  {
    // Fit line that links P1 and P2
    const auto P2 = previousEdgesPoints[knnIndices[ptIndex]].getVector3fMap();
    Eigen::Vector3f dir = (P2 - P1).normalized();

    // Compute number of inliers of this model
    std::vector<unsigned int> inlierIndex;
    for (unsigned int candidateIndex = 1; candidateIndex < neighborhoodSize; ++candidateIndex)
    {
      if (candidateIndex == ptIndex)
        inlierIndex.push_back(candidateIndex);
      else
      {
        const auto Pcdt = previousEdgesPoints[knnIndices[candidateIndex]].getVector3fMap();
        if (((Pcdt - P1).cross(dir)).squaredNorm() < squaredMaxDistInlier)
          inlierIndex.push_back(candidateIndex);
      }
    }
    inliersList.push_back(inlierIndex);
  }

  // Keep the line and its inliers with the most inliers.
  std::size_t maxInliers = 0;
  int indexMaxInliers = -1;
  for (unsigned int k = 0; k < inliersList.size(); ++k)
  {
    if (inliersList[k].size() > maxInliers)
    {
      maxInliers = inliersList[k].size();
      indexMaxInliers = k;
    }
  }

  // fill vectors
  validKnnIndices.clear(); validKnnIndices.reserve(inliersList[indexMaxInliers].size());
  validKnnSqDist.clear(); validKnnSqDist.reserve(inliersList[indexMaxInliers].size());
  validKnnIndices.push_back(knnIndices[0]);
  validKnnSqDist.push_back(knnSqDist[0]);
  for (unsigned int inlier: inliersList[indexMaxInliers])
  {
    validKnnIndices.push_back(knnIndices[inlier]);
    validKnnSqDist.push_back(knnSqDist[inlier]);
  }
}

} // end of LidarSlam namespace