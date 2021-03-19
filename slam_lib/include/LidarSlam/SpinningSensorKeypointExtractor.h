//==============================================================================
// Copyright 2018-2020 Kitware, Inc., Kitware SAS
// Author: Guilbert Pierre (Kitware SAS)
//         Laurenson Nick (Kitware SAS)
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

#include "LidarSlam/LidarPoint.h"
#include "LidarSlam/Enums.h"

#include <pcl/point_cloud.h>

#include <unordered_map>

#define SetMacro(name,type) void Set##name (type _arg) { name = _arg; }
#define GetMacro(name,type) type Get##name () const { return name; }

namespace LidarSlam
{

class SpinningSensorKeypointExtractor
{
public:
  using Point = LidarPoint;
  using PointCloud = pcl::PointCloud<Point>;

  GetMacro(NbThreads, int)
  SetMacro(NbThreads, int)

  GetMacro(NeighborWidth, int)
  SetMacro(NeighborWidth, int)

  GetMacro(MinDistanceToSensor, float)
  SetMacro(MinDistanceToSensor, float)

  GetMacro(PlaneSinAngleThreshold, float)
  SetMacro(PlaneSinAngleThreshold, float)

  GetMacro(EdgeSinAngleThreshold, float)
  SetMacro(EdgeSinAngleThreshold, float)

  GetMacro(EdgeDepthGapThreshold, float)
  SetMacro(EdgeDepthGapThreshold, float)

  GetMacro(EdgeSaliencyThreshold, float)
  SetMacro(EdgeSaliencyThreshold, float)

  GetMacro(EdgeIntensityGapThreshold, float)
  SetMacro(EdgeIntensityGapThreshold, float)

  GetMacro(NbLaserRings, int)
  SetMacro(NbLaserRings, int)

  GetMacro(NbFiringsPerLaserRing, int)
  SetMacro(NbFiringsPerLaserRing, int)

  float GetAzimuthalResolution() const { return 2 * M_PI / this->NbFiringsPerLaserRing; }

  PointCloud::Ptr GetEdgePoints() const { return this->EdgesPoints; }
  PointCloud::Ptr GetPlanarPoints() const { return this->PlanarsPoints; }
  PointCloud::Ptr GetBlobPoints() const { return this->BlobsPoints; }

  // Extract keypoints from the pointcloud. The key points
  // will be separated in two classes : Edges keypoints which
  // correspond to area with high curvature scan lines and
  // planar keypoints which have small curvature.
  // NOTE: This expects that the lowest/bottom laser_id is 0, and is increasing upward.
  void ComputeKeyPoints(const PointCloud::Ptr& pc);

  // Function to enable to have some inside on why a given point was detected as a keypoint
  std::unordered_map<std::string, std::vector<float>> GetDebugArray() const;

private:

  // Initialize the feature vectors and keypoint clouds,
  // and project input points onto the vertex map.
  // This expects that the lowest/bottom laser ring is 0, and is increasing upward.
  void PrepareDataForNewFrame();

  // Invalid the points with bad criteria from the list of possible future keypoints.
  // These points correspond to planar surfaces roughly parallel to laser beam
  // and points close to a gap created by occlusion.
  void InvalidateNotUsablePoints();

  // Compute the curvature and other features within each the scan line.
  // The curvature is not the one of the surface that intersects the lines but
  // the 1D curvature within each isolated scan line.
  void ComputeFeatures();

  // Labelize point to be a keypoints or not
  void SetKeyPointsLabels();

  // Auto estimate NbLaserRings and NbFiringsPerLaserRing from current scan.
  // WARNING: to be correct, the points need to be in the LIDAR sensor
  // coordinates system, where the sensor is spinning around Z axis.
  void EstimateLaserRingsParameters();

  // Extract valid neighbor points' indices on the same scan line within a given
  // signed distance evaluated in number of neighbors
  std::vector<int> GetNeighbors(int laserRing, int firingIndex, int nbNeighborsDist);

  // ---------------------------------------------------------------------------
  //   Parameters
  // ---------------------------------------------------------------------------

  // Max number of threads to use to process points in parallel
  int NbThreads = 1;

  // Width of the neighborhood used to compute discrete differential operators
  int NeighborWidth = 4;

  // Minimal point/sensor sensor to consider a point as valid
  float MinDistanceToSensor = 3.0;  // [m]

  // Sharpness threshold to select a planar keypoint
  float PlaneSinAngleThreshold = 0.5;  // sin(30°) (selected if sin angle is less than threshold)

  // Sharpness threshold to select an edge keypoint
  float EdgeSinAngleThreshold = 0.86;  // ~sin(60°) (selected, if sin angle is more than threshold)
  float DistToLineThreshold = 0.20;  // [m]

  // Threshold upon depth gap in neighborhood to select an edge keypoint
  float EdgeDepthGapThreshold = 0.15;  // [m]

  // Threshold upon saliency of a neighborhood to select an edge keypoint
  float EdgeSaliencyThreshold = 1.5;  // [m]

  // Threshold upon intensity gap to select an edge keypoint
  float EdgeIntensityGapThreshold = 50.;

  // ---------------------------------------------------------------------------
  //   Laser rings parameters
  // ---------------------------------------------------------------------------

  // Number of laser scan lines
  // Common values: 16, 32, 64, 128
  // If it is less or equal to 0, it will be auto-estimated from next input frame.
  int NbLaserRings = 0;

  // Number of laser firings per scan line for a full revolution.
  // Common values: 900, 1800, 3600 or 512, 1024, 2048 depending on spinning frequency.
  // If it is less or equal to 0, it will be auto-estimated from next input frame.
  // This number is directly linked to the azimuthal resolution of the spinning
  // device, which is the horizontal angle between two consecutive firings:
  //   azimutalResolution = 2 * PI / NbFiringsPerLaserRing
  int NbFiringsPerLaserRing = 0;

  // ---------------------------------------------------------------------------
  //   Internal variables
  // ---------------------------------------------------------------------------

  //! Label of a point as a keypoint
  //! We use binary flags as each point can have different keypoint labels.
  using KeypointFlags = std::underlying_type<Keypoint>::type;

  // Projection of points properties on vertex maps
  // (2D image of size NbLaserRings x NbFiringsPerLaserRing)
  template<typename T>
  using VertexMap = Eigen::Array<T, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor>;

  // Curvature and other differential operations (scan by scan, point by point)
  VertexMap<float> Angles;
  VertexMap<float> DepthGap;
  VertexMap<float> Saliency;
  VertexMap<float> IntensityGap;
  VertexMap<KeypointFlags> IsPointValid;
  VertexMap<KeypointFlags> Label;

  // Current point cloud stored in differents formats
  PointCloud::Ptr Scan;    ///< Raw input scan
  VertexMap<int> ScanIds;  ///< Indices of input scan points projected in vertex map

  // Extracted keypoints of current frame
  PointCloud::Ptr EdgesPoints;
  PointCloud::Ptr PlanarsPoints;
  PointCloud::Ptr BlobsPoints;
};

} // end of LidarSlam namespace