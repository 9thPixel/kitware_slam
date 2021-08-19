//==============================================================================
// Copyright 2019-2020 Kitware, Inc., Kitware SAS
// Author: Nicolas Cadart (Kitware SAS)
// Creation date: 2020-11-10
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
#include <string>
#include <map>

namespace LidarSlam
{

//------------------------------------------------------------------------------
//! Type of a keypoint
enum Keypoint
{
  EDGE  = 0,   ///< edge keypoint (sharp local structure)
  PLANE = 1,   ///< plane keypoint (flat local structure)
  BLOB  = 2,   ///< blob keypoint (spherical local structure)
  nKeypointTypes
};

static const std::vector<Keypoint> KeypointTypes = { EDGE, PLANE, BLOB };
static const std::map<Keypoint, std::string> KeypointTypeNames = { {EDGE, "edge"}, {PLANE, "plane"}, {BLOB, "blob"} };

//------------------------------------------------------------------------------
//! How to deal with undistortion
enum UndistortionMode
{
  //! No undistortion is performed:
  //!  - End scan pose is optimized using rigid registration of raw scan and map.
  //!  - Raw input scan is added to map.
  NONE = 0,

  //! Undistortion is performed only once using estimated ego-motion:
  //!  - Begin and end scan poses are linearly interpolated using estimated ego-motion.
  //!  - Scan is linearly undistorted between begin and end scan poses.
  //!  - Scan pose is iteratively optimized using rigid registration of undistorted scan and map.
  //!  - Undistorted scan is added to map.
  ONCE = 1,

  //! Undistortion is iteratively refined using optimized ego-motion:
  //!  - Begin and end scan poses are linearly interpolated using ego-motion.
  //!  - Scan is linearly undistorted between begin and end scan poses.
  //!  - Scan pose is optimized using rigid registration of undistorted scan and map.
  //!  - Iterate the three previous steps with updated ego-motion and poses.
  //!  - Undistorted scan is added to map.
  REFINED = 2
};

//------------------------------------------------------------------------------
//! How to estimate Ego-Motion (approximate relative motion since last frame)
enum class EgoMotionMode
{
  //! No ego-motion step is performed : relative motion is Identity, new
  //! estimated Tworld is equal to previous Tworld.
  //! Fast, but may lead to unstable and imprecise Localization step if motion
  //! is important.
  NONE = 0,

  //! Previous motion is linearly extrapolated to estimate new Tworld pose
  //! from the 2 previous poses.
  //! Fast and precise if motion is roughly constant and continuous.
  MOTION_EXTRAPOLATION = 1,

  //! Estimate Trelative (and therefore Tworld) by globally registering new
  //! frame on previous frame.
  //! Slower and need textured enough environment, but do not rely on
  //! constant motion hypothesis.
  REGISTRATION = 2,

  //! Previous motion is linearly extrapolated to estimate new Tworld pose
  //! from the 2 previous poses. Then this estimation is refined by globally
  //! registering new frame on previous frame.
  //! Slower and need textured enough environment, but should be more precise
  //! and rely less on constant motion hypothesis.
  MOTION_EXTRAPOLATION_AND_REGISTRATION = 3
};

enum Output
{
  POSE_ODOM,             // Output SLAM pose as an Odometry msg on 'slam_odom' topic (default : true).
  POSE_TF,               // Output SLAM pose as a TF from 'odometry_frame' to 'tracking_frame' (default : true).
  POSE_PREDICTION_ODOM,  // Output latency-corrected SLAM pose as an Odometry msg on 'slam_predicted_odom' topic.
  POSE_PREDICTION_TF,    // Output latency-corrected SLAM pose as a TF from 'odometry_frame' to '<tracking_frame>_prediction'.

  EDGES_MAP,             // Output edge keypoints map as a LidarPoint PointCloud2 msg to topic 'maps/edges'.
  PLANES_MAP,            // Output plane keypoints map as a LidarPoint PointCloud2 msg to topic 'maps/planes'.
  BLOBS_MAP,             // Output blob keypoints map as a LidarPoint PointCloud2 msg to topic 'maps/blobs'.

  EDGE_KEYPOINTS,        // Output extracted edge keypoints from current frame as a PointCloud2 msg to topic 'keypoints/edges'.
  PLANE_KEYPOINTS,       // Output extracted plane keypoints from current frame as a PointCloud2 msg to topic 'keypoints/planes'.
  BLOB_KEYPOINTS,        // Output extracted blob keypoints from current frame as a PointCloud2 msg to topic 'keypoints/blobs'.

  SLAM_REGISTERED_POINTS,// Output SLAM pointcloud as LidarPoint PointCloud2 msg to topic 'slam_registered_points'.

  CONFIDENCE,            // Output confidence estimators on output pose to topic 'slam_confidence'.

  PGO_PATH,              // Output optimized SLAM trajectory as Path msg to 'pgo_slam_path' latched topic.
  ICP_CALIB_SLAM_PATH,   // Output ICP-aligned SLAM trajectory as Path msg to 'icp_slam_path' latched topic.
  ICP_CALIB_GPS_PATH ,   // Output ICP-aligned GPS trajectory as Path msg to 'icp_gps_path' latched topic.

  DEBUG_ARRAYS,          // Output std::map containing local error + confidence estimators (explicit names + values)
  KE_DEBUG_ARRAYS,        // Output std::map containing kpts extraction info (explicit names + values)
  nOutputs
};

} // end of LidarSlam namespace