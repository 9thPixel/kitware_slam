#include "LidarSlamNode.h"

#include <pcl_conversions/pcl_conversions.h>
#include <geometry_msgs/TransformStamped.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <tf2/LinearMath/Quaternion.h>

#include <chrono>

//------------------------------------------------------------------------------
LidarSlamNode::LidarSlamNode(ros::NodeHandle& nh, ros::NodeHandle& priv_nh)
{
  // Get SLAM params
  SetSlamParameters(priv_nh);

  // Init laserIdMapping
  std::vector<int> intLaserIdMapping;
  int nLasers;
  // Try to get it directly from ROS param
  if (priv_nh.getParam("laser_id_mapping", intLaserIdMapping))
  {
    this->LaserIdMapping.assign(intLaserIdMapping.begin(), intLaserIdMapping.end());
    ROS_INFO_STREAM("[SLAM] Using laser_id_mapping from ROS param.");
  }
  // Or only try to get number of lasers to build linear mapping
  else if (priv_nh.getParam("n_lasers", nLasers))
  {
    this->LaserIdMapping.resize(nLasers);
    for (int i = 0; i < nLasers; i++)
      this->LaserIdMapping[i] = i;
    ROS_INFO_STREAM("[SLAM] Using 0->" << nLasers << " linear laser_id_mapping from ROS param.");
  }
  // Otherwise, n_lasers will be guessed from 1st frame
  else
  {
    ROS_WARN_STREAM("[SLAM] No laser_id_mapping nor n_lasers params found : "
                    "n_lasers will be guessed from 1st frame to build linear mapping.");
  }

  // Get LiDAR frequency
  priv_nh.getParam("lidar_frequency", this->LidarFreq);

  // Init optional publishers
  priv_nh.getParam("publish_features_maps/edges", this->PublishEdges);
  priv_nh.getParam("publish_features_maps/planars", this->PublishPlanars);
  priv_nh.getParam("publish_features_maps/blobs", this->PublishBlobs);
  if (this->PublishEdges)
    this->EdgesPub = nh.advertise<CloudS>("edges_features", 1);
  if (this->PublishPlanars)
    this->PlanarsPub = nh.advertise<CloudS>("planars_features", 1);
  if (this->PublishBlobs)
    this->BlobsPub = nh.advertise<CloudS>("blobs_features", 1);
  this->DebugCloudPub = nh.advertise<CloudS>("debug_cloud", 1);

  // Init ROS subscribers and publishers
  priv_nh.getParam("slam_origin_frame", this->SlamOriginFrameId);
  this->PoseCovarPub = nh.advertise<geometry_msgs::PoseWithCovarianceStamped>("slam_pose", 1);
  this->CloudSub = nh.subscribe("velodyne_points", 1, &LidarSlamNode::scanCallback, this);

  ROS_INFO_STREAM("\033[1;32m LiDAR SLAM is ready ! \033[0m");
}

//------------------------------------------------------------------------------
void LidarSlamNode::scanCallback(const CloudV& cloudV)
{
  std::chrono::duration<double, std::milli> chrono_ms;
  std::chrono::high_resolution_clock::time_point start = std::chrono::high_resolution_clock::now();

  // Init this->LaserIdMapping if not already done
  if (!this->LaserIdMapping.size())
  {
    // Iterate through pointcloud to find max ring
    int nLasers = 0;
    for(const PointV& point : cloudV)
    {
      if (point.ring > nLasers)
        nLasers = point.ring;
    }
    ++nLasers;

    // Init this->LaserIdMapping with linear mapping
    this->LaserIdMapping.resize(nLasers);
    for (int i = 0; i < nLasers; i++)
      this->LaserIdMapping[i] = i;

    ROS_INFO_STREAM("[SLAM] Using 0->" << nLasers << " linear laser_id_mapping.");
  }

  // Convert pointcloud PointV type to expected PointS type
  CloudS::Ptr cloudS = convertToSlamPointCloud(cloudV);

  // Run SLAM : register new frame and update position and mapping.
  this->LidarSlam.AddFrame(cloudS, this->LaserIdMapping);

  // Get the computed world transform so far
  Transform worldTransform = this->LidarSlam.GetWorldTransform();
  std::vector<double> poseCovar = this->LidarSlam.GetTransformCovariance();

  // Publish TF, pose and covariance
  publishTfPoseCovar(cloudV.header, worldTransform, poseCovar);

  // Publish optional debug info
  publishFeaturesMaps(cloudS);

  chrono_ms = std::chrono::high_resolution_clock::now() - start;
  std::cout << "SLAM perfomed in : " << chrono_ms.count() << " ms" << std::endl;
}

//------------------------------------------------------------------------------
LidarSlamNode::CloudS::Ptr LidarSlamNode::convertToSlamPointCloud(const CloudV& cloudV)
{
  // Init SLAM pointcloud
  CloudS::Ptr cloudS(new CloudS);
  cloudS->resize(cloudV.size());
  cloudS->header = cloudV.header;

  // Get approximate timestamp of the first point
  double stampInit = pcl_conversions::fromPCL(cloudV.header).stamp.toSec();  // timestamp of last Velodyne raw packet
  stampInit -= 1. / this->LidarFreq;  // approximate timestamp of first Velodyne raw packet

  // Build SLAM pointcloud
  for(unsigned int i = 0; i < cloudV.size(); i++)
  {
    const PointV& velodynePoint = cloudV[i];
    PointS slamPoint;
    slamPoint.x = velodynePoint.x;
    slamPoint.y = velodynePoint.y;
    slamPoint.z = velodynePoint.z;
    slamPoint.intensity = velodynePoint.intensity;
    slamPoint.laserId = velodynePoint.ring;
    double frameAdvancement = (M_PI + std::atan2(velodynePoint.y, velodynePoint.x)) / (M_PI * 2);
    slamPoint.time = stampInit + frameAdvancement / this->LidarFreq;
    // slamPoint.time = frameAdvancement;
    cloudS->at(i) = slamPoint;
  }
  return cloudS;
}

//------------------------------------------------------------------------------
void LidarSlamNode::publishTfPoseCovar(const pcl::PCLHeader& headerCloudV, 
                                       const Transform& worldTransform, 
                                       const std::vector<double>& poseCovar)
{
  // publish worldTransform
  geometry_msgs::TransformStamped tfMsg;
  pcl_conversions::fromPCL(headerCloudV, tfMsg.header);
  tfMsg.header.frame_id = this->SlamOriginFrameId;
  tfMsg.child_frame_id = headerCloudV.frame_id;
  tfMsg.transform.translation.x = worldTransform.x;
  tfMsg.transform.translation.y = worldTransform.y;
  tfMsg.transform.translation.z = worldTransform.z;
  tf2::Quaternion q;
  q.setRPY(worldTransform.rx, worldTransform.ry, worldTransform.rz);
  tfMsg.transform.rotation.x = q.x();
  tfMsg.transform.rotation.y = q.y();
  tfMsg.transform.rotation.z = q.z();
  tfMsg.transform.rotation.w = q.w();
  this->TfBroadcaster.sendTransform(tfMsg);

  // publish pose with covariance
  geometry_msgs::PoseWithCovarianceStamped poseCovarMsg;
  poseCovarMsg.header = tfMsg.header;
  poseCovarMsg.pose.pose.orientation = tfMsg.transform.rotation;
  poseCovarMsg.pose.pose.position.x = worldTransform.x;
  poseCovarMsg.pose.pose.position.y = worldTransform.y;
  poseCovarMsg.pose.pose.position.z = worldTransform.z;
  // Reshape covariance from parameters (rX, rY, rZ, X, Y, Z) to (X, Y, Z, rX, rY, rZ)
  const std::vector<double>& c = poseCovar;
  poseCovarMsg.pose.covariance = {c[21], c[22], c[23],   c[18], c[19], c[20],
                                  c[27], c[28], c[29],   c[24], c[25], c[26],
                                  c[33], c[34], c[35],   c[30], c[31], c[32],

                                  c[ 3], c[ 4], c[ 5],   c[ 0], c[ 1], c[ 2],
                                  c[ 9], c[10], c[11],   c[ 6], c[ 7], c[ 8],
                                  c[15], c[16], c[17],   c[12], c[13], c[14]};
  this->PoseCovarPub.publish(poseCovarMsg);
}

//------------------------------------------------------------------------------
void LidarSlamNode::publishFeaturesMaps(const CloudS::Ptr& cloudS)
{
  // Publish pointcloud only if someone is listening to it to spare bandwidth.
  if (this->DebugCloudPub.getNumSubscribers())
    this->DebugCloudPub.publish(cloudS);

  pcl::PCLHeader msgHeader = cloudS->header;
  msgHeader.frame_id = this->SlamOriginFrameId;

  // Publish edges only if recquired and if someone is listening to it.
  if (this->PublishEdges && this->EdgesPub.getNumSubscribers())
  {
    CloudS::Ptr edgesCloud = this->LidarSlam.GetEdgesMap();
    edgesCloud->header = msgHeader;
    this->EdgesPub.publish(edgesCloud);
  }

  // Publish planars only if recquired and if someone is listening to it.
  if (this->PublishPlanars && this->PlanarsPub.getNumSubscribers())
  {
    CloudS::Ptr planarsCloud = this->LidarSlam.GetPlanarsMap();
    planarsCloud->header = msgHeader;
    this->PlanarsPub.publish(planarsCloud);
  }

  // Publish blobs only if recquired and if someone is listening to it.
  if (this->PublishBlobs && this->BlobsPub.getNumSubscribers())
  {
    CloudS::Ptr blobsCloud = this->LidarSlam.GetBlobsMap();
    blobsCloud->header = msgHeader;
    this->BlobsPub.publish(blobsCloud);
  }
}

//------------------------------------------------------------------------------
void LidarSlamNode::SetSlamParameters(ros::NodeHandle& priv_nh)
{
  // common
  bool fastSlam, undistortion;
  double maxDistanceForICPMatching;
  if (priv_nh.getParam("slam/fast_slam", fastSlam))
    LidarSlam.SetFastSlam(fastSlam);
  if (priv_nh.getParam("slam/undistortion", undistortion))
    LidarSlam.SetUndistortion(undistortion);
  if (priv_nh.getParam("slam/max_distance_for_ICP_matching", maxDistanceForICPMatching))
    LidarSlam.SetMaxDistanceForICPMatching(maxDistanceForICPMatching);

  // ego motion
  int egoMotionLMMaxIter, egoMotionICPMaxIter, egoMotionLineDistanceNbrNeighbors, egoMotionMinimumLineNeighborRejection, egoMotionPlaneDistanceNbrNeighbors;
  double egoMotionLineDistancefactor, egoMotionPlaneDistancefactor1, egoMotionPlaneDistancefactor2, egoMotionMaxLineDistance, egoMotionMaxPlaneDistance, egoMotionInitLossScale, egoMotionFinalLossScale;
  if (priv_nh.getParam("slam/ego_motion_LM_max_iter", egoMotionLMMaxIter))
    LidarSlam.SetEgoMotionLMMaxIter(egoMotionLMMaxIter);
  if (priv_nh.getParam("slam/ego_motion_ICP_max_iter", egoMotionICPMaxIter))
    LidarSlam.SetEgoMotionICPMaxIter(egoMotionICPMaxIter);
  if (priv_nh.getParam("slam/ego_motion_line_distance_nbr_neighbors", egoMotionLineDistanceNbrNeighbors))
    LidarSlam.SetEgoMotionLineDistanceNbrNeighbors(egoMotionLineDistanceNbrNeighbors);
  if (priv_nh.getParam("slam/ego_motion_minimum_line_neighbor_rejection", egoMotionMinimumLineNeighborRejection))
    LidarSlam.SetEgoMotionMinimumLineNeighborRejection(egoMotionMinimumLineNeighborRejection);
  if (priv_nh.getParam("slam/ego_motion_line_distance_factor", egoMotionLineDistancefactor))
    LidarSlam.SetEgoMotionLineDistancefactor(egoMotionLineDistancefactor);
  if (priv_nh.getParam("slam/ego_motion_plane_distance_nbr_neighbors", egoMotionPlaneDistanceNbrNeighbors))
    LidarSlam.SetEgoMotionPlaneDistanceNbrNeighbors(egoMotionPlaneDistanceNbrNeighbors);
  if (priv_nh.getParam("slam/ego_motion_plane_distance_factor1", egoMotionPlaneDistancefactor1))
    LidarSlam.SetEgoMotionPlaneDistancefactor1(egoMotionPlaneDistancefactor1);
  if (priv_nh.getParam("slam/ego_motion_plane_distance_factor2", egoMotionPlaneDistancefactor2))
    LidarSlam.SetEgoMotionPlaneDistancefactor2(egoMotionPlaneDistancefactor2);
  if (priv_nh.getParam("slam/ego_motion_max_line_distance", egoMotionMaxLineDistance))
    LidarSlam.SetEgoMotionMaxLineDistance(egoMotionMaxLineDistance);
  if (priv_nh.getParam("slam/ego_motion_max_plane_distance", egoMotionMaxPlaneDistance))
    LidarSlam.SetEgoMotionMaxPlaneDistance(egoMotionMaxPlaneDistance);
  if (priv_nh.getParam("slam/ego_motion_init_loss_scale", egoMotionInitLossScale))
    LidarSlam.SetEgoMotionInitLossScale(egoMotionInitLossScale);
  if (priv_nh.getParam("slam/ego_motion_final_loss_scale", egoMotionFinalLossScale))
    LidarSlam.SetEgoMotionFinalLossScale(egoMotionFinalLossScale);

  // mapping
  int mappingLMMaxIter, mappingICPMaxIter, mappingLineDistanceNbrNeighbors, mappingMinimumLineNeighborRejection, mappingPlaneDistanceNbrNeighbors;
  double mappingLineDistancefactor, mappingPlaneDistancefactor1, mappingPlaneDistancefactor2, mappingMaxLineDistance, mappingMaxPlaneDistance, mappingLineMaxDistInlier, mappingInitLossScale, mappingFinalLossScale;
  if (priv_nh.getParam("slam/mapping_LM_max_iter", mappingLMMaxIter))
    LidarSlam.SetMappingLMMaxIter(mappingLMMaxIter);
  if (priv_nh.getParam("slam/mapping_ICP_max_iter", mappingICPMaxIter))
    LidarSlam.SetMappingICPMaxIter(mappingICPMaxIter);
  if (priv_nh.getParam("slam/mapping_line_distance_nbr_neighbors", mappingLineDistanceNbrNeighbors))
    LidarSlam.SetMappingLineDistanceNbrNeighbors(mappingLineDistanceNbrNeighbors);
  if (priv_nh.getParam("slam/mapping_minimum_line_neighbor_rejection", mappingMinimumLineNeighborRejection))
    LidarSlam.SetMappingMinimumLineNeighborRejection(mappingMinimumLineNeighborRejection);
  if (priv_nh.getParam("slam/mapping_line_distance_factor", mappingLineDistancefactor))
    LidarSlam.SetMappingLineDistancefactor(mappingLineDistancefactor);
  if (priv_nh.getParam("slam/mapping_plane_distance_nbr_neighbors", mappingPlaneDistanceNbrNeighbors))
    LidarSlam.SetMappingPlaneDistanceNbrNeighbors(mappingPlaneDistanceNbrNeighbors);
  if (priv_nh.getParam("slam/mapping_plane_distance_factor1", mappingPlaneDistancefactor1))
    LidarSlam.SetMappingPlaneDistancefactor1(mappingPlaneDistancefactor1);
  if (priv_nh.getParam("slam/mapping_plane_distance_factor2", mappingPlaneDistancefactor2))
    LidarSlam.SetMappingPlaneDistancefactor2(mappingPlaneDistancefactor2);
  if (priv_nh.getParam("slam/mapping_max_line_distance", mappingMaxLineDistance))
    LidarSlam.SetMappingMaxLineDistance(mappingMaxLineDistance);
  if (priv_nh.getParam("slam/mapping_max_plane_distance", mappingMaxPlaneDistance))
    LidarSlam.SetMappingMaxPlaneDistance(mappingMaxPlaneDistance);
  if (priv_nh.getParam("slam/mapping_line_max_dist_inlier", mappingLineMaxDistInlier))
    LidarSlam.SetMappingLineMaxDistInlier(mappingLineMaxDistInlier);
  if (priv_nh.getParam("slam/mapping_init_loss_scale", mappingInitLossScale))
    LidarSlam.SetMappingInitLossScale(mappingInitLossScale);
  if (priv_nh.getParam("slam/mapping_final_loss_scale", mappingFinalLossScale))
    LidarSlam.SetMappingFinalLossScale(mappingFinalLossScale);

  // rolling grids
  double voxelGridLeafSizeEdges, voxelGridLeafSizePlanes, voxelGridLeafSizeBlobs, voxelGridSize, voxelGridResolution;
  if (priv_nh.getParam("slam/voxel_grid_leaf_size_edges", voxelGridLeafSizeEdges))
    LidarSlam.SetVoxelGridLeafSizeEdges(voxelGridLeafSizeEdges);
  if (priv_nh.getParam("slam/voxel_grid_leaf_size_planes", voxelGridLeafSizePlanes))
    LidarSlam.SetVoxelGridLeafSizePlanes(voxelGridLeafSizePlanes);
  if (priv_nh.getParam("slam/voxel_grid_leaf_size_blobs", voxelGridLeafSizeBlobs))
    LidarSlam.SetVoxelGridLeafSizeBlobs(voxelGridLeafSizeBlobs);
  if (priv_nh.getParam("slam/voxel_grid_size", voxelGridSize))
    LidarSlam.SetVoxelGridSize(voxelGridSize);
  if (priv_nh.getParam("slam/voxel_grid_resolution", voxelGridResolution))
    LidarSlam.SetVoxelGridResolution(voxelGridResolution);
}