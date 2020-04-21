#include "LidarSlamNode.h"
#include "ros_transform_utils.h"
#include <LidarSlam/GlobalTrajectoriesRegistration.h>

#include <pcl_conversions/pcl_conversions.h>
#include <geometry_msgs/TransformStamped.h>
#include <nav_msgs/Path.h>

#define GREEN(s) "\033[1;32m" << s << "\033[0m"

enum Output
{
  POSE_ODOM,             // Publish SLAM pose as an Odometry msg on 'slam_odom' topic (default : true).
  POSE_TF,               // Publish SLAM pose as a TF from 'odometry_frame' to 'tracking_frame' (default : true).
  POSE_PREDICTION_ODOM,  // Publish latency-corrected SLAM pose as an Odometry msg on 'slam_predicted_odom' topic.
  POSE_PREDICTION_TF,    // Publish latency-corrected SLAM pose as a TF from 'odometry_frame' to '<tracking_frame>_prediction'.

  EDGES_MAP,             // Publish edges keypoints map as a PointXYZTIId PointCloud2 msg to topic 'edges_map'.
  PLANES_MAP,            // Publish planes keypoints map as a PointXYZTIId PointCloud2 msg to topic 'planes_map'.
  BLOBS_MAP,             // Publish blobs keypoints map as a PointXYZTIId PointCloud2 msg to topic 'blobs_map'.

  SLAM_CLOUD,            // Publish SLAM pointcloud as PointXYZTIId PointCloud2 msg to topic 'slam_cloud'.

  PGO_PATH,              // Publish optimized SLAM trajectory as Path msg to 'optim_slam_traj' latched topic.
  ICP_CALIB_SLAM_PATH,   // Publish ICP-aligned SLAM trajectory as Path msg to 'icp_slam' latched topic.
  ICP_CALIB_GPS_PATH     // Publish ICP-aligned GPS trajectory as Path msg to 'icp_gps' latched topic.
};

//==============================================================================
//   Basic SLAM use
//==============================================================================

//------------------------------------------------------------------------------
LidarSlamNode::LidarSlamNode(ros::NodeHandle& nh, ros::NodeHandle& priv_nh)
  : TfListener(TfBuffer)
{
  // Get SLAM params
  this->SetSlamParameters(priv_nh);

  // ***************************************************************************
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

  // ***************************************************************************
  // Get LiDAR frequency
  priv_nh.getParam("lidar_frequency", this->LidarFreq);

  // Get PCD saving parameters
  int pcdFormat;
  if (priv_nh.getParam("pcd_saving/pcd_format", pcdFormat))
  {
    this->PcdFormat = static_cast<PCDFormat>(pcdFormat);
    if (pcdFormat != PCDFormat::ASCII && pcdFormat != PCDFormat::BINARY && pcdFormat != PCDFormat::BINARY_COMPRESSED)
    {
      ROS_ERROR_STREAM("Incorrect PCD format value (" << pcdFormat << "). Setting it to 'BINARY_COMPRESSED'.");
      this->PcdFormat = PCDFormat::BINARY_COMPRESSED;
    }
  }

  // ***************************************************************************
  // Init GPS/SLAM calibration or Pose Graph Optimization.
  priv_nh.getParam("gps/use_gps", this->UseGps);
  if (this->UseGps)
  {
    // Init logging of GPS data for GPS/SLAM calibration or Pose Graph Optimization.
    this->GpsOdomSub = nh.subscribe("gps_odom", 1, &LidarSlamNode::GpsCallback, this);

    // Init GPS/SLAM calibration to output SLAM pose to world coordinates.
    priv_nh.getParam("gps/calibration/no_roll", this->CalibrationNoRoll);

    // Init optional use of GPS data to perform pose graph optimization to correct SLAM poses and maps.
    priv_nh.getParam("gps/pose_graph_optimization/g2o_file_name", this->PgoG2oFileName);
  }

  // ***************************************************************************
  // Init publishers

  #define initPublisher(publisher, topic, type, rosParam, publishDefault, queue, latch)   \
    priv_nh.param(rosParam, this->Publish[publisher], publishDefault);                    \
    if (this->Publish[publisher])                                                         \
      this->Publishers[publisher] = nh.advertise<type>(topic, queue, latch);

  priv_nh.param("output/pose/tf",           this->Publish[POSE_TF],            true);
  priv_nh.param("output/pose/predicted_tf", this->Publish[POSE_PREDICTION_TF], false);
  initPublisher(POSE_ODOM, "slam_odom",  nav_msgs::Odometry, "output/pose/odom", true, 1, false);
  initPublisher(POSE_PREDICTION_ODOM, "slam_predicted_odom", nav_msgs::Odometry, "output/pose/predicted_odom", false, 1, false);

  initPublisher(EDGES_MAP,  "edges_features",   CloudS, "output/maps/edges",  false, 1, false);
  initPublisher(PLANES_MAP, "planars_features", CloudS, "output/maps/planes", false, 1, false);
  initPublisher(BLOBS_MAP,  "blobs_features",   CloudS, "output/maps/blobs",  false, 1, false);

  initPublisher(SLAM_CLOUD, "slam_cloud", CloudS, "output/debug/cloud", false, 1, false);

  if (this->UseGps)
  {
    initPublisher(PGO_PATH, "optim_slam_traj", nav_msgs::Path, "gps/pose_graph_optimization/publish_optimized_trajectory", false, 1, true);
    initPublisher(ICP_CALIB_SLAM_PATH, "icp_slam", nav_msgs::Path, "gps/calibration/publish_icp_trajectories", false, 1, true);
    initPublisher(ICP_CALIB_GPS_PATH,  "icp_gps",  nav_msgs::Path, "gps/calibration/publish_icp_trajectories", false, 1, true);
  }

  // ***************************************************************************
  // Init basic ROS subscribers
  this->CloudSub = nh.subscribe("velodyne_points", 1, &LidarSlamNode::ScanCallback, this);
  this->SlamCommandSub = nh.subscribe("slam_command", 1,  &LidarSlamNode::SlamCommandCallback, this);

  ROS_INFO_STREAM(GREEN("LiDAR SLAM is ready !"));
}

//------------------------------------------------------------------------------
void LidarSlamNode::ScanCallback(const CloudV& cloudV)
{
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
  CloudS::Ptr cloudS = this->ConvertToSlamPointCloud(cloudV);

  // If no tracking frame is set, track input pointcloud origin
  if (this->TrackingFrameId.empty())
    this->TrackingFrameId = cloudS->header.frame_id;
  // Update TF from BASE to LiDAR
  this->UpdateBaseToLidarOffset(cloudS->header.frame_id, cloudS->header.stamp);

  // Run SLAM : register new frame and update position and mapping.
  this->LidarSlam.AddFrame(cloudS, this->LaserIdMapping);

  // Get and publish the computed world transform so far with its associated covariance
  this->PublishTfOdom();

  // Publish optional info
  // (publish pointclouds only if someone is listening to it to spare bandwidth)
  if (this->Publish[SLAM_CLOUD] && this->Publishers[SLAM_CLOUD].getNumSubscribers())
    this->Publishers[SLAM_CLOUD].publish(cloudS);
  this->PublishFeaturesMaps(cloudS->header.stamp);
}

//------------------------------------------------------------------------------
void LidarSlamNode::GpsCallback(const nav_msgs::Odometry& msg)
{
  // If GPS/SLAM calibration is needed, save GPS pose for later use
  if (this->UseGps)
  {
    // Add new pose and its covariance to buffer
    const auto& c = msg.pose.covariance;
    std::array<double, 9> gpsCovar = {c[ 0], c[ 1], c[ 2],
                                      c[ 6], c[ 7], c[ 8],
                                      c[12], c[13], c[14]};
    this->GpsPoses.emplace_back(PoseMsgToTransform(msg.pose.pose, msg.header.stamp.toSec(), msg.header.frame_id));
    this->GpsCovars.emplace_back(gpsCovar);

    double loggingTimeout = this->LidarSlam.GetLoggingTimeout();
    // If a timeout is defined, forget too old data
    if (loggingTimeout > 0)
    {
      // Forget all previous poses older than loggingTimeout
      while (this->GpsPoses.back().time - this->GpsPoses.front().time > loggingTimeout)
      {
        this->GpsPoses.pop_front();
        this->GpsCovars.pop_front();
      }
    }

    // Update BASE to GPS offset
    // Get the latest transform (we expect a static transform, so timestamp does not matter)
    Tf2LookupTransform(this->BaseToGpsOffset, this->TfBuffer, this->TrackingFrameId, msg.child_frame_id);

    // If there is a request to set SLAM pose from GPS, do it.
    // This should be done only after pose graph optimization.
    if (this->SetSlamPoseFromGpsRequest)
    {
      Transform& gpsPose = this->GpsPoses.back();
      Transform lidarPose = Transform(this->BaseToGpsOffset * gpsPose.GetIsometry(), gpsPose.time, gpsPose.frameid);
      this->LidarSlam.SetWorldTransformFromGuess(lidarPose);
      this->SetSlamPoseFromGpsRequest = false;
      ROS_WARN_STREAM("SLAM pose set from GPS pose to :\n" << this->GpsPoses.back().GetMatrix());
    }
  }
}

//------------------------------------------------------------------------------
void LidarSlamNode::SlamCommandCallback(const lidar_slam::SlamCommand& msg)
{
  // Parse command
  switch(msg.command)
  {
    // Run GPS/SLAM calibration
    case lidar_slam::SlamCommand::GPS_SLAM_CALIBRATION:
      this->GpsSlamCalibration();
      break;

    // Run SLAM pose graph optimization with GPS positions prior
    case lidar_slam::SlamCommand::GPS_SLAM_POSE_GRAPH_OPTIMIZATION:
      // TODO : run PGO in separated thread
      this->PoseGraphOptimization();
      break;

    // Request to set SLAM pose from next GPS pose
    // NOTE : This function should only be called after PGO has been triggered.
    case lidar_slam::SlamCommand::SET_SLAM_POSE_FROM_NEXT_GPS:
      if (!this->UseGps)
      {
        ROS_ERROR_STREAM("Cannot set SLAM pose from GPS as GPS logging has not been enabled. "
                        "Please set 'gps/use_gps' private parameter to 'true'.");
        return;
      }
      this->SetSlamPoseFromGpsRequest = true;
      ROS_WARN_STREAM("Request to set SLAM pose set from next GPS pose received.");
      break;

    // Enable SLAM maps update
    case lidar_slam::SlamCommand::ENABLE_SLAM_MAP_UPDATE:
      this->LidarSlam.SetUpdateMap(true);
      ROS_WARN_STREAM("Enabling SLAM maps update.");
      break;

    // Disable SLAM maps update
    case lidar_slam::SlamCommand::DISABLE_SLAM_MAP_UPDATE:
      this->LidarSlam.SetUpdateMap(false);
      ROS_WARN_STREAM("Disabling SLAM maps update.");
      break;

    // Save SLAM keypoints maps to PCD files
    case lidar_slam::SlamCommand::SAVE_KEYPOINTS_MAPS:
      ROS_INFO_STREAM("Saving keypoints maps to PCD.");
      this->LidarSlam.SaveMapsToPCD(msg.string_arg, this->PcdFormat);
      break;

    // Load SLAM keypoints maps from PCD files
    case lidar_slam::SlamCommand::LOAD_KEYPOINTS_MAPS:
      ROS_INFO_STREAM("Loading keypoints maps from PCD.");
      this->LidarSlam.LoadMapsFromPCD(msg.string_arg);
      break;

    // Unkown command
    default:
      ROS_ERROR_STREAM("Unknown SLAM command : " << (unsigned int) msg.command);
      break;
  }
}

//==============================================================================
//   Special SLAM commands
//==============================================================================

//------------------------------------------------------------------------------
void LidarSlamNode::GpsSlamCalibration()
{
  if (!this->UseGps)
  {
    ROS_ERROR_STREAM("Cannot run GPS/SLAM calibration as GPS logging has not been enabled. "
                     "Please set 'gps/use_gps' private parameter to 'true'.");
    return;
  }

  // Transform to modifiable vectors
  std::vector<Transform> odomToBasePoses = this->LidarSlam.GetTrajectory();
  std::vector<Transform> worldToGpsPoses(this->GpsPoses.begin(), this->GpsPoses.end());

  if (worldToGpsPoses.size() < 2 && odomToBasePoses.size() < 2)
  {
    ROS_ERROR_STREAM("Not enough points to run SLAM/GPS calibration "
                      "(only got " << odomToBasePoses.size() << " slam points "
                      "and " << worldToGpsPoses.size() << " gps points).");
    return;
  }
  // If we have enough GPS and SLAM points, run calibration
  ROS_INFO_STREAM("Running SLAM/GPS calibration with " << odomToBasePoses.size() << " slam points and "
                                                       << worldToGpsPoses.size() << " gps points.");

  // If a sensors offset is given, use it to compute real GPS antenna position in SLAM origin coordinates
  if (!this->BaseToGpsOffset.isApprox(Eigen::Isometry3d::Identity()))
  {
    if (this->LidarSlam.GetVerbosity() >= 2)
    {
      std::cout << "Transforming LiDAR pose aquired by SLAM to GPS antenna pose using LIDAR to GPS antenna offset :"
                << std::endl << this->BaseToGpsOffset.matrix() << std::endl;
    }
    for (Transform& odomToGpsPose : odomToBasePoses)
    {
      Eigen::Isometry3d odomToBase = odomToGpsPose.GetIsometry();
      odomToGpsPose.SetIsometry(odomToBase * this->BaseToGpsOffset);
    }
  }
  // At this point, we now have GPS antenna poses in SLAM coordinates.
  const std::vector<Transform>& odomToGpsPoses = odomToBasePoses;

  // Run calibration : compute transform from SLAM to WORLD
  GlobalTrajectoriesRegistration registration;
  registration.SetNoRoll(this->CalibrationNoRoll);  // DEBUG
  registration.SetVerbose(this->LidarSlam.GetVerbosity() >= 2);
  Eigen::Isometry3d worldToOdom;
  if (!registration.ComputeTransformOffset(odomToGpsPoses, worldToGpsPoses, worldToOdom))
  {
    ROS_ERROR_STREAM("GPS/SLAM calibration failed.");
    return;
  }
  const std::string& gpsFrameId = this->GpsPoses[0].frameid;

  // Publish ICP-matched trajectories
  // GPS antenna trajectory acquired from GPS in WORLD coordinates
  if (this->Publish[ICP_CALIB_GPS_PATH])
  {
    nav_msgs::Path gpsPath;
    gpsPath.header.frame_id = gpsFrameId;
    gpsPath.header.stamp = ros::Time::now();
    for (const Transform& pose: worldToGpsPoses)
    {
      gpsPath.poses.emplace_back(TransformToPoseStampedMsg(pose));
    }
    this->Publishers[ICP_CALIB_GPS_PATH].publish(gpsPath);
  }
  // GPS antenna trajectory acquired from SLAM in WORLD coordinates
  if (this->Publish[ICP_CALIB_SLAM_PATH])
  {
    nav_msgs::Path slamPath;
    slamPath.header.frame_id = gpsFrameId;
    slamPath.header.stamp = ros::Time::now();
    for (const Transform& pose: odomToGpsPoses)
    {
      Transform worldToGpsPose(worldToOdom * pose.GetIsometry(), pose.time, pose.frameid);
      slamPath.poses.emplace_back(TransformToPoseStampedMsg(worldToGpsPose));
    }
    this->Publishers[ICP_CALIB_SLAM_PATH].publish(slamPath);
  }

  // Publish static tf with calibration to link world (UTM) frame to SLAM odometry origin
  geometry_msgs::TransformStamped tfStamped;
  tfStamped.header.stamp = ros::Time::now();
  tfStamped.header.frame_id = gpsFrameId;
  tfStamped.child_frame_id = this->OdometryFrameId;
  tfStamped.transform = TransformToTfMsg(Transform(worldToOdom));
  this->StaticTfBroadcaster.sendTransform(tfStamped);

  Eigen::Vector3d xyz = worldToOdom.translation();
  Eigen::Vector3d ypr = worldToOdom.linear().eulerAngles(2, 1, 0);
  ROS_INFO_STREAM(GREEN("Global transform from '" << gpsFrameId << "' to '" << this->OdometryFrameId << "' " <<
                  "successfully estimated to :\n" << worldToOdom.matrix() << "\n" <<
                  "(tf2 static transform : " << xyz.transpose() << " " << ypr.transpose() << " " << gpsFrameId << " " << this->OdometryFrameId << ")"));
}

//------------------------------------------------------------------------------
void LidarSlamNode::PoseGraphOptimization()
{
  if (!this->UseGps)
  {
    ROS_ERROR_STREAM("Cannot run pose graph optimization as GPS logging has not been enabled. "
                     "Please set 'gps/use_gps' private parameter to 'true'.");
    return;
  }

  // Transform to modifiable vectors
  std::vector<Transform> worldToGpsPositions(this->GpsPoses.begin(), this->GpsPoses.end());
  std::vector<std::array<double, 9>> worldToGpsCovars(this->GpsCovars.begin(), this->GpsCovars.end());

  // Run pose graph optimization
  Eigen::Isometry3d gpsToBaseOffset = this->BaseToGpsOffset.inverse();
  this->LidarSlam.RunPoseGraphOptimization(worldToGpsPositions, worldToGpsCovars,
                                           gpsToBaseOffset, this->PgoG2oFileName);

  // Update GPS/LiDAR calibration
  this->BaseToGpsOffset = gpsToBaseOffset.inverse();

  // Update the display of the computed world transform so far
  this->PublishTfOdom();

  // Publish static tf with calibration to link world (UTM) frame to SLAM origin
  Transform odomToBase = this->LidarSlam.GetWorldTransform();
  geometry_msgs::TransformStamped tfStamped;
  tfStamped.header.stamp = ros::Time(odomToBase.time);
  tfStamped.header.frame_id = this->GpsPoses[0].frameid;
  tfStamped.child_frame_id = this->OdometryFrameId;
  tfStamped.transform = TransformToTfMsg(Transform(gpsToBaseOffset));
  this->StaticTfBroadcaster.sendTransform(tfStamped);

  // Publish optimized SLAM trajectory
  if (this->Publish[PGO_PATH])
  {
    nav_msgs::Path optimSlamTraj;
    optimSlamTraj.header.frame_id = this->OdometryFrameId;
    optimSlamTraj.header.stamp = ros::Time(odomToBase.time);
    std::vector<Transform> optimizedSlamPoses = this->LidarSlam.GetTrajectory();
    for (const Transform& pose: optimizedSlamPoses)
      optimSlamTraj.poses.emplace_back(TransformToPoseStampedMsg(pose));
    this->Publishers[PGO_PATH].publish(optimSlamTraj);
  }

  // Update features maps display
  this->PublishFeaturesMaps(odomToBase.time * 1e6);
}

//==============================================================================
//   Utilities
//==============================================================================

//------------------------------------------------------------------------------
LidarSlamNode::CloudS::Ptr LidarSlamNode::ConvertToSlamPointCloud(const CloudV& cloudV) const
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
    PointS& slamPoint = cloudS->at(i);
    slamPoint.x = velodynePoint.x;
    slamPoint.y = velodynePoint.y;
    slamPoint.z = velodynePoint.z;
    slamPoint.intensity = velodynePoint.intensity;
    slamPoint.laserId = velodynePoint.ring;
    double frameAdvancement = (M_PI + std::atan2(velodynePoint.y, velodynePoint.x)) / (M_PI * 2);
    slamPoint.time = stampInit + frameAdvancement / this->LidarFreq;
    // slamPoint.time = frameAdvancement;
  }
  return cloudS;
}

//------------------------------------------------------------------------------
void LidarSlamNode::UpdateBaseToLidarOffset(const std::string& lidarFrameId, uint64_t pclStamp)
{
  // If tracking frame is different from input frame, get TF from LiDAR to BASE
  if (lidarFrameId != this->TrackingFrameId)
  {
    // We expect a static transform between BASE and LIDAR, so we don't care
    // about timestamp and get only the latest transform
    Eigen::Isometry3d baseToLidar;
    if (Tf2LookupTransform(baseToLidar, this->TfBuffer, this->TrackingFrameId, lidarFrameId))
      this->LidarSlam.SetBaseToLidarOffset(baseToLidar);
  }
}

//------------------------------------------------------------------------------
void LidarSlamNode::PublishTfOdom()
{
  // Publish SLAM pose
  if (this->Publish[POSE_ODOM] || this->Publish[POSE_TF])
  {
    // Get SLAM pose
    Transform odomToBase = this->LidarSlam.GetWorldTransform();

    // Publish as odometry msg
    if (this->Publish[POSE_ODOM])
    {
      nav_msgs::Odometry odomMsg;
      odomMsg.header.stamp = ros::Time(odomToBase.time);
      odomMsg.header.frame_id = this->OdometryFrameId;
      odomMsg.child_frame_id = this->TrackingFrameId;
      odomMsg.pose.pose = TransformToPoseMsg(odomToBase);
      auto covar = this->LidarSlam.GetTransformCovariance();
      std::copy(covar.begin(), covar.end(), odomMsg.pose.covariance.begin());
      this->Publishers[POSE_ODOM].publish(odomMsg);
    }

    // Publish as TF from OdometryFrameId to TrackingFrameId
    if (this->Publish[POSE_TF])
    {
      geometry_msgs::TransformStamped tfMsg;
      tfMsg.header.stamp = ros::Time(odomToBase.time);
      tfMsg.header.frame_id = this->OdometryFrameId;
      tfMsg.child_frame_id = this->TrackingFrameId;
      tfMsg.transform = TransformToTfMsg(odomToBase);
      this->TfBroadcaster.sendTransform(tfMsg);
    }
  }

  // Publish latency compensated SLAM pose
  if (this->Publish[POSE_PREDICTION_ODOM] || this->Publish[POSE_PREDICTION_TF])
  {
    // Get latency corrected SLAM pose
    Transform odomToBasePred = this->LidarSlam.GetLatencyCompensatedWorldTransform();

    // Publish as odometry msg
    if (this->Publish[POSE_PREDICTION_ODOM])
    {
      nav_msgs::Odometry odomMsg;
      odomMsg.header.stamp = ros::Time(odomToBasePred.time);
      odomMsg.header.frame_id = this->OdometryFrameId;
      odomMsg.child_frame_id = this->TrackingFrameId + "_prediction";
      odomMsg.pose.pose = TransformToPoseMsg(odomToBasePred);
      auto covar = this->LidarSlam.GetTransformCovariance();
      std::copy(covar.begin(), covar.end(), odomMsg.pose.covariance.begin());
      this->Publishers[POSE_PREDICTION_ODOM].publish(odomMsg);
    }

    // Publish as TF from OdometryFrameId to <TrackingFrameId>_prediction
    if (this->Publish[POSE_PREDICTION_TF])
    {
      geometry_msgs::TransformStamped tfMsg;
      tfMsg.header.stamp = ros::Time(odomToBasePred.time);
      tfMsg.header.frame_id = this->OdometryFrameId;
      tfMsg.child_frame_id = this->TrackingFrameId + "_prediction";
      tfMsg.transform = TransformToTfMsg(odomToBasePred);
      this->TfBroadcaster.sendTransform(tfMsg);
    }
  }
}

//------------------------------------------------------------------------------
void LidarSlamNode::PublishFeaturesMaps(uint64_t pclStamp)
{
  pcl::PCLHeader msgHeader;
  msgHeader.stamp = pclStamp;
  msgHeader.frame_id = this->OdometryFrameId;

  // Publish edges only if required and if someone is listening to it.
  if (this->Publish[EDGES_MAP] && this->Publishers[EDGES_MAP].getNumSubscribers())
  {
    CloudS::Ptr edgesCloud = this->LidarSlam.GetEdgesMap();
    edgesCloud->header = msgHeader;
    this->Publishers[EDGES_MAP].publish(edgesCloud);
  }

  // Publish planars only if required and if someone is listening to it.
  if (this->Publish[PLANES_MAP] && this->Publishers[PLANES_MAP].getNumSubscribers())
  {
    CloudS::Ptr planarsCloud = this->LidarSlam.GetPlanarsMap();
    planarsCloud->header = msgHeader;
    this->Publishers[PLANES_MAP].publish(planarsCloud);
  }

  // Publish blobs only if required and if someone is listening to it.
  if (this->Publish[BLOBS_MAP] && this->Publishers[BLOBS_MAP].getNumSubscribers())
  {
    CloudS::Ptr blobsCloud = this->LidarSlam.GetBlobsMap();
    blobsCloud->header = msgHeader;
    this->Publishers[BLOBS_MAP].publish(blobsCloud);
  }
}

//------------------------------------------------------------------------------
void LidarSlamNode::SetSlamParameters(ros::NodeHandle& priv_nh)
{
  #define SetSlamParam(type, rosParam, slamParam) { type val; if (priv_nh.getParam(rosParam, val)) this->LidarSlam.Set##slamParam(val); }

  // common
  SetSlamParam(bool, "slam/fast_slam", FastSlam)
  SetSlamParam(bool, "slam/undistortion", Undistortion)
  SetSlamParam(int, "slam/verbosity", Verbosity)
  SetSlamParam(int, "slam/n_threads", NbThreads)
  SetSlamParam(int, "slam/logging_timeout", LoggingTimeout)
  SetSlamParam(double, "slam/max_distance_for_ICP_matching", MaxDistanceForICPMatching)
  int  pointCloudStorage;
  if (priv_nh.getParam("slam/logging_storage", pointCloudStorage))
  {
    PointCloudStorageType storage = static_cast<PointCloudStorageType>(pointCloudStorage);
    if (storage != PCL_CLOUD && storage != OCTREE_COMPRESSED &&
        storage != PCD_ASCII && storage != PCD_BINARY && storage != PCD_BINARY_COMPRESSED)
    {
      ROS_ERROR_STREAM("Incorrect pointcloud logging type value (" << storage << "). Setting it to 'PCL'.");
      storage = PCL_CLOUD;
    }
    LidarSlam.SetLoggingStorage(storage);
  }

  // frame Ids
  priv_nh.param("odometry_frame", this->OdometryFrameId, this->OdometryFrameId);
  this->LidarSlam.SetWorldFrameId(this->OdometryFrameId);
  if (priv_nh.getParam("tracking_frame", this->TrackingFrameId))
    this->LidarSlam.SetBaseFrameId(this->TrackingFrameId);

  // ego motion
  SetSlamParam(int, "slam/ego_motion_LM_max_iter", EgoMotionLMMaxIter)
  SetSlamParam(int, "slam/ego_motion_ICP_max_iter", EgoMotionICPMaxIter)
  SetSlamParam(int, "slam/ego_motion_line_distance_nbr_neighbors", EgoMotionLineDistanceNbrNeighbors)
  SetSlamParam(int, "slam/ego_motion_minimum_line_neighbor_rejection", EgoMotionMinimumLineNeighborRejection)
  SetSlamParam(int, "slam/ego_motion_plane_distance_nbr_neighbors", EgoMotionPlaneDistanceNbrNeighbors)
  SetSlamParam(double, "slam/ego_motion_line_distance_factor", EgoMotionLineDistancefactor)
  SetSlamParam(double, "slam/ego_motion_plane_distance_factor1", EgoMotionPlaneDistancefactor1)
  SetSlamParam(double, "slam/ego_motion_plane_distance_factor2", EgoMotionPlaneDistancefactor2)
  SetSlamParam(double, "slam/ego_motion_max_line_distance", EgoMotionMaxLineDistance)
  SetSlamParam(double, "slam/ego_motion_max_plane_distance", EgoMotionMaxPlaneDistance)
  SetSlamParam(double, "slam/ego_motion_init_loss_scale", EgoMotionInitLossScale)
  SetSlamParam(double, "slam/ego_motion_final_loss_scale", EgoMotionFinalLossScale)

  // mapping
  SetSlamParam(int, "slam/mapping_LM_max_iter", MappingLMMaxIter)
  SetSlamParam(int, "slam/mapping_ICP_max_iter", MappingICPMaxIter)
  SetSlamParam(int, "slam/mapping_line_distance_nbr_neighbors", MappingLineDistanceNbrNeighbors)
  SetSlamParam(int, "slam/mapping_minimum_line_neighbor_rejection", MappingMinimumLineNeighborRejection)
  SetSlamParam(int, "slam/mapping_plane_distance_nbr_neighbors", MappingPlaneDistanceNbrNeighbors)
  SetSlamParam(double, "slam/mapping_line_distance_factor", MappingLineDistancefactor)
  SetSlamParam(double, "slam/mapping_line_max_dist_inlier", MappingLineMaxDistInlier)
  SetSlamParam(double, "slam/mapping_plane_distance_factor1", MappingPlaneDistancefactor1)
  SetSlamParam(double, "slam/mapping_plane_distance_factor2", MappingPlaneDistancefactor2)
  SetSlamParam(double, "slam/mapping_max_line_distance", MappingMaxLineDistance)
  SetSlamParam(double, "slam/mapping_max_plane_distance", MappingMaxPlaneDistance)
  SetSlamParam(double, "slam/mapping_init_loss_scale", MappingInitLossScale)
  SetSlamParam(double, "slam/mapping_final_loss_scale", MappingFinalLossScale)

  // rolling grids
  SetSlamParam(double, "slam/voxel_grid_leaf_size_edges", VoxelGridLeafSizeEdges)
  SetSlamParam(double, "slam/voxel_grid_leaf_size_planes", VoxelGridLeafSizePlanes)
  SetSlamParam(double, "slam/voxel_grid_leaf_size_blobs", VoxelGridLeafSizeBlobs)
  SetSlamParam(double, "slam/voxel_grid_resolution", VoxelGridResolution)
  SetSlamParam(int, "slam/voxel_grid_size", VoxelGridSize)

  // keypoints extractor
  #define SetKeypointsExtractorParam(type, rosParam, keParam) {type val; if (priv_nh.getParam(rosParam, val)) this->LidarSlam.GetKeyPointsExtractor()->Set##keParam(val);}
  SetKeypointsExtractorParam(int, "slam/ke_neighbor_width", NeighborWidth)
  SetKeypointsExtractorParam(double, "slam/ke_min_distance_to_sensor", MinDistanceToSensor)
  SetKeypointsExtractorParam(double, "slam/ke_angle_resolution", AngleResolution)
  SetKeypointsExtractorParam(double, "slam/ke_plane_sin_angle_threshold", PlaneSinAngleThreshold)
  SetKeypointsExtractorParam(double, "slam/ke_edge_sin_angle_threshold", EdgeSinAngleThreshold)
  // SetKeypointsExtractorParam(double, "slam/ke_dist_to_line_threshold", DistToLineThreshold)
  SetKeypointsExtractorParam(double, "slam/ke_edge_depth_gap_threshold", EdgeDepthGapThreshold)
  SetKeypointsExtractorParam(double, "slam/ke_edge_saliency_threshold", EdgeSaliencyThreshold)
  SetKeypointsExtractorParam(double, "slam/ke_edge_intensity_gap_threshold", EdgeIntensityGapThreshold)
}

//------------------------------------------------------------------------------
/*!
 * @brief Main node entry point.
 */
int main(int argc, char **argv)
{
  ros::init(argc, argv, "lidar_slam");
  ros::NodeHandle nh;
  ros::NodeHandle priv_nh("~");

  // create lidar slam node, which subscribes to pointclouds
  LidarSlamNode slam(nh, priv_nh);

  // handle callbacks until shut down
  ros::spin();

  return 0;
}