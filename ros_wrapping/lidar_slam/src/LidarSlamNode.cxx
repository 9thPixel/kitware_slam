#include "LidarSlamNode.h"

#include <pcl_conversions/pcl_conversions.h>
#include <geometry_msgs/TransformStamped.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <tf2/LinearMath/Quaternion.h>

//------------------------------------------------------------------------------
LidarSlamNode::LidarSlamNode(ros::NodeHandle& nh, ros::NodeHandle& priv_nh)
{
  // TODO : Init laser id mapping and nLasers with ROS param
  size_t nLasers = 16;
  laserIdMapping_.resize(nLasers);
  for (unsigned int i = 0; i < laserIdMapping_.size(); i++)
    laserIdMapping_[i] = i;

  // init ROS stuff
  debugCloudPub_ = nh.advertise<CloudS>("debug_cloud", 1);
  poseCovarPub_ = nh.advertise<geometry_msgs::PoseWithCovarianceStamped>("slam_pose", 1);
  cloudSub_ = nh.subscribe("/velodyne_points", 1, &LidarSlamNode::scanCallback, this);
}

//------------------------------------------------------------------------------
void LidarSlamNode::scanCallback(const CloudV& cloudV)
{
  // Convert pointcloud PointV type to expected PointS type
  CloudS::Ptr cloudS = convertToSlamPointCloud(cloudV);

  // publish converted PC2
  debugCloudPub_.publish(cloudS);

  // run SLAM : register new frame and update position and mapping.
  slam_.AddFrame(cloudS, laserIdMapping_);

  // Get the computed world transform so far
  Transform worldTransform = slam_.GetWorldTransform();
  std::vector<double> poseCovar = slam_.GetTransformCovariance();

  // publish TF, pose and covariance
  publishTfPoseCovar(cloudV.header, worldTransform, poseCovar);
}

//------------------------------------------------------------------------------
LidarSlamNode::CloudS::Ptr LidarSlamNode::convertToSlamPointCloud(const CloudV& cloudV)
{
  CloudS::Ptr cloudS(new CloudS);
  cloudS->resize(cloudV.size());
  cloudS->header = cloudV.header;
  for(unsigned int i = 0; i < cloudV.size(); i++)
  {
    const PointV& velodynePoint = cloudV[i];
    PointS slamPoint;
    slamPoint.x = velodynePoint.x;
    slamPoint.y = velodynePoint.y;
    slamPoint.z = velodynePoint.z;
    slamPoint.intensity = velodynePoint.intensity;
    slamPoint.laserId = velodynePoint.ring;
    slamPoint.time =  std::atan2(velodynePoint.y, velodynePoint.x);
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
  tfMsg.header.frame_id = "slam_init";  // TODO : get frame_id from rosparam
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
  tfBroadcaster_.sendTransform(tfMsg);

  // publish pose with covariance
  geometry_msgs::PoseWithCovarianceStamped poseCovarMsg;
  poseCovarMsg.header = tfMsg.header;
  poseCovarMsg.pose.pose.orientation = tfMsg.transform.rotation;
  poseCovarMsg.pose.pose.position.x = worldTransform.x;
  poseCovarMsg.pose.pose.position.y = worldTransform.y;
  poseCovarMsg.pose.pose.position.z = worldTransform.z;
  std::copy(poseCovar.begin(), poseCovar.end(), poseCovarMsg.pose.covariance.begin());
  poseCovarPub_.publish(poseCovarMsg);
}