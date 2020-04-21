#ifndef ROS_TRANSFORM_UTILS_H
#define ROS_TRANSFORM_UTILS_H

#include <geometry_msgs/TransformStamped.h>
#include <geometry_msgs/PoseStamped.h>
#include <tf2_ros/transform_listener.h>
#include <LidarSlam/Transform.h>

//========================== Transform -> ROS msg ==============================

//------------------------------------------------------------------------------
//! Fill a TF msg with a Transform object.
geometry_msgs::Transform TransformToTfMsg(const Transform& transform)
{
  geometry_msgs::Transform tfMsg;
  tfMsg.translation.x = transform.x();
  tfMsg.translation.y = transform.y();
  tfMsg.translation.z = transform.z();
  Eigen::Quaterniond q = transform.GetRotation();
  tfMsg.rotation.x = q.x();
  tfMsg.rotation.y = q.y();
  tfMsg.rotation.z = q.z();
  tfMsg.rotation.w = q.w();
  return tfMsg;
}

//------------------------------------------------------------------------------
//! Fill a Pose msg with a Transform object.
geometry_msgs::Pose TransformToPoseMsg(const Transform& transform)
{
  geometry_msgs::Pose PoseMsg;
  PoseMsg.position.x = transform.x();
  PoseMsg.position.y = transform.y();
  PoseMsg.position.z = transform.z();
  Eigen::Quaterniond q = transform.GetRotation();
  PoseMsg.orientation.x = q.x();
  PoseMsg.orientation.y = q.y();
  PoseMsg.orientation.z = q.z();
  PoseMsg.orientation.w = q.w();
  return PoseMsg;
}

//------------------------------------------------------------------------------
//! Fill a PoseStamped msg with a Transform object.
geometry_msgs::PoseStamped TransformToPoseStampedMsg(const Transform& transform)
{
  geometry_msgs::PoseStamped PoseStampedMsg;
  PoseStampedMsg.header.frame_id = transform.frameid;
  PoseStampedMsg.header.stamp = ros::Time(transform.time);
  PoseStampedMsg.pose.position.x = transform.x();
  PoseStampedMsg.pose.position.y = transform.y();
  PoseStampedMsg.pose.position.z = transform.z();
  Eigen::Quaterniond q = transform.GetRotation();
  PoseStampedMsg.pose.orientation.x = q.x();
  PoseStampedMsg.pose.orientation.y = q.y();
  PoseStampedMsg.pose.orientation.z = q.z();
  PoseStampedMsg.pose.orientation.w = q.w();
  return PoseStampedMsg;
}

//========================== ROS msg -> Transform ==============================

//------------------------------------------------------------------------------
//! Build a Transform object from a Pose msg.
Transform PoseMsgToTransform(const geometry_msgs::Pose& poseMsg, double time = 0., const std::string& frameid = "")
{
  Eigen::Translation3d trans(poseMsg.position.x,
                             poseMsg.position.y,
                             poseMsg.position.z);
  Eigen::Quaterniond rot(poseMsg.orientation.w,
                         poseMsg.orientation.x,
                         poseMsg.orientation.y,
                         poseMsg.orientation.z);
  return Transform(trans, rot, time, frameid);
}

//------------------------------------------------------------------------------
//! Build a Transform object from a PoseStamped msg.
Transform PoseMsgToTransform(const geometry_msgs::PoseStamped& poseStampedMsg)
{
  double time = poseStampedMsg.header.stamp.toSec();
  Eigen::Translation3d trans(poseStampedMsg.pose.position.x,
                             poseStampedMsg.pose.position.y,
                             poseStampedMsg.pose.position.z);
  Eigen::Quaterniond rot(poseStampedMsg.pose.orientation.w,
                         poseStampedMsg.pose.orientation.x,
                         poseStampedMsg.pose.orientation.y,
                         poseStampedMsg.pose.orientation.z);
  return Transform(trans, rot, time, poseStampedMsg.header.frame_id);
}

//========================== ROS TF2 -> Eigen Isometry3d =======================

//------------------------------------------------------------------------------
//! Safely get a transform between 2 frames from TF2 server
bool Tf2LookupTransform(Eigen::Isometry3d& transform,
                        const tf2_ros::Buffer& tfBuffer,
                        const std::string& targetFrame,
                        const std::string& sourceFrame,
                        const ros::Time time = ros::Time(0),
                        const ros::Duration timeout = ros::Duration(0))
{
  geometry_msgs::TransformStamped tfStamped;
  try
  {
    tfStamped = tfBuffer.lookupTransform(targetFrame, sourceFrame, time, timeout);
  }
  catch (tf2::TransformException& ex)
  {
    ROS_ERROR("%s", ex.what());
    return false;
  }
  const geometry_msgs::Transform& t = tfStamped.transform;
  transform = Eigen::Translation3d(t.translation.x, t.translation.y, t.translation.z)
              * Eigen::Quaterniond(t.rotation.w, t.rotation.x, t.rotation.y, t.rotation.z);
  return true;
}

#endif  // ROS_TRANSFORM_UTILS_H