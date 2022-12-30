#include <dynamic_reconfigure/server.h>
#include <lidar_slam/LidarSlamConfig.h>
#include "LidarSlamNode.h"
#include <dynamic_reconfigure/server.h>
#include <lidar_slam/LidarSlamConfig.h>

//------------------------------------------------------------------------------

/*!
 * @brief Main node entry point.
 */
int main(int argc, char **argv)
{
  ros::init(argc, argv, "lidar_slam");

  // Create dynamic parameters server
  dynamic_reconfigure::Server<lidar_slam::LidarSlamConfig> server;

  ros::NodeHandle nh;
  ros::NodeHandle priv_nh("~");

  // Create lidar slam node, which subscribes to pointclouds coming from conversion node
  // and to external sensor messages in parallel.
  LidarSlamNode slam(nh, priv_nh, server);

  // Handle callbacks until shut down
  ros::spin();

  return 0;
}