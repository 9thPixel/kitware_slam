#include "LidarSlamNode.h"

//------------------------------------------------------------------------------
/*!
 * @brief Main node entry point.
 */
int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);

  // Create lidar slam node, which subscribes to pointclouds coming from conversion node
  // and to external sensor messages in parallel.
  std::shared_ptr<LidarSlamNode> slam = std::make_shared<LidarSlamNode>("lidar_slam");

  // Handle callbacks until shut down
  rclcpp::spin(slam);
  rclcpp::shutdown();

  return 0;
}