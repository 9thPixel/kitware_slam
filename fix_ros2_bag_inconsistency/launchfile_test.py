
import os
import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument, Shutdown, ExecuteProcess, GroupAction, SetEnvironmentVariable
from launch.conditions import IfCondition, UnlessCondition, LaunchConfigurationEquals, LaunchConfigurationNotEquals
from launch.substitutions import LaunchConfiguration

def generate_launch_description():

  ###############
  ## ARGUMENTS ##
  ###############
  ld = LaunchDescription([
    # General args
    DeclareLaunchArgument("test_data",       default_value="",      description="Path to the test data"),
    DeclareLaunchArgument("res_path",        default_value="/tmp",  description="Path to the folder where to store the results"),
    DeclareLaunchArgument("ref_path",        default_value="",      description="Path to the reference data folder for results comparison"),
    DeclareLaunchArgument("wait_init",       default_value="1",     description="Wait for test node initialization to replay data"),
    DeclareLaunchArgument("outdoor",         default_value="true",  description="Decide which set of parameters to use"),
    DeclareLaunchArgument("use_sim_time",    default_value="true",  description="Sim Time, used when replaying rosbag files"),
    DeclareLaunchArgument("velodyne_driver", default_value="false", description="If true, start Velodyne driver."),
    DeclareLaunchArgument("verbose",         default_value="false", description="If true, print the difference with reference during the comparison"),
    DeclareLaunchArgument("domain_id", default_value="0", description="Set to different value to avoid interference when several computers running ROS2 on the same network."),
    SetEnvironmentVariable(name='ROS_DOMAIN_ID',value=LaunchConfiguration('domain_id')),

    # Velodyne arguments
    DeclareLaunchArgument("model", default_value="VLP16", description="Model of Velodyne Lidar, choices are : VLP16 / 32C / VLS128"),
    DeclareLaunchArgument("device_ip", default_value=""),
    DeclareLaunchArgument("rpm",  default_value="600.0"),
    DeclareLaunchArgument("port", default_value="2368"),
    DeclareLaunchArgument("pcap", default_value=""),
  ])

  # Play bag
  # If comparison is required, end process when the reference is finished
  # If comparison is not required (simple evaluation), end process when the bag ends
  rosbag_action = ExecuteProcess(
    name='exec_rosbag',
    cmd=['ros2', 'bag', 'play', LaunchConfiguration("test_data"), '--clock', '-d', LaunchConfiguration("wait_init")],
    output= 'screen',
    log_cmd= True,
    on_exit=GroupAction([Shutdown()],
                        condition=LaunchConfigurationEquals('ref_path', ''))
  )

  ld.add_action(rosbag_action)

  return (ld)
