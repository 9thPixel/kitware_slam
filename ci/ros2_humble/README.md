
# Connect ROS2 nodes in Docker :

#### Create network
docker network create ros_net

### Launch Slam node with display options

#### Create Slam container
docker run -it \
  --net ros_net \
  --volume=/tmp/.X11-unix:/tmp/.X11-unix \
  --device=/dev/dri:/dev/dri \
  -e "DISPLAY=$DISPLAY" \
  --name slam_cont \
  ros_humble


#### Launch slam
cmake -E make_directory colcon_ws && cd colcon_ws && \
git clone https://gitlab.kitware.com/keu-computervision/slam.git src/slam --recursive -b feat/ROS2
colcon build --base-paths src/slam/ros2_wrapping --cmake-args -DCMAKE_BUILD_TYPE=Release && \
source install/setup.bash \
ros2 launch lidar_slam slam_velodyne.py


### Launch data

#### Create rosbag container

docker run -it \
  --net ros_net \
  -e ROS_MASTER_URI=http://roscore:11311 \
  --name rosbag_cont \
  ros_humble

##### In an other terminal
docker cp "/path/to/bag" rosbag_cont:/root

##### In the container
ros2 bag play bag -d1 --clock