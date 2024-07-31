# ROS2 Dockerfile

## Build docker images

Kitware provides 2 dockerfiles :
* [Dockerfile.env](Dockerfile.env) : it sets the environment (dependencies, variables, etc.) for development tests.
* [Dockerfile.slam](Dockerfile.slam) : it calls the image built from the previous Dockerfile and adds the SLAM package.

You can build the environment image with the following command in the docker folder:
```
docker build -t ros2_env -f Dockerfile.env .
```

Then you can build the full package image with the following command (still from the docker folder):
```
docker build -t ros2_slam -f Dockerfile.slam .
```

## ROS2 communication with containers

This section explains how to perform ROS2 communication between docker containers.
In this example, we run the slam container, run another container to send the data and we establish a communication between both.

### 1. Create network
sudo docker network create ros_net

### 2. Launch slam with display options

The following steps allow to launch the SLAM ROS nodes expecting data in a first terminal.

#### 2.1 Run slam container

To be able to communicate with the slam container, you need to run the image with a network setting and eventually the domain ID.
```
sudo docker run -it \
  --net ros_net \
  -e ROS_DOMAIN_ID=5 \
  --rm \
  ros2_slam bash
```

To enable graphic display on your host (to use rviz notably), share the tmp driver folder, the DISPLAY environment variable and the device between the container and the host.
The command should become :

```
sudo docker run -it \
  --net ros_net \
  -e ROS_DOMAIN_ID=5 \
  --rm \
  -e "DISPLAY=$DISPLAY" \
  --volume=/tmp/.X11-unix:/tmp/.X11-unix \
  --device=/dev/dri:/dev/dri \
  ros2_slam bash
```

#### 2.2 Launch slam pipeline

```
ros2 launch lidar_slam slam_velodyne.py domain_id:=5
```

**NOTES**:
* Change the launch file relatively to your data.
* You can add the command to the [slam_entrypoint.bash](slam_entrypoint.bash) file and rebuild the image using its [Dockerfile](Dockerfile.slam) if you want the command to be executed at your container start.

### 3. Launch data send container

The following steps allow to launch a new container running ros2 bag node which replays data in a second terminal.

#### 3.1 Download ros humble image
```
docker pull osrf/ros:humble-desktop
```

#### 3.2 Run replay container and mount volume with data
```
sudo docker run -it \
  --rm \
  --net ros_net \
  -e ROS_MASTER_URI=http://roscore:11311 \
  -e ROS_DOMAIN_ID=5 \
  -v=/path/on/host/to/bag_folder:/data \
  --name replay \
  osrf/ros:humble-desktop
```

##### 3.3 Launch replay command
```
ros2 bag play /data/<bag_name> -d1 --clock
```

If you don't want to mount a volume, you can download the data directly in the container from an external source.

## Store image on gitlab registry

This section is for Kitware developers.

To create a container image on gitlab, you first need to build the image locally and then you can push it on the remote repository.

Example to create an image in the Slam repository under the name slam:my_image
```
docker build . -t my_image
docker login gitlab.kitware.com:4567
docker push gitlab.kitware.com:4567/keu-computervision/slam:my_image
```