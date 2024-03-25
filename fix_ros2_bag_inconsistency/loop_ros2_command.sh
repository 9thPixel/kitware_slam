#!/bin/zsh

# source ros for zsh
source /opt/ros/humble/setup.zsh

#! Carefull, your command needs to point to a valid path toward a ros2 bag

# Variables
command="ros2 bag play car_loop --clock -d1"
# command="ros2 launch launchfile_test.py test_data:=car_loop"
file="timestamp.txt"

# Reset file
echo -n > $file

# Test first message
nb_it=30
for i in `seq 1 $nb_it`
do
  # Change ROS_DOMAIN_ID to a value between 1 and 100
  # export ROS_DOMAIN_ID=$((i % 100 + 1))

  # Execute the topic echo command
  echo "command $i / $nb_it"
  ros2 topic echo velodyne_points sensor_msgs/msg/PointCloud2 --once --field header --csv >> $file &
  pid1=$!

  # Execute the ros2 bag play command
  eval $command >/dev/null 2>&1 &;

  # End the bag after 2seconds (enought to publish the first message)
  pid2=$!
  sleep 2s;
  kill $pid2

  # Be sure to finish the processes of the current iteration
  wait $pid1
  killall ros2

done

# Eval results using python program
python3 eval_first_timestamp.py $file 