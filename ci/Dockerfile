FROM gitlab.kitware.com:4567/keu-computervision/slam/ros2_env
LABEL maintainer="Julia Sanchez julia.sanchez@kitware.com"

# Handle testing data
WORKDIR /root
# Dockerfile must be run in a folder containing all data folders used in tests (e.g test1, test2...)
COPY ./test1 ./data/test1
COPY ./test2 ./data/test2

ENTRYPOINT ["/ros_entrypoint.sh"]