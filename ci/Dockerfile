FROM gitlab.kitware.com:4567/keu-computervision/slam/ros2_env
LABEL maintainer="Julia Sanchez julia.sanchez@kitware.com"

# Install gitlab CI utilities
RUN apt update
RUN apt install -y jq unzip
RUN apt install ros-humble-rmw-cyclonedds-cpp -y

# Handle testing data
WORKDIR /root
# Dockerfile must be run in a folder containing all data folders used in tests (e.g test1, test2...)
COPY ./test1 ./data/test1
COPY ./test2 ./data/test2

RUN echo 'export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp' >> /ros_entrypoint.sh

ENTRYPOINT ["/ros_entrypoint.sh"]