FROM gitlab.kitware.com:4567/keu-computervision/slam/ros2_env
LABEL maintainer="Julia Sanchez julia.sanchez@kitware.com"

# Install docker for dind
RUN apt-get update && apt-get install -y \
    ca-certificates \
    curl \
    gnupg \
    lsb-release \
 && mkdir -p /etc/apt/keyrings \
 && curl -fsSL https://download.docker.com/linux/ubuntu/gpg | gpg --dearmor -o /etc/apt/keyrings/docker.gpg \
 && echo \
    "deb [arch=$(dpkg --print-architecture) signed-by=/etc/apt/keyrings/docker.gpg] https://download.docker.com/linux/ubuntu \
    $(lsb_release -cs) stable" | tee /etc/apt/sources.list.d/docker.list > /dev/null \
 && apt-get update \
 && apt-get install -y docker-ce docker-ce-cli containerd.io docker-buildx-plugin docker-compose-plugin

# Ensure Docker is executable
RUN chmod +x /usr/bin/docker

# Clean up
RUN apt-get clean && rm -rf /var/lib/apt/lists/*

# Handle testing data
WORKDIR /root
# Dockerfile must be run in a folder containing all data folders used in tests (e.g test1, test2...)
COPY ./test1 ./data/test1
COPY ./test2 ./data/test2

ENTRYPOINT ["/ros_entrypoint.sh"]