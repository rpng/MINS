# Use the official ROS Noetic base image
FROM ros:noetic

# Install necessary dependencies
RUN apt-get update && apt-get install -y \
    build-essential \
    cmake \
    git \
    python3-rosdep \
    python3-catkin-tools \
    libboost-all-dev \
    libeigen3-dev \
    ros-noetic-cv-bridge \
    ros-noetic-image-transport \
    ros-noetic-tf2-ros \
    ros-noetic-tf2-eigen \
    ros-noetic-rviz \
    ros-noetic-image-geometry \
    ros-noetic-pcl-ros \
    && rm -rf /var/lib/apt/lists/*

# Set up the catkin workspace
RUN mkdir -p /catkin_ws/src
WORKDIR /catkin_ws/src

# Clone the MINS repository
RUN git clone https://github.com/rpng/mins.git

# Initialize rosdep
RUN if [ ! -f /etc/ros/rosdep/sources.list.d/20-default.list ]; then \
        rosdep init; \
    fi && \
    rosdep update

# Install dependencies using rosdep
RUN cd /catkin_ws && rosdep install --from-paths src --ignore-src -r -y

# Build the catkin workspace with catkin_make_isolated
RUN /bin/bash -c "source /opt/ros/noetic/setup.bash && cd /catkin_ws && catkin_make_isolated -j1"

# Source the setup file and set the entrypoint
RUN echo "source /catkin_ws/devel_isolated/setup.bash" >> ~/.bashrc
ENTRYPOINT ["/bin/bash"]

