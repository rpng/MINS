FROM ros:noetic-perception-focal
COPY . /catkin_ws/src/MINS
RUN apt update && apt install python3-catkin-tools -y
RUN /bin/bash -c "chmod +x /opt/ros/noetic/setup.sh && source /opt/ros/noetic/setup.sh && catkin build --workspace /catkin_ws"

WORKDIR /catkin_ws
ENTRYPOINT ["/bin/bash"]
