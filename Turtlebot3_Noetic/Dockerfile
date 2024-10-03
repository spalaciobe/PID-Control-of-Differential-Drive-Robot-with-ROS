FROM osrf/ros:noetic-desktop-full

RUN mkdir -p /catkin_ws
# WORKDIR /catkin_ws

# Based on: https://emanual.robotis.com/docs/en/platform/turtlebot3/quick-start/
RUN apt update && apt-get install -y ros-noetic-joy ros-noetic-teleop-twist-joy \
  ros-noetic-teleop-twist-keyboard ros-noetic-laser-proc \
  ros-noetic-rgbd-launch ros-noetic-rosserial-arduino \
  ros-noetic-rosserial-python ros-noetic-rosserial-client \
  ros-noetic-rosserial-msgs ros-noetic-amcl ros-noetic-map-server \
  ros-noetic-move-base ros-noetic-urdf ros-noetic-xacro \
  ros-noetic-compressed-image-transport ros-noetic-rqt* ros-noetic-rviz \
  ros-noetic-gmapping ros-noetic-navigation ros-noetic-interactive-markers

RUN apt update && apt-get install -y ros-noetic-dynamixel-sdk ros-noetic-turtlebot3-msgs ros-noetic-turtlebot3

# Added by spalaciob Feb 28 
RUN apt-get update && apt-get install -y git gcc clang clang-tools cmake python3 
RUN apt-get install -y net-tools && apt-get install -y iputils-ping
RUN apt-get update && apt-get install -q -y nano ssh 

# Added by spalaciob Mar 10 
RUN apt-get install -y python3-pip
RUN pip install simple-pid

COPY ./autostart.sh /
RUN chmod +x /autostart.sh

# Added by spalaciob Feb 6 - It sources /opt/ros/noetic/setup.bash and /catkin_ws/devel/setup.bash every time a new shell is launched
RUN echo "source /opt/ros/noetic/setup.bash && source /catkin_ws/devel/setup.bash" >> ~/.bashrc


ENTRYPOINT ["/ros_entrypoint.sh"]
