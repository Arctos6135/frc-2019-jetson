catkin init
cd src
export ROS_DISTRO=melodic
wstool update
sudo apt install ros-melodic-camera-info-manager ros-melodic-web-video-server
rosdep install -y --from-paths .
wstool update
sudo chown -R "$USER":"$USER"
source /opt/ros/melodic/setup.bash

