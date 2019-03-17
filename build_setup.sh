catkin init
cd src
export ROS_DISTRO=kinetic
wstool update
sudo apt install ros-kinetic-camera-info-manager ros-kinetic-web-video-server
rosdep install -y --from-paths .
wstool update
sudo chown -R "$USER":"$USER"
source /opt/ros/kinetic/setup.bash

