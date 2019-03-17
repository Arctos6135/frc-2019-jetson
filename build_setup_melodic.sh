catkin init
cd src
export ROS_DISTRO=melodic
sudo wstool update
#rosinstall_generator usb_cam --rosdistro melodic | wstool merge -
#wstool update
sudo apt install ros-melodic-camera-info-manager ros-melodic-web-video-server
rosdep install -y --from-paths .
sudo wstool update
source /opt/ros/melodic/setup.bash

