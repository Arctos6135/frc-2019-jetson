catkin init
cd src
export ROS_DISTRO=kinetic
wstool update
#rosinstall_generator usb_cam --rosdistro kinetic | wstool merge -
#wstool update
rosdep install -y --from-paths .
wstool update
source /opt/ros/kinetic/setup.bash

