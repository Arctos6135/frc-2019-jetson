catkin init
cd src
wstool update
#rosinstall_generator usb_cam --rosdistro kinetic | wstool merge -
#wstool update
rosdep install --from-paths . --ignore-src
source /opt/ros/kinetic/setup.bash

