# frc-2019-jetson
[![Build Status](https://travis-ci.com/Arctos6135/frc-2019-jetson.svg?branch=master)](https://travis-ci.com/Arctos6135/frc-2019-jetson)<br>
Team Arctos 6315's 2019 Jetson TX1 code. Made with help from [@mincrmatt12](https://github.com/mincrmatt12).

<img src="https://upload.wikimedia.org/wikipedia/en/thumb/c/cf/Flag_of_Canada.svg/1280px-Flag_of_Canada.svg.png" alt="CANADA" height="50px"/>&nbsp;&nbsp;&nbsp;<img src="https://avatars0.githubusercontent.com/u/16629663?s=200&v=4" alt="Arctos" height="50px"/>&nbsp;&nbsp;&nbsp;<img src="https://upload.wikimedia.org/wikipedia/commons/thumb/b/bb/Ros_logo.svg/2000px-Ros_logo.svg.png" alt="ROS" height="50px"/>&nbsp;&nbsp;&nbsp;<img src="https://upload.wikimedia.org/wikipedia/commons/thumb/3/32/OpenCV_Logo_with_text_svg_version.svg/1200px-OpenCV_Logo_with_text_svg_version.svg.png" alt="OpenCV" height="50px"/>&nbsp;&nbsp;&nbsp;<img src="https://www.firstinspires.org/sites/default/files/uploads/resource_library/brand/FIRST_Vertical_RGB.jpg" alt="FIRST" height="50"/>

Thank you to our generous sponsors:<br/>
<img src="http://connecttech.com/logo.jpg" alt="Connect Tech Inc." height="200px"/>
<img src="https://user-images.githubusercontent.com/32781310/52970668-acd64780-3382-11e9-857f-85b829690e0c.png" alt="Scotia McLeod" height="200px"/>
<img src="https://kissmybutton.gr/wp-content/uploads/2017/09/ryver.png" alt="Ryver Inc." height="200px"/>
<img src="https://user-images.githubusercontent.com/32781310/52224389-eaf94480-2875-11e9-82ba-78ec58cd20cd.png" alt="The Maker Bean Cafe" height="200px"/>
<img src="https://brafasco.com/media/wysiwyg/HDS_construction_industrial_BF_4C_pos.png" alt="HD Supply Brafasco" height="200px"/>
<img src="https://encrypted-tbn0.gstatic.com/images?q=tbn:ANd9GcRqnEGnLesUirrtMQfhxLGUTZn2xkVWpbROlvmABI2Nk6HzhD1w" alt="Arbour Memorial" height="200px"/>
<img src="https://developer.nordicsemi.com/.webresources/NordicS.jpg" alt="Nordic Semiconductors" height="170px"/>
<img src="https://dynamicmedia.zuza.com/zz/m/original_/3/a/3aae60b3-ff18-4be5-b2b1-e244943a85fb/TDSB_Gallery.png" alt="Toronto District School Board" height="200px"/>

This code runs on an NVIDIA Jetson TX1 and communicates with the roboRIO via NetworkTables.
It is capable of detecting the relative position, direction and orientation of the target to within 2% error and has an MJPEG camera stream on port 1180.
The code is written in C++ with ROS.

### Prerequisites

ROS Kinetic and the package 'ros-kinetic-web-video-server'. Note that unfortunately ROS Kinetic is only avaiable on Ubuntu 16.04 (Xenial).

### To Build
#### Setup
* `bash ./install_ros.sh` if ROS is not installed
* `bash ./build_setup.sh`
* `source /opt/ros/kinetic/setup.bash`
### Build
* `catkin build`

### To Run
* `source devel/setup.bash`
* `roslaunch bot bot.launch`

### NetworkTables

The program communicates with the roboRIO via the table `roborio-jetson`.

| Table Entry | Modified By | Type | Purpose |
| ----------- | ----------- | ---- | ------- |
| `vision-online` | Jetson | boolean | Used to indicate that vision initialization is complete. |
| `vision-enable` | roboRIO | boolean | If set to true, vision mode will be enabled. The camera exposure will be lowered. |
| `enable-success` | Jetson | boolean | Indicates whether the vision enable operation was successful. |
| `shutdown` | roboRIO | boolean | If set to true, the Jetson will execute `shutdown -P now` and gracefully shut down. |
| `horizontal-angle` | Jetson | double (degrees) | The horizontal direction of the centre of the target relative to the centre of the camera. A negative angle means that the target is to the left and vice versa. |
| `angle-offset` | Jetson | double (degrees) | The horizontal angle difference between the centre of the camera and the hatch placement angle. |
| `x-offset` | Jetson | double (inches) | The left-right distance offset of the target. |
| `y-offset` | Jetson | double (inches) | The forwards-backwards distance offset of the target. |
| `restart-server` | roboRIO | boolean | If set to true, the camera server will be restarted. |
