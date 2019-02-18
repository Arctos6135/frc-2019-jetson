FROM ros:kinetic-perception-xenial

ENV DEBIAN_FRONTEND noninteractive

# prerequisites

RUN apt-get update && \
    apt-get install -y \
        sudo \
        apt-utils \
        ros-kinetic-web-video-server && \
    apt-get upgrade -y

# User to build software

ARG username=user
ENV user_name=$username

# Add user with sudo

RUN useradd -G sudo -m -s /bin/bash ${user_name}
RUN echo "${user_name} ALL=(ALL) NOPASSWD:ALL" >> /etc/sudoers

# get source

ARG projecttarball=./frc-2019-jetson-src.tar.gz
ADD $projecttarball /home/${user_name}/frc-2019-jetson
RUN chown -R ${user_name}:${user_name} /home/${user_name}/frc-2019-jetson

# login as build user

USER $user_name
WORKDIR /home/${user_name}/frc-2019-jetson
SHELL ["/bin/bash", "-c"]

# build package

RUN wstool update -t src
RUN source /opt/ros/kinetic/setup.bash && catkin_make
RUN tar -cvzf frc-2019-jetson-build.tar.gz .

# get build

CMD cat frc-2019-jetson-build.tar.gz

ENV DEBIAN_FRONTEND teletype
