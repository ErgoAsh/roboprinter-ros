FROM ros:humble

ENV HOME=/home/ergoash/
ENV ROS2_WORKSPACE=roboprinter_ws
ARG ROS_DISTRO=humble

## Basic packages installation
RUN apt-get update && \
    apt-get install -y \
    build-essential \
    curl \
    gnupg \
    lsb-release \
    git \
    apt-utils \
    python3-dev \
    python3-distutils \
    python3-pip \
    unzip \
    sudo \
    wget \
    software-properties-common \
    iputils-ping \
    vim \
    less

RUN apt-get update  && \
    apt-get upgrade -y && \
    apt-get install -y \
    vim \
    iputils-ping \
    ros-${ROS_DISTRO}-gazebo-ros-pkgs \
    ros-${ROS_DISTRO}-xacro \
    python3-rosdep \
    python3-colcon-common-extensions \
    python3-colcon-mixin \
    python3-vcstool

# Update ROS2
RUN rosdep update --rosdistro=$ROS_DISTRO && \
    apt dist-upgrade

RUN mkdir -p $HOME/$ROS2_WORKSPACE

# Install MoveIt2
RUN mkdir -p $HOME/$ROS2_WORKSPACE/moveit2 && \
    git clone \
    -b $ROS_DISTRO \
    https://github.com/ros-planning/moveit2.git \
    $HOME/$ROS2_WORKSPACE/moveit2

# Simulation - Joint State Publisher
RUN mkdir -p $HOME/$ROS2_WORKSPACE/joint_state_publisher && \
    git clone \
    -b ros2 \
    https://github.com/ros/joint_state_publisher \
    $HOME/$ROS2_WORKSPACE/joint_state_publisher

RUN mkdir -p $HOME/$ROS2_WORKSPACE/ros2_control && \
    git clone \
    -b $ROS_DISTRO \
    https://github.com/ros-controls/ros2_control.git \
    $HOME/$ROS2_WORKSPACE/ros2_control

RUN cd $HOME/$ROS2_WORKSPACE && \
    mkdir -p $HOME/$ROS2_WORKSPACE/moveit2_tutorials && \
    git clone https://github.com/ros-planning/moveit2_tutorials.git -b $ROS_DISTRO --depth 1 $HOME/$ROS2_WORKSPACE/moveit2_tutorials
RUN cd $HOME/$ROS2_WORKSPACE && \
    vcs import < $HOME/$ROS2_WORKSPACE/moveit2_tutorials/moveit2_tutorials.repos; exit 0
RUN cd $HOME/$ROS2_WORKSPACE && \
    rosdep update && \
    rosdep install -r \
    --ignore-src \
    --rosdistro $ROS_DISTRO \
    --from-paths . -y

# Build by source, override any packages in /opt/ros/
RUN cd $HOME/$ROS2_WORKSPACE && \
    . /opt/ros/$ROS_DISTRO/setup.sh && \
    colcon build --parallel-workers 1

RUN echo "source $HOME/$ROS2_WORKSPACE/install/setup.bash" >> /root/.bashrc
RUN echo "Container ready to work with" >> /root/.bashrc

CMD [ "/bin/bash", "-c" ]
