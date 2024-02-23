FROM osrf/ros:noetic-desktop-full

COPY sources.list /etc/apt/sources.list

# somehow this line will be failed!
# RUN apt update && apt install -y ros-noetic-pcl-ros ros-noetic-velodyne-msgs libopencv-dev \
# libgoogle-glog-dev libeigen3-dev libsuitesparse-dev libpcl-dev libyaml-cpp-dev libbtbb-dev libgmock-dev \
# unzip python3-tk python3-pip time git

ENV LD_LIBRARY_PATH="${LD_LIBRARY_PATH}:/usr/local/lib"