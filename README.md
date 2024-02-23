# use docker

build a ubuntu20 image

```
cd /leofile/code_lab/docker
docker build -t ubuntu20-image .
```

build container
```
bash docker_run.sh --run --image-name ubuntu20-image --container-name ubuntu20-container
```

tips: 
1. 写sourcs.list时，请参考https://mirrors.tuna.tsinghua.edu.cn/help/ubuntu/
2. docker/Dockerfile: apt update && apt install 那一步总会失败，创建container后再手动执行吧
```
sudo apt update
sudo apt install -y ros-noetic-pcl-ros ros-noetic-velodyne-msgs libopencv-dev libgoogle-glog-dev libeigen3-dev libsuitesparse-dev libpcl-dev libyaml-cpp-dev libbtbb-dev libgmock-dev unzip python3-tk python3-pip time git vim clang
```
