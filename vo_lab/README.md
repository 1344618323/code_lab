# 方案
写一个libvo_lab，以及一个调用该lib的exe: run_vo

## libvo_lab
### class VO
libvo_lab给外部的调用接口
1. VO::addImage(time, im_left, im_right): 输入一帧图像，输出track pose

### class Frame
普通frame和keyframe都是使用这个class。管理frame的pose与keypoints。其中keypoint通过id与mappoint关联。

### class Map
1. 管理keyframe和mappoint
2. 外部可由`Map::getMappoint(mpid)`获取mappoint obj
