import leo_common.leo_common.csrc.common as cb
import leo_common.leo_common.utility.utility as util
# if you didn't run `pip install leo_common`, import lib by above two lines.
# In addition, these two lines will become invalid after installation

# import leo_common.csrc.common as cb
# import leo_common.utility.utility as util
print(cb.plus(1, 2))
cb.create_img()
p = cb.Pet()
print(p.getName())
p.setName("dog")
print(p.getName())
data_list = util.load_from_file(
    "/home/cxn/leofile/code_lab/slam_lab/data/imu_gnss_odom_10.txt")
