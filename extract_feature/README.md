## 小秋SLAM实战教程

python /media/q/q/xiaoqiuslamshizhanjiaocheng/rgbd_dataset_freiburg1_desk/associate.py /media/q/q/xiaoqiuslamshizhanjiaocheng/rgbd_dataset_freiburg1_desk/rgb.txt /media/q/q/xiaoqiuslamshizhanjiaocheng/rgbd_dataset_freiburg1_desk/depth.txt > /media/q/q/xiaoqiuslamshizhanjiaocheng/rgbd_dataset_freiburg1_desk/associations.txt

./orb_slam2_extract_feature /media/q/q/xiaoqiuslamshizhanjiaocheng/rgbd_dataset_freiburg1_desk/TUM1.yaml /media/q/q/xiaoqiuslamshizhanjiaocheng/rgbd_dataset_freiburg1_desk /media/q/q/xiaoqiuslamshizhanjiaocheng/rgbd_dataset_freiburg1_desk/associations.txt



python /media/q/q/xiaoqiuslamshizhanjiaocheng/rgbd_dataset_freiburg1_rpy/associate.py /media/q/q/xiaoqiuslamshizhanjiaocheng/rgbd_dataset_freiburg1_rpy/rgb.txt /media/q/q/xiaoqiuslamshizhanjiaocheng/rgbd_dataset_freiburg1_rpy/depth.txt > /media/q/q/xiaoqiuslamshizhanjiaocheng/rgbd_dataset_freiburg1_rpy/associations.txt


./orb_slam2_extract_feature /media/q/q/xiaoqiuslamshizhanjiaocheng/rgbd_dataset_freiburg1_rpy/TUM1.yaml /media/q/q/xiaoqiuslamshizhanjiaocheng/rgbd_dataset_freiburg1_rpy /media/q/q/xiaoqiuslamshizhanjiaocheng/rgbd_dataset_freiburg1_rpy/associations.txt

