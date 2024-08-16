# AWV-MOS
version 2024-08-01

## Introduction
- AWV-MOS is **a novel visibility-based MOS method that utilizes an adaptive window considering the uncertainty of points**.
- AWV-MOS-LIO is **a online LiDAR Moving Object Segmentation system integrated with LiDAR inertial odometry**.
- The dataset used for paper experiments can be downloaded from the NAS server.
- Currently, this package only provide codes for Online MOS operation.
- After the paper accept, codes that provide various functions, such as static map creation, will be uploaded.

## Dependency
- [ROS](http://wiki.ros.org/ROS/Installation) (tested with noetic).
    ```
    sudo apt install ros-noetic-desktop
    ```
- [gtsam](https://gtsam.org/get_started/)
    ```
    sudo add-apt-repository ppa:borglab/gtsam-release-4.0
    sudo apt install libgtsam-dev libgtsam-unstable-dev
    ```
- [TBB](https://github.com/oneapi-src/oneTBB/blob/master/INSTALL.md)
## Install
Use the following commands to download and compile the package.

```
$ cd ~/catkin_ws/src
$ git clone http://git.aimlab.co.kr/seongjun/AWV-MOS-LIO.git
$ cd ..
$ catkin_make -DCMAKE_BUILD_TYPE=Release
```
## Download ROS Bag data
- KITTI Raw dataset ROS Bag files used for paper experiments can be downloaded from [the NAS server](https://gofile.me/6Vvuz/njqrKwtxs) 

## Run the package
1. Setup environment variable:
```
$ cd ~/catkin_ws/
$ source devel/setup.bash
```
2. Run the launch file:
```
$ roslaunch awv_mos run_kitti.launch
```
3. Play existing bag files:
```
$ rosbag play {your_bag.bag}
```
## Evaluation on KITTI Raw dataset
1. build with *USE_EVALUATION_POINT_TYPE* macro option
```
$ catkin_make -DUSE_EVALUATION_POINT_TYPE=ON
```
2. Recored the MOS result:
```
$ rosbag record /awv_mos/segmented_new_scan_all
```
3. Run the launch file & Play existing bag files:
```
$ roslaunch awv_mos run_kitti.launch
$ rosbag play {your_bag.bag}
```
4. Setup *bag_record_path* and *bag_file_name* from evaluation.launch:
```
    <arg name="bag_record_path" default="/path/to/recored_bag_file_folder/"/>
    <arg name="bag_file_name" default="bag_file_name.bag"/>
```
5. Run the launch file:
```
$ roslaunch awv_mos evaluation.launch
```
## TODO
- Add static map construction function.
- Add tools for generating required bag files from dataset such as kitti raw, urbanloco.