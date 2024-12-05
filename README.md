# Evaluation of map-based LiDAR Localization Frameworks

Everything is hosted in Docker Containers 🐳📦

*"Diese Dokumentation ist schäbig"* &ensp; [ich, 2024]


## Usage

Build: `./docker/build_docker.sh --dockerfile <name>`

Run: `./docker/run_docker.sh --dockerfile <name>`

### Example

```
./docker/build_docker.sh --dockerfile ros1_bridge
./docker/run_docker.sh --dockerfile ros1_bridge
```

## Tools
### kitti2bag
Convert sequences of the KITTI dataset to a rosbag.  
Example:
```bash
kitti2bag -t 2011_10_03 -r 0027 raw_synced .
```
Your file structure has to look like:  
```
2011_10_03/  
|-- calib_cam_to_cam.txt  
|-- calib_imu_to_velo.txt  
|-- calib_velo_to_cam.txt  
|-- 2011_10_03_drive_0027_sync/
```

### ros1_bridge
Bridge topics between ROS1 and ROS2.  
  
1. Start a terminal with `roscore`
2. Start a second terminal  
    a. source ROS1  
    b. source ROS2  
    c. `export ROS_MASTER_URI=http://localhost:11311`  
    d. `ros2 run ros1_bridge dynamic_bridge`

### bag2pcd
Convert livox-msgs to pointcloud-msgs and export pointcloud-msgs to pcd files.

Conversion:
1. New Terminal: `roslaunch livox_repub livox_repub.launch`
2. New Terminal: `rosbag play /path/to/bag`
3. New Terminal: `rosbag record -a`

Extraction:
1. `rosrun pcl_ros bag_to_pcd <input_file.bag> <topic> <output_directory>`

### rosbags-convert
*NO DOCKER*
  
Convert a ROS1 bag to a ROS2 bag.
1. Create a virtual environment (tested with python=3.10)
2. `pip3 install 'rosbags>=0.9.11'`
3. `rosbags-convert --src /path/to/bag.bag --dst /path/to/output/folder/name`
  

## Localization
### [fast_lio_localization](https://github.com/HViktorTsoi/FAST_LIO_LOCALIZATION)
1. New Terminal: `roslaunch fast_lio_localization localization_velodyne.launch map:=/datasets/map.pcd`
2. New Terminal: `rosbag play /datasets/kitti_2011_10_03_drive_0027_synced.bag`
3. (optional) You can also specify an initial guess, if you are not starting at the map origin.  
    New Terminal: `rosrun fast_lio_localization publish_initial_pose.py x x z yaw pitch roll`


### [liloc](https://github.com/Yixin-F/LiLoc)
Switch mapping or map-based localization with mode "lio" or "relo" in the config.
1. Resolve an issue: - src/imageProjection.cpp → remove lines 417-420 (if is_dense statement)
2. New Terminal: `roscore`
3. New Terminal: `roslaunch liloc run_lio_sam_default.launch`
4. New Terminal: `rviz` with specific config (launch/include/config/rviz.rviz)
5. New Terminal: `rosbag play /datasets/kitti_2011_10_03_drive_0027_synced.bag`


### [lidar_localization_ros2](https://github.com/rsasaki0109/lidar_localization_ros2)
1. New Terminal: `ros2 launch lidar_localization_ros2 lidar_localization.launch.py `
2. New Terminal: `rviz2 -d src/lidar_localization_ros2/rviz/localization.rviz`
3. New Terminal: `ros2 bag play /datasets/KITTI/kitti_2011_10_03_drive_0027_synced/ --topics /kitti/velo/pointcloud --remap /kitti/velo/pointcloud:=/velodyne_points`


### [DLL](https://github.com/robotics-upo/dll/tree/Humble)
Needs a map in .bt (binary octomap format), which does not have negative entries.
1. New Terminal: `ros2 launch dll kitti_launch.xml`
2. New Terminal: `ros2 bag play /datasets/kitti/kitti_2011_10_03_drive_0027_synced/`


### [hdl_localization](https://github.com/koide3/hdl_localization)
1. New Terminal: `roslaunch hdl_localization hdl_localization.launch`
2. New Terminal: `rviz` (config: /dev_ws/src/hdl_localization/rviz/)
3. New Terminal: `rosbag play /datasets/kitti/kitti_2011_10_03_drive_0027_synced.bag`


### [BM-Loc](https://github.com/YixFeng/Block-Map-Based-Localization)
Downlaod the example data from the GitHub and switch the data in Bms_for_test/M2DGR_Bms/M2DGR/street_01/ with your data.  
If you only have a single map, rename your map to 000.pcd and remove everything besides the 000.pcd and the CentroidCloud.pcd.
1. New Terminal: `roslaunch block_localization run_m2dgr.launch`
2. New Terminal: `rosbag play /datasets/kitti/kitti_2011_10_03_drive_0027_synced.bag`


### [range-mcl](https://github.com/PRBonn/range-mcl)
*not yet tested*

### [LiDAR-Localization-100FPS](https://github.com/ShiPC-AI/LiDAR-Localization-100FPS)
tbd

### [G3Reg](https://github.com/HKUST-Aerial-Robotics/G3Reg)
tbd

### [PALoc](https://github.com/JokerJohn/PALoc)
tbd 

## Troubleshooting
1. If you have problems with the attachment to a display, execute `xhost +` before starting the docker.

## Contribute
Feel free to open an issue and a PR.  
If you find a problem, feel free to solve it :)