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
tbd

### [PALoc](https://github.com/JokerJohn/PALoc)
tbd  

### [hdl_localization](https://github.com/koide3/hdl_localization)
tbd  

### [BM-Loc](https://github.com/YixFeng/Block-Map-Based-Localization)
tbd


## Troubleshooting
1. If you have problems with the attachment to a display, execute `xhost +` before starting the docker.

## Contribute
Feel free to open an issue and a PR.  
If you find a problem, feel free to solve it :)