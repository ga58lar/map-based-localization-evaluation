# Evaluation of map-based LiDAR Localization Frameworks

Everything is hosted in Docker Containers 🐳📦

## Usage

Build: `./docker/build_docker.sh --dockerfile <name>`

Run: `./docker/run_docker.sh --dockerfile <name>`

### Example

```
./docker/build_docker.sh --dockerfile ros1_bridge
./docker/run_docker.sh --dockerfile ros1_bridge
```

## kitti2bag
Example:
```bash
kitti2bag -t 2011_10_03 -r 0027 raw_synced .
```
Your file structure has to look like:  
```
2011_10_03  
|-- calib_cam_to_cam.txt  
|-- calib_imu_to_velo.txt  
|-- calib_velo_to_cam.txt  
|-- 2011_10_03_drive_0027_sync/
```