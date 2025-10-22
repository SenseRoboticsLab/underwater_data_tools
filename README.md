# Underwater Data Tools

This repository contains the script for convertting ROS bag to RAW data format of [Tank Dataset](https://senseroboticslab.github.io/underwater-tank-dataset/).

## Prerequisites

- Docker
- Docker Compose

## Setup

The project uses Docker to provide a consistent ROS Noetic environment with all necessary dependencies.

### 1. Build the Docker Image

```bash
docker compose build
```

This will:
- Set up a ROS Noetic environment
- Clone and build the waterlinked_a50_ros_driver package
- Install all Python dependencies (OpenCV, NumPy, etc.)
- Configure user permissions to match your host system

### 2. Start the Container

```bash
docker compose up -d
```

## Using bag2raw.py

The `bag2raw.py` script extracts data from ROS bag files and converts them into images (PNG) and CSV files.

### What it extracts:

- **Images**: 
  - Left camera images (`/camera/left/image_dehazed/compressed`) → `IMG_L/`
  - Right camera images (`/camera/right/image_dehazed/compressed`) → `IMG_R/`
  
- **CSV Files**:
  - `gt.csv` - Ground truth pose from AprilTag SLAM (`/apriltag_slam/GT`)
  - `gt_full.csv` - Full ground truth pose (`/apriltag_slam/GT_full`)
  - `aqua_slam_pose.csv` - AQUA SLAM pose estimates (`/aqua_slam/pose`)
  - `depth.csv` - Depth measurements (`/depth/data`)
  - `imu.csv` - IMU data (angular velocity, linear acceleration) (`/imu/data`)
  - `dvl.csv` - DVL velocity data (`/dvl/data`)

### Usage

Place your `.bag` files under data folder, then run:

```bash
docker exec -it underwater_data_tools bash
python3 ./bag2raw.py ./data
```

If your bag files are outside the workspace, you need to mount them first in compose.yaml file.

### Output Structure

For each `.bag` file processed, a directory with the same name will be created containing:

```
your_bagfile/
├── IMG_L/
│   ├── 1234567890123456789.png
│   ├── 1234567890234567890.png
│   └── ...
├── IMG_R/
│   ├── 1234567890123456789.png
│   ├── 1234567890234567890.png
│   └── ...
├── gt.csv
├── gt_full.csv
├── aqua_slam_pose.csv
├── depth.csv
├── imu.csv
└── dvl.csv
```

**Note**: Image filenames are timestamps in nanoseconds (format: `secs * 1e9 + nsecs`).

### CSV File Formats

#### gt.csv, gt_full.csv, aqua_slam_pose.csv
```
timestamp[ns], pos_x[m], pos_y[m], pos_z[m], orient_qx, orient_qy, orient_qz, orient_qw
```

#### depth.csv
```
timestamp[ns], pos_z[m]
```

#### imu.csv
```
timestamp[ns], ang_vel_x[rad/s], ang_vel_y[rad/s], ang_vel_z[rad/s], lin_acc_x[m/s^2], lin_acc_y[m/s^2], lin_acc_z[m/s^2]
```

#### dvl.csv
```
timestamp[ns], vel_x[m/s], vel_y[m/s], vel_z[m/s], beam1_vel[m/s], beam2_vel[m/s], beam3_vel[m/s], beam4_vel[m/s]
```


## Troubleshooting

### Permission Issues

If you encounter permission issues, ensure the `.env` file exists with your user IDs:

```bash
echo "USER_ID=$(id -u)" > .env
echo "GROUP_ID=$(id -g)" >> .env
```

Then rebuild:
```bash
docker compose build --no-cache
```

### ROS Messages Not Found

If you see errors about missing message types (especially DVL messages), ensure the catkin workspace was built correctly:

```bash
docker compose exec ros_noetic bash -c "source /opt/ros/noetic/setup.bash && source /catkin_ws/devel/setup.bash && rospack list | grep dvl"
```

### Out of Memory

Processing large bag files may require significant memory. You can limit the processing or increase Docker's memory allocation in Docker Desktop settings.

## Dependencies

- ROS Noetic
- Python 3
- OpenCV
- NumPy
- rosbag
- waterlinked_a50_ros_driver (DVL message definitions)

All dependencies are automatically installed in the Docker container.

## LICENSE

This project is licensed under the GPL-3.0 License. See the [LICENSE](LICENSE) file for details.
