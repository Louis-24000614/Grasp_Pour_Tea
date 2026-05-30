# camera_vision

ROS2 depth camera package for publishing:

- RGB image (`vision/raw`)
- Depth image (`vision/depth`)
- Point cloud (`vision/cloud`)

## Dependencies

- ROS2 Humble
- `rclcpp`
- `sensor_msgs`
- Intel RealSense SDK development files (`librealsense2-dev`)

Install RealSense SDK on Ubuntu:

```bash
sudo apt update
sudo apt install librealsense2-dev
```

## Build

```bash
cd /home/nvidia/Capture
colcon build --packages-select camera_vision
source install/setup.bash
```

## Run

```bash
ros2 launch camera_vision dep_cam.launch.py
```

## Parameters

Default parameters are in `config/dep_cam.yaml`.
