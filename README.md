
# mecanum_ros2_yolo

This represents the final project for my thesis. It features 4WD Mecanum wheels controlled via Raspberry Pi Pico (micro-ROS) with custom kinematics/odometry. ROS2 Humble on Raspberry Pi 4B integrates RPLiDAR A1 (SLAM via slam_toolbox) and Pi Camera Module 3 with custom YOLOv8 detection. Includes from-scratch implementations of A path planning and Pure Pursuit path tracking for full autonomy.


## 🙏 Acknowledgements
This project builds upon the excellent work of Jon Durrant and his “DDD Diff Drive Robot” series for Raspberry Pi Pico:

 - [**GitHub Repository**](https://github.com/jondurrant/DDD-Exp)
 - [**YouTube Playlist**](https://www.youtube.com/playlist?list=PLspDyukWAtRXbZAXrNu1SEL6Dzk7cjKYf)

I adapted and extended Jon’s micro-ROS motor-control code to support a 4-wheel Mecanum configuration, redesigned the kinematics/odometry, and integrated a full ROS 2 navigation stack for omnidirectional autonomy.
## 🔧 Hardware

- **Chassis & Drive**  
  - 4× Mecanum wheels
  - 4x DC Motors  
  - 2x Motor driver boards (**TB6612FNG**)  
- **Boards**  
  - **Raspberry Pi Pico** (motor control via micro-ROS)  
  - **Raspberry Pi 4 B** (ROS 2 Humble)  
- **Sensors**  
  - **RPLiDAR A1** 
  - **Raspberry Pi Camera Module 3** 


## 📦 Directory Structure
- **PID_Testing**
    - **firmware/** *Contains the standalone Raspberry Pi Pico firmware for PID loop validation.*
- **ROS_Implementation**
    - **firmware/** *Micro-ROS agent and motor‐control code for the Pi Pico.*
    - **ros2_ws/**
        - **droid1/** 
            - `config/` - YAML files for the joystick controller.
            - `launch/` - Python launch scripts for SLAM, sensing, and navigation.
            - `rviz/` - RViz configuration for visualizing robot state & map.
            - `urdf/` - URDF/XACRO robot description.
        - **tf_pub/** *Custom ROS 2 package that computes and publishes the transform tree between `odom` and `base_link`, based on your Pico’s odometry output.*
        - **yolobot_recognition/** *Autonomous perception & navigation stack: `YOLOv8`‐based object detector node (Pi Camera integration) and `A\*` path planner & `Pure Pursuit` controller launch scripts*





## 🚀 Deployment

### Prerequisites
This project is built and tested on Ubuntu 22.04 LTS with ROS2 Humble LTS.

### Setup and build the worskpace
```bash
cd $HOME
git clone https://github.com/cristianooo1/mecanum_ros2_yolo.git
cd ~/mecanum_ros2_yolo/ros2_ws
colcon build --symlink-install
source install/setup.bash
```
### Launch the microros agent on the Raspberry Pi 4B
```bash
sudo micro-ros-agent serial --dev /dev/ttyACM0 baudrate=115200
```

### Launch the LiDAR module on the Raspberry Pi 4B
```bash
ros2 launch rplidar_ros rplidar_a1_launch.py
```

### Launch the camera module on the Raspberry Pi 4B
```bash
ros2 run v4l2_camera v4l2_camera_node --ros-args -p image_size:="[640,480]" -p camera_frame_id:=camera_link_optical
```

### Launch the slam_toolbox module on the Raspberry Pi 4B
```bash
ros2 launch droid1 online_async_launch.py slam_params_file:=src/droid1/config/mapper_params_online_async.yaml
```

### Launch RVIZ
```bash
cd /src/droid1
ros2 launch droid1 dispOnly.launch.py model:=urdf/AM.urdf.xacro
```

### Launch the TF publisher
```bash
ros2 run tf_pub tf_pub
```

### Launch the real time YOLO inference script
```bash
ros2 run yolobot_recognition yolov8_ros2_pt.py
```

### Launch the script to calculate the 3D position of the detected object
```bash
ros2 run yolobot_recognition detect_object_3D.py
```

### Launch the script to follow the detected object
```bash
ros2 run yolobot_recognition yolov8_follower.py
```

### Launch the script to calculate the trajectory to a target position
```bash
ros2 run yolobot_recognition path_planning.py
```

### Launch the script to follow the calculated trajectory
```bash
ros2 run yolobot_recognition pure_pursuit_omni.py
```

