---
sidebar_position: 3
title: Sensor Simulation in Gazebo
---

# Sensor Simulation in Gazebo

## Overview of Sensor Simulation

Sensor simulation is crucial for robotics development as it allows you to test perception algorithms, sensor fusion, and navigation systems without physical hardware. Gazebo provides realistic simulation of various sensors with accurate noise models and physics-based rendering.

## Types of Sensors in Gazebo

### Camera Sensors

Gazebo can simulate various types of cameras:

#### RGB Camera
```xml
<sensor name="camera" type="camera">
  <camera name="head">
    <horizontal_fov>1.047</horizontal_fov>
    <image>
      <width>640</width>
      <height>480</height>
      <format>R8G8B8</format>
    </image>
    <clip>
      <near>0.1</near>
      <far>100</far>
    </clip>
  </camera>
  <always_on>1</always_on>
  <update_rate>30</update_rate>
  <visualize>true</visualize>
</sensor>
```

#### Depth Camera
```xml
<sensor name="depth_camera" type="depth">
  <camera name="head">
    <horizontal_fov>1.047</horizontal_fov>
    <image>
      <width>640</width>
      <height>480</height>
      <format>R8G8B8</format>
    </image>
    <clip>
      <near>0.1</near>
      <far>10</far>
    </clip>
    <depth_camera>
      <output>depths</output>
    </depth_camera>
  </camera>
  <always_on>1</always_on>
  <update_rate>30</update_rate>
  <visualize>true</visualize>
</sensor>
```

### Range Sensors

#### LiDAR (2D Laser Scanner)
```xml
<sensor name="laser" type="ray">
  <ray>
    <scan>
      <horizontal>
        <samples>720</samples>
        <resolution>1</resolution>
        <min_angle>-1.570796</min_angle>
        <max_angle>1.570796</max_angle>
      </horizontal>
    </scan>
    <range>
      <min>0.10</min>
      <max>30.0</max>
      <resolution>0.01</resolution>
    </range>
  </ray>
  <plugin name="laser_control" filename="libgazebo_ros_laser.so">
    <topic_name>scan</topic_name>
    <frame_name>laser_frame</frame_name>
  </plugin>
  <always_on>1</always_on>
  <update_rate>30</update_rate>
  <visualize>true</visualize>
</sensor>
```

#### 3D LiDAR
```xml
<sensor name="velodyne" type="ray">
  <ray>
    <scan>
      <horizontal>
        <samples>1800</samples>
        <resolution>1</resolution>
        <min_angle>-3.14159</min_angle>
        <max_angle>3.14159</max_angle>
      </horizontal>
      <vertical>
        <samples>32</samples>
        <resolution>1</resolution>
        <min_angle>-0.436332</min_angle>
        <max_angle>0.436332</max_angle>
      </vertical>
    </scan>
    <range>
      <min>0.1</min>
      <max>100</max>
      <resolution>0.01</resolution>
    </range>
  </ray>
  <always_on>1</always_on>
  <update_rate>10</update_rate>
  <visualize>true</visualize>
</sensor>
```

### Inertial Sensors

#### IMU (Inertial Measurement Unit)
```xml
<sensor name="imu_sensor" type="imu">
  <always_on>true</always_on>
  <update_rate>100</update_rate>
  <visualize>false</visualize>
  <imu>
    <angular_velocity>
      <x>
        <noise type="gaussian">
          <mean>0.0</mean>
          <stddev>2e-4</stddev>
        </noise>
      </x>
      <y>
        <noise type="gaussian">
          <mean>0.0</mean>
          <stddev>2e-4</stddev>
        </noise>
      </y>
      <z>
        <noise type="gaussian">
          <mean>0.0</mean>
          <stddev>2e-4</stddev>
        </noise>
      </z>
    </angular_velocity>
    <linear_acceleration>
      <x>
        <noise type="gaussian">
          <mean>0.0</mean>
          <stddev>1.7e-2</stddev>
        </noise>
      </x>
      <y>
        <noise type="gaussian">
          <mean>0.0</mean>
          <stddev>1.7e-2</stddev>
        </noise>
      </y>
      <z>
        <noise type="gaussian">
          <mean>0.0</mean>
          <stddev>1.7e-2</stddev>
        </noise>
      </z>
    </linear_acceleration>
  </imu>
</sensor>
```

## Noise Models

Real sensors have noise, and Gazebo simulates this with various noise models:

```xml
<sensor name="noisy_camera" type="camera">
  <camera name="head">
    <!-- Camera configuration -->
  </camera>
  <plugin name="camera_controller" filename="libgazebo_ros_camera.so">
    <baseline>0.2</baseline>
    <distortion_k1>0.0</distortion_k1>
    <distortion_k2>0.0</distortion_k2>
    <distortion_k3>0.0</distortion_k3>
    <distortion_t1>0.0</distortion_t1>
    <distortion_t2>0.0</distortion_t2>
    <always_on>true</always_on>
    <update_rate>30.0</update_rate>
    <camera_name>my_camera</camera_name>
    <image_topic_name>image_raw</image_topic_name>
    <camera_info_topic_name>camera_info</camera_info_topic_name>
    <frame_name>camera_frame</frame_name>
    <hack_baseline>0.07</hack_baseline>
    <distortion_k1>0.0</distortion_k1>
    <distortion_k2>0.0</distortion_k2>
    <distortion_k3>0.0</distortion_k3>
    <distortion_t1>0.0</distortion_t1>
    <distortion_t2>0.0</distortion_t2>
  </plugin>
</sensor>
```

## Sensor Integration with ROS 2

### Camera Integration

```python
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CameraInfo
from cv_bridge import CvBridge
import cv2

class CameraSubscriber(Node):
    def __init__(self):
        super().__init__('camera_subscriber')
        self.subscription = self.create_subscription(
            Image,
            'camera/image_raw',
            self.listener_callback,
            10)
        self.subscription  # prevent unused variable warning
        self.bridge = CvBridge()

    def listener_callback(self, msg):
        # Convert ROS Image message to OpenCV image
        cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')

        # Process the image (e.g., object detection, feature extraction)
        processed_image = self.process_image(cv_image)

        # Display the image
        cv2.imshow('Camera View', processed_image)
        cv2.waitKey(1)

    def process_image(self, image):
        # Add your image processing logic here
        return image
```

### LiDAR Integration

```python
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
import numpy as np

class LaserSubscriber(Node):
    def __init__(self):
        super().__init__('laser_subscriber')
        self.subscription = self.create_subscription(
            LaserScan,
            'scan',
            self.scan_callback,
            10)
        self.subscription  # prevent unused variable warning

    def scan_callback(self, msg):
        # Extract range data
        ranges = np.array(msg.ranges)

        # Filter out invalid ranges (inf or nan)
        valid_ranges = ranges[np.isfinite(ranges)]

        # Calculate statistics
        if len(valid_ranges) > 0:
            min_distance = np.min(valid_ranges)
            avg_distance = np.mean(valid_ranges)

            self.get_logger().info(f'Min distance: {min_distance:.2f}, Avg distance: {avg_distance:.2f}')
```

## Advanced Sensor Features

### Multi-Camera Systems

For stereo vision or multi-view systems:

```xml
<model name="stereo_camera">
  <link name="left_camera">
    <sensor name="left_cam" type="camera">
      <!-- Left camera configuration -->
    </sensor>
  </link>
  <link name="right_camera">
    <sensor name="right_cam" type="camera">
      <!-- Right camera configuration -->
    </sensor>
  </link>
</model>
```

### Sensor Fusion

Combining multiple sensors for better perception:

```python
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu, LaserScan
from geometry_msgs.msg import PoseWithCovarianceStamped

class SensorFusionNode(Node):
    def __init__(self):
        super().__init__('sensor_fusion')

        # Subscribe to multiple sensors
        self.imu_sub = self.create_subscription(Imu, 'imu/data', self.imu_callback, 10)
        self.scan_sub = self.create_subscription(LaserScan, 'scan', self.scan_callback, 10)

        # Publish fused pose estimate
        self.pose_pub = self.create_publisher(PoseWithCovarianceStamped, 'fused_pose', 10)

        # Initialize filters (e.g., Kalman filter)
        self.initialize_filters()

    def initialize_filters(self):
        # Initialize your sensor fusion algorithm here
        pass

    def imu_callback(self, msg):
        # Process IMU data
        pass

    def scan_callback(self, msg):
        # Process LiDAR data
        pass
```

## Performance Considerations

### Sensor Update Rates

Balance accuracy with performance:
- High update rates: More accurate but computationally expensive
- Low update rates: Less accurate but more efficient

### Visualization

Turn off visualization for sensors when not needed:
```xml
<visualize>false</visualize>
```

## Troubleshooting Sensor Issues

### Common Problems

1. **No sensor data**: Check plugin configuration and topic names
2. **Incorrect frame IDs**: Verify TF tree and frame names
3. **Excessive noise**: Adjust noise parameters in sensor configuration
4. **Performance issues**: Reduce update rates or simplify models

### Debugging Tips

- Use `gz topic -l` to list available topics
- Use `gz topic -e` to echo sensor data
- Check sensor plugin logs in Gazebo

## Exercise

Create a robot model with:
1. RGB camera for visual perception
2. 2D LiDAR for obstacle detection
3. IMU for orientation estimation
4. ROS 2 nodes to process data from each sensor

### Implementation Outline

1. Define a robot model with sensors in an SDF/URDF file
2. Create ROS 2 nodes to subscribe to sensor data
3. Process sensor data to extract meaningful information
4. Visualize or log the processed data

This comprehensive sensor simulation approach will allow you to develop and test perception algorithms in a realistic environment.