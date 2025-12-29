---
sidebar_position: 2
title: Physics Simulation in Gazebo
---

# Physics Simulation in Gazebo

## Understanding Physics Simulation

Physics simulation is at the core of Gazebo's functionality. It allows you to model the physical behavior of robots and their environment with high accuracy. Understanding how physics simulation works is crucial for creating realistic simulations.

## Physics Engine Fundamentals

### Time Stepping

Gazebo advances the simulation in discrete time steps. The size of these steps affects both accuracy and performance:

- **Smaller steps**: More accurate but slower
- **Larger steps**: Faster but less accurate

```xml
<!-- Example physics configuration -->
<physics type="ode">
  <max_step_size>0.001</max_step_size>
  <real_time_factor>1.0</real_time_factor>
  <real_time_update_rate>1000.0</real_time_update_rate>
</physics>
```

### Key Physics Concepts

#### 1. Rigid Body Dynamics

Rigid body dynamics govern how objects move and interact:

- **Position and Orientation**: The pose of an object in 3D space
- **Velocity**: Linear and angular velocity
- **Acceleration**: Rate of change of velocity
- **Forces**: External forces acting on the body
- **Torques**: Rotational forces

#### 2. Collision Detection

Gazebo uses sophisticated algorithms to detect when objects collide:

- **Broad phase**: Quick elimination of non-colliding pairs
- **Narrow phase**: Precise collision detection
- **Contact generation**: Calculation of contact points and forces

#### 3. Contact Physics

When objects collide, Gazebo calculates:

- **Contact points**: Where objects touch
- **Contact forces**: Forces that prevent penetration
- **Friction**: Forces that oppose sliding motion

## Configuring Physics Properties

### Mass and Inertia

Every model needs mass and inertia properties defined:

```xml
<link name="link_name">
  <inertial>
    <mass>1.0</mass>
    <inertia>
      <ixx>0.1</ixx>
      <ixy>0.0</ixy>
      <ixz>0.0</ixz>
      <iyy>0.1</iyy>
      <iyz>0.0</iyz>
      <izz>0.1</izz>
    </inertia>
  </inertial>
</link>
```

### Collision Properties

Collision properties determine how objects interact:

```xml
<collision name="collision">
  <surface>
    <friction>
      <ode>
        <mu>1.0</mu>
        <mu2>1.0</mu2>
      </ode>
    </friction>
    <bounce>
      <restitution_coefficient>0.1</restitution_coefficient>
      <threshold>100000.0</threshold>
    </bounce>
    <contact>
      <ode>
        <soft_cfm>0.0</soft_cfm>
        <soft_erp>0.2</soft_erp>
        <kp>1000000000000.0</kp>
        <kd>1.0</kd>
        <max_vel>0.01</max_vel>
        <min_depth>0.001</min_depth>
      </ode>
    </contact>
  </surface>
</collision>
```

## Gravity and Environment

Gravity is a fundamental force in physics simulation:

```xml
<world name="default">
  <gravity>0 0 -9.8</gravity>
  <physics type="ode">
    <gravity>0 0 -9.8</gravity>
    <!-- Other physics settings -->
  </physics>
</world>
```

## Advanced Physics Features

### Joint Dynamics

Joints connect links and have their own dynamics:

```xml
<joint name="joint_name" type="revolute">
  <parent>parent_link</parent>
  <child>child_link</child>
  <axis>
    <xyz>0 0 1</xyz>
    <limit>
      <lower>-1.57</lower>
      <upper>1.57</upper>
      <effort>100</effort>
      <velocity>1</velocity>
    </limit>
    <dynamics>
      <damping>1.0</damping>
      <friction>0.1</friction>
    </dynamics>
  </axis>
</joint>
```

### Actuators and Controllers

You can apply forces and torques to simulate actuators:

```python
# Example of applying force to a model
import rospy
from std_msgs.msg import Float64
from geometry_msgs.msg import Wrench

# Publish wrench messages to apply forces
wrench_pub = rospy.Publisher('/model_name/joint_name/wrench', Wrench, queue_size=10)

wrench = Wrench()
wrench.force.x = 10.0  # Apply 10N force in x direction
wrench_pub.publish(wrench)
```

## Performance Optimization

### Simulation Parameters

Tune these parameters for better performance:

- **Max step size**: Balance accuracy vs speed
- **Real-time factor**: Control simulation speed
- **Update rate**: How often physics is calculated

### Model Simplification

For better performance, consider:

- **Simplified collision geometries**: Use boxes instead of complex meshes
- **Reduced mesh resolution**: Lower polygon count for visual elements
- **Selective physics**: Disable physics for static objects

## Troubleshooting Common Physics Issues

### Objects Falling Through the Ground

- Check collision properties
- Verify mass and inertia values
- Adjust contact parameters (soft ERP, soft CFM)

### Unstable Simulations

- Reduce time step size
- Adjust solver parameters
- Check for unrealistic mass/inertia values

### Penetration Issues

- Increase stiffness (kp) parameter
- Reduce maximum velocity
- Increase minimum depth

## Interactive Exercise

Create a simulation with:

1. A simple robot with wheels
2. Physics properties for each link
3. Joints with appropriate limits
4. Apply forces to make the robot move

### Solution Outline

1. Define a simple differential drive robot model
2. Set appropriate mass and inertia for each link
3. Configure wheel joints as continuous joints
4. Apply forces to wheel links to make the robot move forward

## Quality of Service Considerations

When integrating with ROS 2, consider the QoS settings for physics-related topics:

```python
# Example QoS for physics state topics
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy

# Physics state updates might use different QoS than other topics
qos_profile = QoSProfile(
    depth=1,
    reliability=QoSReliabilityPolicy.BEST_EFFORT,  # Might be acceptable for physics updates
    history=QoSHistoryPolicy.KEEP_LAST
)
```

This approach to physics simulation ensures that your robot behaves realistically in the simulated environment.