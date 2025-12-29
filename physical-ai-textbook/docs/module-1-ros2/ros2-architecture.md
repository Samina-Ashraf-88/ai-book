---
sidebar_position: 2
title: ROS 2 Architecture
---

# ROS 2 Architecture

## Understanding the ROS 2 Ecosystem

ROS 2 has a more sophisticated architecture compared to ROS 1, designed for production environments. The architecture is built around the concept of DDS (Data Distribution Service) as the underlying communication middleware.

## Key Architectural Components

### 1. DDS (Data Distribution Service)

DDS is the communication middleware that ROS 2 uses. It provides:

- **Data-centricity**: Data is accessible by name rather than through a specific server
- **Native marshalling/unmarshalling**: Automatic conversion between in-memory and network representations
- **Discovery**: Automatic discovery of publishers and subscribers
- **QoS (Quality of Service)**: Configurable policies for reliability, durability, etc.

### 2. RMW (ROS Middleware)

The ROS Middleware layer abstracts the underlying DDS implementation, allowing ROS 2 to work with different DDS vendors.

### 3. rcl (ROS Client Library)

The C client library that provides the core ROS 2 functionality.

### 4. rclcpp and rclpy

Language-specific client libraries for C++ and Python that wrap the rcl layer.

## Nodes and Communication Patterns

### Nodes

In ROS 2, nodes are more robust:

- Each node has its own context
- Nodes can be created and destroyed independently
- No central master required for discovery

### Topics and Publishers/Subscribers

The publish/subscribe pattern remains the same, but with important enhancements:

- **Reliable vs Best Effort**: Choose based on your application needs
- **Keep Last vs Keep All**: Configure message history
- **Durability**: Configure how messages persist

```python
# Example with QoS settings
from rclpy.qos import QoSProfile, QoSDurabilityPolicy

# Create a QoS profile with specific settings
qos_profile = QoSProfile(
    depth=10,
    durability=QoSDurabilityPolicy.TRANSIENT_LOCAL
)

publisher = node.create_publisher(String, 'topic', qos_profile)
```

### Services and Clients

Services in ROS 2 support:

- **Request/Response patterns**: Synchronous communication
- **Timeouts**: Configurable request timeouts
- **QoS settings**: For service communication as well

### Actions

Actions are a new communication pattern in ROS 2:

- **Goal**: Request to perform a task
- **Feedback**: Continuous updates on progress
- **Result**: Final outcome of the task

```python
# Example action client
from rclpy.action import ActionClient
from example_interfaces.action import Fibonacci

class FibonacciActionClient(Node):

    def __init__(self):
        super().__init__('fibonacci_action_client')
        self._action_client = ActionClient(
            self, Fibonacci, 'fibonacci')

    def send_goal(self):
        goal_msg = Fibonacci.Goal()
        goal_msg.order = 10

        self._action_client.wait_for_server()
        self._send_goal_future = self._action_client.send_goal_async(
            goal_msg,
            feedback_callback=self.feedback_callback)

        self._send_goal_future.add_done_callback(self.goal_response_callback)
```

## Package Structure

ROS 2 packages follow the ament build system:

```
my_package/
├── CMakeLists.txt
├── package.xml
├── src/
│   └── my_node.cpp
├── include/
│   └── my_package/
│       └── my_header.hpp
├── launch/
│   └── my_launch_file.py
├── config/
│   └── my_params.yaml
└── test/
    └── test_my_node.cpp
```

## Launch Files

ROS 2 uses Python-based launch files:

```python
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='my_package',
            executable='my_node',
            name='my_node',
            parameters=[
                {'param_name': 'param_value'}
            ]
        )
    ])
```

## Parameter Management

Parameters in ROS 2 are more powerful:

- **Node parameters**: Parameters associated with specific nodes
- **Parameter services**: Dynamic parameter changes
- **Parameter files**: YAML files for parameter configuration

```yaml
# Example parameter file
my_node:
  ros__parameters:
    param1: 10
    param2: "hello"
    param3:
      nested_param: true
```

## Exercise

Create a simple ROS 2 package with a publisher that sends messages with QoS settings configured for reliable delivery. Include a launch file that starts the node.

## Knowledge Check

Test your understanding of ROS 2 architecture with these questions:

### Multiple Choice Questions

1. What does DDS stand for in ROS 2?
   - A) Distributed Data System
   - B) Data Distribution Service
   - C) Dynamic Discovery System
   - D) Distributed Development System

2. Which layer abstracts the underlying DDS implementation in ROS 2?
   - A) rcl
   - B) RMW
   - C) rclcpp
   - D) ament

3. What is the main difference between Services and Actions in ROS 2?
   - A) Services are asynchronous, Actions are synchronous
   - B) Services are synchronous, Actions provide feedback during execution
   - C) There is no difference
   - D) Services can only send strings, Actions can send any data type

### True/False Questions

4. ROS 2 requires a central master node for discovery, just like ROS 1.
   - [ ] True
   - [ ] False

5. Actions in ROS 2 provide three components: Goal, Feedback, and Result.
   - [ ] True
   - [ ] False

### Short Answer Questions

6. Explain the difference between 'Keep Last' and 'Keep All' QoS history policies.

7. What is the purpose of the RMW (ROS Middleware) layer in ROS 2?

### Hands-On Exercise

8. Create a ROS 2 launch file that starts two nodes: one publisher and one subscriber. Configure QoS settings for reliable communication and include parameters for both nodes.

### Solutions

1. B) Data Distribution Service
2. B) RMW
3. B) Services are synchronous, Actions provide feedback during execution
4. False
5. True
6. 'Keep Last' maintains only the most recent messages (up to a specified depth), while 'Keep All' maintains all messages sent on the topic. 'Keep Last' is more memory efficient, while 'Keep All' ensures no data is lost.
7. The RMW layer abstracts the underlying DDS implementation, allowing ROS 2 to work with different DDS vendors without changing the user code.
8. Solution would include creating a launch file with two nodes, QoS configurations, and parameter settings.