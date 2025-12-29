---
sidebar_position: 1
title: Introduction to ROS 2
---

# Introduction to ROS 2

## Overview

ROS 2 (Robot Operating System 2) is a flexible framework for writing robot software. It is a collection of tools, libraries, and conventions that aim to simplify the task of creating complex and robust robot behavior across a wide variety of robot platforms.

## Learning Objectives

- Understand the fundamental concepts of ROS 2
- Learn about the architecture of ROS 2
- Explore the differences between ROS 1 and ROS 2
- Set up a basic ROS 2 environment

## What is ROS 2?

ROS 2 is the next generation of the Robot Operating System, designed to address the limitations of ROS 1 and provide a more robust, scalable, and production-ready framework for robotics development.

### Key Features of ROS 2

- **Distributed computing**: ROS 2 allows multiple computers to work together seamlessly
- **Real-time support**: Improved support for real-time systems
- **Platform independence**: Runs on various operating systems (Linux, Windows, macOS)
- **Security**: Built-in security features for safer robot operation
- **Communication middleware**: Uses DDS (Data Distribution Service) for communication

## ROS 2 Architecture

The architecture of ROS 2 is fundamentally different from ROS 1. It's based on DDS (Data Distribution Service) which provides:

- **Decentralized architecture**: No central master node required
- **Language independence**: Support for multiple programming languages
- **Quality of Service (QoS) settings**: Configurable communication behavior
- **Discovery mechanisms**: Automatic node discovery

### Core Concepts

1. **Nodes**: Basic compute elements that perform computation
2. **Topics**: Named buses over which nodes exchange messages
3. **Services**: Synchronous request/response communication pattern
4. **Actions**: Asynchronous goal-oriented communication pattern
5. **Parameters**: Configuration values that can be changed at runtime

## Interactive Example: Creating a Simple Node

Here's a simple example of a ROS 2 node written in Python:

```python
import rclpy
from rclpy.node import Node

class MinimalPublisher(Node):

    def __init__(self):
        super().__init__('minimal_publisher')
        self.publisher_ = self.create_publisher(String, 'topic', 10)
        timer_period = 0.5  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.i = 0

    def timer_callback(self):
        msg = String()
        msg.data = 'Hello World: %d' % self.i
        self.publisher_.publish(msg)
        self.get_logger().info('Publishing: "%s"' % msg.data)
        self.i += 1

def main(args=None):
    rclpy.init(args=args)
    minimal_publisher = MinimalPublisher()
    rclpy.spin(minimal_publisher)
    minimal_publisher.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

This example demonstrates:
- Creating a node class that inherits from `Node`
- Setting up a publisher to send messages
- Using a timer to periodically publish messages
- Proper initialization and cleanup

## Prerequisites

Before diving into ROS 2, you should have:

- Basic knowledge of Python or C++
- Understanding of Linux command line
- Familiarity with package management (apt, pip)
- Basic understanding of distributed systems concepts

## Next Steps

In the next section, we'll explore ROS 2 nodes, topics, and services in more detail.

## Knowledge Check

Test your understanding of ROS 2 fundamentals with these questions:

### Multiple Choice Questions

1. What does ROS 2 use as its communication middleware?
   - A) TCP/IP
   - B) DDS (Data Distribution Service)
   - C) HTTP
   - D) Bluetooth

2. Which of the following is NOT a core concept in ROS 2?
   - A) Nodes
   - B) Topics
   - C) Services
   - D) Databases

3. What is the main advantage of ROS 2's decentralized architecture compared to ROS 1?
   - A) Faster processing
   - B) No single point of failure with the master node
   - C) Better graphics rendering
   - D) More programming languages supported

### True/False Questions

4. ROS 2 provides built-in security features.
   - [ ] True
   - [ ] False

5. Quality of Service (QoS) settings are available in ROS 2.
   - [ ] True
   - [ ] False

### Short Answer Questions

6. Explain the difference between a Topic and a Service in ROS 2.

7. What are the three main components of a basic ROS 2 publisher node?

### Hands-On Exercise

8. Create a simple ROS 2 subscriber node that subscribes to the "topic" from the example above and logs the received messages to the console. Include proper initialization and cleanup in your implementation.

### Solutions

1. B) DDS (Data Distribution Service)
2. D) Databases
3. B) No single point of failure with the master node
4. True
5. True
6. A Topic provides asynchronous, publish-subscribe communication where publishers send data to a topic and subscribers receive data from that topic. A Service provides synchronous request-response communication where a client sends a request to a service and waits for a response.
7. The three main components are: 1) A publisher that sends messages, 2) A timer or callback to trigger message publishing, 3) Proper initialization and cleanup methods.
8. Solution would include creating a subscriber node with proper ROS 2 initialization, subscription to the topic, and cleanup.