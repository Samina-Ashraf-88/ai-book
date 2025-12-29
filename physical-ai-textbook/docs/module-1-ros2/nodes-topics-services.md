---
sidebar_position: 3
title: Nodes, Topics, and Services
---

# Nodes, Topics, and Services in ROS 2

## Understanding ROS 2 Communication Primitives

ROS 2 provides several communication primitives that allow nodes to interact with each other. Understanding these primitives is crucial for building distributed robotic systems.

## Nodes in ROS 2

### Creating Nodes

In ROS 2, nodes are created differently than in ROS 1. Here's the basic structure:

```python
import rclpy
from rclpy.node import Node

class MyNode(Node):
    def __init__(self):
        super().__init__('my_node_name')
        # Node initialization code here
        self.get_logger().info('MyNode has been started')

def main(args=None):
    rclpy.init(args=args)
    node = MyNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

### Node Names and Namespaces

ROS 2 supports namespaces for organizing nodes:

```python
# Creating a node with namespace
node = MyNode(namespace='robot1')

# This creates a node with name '/robot1/my_node_name'
```

## Topics and Publishers/Subscribers

### Publishers

Creating a publisher in ROS 2:

```python
import rclpy
from rclpy.node import Node
from std_msgs.msg import String

class Talker(Node):

    def __init__(self):
        super().__init__('talker')
        self.publisher_ = self.create_publisher(String, 'chatter', 10)
        timer_period = 0.5  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.i = 0

    def timer_callback(self):
        msg = String()
        msg.data = 'Hello World: %d' % self.i
        self.publisher_.publish(msg)
        self.get_logger().info('Publishing: "%s"' % msg.data)
        self.i += 1
```

### Subscribers

Creating a subscriber in ROS 2:

```python
import rclpy
from rclpy.node import Node
from std_msgs.msg import String

class Listener(Node):

    def __init__(self):
        super().__init__('listener')
        self.subscription = self.create_subscription(
            String,
            'chatter',
            self.listener_callback,
            10)
        self.subscription  # prevent unused variable warning

    def listener_callback(self, msg):
        self.get_logger().info('I heard: "%s"' % msg.data)
```

### Quality of Service (QoS) Settings

QoS settings allow you to configure how messages are delivered:

```python
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy

# Create a QoS profile for reliable communication
qos_profile = QoSProfile(
    depth=10,
    reliability=QoSReliabilityPolicy.RELIABLE,
    history=QoSHistoryPolicy.KEEP_LAST
)

publisher = node.create_publisher(String, 'topic', qos_profile)
```

## Services

Services provide synchronous request/response communication:

### Service Server

```python
from example_interfaces.srv import AddTwoInts
import rclpy
from rclpy.node import Node

class MinimalService(Node):

    def __init__(self):
        super().__init__('minimal_service')
        self.srv = self.create_service(AddTwoInts, 'add_two_ints', self.add_two_ints_callback)

    def add_two_ints_callback(self, request, response):
        response.sum = request.a + request.b
        self.get_logger().info('Incoming request\na: %d b: %d' % (request.a, request.b))
        return response
```

### Service Client

```python
from example_interfaces.srv import AddTwoInts
import rclpy
from rclpy.node import Node

class MinimalClientAsync(Node):

    def __init__(self):
        super().__init__('minimal_client_async')
        self.cli = self.create_client(AddTwoInts, 'add_two_ints')
        while not self.cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('service not available, waiting again...')
        self.req = AddTwoInts.Request()

    def send_request(self, a, b):
        self.req.a = a
        self.req.b = b
        self.future = self.cli.call_async(self.req)
        rclpy.spin_until_future_complete(self, self.future)
        return self.future.result()
```

## Advanced Topic Concepts

### Latching

In ROS 2, the concept of latching is handled through QoS durability settings:

```python
from rclpy.qos import QoSDurabilityPolicy

# Create a publisher that keeps the last message for late-joining subscribers
qos_profile = QoSProfile(
    depth=1,
    durability=QoSDurabilityPolicy.TRANSIENT_LOCAL
)
```

### Publisher/Subscriber Matching

ROS 2 provides callbacks when publishers and subscribers connect:

```python
def chatter_callback(self, msg):
    self.get_logger().info('Received message')

def subscription_match_callback(self, subscription_handle):
    self.get_logger().info('New subscriber connected')

def publisher_match_callback(self, publisher_handle):
    self.get_logger().info('New publisher connected')

# Create subscription with match callbacks
self.subscription = self.create_subscription(
    String,
    'chatter',
    self.chatter_callback,
    10,
    event_callbacks=rclpy.qos.EventCallbacks(
        subscription_matched=self.subscription_match_callback
    )
)

# Create publisher with match callbacks
self.publisher = self.create_publisher(
    String,
    'chatter',
    10,
    event_callbacks=rclpy.qos.EventCallbacks(
        publisher_matched=self.publisher_match_callback
    )
)
```

## Practical Exercise

Create a ROS 2 package with two nodes:

1. A publisher that sends sensor data (temperature, humidity) with QoS settings for reliable delivery
2. A subscriber that receives this data and prints it to the console
3. A service that calculates average values from a list of sensor readings
4. A service client that calls the average calculation service

### Solution Structure

```
sensor_package/
├── CMakeLists.txt
├── package.xml
├── src/
│   ├── sensor_publisher.py
│   ├── sensor_subscriber.py
│   ├── average_service.py
│   └── average_client.py
└── launch/
    └── sensor_launch.py
```

This exercise will help you understand how to implement the basic communication patterns in ROS 2.

## Knowledge Check

Test your understanding of ROS 2 communication patterns with these questions:

### Multiple Choice Questions

1. What is the correct way to create a publisher in ROS 2?
   - A) `self.publisher_ = self.create_publisher(String, 'topic')`
   - B) `self.publisher_ = self.create_publisher(String, 'topic', queue_size=10)`
   - C) `self.publisher_ = self.create_publisher(String, 'topic', 10)`
   - D) `self.publisher_ = Node.create_publisher(String, 'topic', 10)`

2. Which QoS policy ensures that all messages are delivered reliably?
   - A) QoSHistoryPolicy
   - B) QoSReliabilityPolicy.RELIABLE
   - C) QoSDurabilityPolicy
   - D) QoSProfile.DEPTH

3. How do you create a service server in ROS 2?
   - A) `self.create_service(ServiceType, 'service_name', callback)`
   - B) `self.create_server(ServiceType, 'service_name', callback)`
   - C) `Node.create_service(ServiceType, 'service_name', callback)`
   - D) `rclpy.create_service(ServiceType, 'service_name', callback)`

### True/False Questions

4. In ROS 2, nodes must be initialized with `rclpy.init(args=args)` before use.
   - [ ] True
   - [ ] False

5. Quality of Service (QoS) settings are not available in ROS 2.
   - [ ] True
   - [ ] False

### Short Answer Questions

6. Explain the difference between a publisher/subscriber pattern and a service pattern in ROS 2.

7. What are the three main components you need to create a ROS 2 node class?

### Hands-On Exercise

8. Create a ROS 2 service that takes two floating-point numbers and returns their product. Include both the service server and client code, and demonstrate calling the service from the client.

### Solutions

1. C) `self.publisher_ = self.create_publisher(String, 'topic', 10)`
2. B) QoSReliabilityPolicy.RELIABLE
3. A) `self.create_service(ServiceType, 'service_name', callback)`
4. True
5. False
6. Publisher/subscriber is asynchronous communication where publishers send data to topics and subscribers receive from topics. Service is synchronous request/response communication where a client sends a request and waits for a response from a server.
7. The three main components are: 1) A class that inherits from Node, 2) A constructor that calls super().__init__() with a node name, 3) Proper initialization and cleanup methods.
8. Solution would include creating a service definition file, server implementation, client implementation, and demonstration code.