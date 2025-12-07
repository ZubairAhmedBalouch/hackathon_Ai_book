---
title: Module 1 - ROS2
---

# Module 1 - ROS2

## Overview

Robot Operating System 2 (ROS2) is the foundational framework for modern robotics development. This module provides comprehensive coverage of ROS2 architecture, tools, and development practices essential for building robust robotic systems. ROS2 serves as the communication backbone that connects all components of a robotic system, enabling modular development and seamless integration of sensors, actuators, and AI algorithms.

ROS2 has evolved from its predecessor to address critical requirements for production robotics, including real-time performance, security, and multi-robot systems. This module will guide you through ROS2's core concepts, practical implementation, and best practices for building scalable robotic applications.

## Key Concepts

- **ROS2 Architecture**: Client library implementations (RCL), middleware abstraction, and distributed computing model
- **Communication Patterns**: Topics (publish-subscribe), Services (request-response), and Actions (goal-feedback-result)
- **Package Management**: Colcon build system, package.xml, and CMakeLists.txt for dependency management
- **Node Management**: Lifecycle nodes, composition, and resource management
- **Middleware Interface**: Data Distribution Service (DDS) implementations for communication
- **Quality of Service (QoS)**: Policies for reliability, durability, and performance in communication
- **Launch Systems**: Declarative system startup and configuration management

## Details

### ROS2 Architecture and Design Philosophy

ROS2 follows a distributed computing architecture that enables communication between processes running on the same or different machines. The architecture consists of:

1. **Client Libraries**: Language-specific implementations (rclcpp for C++, rclpy for Python) that provide ROS2 functionality
2. **Middleware**: DDS (Data Distribution Service) implementations that handle message passing between nodes
3. **Nodes**: Individual processes that perform specific functions within the robotic system
4. **Communication Primitives**: Topics, services, and actions that define how nodes interact

The design emphasizes:
- **Modularity**: Components can be developed, tested, and deployed independently
- **Scalability**: Systems can scale from single robots to multi-robot teams
- **Flexibility**: Support for different DDS implementations and communication patterns
- **Security**: Built-in security features for production environments

### Core Communication Patterns

**Topics (Publish-Subscribe)**:
- Asynchronous communication for streaming data
- Multiple publishers and subscribers can exist for the same topic
- Used for sensor data, robot state, and other continuous data streams
- Quality of Service settings control reliability and performance

**Services (Request-Response)**:
- Synchronous communication for specific requests
- One-to-one communication pattern
- Used for actions that require confirmation or specific results
- Request-response cycle ensures completion

**Actions (Goal-Feedback-Result)**:
- Asynchronous communication with feedback
- Used for long-running tasks that provide progress updates
- Goal, feedback, and result structure supports complex operations
- Cancellation capability for interruptible tasks

### ROS2 Development Workflow

The standard ROS2 development workflow includes:

1. **Workspace Creation**: Setting up colcon workspace for package management
2. **Package Development**: Creating and organizing code into logical packages
3. **Building**: Using colcon to build all packages in the workspace
4. **Running**: Launching nodes and testing functionality
5. **Debugging**: Using ROS2 tools for system analysis and debugging
6. **Deployment**: Packaging for target hardware and production environments

### Quality of Service (QoS) Profiles

ROS2 provides QoS settings to control communication behavior:

- **Reliability**: Reliable (all messages delivered) vs. Best effort (messages may be lost)
- **Durability**: Transient local (historical data available) vs. Volatile (only new data)
- **History**: Keep all messages vs. Keep last N messages
- **Deadline**: Maximum time between consecutive messages
- **Liveliness**: How to detect if a publisher is still active

## Examples

### Example 1: Basic ROS2 Publisher Node

```cpp
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

class MinimalPublisher : public rclcpp::Node
{
public:
    MinimalPublisher()
    : Node("minimal_publisher"), count_(0)
    {
        publisher_ = this->create_publisher<std_msgs::msg::String>(
            "topic", 10);
        timer_ = this->create_wall_timer(
            500ms, std::bind(&MinimalPublisher::timer_callback, this));
    }

private:
    void timer_callback()
    {
        auto message = std_msgs::msg::String();
        message.data = "Hello, world! " + std::to_string(count_++);
        RCLCPP_INFO(this->get_logger(), "Publishing: '%s'", message.data.c_str());
        publisher_->publish(message);
    }
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_;
    size_t count_;
};
```

### Example 2: ROS2 Subscriber Node

```python
import rclpy
from rclpy.node import Node
from std_msgs.msg import String

class MinimalSubscriber(Node):
    def __init__(self):
        super().__init__('minimal_subscriber')
        self.subscription = self.create_subscription(
            String,
            'topic',
            self.listener_callback,
            10)
        self.subscription  # prevent unused variable warning

    def listener_callback(self, msg):
        self.get_logger().info('I heard: "%s"' % msg.data)

def main(args=None):
    rclpy.init(args=args)
    minimal_subscriber = MinimalSubscriber()
    rclpy.spin(minimal_subscriber)
    minimal_subscriber.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

### Example 3: ROS2 Service Server and Client

**Service Definition** (add_two_ints.srv):
```
int64 a
int64 b
---
int64 sum
```

**Service Server**:
```cpp
#include "rclcpp/rclcpp.hpp"
#include "example_interfaces/srv/add_two_ints.hpp"

class MinimalService : public rclcpp::Node
{
public:
    MinimalService()
    : Node("minimal_service")
    {
        using std::placeholders::_1;
        using std::placeholders::_2;
        service_ = create_service<example_interfaces::srv::AddTwoInts>(
            "add_two_ints",
            std::bind(&MinimalService::add, this, _1, _2));
    }

private:
    void add(const example_interfaces::srv::AddTwoInts::Request::SharedPtr request,
             const example_interfaces::srv::AddTwoInts::Response::SharedPtr response)
    {
        response->sum = request->a + request->b;
        RCLCPP_INFO(rclcpp::get_logger("rclcpp"),
                   "Incoming request\na: %ld, b: %ld", request->a, request->b);
        RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Sending back response: [%ld]", response->sum);
    }
    rclcpp::Service<example_interfaces::srv::AddTwoInts>::SharedPtr service_;
};
```

### Example 4: ROS2 Launch File

**Python Launch File** (launch/demo_nodes.launch.py):
```python
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='demo_nodes_cpp',
            executable='talker',
            name='talker',
            parameters=[
                {'param_name': 'param_value'}
            ],
            remappings=[
                ('chatter', 'my_chatter')
            ]
        ),
        Node(
            package='demo_nodes_cpp',
            executable='listener',
            name='listener'
        )
    ])
```

### Example 5: ROS2 System Architecture Diagram

```
                    ROS2 System Architecture
    ┌─────────────────────────────────────────────────────────┐
    │                    Master Node                          │
    │  (Optional in DDS-based systems)                        │
    └─────────────────┬───────────────────────────────────────┘
                      │
    ┌─────────────────┼─────────────────┐
    │                 │                 │
┌───▼──┐          ┌───▼──┐          ┌───▼──┐
│Sensor│          │Motion│          │Vision│
│Node  │◄────────►│Node  │◄────────►│Node  │
│      │          │      │          │      │
└──────┘          └──────┘          └──────┘
    │                 │                 │
    └─────────────────┼─────────────────┘
                      │
                ┌─────▼─────┐
                │  Action   │
                │ Controller│
                │   Node    │
                └───────────┘
                      │
                ┌─────▼─────┐
                │   UI/     │
                │Monitor Node│
                └───────────┘

    Communication via Topics, Services, and Actions
```

ROS2 provides the essential infrastructure for building complex robotic systems, enabling developers to focus on application logic while leveraging standardized communication patterns and development tools. This module establishes the foundation for all subsequent modules in the Physical AI and Humanoid Robotics curriculum.