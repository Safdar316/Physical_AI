---
title: ROS 2 Architecture and Core Concepts
sidebar_label: ROS 2 Architecture and Core Concepts
description: Introduction to ROS 2 architecture and core concepts for robotic control
keywords: [ros2, robotics, architecture, core concepts, robotic control]
---

## Learning Objectives

- Understand the architecture of ROS 2 and its core components
- Explain the differences between ROS 1 and ROS 2
- Identify the key concepts and design principles of ROS 2
- Describe the DDS-based communication architecture
- Understand the role of RMW (ROS Middleware) layer

## Introduction

Welcome to Week 3 of the Physical Humanoid AI Robotics Course! This week, we'll explore the Robot Operating System 2 (ROS 2), the next-generation framework for robotic applications. ROS 2 represents a significant evolution from ROS 1, designed to address the needs of production robotics with improved security, real-time capabilities, and scalability.

ROS 2 is built on a modern architecture that enables distributed computing, supports multiple platforms, and provides a robust foundation for developing complex robotic systems. Understanding its architecture is crucial for building modular, scalable, and maintainable robotic applications.

## What is ROS 2?

ROS 2 (Robot Operating System 2) is not an actual operating system but a flexible framework for writing robot software. It provides a collection of tools, libraries, and conventions that aim to simplify the task of creating complex and robust robot behavior across a wide variety of robot platforms.

### Key Features of ROS 2

- **Distributed Computing**: Nodes can run on different machines and communicate over networks
- **Language Agnostic**: Support for multiple programming languages (C++, Python, etc.)
- **Real-time Capabilities**: Support for real-time applications and determinism
- **Security**: Built-in security features for production environments
- **Cross-platform**: Runs on Linux, macOS, and Windows
- **Standards-based**: Uses Data Distribution Service (DDS) for communication

### Differences Between ROS 1 and ROS 2

| Feature | ROS 1 | ROS 2 |
|---------|-------|-------|
| Communication | Custom TCP/UDP | DDS-based |
| Master | Centralized ros_master | Distributed |
| Real-time | Limited support | Better support |
| Security | No built-in security | Built-in security |
| Cross-platform | Linux focused | Multi-platform |
| Deployment | Research focused | Production ready |

## ROS 2 Architecture

The architecture of ROS 2 is built around the concept of a distributed system where nodes communicate with each other using a publish-subscribe model, services, and actions. The architecture consists of several layers:

### 1. Client Library Layer

The client libraries provide the API for creating ROS 2 applications in different programming languages:

- **rclcpp**: C++ client library
- **rclpy**: Python client library
- Other languages have experimental support

### 2. ROS Client Library (rcl) Layer

This layer provides a common interface for all client libraries, abstracting the underlying middleware.

### 3. ROS Middleware (RMW) Layer

The ROS Middleware layer abstracts the underlying DDS implementation, allowing ROS 2 to work with different DDS vendors.

### 4. DDS Layer

Data Distribution Service (DDS) is the underlying communication middleware that provides the publish-subscribe communication model.

## Core Concepts in ROS 2

### Nodes

A node is an executable that uses ROS 2 to communicate with other nodes. Nodes are the basic building blocks of a ROS 2 system.

**Characteristics of Nodes:**
- Encapsulate the functionality of a single module
- Communicate with other nodes through topics, services, or actions
- Can be written in different programming languages
- Can run on different machines

**Creating a Node in C++:**
```cpp
#include "rclcpp/rclcpp.hpp"

class MinimalNode : public rclcpp::Node
{
public:
    MinimalNode() : Node("minimal_node") {
        RCLCPP_INFO(this->get_logger(), "Hello from minimal node!");
    }
};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<MinimalNode>());
    rclcpp::shutdown();
    return 0;
}
```

**Creating a Node in Python:**
```python
import rclcpp
from rclcpp.node import Node

class MinimalNode(Node):
    def __init__(self):
        super().__init__('minimal_node')
        self.get_logger().info('Hello from minimal node!')

def main(args=None):
    rclcpp.init(args=args)
    minimal_node = MinimalNode()
    rclcpp.spin(minimal_node)
    minimal_node.destroy_node()
    rclcpp.shutdown()

if __name__ == '__main__':
    main()
```

### Topics and Messages

Topics are named buses over which nodes exchange messages. The publish-subscribe communication pattern allows for loose coupling between publishers and subscribers.

**Message Types:**
- Standard messages (e.g., std_msgs, geometry_msgs)
- Custom messages defined in .msg files
- Built-in types (int32, float64, string, etc.)

**Publisher Example (C++):**
```cpp
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

class MinimalPublisher : public rclcpp::Node
{
public:
    MinimalPublisher() : Node("minimal_publisher"), count_(0)
    {
        publisher_ = this->create_publisher<std_msgs::msg::String>("topic", 10);
        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(500),
            std::bind(&MinimalPublisher::timer_callback, this));
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

### Services and Actions

#### Services
Services provide a request-response communication pattern. A service client sends a request and waits for a response from the service server.

**Service Definition (.srv):**
```
# Request
string name
int32 age
---
# Response
bool success
string message
```

#### Actions
Actions are used for long-running tasks that may take feedback and goal preemption. They consist of a goal, feedback, and result.

**Action Definition (.action):**
```
# Goal
int32 order
---
# Result
int32[] sequence
---
# Feedback
int32[] sequence
```

## DDS and RMW Layer

### Data Distribution Service (DDS)

DDS (Data Distribution Service) is an OMG standard for real-time, distributed, and scalable applications. It provides:

- **Publish-Subscribe Communication**: Automatic discovery and data exchange
- **Quality of Service (QoS) Policies**: Configurable behavior for reliability, durability, etc.
- **Real-time Performance**: Deterministic behavior for time-critical applications
- **Platform Independence**: Language and OS agnostic

### ROS Middleware (RMW) Layer

The RMW layer abstracts the DDS implementation, allowing ROS 2 to work with different DDS vendors (Fast DDS, Cyclone DDS, RTI Connext, etc.).

**QoS Policies in ROS 2:**
- **Reliability**: Best effort or reliable delivery
- **Durability**: Volatile or transient local
- **History**: Keep last N samples or keep all
- **Deadline**: Maximum time between sample deliveries
- **Liveliness**: How to determine if a participant is alive

## ROS 2 Ecosystem

### Core Tools

- **ros2 run**: Execute a node
- **ros2 topic**: Inspect and interact with topics
- **ros2 service**: Inspect and interact with services
- **ros2 action**: Inspect and interact with actions
- **ros2 node**: List and introspect nodes
- **ros2 param**: Manage parameters

### Build System: colcon

ROS 2 uses colcon as its build system, which is an improvement over catkin in ROS 1. It supports building packages in parallel and works with different build systems.

## Package Structure

A ROS 2 package typically contains:

```
my_package/
├── CMakeLists.txt          # Build configuration for C++
├── package.xml             # Package metadata
├── src/                    # Source code
│   ├── publisher_node.cpp
│   └── subscriber_node.cpp
├── include/my_package/     # Header files
├── launch/                 # Launch files
├── config/                 # Configuration files
└── test/                   # Test files
```

## Quality of Service (QoS) in Depth

QoS policies allow you to configure the behavior of your ROS 2 communications:

```cpp
// Example of setting QoS policies
rclcpp::QoS qos_profile(10);  // history depth of 10
qos_profile.reliability(RMW_QOS_POLICY_RELIABILITY_RELIABLE);
qos_profile.durability(RMW_QOS_POLICY_DURABILITY_TRANSIENT_LOCAL);
```

### Common QoS Configurations

1. **Real-time Communication**: Reliable, volatile, keep last
2. **Logging**: Best effort, durable, keep all
3. **Configuration**: Reliable, durable, keep last

## Security in ROS 2

ROS 2 includes built-in security features:

- **Authentication**: Verify node identity
- **Authorization**: Control what nodes can do
- **Encryption**: Protect data in transit
- **Access Control**: Define permissions for topics, services, etc.

## ROS 2 in Production

ROS 2 is designed with production environments in mind:

- **Deterministic Behavior**: Real-time capabilities
- **Security**: Built-in security features
- **Scalability**: Support for large systems
- **Reliability**: Robust error handling
- **Monitoring**: Tools for system introspection

## Summary

ROS 2 provides a modern, scalable architecture for developing robotic applications. Its DDS-based communication system, combined with Quality of Service policies, makes it suitable for both research and production environments. Understanding its architecture is crucial for developing efficient and robust robotic systems.

In the next modules, we'll explore practical aspects of creating packages, nodes, and implementing communication patterns in ROS 2.

## Further Reading

- ROS 2 Documentation: https://docs.ros.org/
- DDS Specification: https://www.omg.org/spec/DDS/
- Real-Time Innovations: https://www.rti.com/
- Eclipse Cyclone DDS: https://cyclonedds.io/

## Assessment

Complete the assessment for this module to track your progress.