---
title: ROS 2 Packages and Nodes
sidebar_label: ROS 2 Packages and Nodes
description: Creating and managing ROS 2 packages and developing nodes for robotic applications
keywords: [ros2, packages, nodes, development, robotic applications]
---

## Learning Objectives

- Create and manage ROS 2 packages using colcon build system
- Develop nodes in both C++ and Python
- Understand package dependencies and build configurations
- Implement proper node lifecycle management
- Create reusable and maintainable node structures

## Introduction

In this module, we'll dive deep into creating ROS 2 packages and developing nodes - the fundamental building blocks of any ROS 2 application. Understanding how to properly structure packages and implement nodes is crucial for developing robust and maintainable robotic systems.

A ROS 2 package is the basic building unit that contains nodes, libraries, and other resources. Nodes are executable programs that perform specific functions within the ROS 2 system. This module will guide you through the process of creating packages, structuring them properly, and implementing nodes with good practices.

## Understanding ROS 2 Packages

### What is a ROS 2 Package?

A package is the fundamental unit of code organization in ROS 2. It contains:

- Source code (C++ and/or Python)
- Launch files
- Configuration files
- Test files
- Package metadata
- Dependencies information

### Package Structure

A typical ROS 2 package follows this structure:

```
my_robot_package/
├── CMakeLists.txt          # Build configuration for C++
├── package.xml             # Package metadata and dependencies
├── src/                    # Source code files
│   ├── publisher_node.cpp
│   └── subscriber_node.cpp
├── include/my_robot_package/ # Header files (for C++)
├── launch/                 # Launch files
│   └── robot.launch.py
├── config/                 # Configuration files
├── test/                   # Unit and integration tests
├── scripts/                # Executable scripts (Python, Bash)
└── README.md               # Documentation
```

## Creating a ROS 2 Package

### Using the Package Creation Tool

ROS 2 provides a command-line tool to create packages:

```bash
# Create a C++ package
ros2 pkg create --build-type ament_cmake my_robot_package

# Create a Python package
ros2 pkg create --build-type ament_python my_robot_package

# Create a mixed package (C++ and Python)
ros2 pkg create --build-type ament_cmake my_robot_package
```

### Package.xml Configuration

The `package.xml` file contains metadata about the package:

```xml
<?xml version="1.0"?>
<?xml-model href="http://download.ros.org/schema/package_format3.xsd" schematypens="http://www.w3.org/2001/XMLSchema"?>
<package format="3">
  <name>my_robot_package</name>
  <version>0.0.0</version>
  <description>Package for my robot functionality</description>
  <maintainer email="user@example.com">User Name</maintainer>
  <license>Apache License 2.0</license>

  <buildtool_depend>ament_cmake</buildtool_depend>

  <depend>rclcpp</depend>
  <depend>rclpy</depend>
  <depend>std_msgs</depend>
  <depend>geometry_msgs</depend>
  <depend>sensor_msgs</depend>

  <test_depend>ament_lint_auto</test_depend>
  <test_depend>ament_lint_common</test_depend>

  <export>
    <build_type>ament_cmake</build_type>
  </export>
</package>
```

## Creating Nodes in C++

### Basic Node Structure

Here's a template for a basic ROS 2 C++ node:

```cpp
#include "rclcpp/rclcpp.hpp"
// Include message types you'll use
#include "std_msgs/msg/string.hpp"
#include "geometry_msgs/msg/twist.hpp"

class MyRobotNode : public rclcpp::Node
{
public:
    MyRobotNode() : Node("my_robot_node")
    {
        // Initialize the node
        RCLCPP_INFO(this->get_logger(), "MyRobotNode initialized");
        
        // Create publishers, subscribers, services, etc.
        setup_components();
    }

private:
    void setup_components()
    {
        // Create a publisher
        publisher_ = this->create_publisher<std_msgs::msg::String>(
            "robot_status", 10);
            
        // Create a subscriber
        subscription_ = this->create_subscription<std_msgs::msg::String>(
            "command", 10,
            std::bind(&MyRobotNode::topic_callback, this, std::placeholders::_1));
            
        // Create a timer
        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(500),
            std::bind(&MyRobotNode::timer_callback, this));
    }
    
    void topic_callback(const std_msgs::msg::String::SharedPtr msg)
    {
        RCLCPP_INFO(this->get_logger(), "Received command: '%s'", msg->data.c_str());
        // Process the command
        process_command(msg->data);
    }
    
    void timer_callback()
    {
        // Periodic tasks
        auto message = std_msgs::msg::String();
        message.data = "Robot is running";
        publisher_->publish(message);
    }
    
    void process_command(const std::string& command)
    {
        // Implement command processing logic
        RCLCPP_INFO(this->get_logger(), "Processing command: %s", command.c_str());
    }

    // Member variables
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_;
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr subscription_;
    rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<MyRobotNode>());
    rclcpp::shutdown();
    return 0;
}
```

### CMakeLists.txt Configuration

The `CMakeLists.txt` file defines how to build the package:

```cmake
cmake_minimum_required(VERSION 3.8)
project(my_robot_package)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclpy REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)

# Create executable
add_executable(robot_node src/robot_node.cpp)
ament_target_dependencies(robot_node 
  rclcpp 
  std_msgs 
  geometry_msgs
  sensor_msgs)

# Install executables
install(TARGETS
  robot_node
  DESTINATION lib/${PROJECT_NAME}
)

# Install other files
install(DIRECTORY
  launch
  config
  DESTINATION share/${PROJECT_NAME}/
)

# Enable testing
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

# Export package
ament_package()
```

## Creating Nodes in Python

### Basic Node Structure

Here's a template for a basic ROS 2 Python node:

```python
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
import math

class MyRobotNode(Node):
    def __init__(self):
        super().__init__('my_robot_node')
        self.get_logger().info('MyRobotNode initialized')
        
        # Initialize variables
        self.command_received = False
        self.robot_position = [0.0, 0.0]
        
        # Setup components
        self.setup_components()
    
    def setup_components(self):
        # Create publisher
        self.publisher_ = self.create_publisher(
            String, 
            'robot_status', 
            10
        )
        
        # Create subscriber
        self.subscription_ = self.create_subscription(
            String,
            'command',
            self.topic_callback,
            10
        )
        
        # Create timer
        self.timer_ = self.create_timer(
            0.5,  # 0.5 seconds
            self.timer_callback
        )
        
        # Create another publisher for robot commands
        self.cmd_publisher_ = self.create_publisher(
            Twist,
            'cmd_vel',
            10
        )
    
    def topic_callback(self, msg):
        self.get_logger().info(f'Received command: {msg.data}')
        self.command_received = True
        self.process_command(msg.data)
    
    def timer_callback(self):
        msg = String()
        msg.data = f'Robot running at position: {self.robot_position}'
        self.publisher_.publish(msg)
    
    def process_command(self, command):
        self.get_logger().info(f'Processing command: {command}')
        # Add your command processing logic here
        if command == 'move_forward':
            self.move_forward()
        elif command == 'turn_left':
            self.turn_left()
    
    def move_forward(self):
        msg = Twist()
        msg.linear.x = 0.5  # Move forward at 0.5 m/s
        self.cmd_publisher_.publish(msg)
    
    def turn_left(self):
        msg = Twist()
        msg.angular.z = 0.5  # Turn left at 0.5 rad/s
        self.cmd_publisher_.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = MyRobotNode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Node interrupted by user')
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
```

### setup.py Configuration for Python Packages

For Python packages, you need a `setup.py` file:

```python
from setuptools import setup
import os
from glob import glob

package_name = 'my_robot_package'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        # Include launch files
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),
        # Include config files
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Your Name',
    maintainer_email='your.email@example.com',
    description='Package for my robot functionality',
    license='Apache License 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'robot_node = my_robot_package.robot_node:main',
        ],
    },
)
```

## Advanced Node Concepts

### Node Parameters

Nodes can have configurable parameters:

```cpp
// C++ example
class ParameterNode : public rclcpp::Node
{
public:
    ParameterNode() : Node("parameter_node")
    {
        // Declare parameters with default values
        this->declare_parameter("robot_name", "default_robot");
        this->declare_parameter("max_speed", 1.0);
        this->declare_parameter("wheel_diameter", 0.1);
        
        // Get parameter values
        robot_name_ = this->get_parameter("robot_name").as_string();
        max_speed_ = this->get_parameter("max_speed").as_double();
        wheel_diameter_ = this->get_parameter("wheel_diameter").as_double();
    }
    
private:
    std::string robot_name_;
    double max_speed_;
    double wheel_diameter_;
};
```

```python
# Python example
class ParameterNode(Node):
    def __init__(self):
        super().__init__('parameter_node')
        
        # Declare parameters with default values
        self.declare_parameter('robot_name', 'default_robot')
        self.declare_parameter('max_speed', 1.0)
        self.declare_parameter('wheel_diameter', 0.1)
        
        # Get parameter values
        self.robot_name = self.get_parameter('robot_name').value
        self.max_speed = self.get_parameter('max_speed').value
        self.wheel_diameter = self.get_parameter('wheel_diameter').value
```

### Node Lifecycle Management

ROS 2 provides lifecycle nodes for more complex state management:

```cpp
#include "rclcpp_lifecycle/lifecycle_node.hpp"
#include "rclcpp_lifecycle/lifecycle_publisher.hpp"

class LifecycleNode : public rclcpp_lifecycle::LifecycleNode
{
public:
    LifecycleNode() : rclcpp_lifecycle::LifecycleNode("lifecycle_node") {}
    
    rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
    on_configure(const rclcpp_lifecycle::State &)
    {
        RCLCPP_INFO(get_logger(), "Configuring");
        pub_ = this->create_publisher<std_msgs::msg::String>("topic", 10);
        return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
    }
    
    rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
    on_activate(const rclcpp_lifecycle::State &)
    {
        RCLCPP_INFO(get_logger(), "Activating");
        pub_->on_activate();
        return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
    }
    
    rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
    on_deactivate(const rclcpp_lifecycle::State &)
    {
        RCLCPP_INFO(get_logger(), "Deactivating");
        pub_->on_deactivate();
        return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
    }
    
    rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
    on_cleanup(const rclcpp_lifecycle::State &)
    {
        RCLCPP_INFO(get_logger(), "Cleaning up");
        pub_.reset();
        return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
    }

private:
    rclcpp_lifecycle::LifecyclePublisher<std_msgs::msg::String>::SharedPtr pub_;
};
```

## Best Practices for Package Development

### 1. Package Organization

- Use descriptive package names
- Group related functionality in the same package
- Separate different types of functionality into different packages
- Follow the ROS 2 naming conventions

### 2. Code Structure

- Separate interfaces (message definitions) into their own packages
- Use consistent naming conventions
- Document your code with comments
- Write unit tests for your components

### 3. Dependency Management

- List all dependencies in package.xml
- Use the minimum required version of dependencies
- Avoid circular dependencies between packages

### 4. Build System

- Use ament_cmake for C++ packages
- Use ament_python for Python packages
- Write proper CMakeLists.txt files
- Test your build process regularly

## Building and Running Packages

### Building Packages

```bash
# Build a specific package
colcon build --packages-select my_robot_package

# Build with specific packages and dependencies
colcon build --packages-up-to my_robot_package

# Build everything
colcon build

# Build with additional flags (for debugging)
colcon build --packages-select my_robot_package --cmake-args -DCMAKE_BUILD_TYPE=Debug
```

### Running Nodes

```bash
# Run a node directly
ros2 run my_robot_package robot_node

# Run with parameters
ros2 run my_robot_package robot_node --ros-args -p robot_name:=my_robot -p max_speed:=2.0

# Run with a launch file
ros2 launch my_robot_package robot.launch.py
```

## Creating Launch Files

Launch files allow you to start multiple nodes with specific configurations:

```python
# launch/robot.launch.py
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='my_robot_package',
            executable='robot_node',
            name='my_robot',
            parameters=[
                {'robot_name': 'my_robot'},
                {'max_speed': 1.0}
            ],
            remappings=[
                ('/original_topic', '/new_topic')
            ],
            arguments=['--ros-args', '--log-level', 'info']
        ),
        Node(
            package='another_package',
            executable='sensor_node',
            name='lidar_sensor'
        )
    ])
```

## Testing Your Packages

### Unit Testing

```cpp
// test/test_robot_node.cpp
#include "gtest/gtest.h"
#include "rclcpp/rclcpp.hpp"
#include "my_robot_package/robot_node.hpp"

TEST(RobotNodeTest, InitializationTest) {
    rclcpp::init(0, nullptr);
    auto node = std::make_shared<MyRobotNode>();
    EXPECT_NE(node, nullptr);
    rclcpp::shutdown();
}

int main(int argc, char ** argv) {
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
```

Add to CMakeLists.txt:
```cmake
find_package(ament_cmake_gtest REQUIRED)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
  
  ament_add_gtest(test_robot_node test/test_robot_node.cpp)
  target_link_libraries(test_robot_node robot_node)
endif()
```

## Debugging Nodes

### Using ROS 2 Tools

```bash
# Check node status
ros2 node list
ros2 node info <node_name>

# Check topic information
ros2 topic list
ros2 topic info <topic_name>
ros2 topic echo <topic_name>

# Check service information
ros2 service list
ros2 service info <service_name>
```

### Using Standard Debugging Tools

- Use `RCLCPP_INFO`, `RCLCPP_WARN`, `RCLCPP_ERROR` for C++
- Use `self.get_logger().info()`, etc. for Python
- Use standard debugging tools like gdb for C++
- Use debuggers like pdb for Python

## Summary

This module covered the fundamentals of creating ROS 2 packages and nodes. We explored:

- The structure and organization of ROS 2 packages
- Creating nodes in both C++ and Python
- Configuring build systems with CMakeLists.txt and setup.py
- Managing parameters and node lifecycles
- Best practices for package development
- Building and running packages
- Creating launch files for complex systems

Understanding how to properly structure packages and implement nodes is crucial for developing robust and maintainable robotic systems. In the next module, we'll explore ROS 2 communication patterns in detail.

## Further Reading

- ROS 2 Package Development: https://docs.ros.org/en/rolling/Tutorials/Beginner-Client-Libraries/Creating-Your-First-ROS2-Package.html
- Node Development: https://docs.ros.org/en/rolling/Tutorials/Beginner-Client-Libraries/Single-Package-Define-And-Use-Interface.html
- Launch Files: https://docs.ros.org/en/rolling/Tutorials/Intermediate/Launch/Creating-Launch-Files.html
- Parameters: https://docs.ros.org/en/rolling/Tutorials/Beginner-Client-Libraries/Using-Parameters-In-A-Class-Python.html

## Assessment

Complete the assessment for this module to track your progress.