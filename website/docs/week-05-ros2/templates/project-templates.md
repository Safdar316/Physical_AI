---
title: ROS 2 Project Templates for Exercises
sidebar_label: ROS 2 Project Templates
description: Template packages and structures for ROS 2 exercises and projects
keywords: [ros2, templates, project, exercises, packages]
---

## Learning Objectives

- Understand the structure of ROS 2 project templates
- Use templates to create new ROS 2 packages quickly
- Implement common robot functionality patterns
- Create reusable components for exercises
- Apply best practices for package structure

## Introduction

This module provides templates and examples for creating ROS 2 packages and projects. These templates are designed to help you get started quickly with common robotic applications and exercises. Each template follows ROS 2 best practices and includes common components needed for robotic systems.

The templates include both C++ and Python implementations where applicable, allowing you to choose the language that best suits your needs.

## Basic Package Template

### Directory Structure

```
robot_exercise_package/
├── CMakeLists.txt          # Build configuration for C++
├── package.xml             # Package metadata and dependencies
├── src/                    # Source code files
│   ├── publisher_node.cpp
│   ├── subscriber_node.cpp
│   ├── service_server.cpp
│   └── service_client.cpp
├── include/robot_exercise_package/ # Header files (for C++)
├── launch/                 # Launch files
│   └── robot_exercise.launch.py
├── config/                 # Configuration files
│   └── parameters.yaml
├── test/                   # Unit and integration tests
├── scripts/                # Python scripts (if needed)
└── README.md               # Documentation
```

### package.xml

```xml
<?xml version="1.0"?>
<?xml-model href="http://download.ros.org/schema/package_format3.xsd" schematypens="http://www.w3.org/2001/XMLSchema"?>
<package format="3">
  <name>robot_exercise_package</name>
  <version>0.0.0</version>
  <description>Template package for robot exercises</description>
  <maintainer email="user@example.com">Your Name</maintainer>
  <license>Apache License 2.0</license>

  <buildtool_depend>ament_cmake</buildtool_depend>

  <depend>rclcpp</depend>
  <depend>rclpy</depend>
  <depend>std_msgs</depend>
  <depend>geometry_msgs</depend>
  <depend>sensor_msgs</depend>
  <depend>nav_msgs</depend>
  <depend>message_runtime</depend>

  <test_depend>ament_lint_auto</test_depend>
  <test_depend>ament_lint_common</test_depend>

  <export>
    <build_type>ament_cmake</build_type>
  </export>
</package>
```

### CMakeLists.txt

```cmake
cmake_minimum_required(VERSION 3.8)
project(robot_exercise_package)

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
find_package(nav_msgs REQUIRED)

# Create executables
add_executable(publisher_node src/publisher_node.cpp)
add_executable(subscriber_node src/subscriber_node.cpp)
add_executable(service_server src/service_server.cpp)
add_executable(service_client src/service_client.cpp)

# Add dependencies
ament_target_dependencies(publisher_node
  rclcpp
  std_msgs
  geometry_msgs)

ament_target_dependencies(subscriber_node
  rclcpp
  std_msgs
  geometry_msgs)

ament_target_dependencies(service_server
  rclcpp
  std_msgs
  geometry_msgs)

ament_target_dependencies(service_client
  rclcpp
  std_msgs
  geometry_msgs)

# Install executables
install(TARGETS
  publisher_node
  subscriber_node
  service_server
  service_client
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

## Publisher Node Template (C++)

```cpp
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "geometry_msgs/msg/twist.hpp"

class PublisherNode : public rclcpp::Node
{
public:
    PublisherNode() : Node("publisher_node")
    {
        // Create publishers
        cmd_vel_publisher_ = this->create_publisher<geometry_msgs::msg::Twist>("cmd_vel", 10);
        status_publisher_ = this->create_publisher<std_msgs::msg::String>("status", 10);
        
        // Create timer for periodic publishing
        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(100),
            std::bind(&PublisherNode::timer_callback, this));
        
        RCLCPP_INFO(this->get_logger(), "Publisher Node initialized");
    }

private:
    void timer_callback()
    {
        // Create and publish velocity command
        auto cmd_vel_msg = geometry_msgs::msg::Twist();
        cmd_vel_msg.linear.x = 0.5;  // Move forward at 0.5 m/s
        cmd_vel_msg.angular.z = 0.2; // Turn left at 0.2 rad/s
        cmd_vel_publisher_->publish(cmd_vel_msg);
        
        // Publish status message
        auto status_msg = std_msgs::msg::String();
        status_msg.data = "Robot moving - " + std::to_string(rclcpp::Clock().now().nanoseconds());
        status_publisher_->publish(status_msg);
    }
    
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_publisher_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr status_publisher_;
    rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<PublisherNode>());
    rclcpp::shutdown();
    return 0;
}
```

## Subscriber Node Template (C++)

```cpp
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "geometry_msgs/msg/twist.hpp"

class SubscriberNode : public rclcpp::Node
{
public:
    SubscriberNode() : Node("subscriber_node")
    {
        // Create subscribers
        status_subscription_ = this->create_subscription<std_msgs::msg::String>(
            "status", 10,
            std::bind(&SubscriberNode::status_callback, this, std::placeholders::_1));
            
        laser_subscription_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
            "laser_scan", 10,
            std::bind(&SubscriberNode::laser_callback, this, std::placeholders::_1));
            
        // Create publisher for processed data
        obstacle_publisher_ = this->create_publisher<std_msgs::msg::String>("obstacle_alert", 10);
        
        RCLCPP_INFO(this->get_logger(), "Subscriber Node initialized");
    }

private:
    void status_callback(const std_msgs::msg::String::SharedPtr msg)
    {
        RCLCPP_INFO(this->get_logger(), "Status: %s", msg->data.c_str());
    }
    
    void laser_callback(const sensor_msgs::msg::LaserScan::SharedPtr msg)
    {
        // Process laser scan data to detect obstacles
        float min_distance = std::numeric_limits<float>::max();
        for (const auto& range : msg->ranges) {
            if (range < min_distance && range >= msg->range_min && range <= msg->range_max) {
                min_distance = range;
            }
        }
        
        if (min_distance < 1.0) { // If obstacle is closer than 1 meter
            auto alert_msg = std_msgs::msg::String();
            alert_msg.data = "Obstacle detected at " + std::to_string(min_distance) + " meters";
            obstacle_publisher_->publish(alert_msg);
        }
    }
    
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr status_subscription_;
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr laser_subscription_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr obstacle_publisher_;
};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<SubscriberNode>());
    rclcpp::shutdown();
    return 0;
}
```

## Service Server Template (C++)

```cpp
#include "rclcpp/rclcpp.hpp"
#include "example_interfaces/srv/add_two_ints.hpp"

class ServiceServer : public rclcpp::Node
{
public:
    ServiceServer() : Node("service_server")
    {
        // Create service server
        service_ = this->create_service<example_interfaces::srv::AddTwoInts>(
            "add_two_ints",
            std::bind(&ServiceServer::handle_request, this, 
                     std::placeholders::_1, std::placeholders::_2));
        
        RCLCPP_INFO(this->get_logger(), "Service Server initialized");
    }

private:
    void handle_request(
        const std::shared_ptr<rmw_request_id_t> request_header,
        const std::shared_ptr<example_interfaces::srv::AddTwoInts::Request> request,
        std::shared_ptr<example_interfaces::srv::AddTwoInts::Response> response)
    {
        (void)request_header;
        response->sum = request->a + request->b;
        
        RCLCPP_INFO(this->get_logger(), 
                   "Received request: %ld + %ld = %ld", 
                   request->a, request->b, response->sum);
    }
    
    rclcpp::Service<example_interfaces::srv::AddTwoInts>::SharedPtr service_;
};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<ServiceServer>());
    rclcpp::shutdown();
    return 0;
}
```

## Service Client Template (C++)

```cpp
#include "rclcpp/rclcpp.hpp"
#include "example_interfaces/srv/add_two_ints.hpp"

class ServiceClient : public rclcpp::Node
{
public:
    ServiceClient() : Node("service_client")
    {
        client_ = this->create_client<example_interfaces::srv::AddTwoInts>("add_two_ints");
        
        // Wait for service to become available
        while (!client_->wait_for_service(std::chrono::seconds(1))) {
            if (!rclcpp::ok()) {
                RCLCPP_ERROR(this->get_logger(), "Interrupted while waiting for the service. Exiting.");
                return;
            }
            RCLCPP_INFO(this->get_logger(), "Service not available, waiting again...");
        }
        
        // Send request after a short delay
        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(1000),
            std::bind(&ServiceClient::send_request, this));
    }

private:
    void send_request()
    {
        auto request = std::make_shared<example_interfaces::srv::AddTwoInts::Request>();
        request->a = 42;
        request->b = 18;
        
        auto future = client_->async_send_request(request);
        future.wait();
        
        RCLCPP_INFO(this->get_logger(), 
                   "Result of %ld + %ld = %ld", 
                   request->a, request->b, future.get()->sum);
    }
    
    rclcpp::Client<example_interfaces::srv::AddTwoInts>::SharedPtr client_;
    rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<ServiceClient>());
    rclcpp::shutdown();
    return 0;
}
```

## Launch File Template

```python
# launch/robot_exercise.launch.py
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    # Declare launch arguments
    use_sim_time = LaunchConfiguration('use_sim_time', default='false')
    
    return LaunchDescription([
        # Declare launch arguments
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='false',
            description='Use simulation (Gazebo) clock if true'
        ),
        
        # Publisher node
        Node(
            package='robot_exercise_package',
            executable='publisher_node',
            name='publisher_node',
            parameters=[
                {'use_sim_time': use_sim_time}
            ],
            output='screen'
        ),
        
        # Subscriber node
        Node(
            package='robot_exercise_package',
            executable='subscriber_node',
            name='subscriber_node',
            parameters=[
                {'use_sim_time': use_sim_time}
            ],
            output='screen'
        ),
        
        # Service server
        Node(
            package='robot_exercise_package',
            executable='service_server',
            name='service_server',
            parameters=[
                {'use_sim_time': use_sim_time}
            ],
            output='screen'
        ),
    ])
```

## Python Package Template

For Python-based packages, the structure is slightly different:

```
robot_exercise_py/
├── package.xml
├── setup.py
├── setup.cfg
├── resource/robot_exercise_py
├── robot_exercise_py/
│   ├── __init__.py
│   ├── publisher_node.py
│   ├── subscriber_node.py
│   └── service_server.py
├── launch/
│   └── robot_exercise_py.launch.py
└── test/
    └── test_copyright.py
```

### setup.py for Python Package

```python
from setuptools import setup
import os
from glob import glob

package_name = 'robot_exercise_py'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Your Name',
    maintainer_email='your.email@example.com',
    description='Python template for robot exercises',
    license='Apache License 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'publisher_node = robot_exercise_py.publisher_node:main',
            'subscriber_node = robot_exercise_py.subscriber_node:main',
            'service_server = robot_exercise_py.service_server:main',
        ],
    },
)
```

### Python Publisher Node Template

```python
#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from geometry_msgs.msg import Twist
import time

class PublisherNode(Node):
    def __init__(self):
        super().__init__('publisher_node')
        
        # Create publishers
        self.cmd_vel_publisher = self.create_publisher(Twist, 'cmd_vel', 10)
        self.status_publisher = self.create_publisher(String, 'status', 10)
        
        # Create timer for periodic publishing
        timer_period = 0.1  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
        
        self.i = 0
        self.get_logger().info('Publisher Node initialized')

    def timer_callback(self):
        # Create and publish velocity command
        cmd_vel_msg = Twist()
        cmd_vel_msg.linear.x = 0.5  # Move forward at 0.5 m/s
        cmd_vel_msg.angular.z = 0.2  # Turn left at 0.2 rad/s
        self.cmd_vel_publisher.publish(cmd_vel_msg)
        
        # Publish status message
        status_msg = String()
        status_msg.data = f'Robot moving - {self.i}'
        self.status_publisher.publish(status_msg)
        
        self.i += 1

def main(args=None):
    rclpy.init(args=args)
    node = PublisherNode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Node stopped by user')
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
```

## Parameter Configuration Template

```yaml
# config/parameters.yaml
/**:
  ros__parameters:
    robot_name: "exercise_robot"
    max_linear_speed: 1.0
    max_angular_speed: 1.5
    safety_distance: 0.5
    update_rate: 10.0
```

## Testing Template

```cpp
// test/test_robot_nodes.cpp
#include "gtest/gtest.h"
#include "rclcpp/rclcpp.hpp"
#include "robot_exercise_package/publisher_node.hpp"

TEST(PublisherNodeTest, InitializationTest) {
    rclcpp::init(0, nullptr);
    auto node = std::make_shared<PublisherNode>();
    EXPECT_NE(node, nullptr);
    rclcpp::shutdown();
}

int main(int argc, char ** argv) {
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
```

## Exercise Templates

### Exercise 1: Basic Publisher-Subscriber
Template for creating a simple publisher-subscriber pair that demonstrates basic communication.

### Exercise 2: Sensor Processing
Template for processing sensor data and publishing processed information.

### Exercise 3: Navigation Stack
Template for implementing a basic navigation system with path planning and execution.

### Exercise 4: Manipulator Control
Template for controlling a robotic manipulator with inverse kinematics.

### Exercise 5: Multi-Robot Coordination
Template for coordinating multiple robots with message passing.

## Best Practices for Templates

### 1. Modularity
- Separate concerns into different nodes
- Use interfaces for loose coupling
- Make components reusable

### 2. Configuration
- Use parameters for configurable values
- Separate configuration from code
- Use YAML files for complex configurations

### 3. Error Handling
- Implement proper error handling
- Use appropriate logging levels
- Gracefully handle exceptions

### 4. Performance
- Optimize message frequency
- Use appropriate QoS settings
- Consider computational complexity

### 5. Documentation
- Include README files
- Document parameters and interfaces
- Provide usage examples

## Using the Templates

### 1. Copy the Template
Copy the appropriate template directory to your workspace:

```bash
cp -r robot_exercise_package ~/ros2_ws/src/
```

### 2. Customize the Package
- Update package.xml with your information
- Modify node names and functionality
- Add your specific logic

### 3. Build the Package
```bash
cd ~/ros2_ws
colcon build --packages-select robot_exercise_package
source install/setup.bash
```

### 4. Run the Package
```bash
# Run individual nodes
ros2 run robot_exercise_package publisher_node
ros2 run robot_exercise_package subscriber_node

# Or run with launch file
ros2 launch robot_exercise_package robot_exercise.launch.py
```

## Advanced Templates

### Action Server Template
For implementing action servers that handle long-running tasks:

```cpp
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "example_interfaces/action/fibonacci.hpp"

class FibonacciActionServer : public rclcpp::Node
{
public:
    using Fibonacci = example_interfaces::action::Fibonacci;
    using GoalHandleFibonacci = rclcpp_action::ServerGoalHandle<Fibonacci>;

    FibonacciActionServer() : Node("fibonacci_action_server")
    {
        using namespace std::placeholders;

        action_server_ = rclcpp_action::create_server<Fibonacci>(
            this->get_node_base_interface(),
            this->get_node_clock_interface(),
            this->get_node_logging_interface(),
            this->get_node_waitables_interface(),
            "fibonacci",
            std::bind(&FibonacciActionServer::handle_goal, this, _1, _2),
            std::bind(&FibonacciActionServer::handle_cancel, this, _1),
            std::bind(&FibonacciActionServer::handle_accepted, this, _1));
    }

private:
    rclcpp_action::GoalResponse handle_goal(
        const rclcpp_action::GoalUUID & uuid,
        std::shared_ptr<const Fibonacci::Goal> goal)
    {
        RCLCPP_INFO(this->get_logger(), "Received goal request with order %d", goal->order);
        (void)uuid;
        return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
    }

    // Additional methods for action handling...
};
```

## Summary

These templates provide a solid foundation for creating ROS 2 packages and projects. They include:

- Proper package structure and configuration
- Examples for all communication patterns
- Launch files for easy execution
- Parameter configuration
- Testing frameworks
- Best practices for development

Using these templates will help you get started quickly with your ROS 2 exercises and projects while following established best practices.

## Further Reading

- ROS 2 Package Creation: https://docs.ros.org/en/rolling/Tutorials/Beginner-Client-Libraries/Creating-Your-First-ROS2-Package.html
- Launch Files: https://docs.ros.org/en/rolling/Tutorials/Intermediate/Launch/Creating-Launch-Files.html
- Testing: https://docs.ros.org/en/rolling/Tutorials/Beginner-Client-Libraries/Unit-Testing-CMake.html

## Assessment

These templates are designed to support your learning of ROS 2 development. Use them to create your own packages and implement the exercises in this course.