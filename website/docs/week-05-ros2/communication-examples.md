---
title: ROS 2 Communication Patterns
sidebar_label: ROS 2 Communication Patterns
description: Understanding and implementing ROS 2 communication patterns for robotic systems
keywords: [ros2, communication, patterns, topics, services, actions, robotics]
---

## Learning Objectives

- Understand the different communication patterns in ROS 2
- Implement publish-subscribe communication using topics
- Create and use services for request-response communication
- Implement actions for long-running tasks with feedback
- Design effective communication architectures for robotic systems
- Apply Quality of Service (QoS) policies appropriately

## Introduction

In this module, we'll explore the communication patterns that form the backbone of ROS 2 systems. ROS 2 provides three primary communication patterns: topics (publish-subscribe), services (request-response), and actions (goal-feedback-result). Understanding these patterns is crucial for designing effective robotic systems that can coordinate complex behaviors.

Communication is the foundation of any distributed robotic system. Whether it's a simple robot with multiple sensors and actuators or a complex multi-robot system, effective communication patterns are essential for coordinating behavior and sharing information.

## Overview of ROS 2 Communication Patterns

ROS 2 provides three main communication patterns:

1. **Topics**: Asynchronous, one-way communication using the publish-subscribe pattern
2. **Services**: Synchronous, two-way communication using the request-response pattern
3. **Actions**: Asynchronous communication for long-running tasks with feedback

Each pattern serves different purposes and has specific use cases in robotic systems.

## Topics: Publish-Subscribe Pattern

### Concept

Topics use the publish-subscribe communication pattern, which is asynchronous and allows for loose coupling between nodes. Publishers send messages to topics, and subscribers receive messages from topics. Multiple publishers and subscribers can exist for the same topic.

### Key Characteristics

- **Asynchronous**: Publishers and subscribers don't need to run simultaneously
- **Loose coupling**: Publishers don't know who subscribes to their topics
- **Broadcast**: One message can go to multiple subscribers
- **Unidirectional**: Communication flows in one direction

### Use Cases

- Sensor data (LIDAR scans, camera images, IMU readings)
- Robot state (position, velocity, battery level)
- Commands (velocity commands, joint positions)
- Status updates (diagnostics, error messages)

### Implementation in C++

```cpp
// Publisher implementation
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

class TopicPublisher : public rclcpp::Node
{
public:
    TopicPublisher() : Node("topic_publisher")
    {
        publisher_ = this->create_publisher<std_msgs::msg::String>("topic_name", 10);
        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(500),
            std::bind(&TopicPublisher::timer_callback, this));
    }

private:
    void timer_callback()
    {
        auto message = std_msgs::msg::String();
        message.data = "Hello, ROS 2! " + std::to_string(count_++);
        RCLCPP_INFO(this->get_logger(), "Publishing: '%s'", message.data.c_str());
        publisher_->publish(message);
    }
    
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_;
    size_t count_ = 0;
};

// Subscriber implementation
class TopicSubscriber : public rclcpp::Node
{
public:
    TopicSubscriber() : Node("topic_subscriber")
    {
        subscription_ = this->create_subscription<std_msgs::msg::String>(
            "topic_name", 10,
            std::bind(&TopicSubscriber::topic_callback, this, std::placeholders::_1));
    }

private:
    void topic_callback(const std_msgs::msg::String::SharedPtr msg)
    {
        RCLCPP_INFO(this->get_logger(), "I heard: '%s'", msg->data.c_str());
    }
    
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr subscription_;
};
```

### Implementation in Python

```python
import rclpy
from rclpy.node import Node
from std_msgs.msg import String

class TopicPublisher(Node):
    def __init__(self):
        super().__init__('topic_publisher')
        self.publisher_ = self.create_publisher(String, 'topic_name', 10)
        timer_period = 0.5  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.i = 0

    def timer_callback(self):
        msg = String()
        msg.data = f'Hello ROS 2: {self.i}'
        self.publisher_.publish(msg)
        self.get_logger().info(f'Publishing: "{msg.data}"')
        self.i += 1

class TopicSubscriber(Node):
    def __init__(self):
        super().__init__('topic_subscriber')
        self.subscription = self.create_subscription(
            String,
            'topic_name',
            self.topic_callback,
            10)
        self.subscription  # prevent unused variable warning

    def topic_callback(self, msg):
        self.get_logger().info(f'I heard: "{msg.data}"')
```

## Services: Request-Response Pattern

### Concept

Services provide synchronous, two-way communication using the request-response pattern. A service client sends a request to a service server and waits for a response. This pattern is synchronous and blocking.

### Key Characteristics

- **Synchronous**: Client waits for response
- **Two-way**: Request and response
- **Blocking**: Client is blocked until response received
- **Reliable**: Request is guaranteed to reach server (with appropriate QoS)

### Use Cases

- Configuration requests (set parameters, change modes)
- One-time calculations (path planning, inverse kinematics)
- Command execution (move to position, take picture)
- Data queries (get robot pose, get map)

### Service Definition

Services require definition files (.srv) that specify the request and response:

```
# Request
string command
int32 target_value
---
# Response
bool success
string message
int32 result
```

### Implementation in C++

```cpp
// Service server
#include "rclcpp/rclcpp.hpp"
#include "example_interfaces/srv/add_two_ints.hpp"

class ServiceServer : public rclcpp::Node
{
public:
    ServiceServer() : Node("service_server")
    {
        service_ = this->create_service<example_interfaces::srv::AddTwoInts>(
            "add_two_ints",
            std::bind(&ServiceServer::handle_request, this, 
                     std::placeholders::_1, std::placeholders::_2));
    }

private:
    void handle_request(
        const std::shared_ptr<rmw_request_id_t> request_header,
        const std::shared_ptr<example_interfaces::srv::AddTwoInts::Request> request,
        std::shared_ptr<example_interfaces::srv::AddTwoInts::Response> response)
    {
        (void)request_header;
        response->sum = request->a + request->b;
        RCLCPP_INFO(this->get_logger(), "Incoming request\na: %ld" " b: %ld",
                    request->a, request->b);
        RCLCPP_INFO(this->get_logger(), "Sending back response: [%ld]", (long int)response->sum);
    }
    
    rclcpp::Service<example_interfaces::srv::AddTwoInts>::SharedPtr service_;
};

// Service client
class ServiceClient : public rclcpp::Node
{
public:
    ServiceClient() : Node("service_client")
    {
        client_ = this->create_client<example_interfaces::srv::AddTwoInts>("add_two_ints");
        
        while (!client_->wait_for_service(std::chrono::seconds(1))) {
            if (!rclcpp::ok()) {
                RCLCPP_ERROR(this->get_logger(), "Interrupted while waiting for the service. Exiting.");
                return;
            }
            RCLCPP_INFO(this->get_logger(), "Service not available, waiting again...");
        }
    }
    
    void send_request()
    {
        auto request = std::make_shared<example_interfaces::srv::AddTwoInts::Request>();
        request->a = 2;
        request->b = 3;
        
        auto result_future = client_->async_send_request(request);
        // Wait for the result
        if (rclcpp::spin_until_future_complete(this->get_node_base_interface(), result_future) ==
            rclcpp::FutureReturnCode::SUCCESS)
        {
            RCLCPP_INFO(this->get_logger(), "Result of add_two_ints: %ld",
                        result_future.get()->sum);
        } else {
            RCLCPP_ERROR(this->get_logger(), "Failed to call service add_two_ints");
        }
    }

private:
    rclcpp::Client<example_interfaces::srv::AddTwoInts>::SharedPtr client_;
};
```

### Implementation in Python

```python
import rclpy
from rclpy.node import Node
from example_interfaces.srv import AddTwoInts

class ServiceServer(Node):
    def __init__(self):
        super().__init__('service_server')
        self.srv = self.create_service(
            AddTwoInts, 
            'add_two_ints', 
            self.add_two_ints_callback
        )

    def add_two_ints_callback(self, request, response):
        response.sum = request.a + request.b
        self.get_logger().info(f'Incoming request\na: {request.a} b: {request.b}')
        return response

class ServiceClient(Node):
    def __init__(self):
        super().__init__('service_client')
        self.cli = self.create_client(AddTwoInts, 'add_two_ints')
        
        while not self.cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Service not available, waiting again...')
        
        self.req = AddTwoInts.Request()

    def send_request(self):
        self.req.a = 41
        self.req.b = 1
        future = self.cli.call_async(self.req)
        rclpy.spin_until_future_complete(self, future)
        return future.result()
```

## Actions: Goal-Feedback-Result Pattern

### Concept

Actions provide asynchronous communication for long-running tasks that may provide feedback and can be preempted. Actions are ideal for tasks that take time to complete and need to provide intermediate status updates.

### Key Characteristics

- **Asynchronous**: Non-blocking communication
- **Long-running**: Designed for tasks that take time
- **Feedback**: Provides intermediate status updates
- **Preemption**: Goals can be canceled or replaced
- **Result**: Final outcome of the action

### Use Cases

- Navigation (move to goal with feedback)
- Manipulation (grasp object with status)
- Calibration (process with progress updates)
- Any task that takes significant time

### Action Definition

Actions require definition files (.action) that specify goal, result, and feedback:

```
# Goal
float64 target_pose
---
# Result
bool success
string message
---
# Feedback
float64 current_pose
float64 distance_remaining
```

### Implementation in C++

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

    rclcpp_action::CancelResponse handle_cancel(
        const std::shared_ptr<GoalHandleFibonacci> goal_handle)
    {
        RCLCPP_INFO(this->get_logger(), "Received request to cancel goal");
        (void)goal_handle;
        return rclcpp_action::CancelResponse::ACCEPT;
    }

    void handle_accepted(const std::shared_ptr<GoalHandleFibonacci> goal_handle)
    {
        using namespace std::placeholders;
        // This needs to return quickly to avoid blocking the executor, so spin up a new thread
        std::thread{std::bind(&FibonacciActionServer::execute, this, _1), goal_handle}.detach();
    }

    void execute(const std::shared_ptr<GoalHandleFibonacci> goal_handle)
    {
        RCLCPP_INFO(this->get_logger(), "Executing goal");
        
        rclcpp::Rate loop_rate(1);
        const auto goal = goal_handle->get_goal();
        auto feedback = std::make_shared<Fibonacci::Feedback>();
        auto result = std::make_shared<Fibonacci::Result>();

        // Start executing the action
        auto sequence = Fibonacci::Result().sequence;
        sequence.push_back(0);
        sequence.push_back(1);

        for (int i = 1; (i < goal->order) && rclcpp::ok(); ++i) {
            // Check if there is a cancel request
            if (goal_handle->is_canceling()) {
                result->sequence = sequence;
                goal_handle->canceled(result);
                RCLCPP_INFO(this->get_logger(), "Goal canceled");
                return;
            }
            
            // Update sequence
            sequence.push_back(sequence[i] + sequence[i - 1]);
            
            // Publish feedback
            feedback->sequence = sequence;
            goal_handle->publish_feedback(feedback);
            RCLCPP_INFO(this->get_logger(), "Publishing feedback");

            loop_rate.sleep();
        }

        // Check if goal is done
        if (rclcpp::ok()) {
            result->sequence = sequence;
            goal_handle->succeed(result);
            RCLCPP_INFO(this->get_logger(), "Goal succeeded");
        }
    }

    rclcpp_action::Server<Fibonacci>::SharedPtr action_server_;
};
```

## Quality of Service (QoS) Policies

### Concept

QoS policies allow you to configure the behavior of your ROS 2 communications. They provide fine-grained control over reliability, durability, and other aspects of communication.

### Key QoS Policies

1. **Reliability**: Ensures message delivery
   - `RMW_QOS_POLICY_RELIABILITY_RELIABLE`: All messages delivered
   - `RMW_QOS_POLICY_RELIABILITY_BEST_EFFORT`: Best effort delivery

2. **Durability**: Persistence of messages
   - `RMW_QOS_POLICY_DURABILITY_TRANSIENT_LOCAL`: Store messages for late joiners
   - `RMW_QOS_POLICY_DURABILITY_VOLATILE`: Don't store messages

3. **History**: How many messages to store
   - `RMW_QOS_POLICY_HISTORY_KEEP_LAST`: Store N last messages
   - `RMW_QOS_POLICY_HISTORY_KEEP_ALL`: Store all messages

4. **Depth**: Number of messages to store (for KEEP_LAST)

### Implementation with QoS

```cpp
// Example with custom QoS
rclcpp::QoS qos_profile = rclcpp::QoS(10)  // history depth of 10
    .reliability(RMW_QOS_POLICY_RELIABILITY_RELIABLE)
    .durability(RMW_QOS_POLICY_DURABILITY_VOLATILE);

auto publisher = this->create_publisher<std_msgs::msg::String>("topic_name", qos_profile);

auto subscription = this->create_subscription<std_msgs::msg::String>(
    "topic_name", 
    qos_profile,
    std::bind(&MyNode::callback, this, std::placeholders::_1));
```

## Communication Architecture Patterns

### 1. Sensor-Processing-Actuator Pattern

```
Sensors → Processing Nodes → Actuators
```
- Sensors publish data to topics
- Processing nodes subscribe, process, and publish commands
- Actuator nodes subscribe to commands and execute

### 2. Coordinator Pattern

```
    → Node 1
Coordinator → Node 2
    → Node 3
```
- Coordinator node manages multiple specialized nodes
- Uses services for configuration and actions for tasks

### 3. Distributed Pattern

```
Node A ↔ Node B ↔ Node C
  ↓       ↓       ↓
Sensors  Process  Actuators
```
- Nodes communicate directly with each other
- Decentralized decision making

## Best Practices for Communication Design

### 1. Choose the Right Pattern

- Use topics for continuous data streams (sensors, status)
- Use services for one-time requests (configuration, computation)
- Use actions for long-running tasks (navigation, manipulation)

### 2. Message Design

- Keep messages small and efficient
- Use appropriate data types
- Consider bandwidth and processing requirements

### 3. QoS Configuration

- Use reliable QoS for critical data
- Use best-effort for high-frequency, less critical data
- Use transient-local for configuration parameters

### 4. Naming Conventions

- Use descriptive topic/service/action names
- Follow ROS 2 naming conventions
- Group related topics with prefixes (e.g., `/arm/joint_states`, `/arm/commands`)

## Advanced Communication Concepts

### 1. Composition

Nodes can be composed within a single process to reduce communication overhead:

```cpp
// Composing nodes in a single process
#include "rclcpp/rclcpp.hpp"
#include "my_package/publisher_node.hpp"
#include "my_package/subscriber_node.hpp"

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    
    auto executor = std::make_shared<rclcpp::executors::MultiThreadedExecutor>();
    
    auto pub_node = std::make_shared<PublisherNode>();
    auto sub_node = std::make_shared<SubscriberNode>();
    
    executor->add_node(pub_node);
    executor->add_node(sub_node);
    
    executor->spin();
    
    rclcpp::shutdown();
    return 0;
}
```

### 2. Intra-Process Communication

When nodes are in the same process, ROS 2 can optimize communication:

```cpp
// Enable intra-process communication
auto options = rclcpp::NodeOptions()
    .use_intra_process_comms(true);

auto node = std::make_shared<rclcpp::Node>("my_node", options);
```

## Troubleshooting Communication Issues

### Common Issues

1. **Topic not connecting**: Check topic names and namespaces
2. **High latency**: Check QoS settings and network configuration
3. **Message loss**: Use reliable QoS or check system performance
4. **Node discovery**: Ensure nodes are on the same ROS domain

### Debugging Tools

```bash
# Check topics
ros2 topic list
ros2 topic info /topic_name
ros2 topic echo /topic_name

# Check services
ros2 service list
ros2 service info /service_name
ros2 service call /service_name service_type "{request: value}"

# Check actions
ros2 action list
ros2 action info /action_name
```

## Summary

This module covered the three primary communication patterns in ROS 2:

- **Topics**: For asynchronous, one-way communication using publish-subscribe
- **Services**: For synchronous, two-way communication using request-response
- **Actions**: For asynchronous, long-running tasks with feedback and preemption

We also explored Quality of Service policies that allow fine-tuning communication behavior and discussed best practices for designing effective communication architectures in robotic systems.

Understanding these communication patterns is crucial for developing complex robotic systems that can coordinate multiple components effectively. In the next modules, we'll apply these concepts to practical scenarios and explore advanced topics in ROS 2 development.

## Further Reading

- ROS 2 Communication: https://docs.ros.org/en/rolling/Concepts/About-Topics.html
- Quality of Service: https://docs.ros.org/en/rolling/Concepts/About-Quality-of-Service.html
- Actions: https://docs.ros.org/en/rolling/Tutorials/Intermediate/Writing-an-Action-Server-Cpp.html
- Services: https://docs.ros.org/en/rolling/Tutorials/Services/Understanding-ROS2-Services.html

## Assessment

Complete the assessment for this module to track your progress.