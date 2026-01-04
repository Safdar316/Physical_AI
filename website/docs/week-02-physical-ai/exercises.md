---
title: Physical AI Exercises
sidebar_label: Physical AI Exercises
description: Practical exercises to reinforce Physical AI concepts and principles
keywords: [robotics, ai, physical ai, exercises, practice, humanoid]
---

## Learning Objectives

- Apply Physical AI concepts to practical scenarios
- Implement basic Physical AI algorithms
- Analyze the impact of physical constraints on robotic behavior
- Design solutions considering physical laws and embodiment

## Introduction

This module provides practical exercises to reinforce the Physical AI concepts covered in Week 2. These exercises combine theoretical understanding with practical implementation, helping you apply Physical AI principles to real-world scenarios.

The exercises are designed to deepen your understanding of how physical laws, embodiment, and environmental interaction influence intelligent behavior in robotic systems. Each exercise builds on the concepts from previous modules and provides hands-on experience with Physical AI implementations.

## Exercise 1: Center of Mass Calculation

### Objective
Calculate and visualize the center of mass for a simple robot configuration.

### Background
Understanding the center of mass (COM) is crucial for robot stability, especially for humanoid robots that must maintain balance while performing tasks.

### Exercise
Consider a simplified humanoid robot model with the following parameters:
- Body mass: 30 kg, located at (0, 1.0, 0)
- Left arm: 5 kg, located at (-0.5, 1.2, 0)
- Right arm: 5 kg, located at (0.5, 1.2, 0)
- Left leg: 10 kg, located at (-0.2, 0.2, 0)
- Right leg: 10 kg, located at (0.2, 0.2, 0)

Calculate the overall center of mass of the robot using the formula:
COM = Σ(m_i * r_i) / Σm_i

Where m_i is the mass of each component and r_i is its position vector.

### Solution Approach
1. Calculate the numerator (sum of mass × position for each component)
2. Calculate the total mass
3. Divide the numerator by the total mass to get the COM coordinates

### Discussion Questions
1. How would moving the arms affect the COM?
2. What happens to the COM when the robot lifts one leg?
3. How does the COM relate to the support polygon for balance?

## Exercise 2: Forward Kinematics Implementation

### Objective
Implement forward kinematics for a simple 2D robotic arm.

### Background
Forward kinematics calculates the end-effector position given joint angles. This is fundamental for robot motion planning.

### Exercise
Create a Python function that calculates the end-effector position for a 2-link planar robotic arm with the following parameters:
- Link 1 length: L1 = 0.5 m
- Link 2 length: L2 = 0.4 m
- Joint angles: θ1, θ2 (in radians)

The forward kinematics equations are:
- x = L1 * cos(θ1) + L2 * cos(θ1 + θ2)
- y = L1 * sin(θ1) + L2 * sin(θ1 + θ2)

### Implementation
```python
import math

def forward_kinematics(theta1, theta2, l1=0.5, l2=0.4):
    """
    Calculate end-effector position for a 2-link planar arm
    Args:
        theta1: angle of first joint (radians)
        theta2: angle of second joint (radians)
        l1: length of first link (default 0.5m)
        l2: length of second link (default 0.4m)
    Returns:
        tuple: (x, y) end-effector position
    """
    x = l1 * math.cos(theta1) + l2 * math.cos(theta1 + theta2)
    y = l1 * math.sin(theta1) + l2 * math.sin(theta1 + theta2)
    return (x, y)

# Example usage:
# Calculate end-effector position for θ1=π/4, θ2=π/6
position = forward_kinematics(math.pi/4, math.pi/6)
print(f"End-effector position: {position}")
```

### Exercise Tasks
1. Calculate the end-effector position for θ1 = π/4 and θ2 = π/6
2. Plot the workspace of the robotic arm by calculating positions for various joint angles
3. Identify the reachable workspace of the arm

### Discussion Questions
1. What happens when θ2 = 0 (arm fully extended)?
2. How does the workspace change if L1 > L2 or L2 > L1?
3. What are the limitations of this simple kinematic model?

## Exercise 3: Friction and Grasping

### Objective
Analyze the role of friction in robotic grasping.

### Background
Friction is essential for robotic grasping. The friction coefficient determines whether a robot can securely hold an object without it slipping.

### Exercise
A robotic gripper is attempting to lift a cylindrical object with mass m = 0.5 kg. The gripper applies a normal force F_normal = 10 N on each side of the object. The coefficient of static friction between the gripper and the object is μ = 0.7.

Calculate whether the object will slip when lifted.

### Solution Approach
1. Calculate the maximum friction force possible: F_friction_max = μ * F_normal_total
2. Calculate the gravitational force on the object: F_gravity = m * g
3. Compare the forces to determine if the grip is sufficient

### Detailed Solution
1. Total normal force: F_normal_total = 2 * 10 N = 20 N (gripper applies force on both sides)
2. Maximum friction force: F_friction_max = 0.7 * 20 N = 14 N
3. Gravitational force: F_gravity = 0.5 kg * 9.81 m/s² = 4.905 N
4. Since F_friction_max > F_gravity, the object will not slip

### Exercise Tasks
1. Determine the minimum coefficient of friction required to lift the object
2. Calculate the maximum mass the gripper can lift with the current normal force and friction coefficient
3. Analyze how the required normal force changes with different friction coefficients

### Discussion Questions
1. How would the required normal force change for a different object shape?
2. What happens if the coefficient of friction is too low?
3. How do real-world factors like surface contamination affect friction?

## Exercise 4: Balance Analysis for a Simple Robot

### Objective
Analyze the balance of a simplified humanoid robot model.

### Background
Humanoid robots must maintain their center of mass within their support polygon to remain stable. This exercise analyzes the balance of a simple model.

### Exercise
Consider a humanoid robot with:
- Total mass: 50 kg
- COM height: 0.8 m above the ground
- Foot separation: 0.3 m (feet centered at (-0.15, 0) and (0.15, 0))
- Current COM position: (0.05, 0.8, 0)

Determine if the robot is stable and calculate the maximum lean angle before tipping.

### Solution Approach
1. Determine the support polygon (area between the feet)
2. Check if the COM projection onto the ground is within the support polygon
3. Calculate the maximum lean angle before the COM projection reaches the edge of the support polygon

### Detailed Solution
1. Support polygon: x coordinates from -0.15 to 0.15 (since feet are at x=-0.15 and x=0.15)
2. COM projection on ground: (0.05, 0) - this is within the support polygon since -0.15 < 0.05 < 0.15
3. The robot is stable
4. Maximum lean to the right before tipping: when COM projection reaches x=0.15
   Maximum lean angle θ = arctan((0.15-0.05)/0.8) = arctan(0.1/0.8) ≈ 7.13°

### Exercise Tasks
1. Calculate the maximum lean to the left before tipping
2. Determine how the stability changes if the robot stands with feet together (0.0m separation)
3. Analyze the effect of COM height on stability

### Discussion Questions
1. How does the robot's stability change during walking?
2. What strategies can robots use to maintain balance during movement?
3. How does this simple model differ from real humanoid robots?

## Exercise 5: Sim-to-Real Considerations

### Objective
Analyze factors that contribute to the sim-to-real gap in robotics.

### Background
The sim-to-real gap refers to the difference in performance between simulation and real-world deployment. Understanding these factors is crucial for Physical AI implementation.

### Exercise
Consider a robot trained in simulation to navigate a simple maze and identify potential sim-to-real gaps.

### Factors to Consider
1. **Physical Properties**
   - Mass, friction, and inertial properties may differ
   - Real-world objects have imperfections not captured in simulation

2. **Sensor Noise**
   - Real sensors have noise, latency, and limited precision
   - Simulated sensors often provide perfect information

3. **Actuator Imperfections**
   - Real actuators have delays, limited precision, and wear
   - Simulated actuators often respond instantly and perfectly

4. **Environmental Factors**
   - Lighting, temperature, and other conditions vary in reality
   - Simulations often use idealized conditions

### Exercise Tasks
1. Identify at least 5 specific sim-to-real gaps for the maze navigation task
2. Propose solutions for each gap identified
3. Discuss how domain randomization might help address these gaps

### Example Solution
1. **Gap**: Wheel slippage not modeled in simulation
   **Solution**: Add wheel slippage models to simulation

2. **Gap**: Camera sensor noise not simulated
   **Solution**: Add realistic noise models to camera simulation

3. **Gap**: Surface friction variations not captured
   **Solution**: Randomize surface friction in simulation

4. **Gap**: Robot wear and tear not simulated
   **Solution**: Model degradation over time in simulation

5. **Gap**: External disturbances not included
   **Solution**: Add random external forces to simulation

### Discussion Questions
1. How does the complexity of the task affect the sim-to-real gap?
2. What is the trade-off between simulation accuracy and computational efficiency?
3. How can real-world data be used to improve simulation accuracy?

## Exercise 6: Embodied Intelligence Application

### Objective
Design a simple embodied system and analyze how its physical form influences its behavior.

### Background
Embodied intelligence emphasizes how the physical form of a system influences its intelligent behavior. This exercise explores this concept through design.

### Exercise
Design a simple physical system (e.g., a wheeled robot, a mechanical walker, or a soft robot) and analyze how its physical properties contribute to its behavior.

### Design Example: Passive Dynamic Walker
Consider a simple walker with legs but no active actuators for leg movement.

**Physical Properties**:
- Leg length: L = 0.5 m
- Mass distribution
- Hip joint design
- Foot design

**Embodied Behaviors**:
- Stable walking emerges from physical design without active control
- Walks stably down a slight incline using gravity
- Gait emerges from interaction of physical structure with environment

### Exercise Tasks
1. Design your own simple embodied system
2. Identify at least 3 ways the physical form influences behavior
3. Compare with a purely computational approach to the same task
4. Discuss advantages and limitations of the embodied approach

### Analysis Framework
1. **Physical Properties**: List key physical characteristics
2. **Emergent Behaviors**: Describe behaviors that emerge from physical interaction
3. **Computational Alternatives**: How would a computational approach differ?
4. **Advantages**: What benefits does the embodied approach provide?
5. **Limitations**: What are the limitations of the embodied approach?

### Discussion Questions
1. How does embodiment affect the complexity of the control system?
2. What tasks are better suited for embodied vs. computational approaches?
3. How can embodiment be leveraged in humanoid robot design?

## Exercise 7: Physical Laws in Manipulation

### Objective
Apply Newton's laws of motion to analyze robotic manipulation tasks.

### Background
When robots manipulate objects, they must account for physical laws such as Newton's laws of motion. This exercise explores how these laws apply to manipulation scenarios.

### Exercise
Consider a robot arm attempting to lift a box from a table:

**Scenario Parameters**:
- Box mass: 2 kg
- Coefficient of static friction between box and table: μs = 0.4
- Robot gripper force: 30 N (normal force applied by gripper)
- Coefficient of static friction between gripper and box: μs = 0.6

### Solution Approach
1. Calculate the minimum force needed to overcome static friction and start moving the box across the table
2. Determine if the gripper can securely hold the box without slipping
3. Analyze the forces involved when lifting the box

### Detailed Solution
1. **Friction force on table**: F_friction_table = μs * m * g = 0.4 * 2 * 9.81 = 7.85 N
2. **Maximum friction force from gripper**: F_friction_gripper = μs * F_normal = 0.6 * 30 = 18 N
3. **Since 18 N > (2 * 9.81) N = 19.62 N, the gripper cannot securely hold the box**

### Exercise Tasks
1. Calculate the minimum gripper force needed to securely hold the 2 kg box
2. Determine how the required force changes if the box mass increases to 5 kg
3. Analyze how the required force changes with different friction coefficients

### Discussion Questions
1. How would surface contamination affect the friction coefficients?
2. What other factors should be considered in real manipulation tasks?
3. How does this simple model differ from real-world manipulation?

## Exercise 8: Balance and Center of Mass in Humanoid Robots

### Objective
Analyze the balance of a humanoid robot using center of mass calculations.

### Background
Humanoid robots must maintain their center of mass within their support polygon to remain stable. This exercise applies this principle to a practical scenario.

### Exercise
Consider a humanoid robot reaching forward to pick up an object:

**Robot Parameters**:
- Body mass (torso): 25 kg, centered at (0, 0.8, 0)
- Head mass: 2 kg, located at (0, 1.6, 0)
- Each arm mass: 3 kg, normally at (0.2, 1.2, 0) and (-0.2, 1.2, 0)
- Each leg mass: 8 kg, located at (0.1, 0.4, 0) and (-0.1, 0.4, 0)
- Foot spacing: 0.4 m (feet at x = 0.2 and x = -0.2)

**Scenario**: Robot reaches forward with right arm to x = 0.6 m

### Solution Approach
1. Calculate the initial center of mass (before reaching)
2. Calculate the new center of mass (after reaching)
3. Determine if the robot remains stable

### Detailed Solution
1. **Initial COM calculation**:
   - Total mass = 25+2+(2×3)+(2×8) = 47 kg
   - X_com = (25×0 + 2×0 + 3×0.2 + 3×(-0.2) + 8×0.1 + 8×(-0.1))/47 = 0
   - Y_com = (25×0.8 + 2×1.6 + 3×1.2 + 3×1.2 + 8×0.4 + 8×0.4)/47 = 0.85 m

2. **After reaching, right arm at (0.6, 1.2, 0)**:
   - X_com = (25×0 + 2×0 + 3×0.6 + 3×(-0.2) + 8×0.1 + 8×(-0.1))/47 = 0.0255 m
   - The robot remains stable as the COM projection (0.0255 m) is within the support polygon (-0.2 to 0.2 m)

### Exercise Tasks
1. Calculate how far the robot can reach before becoming unstable
2. Determine how carrying a load in the hand affects stability
3. Analyze how lifting a leg affects the stability

### Discussion Questions
1. How do humanoid robots maintain balance while walking?
2. What control strategies can be used to maintain balance during reaching?
3. How does this model differ from real humanoid robots?

## Summary

These exercises provide hands-on experience with key Physical AI concepts including kinematics, dynamics, friction, balance, sim-to-real considerations, and embodied intelligence. Each exercise connects theoretical concepts to practical applications, helping develop an intuitive understanding of how physical laws and embodiment influence intelligent robotic behavior.

The exercises demonstrate that Physical AI is not just about implementing algorithms, but understanding how physical constraints and properties shape intelligent behavior. This understanding is crucial for developing effective humanoid robots that can interact safely and efficiently with the physical world.

## Further Exercises

1. Implement inverse kinematics for the 2-link arm from Exercise 2
2. Design a control system to maintain balance for the robot in Exercise 4
3. Create a simulation environment to test the grasping concepts from Exercise 3
4. Explore how changing robot morphology affects the behaviors in Exercise 6

## Assessment

Complete the assessment for this module to track your progress.