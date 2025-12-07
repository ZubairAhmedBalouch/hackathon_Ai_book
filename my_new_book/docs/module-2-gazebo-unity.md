---
title: Module 2 - Gazebo and Unity
---

# Module 2 - Gazebo and Unity

## Overview

Simulation environments are crucial for developing, testing, and validating Physical AI systems before deployment on real hardware. This module covers two leading simulation platforms: Gazebo, the standard robotics simulator with realistic physics, and Unity, the powerful game engine adapted for robotics applications. Understanding both platforms provides flexibility in choosing the right tool for different robotics challenges.

Gazebo excels in physics accuracy and robotics-specific features, while Unity offers advanced graphics, VR/AR capabilities, and cross-platform deployment. This module will guide you through both platforms' capabilities, integration with ROS2, and best practices for simulation-based development.

## Key Concepts

- **Physics Simulation**: Accurate modeling of real-world physics including collision detection, friction, and dynamics
- **Sensor Simulation**: Realistic simulation of cameras, LiDAR, IMUs, and other robotic sensors
- **URDF Integration**: Using Unified Robot Description Format for robot models in simulation
- **ROS2 Integration**: Connecting simulation environments with ROS2 for seamless development
- **Simulation-to-Reality Transfer**: Techniques for ensuring behaviors learned in simulation work on real robots
- **Environment Design**: Creating realistic and challenging simulation worlds
- **Performance Optimization**: Balancing simulation fidelity with computational efficiency

## Details

### Gazebo: The Standard Robotics Simulator

Gazebo is the de facto standard for robotics simulation, offering:

**Physics Engine**:
- ODE (Open Dynamics Engine), Bullet, or DART physics backends
- Accurate collision detection and response
- Realistic friction, damping, and contact models
- Support for articulated bodies and complex mechanisms

**Sensor Simulation**:
- Camera sensors with realistic distortion models
- LiDAR with configurable resolution and range
- IMU sensors with noise models
- Force/torque sensors for contact detection
- GPS and other navigation sensors

**ROS2 Integration**:
- Direct integration through gazebo_ros_pkgs
- Standard message types for sensor data
- Robot state publisher for joint information
- TF publishing for coordinate transforms

**Model Database**:
- Pre-built models for robots, objects, and environments
- Fuel server for sharing models and worlds
- Support for custom model creation

### Unity: Game Engine for Robotics

Unity provides unique advantages for robotics simulation:

**Graphics and Visualization**:
- High-quality rendering for realistic visual simulation
- VR/AR support for immersive development
- Advanced lighting and material systems
- Real-time rendering capabilities

**Physics System**:
- PhysX physics engine with good performance
- Customizable collision detection
- Joint systems for articulated robots
- Force application and constraint systems

**ROS2 Integration**:
- Unity Robotics Package for ROS2 communication
- ROS# bridge for message passing
- Standard ROS2 message support
- TF broadcasting for coordinate systems

**Development Tools**:
- Visual editor for scene creation
- C# scripting for custom behaviors
- Asset store with robotics-specific packages
- Cross-platform deployment capabilities

### Simulation-to-Reality Transfer Challenges

Key challenges in transferring from simulation to reality include:

- **Reality Gap**: Differences in physics, sensor noise, and environmental conditions
- **Visual Domain Randomization**: Techniques to make vision systems robust to appearance variations
- **Dynamics Randomization**: Varying physical parameters to improve generalization
- **System Identification**: Measuring real robot parameters for accurate simulation

### Best Practices for Simulation Development

1. **Start Simple**: Begin with basic models and gradually add complexity
2. **Validate Components**: Test individual sensors and actuators before system integration
3. **Monitor Performance**: Track simulation speed and adjust complexity as needed
4. **Document Differences**: Maintain records of simulation vs. reality discrepancies
5. **Iterative Improvement**: Continuously refine models based on real robot performance

## Examples

### Example 1: Gazebo Robot Model (URDF)

**Simple Differential Drive Robot** (diff_drive_robot.urdf):
```xml
<?xml version="1.0"?>
<robot name="diff_drive_robot">
  <!-- Base Link -->
  <link name="base_link">
    <visual>
      <geometry>
        <box size="0.5 0.3 0.15"/>
      </geometry>
      <material name="blue">
        <color rgba="0 0 1 1"/>
      </material>
    </visual>
    <collision>
      <geometry>
        <box size="0.5 0.3 0.15"/>
      </geometry>
    </collision>
    <inertial>
      <mass value="1.0"/>
      <inertia ixx="0.1" ixy="0" ixz="0" iyy="0.1" iyz="0" izz="0.1"/>
    </inertial>
  </link>

  <!-- Left Wheel -->
  <link name="left_wheel">
    <visual>
      <geometry>
        <cylinder radius="0.1" length="0.05"/>
      </geometry>
    </visual>
    <collision>
      <geometry>
        <cylinder radius="0.1" length="0.05"/>
      </geometry>
    </collision>
    <inertial>
      <mass value="0.2"/>
      <inertia ixx="0.001" ixy="0" ixz="0" iyy="0.001" iyz="0" izz="0.002"/>
    </inertial>
  </link>

  <!-- Joints -->
  <joint name="left_wheel_joint" type="continuous">
    <parent link="base_link"/>
    <child link="left_wheel"/>
    <origin xyz="0 0.2 -0.05" rpy="1.5708 0 0"/>
    <axis xyz="0 0 1"/>
  </joint>

  <!-- Right Wheel (similar to left) -->
  <link name="right_wheel">
    <visual>
      <geometry>
        <cylinder radius="0.1" length="0.05"/>
      </geometry>
    </visual>
    <collision>
      <geometry>
        <cylinder radius="0.1" length="0.05"/>
      </geometry>
    </collision>
    <inertial>
      <mass value="0.2"/>
      <inertia ixx="0.001" ixy="0" ixz="0" iyy="0.001" iyz="0" izz="0.002"/>
    </inertial>
  </link>

  <joint name="right_wheel_joint" type="continuous">
    <parent link="base_link"/>
    <child link="right_wheel"/>
    <origin xyz="0 -0.2 -0.05" rpy="1.5708 0 0"/>
    <axis xyz="0 0 1"/>
  </joint>
</robot>
```

### Example 2: Gazebo World File

**Simple Room Environment** (simple_room.world):
```xml
<?xml version="1.0" ?>
<sdf version="1.6">
  <world name="simple_room">
    <include>
      <uri>model://ground_plane</uri>
    </include>
    <include>
      <uri>model://sun</uri>
    </include>

    <!-- Room Walls -->
    <model name="wall_1">
      <pose>0 5 1 0 0 0</pose>
      <link name="link">
        <visual name="visual">
          <geometry>
            <box>
              <size>10 0.2 2</size>
            </box>
          </geometry>
        </visual>
        <collision name="collision">
          <geometry>
            <box>
              <size>10 0.2 2</size>
            </box>
          </geometry>
        </collision>
      </link>
    </model>

    <!-- Additional walls to form a room -->
    <model name="wall_2">
      <pose>5 0 1 0 0 1.57</pose>
      <link name="link">
        <visual name="visual">
          <geometry>
            <box>
              <size>10 0.2 2</size>
            </box>
          </geometry>
        </visual>
        <collision name="collision">
          <geometry>
            <box>
              <size>10 0.2 2</size>
            </box>
          </geometry>
        </collision>
      </link>
    </model>

    <!-- Box obstacle -->
    <model name="box_obstacle">
      <pose>-2 2 0.5 0 0 0</pose>
      <link name="link">
        <visual name="visual">
          <geometry>
            <box>
              <size>1 1 1</size>
            </box>
          </geometry>
        </visual>
        <collision name="collision">
          <geometry>
            <box>
              <size>1 1 1</size>
            </box>
          </geometry>
        </collision>
      </link>
    </model>
  </world>
</sdf>
```

### Example 3: Unity Robot Controller Script

**C# Script for Unity Robot Control**:
```csharp
using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using Unity.Robotics.ROSTCPConnector;
using RosMessageTypes.Geometry;

public class UnityRobotController : MonoBehaviour
{
    public float linearVelocity = 1.0f;
    public float angularVelocity = 1.0f;

    private ROSConnection ros;
    private string robotTopic = "cmd_vel";

    // Start is called before the first frame update
    void Start()
    {
        ros = ROSConnection.instance;
        ros.Subscribe<TwistMsg>(robotTopic, CmdVelCallback);
    }

    void CmdVelCallback(TwistMsg cmd_vel)
    {
        // Extract linear and angular velocities
        float linear = cmd_vel.linear.x;
        float angular = cmd_vel.angular.z;

        // Apply movement to robot
        transform.Translate(Vector3.forward * linear * Time.deltaTime);
        transform.Rotate(Vector3.up, angular * Time.deltaTime);
    }

    // Update is called once per frame
    void Update()
    {
        // Additional robot logic can go here
    }
}
```

### Example 4: Gazebo-ROS2 Integration Launch File

**Launch file for Gazebo simulation with ROS2**:
```python
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    # Launch configuration variables
    use_sim_time = LaunchConfiguration('use_sim_time')

    # Declare launch arguments
    declare_use_sim_time_cmd = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Use simulation (Gazebo) clock if true'
    )

    # Start Gazebo server
    start_gazebo_cmd = Node(
        package='gazebo_ros',
        executable='gzserver',
        arguments=[
            '-s', 'libgazebo_ros_init.so',
            '-s', 'libgazebo_ros_factory.so'
        ],
        output='screen'
    )

    # Start Gazebo client
    start_gazebo_client_cmd = Node(
        package='gazebo_ros',
        executable='gzclient',
        output='screen'
    )

    # Robot state publisher
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{'use_sim_time': use_sim_time}]
    )

    # Spawn robot in Gazebo
    spawn_entity = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=[
            '-topic', 'robot_description',
            '-entity', 'my_robot'
        ],
        output='screen'
    )

    # Create launch description
    ld = LaunchDescription()

    # Add launch arguments
    ld.add_action(declare_use_sim_time_cmd)

    # Add nodes
    ld.add_action(start_gazebo_cmd)
    ld.add_action(start_gazebo_client_cmd)
    ld.add_action(robot_state_publisher)
    ld.add_action(spawn_entity)

    return ld
```

### Example 5: Simulation Architecture Comparison

```
                    Gazebo vs Unity Architecture
    ┌─────────────────────────────────────────────────────────┐
    │                    Gazebo Architecture                  │
    │  ┌─────────────┐  ┌─────────────┐  ┌─────────────┐     │
    │  │   Physics   │  │   Sensor    │  │   ROS2      │     │
    │  │   Engine    │  │   Plugins   │  │   Bridge    │     │
    │  └─────────────┘  └─────────────┘  └─────────────┘     │
    │         │                │                │           │
    │         └────────────────┼────────────────┘           │
    │                          │                            │
    │                   ┌──────▼──────┐                     │
    │                   │   World     │                     │
    │                   │   Manager   │                     │
    │                   └─────────────┘                     │
    └─────────────────────────────────────────────────────────┘

    ┌─────────────────────────────────────────────────────────┐
    │                   Unity Architecture                    │
    │  ┌─────────────┐  ┌─────────────┐  ┌─────────────┐     │
    │  │   PhysX     │  │   Custom    │  │   ROS#      │     │
    │  │   Physics   │  │   Scripts   │  │   Bridge    │     │
    │  └─────────────┘  └─────────────┘  └─────────────┘     │
    │         │                │                │           │
    │         └────────────────┼────────────────┘           │
    │                          │                            │
    │                   ┌──────▼──────┐                     │
    │                   │   Scene     │                     │
    │                   │   Manager   │                     │
    │                   └─────────────┘                     │
    └─────────────────────────────────────────────────────────┘
```

Both Gazebo and Unity provide powerful simulation capabilities for Physical AI development. Gazebo excels in physics accuracy and robotics-specific features, while Unity offers superior graphics and cross-platform capabilities. The choice between platforms depends on specific project requirements, with many teams using both for different aspects of development.