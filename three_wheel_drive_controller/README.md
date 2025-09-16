# Three Wheel Drive Controller

A ROS 2 controller for mobile robots with a three-wheel configuration: two front differential drive wheels and one rear steered wheel. This controller combines the kinematics of differential drive (for the front wheels) and tricycle drive (for the rear wheel).

## Robot Configuration

This controller is designed for robots with the following wheel configuration:
- **Front Wheels**: Two independently controlled wheels for differential drive
- **Rear Wheel**: One steered wheel that provides both traction and steering

## Features

- Subscribes to `/cmd_vel` (geometry_msgs/Twist) for velocity commands
- Publishes odometry on `/odom` (nav_msgs/Odometry)
- Publishes TF transform from `odom` to `base_link`
- Combines differential drive kinematics (front) with tricycle steering (rear)
- Supports velocity and acceleration limiting
- Configurable wheel parameters and dimensions

## Hardware Interfaces

### Required State Interfaces
- `<left_wheel_name>/velocity` (front left wheel velocity)
- `<right_wheel_name>/velocity` (front right wheel velocity)  
- `<rear_wheel_name>/velocity` (rear wheel velocity)
- `<rear_steering_joint_name>/position` (rear wheel steering angle)

### Required Command Interfaces
- `<left_wheel_name>/velocity` (front left wheel velocity command)
- `<right_wheel_name>/velocity` (front right wheel velocity command)
- `<rear_wheel_name>/velocity` (rear wheel velocity command)
- `<rear_steering_joint_name>/position` (rear wheel steering command)

### Optional State Interfaces (for improved odometry)
- `<left_wheel_name>/position` (front left wheel position)
- `<right_wheel_name>/position` (front right wheel position)
- `<rear_wheel_name>/position` (rear wheel position)

## Parameters

### Required Robot Parameters
- `left_wheel_names`: Names of the left side wheels' joints (array)
- `right_wheel_names`: Names of the right side wheels' joints (array)  
- `rear_wheel_name`: Name of the rear wheel's traction joint (string)
- `rear_steering_joint_name`: Name of the rear wheel's steering joint (string)
- `wheel_separation`: Distance between left and right front wheels (meters)
- `wheelbase`: Distance from front axle center to rear wheel (meters)
- `left_wheel_radius`: Radius of the left front wheel (meters)
- `right_wheel_radius`: Radius of the right front wheel (meters)
- `rear_wheel_radius`: Radius of the rear wheel (meters)
- `max_steering_angle`: Maximum steering angle for rear wheel (radians)

### Optional Parameters
- `wheel_separation_multiplier`: Correction factor for wheel separation (default: 1.0)
- `left_wheel_radius_multiplier`: Correction factor for left wheel radius (default: 1.0)
- `right_wheel_radius_multiplier`: Correction factor for right wheel radius (default: 1.0)
- `rear_wheel_radius_multiplier`: Correction factor for rear wheel radius (default: 1.0)
- `odom_frame_id`: Frame ID for odometry (default: "odom")
- `base_frame_id`: Frame ID for robot base (default: "base_link")
- `cmd_vel_timeout`: Timeout for cmd_vel commands in seconds (default: 0.5)
- `publish_rate`: Publishing rate for odometry in Hz (default: 50.0)
- `enable_odom_tf`: Whether to publish odom->base_link transform (default: true)
- `open_loop`: Use open-loop odometry from commands (default: false)
- `position_feedback`: Use position feedback for odometry (default: true)

## Topics

### Subscribed Topics
- `~/cmd_vel` (geometry_msgs/Twist): Velocity commands

### Published Topics  
- `~/odom` (nav_msgs/Odometry): Robot odometry
- `~/tf` (tf2_msgs/TFMessage): Transform from odom to base_link
- `~/cmd_vel_out` (geometry_msgs/TwistStamped): Limited velocity commands (if enabled)

## Kinematics

The controller implements a hybrid kinematics model:

1. **Front Differential Drive**: The two front wheels operate as a standard differential drive system
2. **Rear Steering**: The rear wheel provides both traction and steering capability

Given a commanded linear velocity (v) and angular velocity (ω):

1. **Front wheel velocities** are calculated using differential drive kinematics:
   - `v_left = v - (ω * wheel_separation / 2)`  
   - `v_right = v + (ω * wheel_separation / 2)`

2. **Rear wheel steering angle** is calculated based on bicycle/tricycle model:
   - `steering_angle = atan(ω * wheelbase / v)` (when v ≠ 0)

3. **Rear wheel velocity** is calculated to maintain kinematic consistency:
   - `v_rear = v / cos(steering_angle)`

## Usage Example

```yaml
# In your controller configuration file
controller_manager:
  ros__parameters:
    three_wheel_drive_controller:
      type: three_wheel_drive_controller/ThreeWheelDriveController

three_wheel_drive_controller:
  ros__parameters:
    left_wheel_names: ["left_front_wheel"]
    right_wheel_names: ["right_front_wheel"]
    rear_wheel_name: "rear_wheel"
    rear_steering_joint_name: "rear_steering_joint"
    
    wheel_separation: 0.5
    wheelbase: 0.8
    left_wheel_radius: 0.1
    right_wheel_radius: 0.1
    rear_wheel_radius: 0.1
    max_steering_angle: 1.57  # 90 degrees
```

## Building

This controller is part of the ros2_controllers package. Build with:

```bash
colcon build --packages-select three_wheel_drive_controller
```

## Loading the Controller

```bash
# Load the controller
ros2 control load_controller three_wheel_drive_controller

# Configure the controller  
ros2 control set_controller_state three_wheel_drive_controller configure

# Activate the controller
ros2 control set_controller_state three_wheel_drive_controller activate
```

## Sending Commands

```bash
# Send velocity commands
ros2 topic pub /three_wheel_drive_controller/cmd_vel geometry_msgs/Twist \
  "linear: {x: 0.5, y: 0.0, z: 0.0}
   angular: {x: 0.0, y: 0.0, z: 0.2}"
```

## Motion Characteristics

This three-wheel configuration provides:
- **Forward/Backward motion**: All three wheels rotate in the same direction
- **Turning**: Front wheels use differential drive, rear wheel steers
- **Zero-radius turns**: Possible by setting opposite front wheel velocities and appropriate rear steering
- **Smooth cornering**: Rear wheel steering provides natural turning motion

## Advantages

- Combines differential drive precision with steered wheel efficiency
- Better turning radius than pure differential drive
- More stable than pure tricycle drive
- Suitable for indoor and outdoor navigation

## Limitations

- More complex than standard differential or tricycle drives
- Requires careful calibration of wheel parameters
- Maximum steering angle limits tight turns
- Kinematic singularities at very low velocities