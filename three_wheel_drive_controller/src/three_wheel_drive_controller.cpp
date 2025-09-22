// Copyright 2024 ros2_controllers Contributors
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include <memory>
#include <queue>
#include <string>
#include <utility>
#include <vector>

#include "three_wheel_drive_controller/three_wheel_drive_controller.hpp"
#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "lifecycle_msgs/msg/state.hpp"
#include "rclcpp/logging.hpp"
#include "tf2/LinearMath/Quaternion.hpp"

namespace
{
constexpr auto DEFAULT_COMMAND_TOPIC = "~/cmd_vel";
constexpr auto DEFAULT_COMMAND_OUT_TOPIC = "~/cmd_vel_out";
constexpr auto DEFAULT_ODOMETRY_TOPIC = "~/odom";
constexpr auto DEFAULT_TRANSFORM_TOPIC = "/tf";
}  // namespace

namespace three_wheel_drive_controller
{
using namespace std::chrono_literals;
using controller_interface::interface_configuration_type;
using controller_interface::InterfaceConfiguration;
using hardware_interface::HW_IF_POSITION;
using hardware_interface::HW_IF_VELOCITY;
using lifecycle_msgs::msg::State;

ThreeWheelDriveController::ThreeWheelDriveController()
: controller_interface::ControllerInterface()
{
}

controller_interface::CallbackReturn ThreeWheelDriveController::on_init()
{
  try
  {
    // Create the parameter listener and get the parameters
    param_listener_ = std::make_shared<ParamListener>(get_node());
    params_ = param_listener_->get_params();
  }
  catch (const std::exception & e)
  {
    fprintf(stderr, "Exception thrown during init stage with message: %s \n", e.what());
    return controller_interface::CallbackReturn::ERROR;
  }

  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::InterfaceConfiguration 
ThreeWheelDriveController::command_interface_configuration() const
{
  std::vector<std::string> conf_names;

  // Front wheel velocity commands
  for (const auto & joint_name : params_.left_wheel_names)
  {
    conf_names.push_back(joint_name + "/" + hardware_interface::HW_IF_VELOCITY);
  }
  for (const auto & joint_name : params_.right_wheel_names)
  {
    conf_names.push_back(joint_name + "/" + hardware_interface::HW_IF_VELOCITY);
  }

  // Rear wheel velocity command always; steering command only if enabled
  conf_names.push_back(params_.rear_wheel_name + "/" + hardware_interface::HW_IF_VELOCITY);
  if (params_.enable_rear_steering)
  {
    conf_names.push_back(params_.rear_steering_joint_name + "/" + hardware_interface::HW_IF_VELOCITY);
  }

  return {interface_configuration_type::INDIVIDUAL, conf_names};
}

controller_interface::InterfaceConfiguration 
ThreeWheelDriveController::state_interface_configuration() const
{
  std::vector<std::string> conf_names;

  // Front wheel velocity states
  for (const auto & joint_name : params_.left_wheel_names)
  {
    conf_names.push_back(joint_name + "/" + hardware_interface::HW_IF_VELOCITY);
    if (params_.position_feedback)
    {
      conf_names.push_back(joint_name + "/" + hardware_interface::HW_IF_POSITION);
    }
  }
  for (const auto & joint_name : params_.right_wheel_names)
  {
    conf_names.push_back(joint_name + "/" + hardware_interface::HW_IF_VELOCITY);
    if (params_.position_feedback)
    {
      conf_names.push_back(joint_name + "/" + hardware_interface::HW_IF_POSITION);
    }
  }

  // Rear wheel velocity and position states (steering position only if enabled)
  conf_names.push_back(params_.rear_wheel_name + "/" + hardware_interface::HW_IF_VELOCITY);
  if (params_.position_feedback)
  {
    conf_names.push_back(params_.rear_wheel_name + "/" + hardware_interface::HW_IF_POSITION);
  }
  if (params_.enable_rear_steering)
  {
    conf_names.push_back(params_.rear_steering_joint_name + "/" + hardware_interface::HW_IF_POSITION);
  }

  return {interface_configuration_type::INDIVIDUAL, conf_names};
}

controller_interface::CallbackReturn ThreeWheelDriveController::on_configure(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  // Update the parameters
  params_ = param_listener_->get_params();

  // Initialize dynamic parameter storage
  rear_steering_zero_offset_ = params_.rear_steering_zero_offset;

  // Setup parameter callback for dynamic parameters
  auto param_callback = std::bind(
    &ThreeWheelDriveController::dynamicParametersCallback, this, std::placeholders::_1);
  param_callback_handle_ = get_node()->add_on_set_parameters_callback(param_callback);

  // Initialize odometry
  odometry_ = std::make_shared<Odometry>(params_.velocity_rolling_window_size);

  odometry_->setWheelParams(
    params_.wheel_separation * params_.wheel_separation_multiplier,
    params_.wheelbase,
    params_.left_wheel_radius * params_.left_wheel_radius_multiplier,
    params_.right_wheel_radius * params_.right_wheel_radius_multiplier,
    params_.rear_wheel_radius * params_.rear_wheel_radius_multiplier);

  odometry_->setVelocityRollingWindowSize(params_.velocity_rolling_window_size);

  last_update_time_ = get_node()->get_clock()->now();

  cmd_vel_timeout_ = std::chrono::milliseconds{static_cast<int>(params_.cmd_vel_timeout * 1000.0)};

  // Setup Publishers and subscribers
  auto qos = rclcpp::QoS(1);
  qos.keep_last(1);
  qos.best_effort();
  qos.durability_volatile();

  // Setup topic names
  std::string controller_namespace = std::string(get_node()->get_namespace());
  
  if (controller_namespace == "/")
  {
    controller_namespace = "";
  }
  else
  {
    controller_namespace = controller_namespace.substr(0, controller_namespace.size() - 1);
  }

  // Subscriber
  velocity_command_subscriber_ = get_node()->create_subscription<TwistStamped>(
    DEFAULT_COMMAND_TOPIC, qos,
    [this](const std::shared_ptr<TwistStamped> msg) -> void
    {
      if (!subscriber_is_active_)
      {
        RCLCPP_WARN(get_node()->get_logger(), "Velocity commands received before activation");
        return;
      }

      if (msg->header.stamp.nanosec == 0u && msg->header.stamp.sec == 0u)
      {
        RCLCPP_WARN_ONCE(
          get_node()->get_logger(),
          "Received TwistStamped with no timestamp. Using current time.");
        msg->header.stamp = get_node()->now();
      }
      received_velocity_msg_ptr_.set(std::move(msg));
    });

  // Steering angle difference subscriber
  steering_angle_diff_subscriber_ = get_node()->create_subscription<std_msgs::msg::Float32>(
    "/steering_angle_diff", qos,
    [this](const std::shared_ptr<std_msgs::msg::Float32> msg) -> void
    {
      if (!subscriber_is_active_)
      {
        RCLCPP_WARN(get_node()->get_logger(), "Steering angle difference received before activation");
        return;
      }
      received_steering_diff_msg_ptr_.set(std::move(msg));
    });

  // Publishers
  odometry_publisher_ = get_node()->create_publisher<nav_msgs::msg::Odometry>(
    DEFAULT_ODOMETRY_TOPIC, rclcpp::SystemDefaultsQoS());
  realtime_odometry_publisher_ =
    std::make_shared<realtime_tools::RealtimePublisher<nav_msgs::msg::Odometry>>(
      odometry_publisher_);

  auto & odometry_message = realtime_odometry_publisher_->msg_;
  odometry_message.header.frame_id = params_.odom_frame_id;
  odometry_message.child_frame_id = params_.base_frame_id;

  // Setup transform broadcaster
  if (params_.enable_odom_tf)
  {
    odometry_transform_publisher_ = get_node()->create_publisher<tf2_msgs::msg::TFMessage>(
      DEFAULT_TRANSFORM_TOPIC, rclcpp::SystemDefaultsQoS());
    realtime_odometry_transform_publisher_ =
      std::make_shared<realtime_tools::RealtimePublisher<tf2_msgs::msg::TFMessage>>(
        odometry_transform_publisher_);
  }

  // Setup limited velocity publisher if required
  if (params_.publish_limited_velocity)
  {
    limited_velocity_publisher_ = get_node()->create_publisher<TwistStamped>(
      DEFAULT_COMMAND_OUT_TOPIC, rclcpp::SystemDefaultsQoS());
    realtime_limited_velocity_publisher_ =
      std::make_shared<realtime_tools::RealtimePublisher<TwistStamped>>(
        limited_velocity_publisher_);
  }

  // Setup speed limiters
  limiter_linear_ = SpeedLimiter(
    params_.linear.x.min_velocity, params_.linear.x.max_velocity,
    params_.linear.x.max_acceleration_reverse, params_.linear.x.max_acceleration,
    params_.linear.x.max_deceleration, params_.linear.x.max_deceleration_reverse,
    params_.linear.x.min_jerk, params_.linear.x.max_jerk);

  limiter_angular_ = SpeedLimiter(
    params_.angular.z.min_velocity, params_.angular.z.max_velocity,
    params_.angular.z.max_acceleration_reverse, params_.angular.z.max_acceleration,
    params_.angular.z.max_deceleration, params_.angular.z.max_deceleration_reverse,
    params_.angular.z.min_jerk, params_.angular.z.max_jerk);

  // Initialize the previous commands queue
  reset_buffers();

  // Set publish rate
  publish_rate_ = params_.publish_rate;
  publish_period_ = rclcpp::Duration::from_nanoseconds(
    static_cast<int64_t>(1e9 / publish_rate_));

  previous_update_timestamp_ = get_node()->now();
  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn ThreeWheelDriveController::get_wheel(
  const std::string & wheel_name, const std::vector<std::string> & wheel_names,
  std::vector<WheelHandle> & registered_handles)
{
  auto logger = get_node()->get_logger();

  if (wheel_names.empty())
  {
    RCLCPP_ERROR(logger, "No '%s' wheel names specified", wheel_name.c_str());
    return controller_interface::CallbackReturn::ERROR;
  }

  // register handles
  registered_handles.reserve(wheel_names.size());
  for (const auto & wheel : wheel_names)
  {
    const auto interface_name = wheel + "/" + hardware_interface::HW_IF_VELOCITY;
    const auto state_handle = std::find_if(
      state_interfaces_.cbegin(), state_interfaces_.cend(),
      [&interface_name](const auto & interface)
      {
        return interface.get_name() == interface_name;
      });

    if (state_handle == state_interfaces_.cend())
    {
      RCLCPP_ERROR(logger, "Unable to obtain joint state handle for %s", interface_name.c_str());
      return controller_interface::CallbackReturn::ERROR;
    }

    const auto command_handle = std::find_if(
      command_interfaces_.begin(), command_interfaces_.end(),
      [&interface_name](const auto & interface)
      {
        return interface.get_name() == interface_name;
      });

    if (command_handle == command_interfaces_.end())
    {
      RCLCPP_ERROR(logger, "Unable to obtain joint command handle for %s", interface_name.c_str());
      return controller_interface::CallbackReturn::ERROR;
    }

    registered_handles.emplace_back(
      WheelHandle{std::ref(*state_handle), std::ref(*command_handle)});
  }

  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn ThreeWheelDriveController::get_rear_wheel(
  const std::string & rear_wheel_name, std::vector<RearWheelHandle> & registered_handles)
{
  auto logger = get_node()->get_logger();

  if (rear_wheel_name.empty())
  {
    RCLCPP_ERROR(logger, "No rear wheel name specified");
    return controller_interface::CallbackReturn::ERROR;
  }

  // Get velocity state and command for rear wheel
  const auto vel_interface_name = rear_wheel_name + "/" + hardware_interface::HW_IF_VELOCITY;
  const auto vel_state_handle = std::find_if(
    state_interfaces_.cbegin(), state_interfaces_.cend(),
    [&vel_interface_name](const auto & interface)
    {
      return interface.get_name() == vel_interface_name;
    });

  if (vel_state_handle == state_interfaces_.cend())
  {
    RCLCPP_ERROR(logger, "Unable to obtain rear wheel velocity state handle for %s", vel_interface_name.c_str());
    return controller_interface::CallbackReturn::ERROR;
  }

  const auto vel_command_handle = std::find_if(
    command_interfaces_.begin(), command_interfaces_.end(),
    [&vel_interface_name](const auto & interface)
    {
      return interface.get_name() == vel_interface_name;
    });

  if (vel_command_handle == command_interfaces_.end())
  {
    RCLCPP_ERROR(logger, "Unable to obtain rear wheel velocity command handle for %s", vel_interface_name.c_str());
    return controller_interface::CallbackReturn::ERROR;
  }

  RCLCPP_INFO(logger, "Successfully found rear wheel velocity interfaces: %s", vel_interface_name.c_str());

  if (params_.enable_rear_steering)
  {
    // Get position state and command for rear steering
    const auto pos_interface_name = params_.rear_steering_joint_name + "/" + hardware_interface::HW_IF_POSITION;
    const auto pos_state_handle = std::find_if(
      state_interfaces_.cbegin(), state_interfaces_.cend(),
      [&pos_interface_name](const auto & interface)
      {
        return interface.get_name() == pos_interface_name;
      });

    if (pos_state_handle == state_interfaces_.cend())
    {
      RCLCPP_ERROR(logger, "Unable to obtain rear steering position state handle for %s", pos_interface_name.c_str());
      return controller_interface::CallbackReturn::ERROR;
    }

    // Get velocity command for rear steering (changed from position to velocity)
    const auto steering_vel_interface_name = params_.rear_steering_joint_name + "/" + hardware_interface::HW_IF_VELOCITY;
    const auto steering_vel_command_handle = std::find_if(
      command_interfaces_.begin(), command_interfaces_.end(),
      [&steering_vel_interface_name](const auto & interface)
      {
        return interface.get_name() == steering_vel_interface_name;
      });

    if (steering_vel_command_handle == command_interfaces_.end())
    {
      RCLCPP_ERROR(logger, "Unable to obtain rear steering velocity command handle for %s", steering_vel_interface_name.c_str());
      return controller_interface::CallbackReturn::ERROR;
    }

    registered_handles.emplace_back(RearWheelHandle{
      std::ref(*vel_state_handle), std::ref(*vel_command_handle),
      std::ref(*pos_state_handle), std::ref(*steering_vel_command_handle)});
  }
  else
  {
    // Steering disabled: reuse velocity state/command handles for position/steering command placeholders.
    registered_handles.emplace_back(RearWheelHandle{
      std::ref(*vel_state_handle), std::ref(*vel_command_handle),
      std::ref(*vel_state_handle), std::ref(*vel_command_handle)});
  }

  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn ThreeWheelDriveController::on_activate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  // Setup the wheel handles
  const auto front_result = get_wheel("left", params_.left_wheel_names, registered_left_wheel_handles_);
  if (front_result != controller_interface::CallbackReturn::SUCCESS)
  {
    return front_result;
  }

  const auto rear_result = get_wheel("right", params_.right_wheel_names, registered_right_wheel_handles_);
  if (rear_result != controller_interface::CallbackReturn::SUCCESS)
  {
    return rear_result;
  }

  const auto rear_wheel_result = get_rear_wheel(params_.rear_wheel_name, registered_rear_wheel_handles_);
  if (rear_wheel_result != controller_interface::CallbackReturn::SUCCESS)
  {
    return rear_wheel_result;
  }

  if (registered_left_wheel_handles_.empty() || registered_right_wheel_handles_.empty() || 
      registered_rear_wheel_handles_.empty())
  {
    RCLCPP_ERROR(get_node()->get_logger(), "Got empty wheel handles during activation");
    return controller_interface::CallbackReturn::ERROR;
  }

  // Initialize odometry
  reset();

  // Publisher is activated by `on_activate` callback
  // Realtime publisher also needs to start
  subscriber_is_active_ = true;

  RCLCPP_DEBUG(get_node()->get_logger(), "Subscriber and publisher are now active.");
  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn ThreeWheelDriveController::on_deactivate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  subscriber_is_active_ = false;
  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn ThreeWheelDriveController::on_cleanup(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  reset_buffers();
  velocity_command_subscriber_.reset();
  steering_angle_diff_subscriber_.reset();
  odometry_publisher_.reset();
  realtime_odometry_publisher_.reset();
  odometry_transform_publisher_.reset();
  realtime_odometry_transform_publisher_.reset();
  limited_velocity_publisher_.reset();
  realtime_limited_velocity_publisher_.reset();

  // Remove parameter callback
  if (param_callback_handle_)
  {
    get_node()->remove_on_set_parameters_callback(param_callback_handle_.get());
    param_callback_handle_.reset();
  }

  received_velocity_msg_ptr_.set(std::make_shared<TwistStamped>());
  received_steering_diff_msg_ptr_.set(std::make_shared<std_msgs::msg::Float32>());
  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn ThreeWheelDriveController::on_error(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  reset_buffers();
  return controller_interface::CallbackReturn::SUCCESS;
}

bool ThreeWheelDriveController::reset()
{
  odometry_->resetOdometry();

  // Set last velocity to zero
  received_velocity_msg_ptr_.set(std::make_shared<TwistStamped>());
  received_steering_diff_msg_ptr_.set(std::make_shared<std_msgs::msg::Float32>());

  reset_buffers();

  subscriber_is_active_ = true;
  // Reset acceleration limiter state
  prev_left_wheel_vel_cmd_ = 0.0;
  prev_right_wheel_vel_cmd_ = 0.0;
  prev_rear_wheel_vel_cmd_ = 0.0;
  steering_at_target_ = true;
  return true;
}

void ThreeWheelDriveController::halt()
{
  const auto halt_wheels = [](auto & wheel_handles)
  {
    for (const auto & wheel_handle : wheel_handles)
    {
      wheel_handle.velocity_command.get().set_value(0.0);
    }
  };

  halt_wheels(registered_left_wheel_handles_);
  halt_wheels(registered_right_wheel_handles_);
  
  // Also halt rear wheel
  for (const auto & rear_wheel_handle : registered_rear_wheel_handles_)
  {
    rear_wheel_handle.velocity_command.get().set_value(0.0);
    rear_wheel_handle.steering_velocity_command.get().set_value(0.0);
  }
}

double ThreeWheelDriveController::map_steering_angle_to_motor_position(double steering_angle_rad)
{
  // Map steering angle in radians to motor position units using conversion factor
  // steering_angle_rad: -max_steering_angle to +max_steering_angle
  // motor_position: computed from zero offset and conversion ratio
  
  // Clamp steering angle to valid range
  double clamped_angle = std::max(-params_.max_steering_angle, 
                                 std::min(params_.max_steering_angle, steering_angle_rad));
  
  // Convert angle to motor position using the conversion factor
  double motor_position_offset = clamped_angle * params_.rear_steering_motor_ticks_per_radian;
  
  // Apply zero offset (use dynamic parameter)
  return rear_steering_zero_offset_ + motor_position_offset;
}

double ThreeWheelDriveController::get_rear_steering_motor_min_position() const
{
  // Minimum position corresponds to maximum negative steering angle
  return rear_steering_zero_offset_ - (params_.max_steering_angle * params_.rear_steering_motor_ticks_per_radian);
}

double ThreeWheelDriveController::get_rear_steering_motor_max_position() const
{
  // Maximum position corresponds to maximum positive steering angle
  return rear_steering_zero_offset_ + (params_.max_steering_angle * params_.rear_steering_motor_ticks_per_radian);
}

void ThreeWheelDriveController::calculate_three_wheel_kinematics(
  double linear_velocity, double angular_velocity,
  double wheelbase, double track_width,
  double & left_wheel_vel, double & right_wheel_vel, 
  double & rear_wheel_vel, double & rear_wheel_pos)
{
  // Replicate prototype kinematics from joystick_motor_control.py
  // Inputs: linear_velocity (desired forward speed of rear wheel u), angular_velocity (desired yaw rate, CCW +)
  // Rear wheel steering angle delta satisfies: omega = -(u / L) * sin(delta) ; front axle center speed = u * cos(delta)
  // Solve for delta first. Handle special cases for numerical stability.

  const double L = wheelbase;
  const double W = track_width;
  const double max_delta = params_.max_steering_angle; // should be <= pi/2

  double u = linear_velocity;               // rear wheel commanded longitudinal speed
  double omega = angular_velocity;          // desired yaw rate

  // Compute steering angle from u and omega. From omega = -(u/L) * sin(delta) -> sin(delta) = -omega * L / u
  if (std::abs(u) < 1e-6)
  {
    // Pure rotation or near zero linear; choose delta to achieve rotation with minimal u.
    // Use maximum steering to minimize required wheel speed magnitude.
    if (std::abs(omega) < 1e-6)
    {
      rear_wheel_pos = 0.0; // stationary
      left_wheel_vel = 0.0;
      right_wheel_vel = 0.0;
      rear_wheel_vel = 0.0;
      return;
    }
    rear_wheel_pos = (omega > 0.0) ? -max_delta : max_delta;
    // With delta = +/-90deg, sin(delta)= +/-1 -> needed rear wheel speed u = -omega * L / sin(delta) = -omega * L * sign(sin(delta))
    // For delta = +max_delta (positive ~ +90deg) and omega>0 -> u = -omega*L (matches prototype logic sign)
    u = -omega * L / std::sin(rear_wheel_pos); // compute minimal wheel speed producing omega
  }
  else
  {
    double s = -omega * L / u;
    if (s > 1.0) s = 1.0;
    if (s < -1.0) s = -1.0;
    rear_wheel_pos = std::asin(s);
  }

  // Clamp steering angle
  if (rear_wheel_pos >  max_delta) rear_wheel_pos =  max_delta;
  if (rear_wheel_pos < -max_delta) rear_wheel_pos = -max_delta;

  // Front axle center linear speed
  double v_front_center = u * std::cos(rear_wheel_pos);
  // Recompute omega from consistent kinematics (authoritative relation)
  double omega_consistent = -(u / L) * std::sin(rear_wheel_pos);

  // Differential front wheel speeds about front axle center
  left_wheel_vel  = v_front_center - omega_consistent * ( +W / 2.0 );
  right_wheel_vel = v_front_center - omega_consistent * ( -W / 2.0 );
  rear_wheel_vel  = u;

  // Saturation preserving ratios
  const double max_wheel_speed = 50.0; // TODO: expose as parameter
  double peak = std::max({std::abs(left_wheel_vel), std::abs(right_wheel_vel), std::abs(rear_wheel_vel)});
  if (peak > max_wheel_speed)
  {
    double scale = max_wheel_speed / peak;
    left_wheel_vel  *= scale;
    right_wheel_vel *= scale;
    rear_wheel_vel  *= scale;
  }
}

controller_interface::return_type ThreeWheelDriveController::update(
  const rclcpp::Time & time, const rclcpp::Duration & period)
{
  auto logger = get_node()->get_logger();
  static bool is_halted = false;
  
  if (get_lifecycle_state().id() == State::PRIMARY_STATE_INACTIVE)
  {
    if (!is_halted)
    {
      halt();
      is_halted = true;
    }
    return controller_interface::return_type::OK;
  }

  auto current_time = time;
  
  std::shared_ptr<TwistStamped> last_command_msg;
  received_velocity_msg_ptr_.get(last_command_msg);

  // Check steering angle difference for gating drive commands
  // Only use topic value when rear steering is not handled by this controller
  if (!params_.enable_rear_steering)
  {
    std::shared_ptr<std_msgs::msg::Float32> steering_diff_msg;
    received_steering_diff_msg_ptr_.get(steering_diff_msg);
    
    if (steering_diff_msg != nullptr)
    {
      steering_at_target_ = steering_diff_msg->data < steering_position_tolerance_;
    }
  }

  if (last_command_msg == nullptr)
  {
    RCLCPP_WARN(logger, "Velocity message received was a nullptr.");
    return controller_interface::return_type::ERROR;
  }

  // Apply speed limiting
  double linear_command = 0.0;
  double angular_command = 0.0;
  
  const auto age_of_last_command = 
    std::chrono::nanoseconds(rclcpp::Time(current_time).nanoseconds() - rclcpp::Time(last_command_msg->header.stamp).nanoseconds());
    
  // Check if cmd_vel has timed out
  if (age_of_last_command > cmd_vel_timeout_)
  {
    // Timeout - force zero velocities and continue processing
    linear_command = 0.0;
    angular_command = 0.0;
    RCLCPP_WARN_THROTTLE(logger, *(get_node()->get_clock()), 1000, 
      "cmd_vel timeout (%.3f s) - stopping robot", static_cast<double>(age_of_last_command.count()) / 1e9);
  }
  else if (rclcpp::Time(last_command_msg->header.stamp) <= rclcpp::Time(previous_update_timestamp_))
  {
    // Command is older than last update - skip processing but don't stop
    return controller_interface::return_type::OK;
  }
  else
  {
    // Valid command - use the received velocities
    linear_command = last_command_msg->twist.linear.x;
    angular_command = last_command_msg->twist.angular.z;
  }

  // Convert to array for limiting
  std::array<double, 2> last_vel_command = {{linear_command, angular_command}};
  std::array<double, 2> prev_cmd_0 = {0.0, 0.0};
  std::array<double, 2> prev_cmd_1 = {0.0, 0.0};
  
  if (!previous_commands_.empty())
  {
    prev_cmd_0 = previous_commands_.back();
    previous_commands_.pop();
    if (!previous_commands_.empty())
    {
      prev_cmd_1 = previous_commands_.back();
      previous_commands_.pop();
    }
  }
  auto dt = period.seconds();

  // Apply velocity limiting
  limiter_linear_.limit(linear_command, prev_cmd_0[0], prev_cmd_1[0], dt);
  limiter_angular_.limit(angular_command, prev_cmd_0[1], prev_cmd_1[1], dt);

  previous_commands_.emplace(last_vel_command);
  previous_update_timestamp_ = current_time;

  // Calculate wheel speeds and steering angle
  double left_wheel_vel, right_wheel_vel, rear_wheel_vel, rear_wheel_pos;
  if (params_.enable_rear_steering)
  {
    calculate_three_wheel_kinematics(
      linear_command, angular_command,
      params_.wheelbase, params_.wheel_separation,
      left_wheel_vel, right_wheel_vel, rear_wheel_vel, rear_wheel_pos);
  }
  else
  {
    calculate_three_wheel_kinematics(
      linear_command, angular_command,
      params_.wheelbase, params_.wheel_separation,
      left_wheel_vel, right_wheel_vel, rear_wheel_vel, rear_wheel_pos);
    rear_wheel_pos  = 0.0;
  }

  // Apply wheel radius scaling  
  left_wheel_vel /= (params_.left_wheel_radius * params_.left_wheel_radius_multiplier);
  right_wheel_vel /= (params_.right_wheel_radius * params_.right_wheel_radius_multiplier);
  rear_wheel_vel /= (params_.rear_wheel_radius * params_.rear_wheel_radius_multiplier);

  
  // First, handle steering positioning
  double target_steering_pos = 0.0;
  double current_steering_pos = 0.0;
  double position_error = 0.0;
  const double steering_tolerance = 0.2; // Tolerance for considering steering at target
  
  if (params_.enable_rear_steering)
  {
    // When rear steering is handled by this controller, calculate steering_at_target locally
    bool local_steering_at_target = true;
    
    for (auto & rear_wheel_handle : registered_rear_wheel_handles_)
    {
      current_steering_pos = rear_wheel_handle.position_state.get().get_value();
      target_steering_pos = map_steering_angle_to_motor_position(rear_wheel_pos);
      position_error = target_steering_pos - current_steering_pos;
      
      if (std::abs(position_error) > steering_tolerance)
      {
        local_steering_at_target = false;
      }
      
      const double kP         = 3.0;
      const double kS         = 0.35;
      const double v_max      = 5.0;
      const double v_min      = 0.20;
      const double a_max      = 10.0;
      const double deadband   = 0.05;
      const double slow_zone  = 0.3;
      double err = position_error;
      double v_des = kP * err;
      const double a_stop = a_max;
      const double e = std::abs(err);
      const double margin = std::max(0.0, e - deadband);
      double v_brake = std::sqrt(2.0 * a_stop * margin);
      double v_step = (dt > 0.0) ? (margin / dt) : v_brake;
      double v_cap = std::min(v_brake, v_step);
      if (std::abs(v_des) > v_cap) { v_des = (v_des >= 0.0 ? +v_cap : -v_cap); }
      if (e <= deadband) { v_des = 0.0; }
      static double v_prev = 0.0;
      const double dv_max = a_max * dt;
      double v_cmd = v_prev + std::clamp(v_des - v_prev, -dv_max, dv_max);
      if ( (err > 0.0 && v_cmd < 0.0) || (err < 0.0 && v_cmd > 0.0) ) {
        if (e < (3.0 * deadband)) { v_cmd = 0.0; }
      }
      v_prev = v_cmd;
      rear_wheel_handle.steering_velocity_command.get().set_value(v_cmd);
    }
    
    // Update the member variable with locally calculated status
    steering_at_target_ = local_steering_at_target;
  }
  
  // Now handle drive wheels - only move if steering is at target position
  {
    // Unified accel/decel limiting (also handles gating by setting desired=0 when steering not ready)
    double dt_sec = period.seconds();
    auto limit_accel = [dt_sec, this](double desired, double & prev) -> double
    {
      const double max_delta = max_wheel_accel_ * dt_sec; // symmetric accel/decel
      double delta = desired - prev;
      if (delta > max_delta) delta = max_delta;
      else if (delta < -max_delta) delta = -max_delta;
      prev += delta;
      return prev;
    };

    // Desired rotational wheel velocities (after radius scaling already applied above)
    // Use member variable steering_at_target_ instead of local variable
    double desired_left  = steering_at_target_ ? left_wheel_vel  : 0.0;
    double desired_right = steering_at_target_ ? right_wheel_vel : 0.0;
    double desired_rear  = steering_at_target_ ? rear_wheel_vel : 0.0; // existing inversion kept

    double limited_left  = limit_accel(desired_left,  prev_left_wheel_vel_cmd_);
    double limited_right = limit_accel(desired_right, prev_right_wheel_vel_cmd_);
    double limited_rear  = limit_accel(desired_rear,  prev_rear_wheel_vel_cmd_);

    RCLCPP_INFO_THROTTLE(
      logger, *(get_node()->get_clock()), 500,
      "Cmd (lin, ang)=(%.3f, %.3f) -> wheel cmd (L, R, Re)=(%.3f, %.3f, %.3f) (desired (L, R, Re)=(%.3f, %.3f, %.3f))",
      linear_command, angular_command,
      limited_left, limited_right, limited_rear,
      desired_left, desired_right, desired_rear);

    for (auto & wheel_handle : registered_left_wheel_handles_)
    {
      wheel_handle.velocity_command.get().set_value(limited_left);
    }
    for (auto & wheel_handle : registered_right_wheel_handles_)
    {
      wheel_handle.velocity_command.get().set_value(limited_right);
    }
    for (auto & rear_wheel_handle : registered_rear_wheel_handles_)
    {
      rear_wheel_handle.velocity_command.get().set_value(limited_rear);
      if (!params_.enable_rear_steering)
      {
        // Ensure no steering command output when disabled
        // rear_wheel_handle.steering_velocity_command.get().set_value(0.0);
      }
    }

  }

  // Update odometry
  if (params_.open_loop)
  {
    odometry_->updateOpenLoop(linear_command, angular_command, current_time);
  }
  else
  {
    // Get current wheel velocities
    double left_vel = 0.0, right_vel = 0.0, rear_vel = 0.0;
    double left_pos = 0.0, right_pos = 0.0, rear_pos = 0.0;
    double rear_steering_pos = 0.0;

    for (const auto & wheel_handle : registered_left_wheel_handles_)
    {
      left_vel += wheel_handle.velocity_state.get().get_value();
      if (params_.position_feedback)
      {
        // Find position state interface for this wheel
        const auto pos_interface_name = wheel_handle.velocity_state.get().get_name().substr(0, 
          wheel_handle.velocity_state.get().get_name().find("/")) + "/" + hardware_interface::HW_IF_POSITION;
        const auto pos_state_handle = std::find_if(
          state_interfaces_.cbegin(), state_interfaces_.cend(),
          [&pos_interface_name](const auto & interface)
          {
            return interface.get_name() == pos_interface_name;
          });
        if (pos_state_handle != state_interfaces_.cend())
        {
          left_pos += pos_state_handle->get_value();
        }
      }
    }
    left_vel /= static_cast<double>(registered_left_wheel_handles_.size());
    left_pos /= static_cast<double>(registered_left_wheel_handles_.size());

    for (const auto & wheel_handle : registered_right_wheel_handles_)
    {
      right_vel += wheel_handle.velocity_state.get().get_value();
      if (params_.position_feedback)
      {
        // Find position state interface for this wheel
        const auto pos_interface_name = wheel_handle.velocity_state.get().get_name().substr(0, 
          wheel_handle.velocity_state.get().get_name().find("/")) + "/" + hardware_interface::HW_IF_POSITION;
        const auto pos_state_handle = std::find_if(
          state_interfaces_.cbegin(), state_interfaces_.cend(),
          [&pos_interface_name](const auto & interface)
          {
            return interface.get_name() == pos_interface_name;
          });
        if (pos_state_handle != state_interfaces_.cend())
        {
          right_pos += pos_state_handle->get_value();
        }
      }
    }
    right_vel /= static_cast<double>(registered_right_wheel_handles_.size());
    right_pos /= static_cast<double>(registered_right_wheel_handles_.size());

    for (const auto & rear_wheel_handle : registered_rear_wheel_handles_)
    {
      rear_vel += rear_wheel_handle.velocity_state.get().get_value();
      if (params_.enable_rear_steering)
      {
        rear_steering_pos += rear_wheel_handle.position_state.get().get_value();
      }
      if (params_.position_feedback)
      {
        // Find position state interface for rear wheel
        const auto pos_interface_name = params_.rear_wheel_name + "/" + hardware_interface::HW_IF_POSITION;
        const auto pos_state_handle = std::find_if(
          state_interfaces_.cbegin(), state_interfaces_.cend(),
          [&pos_interface_name](const auto & interface)
          {
            return interface.get_name() == pos_interface_name;
          });
        if (pos_state_handle != state_interfaces_.cend())
        {
          rear_pos += pos_state_handle->get_value();
        }
      }
    }
    rear_vel /= static_cast<double>(registered_rear_wheel_handles_.size());
    rear_pos /= static_cast<double>(registered_rear_wheel_handles_.size());
    rear_steering_pos = params_.enable_rear_steering ? (rear_steering_pos / static_cast<double>(registered_rear_wheel_handles_.size())) : 0.0;

    // Apply wheel radius for odometry
    left_vel *= (params_.left_wheel_radius * params_.left_wheel_radius_multiplier);
    right_vel *= (params_.right_wheel_radius * params_.right_wheel_radius_multiplier);  
    rear_vel *= (params_.rear_wheel_radius * params_.rear_wheel_radius_multiplier);

    if (params_.position_feedback)
    {
      left_pos *= (params_.left_wheel_radius * params_.left_wheel_radius_multiplier);
      right_pos *= (params_.right_wheel_radius * params_.right_wheel_radius_multiplier);
      rear_pos *= (params_.rear_wheel_radius * params_.rear_wheel_radius_multiplier);
      odometry_->update(left_pos, right_pos, rear_pos, current_time);
    }
    else
    {
      odometry_->updateFromVelocity(left_vel, right_vel, rear_vel, rear_steering_pos, current_time);
    }
  }

  // Publish odometry and tf
  if (current_time - previous_publish_timestamp_ >= publish_period_)
  {
    previous_publish_timestamp_ = current_time;

    if (realtime_odometry_publisher_->trylock())
    {
      auto & odometry_message = realtime_odometry_publisher_->msg_;
      odometry_message.header.stamp = current_time;
      odometry_message.pose.pose.position.x = odometry_->getX();
      odometry_message.pose.pose.position.y = odometry_->getY();
      odometry_message.pose.pose.position.z = 0.0;

      tf2::Quaternion orientation;
      orientation.setRPY(0.0, 0.0, odometry_->getHeading());
      odometry_message.pose.pose.orientation.x = orientation.x();
      odometry_message.pose.pose.orientation.y = orientation.y();
      odometry_message.pose.pose.orientation.z = orientation.z();
      odometry_message.pose.pose.orientation.w = orientation.w();

      odometry_message.twist.twist.linear.x = odometry_->getLinear();
      odometry_message.twist.twist.angular.z = odometry_->getAngular();

      // Set covariances
      for (size_t index = 0; index < 6; ++index)
      {
        // 0, 7, 14, 21, 28, 35
        const size_t diagonal_index = index * 7;
        odometry_message.pose.covariance[diagonal_index] = params_.pose_covariance_diagonal[index];
        odometry_message.twist.covariance[diagonal_index] = params_.twist_covariance_diagonal[index];
      }
      realtime_odometry_publisher_->unlockAndPublish();
    }

    if (params_.enable_odom_tf && realtime_odometry_transform_publisher_->trylock())
    {
      realtime_odometry_transform_publisher_->msg_.transforms.resize(1);
      auto & transform = realtime_odometry_transform_publisher_->msg_.transforms[0];

      transform.header.stamp = current_time;
      transform.header.frame_id = params_.odom_frame_id;
      transform.child_frame_id = params_.base_frame_id;
      transform.transform.translation.x = odometry_->getX();
      transform.transform.translation.y = odometry_->getY();
      transform.transform.translation.z = 0.0;

      tf2::Quaternion orientation;
      orientation.setRPY(0.0, 0.0, odometry_->getHeading());
      transform.transform.rotation.x = orientation.x();
      transform.transform.rotation.y = orientation.y();
      transform.transform.rotation.z = orientation.z();
      transform.transform.rotation.w = orientation.w();

      realtime_odometry_transform_publisher_->unlockAndPublish();
    }
  }

  // Publish limited velocity
  if (params_.publish_limited_velocity && realtime_limited_velocity_publisher_->trylock())
  {
    auto & limited_velocity_command = realtime_limited_velocity_publisher_->msg_;
    limited_velocity_command.header.stamp = current_time;
    limited_velocity_command.twist.linear.x = linear_command;
    limited_velocity_command.twist.angular.z = angular_command;
    realtime_limited_velocity_publisher_->unlockAndPublish();
  }

  // Update time for next iteration
  last_update_time_ = get_node()->get_clock()->now();

  is_halted = false;
  return controller_interface::return_type::OK;
}

void ThreeWheelDriveController::reset_buffers()
{
  std::queue<std::array<double, 2>> empty_queue;
  previous_commands_.swap(empty_queue);
}

rcl_interfaces::msg::SetParametersResult ThreeWheelDriveController::dynamicParametersCallback(
  const std::vector<rclcpp::Parameter> & parameters)
{
  rcl_interfaces::msg::SetParametersResult result;
  result.successful = true;

  for (const auto & parameter : parameters)
  {
    if (parameter.get_name() == "rear_steering_zero_offset")
    {
      if (parameter.get_type() == rclcpp::ParameterType::PARAMETER_DOUBLE)
      {
        rear_steering_zero_offset_ = parameter.as_double();
        RCLCPP_INFO(
          get_node()->get_logger(),
          "Updated rear_steering_zero_offset to: %f", rear_steering_zero_offset_);
      }
      else
      {
        result.successful = false;
        result.reason = "rear_steering_zero_offset must be a double";
      }
    }
  }

  return result;
}

}  // namespace three_wheel_drive_controller

#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(
  three_wheel_drive_controller::ThreeWheelDriveController,
  controller_interface::ControllerInterface)