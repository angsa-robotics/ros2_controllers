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

#include <cmath>

#include "three_wheel_drive_controller/odometry.hpp"

namespace three_wheel_drive_controller
{
Odometry::Odometry(size_t velocity_rolling_window_size)
: timestamp_(0.0),
  x_(0.0),
  y_(0.0),
  heading_(0.0),
  linear_(0.0),
  angular_(0.0),
  wheel_separation_(0.0),
  wheelbase_(0.0),
  left_wheel_radius_(0.0),
  right_wheel_radius_(0.0),
  rear_wheel_radius_(0.0),
  left_wheel_old_pos_(0.0),
  right_wheel_old_pos_(0.0),
  rear_wheel_old_pos_(0.0),
  velocity_rolling_window_size_(velocity_rolling_window_size),
  linear_accumulator_(velocity_rolling_window_size),
  angular_accumulator_(velocity_rolling_window_size)
{
}

void Odometry::init(const rclcpp::Time & time)
{
  // Reset accumulators and timestamp:
  resetAccumulators();
  timestamp_ = time;
}

bool Odometry::update(double left_pos, double right_pos, double rear_pos, const rclcpp::Time & time)
{
  // We cannot estimate the speed with very small time intervals:
  const double dt = time.seconds() - timestamp_.seconds();
  if (dt < 0.0001)
  {
    return false;  // Interval too small to integrate with
  }

  // Get current wheel joint positions:
  const double left_wheel_cur_pos = left_pos * left_wheel_radius_;
  const double right_wheel_cur_pos = right_pos * right_wheel_radius_;
  const double rear_wheel_cur_pos = rear_pos * rear_wheel_radius_;

  // Estimate velocity of wheels using old and current position:
  const double left_wheel_est_vel = (left_wheel_cur_pos - left_wheel_old_pos_) / dt;
  const double right_wheel_est_vel = (right_wheel_cur_pos - right_wheel_old_pos_) / dt;
  const double rear_wheel_est_vel = (rear_wheel_cur_pos - rear_wheel_old_pos_) / dt;

  // Update old position with current:
  left_wheel_old_pos_ = left_wheel_cur_pos;
  right_wheel_old_pos_ = right_wheel_cur_pos;
  rear_wheel_old_pos_ = rear_wheel_cur_pos;

  updateFromVelocity(left_wheel_est_vel, right_wheel_est_vel, rear_wheel_est_vel, 0.0, time);

  return true;
}

bool Odometry::updateFromVelocity(double left_vel, double right_vel, double rear_vel, double rear_steering_pos, const rclcpp::Time & time)
{
  const double dt = time.seconds() - timestamp_.seconds();
  if (dt < 0.0001)
  {
    return false;  // Interval too small to integrate with
  }

  // For three-wheel robot with front differential drive and rear steered wheel
  // Use the front wheels for differential drive computation and rear wheel for validation
  const double front_linear = (left_vel + right_vel) * 0.5;
  const double front_angular = (right_vel - left_vel) / wheel_separation_;

  // Could also compute from rear wheel kinematics, but front differential is more reliable
  // Use front differential computation for primary odometry
  const double linear = front_linear;
  const double angular = front_angular;

  // Integrate odometry:
  integrateExact(linear, angular);

  timestamp_ = time;

  // Estimate speeds using a rolling mean to filter them out:
  linear_accumulator_.accumulate(linear);
  angular_accumulator_.accumulate(angular);

  linear_ = linear_accumulator_.getRollingMean();
  angular_ = angular_accumulator_.getRollingMean();

  return true;
}

void Odometry::updateOpenLoop(double linear, double angular, const rclcpp::Time & time)
{
  // Save last linear and angular velocity:
  linear_ = linear;
  angular_ = angular;

  // Integrate odometry:
  const double dt = time.seconds() - timestamp_.seconds();
  integrateExact(linear * dt, angular * dt);

  timestamp_ = time;
}

void Odometry::resetOdometry()
{
  x_ = 0.0;
  y_ = 0.0;
  heading_ = 0.0;
}

void Odometry::setWheelParams(double wheel_separation, double wheelbase, double left_wheel_radius, double right_wheel_radius, double rear_wheel_radius)
{
  wheel_separation_ = wheel_separation;
  wheelbase_ = wheelbase;
  left_wheel_radius_ = left_wheel_radius;
  right_wheel_radius_ = right_wheel_radius;
  rear_wheel_radius_ = rear_wheel_radius;
}

void Odometry::setVelocityRollingWindowSize(size_t velocity_rolling_window_size)
{
  velocity_rolling_window_size_ = velocity_rolling_window_size;
  resetAccumulators();
}

void Odometry::integrateRungeKutta2(double linear, double angular)
{
  const double direction = heading_ + angular * 0.5;

  // Runge-Kutta 2nd order integration:
  x_ += linear * cos(direction);
  y_ += linear * sin(direction);
  heading_ += angular;
}

void Odometry::integrateExact(double linear, double angular)
{
  if (fabs(angular) < 1e-6)
  {
    integrateRungeKutta2(linear, angular);
  }
  else
  {
    // Exact integration (should solve problems when angular is zero):
    const double heading_old = heading_;
    const double r = linear / angular;
    heading_ += angular;
    x_ += r * (sin(heading_) - sin(heading_old));
    y_ += -r * (cos(heading_) - cos(heading_old));
  }
}

void Odometry::resetAccumulators()
{
  linear_accumulator_ = RollingMeanAccumulator(velocity_rolling_window_size_);
  angular_accumulator_ = RollingMeanAccumulator(velocity_rolling_window_size_);
}

}  // namespace three_wheel_drive_controller