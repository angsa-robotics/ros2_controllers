#include <gmock/gmock.h>

#include <memory>
#include <utility>
#include <vector>

#include "controller_manager/controller_manager.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "geometry_msgs/msg/twist_stamped.hpp"
#include "hardware_interface/resource_manager.hpp"
#include "lifecycle_msgs/msg/state.hpp"
#include "rclcpp/executors/single_threaded_executor.hpp"
#include "rclcpp/rclcpp.hpp"
#include "ros2_control_test_assets/descriptions.hpp"
#include "three_wheel_drive_controller/three_wheel_drive_controller.hpp"

using CallbackReturn = controller_interface::CallbackReturn;
using hardware_interface::LoanedCommandInterface;
using hardware_interface::LoanedStateInterface;
using lifecycle_msgs::msg::State;
using testing::SizeIs;

class TestThreeWheelDriveController : public ::testing::Test
{
protected:
  void SetUp() override
  {
    // setup robot hardware
    hardware_system_ = std::make_unique<hardware_interface::ResourceManager>(
      ros2_control_test_assets::minimal_robot_urdf);
    executor_ = std::make_shared<rclcpp::executors::SingleThreadedExecutor>();

    // setup controller  
    controller_ = std::make_unique<three_wheel_drive_controller::ThreeWheelDriveController>();
  }

  void TearDown() override { controller_.reset(nullptr); }

  std::unique_ptr<hardware_interface::ResourceManager> hardware_system_;
  std::unique_ptr<three_wheel_drive_controller::ThreeWheelDriveController> controller_;
  rclcpp::Executor::SharedPtr executor_;
};

TEST_F(TestThreeWheelDriveController, initialization_test)
{
  const auto result = controller_->init("test_three_wheel_drive_controller", "");
  ASSERT_EQ(result, CallbackReturn::SUCCESS);

  EXPECT_EQ(controller_->get_state().id(), State::PRIMARY_STATE_UNCONFIGURED);
}

TEST_F(TestThreeWheelDriveController, configure_fails_without_parameters)
{
  const auto result = controller_->init("test_three_wheel_drive_controller", "");
  ASSERT_EQ(result, CallbackReturn::SUCCESS);

  EXPECT_EQ(controller_->on_configure(rclcpp_lifecycle::State()), CallbackReturn::ERROR);
}

int main(int argc, char ** argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  rclcpp::init(argc, argv);
  int result = RUN_ALL_TESTS();
  rclcpp::shutdown();
  return result;
}