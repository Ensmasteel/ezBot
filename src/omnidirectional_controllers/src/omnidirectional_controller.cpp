// MIT License

// Copyright (c) 2022 Mateus Menezes

// Permission is hereby granted, free of charge, to any person obtaining a copy
// of this software and associated documentation files (the "Software"), to deal
// in the Software without restriction, including without limitation the rights
// to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
// copies of the Software, and to permit persons to whom the Software is
// furnished to do so, subject to the following conditions:

// The above copyright notice and this permission notice shall be included in all
// copies or substantial portions of the Software.

// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
// IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
// FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
// AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
// LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
// OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
// SOFTWARE.

#include "omnidirectional_controllers/omnidirectional_controller.hpp"


#include <chrono>  // NOLINT
#include <cmath>
#include <exception>

#include "geometry_msgs/msg/twist.hpp"
#include "geometry_msgs/msg/twist_stamped.hpp"
#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "lifecycle_msgs/msg/state.hpp"
#include "rclcpp/logging.hpp"
#include "tf2/LinearMath/Quaternion.h"

namespace {
constexpr auto DEFAULT_COMMAND_TOPIC = "~/cmd_vel";
constexpr auto DEFAULT_COMMAND_UNSTAMPED_TOPIC = "~/cmd_vel_unstamped";
constexpr auto DEFAULT_COMMAND_OUT_TOPIC = "~/cmd_vel_out";
constexpr auto DEFAULT_ODOMETRY_TOPIC = "~/odom";
constexpr auto DEFAULT_TRANSFORM_TOPIC = "/tf";
constexpr auto WHEELS_QUANTITY = 4;
}  // namespace

namespace omnidirectional_controllers {

using namespace std::chrono_literals;   // NOLINT
using controller_interface::interface_configuration_type;
using controller_interface::InterfaceConfiguration;
using hardware_interface::HW_IF_POSITION;
using hardware_interface::HW_IF_VELOCITY;
using lifecycle_msgs::msg::State;
using std::placeholders::_1;

OmnidirectionalController::OmnidirectionalController()
  : controller_interface::ControllerInterface()
  , cmd_vel_(std::make_shared<geometry_msgs::msg::TwistStamped>()) {}

CallbackReturn OmnidirectionalController::on_init() {
  this->node_ = this->get_node();

  try {
    auto_declare<std::vector<std::string>>("wheel_names", std::vector<std::string>());

    auto_declare<double>("robot_radius", robot_params_.robot_radius);
    auto_declare<double>("wheel_radius", robot_params_.wheel_radius);
    auto_declare<double>("gamma", robot_params_.gamma);

    auto_declare<std::string>("odom_frame_id", odom_params_.odom_frame_id);
    auto_declare<std::string>("base_frame_id", odom_params_.base_frame_id);
    auto_declare<std::string>("odom_numeric_integration_method",
      odom_params_.odom_numeric_integration_method);
    auto_declare<std::vector<double>>("pose_covariance_diagonal", std::vector<double>());
    auto_declare<std::vector<double>>("twist_covariance_diagonal", std::vector<double>());
    auto_declare<bool>("open_loop", odom_params_.open_loop);
    auto_declare<bool>("enable_odom_tf", odom_params_.enable_odom_tf);

    auto_declare<double>("cmd_vel_timeout", cmd_vel_timeout_.count() / 1000.0);
    auto_declare<int>("velocity_rolling_window_size", 10);
    auto_declare<bool>("use_stamped_vel", use_stamped_vel_);
  } catch (const std::exception & e) {
    fprintf(stderr, "Exception thrown during init stage with message: %s \n", e.what());
    return CallbackReturn::ERROR;
  }

  return CallbackReturn::SUCCESS;
}

InterfaceConfiguration OmnidirectionalController::command_interface_configuration() const {
  std::vector<std::string> conf_names;

  for (const auto & joint_name : wheel_names_) {
    conf_names.push_back(joint_name + "/" + HW_IF_VELOCITY);
  }

  return {interface_configuration_type::INDIVIDUAL, conf_names};
}

InterfaceConfiguration OmnidirectionalController::state_interface_configuration() const {
  std::vector<std::string> conf_names;
  for (const auto & joint_name : wheel_names_) {
    conf_names.push_back(joint_name + "/" + HW_IF_VELOCITY);
  }
  return {interface_configuration_type::INDIVIDUAL, conf_names};
}

CallbackReturn OmnidirectionalController::on_configure(
    const rclcpp_lifecycle::State & previous_state) {
  auto logger = node_->get_logger();

  RCLCPP_DEBUG(logger,
    "Called on_configure. Previous state was %s",
    previous_state.label().c_str());

  // update parameters
  wheel_names_ = node_->get_parameter("wheel_names").as_string_array();

  if (wheel_names_.size() != WHEELS_QUANTITY) {
    RCLCPP_ERROR(
      logger, "The number of wheels [%zu] and the required [%d] are different",
      wheel_names_.size(), WHEELS_QUANTITY);
    return CallbackReturn::ERROR;
  }

  if (wheel_names_.empty()) {
    RCLCPP_ERROR(logger, "Wheel names parameters are empty!");
    return CallbackReturn::ERROR;
  }

  robot_params_.robot_radius = node_->get_parameter("robot_radius").as_double();
  robot_params_.wheel_radius = node_->get_parameter("wheel_radius").as_double();
  robot_params_.gamma = node_->get_parameter("gamma").as_double();
  robot_params_.gamma = DEG2RAD(robot_params_.gamma);

  omni_robot_kinematics_.setRobotParams(robot_params_);
  odometry_.setRobotParams(robot_params_);

  odom_params_.odom_frame_id = node_->get_parameter("odom_frame_id").as_string();
  odom_params_.base_frame_id = node_->get_parameter("base_frame_id").as_string();

  auto pose_diagonal = node_->get_parameter("pose_covariance_diagonal").as_double_array();
  std::copy(
    pose_diagonal.begin(), pose_diagonal.end(), odom_params_.pose_covariance_diagonal.begin());

  auto twist_diagonal = node_->get_parameter("twist_covariance_diagonal").as_double_array();
  std::copy(
    twist_diagonal.begin(), twist_diagonal.end(), odom_params_.twist_covariance_diagonal.begin());

  odom_params_.open_loop = node_->get_parameter("open_loop").as_bool();
  odom_params_.enable_odom_tf = node_->get_parameter("enable_odom_tf").as_bool();
  odom_params_.odom_numeric_integration_method = node_->get_parameter(
    "odom_numeric_integration_method").as_string();

  if (odom_params_.odom_numeric_integration_method != EULER_FORWARD &&
      odom_params_.odom_numeric_integration_method != RUNGE_KUTTA2) {
    RCLCPP_WARN(logger,
      "Invalid numeric integration got: %s. Using default %s intead",
      odom_params_.odom_numeric_integration_method.c_str(),
      EULER_FORWARD);
      odom_params_.odom_numeric_integration_method = EULER_FORWARD;
  }
  odometry_.setNumericIntegrationMethod(odom_params_.odom_numeric_integration_method);

  cmd_vel_timeout_ = std::chrono::milliseconds{
    static_cast<int>(node_->get_parameter("cmd_vel_timeout").as_double() * 1000.0)};
  use_stamped_vel_ = node_->get_parameter("use_stamped_vel").as_bool();

  // checking parameters of speed, acceleration, and jerk limits
  //linear
  auto linear_has_velocity_limits = node_->get_parameter("linear_has_velocity_limits");
  if (linear_has_velocity_limits.get_type() == rclcpp::ParameterType::PARAMETER_NOT_SET) {
    RCLCPP_WARN(logger, "Parameter 'linear_has_velocity_limits' not set. Using default value.");
    odom_params_.linear_has_velocity_limits = false;
  } else {
    odom_params_.linear_has_velocity_limits = linear_has_velocity_limits.as_bool();
  }
  if (odom_params_.linear_has_velocity_limits) {
    odom_params_.linear_min_velocity = node_->get_parameter("min_velocity").as_double();
    odom_params_.linear_max_velocity = node_->get_parameter("max_velocity").as_double();
  }

  auto linear_has_acceleration_limits = node_->get_parameter("linear_has_acceleration_limits");
  if (linear_has_acceleration_limits.get_type() == rclcpp::ParameterType::PARAMETER_NOT_SET){
    RCLCPP_WARN(logger, "Parameter 'linear_has_acceleration_limits' not set. Using default value.");
    odom_params_.linear_has_acceleration_limits = false;
  } else {
    odom_params_.linear_has_acceleration_limits = linear_has_acceleration_limits.as_bool();
  }
  if (odom_params_.linear_has_acceleration_limits) {
    odom_params_.linear_min_acceleration = node_->get_parameter("min_acceleration").as_double();
    odom_params_.linear_max_acceleration = node_->get_parameter("max_acceleration").as_double();
  }

  auto linear_has_jerk_limits = node_->get_parameter("linear_has_jerk_limits");
  if (linear_has_jerk_limits.get_type() == rclcpp::ParameterType::PARAMETER_NOT_SET){
    RCLCPP_WARN(logger, "Parameter 'linear_has_jerk_limits' not set. Using default value.");
    odom_params_.linear_has_jerk_limits = false;
  } else {
    odom_params_.linear_has_jerk_limits = linear_has_jerk_limits.as_bool();
  }
  if (odom_params_.linear_has_jerk_limits) {
    odom_params_.linear_min_jerk = node_->get_parameter("min_jerk").as_double();
    odom_params_.linear_max_jerk = node_->get_parameter("max_jerk").as_double();
  }


  // angular
  auto angular_has_velocity_limits = node_->get_parameter("angular_has_velocity_limits");
  if (angular_has_velocity_limits.get_type() == rclcpp::ParameterType::PARAMETER_NOT_SET) {
    RCLCPP_WARN(logger, "Parameter 'angular_has_velocity_limits' not set. Using default value.");
    odom_params_.angular_has_velocity_limits = false;
  } else {
    odom_params_.angular_has_velocity_limits = angular_has_velocity_limits.as_bool();
  }
  if (odom_params_.angular_has_velocity_limits) {
    odom_params_.angular_min_velocity = node_->get_parameter("angular_min_velocity").as_double();
    odom_params_.angular_max_velocity = node_->get_parameter("angular_max_velocity").as_double();
  }

  auto angular_has_acceleration_limits = node_->get_parameter("angular_has_acceleration_limits");
  if (angular_has_acceleration_limits.get_type() == rclcpp::ParameterType::PARAMETER_NOT_SET){
    RCLCPP_WARN(logger, "Parameter 'angular_has_acceleration_limits' not set. Using default value.");
    odom_params_.angular_has_acceleration_limits = false;
  } else {
    odom_params_.angular_has_acceleration_limits = angular_has_acceleration_limits.as_bool();
  }
  if (odom_params_.angular_has_acceleration_limits) {
    odom_params_.angular_min_acceleration = node_->get_parameter("angular_min_acceleration").as_double();
    odom_params_.angular_max_acceleration = node_->get_parameter("angular_max_acceleration").as_double();
  }

  auto angular_has_jerk_limits = node_->get_parameter("angular_has_jerk_limits");
  if (angular_has_jerk_limits.get_type() == rclcpp::ParameterType::PARAMETER_NOT_SET){
    RCLCPP_WARN(logger, "Parameter 'angular_has_jerk_limits' not set. Using default value.");
    odom_params_.angular_has_jerk_limits = false;
  } else {
    odom_params_.angular_has_jerk_limits = angular_has_jerk_limits.as_bool();
  }
  if (odom_params_.angular_has_jerk_limits) {
    odom_params_.angular_min_jerk = node_->get_parameter("angular_min_jerk").as_double();
    odom_params_.angular_max_jerk = node_->get_parameter("angular_max_jerk").as_double();
  }


  limiter_linear_ = diff_drive_controller::SpeedLimiter(
    odom_params_.linear_has_velocity_limits, odom_params_.linear_has_acceleration_limits,
    odom_params_.linear_has_jerk_limits, odom_params_.linear_min_velocity, odom_params_.linear_max_velocity,
    odom_params_.linear_min_acceleration, odom_params_.linear_max_acceleration, odom_params_.linear_min_jerk,
    odom_params_.linear_max_jerk);
  limiter_angular_ = diff_drive_controller::SpeedLimiter(
    odom_params_.angular_has_velocity_limits, odom_params_.angular_has_acceleration_limits,
    odom_params_.angular_has_jerk_limits, odom_params_.angular_min_velocity, odom_params_.angular_max_velocity,
    odom_params_.angular_min_acceleration, odom_params_.angular_max_acceleration,
    odom_params_.angular_min_jerk, odom_params_.angular_max_jerk);



  // initialize command subscriber
  if (use_stamped_vel_) {
    vel_cmd_subscriber_ = node_->create_subscription<geometry_msgs::msg::TwistStamped>(
      DEFAULT_COMMAND_TOPIC,
      rclcpp::SystemDefaultsQoS(),
      std::bind(&OmnidirectionalController::velocityCommandStampedCallback, this, _1));
  } else {
    vel_cmd_unstamped_subscriber_ = node_->create_subscription<geometry_msgs::msg::Twist>(
      DEFAULT_COMMAND_UNSTAMPED_TOPIC,
      rclcpp::SystemDefaultsQoS(),
      std::bind(&OmnidirectionalController::velocityCommandUnstampedCallback, this, _1));
  }

  // initialize odometry publisher and messasge
  odometry_publisher_ = node_->create_publisher<nav_msgs::msg::Odometry>(
    DEFAULT_ODOMETRY_TOPIC, rclcpp::SystemDefaultsQoS());

  // limit the publication on the topics /odom and /tf
  publish_rate_ = node_->get_parameter("publish_rate").as_double();
  publish_period_ = rclcpp::Duration::from_seconds(1.0 / publish_rate_);
  previous_publish_timestamp_ = node_->get_clock()->now();

  // initialize odom values zeros
  odometry_message_.twist =
      geometry_msgs::msg::TwistWithCovariance(rosidl_runtime_cpp::MessageInitialization::ALL);

  constexpr size_t NUM_DIMENSIONS = 6;
  for (size_t index = 0; index < 6; ++index) {
    // 0, 7, 14, 21, 28, 35
    const size_t diagonal_index = NUM_DIMENSIONS * index + index;
    odometry_message_.pose.covariance[diagonal_index] =
      odom_params_.pose_covariance_diagonal[index];
    odometry_message_.twist.covariance[diagonal_index] =
      odom_params_.twist_covariance_diagonal[index];
  }

  // initialize transform publisher and message
  odometry_transform_publisher_ = node_->create_publisher<tf2_msgs::msg::TFMessage>(
    DEFAULT_TRANSFORM_TOPIC, rclcpp::SystemDefaultsQoS());

  // keeping track of odom and base_link transforms only
  odometry_transform_message_.transforms.resize(1);
  odometry_transform_message_.transforms.front().header.frame_id = odom_params_.odom_frame_id;
  odometry_transform_message_.transforms.front().child_frame_id = odom_params_.base_frame_id;

  return CallbackReturn::SUCCESS;
}

CallbackReturn OmnidirectionalController::on_activate(
  const rclcpp_lifecycle::State & previous_state) {
  auto logger = node_->get_logger();

  RCLCPP_DEBUG(logger,
    "Called on_activate. Previous state was %s",
    previous_state.label().c_str());

  if (wheel_names_.empty()) {
    RCLCPP_ERROR(logger, "No wheel names specified");
    return CallbackReturn::ERROR;
  }

  // register handles
  registered_wheel_handles_.reserve(wheel_names_.size());
  for (const auto & wheel_name : wheel_names_) {
   std::string interface_name = wheel_name + "/" + HW_IF_VELOCITY;
    const auto state_handle = std::find_if(state_interfaces_.cbegin(), state_interfaces_.cend(),
      [&interface_name](const auto & interface) {
        return interface.get_name() == interface_name;
      });

    if (state_handle == state_interfaces_.cend()) {
      RCLCPP_ERROR(logger, "Unable to obtain joint state handle for %s", wheel_name.c_str());
      return CallbackReturn::ERROR;
    }

    const auto command_handle = std::find_if(
      command_interfaces_.begin(), command_interfaces_.end(),
      [&interface_name](const auto & interface) {
        return interface.get_name() == interface_name;
      });

    if (command_handle == command_interfaces_.end()) {
      RCLCPP_ERROR(logger, "Unable to obtain joint command handle for %s", wheel_name.c_str());
      return CallbackReturn::ERROR;
    }

    registered_wheel_handles_.emplace_back(
      WheelHandle{std::ref(*state_handle), std::ref(*command_handle)});

    RCLCPP_INFO(logger, "Got command interface: %s", command_handle->get_name().c_str());
    RCLCPP_INFO(logger, "Got state interface: %s", state_handle->get_name().c_str());
  }

  subscriber_is_active_ = true;

  RCLCPP_DEBUG(node_->get_logger(), "Subscriber and publisher are now active.");
  previous_update_timestamp_ = node_->get_clock()->now();

  return CallbackReturn::SUCCESS;
}

CallbackReturn OmnidirectionalController::on_deactivate(
  const rclcpp_lifecycle::State & previous_state) {
  RCLCPP_DEBUG(node_->get_logger(),
    "Called on_deactivate. Previous state was %s",
    previous_state.label().c_str());
  subscriber_is_active_ = false;
  odometry_.reset();
  return CallbackReturn::SUCCESS;
}

CallbackReturn OmnidirectionalController::on_cleanup(
  const rclcpp_lifecycle::State & previous_state) {
  RCLCPP_DEBUG(node_->get_logger(),
    "Called on_cleanup. Previous state was %s",
    previous_state.label().c_str());
  return CallbackReturn::SUCCESS;
}

CallbackReturn OmnidirectionalController::on_error(
  const rclcpp_lifecycle::State & previous_state) {
  RCLCPP_DEBUG(node_->get_logger(),
    "Called on_error. Previous state was %s",
    previous_state.label().c_str());
  return CallbackReturn::SUCCESS;
}

CallbackReturn OmnidirectionalController::on_shutdown(
  const rclcpp_lifecycle::State & previous_state) {
  RCLCPP_DEBUG(node_->get_logger(),
    "Called on_error. Previous state was %s",
    previous_state.label().c_str());
  return CallbackReturn::SUCCESS;
}

void OmnidirectionalController::velocityCommandStampedCallback(
  const geometry_msgs::msg::TwistStamped::SharedPtr cmd_vel) {
  if (!subscriber_is_active_) {
    RCLCPP_WARN(node_->get_logger(), "Can't accept new commands. subscriber is inactive");
    return;
  }
  if ((cmd_vel->header.stamp.sec == 0) && (cmd_vel->header.stamp.nanosec == 0)) {
    RCLCPP_WARN_ONCE(
      node_->get_logger(),
      "Received TwistStamped with zero timestamp, setting it to current "
      "time, this message will only be shown once");
    cmd_vel->header.stamp = node_->get_clock()->now();
  }

  this->cmd_vel_ = std::move(cmd_vel);
}

void OmnidirectionalController::velocityCommandUnstampedCallback(
  const geometry_msgs::msg::Twist::SharedPtr cmd_vel) {
  if (!subscriber_is_active_) {
    RCLCPP_WARN(node_->get_logger(), "Can't accept new commands. subscriber is inactive");
    return;
  }

  this->cmd_vel_->twist = *cmd_vel;
  this->cmd_vel_->header.stamp = node_->get_clock()->now();
}

controller_interface::return_type OmnidirectionalController::update(
    const rclcpp::Time & time,
    const rclcpp::Duration & period) {
  auto logger = node_->get_logger();
  const auto current_time = time;

  const auto dt = current_time - cmd_vel_->header.stamp;
  // Brake if cmd_vel has timeout, override the stored command
  if (dt > cmd_vel_timeout_) {
    cmd_vel_->twist.linear.x = 0.0;
    cmd_vel_->twist.linear.y = 0.0;
    cmd_vel_->twist.angular.z = 0.0;
  }

  Twist command = *cmd_vel_;

  if (odom_params_.open_loop) {
    odometry_.updateOpenLoop(
      {cmd_vel_->twist.linear.x,  cmd_vel_->twist.linear.y, cmd_vel_->twist.angular.z},
      period.seconds());
  } else {
    std::vector<double> wheels_angular_velocity({0, 0, 0, 0});
    wheels_angular_velocity[0] = registered_wheel_handles_[0].velocity_state.get().get_value();
    wheels_angular_velocity[1] = registered_wheel_handles_[1].velocity_state.get().get_value();
    wheels_angular_velocity[2] = registered_wheel_handles_[2].velocity_state.get().get_value();
    wheels_angular_velocity[3] = registered_wheel_handles_[3].velocity_state.get().get_value();
    try {
      odometry_.update(wheels_angular_velocity, period.seconds());
    } catch(const std::runtime_error& e) {
      RCLCPP_ERROR(logger, e.what());
      rclcpp::shutdown();
    }
  }

  tf2::Quaternion orientation;
  orientation.setRPY(0.0, 0.0, odometry_.getPose().theta);

  if (previous_publish_timestamp_ + publish_period_ < current_time) {
    previous_publish_timestamp_ += publish_period_;
    odometry_message_.header.stamp = current_time;
    odometry_message_.pose.pose.position.x = odometry_.getPose().x;
    odometry_message_.pose.pose.position.y = odometry_.getPose().y;
    odometry_message_.pose.pose.orientation.x = orientation.x();
    odometry_message_.pose.pose.orientation.y = orientation.y();
    odometry_message_.pose.pose.orientation.z = orientation.z();
    odometry_message_.pose.pose.orientation.w = orientation.w();
    odometry_message_.twist.twist.linear = cmd_vel_->twist.linear;
    odometry_message_.twist.twist.angular = cmd_vel_->twist.angular;
    odometry_publisher_->publish(odometry_message_);
  }

  // if (odom_params_.enable_odom_tf) {
  //   odometry_transform_message_.header.stamp = current_time;
  //   odometry_transform_message_.transform.translation.x = odometry_.getPose().x;
  //   odometry_transform_message_.transform.translation.y = odometry_.getPose().y;
  //   odometry_transform_message_.transform.rotation.x = orientation.x();
  //   odometry_transform_message_.transform.rotation.y = orientation.y();
  //   odometry_transform_message_.transform.rotation.z = orientation.z();
  //   odometry_transform_message_.transform.rotation.w = orientation.w();
  // }

  auto & last_command = previous_commands_.back().twist;
  auto & second_to_last_command = previous_commands_.front().twist;

  limiter_linear_.limit(command.twist.linear.x, second_to_last_command.linear.x, last_command.linear.x, 
                        period.seconds());
  limiter_linear_.limit(command.twist.linear.y, second_to_last_command.linear.y, last_command.linear.y,
                        period.seconds());

  limiter_angular_.limit(command.twist.angular.z, second_to_last_command.angular.z, last_command.angular.z,
                        period.seconds());


  previous_commands_.pop();
  previous_commands_.emplace(command);

  // Compute wheels velocities:
  RobotVelocity body_vel_setpoint;
  body_vel_setpoint.vx = cmd_vel_->twist.linear.x;
  body_vel_setpoint.vy = cmd_vel_->twist.linear.y;
  body_vel_setpoint.omega = cmd_vel_->twist.angular.z;

  std::vector<double> wheels_angular_velocity;
  wheels_angular_velocity = omni_robot_kinematics_.getWheelsAngularVelocities(body_vel_setpoint);

  // Set wheels velocities:
  registered_wheel_handles_[0].velocity_command.get().set_value(wheels_angular_velocity.at(0));
  registered_wheel_handles_[1].velocity_command.get().set_value(wheels_angular_velocity.at(1));
  registered_wheel_handles_[2].velocity_command.get().set_value(wheels_angular_velocity.at(2));
  registered_wheel_handles_[3].velocity_command.get().set_value(wheels_angular_velocity.at(3));

  return controller_interface::return_type::OK;
}

OmnidirectionalController::~OmnidirectionalController() {}

}  // namespace omnidirectional_controllers

#include "class_loader/register_macro.hpp"

CLASS_LOADER_REGISTER_CLASS(
  omnidirectional_controllers::OmnidirectionalController, controller_interface::ControllerInterface)
