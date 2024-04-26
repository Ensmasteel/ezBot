#include "omnidrive_rppico/omnibot_system.hpp"

#include <chrono>
#include <cmath>
#include <limits>
#include <memory>
#include <vector>

#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "rclcpp/rclcpp.hpp"

namespace omnidrive_rppico
{
hardware_interface::CallbackReturn OmniDriveRpPicoHardware::on_init(
  const hardware_interface::HardwareInfo & info)
{
  if (
    hardware_interface::SystemInterface::on_init(info) !=
    hardware_interface::CallbackReturn::SUCCESS)
  {
    return hardware_interface::CallbackReturn::ERROR;
  }


  cfg_.front_wheel_name = info_.hardware_parameters["front_wheel_name"];
  cfg_.left_wheel_name = info_.hardware_parameters["left_wheel_name"];
  cfg_.back_wheel_name = info_.hardware_parameters["back_wheel_name"];
  cfg_.right_wheel_name = info_.hardware_parameters["right_wheel_name"];
  
  cfg_.loop_rate = std::stof(info_.hardware_parameters["loop_rate"]);
  RCLCPP_INFO(rclcpp::get_logger("OmniDriveRpPicoHardware"), "Setting params from URDF");

  cfg_.device = info_.hardware_parameters["device"];
  cfg_.baud_rate = std::stoi(info_.hardware_parameters["baudrate"]);
  cfg_.timeout_ms = std::stoi(info_.hardware_parameters["timeout_ms"]);
  cfg_.enc_counts_per_rev = std::stoi(info_.hardware_parameters["enc_counts_per_rev"]);
  if (info_.hardware_parameters.count("pid_p") > 0)
  {
    cfg_.pid_p = std::stoi(info_.hardware_parameters["pid_p"]);
    cfg_.pid_d = std::stoi(info_.hardware_parameters["pid_d"]);
    cfg_.pid_i = std::stoi(info_.hardware_parameters["pid_i"]);
    cfg_.pid_o = std::stoi(info_.hardware_parameters["pid_o"]);
  }
  else
  {
    RCLCPP_INFO(rclcpp::get_logger("OmniDriveRpPicoHardware"), "PID values not supplied, using defaults.");
  }
  

  wheel_f_.setup(cfg_.front_wheel_name, cfg_.enc_counts_per_rev);
  wheel_l_.setup(cfg_.left_wheel_name, cfg_.enc_counts_per_rev);
  wheel_b_.setup(cfg_.back_wheel_name, cfg_.enc_counts_per_rev);
  wheel_r_.setup(cfg_.right_wheel_name, cfg_.enc_counts_per_rev);

  RCLCPP_INFO(rclcpp::get_logger("OmniDriveRpPicoHardware"), "Setting up hardware interfaces");
  for (const hardware_interface::ComponentInfo & joint : info_.joints)
  {
    RCLCPP_INFO(
      rclcpp::get_logger("OmniDriveRpPicoHardware"), "Loading joint '%s'", joint.name.c_str());

    // DiffBotSystem has exactly two states and one command interface on each joint
    if (joint.command_interfaces.size() != 1)
    {
      RCLCPP_FATAL(
        rclcpp::get_logger("OmniDriveRpPicoHardware"),
        "Joint '%s' has %zu command interfaces found. 1 expected.", joint.name.c_str(),
        joint.command_interfaces.size());
      return hardware_interface::CallbackReturn::ERROR;
    }

    if (joint.command_interfaces[0].name != hardware_interface::HW_IF_VELOCITY)
    {
      RCLCPP_FATAL(
        rclcpp::get_logger("OmniDriveRpPicoHardware"),
        "Joint '%s' have %s command interfaces found. '%s' expected.", joint.name.c_str(),
        joint.command_interfaces[0].name.c_str(), hardware_interface::HW_IF_VELOCITY);
      return hardware_interface::CallbackReturn::ERROR;
    }

    if (joint.state_interfaces.size() != 2)
    {
      RCLCPP_FATAL(
        rclcpp::get_logger("OmniDriveRpPicoHardware"),
        "Joint '%s' has %zu state interface. 2 expected.", joint.name.c_str(),
        joint.state_interfaces.size());
      return hardware_interface::CallbackReturn::ERROR;
    }

    if (joint.state_interfaces[0].name != hardware_interface::HW_IF_POSITION)
    {
      RCLCPP_FATAL(
        rclcpp::get_logger("OmniDriveRpPicoHardware"),
        "Joint '%s' have '%s' as first state interface. '%s' expected.", joint.name.c_str(),
        joint.state_interfaces[0].name.c_str(), hardware_interface::HW_IF_POSITION);
      return hardware_interface::CallbackReturn::ERROR;
    }

    if (joint.state_interfaces[1].name != hardware_interface::HW_IF_VELOCITY)
    {
      RCLCPP_FATAL(
        rclcpp::get_logger("OmniDriveRpPicoHardware"),
        "Joint '%s' have '%s' as second state interface. '%s' expected.", joint.name.c_str(),
        joint.state_interfaces[1].name.c_str(), hardware_interface::HW_IF_VELOCITY);
      return hardware_interface::CallbackReturn::ERROR;
    }
  }
  RCLCPP_INFO(rclcpp::get_logger("OmniDriveRpPicoHardware"), "Successfully set up hardware interfaces");
  return hardware_interface::CallbackReturn::SUCCESS;
}

std::vector<hardware_interface::StateInterface> OmniDriveRpPicoHardware::export_state_interfaces()
{
  std::vector<hardware_interface::StateInterface> state_interfaces;
  RCLCPP_INFO(rclcpp::get_logger("OmniDriveRpPicoHardware"), "Exporting state interfaces");
  
  state_interfaces.emplace_back(hardware_interface::StateInterface(
    wheel_f_.name, hardware_interface::HW_IF_POSITION, &wheel_f_.pos));
  state_interfaces.emplace_back(hardware_interface::StateInterface(
    wheel_f_.name, hardware_interface::HW_IF_VELOCITY, &wheel_f_.vel));
  
  state_interfaces.emplace_back(hardware_interface::StateInterface(
    wheel_l_.name, hardware_interface::HW_IF_POSITION, &wheel_l_.pos));
  state_interfaces.emplace_back(hardware_interface::StateInterface(
    wheel_l_.name, hardware_interface::HW_IF_VELOCITY, &wheel_l_.vel));

  state_interfaces.emplace_back(hardware_interface::StateInterface(
    wheel_b_.name, hardware_interface::HW_IF_POSITION, &wheel_b_.pos));
  state_interfaces.emplace_back(hardware_interface::StateInterface(
    wheel_b_.name, hardware_interface::HW_IF_VELOCITY, &wheel_b_.vel));  

  state_interfaces.emplace_back(hardware_interface::StateInterface(
    wheel_r_.name, hardware_interface::HW_IF_POSITION, &wheel_r_.pos));
  state_interfaces.emplace_back(hardware_interface::StateInterface(
    wheel_r_.name, hardware_interface::HW_IF_VELOCITY, &wheel_r_.vel));

  return state_interfaces;
}

std::vector<hardware_interface::CommandInterface> OmniDriveRpPicoHardware::export_command_interfaces()
{
  std::vector<hardware_interface::CommandInterface> command_interfaces;

  command_interfaces.emplace_back(hardware_interface::CommandInterface(
    wheel_f_.name, hardware_interface::HW_IF_VELOCITY, &wheel_f_.cmd));

  command_interfaces.emplace_back(hardware_interface::CommandInterface(
    wheel_l_.name, hardware_interface::HW_IF_VELOCITY, &wheel_l_.cmd));

  command_interfaces.emplace_back(hardware_interface::CommandInterface(
    wheel_b_.name, hardware_interface::HW_IF_VELOCITY, &wheel_b_.cmd));

  command_interfaces.emplace_back(hardware_interface::CommandInterface(
    wheel_r_.name, hardware_interface::HW_IF_VELOCITY, &wheel_r_.cmd));

  return command_interfaces;
}

hardware_interface::CallbackReturn OmniDriveRpPicoHardware::on_configure(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  RCLCPP_INFO(rclcpp::get_logger("OmniDriveRpPicoHardware"), "Configuring ...please wait...");
  if (comms_.connected())
  {
    comms_.disconnect();
  }
  comms_.connect(cfg_.device, cfg_.baud_rate, cfg_.timeout_ms);
  RCLCPP_INFO(rclcpp::get_logger("OmniDriveRpPicoHardware"), "Successfully configured!");

  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn OmniDriveRpPicoHardware::on_cleanup(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  RCLCPP_INFO(rclcpp::get_logger("OmniDriveRpPicoHardware"), "Cleaning up ...please wait...");
  if (comms_.connected())
  {
    comms_.disconnect();
  }
  RCLCPP_INFO(rclcpp::get_logger("OmniDriveRpPicoHardware"), "Successfully cleaned up!");

  return hardware_interface::CallbackReturn::SUCCESS;
}


hardware_interface::CallbackReturn OmniDriveRpPicoHardware::on_activate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  RCLCPP_INFO(rclcpp::get_logger("OmniDriveRpPicoHardware"), "Activating ...please wait...");
  if (!comms_.connected())
  {
    return hardware_interface::CallbackReturn::ERROR;
  }
  if (cfg_.pid_p > 0)
  {
    comms_.set_pid_values(cfg_.pid_p,cfg_.pid_d,cfg_.pid_i,cfg_.pid_o);
  }
  RCLCPP_INFO(rclcpp::get_logger("OmniDriveRpPicoHardware"), "Successfully activated!");

  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn OmniDriveRpPicoHardware::on_deactivate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  RCLCPP_INFO(rclcpp::get_logger("OmniDriveRpPicoHardware"), "Deactivating ...please wait...");
  RCLCPP_INFO(rclcpp::get_logger("OmniDriveRpPicoHardware"), "Successfully deactivated!");

  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::return_type OmniDriveRpPicoHardware::read(
  const rclcpp::Time & /*time*/, const rclcpp::Duration & period)
{
  if (!comms_.connected())
  {
    return hardware_interface::return_type::ERROR;
  }

  comms_.read_encoder_values(wheel_f_.enc, wheel_l_.enc, wheel_b_.enc, wheel_r_.enc);

  double delta_seconds = period.seconds();

  double pos_prev = wheel_f_.pos;
  wheel_f_.pos = wheel_f_.calc_enc_angle();
  wheel_f_.vel = (wheel_f_.pos - pos_prev) / delta_seconds;
  
  pos_prev = wheel_f_.pos;
  wheel_l_.pos = wheel_l_.calc_enc_angle();
  wheel_l_.vel = (wheel_l_.pos - pos_prev) / delta_seconds;

  pos_prev = wheel_b_.pos;
  wheel_b_.pos = wheel_b_.calc_enc_angle();
  wheel_b_.vel = (wheel_b_.pos - pos_prev) / delta_seconds;

  pos_prev = wheel_r_.pos;
  wheel_r_.pos = wheel_r_.calc_enc_angle();
  wheel_r_.vel = (wheel_r_.pos - pos_prev) / delta_seconds;


  return hardware_interface::return_type::OK;
}

hardware_interface::return_type omnidrive_rppico ::OmniDriveRpPicoHardware::write(
  const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/)
{
  if (!comms_.connected())
  {
    return hardware_interface::return_type::ERROR;
  }

  int motor_f_counts_per_loop = wheel_f_.cmd / wheel_f_.rads_per_count;
  int motor_l_counts_per_loop = wheel_l_.cmd / wheel_l_.rads_per_count;
  int motor_b_counts_per_loop = wheel_b_.cmd / wheel_b_.rads_per_count;
  int motor_r_counts_per_loop = wheel_r_.cmd / wheel_r_.rads_per_count;
  


  comms_.set_motor_values(motor_f_counts_per_loop, 
                          motor_l_counts_per_loop,
                          motor_b_counts_per_loop, 
                          motor_r_counts_per_loop);
  return hardware_interface::return_type::OK;
}

}  // namespace omnidrive_rppico

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(
  omnidrive_rppico::OmniDriveRpPicoHardware, hardware_interface::SystemInterface)
