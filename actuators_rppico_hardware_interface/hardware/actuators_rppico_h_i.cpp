#include "actuators_rppico_h_i/actuators_rppico_h_i.hpp"

#include <chrono>
#include <cmath>
#include <limits>
#include <memory>
#include <vector>

#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "rclcpp/rclcpp.hpp"

namespace actuators_rppico_h_i
{
hardware_interface::CallbackReturn ActuatorsRpPicoHardware::on_init(
  const hardware_interface::HardwareInfo & info)
{
  if (
    hardware_interface::SystemInterface::on_init(info) !=
    hardware_interface::CallbackReturn::SUCCESS)
  {
    return hardware_interface::CallbackReturn::ERROR;
  }

  number_of_servos = 0;
  for (int i = 1; i <= 12; i++) {
    std::string param_name = "servo" + std::to_string(i) + "_name";
    if (info_.hardware_parameters.count(param_name) > 0) {
      cfg_.servo_names[i] = info_.hardware_parameters[param_name];
      number_of_servos++;
    }
  }

  RCLCPP_INFO(rclcpp::get_logger("ActuatorsRpPicoHardware"), "%i servos found", number_of_servos);


  
  cfg_.loop_rate = std::stof(info_.hardware_parameters["loop_rate"]);
  RCLCPP_INFO(rclcpp::get_logger("ActuatorsRpPicoHardware"), "Setting params from URDF");

  cfg_.device = info_.hardware_parameters["device"];
  cfg_.baud_rate = std::stoi(info_.hardware_parameters["baudrate"]);
  cfg_.timeout_ms = std::stoi(info_.hardware_parameters["timeout_ms"]);
  
  

  for (int i = 1; i <= 12; i++) {
    if (cfg_.servo_names[i] != "") {
      /*RCLCPP_INFO(
        rclcpp::get_logger("ActuatorsRpPicoHardware"), "Setting up servo %s", cfg_.servo_names[i].c_str());*/
      servos_[i].setup(cfg_.servo_names[i], i);
    }
  }

  RCLCPP_INFO(rclcpp::get_logger("ActuatorsRpPicoHardware"), "Setting up hardware interfaces");
  for (const hardware_interface::ComponentInfo & joint : info_.joints)
  {
    RCLCPP_INFO(
      rclcpp::get_logger("ActuatorsRpPicoHardware"), "Loading joint '%s'", joint.name.c_str());

    // servos have exactly one states and one command interface on each joint
    if (joint.command_interfaces.size() != number_of_servos * 1)
    {
      RCLCPP_FATAL(
        rclcpp::get_logger("ActuatorsRpPicoHardware"),
        "Joint '%s' has %zu command interfaces found. %i expected.", joint.name.c_str(),
        joint.command_interfaces.size()), number_of_servos * 1;
      return hardware_interface::CallbackReturn::ERROR;
    }
    // TODO verify if every servo has a command interface
    if (joint.command_interfaces[0].name != hardware_interface::HW_IF_POSITION)
    {
      RCLCPP_FATAL(
        rclcpp::get_logger("ActuatorsRpPicoHardware"),
        "Joint '%s' have %s command interfaces found. '%s' expected.", joint.name.c_str(),
        joint.command_interfaces[0].name.c_str(), hardware_interface::HW_IF_POSITION);
      return hardware_interface::CallbackReturn::ERROR;
    }

    if (joint.state_interfaces.size() != number_of_servos * 1)
    {
      RCLCPP_FATAL(
        rclcpp::get_logger("ActuatorsRpPicoHardware"),
        "Joint '%s' has %zu state interface. 1 per servo expected ie %i.", joint.name.c_str(),
        joint.state_interfaces.size()), number_of_servos * 1;
      return hardware_interface::CallbackReturn::ERROR;
    }

    if (joint.state_interfaces[0].name != hardware_interface::HW_IF_POSITION)
    {
      RCLCPP_FATAL(
        rclcpp::get_logger("ActuatorsRpPicoHardware"),
        "Joint '%s' have '%s' as first state interface. '%s' expected.", joint.name.c_str(),
        joint.state_interfaces[0].name.c_str(), hardware_interface::HW_IF_POSITION);
      return hardware_interface::CallbackReturn::ERROR;
    }

   
  
  }
  RCLCPP_INFO(rclcpp::get_logger("ActuatorsRpPicoHardware"), "Successfully set up hardware interfaces");
  return hardware_interface::CallbackReturn::SUCCESS;
}
std::vector<hardware_interface::StateInterface> ActuatorsRpPicoHardware::export_state_interfaces()
{
  std::vector<hardware_interface::StateInterface> state_interfaces;
  RCLCPP_INFO(rclcpp::get_logger("ActuatorsRpPicoHardware"), "Exporting state interfaces");
  
  for (int i = 1; i <= 12; i++) {
    if (cfg_.servo_names[i] != "") {
      state_interfaces.emplace_back(hardware_interface::StateInterface(
        cfg_.servo_names[i], hardware_interface::HW_IF_POSITION, &servos_[i].angle));
    }
  }

  return state_interfaces;
}

std::vector<hardware_interface::CommandInterface> ActuatorsRpPicoHardware::export_command_interfaces()
{
  std::vector<hardware_interface::CommandInterface> command_interfaces;
  for (int i = 1; i <= 12; i++) {
    if (cfg_.servo_names[i] != "") {
      command_interfaces.emplace_back(hardware_interface::CommandInterface(
        cfg_.servo_names[i], hardware_interface::HW_IF_POSITION, &servos_[i].cmd));
    }
  }

  return command_interfaces;
}

hardware_interface::CallbackReturn ActuatorsRpPicoHardware::on_configure(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  RCLCPP_INFO(rclcpp::get_logger("ActuatorsRpPicoHardware"), "Configuring ...please wait...");
  if (comms_.connected())
  {
    comms_.disconnect();
  }
  comms_.connect(cfg_.device, cfg_.baud_rate, cfg_.timeout_ms);
  RCLCPP_INFO(rclcpp::get_logger("ActuatorsRpPicoHardware"), "Successfully configured!");

  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn ActuatorsRpPicoHardware::on_cleanup(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  RCLCPP_INFO(rclcpp::get_logger("ActuatorsRpPicoHardware"), "Cleaning up ...please wait...");
  if (comms_.connected())
  {
    comms_.disconnect();
  }
  RCLCPP_INFO(rclcpp::get_logger("ActuatorsRpPicoHardware"), "Successfully cleaned up!");

  return hardware_interface::CallbackReturn::SUCCESS;
}


hardware_interface::CallbackReturn ActuatorsRpPicoHardware::on_activate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  RCLCPP_INFO(rclcpp::get_logger("ActuatorsRpPicoHardware"), "Activating ...please wait...");
  if (!comms_.connected())
  {
    return hardware_interface::CallbackReturn::ERROR;
  }


  RCLCPP_INFO(rclcpp::get_logger("ActuatorsRpPicoHardware"), "Successfully activated!");

  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn ActuatorsRpPicoHardware::on_deactivate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  RCLCPP_INFO(rclcpp::get_logger("ActuatorsRpPicoHardware"), "Deactivating ...please wait...");
  RCLCPP_INFO(rclcpp::get_logger("ActuatorsRpPicoHardware"), "Successfully deactivated!");

  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::return_type ActuatorsRpPicoHardware::read(
  const rclcpp::Time & /*time*/, const rclcpp::Duration & period)
{
  if (!comms_.connected())
  {
    return hardware_interface::return_type::ERROR;
  }
  
  return hardware_interface::return_type::OK;
}

hardware_interface::return_type actuators_rppico_h_i ::ActuatorsRpPicoHardware::write(
  const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/)
{
  if (!comms_.connected())
  {
    return hardware_interface::return_type::ERROR;
  }

  for (int i = 1; i <= 12; i++) {
    if (cfg_.servo_names[i] != "") {
      comms_.set_servo_angle(i, servos_[i].cmd);
    }
  }

  return hardware_interface::return_type::OK;
}


}  // namespace actuators_rppico_h_i
#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(
  actuators_rppico_h_i::ActuatorsRpPicoHardware, hardware_interface::SystemInterface)
