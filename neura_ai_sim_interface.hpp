#pragma once

#include <memory>
#include <string>
#include <vector>
#include <unordered_map>  // ← needed for std::unordered_map

#include "hardware_interface/handle.hpp"
#include "hardware_interface/hardware_info.hpp"
#include "hardware_interface/system_interface.hpp"
#include "hardware_interface/types/hardware_interface_return_values.hpp"

#include "rclcpp/clock.hpp"
#include "rclcpp/logger.hpp"
#include "rclcpp/macros.hpp"
#include "rclcpp_lifecycle/node_interfaces/lifecycle_node_interface.hpp"
#include "rclcpp_lifecycle/state.hpp"

namespace neura_ai_robot_driver
{

class NeuraAISimInterface : public hardware_interface::SystemInterface
{
public:
  RCLCPP_SHARED_PTR_DEFINITIONS(NeuraAISimInterface);

  hardware_interface::CallbackReturn on_init(const hardware_interface::HardwareInfo& info) override;
  hardware_interface::CallbackReturn on_configure(const rclcpp_lifecycle::State& previous_state) override;
  std::vector<hardware_interface::StateInterface>   export_state_interfaces() override;
  std::vector<hardware_interface::CommandInterface> export_command_interfaces() override;
  hardware_interface::CallbackReturn on_activate(const rclcpp_lifecycle::State& previous_state) override;
  hardware_interface::CallbackReturn on_deactivate(const rclcpp_lifecycle::State& previous_state) override;
  hardware_interface::return_type read(const rclcpp::Time& time, const rclcpp::Duration& period) override;
  hardware_interface::return_type write(const rclcpp::Time& time, const rclcpp::Duration& period) override;

private:
  /// Get the logger of the SystemInterface.
  rclcpp::Logger get_logger() const
  {
    return *logger_;
  }

  /// Get the clock of the SystemInterface.
  rclcpp::Clock::SharedPtr get_clock() const
  {
    return clock_;
  }

  // Objects for logging
  std::shared_ptr<rclcpp::Logger> logger_;
  rclcpp::Clock::SharedPtr        clock_;

  /**
   * @brief these are the command interfaces with the hardware interface
   */
  std::vector<double> hw_commands_positions_;
  std::vector<double> hw_commands_velocities_;

  /**
   * @brief these are the state interfaces with the hardware interface
   */
  std::vector<double> hw_states_positions_;
  std::vector<double> hw_states_velocities_;

  /**
   * @brief maps each joint name to the list of interface types it supports
   */
  std::unordered_map<std::string, std::vector<std::string>> joint_interfaces_ =
  {
    {"position", {}},
    {"velocity", {}}
  };
};

}  // namespace neura_ai_robot_driver
