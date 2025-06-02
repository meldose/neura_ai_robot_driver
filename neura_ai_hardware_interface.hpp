#include "hardware_interface/handle.hpp"
#include "hardware_interface/hardware_info.hpp"
#include "hardware_interface/system_interface.hpp"
#include "hardware_interface/types/hardware_interface_return_values.hpp"

#include "neura_ai_robot_driver/neura_ai_robot.hpp"

#include "rcl_interfaces/msg/set_parameters_result.hpp"
#include "rclcpp/clock.hpp"
#include "rclcpp/logger.hpp"
#include "rclcpp/macros.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/node_interfaces/lifecycle_node_interface.hpp"
#include "rclcpp_lifecycle/state.hpp"

namespace neura_ai_robot_driver
{

class NeuraAIHardwareInterface : public hardware_interface::SystemInterface
{
public:
  RCLCPP_SHARED_PTR_DEFINITIONS(NeuraAIHardwareInterface);

  hardware_interface::CallbackReturn on_init(const hardware_interface::HardwareInfo& info) override;
  hardware_interface::CallbackReturn on_configure(const rclcpp_lifecycle::State& previous_state) override;
  std::vector<hardware_interface::StateInterface>   export_state_interfaces() override;
  std::vector<hardware_interface::CommandInterface> export_command_interfaces() override;neura_ai_hardware_interface.hpp
  hardware_interface::CallbackReturn on_activate(const rclcpp_lifecycle::State& previous_state) override;
  hardware_interface::CallbackReturn on_deactivate(const rclcpp_lifecycle::State& previous_state) override;
  hardware_interface::return_type read(const rclcpp::Time& time, const rclcpp::Duration& period) override;
  hardware_interface::return_type write(const rclcpp::Time& time, const rclcpp::Duration& period) override;

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

private:
  int robot_serial_number_{0};

  // Logger and clock for lifecycle and debugging
  std::shared_ptr<rclcpp::Logger> logger_;
  rclcpp::Clock::SharedPtr        clock_;

  /// The NeuraAIRobot wrapper (which itself holds a std::shared_ptr<neura::Robot>)
  std::shared_ptr<neura_ai_robot_driver::NeuraAIRobot> robot_;

  /// Buffers for sending commands
  std::vector<double> hw_commands_positions_, hw_commands_velocities_;

  /// Buffers for reading state
  std::vector<double> hw_states_positions_, hw_states_velocities_;

  std::thread background_thread_;
  std::mutex  parameter_mutex_;

  /// Number of joints, set in on_init(…) from info.joints.size()
  int dof_{0};
};

}  // namespace neura_ai_robot_driver
