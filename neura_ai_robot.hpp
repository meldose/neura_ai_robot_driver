#pragma once

#include <iostream>
#include <memory>
#include <string>
#include <vector>

// ─── Include the actual Neura SDK headers ─────────────────────────────────────
//    (Adjust paths if your SDK is installed elsewhere)
// #include <neura/robot.hpp>
// #include <neura/Utils.hpp>

namespace neura_ai_robot_driver
{
/**
 * @brief the following class is responsible for interfacing with the Neura robot hardware.
 */
class NeuraAIRobot
{
  public:
    /**
     * @brief Construct a new Neura Hardware Interface object
     *
     * @param robot_serial_number the robot serial number
     * @param dof degree of freedom of the robot (auto calculated from joints in urdf of the robot model)
     */
    NeuraAIRobot(int robot_serial_number, double dof);

    /**
     * @brief Destroy the Neura Hardware Interface object
     */
    ~NeuraAIRobot();

    /**
     * @brief initialize the robot communication channel to communicate using the serial number of the
     * robot (passed as the "robot_serial_number" parameter through launch file, or 0 if not specified)
     */
    void initRobotCommChannel();

    /**
     * @brief get the current joint positions from the robot and store them in the provided vector.
     *
     * @param joint_positions  vector to store the joint positions
     * @return true if successful
     * @return false if an error occurred
     */
    bool updateWithCurrentJointPositions(std::vector<double>& joint_positions);

    /**
     * @brief get the current joint velocities from the robot and store them in the provided vector.
     *
     * @param joint_velocities vector to store the joint velocities
     * @return true if successful
     * @return false if an error occurred
     */
    bool updateWithCurrentJointVelocities(std::vector<double>& joint_velocities);

    /**
     * @brief send a command to the robot to move to the provided joint positions and velocities.
     *
     * @param joint_positions_command position command in the joint space (in rad)
     * @param joint_velocities_command velocity command in the joint space (in rad/s)
     * @return true if successful sent to the robot
     * @return false if an error occurred
     */
    bool goToTargetCommand(const std::vector<double>& joint_positions_command,
                           const std::vector<double>& joint_velocities_command);

    /**
     * @brief Set the Robot Power status
     *
     * @param set_power if true to turn on the robot, false to turn off
     * @return true if successful set the power status
     * @return false otherwise
     */
    bool setRobotPower(const bool& set_power);

  private:
    std::shared_ptr<neura::Robot> m_robot_;      // pointer to the robot communication client
    double                        dof_;          // degree of freedom of the robot
    int                           serial_number_{0};  // serial number of the robot
    bool                          is_comm_channel_initialized_{false};  // flag indicating if the communication channel is initialized

    std::vector<double> client_pos_buf_;    // buffer for joint positions from communication client
    std::vector<double> client_vel_buf_;    // buffer for joint velocities from communication client
    std::vector<double> client_accel_buf_;  // buffer for joint accelerations from communication client
    double              client_time_buf_;   // buffer for time from communication client
};

}  // namespace neura_ai_robot_driver
