/* ----------------------------------------------------------------------------

 * Copyright 2026, ICube Laboratory, University of Strasbourg, France
 * License: Apache License 2.0
 * Author: Mahdi Chaari
 * Email: chaari.mahdi@outlook.com
 * -------------------------------------------------------------------------- */

/**
 * @file qualisys_sensor.hpp
 * @date January 06, 2026
 * @author Mahdi Chaari
 * @brief Hardware interface for Qualisys Motion Capture System header file.
 */

#ifndef QUALISYS_ROS2_CONTROL_HW__QUALISYS_SENSOR_HPP_
#define QUALISYS_ROS2_CONTROL_HW__QUALISYS_SENSOR_HPP_


#include <string>
#include <vector>

#include <cmath>
#include <limits>
#include <unordered_map>

#include <limits>

#include "hardware_interface/sensor_interface.hpp"
#include "hardware_interface/types/hardware_interface_return_values.hpp"
#include "hardware_interface/hardware_info.hpp"

#include "rclcpp/logger.hpp"
#include "rclcpp/logging.hpp"

#include "RTProtocol.h"

#include "pluginlib/class_list_macros.hpp"

namespace qualisys_ros2_control_hw
{

class QualisysSensor : public hardware_interface::SensorInterface
{
public:
    RCLCPP_SHARED_PTR_DEFINITIONS(QualisysSensor)

    hardware_interface::CallbackReturn on_init(
        const hardware_interface::HardwareInfo & info) override;

    hardware_interface::CallbackReturn on_configure(
        const rclcpp_lifecycle::State & prev) override;

    hardware_interface::CallbackReturn on_activate(
        const rclcpp_lifecycle::State & prev) override;

    hardware_interface::CallbackReturn on_deactivate(
        const rclcpp_lifecycle::State & prev) override;

    std::vector<hardware_interface::StateInterface> export_state_interfaces() override;

    hardware_interface::return_type read(
        const rclcpp::Time & time,
        const rclcpp::Duration & period) override;
    
    /// Get the logger of the SensorInterface.
    /**
     * \return logger of the SensorInterface.
     */
    rclcpp::Logger get_logger() const { return *logger_; }

private:

    // ---- Qualisys helpers ----
    bool connect_qualisys();
    void disconnect_qualisys();
    void read_frame();


    // ---- Config ---- //
    std::string host_name_;
    int host_port_{22222};

    // Communication protocol
    CRTProtocol protocol_;

    // Sensor mapping
    std::unordered_map<std::string, size_t> label_to_sensor_index_;

    // Per joint(slot): pose state
    std::vector<double> x_, y_, z_;
    std::vector<double> qx_, qy_, qz_, qw_;

    // Objects for logging
    std::shared_ptr<rclcpp::Logger> logger_;
};
}

#endif // QUALISYS_ROS2_CONTROL_HW__QUALISYS_SENSOR_HPP_