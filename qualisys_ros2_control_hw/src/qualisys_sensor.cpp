/* ----------------------------------------------------------------------------

 * Copyright 2026, ICube Laboratory, University of Strasbourg, France
 * License: Apache License 2.0
 * Author: Mahdi Chaari
 * Email: chaari.mahdi@outlook.com
 * -------------------------------------------------------------------------- */

/**
 * @file qualisys_sensor.cpp
 * @date January 06, 2026
 * @author Mahdi Chaari
 * @brief Hardware interface for Qualisys Motion Capture System source file.
 */

#include "qualisys_ros2_control_hw/qualisys_sensor.hpp"

namespace qualisys_ros2_control_hw
{

struct Quaternion
{
  float w{1.f}, x{0.f}, y{0.f}, z{0.f};
};

Quaternion matrixToQuaternion(float* matrix) {
    Quaternion quaternion;

    float trace = matrix[0] + matrix[4] + matrix[8];
    if (trace > 0) {
        float s = 0.5f / std::sqrt(trace + 1.0f);
        quaternion.w = 0.25f / s;
        quaternion.x = (matrix[5] - matrix[7]) * s;
        quaternion.y = (matrix[6] - matrix[2]) * s;
        quaternion.z = (matrix[1] - matrix[3]) * s;
    } else {
        if (matrix[0] > matrix[4] && matrix[0] > matrix[8]) {
            float s = 2.0f * std::sqrt(1.0f + matrix[0] - matrix[4] - matrix[8]);
            quaternion.w = (matrix[5] - matrix[7]) / s;
            quaternion.x = 0.25f * s;
            quaternion.y = (matrix[3] + matrix[1]) / s;
            quaternion.z = (matrix[6] + matrix[2]) / s;
        } else if (matrix[4] > matrix[8]) {
            float s = 2.0f * std::sqrt(1.0f + matrix[4] - matrix[0] - matrix[8]);
            quaternion.w = (matrix[6] - matrix[2]) / s;
            quaternion.x = (matrix[3] + matrix[1]) / s;
            quaternion.y = 0.25f * s;
            quaternion.z = (matrix[7] + matrix[5]) / s;
        } else {
            float s = 2.0f * std::sqrt(1.0f + matrix[8] - matrix[0] - matrix[4]);
            quaternion.w = (matrix[1] - matrix[3]) / s;
            quaternion.x = (matrix[6] + matrix[2]) / s;
            quaternion.y = (matrix[7] + matrix[5]) / s;
            quaternion.z = 0.25f * s;
        }
    }

    return quaternion;
}

hardware_interface::CallbackReturn
QualisysSensor::on_init(const hardware_interface::HardwareInfo & info)
{
    if (SensorInterface::on_init(info) != CallbackReturn::SUCCESS) {
        return CallbackReturn::ERROR;
    }
    logger_ = std::make_shared<rclcpp::Logger>(rclcpp::get_logger(
        "controller_manager.resource_manager.hardware_component.sensor.QualisysSensor"));

    // Required hardware params
    auto it_host = info_.hardware_parameters.find("host_name");
    auto it_port = info_.hardware_parameters.find("host_port");

    if (it_host == info_.hardware_parameters.end() || it_host->second.empty()) {
        RCLCPP_FATAL(get_logger(), "Missing hardware param: host_name");
        return CallbackReturn::ERROR;
    }
    if (it_port == info_.hardware_parameters.end()) {
        RCLCPP_FATAL(get_logger(), "Missing hardware param: host_port");
        return CallbackReturn::ERROR;
    }

    host_name_ = it_host->second;
    host_port_ = std::stoi(it_port->second);

    const size_t n_sensors = info_.sensors.size();
    if (n_sensors == 0) {
        RCLCPP_FATAL(get_logger(), "No sensors defined in URDF");
        return CallbackReturn::ERROR;
    }

    x_.assign(n_sensors, std::numeric_limits<double>::quiet_NaN());
    y_.assign(n_sensors, std::numeric_limits<double>::quiet_NaN());
    z_.assign(n_sensors, std::numeric_limits<double>::quiet_NaN());
    qx_.assign(n_sensors, std::numeric_limits<double>::quiet_NaN());
    qy_.assign(n_sensors, std::numeric_limits<double>::quiet_NaN());
    qz_.assign(n_sensors, std::numeric_limits<double>::quiet_NaN());
    qw_.assign(n_sensors, std::numeric_limits<double>::quiet_NaN());

    // Expect: <param name="label">qualisys_label</param> inside each <sensor>
    for (size_t i = 0; i < n_sensors; ++i) {
        const auto & sensor = info_.sensors[i];

        auto it = sensor.parameters.find("label");
        if (it == sensor.parameters.end() || it->second.empty()) {
        RCLCPP_FATAL(
            get_logger(),
            "Sensor '%s' is missing required param 'label'",
            sensor.name.c_str());
        return CallbackReturn::ERROR;
        }

        label_to_sensor_index_[it->second] = i;

        RCLCPP_INFO(
        get_logger(),
        "Mapped sensor '%s' → Qualisys rigid body '%s'",
        sensor.name.c_str(),
        it->second.c_str());
    }

    return CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn
QualisysSensor::on_activate(const rclcpp_lifecycle::State &)
{
    return connect_qualisys() ? CallbackReturn::SUCCESS : CallbackReturn::ERROR;
}

hardware_interface::CallbackReturn
QualisysSensor::on_configure(const rclcpp_lifecycle::State &)
{
  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn
QualisysSensor::on_deactivate(const rclcpp_lifecycle::State &)
{
    disconnect_qualisys();
    return CallbackReturn::SUCCESS;
}

std::vector<hardware_interface::StateInterface>
QualisysSensor::export_state_interfaces()
{
    std::vector<hardware_interface::StateInterface> interfaces;

    for (size_t i = 0; i < info_.sensors.size(); ++i) {
        const std::string & name = info_.sensors[i].name;

        interfaces.emplace_back(name, "position.x", &x_[i]);
        interfaces.emplace_back(name, "position.y", &y_[i]);
        interfaces.emplace_back(name, "position.z", &z_[i]);

        interfaces.emplace_back(name, "orientation.x", &qx_[i]);
        interfaces.emplace_back(name, "orientation.y", &qy_[i]);
        interfaces.emplace_back(name, "orientation.z", &qz_[i]);
        interfaces.emplace_back(name, "orientation.w", &qw_[i]);
    }

    return interfaces;
}

hardware_interface::return_type
QualisysSensor::read(const rclcpp::Time &, const rclcpp::Duration &)
{
    read_frame();
    return hardware_interface::return_type::OK;
}

// ---------------- Qualisys I/O ----------------

bool QualisysSensor::connect_qualisys()
{
    RCLCPP_INFO(
        get_logger(),
        "Connecting to Qualisys at %s:%d",
        host_name_.c_str(),
        host_port_);

    if (!protocol_.Connect(host_name_.c_str(), host_port_, 0, 1, 7)) {
        return false;
    }

    RCLCPP_INFO(get_logger(), "Connected to Qualisys");

    bool ok = false;
    protocol_.Read6DOFSettings(ok);
    return ok;
}

void QualisysSensor::disconnect_qualisys()
{
    protocol_.StreamFramesStop();
    protocol_.Disconnect();
}

void QualisysSensor::read_frame()
{
    CRTPacket * packet = protocol_.GetRTPacket();
    CRTPacket::EPacketType type;

    protocol_.GetCurrentFrame(CRTProtocol::cComponent6d);

    if (!protocol_.ReceiveRTPacket(type, true)) 
    {
        RCLCPP_WARN(get_logger(), "No data received from Qualisys");
        return;
    }
    if (type != CRTPacket::PacketData) 
    {
        RCLCPP_WARN(get_logger(), "Received non-data packet from Qualisys");
        return;
    }

    const unsigned int n_bodies = packet->Get6DOFBodyCount();
    RCLCPP_INFO(get_logger(), "Number of bodies: %u", n_bodies);

    for (unsigned int i = 0; i < n_bodies; ++i) {
        float x, y, z;
        float R[9];
        packet->Get6DOFBody(i, x, y, z, R);

        const char * label = protocol_.Get6DOFBodyName(i);
        if (!label) continue;

        auto it = label_to_sensor_index_.find(label);
        if (it == label_to_sensor_index_.end()) continue;

        const size_t idx = it->second;
        Quaternion quaternion = matrixToQuaternion(R);

        if ((!std::isnan(x)) && (!std::isnan(y)) && (!std::isnan(z)) && 
            (!std::isnan(quaternion.x)) && (!std::isnan(quaternion.y)) && (!std::isnan(quaternion.z))
           && (!std::isnan(quaternion.w))){
            
            x_[idx] = x / 1000.0;
            y_[idx] = y / 1000.0;
            z_[idx] = z / 1000.0;
            qw_[idx] = quaternion.w;
            qx_[idx] = quaternion.x;
            qy_[idx] = quaternion.y;
            qz_[idx] = quaternion.z;

            RCLCPP_INFO(
                get_logger(),
                "Sensor '%s': Pos(%.3f, %.3f, %.3f) Orient(%.3f, %.3f, %.3f, %.3f)",
                label,
                x_[idx],
                y_[idx],
                z_[idx],
                qw_[idx],
                qx_[idx],
                qy_[idx],
                qz_[idx]);
        }else{
            RCLCPP_WARN(
                get_logger(),
                "Sensor '%s': Received NaN values, skipping update.",
                label);
        }

  
    }
}

}  // namespace qualisys_ros2_control_hw

PLUGINLIB_EXPORT_CLASS(
  qualisys_ros2_control_hw::QualisysSensor,
  hardware_interface::SensorInterface)
