// Copyright (c) 2022 ChenJun
// Licensed under the Apache-2.0 License.

#include "rm_serial_driver.hpp"

// ROS
// #include <rclcpp/logging.hpp>
// #include <rclcpp/qos.hpp>
// #include <rclcpp/utilities.hpp>
// #include <serial_driver/serial_driver.hpp>

// C++ system
#include <cstdint>
#include <functional>
#include <map>
#include <memory>
#include <string>
#include <vector>

#include "rm_serial_driver/crc.hpp"
#include "rm_serial_driver/packet.hpp"

template <class T> auto declare_parameter(std::string s, T t) -> T
{
    std::cout << s << std::endl;
    return t;
}

namespace rm_serial_driver
{
RMSerialDriver::RMSerialDriver(const std::string &serial_port, const uint32_t &baud_rate)
{
    // 创建并打开 CSV 文件
    csv_file_ = std::ofstream("data.csv", std::ios::app);
    if (!csv_file_.is_open())
    {
        std::cout << "Failed to open CSV file for writing." << std::endl;
        exit(-1);
        // 处理打开文件失败的情况
    }

    std::cout << "Start RMSerialDriver!" << std::endl;

    getParams();

    // Create Publisher
    timestamp_offset_ = declare_parameter("timestamp_offset", 0.0);
    // joint_state_pub_ =this->create_publisher<sensor_msgs::msg::JointState>("/joint_states", rclcpp::QoS(rclcpp::KeepLast(1)));
    // latency_pub_ = this->create_publisher<std_msgs::msg::Float64>("/latency", 10);
    // marker_pub_ = this->create_publisher<visualization_msgs::msg::Marker>("/aiming_point", 10);
    // velocity_pub_ = this->create_publisher<auto_aim_interfaces::msg::Velocity>("/current_velocity", 10);

    // try
    // {
    //     serial_driver_->init_port(device_name_, *device_config_);
    //     if (!serial_driver_->port()->is_open())
    //     {
    //         serial_driver_->port()->open();
    //         receive_thread_ = std::thread(&RMSerialDriver::receiveData, this);
    //     }
    // }
    // catch (const std::exception &ex)
    // {
    //     RCLCPP_ERROR(get_logger(), "Error creating serial port: %s - %s", device_name_.c_str(), ex.what());
    //     throw ex;
    // }

    // Create Subscription
    // send_sub_ = this->create_subscription<auto_aim_interfaces::msg::Send>(
    //     "/tracker/send", rclcpp::SensorDataQoS(), std::bind(&RMSerialDriver::sendData, this, std::placeholders::_1));

    // target_sub_ = this->create_subscription<auto_aim_interfaces::msg::Target>(
    //   "/tracker/target", rclcpp::SensorDataQoS(),
    //   std::bind(&RMSerialDriver::sendData, this, std::placeholders::_1));

    
}

RMSerialDriver::~RMSerialDriver()
{

    // 关闭 CSV 文件
    if (csv_file_.is_open())
    {
        csv_file_.close();
    }
    if (receive_thread_.joinable())
    {
        receive_thread_.join();
    }

    if (serial_driver_->port()->is_open())
    {
        serial_driver_->port()->close();
    }

    if (owned_ctx_)
    {
        owned_ctx_->waitForExit();
    }
}

void RMSerialDriver::receiveData()
{
    std::vector<uint8_t> header(1);
    std::vector<uint8_t> data;
    data.reserve(sizeof(ReceivePacket));

    while (rclcpp::ok())
    {
        try
        {
            serial_driver_->port()->receive(header);

            if (header[0] == 0x5A)
            {
                data.resize(sizeof(ReceivePacket) - 1);
                serial_driver_->port()->receive(data);

                data.insert(data.begin(), header[0]);
                ReceivePacket packet = fromVector(data);

                bool crc_ok = crc16::Verify_CRC16_Check_Sum(reinterpret_cast<const uint8_t *>(&packet), sizeof(packet));

                if (crc_ok)
                {
                    if (!initial_set_param_ || packet.detect_color != previous_receive_color_)
                    {
                        setParam(rclcpp::Parameter("detect_color", packet.detect_color));
                        previous_receive_color_ = packet.detect_color;
                    }

                    if (packet.reset_tracker)
                    {
                        resetTracker();
                    }

                    // 打印 data 结构体中的 xyz 和 yaw 值
                    // std::cout << "xyz: (" << packet.aim_x << ", " << packet.aim_y << ", " << packet.aim_z << ")" <<
                    // std::endl; std::cout << "pitch: " << packet.pitch << "yaw: " << packet.yaw << std::endl;

                    sensor_msgs::msg::JointState joint_state;
                    timestamp_offset_ = this->get_parameter("timestamp_offset").as_double();
                    joint_state.header.stamp = this->now() + rclcpp::Duration::from_seconds(timestamp_offset_);
                    joint_state.name.push_back("pitch_joint");
                    joint_state.name.push_back("yaw_joint");
                    joint_state.position.push_back(packet.pitch);
                    joint_state.position.push_back(packet.yaw);
                    joint_state_pub_->publish(joint_state);

                    auto_aim_interfaces::msg::Velocity current_velocity;
                    timestamp_offset_ = this->get_parameter("timestamp_offset").as_double();
                    current_velocity.header.stamp = this->now() + rclcpp::Duration::from_seconds(timestamp_offset_);
                    current_velocity.velocity = packet.current_v;
                    velocity_pub_->publish(current_velocity);

                    if (abs(packet.aim_x) > 0.01)
                    {
                        aiming_point_.header.stamp = this->now();
                        aiming_point_.pose.position.x = packet.aim_x;
                        aiming_point_.pose.position.y = packet.aim_y;
                        aiming_point_.pose.position.z = packet.aim_z;
                        marker_pub_->publish(aiming_point_);
                    }
                }
                else
                {
                    RCLCPP_ERROR(get_logger(), "CRC error!");
                }
            }
            else
            {
                RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 20, "Invalid header: %02X", header[0]);
            }
        }
        catch (const std::exception &ex)
        {
            RCLCPP_ERROR_THROTTLE(get_logger(), *get_clock(), 20, "Error while receiving data: %s", ex.what());
            reopenPort();
        }
    }
}

void RMSerialDriver::sendData(const auto_aim_interfaces::msg::Send::SharedPtr msg)
{
    const static std::map<std::string, uint8_t> id_unit8_map{
        {"", 0}, {"outpost", 0}, {"1", 1}, {"1", 1}, {"2", 2}, {"3", 3}, {"4", 4}, {"5", 5}, {"guard", 6}, {"base", 7}};

    try
    {
        SendPacket packet;
        packet.tracking = msg->tracking;
        packet.x = msg->position.x;
        packet.y = msg->position.y;
        packet.z = msg->position.z;
        packet.pitch = msg->pitch;
        packet.yaw = msg->yaw;

        crc16::Append_CRC16_Check_Sum(reinterpret_cast<uint8_t *>(&packet), sizeof(packet));

        // 打印 data 结构体中的 xyz 和 yaw 值
        std::cout << "xyz: (" << packet.x << ", " << packet.y << ", " << packet.z << ")" << std::endl;
        std::cout << "pitch: " << packet.pitch << "yaw: " << packet.yaw << std::endl;

        std::vector<uint8_t> data = toVector(packet);

        //  // 将数据以十六进制格式写入 CSV 文件
        //   for (const auto& byte : data) {
        //     csv_file_ << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(byte) << ",";
        //   }
        //   csv_file_ << "\n"; // 写入换行符表示新的一行
        //   csv_file_.flush(); // 刷新文件缓冲区，确保数据被写入文件

        serial_driver_->port()->send(data);

        std_msgs::msg::Float64 latency;
        latency.data = (this->now() - msg->header.stamp).seconds() * 1000.0;
        RCLCPP_DEBUG_STREAM(get_logger(), "Total latency: " + std::to_string(latency.data) + "ms");
        latency_pub_->publish(latency);

        //     // 保存到CSV文件
        // std::ofstream file("/ros_ws/src/rm_serial_driver/src/data.csv", std::ios::app); // 打开CSV文件，使用追加模式
        // file << packet.x << "," << packet.y << "," << packet.z << "," << packet.yaw << "\n"; // 写入数据
        // file.close(); // 关闭文件
    }
    catch (const std::exception &ex)
    {
        RCLCPP_ERROR(get_logger(), "Error while sending data: %s", ex.what());
        reopenPort();
    }
}

void RMSerialDriver::getParams()
{
    using FlowControl = drivers::serial_driver::FlowControl;
    using Parity = drivers::serial_driver::Parity;
    using StopBits = drivers::serial_driver::StopBits;

    uint32_t baud_rate{};
    auto fc = FlowControl::NONE;
    auto pt = Parity::NONE;
    auto sb = StopBits::ONE;

    try
    {
        device_name_ = declare_parameter<std::string>("device_name", "");
    }
    catch (rclcpp::ParameterTypeException &ex)
    {
        RCLCPP_ERROR(get_logger(), "The device name provided was invalid");
        throw ex;
    }

    try
    {
        baud_rate = declare_parameter<int>("baud_rate", 0);
    }
    catch (rclcpp::ParameterTypeException &ex)
    {
        RCLCPP_ERROR(get_logger(), "The baud_rate provided was invalid");
        throw ex;
    }

    try
    {
        const auto fc_string = declare_parameter<std::string>("flow_control", "");

        if (fc_string == "none")
        {
            fc = FlowControl::NONE;
        }
        else if (fc_string == "hardware")
        {
            fc = FlowControl::HARDWARE;
        }
        else if (fc_string == "software")
        {
            fc = FlowControl::SOFTWARE;
        }
        else
        {
            throw std::invalid_argument{"The flow_control parameter must be one of: none, software, or hardware."};
        }
    }
    catch (rclcpp::ParameterTypeException &ex)
    {
        RCLCPP_ERROR(get_logger(), "The flow_control provided was invalid");
        throw ex;
    }

    try
    {
        const auto pt_string = declare_parameter<std::string>("parity", "");

        if (pt_string == "none")
        {
            pt = Parity::NONE;
        }
        else if (pt_string == "odd")
        {
            pt = Parity::ODD;
        }
        else if (pt_string == "even")
        {
            pt = Parity::EVEN;
        }
        else
        {
            throw std::invalid_argument{"The parity parameter must be one of: none, odd, or even."};
        }
    }
    catch (rclcpp::ParameterTypeException &ex)
    {
        RCLCPP_ERROR(get_logger(), "The parity provided was invalid");
        throw ex;
    }

    try
    {
        const auto sb_string = declare_parameter<std::string>("stop_bits", "");

        if (sb_string == "1" || sb_string == "1.0")
        {
            sb = StopBits::ONE;
        }
        else if (sb_string == "1.5")
        {
            sb = StopBits::ONE_POINT_FIVE;
        }
        else if (sb_string == "2" || sb_string == "2.0")
        {
            sb = StopBits::TWO;
        }
        else
        {
            throw std::invalid_argument{"The stop_bits parameter must be one of: 1, 1.5, or 2."};
        }
    }
    catch (rclcpp::ParameterTypeException &ex)
    {
        RCLCPP_ERROR(get_logger(), "The stop_bits provided was invalid");
        throw ex;
    }

    device_config_ = std::make_unique<drivers::serial_driver::SerialPortConfig>(baud_rate, fc, pt, sb);
}

void RMSerialDriver::reopenPort()
{
    RCLCPP_WARN(get_logger(), "Attempting to reopen port");
    try
    {
        if (serial_driver_->port()->is_open())
        {
            serial_driver_->port()->close();
        }
        serial_driver_->port()->open();
        RCLCPP_INFO(get_logger(), "Successfully reopened port");
    }
    catch (const std::exception &ex)
    {
        RCLCPP_ERROR(get_logger(), "Error while reopening port: %s", ex.what());
        if (rclcpp::ok())
        {
            rclcpp::sleep_for(std::chrono::seconds(1));
            reopenPort();
        }
    }
}

void RMSerialDriver::setParam(const rclcpp::Parameter &param)
{
    if (!detector_param_client_->service_is_ready())
    {
        RCLCPP_WARN(get_logger(), "Service not ready, skipping parameter set");
        return;
    }

    if (!set_param_future_.valid() || set_param_future_.wait_for(std::chrono::seconds(0)) == std::future_status::ready)
    {
        RCLCPP_INFO(get_logger(), "Setting detect_color to %ld...", param.as_int());
        set_param_future_ =
            detector_param_client_->set_parameters({param}, [this, param](const ResultFuturePtr &results) {
                for (const auto &result : results.get())
                {
                    if (!result.successful)
                    {
                        RCLCPP_ERROR(get_logger(), "Failed to set parameter: %s", result.reason.c_str());
                        return;
                    }
                }
                RCLCPP_INFO(get_logger(), "Successfully set detect_color to %ld!", param.as_int());
                initial_set_param_ = true;
            });
    }
}

void RMSerialDriver::resetTracker()
{
    if (!reset_tracker_client_->service_is_ready())
    {
        RCLCPP_WARN(get_logger(), "Service not ready, skipping tracker reset");
        return;
    }

    auto request = std::make_shared<std_srvs::srv::Trigger::Request>();
    reset_tracker_client_->async_send_request(request);
    RCLCPP_INFO(get_logger(), "Reset tracker!");
}

} // namespace rm_serial_driver

#include "rclcpp_components/register_node_macro.hpp"

// Register the component with class_loader.
// This acts as a sort of entry point, allowing the component to be discoverable when its library
// is being loaded into a running process.
RCLCPP_COMPONENTS_REGISTER_NODE(rm_serial_driver::RMSerialDriver)
