// Copyright (c) 2022 ChenJun
// Licensed under the Apache-2.0 License.

#ifndef RM_SERIAL_DRIVER__RM_SERIAL_DRIVER_HPP_
#define RM_SERIAL_DRIVER__RM_SERIAL_DRIVER_HPP_

// C++ system
#include "Predictor/msg.hpp"
#include "crc.hpp"
#include "packet.hpp"
#include <fcntl.h>
#include <fmt/color.h>
#include <fmt/core.h>
#include <fstream>
#include <iomanip>
#include <opencv2/opencv.hpp>
#include <string>
#include <termios.h>

struct Serial_Config
{
    std::string preferred_device = "/dev/ttyUSB0";
    int set_baudrate = 0;
    int show_serial_information = 0;
};

namespace rm_serial_driver
{
class RMSerialDriver
{
  public:
    explicit RMSerialDriver(std::string _serial_config);

    ~RMSerialDriver();

  private:
    // 在 RMSerialDriver 类的头文件中添加成员变量
    std::ofstream csv_file_;

    void receiveData();

    void sendData(const msg::Send &msg);

    void resetTracker();

    void RMSerialInit();

    Serial_Config serial_config_;
    int fd;
};
} // namespace rm_serial_driver

#endif // RM_SERIAL_DRIVER__RM_SERIAL_DRIVER_HPP_
