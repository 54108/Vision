#include"Serial.hpp"

using namespace std;
using namespace cv;

auto idntifier_green = fmt::format(fg(fmt::color::green) | fmt::emphasis::bold, "uart_serial");
auto idntifier_red   = fmt::format(fg(fmt::color::red)   | fmt::emphasis::bold, "uart_serial");

Serial::Serial(std::string _serial_config)
{
    cv::FileStorage fs_serial(_serial_config, cv::FileStorage::READ);
    
    fs_serial["PREFERRED_DEVICE"]        >> serial_config_.preferred_device;
    fs_serial["SET_BAUDRATE"]            >> serial_config_.set_baudrate;
    fs_serial["SHOW_SERIAL_INFORMATION"] >> serial_config_.show_serial_information;

    serialInit();
}

Serial::~Serial()
{
    if (!close(fd)) 
    { 
        fmt::print("[{}] Close serial device success: {}\n", idntifier_green, fd); 
        std::cout<<"Close serial device success"<<endl;
    }
}

void Serial::serialInit()
{
    const char* DeviceName[] = {serial_config_.preferred_device.c_str(), "/dev/ttyUSB0", "/dev/ttyUSB2", "/dev/ttyUSB3"};

    struct termios newstate;
    bzero(&newstate, sizeof(newstate));

    for (size_t i = 0; i != sizeof(DeviceName) / sizeof(char*); ++i) 
    {
        fd = open(DeviceName[i], O_RDWR | O_NONBLOCK | O_NOCTTY | O_NDELAY);
        if (fd == -1) 
        {
            cout<<"Open serial device failed"<<endl;
            //fmt::print("[{}] Open serial device failed: {}\n", idntifier_red, DeviceName[i]);
        } 
        else 
        {
            cout<<"Open serial device success"<<endl;
            fmt::print("[{}] Open serial device success: {}\n", idntifier_green, DeviceName[i]);

            break;
        }
    }
    switch (serial_config_.set_baudrate)
    {
        case 1:
            cfsetospeed(&newstate, B115200);
            cfsetispeed(&newstate, B115200);
        break;
        case 10:
            cfsetospeed(&newstate, B921600);
            cfsetispeed(&newstate, B921600);
        break;
        default:
            cfsetospeed(&newstate, B115200);
            cfsetispeed(&newstate, B115200);
        break;
    }

  newstate.c_cflag |= CLOCAL | CREAD;
  newstate.c_cflag &= ~CSIZE;
  newstate.c_cflag &= ~CSTOPB;
  newstate.c_cflag |= CS8;
  newstate.c_cflag &= ~PARENB;

  newstate.c_cc[VTIME] = 0;
  newstate.c_cc[VMIN]  = 0;

  tcflush(fd, TCIOFLUSH);
  tcsetattr(fd, TCSANOW, &newstate);
}


void Serial::getSendData(const int _isFindTarget,
                     const float _yaw,
                     const float _pitch,
                     const int _distance)
{

    send_data.is_find_target    = _isFindTarget > 1 ? 1 : _isFindTarget;
    send_data.yaw_symbol   = _yaw >= 0 ? 1 : 0;
    send_data.yaw_angle    = fabs(_yaw) * 100;

    cout<<"       "<<send_data.yaw_angle<<endl;

    send_data.pitch_symbol = _pitch >= 0 ? 1 : 0;
    send_data.pitch_angle  = fabs(_pitch) * 100;
    cout<<"       "<<send_data.pitch_angle<<endl;
    send_data.distance        = _distance;
}

void Serial::setSendBuffer(const uint8_t& CRC)
{
    write_buff_[0]  = 0x53;
    write_buff_[1]  = static_cast<unsigned char>(send_data.is_find_target);
    write_buff_[2] =0;
    write_buff_[3]  = static_cast<unsigned char>(send_data.yaw_symbol);
    write_buff_[4]  = returnLowBit(send_data.yaw_angle);
    write_buff_[5]  = returnHighBit(send_data.yaw_angle);
    write_buff_[6]  = static_cast<unsigned char>(send_data.pitch_symbol);
    write_buff_[7]  = returnLowBit(send_data.pitch_angle);
    write_buff_[8]  = returnHighBit(send_data.pitch_angle);
    write_buff_[9]  = returnLowBit(send_data.distance);
    write_buff_[10] = returnHighBit(send_data.distance);
    write_buff_[11] = CRC & 0xff;
    write_buff_[12] = 0x45; 
}

void Serial::setCrcBuffer()
{
    crc_buff_[0]  = 0x53;
    crc_buff_[1]  = static_cast<unsigned char>(send_data.is_find_target);
    crc_buff_[2]   =0;
    crc_buff_[3]  = static_cast<unsigned char>(send_data.yaw_symbol);
    crc_buff_[4]  = returnLowBit(send_data.yaw_angle);
    crc_buff_[5]  = returnHighBit(send_data.yaw_angle);
    crc_buff_[6]  = static_cast<unsigned char>(send_data.pitch_symbol);
    crc_buff_[7]  = returnLowBit(send_data.pitch_angle);
    crc_buff_[8]  = returnHighBit(send_data.pitch_angle);
    crc_buff_[9]  = returnLowBit(send_data.distance);
    crc_buff_[10] = returnHighBit(send_data.distance);
  
}

uint8_t Serial::checksumCRC(unsigned char* buf, uint16_t len) 
{
  uint8_t check = 0;

  while (len--) { check = CRC8_Table[check ^ (*buf++)]; }

  return check;
}

void Serial::sendData(const int  isFindTarget,
                     const float yaw,
                     const float  pitch,
                     const int distance)
{
    getSendData(isFindTarget,
                     yaw,
                     pitch,
                     distance);
    setCrcBuffer();
    uint8_t CRC = checksumCRC(crc_buff_, sizeof(crc_buff_));
    setSendBuffer(CRC);

    write_message_ = write(fd, write_buff_, sizeof(write_buff_));
      
    yaw_reduction_   = mergeIntoBytes(write_buff_[5], write_buff_[4]);
    
      pitch_reduction_ = mergeIntoBytes(write_buff_[8], write_buff_[7]);
      depth_reduction_ = mergeIntoBytes(write_buff_[10], write_buff_[9]);

    fmt::print("[{}] writeData() ->", idntifier_green);
    for (size_t i = 0; i != 13; ++i) {
      fmt::print(" {}", write_buff_[i]);
    }
    fmt::print("\n");


      fmt::print("[{}] writeData() ->", idntifier_green);
      for (size_t i = 0; i != 4; ++i) 
      {
        fmt::print(" {}", write_buff_[i]);
      }
      fmt::print(" {} {} {} {}", static_cast<float>(yaw_reduction_) / 100, static_cast<int>(write_buff_[6]), static_cast<float>(pitch_reduction_) / 100, static_cast<float>(depth_reduction_));
      for (size_t i = 10; i != 13; ++i) 
      {
        fmt::print(" {}", write_buff_[i]);
      }
      fmt::print("\n");

      yaw_reduction_   = 0x0000;
      pitch_reduction_ = 0x0000;
      depth_reduction_ = 0x0000;
}

