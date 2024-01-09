#include "Serial.hpp"

using namespace std;
using namespace cv;

auto idntifier_green = fmt::format(fg(fmt::color::green) | fmt::emphasis::bold, "uart_serial");
auto idntifier_red = fmt::format(fg(fmt::color::red) | fmt::emphasis::bold, "uart_serial");

Serial::Serial(std::string _serial_config)
{
    cv::FileStorage fs_serial(_serial_config, cv::FileStorage::READ);

    fs_serial["PREFERRED_DEVICE"] >> serial_config.preferred_device;
    fs_serial["SET_BAUDRATE"] >> serial_config.set_baudrate;
    fs_serial["SHOW_SERIAL_INFORMATION"] >> serial_config.show_serial_information;

    serialInit();
}

Serial::~Serial()
{
    if (!close(fd))
    {
        fmt::print("[{}] Close serial device success: {}\n", idntifier_green, fd);
        std::cout << "Close serial device success" << endl;
    }
}

void Serial::serialInit()
{
    const char *DeviceName[] = {serial_config.preferred_device.c_str(), "/dev/ttyUSB0", "/dev/ttyUSB2", "/dev/ttyUSB3"};

    struct termios newstate;
    bzero(&newstate, sizeof(newstate));

    for (size_t i = 0; i != sizeof(DeviceName) / sizeof(char *); ++i)
    {
        fd = open(DeviceName[i], O_RDWR | O_NONBLOCK | O_NOCTTY | O_NDELAY);
        if (fd == -1)
        {
            cout << "Open serial device failed" << endl;
            // fmt::print("[{}] Open serial device failed: {}\n", idntifier_red, DeviceName[i]);
        }
        else
        {
            cout << "Open serial device success" << endl;
            fmt::print("[{}] Open serial device success: {}\n", idntifier_green, DeviceName[i]);

            break;
        }
    }
    switch (serial_config.set_baudrate)
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
    newstate.c_cc[VMIN] = 0;

    tcflush(fd, TCIOFLUSH);
    tcsetattr(fd, TCSANOW, &newstate);
}

void Serial::getSendData(const int _isFindTarget, const float _yaw, const float _pitch, const int _distance)
{

    send_data.tracking = _isFindTarget > 1 ? 1 : _isFindTarget;
    send_data.yaw = _yaw >= 0 ? 1 : 0;
    send_data.yaw_angle = static_cast<int>(fabs(_yaw) * 100);

    // cout<<"       "<<send_data.yaw_angle<<endl;

    send_data.pitch = _pitch >= 0 ? 1 : 0;
    send_data.pitch_angle = static_cast<float>(fabs(_pitch) * 100);
    // cout<<"       "<<send_data.pitch_angle<<endl;
    send_data.distance = _distance;
}

void Serial::setSendBuffer(const uint8_t &CRC)
{
    write_buff[0] = 0x53;
    write_buff[1] = 1;
    write_buff[2] = static_cast<unsigned char>(send_data.tracking);
    write_buff[3] = static_cast<unsigned char>(send_data.yaw);
    write_buff[4] = returnLowBit(send_data.yaw_angle);
    write_buff[5] = returnHighBit(send_data.yaw_angle);
    write_buff[6] = static_cast<unsigned char>(send_data.pitch);
    write_buff[7] = returnLowBit(send_data.pitch_angle);
    write_buff[8] = returnHighBit(send_data.pitch_angle);
    write_buff[9] = returnLowBit(send_data.distance);
    write_buff[10] = returnHighBit(send_data.distance);
    write_buff[11] = CRC & 0xff;
    write_buff[12] = 0x45;
}

void Serial::setCrcBuffer()
{
    crc_buff[0] = 0x53;
    crc_buff[1] = static_cast<unsigned char>(send_data.tracking);
    crc_buff[2] = 0;
    crc_buff[3] = static_cast<unsigned char>(send_data.yaw);
    crc_buff[4] = returnLowBit(send_data.yaw_angle);
    crc_buff[5] = returnHighBit(send_data.yaw_angle);
    crc_buff[6] = static_cast<unsigned char>(send_data.pitch);
    crc_buff[7] = returnLowBit(send_data.pitch_angle);
    crc_buff[8] = returnHighBit(send_data.pitch_angle);
    crc_buff[9] = returnLowBit(send_data.distance);
    crc_buff[10] = returnHighBit(send_data.distance);
}

uint8_t Serial::checksumCRC(unsigned char *buf, uint16_t len)
{
    uint8_t check = 0;

    while (len--)
    {
        check = CRC8_Table[check ^ (*buf++)];
    }

    return check;
}

void Serial::sendData(const int isFindTarget, const float yaw, const float pitch, const int distance)
{
    getSendData(isFindTarget, yaw, pitch, distance);
    setCrcBuffer();
    uint8_t CRC = checksumCRC(crc_buff, sizeof(crc_buff));
    setSendBuffer(CRC);

    write_message_ = write(fd, write_buff, sizeof(write_buff));

    yaw_reduction = mergeIntoBytes(write_buff[5], write_buff[4]);

    pitch_reduction = mergeIntoBytes(write_buff[8], write_buff[7]);

    depth_reduction = mergeIntoBytes(write_buff[10], write_buff[9]);

    fmt::print("[{}] writeData() ->", idntifier_green);
    for (size_t i = 0; i != 13; ++i)
    {
        fmt::print(" {}", write_buff[i]);
    }
    fmt::print("\n");

    fmt::print("[{}] writeData() ->", idntifier_green);
    for (size_t i = 0; i != 4; ++i)
    {
        fmt::print(" {}", write_buff[i]);
    }
    fmt::print(" {} {} {} {}", static_cast<float>(yaw_reduction) / 100, static_cast<int>(write_buff[6]),
               static_cast<float>(pitch_reduction) / 100, static_cast<float>(depth_reduction));
    for (size_t i = 10; i != 13; ++i)
    {
        fmt::print(" {}", write_buff[i]);
    }
    fmt::print("\n");

    yaw_reduction = 0x0000;
    pitch_reduction = 0x0000;
    depth_reduction = 0x0000;
}

void Serial::receiveData()
{
    memset(receive_buff, '0', RECEIVE_BUFF_LENGTH * 2);

    read_message_ = read(fd, receive_buff_temp, sizeof(receive_buff_temp));

    for (size_t i = 0; i != sizeof(receive_buff_temp); ++i)
    {
        if (receive_buff_temp[i] == 'S' && receive_buff_temp[i + sizeof(receive_buff) - 1] == 'E')
        {
            if (serial_config.show_serial_information == 1)
            {
                fmt::print("[{}] receiveData() ->", idntifier_green);

                for (size_t j = 0; j != sizeof(receive_buff); ++j)
                {
                    receive_buff[j] = receive_buff_temp[i + j];

                    fmt::print(" {:d}", receive_buff[j]);
                }

                fmt::print("\n");
            }
            else
            {
                for (size_t j = 0; j != sizeof(receive_buff); ++j)
                {
                    receive_buff[j] = receive_buff_temp[i + j];
                }
            }

            break;
        }
    }

    tcflush(fd, TCIFLUSH);
}

bool Serial::isEmpty()
{
    if (receive_buff[0] != '0' || receive_buff[RECEIVE_BUFF_LENGTH - 1] != '0')
    {
        return false;
    }
    else
    {
        return true;
    }
}

void Serial::getReceiveData(ReceiveData &receive_data)
{
    receiveData();

    if (isEmpty())
    {
        return;
    }

    else
    {
        last_receive_data = receive_data;
    }

    switch (receive_buff[1])
    {
    case 0:
        receive_data.my_color = 0;
        break;
    case 1:
        receive_data.my_color = 1;
        break;
    default:
        receive_data.my_color = 2;
        break;
    }

    switch (receive_buff[2])
    {
    case 0:
        receive_data.detect_mode = 0;
        break;
    case 1:
        receive_data.detect_mode = 1;
        break;
    default:
        receive_data.detect_mode = 0;
        break;
    }

    receive_data.bullet_speed = receive_buff[3];

    // q0=mergeIntoBytes(receive_buff[5],receive_buff[4]);

    // q1=mergeIntoBytes(receive_buff[7],receive_buff[6]);

    // q2=mergeIntoBytes(receive_buff[9],receive_buff[8]);

    // q3=mergeIntoBytes(receive_buff[11],receive_buff[10]);

    receive_data.q = Eigen::Quaternionf(static_cast<float>(mergeIntoBytes(receive_buff[5], receive_buff[4])) / 1000,
                                        static_cast<float>(mergeIntoBytes(receive_buff[7], receive_buff[6])) / 1000,
                                        static_cast<float>(mergeIntoBytes(receive_buff[9], receive_buff[8])) / 1000,
                                        static_cast<float>(mergeIntoBytes(receive_buff[11], receive_buff[10])) / 1000);
}
