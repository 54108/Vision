#ifndef SERIAL_HPP
#define SERIAL_HPP
#include <fcntl.h>
#include <string>
#include <termios.h>
#include <unistd.h>

#include <Eigen/Core>
#include <Eigen/Dense>

#include <fmt/color.h>
#include <fmt/core.h>

#include <opencv2/opencv.hpp>

// buff长度
enum BufferLength
{
    // 发送信息数据长度
    //  The send length of the array after append CRC auth code
    SEND_BUFF_LENGTH = 13,

    // 接收信息数据长度
    //  The recieve length of the array obtained after decoding
    RECEIVE_BUFF_LENGTH = 22,

    // CRC校验数据长度
    //  The send length of the array for CRC auth code code calculating
    CRC_BUFF_LENGTH = 11,
};

// 串口参数
struct SerialConfig
{
    std::string preferred_device = "/dev/ttyUSB0";
    int set_baudrate = 0;
    int show_serial_information = 0;
};

struct ReceivePacket
{
    uint8_t header = 0x5A;
    uint8_t detect_color : 1; // 0-red 1-blue
    bool reset_tracker : 1;
    uint8_t reserved : 6;
    float current_v; // m/s
    float pitch;
    float yaw;
    float aim_x;
    float aim_y;
    float aim_z;
    uint16_t checksum = 0;
} __attribute__((packed));

struct SendPacket
{
    uint8_t header = 0xA5;
    bool tracking : 1;
    uint8_t reserved : 7;
    float x;
    float y;
    float z;
    float cx;
    float cy;
    float cz;
    float pitch;
    float yaw;
    float rowyaw;
    float r1;
    float r2;
    float dz;
    uint16_t checksum = 0;
} __attribute__((packed));

// 串口发送信息
// Serial port message sending structure
struct SendData
{
    int tracking;
    int yaw;
    int pitch;
    int distance;
    int yaw_angle;
    int pitch_angle;
    int velocity;
    SendData()
    {
        yaw = 0;
        pitch = 0;
        distance = 0;
        tracking = 0;
        yaw_angle = 0.f;
        pitch_angle = 0.f;
        velocity = 0;
    }
};

/* Receiving protocol:
 *  0:      'S'
 *  1:      color
 *  2:      mode
 *  3:      bullet_speed
 *  4~7:    q0 (union)
 *  8~11:   q1 (union)
 *  12~15:  q2 (union)
 *  16~19:  q3 (union)
 *  20:     'E'
 */
struct ReceiveData
{
    int my_color;
    int detect_mode;
    int bullet_speed;
    Eigen::Quaternionf q;
};

union Fp32 {
    uint32_t u;
    float f;
};

class Serial
{
  public:
    Serial() = default;
    explicit Serial(std::string _serial_config);

    ~Serial();

    void serialInit();

    void sendData(const int isFindTarget, const float yaw, const float pitch, const int distance);

    void getSendData(const int isFindTarget, const float yaw, const float pitch, const int distance);

    void setSendBuffer(const uint8_t &CRC);

    void setReceiveBuffer();

    void receiveData();

    void getReceiveData(ReceiveData &receive_data);

    void setCrcBuffer();

    bool isEmpty();

    inline uint8_t checksumCRC(unsigned char *buff, uint16_t length);

    /**
     * @brief 返回高八位数据
     *
     * @param Byte
     * @return unsigned char
     */
    inline unsigned char returnHighBit(const int &Byte)
    {
        exchangebyte_ = (Byte >> 8) & 0xff;

        return exchangebyte_;
    }

    /**
     * @brief 返回低八位数据
     *
     * @param Byte
     * @return unsigned char
     */
    inline unsigned char returnLowBit(const int &Byte)
    {
        exchangebyte_ = Byte & 0xff;

        return exchangebyte_;
    }

    /**
     * @brief 合并数据
     *
     * @param highbit   高八位数据
     * @param lowbit    低八位数据
     * @return int16_t  合并后数据
     */
    inline int16_t mergeIntoBytes(const unsigned char &highbit, const unsigned char &lowbit)
    {
        exchangebit_ = (highbit << 8) | lowbit;

        return exchangebit_;
    }

  private:
    int fd;
    int transform_arr[4];
    unsigned char exchangebyte_;

    int16_t yaw_reduction;
    int16_t pitch_reduction;
    int16_t depth_reduction;

    int16_t exchangebit_;

    int16_t q0;
    int16_t q1;
    int16_t q2;
    int16_t q3;

    SerialConfig serial_config;

    ssize_t read_message_;
    ssize_t write_message_;

    SendData send_data;
    ReceiveData receive_data;
    ReceiveData last_receive_data;

    unsigned char write_buff[SEND_BUFF_LENGTH];
    unsigned char receive_buff[RECEIVE_BUFF_LENGTH];
    unsigned char receive_buff_temp[RECEIVE_BUFF_LENGTH * 2];
    unsigned char crc_buff[CRC_BUFF_LENGTH];
};

static constexpr unsigned char CRC8_Table[] = {
    0,   94,  188, 226, 97,  63,  221, 131, 194, 156, 126, 32,  163, 253, 31,  65,  157, 195, 33,  127, 252, 162,
    64,  30,  95,  1,   227, 189, 62,  96,  130, 220, 35,  125, 159, 193, 66,  28,  254, 160, 225, 191, 93,  3,
    128, 222, 60,  98,  190, 224, 2,   92,  223, 129, 99,  61,  124, 34,  192, 158, 29,  67,  161, 255, 70,  24,
    250, 164, 39,  121, 155, 197, 132, 218, 56,  102, 229, 187, 89,  7,   219, 133, 103, 57,  186, 228, 6,   88,
    25,  71,  165, 251, 120, 38,  196, 154, 101, 59,  217, 135, 4,   90,  184, 230, 167, 249, 27,  69,  198, 152,
    122, 36,  248, 166, 68,  26,  153, 199, 37,  123, 58,  100, 134, 216, 91,  5,   231, 185, 140, 210, 48,  110,
    237, 179, 81,  15,  78,  16,  242, 172, 47,  113, 147, 205, 17,  79,  173, 243, 112, 46,  204, 146, 211, 141,
    111, 49,  178, 236, 14,  80,  175, 241, 19,  77,  206, 144, 114, 44,  109, 51,  209, 143, 12,  82,  176, 238,
    50,  108, 142, 208, 83,  13,  239, 177, 240, 174, 76,  18,  145, 207, 45,  115, 202, 148, 118, 40,  171, 245,
    23,  73,  8,   86,  180, 234, 105, 55,  213, 139, 87,  9,   235, 181, 54,  104, 138, 212, 149, 203, 41,  119,
    244, 170, 72,  22,  233, 183, 85,  11,  136, 214, 52,  106, 43,  117, 151, 201, 74,  20,  246, 168, 116, 42,
    200, 150, 21,  75,  169, 247, 182, 232, 10,  84,  215, 137, 107, 53};

#endif