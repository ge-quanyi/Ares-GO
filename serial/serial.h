//
// Created by Quanyi on 2022/12/3.
//

#ifndef ARESCV_SERIAL_H
#define ARESCV_SERIAL_H

#define COMMAND_BUFF_LEN 50
#define COM_BUFF_LEN 25
#define HEAD_LEN 4
#define DATA_LEN 16
#include <mutex>

struct RobotInfo{
    char color;
    double ptz_pitch;
    double ptz_yaw;
    double bullet_speed;
};
class SerialPort
{
public:
    char buff_w_[COM_BUFF_LEN]; // 发送的数据
    char buff_r_[COM_BUFF_LEN]; // 读取的数据 校验之后的数据
    char buff_l_[COM_BUFF_LEN]; // 读取数据缓存
    double receive[16];
    //char rrr[3];
    explicit SerialPort(const char* id, const int speed);//构造函数
    bool PortInit();//串口初始化
    int Read(char *buff, size_t length);//串口读取
    bool Write(char *buff, size_t length);//写串口数据
    bool SendBuff(char command, char *data, unsigned short length);//发送数据
    int ReceiveBuff();//接受数据
    void receive_thread();
    RobotInfo robotInfo_;
private:
    int fd_;//串口文件
    const char* devices;
    const int baudrate;
    int OpenDev(const char *dev);
    bool SetSpeed(int fd, int speed);
    bool SetParity(int &fd, int data_bits, char parity, int stop_bits);
    void ISO14443AAppendCRCA(void* buffer, unsigned short byte_count);
    unsigned char ISO14443ACheckCRCA(void* buffer, unsigned short byte_count);
    bool ISO14443ACheckLen(unsigned  char* buffer);
};

#endif //ARESCV_SERIAL_H
