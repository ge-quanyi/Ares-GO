//
// Created by Quanyi on 2022/12/3.
//

#pragma once

#include "data_type.h"
#include "Message.hpp"

#define COMMAND_BUFF_LEN 50
#define COM_BUFF_LEN 25
#define HEAD_LEN 4
#define DATA_LEN 25


class WT{

public:
    explicit WT(const char* id, const int speed);
    char buff_w_[COM_BUFF_LEN]; // 发送的数据
    char buff_r_[COM_BUFF_LEN]; // 读取的数据 校验之后的数据
    char buff_l_[COM_BUFF_LEN]; // 读取数据缓存
    //char rrr[3];
    bool PortInit();//串口初始化
    
    bool SendBuff(char command, char *data, unsigned short length);//发送数据
    int ReceiveBuff();//接受数据
    void receive_thread();

private:

    int fd_;//串口文件
    const char* devices;
    int baudrate;
    int OpenDev(const char *dev);
    bool SetSpeed(int fd, int speed);
    bool SetParity(int &fd, int data_bits, char parity, int stop_bits);
    bool imudataSumCheck(char* buffer);

    int Read(char *buff, size_t length);//串口读取
    bool Write(char *buff, size_t length);//写串口数据

};

