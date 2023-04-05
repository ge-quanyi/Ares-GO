//
// Created by Quanyi on 2022/12/3.
//

#include <iostream>
#include <unistd.h>          // Unix 标准函数定义
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>           // 文件控制定义
#include <termios.h>         // PPSIX 终端控制定义
#include <string.h>
#include <thread>
#include <math.h>
#include "imu_receive.h"
#include <glog/logging.h>
using namespace std;
/*************************************************
Function:       SerialPort
Description:    构造函数
Input:
Output:
Return:
Others:         初始化变量 创建接收线程
*************************************************/
WT::WT(const char* id, const int speed)
{
    devices = id;
    baudrate = speed;
    for (int i = 0; i < COM_BUFF_LEN; i++)
    {
        buff_w_[i] = 0;
        buff_r_[i] = 0;
    }
    // serial_publisher = this->create_publisher<my_interfaces::msg::RobotStatus>("/serial/robotinfo",10);
    this->PortInit();
    // data_sub_ = this->create_subscription<my_interfaces::msg::SendData>("/predictor/send_data", 10,\
    //  std::bind(&SerialPort::data_send, this, std::placeholders::_1));
}
/*************************************************
Function:       OpenDev
Description:    打开串口
Input:          device
Output:
Return:         fd or -1
Others:         LINUX中 open 函数作用：打开和创建文件
                O_RDONLY 只读打开  O_WRONLY 只写打开  O_RDWR 读，写打开
                对于串口的打开操作，必须使用O_NOCTTY参数，它表示打开的是一个终端设备，程序不会成为该端口的控制终端。如果不使用此标志，任务的一个输入(比如键盘终止信号等)都会影响进程
                O_NDELAY表示不关心DCD信号所处的状态（端口的另一端是否激活或者停止）
                O_NONBLOCK 设置为非阻塞模式，在read时不会阻塞住，在读的时候将read放在while循环中
*************************************************/
int WT::OpenDev(const char *dev)
{
    int fd = open(dev, O_RDWR | O_NOCTTY | O_NDELAY);//|O_NONBLOCK);
    if (fd == -1)
        cout << "Open imu error!!!" << endl;
    else
        tcflush(fd, TCIOFLUSH);   // 清空输入输出缓存

    return (fd);
}

/*************************************************
Function:       SetSpeed
Description:    设置波特率
Input:          fd speed
Output:
Return:         true or false
Others:
*************************************************/
unsigned int speed_arr[] = {B115200, B57600, B38400, B19200, B9600, B4800, B2400, B1200, B300};
unsigned int name_arr[] = {115200, 57600, 38400, 19200, 9600, 4800, 2400, 1200, 300};
bool WT::SetSpeed(int fd, int speed)
{
    int status;
    struct termios options;
    tcgetattr(fd, &options);
    for (int i= 0; i < sizeof(speed_arr) / sizeof(int); i++)
    {
        if (speed == name_arr[i])
        {
            tcflush(fd, TCIOFLUSH);
            cfsetispeed(&options, speed_arr[i]);//输入波特率
            cfsetospeed(&options, speed_arr[i]);//输出波特率
            options.c_cflag |= (CLOCAL | CREAD);//本地控制模式 //保证程序不会成为端的占有者//使端口能读取输入的数据
            status = tcsetattr(fd, TCSANOW, &options);//设置串口文件
            if (status != 0)
            {
                cout << "波特率设置失败" << endl;
                return false;
            }
        }
    }
    return true;
}
/*************************************************
Function:       SetParity
Description:    设置串口数据位，效验和停止位
Input:          fd         类型 int  打开的串口文件句柄
                data_bits  类型 int  数据位 取值 为 7 或者 8
                parity     类型 char 效验类型 取值为 N, E, O, S
                stop_bits  类型 int  停止位 取值为 1 或者 2
Output:
Return:
Others:
*************************************************/
bool WT::SetParity(int &fd, int data_bits, char parity, int stop_bits)
{
    struct termios options;
    /*保存测试现有串口参数设置，在这里如果串口号等出错，会有相关错误信息*/
    if (tcgetattr(fd, &options) != 0)
    {
        cout << "串口设置失败" << endl;
        return false;
    }
    /*options.c_cflag &= ~ CSIZE;
    switch (data_bits)
    {
        case 7:
            options.c_cflag |= CS7;
            break;
        case 8:
            options.c_cflag |= CS8;
            break;
        default:
            cout << "不支持的数据位" << endl;
            return false;
    }
    switch (parity)
    {
        case 'n':
        case 'N':
            options.c_cflag &= ~PARENB; // Clear parity enable
            options.c_iflag &= ~INPCK; // Enable parity checking
            break;
        case 'o':
        case 'O':
            options.c_cflag |= (PARODD | PARENB); // 设置为奇效验
            options.c_iflag |= INPCK; // Disable parity checking
            break;
        case 'e':
        case 'E':
            options.c_cflag |= PARENB; // Enable parity
            options.c_cflag &= ~PARODD; // 转换为偶效验
            options.c_iflag |= INPCK; // Disable parity checking
            break;
        case 's':
        case 'S': // as no parity
            options.c_cflag &= ~PARENB;
            options.c_cflag &= ~CSTOPB;
            break;
        default:
            cout << "不支持的校验类型" << endl;
            return false;
    }
    switch (stop_bits)
    {
        case 1:
            options.c_cflag &= ~CSTOPB;
            break;
        case 2:
            options.c_cflag |= CSTOPB;
            break;
        default:
            cout << "不支持的停止位" << endl;
            return false;
    }
    options.c_lflag &= ~(ICANON | ECHO | ECHOE | ISIG); // 设置为原始模式
    options.c_oflag &= ~ OPOST;*/
/*  直接设置
    options.c_iflag = 0;
    options.c_oflag = 0;
    options.c_cflag = 2237; // 6322-115200 2237-9600
    options.c_lflag = 0;
    options.c_cc[VTIME] = 0; // 等待时间
    options.c_cc[VMIN] = 0; // 最小接收字符
*/  //fcntl(fd,F_SETFL,FNDELAY);
    //直接设置
    options.c_lflag &= ~(ICANON | ECHO | ECHOE | ISIG);
    options.c_cflag &= ~PARENB;
    options.c_cflag &= ~CSTOPB;
    options.c_cflag &= ~CSIZE;
    options.c_cflag |= CS8;
    if (tcsetattr(fd, TCSANOW, &options) != 0) // Update the options and do it NOW
    {
        cout << "串口错误" << endl;
        return false;
    }

    cout << options.c_iflag << endl;
    cout << options.c_oflag << endl;
    cout << options.c_cflag << endl;
    cout << options.c_lflag << endl;
    cout << options.c_ispeed << endl;
    cout << options.c_ospeed << endl;
    return true;
}
/*************************************************
Function:       PortInit
Description:    初始化串口
Input:          串口号 波特率
Output:
Return:         true or false
Others:         bool
*************************************************/
bool WT::PortInit()
{
    fd_ = OpenDev(devices);
    if (fd_ == -1){
        return false;
    }

    if (!SetSpeed(fd_, baudrate)){
        return false;
    }

    // 数据位 8  校验 无  停止位 1
    if (!SetParity(fd_, 8, 'N', 1)){
        return false;
    }

    return true;
}
/*************************************************
Function:       Read
Description:    串口读数据
Input:          buff length
Output:
Return:         true or false
Others:         bool；read()默认为阻塞模式，没有读到数据会阻塞住；若在前面设置为非阻塞模式，没有读到数据会返回-1
*************************************************/
int WT::Read(char *r_buff, size_t length)
{
    size_t byte_read = 0;
    byte_read = read(fd_, r_buff, length);

    // if(byte_read == 0){ PortInit(0, 115200);}
    tcflush(fd_, TCIFLUSH); // 清空输入队列  TCIFLUSH 输入队列  TCOFLUSH 输出队列  TCIOFLUSH 输入输出队列
    return byte_read;
}
/*************************************************
Function:       Write
Description:    串口写数据
Input:          buff length
Output:
Return:         true or false
Others:         bool
*************************************************/
bool WT::Write(char *w_buff, size_t length)
{
    //cout << length << endl;
    write(fd_, w_buff, length);
    //tcflush(fd_, TCOFLUSH); // 清空输出队列  TCIFLUSH 输入队列  TCOFLUSH 输出队列  TCIOFLUSH 输入输出队列
    return true;
}

/*************************************************
Function:       ISO14443AAppendCRCA
Description:    CRC16 数据长度检验
Input:          buff
Output:
Return:
Others:
*************************************************/

bool WT::imudataSumCheck(char* buffer)
{
    short sum = 0;
    for(int i = 0; i < 10; i++){
        sum += *(buffer+i);
    }
    char tmp = sum & 0xff;
    if(tmp == *(buffer+10))
        return true;
    else 
        return false;
}
/*************************************************
Function:       SendBuff
Description:    发送数据包
Input:          buff length
Output:
Return:         true or false
Others:         bool
*************************************************/
bool WT::SendBuff(char command, char *data, unsigned short length)
{
    unsigned char buffer[COMMAND_BUFF_LEN]; //50
    buffer[0] = 0x55; // 帧头
    buffer[1] = command; // 命令
    buffer[2] = length; // 数据长度
    buffer[3] = 0xff - length; // 数据长度取反
    memcpy(buffer + HEAD_LEN, data, DATA_LEN); // 拷贝数据   从data中复制DATA_LEN个字节到第一个参数

    if (Write((char*)buffer, length + HEAD_LEN + 2))
        return true;
    else
        return false;
}
/*************************************************
Function:       ReceiveBuff
Description:    读取数据包
Input:          src_buff dst_buff
Output:
Return:         true or false
Others:         bool
*************************************************/
int WT::ReceiveBuff()
{
    //NULL;
    //usleep(20000);
    size_t read_length = Read(buff_l_, COM_BUFF_LEN);
    // std::clog<<"llllllllllength "<<read_length<<std::endl;
    if (read_length == 0){return -1;}
//    cout << buff_l_[0] <<endl;
    static int cnt = 0;
    if (buff_l_[0] == 0x55 ) //check head
    {

//        std::cout<<"check q \n";

//        if(buff_l_[1] == 0x53){ //angle
//            if(imudataSumCheck(buff_l_)){
//                memcpy(buff_r_, buff_l_,DATA_LEN);
//                short data[3];  // x y z
//                data[0] = (short)((uint16_t)buff_r_[3]<<8 | (unsigned char)buff_r_[2]);
//                data[1] = (short)((uint16_t)buff_r_[5]<<8 | (unsigned char)buff_r_[4]);
//                data[2] = (short)((uint16_t)buff_r_[7]<<8 | (unsigned char)buff_r_[6]);
//
//                double x_rote_angle = (double)((double)data[0] / 32768 * CV_PI);
//                double y_rote_angle = (double)((double)data[1] / 32768 * CV_PI);
//                double z_rote_angle = (double)((double)data[2] / 32768 * CV_PI);
//
//            }

        if(buff_l_[1] == 0x59){ //q
            if(imudataSumCheck(buff_l_)){
                memcpy(buff_r_, buff_l_,DATA_LEN);
                short tmp_q[4];
                tmp_q[0] = (short)((uint16_t)buff_r_[3]<<8 | (unsigned char)buff_r_[2]);
                tmp_q[1] = (short)((uint16_t)buff_r_[5]<<8 | (unsigned char)buff_r_[4]);
                tmp_q[2] = (short)((uint16_t)buff_r_[7]<<8 | (unsigned char)buff_r_[6]);
                tmp_q[3] = (short)((uint16_t)buff_r_[9]<<8 | (unsigned char)buff_r_[8]);

                double q0 = (double)((double)tmp_q[0]/32768.0);
                double q1 = (double)((double)tmp_q[1]/32768.0);
                double q2 = (double)((double)tmp_q[2]/32768.0);
                double q3 = (double)((double)tmp_q[3]/32768.0);
                wt_lock.lock();
                wt = {q0, q1,q2,q3};
                wt_lock.unlock();

            }
        }else{
            //cout<<"[error ] receive check len error"<<endl;
            for (int i = 0; i < DATA_LEN; i++) {
                buff_r_[i] = 0;
            }
            return 0;
        }
    }else{
        //cout<<"[error ] receive check head error "<<endl;
        //cout << "SERIAL error1" << endl;
        for (int i = 0; i < DATA_LEN; i++){
            buff_r_[i] = 0;
            //receive[i] = 0;
        }
        return 0;
    }
}

void WT::receive_thread() {
     std::cout<<"serial thread create success :"<<std::endl;
     while(!this->PortInit());
    
     while(1){
        int status = this->ReceiveBuff();
        //std::cout<<"ssssssssssss"<<status<<std::endl;
        if(status == -1){   //serial offline test
            LOG(ERROR)<<"imu serial offline";
            while(!this->PortInit());
         }
    }
}



