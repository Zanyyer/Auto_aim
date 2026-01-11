#include "serial.hpp"

#include <fcntl.h>   // 文件控制
#include <unistd.h>  // 引入POSIX标准系统调用头文件
#include <termios.h> // 引入串口配置相关头文件，提供termios结构体及相关配置函数
#include <cstring>
#include <iostream>

namespace io {
// 初始化串口设备路径、波特率和文件描述符
Serial::Serial(const std::string& port, int baudrate)
    : fd_(-1), port_(port), baudrate_(baudrate) {}

Serial::~Serial() {close();}

bool Serial::open() {
    // O_RDWR: 读写模式 O_NOCTTY: 不将串口作为控制终端 O_NONBLOCK: 非阻塞模式
    fd_ = ::open(port_.c_str(), O_RDWR | O_NOCTTY | O_NONBLOCK); // ：：open()全局的
    if (fd_ < 0) {
        perror("Serial open failed");
        return false;
    }
   
    termios tty{}; // 定义termios结构体变量tty，用于存储串口配置参数，初始化为全0
    tcgetattr(fd_, &tty);// 获取当前串口的配置参数，存入tty结构体
    cfmakeraw(&tty); // 将串口设置为原始模式，禁用所有输入输出处理

    // 波特率
    speed_t speed = B115200;
    if (baudrate_ == 921600) speed = B921600;

    cfsetispeed(&tty, speed); // serial in
    cfsetospeed(&tty, speed); // serial out

    tty.c_cflag |= (CLOCAL | CREAD); // 配置串口控制标志位：CLOCAL忽略调制解调器状态线 CREAD启用接收功能
    tty.c_cflag &= ~CSTOPB; // 清除CSTOPB标志位，设置为1个停止位（默认）
    tty.c_cflag &= ~CRTSCTS; // 清除CRTSCTS标志位，禁用硬件流控（RTS/CTS）

    tcsetattr(fd_, TCSANOW, &tty); // 将修改后的tty配置应用到串口，TCSANOW表示立即生效
    return true;
}

void Serial::close() {
    if (fd_ >= 0) {
        ::close(fd_);
        fd_ = -1;
    }
}

bool Serial::isOpened() const {
    return fd_ >= 0;
}

bool Serial::send(const uint8_t* data, size_t size) {
    if (fd_ < 0) return false;
    return ::write(fd_, data, size) == (ssize_t)size;
}

} // namespace io
