#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <fcntl.h>
#include <errno.h>
#include <termios.h>
#include <signal.h>
#include <time.h>
#include <sys/select.h>
#include <sys/ioctl.h>
#include <sys/types.h>
#include <sys/stat.h>

#define BUFFER_SIZE 256
#define UART_DEVICE_SRC "/dev/ttyS7"
#define UART_DEVICE_DST "/dev/ttyS3"

// 全局变量
volatile int running = 1;
volatile long total_bytes_rx = 0;
volatile long total_bytes_tx = 0;
time_t last_report_time = 0;

// 信号处理函数
void handle_signal(int sig)
{
    if (sig == SIGINT) {
        printf("\n接收到中断信号，程序将退出\n");
        printf("总共接收: %ld 字节, 总共发送: %ld 字节\n", total_bytes_rx, total_bytes_tx);
        running = 0;
    }
}

// 配置串口
int configure_uart(int fd)
{
    struct termios options;
    
    // 获取当前配置
    if (tcgetattr(fd, &options) < 0) {
        printf("无法获取串口配置: %s\n", strerror(errno));
        return -1;
    }
    
    // 设置输入输出波特率
    cfsetispeed(&options, B9600);
    cfsetospeed(&options, B9600);
    
    // 基本配置
    options.c_cflag &= ~PARENB;            // 无奇偶校验
    options.c_cflag &= ~CSTOPB;            // 1个停止位
    options.c_cflag &= ~CSIZE;             // 清除数据位设置
    options.c_cflag |= CS8;                // 8个数据位
    options.c_cflag |= CLOCAL | CREAD;     // 启用接收器，忽略调制解调器控制线
    
    // 输入配置 - 不执行任何特殊处理
    options.c_iflag = 0;
    
    // 输出配置 - 不执行任何特殊处理
    options.c_oflag = 0;
    
    // 本地配置 - 不回显、不处理控制字符等
    options.c_lflag = 0;
    
    // 读取配置：完全阻塞模式，直到收到至少1个字节
    options.c_cc[VMIN] = 1;    // 最少读取1个字符
    options.c_cc[VTIME] = 0;   // 不设超时，一直等待直到有数据
    
    // 清空输入输出缓冲
    tcflush(fd, TCIOFLUSH);
    
    // 应用配置
    if (tcsetattr(fd, TCSANOW, &options) < 0) {
        printf("无法设置串口配置: %s\n", strerror(errno));
        return -1;
    }
    
    // 设置DTR和RTS信号为高电平
    int status;
    if (ioctl(fd, TIOCMGET, &status) != -1) {
        status |= TIOCM_DTR | TIOCM_RTS;
        if (ioctl(fd, TIOCMSET, &status) == -1) {
            printf("设置DTR/RTS信号失败: %s\n", strerror(errno));
        }
    }
    
    return 0;
}

// 打印调试信息
void print_status()
{
    time_t now = time(NULL);
    if (now - last_report_time >= 5) {  // 每5秒输出一次状态
        printf("[状态] 已接收: %ld 字节, 已发送: %ld 字节\n", total_bytes_rx, total_bytes_tx);
        last_report_time = now;
    }
}

int main()
{
    int fd_src, fd_dst;
    unsigned char buffer[BUFFER_SIZE];
    int bytes_read, bytes_written;
    
    // 注册信号处理
    signal(SIGINT, handle_signal);
    
    printf("串口数据透传程序启动\n");
    printf("源设备: %s, 目标设备: %s, 波特率: 115200\n", UART_DEVICE_SRC, UART_DEVICE_DST);
    
    // 打开源串口设备
    fd_src = open(UART_DEVICE_SRC, O_RDWR | O_NOCTTY);
    if (fd_src < 0) {
        printf("无法打开源串口设备 %s: %s\n", UART_DEVICE_SRC, strerror(errno));
        return -1;
    }
    
    printf("源串口设备打开成功\n");
    
    // 打开目标串口设备
    fd_dst = open(UART_DEVICE_DST, O_RDWR | O_NOCTTY);
    if (fd_dst < 0) {
        printf("无法打开目标串口设备 %s: %s\n", UART_DEVICE_DST, strerror(errno));
        close(fd_src);
        return -1;
    }
    
    printf("目标串口设备打开成功\n");
    
    // 配置串口
    if (configure_uart(fd_src) < 0) {
        printf("源串口配置失败\n");
        close(fd_src);
        close(fd_dst);
        return -1;
    }
    
    if (configure_uart(fd_dst) < 0) {
        printf("目标串口配置失败\n");
        close(fd_src);
        close(fd_dst);
        return -1;
    }
    
    printf("串口配置完成: 115200 波特率, 8N1, 无流控制\n");
    
    // 清空可能存在的数据
    tcflush(fd_src, TCIOFLUSH);
    tcflush(fd_dst, TCIOFLUSH);
    
    printf("开始透传数据 %s -> %s...\n", UART_DEVICE_SRC, UART_DEVICE_DST);
    last_report_time = time(NULL);
    
    // 主循环 - 使用select监控串口
    while (running) {
        fd_set readfds;
        struct timeval timeout;
        
        FD_ZERO(&readfds);
        FD_SET(fd_src, &readfds);
        
        // 使用较短的超时，允许定期检查状态
        timeout.tv_sec = 0;
        timeout.tv_usec = 200000;  // 200毫秒
        
        int result = select(fd_src + 1, &readfds, NULL, NULL, &timeout);
        
        if (result < 0) {
            if (errno == EINTR) {
                continue;  // 被信号中断，继续循环
            }
            printf("select错误: %s\n", strerror(errno));
            break;
        }
        else if (result == 0) {
            // 超时，没有数据，打印状态信息
            print_status();
        }
        else {
            // 有数据可读
            bytes_read = read(fd_src, buffer, BUFFER_SIZE);
            
            if (bytes_read > 0) {
                total_bytes_rx += bytes_read;
                
                // 打印接收到的数据
                printf("接收 (%d字节): ", bytes_read);
                for (int i = 0; i < bytes_read; i++) {
                    printf("%02X ", buffer[i]);
                }
                printf("| ");
                for (int i = 0; i < bytes_read; i++) {
                    if (buffer[i] >= 32 && buffer[i] <= 126) {
                        printf("%c", buffer[i]);
                    } else {
                        printf(".");
                    }
                }
                printf("\n");
                
                // 将数据写入目标串口
                bytes_written = write(fd_dst, buffer, bytes_read);
                if (bytes_written < 0) {
                    printf("写入目标串口错误: %s\n", strerror(errno));
                } else {
                    total_bytes_tx += bytes_written;
                    printf("发送 (%d字节)\n", bytes_written);
                    
                    // 确保所有数据都被发送出去
                    tcdrain(fd_dst);
                }
            }
            else if (bytes_read < 0) {
                if (errno != EAGAIN && errno != EWOULDBLOCK) {
                    printf("读取错误: %s\n", strerror(errno));
                    break;
                }
            }
        }
    }
    
    // 关闭串口
    close(fd_src);
    close(fd_dst);
    printf("程序结束\n");
    
    return 0;
} 