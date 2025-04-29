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
#include <sys/time.h>  // 添加用于gettimeofday()的头文件

#define BUFFER_SIZE 1024
#define PACKET_BUFFER_SIZE 4096
#define UART_DEVICE_SRC "/dev/ttyS7"
#define UART_DEVICE_DST "/dev/ttyS3"
#define SHELL_TIMEOUT_MS 1000  // 300毫秒判定shell输出结束

// 数据包结构
#define HEAD_SIZE 2
#define LEN_SIZE 2
#define SN_SIZE 2
#define CMD_SIZE 1
#define SUM_SIZE 2
#define HEAD_VALUE_0 0x5A
#define HEAD_VALUE_1 0xA5

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
int configure_uart(int fd, int baud_rate)
{
    struct termios options;
    speed_t speed;
    
    // 根据传入的波特率设置对应的常量
    switch(baud_rate) {
        case 9600: speed = B9600; break;
        case 19200: speed = B19200; break;
        case 38400: speed = B38400; break;
        case 57600: speed = B57600; break;
        case 115200: speed = B115200; break;
        default:
            printf("不支持的波特率: %d，使用默认值9600\n", baud_rate);
            speed = B9600;
            break;
    }
    
    // 获取当前配置
    if (tcgetattr(fd, &options) < 0) {
        printf("无法获取串口配置: %s\n", strerror(errno));
        return -1;
    }
    
    // 设置输入输出波特率
    cfsetispeed(&options, speed);
    cfsetospeed(&options, speed);
    
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

// 打印数据包内容
void print_packet(unsigned char *packet, int length) {
    printf("数据包内容 (%d字节): ", length);
    for (int i = 0; i < length; i++) {
        printf("%02X ", packet[i]);
    }
    printf("\n");
    
    if (length >= (HEAD_SIZE + LEN_SIZE + SN_SIZE + CMD_SIZE)) {
        printf("数据包解析:\n");
        printf("  HEAD: %02X %02X\n", packet[0], packet[1]);
        
        unsigned short len = (packet[2] << 8) | packet[3];
        printf("  LEN: %04X (%d)\n", len, len);
        
        unsigned short sn = (packet[4] << 8) | packet[5];
        printf("  SN: %04X (%d)\n", sn, sn);
        
        unsigned char cmd = packet[6];
        printf("  CMD: %02X\n", cmd);
        
        printf("  DATA: ");
        int data_len = len - SN_SIZE - CMD_SIZE;
        for (int i = 0; i < data_len; i++) {
            printf("%02X ", packet[7 + i]);
        }
        printf("\n");
        
        unsigned short sum = (packet[length - 2] << 8) | packet[length - 1];
        printf("  SUM: %04X\n", sum);
    }
}

// 计算校验和
unsigned short calculate_sum(unsigned char *data, int length) {
    unsigned short sum = 0;
    for (int i = 0; i < length; i++) {
        sum += data[i];
    }
    return sum;
}

// 执行shell命令并获取输出
int execute_shell_command(unsigned char *cmd_data, int cmd_length, unsigned char *response, int max_response_length) {
    // 记录开始时间
    struct timeval start_time, end_time;
    gettimeofday(&start_time, NULL);
    
    // 创建一个临时的空终止字符串用于shell命令
    char *shell_cmd = (char *)malloc(cmd_length + 1);
    if (!shell_cmd) {
        printf("内存分配失败\n");
        return -1;
    }
    
    memcpy(shell_cmd, cmd_data, cmd_length);
    shell_cmd[cmd_length] = '\0';
    
    printf("执行shell命令: %s\n", shell_cmd);
    
    // 使用popen执行命令
    FILE *fp = popen(shell_cmd, "r");
    free(shell_cmd);
    
    if (fp == NULL) {
        printf("popen()失败: %s\n", strerror(errno));
        return -1;
    }
    
    // 读取命令输出
    int total_read = 0;
    char buffer[256];
    
    while (fgets(buffer, sizeof(buffer), fp) != NULL && total_read < max_response_length) {
        int len = strlen(buffer);
        memcpy(response + total_read, buffer, len);
        total_read += len;
        printf("Shell输出(ASCII): %s", buffer);  // 直接打印ASCII输出
    }
    
    pclose(fp);
    
    // 计算并打印执行时间
    gettimeofday(&end_time, NULL);
    long elapsed_ms = (end_time.tv_sec - start_time.tv_sec) * 1000 + (end_time.tv_usec - start_time.tv_usec) / 1000;
    printf("Shell命令执行完成，耗时: %ld 毫秒，输出长度: %d字节\n", elapsed_ms, total_read);
    
    return total_read;
}

// 构建返回数据包
int build_response_packet(unsigned char *packet, unsigned short sn, unsigned char cmd,
                        unsigned char *data, int data_length, unsigned char *response_packet) {
    
    int packet_length = HEAD_SIZE + LEN_SIZE + SN_SIZE + CMD_SIZE + data_length + SUM_SIZE;
    
    // 头部
    response_packet[0] = HEAD_VALUE_0;
    response_packet[1] = HEAD_VALUE_1;
    
    // 长度
    unsigned short len = SN_SIZE + CMD_SIZE + data_length;
    response_packet[2] = (len >> 8) & 0xFF;
    response_packet[3] = len & 0xFF;
    
    // SN和CMD
    response_packet[4] = (sn >> 8) & 0xFF;
    response_packet[5] = sn & 0xFF;
    response_packet[6] = cmd;
    
    // 数据
    memcpy(response_packet + HEAD_SIZE + LEN_SIZE + SN_SIZE + CMD_SIZE, data, data_length);
    
    // 计算校验和
    unsigned short sum = calculate_sum(response_packet, packet_length - SUM_SIZE);
    response_packet[packet_length - 2] = (sum >> 8) & 0xFF;
    response_packet[packet_length - 1] = sum & 0xFF;
    
    printf("构建响应数据包，长度: %d字节\n", packet_length);
    
    // 打印ASCII形式的数据内容
    printf("响应包数据内容(ASCII): ");
    for (int i = HEAD_SIZE + LEN_SIZE + SN_SIZE + CMD_SIZE; i < packet_length - SUM_SIZE; i++) {
        if (response_packet[i] >= 32 && response_packet[i] <= 126) {
            printf("%c", response_packet[i]);
        } else {
            printf(".");
        }
    }
    printf("\n");
    
    // 十六进制形式的完整包
    print_packet(response_packet, packet_length);
    
    return packet_length;
}

int main()
{
    int fd_src, fd_dst;
    unsigned char buffer[BUFFER_SIZE];
    unsigned char packet_buffer[PACKET_BUFFER_SIZE];
    unsigned char response_buffer[PACKET_BUFFER_SIZE];
    int packet_pos = 0;
    int parsing_state = 0;  // 0: 等待头部第一字节, 1: 等待头部第二字节, 2: 正在接收数据
    int expected_length = 0;
    int bytes_read, bytes_written;
    
    // 注册信号处理
    signal(SIGINT, handle_signal);
    
    printf("串口数据透传程序启动\n");
    printf("源设备: %s, 目标设备: %s, 波特率: 9600\n", UART_DEVICE_SRC, UART_DEVICE_DST);
    
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
    if (configure_uart(fd_src, 9600) < 0) {
        printf("源串口配置失败\n");
        close(fd_src);
        close(fd_dst);
        return -1;
    }
    
    if (configure_uart(fd_dst, 9600) < 0) {
        printf("目标串口配置失败\n");
        close(fd_src);
        close(fd_dst);
        return -1;
    }
    
    printf("串口配置完成: 9600 波特率, 8N1, 无流控制\n");
    
    // 清空可能存在的数据
    tcflush(fd_src, TCIOFLUSH);
    tcflush(fd_dst, TCIOFLUSH);
    
    printf("开始解析数据包...\n");
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
                
                printf("接收 (%d字节): ", bytes_read);
                for (int i = 0; i < bytes_read; i++) {
                    printf("%02X ", buffer[i]);
                }
                printf("\n");
                
                // 处理接收到的每个字节
                for (int i = 0; i < bytes_read; i++) {
                    unsigned char byte = buffer[i];
                    
                    // 基于当前解析状态处理字节
                    switch (parsing_state) {
                        case 0:  // 等待头部第一字节
                            if (byte == HEAD_VALUE_0) {
                                printf("找到HEAD第一字节: 0x%02X\n", byte);
                                packet_buffer[0] = byte;
                                packet_pos = 1;
                                parsing_state = 1;
                            }
                            break;
                            
                        case 1:  // 等待头部第二字节
                            if (byte == HEAD_VALUE_1) {
                                printf("找到HEAD第二字节: 0x%02X\n", byte);
                                packet_buffer[1] = byte;
                                packet_pos = 2;
                                parsing_state = 2;
                            } else {
                                printf("无效的HEAD第二字节: 0x%02X，重新开始搜索\n", byte);
                                parsing_state = 0;
                                if (byte == HEAD_VALUE_0) {
                                    packet_buffer[0] = byte;
                                    packet_pos = 1;
                                    parsing_state = 1;
                                }
                            }
                            break;
                            
                        case 2:  // 正在接收数据
                            packet_buffer[packet_pos++] = byte;
                            
                            // 当我们收到足够的字节来确定包长度时
                            if (packet_pos == (HEAD_SIZE + LEN_SIZE)) {
                                expected_length = (packet_buffer[2] << 8) | packet_buffer[3];
                                printf("包长度字段解析为: %d字节\n", expected_length);
                                
                                // 总长度 = HEAD + LEN + 实际长度 + SUM
                                expected_length = HEAD_SIZE + LEN_SIZE + expected_length + SUM_SIZE;
                                printf("预期总包长度: %d字节\n", expected_length);
                            }
                            
                            // 当我们已经接收到了完整的包
                            if (expected_length > 0 && packet_pos == expected_length) {
                                printf("接收到完整数据包，长度: %d字节\n", packet_pos);
                                print_packet(packet_buffer, packet_pos);
                                
                                // 验证校验和
                                unsigned short calculated_sum = calculate_sum(packet_buffer, packet_pos - SUM_SIZE);
                                unsigned short packet_sum = (packet_buffer[packet_pos - 2] << 8) | packet_buffer[packet_pos - 1];
                                
                                printf("校验和: 包中值=0x%04X, 计算值=0x%04X\n", packet_sum, calculated_sum);
                                printf("注意: 校验和验证已暂时禁用，只检查包长度\n");
                                
                                // 暂时屏蔽校验和验证，只要长度满足就认为校验通过
                                // if (calculated_sum == packet_sum) {
                                if (1) {  // 暂时直接返回true，跳过校验和检查
                                    printf("包长度验证成功\n");
                                    
                                    // 提取CMD
                                    unsigned char cmd = packet_buffer[HEAD_SIZE + LEN_SIZE + SN_SIZE];
                                    unsigned short sn = (packet_buffer[4] << 8) | packet_buffer[5];
                                    
                                    printf("命令: 0x%02X, 序列号: %d\n", cmd, sn);
                                    
                                    // 根据不同CMD处理
                                    if (cmd == 0x01) {
                                        // CMD=0x01: 将整个包转发到ttyS3
                                        printf("CMD=0x01: 将数据包转发到ttyS3\n");
                                        bytes_written = write(fd_dst, packet_buffer, packet_pos);
                                        if (bytes_written < 0) {
                                            printf("写入目标串口错误: %s\n", strerror(errno));
                                        } else {
                                            total_bytes_tx += bytes_written;
                                            printf("成功发送 %d 字节到ttyS3\n", bytes_written);
                                            
                                            // 确保所有数据都被发送出去
                                            tcdrain(fd_dst);
                                        }
                                    } else if (cmd == 0x02) {
                                        // CMD=0x02: 执行shell命令
                                        printf("CMD=0x02: 执行shell命令\n");
                                        
                                        // 记录开始时间
                                        struct timeval cmd_start_time, cmd_end_time;
                                        gettimeofday(&cmd_start_time, NULL);
                                        
                                        // 提取命令数据
                                        int data_len = (packet_buffer[2] << 8) | packet_buffer[3];
                                        data_len -= (SN_SIZE + CMD_SIZE);  // 减去SN和CMD的长度
                                        
                                        unsigned char *cmd_data = &packet_buffer[HEAD_SIZE + LEN_SIZE + SN_SIZE + CMD_SIZE];
                                        
                                        // 执行shell命令
                                        int response_length = execute_shell_command(cmd_data, data_len, response_buffer, PACKET_BUFFER_SIZE - 100);
                                        
                                        if (response_length >= 0) {
                                            // 构建响应包
                                            int packet_length = build_response_packet(packet_buffer, sn, cmd, 
                                                                                    response_buffer, response_length, 
                                                                                    response_buffer + PACKET_BUFFER_SIZE/2);
                                            
                                            // 发送响应到ttyS7
                                            bytes_written = write(fd_src, response_buffer + PACKET_BUFFER_SIZE/2, packet_length);
                                            if (bytes_written < 0) {
                                                printf("写入ttyS7错误: %s\n", strerror(errno));
                                            } else {
                                                total_bytes_tx += bytes_written;
                                                printf("成功发送 %d 字节到ttyS7\n", bytes_written);
                                                
                                                // 确保所有数据都被发送出去
                                                tcdrain(fd_src);
                                            }
                                        }
                                        
                                        // 计算并打印总耗时
                                        gettimeofday(&cmd_end_time, NULL);
                                        long total_elapsed_ms = (cmd_end_time.tv_sec - cmd_start_time.tv_sec) * 1000 + 
                                                                (cmd_end_time.tv_usec - cmd_start_time.tv_usec) / 1000;
                                        printf("CMD=0x02处理完成，总耗时: %ld 毫秒\n", total_elapsed_ms);
                                    } else {
                                        printf("未知CMD: 0x%02X, 忽略此数据包\n", cmd);
                                    }
                                }
                                
                                // 重置解析状态，准备接收下一个包
                                parsing_state = 0;
                                packet_pos = 0;
                                expected_length = 0;
                            }
                            break;
                    }
                    
                    // 防止缓冲区溢出
                    if (packet_pos >= PACKET_BUFFER_SIZE) {
                        printf("警告: 包缓冲区溢出，重置解析状态\n");
                        parsing_state = 0;
                        packet_pos = 0;
                        expected_length = 0;
                    }
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