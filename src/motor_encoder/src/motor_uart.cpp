/// @file  motor_uart.cpp
/// @brief 电机串口配置文件
///
///        电机波特率配置为�?600
///        电机型号PD20-1-1210,串口通信
/// @author zhouhao <1198826224@qq.com>
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <fcntl.h>
#include <sys/stat.h>
#include <sys/types.h>
#include <termios.h>
#include <string.h>
#include <sys/ioctl.h>

#define MOTOR_PORT_PATH "/dev/motor"

static int _MotorUartOpen(int fd, const char *path)
{
    printf("try to open %s ...\r\n", path);
    for(int i=0;i<10;i++)
    {
        fd = open(path, O_RDWR | O_NOCTTY | O_NONBLOCK);//非阻塞式读写
        if(fd > 2)
        {
            break;
        }
        else
        {
            perror("open fail number \n");
            sleep(10);
        }
    }
    if (-1 == fd){
        perror("Can't Open Serial Port");
        return(-1);
    }
    else
        printf("open %s .....\r\n", path);

    printf("fd-open=%d\n",fd);
    return fd;
}

/* 五个参量 fd打开文件 speed设置波特�?bit数据位设�?  nevent奇偶校验�?stop停止�?*/
static int _MotorUartSet(int fd, int nSpeed, int nBits, char nEvent, int nStop)
{
    struct termios newtio,oldtio;
    int baud;

    if ( tcgetattr( fd,&oldtio) != 0) {
        perror("SetupSerial 1");
        return -1;
    }

    bzero( &newtio, sizeof( newtio ) );
    newtio.c_cflag |= CLOCAL | CREAD;
    newtio.c_cflag &= ~CSIZE;

    switch( nBits )
    {
        case 7: newtio.c_cflag |= CS7;
        break;
        case 8: newtio.c_cflag |= CS8;
        break;
    }

    switch( nEvent )
    {
        case 'O':
            newtio.c_cflag |= PARENB;
            newtio.c_cflag |= PARODD;
            newtio.c_iflag |= (INPCK | ISTRIP);
            break;
        case 'E':
            newtio.c_iflag |= (INPCK | ISTRIP);
            newtio.c_cflag |= PARENB;
            newtio.c_cflag &= ~PARODD;
            break;
        case 'N':
            newtio.c_cflag &= ~PARENB;
            break;
    }

    switch( nSpeed )
    {
        case 1200:   baud = B1200;   break;
        case 2400:   baud = B2400;   break;
        case 4800:   baud = B4800;   break;
        case 9600:   baud = B9600;   break;
        case 19200:  baud = B19200;  break;
        case 38400:  baud = B38400;  break;
        case 57600:  baud = B57600;  break;
        case 115200: baud = B115200; break;
        case 230400: baud = B230400; break;
        case 460800: baud = B460800; break;
        default:     baud = B9600;   break;
    }
    cfsetispeed(&newtio, baud);
    cfsetospeed(&newtio, baud);

    if( nStop == 1 )       newtio.c_cflag &= ~CSTOPB;
    else if ( nStop == 2 ) newtio.c_cflag |= CSTOPB;

    newtio.c_cc[VTIME] = 0;
    newtio.c_cc[VMIN] = 0;
//串口只是传输数据，不需要串口来处理，则使用原始模式(Raw Mode)
    newtio.c_lflag  &= ~(ICANON | ECHO | ECHOE | ISIG);  /*Input*/
    newtio.c_oflag  &= ~OPOST;   /*Output*/

    tcflush(fd, TCIFLUSH);
    if((tcsetattr(fd,TCSANOW,&newtio))!=0)
    {
        perror("com set error");
        return -1;
    }
    printf("set done!\n");

    return 0;
}

int MotorUartInit(void)
{
    int fd = 0;

    if((fd = _MotorUartOpen(fd, MOTOR_PORT_PATH)) < 0)
    {
        return -1;
    }

    if((_MotorUartSet(fd,9600,8,'N',1)) < 0)
    {
        return -1;
    }

    return fd;
}
