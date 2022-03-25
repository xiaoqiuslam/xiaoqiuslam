
#include "unistd.h" // 符号常量的头文件
#include <stdio.h>      // 标准输入输出头文件
#include <fcntl.h>     // 文件的打开、数据写入、数据读取、关闭文件的操作。
#include <string.h> // 字符串函数
#include <stdlib.h> // 定义了四个变量类型、一些宏和各种通用工具函数
#include <termio.h>     // 提供了一个常规的终端接口，用于控制非同步通信端口
#include <sys/time.h>   // 时间
#include <unistd.h>     //  系统调用的封装如 fork、pipe 、read、write、close  
#include <assert.h>　// assert.h头文件定义了宏assert()，用于在运行时确保程序符合指定条件，如果不符合，就报错终止运行。这个宏常常被称为“断言”。
#include <errno.h>      // 该头文件定义了通过错误码来回报错误资讯的宏。errno 宏定义为一个 int 型态的左值, 包含任何函式使用errno功能所产生的上一个错误码。
#include <sys/mman.h>  // 处理内存分配
#include "stdint.h" // 定义整形类型的C标准库头文件
#include "bsp_Serial.h"

#define  CMSPAR 010000000000  

#define SERIAL_NUM      4

typedef struct 
{
    int Fd;
    struct termios OldTios;
    char Name[30];
}SERIAL_T;

SERIAL_T sSerial[SERIAL_NUM];

void bspSerial_Init(void)
{
	strcpy(sSerial[0].Name,"/dev/ttyUSB0");
	strcpy(sSerial[1].Name,"/dev/ttyS0");
	strcpy(sSerial[2].Name,"/dev/ttyS1");
	strcpy(sSerial[3].Name,"/dev/ttyS2");
	sSerial[0].Fd = -1;
	sSerial[1].Fd = -1;
	sSerial[2].Fd = -1;
	sSerial[3].Fd = -1;
}

int bspSerial_GetDevName(char *serial_name)
{
	if (strcasecmp(serial_name, sSerial[0].Name) == 0)
		return 0;
	else if (strcasecmp(serial_name, sSerial[1].Name) == 0)
		return 1;
	else if (strcasecmp(serial_name, sSerial[2].Name) == 0)
		return 2;
	else if (strcasecmp(serial_name, sSerial[3].Name) == 0)
		return 3;
	else		
		return -1;
}

void bspSerial_print(uint8_t *buf,int len)
{
	int i;
	for(i=0;i<len;i++)
		printf("%02X ",buf[i]);
	printf("\n\r");
}

static int termios_init(struct termios *tios,int baud,int parity,int data_bits,int stop_bits)
{
	speed_t baud_rate;

	if (tios == NULL)
		return -1;

	tios->c_line  = 0;

	tios->c_cc[VMIN ] = 0;
	tios->c_cc[VTIME] = 0;

	/* configure the input modes... */
	tios->c_iflag =  IGNBRK | IGNPAR | INPCK;

	/* configure the output modes... */
	tios->c_oflag = 0;     /* enable implementation-defined output processing */
	/* configure the control modes... */
	tios->c_cflag = CREAD | CLOCAL; 
	
	if (data_bits == 5)
		tios->c_cflag |= CS5;
	else if (data_bits == 6)
		tios->c_cflag |= CS6;
	else if (data_bits == 7)
		tios->c_cflag |= CS7;
	else if (data_bits == 8)
		tios->c_cflag |= CS8;
	else
		return -1;

	if (stop_bits == 1)
		tios->c_cflag &=~ CSTOPB;
	else if (stop_bits == 2)
		tios->c_cflag |= CSTOPB;
	else
		return -1;

	if(parity == 0)
	{ /* none */
		tios->c_cflag &=~ PARENB;
		tios->c_cflag &=~ PARODD;
	}
	else if(parity == 2)
	{ /* even */
		tios->c_cflag |= PARENB;
		tios->c_cflag &=~ PARODD;
	}
	else if(parity == 1)
	{ /* odd */
		tios->c_cflag |= PARENB;
		tios->c_cflag |= PARODD;
	}
        else if (parity == 3)
        {
        /* mark */
                tios->c_cflag |= PARENB;
                tios->c_cflag |= CMSPAR;
                tios->c_cflag |= PARODD;
        }
        else if (parity == 4)
        {
        /* space */
                tios->c_cflag |= PARENB;
                tios->c_cflag |= CMSPAR;
        }
	else
		return -1;

	/* configure the local modes... */
	tios->c_lflag = 0;    /* enable implementation-defined input processing   */

	/* Set the baud rate */
	switch(baud)
	{
		case 110:
			baud_rate = B110;
			break;
		case 300:
			baud_rate = B300;
			break;
		case 600:
			baud_rate = B600;
			break;
		case 1200:
			baud_rate = B1200;
			break;
		case 2400:
			baud_rate = B2400;
			break;
		case 4800:
			baud_rate = B4800;
			break;
		case 9600:
			baud_rate = B9600;
			break;
		case 19200:
			baud_rate = B19200;
			break;
		case 38400:
			baud_rate = B38400;
			break;
		case 57600:
			baud_rate = B57600;
			break;
		case 115200:
			baud_rate = B115200;
			break;
		case 230400:
			baud_rate = B230400;
			break;
		case 460800:
			baud_rate = B460800;
			break;
		case 576000:
			baud_rate = B576000;
			break;
		case 921600:
			baud_rate = B921600;
			break;

		default:
			return -1;
	} 

	if ((cfsetispeed(tios, baud_rate) < 0) ||(cfsetospeed(tios, baud_rate) < 0))
		return -1;
	return 0;
}

//return serial fd, error return -1
int bspSerial_Open(int num, int baud,int parity,int data_bits,int stop_bits,int timeout)
{
	struct termios settings;
	int fd=-1;
	char *serial_dev_name;

    if(num>=SERIAL_NUM)
        return 0;

	if (sSerial[num].Fd >= 0)
		return sSerial[num].Fd;

	if (termios_init(&settings,baud,parity,data_bits,stop_bits) < 0)
		return 0;
	
	serial_dev_name = sSerial[num].Name;

	if (serial_dev_name == NULL)
	{
		fprintf(stderr, "invalid serial name:%s\n", sSerial[num].Name);
		return 0;
	}

	if((fd = open(serial_dev_name, O_RDWR | O_NOCTTY | O_NDELAY))< 0)
		return 0;

	if(tcgetattr(fd, &sSerial[num].OldTios) < 0)
	{
		close(fd);
		return 0;
	}

	if(tcsetattr(fd, TCSANOW, &settings) < 0)
	{
		close(fd);
		return 0;
	}

	sSerial[num].Fd = fd;
	return 1;
}

int bspSerial_Close(int num)
{
	if (sSerial[num].Fd < 0)
		return 1;
	tcsetattr(sSerial[num].Fd, TCSANOW, &(sSerial[num].OldTios));
	close(sSerial[num].Fd);
	sSerial[num].Fd = -1;
    
	return 1;
}

int serial_flush(int num,int flag)
{
	if (sSerial[num].Fd < 0)
		return 0;

	if (flag == SERIAL_FLUSH_TX)
		tcflush(sSerial[num].Fd,TCOFLUSH);
	else if (flag == SERIAL_FLUSH_RX)
		tcflush(sSerial[num].Fd,TCIFLUSH);
	else if (flag == (SERIAL_FLUSH_RX|SERIAL_FLUSH_TX) )
		tcflush(sSerial[num].Fd,TCIOFLUSH);
	return 1;
}

int serial_write(int num,uint8_t *buf,size_t size)
{
	int writesize = 0;

	writesize = write(sSerial[num].Fd,buf,size);
	return writesize;
}

int serial_read(int num,uint8_t *buf,size_t size)
{
	int readsize = 0;

	if (size<=0)
		return 0;

	readsize = read(sSerial[num].Fd,buf,size);
	return readsize;
}


//return 1: poll ok, 0: timeout, -1: error
int serial_poll(int num,int timeout)
{
	int fd = sSerial[num].Fd;
	
	fd_set rfds;
	struct timeval tv;
	int sel_res;

	FD_ZERO(&rfds);
	FD_SET(fd, &rfds);
	tv.tv_sec = timeout/1000;
	tv.tv_usec = (timeout % 1000)*1000;
	return select(fd + 1, &rfds, NULL,NULL,&tv);
}
