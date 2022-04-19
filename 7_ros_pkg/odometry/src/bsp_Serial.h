#ifndef _BSP_SERIAL_H
#define _BSP_SERIAL_H

#include <stdint.h>
#include <stdio.h>    

#define SERIAL_FLUSH_TX		1
#define SERIAL_FLUSH_RX		2

#define SERIAL_PARITY_NO        0
#define SERIAL_PARITY_ODD       1
#define SERIAL_PARITY_EVENT     2
#define SERIAL_PARITY_MARK      3
#define SERIAL_PARITY_SPACE     4

#define SERIAL_STOPBIT_ONE      1
#define SERIAL_STOPBIT_TWO      2

#define SERIAL_COM_1    0
#define SERIAL_COM_2    1
#define SERIAL_COM_3    2
#define SERIAL_COM_4    3

void bspSerial_Init(void);
int bspSerial_Open(int num, int baud,int parity,int data_bits,int stop_bits,int timeout);
int bspSerial_Close(int num);
int serial_write(int num,uint8_t *buf,size_t size);
int serial_read(int num,uint8_t *buf,size_t size);
int serial_poll(int num,int timeout);
void bspSerial_print(uint8_t *buf,int len);
int serial_flush(int num,int flag);
int bspSerial_GetDevName(char *serial_name);

#endif
