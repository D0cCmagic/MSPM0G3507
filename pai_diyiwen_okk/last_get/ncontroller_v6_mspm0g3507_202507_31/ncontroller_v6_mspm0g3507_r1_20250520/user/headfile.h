#ifndef __HEADFILE_H
#define __HEADFILE_H





#define heater_port PORTB_PORT
#define heater_pin  PORTB_HEATER_PIN



#define KEYUP  GPIOPinRead(keyup_port  ,keyup_pin)
#define KEYDN  GPIOPinRead(keydown_port,keydown_pin)



#define  USER_INT0  0x00   //PPM     ң����PPM���ݽ���  0x00<<6
#define  USER_INT1  0x20   //UART6   GPS���ݽ���/ROSͨѶ����	921600/460800
#define  USER_INT2  0x40   //UART0   �ײ�OPENMV���ݽ���	256000
                           //UART3   ǰ��OPENMV���ݽ���	256000
													 
#define  USER_INT3  0x60   //UART7   ������ͨѶ����   921600
													 //UART1   �������µ���վ����	921600
													 
#define  USER_INT4  0x80   //UART2   �������ݽ���19200/�����״�230400
#define  USER_INT5  0xA0	 //TIMER1A   1ms
#define  USER_INT6  0xC0   //TIMER0		 5ms
#define  USER_INT7  0xE0   //TIMER2    10ms
													 //ADC0SS0




#include "datatype.h"
#include "main.h"


/*********************************************************/
#include "wp_math.h"
#include "filter.h"
#include "pid.h"
#include "system.h"
#include "ntimer.h"
#include "npwm.h"
#include "nppm.h"
#include "nqei.h"
#include "nadc.h"
#include "nuart.h"
#include "oled.h"
#include "ssd1306.h"
#include "ni2c.h"
#include "neeprom.h"
#include "w25qxx.h"
#include "rc.h"
#include "nbutton.h"
#include "ngpio.h"
#include "us100.h"
#include "rgb.h"
#include "icm20608.h"
#include "vision.h"
#include "gray_detection.h"
#include "sensor.h"					
#include "self_balancing.h"
#include "motor_control.h"
#include "attitude_selfstable.h"
#include "ui.h"
#include "sdk.h"
#include "subtask.h"
#include "uart1_dma.h"
#include "uart1_parser.h"
#include "developer_mode.h"
#include "nclink.h"
#endif
