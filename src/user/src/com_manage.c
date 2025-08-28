#include "com_manage.h"
#include <stdlib.h>
#include <string.h>
#include "msg_que.h"
#include "ring.h"
#include <stdio.h>
#include "usart.h"
#include "led.h"
#include "le_be.h"

enum
{
	CMD_OFF,
	CMD_ON,
};

typedef enum
{
	SW_ALL,
	SW_CH1,
	SW_CH2,
	SW_CH3,
	SW_CH4,
	SW_MAX
}sw_ch_e;

typedef struct{
	uint8_t *dat;
	uint16_t len;
	void (*callback)(uint8_t *buf, uint16_t len);
}msg_t;

typedef struct{
	unsigned char buf[APP_FRAME_MAX_LEN];
	unsigned char len;
	unsigned char timeout;
}uart_recv_t;

static uart_recv_t uart_rx_hd = {{0},0,0};
//缓存
static unsigned char rx_data[APP_FRAME_MAX_LEN] = {0};//接收buff
//static unsigned char tx_data[APP_FRAME_MAX_LEN] = {0};//发送buff

static unsigned char com_init = 0;

Queue tx_mq_hd;
Queue rx_mq_hd;

msg_t *tx_msg = NULL;
msg_t *rx_msg = NULL;

#define LOG_HEX(raw, len)    do{ const int l=(len); int x;                          \
                                 for(x=0 ; x<l ; x++) printf("0x%02x ",*((raw)+x)); \
                                 printf("\r\n");}while(0)



//void com_uart_recv_process(uint8_t *buf, uint16_t len);
void com_message_post(Queue *que_hd, void(*cb)(uint8_t *buf, uint16_t len), uint8_t *buf, uint16_t len);

/*
 * @brief:   crc8 calculate
 *
 * */
uint8_t crc8Calculate(uint16_t type, uint16_t length, uint8_t *data)
{
	uint16_t n;
	uint8_t crc8;
	crc8  = (type   >> 0) & 0xff;
	crc8 ^= (type   >> 8) & 0xff;
	crc8 ^= (length >> 0) & 0xff;
	crc8 ^= (length >> 8) & 0xff;
	for(n = 0; n < length; n++)	
	{
		crc8 ^= data[n];
	}
	return crc8;
}

// 全开:01 0F 00 00 00 04 01 0F 7E 92 	全关:01 0F 00 00 00 04 01 00 3E 96
// 第一个开:01 05 00 00 FF 00 8C 3A   	第一个关闭:01 05 00 00 00 00 CD CA
// 第二个开:01 05 00 01 FF 00 DD FA   	第二个关闭:01 05 00 01 00 00 9C 0A
// 第三个开:01 05 00 02 FF 00 2D FA   	第三个关闭:01 05 00 02 00 00 6C 0A
// 第四个开:01 05 00 03 FF 00 7C 3A   	第四个关闭:01 05 00 03 00 00 3D CA
const uint8_t sw_all_on_cmd[] =  {0x01, 0x0F, 0x00, 0x00, 0x00, 0x04, 0x01, 0x0F, 0x7E, 0x92}; //全开
const uint8_t sw_all_off_cmd[] = {0x01, 0x0F, 0x00, 0x00, 0x00, 0x04, 0x01, 0x00, 0x3E, 0x96}; //全关
const uint8_t sw1_on_cmd[] =  {0x01, 0x05, 0x00, 0x00, 0xFF, 0x00, 0x8C, 0x3A};
const uint8_t sw1_off_cmd[] = {0x01, 0x05, 0x00, 0x00, 0x00, 0x00, 0xCD, 0xCA};
const uint8_t sw2_on_cmd[] =  {0x01, 0x05, 0x00, 0x01, 0xFF, 0x00, 0xDD, 0xFA};
const uint8_t sw2_off_cmd[] = {0x01, 0x05, 0x00, 0x01, 0x00, 0x00, 0x9C, 0x0A};
const uint8_t sw3_on_cmd[] =  {0x01, 0x05, 0x00, 0x02, 0xFF, 0x00, 0x2D, 0xFA};
const uint8_t sw3_off_cmd[] = {0x01, 0x05, 0x00, 0x02, 0x00, 0x00, 0x6C, 0x0A};
const uint8_t sw4_on_cmd[] =  {0x01, 0x05, 0x00, 0x03, 0xFF, 0x00, 0x7C, 0x3A};
const uint8_t sw4_off_cmd[] = {0x01, 0x05, 0x00, 0x03, 0x00, 0x00, 0x3D, 0xCA};

void usart2_send_to_wifi(uint8_t *buf, uint16_t len)
{
	HAL_UART_Transmit(&huart2, buf, len, 1000);
}
void usart3_send_to_mcu(uint8_t *buf, uint16_t len)
{
	HAL_UART_Transmit(&huart3, buf, len, 1000);
}

//发送485控制指令, 控制继电器开关
void com_control_swtich(uint8_t *buf, uint16_t len)
{
	com_message_post(&tx_mq_hd, usart3_send_to_mcu, buf, len);
}
//MCU上报开关状态
void com_report_switch_state(sw_ch_e ch, uint8_t on_off_cmd)
{
	if( ch >= SW_MAX ) return;

	uint8_t buf[100];
	snprintf((char *)buf, 100, "AT+REPORT=switch,{\"on\":%d}\r\n", on_off_cmd);
	// if( ch==SW_ALL)
	// {
	// 	snprintf((char *)buf, 100, "AT+REPORT={\"sid\":\"switch\",\"data\":{\"on\":%d}}\r\n", on_off_cmd);
	// }
	// else
	// {
	// 	snprintf((char *)buf, 100, "AT+REPORT={\"sid\":\"switch%d\",\"data\":{\"on\":%d}}\r\n", ch,on_off_cmd);
	// }
	com_message_post(&tx_mq_hd, usart2_send_to_wifi, buf, strlen((char *)buf));
}
//MCU上报 电压 电流 功率
void com_report_vaw(uint16_t v, float a, uint16_t w)
{
	uint8_t buf[100];
	snprintf((char *)buf, 100, "AT+REPORT={\"numbertype\":\"numberfor\",\"numberdata\":{\"numberv\":%d,\"numbera\":%.1f,\"numberw\":%d}}\r\n", v,a,w);
	com_message_post(&tx_mq_hd, usart2_send_to_wifi, buf, strlen((char *)buf));
}
////////////////////////////////////////////////////////////////////////////////////////////////////////////
//AT指令回调函数
void cmd_putchar_all_off(uint8_t *buf, uint16_t len)
{
	com_control_swtich((uint8_t *)sw_all_off_cmd, sizeof(sw_all_off_cmd));
	com_report_switch_state(SW_ALL, CMD_OFF);
}
void cmd_putchar_all_on(uint8_t *buf, uint16_t len)
{
	com_control_swtich((uint8_t *)sw_all_on_cmd, sizeof(sw_all_on_cmd));
	com_report_switch_state(SW_ALL, CMD_ON);
}
void cmd_putchar_sw1_off(uint8_t *buf, uint16_t len)
{
	com_control_swtich((uint8_t *)sw1_off_cmd, sizeof(sw1_off_cmd));
	com_report_switch_state(SW_CH1, CMD_OFF);
}
void cmd_putchar_sw1_on(uint8_t *buf, uint16_t len)
{
	com_control_swtich((uint8_t *)sw1_on_cmd, sizeof(sw1_on_cmd));
	com_report_switch_state(SW_CH1, CMD_ON);
}
void cmd_putchar_sw2_off(uint8_t *buf, uint16_t len)
{
	com_control_swtich((uint8_t *)sw2_off_cmd, sizeof(sw2_off_cmd));
	com_report_switch_state(SW_CH2, CMD_OFF);
}
void cmd_putchar_sw2_on(uint8_t *buf, uint16_t len)
{
	com_control_swtich((uint8_t *)sw2_on_cmd, sizeof(sw2_on_cmd));
	com_report_switch_state(SW_CH2, CMD_ON);
}
void cmd_putchar_sw3_off(uint8_t *buf, uint16_t len)
{
	com_control_swtich((uint8_t *)sw3_off_cmd, sizeof(sw3_off_cmd));
	com_report_switch_state(SW_CH3, CMD_OFF);
}
void cmd_putchar_sw3_on(uint8_t *buf, uint16_t len)
{	
	com_control_swtich((uint8_t *)sw3_on_cmd, sizeof(sw3_on_cmd));
	com_report_switch_state(SW_CH3, CMD_ON);
}
void cmd_putchar_sw4_off(uint8_t *buf, uint16_t len)
{	
	com_control_swtich((uint8_t *)sw4_off_cmd, sizeof(sw4_off_cmd));
	com_report_switch_state(SW_CH4, CMD_OFF);
}
void cmd_putchar_sw4_on(uint8_t *buf, uint16_t len)
{
	com_control_swtich((uint8_t *)sw4_on_cmd, sizeof(sw4_on_cmd));
	com_report_switch_state(SW_CH4, CMD_ON);
}

typedef struct{
	uint8_t *cmd;
	void (*docmd)(uint8_t *buf, uint16_t len);
}cmd_table_t;

cmd_table_t cmd_table[] = {
							{"+PUTCHAR=switch,{\"on\":0}",	cmd_putchar_all_off}, //APP下发数据 关
							{"+PUTCHAR=switch,{\"on\":1}",	cmd_putchar_all_on},  //APP下发数据 开
							{"+PUTCHAR=switch1,{\"on\":0}",	cmd_putchar_sw1_off},
							{"+PUTCHAR=switch1,{\"on\":1}",	cmd_putchar_sw1_on},
							{"+PUTCHAR=switch2,{\"on\":0}",	cmd_putchar_sw2_off},
							{"+PUTCHAR=switch2,{\"on\":1}",	cmd_putchar_sw2_on},
							{"+PUTCHAR=switch3,{\"on\":0}",	cmd_putchar_sw3_off},
							{"+PUTCHAR=switch3,{\"on\":1}",	cmd_putchar_sw3_on},
							{"+PUTCHAR=switch4,{\"on\":0}",	cmd_putchar_sw4_off},
							{"+PUTCHAR=switch4,{\"on\":1}",	cmd_putchar_sw4_on},
							};

void com_rx_message_analy(uint8_t *buf, uint16_t len)
{
	uint8_t table_size = sizeof(cmd_table)/sizeof(cmd_table[0]);
	uint8_t i=0;
	cmd_table_t *cmd=NULL;
	
	if( (buf == NULL) || (len == 0) ) return;

	for(i=0; i<table_size; i++ )
	{
		if(strncmp((char *)buf,(char *)(cmd_table[i].cmd),strlen((char *)(cmd_table[i].cmd))) == 0)
		{
			cmd = &cmd_table[i];
			break;
		}
	}

	if(i==table_size)
	{	
		printf("[%s %d] unknow cmdtype\r\n",__func__,__LINE__);
		return;
	}

	if(cmd)
	{
		if(cmd->docmd)
		{
			cmd->docmd(buf, len);
		}
	}
}
////////////////////////////////////////////////////////////////////////////////////////////////////////////
void com_message_post(Queue *que_hd, void(*cb)(uint8_t *buf, uint16_t len), uint8_t *buf, uint16_t len)
{
	msg_t *msg;
	msg = malloc(sizeof(msg_t) + len);
	if(msg == NULL)
	{
		return;
	}

	msg->dat = (uint8_t *)(msg + 1);
	memcpy(msg->dat, buf, len);
	msg->len = len;
	msg->callback = cb;
	if(EnterQueue(que_hd, &msg)==0)
	{
		free(msg);
	}
}


void com_uart_recv_process(uint8_t *buf, uint16_t len)
{
	if( len==3 )
	{
		if((buf[0]=='\r')&&(buf[1]=='\n'))//直接过滤掉
		{
			return;
		}
	}
	
	com_message_post(&rx_mq_hd, com_rx_message_analy, buf, len);
	printf("uart recv %d bytes <------:%s", len,buf);
}

void com_mq_init(void)
{
	InitQueue(&tx_mq_hd);
	InitQueue(&rx_mq_hd);
	ring_fifo_init();
}
//串口发送任务
void com_tx_task_10ms(void)
{
	if(!com_init)
	{
		com_mq_init();
		tx_msg = NULL;
		rx_msg = NULL;
		com_init = 1;
		return;
	}

	if(OutQueue(&tx_mq_hd, &tx_msg))
	{
		if(tx_msg->callback)
		{
			if(tx_msg->len)
			{
				tx_msg->callback(tx_msg->dat, tx_msg->len);
			}
		}
		free(tx_msg);
		tx_msg = NULL;
	}
}
//串口接收任务
void com_rx_task_10ms(void)
{
	unsigned char len = 0;
	unsigned char i = 0;

	if(!com_init)
	{
		return;
	}

	len = ring_fifo_read(ring_0, rx_data, APP_FRAME_MAX_LEN);
	if( len )
	{
		for(i=0; i<len; i++)
		{
			if( uart_rx_hd.len==0 )
			{
				uart_rx_hd.buf[uart_rx_hd.len++] = rx_data[i];
			}
			else
			{
				uart_rx_hd.buf[uart_rx_hd.len++] = rx_data[i];
				if( uart_rx_hd.len >= 2 )
				{
					if( (uart_rx_hd.buf[uart_rx_hd.len-2]=='\r') && (uart_rx_hd.buf[uart_rx_hd.len-1]=='\n') )
					{
						uart_rx_hd.buf[uart_rx_hd.len] = 0;
						com_uart_recv_process(uart_rx_hd.buf, uart_rx_hd.len+1);
						uart_rx_hd.len = 0;
						uart_rx_hd.timeout = 0;
					}
				}

				if( uart_rx_hd.len==APP_FRAME_MAX_LEN )
				{
					uart_rx_hd.len = 0;
					uart_rx_hd.timeout = 0;
				}
			}
		}
	}
	else
	{
		if(uart_rx_hd.len > 0)
		{
			if(uart_rx_hd.timeout++ > 200)
			{
				uart_rx_hd.len = 0;
				uart_rx_hd.timeout = 0;
			}
		}
	}
}
//接收任务 执行回调函数
void com_oam_task(void)
{
	if(!com_init)
	{
		return;
	}

	if( OutQueue(&rx_mq_hd, &rx_msg) )
	{
		if(rx_msg->callback)
		{
			rx_msg->callback(rx_msg->dat, rx_msg->len);
		}
		free(rx_msg);
		rx_msg = NULL;
	}
}
//定时上报任务
void com_timer_task(void)
{
	com_report_vaw(220, 1.5, 330);
}
