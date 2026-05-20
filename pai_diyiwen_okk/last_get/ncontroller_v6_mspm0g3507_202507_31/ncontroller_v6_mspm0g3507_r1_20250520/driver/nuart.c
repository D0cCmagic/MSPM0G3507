#include "ti_msp_dl_config.h"
#include "stdio.h"
#include "datatype.h"
#include "system.h"
#include "motor_control.h"
#include "sensor.h"
#include "nclink.h"
#include "us100.h"
#include "vision.h"
#include "nuart.h"
#include "uart1_dma.h"

void usart_irq_config(void)
{
  NVIC_ClearPendingIRQ(UART_0_INST_INT_IRQN);
  NVIC_ClearPendingIRQ(UART_1_INST_INT_IRQN);
  NVIC_ClearPendingIRQ(UART_2_INST_INT_IRQN);
  //NVIC_ClearPendingIRQ(UART_3_INST_INT_IRQN);
  NVIC_EnableIRQ(UART_0_INST_INT_IRQN);
  /* UART1 RX now DMA-driven — CPU interrupt disabled */
//  NVIC_EnableIRQ(UART_1_INST_INT_IRQN);
  NVIC_EnableIRQ(UART_2_INST_INT_IRQN);
//  NVIC_EnableIRQ(UART_3_INST_INT_IRQN);
	DL_UART_clearInterruptStatus(UART_0_INST,DL_UART_INTERRUPT_RX);//����жϱ�־λ
	/* UART1 RX now DMA-driven — interrupt flag no longer needed */
//	DL_UART_clearInterruptStatus(UART_1_INST,DL_UART_INTERRUPT_RX);//����жϱ�־λ
	DL_UART_clearInterruptStatus(UART_2_INST,DL_UART_INTERRUPT_RX);//����жϱ�־λ
	//DL_UART_clearInterruptStatus(UART_3_INST,DL_UART_INTERRUPT_RX);//����жϱ�־λ
	
	NVIC_SetPriority(SysTick_IRQn,0);
}

void UART_0_INST_IRQHandler(void)
{
  if(DL_UART_getEnabledInterruptStatus(UART_0_INST,DL_UART_INTERRUPT_RX) == DL_UART_INTERRUPT_RX)
  {
		uint8_t ch = DL_UART_receiveData(UART_0_INST);
		NCLink_Data_Prase_Prepare_Lite(ch);
		DL_UART_clearInterruptStatus(UART_0_INST,DL_UART_INTERRUPT_RX);//����жϱ�־λ
  }
}


//void UART_1_INST_IRQHandler(void)
//{
//  if(DL_UART_getEnabledInterruptStatus(UART_1_INST,DL_UART_INTERRUPT_RX) == DL_UART_INTERRUPT_RX)
//  {
//		uint8_t ch = DL_UART_receiveData(UART_1_INST);
//		//SDK_Data_Receive_Prepare_1(ch);
//		parse_char(ch);
//		//DL_UART_transmitData(UART_1_INST,values[0]);
//		DL_UART_clearInterruptStatus(UART_1_INST,DL_UART_INTERRUPT_RX);//����жϱ�־λ
//  }
//}

/*
 * UART1 RX is now handled by DMA on channel 1.
 * The CPU interrupt is disabled. See driver/uart1_dma.c.
 */
//void UART_1_INST_IRQHandler(void)
//{
//  if(DL_UART_getEnabledInterruptStatus(UART_1_INST,DL_UART_INTERRUPT_RX) == DL_UART_INTERRUPT_RX)
//  {
//    uint8_t ch = DL_UART_receiveData(UART_1_INST);
//    parse_char(ch);
//    DL_UART_clearInterruptStatus(UART_1_INST, DL_UART_INTERRUPT_RX); // ����жϱ�־λ
//  }
//}
systime uart2_dma_t;
void UART_2_INST_IRQHandler(void)
{
  if(DL_UART_getEnabledInterruptStatus(UART_2_INST,DL_UART_INTERRUPT_RX) == DL_UART_INTERRUPT_RX)
  {
		uint8_t ch = DL_UART_receiveData(UART_2_INST);
		bluetooth_app_prase(ch);
		DL_UART_clearInterruptStatus(UART_2_INST,DL_UART_INTERRUPT_RX);//����жϱ�־λ
  }

	switch (DL_UART_Main_getPendingInterrupt(UART2)) 
	{
		case DL_UART_MAIN_IIDX_DMA_DONE_TX:
			get_systime(&uart2_dma_t);
		break;
		default:break;
	}
}

void UART_3_INST_IRQHandler(void)
{
//  if(DL_UART_getEnabledInterruptStatus(UART_3_INST,DL_UART_INTERRUPT_RX) == DL_UART_INTERRUPT_RX)
//  {
//		uint8_t ch = DL_UART_receiveData(UART_3_INST);
//		if(com3_rx_cnt>=2) com3_rx_cnt=0;
//		com3_rx_buf[com3_rx_cnt++]=ch;
//		DL_UART_clearInterruptStatus(UART_3_INST,DL_UART_INTERRUPT_RX);//����жϱ�־λ
//  }
}


void usart0_send_string(uint8_t *ucstr)
{
  while(ucstr && *ucstr)
  {
    //DL_UART_Main_transmitData(UART_0_INST, *ucstr++);
    DL_UART_Main_transmitDataBlocking(UART_0_INST, *ucstr++);
  }
}


void usart0_send_bytes(unsigned char *buf, int len)
{
  while(len--)
  {
    DL_UART_Main_transmitDataBlocking(UART_0_INST, *buf);//DL_UART_Main_transmitData(UART_0_INST, *buf);
    buf++;
  }
}


//int fputc(int ch, FILE *f)
//{
//  DL_UART_Main_transmitDataBlocking(UART_0_INST, ch);
//  return ch;
//}




void usart1_send_bytes(unsigned char *buf, int len)
{
  while(len--)
  {
    DL_UART_Main_transmitDataBlocking(UART_1_INST, *buf);
    buf++;
  }
}

/***************************************
������:	void UART_SendBytes(uint32_t port,uint8_t *ubuf, uint32_t len)
˵��: ����N���ֽڳ��ȵ�����
���:	uint32_t port-���ں�
			uint8_t *ubuf-���������ݵ�ַ 
			uint32_t len-���������ݳ���
����:	��
��ע:	��
����:	��������
***************************************/
void UART_SendBytes(UART_Regs *port,uint8_t *ubuf, uint32_t len)
{
  while(len--)
  {
    DL_UART_Main_transmitDataBlocking(port, *ubuf);
    ubuf++;
  }
}

/***************************************
������:	void UART_SendByte(uint32_t port,uint8_t data)
˵��: ����1���ֽڳ��ȵ�����
���:	uint32_t port-���ں�
			uint8_t data-����������
����:	��
��ע:	��
����:	��������
***************************************/
void UART_SendByte(UART_Regs *port,uint8_t data)
{
  DL_UART_Main_transmitDataBlocking(port, data);//DL_UART_Main_transmitData(UART_0_INST, *buf);
}



int fputc(int _c, FILE *_fp)
{
    DL_UART_Main_transmitDataBlocking(UART_1_INST,_c);
    return _c;
}

int fputs(const char *restrict _ptr, FILE *restrict _fp)
{
    uint16_t i,len;
    len = strlen(_ptr);
    for(int i=0;i<len;i++)
    {
        DL_UART_Main_transmitDataBlocking(UART_1_INST,_ptr[i]);
    }
    return len;
}

int puts(const char *_ptr)
{
    int count = fputs(_ptr,stdout);
    count += fputs("\n", stdout);
    return count;
}


