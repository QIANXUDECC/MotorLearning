/*
作者：拾小白电控
日期：2024.1.20
qq群：1群：860465413 
备注：代码仅供学习参考，未经允许，禁止商用或售卖


功能：上位机通信交互,这块代码尽量不要碰,看不懂没关系会用就行
*/
#include "Comm_usart.h"
#include "motorlib.h"

extern DMA_HandleTypeDef hdma_usart1_rx;
extern DMA_HandleTypeDef hdma_usart1_tx;
extern UART_HandleTypeDef huart1;

uint16_t data_length;
uint8_t Scope_buff[90];  

uint8_t receiveflag = 0;								
uint8_t transmit_buffer[9];  
uint16_t writeCD = 0;              	//写入冷却，避免猛发指令
uint8_t receive_buff[BUFFER_SIZE];    
uint8_t receive_data[BUFFER_SIZE];   
FloatToBytesUnion tmpUnion;
void ReceiveDeal(uint8_t* rcvbuf);
int dmaGGflag;
void ScopeCommPro(void)
{	
	datasave();	
	static uint16_t iComm;	
	static uint16_t index;	
	
	if(iComm>9)
		iComm = 0;
	
	int16_t tmp1 = (int16_t)(*i32OnlyRead_Arr[index]>>16);	
	int16_t tmp2 = (int16_t)(*i32OnlyRead_Arr[index]);
	
	Scope_buff[0] = 0x09;
	Scope_buff[1] = 0x09;
	Scope_buff[2] = 85;	

	Scope_buff[iComm*8+0+3] = DATATABLE[scope_s.scope1]>>8;
	Scope_buff[iComm*8+1+3] = DATATABLE[scope_s.scope1];

	Scope_buff[iComm*8+2+3] = DATATABLE[scope_s.scope2]>>8;
	Scope_buff[iComm*8+3+3] = DATATABLE[scope_s.scope2];

	Scope_buff[iComm*8+4+3] = DATATABLE[scope_s.scope3]>>8;
	Scope_buff[iComm*8+5+3] = DATATABLE[scope_s.scope3];

	Scope_buff[iComm*8+6+3] = DATATABLE[scope_s.scope4]>>8;
	Scope_buff[iComm*8+7+3] = DATATABLE[scope_s.scope4];	
	
	Scope_buff[83] = index;  
	Scope_buff[84] = tmp1>>8;
	Scope_buff[85] = tmp1;
	Scope_buff[86] = tmp2>>8;
	Scope_buff[87] = tmp2;//这里轮询采集状态量	
	
	Scope_buff[88] = 0x33;
	Scope_buff[89] = 0x44;

	if(iComm==9)
	{
		if(receiveflag == 0)
		{
			if(HAL_DMA_GetState(&hdma_usart1_tx)==HAL_DMA_STATE_READY)
			{			
				HAL_UART_Transmit_DMA(&huart1,Scope_buff,90);	
				index++;if(index>9) index = 0;			
			}
		}
	}
	if(receiveflag != 0)
	{
		receiveflag++;
		if((HAL_DMA_GetState(&hdma_usart1_tx)==HAL_DMA_STATE_READY))
		{
			if(receiveflag>5)
			{
				ReceiveDeal(receive_data);
				HAL_UART_Transmit_DMA(&huart1,transmit_buffer,9);
			}
			else if(receiveflag>10)
			{
				receiveflag = 0;	
			}
		}
	}
	iComm++;
	if(writeCD!=0)
	{
		if(writeCD++>100)
			writeCD = 0;
	}
	
	if(dmaGGflag == 1)
	{
		dmaGGflag = 0;
				HAL_UART_DMAStop(&huart1); //  停止DMA传输
			__HAL_UART_ENABLE_IT(&huart1, UART_IT_IDLE); 
			HAL_UART_Receive_DMA(&huart1, (uint8_t*)receive_buff, BUFFER_SIZE);
	}
}

void ReceiveDeal(uint8_t* rcvbuf)//使用自定义通信协议
{
	uint8_t index;
	int16_t value1;
	int16_t value2;
	if(!((rcvbuf[0] == 0x01&&data_length == 7)||(rcvbuf[0] == 0x02&&data_length == 7)||(rcvbuf[0] == 0x03&&data_length == 7))) //01整数16 02浮点数 03整数32
	{
		__nop();
		return;
	}
	index = rcvbuf[2];  //数据索引
/*********************0x03读**********************/
	if(rcvbuf[1] == 0x03) 
	{
		if(rcvbuf[0] == 0x01)//整数
		{
			value1 = (int16_t)*data_Arr16[index];
			value2 = 0x0000;
		}
		else if(rcvbuf[0] == 0x02)//浮点数
		{ 

			tmpUnion.f = *fdata_Arr[index];	
			value1 = tmpUnion.bytes[0]*256+tmpUnion.bytes[1];
			value2 = tmpUnion.bytes[2]*256+tmpUnion.bytes[3];		
		}
		else if(rcvbuf[0] == 0x03)
		{
			value1 = (int16_t)(*data_Arr32[index]>>16);
			value2 = (int16_t)*data_Arr32[index];
		}
	}
/*********************0x06写**********************/
	else if(rcvbuf[1] == 0x06) 
	{		
		if(rcvbuf[0] == 0x01)//16位整数
		{
			value1 = (int16_t)(rcvbuf[3]*256+rcvbuf[4]);
			value2 = 0x0000; 
			if(writeCD == 0)
				*data_Arr16[index] = value1;
		}
		else if(rcvbuf[0] == 0x02)//浮点数
		{ 
			tmpUnion.bytes[0] = rcvbuf[3];
			tmpUnion.bytes[1] = rcvbuf[4];
			tmpUnion.bytes[2] = rcvbuf[5];
			tmpUnion.bytes[3] = rcvbuf[6];
			if(writeCD == 0)
				*fdata_Arr[index] = tmpUnion.f;	
			value1 = (int16_t)(tmpUnion.bytes[0]*256+tmpUnion.bytes[1]);
			value2 = (int16_t)(tmpUnion.bytes[2]*256+tmpUnion.bytes[3]);
//			
		}
		else if(rcvbuf[0] == 0x03)  //32位整数
		{
			value1 = (rcvbuf[3]<<8)|rcvbuf[4];
			value2 = (rcvbuf[5]<<8)|rcvbuf[6];
			if(writeCD == 0)
				*data_Arr32[index] = (rcvbuf[3]<<24)|(rcvbuf[4]<<16)|(rcvbuf[5]<<8)|rcvbuf[6];
		}
		writeCD = 1;
	}
	transmit_buffer[0] = rcvbuf[0];
	transmit_buffer[1] = rcvbuf[1];
	transmit_buffer[2] = rcvbuf[2];
	transmit_buffer[3] = value1>>8;
	transmit_buffer[4] = value1;
	transmit_buffer[5] = value2>>8;
	transmit_buffer[6] = value2;		
	uint16_t tmp = 0;
	for(int i =0;i<7;i++)
	{
		tmp+=(uint16_t)transmit_buffer[i];
	}
//	transmit_buffer[7] =(uint8_t)((tmp>>8)&0x00ff);
//	transmit_buffer[8] =(uint8_t)(tmp&0x00ff);	
	//校验
	transmit_buffer[7] =0xFE;
	transmit_buffer[8] =0xEF;
}

void USER_IRQHandler(void)
{
	if(__HAL_UART_GET_FLAG(&huart1,UART_FLAG_IDLE)==SET)//判断是否是空闲中断
	{		
		__HAL_UART_CLEAR_IDLEFLAG(&huart1);     
				HAL_UART_DMAStop(&huart1); //  停止DMA传输
    // 计算接收到的数据长度
    data_length  = BUFFER_SIZE - __HAL_DMA_GET_COUNTER(&hdma_usart1_rx);  
		if(data_length == 1)//强制复位标志,避免意外导致通信卡死
		{
			dmaGGflag = 1;
		}
		if(data_length>=1)
		{
			if(receiveflag == 0)
				receiveflag = 1;
			for(int i = 0;i<data_length;i++)
			{
				receive_data[i] = receive_buff[i];
			}
		}		
    //清除空闲中断标志
		HAL_UART_Receive_DMA(&huart1,(uint8_t*)receive_buff,BUFFER_SIZE);//重新打开DMA接收
		__HAL_DMA_ENABLE(&hdma_usart1_rx);
		__HAL_UART_ENABLE_IT(&huart1, UART_IT_IDLE); 

	}
}




