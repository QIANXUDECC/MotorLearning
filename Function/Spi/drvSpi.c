/*
作者：拾小白电控
日期：2024.1.20
qq群：1群：860465413  
备注：代码仅供学习参考，未经允许，禁止商用或售卖


功能：SPI读编码器,勿动
*/

#include "drvSpi.h"
#include "spi.h"
//计算奇偶函数
uint16_t Parity_bit_Calculate(uint16_t data_2_cal)
{
	uint16_t parity_bit_value=0;
	while(data_2_cal != 0)
	{
		parity_bit_value ^= data_2_cal; 
		data_2_cal >>=1;
	}
	return (parity_bit_value & 0x1); 
}
//SPI发送读取函数
uint16_t SPI_ReadWrite_OneByte(uint16_t _txdata)
{

	AS5407P_CS_L;	//cs拉低
	uint16_t rxdata;
	if(HAL_SPI_TransmitReceive(&hspi1,(uint8_t *)&_txdata,(uint8_t *)&rxdata,1,1000) !=HAL_OK)
		rxdata=0;
  AS5407P_CS_H;		//cs拉高
	return rxdata;
}
uint16_t AS5047_read(uint16_t add)
{
	uint16_t data;
	add |= 0x4000;	//读指令 bit14 置1
	if(Parity_bit_Calculate(add)==1) add=add|0x8000; //如果前15位 1的个数位偶数，则Bit15 置1
	SPI_ReadWrite_OneByte(add);		//发送一条指令，不管读回的数据
		data=SPI_ReadWrite_OneByte(0X0000|0x4000); //发送一条空指令，读取上一次指令返回的数据。
	data &=0x3fff;
	return data;
}
