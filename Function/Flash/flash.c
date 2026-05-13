/*
作者：拾小白电控
日期：2024.1.20
qq群：1群：860465413  
备注：代码仅供学习参考，未经允许，禁止商用或售卖


功能：保存相关参数到Flash中，目前只对初始定位和控制状态进行了保存，这块代码也不要动
*/
#include "flash.h"
union u_tag {
    float f;
    uint32_t i;
};

uint32_t flashpage1data[16] = {0};
uint32_t flashpage1data2[16] = {0};

uint16_t ttest1,ttest2,ttest3,ttest4,ttest5;
	float bbb = 0.1;
void Test_Flash(void)
{
//  uint32_t add = 0x08010000;       
//	uint32_t error = 0;
//	uint64_t dat = 0x0123456776543210;
//	//uint64_t read_dat = 0 ;
//	FLASH_EraseInitTypeDef flash_dat;         
//	
//	HAL_FLASH_Unlock();                                                        
//	flash_dat.TypeErase = FLASH_TYPEERASE_PAGES;         
//	flash_dat.Page = (uint32_t)((add-0x08000000)/2048);          
//	flash_dat.NbPages = 1;                              
//	flash_dat.Banks=1;
//	HAL_FLASHEx_Erase(&flash_dat,&error);           
//	
//	HAL_FLASH_Program(FLASH_TYPEPROGRAM_DOUBLEWORD, add, dat);
//	HAL_FLASH_Lock();             	
	
	float aaa = 0.3;

	flashpage1data[0] = Float_HEX(aaa);
	flashpage1data[1] = 0x88888888;
	flashpage1data[2]  = 0x77777777;
	flashpage1data[3]  = 0x66666666;
	flashpage1data[4]  = 0x55555555;
	flashpage1data[5]  = 0x44444444;
	flashpage1data[6]  = 0x33333333;
	flashpage1data[7]  = 0x22222222;
	flashpage1data[8]  = 0x11111111;
	flashpage1data[9]  = 0x00000000;
	
	if(ttest1 == 3)
	{
		Flash_Page1_Save((uint32_t *)flashpage1data);
		ttest1 = 0;
		Flash_Page1_Read((uint32_t *)flashpage1data2);
		bbb = HEX_float(flashpage1data2[0]);
	}
	
}


//存储page1单字节32位数据page1大小2048
void Flash_Page1_Save(uint32_t * dataarr)
{
	uint32_t error = 0;
	uint32_t *pdata = dataarr;
	uint16_t length  = sizeof(pdata)*2;
	FLASH_EraseInitTypeDef flash_dat;        
	HAL_FLASH_Unlock();                                             
	flash_dat.TypeErase = FLASH_TYPEERASE_PAGES;         
	flash_dat.Page = (uint32_t)((DATAADDR1-0x08000000)/2048);           
	flash_dat.NbPages = 1;                             
	flash_dat.Banks=1;
	HAL_FLASHEx_Erase(&flash_dat,&error);    
	static uint64_t data64;
	for(int i = 0;i<length;i++)
	{
		data64 = (((uint64_t)pdata[2*i]<<32)&0xffffffff00000000)|(((uint64_t)pdata[2*i+1]&0x00000000ffffffff));
		HAL_FLASH_Program(FLASH_TYPEPROGRAM_DOUBLEWORD, DATAADDR1+i*8, data64);
	}	
	HAL_FLASH_Lock();          
}

//读取page1单字节32位数据page1大小2048
void Flash_Page1_Read(uint32_t * dataarr)
{
  uint64_t read_dat = 0 ;
	uint32_t add = DATAADDR1;  
	uint32_t *pdata = dataarr;
	uint16_t length  = sizeof(pdata)*2;	
	for(int i = 0;i<length;i++)
	{
		add = DATAADDR1+i*8;
		read_dat = *(__I uint64_t *)add;	   //读出flash中的数据
		pdata[2*i] = read_dat>>32;   //H
		pdata[2*i+1] = (read_dat&0x00000000FFFFFFFF);   //L
	}
}


void Flash_Save64(uint32_t add,uint32_t datah,uint32_t datal)
{
	uint32_t error = 0;
	uint64_t dat = (uint64_t)((((uint64_t)datah&0xffffffff)<<32)|datal);
	//uint64_t read_dat = 0 ;
	FLASH_EraseInitTypeDef flash_dat;        
	
	HAL_FLASH_Unlock();                                             
	flash_dat.TypeErase = FLASH_TYPEERASE_PAGES;         
	flash_dat.Page = (uint32_t)((add-0x08000000)/2048);           
	flash_dat.NbPages = 1;                             
	flash_dat.Banks=1;
	HAL_FLASHEx_Erase(&flash_dat,&error);           
	
	HAL_FLASH_Program(FLASH_TYPEPROGRAM_DOUBLEWORD, add, dat);
	HAL_FLASH_Lock();                                     
}



//32高位
uint32_t Flash_ReadH(uint32_t add)
{
  uint64_t read_dat = 0 ;
	//uint32_t add = 0x08010000;  
	read_dat = *(__I uint64_t *)add;	   //读出flash中的数据
	uint32_t read_dat1=read_dat>>32;
	uint32_t read_dat2=read_dat1&0x00000000FFFFFFFF;
	
	return read_dat2;
}
//32低位
uint32_t Flash_ReadL(uint32_t add)
{
  uint64_t read_dat = 0 ;
	//uint32_t add = 0x08010000;  
	read_dat = *(__I uint64_t *)add;	   //读出flash中的数据
	//uint32_t read_dat1=read_dat>>32;
	uint32_t read_dat2=read_dat&0x00000000FFFFFFFF;
	
	return read_dat2;
}


uint32_t Float_HEX (float fdata)
{
	union u_tag u_tag1;
	u_tag1.f = fdata;
	return u_tag1.i;
	
}

float HEX_float(uint32_t data)
{
	union u_tag u_tag1;
	u_tag1.i = data;
	return u_tag1.f;
}


