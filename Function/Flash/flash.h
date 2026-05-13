#include "main.h"



#define DATAADDR1 (0x08010000+8*0)
//#define DATAADDR2 (0x08010000+8*1)
//#define DATAADDR3 (0x08010000+8*2)
//#define DATAADDR4 (0x08010000+8*3)
//#define DATAADDR5 (0x08010000+8*4)

#define FLASH_ZEROFLAG 0   //零点保存标志位
#define FLASH_POSINIT 1    //零位对齐位置
#define FLASH_MOTORDIR 2    //编码器方向
#define FLASH_OFFERIU 3 	//u相电流校正
#define FLASH_OFFERIV 4 	//V相电流校正
#define FLASH_OFFERIW 5 	//W相电流校正

//保存初始位置
void Flash_Save64(uint32_t add,uint32_t datah,uint32_t datal);
uint32_t Flash_ReadH(uint32_t add);
uint32_t Flash_ReadL(uint32_t add);
void Flash_Page1_Save(uint32_t * dataarr);
void Flash_Page1_Read(uint32_t * dataarr);
void Test_Flash(void);

uint32_t Float_HEX (float fdata);
float HEX_float(uint32_t data);
