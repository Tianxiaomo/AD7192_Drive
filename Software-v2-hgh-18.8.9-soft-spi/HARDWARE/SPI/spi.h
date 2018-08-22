#ifndef __SPI_H
#define __SPI_H
#include "sys.h"

				    
 								  
void SPI1_Init(void);			 //初始化SPI1口

u8 MSPI_Init(u8 lsbFirst,
		   u32 clockFreq,
		   u8 clockPol,
		   u8 clockEdg);

void SPI1_SetSpeed(u8 SpeedSet); //设置SPI1速度   
u8 SPI1_ReadWriteByte(u8 TxData);//SPI1总线读写一个字节

unsigned char SPI_Write(unsigned char* data,
                        unsigned char bytesNumber);
						
unsigned char SPI_Read(unsigned char* data,
						unsigned char bytesNumber);

#endif

