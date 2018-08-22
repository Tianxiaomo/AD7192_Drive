/********************************************************************************
 Author : CAC (China Applications Support Team) 

 Date :   January, 2012

 File name :   ADuC7026Driver.c

 Description :	 Use the GPIO to simulate the SPI communication of AD7192

 Hardware plateform : 	ADuC7026 and AD7190/92EBZ
********************************************************************************/

#include "ADuC7026Driver.h"
#include "stdio.h"
#include "sys.h"
#include "delay.h"

#include "led.h"

void MSPI1_Init(){
	GPIO_InitTypeDef  GPIO_InitStructure;

	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB, ENABLE);//使能GPIOF时钟

	//GPIOF9,F10初始化设置
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_3|GPIO_Pin_5 | GPIO_Pin_6 | GPIO_Pin_7;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;//普通输出模式
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;//推挽输出
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;//100MHz
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;//上拉
	GPIO_Init(GPIOB, &GPIO_InitStructure);//初始化

	GPIO_SetBits(GPIOB,GPIO_Pin_3|GPIO_Pin_5 | GPIO_Pin_6 | GPIO_Pin_7);//GPIOF9,F10设置高，灯灭
	
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_4; //KEY0 KEY1 KEY2对应引脚
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN;//普通输入模式
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
	GPIO_Init(GPIOB, &GPIO_InitStructure);
	
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOG, ENABLE);//使能GPIOF时钟
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_15; //KEY0 KEY1 KEY2对应引脚
	GPIO_Init(GPIOG, &GPIO_InitStructure);

}


/* GPIO Control */
unsigned char ADuC7026InputBit(unsigned char GPIONum)
{
	return PBin(GPIONum);
}

void ADuC7026OutputBit(unsigned char GPIONum, unsigned char Data)
{
	PBout(GPIONum) = Data;
}

void ADuC7026SpiOperation(unsigned char* WriteBuffer, unsigned char *ReadBuffer, unsigned char NumberOfByte)
{
	unsigned	char	WriteData, ReadData;
  	unsigned	char	i, j;

	ADuC7026OutputBit(SCLK,1);	
	for(i=0; i<NumberOfByte; i++)
 	{
	 	WriteData = *(WriteBuffer + i);
		ReadData = 0;

		for(j=0; j<8; j++)
		{
			ADuC7026OutputBit(SCLK,0);
			if(0x80 == (WriteData & 0x80))
			{
				ADuC7026OutputBit(SDI,1);	  //Send one to SDI pin
			}
			else
			{
				ADuC7026OutputBit(SDI,0);	  //Send zero to SDI pin
			}
			WriteData = WriteData << 1;
			ReadData = (ReadData<<1) | SPI_IN; 
			delay_us(1);
			ADuC7026OutputBit(SCLK,1);	
			delay_us(1);

		}
		*(ReadBuffer + i)= ReadData;
//		printf("0x%x\r\n",ReadData);
//		delay_us(5);
	}  
	ADuC7026OutputBit(SCLK,1);
}
/*    Rewrite the serial port function  */
