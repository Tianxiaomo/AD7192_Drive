/********************************************************************************
 Author : CAC (China Applications Support Team) 

 Date :   January, 2012

 File name :   ADuC7026Driver.h

 Description :	 Use the GPIO to simulate the SPI communication of AD7192

 Hardware plateform : 	ADuC7026 and AD7190/92EBZ
********************************************************************************/

#ifndef ADUC7026_DRIVER_H
#define ADUC7026_DRIVER_H

// add the header file here
//GPIO Define

#define CS			6	// pin CS = PB6
#define SYNC		7	// pin SYNC  = PB7
#define SCLK		3	// pin SCLK  = PB3
#define	SDI			5	// pin SDI   = PB4
#define	SDO			4	// pin SDO   = PB5

#define SPI_IN 		GPIO_ReadInputDataBit(GPIOB,GPIO_Pin_4) //PD3

void MSPI1_Init(void);
unsigned char ADuC7026InputBit(unsigned char GPIONum);
void ADuC7026OutputBit(unsigned char GPIONum, unsigned char Data);	

void ADuC7026SpiOperation(unsigned char* WriteBuffer, unsigned char *ReadBuffer, unsigned char NumberOfByte);

#endif





