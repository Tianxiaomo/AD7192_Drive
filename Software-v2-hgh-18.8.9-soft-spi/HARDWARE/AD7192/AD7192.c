/********************************************************************************
 Author : CAC (China Applications Support Team) 

 Date :   January, 2012

 File name :   AD7192.c

 Description :	 Use the GPIO to simulate the SPI communication of AD7192

 Hardware plateform : 	ADuC7026 and AD7190/92EBZ
********************************************************************************/

#include "ADuC7026Driver.h"
#include "AD7192.h"
#include "stdio.h"
#include "sys.h"
#include "delay.h"
#include "spi.h"

unsigned long int AD7192Registers[8]={0,0,0,0,0,0,0,0};		
unsigned long int AD7192Data = 0;

void AD7192Initialization()
{
	MSPI1_Init();
	AD7192SoftwareReset();
	delay_us(50);
#if DEBUG
	u8 address=REG_FS;
	unsigned long int data=0;
	ReadFromAD7192ViaSPI(address,1,&data,0);
	printf("id = 0x%x\r\n",data);
	
	address=REG_ID;
	ReadFromAD7192ViaSPI(address,1,&data,0);
	printf("id = 0x%x\r\n",data);
	
	address=REG_CONF;
	ReadFromAD7192ViaSPI(address,1,&data,0);
	printf("id = 0x%x\r\n",data);
#endif
	AD7192InternalZeroScaleCalibration();
	AD7192InternalFullScaleCalibration();
#if DEBUG
	ReadFromAD7192ViaSPI(address,1,&data,0);
	printf("id = 0x%x\r\n",data);
#endif
}




void AD7192SoftwareReset()
{
	
	unsigned char WriteBuf[1];
	unsigned char ReadBuf[1];
	unsigned char i;

	ADuC7026OutputBit(CS,0);
	WriteBuf[0]	= 0xFF;	 

	for(i=0; i<5; i++)
	{
		ADuC7026SpiOperation(WriteBuf, ReadBuf, 1);
	}
	ADuC7026OutputBit(CS,1);
}

unsigned char WriteToAD7192ViaSPI(const unsigned char RegisterStartAddress, const unsigned char NumberOfRegistersToWrite, unsigned long int *DataBuffer, const unsigned char OffsetInBuffer)
{
	unsigned char WriteBuf[4];
	unsigned char ReadBuf[4];
	unsigned char i;

	ADuC7026OutputBit(CS,0);

	for(i=0; i<NumberOfRegistersToWrite; i++)
	{
		WriteBuf[0]	= WEN|RW_W|((RegisterStartAddress + i)<<3)|CREAD_DIS;
		WriteBuf[1] = DataBuffer[RegisterStartAddress + i]>>16;
		WriteBuf[2] = DataBuffer[RegisterStartAddress + i]>>8;
		WriteBuf[3]	= DataBuffer[RegisterStartAddress + i];
		ADuC7026SpiOperation(WriteBuf, ReadBuf, 4);
	}
	ADuC7026OutputBit(CS,1);
	return 0;
}

unsigned char ReadFromAD7192ViaSPI(const unsigned char RegisterStartAddress, const unsigned char NumberOfRegistersToRead, unsigned long int *DataBuffer, const unsigned char OffsetInBuffer)
{
	unsigned char WriteBuf[4];
	unsigned char ReadBuf[4] = {0};
	unsigned char i;

	ADuC7026OutputBit(CS,0);
	for(i=0; i < NumberOfRegistersToRead; i++)
	{

		WriteBuf[0] = WEN|RW_R|((RegisterStartAddress + i)<<3)|CREAD_DIS;	 

		ADuC7026SpiOperation(WriteBuf, ReadBuf, 1);

		WriteBuf[0] = NOP;
		WriteBuf[1]	= NOP;
		WriteBuf[2]	= NOP;
		WriteBuf[3]	= NOP;

		switch(RegisterStartAddress + i){

			case REG_ID		 :
				ADuC7026SpiOperation(WriteBuf, ReadBuf, 1);
				DataBuffer[OffsetInBuffer + i ] = ReadBuf[0];
				break;
			case REG_COM_STA : 
			case REG_GPOCON  :
				ADuC7026SpiOperation(WriteBuf, ReadBuf, 1);
				DataBuffer[OffsetInBuffer + i ] = ReadBuf[0];
				break;
				
			case REG_MODE    : 
			case REG_CONF	 : 
				ADuC7026SpiOperation(WriteBuf, ReadBuf, 3);	   
				DataBuffer[OffsetInBuffer + i ] = ReadBuf[0];
				DataBuffer[OffsetInBuffer + i ] = (DataBuffer[OffsetInBuffer + i ]<<8) + ReadBuf[1];
				DataBuffer[OffsetInBuffer + i ] = (DataBuffer[OffsetInBuffer + i ]<<8) + ReadBuf[2];  
				break;
			case REG_OFFSET  :
			case REG_FS		 : 
				ADuC7026SpiOperation(WriteBuf, ReadBuf, 3);	   
				DataBuffer[OffsetInBuffer + i ] = ReadBuf[0];
				DataBuffer[OffsetInBuffer + i ] = (DataBuffer[OffsetInBuffer + i ]<<8) + ReadBuf[1];
				DataBuffer[OffsetInBuffer + i ] = (DataBuffer[OffsetInBuffer + i ]<<8) + ReadBuf[2];  
				break;
				
			case REG_DATA	 : 
				if (AD7192Registers[REG_MODE] & DAT_STA_EN)	{

					ADuC7026SpiOperation(WriteBuf, ReadBuf, 4);	  
					DataBuffer[OffsetInBuffer + i ] = ReadBuf[0];
					DataBuffer[OffsetInBuffer + i ] = (DataBuffer[OffsetInBuffer + i ]<<8) + ReadBuf[1];
					DataBuffer[OffsetInBuffer + i ] = (DataBuffer[OffsetInBuffer + i ]<<8) + ReadBuf[2];					
					DataBuffer[OffsetInBuffer + i ] = (DataBuffer[OffsetInBuffer + i ]<<8) + ReadBuf[3];
					break;
					}
				else {

					ADuC7026SpiOperation(WriteBuf, ReadBuf, 3);	   //do not transfer the status contents after read data register 
					DataBuffer[OffsetInBuffer + i ] = ReadBuf[0];
					DataBuffer[OffsetInBuffer + i ] = (DataBuffer[OffsetInBuffer + i ]<<8) + ReadBuf[1];
					DataBuffer[OffsetInBuffer + i ] = (DataBuffer[OffsetInBuffer + i ]<<8) + ReadBuf[2];
					break;
					}
	
			default			 : 
					break;
															
		}
	}
	ADuC7026OutputBit(CS,1);
	return 0;
}

void AD7192StartContinuousRead()
{

	unsigned char WriteBuf[1];
	unsigned char ReadBuf[1];

	ADuC7026OutputBit(CS,0);

	WriteBuf[0] = WEN|RW_R|(REG_DATA<<3)|CREAD_EN;

	ADuC7026SpiOperation(WriteBuf, ReadBuf, 1);	

}

unsigned long int AD7192ContinuousRead()
{
	unsigned char WriteBuf[4];
	unsigned char ReadBuf[4];
	unsigned long int DataBuffer;

	WriteBuf[0] = NOP;
	WriteBuf[1] = NOP;	
	WriteBuf[2] = NOP;
	WriteBuf[3]	= NOP;

	ADuC7026OutputBit(CS,0);
	while(ADuC7026InputBit(SDO)== 0){;}			
	while(ADuC7026InputBit(SDO)== 1){;}			//	waiting the 1st RDY failling edge;

	if ((AD7192Registers[REG_MODE] & DAT_STA_EN) == DAT_STA_EN)	{

		ADuC7026SpiOperation(WriteBuf, ReadBuf, 4);	  
		DataBuffer = ReadBuf[0];
		DataBuffer = (DataBuffer<<8) + ReadBuf[1];
		DataBuffer = (DataBuffer<<8) + ReadBuf[2];
		DataBuffer = (DataBuffer<<8) + ReadBuf[3];
		}
	else {

		ADuC7026SpiOperation(WriteBuf, ReadBuf, 3);	   //do not transfer the status contents after read data register 
		DataBuffer = ReadBuf[0];
		DataBuffer = (DataBuffer<<8) + ReadBuf[1];
		DataBuffer = (DataBuffer<<8) + ReadBuf[2];
		}

	return DataBuffer;
}


/*单次转换数据获取*/
u32 GetSingleConvertionValue(u32 Channels)
{
  uint32_t dataCode=0;
  AD7192StartSingleConvertion(Channels);
  
  for(int i=0;i<72000;i++)
  {
    ;
  }
  
  dataCode = AD7192ReadConvertingData();
  dataCode =dataCode & 0x00FFFFFF;

  ReadFromAD7192ViaSPI(REG_DATA,1,AD7192Registers,1);
  
  return dataCode;
}

unsigned long int AD7192ReadConvertingData()
{
	unsigned char WriteBuf[4];
	unsigned char ReadBuf[4];
	unsigned long int DataBuffer;

	ADuC7026OutputBit(CS,0);
	WriteBuf[0] = WEN|RW_R|((REG_DATA)<<3)|CREAD_DIS;	 

	ADuC7026SpiOperation(WriteBuf, ReadBuf, 1);

	WriteBuf[0] = NOP;
	WriteBuf[1] = NOP;	
	WriteBuf[2] = NOP;
	WriteBuf[3]	= NOP;

	while(SPI_IN== 0){;}			
	while(SPI_IN== 1){;}			//	waiting the 1st RDY failling edge;

	if ((AD7192Registers[REG_MODE] & DAT_STA_EN) == DAT_STA_EN)	{

		ADuC7026SpiOperation(WriteBuf, ReadBuf, 4);	  
		DataBuffer = ReadBuf[0];
		DataBuffer = (DataBuffer<<8) + ReadBuf[1];
		DataBuffer = (DataBuffer<<8) + ReadBuf[2];
		DataBuffer = (DataBuffer<<8) + ReadBuf[3];
		}
	else {

		ADuC7026SpiOperation(WriteBuf, ReadBuf, 3);	   //do not transfer the status contents after read data register 
		DataBuffer = ReadBuf[0];
		DataBuffer = (DataBuffer<<8) + ReadBuf[1];
		DataBuffer = (DataBuffer<<8) + ReadBuf[2];
		}
//	ADuC7026OutputBit(CS,1);
	return DataBuffer;
}


void AD7192ExitContinuousRead()
{

	unsigned char WriteBuf[1];
	unsigned char ReadBuf[1];

	while(ADuC7026InputBit(SDO)== 0){;}			
	while(ADuC7026InputBit(SDO)== 1){;}			//	waiting the 1st RDY failling edge;

	WriteBuf[0]	= WEN|RW_R|(REG_DATA<<3)|CREAD_DIS;

	ADuC7026SpiOperation(WriteBuf, ReadBuf, 1);		

	ADuC7026OutputBit(CS,1);
}

void AD7192InternalZeroScaleCalibration()
{	
	AD7192Registers[REG_MODE] = 0;
	AD7192Registers[REG_CONF] = 0;
	//斩波禁用，在 REFIN1(+) 与 REFIN1( ? ) 之间施加的外部基准电压。
	AD7192Registers[REG_CONF] = CHOP_DIS|REF_IN1|AIN1_COM|AIN2_COM|BURN_DIS|REFDET_DIS|BUF_DIS|UB_UNI|GAIN_1;	// Gain = 1
	WriteToAD7192ViaSPI(REG_CONF, 1, AD7192Registers, REG_CONF);

	AD7192Registers[REG_MODE] = MODE_INZCL|DAT_STA_DIS|INCLK_MCLK2EN|SINC_4|ENPAR_EN|CLK_DIV_DIS|SINGLECYCLE_DIS|REJ60_DIS|0x080;		
	WriteToAD7192ViaSPI(REG_MODE, 1, AD7192Registers, REG_MODE);
	ADuC7026OutputBit(CS,0);

	while(SPI_IN== 1){;}			//	wait until RDY = 0;
	ADuC7026OutputBit(CS,1);
}

void AD7192InternalFullScaleCalibration()
{
	AD7192Registers[REG_MODE] = 0;
	AD7192Registers[REG_CONF] = 0;
	AD7192Registers[REG_CONF] = CHOP_DIS|REF_IN1|AIN1_COM|AIN2_COM|BURN_DIS|REFDET_DIS|BUF_DIS|UB_UNI|GAIN_1;	// Gain = 1
	WriteToAD7192ViaSPI(REG_CONF, 1, AD7192Registers, REG_CONF);

	AD7192Registers[REG_MODE] = 0;
	AD7192Registers[REG_MODE] = MODE_INFCL|DAT_STA_DIS|INCLK_MCLK2EN|SINC_4|ENPAR_EN|CLK_DIV_2|SINGLECYCLE_DIS|REJ60_DIS|0x080;		
	WriteToAD7192ViaSPI(REG_MODE, 1, AD7192Registers, REG_MODE);

	ADuC7026OutputBit(CS,0);
	while(SPI_IN== 1){;}			//	wait until RDY = 0;
	ADuC7026OutputBit(CS,1);

}

void AD7192ExternalZeroScaleCalibration()
{
	AD7192Registers[REG_MODE] = 0;
	AD7192Registers[REG_CONF] = 0;
	AD7192Registers[REG_CONF] = CHOP_DIS|REF_IN1|AIN1_AIN2|BURN_DIS|REFDET_DIS|BUF_DIS|UB_UNI|GAIN_1;	// Gain = 1
	WriteToAD7192ViaSPI(REG_CONF, 1, AD7192Registers, REG_CONF);

//	The user should connect the system zero-scale input to the channel input pins as selected by the CH7 to CH0 bits in the configuration register	
	AD7192Registers[REG_MODE] = 0;
	AD7192Registers[REG_MODE] = MODE_INZCL|DAT_STA_DIS|INCLK_MCLK2EN|SINC_4|ENPAR_EN|CLK_DIV_DIS|SINGLECYCLE_DIS|REJ60_DIS|0x080;		
	WriteToAD7192ViaSPI(REG_MODE, 1, AD7192Registers, REG_MODE);

	ADuC7026OutputBit(CS,0);
	while(ADuC7026InputBit(SDO)== 1){;}			//	wait until RDY = 0;
	ADuC7026OutputBit(CS,1);

}


void AD7192ExternalFullScaleCalibration()
{
	AD7192Registers[REG_MODE] = 0;
	AD7192Registers[REG_CONF] = 0;
	AD7192Registers[REG_CONF] = CHOP_DIS|REF_IN1|AIN1_AIN2|BURN_DIS|REFDET_DIS|BUF_DIS|UB_UNI|GAIN_1;	// Gain = 1
	WriteToAD7192ViaSPI(REG_CONF, 1, AD7192Registers, REG_CONF);

//	The user should connect the system full-scale input to the channel input pins as selected by the CH7 to CH0 bits in the configuration register
	AD7192Registers[REG_MODE] = 0;
	AD7192Registers[REG_MODE] = MODE_INFCL|DAT_STA_DIS|INCLK_MCLK2EN|SINC_4|ENPAR_EN|CLK_DIV_2|SINGLECYCLE_DIS|REJ60_DIS|0x080;		
	WriteToAD7192ViaSPI(REG_MODE, 1, AD7192Registers, REG_MODE);

	ADuC7026OutputBit(CS,0);
	while(ADuC7026InputBit(SDO)== 1){;}			//	wait until RDY = 0;
	ADuC7026OutputBit(CS,1);

}

void AD7192StartSingleConvertion(unsigned long int Channels)
{
ADuC7026OutputBit(CS,0);

	AD7192Registers[REG_MODE] = 0;
	AD7192Registers[REG_CONF] = 0;
	AD7192Registers[REG_CONF] = CHOP_DIS|REF_IN1|Channels|BURN_DIS|REFDET_DIS|BUF_DIS|UB_UNI|GAIN_1;
	WriteToAD7192ViaSPI(REG_CONF, 1, AD7192Registers, REG_CONF);
	AD7192Registers[REG_MODE] = MODE_SING|DAT_STA_DIS|INCLK_MCLK2EN|SINC_4|ENPAR_EN|CLK_DIV_DIS|SINGLECYCLE_DIS|REJ60_DIS|0x080;
	WriteToAD7192ViaSPI(REG_MODE, 1, AD7192Registers, REG_MODE);

}

void AD7192StartContinuousConvertion(unsigned long int Channels)
{

	AD7192Registers[REG_MODE] = 0;
	AD7192Registers[REG_CONF] = 0;
	AD7192Registers[REG_MODE] = MODE_CONT|DAT_STA_EN|INCLK_MCLK2EN|SINC_4|ENPAR_EN|CLK_DIV_DIS|SINGLECYCLE_DIS|REJ60_DIS|0x080;		//Output Rate =	MCLK/1024/128 without chop
	AD7192Registers[REG_CONF] = CHOP_DIS|REF_IN1|Channels|BURN_DIS|REFDET_DIS|BUF_DIS|UB_UNI|GAIN_1;	// Gain = 1
	WriteToAD7192ViaSPI(REG_MODE, 2, AD7192Registers, REG_MODE);

}


float AD7192ReadTemperature()
{
ADuC7026OutputBit(CS,0);
	AD7192Registers[REG_COM_STA] = 0;
	AD7192Registers[REG_MODE] = MODE_SING|DAT_STA_DIS|INCLK_MCLK2EN|SINC_4|ENPAR_EN|CLK_DIV_DIS|SINGLECYCLE_DIS|REJ60_DIS|0x080;
//	AD7192Registers[REG_MODE] = MODE_CONT|DAT_STA_DIS|INCLK_MCLK2TRI|SINC_4|ENPAR_DIS|CLK_DIV_DIS|SINGLECYCLE_DIS|REJ60_DIS|0x060;
	WriteToAD7192ViaSPI(REG_MODE, 1, AD7192Registers, REG_MODE);
	AD7192Registers[REG_CONF] = CHOP_DIS|REF_IN1|TEMP|BURN_DIS|REFDET_DIS|BUF_DIS|UB_BI|GAIN_1;
	WriteToAD7192ViaSPI(REG_CONF, 1, AD7192Registers, REG_CONF);

	AD7192Data = AD7192ReadConvertingData();	

	float temp = 0.0;
	temp = (AD7192Data-0x800000)/2815.0-273;
	return temp;
}





