#include "sys.h"
#include "delay.h"
#include "usart.h"
#include "led.h"
#include "lcd.h"
#include "key.h"  
#include "key.h"  
#include "sram.h"   
#include "malloc.h" 
#include "sdio_sdcard.h"    
#include "malloc.h" 
#include "ff.h"  
#include "exfuns.h"   
#include "fattester.h"
#include "string.h"
#include "ANO_DT.h"
#include "AD7192.h"
#include "ADuC7026Driver.h"

void test(){
		MSPI1_Init();

	while(1){
		ADuC7026OutputBit(SDI,1);
//		PBout(5) = 1;
		delay_us(1);
		ADuC7026OutputBit(SDI,0);
//		PBout(5) = 0;
		delay_us(1);
	}
}

int main(void)
{ 
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);//设置系统中断优先级分组2
	delay_init(168);    							//初始化延时函数
	uart_init(115200);								//初始化串口波特率为115200
	LED_Init();										//初始化LED 
	KEY_Init(); 									//按键初始化  
	LED0 = 1;
	LED1 = 0;
	printf("Initialization AD7192\r\n");
//#if DEBUG
//	MSPI1_Init();
//	while(1){
//		delay_s(1);
//		printf("in = %d",GPIO_ReadInputDataBit(GPIOB,GPIO_Pin_4));
//	}
//#endif

//#if DEBUG
//	test();
//#endif
	AD7192Initialization();
	printf("Initialization AD7192 Succeed!!!\r\n");
	float temp = AD7192ReadTemperature();
	printf("AD7192 measure temperature %f \r\n", temp);
	AD7192StartContinuousConvertion(AIN1_COM);
	while(1){
		LED0 = 1;
		LED1 = 0;
//		temp = GetSingleConvertionValue(AIN1_COM);
		temp = AD7192ReadConvertingData();
		printf("%f\r\n",temp);
		LED0 = 0;
		LED1 = 1;
//		delay_ms(10);
	}
}


