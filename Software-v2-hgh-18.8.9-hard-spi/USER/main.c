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
	AD7192Initialization();
	printf("Initialization AD7192 Succeed!!!\r\n");
	float temp = AD7192ReadTemperature();
	printf("AD7192 measure temperature %f \r\n", temp);
	AD7192StartContinuousConvertion(AIN1_COM);
	while(1){
		LED0 = 1;
//		LED1 = 0;
//				delay_ms(10);
//		temp = GetSingleConvertionValue(AIN1_COM);
		temp = AD7192ReadConvertingData();
//		printf("%f\r\n",temp);
		LED0 = 0;
//		LED1 = 1;
		temp = AD7192ReadConvertingData();
//		delay_ms(10);
	}
}


