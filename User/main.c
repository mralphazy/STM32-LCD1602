/************************************************************************************
**	  红龙开发板（V1.0）
**	  LED流水灯实验
**
**	  论    坛：bbs.openmcu.com
**	  旺    宝：www.openmcu.com
**	  邮    箱：support@openmcu.com
**
**    版    本：V1.0
**	  作    者：FXL
**	  完成日期:	2012.7.20
------------------------------------------------------------------------------------
**	程序测试说明: #define TEST_1  TEST_1 LED灯测试
**				  #define TEST_2  TEST_2 按键测试
**	LED 测试说明：注释掉 TEST_2 按键测试   //#define TEST_2  
**				  初始化系统时钟
**				  LED灯的IO口初始化
**
**				  JLINK下载运行后，三个LED灯轮番闪烁。
**
**	按键测试说明：注释掉 TEST_1 LED灯测试   //#define TEST_1
**				  初始化系统时钟
**				  LED灯的IO口初始化
**				  按键IO口初始化
**
**				  JLINK下载运行后，按下按按键USER1 LED1灯亮；松开USER1 LED1灯灭。
**								   按下按按键USER2 LED2依次取反。
**                                 按下按按键WAKE UP  打开和关闭LED3由亮渐灭控制。
************************************************************************************/


#include "stm32f10x.h"
#include <stdio.h>
#include "SystemClock.h"
#include "Gpio_Led.h"


#define TEST_1	     	 //TEST_1 LED灯测试
//#define TEST_2         //TEST_2 按键测试
u8 AAA;

/*****主函数*****/
int main(void)
{
	RCC_Configuration(); //系统时钟初始化
	GpioLed_Init();		 //LED灯初始化

#ifdef TEST_2
	Key_Init();
#endif

	while (1)
	{
#ifdef TEST_1
	LED_Display();	 //LED闪烁
	GPIO_Write(GPIOF,0xffff);
	GPIO_Write(GPIOF,0x00);
	WriteData_8(0xaa);
	AAA = GPIO_ReadInputData(GPIOF);

#endif

#ifdef TEST_2
	  Key_Test();		 //按键测试
#endif
	}
}

