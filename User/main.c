/************************************************************************************
**	  ���������壨V1.0��
**	  LED��ˮ��ʵ��
**
**	  ��    ̳��bbs.openmcu.com
**	  ��    ����www.openmcu.com
**	  ��    �䣺support@openmcu.com
**
**    ��    ����V1.0
**	  ��    �ߣ�FXL
**	  �������:	2012.7.20
------------------------------------------------------------------------------------
**	�������˵��: #define TEST_1  TEST_1 LED�Ʋ���
**				  #define TEST_2  TEST_2 ��������
**	LED ����˵����ע�͵� TEST_2 ��������   //#define TEST_2  
**				  ��ʼ��ϵͳʱ��
**				  LED�Ƶ�IO�ڳ�ʼ��
**
**				  JLINK�������к�����LED���ַ���˸��
**
**	��������˵����ע�͵� TEST_1 LED�Ʋ���   //#define TEST_1
**				  ��ʼ��ϵͳʱ��
**				  LED�Ƶ�IO�ڳ�ʼ��
**				  ����IO�ڳ�ʼ��
**
**				  JLINK�������к󣬰��°�����USER1 LED1�������ɿ�USER1 LED1����
**								   ���°�����USER2 LED2����ȡ����
**                                 ���°�����WAKE UP  �򿪺͹ر�LED3����������ơ�
************************************************************************************/


#include "stm32f10x.h"
#include <stdio.h>
#include "SystemClock.h"
#include "Gpio_Led.h"


#define TEST_1	     	 //TEST_1 LED�Ʋ���
//#define TEST_2         //TEST_2 ��������
u8 AAA;

/*****������*****/
int main(void)
{
	RCC_Configuration(); //ϵͳʱ�ӳ�ʼ��
	GpioLed_Init();		 //LED�Ƴ�ʼ��

#ifdef TEST_2
	Key_Init();
#endif

	while (1)
	{
#ifdef TEST_1
	LED_Display();	 //LED��˸
	GPIO_Write(GPIOF,0xffff);
	GPIO_Write(GPIOF,0x00);
	WriteData_8(0xaa);
	AAA = GPIO_ReadInputData(GPIOF);

#endif

#ifdef TEST_2
	  Key_Test();		 //��������
#endif
	}
}

