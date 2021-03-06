/******************************************************************
**	  红龙开发板（V1.0）
**	  Gpio配置文件
**
**	  论    坛：bbs.openmcu.com
**	  旺    宝：www.openmcu.com
**	  邮    箱：support@openmcu.com
**
**    版    本：V1.0
**	  作    者：FXL
**	  完成日期:	2012.7.20
********************************************************************/
#include "stm32f10x.h"
#include "Gpio_Led.h"

Key_Info key_info;

/********************************************************************************************
*函数名称：void GpioLed_Init(void)
*
*入口参数：无
*
*出口参数：无
*
*功能说明：led灯初始化配置
*******************************************************************************************/
void GpioLed_Init(void)
{
	u16 i,PinNum;
	GPIO_InitTypeDef GPIO_InitStructure;	//结构声明
	RCC_APB2PeriphClockCmd(Data_RCC_APB2Periph , ENABLE);// 使能APB2外设LED1时钟
	for (i=0;i<11;i++)
	{
		PinNum = 1;
		PinNum = PinNum<<i;
		GPIO_InitStructure.GPIO_Pin	= PinNum; 
		GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;	 //管脚频率为50MHZ
		GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;	 //模式为推挽输出
		GPIO_Init(IO_LCD, &GPIO_InitStructure);           //初始化led1寄存器
	}
}

/********************************************************************************************
*函数名称：void WriteData_8(u8 Data_8)
*
*入口参数：无
*
*出口参数：无
*
*功能说明：写8位数据。用PinNum变量作为二进制掩码，来判断引脚某一位的值。单片机引脚号与PinNum相同
*******************************************************************************************/
void WriteData_8(u8 Data_8)
{
	u8 PinNum =1,i;
	for (i = 0;i<8;i++ )
	{
		if ((PinNum&Data_8)==PinNum)
		{
			GPIO_WriteBit(GPIOF,PinNum,Bit_SET);
		}
		else if ((PinNum&Data_8) == ~PinNum)
		{
			GPIO_WriteBit(GPIOF,PinNum,Bit_RESET);
		}
		PinNum = PinNum<<1;
	}
}

/********************************************************************************************
*函数名称：void ReadData_8(u8 Data_8)
*
*入口参数：无
*
*出口参数：无
*
*功能说明：写8位数据。用PinNum变量作为二进制掩码，来判断引脚某一位的值。单片机引脚号与PinNum相同
*******************************************************************************************/
u8 ReadData_8(void)
{
	//u8 PinNum =1,i;
	u16	PinVal;
	u8 DataVal;
	//WriteData_8(0xff);
	PinVal = GPIO_ReadInputData(GPIOF);
	DataVal = PinVal;
	return DataVal;
}

/********************************************************************************************
*函数名称：static void Delay(u32 counter)
*
*入口参数：u32 counter：计数个数
*
*出口参数：无
*
*功能说明：延时函数
*******************************************************************************************/
static void Delay(u32 counter)
{
	while(counter--);
}

/********************************************************************************************
*函数名称：void LED_Disply(void)
*
*入口参数：无
*
*出口参数：无
*
*功能说明：LED闪烁
*******************************************************************************************/
void LED_Display(void)
{
	GPIO_SetBits(IO_LCD,RS);
	Delay(0xfffff);
	GPIO_ResetBits(IO_LCD,RS);

	GPIO_SetBits(IO_LCD,RW);
	Delay(0xfffff);
	GPIO_ResetBits(IO_LCD,RW);

	GPIO_SetBits(IO_LCD,EN);
	Delay(0xfffff);
	GPIO_ResetBits(IO_LCD,EN);
}

/********************************************************************************************
*函数名称：void Key_Init(void)
*
*入口参数：无
*
*出口参数：无
*
*功能说明：按键初始化配置
*******************************************************************************************/
void Key_Init(void)
{
  	GPIO_InitTypeDef GPIO_InitStructure;
  
	RCC_APB2PeriphClockCmd(KEY1_RCC_APB2Periph , ENABLE);// 使能APB2外设KEY1时钟
	RCC_APB2PeriphClockCmd(KEY2_RCC_APB2Periph , ENABLE);// 使能APB2外设KEY2时钟
	RCC_APB2PeriphClockCmd(KEY3_RCC_APB2Periph , ENABLE);// 使能APB2外设KEY3时钟

  	GPIO_InitStructure.GPIO_Pin	= KEY1_GPIO_Pin;         //选择KEY1
  	//GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;	 //管脚频率为50MHZ
  	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;//模式为输入浮空
  	GPIO_Init(KEY1_GPIO, &GPIO_InitStructure);           //初始化KEY1寄存器

  	GPIO_InitStructure.GPIO_Pin	= KEY2_GPIO_Pin;         //选择KEY2
  	//GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;	 //管脚频率为50MHZ
  	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;//模式为输入浮空
  	GPIO_Init(KEY2_GPIO, &GPIO_InitStructure);           //初始化KEY2寄存器

	GPIO_InitStructure.GPIO_Pin	= KEY3_GPIO_Pin;         //选择KEY3
  	//GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz; 	 //管脚频率为50MHZ
  	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;//模式为输入浮空
  	GPIO_Init(KEY3_GPIO, &GPIO_InitStructure);           //初始化KEY3寄存器
}

/********************************************************************************************
*函数名称：void Key_Test(void)
*
*入口参数：无
*
*出口参数：无
*
*功能说明：key测试
*******************************************************************************************/
void Key_Test(void)
{
    /***************按键1的测试********************/
	if(GPIO_ReadInputDataBit(KEY1_GPIO,KEY1_GPIO_Pin) == Bit_RESET)
	{
	    GPIO_ResetBits(IO_LCD,D0);    
	}
	else if(GPIO_ReadInputDataBit(KEY1_GPIO,KEY1_GPIO_Pin) == Bit_SET)
	{
	    GPIO_SetBits(IO_LCD,D0);    
	}
	/***************按键2的测试********************/
	if(GPIO_ReadInputDataBit(KEY2_GPIO,KEY2_GPIO_Pin) == Bit_RESET)
	{
	    Delay(0xfffff);
		if(GPIO_ReadInputDataBit(KEY2_GPIO,KEY2_GPIO_Pin) == Bit_RESET)
		{
		    if(key_info.Key2_state == true)
			    key_info.Key2_state = false;
			else
				key_info.Key2_state = true;
		}		
	}

	if(key_info.Key2_state == true)
	    GPIO_ResetBits(IO_LCD,D1);
	else 
		GPIO_SetBits(IO_LCD,D1);

    /***************按键3的测试********************/
	if(GPIO_ReadInputDataBit(IO_LCD,D3) == Bit_RESET)
	{
	    Delay(0xffff);
		if(GPIO_ReadInputDataBit(IO_LCD,D3) == Bit_RESET)
		{
		    if(key_info.Key3_state == true)
			    key_info.Key3_state = false;
			else
				key_info.Key3_state = true;
		}		
	}

	key_info.Counter2 += 0xfff;
	if(key_info.Counter2 > 0x2fffff)
    key_info.Counter2 = 0;

	if(key_info.Key3_state == true)
	{
	    key_info.Counter1 += 1;
		if(key_info.Counter1 > 0x2ff)
		    key_info.Counter1 = 0;
		
		if(key_info.Counter2 > key_info.Counter1 * 0xfff)
		    GPIO_SetBits(IO_LCD,D3);
		else
		    GPIO_ResetBits(IO_LCD,D3);
	}
	else
	    GPIO_SetBits(IO_LCD,D3);
}






