/******************************************************************
**	  ���������壨V1.0��
**	  Gpio�����ļ�
**
**	  ��    ̳��bbs.openmcu.com
**	  ��    ����www.openmcu.com
**	  ��    �䣺support@openmcu.com
**
**    ��    ����V1.0
**	  ��    �ߣ�FXL
**	  �������:	2012.7.20
********************************************************************/
#include "stm32f10x.h"
#include "Gpio_Led.h"

Key_Info key_info;

/********************************************************************************************
*�������ƣ�void GpioLed_Init(void)
*
*��ڲ�������
*
*���ڲ�������
*
*����˵����led�Ƴ�ʼ������
*******************************************************************************************/
void GpioLed_Init(void)
{
	u16 i,PinNum;
	GPIO_InitTypeDef GPIO_InitStructure;	//�ṹ����
	RCC_APB2PeriphClockCmd(Data_RCC_APB2Periph , ENABLE);// ʹ��APB2����LED1ʱ��
	for (i=0;i<11;i++)
	{
		PinNum = 1;
		PinNum = PinNum<<i;
		GPIO_InitStructure.GPIO_Pin	= PinNum; 
		GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;	 //�ܽ�Ƶ��Ϊ50MHZ
		GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;	 //ģʽΪ�������
		GPIO_Init(IO_LCD, &GPIO_InitStructure);           //��ʼ��led1�Ĵ���
	}
}

/********************************************************************************************
*�������ƣ�void WriteData_8(u8 Data_8)
*
*��ڲ�������
*
*���ڲ�������
*
*����˵����д8λ���ݡ���PinNum������Ϊ���������룬���ж�����ĳһλ��ֵ����Ƭ�����ź���PinNum��ͬ
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
*�������ƣ�void ReadData_8(u8 Data_8)
*
*��ڲ�������
*
*���ڲ�������
*
*����˵����д8λ���ݡ���PinNum������Ϊ���������룬���ж�����ĳһλ��ֵ����Ƭ�����ź���PinNum��ͬ
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
*�������ƣ�static void Delay(u32 counter)
*
*��ڲ�����u32 counter����������
*
*���ڲ�������
*
*����˵������ʱ����
*******************************************************************************************/
static void Delay(u32 counter)
{
	while(counter--);
}

/********************************************************************************************
*�������ƣ�void LED_Disply(void)
*
*��ڲ�������
*
*���ڲ�������
*
*����˵����LED��˸
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
*�������ƣ�void Key_Init(void)
*
*��ڲ�������
*
*���ڲ�������
*
*����˵����������ʼ������
*******************************************************************************************/
void Key_Init(void)
{
  	GPIO_InitTypeDef GPIO_InitStructure;
  
	RCC_APB2PeriphClockCmd(KEY1_RCC_APB2Periph , ENABLE);// ʹ��APB2����KEY1ʱ��
	RCC_APB2PeriphClockCmd(KEY2_RCC_APB2Periph , ENABLE);// ʹ��APB2����KEY2ʱ��
	RCC_APB2PeriphClockCmd(KEY3_RCC_APB2Periph , ENABLE);// ʹ��APB2����KEY3ʱ��

  	GPIO_InitStructure.GPIO_Pin	= KEY1_GPIO_Pin;         //ѡ��KEY1
  	//GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;	 //�ܽ�Ƶ��Ϊ50MHZ
  	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;//ģʽΪ���븡��
  	GPIO_Init(KEY1_GPIO, &GPIO_InitStructure);           //��ʼ��KEY1�Ĵ���

  	GPIO_InitStructure.GPIO_Pin	= KEY2_GPIO_Pin;         //ѡ��KEY2
  	//GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;	 //�ܽ�Ƶ��Ϊ50MHZ
  	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;//ģʽΪ���븡��
  	GPIO_Init(KEY2_GPIO, &GPIO_InitStructure);           //��ʼ��KEY2�Ĵ���

	GPIO_InitStructure.GPIO_Pin	= KEY3_GPIO_Pin;         //ѡ��KEY3
  	//GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz; 	 //�ܽ�Ƶ��Ϊ50MHZ
  	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;//ģʽΪ���븡��
  	GPIO_Init(KEY3_GPIO, &GPIO_InitStructure);           //��ʼ��KEY3�Ĵ���
}

/********************************************************************************************
*�������ƣ�void Key_Test(void)
*
*��ڲ�������
*
*���ڲ�������
*
*����˵����key����
*******************************************************************************************/
void Key_Test(void)
{
    /***************����1�Ĳ���********************/
	if(GPIO_ReadInputDataBit(KEY1_GPIO,KEY1_GPIO_Pin) == Bit_RESET)
	{
	    GPIO_ResetBits(IO_LCD,D0);    
	}
	else if(GPIO_ReadInputDataBit(KEY1_GPIO,KEY1_GPIO_Pin) == Bit_SET)
	{
	    GPIO_SetBits(IO_LCD,D0);    
	}
	/***************����2�Ĳ���********************/
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

    /***************����3�Ĳ���********************/
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





