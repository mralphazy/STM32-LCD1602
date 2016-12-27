#ifndef _GPIO_LED_H_
#define _GPIO_LED_H_ 

/*****D0~7接口声明*****/
#define Data_RCC_APB2Periph	RCC_APB2Periph_GPIOF
#define IO_LCD			GPIOF
#define D0			GPIO_Pin_0	//引脚0～11
#define D1			GPIO_Pin_1
#define D2			GPIO_Pin_2
#define D3			GPIO_Pin_3
#define D4			GPIO_Pin_4
#define D5			GPIO_Pin_5
#define D6			GPIO_Pin_6
#define D7			GPIO_Pin_7
#define RS			GPIO_Pin_8
#define RW			GPIO_Pin_9
#define EN			GPIO_Pin_10
#define BackLight	GPIO_Pin_11

/*****按键USER1接口声明*****/
#define KEY1_RCC_APB2Periph  RCC_APB2Periph_GPIOA
#define KEY1_GPIO            GPIOA
#define KEY1_GPIO_Pin        GPIO_Pin_8

/*****按键USER2接口声明*****/
#define KEY2_RCC_APB2Periph  RCC_APB2Periph_GPIOD
#define KEY2_GPIO            GPIOD
#define KEY2_GPIO_Pin        GPIO_Pin_3

/*****按键WAKEUP接口声明*****/
#define KEY3_RCC_APB2Periph  RCC_APB2Periph_GPIOA
#define KEY3_GPIO            GPIOA
#define KEY3_GPIO_Pin        GPIO_Pin_0

typedef enum
{
    false=0,
	true
}Bool;

typedef struct
{
    Bool Key1_state;	//按键1的状态标记
	Bool Key2_state;	//按键2的状态标记
	Bool Key3_state;	//按键3的状态标记
	u32  Counter1;      //临时计数
	u32  Counter2;      //临时计数
}Key_Info;

void GpioLed_Init(void);
void LED_Display(void);
void Key_Init(void);
void Key_Test(void);
void WriteData_8(u8 Data_8);
u8 ReadData_8(void);


#endif
