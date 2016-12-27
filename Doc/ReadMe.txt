/**********************************************************************************
** 红龙开发板
** 论坛：bbs.openmcu.com
** 旺宝：www.openmcu.com
** 邮箱：support@openmcu.com

** 实验名：红龙板_gpio（带视频）
** KEIL MDK-ARM Standard Version：4.11
** 软件库(keil) Version: unused
** 固件库(ST) Version: 3.5
** 使用外设: LED key
**********************************************************************************/
程序测试说明: #define TEST_1  TEST_1 LED灯测试
			  #define TEST_2  TEST_2 按键测试
LED 测试说明：注释掉 TEST_2 按键测试   //#define TEST_2  
			  初始化系统时钟
			  LED灯的IO口初始化

测试现象：    JLINK下载运行后，三个LED灯轮番闪烁。
-----------------------------------------------------------------------------------
按键测试说明：注释掉 TEST_1 LED灯测试   //#define TEST_1
			  初始化系统时钟
			  LED灯的IO口初始化
			  按键IO口初始化

测试现象：	  JLINK下载运行后，按下按按键USER1 LED1灯亮；松开USER1 LED1灯灭。
							   按下按按键USER2 LED2依次取反。
                               按下按按键WAKE UP  打开和关闭LED3由亮渐灭控制。