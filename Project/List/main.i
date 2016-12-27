#line 1 "..\\User\\main.c"




























 


#line 1 "..\\Libraries\\CMSIS\\CM3\\DeviceSupport\\ST\\STM32F10x\\stm32f10x.h"







































 



 



 
    






  


 
  


 

#line 75 "..\\Libraries\\CMSIS\\CM3\\DeviceSupport\\ST\\STM32F10x\\stm32f10x.h"


















 





#line 107 "..\\Libraries\\CMSIS\\CM3\\DeviceSupport\\ST\\STM32F10x\\stm32f10x.h"







            
#line 122 "..\\Libraries\\CMSIS\\CM3\\DeviceSupport\\ST\\STM32F10x\\stm32f10x.h"





 






 
#line 143 "..\\Libraries\\CMSIS\\CM3\\DeviceSupport\\ST\\STM32F10x\\stm32f10x.h"



 



 



 
#line 162 "..\\Libraries\\CMSIS\\CM3\\DeviceSupport\\ST\\STM32F10x\\stm32f10x.h"




 
typedef enum IRQn
{
 
  NonMaskableInt_IRQn         = -14,     
  MemoryManagement_IRQn       = -12,     
  BusFault_IRQn               = -11,     
  UsageFault_IRQn             = -10,     
  SVCall_IRQn                 = -5,      
  DebugMonitor_IRQn           = -4,      
  PendSV_IRQn                 = -2,      
  SysTick_IRQn                = -1,      

 
  WWDG_IRQn                   = 0,       
  PVD_IRQn                    = 1,       
  TAMPER_IRQn                 = 2,       
  RTC_IRQn                    = 3,       
  FLASH_IRQn                  = 4,       
  RCC_IRQn                    = 5,       
  EXTI0_IRQn                  = 6,       
  EXTI1_IRQn                  = 7,       
  EXTI2_IRQn                  = 8,       
  EXTI3_IRQn                  = 9,       
  EXTI4_IRQn                  = 10,      
  DMA1_Channel1_IRQn          = 11,      
  DMA1_Channel2_IRQn          = 12,      
  DMA1_Channel3_IRQn          = 13,      
  DMA1_Channel4_IRQn          = 14,      
  DMA1_Channel5_IRQn          = 15,      
  DMA1_Channel6_IRQn          = 16,      
  DMA1_Channel7_IRQn          = 17,      

#line 221 "..\\Libraries\\CMSIS\\CM3\\DeviceSupport\\ST\\STM32F10x\\stm32f10x.h"

#line 242 "..\\Libraries\\CMSIS\\CM3\\DeviceSupport\\ST\\STM32F10x\\stm32f10x.h"

#line 270 "..\\Libraries\\CMSIS\\CM3\\DeviceSupport\\ST\\STM32F10x\\stm32f10x.h"

#line 296 "..\\Libraries\\CMSIS\\CM3\\DeviceSupport\\ST\\STM32F10x\\stm32f10x.h"


  ADC1_2_IRQn                 = 18,      
  USB_HP_CAN1_TX_IRQn         = 19,      
  USB_LP_CAN1_RX0_IRQn        = 20,      
  CAN1_RX1_IRQn               = 21,      
  CAN1_SCE_IRQn               = 22,      
  EXTI9_5_IRQn                = 23,      
  TIM1_BRK_IRQn               = 24,      
  TIM1_UP_IRQn                = 25,      
  TIM1_TRG_COM_IRQn           = 26,      
  TIM1_CC_IRQn                = 27,      
  TIM2_IRQn                   = 28,      
  TIM3_IRQn                   = 29,      
  TIM4_IRQn                   = 30,      
  I2C1_EV_IRQn                = 31,      
  I2C1_ER_IRQn                = 32,      
  I2C2_EV_IRQn                = 33,      
  I2C2_ER_IRQn                = 34,      
  SPI1_IRQn                   = 35,      
  SPI2_IRQn                   = 36,      
  USART1_IRQn                 = 37,      
  USART2_IRQn                 = 38,      
  USART3_IRQn                 = 39,      
  EXTI15_10_IRQn              = 40,      
  RTCAlarm_IRQn               = 41,      
  USBWakeUp_IRQn              = 42,      
  TIM8_BRK_IRQn               = 43,      
  TIM8_UP_IRQn                = 44,      
  TIM8_TRG_COM_IRQn           = 45,      
  TIM8_CC_IRQn                = 46,      
  ADC3_IRQn                   = 47,      
  FSMC_IRQn                   = 48,      
  SDIO_IRQn                   = 49,      
  TIM5_IRQn                   = 50,      
  SPI3_IRQn                   = 51,      
  UART4_IRQn                  = 52,      
  UART5_IRQn                  = 53,      
  TIM6_IRQn                   = 54,      
  TIM7_IRQn                   = 55,      
  DMA2_Channel1_IRQn          = 56,      
  DMA2_Channel2_IRQn          = 57,      
  DMA2_Channel3_IRQn          = 58,      
  DMA2_Channel4_5_IRQn        = 59       


#line 381 "..\\Libraries\\CMSIS\\CM3\\DeviceSupport\\ST\\STM32F10x\\stm32f10x.h"

#line 426 "..\\Libraries\\CMSIS\\CM3\\DeviceSupport\\ST\\STM32F10x\\stm32f10x.h"

#line 472 "..\\Libraries\\CMSIS\\CM3\\DeviceSupport\\ST\\STM32F10x\\stm32f10x.h"
} IRQn_Type;



 

#line 1 "..\\Libraries\\CMSIS\\CM3\\CoreSupport\\core_cm3.h"
 




















 





































 

 
 
 
 
 
 
 
 








 











#line 1 "d:\\Keil\\ARM\\ARMCC\\bin\\..\\include\\stdint.h"
 
 





 










#line 26 "d:\\Keil\\ARM\\ARMCC\\bin\\..\\include\\stdint.h"







 

     

     
typedef   signed          char int8_t;
typedef   signed short     int int16_t;
typedef   signed           int int32_t;
typedef   signed       __int64 int64_t;

     
typedef unsigned          char uint8_t;
typedef unsigned short     int uint16_t;
typedef unsigned           int uint32_t;
typedef unsigned       __int64 uint64_t;

     

     
     
typedef   signed          char int_least8_t;
typedef   signed short     int int_least16_t;
typedef   signed           int int_least32_t;
typedef   signed       __int64 int_least64_t;

     
typedef unsigned          char uint_least8_t;
typedef unsigned short     int uint_least16_t;
typedef unsigned           int uint_least32_t;
typedef unsigned       __int64 uint_least64_t;

     

     
typedef   signed           int int_fast8_t;
typedef   signed           int int_fast16_t;
typedef   signed           int int_fast32_t;
typedef   signed       __int64 int_fast64_t;

     
typedef unsigned           int uint_fast8_t;
typedef unsigned           int uint_fast16_t;
typedef unsigned           int uint_fast32_t;
typedef unsigned       __int64 uint_fast64_t;

     
typedef   signed           int intptr_t;
typedef unsigned           int uintptr_t;

     
typedef   signed       __int64 intmax_t;
typedef unsigned       __int64 uintmax_t;




     

     





     





     





     

     





     





     





     

     





     





     





     

     


     


     


     

     


     


     


     

     



     



     


     
    
 



#line 197 "d:\\Keil\\ARM\\ARMCC\\bin\\..\\include\\stdint.h"

     







     










     











#line 261 "d:\\Keil\\ARM\\ARMCC\\bin\\..\\include\\stdint.h"



 



#line 91 "..\\Libraries\\CMSIS\\CM3\\CoreSupport\\core_cm3.h"

















 

#line 117 "..\\Libraries\\CMSIS\\CM3\\CoreSupport\\core_cm3.h"





 


 





 
typedef struct
{
  volatile uint32_t ISER[8];                       
       uint32_t RESERVED0[24];                                   
  volatile uint32_t ICER[8];                       
       uint32_t RSERVED1[24];                                    
  volatile uint32_t ISPR[8];                       
       uint32_t RESERVED2[24];                                   
  volatile uint32_t ICPR[8];                       
       uint32_t RESERVED3[24];                                   
  volatile uint32_t IABR[8];                       
       uint32_t RESERVED4[56];                                   
  volatile uint8_t  IP[240];                       
       uint32_t RESERVED5[644];                                  
  volatile  uint32_t STIR;                          
}  NVIC_Type;                                               
   





 
typedef struct
{
  volatile const  uint32_t CPUID;                         
  volatile uint32_t ICSR;                          
  volatile uint32_t VTOR;                          
  volatile uint32_t AIRCR;                         
  volatile uint32_t SCR;                           
  volatile uint32_t CCR;                           
  volatile uint8_t  SHP[12];                       
  volatile uint32_t SHCSR;                         
  volatile uint32_t CFSR;                          
  volatile uint32_t HFSR;                          
  volatile uint32_t DFSR;                          
  volatile uint32_t MMFAR;                         
  volatile uint32_t BFAR;                          
  volatile uint32_t AFSR;                          
  volatile const  uint32_t PFR[2];                        
  volatile const  uint32_t DFR;                           
  volatile const  uint32_t ADR;                           
  volatile const  uint32_t MMFR[4];                       
  volatile const  uint32_t ISAR[5];                       
} SCB_Type;                                                

 












 






























 






 





















 









 


















 
































                                     









 









 









 














   





 
typedef struct
{
  volatile uint32_t CTRL;                          
  volatile uint32_t LOAD;                          
  volatile uint32_t VAL;                           
  volatile const  uint32_t CALIB;                         
} SysTick_Type;

 












 



 



 








   





 
typedef struct
{
  volatile  union  
  {
    volatile  uint8_t    u8;                        
    volatile  uint16_t   u16;                       
    volatile  uint32_t   u32;                       
  }  PORT [32];                                
       uint32_t RESERVED0[864];                                 
  volatile uint32_t TER;                           
       uint32_t RESERVED1[15];                                  
  volatile uint32_t TPR;                           
       uint32_t RESERVED2[15];                                  
  volatile uint32_t TCR;                           
       uint32_t RESERVED3[29];                                  
  volatile uint32_t IWR;                           
  volatile uint32_t IRR;                           
  volatile uint32_t IMCR;                          
       uint32_t RESERVED4[43];                                  
  volatile uint32_t LAR;                           
  volatile uint32_t LSR;                           
       uint32_t RESERVED5[6];                                   
  volatile const  uint32_t PID4;                          
  volatile const  uint32_t PID5;                          
  volatile const  uint32_t PID6;                          
  volatile const  uint32_t PID7;                          
  volatile const  uint32_t PID0;                          
  volatile const  uint32_t PID1;                          
  volatile const  uint32_t PID2;                          
  volatile const  uint32_t PID3;                          
  volatile const  uint32_t CID0;                          
  volatile const  uint32_t CID1;                          
  volatile const  uint32_t CID2;                          
  volatile const  uint32_t CID3;                          
} ITM_Type;                                                

 



 
























 



 



 



 








   





 
typedef struct
{
       uint32_t RESERVED0;
  volatile const  uint32_t ICTR;                          



       uint32_t RESERVED1;

} InterruptType_Type;

 



 








   


#line 614 "..\\Libraries\\CMSIS\\CM3\\CoreSupport\\core_cm3.h"





 
typedef struct
{
  volatile uint32_t DHCSR;                         
  volatile  uint32_t DCRSR;                         
  volatile uint32_t DCRDR;                         
  volatile uint32_t DEMCR;                         
} CoreDebug_Type;

 




































 






 






































   


 
#line 721 "..\\Libraries\\CMSIS\\CM3\\CoreSupport\\core_cm3.h"

#line 728 "..\\Libraries\\CMSIS\\CM3\\CoreSupport\\core_cm3.h"






   




 





#line 758 "..\\Libraries\\CMSIS\\CM3\\CoreSupport\\core_cm3.h"


 


 




#line 783 "..\\Libraries\\CMSIS\\CM3\\CoreSupport\\core_cm3.h"


 
 
 
 








 
extern uint32_t __get_PSP(void);








 
extern void __set_PSP(uint32_t topOfProcStack);








 
extern uint32_t __get_MSP(void);








 
extern void __set_MSP(uint32_t topOfMainStack);








 
extern uint32_t __REV16(uint16_t value);








 
extern int32_t __REVSH(int16_t value);


#line 933 "..\\Libraries\\CMSIS\\CM3\\CoreSupport\\core_cm3.h"





 








 
static __inline uint32_t  __get_BASEPRI(void)
{
  register uint32_t __regBasePri         __asm("basepri");
  return(__regBasePri);
}







 
static __inline void __set_BASEPRI(uint32_t basePri)
{
  register uint32_t __regBasePri         __asm("basepri");
  __regBasePri = (basePri & 0xff);
}







 
static __inline uint32_t __get_PRIMASK(void)
{
  register uint32_t __regPriMask         __asm("primask");
  return(__regPriMask);
}







 
static __inline void __set_PRIMASK(uint32_t priMask)
{
  register uint32_t __regPriMask         __asm("primask");
  __regPriMask = (priMask);
}







 
static __inline uint32_t __get_FAULTMASK(void)
{
  register uint32_t __regFaultMask       __asm("faultmask");
  return(__regFaultMask);
}







 
static __inline void __set_FAULTMASK(uint32_t faultMask)
{
  register uint32_t __regFaultMask       __asm("faultmask");
  __regFaultMask = (faultMask & 1);
}







 
static __inline uint32_t __get_CONTROL(void)
{
  register uint32_t __regControl         __asm("control");
  return(__regControl);
}







 
static __inline void __set_CONTROL(uint32_t control)
{
  register uint32_t __regControl         __asm("control");
  __regControl = control;
}





#line 1445 "..\\Libraries\\CMSIS\\CM3\\CoreSupport\\core_cm3.h"







 
 

 











 
static __inline void NVIC_SetPriorityGrouping(uint32_t PriorityGroup)
{
  uint32_t reg_value;
  uint32_t PriorityGroupTmp = (PriorityGroup & 0x07);                          
  
  reg_value  =  ((SCB_Type *) ((0xE000E000) + 0x0D00))->AIRCR;                                                    
  reg_value &= ~((0xFFFFul << 16) | (7ul << 8));              
  reg_value  =  (reg_value                       |
                (0x5FA << 16) | 
                (PriorityGroupTmp << 8));                                      
  ((SCB_Type *) ((0xE000E000) + 0x0D00))->AIRCR =  reg_value;
}








 
static __inline uint32_t NVIC_GetPriorityGrouping(void)
{
  return ((((SCB_Type *) ((0xE000E000) + 0x0D00))->AIRCR & (7ul << 8)) >> 8);    
}








 
static __inline void NVIC_EnableIRQ(IRQn_Type IRQn)
{
  ((NVIC_Type *) ((0xE000E000) + 0x0100))->ISER[((uint32_t)(IRQn) >> 5)] = (1 << ((uint32_t)(IRQn) & 0x1F));  
}








 
static __inline void NVIC_DisableIRQ(IRQn_Type IRQn)
{
  ((NVIC_Type *) ((0xE000E000) + 0x0100))->ICER[((uint32_t)(IRQn) >> 5)] = (1 << ((uint32_t)(IRQn) & 0x1F));  
}









 
static __inline uint32_t NVIC_GetPendingIRQ(IRQn_Type IRQn)
{
  return((uint32_t) ((((NVIC_Type *) ((0xE000E000) + 0x0100))->ISPR[(uint32_t)(IRQn) >> 5] & (1 << ((uint32_t)(IRQn) & 0x1F)))?1:0));  
}








 
static __inline void NVIC_SetPendingIRQ(IRQn_Type IRQn)
{
  ((NVIC_Type *) ((0xE000E000) + 0x0100))->ISPR[((uint32_t)(IRQn) >> 5)] = (1 << ((uint32_t)(IRQn) & 0x1F));  
}








 
static __inline void NVIC_ClearPendingIRQ(IRQn_Type IRQn)
{
  ((NVIC_Type *) ((0xE000E000) + 0x0100))->ICPR[((uint32_t)(IRQn) >> 5)] = (1 << ((uint32_t)(IRQn) & 0x1F));  
}









 
static __inline uint32_t NVIC_GetActive(IRQn_Type IRQn)
{
  return((uint32_t)((((NVIC_Type *) ((0xE000E000) + 0x0100))->IABR[(uint32_t)(IRQn) >> 5] & (1 << ((uint32_t)(IRQn) & 0x1F)))?1:0));  
}












 
static __inline void NVIC_SetPriority(IRQn_Type IRQn, uint32_t priority)
{
  if(IRQn < 0) {
    ((SCB_Type *) ((0xE000E000) + 0x0D00))->SHP[((uint32_t)(IRQn) & 0xF)-4] = ((priority << (8 - 4)) & 0xff); }  
  else {
    ((NVIC_Type *) ((0xE000E000) + 0x0100))->IP[(uint32_t)(IRQn)] = ((priority << (8 - 4)) & 0xff);    }         
}















 
static __inline uint32_t NVIC_GetPriority(IRQn_Type IRQn)
{

  if(IRQn < 0) {
    return((uint32_t)(((SCB_Type *) ((0xE000E000) + 0x0D00))->SHP[((uint32_t)(IRQn) & 0xF)-4] >> (8 - 4)));  }  
  else {
    return((uint32_t)(((NVIC_Type *) ((0xE000E000) + 0x0100))->IP[(uint32_t)(IRQn)]           >> (8 - 4)));  }  
}
















 
static __inline uint32_t NVIC_EncodePriority (uint32_t PriorityGroup, uint32_t PreemptPriority, uint32_t SubPriority)
{
  uint32_t PriorityGroupTmp = (PriorityGroup & 0x07);           
  uint32_t PreemptPriorityBits;
  uint32_t SubPriorityBits;

  PreemptPriorityBits = ((7 - PriorityGroupTmp) > 4) ? 4 : 7 - PriorityGroupTmp;
  SubPriorityBits     = ((PriorityGroupTmp + 4) < 7) ? 0 : PriorityGroupTmp - 7 + 4;
 
  return (
           ((PreemptPriority & ((1 << (PreemptPriorityBits)) - 1)) << SubPriorityBits) |
           ((SubPriority     & ((1 << (SubPriorityBits    )) - 1)))
         );
}
















 
static __inline void NVIC_DecodePriority (uint32_t Priority, uint32_t PriorityGroup, uint32_t* pPreemptPriority, uint32_t* pSubPriority)
{
  uint32_t PriorityGroupTmp = (PriorityGroup & 0x07);           
  uint32_t PreemptPriorityBits;
  uint32_t SubPriorityBits;

  PreemptPriorityBits = ((7 - PriorityGroupTmp) > 4) ? 4 : 7 - PriorityGroupTmp;
  SubPriorityBits     = ((PriorityGroupTmp + 4) < 7) ? 0 : PriorityGroupTmp - 7 + 4;
  
  *pPreemptPriority = (Priority >> SubPriorityBits) & ((1 << (PreemptPriorityBits)) - 1);
  *pSubPriority     = (Priority                   ) & ((1 << (SubPriorityBits    )) - 1);
}



 












 
static __inline uint32_t SysTick_Config(uint32_t ticks)
{ 
  if (ticks > (0xFFFFFFul << 0))  return (1);             
                                                               
  ((SysTick_Type *) ((0xE000E000) + 0x0010))->LOAD  = (ticks & (0xFFFFFFul << 0)) - 1;       
  NVIC_SetPriority (SysTick_IRQn, (1<<4) - 1);   
  ((SysTick_Type *) ((0xE000E000) + 0x0010))->VAL   = 0;                                           
  ((SysTick_Type *) ((0xE000E000) + 0x0010))->CTRL  = (1ul << 2) | 
                   (1ul << 1)   | 
                   (1ul << 0);                     
  return (0);                                                   
}






 





 
static __inline void NVIC_SystemReset(void)
{
  ((SCB_Type *) ((0xE000E000) + 0x0D00))->AIRCR  = ((0x5FA << 16)      | 
                 (((SCB_Type *) ((0xE000E000) + 0x0D00))->AIRCR & (7ul << 8)) | 
                 (1ul << 2));                    
  __dsb(0);                                                                    
  while(1);                                                     
}

   



 






 
 

extern volatile int ITM_RxBuffer;                     












 
static __inline uint32_t ITM_SendChar (uint32_t ch)
{
  if ((((CoreDebug_Type *) (0xE000EDF0))->DEMCR & (1ul << 24))  &&       
      (((ITM_Type *) (0xE0000000))->TCR & (1ul << 0))                  &&       
      (((ITM_Type *) (0xE0000000))->TER & (1ul << 0)        )                    )      
  {
    while (((ITM_Type *) (0xE0000000))->PORT[0].u32 == 0);
    ((ITM_Type *) (0xE0000000))->PORT[0].u8 = (uint8_t) ch;
  }  
  return (ch);
}










 
static __inline int ITM_ReceiveChar (void) {
  int ch = -1;                                

  if (ITM_RxBuffer != 0x5AA55AA5) {
    ch = ITM_RxBuffer;
    ITM_RxBuffer = 0x5AA55AA5;        
  }
  
  return (ch); 
}









 
static __inline int ITM_CheckChar (void) {

  if (ITM_RxBuffer == 0x5AA55AA5) {
    return (0);                                  
  } else {
    return (1);                                  
  }
}

   






   



 
#line 479 "..\\Libraries\\CMSIS\\CM3\\DeviceSupport\\ST\\STM32F10x\\stm32f10x.h"
#line 1 "..\\Libraries\\CMSIS\\CM3\\DeviceSupport\\ST\\STM32F10x\\system_stm32f10x.h"



















 



 



   
  


 









 



 




 

extern uint32_t SystemCoreClock;           



 



 



 



 



 



 
  
extern void SystemInit(void);
extern void SystemCoreClockUpdate(void);


 









 
  


   
 
#line 480 "..\\Libraries\\CMSIS\\CM3\\DeviceSupport\\ST\\STM32F10x\\stm32f10x.h"
#line 481 "..\\Libraries\\CMSIS\\CM3\\DeviceSupport\\ST\\STM32F10x\\stm32f10x.h"



   

 
typedef int32_t  s32;
typedef int16_t s16;
typedef int8_t  s8;

typedef const int32_t sc32;   
typedef const int16_t sc16;   
typedef const int8_t sc8;    

typedef volatile int32_t  vs32;
typedef volatile int16_t  vs16;
typedef volatile int8_t   vs8;

typedef volatile const int32_t vsc32;   
typedef volatile const int16_t vsc16;   
typedef volatile const int8_t vsc8;    

typedef uint32_t  u32;
typedef uint16_t u16;
typedef uint8_t  u8;

typedef const uint32_t uc32;   
typedef const uint16_t uc16;   
typedef const uint8_t uc8;    

typedef volatile uint32_t  vu32;
typedef volatile uint16_t vu16;
typedef volatile uint8_t  vu8;

typedef volatile const uint32_t vuc32;   
typedef volatile const uint16_t vuc16;   
typedef volatile const uint8_t vuc8;    

typedef enum {RESET = 0, SET = !RESET} FlagStatus, ITStatus;

typedef enum {DISABLE = 0, ENABLE = !DISABLE} FunctionalState;


typedef enum {ERROR = 0, SUCCESS = !ERROR} ErrorStatus;

 





 



    



 

typedef struct
{
  volatile uint32_t SR;
  volatile uint32_t CR1;
  volatile uint32_t CR2;
  volatile uint32_t SMPR1;
  volatile uint32_t SMPR2;
  volatile uint32_t JOFR1;
  volatile uint32_t JOFR2;
  volatile uint32_t JOFR3;
  volatile uint32_t JOFR4;
  volatile uint32_t HTR;
  volatile uint32_t LTR;
  volatile uint32_t SQR1;
  volatile uint32_t SQR2;
  volatile uint32_t SQR3;
  volatile uint32_t JSQR;
  volatile uint32_t JDR1;
  volatile uint32_t JDR2;
  volatile uint32_t JDR3;
  volatile uint32_t JDR4;
  volatile uint32_t DR;
} ADC_TypeDef;



 

typedef struct
{
  uint32_t  RESERVED0;
  volatile uint16_t DR1;
  uint16_t  RESERVED1;
  volatile uint16_t DR2;
  uint16_t  RESERVED2;
  volatile uint16_t DR3;
  uint16_t  RESERVED3;
  volatile uint16_t DR4;
  uint16_t  RESERVED4;
  volatile uint16_t DR5;
  uint16_t  RESERVED5;
  volatile uint16_t DR6;
  uint16_t  RESERVED6;
  volatile uint16_t DR7;
  uint16_t  RESERVED7;
  volatile uint16_t DR8;
  uint16_t  RESERVED8;
  volatile uint16_t DR9;
  uint16_t  RESERVED9;
  volatile uint16_t DR10;
  uint16_t  RESERVED10; 
  volatile uint16_t RTCCR;
  uint16_t  RESERVED11;
  volatile uint16_t CR;
  uint16_t  RESERVED12;
  volatile uint16_t CSR;
  uint16_t  RESERVED13[5];
  volatile uint16_t DR11;
  uint16_t  RESERVED14;
  volatile uint16_t DR12;
  uint16_t  RESERVED15;
  volatile uint16_t DR13;
  uint16_t  RESERVED16;
  volatile uint16_t DR14;
  uint16_t  RESERVED17;
  volatile uint16_t DR15;
  uint16_t  RESERVED18;
  volatile uint16_t DR16;
  uint16_t  RESERVED19;
  volatile uint16_t DR17;
  uint16_t  RESERVED20;
  volatile uint16_t DR18;
  uint16_t  RESERVED21;
  volatile uint16_t DR19;
  uint16_t  RESERVED22;
  volatile uint16_t DR20;
  uint16_t  RESERVED23;
  volatile uint16_t DR21;
  uint16_t  RESERVED24;
  volatile uint16_t DR22;
  uint16_t  RESERVED25;
  volatile uint16_t DR23;
  uint16_t  RESERVED26;
  volatile uint16_t DR24;
  uint16_t  RESERVED27;
  volatile uint16_t DR25;
  uint16_t  RESERVED28;
  volatile uint16_t DR26;
  uint16_t  RESERVED29;
  volatile uint16_t DR27;
  uint16_t  RESERVED30;
  volatile uint16_t DR28;
  uint16_t  RESERVED31;
  volatile uint16_t DR29;
  uint16_t  RESERVED32;
  volatile uint16_t DR30;
  uint16_t  RESERVED33; 
  volatile uint16_t DR31;
  uint16_t  RESERVED34;
  volatile uint16_t DR32;
  uint16_t  RESERVED35;
  volatile uint16_t DR33;
  uint16_t  RESERVED36;
  volatile uint16_t DR34;
  uint16_t  RESERVED37;
  volatile uint16_t DR35;
  uint16_t  RESERVED38;
  volatile uint16_t DR36;
  uint16_t  RESERVED39;
  volatile uint16_t DR37;
  uint16_t  RESERVED40;
  volatile uint16_t DR38;
  uint16_t  RESERVED41;
  volatile uint16_t DR39;
  uint16_t  RESERVED42;
  volatile uint16_t DR40;
  uint16_t  RESERVED43;
  volatile uint16_t DR41;
  uint16_t  RESERVED44;
  volatile uint16_t DR42;
  uint16_t  RESERVED45;    
} BKP_TypeDef;
  


 

typedef struct
{
  volatile uint32_t TIR;
  volatile uint32_t TDTR;
  volatile uint32_t TDLR;
  volatile uint32_t TDHR;
} CAN_TxMailBox_TypeDef;



 
  
typedef struct
{
  volatile uint32_t RIR;
  volatile uint32_t RDTR;
  volatile uint32_t RDLR;
  volatile uint32_t RDHR;
} CAN_FIFOMailBox_TypeDef;



 
  
typedef struct
{
  volatile uint32_t FR1;
  volatile uint32_t FR2;
} CAN_FilterRegister_TypeDef;



 
  
typedef struct
{
  volatile uint32_t MCR;
  volatile uint32_t MSR;
  volatile uint32_t TSR;
  volatile uint32_t RF0R;
  volatile uint32_t RF1R;
  volatile uint32_t IER;
  volatile uint32_t ESR;
  volatile uint32_t BTR;
  uint32_t  RESERVED0[88];
  CAN_TxMailBox_TypeDef sTxMailBox[3];
  CAN_FIFOMailBox_TypeDef sFIFOMailBox[2];
  uint32_t  RESERVED1[12];
  volatile uint32_t FMR;
  volatile uint32_t FM1R;
  uint32_t  RESERVED2;
  volatile uint32_t FS1R;
  uint32_t  RESERVED3;
  volatile uint32_t FFA1R;
  uint32_t  RESERVED4;
  volatile uint32_t FA1R;
  uint32_t  RESERVED5[8];

  CAN_FilterRegister_TypeDef sFilterRegister[14];



} CAN_TypeDef;



 
typedef struct
{
  volatile uint32_t CFGR;
  volatile uint32_t OAR;
  volatile uint32_t PRES;
  volatile uint32_t ESR;
  volatile uint32_t CSR;
  volatile uint32_t TXD;
  volatile uint32_t RXD;  
} CEC_TypeDef;



 

typedef struct
{
  volatile uint32_t DR;
  volatile uint8_t  IDR;
  uint8_t   RESERVED0;
  uint16_t  RESERVED1;
  volatile uint32_t CR;
} CRC_TypeDef;



 

typedef struct
{
  volatile uint32_t CR;
  volatile uint32_t SWTRIGR;
  volatile uint32_t DHR12R1;
  volatile uint32_t DHR12L1;
  volatile uint32_t DHR8R1;
  volatile uint32_t DHR12R2;
  volatile uint32_t DHR12L2;
  volatile uint32_t DHR8R2;
  volatile uint32_t DHR12RD;
  volatile uint32_t DHR12LD;
  volatile uint32_t DHR8RD;
  volatile uint32_t DOR1;
  volatile uint32_t DOR2;



} DAC_TypeDef;



 

typedef struct
{
  volatile uint32_t IDCODE;
  volatile uint32_t CR;	
}DBGMCU_TypeDef;



 

typedef struct
{
  volatile uint32_t CCR;
  volatile uint32_t CNDTR;
  volatile uint32_t CPAR;
  volatile uint32_t CMAR;
} DMA_Channel_TypeDef;

typedef struct
{
  volatile uint32_t ISR;
  volatile uint32_t IFCR;
} DMA_TypeDef;



 

typedef struct
{
  volatile uint32_t MACCR;
  volatile uint32_t MACFFR;
  volatile uint32_t MACHTHR;
  volatile uint32_t MACHTLR;
  volatile uint32_t MACMIIAR;
  volatile uint32_t MACMIIDR;
  volatile uint32_t MACFCR;
  volatile uint32_t MACVLANTR;              
       uint32_t RESERVED0[2];
  volatile uint32_t MACRWUFFR;              
  volatile uint32_t MACPMTCSR;
       uint32_t RESERVED1[2];
  volatile uint32_t MACSR;                  
  volatile uint32_t MACIMR;
  volatile uint32_t MACA0HR;
  volatile uint32_t MACA0LR;
  volatile uint32_t MACA1HR;
  volatile uint32_t MACA1LR;
  volatile uint32_t MACA2HR;
  volatile uint32_t MACA2LR;
  volatile uint32_t MACA3HR;
  volatile uint32_t MACA3LR;                
       uint32_t RESERVED2[40];
  volatile uint32_t MMCCR;                  
  volatile uint32_t MMCRIR;
  volatile uint32_t MMCTIR;
  volatile uint32_t MMCRIMR;
  volatile uint32_t MMCTIMR;                
       uint32_t RESERVED3[14];
  volatile uint32_t MMCTGFSCCR;             
  volatile uint32_t MMCTGFMSCCR;
       uint32_t RESERVED4[5];
  volatile uint32_t MMCTGFCR;
       uint32_t RESERVED5[10];
  volatile uint32_t MMCRFCECR;
  volatile uint32_t MMCRFAECR;
       uint32_t RESERVED6[10];
  volatile uint32_t MMCRGUFCR;
       uint32_t RESERVED7[334];
  volatile uint32_t PTPTSCR;
  volatile uint32_t PTPSSIR;
  volatile uint32_t PTPTSHR;
  volatile uint32_t PTPTSLR;
  volatile uint32_t PTPTSHUR;
  volatile uint32_t PTPTSLUR;
  volatile uint32_t PTPTSAR;
  volatile uint32_t PTPTTHR;
  volatile uint32_t PTPTTLR;
       uint32_t RESERVED8[567];
  volatile uint32_t DMABMR;
  volatile uint32_t DMATPDR;
  volatile uint32_t DMARPDR;
  volatile uint32_t DMARDLAR;
  volatile uint32_t DMATDLAR;
  volatile uint32_t DMASR;
  volatile uint32_t DMAOMR;
  volatile uint32_t DMAIER;
  volatile uint32_t DMAMFBOCR;
       uint32_t RESERVED9[9];
  volatile uint32_t DMACHTDR;
  volatile uint32_t DMACHRDR;
  volatile uint32_t DMACHTBAR;
  volatile uint32_t DMACHRBAR;
} ETH_TypeDef;



 

typedef struct
{
  volatile uint32_t IMR;
  volatile uint32_t EMR;
  volatile uint32_t RTSR;
  volatile uint32_t FTSR;
  volatile uint32_t SWIER;
  volatile uint32_t PR;
} EXTI_TypeDef;



 

typedef struct
{
  volatile uint32_t ACR;
  volatile uint32_t KEYR;
  volatile uint32_t OPTKEYR;
  volatile uint32_t SR;
  volatile uint32_t CR;
  volatile uint32_t AR;
  volatile uint32_t RESERVED;
  volatile uint32_t OBR;
  volatile uint32_t WRPR;
#line 920 "..\\Libraries\\CMSIS\\CM3\\DeviceSupport\\ST\\STM32F10x\\stm32f10x.h"
} FLASH_TypeDef;



 
  
typedef struct
{
  volatile uint16_t RDP;
  volatile uint16_t USER;
  volatile uint16_t Data0;
  volatile uint16_t Data1;
  volatile uint16_t WRP0;
  volatile uint16_t WRP1;
  volatile uint16_t WRP2;
  volatile uint16_t WRP3;
} OB_TypeDef;



 

typedef struct
{
  volatile uint32_t BTCR[8];   
} FSMC_Bank1_TypeDef; 



 
  
typedef struct
{
  volatile uint32_t BWTR[7];
} FSMC_Bank1E_TypeDef;



 
  
typedef struct
{
  volatile uint32_t PCR2;
  volatile uint32_t SR2;
  volatile uint32_t PMEM2;
  volatile uint32_t PATT2;
  uint32_t  RESERVED0;   
  volatile uint32_t ECCR2; 
} FSMC_Bank2_TypeDef;  



 
  
typedef struct
{
  volatile uint32_t PCR3;
  volatile uint32_t SR3;
  volatile uint32_t PMEM3;
  volatile uint32_t PATT3;
  uint32_t  RESERVED0;   
  volatile uint32_t ECCR3; 
} FSMC_Bank3_TypeDef; 



 
  
typedef struct
{
  volatile uint32_t PCR4;
  volatile uint32_t SR4;
  volatile uint32_t PMEM4;
  volatile uint32_t PATT4;
  volatile uint32_t PIO4; 
} FSMC_Bank4_TypeDef; 



 

typedef struct
{
  volatile uint32_t CRL;
  volatile uint32_t CRH;
  volatile uint32_t IDR;
  volatile uint32_t ODR;
  volatile uint32_t BSRR;
  volatile uint32_t BRR;
  volatile uint32_t LCKR;
} GPIO_TypeDef;



 

typedef struct
{
  volatile uint32_t EVCR;
  volatile uint32_t MAPR;
  volatile uint32_t EXTICR[4];
  uint32_t RESERVED0;
  volatile uint32_t MAPR2;  
} AFIO_TypeDef;


 

typedef struct
{
  volatile uint16_t CR1;
  uint16_t  RESERVED0;
  volatile uint16_t CR2;
  uint16_t  RESERVED1;
  volatile uint16_t OAR1;
  uint16_t  RESERVED2;
  volatile uint16_t OAR2;
  uint16_t  RESERVED3;
  volatile uint16_t DR;
  uint16_t  RESERVED4;
  volatile uint16_t SR1;
  uint16_t  RESERVED5;
  volatile uint16_t SR2;
  uint16_t  RESERVED6;
  volatile uint16_t CCR;
  uint16_t  RESERVED7;
  volatile uint16_t TRISE;
  uint16_t  RESERVED8;
} I2C_TypeDef;



 

typedef struct
{
  volatile uint32_t KR;
  volatile uint32_t PR;
  volatile uint32_t RLR;
  volatile uint32_t SR;
} IWDG_TypeDef;



 

typedef struct
{
  volatile uint32_t CR;
  volatile uint32_t CSR;
} PWR_TypeDef;



 

typedef struct
{
  volatile uint32_t CR;
  volatile uint32_t CFGR;
  volatile uint32_t CIR;
  volatile uint32_t APB2RSTR;
  volatile uint32_t APB1RSTR;
  volatile uint32_t AHBENR;
  volatile uint32_t APB2ENR;
  volatile uint32_t APB1ENR;
  volatile uint32_t BDCR;
  volatile uint32_t CSR;










} RCC_TypeDef;



 

typedef struct
{
  volatile uint16_t CRH;
  uint16_t  RESERVED0;
  volatile uint16_t CRL;
  uint16_t  RESERVED1;
  volatile uint16_t PRLH;
  uint16_t  RESERVED2;
  volatile uint16_t PRLL;
  uint16_t  RESERVED3;
  volatile uint16_t DIVH;
  uint16_t  RESERVED4;
  volatile uint16_t DIVL;
  uint16_t  RESERVED5;
  volatile uint16_t CNTH;
  uint16_t  RESERVED6;
  volatile uint16_t CNTL;
  uint16_t  RESERVED7;
  volatile uint16_t ALRH;
  uint16_t  RESERVED8;
  volatile uint16_t ALRL;
  uint16_t  RESERVED9;
} RTC_TypeDef;



 

typedef struct
{
  volatile uint32_t POWER;
  volatile uint32_t CLKCR;
  volatile uint32_t ARG;
  volatile uint32_t CMD;
  volatile const uint32_t RESPCMD;
  volatile const uint32_t RESP1;
  volatile const uint32_t RESP2;
  volatile const uint32_t RESP3;
  volatile const uint32_t RESP4;
  volatile uint32_t DTIMER;
  volatile uint32_t DLEN;
  volatile uint32_t DCTRL;
  volatile const uint32_t DCOUNT;
  volatile const uint32_t STA;
  volatile uint32_t ICR;
  volatile uint32_t MASK;
  uint32_t  RESERVED0[2];
  volatile const uint32_t FIFOCNT;
  uint32_t  RESERVED1[13];
  volatile uint32_t FIFO;
} SDIO_TypeDef;



 

typedef struct
{
  volatile uint16_t CR1;
  uint16_t  RESERVED0;
  volatile uint16_t CR2;
  uint16_t  RESERVED1;
  volatile uint16_t SR;
  uint16_t  RESERVED2;
  volatile uint16_t DR;
  uint16_t  RESERVED3;
  volatile uint16_t CRCPR;
  uint16_t  RESERVED4;
  volatile uint16_t RXCRCR;
  uint16_t  RESERVED5;
  volatile uint16_t TXCRCR;
  uint16_t  RESERVED6;
  volatile uint16_t I2SCFGR;
  uint16_t  RESERVED7;
  volatile uint16_t I2SPR;
  uint16_t  RESERVED8;  
} SPI_TypeDef;



 

typedef struct
{
  volatile uint16_t CR1;
  uint16_t  RESERVED0;
  volatile uint16_t CR2;
  uint16_t  RESERVED1;
  volatile uint16_t SMCR;
  uint16_t  RESERVED2;
  volatile uint16_t DIER;
  uint16_t  RESERVED3;
  volatile uint16_t SR;
  uint16_t  RESERVED4;
  volatile uint16_t EGR;
  uint16_t  RESERVED5;
  volatile uint16_t CCMR1;
  uint16_t  RESERVED6;
  volatile uint16_t CCMR2;
  uint16_t  RESERVED7;
  volatile uint16_t CCER;
  uint16_t  RESERVED8;
  volatile uint16_t CNT;
  uint16_t  RESERVED9;
  volatile uint16_t PSC;
  uint16_t  RESERVED10;
  volatile uint16_t ARR;
  uint16_t  RESERVED11;
  volatile uint16_t RCR;
  uint16_t  RESERVED12;
  volatile uint16_t CCR1;
  uint16_t  RESERVED13;
  volatile uint16_t CCR2;
  uint16_t  RESERVED14;
  volatile uint16_t CCR3;
  uint16_t  RESERVED15;
  volatile uint16_t CCR4;
  uint16_t  RESERVED16;
  volatile uint16_t BDTR;
  uint16_t  RESERVED17;
  volatile uint16_t DCR;
  uint16_t  RESERVED18;
  volatile uint16_t DMAR;
  uint16_t  RESERVED19;
} TIM_TypeDef;



 
 
typedef struct
{
  volatile uint16_t SR;
  uint16_t  RESERVED0;
  volatile uint16_t DR;
  uint16_t  RESERVED1;
  volatile uint16_t BRR;
  uint16_t  RESERVED2;
  volatile uint16_t CR1;
  uint16_t  RESERVED3;
  volatile uint16_t CR2;
  uint16_t  RESERVED4;
  volatile uint16_t CR3;
  uint16_t  RESERVED5;
  volatile uint16_t GTPR;
  uint16_t  RESERVED6;
} USART_TypeDef;



 

typedef struct
{
  volatile uint32_t CR;
  volatile uint32_t CFR;
  volatile uint32_t SR;
} WWDG_TypeDef;



 
  


 











 




#line 1312 "..\\Libraries\\CMSIS\\CM3\\DeviceSupport\\ST\\STM32F10x\\stm32f10x.h"

#line 1335 "..\\Libraries\\CMSIS\\CM3\\DeviceSupport\\ST\\STM32F10x\\stm32f10x.h"



#line 1354 "..\\Libraries\\CMSIS\\CM3\\DeviceSupport\\ST\\STM32F10x\\stm32f10x.h"




















 
  


   

#line 1454 "..\\Libraries\\CMSIS\\CM3\\DeviceSupport\\ST\\STM32F10x\\stm32f10x.h"



 



 
  
  

 
    
 
 
 

 
 
 
 
 

 



 



 


 
 
 
 
 

 











 
#line 1515 "..\\Libraries\\CMSIS\\CM3\\DeviceSupport\\ST\\STM32F10x\\stm32f10x.h"




 





 
 
 
 
 

 


 


 


 


 


 


 


 


 


 


 


 


 


 


 


 


 


 


 


 


 


 


 


 


 


 


 


 


 


 


 


 


 


 


 


 


 


 


 


 


 


 


 





 



 






 
 
 
 
 

 
#line 1691 "..\\Libraries\\CMSIS\\CM3\\DeviceSupport\\ST\\STM32F10x\\stm32f10x.h"

#line 1698 "..\\Libraries\\CMSIS\\CM3\\DeviceSupport\\ST\\STM32F10x\\stm32f10x.h"

 
 








 








 






#line 1734 "..\\Libraries\\CMSIS\\CM3\\DeviceSupport\\ST\\STM32F10x\\stm32f10x.h"

 











 











 













 






#line 1850 "..\\Libraries\\CMSIS\\CM3\\DeviceSupport\\ST\\STM32F10x\\stm32f10x.h"




#line 1870 "..\\Libraries\\CMSIS\\CM3\\DeviceSupport\\ST\\STM32F10x\\stm32f10x.h"

 





#line 1883 "..\\Libraries\\CMSIS\\CM3\\DeviceSupport\\ST\\STM32F10x\\stm32f10x.h"

 
#line 1902 "..\\Libraries\\CMSIS\\CM3\\DeviceSupport\\ST\\STM32F10x\\stm32f10x.h"

#line 1911 "..\\Libraries\\CMSIS\\CM3\\DeviceSupport\\ST\\STM32F10x\\stm32f10x.h"

 
#line 1919 "..\\Libraries\\CMSIS\\CM3\\DeviceSupport\\ST\\STM32F10x\\stm32f10x.h"



















#line 1944 "..\\Libraries\\CMSIS\\CM3\\DeviceSupport\\ST\\STM32F10x\\stm32f10x.h"












 













#line 1976 "..\\Libraries\\CMSIS\\CM3\\DeviceSupport\\ST\\STM32F10x\\stm32f10x.h"





#line 1990 "..\\Libraries\\CMSIS\\CM3\\DeviceSupport\\ST\\STM32F10x\\stm32f10x.h"

#line 1997 "..\\Libraries\\CMSIS\\CM3\\DeviceSupport\\ST\\STM32F10x\\stm32f10x.h"

#line 2007 "..\\Libraries\\CMSIS\\CM3\\DeviceSupport\\ST\\STM32F10x\\stm32f10x.h"











 


















#line 2043 "..\\Libraries\\CMSIS\\CM3\\DeviceSupport\\ST\\STM32F10x\\stm32f10x.h"

 
#line 2051 "..\\Libraries\\CMSIS\\CM3\\DeviceSupport\\ST\\STM32F10x\\stm32f10x.h"



















#line 2076 "..\\Libraries\\CMSIS\\CM3\\DeviceSupport\\ST\\STM32F10x\\stm32f10x.h"












 













#line 2108 "..\\Libraries\\CMSIS\\CM3\\DeviceSupport\\ST\\STM32F10x\\stm32f10x.h"





#line 2122 "..\\Libraries\\CMSIS\\CM3\\DeviceSupport\\ST\\STM32F10x\\stm32f10x.h"

#line 2129 "..\\Libraries\\CMSIS\\CM3\\DeviceSupport\\ST\\STM32F10x\\stm32f10x.h"

#line 2139 "..\\Libraries\\CMSIS\\CM3\\DeviceSupport\\ST\\STM32F10x\\stm32f10x.h"











 








 








   
#line 2178 "..\\Libraries\\CMSIS\\CM3\\DeviceSupport\\ST\\STM32F10x\\stm32f10x.h"

#line 2273 "..\\Libraries\\CMSIS\\CM3\\DeviceSupport\\ST\\STM32F10x\\stm32f10x.h"

#line 2300 "..\\Libraries\\CMSIS\\CM3\\DeviceSupport\\ST\\STM32F10x\\stm32f10x.h"
 
 
 
 
 
 

 




































































 




































































 
#line 2462 "..\\Libraries\\CMSIS\\CM3\\DeviceSupport\\ST\\STM32F10x\\stm32f10x.h"

 
#line 2480 "..\\Libraries\\CMSIS\\CM3\\DeviceSupport\\ST\\STM32F10x\\stm32f10x.h"

 
#line 2498 "..\\Libraries\\CMSIS\\CM3\\DeviceSupport\\ST\\STM32F10x\\stm32f10x.h"

#line 2515 "..\\Libraries\\CMSIS\\CM3\\DeviceSupport\\ST\\STM32F10x\\stm32f10x.h"

 
#line 2533 "..\\Libraries\\CMSIS\\CM3\\DeviceSupport\\ST\\STM32F10x\\stm32f10x.h"

 
#line 2552 "..\\Libraries\\CMSIS\\CM3\\DeviceSupport\\ST\\STM32F10x\\stm32f10x.h"

 

 






 
#line 2579 "..\\Libraries\\CMSIS\\CM3\\DeviceSupport\\ST\\STM32F10x\\stm32f10x.h"






 








 









 








 








 









 










 




#line 2654 "..\\Libraries\\CMSIS\\CM3\\DeviceSupport\\ST\\STM32F10x\\stm32f10x.h"

 










#line 2685 "..\\Libraries\\CMSIS\\CM3\\DeviceSupport\\ST\\STM32F10x\\stm32f10x.h"

 





 
#line 2700 "..\\Libraries\\CMSIS\\CM3\\DeviceSupport\\ST\\STM32F10x\\stm32f10x.h"

 
#line 2709 "..\\Libraries\\CMSIS\\CM3\\DeviceSupport\\ST\\STM32F10x\\stm32f10x.h"

   
#line 2718 "..\\Libraries\\CMSIS\\CM3\\DeviceSupport\\ST\\STM32F10x\\stm32f10x.h"

 
#line 2727 "..\\Libraries\\CMSIS\\CM3\\DeviceSupport\\ST\\STM32F10x\\stm32f10x.h"

 





 
#line 2742 "..\\Libraries\\CMSIS\\CM3\\DeviceSupport\\ST\\STM32F10x\\stm32f10x.h"

 
#line 2751 "..\\Libraries\\CMSIS\\CM3\\DeviceSupport\\ST\\STM32F10x\\stm32f10x.h"

   
#line 2760 "..\\Libraries\\CMSIS\\CM3\\DeviceSupport\\ST\\STM32F10x\\stm32f10x.h"

 
#line 2769 "..\\Libraries\\CMSIS\\CM3\\DeviceSupport\\ST\\STM32F10x\\stm32f10x.h"

 





 
#line 2784 "..\\Libraries\\CMSIS\\CM3\\DeviceSupport\\ST\\STM32F10x\\stm32f10x.h"

 
#line 2793 "..\\Libraries\\CMSIS\\CM3\\DeviceSupport\\ST\\STM32F10x\\stm32f10x.h"

   
#line 2802 "..\\Libraries\\CMSIS\\CM3\\DeviceSupport\\ST\\STM32F10x\\stm32f10x.h"

 
#line 2811 "..\\Libraries\\CMSIS\\CM3\\DeviceSupport\\ST\\STM32F10x\\stm32f10x.h"

 





 
#line 2826 "..\\Libraries\\CMSIS\\CM3\\DeviceSupport\\ST\\STM32F10x\\stm32f10x.h"

 
#line 2835 "..\\Libraries\\CMSIS\\CM3\\DeviceSupport\\ST\\STM32F10x\\stm32f10x.h"

   
#line 2844 "..\\Libraries\\CMSIS\\CM3\\DeviceSupport\\ST\\STM32F10x\\stm32f10x.h"

 
#line 2853 "..\\Libraries\\CMSIS\\CM3\\DeviceSupport\\ST\\STM32F10x\\stm32f10x.h"

#line 2862 "..\\Libraries\\CMSIS\\CM3\\DeviceSupport\\ST\\STM32F10x\\stm32f10x.h"

#line 2871 "..\\Libraries\\CMSIS\\CM3\\DeviceSupport\\ST\\STM32F10x\\stm32f10x.h"

#line 2881 "..\\Libraries\\CMSIS\\CM3\\DeviceSupport\\ST\\STM32F10x\\stm32f10x.h"

 
 
 
 
 

 





 


 


 




 
 
 
 
 

 
#line 2945 "..\\Libraries\\CMSIS\\CM3\\DeviceSupport\\ST\\STM32F10x\\stm32f10x.h"

 
#line 2980 "..\\Libraries\\CMSIS\\CM3\\DeviceSupport\\ST\\STM32F10x\\stm32f10x.h"

 
#line 3015 "..\\Libraries\\CMSIS\\CM3\\DeviceSupport\\ST\\STM32F10x\\stm32f10x.h"

 
#line 3050 "..\\Libraries\\CMSIS\\CM3\\DeviceSupport\\ST\\STM32F10x\\stm32f10x.h"

 
#line 3085 "..\\Libraries\\CMSIS\\CM3\\DeviceSupport\\ST\\STM32F10x\\stm32f10x.h"

 





 





 





 





 





 





 





 





 






 
#line 3152 "..\\Libraries\\CMSIS\\CM3\\DeviceSupport\\ST\\STM32F10x\\stm32f10x.h"

 



 









 
#line 3176 "..\\Libraries\\CMSIS\\CM3\\DeviceSupport\\ST\\STM32F10x\\stm32f10x.h"




 




 
#line 3192 "..\\Libraries\\CMSIS\\CM3\\DeviceSupport\\ST\\STM32F10x\\stm32f10x.h"

 





 
#line 3214 "..\\Libraries\\CMSIS\\CM3\\DeviceSupport\\ST\\STM32F10x\\stm32f10x.h"

 
 





 
#line 3229 "..\\Libraries\\CMSIS\\CM3\\DeviceSupport\\ST\\STM32F10x\\stm32f10x.h"
 
#line 3236 "..\\Libraries\\CMSIS\\CM3\\DeviceSupport\\ST\\STM32F10x\\stm32f10x.h"

 




 






 


 


 


 
 
 
 
 

 
#line 3285 "..\\Libraries\\CMSIS\\CM3\\DeviceSupport\\ST\\STM32F10x\\stm32f10x.h"

 
#line 3307 "..\\Libraries\\CMSIS\\CM3\\DeviceSupport\\ST\\STM32F10x\\stm32f10x.h"

 
#line 3329 "..\\Libraries\\CMSIS\\CM3\\DeviceSupport\\ST\\STM32F10x\\stm32f10x.h"

 
#line 3351 "..\\Libraries\\CMSIS\\CM3\\DeviceSupport\\ST\\STM32F10x\\stm32f10x.h"

 
#line 3373 "..\\Libraries\\CMSIS\\CM3\\DeviceSupport\\ST\\STM32F10x\\stm32f10x.h"

 
#line 3395 "..\\Libraries\\CMSIS\\CM3\\DeviceSupport\\ST\\STM32F10x\\stm32f10x.h"

 
 
 
 
 

 
#line 3431 "..\\Libraries\\CMSIS\\CM3\\DeviceSupport\\ST\\STM32F10x\\stm32f10x.h"

 
#line 3461 "..\\Libraries\\CMSIS\\CM3\\DeviceSupport\\ST\\STM32F10x\\stm32f10x.h"

 
#line 3471 "..\\Libraries\\CMSIS\\CM3\\DeviceSupport\\ST\\STM32F10x\\stm32f10x.h"















 
#line 3495 "..\\Libraries\\CMSIS\\CM3\\DeviceSupport\\ST\\STM32F10x\\stm32f10x.h"















 
#line 3519 "..\\Libraries\\CMSIS\\CM3\\DeviceSupport\\ST\\STM32F10x\\stm32f10x.h"















 
#line 3543 "..\\Libraries\\CMSIS\\CM3\\DeviceSupport\\ST\\STM32F10x\\stm32f10x.h"















 
#line 3567 "..\\Libraries\\CMSIS\\CM3\\DeviceSupport\\ST\\STM32F10x\\stm32f10x.h"















 
#line 3591 "..\\Libraries\\CMSIS\\CM3\\DeviceSupport\\ST\\STM32F10x\\stm32f10x.h"















 
#line 3615 "..\\Libraries\\CMSIS\\CM3\\DeviceSupport\\ST\\STM32F10x\\stm32f10x.h"















 


 


 


 


 


 


 


 


 


 



 


 


 



 


 


 


 



 


 


 


 


 
 
 
 
 

 






 
#line 3716 "..\\Libraries\\CMSIS\\CM3\\DeviceSupport\\ST\\STM32F10x\\stm32f10x.h"

#line 3725 "..\\Libraries\\CMSIS\\CM3\\DeviceSupport\\ST\\STM32F10x\\stm32f10x.h"















  
 
#line 3748 "..\\Libraries\\CMSIS\\CM3\\DeviceSupport\\ST\\STM32F10x\\stm32f10x.h"


















 








































 


















































 


 


 


 


 


 


 
#line 3883 "..\\Libraries\\CMSIS\\CM3\\DeviceSupport\\ST\\STM32F10x\\stm32f10x.h"

#line 3890 "..\\Libraries\\CMSIS\\CM3\\DeviceSupport\\ST\\STM32F10x\\stm32f10x.h"

#line 3897 "..\\Libraries\\CMSIS\\CM3\\DeviceSupport\\ST\\STM32F10x\\stm32f10x.h"

#line 3904 "..\\Libraries\\CMSIS\\CM3\\DeviceSupport\\ST\\STM32F10x\\stm32f10x.h"







 
#line 3918 "..\\Libraries\\CMSIS\\CM3\\DeviceSupport\\ST\\STM32F10x\\stm32f10x.h"

#line 3925 "..\\Libraries\\CMSIS\\CM3\\DeviceSupport\\ST\\STM32F10x\\stm32f10x.h"

#line 3932 "..\\Libraries\\CMSIS\\CM3\\DeviceSupport\\ST\\STM32F10x\\stm32f10x.h"

#line 3939 "..\\Libraries\\CMSIS\\CM3\\DeviceSupport\\ST\\STM32F10x\\stm32f10x.h"

#line 3946 "..\\Libraries\\CMSIS\\CM3\\DeviceSupport\\ST\\STM32F10x\\stm32f10x.h"

#line 3953 "..\\Libraries\\CMSIS\\CM3\\DeviceSupport\\ST\\STM32F10x\\stm32f10x.h"

 
#line 3961 "..\\Libraries\\CMSIS\\CM3\\DeviceSupport\\ST\\STM32F10x\\stm32f10x.h"

#line 3968 "..\\Libraries\\CMSIS\\CM3\\DeviceSupport\\ST\\STM32F10x\\stm32f10x.h"

#line 3975 "..\\Libraries\\CMSIS\\CM3\\DeviceSupport\\ST\\STM32F10x\\stm32f10x.h"

#line 3982 "..\\Libraries\\CMSIS\\CM3\\DeviceSupport\\ST\\STM32F10x\\stm32f10x.h"

#line 3989 "..\\Libraries\\CMSIS\\CM3\\DeviceSupport\\ST\\STM32F10x\\stm32f10x.h"

#line 3996 "..\\Libraries\\CMSIS\\CM3\\DeviceSupport\\ST\\STM32F10x\\stm32f10x.h"

 
#line 4004 "..\\Libraries\\CMSIS\\CM3\\DeviceSupport\\ST\\STM32F10x\\stm32f10x.h"

#line 4011 "..\\Libraries\\CMSIS\\CM3\\DeviceSupport\\ST\\STM32F10x\\stm32f10x.h"

#line 4018 "..\\Libraries\\CMSIS\\CM3\\DeviceSupport\\ST\\STM32F10x\\stm32f10x.h"

#line 4025 "..\\Libraries\\CMSIS\\CM3\\DeviceSupport\\ST\\STM32F10x\\stm32f10x.h"





 


 


 


 


 



 
 
 
 
 

 









































 



 


 


 


 


 


 


 



 



 



 


 


 



 
 
 
 
 
 





 






 


 
#line 4167 "..\\Libraries\\CMSIS\\CM3\\DeviceSupport\\ST\\STM32F10x\\stm32f10x.h"

 
#line 4177 "..\\Libraries\\CMSIS\\CM3\\DeviceSupport\\ST\\STM32F10x\\stm32f10x.h"

 


 


 
 
 
 
 

 
















 









#line 4225 "..\\Libraries\\CMSIS\\CM3\\DeviceSupport\\ST\\STM32F10x\\stm32f10x.h"

 

























 
#line 4268 "..\\Libraries\\CMSIS\\CM3\\DeviceSupport\\ST\\STM32F10x\\stm32f10x.h"

 
#line 4282 "..\\Libraries\\CMSIS\\CM3\\DeviceSupport\\ST\\STM32F10x\\stm32f10x.h"

 
#line 4292 "..\\Libraries\\CMSIS\\CM3\\DeviceSupport\\ST\\STM32F10x\\stm32f10x.h"

 




























 





















 




























 





















 
#line 4411 "..\\Libraries\\CMSIS\\CM3\\DeviceSupport\\ST\\STM32F10x\\stm32f10x.h"

 


 


 


 


 


 


 


 


 
#line 4446 "..\\Libraries\\CMSIS\\CM3\\DeviceSupport\\ST\\STM32F10x\\stm32f10x.h"





#line 4457 "..\\Libraries\\CMSIS\\CM3\\DeviceSupport\\ST\\STM32F10x\\stm32f10x.h"

 
#line 4465 "..\\Libraries\\CMSIS\\CM3\\DeviceSupport\\ST\\STM32F10x\\stm32f10x.h"

#line 4472 "..\\Libraries\\CMSIS\\CM3\\DeviceSupport\\ST\\STM32F10x\\stm32f10x.h"

 


 
 
 
 
 

 




 
#line 4494 "..\\Libraries\\CMSIS\\CM3\\DeviceSupport\\ST\\STM32F10x\\stm32f10x.h"

 


 


 


 


 


 


 


 


 
 
 
 
 

 


 





 


 



 
 
 
 
 

 
#line 4556 "..\\Libraries\\CMSIS\\CM3\\DeviceSupport\\ST\\STM32F10x\\stm32f10x.h"



 
#line 4568 "..\\Libraries\\CMSIS\\CM3\\DeviceSupport\\ST\\STM32F10x\\stm32f10x.h"







 


 
 
 
 
 

 











#line 4606 "..\\Libraries\\CMSIS\\CM3\\DeviceSupport\\ST\\STM32F10x\\stm32f10x.h"

 











#line 4629 "..\\Libraries\\CMSIS\\CM3\\DeviceSupport\\ST\\STM32F10x\\stm32f10x.h"

 











#line 4652 "..\\Libraries\\CMSIS\\CM3\\DeviceSupport\\ST\\STM32F10x\\stm32f10x.h"

 











#line 4675 "..\\Libraries\\CMSIS\\CM3\\DeviceSupport\\ST\\STM32F10x\\stm32f10x.h"

 








































 








































 








































 








































 


































 


































 


































 


































 



























 



























 



























 
#line 5072 "..\\Libraries\\CMSIS\\CM3\\DeviceSupport\\ST\\STM32F10x\\stm32f10x.h"

 
#line 5081 "..\\Libraries\\CMSIS\\CM3\\DeviceSupport\\ST\\STM32F10x\\stm32f10x.h"

 
#line 5090 "..\\Libraries\\CMSIS\\CM3\\DeviceSupport\\ST\\STM32F10x\\stm32f10x.h"

 
#line 5101 "..\\Libraries\\CMSIS\\CM3\\DeviceSupport\\ST\\STM32F10x\\stm32f10x.h"

#line 5111 "..\\Libraries\\CMSIS\\CM3\\DeviceSupport\\ST\\STM32F10x\\stm32f10x.h"

#line 5121 "..\\Libraries\\CMSIS\\CM3\\DeviceSupport\\ST\\STM32F10x\\stm32f10x.h"

#line 5131 "..\\Libraries\\CMSIS\\CM3\\DeviceSupport\\ST\\STM32F10x\\stm32f10x.h"

 
#line 5142 "..\\Libraries\\CMSIS\\CM3\\DeviceSupport\\ST\\STM32F10x\\stm32f10x.h"

#line 5152 "..\\Libraries\\CMSIS\\CM3\\DeviceSupport\\ST\\STM32F10x\\stm32f10x.h"

#line 5162 "..\\Libraries\\CMSIS\\CM3\\DeviceSupport\\ST\\STM32F10x\\stm32f10x.h"

#line 5172 "..\\Libraries\\CMSIS\\CM3\\DeviceSupport\\ST\\STM32F10x\\stm32f10x.h"

 
#line 5183 "..\\Libraries\\CMSIS\\CM3\\DeviceSupport\\ST\\STM32F10x\\stm32f10x.h"

#line 5193 "..\\Libraries\\CMSIS\\CM3\\DeviceSupport\\ST\\STM32F10x\\stm32f10x.h"

#line 5203 "..\\Libraries\\CMSIS\\CM3\\DeviceSupport\\ST\\STM32F10x\\stm32f10x.h"

#line 5213 "..\\Libraries\\CMSIS\\CM3\\DeviceSupport\\ST\\STM32F10x\\stm32f10x.h"

 
#line 5224 "..\\Libraries\\CMSIS\\CM3\\DeviceSupport\\ST\\STM32F10x\\stm32f10x.h"

#line 5234 "..\\Libraries\\CMSIS\\CM3\\DeviceSupport\\ST\\STM32F10x\\stm32f10x.h"

#line 5244 "..\\Libraries\\CMSIS\\CM3\\DeviceSupport\\ST\\STM32F10x\\stm32f10x.h"

#line 5254 "..\\Libraries\\CMSIS\\CM3\\DeviceSupport\\ST\\STM32F10x\\stm32f10x.h"

 
#line 5265 "..\\Libraries\\CMSIS\\CM3\\DeviceSupport\\ST\\STM32F10x\\stm32f10x.h"

#line 5275 "..\\Libraries\\CMSIS\\CM3\\DeviceSupport\\ST\\STM32F10x\\stm32f10x.h"

#line 5285 "..\\Libraries\\CMSIS\\CM3\\DeviceSupport\\ST\\STM32F10x\\stm32f10x.h"

#line 5295 "..\\Libraries\\CMSIS\\CM3\\DeviceSupport\\ST\\STM32F10x\\stm32f10x.h"

 
#line 5306 "..\\Libraries\\CMSIS\\CM3\\DeviceSupport\\ST\\STM32F10x\\stm32f10x.h"

#line 5316 "..\\Libraries\\CMSIS\\CM3\\DeviceSupport\\ST\\STM32F10x\\stm32f10x.h"

#line 5326 "..\\Libraries\\CMSIS\\CM3\\DeviceSupport\\ST\\STM32F10x\\stm32f10x.h"

#line 5336 "..\\Libraries\\CMSIS\\CM3\\DeviceSupport\\ST\\STM32F10x\\stm32f10x.h"

 
#line 5347 "..\\Libraries\\CMSIS\\CM3\\DeviceSupport\\ST\\STM32F10x\\stm32f10x.h"

#line 5357 "..\\Libraries\\CMSIS\\CM3\\DeviceSupport\\ST\\STM32F10x\\stm32f10x.h"

#line 5367 "..\\Libraries\\CMSIS\\CM3\\DeviceSupport\\ST\\STM32F10x\\stm32f10x.h"

#line 5377 "..\\Libraries\\CMSIS\\CM3\\DeviceSupport\\ST\\STM32F10x\\stm32f10x.h"

 


 


 
 
 
 
 

 




 












 


 






#line 5425 "..\\Libraries\\CMSIS\\CM3\\DeviceSupport\\ST\\STM32F10x\\stm32f10x.h"

 


 


 


 


 


 


 


 


 
















 


 
#line 5495 "..\\Libraries\\CMSIS\\CM3\\DeviceSupport\\ST\\STM32F10x\\stm32f10x.h"

 
#line 5510 "..\\Libraries\\CMSIS\\CM3\\DeviceSupport\\ST\\STM32F10x\\stm32f10x.h"

 
#line 5536 "..\\Libraries\\CMSIS\\CM3\\DeviceSupport\\ST\\STM32F10x\\stm32f10x.h"

 


 


 
 
 
 
 

 
 























 























 























 























 























 























 























 























 
 
#line 5757 "..\\Libraries\\CMSIS\\CM3\\DeviceSupport\\ST\\STM32F10x\\stm32f10x.h"

 
#line 5769 "..\\Libraries\\CMSIS\\CM3\\DeviceSupport\\ST\\STM32F10x\\stm32f10x.h"

 






 
#line 5786 "..\\Libraries\\CMSIS\\CM3\\DeviceSupport\\ST\\STM32F10x\\stm32f10x.h"



     


 
 


 


 


 


 


 


 


 


 

 


 


 


 


 


 


 


 


 

 


 


 


 


 


 


 


 


 


 


 


 


 


 


 


 


 

 


 


 


 


 


 


 


 


 

 


#line 5930 "..\\Libraries\\CMSIS\\CM3\\DeviceSupport\\ST\\STM32F10x\\stm32f10x.h"



 


#line 5942 "..\\Libraries\\CMSIS\\CM3\\DeviceSupport\\ST\\STM32F10x\\stm32f10x.h"



 


#line 5954 "..\\Libraries\\CMSIS\\CM3\\DeviceSupport\\ST\\STM32F10x\\stm32f10x.h"



 


#line 5966 "..\\Libraries\\CMSIS\\CM3\\DeviceSupport\\ST\\STM32F10x\\stm32f10x.h"



 


#line 5978 "..\\Libraries\\CMSIS\\CM3\\DeviceSupport\\ST\\STM32F10x\\stm32f10x.h"



 


#line 5990 "..\\Libraries\\CMSIS\\CM3\\DeviceSupport\\ST\\STM32F10x\\stm32f10x.h"



 


#line 6002 "..\\Libraries\\CMSIS\\CM3\\DeviceSupport\\ST\\STM32F10x\\stm32f10x.h"



 


#line 6014 "..\\Libraries\\CMSIS\\CM3\\DeviceSupport\\ST\\STM32F10x\\stm32f10x.h"



 

 


#line 6028 "..\\Libraries\\CMSIS\\CM3\\DeviceSupport\\ST\\STM32F10x\\stm32f10x.h"



 


#line 6040 "..\\Libraries\\CMSIS\\CM3\\DeviceSupport\\ST\\STM32F10x\\stm32f10x.h"



 


#line 6052 "..\\Libraries\\CMSIS\\CM3\\DeviceSupport\\ST\\STM32F10x\\stm32f10x.h"



 


#line 6064 "..\\Libraries\\CMSIS\\CM3\\DeviceSupport\\ST\\STM32F10x\\stm32f10x.h"



 


#line 6076 "..\\Libraries\\CMSIS\\CM3\\DeviceSupport\\ST\\STM32F10x\\stm32f10x.h"



 


#line 6088 "..\\Libraries\\CMSIS\\CM3\\DeviceSupport\\ST\\STM32F10x\\stm32f10x.h"



 


#line 6100 "..\\Libraries\\CMSIS\\CM3\\DeviceSupport\\ST\\STM32F10x\\stm32f10x.h"



 


#line 6112 "..\\Libraries\\CMSIS\\CM3\\DeviceSupport\\ST\\STM32F10x\\stm32f10x.h"



 


#line 6124 "..\\Libraries\\CMSIS\\CM3\\DeviceSupport\\ST\\STM32F10x\\stm32f10x.h"



 


#line 6136 "..\\Libraries\\CMSIS\\CM3\\DeviceSupport\\ST\\STM32F10x\\stm32f10x.h"



 


#line 6148 "..\\Libraries\\CMSIS\\CM3\\DeviceSupport\\ST\\STM32F10x\\stm32f10x.h"



 


#line 6160 "..\\Libraries\\CMSIS\\CM3\\DeviceSupport\\ST\\STM32F10x\\stm32f10x.h"



 


#line 6172 "..\\Libraries\\CMSIS\\CM3\\DeviceSupport\\ST\\STM32F10x\\stm32f10x.h"



 


#line 6184 "..\\Libraries\\CMSIS\\CM3\\DeviceSupport\\ST\\STM32F10x\\stm32f10x.h"



 


#line 6196 "..\\Libraries\\CMSIS\\CM3\\DeviceSupport\\ST\\STM32F10x\\stm32f10x.h"



 


#line 6208 "..\\Libraries\\CMSIS\\CM3\\DeviceSupport\\ST\\STM32F10x\\stm32f10x.h"



 
 
 
 
 

 
 
#line 6228 "..\\Libraries\\CMSIS\\CM3\\DeviceSupport\\ST\\STM32F10x\\stm32f10x.h"

 
#line 6239 "..\\Libraries\\CMSIS\\CM3\\DeviceSupport\\ST\\STM32F10x\\stm32f10x.h"

 
#line 6257 "..\\Libraries\\CMSIS\\CM3\\DeviceSupport\\ST\\STM32F10x\\stm32f10x.h"











 





 





 
#line 6295 "..\\Libraries\\CMSIS\\CM3\\DeviceSupport\\ST\\STM32F10x\\stm32f10x.h"

 












 
#line 6316 "..\\Libraries\\CMSIS\\CM3\\DeviceSupport\\ST\\STM32F10x\\stm32f10x.h"

 
 






 




 





 





 






 




 





 





 






   




 





 





 





 




 





 





 





 




 





 





 
 


 
#line 6456 "..\\Libraries\\CMSIS\\CM3\\DeviceSupport\\ST\\STM32F10x\\stm32f10x.h"

 
#line 6473 "..\\Libraries\\CMSIS\\CM3\\DeviceSupport\\ST\\STM32F10x\\stm32f10x.h"

 
#line 6490 "..\\Libraries\\CMSIS\\CM3\\DeviceSupport\\ST\\STM32F10x\\stm32f10x.h"

 
#line 6507 "..\\Libraries\\CMSIS\\CM3\\DeviceSupport\\ST\\STM32F10x\\stm32f10x.h"

 
#line 6541 "..\\Libraries\\CMSIS\\CM3\\DeviceSupport\\ST\\STM32F10x\\stm32f10x.h"

 
#line 6575 "..\\Libraries\\CMSIS\\CM3\\DeviceSupport\\ST\\STM32F10x\\stm32f10x.h"

 
#line 6609 "..\\Libraries\\CMSIS\\CM3\\DeviceSupport\\ST\\STM32F10x\\stm32f10x.h"

 
#line 6643 "..\\Libraries\\CMSIS\\CM3\\DeviceSupport\\ST\\STM32F10x\\stm32f10x.h"

 
#line 6677 "..\\Libraries\\CMSIS\\CM3\\DeviceSupport\\ST\\STM32F10x\\stm32f10x.h"

 
#line 6711 "..\\Libraries\\CMSIS\\CM3\\DeviceSupport\\ST\\STM32F10x\\stm32f10x.h"

 
#line 6745 "..\\Libraries\\CMSIS\\CM3\\DeviceSupport\\ST\\STM32F10x\\stm32f10x.h"

 
#line 6779 "..\\Libraries\\CMSIS\\CM3\\DeviceSupport\\ST\\STM32F10x\\stm32f10x.h"

 
#line 6813 "..\\Libraries\\CMSIS\\CM3\\DeviceSupport\\ST\\STM32F10x\\stm32f10x.h"

 
#line 6847 "..\\Libraries\\CMSIS\\CM3\\DeviceSupport\\ST\\STM32F10x\\stm32f10x.h"

 
#line 6881 "..\\Libraries\\CMSIS\\CM3\\DeviceSupport\\ST\\STM32F10x\\stm32f10x.h"

 
#line 6915 "..\\Libraries\\CMSIS\\CM3\\DeviceSupport\\ST\\STM32F10x\\stm32f10x.h"

 
#line 6949 "..\\Libraries\\CMSIS\\CM3\\DeviceSupport\\ST\\STM32F10x\\stm32f10x.h"

 
#line 6983 "..\\Libraries\\CMSIS\\CM3\\DeviceSupport\\ST\\STM32F10x\\stm32f10x.h"

 
#line 7017 "..\\Libraries\\CMSIS\\CM3\\DeviceSupport\\ST\\STM32F10x\\stm32f10x.h"

 
#line 7051 "..\\Libraries\\CMSIS\\CM3\\DeviceSupport\\ST\\STM32F10x\\stm32f10x.h"

 
#line 7085 "..\\Libraries\\CMSIS\\CM3\\DeviceSupport\\ST\\STM32F10x\\stm32f10x.h"

 
#line 7119 "..\\Libraries\\CMSIS\\CM3\\DeviceSupport\\ST\\STM32F10x\\stm32f10x.h"

 
#line 7153 "..\\Libraries\\CMSIS\\CM3\\DeviceSupport\\ST\\STM32F10x\\stm32f10x.h"

 
#line 7187 "..\\Libraries\\CMSIS\\CM3\\DeviceSupport\\ST\\STM32F10x\\stm32f10x.h"

 
#line 7221 "..\\Libraries\\CMSIS\\CM3\\DeviceSupport\\ST\\STM32F10x\\stm32f10x.h"

 
#line 7255 "..\\Libraries\\CMSIS\\CM3\\DeviceSupport\\ST\\STM32F10x\\stm32f10x.h"

 
#line 7289 "..\\Libraries\\CMSIS\\CM3\\DeviceSupport\\ST\\STM32F10x\\stm32f10x.h"

 
#line 7323 "..\\Libraries\\CMSIS\\CM3\\DeviceSupport\\ST\\STM32F10x\\stm32f10x.h"

 
#line 7357 "..\\Libraries\\CMSIS\\CM3\\DeviceSupport\\ST\\STM32F10x\\stm32f10x.h"

 
#line 7391 "..\\Libraries\\CMSIS\\CM3\\DeviceSupport\\ST\\STM32F10x\\stm32f10x.h"

 
#line 7425 "..\\Libraries\\CMSIS\\CM3\\DeviceSupport\\ST\\STM32F10x\\stm32f10x.h"

 
#line 7459 "..\\Libraries\\CMSIS\\CM3\\DeviceSupport\\ST\\STM32F10x\\stm32f10x.h"

 
 
 
 
 

 









#line 7486 "..\\Libraries\\CMSIS\\CM3\\DeviceSupport\\ST\\STM32F10x\\stm32f10x.h"

 
#line 7494 "..\\Libraries\\CMSIS\\CM3\\DeviceSupport\\ST\\STM32F10x\\stm32f10x.h"

 
#line 7504 "..\\Libraries\\CMSIS\\CM3\\DeviceSupport\\ST\\STM32F10x\\stm32f10x.h"

 


 


 


 


 





















 




 
 
 
 
 

 
#line 7565 "..\\Libraries\\CMSIS\\CM3\\DeviceSupport\\ST\\STM32F10x\\stm32f10x.h"

 
#line 7574 "..\\Libraries\\CMSIS\\CM3\\DeviceSupport\\ST\\STM32F10x\\stm32f10x.h"







 



#line 7595 "..\\Libraries\\CMSIS\\CM3\\DeviceSupport\\ST\\STM32F10x\\stm32f10x.h"



 



 


 
#line 7620 "..\\Libraries\\CMSIS\\CM3\\DeviceSupport\\ST\\STM32F10x\\stm32f10x.h"

 
#line 7630 "..\\Libraries\\CMSIS\\CM3\\DeviceSupport\\ST\\STM32F10x\\stm32f10x.h"

 




 


 
 
 
 
 

 
#line 7656 "..\\Libraries\\CMSIS\\CM3\\DeviceSupport\\ST\\STM32F10x\\stm32f10x.h"

 


 



 
#line 7680 "..\\Libraries\\CMSIS\\CM3\\DeviceSupport\\ST\\STM32F10x\\stm32f10x.h"

 
#line 7689 "..\\Libraries\\CMSIS\\CM3\\DeviceSupport\\ST\\STM32F10x\\stm32f10x.h"







 
#line 7709 "..\\Libraries\\CMSIS\\CM3\\DeviceSupport\\ST\\STM32F10x\\stm32f10x.h"

 
#line 7720 "..\\Libraries\\CMSIS\\CM3\\DeviceSupport\\ST\\STM32F10x\\stm32f10x.h"



 
 
 
 
 

 


#line 7749 "..\\Libraries\\CMSIS\\CM3\\DeviceSupport\\ST\\STM32F10x\\stm32f10x.h"

 









#line 7783 "..\\Libraries\\CMSIS\\CM3\\DeviceSupport\\ST\\STM32F10x\\stm32f10x.h"

 
 
 
 
 

 









 


 


 





 
#line 7823 "..\\Libraries\\CMSIS\\CM3\\DeviceSupport\\ST\\STM32F10x\\stm32f10x.h"

 


 









 


 

 



 



 



 



 



 



 



 



#line 8287 "..\\Libraries\\CMSIS\\CM3\\DeviceSupport\\ST\\STM32F10x\\stm32f10x.h"



 

 

  

#line 1 "..\\User\\stm32f10x_conf.h"



















 

 



 
 









#line 1 "..\\Libraries\\STM32F10x_StdPeriph_Driver\\inc\\stm32f10x_flash.h"




















 

 







 
#line 1 "..\\Libraries\\CMSIS\\CM3\\DeviceSupport\\ST\\STM32F10x\\stm32f10x.h"







































 



 



 
    
#line 8327 "..\\Libraries\\CMSIS\\CM3\\DeviceSupport\\ST\\STM32F10x\\stm32f10x.h"



 

  

 

 
#line 33 "..\\Libraries\\STM32F10x_StdPeriph_Driver\\inc\\stm32f10x_flash.h"



 



 



 



 

typedef enum
{ 
  FLASH_BUSY = 1,
  FLASH_ERROR_PG,
  FLASH_ERROR_WRP,
  FLASH_COMPLETE,
  FLASH_TIMEOUT
}FLASH_Status;



 



 



 

#line 77 "..\\Libraries\\STM32F10x_StdPeriph_Driver\\inc\\stm32f10x_flash.h"


 



 







 



 







 



 

 
#line 118 "..\\Libraries\\STM32F10x_StdPeriph_Driver\\inc\\stm32f10x_flash.h"

 
#line 144 "..\\Libraries\\STM32F10x_StdPeriph_Driver\\inc\\stm32f10x_flash.h"

 
#line 211 "..\\Libraries\\STM32F10x_StdPeriph_Driver\\inc\\stm32f10x_flash.h"











 



 







 



 







 



 





#line 270 "..\\Libraries\\STM32F10x_StdPeriph_Driver\\inc\\stm32f10x_flash.h"


 


 
#line 291 "..\\Libraries\\STM32F10x_StdPeriph_Driver\\inc\\stm32f10x_flash.h"






 



 
#line 333 "..\\Libraries\\STM32F10x_StdPeriph_Driver\\inc\\stm32f10x_flash.h"





 
#line 346 "..\\Libraries\\STM32F10x_StdPeriph_Driver\\inc\\stm32f10x_flash.h"



 



 



 



 



 

 
void FLASH_SetLatency(uint32_t FLASH_Latency);
void FLASH_HalfCycleAccessCmd(uint32_t FLASH_HalfCycleAccess);
void FLASH_PrefetchBufferCmd(uint32_t FLASH_PrefetchBuffer);
void FLASH_Unlock(void);
void FLASH_Lock(void);
FLASH_Status FLASH_ErasePage(uint32_t Page_Address);
FLASH_Status FLASH_EraseAllPages(void);
FLASH_Status FLASH_EraseOptionBytes(void);
FLASH_Status FLASH_ProgramWord(uint32_t Address, uint32_t Data);
FLASH_Status FLASH_ProgramHalfWord(uint32_t Address, uint16_t Data);
FLASH_Status FLASH_ProgramOptionByteData(uint32_t Address, uint8_t Data);
FLASH_Status FLASH_EnableWriteProtection(uint32_t FLASH_Pages);
FLASH_Status FLASH_ReadOutProtection(FunctionalState NewState);
FLASH_Status FLASH_UserOptionByteConfig(uint16_t OB_IWDG, uint16_t OB_STOP, uint16_t OB_STDBY);
uint32_t FLASH_GetUserOptionByte(void);
uint32_t FLASH_GetWriteProtectionOptionByte(void);
FlagStatus FLASH_GetReadOutProtectionStatus(void);
FlagStatus FLASH_GetPrefetchBufferStatus(void);
void FLASH_ITConfig(uint32_t FLASH_IT, FunctionalState NewState);
FlagStatus FLASH_GetFlagStatus(uint32_t FLASH_FLAG);
void FLASH_ClearFlag(uint32_t FLASH_FLAG);
FLASH_Status FLASH_GetStatus(void);
FLASH_Status FLASH_WaitForLastOperation(uint32_t Timeout);

 
void FLASH_UnlockBank1(void);
void FLASH_LockBank1(void);
FLASH_Status FLASH_EraseAllBank1Pages(void);
FLASH_Status FLASH_GetBank1Status(void);
FLASH_Status FLASH_WaitForLastBank1Operation(uint32_t Timeout);

#line 408 "..\\Libraries\\STM32F10x_StdPeriph_Driver\\inc\\stm32f10x_flash.h"








 



 



 

 
#line 38 "..\\User\\stm32f10x_conf.h"

#line 1 "..\\Libraries\\STM32F10x_StdPeriph_Driver\\inc\\stm32f10x_gpio.h"




















 

 







 
#line 33 "..\\Libraries\\STM32F10x_StdPeriph_Driver\\inc\\stm32f10x_gpio.h"



 



 



 

#line 53 "..\\Libraries\\STM32F10x_StdPeriph_Driver\\inc\\stm32f10x_gpio.h"
                                     


 

typedef enum
{ 
  GPIO_Speed_10MHz = 1,
  GPIO_Speed_2MHz, 
  GPIO_Speed_50MHz
}GPIOSpeed_TypeDef;





 

typedef enum
{ GPIO_Mode_AIN = 0x0,
  GPIO_Mode_IN_FLOATING = 0x04,
  GPIO_Mode_IPD = 0x28,
  GPIO_Mode_IPU = 0x48,
  GPIO_Mode_Out_OD = 0x14,
  GPIO_Mode_Out_PP = 0x10,
  GPIO_Mode_AF_OD = 0x1C,
  GPIO_Mode_AF_PP = 0x18
}GPIOMode_TypeDef;








 

typedef struct
{
  uint16_t GPIO_Pin;             
 

  GPIOSpeed_TypeDef GPIO_Speed;  
 

  GPIOMode_TypeDef GPIO_Mode;    
 
}GPIO_InitTypeDef;




 

typedef enum
{ Bit_RESET = 0,
  Bit_SET
}BitAction;





 



 



 

#line 144 "..\\Libraries\\STM32F10x_StdPeriph_Driver\\inc\\stm32f10x_gpio.h"



#line 163 "..\\Libraries\\STM32F10x_StdPeriph_Driver\\inc\\stm32f10x_gpio.h"



 



 

#line 204 "..\\Libraries\\STM32F10x_StdPeriph_Driver\\inc\\stm32f10x_gpio.h"







#line 217 "..\\Libraries\\STM32F10x_StdPeriph_Driver\\inc\\stm32f10x_gpio.h"






#line 245 "..\\Libraries\\STM32F10x_StdPeriph_Driver\\inc\\stm32f10x_gpio.h"
                              


  



 

#line 266 "..\\Libraries\\STM32F10x_StdPeriph_Driver\\inc\\stm32f10x_gpio.h"

#line 274 "..\\Libraries\\STM32F10x_StdPeriph_Driver\\inc\\stm32f10x_gpio.h"



 



 

#line 299 "..\\Libraries\\STM32F10x_StdPeriph_Driver\\inc\\stm32f10x_gpio.h"

#line 316 "..\\Libraries\\STM32F10x_StdPeriph_Driver\\inc\\stm32f10x_gpio.h"



 



  








                                                 


 



 



 



 

void GPIO_DeInit(GPIO_TypeDef* GPIOx);
void GPIO_AFIODeInit(void);
void GPIO_Init(GPIO_TypeDef* GPIOx, GPIO_InitTypeDef* GPIO_InitStruct);
void GPIO_StructInit(GPIO_InitTypeDef* GPIO_InitStruct);
uint8_t GPIO_ReadInputDataBit(GPIO_TypeDef* GPIOx, uint16_t GPIO_Pin);
uint16_t GPIO_ReadInputData(GPIO_TypeDef* GPIOx);
uint8_t GPIO_ReadOutputDataBit(GPIO_TypeDef* GPIOx, uint16_t GPIO_Pin);
uint16_t GPIO_ReadOutputData(GPIO_TypeDef* GPIOx);
void GPIO_SetBits(GPIO_TypeDef* GPIOx, uint16_t GPIO_Pin);
void GPIO_ResetBits(GPIO_TypeDef* GPIOx, uint16_t GPIO_Pin);
void GPIO_WriteBit(GPIO_TypeDef* GPIOx, uint16_t GPIO_Pin, BitAction BitVal);
void GPIO_Write(GPIO_TypeDef* GPIOx, uint16_t PortVal);
void GPIO_PinLockConfig(GPIO_TypeDef* GPIOx, uint16_t GPIO_Pin);
void GPIO_EventOutputConfig(uint8_t GPIO_PortSource, uint8_t GPIO_PinSource);
void GPIO_EventOutputCmd(FunctionalState NewState);
void GPIO_PinRemapConfig(uint32_t GPIO_Remap, FunctionalState NewState);
void GPIO_EXTILineConfig(uint8_t GPIO_PortSource, uint8_t GPIO_PinSource);
void GPIO_ETH_MediaInterfaceConfig(uint32_t GPIO_ETH_MediaInterface);








 



 



 

 
#line 40 "..\\User\\stm32f10x_conf.h"



#line 1 "..\\Libraries\\STM32F10x_StdPeriph_Driver\\inc\\stm32f10x_rcc.h"




















 

 







 
#line 33 "..\\Libraries\\STM32F10x_StdPeriph_Driver\\inc\\stm32f10x_rcc.h"



 



 



 

typedef struct
{
  uint32_t SYSCLK_Frequency;   
  uint32_t HCLK_Frequency;     
  uint32_t PCLK1_Frequency;    
  uint32_t PCLK2_Frequency;    
  uint32_t ADCCLK_Frequency;   
}RCC_ClocksTypeDef;



 



 



 









  



 



#line 94 "..\\Libraries\\STM32F10x_StdPeriph_Driver\\inc\\stm32f10x_rcc.h"



  



 
#line 126 "..\\Libraries\\STM32F10x_StdPeriph_Driver\\inc\\stm32f10x_rcc.h"

#line 141 "..\\Libraries\\STM32F10x_StdPeriph_Driver\\inc\\stm32f10x_rcc.h"


 



 
#line 175 "..\\Libraries\\STM32F10x_StdPeriph_Driver\\inc\\stm32f10x_rcc.h"


 




 
#line 196 "..\\Libraries\\STM32F10x_StdPeriph_Driver\\inc\\stm32f10x_rcc.h"


 

#line 283 "..\\Libraries\\STM32F10x_StdPeriph_Driver\\inc\\stm32f10x_rcc.h"




 

#line 295 "..\\Libraries\\STM32F10x_StdPeriph_Driver\\inc\\stm32f10x_rcc.h"


 



 

#line 317 "..\\Libraries\\STM32F10x_StdPeriph_Driver\\inc\\stm32f10x_rcc.h"


  



 

#line 333 "..\\Libraries\\STM32F10x_StdPeriph_Driver\\inc\\stm32f10x_rcc.h"


 



 

#line 347 "..\\Libraries\\STM32F10x_StdPeriph_Driver\\inc\\stm32f10x_rcc.h"

#line 364 "..\\Libraries\\STM32F10x_StdPeriph_Driver\\inc\\stm32f10x_rcc.h"




 




 








 
#line 396 "..\\Libraries\\STM32F10x_StdPeriph_Driver\\inc\\stm32f10x_rcc.h"


#line 423 "..\\Libraries\\STM32F10x_StdPeriph_Driver\\inc\\stm32f10x_rcc.h"
  



 

#line 435 "..\\Libraries\\STM32F10x_StdPeriph_Driver\\inc\\stm32f10x_rcc.h"


 



 








 



 

#line 462 "..\\Libraries\\STM32F10x_StdPeriph_Driver\\inc\\stm32f10x_rcc.h"


 



 







#line 489 "..\\Libraries\\STM32F10x_StdPeriph_Driver\\inc\\stm32f10x_rcc.h"


 



 

#line 518 "..\\Libraries\\STM32F10x_StdPeriph_Driver\\inc\\stm32f10x_rcc.h"




  



 

#line 553 "..\\Libraries\\STM32F10x_StdPeriph_Driver\\inc\\stm32f10x_rcc.h"
 




 



 







#line 586 "..\\Libraries\\STM32F10x_StdPeriph_Driver\\inc\\stm32f10x_rcc.h"



 



 

#line 606 "..\\Libraries\\STM32F10x_StdPeriph_Driver\\inc\\stm32f10x_rcc.h"

#line 625 "..\\Libraries\\STM32F10x_StdPeriph_Driver\\inc\\stm32f10x_rcc.h"




 



 



 



 



 

void RCC_DeInit(void);
void RCC_HSEConfig(uint32_t RCC_HSE);
ErrorStatus RCC_WaitForHSEStartUp(void);
void RCC_AdjustHSICalibrationValue(uint8_t HSICalibrationValue);
void RCC_HSICmd(FunctionalState NewState);
void RCC_PLLConfig(uint32_t RCC_PLLSource, uint32_t RCC_PLLMul);
void RCC_PLLCmd(FunctionalState NewState);





#line 666 "..\\Libraries\\STM32F10x_StdPeriph_Driver\\inc\\stm32f10x_rcc.h"

void RCC_SYSCLKConfig(uint32_t RCC_SYSCLKSource);
uint8_t RCC_GetSYSCLKSource(void);
void RCC_HCLKConfig(uint32_t RCC_SYSCLK);
void RCC_PCLK1Config(uint32_t RCC_HCLK);
void RCC_PCLK2Config(uint32_t RCC_HCLK);
void RCC_ITConfig(uint8_t RCC_IT, FunctionalState NewState);


 void RCC_USBCLKConfig(uint32_t RCC_USBCLKSource);




void RCC_ADCCLKConfig(uint32_t RCC_PCLK2);






void RCC_LSEConfig(uint8_t RCC_LSE);
void RCC_LSICmd(FunctionalState NewState);
void RCC_RTCCLKConfig(uint32_t RCC_RTCCLKSource);
void RCC_RTCCLKCmd(FunctionalState NewState);
void RCC_GetClocksFreq(RCC_ClocksTypeDef* RCC_Clocks);
void RCC_AHBPeriphClockCmd(uint32_t RCC_AHBPeriph, FunctionalState NewState);
void RCC_APB2PeriphClockCmd(uint32_t RCC_APB2Periph, FunctionalState NewState);
void RCC_APB1PeriphClockCmd(uint32_t RCC_APB1Periph, FunctionalState NewState);





void RCC_APB2PeriphResetCmd(uint32_t RCC_APB2Periph, FunctionalState NewState);
void RCC_APB1PeriphResetCmd(uint32_t RCC_APB1Periph, FunctionalState NewState);
void RCC_BackupResetCmd(FunctionalState NewState);
void RCC_ClockSecuritySystemCmd(FunctionalState NewState);
void RCC_MCOConfig(uint8_t RCC_MCO);
FlagStatus RCC_GetFlagStatus(uint8_t RCC_FLAG);
void RCC_ClearFlag(void);
ITStatus RCC_GetITStatus(uint8_t RCC_IT);
void RCC_ClearITPendingBit(uint8_t RCC_IT);








 



 



  

 
#line 44 "..\\User\\stm32f10x_conf.h"






#line 1 "..\\Libraries\\STM32F10x_StdPeriph_Driver\\inc\\misc.h"




















 

 







 
#line 33 "..\\Libraries\\STM32F10x_StdPeriph_Driver\\inc\\misc.h"



 



 



 



 

typedef struct
{
  uint8_t NVIC_IRQChannel;                    


 

  uint8_t NVIC_IRQChannelPreemptionPriority;  

 

  uint8_t NVIC_IRQChannelSubPriority;         

 

  FunctionalState NVIC_IRQChannelCmd;         

    
} NVIC_InitTypeDef;
 


 



 
























 



 



 



 







 



 

#line 133 "..\\Libraries\\STM32F10x_StdPeriph_Driver\\inc\\misc.h"


 



 

#line 151 "..\\Libraries\\STM32F10x_StdPeriph_Driver\\inc\\misc.h"















 



 







 



 



 



 



 

void NVIC_PriorityGroupConfig(uint32_t NVIC_PriorityGroup);
void NVIC_Init(NVIC_InitTypeDef* NVIC_InitStruct);
void NVIC_SetVectorTable(uint32_t NVIC_VectTab, uint32_t Offset);
void NVIC_SystemLPConfig(uint8_t LowPowerMode, FunctionalState NewState);
void SysTick_CLKSourceConfig(uint32_t SysTick_CLKSource);









 



 



 

 
#line 51 "..\\User\\stm32f10x_conf.h"

 
 

 
 

 
#line 74 "..\\User\\stm32f10x_conf.h"



 
#line 8298 "..\\Libraries\\CMSIS\\CM3\\DeviceSupport\\ST\\STM32F10x\\stm32f10x.h"




 

















 









 

  

 

 
#line 33 "..\\User\\main.c"
#line 1 "d:\\Keil\\ARM\\ARMCC\\bin\\..\\include\\stdio.h"
 
 
 





 






 













#line 38 "d:\\Keil\\ARM\\ARMCC\\bin\\..\\include\\stdio.h"


  
  typedef unsigned int size_t;    








 
 

 
  typedef struct __va_list __va_list;





   




 




typedef struct __fpos_t_struct {
    unsigned __int64 __pos;
    



 
    struct {
        unsigned int __state1, __state2;
    } __mbstate;
} fpos_t;
   


 


   

 

typedef struct __FILE FILE;
   






 

extern FILE __stdin, __stdout, __stderr;
extern FILE *__aeabi_stdin, *__aeabi_stdout, *__aeabi_stderr;

#line 129 "d:\\Keil\\ARM\\ARMCC\\bin\\..\\include\\stdio.h"
    

    

    





     



   


 


   


 

   



 

   


 




   


 





    


 






extern __declspec(__nothrow) int remove(const char *  ) __attribute__((__nonnull__(1)));
   





 
extern __declspec(__nothrow) int rename(const char *  , const char *  ) __attribute__((__nonnull__(1,2)));
   








 
extern __declspec(__nothrow) FILE *tmpfile(void);
   




 
extern __declspec(__nothrow) char *tmpnam(char *  );
   











 

extern __declspec(__nothrow) int fclose(FILE *  ) __attribute__((__nonnull__(1)));
   







 
extern __declspec(__nothrow) int fflush(FILE *  );
   







 
extern __declspec(__nothrow) FILE *fopen(const char * __restrict  ,
                           const char * __restrict  ) __attribute__((__nonnull__(1,2)));
   








































 
extern __declspec(__nothrow) FILE *freopen(const char * __restrict  ,
                    const char * __restrict  ,
                    FILE * __restrict  ) __attribute__((__nonnull__(2,3)));
   








 
extern __declspec(__nothrow) void setbuf(FILE * __restrict  ,
                    char * __restrict  ) __attribute__((__nonnull__(1)));
   




 
extern __declspec(__nothrow) int setvbuf(FILE * __restrict  ,
                   char * __restrict  ,
                   int  , size_t  ) __attribute__((__nonnull__(1)));
   















 
#pragma __printf_args
extern __declspec(__nothrow) int fprintf(FILE * __restrict  ,
                    const char * __restrict  , ...) __attribute__((__nonnull__(1,2)));
   


















 
#pragma __printf_args
extern __declspec(__nothrow) int _fprintf(FILE * __restrict  ,
                     const char * __restrict  , ...) __attribute__((__nonnull__(1,2)));
   



 
#pragma __printf_args
extern __declspec(__nothrow) int printf(const char * __restrict  , ...) __attribute__((__nonnull__(1)));
   




 
#pragma __printf_args
extern __declspec(__nothrow) int _printf(const char * __restrict  , ...) __attribute__((__nonnull__(1)));
   



 
#pragma __printf_args
extern __declspec(__nothrow) int sprintf(char * __restrict  , const char * __restrict  , ...) __attribute__((__nonnull__(1,2)));
   






 
#pragma __printf_args
extern __declspec(__nothrow) int _sprintf(char * __restrict  , const char * __restrict  , ...) __attribute__((__nonnull__(1,2)));
   



 

#pragma __printf_args
extern __declspec(__nothrow) int snprintf(char * __restrict  , size_t  ,
                     const char * __restrict  , ...) __attribute__((__nonnull__(3)));
   















 

#pragma __printf_args
extern __declspec(__nothrow) int _snprintf(char * __restrict  , size_t  ,
                      const char * __restrict  , ...) __attribute__((__nonnull__(3)));
   



 
#pragma __scanf_args
extern __declspec(__nothrow) int fscanf(FILE * __restrict  ,
                    const char * __restrict  , ...) __attribute__((__nonnull__(1,2)));
   






























 
#pragma __scanf_args
extern __declspec(__nothrow) int _fscanf(FILE * __restrict  ,
                     const char * __restrict  , ...) __attribute__((__nonnull__(1,2)));
   



 
#pragma __scanf_args
extern __declspec(__nothrow) int scanf(const char * __restrict  , ...) __attribute__((__nonnull__(1)));
   






 
#pragma __scanf_args
extern __declspec(__nothrow) int _scanf(const char * __restrict  , ...) __attribute__((__nonnull__(1)));
   



 
#pragma __scanf_args
extern __declspec(__nothrow) int sscanf(const char * __restrict  ,
                    const char * __restrict  , ...) __attribute__((__nonnull__(1,2)));
   








 
#pragma __scanf_args
extern __declspec(__nothrow) int _sscanf(const char * __restrict  ,
                     const char * __restrict  , ...) __attribute__((__nonnull__(1,2)));
   



 

 
extern __declspec(__nothrow) int vfscanf(FILE * __restrict  , const char * __restrict  , __va_list) __attribute__((__nonnull__(1,2)));
extern __declspec(__nothrow) int vscanf(const char * __restrict  , __va_list) __attribute__((__nonnull__(1)));
extern __declspec(__nothrow) int vsscanf(const char * __restrict  , const char * __restrict  , __va_list) __attribute__((__nonnull__(1,2)));

extern __declspec(__nothrow) int _vfscanf(FILE * __restrict  , const char * __restrict  , __va_list) __attribute__((__nonnull__(1,2)));
extern __declspec(__nothrow) int _vscanf(const char * __restrict  , __va_list) __attribute__((__nonnull__(1)));
extern __declspec(__nothrow) int _vsscanf(const char * __restrict  , const char * __restrict  , __va_list) __attribute__((__nonnull__(1,2)));

extern __declspec(__nothrow) int vprintf(const char * __restrict  , __va_list  ) __attribute__((__nonnull__(1)));
   





 
extern __declspec(__nothrow) int _vprintf(const char * __restrict  , __va_list  ) __attribute__((__nonnull__(1)));
   



 
extern __declspec(__nothrow) int vfprintf(FILE * __restrict  ,
                    const char * __restrict  , __va_list  ) __attribute__((__nonnull__(1,2)));
   






 
extern __declspec(__nothrow) int vsprintf(char * __restrict  ,
                     const char * __restrict  , __va_list  ) __attribute__((__nonnull__(1,2)));
   






 

extern __declspec(__nothrow) int vsnprintf(char * __restrict  , size_t  ,
                     const char * __restrict  , __va_list  ) __attribute__((__nonnull__(3)));
   







 

extern __declspec(__nothrow) int _vsprintf(char * __restrict  ,
                      const char * __restrict  , __va_list  ) __attribute__((__nonnull__(1,2)));
   



 
extern __declspec(__nothrow) int _vfprintf(FILE * __restrict  ,
                     const char * __restrict  , __va_list  ) __attribute__((__nonnull__(1,2)));
   



 
extern __declspec(__nothrow) int _vsnprintf(char * __restrict  , size_t  ,
                      const char * __restrict  , __va_list  ) __attribute__((__nonnull__(3)));
   



 
extern __declspec(__nothrow) int fgetc(FILE *  ) __attribute__((__nonnull__(1)));
   







 
extern __declspec(__nothrow) char *fgets(char * __restrict  , int  ,
                    FILE * __restrict  ) __attribute__((__nonnull__(1,3)));
   










 
extern __declspec(__nothrow) int fputc(int  , FILE *  ) __attribute__((__nonnull__(2)));
   







 
extern __declspec(__nothrow) int fputs(const char * __restrict  , FILE * __restrict  ) __attribute__((__nonnull__(1,2)));
   




 
extern __declspec(__nothrow) int getc(FILE *  ) __attribute__((__nonnull__(1)));
   







 




    extern __declspec(__nothrow) int (getchar)(void);

   





 
extern __declspec(__nothrow) char *gets(char *  ) __attribute__((__nonnull__(1)));
   









 
extern __declspec(__nothrow) int putc(int  , FILE *  ) __attribute__((__nonnull__(2)));
   





 




    extern __declspec(__nothrow) int (putchar)(int  );

   



 
extern __declspec(__nothrow) int puts(const char *  ) __attribute__((__nonnull__(1)));
   





 
extern __declspec(__nothrow) int ungetc(int  , FILE *  ) __attribute__((__nonnull__(2)));
   






















 

extern __declspec(__nothrow) size_t fread(void * __restrict  ,
                    size_t  , size_t  , FILE * __restrict  ) __attribute__((__nonnull__(1,4)));
   











 

extern __declspec(__nothrow) size_t __fread_bytes_avail(void * __restrict  ,
                    size_t  , FILE * __restrict  ) __attribute__((__nonnull__(1,3)));
   











 

extern __declspec(__nothrow) size_t fwrite(const void * __restrict  ,
                    size_t  , size_t  , FILE * __restrict  ) __attribute__((__nonnull__(1,4)));
   







 

extern __declspec(__nothrow) int fgetpos(FILE * __restrict  , fpos_t * __restrict  ) __attribute__((__nonnull__(1,2)));
   








 
extern __declspec(__nothrow) int fseek(FILE *  , long int  , int  ) __attribute__((__nonnull__(1)));
   














 
extern __declspec(__nothrow) int fsetpos(FILE * __restrict  , const fpos_t * __restrict  ) __attribute__((__nonnull__(1,2)));
   










 
extern __declspec(__nothrow) long int ftell(FILE *  ) __attribute__((__nonnull__(1)));
   











 
extern __declspec(__nothrow) void rewind(FILE *  ) __attribute__((__nonnull__(1)));
   





 

extern __declspec(__nothrow) void clearerr(FILE *  ) __attribute__((__nonnull__(1)));
   




 

extern __declspec(__nothrow) int feof(FILE *  ) __attribute__((__nonnull__(1)));
   


 
extern __declspec(__nothrow) int ferror(FILE *  ) __attribute__((__nonnull__(1)));
   


 
extern __declspec(__nothrow) void perror(const char *  );
   









 

extern __declspec(__nothrow) int _fisatty(FILE *   ) __attribute__((__nonnull__(1)));
    
 

extern __declspec(__nothrow) void __use_no_semihosting_swi(void);
extern __declspec(__nothrow) void __use_no_semihosting(void);
    





 











#line 948 "d:\\Keil\\ARM\\ARMCC\\bin\\..\\include\\stdio.h"



 

#line 34 "..\\User\\main.c"
#line 1 "..\\BaseDrive\\SystemClock.h"



void RCC_Configuration(void);

#line 35 "..\\User\\main.c"
#line 1 "..\\BaseDrive\\Gpio_Led.h"



 
#line 19 "..\\BaseDrive\\Gpio_Led.h"

 




 




 




typedef enum
{
    false=0,
	true
}Bool;

typedef struct
{
    Bool Key1_state;	
	Bool Key2_state;	
	Bool Key3_state;	
	u32  Counter1;      
	u32  Counter2;      
}Key_Info;

void GpioLed_Init(void);
void LED_Display(void);
void Key_Init(void);
void Key_Test(void);
void WriteData_8(u8 Data_8);
u8 ReadData_8(void);


#line 36 "..\\User\\main.c"




u8 AAA;

 
int main(void)
{
	RCC_Configuration(); 
	GpioLed_Init();		 





	while (1)
	{

	LED_Display();	 
	GPIO_Write(((GPIO_TypeDef *) ((((uint32_t)0x40000000) + 0x10000) + 0x1C00)),0xffff);
	GPIO_Write(((GPIO_TypeDef *) ((((uint32_t)0x40000000) + 0x10000) + 0x1C00)),0x00);
	WriteData_8(0xaa);
	AAA = GPIO_ReadInputData(((GPIO_TypeDef *) ((((uint32_t)0x40000000) + 0x10000) + 0x1C00)));






	}
}

