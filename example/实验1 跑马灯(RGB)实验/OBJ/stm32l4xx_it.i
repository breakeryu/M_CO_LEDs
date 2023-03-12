#line 1 "stm32l4xx_it.c"



































 

 
#line 1 "main.h"

































 
  
 



 
#line 1 "..\\HALLIB\\STM32L4xx_HAL_Driver\\Inc\\stm32l4xx_hal.h"


































 

 







 
#line 1 "..\\USER\\stm32l4xx_hal_conf.h"
































  

 







#line 1 "..\\USER\\main.h"

































 
  
 
#line 49 "..\\USER\\main.h"

 
#line 44 "..\\USER\\stm32l4xx_hal_conf.h"
 
 

 


 

#line 96 "..\\USER\\stm32l4xx_hal_conf.h"

#line 104 "..\\USER\\stm32l4xx_hal_conf.h"

 




 











 







 










 







 









 












 








 





 

 


      
  
#line 200 "..\\USER\\stm32l4xx_hal_conf.h"

 



 
 

 




 



 


 

#line 1 "..\\HALLIB\\STM32L4xx_HAL_Driver\\Inc\\stm32l4xx_hal_rcc.h"

































 

 







 
#line 1 "..\\HALLIB\\STM32L4xx_HAL_Driver\\Inc\\stm32l4xx_hal_def.h"


































 

 







 
#line 1 "..\\USER\\stm32l4xx.h"











































 



 



 










 



 






 

#line 106 "..\\USER\\stm32l4xx.h"



 
#line 118 "..\\USER\\stm32l4xx.h"



 
#line 130 "..\\USER\\stm32l4xx.h"



 



 

#line 1 "..\\USER\\stm32l475xx.h"







































 



 



 










 



 








 



 




 
typedef enum
{
 
  NonMaskableInt_IRQn         = -14,     
  HardFault_IRQn              = -13,     
  MemoryManagement_IRQn       = -12,     
  BusFault_IRQn               = -11,     
  UsageFault_IRQn             = -10,     
  SVCall_IRQn                 = -5,      
  DebugMonitor_IRQn           = -4,      
  PendSV_IRQn                 = -2,      
  SysTick_IRQn                = -1,      
 
  WWDG_IRQn                   = 0,       
  PVD_PVM_IRQn                = 1,       
  TAMP_STAMP_IRQn             = 2,       
  RTC_WKUP_IRQn               = 3,       
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
  ADC1_2_IRQn                 = 18,      
  CAN1_TX_IRQn                = 19,      
  CAN1_RX0_IRQn               = 20,      
  CAN1_RX1_IRQn               = 21,      
  CAN1_SCE_IRQn               = 22,      
  EXTI9_5_IRQn                = 23,      
  TIM1_BRK_TIM15_IRQn         = 24,      
  TIM1_UP_TIM16_IRQn          = 25,      
  TIM1_TRG_COM_TIM17_IRQn     = 26,      
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
  RTC_Alarm_IRQn              = 41,      
  DFSDM1_FLT3_IRQn            = 42,      
  TIM8_BRK_IRQn               = 43,      
  TIM8_UP_IRQn                = 44,      
  TIM8_TRG_COM_IRQn           = 45,      
  TIM8_CC_IRQn                = 46,      
  ADC3_IRQn                   = 47,      
  FMC_IRQn                    = 48,      
  SDMMC1_IRQn                 = 49,      
  TIM5_IRQn                   = 50,      
  SPI3_IRQn                   = 51,      
  UART4_IRQn                  = 52,      
  UART5_IRQn                  = 53,      
  TIM6_DAC_IRQn               = 54,      
  TIM7_IRQn                   = 55,      
  DMA2_Channel1_IRQn          = 56,      
  DMA2_Channel2_IRQn          = 57,      
  DMA2_Channel3_IRQn          = 58,      
  DMA2_Channel4_IRQn          = 59,      
  DMA2_Channel5_IRQn          = 60,      
  DFSDM1_FLT0_IRQn            = 61,      
  DFSDM1_FLT1_IRQn            = 62,      
  DFSDM1_FLT2_IRQn            = 63,      
  COMP_IRQn                   = 64,      
  LPTIM1_IRQn                 = 65,      
  LPTIM2_IRQn                 = 66,      
  OTG_FS_IRQn                 = 67,      
  DMA2_Channel6_IRQn          = 68,      
  DMA2_Channel7_IRQn          = 69,      
  LPUART1_IRQn                = 70,      
  QUADSPI_IRQn                = 71,      
  I2C3_EV_IRQn                = 72,      
  I2C3_ER_IRQn                = 73,      
  SAI1_IRQn                   = 74,      
  SAI2_IRQn                   = 75,      
  SWPMI1_IRQn                 = 76,      
  TSC_IRQn                    = 77,      
  RNG_IRQn                    = 80,      
  FPU_IRQn                    = 81       
} IRQn_Type;



 

#line 1 "..\\CORE\\core_cm4.h"
 




 

























 











#line 1 "F:\\KEIL5\\ARM\\ARMCC\\Bin\\..\\include\\stdint.h"
 
 





 









     
#line 27 "F:\\KEIL5\\ARM\\ARMCC\\Bin\\..\\include\\stdint.h"
     











#line 46 "F:\\KEIL5\\ARM\\ARMCC\\Bin\\..\\include\\stdint.h"





 

     

     
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


     
typedef   signed     long long intmax_t;
typedef unsigned     long long uintmax_t;




     

     





     





     





     

     





     





     





     

     





     





     





     

     






     






     






     

     


     


     


     

     
#line 216 "F:\\KEIL5\\ARM\\ARMCC\\Bin\\..\\include\\stdint.h"

     



     






     
    
 



#line 241 "F:\\KEIL5\\ARM\\ARMCC\\Bin\\..\\include\\stdint.h"

     







     










     











#line 305 "F:\\KEIL5\\ARM\\ARMCC\\Bin\\..\\include\\stdint.h"






 
#line 45 "..\\CORE\\core_cm4.h"

















 




 



 

 













#line 120 "..\\CORE\\core_cm4.h"



 
#line 135 "..\\CORE\\core_cm4.h"

#line 209 "..\\CORE\\core_cm4.h"

#line 1 "..\\CORE\\core_cmInstr.h"
 




 

























 












 



 

 
#line 1 "..\\CORE\\cmsis_armcc.h"
 




 

























 










 



 

 
 





 
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






 
static __inline uint32_t __get_IPSR(void)
{
  register uint32_t __regIPSR          __asm("ipsr");
  return(__regIPSR);
}






 
static __inline uint32_t __get_APSR(void)
{
  register uint32_t __regAPSR          __asm("apsr");
  return(__regAPSR);
}






 
static __inline uint32_t __get_xPSR(void)
{
  register uint32_t __regXPSR          __asm("xpsr");
  return(__regXPSR);
}






 
static __inline uint32_t __get_PSP(void)
{
  register uint32_t __regProcessStackPointer  __asm("psp");
  return(__regProcessStackPointer);
}






 
static __inline void __set_PSP(uint32_t topOfProcStack)
{
  register uint32_t __regProcessStackPointer  __asm("psp");
  __regProcessStackPointer = topOfProcStack;
}






 
static __inline uint32_t __get_MSP(void)
{
  register uint32_t __regMainStackPointer     __asm("msp");
  return(__regMainStackPointer);
}






 
static __inline void __set_MSP(uint32_t topOfMainStack)
{
  register uint32_t __regMainStackPointer     __asm("msp");
  __regMainStackPointer = topOfMainStack;
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








 







 







 
static __inline uint32_t  __get_BASEPRI(void)
{
  register uint32_t __regBasePri         __asm("basepri");
  return(__regBasePri);
}






 
static __inline void __set_BASEPRI(uint32_t basePri)
{
  register uint32_t __regBasePri         __asm("basepri");
  __regBasePri = (basePri & 0xFFU);
}







 
static __inline void __set_BASEPRI_MAX(uint32_t basePri)
{
  register uint32_t __regBasePriMax      __asm("basepri_max");
  __regBasePriMax = (basePri & 0xFFU);
}






 
static __inline uint32_t __get_FAULTMASK(void)
{
  register uint32_t __regFaultMask       __asm("faultmask");
  return(__regFaultMask);
}






 
static __inline void __set_FAULTMASK(uint32_t faultMask)
{
  register uint32_t __regFaultMask       __asm("faultmask");
  __regFaultMask = (faultMask & (uint32_t)1);
}










 
static __inline uint32_t __get_FPSCR(void)
{

  register uint32_t __regfpscr         __asm("fpscr");
  return(__regfpscr);



}






 
static __inline void __set_FPSCR(uint32_t fpscr)
{

  register uint32_t __regfpscr         __asm("fpscr");
  __regfpscr = (fpscr);

}





 


 



 




 






 







 






 








 










 










 











 








 

__attribute__((section(".rev16_text"))) static __inline __asm uint32_t __REV16(uint32_t value)
{
  rev16 r0, r0
  bx lr
}







 

__attribute__((section(".revsh_text"))) static __inline __asm int32_t __REVSH(int32_t value)
{
  revsh r0, r0
  bx lr
}









 









 








 
#line 455 "..\\CORE\\cmsis_armcc.h"







 










 












 












 














 














 














 










 









 









 









 

__attribute__((section(".rrx_text"))) static __inline __asm uint32_t __RRX(uint32_t value)
{
  rrx r0, r0
  bx lr
}








 








 








 








 








 








 




   


 



 



#line 720 "..\\CORE\\cmsis_armcc.h"











 


#line 54 "..\\CORE\\core_cmInstr.h"

 
#line 84 "..\\CORE\\core_cmInstr.h"

   

#line 211 "..\\CORE\\core_cm4.h"
#line 1 "..\\CORE\\core_cmFunc.h"
 




 

























 












 



 

 
#line 54 "..\\CORE\\core_cmFunc.h"

 
#line 84 "..\\CORE\\core_cmFunc.h"

 

#line 212 "..\\CORE\\core_cm4.h"
#line 1 "..\\CORE\\core_cmSimd.h"
 




 

























 
















 



 

 
#line 58 "..\\CORE\\core_cmSimd.h"

 
#line 88 "..\\CORE\\core_cmSimd.h"

 






#line 213 "..\\CORE\\core_cm4.h"
















 
#line 256 "..\\CORE\\core_cm4.h"

 






 
#line 272 "..\\CORE\\core_cm4.h"

 




 













 



 






 



 
typedef union
{
  struct
  {
    uint32_t _reserved0:16;               
    uint32_t GE:4;                        
    uint32_t _reserved1:7;                
    uint32_t Q:1;                         
    uint32_t V:1;                         
    uint32_t C:1;                         
    uint32_t Z:1;                         
    uint32_t N:1;                         
  } b;                                    
  uint32_t w;                             
} APSR_Type;

 





















 
typedef union
{
  struct
  {
    uint32_t ISR:9;                       
    uint32_t _reserved0:23;               
  } b;                                    
  uint32_t w;                             
} IPSR_Type;

 






 
typedef union
{
  struct
  {
    uint32_t ISR:9;                       
    uint32_t _reserved0:7;                
    uint32_t GE:4;                        
    uint32_t _reserved1:4;                
    uint32_t T:1;                         
    uint32_t IT:2;                        
    uint32_t Q:1;                         
    uint32_t V:1;                         
    uint32_t C:1;                         
    uint32_t Z:1;                         
    uint32_t N:1;                         
  } b;                                    
  uint32_t w;                             
} xPSR_Type;

 






























 
typedef union
{
  struct
  {
    uint32_t nPRIV:1;                     
    uint32_t SPSEL:1;                     
    uint32_t FPCA:1;                      
    uint32_t _reserved0:29;               
  } b;                                    
  uint32_t w;                             
} CONTROL_Type;

 









 







 



 
typedef struct
{
  volatile uint32_t ISER[8U];                
        uint32_t RESERVED0[24U];
  volatile uint32_t ICER[8U];                
        uint32_t RSERVED1[24U];
  volatile uint32_t ISPR[8U];                
        uint32_t RESERVED2[24U];
  volatile uint32_t ICPR[8U];                
        uint32_t RESERVED3[24U];
  volatile uint32_t IABR[8U];                
        uint32_t RESERVED4[56U];
  volatile uint8_t  IP[240U];                
        uint32_t RESERVED5[644U];
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
  volatile uint8_t  SHP[12U];                
  volatile uint32_t SHCSR;                   
  volatile uint32_t CFSR;                    
  volatile uint32_t HFSR;                    
  volatile uint32_t DFSR;                    
  volatile uint32_t MMFAR;                   
  volatile uint32_t BFAR;                    
  volatile uint32_t AFSR;                    
  volatile const  uint32_t PFR[2U];                 
  volatile const  uint32_t DFR;                     
  volatile const  uint32_t ADR;                     
  volatile const  uint32_t MMFR[4U];                
  volatile const  uint32_t ISAR[5U];                
        uint32_t RESERVED0[5U];
  volatile uint32_t CPACR;                   
} SCB_Type;

 















 






























 



 





















 









 


















 










































 









 









 















 







 



 
typedef struct
{
        uint32_t RESERVED0[1U];
  volatile const  uint32_t ICTR;                    
  volatile uint32_t ACTLR;                   
} SCnSCB_Type;

 



 















 







 



 
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
  }  PORT [32U];                          
        uint32_t RESERVED0[864U];
  volatile uint32_t TER;                     
        uint32_t RESERVED1[15U];
  volatile uint32_t TPR;                     
        uint32_t RESERVED2[15U];
  volatile uint32_t TCR;                     
        uint32_t RESERVED3[29U];
  volatile  uint32_t IWR;                     
  volatile const  uint32_t IRR;                     
  volatile uint32_t IMCR;                    
        uint32_t RESERVED4[43U];
  volatile  uint32_t LAR;                     
  volatile const  uint32_t LSR;                     
        uint32_t RESERVED5[6U];
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
  volatile uint32_t CTRL;                    
  volatile uint32_t CYCCNT;                  
  volatile uint32_t CPICNT;                  
  volatile uint32_t EXCCNT;                  
  volatile uint32_t SLEEPCNT;                
  volatile uint32_t LSUCNT;                  
  volatile uint32_t FOLDCNT;                 
  volatile const  uint32_t PCSR;                    
  volatile uint32_t COMP0;                   
  volatile uint32_t MASK0;                   
  volatile uint32_t FUNCTION0;               
        uint32_t RESERVED0[1U];
  volatile uint32_t COMP1;                   
  volatile uint32_t MASK1;                   
  volatile uint32_t FUNCTION1;               
        uint32_t RESERVED1[1U];
  volatile uint32_t COMP2;                   
  volatile uint32_t MASK2;                   
  volatile uint32_t FUNCTION2;               
        uint32_t RESERVED2[1U];
  volatile uint32_t COMP3;                   
  volatile uint32_t MASK3;                   
  volatile uint32_t FUNCTION3;               
} DWT_Type;

 






















































 



 



 



 



 



 



 



























   







 



 
typedef struct
{
  volatile uint32_t SSPSR;                   
  volatile uint32_t CSPSR;                   
        uint32_t RESERVED0[2U];
  volatile uint32_t ACPR;                    
        uint32_t RESERVED1[55U];
  volatile uint32_t SPPR;                    
        uint32_t RESERVED2[131U];
  volatile const  uint32_t FFSR;                    
  volatile uint32_t FFCR;                    
  volatile const  uint32_t FSCR;                    
        uint32_t RESERVED3[759U];
  volatile const  uint32_t TRIGGER;                 
  volatile const  uint32_t FIFO0;                   
  volatile const  uint32_t ITATBCTR2;               
        uint32_t RESERVED4[1U];
  volatile const  uint32_t ITATBCTR0;               
  volatile const  uint32_t FIFO1;                   
  volatile uint32_t ITCTRL;                  
        uint32_t RESERVED5[39U];
  volatile uint32_t CLAIMSET;                
  volatile uint32_t CLAIMCLR;                
        uint32_t RESERVED7[8U];
  volatile const  uint32_t DEVID;                   
  volatile const  uint32_t DEVTYPE;                 
} TPI_Type;

 



 



 












 






 



 





















 



 





















 



 



 


















 






   








 



 
typedef struct
{
  volatile const  uint32_t TYPE;                    
  volatile uint32_t CTRL;                    
  volatile uint32_t RNR;                     
  volatile uint32_t RBAR;                    
  volatile uint32_t RASR;                    
  volatile uint32_t RBAR_A1;                 
  volatile uint32_t RASR_A1;                 
  volatile uint32_t RBAR_A2;                 
  volatile uint32_t RASR_A2;                 
  volatile uint32_t RBAR_A3;                 
  volatile uint32_t RASR_A3;                 
} MPU_Type;

 









 









 



 









 






























 









 



 
typedef struct
{
        uint32_t RESERVED0[1U];
  volatile uint32_t FPCCR;                   
  volatile uint32_t FPCAR;                   
  volatile uint32_t FPDSCR;                  
  volatile const  uint32_t MVFR0;                   
  volatile const  uint32_t MVFR1;                   
} FPU_Type;

 



























 



 












 
























 












 








 



 
typedef struct
{
  volatile uint32_t DHCSR;                   
  volatile  uint32_t DCRSR;                   
  volatile uint32_t DCRDR;                   
  volatile uint32_t DEMCR;                   
} CoreDebug_Type;

 




































 






 







































 







 






 







 


 







 

 
#line 1541 "..\\CORE\\core_cm4.h"

#line 1550 "..\\CORE\\core_cm4.h"











 










 


 



 





 









 
static __inline void NVIC_SetPriorityGrouping(uint32_t PriorityGroup)
{
  uint32_t reg_value;
  uint32_t PriorityGroupTmp = (PriorityGroup & (uint32_t)0x07UL);              

  reg_value  =  ((SCB_Type *) ((0xE000E000UL) + 0x0D00UL) )->AIRCR;                                                    
  reg_value &= ~((uint32_t)((0xFFFFUL << 16U) | (7UL << 8U)));  
  reg_value  =  (reg_value                                   |
                ((uint32_t)0x5FAUL << 16U) |
                (PriorityGroupTmp << 8U)                      );               
  ((SCB_Type *) ((0xE000E000UL) + 0x0D00UL) )->AIRCR =  reg_value;
}






 
static __inline uint32_t NVIC_GetPriorityGrouping(void)
{
  return ((uint32_t)((((SCB_Type *) ((0xE000E000UL) + 0x0D00UL) )->AIRCR & (7UL << 8U)) >> 8U));
}






 
static __inline void NVIC_EnableIRQ(IRQn_Type IRQn)
{
  ((NVIC_Type *) ((0xE000E000UL) + 0x0100UL) )->ISER[(((uint32_t)(int32_t)IRQn) >> 5UL)] = (uint32_t)(1UL << (((uint32_t)(int32_t)IRQn) & 0x1FUL));
}






 
static __inline void NVIC_DisableIRQ(IRQn_Type IRQn)
{
  ((NVIC_Type *) ((0xE000E000UL) + 0x0100UL) )->ICER[(((uint32_t)(int32_t)IRQn) >> 5UL)] = (uint32_t)(1UL << (((uint32_t)(int32_t)IRQn) & 0x1FUL));
}








 
static __inline uint32_t NVIC_GetPendingIRQ(IRQn_Type IRQn)
{
  return((uint32_t)(((((NVIC_Type *) ((0xE000E000UL) + 0x0100UL) )->ISPR[(((uint32_t)(int32_t)IRQn) >> 5UL)] & (1UL << (((uint32_t)(int32_t)IRQn) & 0x1FUL))) != 0UL) ? 1UL : 0UL));
}






 
static __inline void NVIC_SetPendingIRQ(IRQn_Type IRQn)
{
  ((NVIC_Type *) ((0xE000E000UL) + 0x0100UL) )->ISPR[(((uint32_t)(int32_t)IRQn) >> 5UL)] = (uint32_t)(1UL << (((uint32_t)(int32_t)IRQn) & 0x1FUL));
}






 
static __inline void NVIC_ClearPendingIRQ(IRQn_Type IRQn)
{
  ((NVIC_Type *) ((0xE000E000UL) + 0x0100UL) )->ICPR[(((uint32_t)(int32_t)IRQn) >> 5UL)] = (uint32_t)(1UL << (((uint32_t)(int32_t)IRQn) & 0x1FUL));
}








 
static __inline uint32_t NVIC_GetActive(IRQn_Type IRQn)
{
  return((uint32_t)(((((NVIC_Type *) ((0xE000E000UL) + 0x0100UL) )->IABR[(((uint32_t)(int32_t)IRQn) >> 5UL)] & (1UL << (((uint32_t)(int32_t)IRQn) & 0x1FUL))) != 0UL) ? 1UL : 0UL));
}








 
static __inline void NVIC_SetPriority(IRQn_Type IRQn, uint32_t priority)
{
  if ((int32_t)(IRQn) < 0)
  {
    ((SCB_Type *) ((0xE000E000UL) + 0x0D00UL) )->SHP[(((uint32_t)(int32_t)IRQn) & 0xFUL)-4UL] = (uint8_t)((priority << (8U - 4)) & (uint32_t)0xFFUL);
  }
  else
  {
    ((NVIC_Type *) ((0xE000E000UL) + 0x0100UL) )->IP[((uint32_t)(int32_t)IRQn)]               = (uint8_t)((priority << (8U - 4)) & (uint32_t)0xFFUL);
  }
}










 
static __inline uint32_t NVIC_GetPriority(IRQn_Type IRQn)
{

  if ((int32_t)(IRQn) < 0)
  {
    return(((uint32_t)((SCB_Type *) ((0xE000E000UL) + 0x0D00UL) )->SHP[(((uint32_t)(int32_t)IRQn) & 0xFUL)-4UL] >> (8U - 4)));
  }
  else
  {
    return(((uint32_t)((NVIC_Type *) ((0xE000E000UL) + 0x0100UL) )->IP[((uint32_t)(int32_t)IRQn)]               >> (8U - 4)));
  }
}












 
static __inline uint32_t NVIC_EncodePriority (uint32_t PriorityGroup, uint32_t PreemptPriority, uint32_t SubPriority)
{
  uint32_t PriorityGroupTmp = (PriorityGroup & (uint32_t)0x07UL);    
  uint32_t PreemptPriorityBits;
  uint32_t SubPriorityBits;

  PreemptPriorityBits = ((7UL - PriorityGroupTmp) > (uint32_t)(4)) ? (uint32_t)(4) : (uint32_t)(7UL - PriorityGroupTmp);
  SubPriorityBits     = ((PriorityGroupTmp + (uint32_t)(4)) < (uint32_t)7UL) ? (uint32_t)0UL : (uint32_t)((PriorityGroupTmp - 7UL) + (uint32_t)(4));

  return (
           ((PreemptPriority & (uint32_t)((1UL << (PreemptPriorityBits)) - 1UL)) << SubPriorityBits) |
           ((SubPriority     & (uint32_t)((1UL << (SubPriorityBits    )) - 1UL)))
         );
}












 
static __inline void NVIC_DecodePriority (uint32_t Priority, uint32_t PriorityGroup, uint32_t* const pPreemptPriority, uint32_t* const pSubPriority)
{
  uint32_t PriorityGroupTmp = (PriorityGroup & (uint32_t)0x07UL);    
  uint32_t PreemptPriorityBits;
  uint32_t SubPriorityBits;

  PreemptPriorityBits = ((7UL - PriorityGroupTmp) > (uint32_t)(4)) ? (uint32_t)(4) : (uint32_t)(7UL - PriorityGroupTmp);
  SubPriorityBits     = ((PriorityGroupTmp + (uint32_t)(4)) < (uint32_t)7UL) ? (uint32_t)0UL : (uint32_t)((PriorityGroupTmp - 7UL) + (uint32_t)(4));

  *pPreemptPriority = (Priority >> SubPriorityBits) & (uint32_t)((1UL << (PreemptPriorityBits)) - 1UL);
  *pSubPriority     = (Priority                   ) & (uint32_t)((1UL << (SubPriorityBits    )) - 1UL);
}





 
static __inline void NVIC_SystemReset(void)
{
  do { __schedule_barrier(); __dsb(0xF); __schedule_barrier(); } while (0U);                                                          
 
  ((SCB_Type *) ((0xE000E000UL) + 0x0D00UL) )->AIRCR  = (uint32_t)((0x5FAUL << 16U)    |
                           (((SCB_Type *) ((0xE000E000UL) + 0x0D00UL) )->AIRCR & (7UL << 8U)) |
                            (1UL << 2U)    );          
  do { __schedule_barrier(); __dsb(0xF); __schedule_barrier(); } while (0U);                                                           

  for(;;)                                                            
  {
    __nop();
  }
}

 



 





 













 
static __inline uint32_t SysTick_Config(uint32_t ticks)
{
  if ((ticks - 1UL) > (0xFFFFFFUL ))
  {
    return (1UL);                                                    
  }

  ((SysTick_Type *) ((0xE000E000UL) + 0x0010UL) )->LOAD  = (uint32_t)(ticks - 1UL);                          
  NVIC_SetPriority (SysTick_IRQn, (1UL << 4) - 1UL);  
  ((SysTick_Type *) ((0xE000E000UL) + 0x0010UL) )->VAL   = 0UL;                                              
  ((SysTick_Type *) ((0xE000E000UL) + 0x0010UL) )->CTRL  = (1UL << 2U) |
                   (1UL << 1U)   |
                   (1UL );                          
  return (0UL);                                                      
}



 



 





 

extern volatile int32_t ITM_RxBuffer;                     










 
static __inline uint32_t ITM_SendChar (uint32_t ch)
{
  if (((((ITM_Type *) (0xE0000000UL) )->TCR & (1UL )) != 0UL) &&       
      ((((ITM_Type *) (0xE0000000UL) )->TER & 1UL               ) != 0UL)   )      
  {
    while (((ITM_Type *) (0xE0000000UL) )->PORT[0U].u32 == 0UL)
    {
      __nop();
    }
    ((ITM_Type *) (0xE0000000UL) )->PORT[0U].u8 = (uint8_t)ch;
  }
  return (ch);
}







 
static __inline int32_t ITM_ReceiveChar (void)
{
  int32_t ch = -1;                            

  if (ITM_RxBuffer != 0x5AA55AA5U)
  {
    ch = ITM_RxBuffer;
    ITM_RxBuffer = 0x5AA55AA5U;        
  }

  return (ch);
}







 
static __inline int32_t ITM_CheckChar (void)
{

  if (ITM_RxBuffer == 0x5AA55AA5U)
  {
    return (0);                               
  }
  else
  {
    return (1);                               
  }
}

 










#line 182 "..\\USER\\stm32l475xx.h"
#line 1 "..\\USER\\system_stm32l4xx.h"

































 



 



 



 









 



 




 
  






 
extern uint32_t SystemCoreClock;             

extern const uint8_t  AHBPrescTable[16];     
extern const uint8_t  APBPrescTable[8];      
extern const uint32_t MSIRangeTable[12];     



 



 



 



 



 



 

extern void SystemInit(void);
extern void SystemCoreClockUpdate(void);


 









 



 
 
#line 183 "..\\USER\\stm32l475xx.h"
#line 184 "..\\USER\\stm32l475xx.h"



 



 

typedef struct
{
  volatile uint32_t ISR;           
  volatile uint32_t IER;           
  volatile uint32_t CR;            
  volatile uint32_t CFGR;          
  volatile uint32_t CFGR2;         
  volatile uint32_t SMPR1;         
  volatile uint32_t SMPR2;         
       uint32_t RESERVED1;     
  volatile uint32_t TR1;           
  volatile uint32_t TR2;           
  volatile uint32_t TR3;           
       uint32_t RESERVED2;     
  volatile uint32_t SQR1;          
  volatile uint32_t SQR2;          
  volatile uint32_t SQR3;          
  volatile uint32_t SQR4;          
  volatile uint32_t DR;            
       uint32_t RESERVED3;     
       uint32_t RESERVED4;     
  volatile uint32_t JSQR;          
       uint32_t RESERVED5[4];  
  volatile uint32_t OFR1;          
  volatile uint32_t OFR2;          
  volatile uint32_t OFR3;          
  volatile uint32_t OFR4;          
       uint32_t RESERVED6[4];  
  volatile uint32_t JDR1;          
  volatile uint32_t JDR2;          
  volatile uint32_t JDR3;          
  volatile uint32_t JDR4;          
       uint32_t RESERVED7[4];  
  volatile uint32_t AWD2CR;        
  volatile uint32_t AWD3CR;        
       uint32_t RESERVED8;     
       uint32_t RESERVED9;     
  volatile uint32_t DIFSEL;        
  volatile uint32_t CALFACT;       

} ADC_TypeDef;

typedef struct
{
  volatile uint32_t CSR;           
  uint32_t      RESERVED;      
  volatile uint32_t CCR;           
  volatile uint32_t CDR;           
} ADC_Common_TypeDef;




 

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
  volatile uint32_t              MCR;                  
  volatile uint32_t              MSR;                  
  volatile uint32_t              TSR;                  
  volatile uint32_t              RF0R;                 
  volatile uint32_t              RF1R;                 
  volatile uint32_t              IER;                  
  volatile uint32_t              ESR;                  
  volatile uint32_t              BTR;                  
  uint32_t                   RESERVED0[88];        
  CAN_TxMailBox_TypeDef      sTxMailBox[3];        
  CAN_FIFOMailBox_TypeDef    sFIFOMailBox[2];      
  uint32_t                   RESERVED1[12];        
  volatile uint32_t              FMR;                  
  volatile uint32_t              FM1R;                 
  uint32_t                   RESERVED2;            
  volatile uint32_t              FS1R;                 
  uint32_t                   RESERVED3;            
  volatile uint32_t              FFA1R;                
  uint32_t                   RESERVED4;            
  volatile uint32_t              FA1R;                 
  uint32_t                   RESERVED5[8];         
  CAN_FilterRegister_TypeDef sFilterRegister[28];  
} CAN_TypeDef;




 

typedef struct
{
  volatile uint32_t CSR;          
} COMP_TypeDef;

typedef struct
{
  volatile uint32_t CSR;          
} COMP_Common_TypeDef;



 

typedef struct
{
  volatile uint32_t DR;           
  volatile uint8_t  IDR;          
  uint8_t       RESERVED0;    
  uint16_t      RESERVED1;    
  volatile uint32_t CR;           
  uint32_t      RESERVED2;    
  volatile uint32_t INIT;         
  volatile uint32_t POL;          
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
  volatile uint32_t SR;           
  volatile uint32_t CCR;          
  volatile uint32_t MCR;          
  volatile uint32_t SHSR1;        
  volatile uint32_t SHSR2;        
  volatile uint32_t SHHR;         
  volatile uint32_t SHRR;         
} DAC_TypeDef;



 
typedef struct
{
  volatile uint32_t FLTCR1;       
  volatile uint32_t FLTCR2;       
  volatile uint32_t FLTISR;       
  volatile uint32_t FLTICR;       
  volatile uint32_t FLTJCHGR;     
  volatile uint32_t FLTFCR;       
  volatile uint32_t FLTJDATAR;    
  volatile uint32_t FLTRDATAR;    
  volatile uint32_t FLTAWHTR;     
  volatile uint32_t FLTAWLTR;     
  volatile uint32_t FLTAWSR;      
  volatile uint32_t FLTAWCFR;     
  volatile uint32_t FLTEXMAX;     
  volatile uint32_t FLTEXMIN;     
  volatile uint32_t FLTCNVTIMR;   
} DFSDM_Filter_TypeDef;



 
typedef struct
{
  volatile uint32_t CHCFGR1;      
  volatile uint32_t CHCFGR2;      
  volatile uint32_t CHAWSCDR;    
 
  volatile uint32_t CHWDATAR;     
  volatile uint32_t CHDATINR;     
} DFSDM_Channel_TypeDef;



 

typedef struct
{
  volatile uint32_t IDCODE;       
  volatile uint32_t CR;           
  volatile uint32_t APB1FZR1;     
  volatile uint32_t APB1FZR2;     
  volatile uint32_t APB2FZ;       
} DBGMCU_TypeDef;




 

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
  volatile uint32_t CSELR;        
} DMA_Request_TypeDef;

 





 

typedef struct
{
  volatile uint32_t IMR1;         
  volatile uint32_t EMR1;         
  volatile uint32_t RTSR1;        
  volatile uint32_t FTSR1;        
  volatile uint32_t SWIER1;       
  volatile uint32_t PR1;          
  uint32_t      RESERVED1;    
  uint32_t      RESERVED2;    
  volatile uint32_t IMR2;         
  volatile uint32_t EMR2;         
  volatile uint32_t RTSR2;        
  volatile uint32_t FTSR2;        
  volatile uint32_t SWIER2;       
  volatile uint32_t PR2;          
} EXTI_TypeDef;




 

typedef struct
{
  volatile uint32_t CSSA;         
  volatile uint32_t CSL;          
  volatile uint32_t NVDSSA;       
  volatile uint32_t NVDSL;        
  volatile uint32_t VDSSA ;       
  volatile uint32_t VDSL ;        
  uint32_t      RESERVED1;    
  uint32_t      RESERVED2;    
  volatile uint32_t CR ;          
} FIREWALL_TypeDef;




 

typedef struct
{
  volatile uint32_t ACR;               
  volatile uint32_t PDKEYR;            
  volatile uint32_t KEYR;              
  volatile uint32_t OPTKEYR;           
  volatile uint32_t SR;                
  volatile uint32_t CR;                
  volatile uint32_t ECCR;              
  volatile uint32_t RESERVED1;         
  volatile uint32_t OPTR;              
  volatile uint32_t PCROP1SR;          
  volatile uint32_t PCROP1ER;          
  volatile uint32_t WRP1AR;            
  volatile uint32_t WRP1BR;            
       uint32_t RESERVED2[4];      
  volatile uint32_t PCROP2SR;          
  volatile uint32_t PCROP2ER;          
  volatile uint32_t WRP2AR;            
  volatile uint32_t WRP2BR;            
} FLASH_TypeDef;




 

typedef struct
{
  volatile uint32_t BTCR[8];      
} FMC_Bank1_TypeDef;



 

typedef struct
{
  volatile uint32_t BWTR[7];      
} FMC_Bank1E_TypeDef;



 

typedef struct
{
  volatile uint32_t PCR;         
  volatile uint32_t SR;          
  volatile uint32_t PMEM;        
  volatile uint32_t PATT;        
  uint32_t      RESERVED0;   
  volatile uint32_t ECCR;        
} FMC_Bank3_TypeDef;



 

typedef struct
{
  volatile uint32_t MODER;        
  volatile uint32_t OTYPER;       
  volatile uint32_t OSPEEDR;      
  volatile uint32_t PUPDR;        
  volatile uint32_t IDR;          
  volatile uint32_t ODR;          
  volatile uint32_t BSRR;         
  volatile uint32_t LCKR;         
  volatile uint32_t AFR[2];       
  volatile uint32_t BRR;          
  volatile uint32_t ASCR;         

} GPIO_TypeDef;




 

typedef struct
{
  volatile uint32_t CR1;          
  volatile uint32_t CR2;          
  volatile uint32_t OAR1;         
  volatile uint32_t OAR2;         
  volatile uint32_t TIMINGR;      
  volatile uint32_t TIMEOUTR;     
  volatile uint32_t ISR;          
  volatile uint32_t ICR;          
  volatile uint32_t PECR;         
  volatile uint32_t RXDR;         
  volatile uint32_t TXDR;         
} I2C_TypeDef;



 

typedef struct
{
  volatile uint32_t KR;           
  volatile uint32_t PR;           
  volatile uint32_t RLR;          
  volatile uint32_t SR;           
  volatile uint32_t WINR;         
} IWDG_TypeDef;



 
typedef struct
{
  volatile uint32_t ISR;          
  volatile uint32_t ICR;          
  volatile uint32_t IER;          
  volatile uint32_t CFGR;         
  volatile uint32_t CR;           
  volatile uint32_t CMP;          
  volatile uint32_t ARR;          
  volatile uint32_t CNT;          
  volatile uint32_t OR;           
} LPTIM_TypeDef;



 

typedef struct
{
  volatile uint32_t CSR;          
  volatile uint32_t OTR;          
  volatile uint32_t LPOTR;        
} OPAMP_TypeDef;

typedef struct
{
  volatile uint32_t CSR;          
} OPAMP_Common_TypeDef;



 

typedef struct
{
  volatile uint32_t CR1;    
  volatile uint32_t CR2;    
  volatile uint32_t CR3;    
  volatile uint32_t CR4;    
  volatile uint32_t SR1;    
  volatile uint32_t SR2;    
  volatile uint32_t SCR;    
  uint32_t RESERVED;    
  volatile uint32_t PUCRA;  
  volatile uint32_t PDCRA;  
  volatile uint32_t PUCRB;  
  volatile uint32_t PDCRB;  
  volatile uint32_t PUCRC;  
  volatile uint32_t PDCRC;  
  volatile uint32_t PUCRD;  
  volatile uint32_t PDCRD;  
  volatile uint32_t PUCRE;  
  volatile uint32_t PDCRE;  
  volatile uint32_t PUCRF;  
  volatile uint32_t PDCRF;  
  volatile uint32_t PUCRG;  
  volatile uint32_t PDCRG;  
  volatile uint32_t PUCRH;  
  volatile uint32_t PDCRH;  
} PWR_TypeDef;




 

typedef struct
{
  volatile uint32_t CR;           
  volatile uint32_t DCR;          
  volatile uint32_t SR;           
  volatile uint32_t FCR;          
  volatile uint32_t DLR;          
  volatile uint32_t CCR;          
  volatile uint32_t AR;           
  volatile uint32_t ABR;          
  volatile uint32_t DR;           
  volatile uint32_t PSMKR;        
  volatile uint32_t PSMAR;        
  volatile uint32_t PIR;          
  volatile uint32_t LPTR;         
} QUADSPI_TypeDef;




 

typedef struct
{
  volatile uint32_t CR;           
  volatile uint32_t ICSCR;        
  volatile uint32_t CFGR;         
  volatile uint32_t PLLCFGR;      
  volatile uint32_t PLLSAI1CFGR;  
  volatile uint32_t PLLSAI2CFGR;  
  volatile uint32_t CIER;         
  volatile uint32_t CIFR;         
  volatile uint32_t CICR;         
  uint32_t      RESERVED0;    
  volatile uint32_t AHB1RSTR;     
  volatile uint32_t AHB2RSTR;     
  volatile uint32_t AHB3RSTR;     
  uint32_t      RESERVED1;    
  volatile uint32_t APB1RSTR1;    
  volatile uint32_t APB1RSTR2;    
  volatile uint32_t APB2RSTR;     
  uint32_t      RESERVED2;    
  volatile uint32_t AHB1ENR;      
  volatile uint32_t AHB2ENR;      
  volatile uint32_t AHB3ENR;      
  uint32_t      RESERVED3;    
  volatile uint32_t APB1ENR1;     
  volatile uint32_t APB1ENR2;     
  volatile uint32_t APB2ENR;      
  uint32_t      RESERVED4;    
  volatile uint32_t AHB1SMENR;    
  volatile uint32_t AHB2SMENR;    
  volatile uint32_t AHB3SMENR;    
  uint32_t      RESERVED5;    
  volatile uint32_t APB1SMENR1;   
  volatile uint32_t APB1SMENR2;   
  volatile uint32_t APB2SMENR;    
  uint32_t      RESERVED6;    
  volatile uint32_t CCIPR;        
  uint32_t      RESERVED7;    
  volatile uint32_t BDCR;         
  volatile uint32_t CSR;          
} RCC_TypeDef;



 

typedef struct
{
  volatile uint32_t TR;           
  volatile uint32_t DR;           
  volatile uint32_t CR;           
  volatile uint32_t ISR;          
  volatile uint32_t PRER;         
  volatile uint32_t WUTR;         
       uint32_t reserved;     
  volatile uint32_t ALRMAR;       
  volatile uint32_t ALRMBR;       
  volatile uint32_t WPR;          
  volatile uint32_t SSR;          
  volatile uint32_t SHIFTR;       
  volatile uint32_t TSTR;         
  volatile uint32_t TSDR;         
  volatile uint32_t TSSSR;        
  volatile uint32_t CALR;         
  volatile uint32_t TAMPCR;       
  volatile uint32_t ALRMASSR;     
  volatile uint32_t ALRMBSSR;     
  volatile uint32_t OR;           
  volatile uint32_t BKP0R;        
  volatile uint32_t BKP1R;        
  volatile uint32_t BKP2R;        
  volatile uint32_t BKP3R;        
  volatile uint32_t BKP4R;        
  volatile uint32_t BKP5R;        
  volatile uint32_t BKP6R;        
  volatile uint32_t BKP7R;        
  volatile uint32_t BKP8R;        
  volatile uint32_t BKP9R;        
  volatile uint32_t BKP10R;       
  volatile uint32_t BKP11R;       
  volatile uint32_t BKP12R;       
  volatile uint32_t BKP13R;       
  volatile uint32_t BKP14R;       
  volatile uint32_t BKP15R;       
  volatile uint32_t BKP16R;       
  volatile uint32_t BKP17R;       
  volatile uint32_t BKP18R;       
  volatile uint32_t BKP19R;       
  volatile uint32_t BKP20R;       
  volatile uint32_t BKP21R;       
  volatile uint32_t BKP22R;       
  volatile uint32_t BKP23R;       
  volatile uint32_t BKP24R;       
  volatile uint32_t BKP25R;       
  volatile uint32_t BKP26R;       
  volatile uint32_t BKP27R;       
  volatile uint32_t BKP28R;       
  volatile uint32_t BKP29R;       
  volatile uint32_t BKP30R;       
  volatile uint32_t BKP31R;       
} RTC_TypeDef;



 

typedef struct
{
  volatile uint32_t GCR;          
} SAI_TypeDef;

typedef struct
{
  volatile uint32_t CR1;          
  volatile uint32_t CR2;          
  volatile uint32_t FRCR;         
  volatile uint32_t SLOTR;        
  volatile uint32_t IMR;          
  volatile uint32_t SR;           
  volatile uint32_t CLRFR;        
  volatile uint32_t DR;           
} SAI_Block_TypeDef;




 

typedef struct
{
  volatile uint32_t POWER;           
  volatile uint32_t CLKCR;           
  volatile uint32_t ARG;             
  volatile uint32_t CMD;             
  volatile const uint32_t  RESPCMD;         
  volatile const uint32_t  RESP1;           
  volatile const uint32_t  RESP2;           
  volatile const uint32_t  RESP3;           
  volatile const uint32_t  RESP4;           
  volatile uint32_t DTIMER;          
  volatile uint32_t DLEN;            
  volatile uint32_t DCTRL;           
  volatile const uint32_t  DCOUNT;          
  volatile const uint32_t  STA;             
  volatile uint32_t ICR;             
  volatile uint32_t MASK;            
  uint32_t      RESERVED0[2];    
  volatile const uint32_t  FIFOCNT;         
  uint32_t      RESERVED1[13];   
  volatile uint32_t FIFO;            
} SDMMC_TypeDef;




 

typedef struct
{
  volatile uint32_t CR1;          
  volatile uint32_t CR2;          
  volatile uint32_t SR;           
  volatile uint32_t DR;           
  volatile uint32_t CRCPR;        
  volatile uint32_t RXCRCR;       
  volatile uint32_t TXCRCR;       
} SPI_TypeDef;




 

typedef struct
{
  volatile uint32_t CR;           
  volatile uint32_t BRR;          
    uint32_t  RESERVED1;      
  volatile uint32_t ISR;          
  volatile uint32_t ICR;          
  volatile uint32_t IER;          
  volatile uint32_t RFL;          
  volatile uint32_t TDR;          
  volatile uint32_t RDR;          
  volatile uint32_t OR;           
} SWPMI_TypeDef;




 

typedef struct
{
  volatile uint32_t MEMRMP;       
  volatile uint32_t CFGR1;        
  volatile uint32_t EXTICR[4];    
  volatile uint32_t SCSR;         
  volatile uint32_t CFGR2;        
  volatile uint32_t SWPR;         
  volatile uint32_t SKR;          
} SYSCFG_TypeDef;




 

typedef struct
{
  volatile uint32_t CR1;          
  volatile uint32_t CR2;          
  volatile uint32_t SMCR;         
  volatile uint32_t DIER;         
  volatile uint32_t SR;           
  volatile uint32_t EGR;          
  volatile uint32_t CCMR1;        
  volatile uint32_t CCMR2;        
  volatile uint32_t CCER;         
  volatile uint32_t CNT;          
  volatile uint32_t PSC;          
  volatile uint32_t ARR;          
  volatile uint32_t RCR;          
  volatile uint32_t CCR1;         
  volatile uint32_t CCR2;         
  volatile uint32_t CCR3;         
  volatile uint32_t CCR4;         
  volatile uint32_t BDTR;         
  volatile uint32_t DCR;          
  volatile uint32_t DMAR;         
  volatile uint32_t OR1;          
  volatile uint32_t CCMR3;        
  volatile uint32_t CCR5;         
  volatile uint32_t CCR6;         
  volatile uint32_t OR2;          
  volatile uint32_t OR3;          
} TIM_TypeDef;




 

typedef struct
{
  volatile uint32_t CR;             
  volatile uint32_t IER;            
  volatile uint32_t ICR;            
  volatile uint32_t ISR;            
  volatile uint32_t IOHCR;          
  uint32_t      RESERVED1;      
  volatile uint32_t IOASCR;         
  uint32_t      RESERVED2;      
  volatile uint32_t IOSCR;          
  uint32_t      RESERVED3;      
  volatile uint32_t IOCCR;          
  uint32_t      RESERVED4;      
  volatile uint32_t IOGCSR;         
  volatile uint32_t IOGXCR[8];      
} TSC_TypeDef;



 

typedef struct
{
  volatile uint32_t CR1;          
  volatile uint32_t CR2;          
  volatile uint32_t CR3;          
  volatile uint32_t BRR;          
  volatile uint16_t GTPR;         
  uint16_t  RESERVED2;        
  volatile uint32_t RTOR;         
  volatile uint16_t RQR;          
  uint16_t  RESERVED3;        
  volatile uint32_t ISR;          
  volatile uint32_t ICR;          
  volatile uint16_t RDR;          
  uint16_t  RESERVED4;        
  volatile uint16_t TDR;          
  uint16_t  RESERVED5;        
} USART_TypeDef;



 

typedef struct
{
  volatile uint32_t CSR;          
  volatile uint32_t CCR;          
} VREFBUF_TypeDef;



 

typedef struct
{
  volatile uint32_t CR;           
  volatile uint32_t CFR;          
  volatile uint32_t SR;           
} WWDG_TypeDef;



 

typedef struct
{
  volatile uint32_t CR;   
  volatile uint32_t SR;   
  volatile uint32_t DR;   
} RNG_TypeDef;



 
typedef struct
{
  volatile uint32_t GOTGCTL;               
  volatile uint32_t GOTGINT;               
  volatile uint32_t GAHBCFG;               
  volatile uint32_t GUSBCFG;               
  volatile uint32_t GRSTCTL;               
  volatile uint32_t GINTSTS;               
  volatile uint32_t GINTMSK;               
  volatile uint32_t GRXSTSR;               
  volatile uint32_t GRXSTSP;               
  volatile uint32_t GRXFSIZ;               
  volatile uint32_t DIEPTXF0_HNPTXFSIZ;    
  volatile uint32_t HNPTXSTS;              
  uint32_t Reserved30[2];              
  volatile uint32_t GCCFG;                 
  volatile uint32_t CID;                   
  volatile uint32_t GSNPSID;               
  volatile uint32_t GHWCFG1;               
  volatile uint32_t GHWCFG2;               
  volatile uint32_t GHWCFG3;               
  uint32_t  Reserved6;                 
  volatile uint32_t GLPMCFG;               
  volatile uint32_t GPWRDN;                
  volatile uint32_t GDFIFOCFG;             
   volatile uint32_t GADPCTL;              
    uint32_t  Reserved43[39];          
  volatile uint32_t HPTXFSIZ;              
  volatile uint32_t DIEPTXF[0x0F];         
} USB_OTG_GlobalTypeDef;



 
typedef struct
{
  volatile uint32_t DCFG;         
  volatile uint32_t DCTL;         
  volatile uint32_t DSTS;         
  uint32_t Reserved0C;        
  volatile uint32_t DIEPMSK;      
  volatile uint32_t DOEPMSK;      
  volatile uint32_t DAINT;        
  volatile uint32_t DAINTMSK;     
  uint32_t Reserved20;        
  uint32_t Reserved24;        
  volatile uint32_t DVBUSDIS;     
  volatile uint32_t DVBUSPULSE;   
  volatile uint32_t DTHRCTL;      
  volatile uint32_t DIEPEMPMSK;   
  volatile uint32_t DEACHINT;     
  volatile uint32_t DEACHMSK;     
  uint32_t Reserved40;        
  volatile uint32_t DINEP1MSK;    
  uint32_t  Reserved44[15];   
  volatile uint32_t DOUTEP1MSK;   
} USB_OTG_DeviceTypeDef;



 
typedef struct
{
  volatile uint32_t DIEPCTL;      
  uint32_t Reserved04;        
  volatile uint32_t DIEPINT;      
  uint32_t Reserved0C;        
  volatile uint32_t DIEPTSIZ;     
  volatile uint32_t DIEPDMA;      
  volatile uint32_t DTXFSTS;      
  uint32_t Reserved18;        
} USB_OTG_INEndpointTypeDef;



 
typedef struct
{
  volatile uint32_t DOEPCTL;      
  uint32_t Reserved04;        
  volatile uint32_t DOEPINT;      
  uint32_t Reserved0C;        
  volatile uint32_t DOEPTSIZ;     
  volatile uint32_t DOEPDMA;      
  uint32_t Reserved18[2];     
} USB_OTG_OUTEndpointTypeDef;



 
typedef struct
{
  volatile uint32_t HCFG;         
  volatile uint32_t HFIR;         
  volatile uint32_t HFNUM;        
  uint32_t Reserved40C;       
  volatile uint32_t HPTXSTS;      
  volatile uint32_t HAINT;        
  volatile uint32_t HAINTMSK;     
} USB_OTG_HostTypeDef;



 
typedef struct
{
  volatile uint32_t HCCHAR;
  volatile uint32_t HCSPLT;
  volatile uint32_t HCINT;
  volatile uint32_t HCINTMSK;
  volatile uint32_t HCTSIZ;
  volatile uint32_t HCDMA;
  uint32_t Reserved[2];
} USB_OTG_HostChannelTypeDef;



 



 
#line 1134 "..\\USER\\stm32l475xx.h"






 






 





#line 1159 "..\\USER\\stm32l475xx.h"

 
#line 1190 "..\\USER\\stm32l475xx.h"


 
#line 1226 "..\\USER\\stm32l475xx.h"

 
#line 1234 "..\\USER\\stm32l475xx.h"


#line 1244 "..\\USER\\stm32l475xx.h"


#line 1254 "..\\USER\\stm32l475xx.h"


 
#line 1265 "..\\USER\\stm32l475xx.h"












 




 


 


#line 1300 "..\\USER\\stm32l475xx.h"







 



 
#line 1343 "..\\USER\\stm32l475xx.h"

#line 1377 "..\\USER\\stm32l475xx.h"
 
#line 1396 "..\\USER\\stm32l475xx.h"

#line 1410 "..\\USER\\stm32l475xx.h"


#line 1420 "..\\USER\\stm32l475xx.h"


#line 1430 "..\\USER\\stm32l475xx.h"













 



 



 

 
 
 

 
 
 
 
 



 


 
#line 1502 "..\\USER\\stm32l475xx.h"

 
#line 1537 "..\\USER\\stm32l475xx.h"

 
#line 1550 "..\\USER\\stm32l475xx.h"

 
#line 1582 "..\\USER\\stm32l475xx.h"

 
#line 1590 "..\\USER\\stm32l475xx.h"











#line 1608 "..\\USER\\stm32l475xx.h"







#line 1624 "..\\USER\\stm32l475xx.h"





#line 1635 "..\\USER\\stm32l475xx.h"

#line 1654 "..\\USER\\stm32l475xx.h"

#line 1663 "..\\USER\\stm32l475xx.h"





 
#line 1675 "..\\USER\\stm32l475xx.h"

#line 1682 "..\\USER\\stm32l475xx.h"

#line 1690 "..\\USER\\stm32l475xx.h"

#line 1697 "..\\USER\\stm32l475xx.h"

 
#line 1705 "..\\USER\\stm32l475xx.h"

#line 1712 "..\\USER\\stm32l475xx.h"

#line 1719 "..\\USER\\stm32l475xx.h"

#line 1726 "..\\USER\\stm32l475xx.h"

#line 1733 "..\\USER\\stm32l475xx.h"

#line 1740 "..\\USER\\stm32l475xx.h"

#line 1747 "..\\USER\\stm32l475xx.h"

#line 1754 "..\\USER\\stm32l475xx.h"

#line 1761 "..\\USER\\stm32l475xx.h"

#line 1768 "..\\USER\\stm32l475xx.h"

 
#line 1776 "..\\USER\\stm32l475xx.h"

#line 1783 "..\\USER\\stm32l475xx.h"

#line 1790 "..\\USER\\stm32l475xx.h"

#line 1797 "..\\USER\\stm32l475xx.h"

#line 1804 "..\\USER\\stm32l475xx.h"

#line 1811 "..\\USER\\stm32l475xx.h"

#line 1818 "..\\USER\\stm32l475xx.h"

#line 1825 "..\\USER\\stm32l475xx.h"

#line 1832 "..\\USER\\stm32l475xx.h"

 
#line 1849 "..\\USER\\stm32l475xx.h"

#line 1865 "..\\USER\\stm32l475xx.h"

 
#line 1878 "..\\USER\\stm32l475xx.h"

#line 1890 "..\\USER\\stm32l475xx.h"

 
#line 1903 "..\\USER\\stm32l475xx.h"

#line 1915 "..\\USER\\stm32l475xx.h"

 
#line 1924 "..\\USER\\stm32l475xx.h"

#line 1933 "..\\USER\\stm32l475xx.h"

#line 1942 "..\\USER\\stm32l475xx.h"

#line 1951 "..\\USER\\stm32l475xx.h"

#line 1960 "..\\USER\\stm32l475xx.h"

 
#line 1970 "..\\USER\\stm32l475xx.h"

#line 1979 "..\\USER\\stm32l475xx.h"

#line 1988 "..\\USER\\stm32l475xx.h"

#line 1997 "..\\USER\\stm32l475xx.h"

#line 2006 "..\\USER\\stm32l475xx.h"

 
#line 2016 "..\\USER\\stm32l475xx.h"

#line 2025 "..\\USER\\stm32l475xx.h"

#line 2034 "..\\USER\\stm32l475xx.h"

#line 2043 "..\\USER\\stm32l475xx.h"

#line 2052 "..\\USER\\stm32l475xx.h"

 
#line 2062 "..\\USER\\stm32l475xx.h"

#line 2071 "..\\USER\\stm32l475xx.h"

 
#line 2092 "..\\USER\\stm32l475xx.h"

 






#line 2107 "..\\USER\\stm32l475xx.h"







#line 2122 "..\\USER\\stm32l475xx.h"

#line 2131 "..\\USER\\stm32l475xx.h"

#line 2140 "..\\USER\\stm32l475xx.h"

#line 2149 "..\\USER\\stm32l475xx.h"

 
#line 2166 "..\\USER\\stm32l475xx.h"

#line 2175 "..\\USER\\stm32l475xx.h"





 
#line 2196 "..\\USER\\stm32l475xx.h"

#line 2205 "..\\USER\\stm32l475xx.h"





 
#line 2226 "..\\USER\\stm32l475xx.h"

#line 2235 "..\\USER\\stm32l475xx.h"





 
#line 2256 "..\\USER\\stm32l475xx.h"

#line 2265 "..\\USER\\stm32l475xx.h"





 
#line 2290 "..\\USER\\stm32l475xx.h"

 
#line 2311 "..\\USER\\stm32l475xx.h"

 
#line 2332 "..\\USER\\stm32l475xx.h"

 
#line 2353 "..\\USER\\stm32l475xx.h"

 
#line 2377 "..\\USER\\stm32l475xx.h"

 
#line 2401 "..\\USER\\stm32l475xx.h"

 
#line 2425 "..\\USER\\stm32l475xx.h"

 
#line 2437 "..\\USER\\stm32l475xx.h"

#line 2448 "..\\USER\\stm32l475xx.h"

 
 
#line 2484 "..\\USER\\stm32l475xx.h"

#line 2518 "..\\USER\\stm32l475xx.h"

 
#line 2528 "..\\USER\\stm32l475xx.h"

#line 2536 "..\\USER\\stm32l475xx.h"

















#line 2560 "..\\USER\\stm32l475xx.h"

#line 2570 "..\\USER\\stm32l475xx.h"

 
#line 2591 "..\\USER\\stm32l475xx.h"

#line 2611 "..\\USER\\stm32l475xx.h"

 
 
 
 
 
 
 
#line 2646 "..\\USER\\stm32l475xx.h"

 
#line 2675 "..\\USER\\stm32l475xx.h"

 
#line 2725 "..\\USER\\stm32l475xx.h"

#line 2738 "..\\USER\\stm32l475xx.h"

#line 2751 "..\\USER\\stm32l475xx.h"

 
#line 2765 "..\\USER\\stm32l475xx.h"

 
#line 2779 "..\\USER\\stm32l475xx.h"

 
#line 2823 "..\\USER\\stm32l475xx.h"

 
#line 2834 "..\\USER\\stm32l475xx.h"

#line 2841 "..\\USER\\stm32l475xx.h"

#line 2848 "..\\USER\\stm32l475xx.h"

 
#line 2877 "..\\USER\\stm32l475xx.h"

 
 
#line 2895 "..\\USER\\stm32l475xx.h"

 
#line 2906 "..\\USER\\stm32l475xx.h"

 
#line 2920 "..\\USER\\stm32l475xx.h"

 
#line 2934 "..\\USER\\stm32l475xx.h"

 
#line 2951 "..\\USER\\stm32l475xx.h"

 
#line 2962 "..\\USER\\stm32l475xx.h"

 
#line 2976 "..\\USER\\stm32l475xx.h"

 
#line 2990 "..\\USER\\stm32l475xx.h"

 
#line 3007 "..\\USER\\stm32l475xx.h"

 
#line 3018 "..\\USER\\stm32l475xx.h"

 
#line 3032 "..\\USER\\stm32l475xx.h"

 
#line 3046 "..\\USER\\stm32l475xx.h"

 
#line 3060 "..\\USER\\stm32l475xx.h"

 
#line 3071 "..\\USER\\stm32l475xx.h"

 
#line 3085 "..\\USER\\stm32l475xx.h"

 
#line 3099 "..\\USER\\stm32l475xx.h"

 
#line 3113 "..\\USER\\stm32l475xx.h"

 
#line 3124 "..\\USER\\stm32l475xx.h"

 
#line 3138 "..\\USER\\stm32l475xx.h"

 
#line 3152 "..\\USER\\stm32l475xx.h"

 
 




 
#line 3205 "..\\USER\\stm32l475xx.h"

 
#line 3252 "..\\USER\\stm32l475xx.h"

 
#line 3299 "..\\USER\\stm32l475xx.h"

 
#line 3346 "..\\USER\\stm32l475xx.h"

 
#line 3444 "..\\USER\\stm32l475xx.h"

 
#line 3542 "..\\USER\\stm32l475xx.h"

 
#line 3640 "..\\USER\\stm32l475xx.h"

 
#line 3738 "..\\USER\\stm32l475xx.h"

 
#line 3836 "..\\USER\\stm32l475xx.h"

 
#line 3934 "..\\USER\\stm32l475xx.h"

 
#line 4032 "..\\USER\\stm32l475xx.h"

 
#line 4130 "..\\USER\\stm32l475xx.h"

 
#line 4228 "..\\USER\\stm32l475xx.h"

 
#line 4326 "..\\USER\\stm32l475xx.h"

 
#line 4424 "..\\USER\\stm32l475xx.h"

 
#line 4522 "..\\USER\\stm32l475xx.h"

 
#line 4620 "..\\USER\\stm32l475xx.h"

 
#line 4718 "..\\USER\\stm32l475xx.h"

 
#line 4816 "..\\USER\\stm32l475xx.h"

 
#line 4914 "..\\USER\\stm32l475xx.h"

 
#line 5012 "..\\USER\\stm32l475xx.h"

 
#line 5110 "..\\USER\\stm32l475xx.h"

 
#line 5208 "..\\USER\\stm32l475xx.h"

 
#line 5306 "..\\USER\\stm32l475xx.h"

 
#line 5404 "..\\USER\\stm32l475xx.h"

 
#line 5502 "..\\USER\\stm32l475xx.h"

 
#line 5600 "..\\USER\\stm32l475xx.h"

 
#line 5698 "..\\USER\\stm32l475xx.h"

 
#line 5796 "..\\USER\\stm32l475xx.h"

 
#line 5894 "..\\USER\\stm32l475xx.h"

 
#line 5992 "..\\USER\\stm32l475xx.h"

 
#line 6090 "..\\USER\\stm32l475xx.h"

 
 
 
 
 
 




 




 
#line 6123 "..\\USER\\stm32l475xx.h"

 




 




 
 
 
 
 


 


 
#line 6151 "..\\USER\\stm32l475xx.h"

#line 6158 "..\\USER\\stm32l475xx.h"







#line 6172 "..\\USER\\stm32l475xx.h"

#line 6182 "..\\USER\\stm32l475xx.h"

#line 6189 "..\\USER\\stm32l475xx.h"

#line 6196 "..\\USER\\stm32l475xx.h"







#line 6210 "..\\USER\\stm32l475xx.h"

#line 6220 "..\\USER\\stm32l475xx.h"

 
#line 6228 "..\\USER\\stm32l475xx.h"

 




 




 




 




 




 




 
#line 6266 "..\\USER\\stm32l475xx.h"

 
#line 6274 "..\\USER\\stm32l475xx.h"

 
#line 6282 "..\\USER\\stm32l475xx.h"

 




 




 
#line 6303 "..\\USER\\stm32l475xx.h"

#line 6313 "..\\USER\\stm32l475xx.h"

 
#line 6321 "..\\USER\\stm32l475xx.h"

 
#line 6329 "..\\USER\\stm32l475xx.h"

#line 6336 "..\\USER\\stm32l475xx.h"

 




 




 
#line 6354 "..\\USER\\stm32l475xx.h"

 
#line 6362 "..\\USER\\stm32l475xx.h"

 
 
 
 
 

 

 
#line 6413 "..\\USER\\stm32l475xx.h"

 
#line 6421 "..\\USER\\stm32l475xx.h"

 
#line 6437 "..\\USER\\stm32l475xx.h"

 




 
#line 6450 "..\\USER\\stm32l475xx.h"

 

 
#line 6501 "..\\USER\\stm32l475xx.h"

 
#line 6530 "..\\USER\\stm32l475xx.h"

 
#line 6559 "..\\USER\\stm32l475xx.h"

 
#line 6573 "..\\USER\\stm32l475xx.h"

 




 
#line 6592 "..\\USER\\stm32l475xx.h"

 
#line 6600 "..\\USER\\stm32l475xx.h"

 
#line 6611 "..\\USER\\stm32l475xx.h"

 
#line 6619 "..\\USER\\stm32l475xx.h"

 
#line 6627 "..\\USER\\stm32l475xx.h"

 
#line 6635 "..\\USER\\stm32l475xx.h"

 
#line 6643 "..\\USER\\stm32l475xx.h"

 
#line 6651 "..\\USER\\stm32l475xx.h"

 
#line 6659 "..\\USER\\stm32l475xx.h"

 




 
 
 
 
 

 
#line 6756 "..\\USER\\stm32l475xx.h"

 
#line 6842 "..\\USER\\stm32l475xx.h"

 
#line 6868 "..\\USER\\stm32l475xx.h"























 




 




 





 
#line 6929 "..\\USER\\stm32l475xx.h"

 
 
 
 
 
 
#line 7035 "..\\USER\\stm32l475xx.h"

 
#line 7133 "..\\USER\\stm32l475xx.h"

 
#line 7201 "..\\USER\\stm32l475xx.h"

 
#line 7269 "..\\USER\\stm32l475xx.h"

 
#line 7337 "..\\USER\\stm32l475xx.h"

 
#line 7405 "..\\USER\\stm32l475xx.h"

 
#line 7431 "..\\USER\\stm32l475xx.h"

 
#line 7457 "..\\USER\\stm32l475xx.h"

 
#line 7471 "..\\USER\\stm32l475xx.h"

 
#line 7485 "..\\USER\\stm32l475xx.h"

 
#line 7499 "..\\USER\\stm32l475xx.h"

 
#line 7513 "..\\USER\\stm32l475xx.h"


 
 
 
 
 
 
#line 7550 "..\\USER\\stm32l475xx.h"

 
#line 7588 "..\\USER\\stm32l475xx.h"

 
#line 7635 "..\\USER\\stm32l475xx.h"

 
#line 7655 "..\\USER\\stm32l475xx.h"

 
#line 7704 "..\\USER\\stm32l475xx.h"

 




 
#line 7717 "..\\USER\\stm32l475xx.h"

 
#line 7725 "..\\USER\\stm32l475xx.h"

 
#line 7733 "..\\USER\\stm32l475xx.h"

 




 




 
#line 7751 "..\\USER\\stm32l475xx.h"

 
#line 7759 "..\\USER\\stm32l475xx.h"


 
 
 
 
 
 




 
#line 7778 "..\\USER\\stm32l475xx.h"













#line 7815 "..\\USER\\stm32l475xx.h"

#line 7822 "..\\USER\\stm32l475xx.h"





 
#line 7835 "..\\USER\\stm32l475xx.h"

#line 7843 "..\\USER\\stm32l475xx.h"

#line 7855 "..\\USER\\stm32l475xx.h"

#line 7863 "..\\USER\\stm32l475xx.h"

#line 7871 "..\\USER\\stm32l475xx.h"

#line 7879 "..\\USER\\stm32l475xx.h"







 
#line 7894 "..\\USER\\stm32l475xx.h"

#line 7902 "..\\USER\\stm32l475xx.h"

#line 7914 "..\\USER\\stm32l475xx.h"

#line 7922 "..\\USER\\stm32l475xx.h"







 
#line 7939 "..\\USER\\stm32l475xx.h"











#line 7957 "..\\USER\\stm32l475xx.h"

#line 7965 "..\\USER\\stm32l475xx.h"

#line 7972 "..\\USER\\stm32l475xx.h"

 
#line 7995 "..\\USER\\stm32l475xx.h"

 
#line 8008 "..\\USER\\stm32l475xx.h"

#line 8020 "..\\USER\\stm32l475xx.h"

#line 8032 "..\\USER\\stm32l475xx.h"

#line 8044 "..\\USER\\stm32l475xx.h"

 
#line 8057 "..\\USER\\stm32l475xx.h"

#line 8069 "..\\USER\\stm32l475xx.h"

#line 8081 "..\\USER\\stm32l475xx.h"

#line 8093 "..\\USER\\stm32l475xx.h"

 




 
 
 
 
 
 
#line 8185 "..\\USER\\stm32l475xx.h"

 
#line 8235 "..\\USER\\stm32l475xx.h"

 
#line 8285 "..\\USER\\stm32l475xx.h"

 
#line 8303 "..\\USER\\stm32l475xx.h"

 
#line 8385 "..\\USER\\stm32l475xx.h"

 
#line 8435 "..\\USER\\stm32l475xx.h"

 
#line 8517 "..\\USER\\stm32l475xx.h"

 
#line 8567 "..\\USER\\stm32l475xx.h"

 
#line 8617 "..\\USER\\stm32l475xx.h"

 
#line 8635 "..\\USER\\stm32l475xx.h"

 
#line 8653 "..\\USER\\stm32l475xx.h"

 
#line 8703 "..\\USER\\stm32l475xx.h"

 
#line 8721 "..\\USER\\stm32l475xx.h"

 
#line 8739 "..\\USER\\stm32l475xx.h"

 
#line 8837 "..\\USER\\stm32l475xx.h"

 
#line 8871 "..\\USER\\stm32l475xx.h"

 
#line 8924 "..\\USER\\stm32l475xx.h"

 
#line 8982 "..\\USER\\stm32l475xx.h"

 
#line 8992 "..\\USER\\stm32l475xx.h"

 
#line 9050 "..\\USER\\stm32l475xx.h"

 
#line 9060 "..\\USER\\stm32l475xx.h"

 
#line 9110 "..\\USER\\stm32l475xx.h"

 
#line 9128 "..\\USER\\stm32l475xx.h"


 
#line 9179 "..\\USER\\stm32l475xx.h"

 
#line 9197 "..\\USER\\stm32l475xx.h"

 
 
 
 
 
 
#line 9267 "..\\USER\\stm32l475xx.h"

 
#line 9302 "..\\USER\\stm32l475xx.h"

 
#line 9313 "..\\USER\\stm32l475xx.h"

 
#line 9346 "..\\USER\\stm32l475xx.h"

 
#line 9363 "..\\USER\\stm32l475xx.h"

 
#line 9380 "..\\USER\\stm32l475xx.h"

 
#line 9433 "..\\USER\\stm32l475xx.h"

 
#line 9462 "..\\USER\\stm32l475xx.h"

 




 




 




 
 
 
 
 
 




 
#line 9495 "..\\USER\\stm32l475xx.h"

 




 
#line 9511 "..\\USER\\stm32l475xx.h"

 




 
 
 
 
 

 
#line 9542 "..\\USER\\stm32l475xx.h"

 
#line 9553 "..\\USER\\stm32l475xx.h"

 
 
 
 
 

 

#line 9589 "..\\USER\\stm32l475xx.h"


 
#line 9598 "..\\USER\\stm32l475xx.h"
 
#line 9614 "..\\USER\\stm32l475xx.h"
 
#line 9643 "..\\USER\\stm32l475xx.h"

 
#line 9672 "..\\USER\\stm32l475xx.h"

 





 
#line 9701 "..\\USER\\stm32l475xx.h"

 
#line 9727 "..\\USER\\stm32l475xx.h"

 
#line 9753 "..\\USER\\stm32l475xx.h"

 
#line 9776 "..\\USER\\stm32l475xx.h"

 
#line 9823 "..\\USER\\stm32l475xx.h"

 
#line 9867 "..\\USER\\stm32l475xx.h"

 
#line 9917 "..\\USER\\stm32l475xx.h"

 
#line 9964 "..\\USER\\stm32l475xx.h"

 
#line 10014 "..\\USER\\stm32l475xx.h"

 
#line 10064 "..\\USER\\stm32l475xx.h"

 
#line 10114 "..\\USER\\stm32l475xx.h"

 
#line 10164 "..\\USER\\stm32l475xx.h"

 
#line 10214 "..\\USER\\stm32l475xx.h"

 
#line 10264 "..\\USER\\stm32l475xx.h"

 
#line 10314 "..\\USER\\stm32l475xx.h"

 
#line 10364 "..\\USER\\stm32l475xx.h"

 
#line 10414 "..\\USER\\stm32l475xx.h"

 
#line 10464 "..\\USER\\stm32l475xx.h"

 
#line 10472 "..\\USER\\stm32l475xx.h"

 
#line 10480 "..\\USER\\stm32l475xx.h"


 
 
 
 
 


 




 
#line 10507 "..\\USER\\stm32l475xx.h"

 
#line 10524 "..\\USER\\stm32l475xx.h"

#line 10537 "..\\USER\\stm32l475xx.h"

#line 10550 "..\\USER\\stm32l475xx.h"

#line 10569 "..\\USER\\stm32l475xx.h"

 
 
#line 10583 "..\\USER\\stm32l475xx.h"

 
#line 10596 "..\\USER\\stm32l475xx.h"

 
#line 10609 "..\\USER\\stm32l475xx.h"

 
#line 10619 "..\\USER\\stm32l475xx.h"

 
 











 











 
#line 10653 "..\\USER\\stm32l475xx.h"

#line 10663 "..\\USER\\stm32l475xx.h"

 
#line 10671 "..\\USER\\stm32l475xx.h"







 
#line 10685 "..\\USER\\stm32l475xx.h"











 
#line 10703 "..\\USER\\stm32l475xx.h"

#line 10710 "..\\USER\\stm32l475xx.h"







 
#line 10724 "..\\USER\\stm32l475xx.h"

 




#line 10739 "..\\USER\\stm32l475xx.h"

#line 10746 "..\\USER\\stm32l475xx.h"

#line 10757 "..\\USER\\stm32l475xx.h"

#line 10767 "..\\USER\\stm32l475xx.h"







#line 10782 "..\\USER\\stm32l475xx.h"

 
#line 10794 "..\\USER\\stm32l475xx.h"

#line 10801 "..\\USER\\stm32l475xx.h"

#line 10810 "..\\USER\\stm32l475xx.h"

#line 10819 "..\\USER\\stm32l475xx.h"

 
#line 10831 "..\\USER\\stm32l475xx.h"

#line 10838 "..\\USER\\stm32l475xx.h"

#line 10847 "..\\USER\\stm32l475xx.h"

 
#line 10876 "..\\USER\\stm32l475xx.h"

 
#line 10908 "..\\USER\\stm32l475xx.h"

 
#line 10940 "..\\USER\\stm32l475xx.h"

 
#line 10957 "..\\USER\\stm32l475xx.h"

 
#line 10992 "..\\USER\\stm32l475xx.h"

 
#line 11000 "..\\USER\\stm32l475xx.h"

 
#line 11062 "..\\USER\\stm32l475xx.h"

 
#line 11073 "..\\USER\\stm32l475xx.h"

 
#line 11111 "..\\USER\\stm32l475xx.h"

 
#line 11128 "..\\USER\\stm32l475xx.h"

 
#line 11163 "..\\USER\\stm32l475xx.h"

 
#line 11171 "..\\USER\\stm32l475xx.h"

 
#line 11236 "..\\USER\\stm32l475xx.h"

 
#line 11247 "..\\USER\\stm32l475xx.h"

 
#line 11288 "..\\USER\\stm32l475xx.h"

 
#line 11308 "..\\USER\\stm32l475xx.h"

 
#line 11346 "..\\USER\\stm32l475xx.h"

 
#line 11354 "..\\USER\\stm32l475xx.h"

 
#line 11419 "..\\USER\\stm32l475xx.h"

 
#line 11430 "..\\USER\\stm32l475xx.h"

 
#line 11468 "..\\USER\\stm32l475xx.h"

 


































































































 
#line 11578 "..\\USER\\stm32l475xx.h"







#line 11591 "..\\USER\\stm32l475xx.h"







#line 11610 "..\\USER\\stm32l475xx.h"

 
#line 11618 "..\\USER\\stm32l475xx.h"

#line 11626 "..\\USER\\stm32l475xx.h"

#line 11654 "..\\USER\\stm32l475xx.h"

 
 
 
 
 
 
#line 11667 "..\\USER\\stm32l475xx.h"

 
#line 11684 "..\\USER\\stm32l475xx.h"

 
 
 
 
 


 






 
#line 11741 "..\\USER\\stm32l475xx.h"

 
#line 11785 "..\\USER\\stm32l475xx.h"

 
#line 11855 "..\\USER\\stm32l475xx.h"

 




 
#line 11916 "..\\USER\\stm32l475xx.h"

 
#line 11924 "..\\USER\\stm32l475xx.h"

 




 
#line 11999 "..\\USER\\stm32l475xx.h"

 
#line 12069 "..\\USER\\stm32l475xx.h"

 




 




 
#line 12087 "..\\USER\\stm32l475xx.h"

 
#line 12130 "..\\USER\\stm32l475xx.h"

 
#line 12160 "..\\USER\\stm32l475xx.h"

 




 
#line 12188 "..\\USER\\stm32l475xx.h"

 
#line 12260 "..\\USER\\stm32l475xx.h"

 
#line 12272 "..\\USER\\stm32l475xx.h"

 
#line 12284 "..\\USER\\stm32l475xx.h"

 
#line 12292 "..\\USER\\stm32l475xx.h"


 




 




 




 




 




 




 




 




 




 




 




 




 




 




 




 




 




 




 




 




 




 




 




 




 




 




 




 




 




 




 




 




 


 
 
 
 
 
 












 












#line 12494 "..\\USER\\stm32l475xx.h"

#line 12501 "..\\USER\\stm32l475xx.h"







#line 12523 "..\\USER\\stm32l475xx.h"

#line 12531 "..\\USER\\stm32l475xx.h"

 
#line 12539 "..\\USER\\stm32l475xx.h"

#line 12552 "..\\USER\\stm32l475xx.h"


#line 12563 "..\\USER\\stm32l475xx.h"

#line 12572 "..\\USER\\stm32l475xx.h"


 
#line 12586 "..\\USER\\stm32l475xx.h"

#line 12597 "..\\USER\\stm32l475xx.h"

#line 12607 "..\\USER\\stm32l475xx.h"

 
#line 12617 "..\\USER\\stm32l475xx.h"







#line 12631 "..\\USER\\stm32l475xx.h"





 
#line 12658 "..\\USER\\stm32l475xx.h"

 
#line 12681 "..\\USER\\stm32l475xx.h"

#line 12688 "..\\USER\\stm32l475xx.h"

 
#line 12711 "..\\USER\\stm32l475xx.h"

 




 
 
 
 
 
 






 
#line 12742 "..\\USER\\stm32l475xx.h"







#line 12755 "..\\USER\\stm32l475xx.h"

 




 










#line 12784 "..\\USER\\stm32l475xx.h"

 




 




 




 




 




 




 




 
#line 12833 "..\\USER\\stm32l475xx.h"

#line 12841 "..\\USER\\stm32l475xx.h"

#line 12854 "..\\USER\\stm32l475xx.h"

 




 
#line 12930 "..\\USER\\stm32l475xx.h"

 
#line 12968 "..\\USER\\stm32l475xx.h"

 
#line 13036 "..\\USER\\stm32l475xx.h"

 




 




 
 
 
 
 
 
#line 13062 "..\\USER\\stm32l475xx.h"

#line 13069 "..\\USER\\stm32l475xx.h"

#line 13100 "..\\USER\\stm32l475xx.h"

 
#line 13142 "..\\USER\\stm32l475xx.h"

 
#line 13181 "..\\USER\\stm32l475xx.h"

 




 




 




 




 
 
 
 
 
 
#line 13250 "..\\USER\\stm32l475xx.h"

 
#line 13264 "..\\USER\\stm32l475xx.h"

 
#line 13287 "..\\USER\\stm32l475xx.h"

 
#line 13301 "..\\USER\\stm32l475xx.h"

 




 
#line 13355 "..\\USER\\stm32l475xx.h"

 




 




 




 




 




 




 




 
 
 
 
 
 
#line 13403 "..\\USER\\stm32l475xx.h"





 
#line 13442 "..\\USER\\stm32l475xx.h"

 
#line 13456 "..\\USER\\stm32l475xx.h"



 
#line 13468 "..\\USER\\stm32l475xx.h"



 
#line 13480 "..\\USER\\stm32l475xx.h"



 
#line 13491 "..\\USER\\stm32l475xx.h"



 
#line 13502 "..\\USER\\stm32l475xx.h"

 
#line 13516 "..\\USER\\stm32l475xx.h"


 
#line 13526 "..\\USER\\stm32l475xx.h"



 
#line 13537 "..\\USER\\stm32l475xx.h"



 
#line 13548 "..\\USER\\stm32l475xx.h"



 
#line 13559 "..\\USER\\stm32l475xx.h"

 
#line 13573 "..\\USER\\stm32l475xx.h"



 
#line 13584 "..\\USER\\stm32l475xx.h"



 
#line 13595 "..\\USER\\stm32l475xx.h"



 
#line 13606 "..\\USER\\stm32l475xx.h"



 
#line 13617 "..\\USER\\stm32l475xx.h"

 
#line 13631 "..\\USER\\stm32l475xx.h"



 
#line 13642 "..\\USER\\stm32l475xx.h"



 
#line 13653 "..\\USER\\stm32l475xx.h"



 
#line 13664 "..\\USER\\stm32l475xx.h"



 
#line 13675 "..\\USER\\stm32l475xx.h"

 
#line 13683 "..\\USER\\stm32l475xx.h"

 
#line 13700 "..\\USER\\stm32l475xx.h"

 
#line 13798 "..\\USER\\stm32l475xx.h"

 







 
 
 
 
 
 
#line 13828 "..\\USER\\stm32l475xx.h"





















 
#line 13859 "..\\USER\\stm32l475xx.h"

#line 13866 "..\\USER\\stm32l475xx.h"

#line 13897 "..\\USER\\stm32l475xx.h"

#line 13905 "..\\USER\\stm32l475xx.h"

 
#line 13914 "..\\USER\\stm32l475xx.h"





#line 13925 "..\\USER\\stm32l475xx.h"





#line 13937 "..\\USER\\stm32l475xx.h"







#line 13950 "..\\USER\\stm32l475xx.h"

 
#line 13997 "..\\USER\\stm32l475xx.h"

 
#line 14047 "..\\USER\\stm32l475xx.h"


 
#line 14077 "..\\USER\\stm32l475xx.h"


 






#line 14092 "..\\USER\\stm32l475xx.h"

#line 14100 "..\\USER\\stm32l475xx.h"











#line 14117 "..\\USER\\stm32l475xx.h"

#line 14125 "..\\USER\\stm32l475xx.h"





 






#line 14144 "..\\USER\\stm32l475xx.h"







#line 14158 "..\\USER\\stm32l475xx.h"

 






#line 14172 "..\\USER\\stm32l475xx.h"

#line 14180 "..\\USER\\stm32l475xx.h"











#line 14197 "..\\USER\\stm32l475xx.h"

#line 14205 "..\\USER\\stm32l475xx.h"





 






#line 14224 "..\\USER\\stm32l475xx.h"







#line 14238 "..\\USER\\stm32l475xx.h"

 
#line 14246 "..\\USER\\stm32l475xx.h"

#line 14254 "..\\USER\\stm32l475xx.h"





#line 14265 "..\\USER\\stm32l475xx.h"

#line 14273 "..\\USER\\stm32l475xx.h"





 
#line 14336 "..\\USER\\stm32l475xx.h"

 
#line 14344 "..\\USER\\stm32l475xx.h"

 




 




 




 




 




 




 




 
#line 14393 "..\\USER\\stm32l475xx.h"

 




 
#line 14411 "..\\USER\\stm32l475xx.h"







#line 14436 "..\\USER\\stm32l475xx.h"

#line 14443 "..\\USER\\stm32l475xx.h"

#line 14450 "..\\USER\\stm32l475xx.h"

 
#line 14460 "..\\USER\\stm32l475xx.h"

#line 14469 "..\\USER\\stm32l475xx.h"

 




 
















 
#line 14514 "..\\USER\\stm32l475xx.h"

#line 14521 "..\\USER\\stm32l475xx.h"

 
#line 14544 "..\\USER\\stm32l475xx.h"

 
















 
#line 14584 "..\\USER\\stm32l475xx.h"

#line 14591 "..\\USER\\stm32l475xx.h"

 
#line 14614 "..\\USER\\stm32l475xx.h"

 
#line 14622 "..\\USER\\stm32l475xx.h"







 
#line 14636 "..\\USER\\stm32l475xx.h"

 






 
#line 14651 "..\\USER\\stm32l475xx.h"

 










 
#line 14685 "..\\USER\\stm32l475xx.h"

 






 
#line 14715 "..\\USER\\stm32l475xx.h"

 






 
#line 14745 "..\\USER\\stm32l475xx.h"

 
 
 
 
 
 
#line 14773 "..\\USER\\stm32l475xx.h"

 
#line 14796 "..\\USER\\stm32l475xx.h"

 
#line 14819 "..\\USER\\stm32l475xx.h"

 






















#line 14849 "..\\USER\\stm32l475xx.h"

#line 14856 "..\\USER\\stm32l475xx.h"







#line 14881 "..\\USER\\stm32l475xx.h"

 
#line 14892 "..\\USER\\stm32l475xx.h"

 




 




 




 






 
 
 
 
 
 










#line 14937 "..\\USER\\stm32l475xx.h"




















#line 14963 "..\\USER\\stm32l475xx.h"

#line 14970 "..\\USER\\stm32l475xx.h"









 
 
 
 
 
 
#line 14991 "..\\USER\\stm32l475xx.h"



















#line 15025 "..\\USER\\stm32l475xx.h"

 
#line 15033 "..\\USER\\stm32l475xx.h"



















#line 15067 "..\\USER\\stm32l475xx.h"





 
#line 15079 "..\\USER\\stm32l475xx.h"



















#line 15113 "..\\USER\\stm32l475xx.h"

 
#line 15121 "..\\USER\\stm32l475xx.h"

 
#line 15129 "..\\USER\\stm32l475xx.h"

 
#line 15137 "..\\USER\\stm32l475xx.h"

 
#line 15145 "..\\USER\\stm32l475xx.h"

 
#line 15153 "..\\USER\\stm32l475xx.h"

 
#line 15161 "..\\USER\\stm32l475xx.h"

 
 
 
 
 
 
#line 15183 "..\\USER\\stm32l475xx.h"

#line 15190 "..\\USER\\stm32l475xx.h"

#line 15197 "..\\USER\\stm32l475xx.h"

#line 15204 "..\\USER\\stm32l475xx.h"

#line 15215 "..\\USER\\stm32l475xx.h"

#line 15223 "..\\USER\\stm32l475xx.h"

#line 15231 "..\\USER\\stm32l475xx.h"

 
#line 15239 "..\\USER\\stm32l475xx.h"

 
#line 15247 "..\\USER\\stm32l475xx.h"

 
#line 15255 "..\\USER\\stm32l475xx.h"

 
#line 15353 "..\\USER\\stm32l475xx.h"

 
#line 15451 "..\\USER\\stm32l475xx.h"

 
#line 15549 "..\\USER\\stm32l475xx.h"

 
#line 15647 "..\\USER\\stm32l475xx.h"

 
#line 15697 "..\\USER\\stm32l475xx.h"

 




 
 
 
 
 
 
#line 15785 "..\\USER\\stm32l475xx.h"

 
#line 15845 "..\\USER\\stm32l475xx.h"

 
#line 15912 "..\\USER\\stm32l475xx.h"

 
#line 15920 "..\\USER\\stm32l475xx.h"

 
#line 15928 "..\\USER\\stm32l475xx.h"

 
#line 15936 "..\\USER\\stm32l475xx.h"

 
#line 15953 "..\\USER\\stm32l475xx.h"

 
#line 16021 "..\\USER\\stm32l475xx.h"

 
#line 16059 "..\\USER\\stm32l475xx.h"

 




 




 




 
 
 
 
 

 
#line 16103 "..\\USER\\stm32l475xx.h"

 




 
#line 16143 "..\\USER\\stm32l475xx.h"

 
#line 16166 "..\\USER\\stm32l475xx.h"

 
#line 16195 "..\\USER\\stm32l475xx.h"

 
#line 16203 "..\\USER\\stm32l475xx.h"

 




 




 
#line 16221 "..\\USER\\stm32l475xx.h"

 
 
 
 
 
 
#line 16240 "..\\USER\\stm32l475xx.h"

 




 
 
 
 
 
 
#line 16262 "..\\USER\\stm32l475xx.h"





 
#line 16278 "..\\USER\\stm32l475xx.h"











 





 
 
 
 
 
 
#line 16307 "..\\USER\\stm32l475xx.h"

 
#line 16321 "..\\USER\\stm32l475xx.h"







 
#line 16371 "..\\USER\\stm32l475xx.h"

 




 
#line 16393 "..\\USER\\stm32l475xx.h"

 
 
 
 
 
 
#line 16427 "..\\USER\\stm32l475xx.h"

 
#line 16447 "..\\USER\\stm32l475xx.h"

 
#line 16468 "..\\USER\\stm32l475xx.h"

 
#line 16531 "..\\USER\\stm32l475xx.h"

 
#line 16562 "..\\USER\\stm32l475xx.h"

 
#line 16645 "..\\USER\\stm32l475xx.h"

 
#line 16728 "..\\USER\\stm32l475xx.h"

 
 
#line 16738 "..\\USER\\stm32l475xx.h"
 
#line 16753 "..\\USER\\stm32l475xx.h"
 
#line 16769 "..\\USER\\stm32l475xx.h"

 
#line 16783 "..\\USER\\stm32l475xx.h"

 




 
#line 16802 "..\\USER\\stm32l475xx.h"

 
#line 16818 "..\\USER\\stm32l475xx.h"

#line 16829 "..\\USER\\stm32l475xx.h"

 
#line 16861 "..\\USER\\stm32l475xx.h"

 




 
#line 16913 "..\\USER\\stm32l475xx.h"

 




 




 
#line 16931 "..\\USER\\stm32l475xx.h"

 
#line 16939 "..\\USER\\stm32l475xx.h"

 
#line 16949 "..\\USER\\stm32l475xx.h"

 




 
#line 16962 "..\\USER\\stm32l475xx.h"

 
#line 16978 "..\\USER\\stm32l475xx.h"

#line 16990 "..\\USER\\stm32l475xx.h"

 




 




 
#line 17029 "..\\USER\\stm32l475xx.h"

#line 17038 "..\\USER\\stm32l475xx.h"

#line 17046 "..\\USER\\stm32l475xx.h"







 




#line 17071 "..\\USER\\stm32l475xx.h"













#line 17103 "..\\USER\\stm32l475xx.h"

 
#line 17138 "..\\USER\\stm32l475xx.h"

 
#line 17173 "..\\USER\\stm32l475xx.h"

 
#line 17189 "..\\USER\\stm32l475xx.h"

 




 
#line 17224 "..\\USER\\stm32l475xx.h"

 
#line 17259 "..\\USER\\stm32l475xx.h"

 
#line 17275 "..\\USER\\stm32l475xx.h"

 
#line 17301 "..\\USER\\stm32l475xx.h"

 
#line 17330 "..\\USER\\stm32l475xx.h"

 
#line 17353 "..\\USER\\stm32l475xx.h"

 
#line 17388 "..\\USER\\stm32l475xx.h"

 
#line 17396 "..\\USER\\stm32l475xx.h"

 
#line 17404 "..\\USER\\stm32l475xx.h"

 




 




 
#line 17452 "..\\USER\\stm32l475xx.h"

 




 
#line 17465 "..\\USER\\stm32l475xx.h"

 
#line 17473 "..\\USER\\stm32l475xx.h"

 
#line 17520 "..\\USER\\stm32l475xx.h"

 
#line 17555 "..\\USER\\stm32l475xx.h"

 
#line 17566 "..\\USER\\stm32l475xx.h"

 




 




 
#line 17616 "..\\USER\\stm32l475xx.h"

 
#line 17636 "..\\USER\\stm32l475xx.h"

 
#line 17649 "..\\USER\\stm32l475xx.h"

 
#line 17660 "..\\USER\\stm32l475xx.h"

 
#line 17711 "..\\USER\\stm32l475xx.h"




 



 



 

 








 


 





 


 


 


 





#line 17766 "..\\USER\\stm32l475xx.h"

 
#line 17782 "..\\USER\\stm32l475xx.h"

 
#line 17792 "..\\USER\\stm32l475xx.h"

 
 


 
 


 




 


 


 





 


 


 


 


 





 


 




 




 


 



 
#line 17868 "..\\USER\\stm32l475xx.h"

 



 






 






 



 
#line 17901 "..\\USER\\stm32l475xx.h"

 
#line 17910 "..\\USER\\stm32l475xx.h"

 
#line 17918 "..\\USER\\stm32l475xx.h"

 
#line 17926 "..\\USER\\stm32l475xx.h"

 



 



 






 
#line 17954 "..\\USER\\stm32l475xx.h"

 
#line 17965 "..\\USER\\stm32l475xx.h"

 
#line 17976 "..\\USER\\stm32l475xx.h"

 
#line 18028 "..\\USER\\stm32l475xx.h"

 
#line 18049 "..\\USER\\stm32l475xx.h"

 
#line 18060 "..\\USER\\stm32l475xx.h"

 
#line 18069 "..\\USER\\stm32l475xx.h"

 
#line 18077 "..\\USER\\stm32l475xx.h"

 
#line 18086 "..\\USER\\stm32l475xx.h"

 
#line 18095 "..\\USER\\stm32l475xx.h"

 



 






 
#line 18114 "..\\USER\\stm32l475xx.h"

 
#line 18122 "..\\USER\\stm32l475xx.h"

 
#line 18130 "..\\USER\\stm32l475xx.h"

 
#line 18138 "..\\USER\\stm32l475xx.h"

 





 
#line 18155 "..\\USER\\stm32l475xx.h"

 
#line 18164 "..\\USER\\stm32l475xx.h"

 
#line 18172 "..\\USER\\stm32l475xx.h"

 
#line 18181 "..\\USER\\stm32l475xx.h"

 






 


 



 
#line 18204 "..\\USER\\stm32l475xx.h"

 



 


 




 






 






 
#line 18238 "..\\USER\\stm32l475xx.h"

 
#line 18246 "..\\USER\\stm32l475xx.h"

 
#line 18254 "..\\USER\\stm32l475xx.h"

 






 
#line 18269 "..\\USER\\stm32l475xx.h"

 






 




 


 


 




 


 
 
 
 
 
 
 

 
#line 18314 "..\\USER\\stm32l475xx.h"

 
#line 18325 "..\\USER\\stm32l475xx.h"









 

  

 

 
#line 163 "..\\USER\\stm32l4xx.h"
#line 188 "..\\USER\\stm32l4xx.h"



 



 
typedef enum
{
  RESET = 0,
  SET = !RESET
} FlagStatus, ITStatus;

typedef enum
{
  DISABLE = 0,
  ENABLE = !DISABLE
} FunctionalState;


typedef enum
{
  ERROR = 0,
  SUCCESS = !ERROR
} ErrorStatus;



 




 



















 

#line 1 "..\\HALLIB\\STM32L4xx_HAL_Driver\\Inc\\stm32l4xx_hal.h"


































 

 
#line 678 "..\\HALLIB\\STM32L4xx_HAL_Driver\\Inc\\stm32l4xx_hal.h"

 
#line 246 "..\\USER\\stm32l4xx.h"









 



 




 
#line 47 "..\\HALLIB\\STM32L4xx_HAL_Driver\\Inc\\stm32l4xx_hal_def.h"
#line 1 "..\\HALLIB\\STM32L4xx_HAL_Driver\\Inc\\Legacy/stm32_hal_legacy.h"


































 

 







 
 
 



 








 



 
#line 105 "..\\HALLIB\\STM32L4xx_HAL_Driver\\Inc\\Legacy/stm32_hal_legacy.h"

#line 113 "..\\HALLIB\\STM32L4xx_HAL_Driver\\Inc\\Legacy/stm32_hal_legacy.h"


 



 





 



 
#line 147 "..\\HALLIB\\STM32L4xx_HAL_Driver\\Inc\\Legacy/stm32_hal_legacy.h"




#line 157 "..\\HALLIB\\STM32L4xx_HAL_Driver\\Inc\\Legacy/stm32_hal_legacy.h"

#line 179 "..\\HALLIB\\STM32L4xx_HAL_Driver\\Inc\\Legacy/stm32_hal_legacy.h"




 
 
#line 192 "..\\HALLIB\\STM32L4xx_HAL_Driver\\Inc\\Legacy/stm32_hal_legacy.h"

#line 202 "..\\HALLIB\\STM32L4xx_HAL_Driver\\Inc\\Legacy/stm32_hal_legacy.h"

#line 212 "..\\HALLIB\\STM32L4xx_HAL_Driver\\Inc\\Legacy/stm32_hal_legacy.h"




 



 



 



 






 



 

#line 250 "..\\HALLIB\\STM32L4xx_HAL_Driver\\Inc\\Legacy/stm32_hal_legacy.h"



 



 
#line 272 "..\\HALLIB\\STM32L4xx_HAL_Driver\\Inc\\Legacy/stm32_hal_legacy.h"







#line 305 "..\\HALLIB\\STM32L4xx_HAL_Driver\\Inc\\Legacy/stm32_hal_legacy.h"








#line 367 "..\\HALLIB\\STM32L4xx_HAL_Driver\\Inc\\Legacy/stm32_hal_legacy.h"



 



 

#line 449 "..\\HALLIB\\STM32L4xx_HAL_Driver\\Inc\\Legacy/stm32_hal_legacy.h"

#line 458 "..\\HALLIB\\STM32L4xx_HAL_Driver\\Inc\\Legacy/stm32_hal_legacy.h"



 



 

#line 475 "..\\HALLIB\\STM32L4xx_HAL_Driver\\Inc\\Legacy/stm32_hal_legacy.h"



 



 

#line 493 "..\\HALLIB\\STM32L4xx_HAL_Driver\\Inc\\Legacy/stm32_hal_legacy.h"


 




 
#line 512 "..\\HALLIB\\STM32L4xx_HAL_Driver\\Inc\\Legacy/stm32_hal_legacy.h"


 



 





 



 


















#line 555 "..\\HALLIB\\STM32L4xx_HAL_Driver\\Inc\\Legacy/stm32_hal_legacy.h"





#line 566 "..\\HALLIB\\STM32L4xx_HAL_Driver\\Inc\\Legacy/stm32_hal_legacy.h"

#line 573 "..\\HALLIB\\STM32L4xx_HAL_Driver\\Inc\\Legacy/stm32_hal_legacy.h"










 



 
#line 597 "..\\HALLIB\\STM32L4xx_HAL_Driver\\Inc\\Legacy/stm32_hal_legacy.h"

#line 606 "..\\HALLIB\\STM32L4xx_HAL_Driver\\Inc\\Legacy/stm32_hal_legacy.h"


 



 
#line 629 "..\\HALLIB\\STM32L4xx_HAL_Driver\\Inc\\Legacy/stm32_hal_legacy.h"


 



 





 



 






 



 















 
 






 



 














 



 










 



 




























 



 


#line 762 "..\\HALLIB\\STM32L4xx_HAL_Driver\\Inc\\Legacy/stm32_hal_legacy.h"






 



 

 
#line 784 "..\\HALLIB\\STM32L4xx_HAL_Driver\\Inc\\Legacy/stm32_hal_legacy.h"

 












 



 






























 




 















 




 
#line 871 "..\\HALLIB\\STM32L4xx_HAL_Driver\\Inc\\Legacy/stm32_hal_legacy.h"


 



 









#line 901 "..\\HALLIB\\STM32L4xx_HAL_Driver\\Inc\\Legacy/stm32_hal_legacy.h"



 



 



#line 939 "..\\HALLIB\\STM32L4xx_HAL_Driver\\Inc\\Legacy/stm32_hal_legacy.h"

#line 949 "..\\HALLIB\\STM32L4xx_HAL_Driver\\Inc\\Legacy/stm32_hal_legacy.h"

#line 968 "..\\HALLIB\\STM32L4xx_HAL_Driver\\Inc\\Legacy/stm32_hal_legacy.h"











 



 




 



 

























 




 








 



 




 



 
#line 1056 "..\\HALLIB\\STM32L4xx_HAL_Driver\\Inc\\Legacy/stm32_hal_legacy.h"



 



 

#line 1073 "..\\HALLIB\\STM32L4xx_HAL_Driver\\Inc\\Legacy/stm32_hal_legacy.h"

#line 1085 "..\\HALLIB\\STM32L4xx_HAL_Driver\\Inc\\Legacy/stm32_hal_legacy.h"

#line 1116 "..\\HALLIB\\STM32L4xx_HAL_Driver\\Inc\\Legacy/stm32_hal_legacy.h"



 



 











 





 






#line 1159 "..\\HALLIB\\STM32L4xx_HAL_Driver\\Inc\\Legacy/stm32_hal_legacy.h"


 




 



 

 



 



 



 
#line 1191 "..\\HALLIB\\STM32L4xx_HAL_Driver\\Inc\\Legacy/stm32_hal_legacy.h"

 













 



 
#line 1226 "..\\HALLIB\\STM32L4xx_HAL_Driver\\Inc\\Legacy/stm32_hal_legacy.h"


 



 
#line 1240 "..\\HALLIB\\STM32L4xx_HAL_Driver\\Inc\\Legacy/stm32_hal_legacy.h"

 

 



 






 

 



 
#line 1277 "..\\HALLIB\\STM32L4xx_HAL_Driver\\Inc\\Legacy/stm32_hal_legacy.h"

#line 1285 "..\\HALLIB\\STM32L4xx_HAL_Driver\\Inc\\Legacy/stm32_hal_legacy.h"






#line 1301 "..\\HALLIB\\STM32L4xx_HAL_Driver\\Inc\\Legacy/stm32_hal_legacy.h"



 

 



 





 



 



 



 
#line 1341 "..\\HALLIB\\STM32L4xx_HAL_Driver\\Inc\\Legacy/stm32_hal_legacy.h"


 



 



 



 






 




 



 

 



 





 



 
#line 1402 "..\\HALLIB\\STM32L4xx_HAL_Driver\\Inc\\Legacy/stm32_hal_legacy.h"









 




 
#line 1430 "..\\HALLIB\\STM32L4xx_HAL_Driver\\Inc\\Legacy/stm32_hal_legacy.h"

#line 1451 "..\\HALLIB\\STM32L4xx_HAL_Driver\\Inc\\Legacy/stm32_hal_legacy.h"

#line 1462 "..\\HALLIB\\STM32L4xx_HAL_Driver\\Inc\\Legacy/stm32_hal_legacy.h"

#line 1471 "..\\HALLIB\\STM32L4xx_HAL_Driver\\Inc\\Legacy/stm32_hal_legacy.h"

#line 1484 "..\\HALLIB\\STM32L4xx_HAL_Driver\\Inc\\Legacy/stm32_hal_legacy.h"

#line 1493 "..\\HALLIB\\STM32L4xx_HAL_Driver\\Inc\\Legacy/stm32_hal_legacy.h"



 



 







 



 
#line 1529 "..\\HALLIB\\STM32L4xx_HAL_Driver\\Inc\\Legacy/stm32_hal_legacy.h"

#line 1544 "..\\HALLIB\\STM32L4xx_HAL_Driver\\Inc\\Legacy/stm32_hal_legacy.h"


#line 1570 "..\\HALLIB\\STM32L4xx_HAL_Driver\\Inc\\Legacy/stm32_hal_legacy.h"



 



 
#line 1737 "..\\HALLIB\\STM32L4xx_HAL_Driver\\Inc\\Legacy/stm32_hal_legacy.h"




 
 
 
 




 




 




 




 







 



 

#line 1784 "..\\HALLIB\\STM32L4xx_HAL_Driver\\Inc\\Legacy/stm32_hal_legacy.h"



 



 

#line 1812 "..\\HALLIB\\STM32L4xx_HAL_Driver\\Inc\\Legacy/stm32_hal_legacy.h"


 



 










 



 














 




 




 




 







 




 
#line 1890 "..\\HALLIB\\STM32L4xx_HAL_Driver\\Inc\\Legacy/stm32_hal_legacy.h"



 




 
#line 1934 "..\\HALLIB\\STM32L4xx_HAL_Driver\\Inc\\Legacy/stm32_hal_legacy.h"

#line 1948 "..\\HALLIB\\STM32L4xx_HAL_Driver\\Inc\\Legacy/stm32_hal_legacy.h"


 




 







#line 2221 "..\\HALLIB\\STM32L4xx_HAL_Driver\\Inc\\Legacy/stm32_hal_legacy.h"


#line 2465 "..\\HALLIB\\STM32L4xx_HAL_Driver\\Inc\\Legacy/stm32_hal_legacy.h"

#line 2613 "..\\HALLIB\\STM32L4xx_HAL_Driver\\Inc\\Legacy/stm32_hal_legacy.h"

 



#line 2638 "..\\HALLIB\\STM32L4xx_HAL_Driver\\Inc\\Legacy/stm32_hal_legacy.h"

#line 2659 "..\\HALLIB\\STM32L4xx_HAL_Driver\\Inc\\Legacy/stm32_hal_legacy.h"

#line 2776 "..\\HALLIB\\STM32L4xx_HAL_Driver\\Inc\\Legacy/stm32_hal_legacy.h"

#line 2793 "..\\HALLIB\\STM32L4xx_HAL_Driver\\Inc\\Legacy/stm32_hal_legacy.h"

#line 2808 "..\\HALLIB\\STM32L4xx_HAL_Driver\\Inc\\Legacy/stm32_hal_legacy.h"






#line 2837 "..\\HALLIB\\STM32L4xx_HAL_Driver\\Inc\\Legacy/stm32_hal_legacy.h"

















#line 2863 "..\\HALLIB\\STM32L4xx_HAL_Driver\\Inc\\Legacy/stm32_hal_legacy.h"





#line 2890 "..\\HALLIB\\STM32L4xx_HAL_Driver\\Inc\\Legacy/stm32_hal_legacy.h"







#line 2905 "..\\HALLIB\\STM32L4xx_HAL_Driver\\Inc\\Legacy/stm32_hal_legacy.h"

#line 2938 "..\\HALLIB\\STM32L4xx_HAL_Driver\\Inc\\Legacy/stm32_hal_legacy.h"

#line 2956 "..\\HALLIB\\STM32L4xx_HAL_Driver\\Inc\\Legacy/stm32_hal_legacy.h"












#line 2974 "..\\HALLIB\\STM32L4xx_HAL_Driver\\Inc\\Legacy/stm32_hal_legacy.h"

#line 2995 "..\\HALLIB\\STM32L4xx_HAL_Driver\\Inc\\Legacy/stm32_hal_legacy.h"

#line 3003 "..\\HALLIB\\STM32L4xx_HAL_Driver\\Inc\\Legacy/stm32_hal_legacy.h"



 



 




 



 
#line 3026 "..\\HALLIB\\STM32L4xx_HAL_Driver\\Inc\\Legacy/stm32_hal_legacy.h"

#line 3054 "..\\HALLIB\\STM32L4xx_HAL_Driver\\Inc\\Legacy/stm32_hal_legacy.h"

#line 3069 "..\\HALLIB\\STM32L4xx_HAL_Driver\\Inc\\Legacy/stm32_hal_legacy.h"






 



 




#line 3109 "..\\HALLIB\\STM32L4xx_HAL_Driver\\Inc\\Legacy/stm32_hal_legacy.h"

#line 3131 "..\\HALLIB\\STM32L4xx_HAL_Driver\\Inc\\Legacy/stm32_hal_legacy.h"
 




#line 3142 "..\\HALLIB\\STM32L4xx_HAL_Driver\\Inc\\Legacy/stm32_hal_legacy.h"

#line 3154 "..\\HALLIB\\STM32L4xx_HAL_Driver\\Inc\\Legacy/stm32_hal_legacy.h"


 



 

#line 3168 "..\\HALLIB\\STM32L4xx_HAL_Driver\\Inc\\Legacy/stm32_hal_legacy.h"








 



 
#line 3189 "..\\HALLIB\\STM32L4xx_HAL_Driver\\Inc\\Legacy/stm32_hal_legacy.h"


 



 







 



 













 




 











 



 












#line 3262 "..\\HALLIB\\STM32L4xx_HAL_Driver\\Inc\\Legacy/stm32_hal_legacy.h"

#line 3271 "..\\HALLIB\\STM32L4xx_HAL_Driver\\Inc\\Legacy/stm32_hal_legacy.h"

#line 3280 "..\\HALLIB\\STM32L4xx_HAL_Driver\\Inc\\Legacy/stm32_hal_legacy.h"








 



 








#line 3313 "..\\HALLIB\\STM32L4xx_HAL_Driver\\Inc\\Legacy/stm32_hal_legacy.h"




 



 

#line 3330 "..\\HALLIB\\STM32L4xx_HAL_Driver\\Inc\\Legacy/stm32_hal_legacy.h"






 



 




 



 
#line 3364 "..\\HALLIB\\STM32L4xx_HAL_Driver\\Inc\\Legacy/stm32_hal_legacy.h"


 



 







 



 



 







 

#line 48 "..\\HALLIB\\STM32L4xx_HAL_Driver\\Inc\\stm32l4xx_hal_def.h"
#line 1 "F:\\KEIL5\\ARM\\ARMCC\\Bin\\..\\include\\stddef.h"
 






 

 
 
 





 





#line 34 "F:\\KEIL5\\ARM\\ARMCC\\Bin\\..\\include\\stddef.h"




  typedef signed int ptrdiff_t;



  



    typedef unsigned int size_t;    
#line 57 "F:\\KEIL5\\ARM\\ARMCC\\Bin\\..\\include\\stddef.h"



   



      typedef unsigned short wchar_t;  
#line 82 "F:\\KEIL5\\ARM\\ARMCC\\Bin\\..\\include\\stddef.h"



    




   




  typedef long double max_align_t;









#line 114 "F:\\KEIL5\\ARM\\ARMCC\\Bin\\..\\include\\stddef.h"



 

#line 49 "..\\HALLIB\\STM32L4xx_HAL_Driver\\Inc\\stm32l4xx_hal_def.h"

 



 
typedef enum
{
  HAL_OK       = 0x00,
  HAL_ERROR    = 0x01,
  HAL_BUSY     = 0x02,
  HAL_TIMEOUT  = 0x03
} HAL_StatusTypeDef;



 
typedef enum
{
  HAL_UNLOCKED = 0x00,
  HAL_LOCKED   = 0x01
} HAL_LockTypeDef;

 




























 


#line 119 "..\\HALLIB\\STM32L4xx_HAL_Driver\\Inc\\stm32l4xx_hal_def.h"







#line 134 "..\\HALLIB\\STM32L4xx_HAL_Driver\\Inc\\stm32l4xx_hal_def.h"


 
#line 156 "..\\HALLIB\\STM32L4xx_HAL_Driver\\Inc\\stm32l4xx_hal_def.h"



 









 


#line 188 "..\\HALLIB\\STM32L4xx_HAL_Driver\\Inc\\stm32l4xx_hal_def.h"



 



 


#line 205 "..\\HALLIB\\STM32L4xx_HAL_Driver\\Inc\\stm32l4xx_hal_def.h"








 
#line 46 "..\\HALLIB\\STM32L4xx_HAL_Driver\\Inc\\stm32l4xx_hal_rcc.h"



 



 

 


 



 
typedef struct
{
  uint32_t PLLState;   
 

  uint32_t PLLSource;  
 

  uint32_t PLLM;       

 

  uint32_t PLLN;       
 


  uint32_t PLLP;       
 


  uint32_t PLLQ;       
 

  uint32_t PLLR;       


 

}RCC_PLLInitTypeDef;



 
typedef struct
{
  uint32_t OscillatorType;       
 

  uint32_t HSEState;             
 

  uint32_t LSEState;             
 

  uint32_t HSIState;             
 

  uint32_t HSICalibrationValue;  

 

  uint32_t LSIState;             
 






  uint32_t MSIState;             
 

  uint32_t MSICalibrationValue;  
 

  uint32_t MSIClockRange;        
 

  uint32_t HSI48State;             
 

  RCC_PLLInitTypeDef PLL;         

}RCC_OscInitTypeDef;



 
typedef struct
{
  uint32_t ClockType;             
 

  uint32_t SYSCLKSource;          
 

  uint32_t AHBCLKDivider;         
 

  uint32_t APB1CLKDivider;        
 

  uint32_t APB2CLKDivider;        
 

}RCC_ClkInitTypeDef;



 

 


 



 




 



 
#line 190 "..\\HALLIB\\STM32L4xx_HAL_Driver\\Inc\\stm32l4xx_hal_rcc.h"


 



 





 



 
#line 214 "..\\HALLIB\\STM32L4xx_HAL_Driver\\Inc\\stm32l4xx_hal_rcc.h"


 



 



#line 230 "..\\HALLIB\\STM32L4xx_HAL_Driver\\Inc\\stm32l4xx_hal_rcc.h"
        


 



 




 
#line 254 "..\\HALLIB\\STM32L4xx_HAL_Driver\\Inc\\stm32l4xx_hal_rcc.h"



 






 

#line 276 "..\\HALLIB\\STM32L4xx_HAL_Driver\\Inc\\stm32l4xx_hal_rcc.h"


 



 




 





 




 
#line 334 "..\\HALLIB\\STM32L4xx_HAL_Driver\\Inc\\stm32l4xx_hal_rcc.h"


 




 






 



 






 



 






 



 
#line 382 "..\\HALLIB\\STM32L4xx_HAL_Driver\\Inc\\stm32l4xx_hal_rcc.h"


 




 





 






 
#line 412 "..\\HALLIB\\STM32L4xx_HAL_Driver\\Inc\\stm32l4xx_hal_rcc.h"


 





 
#line 433 "..\\HALLIB\\STM32L4xx_HAL_Driver\\Inc\\stm32l4xx_hal_rcc.h"


 



 






 



 






 



 






 



 
#line 482 "..\\HALLIB\\STM32L4xx_HAL_Driver\\Inc\\stm32l4xx_hal_rcc.h"


 



 







 



 






 



 




 



 
#line 532 "..\\HALLIB\\STM32L4xx_HAL_Driver\\Inc\\stm32l4xx_hal_rcc.h"


 



 







 



 
#line 568 "..\\HALLIB\\STM32L4xx_HAL_Driver\\Inc\\stm32l4xx_hal_rcc.h"


 










 
 
#line 593 "..\\HALLIB\\STM32L4xx_HAL_Driver\\Inc\\stm32l4xx_hal_rcc.h"

 



 
#line 608 "..\\HALLIB\\STM32L4xx_HAL_Driver\\Inc\\stm32l4xx_hal_rcc.h"







 



 






 



 




 



 

 



 







 

#line 662 "..\\HALLIB\\STM32L4xx_HAL_Driver\\Inc\\stm32l4xx_hal_rcc.h"

#line 670 "..\\HALLIB\\STM32L4xx_HAL_Driver\\Inc\\stm32l4xx_hal_rcc.h"

#line 680 "..\\HALLIB\\STM32L4xx_HAL_Driver\\Inc\\stm32l4xx_hal_rcc.h"

#line 688 "..\\HALLIB\\STM32L4xx_HAL_Driver\\Inc\\stm32l4xx_hal_rcc.h"

#line 696 "..\\HALLIB\\STM32L4xx_HAL_Driver\\Inc\\stm32l4xx_hal_rcc.h"

#line 704 "..\\HALLIB\\STM32L4xx_HAL_Driver\\Inc\\stm32l4xx_hal_rcc.h"

#line 714 "..\\HALLIB\\STM32L4xx_HAL_Driver\\Inc\\stm32l4xx_hal_rcc.h"

#line 724 "..\\HALLIB\\STM32L4xx_HAL_Driver\\Inc\\stm32l4xx_hal_rcc.h"


























 







 

#line 767 "..\\HALLIB\\STM32L4xx_HAL_Driver\\Inc\\stm32l4xx_hal_rcc.h"

#line 775 "..\\HALLIB\\STM32L4xx_HAL_Driver\\Inc\\stm32l4xx_hal_rcc.h"

#line 783 "..\\HALLIB\\STM32L4xx_HAL_Driver\\Inc\\stm32l4xx_hal_rcc.h"

#line 793 "..\\HALLIB\\STM32L4xx_HAL_Driver\\Inc\\stm32l4xx_hal_rcc.h"

#line 803 "..\\HALLIB\\STM32L4xx_HAL_Driver\\Inc\\stm32l4xx_hal_rcc.h"

#line 813 "..\\HALLIB\\STM32L4xx_HAL_Driver\\Inc\\stm32l4xx_hal_rcc.h"

#line 823 "..\\HALLIB\\STM32L4xx_HAL_Driver\\Inc\\stm32l4xx_hal_rcc.h"

#line 831 "..\\HALLIB\\STM32L4xx_HAL_Driver\\Inc\\stm32l4xx_hal_rcc.h"

#line 841 "..\\HALLIB\\STM32L4xx_HAL_Driver\\Inc\\stm32l4xx_hal_rcc.h"

#line 851 "..\\HALLIB\\STM32L4xx_HAL_Driver\\Inc\\stm32l4xx_hal_rcc.h"

#line 859 "..\\HALLIB\\STM32L4xx_HAL_Driver\\Inc\\stm32l4xx_hal_rcc.h"

#line 869 "..\\HALLIB\\STM32L4xx_HAL_Driver\\Inc\\stm32l4xx_hal_rcc.h"

#line 879 "..\\HALLIB\\STM32L4xx_HAL_Driver\\Inc\\stm32l4xx_hal_rcc.h"

#line 889 "..\\HALLIB\\STM32L4xx_HAL_Driver\\Inc\\stm32l4xx_hal_rcc.h"

#line 897 "..\\HALLIB\\STM32L4xx_HAL_Driver\\Inc\\stm32l4xx_hal_rcc.h"

#line 907 "..\\HALLIB\\STM32L4xx_HAL_Driver\\Inc\\stm32l4xx_hal_rcc.h"

#line 917 "..\\HALLIB\\STM32L4xx_HAL_Driver\\Inc\\stm32l4xx_hal_rcc.h"




























































 







 

#line 996 "..\\HALLIB\\STM32L4xx_HAL_Driver\\Inc\\stm32l4xx_hal_rcc.h"

#line 1006 "..\\HALLIB\\STM32L4xx_HAL_Driver\\Inc\\stm32l4xx_hal_rcc.h"

#line 1016 "..\\HALLIB\\STM32L4xx_HAL_Driver\\Inc\\stm32l4xx_hal_rcc.h"

#line 1026 "..\\HALLIB\\STM32L4xx_HAL_Driver\\Inc\\stm32l4xx_hal_rcc.h"



















 







 

#line 1062 "..\\HALLIB\\STM32L4xx_HAL_Driver\\Inc\\stm32l4xx_hal_rcc.h"

#line 1072 "..\\HALLIB\\STM32L4xx_HAL_Driver\\Inc\\stm32l4xx_hal_rcc.h"

#line 1082 "..\\HALLIB\\STM32L4xx_HAL_Driver\\Inc\\stm32l4xx_hal_rcc.h"

#line 1092 "..\\HALLIB\\STM32L4xx_HAL_Driver\\Inc\\stm32l4xx_hal_rcc.h"

#line 1100 "..\\HALLIB\\STM32L4xx_HAL_Driver\\Inc\\stm32l4xx_hal_rcc.h"

#line 1110 "..\\HALLIB\\STM32L4xx_HAL_Driver\\Inc\\stm32l4xx_hal_rcc.h"

#line 1120 "..\\HALLIB\\STM32L4xx_HAL_Driver\\Inc\\stm32l4xx_hal_rcc.h"

#line 1130 "..\\HALLIB\\STM32L4xx_HAL_Driver\\Inc\\stm32l4xx_hal_rcc.h"

#line 1138 "..\\HALLIB\\STM32L4xx_HAL_Driver\\Inc\\stm32l4xx_hal_rcc.h"

#line 1148 "..\\HALLIB\\STM32L4xx_HAL_Driver\\Inc\\stm32l4xx_hal_rcc.h"

#line 1158 "..\\HALLIB\\STM32L4xx_HAL_Driver\\Inc\\stm32l4xx_hal_rcc.h"

#line 1166 "..\\HALLIB\\STM32L4xx_HAL_Driver\\Inc\\stm32l4xx_hal_rcc.h"

#line 1176 "..\\HALLIB\\STM32L4xx_HAL_Driver\\Inc\\stm32l4xx_hal_rcc.h"

#line 1186 "..\\HALLIB\\STM32L4xx_HAL_Driver\\Inc\\stm32l4xx_hal_rcc.h"

#line 1196 "..\\HALLIB\\STM32L4xx_HAL_Driver\\Inc\\stm32l4xx_hal_rcc.h"

#line 1204 "..\\HALLIB\\STM32L4xx_HAL_Driver\\Inc\\stm32l4xx_hal_rcc.h"

#line 1214 "..\\HALLIB\\STM32L4xx_HAL_Driver\\Inc\\stm32l4xx_hal_rcc.h"

#line 1222 "..\\HALLIB\\STM32L4xx_HAL_Driver\\Inc\\stm32l4xx_hal_rcc.h"

#line 1232 "..\\HALLIB\\STM32L4xx_HAL_Driver\\Inc\\stm32l4xx_hal_rcc.h"

#line 1242 "..\\HALLIB\\STM32L4xx_HAL_Driver\\Inc\\stm32l4xx_hal_rcc.h"

#line 1252 "..\\HALLIB\\STM32L4xx_HAL_Driver\\Inc\\stm32l4xx_hal_rcc.h"

#line 1262 "..\\HALLIB\\STM32L4xx_HAL_Driver\\Inc\\stm32l4xx_hal_rcc.h"

#line 1272 "..\\HALLIB\\STM32L4xx_HAL_Driver\\Inc\\stm32l4xx_hal_rcc.h"

#line 1280 "..\\HALLIB\\STM32L4xx_HAL_Driver\\Inc\\stm32l4xx_hal_rcc.h"

#line 1290 "..\\HALLIB\\STM32L4xx_HAL_Driver\\Inc\\stm32l4xx_hal_rcc.h"

#line 1298 "..\\HALLIB\\STM32L4xx_HAL_Driver\\Inc\\stm32l4xx_hal_rcc.h"

#line 1306 "..\\HALLIB\\STM32L4xx_HAL_Driver\\Inc\\stm32l4xx_hal_rcc.h"

#line 1314 "..\\HALLIB\\STM32L4xx_HAL_Driver\\Inc\\stm32l4xx_hal_rcc.h"

#line 1324 "..\\HALLIB\\STM32L4xx_HAL_Driver\\Inc\\stm32l4xx_hal_rcc.h"

#line 1332 "..\\HALLIB\\STM32L4xx_HAL_Driver\\Inc\\stm32l4xx_hal_rcc.h"




































































































 







 

#line 1449 "..\\HALLIB\\STM32L4xx_HAL_Driver\\Inc\\stm32l4xx_hal_rcc.h"

#line 1457 "..\\HALLIB\\STM32L4xx_HAL_Driver\\Inc\\stm32l4xx_hal_rcc.h"

#line 1467 "..\\HALLIB\\STM32L4xx_HAL_Driver\\Inc\\stm32l4xx_hal_rcc.h"

#line 1475 "..\\HALLIB\\STM32L4xx_HAL_Driver\\Inc\\stm32l4xx_hal_rcc.h"

#line 1483 "..\\HALLIB\\STM32L4xx_HAL_Driver\\Inc\\stm32l4xx_hal_rcc.h"

#line 1493 "..\\HALLIB\\STM32L4xx_HAL_Driver\\Inc\\stm32l4xx_hal_rcc.h"

#line 1501 "..\\HALLIB\\STM32L4xx_HAL_Driver\\Inc\\stm32l4xx_hal_rcc.h"


#line 1510 "..\\HALLIB\\STM32L4xx_HAL_Driver\\Inc\\stm32l4xx_hal_rcc.h"

#line 1518 "..\\HALLIB\\STM32L4xx_HAL_Driver\\Inc\\stm32l4xx_hal_rcc.h"

#line 1528 "..\\HALLIB\\STM32L4xx_HAL_Driver\\Inc\\stm32l4xx_hal_rcc.h"

#line 1538 "..\\HALLIB\\STM32L4xx_HAL_Driver\\Inc\\stm32l4xx_hal_rcc.h"

#line 1548 "..\\HALLIB\\STM32L4xx_HAL_Driver\\Inc\\stm32l4xx_hal_rcc.h"

#line 1558 "..\\HALLIB\\STM32L4xx_HAL_Driver\\Inc\\stm32l4xx_hal_rcc.h"

#line 1568 "..\\HALLIB\\STM32L4xx_HAL_Driver\\Inc\\stm32l4xx_hal_rcc.h"

#line 1578 "..\\HALLIB\\STM32L4xx_HAL_Driver\\Inc\\stm32l4xx_hal_rcc.h"
















































 







 
















































 







 




































































































 







 



















 







 








































































































































































































 







 






























































































 




 



















































 




 























































































































 




 






































 




 































































































































































































 




 































































































 








 




















































 








 
































































































































 








 



































 








 








































































































































































































 








 




























































































 








 




















































 








 
































































































































 








 




































 








 








































































































































































































 








 




























































































 



 






 






 



 








 






 
















 











 









 












 




















 













 




























 
















 


















 













 


























 
#line 4020 "..\\HALLIB\\STM32L4xx_HAL_Driver\\Inc\\stm32l4xx_hal_rcc.h"




















 
#line 4058 "..\\HALLIB\\STM32L4xx_HAL_Driver\\Inc\\stm32l4xx_hal_rcc.h"

#line 4073 "..\\HALLIB\\STM32L4xx_HAL_Driver\\Inc\\stm32l4xx_hal_rcc.h"
























 










 









 















 













 









































 
#line 4199 "..\\HALLIB\\STM32L4xx_HAL_Driver\\Inc\\stm32l4xx_hal_rcc.h"






#line 4212 "..\\HALLIB\\STM32L4xx_HAL_Driver\\Inc\\stm32l4xx_hal_rcc.h"








 















 














 











 










 















 










 




























 






 




















 





















 






















 






















 






 





























 
#line 4476 "..\\HALLIB\\STM32L4xx_HAL_Driver\\Inc\\stm32l4xx_hal_rcc.h"



 



 

 


 
 
#line 4496 "..\\HALLIB\\STM32L4xx_HAL_Driver\\Inc\\stm32l4xx_hal_rcc.h"




 

 


 

#line 4523 "..\\HALLIB\\STM32L4xx_HAL_Driver\\Inc\\stm32l4xx_hal_rcc.h"




#line 4534 "..\\HALLIB\\STM32L4xx_HAL_Driver\\Inc\\stm32l4xx_hal_rcc.h"















































#line 4587 "..\\HALLIB\\STM32L4xx_HAL_Driver\\Inc\\stm32l4xx_hal_rcc.h"

#line 4600 "..\\HALLIB\\STM32L4xx_HAL_Driver\\Inc\\stm32l4xx_hal_rcc.h"

#line 4613 "..\\HALLIB\\STM32L4xx_HAL_Driver\\Inc\\stm32l4xx_hal_rcc.h"






























#line 4663 "..\\HALLIB\\STM32L4xx_HAL_Driver\\Inc\\stm32l4xx_hal_rcc.h"














 

 
#line 1 "..\\HALLIB\\STM32L4xx_HAL_Driver\\Inc\\stm32l4xx_hal_rcc_ex.h"

































 

 







 
#line 46 "..\\HALLIB\\STM32L4xx_HAL_Driver\\Inc\\stm32l4xx_hal_rcc_ex.h"



 



 

 



 




 
typedef struct
{

  uint32_t PLLSAI1Source;    
 





  uint32_t PLLSAI1M;         
 


  uint32_t PLLSAI1N;         
 

  uint32_t PLLSAI1P;         
 

  uint32_t PLLSAI1Q;         
 

  uint32_t PLLSAI1R;         
 

  uint32_t PLLSAI1ClockOut;  
 
}RCC_PLLSAI1InitTypeDef;





 
typedef struct
{

  uint32_t PLLSAI2Source;    
 





  uint32_t PLLSAI2M;         
 


  uint32_t PLLSAI2N;         
 

  uint32_t PLLSAI2P;         
 






  uint32_t PLLSAI2R;         
 

  uint32_t PLLSAI2ClockOut;  
 
}RCC_PLLSAI2InitTypeDef;





 
typedef struct
{
  uint32_t PeriphClockSelection;   
 


  RCC_PLLSAI1InitTypeDef PLLSAI1;  
 



  RCC_PLLSAI2InitTypeDef PLLSAI2;  
 



  uint32_t Usart1ClockSelection;   
 

  uint32_t Usart2ClockSelection;   
 



  uint32_t Usart3ClockSelection;   
 





  uint32_t Uart4ClockSelection;    
 





  uint32_t Uart5ClockSelection;    
 



  uint32_t Lpuart1ClockSelection;  
 

  uint32_t I2c1ClockSelection;     
 



  uint32_t I2c2ClockSelection;     
 



  uint32_t I2c3ClockSelection;     
 

#line 202 "..\\HALLIB\\STM32L4xx_HAL_Driver\\Inc\\stm32l4xx_hal_rcc_ex.h"

  uint32_t Lptim1ClockSelection;   
 

  uint32_t Lptim2ClockSelection;   
 


  uint32_t Sai1ClockSelection;     
 




  uint32_t Sai2ClockSelection;     
 





  uint32_t UsbClockSelection;      
 





  uint32_t Sdmmc1ClockSelection;   
 



  uint32_t RngClockSelection;      
 


  uint32_t AdcClockSelection;      
 




  uint32_t Swpmi1ClockSelection;   
 





  uint32_t Dfsdm1ClockSelection;   
 









#line 269 "..\\HALLIB\\STM32L4xx_HAL_Driver\\Inc\\stm32l4xx_hal_rcc_ex.h"

#line 276 "..\\HALLIB\\STM32L4xx_HAL_Driver\\Inc\\stm32l4xx_hal_rcc_ex.h"

#line 283 "..\\HALLIB\\STM32L4xx_HAL_Driver\\Inc\\stm32l4xx_hal_rcc_ex.h"

  uint32_t RTCClockSelection;      
 
}RCC_PeriphCLKInitTypeDef;

#line 340 "..\\HALLIB\\STM32L4xx_HAL_Driver\\Inc\\stm32l4xx_hal_rcc_ex.h"


 

 


 



 




 



 
#line 416 "..\\HALLIB\\STM32L4xx_HAL_Driver\\Inc\\stm32l4xx_hal_rcc_ex.h"


 




 






 



 






 




 






 





 






 





 






 




 






 



 





 




 





 




 





 

#line 536 "..\\HALLIB\\STM32L4xx_HAL_Driver\\Inc\\stm32l4xx_hal_rcc_ex.h"




 
#line 557 "..\\HALLIB\\STM32L4xx_HAL_Driver\\Inc\\stm32l4xx_hal_rcc_ex.h"


 





 
#line 577 "..\\HALLIB\\STM32L4xx_HAL_Driver\\Inc\\stm32l4xx_hal_rcc_ex.h"


 




 






 



 






 




 
#line 619 "..\\HALLIB\\STM32L4xx_HAL_Driver\\Inc\\stm32l4xx_hal_rcc_ex.h"


 




 
#line 637 "..\\HALLIB\\STM32L4xx_HAL_Driver\\Inc\\stm32l4xx_hal_rcc_ex.h"


 




 
#line 655 "..\\HALLIB\\STM32L4xx_HAL_Driver\\Inc\\stm32l4xx_hal_rcc_ex.h"


 




 
#line 675 "..\\HALLIB\\STM32L4xx_HAL_Driver\\Inc\\stm32l4xx_hal_rcc_ex.h"


 




 




 





 
#line 700 "..\\HALLIB\\STM32L4xx_HAL_Driver\\Inc\\stm32l4xx_hal_rcc_ex.h"


 

#line 716 "..\\HALLIB\\STM32L4xx_HAL_Driver\\Inc\\stm32l4xx_hal_rcc_ex.h"

#line 729 "..\\HALLIB\\STM32L4xx_HAL_Driver\\Inc\\stm32l4xx_hal_rcc_ex.h"

#line 740 "..\\HALLIB\\STM32L4xx_HAL_Driver\\Inc\\stm32l4xx_hal_rcc_ex.h"

#line 752 "..\\HALLIB\\STM32L4xx_HAL_Driver\\Inc\\stm32l4xx_hal_rcc_ex.h"



 



 

#line 884 "..\\HALLIB\\STM32L4xx_HAL_Driver\\Inc\\stm32l4xx_hal_rcc_ex.h"



 

 


 



































 
#line 952 "..\\HALLIB\\STM32L4xx_HAL_Driver\\Inc\\stm32l4xx_hal_rcc_ex.h"

#line 962 "..\\HALLIB\\STM32L4xx_HAL_Driver\\Inc\\stm32l4xx_hal_rcc_ex.h"

























 



#line 1009 "..\\HALLIB\\STM32L4xx_HAL_Driver\\Inc\\stm32l4xx_hal_rcc_ex.h"













 
#line 1029 "..\\HALLIB\\STM32L4xx_HAL_Driver\\Inc\\stm32l4xx_hal_rcc_ex.h"

















 














 







 

















 















 






































 

#line 1174 "..\\HALLIB\\STM32L4xx_HAL_Driver\\Inc\\stm32l4xx_hal_rcc_ex.h"

#line 1191 "..\\HALLIB\\STM32L4xx_HAL_Driver\\Inc\\stm32l4xx_hal_rcc_ex.h"

























 



#line 1238 "..\\HALLIB\\STM32L4xx_HAL_Driver\\Inc\\stm32l4xx_hal_rcc_ex.h"












 



#line 1272 "..\\HALLIB\\STM32L4xx_HAL_Driver\\Inc\\stm32l4xx_hal_rcc_ex.h"












 







 



























 

























 


























 
#line 1381 "..\\HALLIB\\STM32L4xx_HAL_Driver\\Inc\\stm32l4xx_hal_rcc_ex.h"













 
























 
#line 1427 "..\\HALLIB\\STM32L4xx_HAL_Driver\\Inc\\stm32l4xx_hal_rcc_ex.h"







 
















 








 












 








 












 








 


#line 1530 "..\\HALLIB\\STM32L4xx_HAL_Driver\\Inc\\stm32l4xx_hal_rcc_ex.h"











 









 











 









 













 









 















 









 















 









 













 









 











 









 











 









 


































 
#line 1771 "..\\HALLIB\\STM32L4xx_HAL_Driver\\Inc\\stm32l4xx_hal_rcc_ex.h"




















 
#line 1799 "..\\HALLIB\\STM32L4xx_HAL_Driver\\Inc\\stm32l4xx_hal_rcc_ex.h"



















 














 




















 














 
















 











 
#line 1910 "..\\HALLIB\\STM32L4xx_HAL_Driver\\Inc\\stm32l4xx_hal_rcc_ex.h"









 







 











 
#line 1947 "..\\HALLIB\\STM32L4xx_HAL_Driver\\Inc\\stm32l4xx_hal_rcc_ex.h"





 






#line 1981 "..\\HALLIB\\STM32L4xx_HAL_Driver\\Inc\\stm32l4xx_hal_rcc_ex.h"



#line 2008 "..\\HALLIB\\STM32L4xx_HAL_Driver\\Inc\\stm32l4xx_hal_rcc_ex.h"

#line 2029 "..\\HALLIB\\STM32L4xx_HAL_Driver\\Inc\\stm32l4xx_hal_rcc_ex.h"

#line 2052 "..\\HALLIB\\STM32L4xx_HAL_Driver\\Inc\\stm32l4xx_hal_rcc_ex.h"




 




 




 




 




 




 








 




 




 




 




 








 





 





 





 






 






 






 





 





 









 









 





 





 



#line 2313 "..\\HALLIB\\STM32L4xx_HAL_Driver\\Inc\\stm32l4xx_hal_rcc_ex.h"



 

#line 2366 "..\\HALLIB\\STM32L4xx_HAL_Driver\\Inc\\stm32l4xx_hal_rcc_ex.h"



 

 


 



 

HAL_StatusTypeDef HAL_RCCEx_PeriphCLKConfig(RCC_PeriphCLKInitTypeDef  *PeriphClkInit);
void              HAL_RCCEx_GetPeriphCLKConfig(RCC_PeriphCLKInitTypeDef  *PeriphClkInit);
uint32_t          HAL_RCCEx_GetPeriphCLKFreq(uint32_t PeriphClk);



 



 


HAL_StatusTypeDef HAL_RCCEx_EnablePLLSAI1(RCC_PLLSAI1InitTypeDef  *PLLSAI1Init);
HAL_StatusTypeDef HAL_RCCEx_DisablePLLSAI1(void);





HAL_StatusTypeDef HAL_RCCEx_EnablePLLSAI2(RCC_PLLSAI2InitTypeDef  *PLLSAI2Init);
HAL_StatusTypeDef HAL_RCCEx_DisablePLLSAI2(void);



void              HAL_RCCEx_WakeUpStopCLKConfig(uint32_t WakeUpClk);
void              HAL_RCCEx_StandbyMSIRangeConfig(uint32_t MSIRange);
void              HAL_RCCEx_EnableLSECSS(void);
void              HAL_RCCEx_DisableLSECSS(void);
void              HAL_RCCEx_EnableLSECSS_IT(void);
void              HAL_RCCEx_LSECSS_IRQHandler(void);
void              HAL_RCCEx_LSECSS_Callback(void);
void              HAL_RCCEx_EnableLSCO(uint32_t LSCOSource);
void              HAL_RCCEx_DisableLSCO(void);
void              HAL_RCCEx_EnableMSIPLLMode(void);
void              HAL_RCCEx_DisableMSIPLLMode(void);



 

#line 2442 "..\\HALLIB\\STM32L4xx_HAL_Driver\\Inc\\stm32l4xx_hal_rcc_ex.h"



 

 


 




#line 2701 "..\\HALLIB\\STM32L4xx_HAL_Driver\\Inc\\stm32l4xx_hal_rcc_ex.h"

#line 2723 "..\\HALLIB\\STM32L4xx_HAL_Driver\\Inc\\stm32l4xx_hal_rcc_ex.h"






































































#line 2801 "..\\HALLIB\\STM32L4xx_HAL_Driver\\Inc\\stm32l4xx_hal_rcc_ex.h"



#line 2818 "..\\HALLIB\\STM32L4xx_HAL_Driver\\Inc\\stm32l4xx_hal_rcc_ex.h"

#line 2827 "..\\HALLIB\\STM32L4xx_HAL_Driver\\Inc\\stm32l4xx_hal_rcc_ex.h"



#line 2844 "..\\HALLIB\\STM32L4xx_HAL_Driver\\Inc\\stm32l4xx_hal_rcc_ex.h"















#line 2877 "..\\HALLIB\\STM32L4xx_HAL_Driver\\Inc\\stm32l4xx_hal_rcc_ex.h"










#line 2903 "..\\HALLIB\\STM32L4xx_HAL_Driver\\Inc\\stm32l4xx_hal_rcc_ex.h"









#line 2929 "..\\HALLIB\\STM32L4xx_HAL_Driver\\Inc\\stm32l4xx_hal_rcc_ex.h"


















#line 2961 "..\\HALLIB\\STM32L4xx_HAL_Driver\\Inc\\stm32l4xx_hal_rcc_ex.h"















#line 2984 "..\\HALLIB\\STM32L4xx_HAL_Driver\\Inc\\stm32l4xx_hal_rcc_ex.h"



#line 2996 "..\\HALLIB\\STM32L4xx_HAL_Driver\\Inc\\stm32l4xx_hal_rcc_ex.h"

#line 3004 "..\\HALLIB\\STM32L4xx_HAL_Driver\\Inc\\stm32l4xx_hal_rcc_ex.h"

#line 3013 "..\\HALLIB\\STM32L4xx_HAL_Driver\\Inc\\stm32l4xx_hal_rcc_ex.h"























































#line 3092 "..\\HALLIB\\STM32L4xx_HAL_Driver\\Inc\\stm32l4xx_hal_rcc_ex.h"



 



 



 







 
#line 4681 "..\\HALLIB\\STM32L4xx_HAL_Driver\\Inc\\stm32l4xx_hal_rcc.h"

 


 




 

 
HAL_StatusTypeDef HAL_RCC_DeInit(void);
HAL_StatusTypeDef HAL_RCC_OscConfig(RCC_OscInitTypeDef *RCC_OscInitStruct);
HAL_StatusTypeDef HAL_RCC_ClockConfig(RCC_ClkInitTypeDef *RCC_ClkInitStruct, uint32_t FLatency);



 



 

 
void              HAL_RCC_MCOConfig(uint32_t RCC_MCOx, uint32_t RCC_MCOSource, uint32_t RCC_MCODiv);
void              HAL_RCC_EnableCSS(void);
uint32_t          HAL_RCC_GetSysClockFreq(void);
uint32_t          HAL_RCC_GetHCLKFreq(void);
uint32_t          HAL_RCC_GetPCLK1Freq(void);
uint32_t          HAL_RCC_GetPCLK2Freq(void);
void              HAL_RCC_GetOscConfig(RCC_OscInitTypeDef *RCC_OscInitStruct);
void              HAL_RCC_GetClockConfig(RCC_ClkInitTypeDef *RCC_ClkInitStruct, uint32_t *pFLatency);
 
void              HAL_RCC_NMI_IRQHandler(void);
 
void              HAL_RCC_CSSCallback(void);



 



 



 



 







 
#line 224 "..\\USER\\stm32l4xx_hal_conf.h"
#line 225 "..\\USER\\stm32l4xx_hal_conf.h"






#line 1 "..\\HALLIB\\STM32L4xx_HAL_Driver\\Inc\\stm32l4xx_hal_gpio.h"

































 

 







 
#line 46 "..\\HALLIB\\STM32L4xx_HAL_Driver\\Inc\\stm32l4xx_hal_gpio.h"



 



 

 



 


 
typedef struct
{
  uint32_t Pin;        
 

  uint32_t Mode;       
 

  uint32_t Pull;       
 

  uint32_t Speed;      
 

  uint32_t Alternate;  
 
}GPIO_InitTypeDef;



 
typedef enum
{
  GPIO_PIN_RESET = 0U,
  GPIO_PIN_SET
}GPIO_PinState;


 

 


 


 
#line 117 "..\\HALLIB\\STM32L4xx_HAL_Driver\\Inc\\stm32l4xx_hal_gpio.h"




 










 
#line 146 "..\\HALLIB\\STM32L4xx_HAL_Driver\\Inc\\stm32l4xx_hal_gpio.h"


 




 






 

 


 





 



 

 


 






 







 







 







 







 




 

 


 





#line 248 "..\\HALLIB\\STM32L4xx_HAL_Driver\\Inc\\stm32l4xx_hal_gpio.h"











 

 
#line 1 "..\\HALLIB\\STM32L4xx_HAL_Driver\\Inc\\stm32l4xx_hal_gpio_ex.h"

































 

 







 
#line 46 "..\\HALLIB\\STM32L4xx_HAL_Driver\\Inc\\stm32l4xx_hal_gpio_ex.h"



 




 

 
 


 



 

#line 159 "..\\HALLIB\\STM32L4xx_HAL_Driver\\Inc\\stm32l4xx_hal_gpio_ex.h"

#line 275 "..\\HALLIB\\STM32L4xx_HAL_Driver\\Inc\\stm32l4xx_hal_gpio_ex.h"

#line 396 "..\\HALLIB\\STM32L4xx_HAL_Driver\\Inc\\stm32l4xx_hal_gpio_ex.h"


 


 
#line 409 "..\\HALLIB\\STM32L4xx_HAL_Driver\\Inc\\stm32l4xx_hal_gpio_ex.h"



 
#line 419 "..\\HALLIB\\STM32L4xx_HAL_Driver\\Inc\\stm32l4xx_hal_gpio_ex.h"



 








 






 






 





 





 






 







 





 





#line 490 "..\\HALLIB\\STM32L4xx_HAL_Driver\\Inc\\stm32l4xx_hal_gpio_ex.h"



 








 







 
#line 517 "..\\HALLIB\\STM32L4xx_HAL_Driver\\Inc\\stm32l4xx_hal_gpio_ex.h"



 






#line 666 "..\\HALLIB\\STM32L4xx_HAL_Driver\\Inc\\stm32l4xx_hal_gpio_ex.h"

#line 810 "..\\HALLIB\\STM32L4xx_HAL_Driver\\Inc\\stm32l4xx_hal_gpio_ex.h"



 



 

 


 



 
#line 835 "..\\HALLIB\\STM32L4xx_HAL_Driver\\Inc\\stm32l4xx_hal_gpio_ex.h"

#line 845 "..\\HALLIB\\STM32L4xx_HAL_Driver\\Inc\\stm32l4xx_hal_gpio_ex.h"

#line 853 "..\\HALLIB\\STM32L4xx_HAL_Driver\\Inc\\stm32l4xx_hal_gpio_ex.h"

#line 863 "..\\HALLIB\\STM32L4xx_HAL_Driver\\Inc\\stm32l4xx_hal_gpio_ex.h"



#line 873 "..\\HALLIB\\STM32L4xx_HAL_Driver\\Inc\\stm32l4xx_hal_gpio_ex.h"



#line 888 "..\\HALLIB\\STM32L4xx_HAL_Driver\\Inc\\stm32l4xx_hal_gpio_ex.h"

#line 901 "..\\HALLIB\\STM32L4xx_HAL_Driver\\Inc\\stm32l4xx_hal_gpio_ex.h"



 



 

 


 



 







 
#line 263 "..\\HALLIB\\STM32L4xx_HAL_Driver\\Inc\\stm32l4xx_hal_gpio.h"

 


 




 

 
void              HAL_GPIO_Init(GPIO_TypeDef  *GPIOx, GPIO_InitTypeDef *GPIO_Init);
void              HAL_GPIO_DeInit(GPIO_TypeDef  *GPIOx, uint32_t GPIO_Pin);



 



 

 
GPIO_PinState     HAL_GPIO_ReadPin(GPIO_TypeDef* GPIOx, uint16_t GPIO_Pin);
void              HAL_GPIO_WritePin(GPIO_TypeDef* GPIOx, uint16_t GPIO_Pin, GPIO_PinState PinState);
void              HAL_GPIO_TogglePin(GPIO_TypeDef* GPIOx, uint16_t GPIO_Pin);
HAL_StatusTypeDef HAL_GPIO_LockPin(GPIO_TypeDef* GPIOx, uint16_t GPIO_Pin);
void              HAL_GPIO_EXTI_IRQHandler(uint16_t GPIO_Pin);
void              HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin);



 



 



 



 







 
#line 233 "..\\USER\\stm32l4xx_hal_conf.h"


#line 1 "..\\HALLIB\\STM32L4xx_HAL_Driver\\Inc\\stm32l4xx_hal_dma.h"

































 

 







 
#line 46 "..\\HALLIB\\STM32L4xx_HAL_Driver\\Inc\\stm32l4xx_hal_dma.h"



 



 

 


 



 
typedef struct
{
  uint32_t Request;                   
 

  uint32_t Direction;                 

 

  uint32_t PeriphInc;                 
 

  uint32_t MemInc;                    
 

  uint32_t PeriphDataAlignment;       
 

  uint32_t MemDataAlignment;          
 

  uint32_t Mode;                      


 

  uint32_t Priority;                  
 
} DMA_InitTypeDef;



 
typedef enum
{
  HAL_DMA_STATE_RESET             = 0x00U,   
  HAL_DMA_STATE_READY             = 0x01U,   
  HAL_DMA_STATE_BUSY              = 0x02U,   
  HAL_DMA_STATE_TIMEOUT           = 0x03U,   
}HAL_DMA_StateTypeDef;



 
typedef enum
{
  HAL_DMA_FULL_TRANSFER      = 0x00U,     
  HAL_DMA_HALF_TRANSFER      = 0x01U      
}HAL_DMA_LevelCompleteTypeDef;




 
typedef enum
{
  HAL_DMA_XFER_CPLT_CB_ID          = 0x00U,     
  HAL_DMA_XFER_HALFCPLT_CB_ID      = 0x01U,     
  HAL_DMA_XFER_ERROR_CB_ID         = 0x02U,     
  HAL_DMA_XFER_ABORT_CB_ID         = 0x03U,     
  HAL_DMA_XFER_ALL_CB_ID           = 0x04U      
}HAL_DMA_CallbackIDTypeDef;



 
typedef struct __DMA_HandleTypeDef
{
  DMA_Channel_TypeDef    *Instance;                                                   

  DMA_InitTypeDef       Init;                                                         

  HAL_LockTypeDef       Lock;                                                         

  volatile HAL_DMA_StateTypeDef  State;                                                   

  void                  *Parent;                                                      

  void                  (* XferCpltCallback)(struct __DMA_HandleTypeDef * hdma);      

  void                  (* XferHalfCpltCallback)(struct __DMA_HandleTypeDef * hdma);  

  void                  (* XferErrorCallback)(struct __DMA_HandleTypeDef * hdma);     

  void                  (* XferAbortCallback)( struct __DMA_HandleTypeDef * hdma);    

  volatile uint32_t          ErrorCode;                                                   

  DMA_TypeDef            *DmaBaseAddress;                                             

  uint32_t               ChannelIndex;                                                

#line 169 "..\\HALLIB\\STM32L4xx_HAL_Driver\\Inc\\stm32l4xx_hal_dma.h"

}DMA_HandleTypeDef;



 

 



 



 
#line 192 "..\\HALLIB\\STM32L4xx_HAL_Driver\\Inc\\stm32l4xx_hal_dma.h"



 



 


#line 210 "..\\HALLIB\\STM32L4xx_HAL_Driver\\Inc\\stm32l4xx_hal_dma.h"



#line 334 "..\\HALLIB\\STM32L4xx_HAL_Driver\\Inc\\stm32l4xx_hal_dma.h"



 



 





 



 




 



 




 



 





 



 





 



 




 



 






 




 





 



 
#line 449 "..\\HALLIB\\STM32L4xx_HAL_Driver\\Inc\\stm32l4xx_hal_dma.h"


 



 

 


 




 






 






 



 





 

#line 505 "..\\HALLIB\\STM32L4xx_HAL_Driver\\Inc\\stm32l4xx_hal_dma.h"





 
#line 525 "..\\HALLIB\\STM32L4xx_HAL_Driver\\Inc\\stm32l4xx_hal_dma.h"





 
#line 545 "..\\HALLIB\\STM32L4xx_HAL_Driver\\Inc\\stm32l4xx_hal_dma.h"





 
#line 565 "..\\HALLIB\\STM32L4xx_HAL_Driver\\Inc\\stm32l4xx_hal_dma.h"












 














 












 











 











 






 




 






 



 



 
 
HAL_StatusTypeDef HAL_DMA_Init(DMA_HandleTypeDef *hdma);
HAL_StatusTypeDef HAL_DMA_DeInit (DMA_HandleTypeDef *hdma);


 



 
 
HAL_StatusTypeDef HAL_DMA_Start (DMA_HandleTypeDef *hdma, uint32_t SrcAddress, uint32_t DstAddress, uint32_t DataLength);
HAL_StatusTypeDef HAL_DMA_Start_IT(DMA_HandleTypeDef *hdma, uint32_t SrcAddress, uint32_t DstAddress, uint32_t DataLength);
HAL_StatusTypeDef HAL_DMA_Abort(DMA_HandleTypeDef *hdma);
HAL_StatusTypeDef HAL_DMA_Abort_IT(DMA_HandleTypeDef *hdma);
HAL_StatusTypeDef HAL_DMA_PollForTransfer(DMA_HandleTypeDef *hdma, HAL_DMA_LevelCompleteTypeDef CompleteLevel, uint32_t Timeout);
void HAL_DMA_IRQHandler(DMA_HandleTypeDef *hdma);
HAL_StatusTypeDef HAL_DMA_RegisterCallback(DMA_HandleTypeDef *hdma, HAL_DMA_CallbackIDTypeDef CallbackID, void (* pCallback)( DMA_HandleTypeDef * _hdma));
HAL_StatusTypeDef HAL_DMA_UnRegisterCallback(DMA_HandleTypeDef *hdma, HAL_DMA_CallbackIDTypeDef CallbackID);



 



 
 
HAL_DMA_StateTypeDef HAL_DMA_GetState(DMA_HandleTypeDef *hdma);
uint32_t             HAL_DMA_GetError(DMA_HandleTypeDef *hdma);


 



 

 


 















#line 723 "..\\HALLIB\\STM32L4xx_HAL_Driver\\Inc\\stm32l4xx_hal_dma.h"

























 

 



 



 







 
#line 237 "..\\USER\\stm32l4xx_hal_conf.h"
#line 1 "..\\HALLIB\\STM32L4xx_HAL_Driver\\Inc\\stm32l4xx_hal_dma_ex.h"

































 

 







#line 291 "..\\HALLIB\\STM32L4xx_HAL_Driver\\Inc\\stm32l4xx_hal_dma_ex.h"







 
#line 238 "..\\USER\\stm32l4xx_hal_conf.h"


#line 1 "..\\HALLIB\\STM32L4xx_HAL_Driver\\Inc\\stm32l4xx_hal_dfsdm.h"

































 

 












 
#line 51 "..\\HALLIB\\STM32L4xx_HAL_Driver\\Inc\\stm32l4xx_hal_dfsdm.h"



 



 

 


 



 
typedef enum
{
  HAL_DFSDM_CHANNEL_STATE_RESET = 0x00U,  
  HAL_DFSDM_CHANNEL_STATE_READY = 0x01U,  
  HAL_DFSDM_CHANNEL_STATE_ERROR = 0xFFU   
} HAL_DFSDM_Channel_StateTypeDef;



 
typedef struct
{
  FunctionalState Activation;  
  uint32_t        Selection;  
 
  uint32_t        Divider;    
 
} DFSDM_Channel_OutputClockTypeDef;



 
typedef struct
{
  uint32_t Multiplexer; 



 
  uint32_t DataPacking; 
 
  uint32_t Pins;        
 
} DFSDM_Channel_InputTypeDef;



 
typedef struct
{
  uint32_t Type;     
 
  uint32_t SpiClock; 
 
} DFSDM_Channel_SerialInterfaceTypeDef;



 
typedef struct
{
  uint32_t FilterOrder;  
 
  uint32_t Oversampling; 
 
} DFSDM_Channel_AwdTypeDef;



 
typedef struct
{
  DFSDM_Channel_OutputClockTypeDef     OutputClock;      
  DFSDM_Channel_InputTypeDef           Input;            
  DFSDM_Channel_SerialInterfaceTypeDef SerialInterface;  
  DFSDM_Channel_AwdTypeDef             Awd;              
  int32_t                              Offset;          
 
  uint32_t                             RightBitShift;   
 
} DFSDM_Channel_InitTypeDef;



 
typedef struct __DFSDM_Channel_HandleTypeDef
{
  DFSDM_Channel_TypeDef          *Instance;  
  DFSDM_Channel_InitTypeDef      Init;       
  HAL_DFSDM_Channel_StateTypeDef State;      
#line 154 "..\\HALLIB\\STM32L4xx_HAL_Driver\\Inc\\stm32l4xx_hal_dfsdm.h"
} DFSDM_Channel_HandleTypeDef;

#line 173 "..\\HALLIB\\STM32L4xx_HAL_Driver\\Inc\\stm32l4xx_hal_dfsdm.h"



 
typedef enum
{
  HAL_DFSDM_FILTER_STATE_RESET   = 0x00U,  
  HAL_DFSDM_FILTER_STATE_READY   = 0x01U,  
  HAL_DFSDM_FILTER_STATE_REG     = 0x02U,  
  HAL_DFSDM_FILTER_STATE_INJ     = 0x03U,  
  HAL_DFSDM_FILTER_STATE_REG_INJ = 0x04U,  
  HAL_DFSDM_FILTER_STATE_ERROR   = 0xFFU   
} HAL_DFSDM_Filter_StateTypeDef;



 
typedef struct
{
  uint32_t        Trigger;  
 
  FunctionalState FastMode;  
  FunctionalState DmaMode;   
} DFSDM_Filter_RegularParamTypeDef;



 
typedef struct
{
  uint32_t        Trigger;        
 
  FunctionalState ScanMode;        
  FunctionalState DmaMode;         
  uint32_t        ExtTrigger;     
 
  uint32_t        ExtTriggerEdge; 
 
} DFSDM_Filter_InjectedParamTypeDef;



 
typedef struct
{
  uint32_t SincOrder;       
 
  uint32_t Oversampling;    
 
  uint32_t IntOversampling; 
 
} DFSDM_Filter_FilterParamTypeDef;



 
typedef struct
{
  DFSDM_Filter_RegularParamTypeDef  RegularParam;   
  DFSDM_Filter_InjectedParamTypeDef InjectedParam;  
  DFSDM_Filter_FilterParamTypeDef   FilterParam;    
} DFSDM_Filter_InitTypeDef;



 
typedef struct __DFSDM_Filter_HandleTypeDef
{
  DFSDM_Filter_TypeDef          *Instance;            
  DFSDM_Filter_InitTypeDef      Init;                 
  DMA_HandleTypeDef             *hdmaReg;             
  DMA_HandleTypeDef             *hdmaInj;             
  uint32_t                      RegularContMode;      
  uint32_t                      RegularTrigger;       
  uint32_t                      InjectedTrigger;      
  uint32_t                      ExtTriggerEdge;       
  FunctionalState               InjectedScanMode;     
  uint32_t                      InjectedChannelsNbr;  
  uint32_t                      InjConvRemaining;     
  HAL_DFSDM_Filter_StateTypeDef State;                
  uint32_t                      ErrorCode;            
#line 265 "..\\HALLIB\\STM32L4xx_HAL_Driver\\Inc\\stm32l4xx_hal_dfsdm.h"
} DFSDM_Filter_HandleTypeDef;



 
typedef struct
{
  uint32_t DataSource;      
 
  uint32_t Channel;         
 
  int32_t  HighThreshold;   
 
  int32_t  LowThreshold;    
 
  uint32_t HighBreakSignal; 
 
  uint32_t LowBreakSignal;  
 
} DFSDM_Filter_AwdParamTypeDef;

#line 307 "..\\HALLIB\\STM32L4xx_HAL_Driver\\Inc\\stm32l4xx_hal_dfsdm.h"



 
 

 


 



 




 



 
#line 337 "..\\HALLIB\\STM32L4xx_HAL_Driver\\Inc\\stm32l4xx_hal_dfsdm.h"


 



 





 



 




 



 






 



 






 



 






 



 





 



 
#line 443 "..\\HALLIB\\STM32L4xx_HAL_Driver\\Inc\\stm32l4xx_hal_dfsdm.h"


 



 





 



 
#line 466 "..\\HALLIB\\STM32L4xx_HAL_Driver\\Inc\\stm32l4xx_hal_dfsdm.h"


 



 




 



 
#line 489 "..\\HALLIB\\STM32L4xx_HAL_Driver\\Inc\\stm32l4xx_hal_dfsdm.h"


 



 







 



 
 






 
#line 531 "..\\HALLIB\\STM32L4xx_HAL_Driver\\Inc\\stm32l4xx_hal_dfsdm.h"


 



 




 



 




 



 
 

 


 




 
#line 576 "..\\HALLIB\\STM32L4xx_HAL_Driver\\Inc\\stm32l4xx_hal_dfsdm.h"




 
#line 590 "..\\HALLIB\\STM32L4xx_HAL_Driver\\Inc\\stm32l4xx_hal_dfsdm.h"



 
 






 


 



 
 
HAL_StatusTypeDef HAL_DFSDM_ChannelInit(DFSDM_Channel_HandleTypeDef *hdfsdm_channel);
HAL_StatusTypeDef HAL_DFSDM_ChannelDeInit(DFSDM_Channel_HandleTypeDef *hdfsdm_channel);
void HAL_DFSDM_ChannelMspInit(DFSDM_Channel_HandleTypeDef *hdfsdm_channel);
void HAL_DFSDM_ChannelMspDeInit(DFSDM_Channel_HandleTypeDef *hdfsdm_channel);

#line 623 "..\\HALLIB\\STM32L4xx_HAL_Driver\\Inc\\stm32l4xx_hal_dfsdm.h"


 



 
 
HAL_StatusTypeDef HAL_DFSDM_ChannelCkabStart(DFSDM_Channel_HandleTypeDef *hdfsdm_channel);
HAL_StatusTypeDef HAL_DFSDM_ChannelCkabStart_IT(DFSDM_Channel_HandleTypeDef *hdfsdm_channel);
HAL_StatusTypeDef HAL_DFSDM_ChannelCkabStop(DFSDM_Channel_HandleTypeDef *hdfsdm_channel);
HAL_StatusTypeDef HAL_DFSDM_ChannelCkabStop_IT(DFSDM_Channel_HandleTypeDef *hdfsdm_channel);

HAL_StatusTypeDef HAL_DFSDM_ChannelScdStart(DFSDM_Channel_HandleTypeDef *hdfsdm_channel, uint32_t Threshold, uint32_t BreakSignal);
HAL_StatusTypeDef HAL_DFSDM_ChannelScdStart_IT(DFSDM_Channel_HandleTypeDef *hdfsdm_channel, uint32_t Threshold, uint32_t BreakSignal);
HAL_StatusTypeDef HAL_DFSDM_ChannelScdStop(DFSDM_Channel_HandleTypeDef *hdfsdm_channel);
HAL_StatusTypeDef HAL_DFSDM_ChannelScdStop_IT(DFSDM_Channel_HandleTypeDef *hdfsdm_channel);

int16_t           HAL_DFSDM_ChannelGetAwdValue(DFSDM_Channel_HandleTypeDef *hdfsdm_channel);
HAL_StatusTypeDef HAL_DFSDM_ChannelModifyOffset(DFSDM_Channel_HandleTypeDef *hdfsdm_channel, int32_t Offset);

HAL_StatusTypeDef HAL_DFSDM_ChannelPollForCkab(DFSDM_Channel_HandleTypeDef *hdfsdm_channel, uint32_t Timeout);
HAL_StatusTypeDef HAL_DFSDM_ChannelPollForScd(DFSDM_Channel_HandleTypeDef *hdfsdm_channel, uint32_t Timeout);

void HAL_DFSDM_ChannelCkabCallback(DFSDM_Channel_HandleTypeDef *hdfsdm_channel);
void HAL_DFSDM_ChannelScdCallback(DFSDM_Channel_HandleTypeDef *hdfsdm_channel);


 



 
 
HAL_DFSDM_Channel_StateTypeDef HAL_DFSDM_ChannelGetState(DFSDM_Channel_HandleTypeDef *hdfsdm_channel);


 



 
 
HAL_StatusTypeDef HAL_DFSDM_FilterInit(DFSDM_Filter_HandleTypeDef *hdfsdm_filter);
HAL_StatusTypeDef HAL_DFSDM_FilterDeInit(DFSDM_Filter_HandleTypeDef *hdfsdm_filter);
void HAL_DFSDM_FilterMspInit(DFSDM_Filter_HandleTypeDef *hdfsdm_filter);
void HAL_DFSDM_FilterMspDeInit(DFSDM_Filter_HandleTypeDef *hdfsdm_filter);

#line 682 "..\\HALLIB\\STM32L4xx_HAL_Driver\\Inc\\stm32l4xx_hal_dfsdm.h"


 



 
 
HAL_StatusTypeDef HAL_DFSDM_FilterConfigRegChannel(DFSDM_Filter_HandleTypeDef *hdfsdm_filter,
                                                   uint32_t                    Channel,
                                                   uint32_t                    ContinuousMode);
HAL_StatusTypeDef HAL_DFSDM_FilterConfigInjChannel(DFSDM_Filter_HandleTypeDef *hdfsdm_filter,
                                                   uint32_t                    Channel);


 



 
 
HAL_StatusTypeDef HAL_DFSDM_FilterRegularStart(DFSDM_Filter_HandleTypeDef *hdfsdm_filter);
HAL_StatusTypeDef HAL_DFSDM_FilterRegularStart_IT(DFSDM_Filter_HandleTypeDef *hdfsdm_filter);
HAL_StatusTypeDef HAL_DFSDM_FilterRegularStart_DMA(DFSDM_Filter_HandleTypeDef *hdfsdm_filter, int32_t *pData, uint32_t Length);
HAL_StatusTypeDef HAL_DFSDM_FilterRegularMsbStart_DMA(DFSDM_Filter_HandleTypeDef *hdfsdm_filter, int16_t *pData, uint32_t Length);
HAL_StatusTypeDef HAL_DFSDM_FilterRegularStop(DFSDM_Filter_HandleTypeDef *hdfsdm_filter);
HAL_StatusTypeDef HAL_DFSDM_FilterRegularStop_IT(DFSDM_Filter_HandleTypeDef *hdfsdm_filter);
HAL_StatusTypeDef HAL_DFSDM_FilterRegularStop_DMA(DFSDM_Filter_HandleTypeDef *hdfsdm_filter);
HAL_StatusTypeDef HAL_DFSDM_FilterInjectedStart(DFSDM_Filter_HandleTypeDef *hdfsdm_filter);
HAL_StatusTypeDef HAL_DFSDM_FilterInjectedStart_IT(DFSDM_Filter_HandleTypeDef *hdfsdm_filter);
HAL_StatusTypeDef HAL_DFSDM_FilterInjectedStart_DMA(DFSDM_Filter_HandleTypeDef *hdfsdm_filter, int32_t *pData, uint32_t Length);
HAL_StatusTypeDef HAL_DFSDM_FilterInjectedMsbStart_DMA(DFSDM_Filter_HandleTypeDef *hdfsdm_filter, int16_t *pData, uint32_t Length);
HAL_StatusTypeDef HAL_DFSDM_FilterInjectedStop(DFSDM_Filter_HandleTypeDef *hdfsdm_filter);
HAL_StatusTypeDef HAL_DFSDM_FilterInjectedStop_IT(DFSDM_Filter_HandleTypeDef *hdfsdm_filter);
HAL_StatusTypeDef HAL_DFSDM_FilterInjectedStop_DMA(DFSDM_Filter_HandleTypeDef *hdfsdm_filter);
HAL_StatusTypeDef HAL_DFSDM_FilterAwdStart_IT(DFSDM_Filter_HandleTypeDef *hdfsdm_filter,
                                              DFSDM_Filter_AwdParamTypeDef *awdParam);
HAL_StatusTypeDef HAL_DFSDM_FilterAwdStop_IT(DFSDM_Filter_HandleTypeDef *hdfsdm_filter);
HAL_StatusTypeDef HAL_DFSDM_FilterExdStart(DFSDM_Filter_HandleTypeDef *hdfsdm_filter, uint32_t Channel);
HAL_StatusTypeDef HAL_DFSDM_FilterExdStop(DFSDM_Filter_HandleTypeDef *hdfsdm_filter);

int32_t  HAL_DFSDM_FilterGetRegularValue(DFSDM_Filter_HandleTypeDef *hdfsdm_filter, uint32_t *Channel);
int32_t  HAL_DFSDM_FilterGetInjectedValue(DFSDM_Filter_HandleTypeDef *hdfsdm_filter, uint32_t *Channel);
int32_t  HAL_DFSDM_FilterGetExdMaxValue(DFSDM_Filter_HandleTypeDef *hdfsdm_filter, uint32_t *Channel);
int32_t  HAL_DFSDM_FilterGetExdMinValue(DFSDM_Filter_HandleTypeDef *hdfsdm_filter, uint32_t *Channel);
uint32_t HAL_DFSDM_FilterGetConvTimeValue(DFSDM_Filter_HandleTypeDef *hdfsdm_filter);

void HAL_DFSDM_IRQHandler(DFSDM_Filter_HandleTypeDef *hdfsdm_filter);

HAL_StatusTypeDef HAL_DFSDM_FilterPollForRegConversion(DFSDM_Filter_HandleTypeDef *hdfsdm_filter, uint32_t Timeout);
HAL_StatusTypeDef HAL_DFSDM_FilterPollForInjConversion(DFSDM_Filter_HandleTypeDef *hdfsdm_filter, uint32_t Timeout);

void HAL_DFSDM_FilterRegConvCpltCallback(DFSDM_Filter_HandleTypeDef *hdfsdm_filter);
void HAL_DFSDM_FilterRegConvHalfCpltCallback(DFSDM_Filter_HandleTypeDef *hdfsdm_filter);
void HAL_DFSDM_FilterInjConvCpltCallback(DFSDM_Filter_HandleTypeDef *hdfsdm_filter);
void HAL_DFSDM_FilterInjConvHalfCpltCallback(DFSDM_Filter_HandleTypeDef *hdfsdm_filter);
void HAL_DFSDM_FilterAwdCallback(DFSDM_Filter_HandleTypeDef *hdfsdm_filter, uint32_t Channel, uint32_t Threshold);
void HAL_DFSDM_FilterErrorCallback(DFSDM_Filter_HandleTypeDef *hdfsdm_filter);


 



 
 
HAL_DFSDM_Filter_StateTypeDef HAL_DFSDM_FilterGetState(DFSDM_Filter_HandleTypeDef *hdfsdm_filter);
uint32_t                      HAL_DFSDM_FilterGetError(DFSDM_Filter_HandleTypeDef *hdfsdm_filter);


 



 
 

 


 
#line 776 "..\\HALLIB\\STM32L4xx_HAL_Driver\\Inc\\stm32l4xx_hal_dfsdm.h"
 
 
#line 872 "..\\HALLIB\\STM32L4xx_HAL_Driver\\Inc\\stm32l4xx_hal_dfsdm.h"


 
 



 



 

 
 
 







 
#line 242 "..\\USER\\stm32l4xx_hal_conf.h"


#line 1 "..\\HALLIB\\STM32L4xx_HAL_Driver\\Inc\\stm32l4xx_hal_cortex.h"

































 

 







 
#line 46 "..\\HALLIB\\STM32L4xx_HAL_Driver\\Inc\\stm32l4xx_hal_cortex.h"



 



 

 


 




 
typedef struct
{
  uint8_t                Enable;                
 
  uint8_t                Number;                
 
  uint32_t               BaseAddress;            
  uint8_t                Size;                  
 
  uint8_t                SubRegionDisable;      
 
  uint8_t                TypeExtField;          
 
  uint8_t                AccessPermission;      
 
  uint8_t                DisableExec;           
 
  uint8_t                IsShareable;           
 
  uint8_t                IsCacheable;           
 
  uint8_t                IsBufferable;          
 
}MPU_Region_InitTypeDef;


 




 

 



 



 
#line 116 "..\\HALLIB\\STM32L4xx_HAL_Driver\\Inc\\stm32l4xx_hal_cortex.h"


 



 




 




 






 



 




 



 




 



 




 



 




 



 




 



 





 



 
#line 227 "..\\HALLIB\\STM32L4xx_HAL_Driver\\Inc\\stm32l4xx_hal_cortex.h"


 



 
#line 240 "..\\HALLIB\\STM32L4xx_HAL_Driver\\Inc\\stm32l4xx_hal_cortex.h"


 



 
#line 255 "..\\HALLIB\\STM32L4xx_HAL_Driver\\Inc\\stm32l4xx_hal_cortex.h"


 




 

 


 



 

 


 




 
 
void HAL_NVIC_SetPriorityGrouping(uint32_t PriorityGroup);
void HAL_NVIC_SetPriority(IRQn_Type IRQn, uint32_t PreemptPriority, uint32_t SubPriority);
void HAL_NVIC_EnableIRQ(IRQn_Type IRQn);
void HAL_NVIC_DisableIRQ(IRQn_Type IRQn);
void HAL_NVIC_SystemReset(void);
uint32_t HAL_SYSTICK_Config(uint32_t TicksNumb);



 




 
 
uint32_t HAL_NVIC_GetPriorityGrouping(void);
void HAL_NVIC_GetPriority(IRQn_Type IRQn, uint32_t PriorityGroup, uint32_t* pPreemptPriority, uint32_t* pSubPriority);
uint32_t HAL_NVIC_GetPendingIRQ(IRQn_Type IRQn);
void HAL_NVIC_SetPendingIRQ(IRQn_Type IRQn);
void HAL_NVIC_ClearPendingIRQ(IRQn_Type IRQn);
uint32_t HAL_NVIC_GetActive(IRQn_Type IRQn);
void HAL_SYSTICK_CLKSourceConfig(uint32_t CLKSource);
void HAL_SYSTICK_IRQHandler(void);
void HAL_SYSTICK_Callback(void);


void HAL_MPU_Enable(uint32_t MPU_Control);
void HAL_MPU_Disable(void);
void HAL_MPU_ConfigRegion(MPU_Region_InitTypeDef *MPU_Init);



 



 

  
 
 
 


 



































#line 370 "..\\HALLIB\\STM32L4xx_HAL_Driver\\Inc\\stm32l4xx_hal_cortex.h"

#line 379 "..\\HALLIB\\STM32L4xx_HAL_Driver\\Inc\\stm32l4xx_hal_cortex.h"

#line 408 "..\\HALLIB\\STM32L4xx_HAL_Driver\\Inc\\stm32l4xx_hal_cortex.h"






 

 



 



 








 
#line 246 "..\\USER\\stm32l4xx_hal_conf.h"


#line 1 "..\\HALLIB\\STM32L4xx_HAL_Driver\\Inc\\stm32l4xx_hal_adc.h"

































 

 







 
#line 46 "..\\HALLIB\\STM32L4xx_HAL_Driver\\Inc\\stm32l4xx_hal_adc.h"

 
#line 1 "..\\HALLIB\\STM32L4xx_HAL_Driver\\Inc\\stm32l4xx_ll_adc.h"

































 

 







 
#line 46 "..\\HALLIB\\STM32L4xx_HAL_Driver\\Inc\\stm32l4xx_ll_adc.h"



 





 

 
 

 


 

 
 
 
 

 
 









 
 
#line 99 "..\\HALLIB\\STM32L4xx_HAL_Driver\\Inc\\stm32l4xx_ll_adc.h"



 
 
 
 

 
 









 
 







 
 
 
 


 
 
 





 
 
 





 





 
 
 
 


 
 
 





 
 
 





 








 
 
 
 
 
 
 
 




 


 




 
 








 
 
#line 237 "..\\HALLIB\\STM32L4xx_HAL_Driver\\Inc\\stm32l4xx_ll_adc.h"

 
 
#line 259 "..\\HALLIB\\STM32L4xx_HAL_Driver\\Inc\\stm32l4xx_ll_adc.h"

 
 
#line 281 "..\\HALLIB\\STM32L4xx_HAL_Driver\\Inc\\stm32l4xx_ll_adc.h"


 
 
 
 
 
 
#line 296 "..\\HALLIB\\STM32L4xx_HAL_Driver\\Inc\\stm32l4xx_ll_adc.h"

 
 
 
 
 
 
 
 

 




 
 











 
#line 333 "..\\HALLIB\\STM32L4xx_HAL_Driver\\Inc\\stm32l4xx_ll_adc.h"

 
 







 







 



 
 


 
#line 369 "..\\HALLIB\\STM32L4xx_HAL_Driver\\Inc\\stm32l4xx_ll_adc.h"




 


 


 








 





 


 
#line 594 "..\\HALLIB\\STM32L4xx_HAL_Driver\\Inc\\stm32l4xx_ll_adc.h"

 


 




 
#line 639 "..\\HALLIB\\STM32L4xx_HAL_Driver\\Inc\\stm32l4xx_ll_adc.h"


 




 
#line 658 "..\\HALLIB\\STM32L4xx_HAL_Driver\\Inc\\stm32l4xx_ll_adc.h"


 



 
 
 
 






 



 
#line 694 "..\\HALLIB\\STM32L4xx_HAL_Driver\\Inc\\stm32l4xx_ll_adc.h"


 



 
 
 
 
 
 






 



 






 



 




 



 




 



 






 



 




 



 





 



 
#line 809 "..\\HALLIB\\STM32L4xx_HAL_Driver\\Inc\\stm32l4xx_ll_adc.h"


 



 
#line 833 "..\\HALLIB\\STM32L4xx_HAL_Driver\\Inc\\stm32l4xx_ll_adc.h"


 



 





 



 




 



 





 

#line 876 "..\\HALLIB\\STM32L4xx_HAL_Driver\\Inc\\stm32l4xx_ll_adc.h"

#line 887 "..\\HALLIB\\STM32L4xx_HAL_Driver\\Inc\\stm32l4xx_ll_adc.h"



 




 



 
#line 916 "..\\HALLIB\\STM32L4xx_HAL_Driver\\Inc\\stm32l4xx_ll_adc.h"


 



 
#line 932 "..\\HALLIB\\STM32L4xx_HAL_Driver\\Inc\\stm32l4xx_ll_adc.h"


 



 
#line 955 "..\\HALLIB\\STM32L4xx_HAL_Driver\\Inc\\stm32l4xx_ll_adc.h"


 



 
#line 979 "..\\HALLIB\\STM32L4xx_HAL_Driver\\Inc\\stm32l4xx_ll_adc.h"


 



 





 



 




 



 





 



 






 



 




 



 






 



 
#line 1054 "..\\HALLIB\\STM32L4xx_HAL_Driver\\Inc\\stm32l4xx_ll_adc.h"


 



 





 



 





 



 
#line 1174 "..\\HALLIB\\STM32L4xx_HAL_Driver\\Inc\\stm32l4xx_ll_adc.h"


 



 





 



 







 



 




 



 
#line 1220 "..\\HALLIB\\STM32L4xx_HAL_Driver\\Inc\\stm32l4xx_ll_adc.h"


 



 
#line 1236 "..\\HALLIB\\STM32L4xx_HAL_Driver\\Inc\\stm32l4xx_ll_adc.h"


 




 
#line 1252 "..\\HALLIB\\STM32L4xx_HAL_Driver\\Inc\\stm32l4xx_ll_adc.h"


 



 







 



 
#line 1283 "..\\HALLIB\\STM32L4xx_HAL_Driver\\Inc\\stm32l4xx_ll_adc.h"


 



 





 





 
#line 1309 "..\\HALLIB\\STM32L4xx_HAL_Driver\\Inc\\stm32l4xx_ll_adc.h"

#line 1317 "..\\HALLIB\\STM32L4xx_HAL_Driver\\Inc\\stm32l4xx_ll_adc.h"

#line 1327 "..\\HALLIB\\STM32L4xx_HAL_Driver\\Inc\\stm32l4xx_ll_adc.h"



 








 
  
 
 
 
 
 
 
 
 
 
 
 
 
 
 
 
 
 

 
 
 
 


 
 
 
 


 
 
 
 


 
 
 
 
 
 
 




 



 


 


 



 







 







 



 



 

















































 
#line 1487 "..\\HALLIB\\STM32L4xx_HAL_Driver\\Inc\\stm32l4xx_ll_adc.h"

















































 
#line 1551 "..\\HALLIB\\STM32L4xx_HAL_Driver\\Inc\\stm32l4xx_ll_adc.h"


























































 










































































 




































 
#line 1777 "..\\HALLIB\\STM32L4xx_HAL_Driver\\Inc\\stm32l4xx_ll_adc.h"
























































































































































 
#line 1939 "..\\HALLIB\\STM32L4xx_HAL_Driver\\Inc\\stm32l4xx_ll_adc.h"




















 





















 














 















 
















 
















 
#line 2060 "..\\HALLIB\\STM32L4xx_HAL_Driver\\Inc\\stm32l4xx_ll_adc.h"










 
#line 2081 "..\\HALLIB\\STM32L4xx_HAL_Driver\\Inc\\stm32l4xx_ll_adc.h"

















 
#line 2112 "..\\HALLIB\\STM32L4xx_HAL_Driver\\Inc\\stm32l4xx_ll_adc.h"













 




















 
#line 2154 "..\\HALLIB\\STM32L4xx_HAL_Driver\\Inc\\stm32l4xx_ll_adc.h"
















 
#line 2177 "..\\HALLIB\\STM32L4xx_HAL_Driver\\Inc\\stm32l4xx_ll_adc.h"

 


























 
#line 2213 "..\\HALLIB\\STM32L4xx_HAL_Driver\\Inc\\stm32l4xx_ll_adc.h"













































 
#line 2272 "..\\HALLIB\\STM32L4xx_HAL_Driver\\Inc\\stm32l4xx_ll_adc.h"












































 
#line 2334 "..\\HALLIB\\STM32L4xx_HAL_Driver\\Inc\\stm32l4xx_ll_adc.h"



 



 


 


 



 
 
 
 






























 

static __inline uint32_t LL_ADC_DMA_GetRegAddr(ADC_TypeDef *ADCx, uint32_t Register)
{
  register uint32_t data_reg_addr;
  
  if (Register == (0x00000000UL))
  {
     
    data_reg_addr = (uint32_t)&(ADCx->DR);
  }
  else  
  {
     
    data_reg_addr = (uint32_t)&(((((ADC_Common_TypeDef *) (((0x40000000UL) + 0x08000000UL) + 0x08040300UL))))->CDR);
  }
  
  return data_reg_addr;
}
#line 2414 "..\\HALLIB\\STM32L4xx_HAL_Driver\\Inc\\stm32l4xx_ll_adc.h"



 



 


































 
static __inline void LL_ADC_SetCommonClock(ADC_Common_TypeDef *ADCxy_COMMON, uint32_t CommonClock)
{
  (((ADCxy_COMMON->CCR)) = ((((((ADCxy_COMMON->CCR))) & (~((0x3UL << (16U)) | (0xFUL << (18U))))) | (CommonClock))));
}























 
static __inline uint32_t LL_ADC_GetCommonClock(ADC_Common_TypeDef *ADCxy_COMMON)
{
  return (uint32_t)(((ADCxy_COMMON->CCR) & ((0x3UL << (16U)) | (0xFUL << (18U)))));
}





























 
static __inline void LL_ADC_SetCommonPathInternalCh(ADC_Common_TypeDef *ADCxy_COMMON, uint32_t PathInternal)
{
  (((ADCxy_COMMON->CCR)) = ((((((ADCxy_COMMON->CCR))) & (~((0x1UL << (22U)) | (0x1UL << (23U)) | (0x1UL << (24U))))) | (PathInternal))));
}

















 
static __inline uint32_t LL_ADC_GetCommonPathInternalCh(ADC_Common_TypeDef *ADCxy_COMMON)
{
  return (uint32_t)(((ADCxy_COMMON->CCR) & ((0x1UL << (22U)) | (0x1UL << (23U)) | (0x1UL << (24U)))));
}



 



 































 
static __inline void LL_ADC_SetCalibrationFactor(ADC_TypeDef *ADCx, uint32_t SingleDiff, uint32_t CalibrationFactor)
{
  (((ADCx->CALFACT)) = ((((((ADCx->CALFACT))) & (~(SingleDiff & ((0x7FUL << (16U)) | (0x7FUL << (0U)))))) | (CalibrationFactor << (((SingleDiff & (0x00010000UL)) >> ((16UL) - 4UL)) & ~(SingleDiff & (0x7FUL << (0U))))))));


}
















 
static __inline uint32_t LL_ADC_GetCalibrationFactor(ADC_TypeDef *ADCx, uint32_t SingleDiff)
{
   
   
   
   
  return (uint32_t)(((ADCx->CALFACT) & ((SingleDiff & ((0x7FUL << (16U)) | (0x7FUL << (0U)))))) >> ((SingleDiff & (0x00010000UL)) >> ((16UL) - 4UL)));
}

















 
static __inline void LL_ADC_SetResolution(ADC_TypeDef *ADCx, uint32_t Resolution)
{
  (((ADCx->CFGR)) = ((((((ADCx->CFGR))) & (~((0x3UL << (3U))))) | (Resolution))));
}












 
static __inline uint32_t LL_ADC_GetResolution(ADC_TypeDef *ADCx)
{
  return (uint32_t)(((ADCx->CFGR) & ((0x3UL << (3U)))));
}















 
static __inline void LL_ADC_SetDataAlignment(ADC_TypeDef *ADCx, uint32_t DataAlignment)
{
  (((ADCx->CFGR)) = ((((((ADCx->CFGR))) & (~((0x1UL << (5U))))) | (DataAlignment))));
}










 
static __inline uint32_t LL_ADC_GetDataAlignment(ADC_TypeDef *ADCx)
{
  return (uint32_t)(((ADCx->CFGR) & ((0x1UL << (5U)))));
}
















































 
static __inline void LL_ADC_SetLowPowerMode(ADC_TypeDef *ADCx, uint32_t LowPowerMode)
{
  (((ADCx->CFGR)) = ((((((ADCx->CFGR))) & (~((0x1UL << (14U))))) | (LowPowerMode))));
}











































 
static __inline uint32_t LL_ADC_GetLowPowerMode(ADC_TypeDef *ADCx)
{
  return (uint32_t)(((ADCx->CFGR) & ((0x1UL << (14U)))));
}
















































































 
static __inline void LL_ADC_SetOffset(ADC_TypeDef *ADCx, uint32_t Offsety, uint32_t Channel, uint32_t OffsetLevel)
{
  register uint32_t *preg = ((uint32_t *)((uint32_t) ((uint32_t)(&(ADCx->OFR1)) + ((Offsety) << 2UL))));
  
  (((*preg)) = ((((((*preg))) & (~((0x1UL << (31U)) | (0x1FUL << (26U)) | (0xFFFUL << (0U))))) | ((0x1UL << (31U)) | (Channel & ((0x1FUL << (26U)))) | OffsetLevel))));


}






































































 
static __inline uint32_t LL_ADC_GetOffsetChannel(ADC_TypeDef *ADCx, uint32_t Offsety)
{
  register const uint32_t *preg = ((uint32_t *)((uint32_t) ((uint32_t)(&(ADCx->OFR1)) + ((Offsety) << 2UL))));
  
  return (uint32_t) ((*preg) & ((0x1FUL << (26U))));
}



















 
static __inline uint32_t LL_ADC_GetOffsetLevel(ADC_TypeDef *ADCx, uint32_t Offsety)
{
  register const uint32_t *preg = ((uint32_t *)((uint32_t) ((uint32_t)(&(ADCx->OFR1)) + ((Offsety) << 2UL))));
  
  return (uint32_t) ((*preg) & ((0xFFFUL << (0U))));
}


























 
static __inline void LL_ADC_SetOffsetState(ADC_TypeDef *ADCx, uint32_t Offsety, uint32_t OffsetState)
{
  register uint32_t *preg = ((uint32_t *)((uint32_t) ((uint32_t)(&(ADCx->OFR1)) + ((Offsety) << 2UL))));
  
  (((*preg)) = ((((((*preg))) & (~((0x1UL << (31U))))) | (OffsetState))));


}

















 
static __inline uint32_t LL_ADC_GetOffsetState(ADC_TypeDef *ADCx, uint32_t Offsety)
{
  register const uint32_t *preg = ((uint32_t *)((uint32_t) ((uint32_t)(&(ADCx->OFR1)) + ((Offsety) << 2UL))));
  
  return (uint32_t) ((*preg) & ((0x1UL << (31U))));
}

#line 3077 "..\\HALLIB\\STM32L4xx_HAL_Driver\\Inc\\stm32l4xx_ll_adc.h"



 



 







































 
static __inline void LL_ADC_REG_SetTriggerSource(ADC_TypeDef *ADCx, uint32_t TriggerSource)
{
  (((ADCx->CFGR)) = ((((((ADCx->CFGR))) & (~((0x3UL << (10U)) | (0xFUL << (6U))))) | (TriggerSource))));
}


































 
static __inline uint32_t LL_ADC_REG_GetTriggerSource(ADC_TypeDef *ADCx)
{
  register uint32_t TriggerSource = ((ADCx->CFGR) & ((0xFUL << (6U)) | (0x3UL << (10U))));
  
   
   
  register uint32_t ShiftExten = ((TriggerSource & (0x3UL << (10U))) >> ((10UL) - 2UL));
  
   
   
  return ((TriggerSource
           & (((((0x00000000UL) & (0xFUL << (6U))) << (4U * 0UL)) | (((0xFUL << (6U))) << (4U * 1UL)) | (((0xFUL << (6U))) << (4U * 2UL)) | (((0xFUL << (6U))) << (4U * 3UL)) ) >> ShiftExten) & (0xFUL << (6U)))
          | ((((((0x00000000UL) & (0x3UL << (10U))) << (4U * 0UL)) | ((((0x1UL << (10U)))) << (4U * 1UL)) | ((((0x1UL << (10U)))) << (4U * 2UL)) | ((((0x1UL << (10U)))) << (4U * 3UL)) ) >> ShiftExten) & (0x3UL << (10U)))
         );
}











 
static __inline uint32_t LL_ADC_REG_IsTriggerSourceSWStart(ADC_TypeDef *ADCx)
{
  return ((((ADCx->CFGR) & ((0x3UL << (10U)))) == ((0x00000000UL) & (0x3UL << (10U)))) ? 1UL : 0UL);
}















 
static __inline void LL_ADC_REG_SetTriggerEdge(ADC_TypeDef *ADCx, uint32_t ExternalTriggerEdge)
{
  (((ADCx->CFGR)) = ((((((ADCx->CFGR))) & (~((0x3UL << (10U))))) | (ExternalTriggerEdge))));
}










 
static __inline uint32_t LL_ADC_REG_GetTriggerEdge(ADC_TypeDef *ADCx)
{
  return (uint32_t)(((ADCx->CFGR) & ((0x3UL << (10U)))));
}






















































 
static __inline void LL_ADC_REG_SetSequencerLength(ADC_TypeDef *ADCx, uint32_t SequencerNbRanks)
{
  (((ADCx->SQR1)) = ((((((ADCx->SQR1))) & (~((0xFUL << (0U))))) | (SequencerNbRanks))));
}

















































 
static __inline uint32_t LL_ADC_REG_GetSequencerLength(ADC_TypeDef *ADCx)
{
  return (uint32_t)(((ADCx->SQR1) & ((0xFUL << (0U)))));
}



























 
static __inline void LL_ADC_REG_SetSequencerDiscont(ADC_TypeDef *ADCx, uint32_t SeqDiscont)
{
  (((ADCx->CFGR)) = ((((((ADCx->CFGR))) & (~((0x1UL << (16U)) | (0x7UL << (17U))))) | (SeqDiscont))));
}


















 
static __inline uint32_t LL_ADC_REG_GetSequencerDiscont(ADC_TypeDef *ADCx)
{
  return (uint32_t)(((ADCx->CFGR) & ((0x1UL << (16U)) | (0x7UL << (17U)))));
}






























































































 
static __inline void LL_ADC_REG_SetSequencerRanks(ADC_TypeDef *ADCx, uint32_t Rank, uint32_t Channel)
{
   
   
   
   
  register uint32_t *preg = ((uint32_t *)((uint32_t) ((uint32_t)(&(ADCx->SQR1)) + ((((Rank & ((0x00000000UL) | (0x00000100UL) | (0x00000200UL) | (0x00000300UL))) >> (8UL))) << 2UL))));
  
  (((*preg)) = ((((((*preg))) & (~(((0x1FUL << (0U))) << (Rank & (((0x1FUL << (0U)))))))) | (((Channel & ((0x1FUL << (26U)))) >> (26UL)) << (Rank & (((0x1FUL << (0U)))))))));


}
































































































 
static __inline uint32_t LL_ADC_REG_GetSequencerRanks(ADC_TypeDef *ADCx, uint32_t Rank)
{
  register const uint32_t *preg = ((uint32_t *)((uint32_t) ((uint32_t)(&(ADCx->SQR1)) + ((((Rank & ((0x00000000UL) | (0x00000100UL) | (0x00000200UL) | (0x00000300UL))) >> (8UL))) << 2UL))));
  
  return (uint32_t) ((((*preg) & (((0x1FUL << (0U))) << (Rank & (((0x1FUL << (0U)))))))

                     >> (Rank & (((0x1FUL << (0U)))))) << (26UL)
                    );
}



















 
static __inline void LL_ADC_REG_SetContinuousMode(ADC_TypeDef *ADCx, uint32_t Continuous)
{
  (((ADCx->CFGR)) = ((((((ADCx->CFGR))) & (~((0x1UL << (13U))))) | (Continuous))));
}












 
static __inline uint32_t LL_ADC_REG_GetContinuousMode(ADC_TypeDef *ADCx)
{
  return (uint32_t)(((ADCx->CFGR) & ((0x1UL << (13U)))));
}



































 
static __inline void LL_ADC_REG_SetDMATransfer(ADC_TypeDef *ADCx, uint32_t DMATransfer)
{
  (((ADCx->CFGR)) = ((((((ADCx->CFGR))) & (~((0x1UL << (0U)) | (0x1UL << (1U))))) | (DMATransfer))));
}






























 
static __inline uint32_t LL_ADC_REG_GetDMATransfer(ADC_TypeDef *ADCx)
{
  return (uint32_t)(((ADCx->CFGR) & ((0x1UL << (0U)) | (0x1UL << (1U)))));
}

#line 3764 "..\\HALLIB\\STM32L4xx_HAL_Driver\\Inc\\stm32l4xx_ll_adc.h"




















 
static __inline void LL_ADC_REG_SetOverrun(ADC_TypeDef *ADCx, uint32_t Overrun)
{
  (((ADCx->CFGR)) = ((((((ADCx->CFGR))) & (~((0x1UL << (12U))))) | (Overrun))));
}









 
static __inline uint32_t LL_ADC_REG_GetOverrun(ADC_TypeDef *ADCx)
{
  return (uint32_t)(((ADCx->CFGR) & ((0x1UL << (12U)))));
}



 



 







































 
static __inline void LL_ADC_INJ_SetTriggerSource(ADC_TypeDef *ADCx, uint32_t TriggerSource)
{
  (((ADCx->JSQR)) = ((((((ADCx->JSQR))) & (~((0xFUL << (2U)) | (0x3UL << (6U))))) | (TriggerSource))));
}


































 
static __inline uint32_t LL_ADC_INJ_GetTriggerSource(ADC_TypeDef *ADCx)
{
  register uint32_t TriggerSource = ((ADCx->JSQR) & ((0xFUL << (2U)) | (0x3UL << (6U))));
  
   
   
  register uint32_t ShiftJexten = ((TriggerSource & (0x3UL << (6U))) >> (( 6UL) - 2UL));
  
   
   
  return ((TriggerSource
           & (((((0x00000000UL) & (0xFUL << (2U))) << (4U * 0UL)) | (((0xFUL << (2U))) << (4U * 1UL)) | (((0xFUL << (2U))) << (4U * 2UL)) | (((0xFUL << (2U))) << (4U * 3UL)) ) >> ShiftJexten) & (0xFUL << (2U)))
          | ((((((0x00000000UL) & (0x3UL << (6U))) << (4U * 0UL)) | ((((0x1UL << (6U)))) << (4U * 1UL)) | ((((0x1UL << (6U)))) << (4U * 2UL)) | ((((0x1UL << (6U)))) << (4U * 3UL)) ) >> ShiftJexten) & (0x3UL << (6U)))
         );
}











 
static __inline uint32_t LL_ADC_INJ_IsTriggerSourceSWStart(ADC_TypeDef *ADCx)
{
  return ((((ADCx->JSQR) & ((0x3UL << (6U)))) == ((0x00000000UL) & (0x3UL << (6U)))) ? 1UL : 0UL);
}















 
static __inline void LL_ADC_INJ_SetTriggerEdge(ADC_TypeDef *ADCx, uint32_t ExternalTriggerEdge)
{
  (((ADCx->JSQR)) = ((((((ADCx->JSQR))) & (~((0x3UL << (6U))))) | (ExternalTriggerEdge))));
}










 
static __inline uint32_t LL_ADC_INJ_GetTriggerEdge(ADC_TypeDef *ADCx)
{
  return (uint32_t)(((ADCx->JSQR) & ((0x3UL << (6U)))));
}





















 
static __inline void LL_ADC_INJ_SetSequencerLength(ADC_TypeDef *ADCx, uint32_t SequencerNbRanks)
{
  (((ADCx->JSQR)) = ((((((ADCx->JSQR))) & (~((0x3UL << (0U))))) | (SequencerNbRanks))));
}
















 
static __inline uint32_t LL_ADC_INJ_GetSequencerLength(ADC_TypeDef *ADCx)
{
  return (uint32_t)(((ADCx->JSQR) & ((0x3UL << (0U)))));
}













 
static __inline void LL_ADC_INJ_SetSequencerDiscont(ADC_TypeDef *ADCx, uint32_t SeqDiscont)
{
  (((ADCx->CFGR)) = ((((((ADCx->CFGR))) & (~((0x1UL << (20U))))) | (SeqDiscont))));
}










 
static __inline uint32_t LL_ADC_INJ_GetSequencerDiscont(ADC_TypeDef *ADCx)
{
  return (uint32_t)(((ADCx->CFGR) & ((0x1UL << (20U)))));
}

































































 
static __inline void LL_ADC_INJ_SetSequencerRanks(ADC_TypeDef *ADCx, uint32_t Rank, uint32_t Channel)
{
   
   
   
   
  (((ADCx->JSQR)) = ((((((ADCx->JSQR))) & (~((((0x1FUL << (26U))) >> (26UL)) << (Rank & (((0x1FUL << (0U)))))))) | (((Channel & ((0x1FUL << (26U)))) >> (26UL)) << (Rank & (((0x1FUL << (0U)))))))));


}




































































 
static __inline uint32_t LL_ADC_INJ_GetSequencerRanks(ADC_TypeDef *ADCx, uint32_t Rank)
{
  return (uint32_t)((((ADCx->JSQR) & ((((0x1FUL << (26U))) >> (26UL)) << (Rank & (((0x1FUL << (0U)))))))

                    >> (Rank & (((0x1FUL << (0U)))))) << (26UL)
                   );
}






























 
static __inline void LL_ADC_INJ_SetTrigAuto(ADC_TypeDef *ADCx, uint32_t TrigAuto)
{
  (((ADCx->CFGR)) = ((((((ADCx->CFGR))) & (~((0x1UL << (25U))))) | (TrigAuto))));
}









 
static __inline uint32_t LL_ADC_INJ_GetTrigAuto(ADC_TypeDef *ADCx)
{
  return (uint32_t)(((ADCx->CFGR) & ((0x1UL << (25U)))));
}









































 
static __inline void LL_ADC_INJ_SetQueueMode(ADC_TypeDef *ADCx, uint32_t QueueMode)
{
  (((ADCx->CFGR)) = ((((((ADCx->CFGR))) & (~((0x1UL << (21U)) | (0x1UL << (31U))))) | (QueueMode))));
}










 
static __inline uint32_t LL_ADC_INJ_GetQueueMode(ADC_TypeDef *ADCx)
{
  return (uint32_t)(((ADCx->CFGR) & ((0x1UL << (21U)) | (0x1UL << (31U)))));
}



























































































































































































































 
static __inline void LL_ADC_INJ_ConfigQueueContext(ADC_TypeDef *ADCx,
                                                   uint32_t TriggerSource,
                                                   uint32_t ExternalTriggerEdge,
                                                   uint32_t SequencerNbRanks,
                                                   uint32_t Rank1_Channel,
                                                   uint32_t Rank2_Channel,
                                                   uint32_t Rank3_Channel,
                                                   uint32_t Rank4_Channel)
{
   
   
   
   
   
   
  register uint32_t is_trigger_not_sw = (uint32_t)((TriggerSource != (0x00000000UL)) ? 1UL : 0UL);
  (((ADCx->JSQR)) = ((((((ADCx->JSQR))) & (~((0xFUL << (2U)) | (0x3UL << (6U)) | (0x1FUL << (26U)) | (0x1FUL << (20U)) | (0x1FUL << (14U)) | (0x1FUL << (8U)) | (0x3UL << (0U))))) | (TriggerSource | (ExternalTriggerEdge * (is_trigger_not_sw)) | (((Rank4_Channel & ((0x1FUL << (26U)))) >> (26UL)) << (((0x00000300UL) | (26UL)) & (((0x1FUL << (0U)))))) | (((Rank3_Channel & ((0x1FUL << (26U)))) >> (26UL)) << (((0x00000200UL) | (20UL)) & (((0x1FUL << (0U)))))) | (((Rank2_Channel & ((0x1FUL << (26U)))) >> (26UL)) << (((0x00000100UL) | (14UL)) & (((0x1FUL << (0U)))))) | (((Rank1_Channel & ((0x1FUL << (26U)))) >> (26UL)) << (((0x00000000UL) | ( 8UL)) & (((0x1FUL << (0U)))))) | SequencerNbRanks))));
#line 4550 "..\\HALLIB\\STM32L4xx_HAL_Driver\\Inc\\stm32l4xx_ll_adc.h"
}



 



 



































































































 
static __inline void LL_ADC_SetChannelSamplingTime(ADC_TypeDef *ADCx, uint32_t Channel, uint32_t SamplingTime)
{
   
   
   
   
  register uint32_t *preg = ((uint32_t *)((uint32_t) ((uint32_t)(&(ADCx->SMPR1)) + ((((Channel & ((0x00000000UL) | (0x02000000UL))) >> (25UL))) << 2UL))));
  
  (((*preg)) = ((((((*preg))) & (~((0x7UL << (0U)) << ((Channel & (0x01F00000UL)) >> (20UL))))) | (SamplingTime << ((Channel & (0x01F00000UL)) >> (20UL))))));


}



















































































 
static __inline uint32_t LL_ADC_GetChannelSamplingTime(ADC_TypeDef *ADCx, uint32_t Channel)
{
  register const uint32_t *preg = ((uint32_t *)((uint32_t) ((uint32_t)(&(ADCx->SMPR1)) + ((((Channel & ((0x00000000UL) | (0x02000000UL))) >> (25UL))) << 2UL))));
  
  return (uint32_t)(((*preg) & ((0x7UL << (0U)) << ((Channel & (0x01F00000UL)) >> (20UL))))

                    >> ((Channel & (0x01F00000UL)) >> (20UL))
                   );
}
















































 
static __inline void LL_ADC_SetChannelSingleDiff(ADC_TypeDef *ADCx, uint32_t Channel, uint32_t SingleDiff)
{
   
   
   
  (((ADCx->DIFSEL)) = ((((((ADCx->DIFSEL))) & (~(Channel & (((0x7FFFFUL << (0U))))))) | ((Channel & (((0x7FFFFUL << (0U))))) & ((0x7FFFFUL << (0U)) >> (SingleDiff & ((0x10UL << (0U)) | (0x08UL << (0U)))))))));


}








































 
static __inline uint32_t LL_ADC_GetChannelSingleDiff(ADC_TypeDef *ADCx, uint32_t Channel)
{
  return (uint32_t)(((ADCx->DIFSEL) & ((Channel & (((0x7FFFFUL << (0U))))))));
}



 



 
















































































































































 
static __inline void LL_ADC_SetAnalogWDMonitChannels(ADC_TypeDef *ADCx, uint32_t AWDy, uint32_t AWDChannelGroup)
{
   
   
   
   
  register uint32_t *preg = ((uint32_t *)((uint32_t) ((uint32_t)(&(ADCx->CFGR)) + ((((AWDy & ((0x00000000UL) | (0x00100000UL) | (0x00200000UL))) >> (20UL)) + ((AWDy & ((0x00001UL << (0U)))) * (0x00000024UL))) << 2UL))));

  
  (((*preg)) = ((((((*preg))) & (~((AWDy & (((0x1FUL << (26U)) | (0x1UL << (24U)) | (0x1UL << (23U)) | (0x1UL << (22U))) | ((0x7FFFFUL << (0U)))))))) | (AWDChannelGroup & AWDy))));


}


























































































































 
static __inline uint32_t LL_ADC_GetAnalogWDMonitChannels(ADC_TypeDef *ADCx, uint32_t AWDy)
{
  register const uint32_t *preg = ((uint32_t *)((uint32_t) ((uint32_t)(&(ADCx->CFGR)) + ((((AWDy & ((0x00000000UL) | (0x00100000UL) | (0x00200000UL))) >> (20UL)) + ((AWDy & ((0x00001UL << (0U)))) * (0x00000024UL))) << 2UL))));

  
  register uint32_t AnalogWDMonitChannels = (((*preg) & (AWDy)) & AWDy & (((0x1FUL << (26U)) | (0x1UL << (24U)) | (0x1UL << (23U)) | (0x1UL << (22U))) | ((0x7FFFFUL << (0U)))));
  
   
   
   
   
  if(AnalogWDMonitChannels != 0UL)
  {
    if(AWDy == (((0x1FUL << (26U)) | (0x1UL << (24U)) | (0x1UL << (23U)) | (0x1UL << (22U))) | (0x00000000UL)))
    {
      if((AnalogWDMonitChannels & (0x1UL << (22U))) == 0UL)
      {
         
        AnalogWDMonitChannels = ((  AnalogWDMonitChannels
                                  | (((0x7FFFFUL << (0U))))
                                 )
                                 & (~((0x1FUL << (26U))))
                                );
      }
      else
      {
         
        AnalogWDMonitChannels = (AnalogWDMonitChannels
                                 | ((0x00001UL << (0U)) << (AnalogWDMonitChannels >> (26U)))
                                );
      }
    }
    else
    {
      if((AnalogWDMonitChannels & ((0x7FFFFUL << (0U)))) == ((0x7FFFFUL << (0U))))
      {
         
        AnalogWDMonitChannels = (  ((0x7FFFFUL << (0U)))
                                 | (((0x1UL << (24U)) | (0x1UL << (23U))))
                                );
      }
      else
      {
         
         
        AnalogWDMonitChannels = (  AnalogWDMonitChannels
                                 | ((0x1UL << (24U)) | (0x1UL << (23U)) | (0x1UL << (22U)))
                                 | (((((AnalogWDMonitChannels) & ((0x7FFFFUL << (0U)))) == 0UL) ? ( ((AnalogWDMonitChannels) & ((0x1FUL << (26U)))) >> (26UL) ) : ( (__clz(__rbit((AnalogWDMonitChannels)))) ) ) << (26U))
                                );
      }
    }
  }
  
  return AnalogWDMonitChannels;

}




















































 
static __inline void LL_ADC_ConfigAnalogWDThresholds(ADC_TypeDef *ADCx, uint32_t AWDy, uint32_t AWDThresholdHighValue, uint32_t AWDThresholdLowValue)
{
   
   
   
   
   
  register uint32_t *preg = ((uint32_t *)((uint32_t) ((uint32_t)(&(ADCx->TR1)) + ((((AWDy & (((0x00000000UL)) | ((0x00100000UL)) | ((0x00200000UL)))) >> ((20UL)))) << 2UL))));
  
  (((*preg)) = ((((((*preg))) & (~((0xFFFUL << (16U)) | (0xFFFUL << (0U))))) | ((AWDThresholdHighValue << (16UL)) | AWDThresholdLowValue))));


}






















































 
static __inline void LL_ADC_SetAnalogWDThresholds(ADC_TypeDef *ADCx, uint32_t AWDy, uint32_t AWDThresholdsHighLow, uint32_t AWDThresholdValue)
{
   
   
   
   
   
  register uint32_t *preg = ((uint32_t *)((uint32_t) ((uint32_t)(&(ADCx->TR1)) + ((((AWDy & (((0x00000000UL)) | ((0x00100000UL)) | ((0x00200000UL)))) >> ((20UL)))) << 2UL))));
  
  (((*preg)) = ((((((*preg))) & (~(AWDThresholdsHighLow))) | (AWDThresholdValue << ((AWDThresholdsHighLow & (0x00010000UL)) >> ((16UL) - 4UL))))));


}




























 
static __inline uint32_t LL_ADC_GetAnalogWDThresholds(ADC_TypeDef *ADCx, uint32_t AWDy, uint32_t AWDThresholdsHighLow)
{
  register const uint32_t *preg = ((uint32_t *)((uint32_t) ((uint32_t)(&(ADCx->TR1)) + ((((AWDy & (((0x00000000UL)) | ((0x00100000UL)) | ((0x00200000UL)))) >> ((20UL)))) << 2UL))));
  
  return (uint32_t)(((*preg) & ((AWDThresholdsHighLow | (0xFFFUL << (0U)))))

                    >> (((AWDThresholdsHighLow & (0x00010000UL)) >> ((16UL) - 4UL)) & ~(AWDThresholdsHighLow & (0xFFFUL << (0U))))
                   );
}



 



 

























 
static __inline void LL_ADC_SetOverSamplingScope(ADC_TypeDef *ADCx, uint32_t OvsScope)
{
  (((ADCx->CFGR2)) = ((((((ADCx->CFGR2))) & (~((0x1UL << (0U)) | (0x1UL << (1U)) | (0x1UL << (10U))))) | (OvsScope))));
}




















 
static __inline uint32_t LL_ADC_GetOverSamplingScope(ADC_TypeDef *ADCx)
{
  return (uint32_t)(((ADCx->CFGR2) & ((0x1UL << (0U)) | (0x1UL << (1U)) | (0x1UL << (10U)))));
}






















 
static __inline void LL_ADC_SetOverSamplingDiscont(ADC_TypeDef *ADCx, uint32_t OverSamplingDiscont)
{
  (((ADCx->CFGR2)) = ((((((ADCx->CFGR2))) & (~((0x1UL << (9U))))) | (OverSamplingDiscont))));
}














 
static __inline uint32_t LL_ADC_GetOverSamplingDiscont(ADC_TypeDef *ADCx)
{
  return (uint32_t)(((ADCx->CFGR2) & ((0x1UL << (9U)))));
}


































 
static __inline void LL_ADC_ConfigOverSamplingRatioShift(ADC_TypeDef *ADCx, uint32_t Ratio, uint32_t Shift)
{
  (((ADCx->CFGR2)) = ((((((ADCx->CFGR2))) & (~(((0xFUL << (5U)) | (0x7UL << (2U)))))) | ((Shift | Ratio)))));
}















 
static __inline uint32_t LL_ADC_GetOverSamplingRatio(ADC_TypeDef *ADCx)
{
  return (uint32_t)(((ADCx->CFGR2) & ((0x7UL << (2U)))));
}
















 
static __inline uint32_t LL_ADC_GetOverSamplingShift(ADC_TypeDef *ADCx)
{
  return (uint32_t)(((ADCx->CFGR2) & ((0xFUL << (5U)))));
}



 



 



























 
static __inline void LL_ADC_SetMultimode(ADC_Common_TypeDef *ADCxy_COMMON, uint32_t Multimode)
{
  (((ADCxy_COMMON->CCR)) = ((((((ADCxy_COMMON->CCR))) & (~((0x1FUL << (0U))))) | (Multimode))));
}



















 
static __inline uint32_t LL_ADC_GetMultimode(ADC_Common_TypeDef *ADCxy_COMMON)
{
  return (uint32_t)(((ADCxy_COMMON->CCR) & ((0x1FUL << (0U)))));
}














































 
static __inline void LL_ADC_SetMultiDMATransfer(ADC_Common_TypeDef *ADCxy_COMMON, uint32_t MultiDMATransfer)
{
  (((ADCxy_COMMON->CCR)) = ((((((ADCxy_COMMON->CCR))) & (~((0x3UL << (14U)) | (0x1UL << (13U))))) | (MultiDMATransfer))));
}









































 
static __inline uint32_t LL_ADC_GetMultiDMATransfer(ADC_Common_TypeDef *ADCxy_COMMON)
{
  return (uint32_t)(((ADCxy_COMMON->CCR) & ((0x3UL << (14U)) | (0x1UL << (13U)))));
}



































 
static __inline void LL_ADC_SetMultiTwoSamplingDelay(ADC_Common_TypeDef *ADCxy_COMMON, uint32_t MultiTwoSamplingDelay)
{
  (((ADCxy_COMMON->CCR)) = ((((((ADCxy_COMMON->CCR))) & (~((0xFUL << (8U))))) | (MultiTwoSamplingDelay))));
}























 
static __inline uint32_t LL_ADC_GetMultiTwoSamplingDelay(ADC_Common_TypeDef *ADCxy_COMMON)
{
  return (uint32_t)(((ADCxy_COMMON->CCR) & ((0xFUL << (8U)))));
}




 


 
 
 
static __inline void LL_ADC_REG_SetTrigSource(ADC_TypeDef *ADCx, uint32_t TriggerSource)
{
  LL_ADC_REG_SetTriggerSource(ADCx, TriggerSource);
}
static __inline void LL_ADC_INJ_SetTrigSource(ADC_TypeDef *ADCx, uint32_t TriggerSource)
{
  LL_ADC_INJ_SetTriggerSource(ADCx, TriggerSource);
}



 



 













 
static __inline void LL_ADC_EnableDeepPowerDown(ADC_TypeDef *ADCx)
{
   
   
   
  (((ADCx->CR)) = ((((((ADCx->CR))) & (~(((0x1UL << (31U)) | (0x1UL << (5U)) | (0x1UL << (4U)) | (0x1UL << (3U)) | (0x1UL << (2U)) | (0x1UL << (1U)) | (0x1UL << (0U)))))) | ((0x1UL << (29U))))));


}













 
static __inline void LL_ADC_DisableDeepPowerDown(ADC_TypeDef *ADCx)
{
   
   
   
  ((ADCx->CR) &= ~(((0x1UL << (29U)) | ((0x1UL << (31U)) | (0x1UL << (5U)) | (0x1UL << (4U)) | (0x1UL << (3U)) | (0x1UL << (2U)) | (0x1UL << (1U)) | (0x1UL << (0U))))));
}






 
static __inline uint32_t LL_ADC_IsDeepPowerDownEnabled(ADC_TypeDef *ADCx)
{
  return ((((ADCx->CR) & ((0x1UL << (29U)))) == ((0x1UL << (29U)))) ? 1UL : 0UL);
}














 
static __inline void LL_ADC_EnableInternalRegulator(ADC_TypeDef *ADCx)
{
   
   
   
  (((ADCx->CR)) = ((((((ADCx->CR))) & (~(((0x1UL << (31U)) | (0x1UL << (5U)) | (0x1UL << (4U)) | (0x1UL << (3U)) | (0x1UL << (2U)) | (0x1UL << (1U)) | (0x1UL << (0U)))))) | ((0x1UL << (28U))))));


}









 
static __inline void LL_ADC_DisableInternalRegulator(ADC_TypeDef *ADCx)
{
  ((ADCx->CR) &= ~(((0x1UL << (28U)) | ((0x1UL << (31U)) | (0x1UL << (5U)) | (0x1UL << (4U)) | (0x1UL << (3U)) | (0x1UL << (2U)) | (0x1UL << (1U)) | (0x1UL << (0U))))));
}






 
static __inline uint32_t LL_ADC_IsInternalRegulatorEnabled(ADC_TypeDef *ADCx)
{
  return ((((ADCx->CR) & ((0x1UL << (28U)))) == ((0x1UL << (28U)))) ? 1UL : 0UL);
}
















 
static __inline void LL_ADC_Enable(ADC_TypeDef *ADCx)
{
   
   
   
  (((ADCx->CR)) = ((((((ADCx->CR))) & (~(((0x1UL << (31U)) | (0x1UL << (5U)) | (0x1UL << (4U)) | (0x1UL << (3U)) | (0x1UL << (2U)) | (0x1UL << (1U)) | (0x1UL << (0U)))))) | ((0x1UL << (0U))))));


}










 
static __inline void LL_ADC_Disable(ADC_TypeDef *ADCx)
{
   
   
   
  (((ADCx->CR)) = ((((((ADCx->CR))) & (~(((0x1UL << (31U)) | (0x1UL << (5U)) | (0x1UL << (4U)) | (0x1UL << (3U)) | (0x1UL << (2U)) | (0x1UL << (1U)) | (0x1UL << (0U)))))) | ((0x1UL << (1U))))));


}









 
static __inline uint32_t LL_ADC_IsEnabled(ADC_TypeDef *ADCx)
{
  return ((((ADCx->CR) & ((0x1UL << (0U)))) == ((0x1UL << (0U)))) ? 1UL : 0UL);
}






 
static __inline uint32_t LL_ADC_IsDisableOngoing(ADC_TypeDef *ADCx)
{
  return ((((ADCx->CR) & ((0x1UL << (1U)))) == ((0x1UL << (1U)))) ? 1UL : 0UL);
}























 
static __inline void LL_ADC_StartCalibration(ADC_TypeDef *ADCx, uint32_t SingleDiff)
{
   
   
   
  (((ADCx->CR)) = ((((((ADCx->CR))) & (~((0x1UL << (30U)) | ((0x1UL << (31U)) | (0x1UL << (5U)) | (0x1UL << (4U)) | (0x1UL << (3U)) | (0x1UL << (2U)) | (0x1UL << (1U)) | (0x1UL << (0U)))))) | ((0x1UL << (31U)) | (SingleDiff & ((0x1UL << (30U))))))));


}






 
static __inline uint32_t LL_ADC_IsCalibrationOnGoing(ADC_TypeDef *ADCx)
{
  return ((((ADCx->CR) & ((0x1UL << (31U)))) == ((0x1UL << (31U)))) ? 1UL : 0UL);
}



 



 


















 
static __inline void LL_ADC_REG_StartConversion(ADC_TypeDef *ADCx)
{
   
   
   
  (((ADCx->CR)) = ((((((ADCx->CR))) & (~(((0x1UL << (31U)) | (0x1UL << (5U)) | (0x1UL << (4U)) | (0x1UL << (3U)) | (0x1UL << (2U)) | (0x1UL << (1U)) | (0x1UL << (0U)))))) | ((0x1UL << (2U))))));


}










 
static __inline void LL_ADC_REG_StopConversion(ADC_TypeDef *ADCx)
{
   
   
   
  (((ADCx->CR)) = ((((((ADCx->CR))) & (~(((0x1UL << (31U)) | (0x1UL << (5U)) | (0x1UL << (4U)) | (0x1UL << (3U)) | (0x1UL << (2U)) | (0x1UL << (1U)) | (0x1UL << (0U)))))) | ((0x1UL << (4U))))));


}






 
static __inline uint32_t LL_ADC_REG_IsConversionOngoing(ADC_TypeDef *ADCx)
{
  return ((((ADCx->CR) & ((0x1UL << (2U)))) == ((0x1UL << (2U)))) ? 1UL : 0UL);
}






 
static __inline uint32_t LL_ADC_REG_IsStopConversionOngoing(ADC_TypeDef *ADCx)
{
  return ((((ADCx->CR) & ((0x1UL << (4U)))) == ((0x1UL << (4U)))) ? 1UL : 0UL);
}









 
static __inline uint32_t LL_ADC_REG_ReadConversionData32(ADC_TypeDef *ADCx)
{
  return (uint32_t)(((ADCx->DR) & ((0xFFFFUL << (0U)))));
}










 
static __inline uint16_t LL_ADC_REG_ReadConversionData12(ADC_TypeDef *ADCx)
{
  return (uint16_t)(((ADCx->DR) & ((0xFFFFUL << (0U)))));
}










 
static __inline uint16_t LL_ADC_REG_ReadConversionData10(ADC_TypeDef *ADCx)
{
  return (uint16_t)(((ADCx->DR) & ((0xFFFFUL << (0U)))));
}










 
static __inline uint8_t LL_ADC_REG_ReadConversionData8(ADC_TypeDef *ADCx)
{
  return (uint8_t)(((ADCx->DR) & ((0xFFFFUL << (0U)))));
}










 
static __inline uint8_t LL_ADC_REG_ReadConversionData6(ADC_TypeDef *ADCx)
{
  return (uint8_t)(((ADCx->DR) & ((0xFFFFUL << (0U)))));
}






















 
static __inline uint32_t LL_ADC_REG_ReadMultiConversionData32(ADC_Common_TypeDef *ADCxy_COMMON, uint32_t ConversionData)
{
  return (uint32_t)(((ADCxy_COMMON->CDR) & (ConversionData))

                    >> ((__clz(__rbit(ConversionData))) & 0x1FUL)
                   );
}




 



 


















 
static __inline void LL_ADC_INJ_StartConversion(ADC_TypeDef *ADCx)
{
   
   
   
  (((ADCx->CR)) = ((((((ADCx->CR))) & (~(((0x1UL << (31U)) | (0x1UL << (5U)) | (0x1UL << (4U)) | (0x1UL << (3U)) | (0x1UL << (2U)) | (0x1UL << (1U)) | (0x1UL << (0U)))))) | ((0x1UL << (3U))))));


}










 
static __inline void LL_ADC_INJ_StopConversion(ADC_TypeDef *ADCx)
{
   
   
   
  (((ADCx->CR)) = ((((((ADCx->CR))) & (~(((0x1UL << (31U)) | (0x1UL << (5U)) | (0x1UL << (4U)) | (0x1UL << (3U)) | (0x1UL << (2U)) | (0x1UL << (1U)) | (0x1UL << (0U)))))) | ((0x1UL << (5U))))));


}






 
static __inline uint32_t LL_ADC_INJ_IsConversionOngoing(ADC_TypeDef *ADCx)
{
  return ((((ADCx->CR) & ((0x1UL << (3U)))) == ((0x1UL << (3U)))) ? 1UL : 0UL);
}






 
static __inline uint32_t LL_ADC_INJ_IsStopConversionOngoing(ADC_TypeDef *ADCx)
{
  return ((((ADCx->CR) & ((0x1UL << (5U)))) == ((0x1UL << (5U)))) ? 1UL : 0UL);
}

















 
static __inline uint32_t LL_ADC_INJ_ReadConversionData32(ADC_TypeDef *ADCx, uint32_t Rank)
{
  register const uint32_t *preg = ((uint32_t *)((uint32_t) ((uint32_t)(&(ADCx->JDR1)) + ((((Rank & ((0x00000000UL) | (0x00000100UL) | (0x00000200UL) | (0x00000300UL))) >> (8UL))) << 2UL))));
  
  return (uint32_t)(((*preg) & ((0xFFFFUL << (0U))))

                   );
}


















 
static __inline uint16_t LL_ADC_INJ_ReadConversionData12(ADC_TypeDef *ADCx, uint32_t Rank)
{
  register const uint32_t *preg = ((uint32_t *)((uint32_t) ((uint32_t)(&(ADCx->JDR1)) + ((((Rank & ((0x00000000UL) | (0x00000100UL) | (0x00000200UL) | (0x00000300UL))) >> (8UL))) << 2UL))));
  
  return (uint16_t)(((*preg) & ((0xFFFFUL << (0U))))

                   );
}


















 
static __inline uint16_t LL_ADC_INJ_ReadConversionData10(ADC_TypeDef *ADCx, uint32_t Rank)
{
  register const uint32_t *preg = ((uint32_t *)((uint32_t) ((uint32_t)(&(ADCx->JDR1)) + ((((Rank & ((0x00000000UL) | (0x00000100UL) | (0x00000200UL) | (0x00000300UL))) >> (8UL))) << 2UL))));
  
  return (uint16_t)(((*preg) & ((0xFFFFUL << (0U))))

                   );
}


















 
static __inline uint8_t LL_ADC_INJ_ReadConversionData8(ADC_TypeDef *ADCx, uint32_t Rank)
{
  register const uint32_t *preg = ((uint32_t *)((uint32_t) ((uint32_t)(&(ADCx->JDR1)) + ((((Rank & ((0x00000000UL) | (0x00000100UL) | (0x00000200UL) | (0x00000300UL))) >> (8UL))) << 2UL))));
  
  return (uint8_t)(((*preg) & ((0xFFFFUL << (0U))))

                  );
}


















 
static __inline uint8_t LL_ADC_INJ_ReadConversionData6(ADC_TypeDef *ADCx, uint32_t Rank)
{
  register const uint32_t *preg = ((uint32_t *)((uint32_t) ((uint32_t)(&(ADCx->JDR1)) + ((((Rank & ((0x00000000UL) | (0x00000100UL) | (0x00000200UL) | (0x00000300UL))) >> (8UL))) << 2UL))));
  
  return (uint8_t)(((*preg) & ((0xFFFFUL << (0U))))

                  );
}



 



 









 
static __inline uint32_t LL_ADC_IsActiveFlag_ADRDY(ADC_TypeDef *ADCx)
{
  return ((((ADCx->ISR) & ((0x1UL << (0U)))) == ((0x1UL << (0U)))) ? 1UL : 0UL);
}






 
static __inline uint32_t LL_ADC_IsActiveFlag_EOC(ADC_TypeDef *ADCx)
{
  return ((((ADCx->ISR) & ((0x1UL << (2U)))) == ((0x1UL << (2U)))) ? 1UL : 0UL);
}






 
static __inline uint32_t LL_ADC_IsActiveFlag_EOS(ADC_TypeDef *ADCx)
{
  return ((((ADCx->ISR) & ((0x1UL << (3U)))) == ((0x1UL << (3U)))) ? 1UL : 0UL);
}






 
static __inline uint32_t LL_ADC_IsActiveFlag_OVR(ADC_TypeDef *ADCx)
{
  return ((((ADCx->ISR) & ((0x1UL << (4U)))) == ((0x1UL << (4U)))) ? 1UL : 0UL);
}






 
static __inline uint32_t LL_ADC_IsActiveFlag_EOSMP(ADC_TypeDef *ADCx)
{
  return ((((ADCx->ISR) & ((0x1UL << (1U)))) == ((0x1UL << (1U)))) ? 1UL : 0UL);
}






 
static __inline uint32_t LL_ADC_IsActiveFlag_JEOC(ADC_TypeDef *ADCx)
{
  return ((((ADCx->ISR) & ((0x1UL << (5U)))) == ((0x1UL << (5U)))) ? 1UL : 0UL);
}






 
static __inline uint32_t LL_ADC_IsActiveFlag_JEOS(ADC_TypeDef *ADCx)
{
  return ((((ADCx->ISR) & ((0x1UL << (6U)))) == ((0x1UL << (6U)))) ? 1UL : 0UL);
}






 
static __inline uint32_t LL_ADC_IsActiveFlag_JQOVF(ADC_TypeDef *ADCx)
{
  return ((((ADCx->ISR) & ((0x1UL << (10U)))) == ((0x1UL << (10U)))) ? 1UL : 0UL);
}






 
static __inline uint32_t LL_ADC_IsActiveFlag_AWD1(ADC_TypeDef *ADCx)
{
  return ((((ADCx->ISR) & ((0x1UL << (7U)))) == ((0x1UL << (7U)))) ? 1UL : 0UL);
}






 
static __inline uint32_t LL_ADC_IsActiveFlag_AWD2(ADC_TypeDef *ADCx)
{
  return ((((ADCx->ISR) & ((0x1UL << (8U)))) == ((0x1UL << (8U)))) ? 1UL : 0UL);
}






 
static __inline uint32_t LL_ADC_IsActiveFlag_AWD3(ADC_TypeDef *ADCx)
{
  return ((((ADCx->ISR) & ((0x1UL << (9U)))) == ((0x1UL << (9U)))) ? 1UL : 0UL);
}









 
static __inline void LL_ADC_ClearFlag_ADRDY(ADC_TypeDef *ADCx)
{
  ((ADCx->ISR) = ((0x1UL << (0U))));
}






 
static __inline void LL_ADC_ClearFlag_EOC(ADC_TypeDef *ADCx)
{
  ((ADCx->ISR) = ((0x1UL << (2U))));
}






 
static __inline void LL_ADC_ClearFlag_EOS(ADC_TypeDef *ADCx)
{
  ((ADCx->ISR) = ((0x1UL << (3U))));
}






 
static __inline void LL_ADC_ClearFlag_OVR(ADC_TypeDef *ADCx)
{
  ((ADCx->ISR) = ((0x1UL << (4U))));
}






 
static __inline void LL_ADC_ClearFlag_EOSMP(ADC_TypeDef *ADCx)
{
  ((ADCx->ISR) = ((0x1UL << (1U))));
}






 
static __inline void LL_ADC_ClearFlag_JEOC(ADC_TypeDef *ADCx)
{
  ((ADCx->ISR) = ((0x1UL << (5U))));
}






 
static __inline void LL_ADC_ClearFlag_JEOS(ADC_TypeDef *ADCx)
{
  ((ADCx->ISR) = ((0x1UL << (6U))));
}






 
static __inline void LL_ADC_ClearFlag_JQOVF(ADC_TypeDef *ADCx)
{
  ((ADCx->ISR) = ((0x1UL << (10U))));
}






 
static __inline void LL_ADC_ClearFlag_AWD1(ADC_TypeDef *ADCx)
{
  ((ADCx->ISR) = ((0x1UL << (7U))));
}






 
static __inline void LL_ADC_ClearFlag_AWD2(ADC_TypeDef *ADCx)
{
  ((ADCx->ISR) = ((0x1UL << (8U))));
}






 
static __inline void LL_ADC_ClearFlag_AWD3(ADC_TypeDef *ADCx)
{
  ((ADCx->ISR) = ((0x1UL << (9U))));
}








 
static __inline uint32_t LL_ADC_IsActiveFlag_MST_ADRDY(ADC_Common_TypeDef *ADCxy_COMMON)
{
  return ((((ADCxy_COMMON->CSR) & ((0x1UL << (0U)))) == ((0x1UL << (0U)))) ? 1UL : 0UL);
}







 
static __inline uint32_t LL_ADC_IsActiveFlag_SLV_ADRDY(ADC_Common_TypeDef *ADCxy_COMMON)
{
  return ((((ADCxy_COMMON->CSR) & ((0x1UL << (16U)))) == ((0x1UL << (16U)))) ? 1UL : 0UL);
}







 
static __inline uint32_t LL_ADC_IsActiveFlag_MST_EOC(ADC_Common_TypeDef *ADCxy_COMMON)
{
  return ((((ADCxy_COMMON->CSR) & ((0x1UL << (18U)))) == ((0x1UL << (18U)))) ? 1UL : 0UL);
}







 
static __inline uint32_t LL_ADC_IsActiveFlag_SLV_EOC(ADC_Common_TypeDef *ADCxy_COMMON)
{
  return ((((ADCxy_COMMON->CSR) & ((0x1UL << (18U)))) == ((0x1UL << (18U)))) ? 1UL : 0UL);
}







 
static __inline uint32_t LL_ADC_IsActiveFlag_MST_EOS(ADC_Common_TypeDef *ADCxy_COMMON)
{
  return ((((ADCxy_COMMON->CSR) & ((0x1UL << (3U)))) == ((0x1UL << (3U)))) ? 1UL : 0UL);
}







 
static __inline uint32_t LL_ADC_IsActiveFlag_SLV_EOS(ADC_Common_TypeDef *ADCxy_COMMON)
{
  return ((((ADCxy_COMMON->CSR) & ((0x1UL << (19U)))) == ((0x1UL << (19U)))) ? 1UL : 0UL);
}







 
static __inline uint32_t LL_ADC_IsActiveFlag_MST_OVR(ADC_Common_TypeDef *ADCxy_COMMON)
{
  return ((((ADCxy_COMMON->CSR) & ((0x1UL << (4U)))) == ((0x1UL << (4U)))) ? 1UL : 0UL);
}







 
static __inline uint32_t LL_ADC_IsActiveFlag_SLV_OVR(ADC_Common_TypeDef *ADCxy_COMMON)
{
  return ((((ADCxy_COMMON->CSR) & ((0x1UL << (20U)))) == ((0x1UL << (20U)))) ? 1UL : 0UL);
}







 
static __inline uint32_t LL_ADC_IsActiveFlag_MST_EOSMP(ADC_Common_TypeDef *ADCxy_COMMON)
{
  return ((((ADCxy_COMMON->CSR) & ((0x1UL << (1U)))) == ((0x1UL << (1U)))) ? 1UL : 0UL);
}







 
static __inline uint32_t LL_ADC_IsActiveFlag_SLV_EOSMP(ADC_Common_TypeDef *ADCxy_COMMON)
{
  return ((((ADCxy_COMMON->CSR) & ((0x1UL << (17U)))) == ((0x1UL << (17U)))) ? 1UL : 0UL);
}







 
static __inline uint32_t LL_ADC_IsActiveFlag_MST_JEOC(ADC_Common_TypeDef *ADCxy_COMMON)
{
  return ((((ADCxy_COMMON->CSR) & ((0x1UL << (5U)))) == ((0x1UL << (5U)))) ? 1UL : 0UL);
}







 
static __inline uint32_t LL_ADC_IsActiveFlag_SLV_JEOC(ADC_Common_TypeDef *ADCxy_COMMON)
{
  return ((((ADCxy_COMMON->CSR) & ((0x1UL << (21U)))) == ((0x1UL << (21U)))) ? 1UL : 0UL);
}







 
static __inline uint32_t LL_ADC_IsActiveFlag_MST_JEOS(ADC_Common_TypeDef *ADCxy_COMMON)
{
  return ((((ADCxy_COMMON->CSR) & ((0x1UL << (6U)))) == ((0x1UL << (6U)))) ? 1UL : 0UL);
}







 
static __inline uint32_t LL_ADC_IsActiveFlag_SLV_JEOS(ADC_Common_TypeDef *ADCxy_COMMON)
{
  return ((((ADCxy_COMMON->CSR) & ((0x1UL << (22U)))) == ((0x1UL << (22U)))) ? 1UL : 0UL);
}







 
static __inline uint32_t LL_ADC_IsActiveFlag_MST_JQOVF(ADC_Common_TypeDef *ADCxy_COMMON)
{
  return ((((ADCxy_COMMON->CSR) & ((0x1UL << (10U)))) == ((0x1UL << (10U)))) ? 1UL : 0UL);
}







 
static __inline uint32_t LL_ADC_IsActiveFlag_SLV_JQOVF(ADC_Common_TypeDef *ADCxy_COMMON)
{
  return ((((ADCxy_COMMON->CSR) & ((0x1UL << (26U)))) == ((0x1UL << (26U)))) ? 1UL : 0UL);
}







 
static __inline uint32_t LL_ADC_IsActiveFlag_MST_AWD1(ADC_Common_TypeDef *ADCxy_COMMON)
{
  return ((((ADCxy_COMMON->CSR) & ((0x1UL << (7U)))) == ((0x1UL << (7U)))) ? 1UL : 0UL);
}







 
static __inline uint32_t LL_ADC_IsActiveFlag_SLV_AWD1(ADC_Common_TypeDef *ADCxy_COMMON)
{
  return ((((ADCxy_COMMON->CSR) & ((0x1UL << (23U)))) == ((0x1UL << (23U)))) ? 1UL : 0UL);
}







 
static __inline uint32_t LL_ADC_IsActiveFlag_MST_AWD2(ADC_Common_TypeDef *ADCxy_COMMON)
{
  return ((((ADCxy_COMMON->CSR) & ((0x1UL << (8U)))) == ((0x1UL << (8U)))) ? 1UL : 0UL);
}







 
static __inline uint32_t LL_ADC_IsActiveFlag_SLV_AWD2(ADC_Common_TypeDef *ADCxy_COMMON)
{
  return ((((ADCxy_COMMON->CSR) & ((0x1UL << (24U)))) == ((0x1UL << (24U)))) ? 1UL : 0UL);
}







 
static __inline uint32_t LL_ADC_IsActiveFlag_MST_AWD3(ADC_Common_TypeDef *ADCxy_COMMON)
{
  return ((((ADCxy_COMMON->CSR) & ((0x1UL << (9U)))) == ((0x1UL << (9U)))) ? 1UL : 0UL);
}







 
static __inline uint32_t LL_ADC_IsActiveFlag_SLV_AWD3(ADC_Common_TypeDef *ADCxy_COMMON)
{
  return ((((ADCxy_COMMON->CSR) & ((0x1UL << (25U)))) == ((0x1UL << (25U)))) ? 1UL : 0UL);
}




 



 






 
static __inline void LL_ADC_EnableIT_ADRDY(ADC_TypeDef *ADCx)
{
  ((ADCx->IER) |= ((0x1UL << (0U))));
}






 
static __inline void LL_ADC_EnableIT_EOC(ADC_TypeDef *ADCx)
{
  ((ADCx->IER) |= ((0x1UL << (2U))));
}






 
static __inline void LL_ADC_EnableIT_EOS(ADC_TypeDef *ADCx)
{
  ((ADCx->IER) |= ((0x1UL << (3U))));
}






 
static __inline void LL_ADC_EnableIT_OVR(ADC_TypeDef *ADCx)
{
  ((ADCx->IER) |= ((0x1UL << (4U))));
}






 
static __inline void LL_ADC_EnableIT_EOSMP(ADC_TypeDef *ADCx)
{
  ((ADCx->IER) |= ((0x1UL << (1U))));
}






 
static __inline void LL_ADC_EnableIT_JEOC(ADC_TypeDef *ADCx)
{
  ((ADCx->IER) |= ((0x1UL << (5U))));
}






 
static __inline void LL_ADC_EnableIT_JEOS(ADC_TypeDef *ADCx)
{
  ((ADCx->IER) |= ((0x1UL << (6U))));
}






 
static __inline void LL_ADC_EnableIT_JQOVF(ADC_TypeDef *ADCx)
{
  ((ADCx->IER) |= ((0x1UL << (10U))));
}






 
static __inline void LL_ADC_EnableIT_AWD1(ADC_TypeDef *ADCx)
{
  ((ADCx->IER) |= ((0x1UL << (7U))));
}






 
static __inline void LL_ADC_EnableIT_AWD2(ADC_TypeDef *ADCx)
{
  ((ADCx->IER) |= ((0x1UL << (8U))));
}






 
static __inline void LL_ADC_EnableIT_AWD3(ADC_TypeDef *ADCx)
{
  ((ADCx->IER) |= ((0x1UL << (9U))));
}






 
static __inline void LL_ADC_DisableIT_ADRDY(ADC_TypeDef *ADCx)
{
  ((ADCx->IER) &= ~((0x1UL << (0U))));
}






 
static __inline void LL_ADC_DisableIT_EOC(ADC_TypeDef *ADCx)
{
  ((ADCx->IER) &= ~((0x1UL << (2U))));
}






 
static __inline void LL_ADC_DisableIT_EOS(ADC_TypeDef *ADCx)
{
  ((ADCx->IER) &= ~((0x1UL << (3U))));
}






 
static __inline void LL_ADC_DisableIT_OVR(ADC_TypeDef *ADCx)
{
  ((ADCx->IER) &= ~((0x1UL << (4U))));
}






 
static __inline void LL_ADC_DisableIT_EOSMP(ADC_TypeDef *ADCx)
{
  ((ADCx->IER) &= ~((0x1UL << (1U))));
}






 
static __inline void LL_ADC_DisableIT_JEOC(ADC_TypeDef *ADCx)
{
  ((ADCx->IER) &= ~((0x1UL << (5U))));
}






 
static __inline void LL_ADC_DisableIT_JEOS(ADC_TypeDef *ADCx)
{
  ((ADCx->IER) &= ~((0x1UL << (6U))));
}






 
static __inline void LL_ADC_DisableIT_JQOVF(ADC_TypeDef *ADCx)
{
  ((ADCx->IER) &= ~((0x1UL << (10U))));
}






 
static __inline void LL_ADC_DisableIT_AWD1(ADC_TypeDef *ADCx)
{
  ((ADCx->IER) &= ~((0x1UL << (7U))));
}






 
static __inline void LL_ADC_DisableIT_AWD2(ADC_TypeDef *ADCx)
{
  ((ADCx->IER) &= ~((0x1UL << (8U))));
}






 
static __inline void LL_ADC_DisableIT_AWD3(ADC_TypeDef *ADCx)
{
  ((ADCx->IER) &= ~((0x1UL << (9U))));
}







 
static __inline uint32_t LL_ADC_IsEnabledIT_ADRDY(ADC_TypeDef *ADCx)
{
  return ((((ADCx->IER) & ((0x1UL << (0U)))) == ((0x1UL << (0U)))) ? 1UL : 0UL);
}







 
static __inline uint32_t LL_ADC_IsEnabledIT_EOC(ADC_TypeDef *ADCx)
{
  return ((((ADCx->IER) & ((0x1UL << (2U)))) == ((0x1UL << (2U)))) ? 1UL : 0UL);
}







 
static __inline uint32_t LL_ADC_IsEnabledIT_EOS(ADC_TypeDef *ADCx)
{
  return ((((ADCx->IER) & ((0x1UL << (3U)))) == ((0x1UL << (3U)))) ? 1UL : 0UL);
}







 
static __inline uint32_t LL_ADC_IsEnabledIT_OVR(ADC_TypeDef *ADCx)
{
  return ((((ADCx->IER) & ((0x1UL << (4U)))) == ((0x1UL << (4U)))) ? 1UL : 0UL);
}







 
static __inline uint32_t LL_ADC_IsEnabledIT_EOSMP(ADC_TypeDef *ADCx)
{
  return ((((ADCx->IER) & ((0x1UL << (1U)))) == ((0x1UL << (1U)))) ? 1UL : 0UL);
}







 
static __inline uint32_t LL_ADC_IsEnabledIT_JEOC(ADC_TypeDef *ADCx)
{
  return ((((ADCx->IER) & ((0x1UL << (5U)))) == ((0x1UL << (5U)))) ? 1UL : 0UL);
}







 
static __inline uint32_t LL_ADC_IsEnabledIT_JEOS(ADC_TypeDef *ADCx)
{
  return ((((ADCx->IER) & ((0x1UL << (6U)))) == ((0x1UL << (6U)))) ? 1UL : 0UL);
}







 
static __inline uint32_t LL_ADC_IsEnabledIT_JQOVF(ADC_TypeDef *ADCx)
{
  return ((((ADCx->IER) & ((0x1UL << (10U)))) == ((0x1UL << (10U)))) ? 1UL : 0UL);
}







 
static __inline uint32_t LL_ADC_IsEnabledIT_AWD1(ADC_TypeDef *ADCx)
{
  return ((((ADCx->IER) & ((0x1UL << (7U)))) == ((0x1UL << (7U)))) ? 1UL : 0UL);
}







 
static __inline uint32_t LL_ADC_IsEnabledIT_AWD2(ADC_TypeDef *ADCx)
{
  return ((((ADCx->IER) & ((0x1UL << (8U)))) == ((0x1UL << (8U)))) ? 1UL : 0UL);
}







 
static __inline uint32_t LL_ADC_IsEnabledIT_AWD3(ADC_TypeDef *ADCx)
{
  return ((((ADCx->IER) & ((0x1UL << (9U)))) == ((0x1UL << (9U)))) ? 1UL : 0UL);
}



 

#line 7381 "..\\HALLIB\\STM32L4xx_HAL_Driver\\Inc\\stm32l4xx_ll_adc.h"



 



 





 







 
#line 49 "..\\HALLIB\\STM32L4xx_HAL_Driver\\Inc\\stm32l4xx_hal_adc.h"



 



 

 


 



 
typedef struct
{
  uint32_t Ratio;                         
 

  uint32_t RightBitShift;                 
 

  uint32_t TriggeredMode;                 
 

  uint32_t OversamplingStopReset;         





                                                

}ADC_OversamplingTypeDef;
















 
typedef struct
{
  uint32_t ClockPrescaler;        








 

  uint32_t Resolution;            
 

  uint32_t DataAlign;             

 

  uint32_t ScanConvMode;          





 

  uint32_t EOCSelection;          
 

  FunctionalState LowPowerAutoWait; 









 

  FunctionalState ContinuousConvMode; 

 

  uint32_t NbrOfConversion;       



 

  FunctionalState DiscontinuousConvMode; 



 

  uint32_t NbrOfDiscConversion;   

 

  uint32_t ExternalTrigConv;      


 
                                                                                                        
  uint32_t ExternalTrigConvEdge;  

 

  FunctionalState DMAContinuousRequests; 


 

  uint32_t Overrun;               








 

  FunctionalState OversamplingMode;       

 

  ADC_OversamplingTypeDef Oversampling;   
 

#line 204 "..\\HALLIB\\STM32L4xx_HAL_Driver\\Inc\\stm32l4xx_hal_adc.h"
}ADC_InitTypeDef;











 
typedef struct
{
  uint32_t Channel;                

 

  uint32_t Rank;                   


 

  uint32_t SamplingTime;           








 

  uint32_t SingleDiff;             









 

  uint32_t OffsetNumber;           

 

  uint32_t Offset;                 




 

}ADC_ChannelConfTypeDef;






 
typedef struct
{
  uint32_t WatchdogNumber;    


 

  uint32_t WatchdogMode;      


 

  uint32_t Channel;           


 

  FunctionalState ITMode;     
 

  uint32_t HighThreshold;     







 

  uint32_t LowThreshold;      







 
}ADC_AnalogWDGConfTypeDef;




 
typedef struct
{
  uint32_t ContextQueue;                 

 
                                               
  uint32_t ChannelCount;                  
}ADC_InjectionConfigTypeDef;  



 








 
 





 




 






 





 




 




 



 
typedef struct __ADC_HandleTypeDef
{
  ADC_TypeDef                   *Instance;               
  ADC_InitTypeDef               Init;                    
  DMA_HandleTypeDef             *DMA_Handle;             
  HAL_LockTypeDef               Lock;                    
  volatile uint32_t                 State;                   
  volatile uint32_t                 ErrorCode;               
  ADC_InjectionConfigTypeDef    InjectionConfig ;        
#line 397 "..\\HALLIB\\STM32L4xx_HAL_Driver\\Inc\\stm32l4xx_hal_adc.h"
}ADC_HandleTypeDef;

#line 424 "..\\HALLIB\\STM32L4xx_HAL_Driver\\Inc\\stm32l4xx_hal_adc.h"



 


 



 



 
#line 448 "..\\HALLIB\\STM32L4xx_HAL_Driver\\Inc\\stm32l4xx_hal_adc.h"


 



 




#line 471 "..\\HALLIB\\STM32L4xx_HAL_Driver\\Inc\\stm32l4xx_hal_adc.h"


 



 






 



 




 



 




 



 
 
#line 525 "..\\HALLIB\\STM32L4xx_HAL_Driver\\Inc\\stm32l4xx_hal_adc.h"


 



 






 



 




 



 




 



 
#line 577 "..\\HALLIB\\STM32L4xx_HAL_Driver\\Inc\\stm32l4xx_hal_adc.h"


 



 
#line 595 "..\\HALLIB\\STM32L4xx_HAL_Driver\\Inc\\stm32l4xx_hal_adc.h"


 



 
 
 
#line 637 "..\\HALLIB\\STM32L4xx_HAL_Driver\\Inc\\stm32l4xx_hal_adc.h"


 



 





 



 
#line 661 "..\\HALLIB\\STM32L4xx_HAL_Driver\\Inc\\stm32l4xx_hal_adc.h"


 



 
#line 676 "..\\HALLIB\\STM32L4xx_HAL_Driver\\Inc\\stm32l4xx_hal_adc.h"


 



 
#line 692 "..\\HALLIB\\STM32L4xx_HAL_Driver\\Inc\\stm32l4xx_hal_adc.h"


 



 




 



 




 




 
#line 724 "..\\HALLIB\\STM32L4xx_HAL_Driver\\Inc\\stm32l4xx_hal_adc.h"


 




 
#line 743 "..\\HALLIB\\STM32L4xx_HAL_Driver\\Inc\\stm32l4xx_hal_adc.h"





 



 
#line 764 "..\\HALLIB\\STM32L4xx_HAL_Driver\\Inc\\stm32l4xx_hal_adc.h"







 






 



 

 



 
 
 





 







 






 









 









 







 







 







 







 
#line 879 "..\\HALLIB\\STM32L4xx_HAL_Driver\\Inc\\stm32l4xx_hal_adc.h"





 




                             




  







 







 







 










 
#line 947 "..\\HALLIB\\STM32L4xx_HAL_Driver\\Inc\\stm32l4xx_hal_adc.h"





 







 







 
#line 989 "..\\HALLIB\\STM32L4xx_HAL_Driver\\Inc\\stm32l4xx_hal_adc.h"





 
#line 1011 "..\\HALLIB\\STM32L4xx_HAL_Driver\\Inc\\stm32l4xx_hal_adc.h"



 


 



 

 
 
 
 
 
 
 


 
 
 




 

 



 
 
 



 




 
#line 1068 "..\\HALLIB\\STM32L4xx_HAL_Driver\\Inc\\stm32l4xx_hal_adc.h"


















 




















 



















 


    

















 




















 
 





 



 

















































 



















































 




























































 










































































 




































 
















 













 



















 















 




















 
#line 1550 "..\\HALLIB\\STM32L4xx_HAL_Driver\\Inc\\stm32l4xx_hal_adc.h"
















 
#line 1573 "..\\HALLIB\\STM32L4xx_HAL_Driver\\Inc\\stm32l4xx_hal_adc.h"

























 

















































 
#line 1655 "..\\HALLIB\\STM32L4xx_HAL_Driver\\Inc\\stm32l4xx_hal_adc.h"












































 
#line 1712 "..\\HALLIB\\STM32L4xx_HAL_Driver\\Inc\\stm32l4xx_hal_adc.h"



 



 

 
#line 1 "..\\HALLIB\\STM32L4xx_HAL_Driver\\Inc\\stm32l4xx_hal_adc_ex.h"

































 

 







 
#line 46 "..\\HALLIB\\STM32L4xx_HAL_Driver\\Inc\\stm32l4xx_hal_adc_ex.h"



 



  

 


 



 
typedef struct
{
  uint32_t Ratio;                         
 
                                               
  uint32_t RightBitShift;                 
 
}ADC_InjOversamplingTypeDef;  
















 
typedef struct 
{
  uint32_t InjectedChannel;               

 

  uint32_t InjectedRank;                  


 

  uint32_t InjectedSamplingTime;          








 

  uint32_t InjectedSingleDiff;            









 

  uint32_t InjectedOffsetNumber;          

 

  uint32_t InjectedOffset;                




 

  uint32_t InjectedNbrOfConversion;       



 

  FunctionalState InjectedDiscontinuousConvMode; 







 

  FunctionalState AutoInjectedConv;       






 

  FunctionalState QueueInjectedContext;   








 

  uint32_t ExternalTrigInjecConv;         



 

  uint32_t ExternalTrigInjecConvEdge;     



 

  FunctionalState InjecOversamplingMode;         

 

  ADC_InjOversamplingTypeDef  InjecOversampling; 

 
}ADC_InjectionConfTypeDef;






 
typedef struct
{
  uint32_t Mode;              
 

  uint32_t DMAAccessMode;     

 

  uint32_t TwoSamplingDelay;  



 
}ADC_MultiModeTypeDef;




 

 



 



 
 
#line 244 "..\\HALLIB\\STM32L4xx_HAL_Driver\\Inc\\stm32l4xx_hal_adc_ex.h"


 



 






 



 




 



 







 



 






 




 
#line 303 "..\\HALLIB\\STM32L4xx_HAL_Driver\\Inc\\stm32l4xx_hal_adc_ex.h"



 





 



 
#line 329 "..\\HALLIB\\STM32L4xx_HAL_Driver\\Inc\\stm32l4xx_hal_adc_ex.h"


 



 




 





 



 
#line 366 "..\\HALLIB\\STM32L4xx_HAL_Driver\\Inc\\stm32l4xx_hal_adc_ex.h"


 
  


 
#line 384 "..\\HALLIB\\STM32L4xx_HAL_Driver\\Inc\\stm32l4xx_hal_adc_ex.h"


 



 

 







 

#line 412 "..\\HALLIB\\STM32L4xx_HAL_Driver\\Inc\\stm32l4xx_hal_adc_ex.h"



 

 




 














 





 


 



 
 
 






 







 








 








 
#line 499 "..\\HALLIB\\STM32L4xx_HAL_Driver\\Inc\\stm32l4xx_hal_adc_ex.h"






 






 






 






 






 






 






 






 






 






 






 






 






 







 














 














 













 










 
#line 671 "..\\HALLIB\\STM32L4xx_HAL_Driver\\Inc\\stm32l4xx_hal_adc_ex.h"








 



 





 




 







 




 







 
 






 






 








 
#line 863 "..\\HALLIB\\STM32L4xx_HAL_Driver\\Inc\\stm32l4xx_hal_adc_ex.h"






 
#line 887 "..\\HALLIB\\STM32L4xx_HAL_Driver\\Inc\\stm32l4xx_hal_adc_ex.h"
    


 
#line 920 "..\\HALLIB\\STM32L4xx_HAL_Driver\\Inc\\stm32l4xx_hal_adc_ex.h"





 







 










 










 
#line 973 "..\\HALLIB\\STM32L4xx_HAL_Driver\\Inc\\stm32l4xx_hal_adc_ex.h"





  










  
#line 998 "..\\HALLIB\\STM32L4xx_HAL_Driver\\Inc\\stm32l4xx_hal_adc_ex.h"





 








 
#line 1026 "..\\HALLIB\\STM32L4xx_HAL_Driver\\Inc\\stm32l4xx_hal_adc_ex.h"





 








 
#line 1048 "..\\HALLIB\\STM32L4xx_HAL_Driver\\Inc\\stm32l4xx_hal_adc_ex.h"





 








 
#line 1069 "..\\HALLIB\\STM32L4xx_HAL_Driver\\Inc\\stm32l4xx_hal_adc_ex.h"





 
#line 1083 "..\\HALLIB\\STM32L4xx_HAL_Driver\\Inc\\stm32l4xx_hal_adc_ex.h"





 
#line 1098 "..\\HALLIB\\STM32L4xx_HAL_Driver\\Inc\\stm32l4xx_hal_adc_ex.h"





 







 










 
#line 1129 "..\\HALLIB\\STM32L4xx_HAL_Driver\\Inc\\stm32l4xx_hal_adc_ex.h"








 








 


 


 



 
 

 
HAL_StatusTypeDef       HAL_ADCEx_Calibration_Start(ADC_HandleTypeDef* hadc, uint32_t SingleDiff);
uint32_t                HAL_ADCEx_Calibration_GetValue(ADC_HandleTypeDef *hadc, uint32_t SingleDiff);
HAL_StatusTypeDef       HAL_ADCEx_Calibration_SetValue(ADC_HandleTypeDef *hadc, uint32_t SingleDiff, uint32_t CalibrationFactor);

 
HAL_StatusTypeDef       HAL_ADCEx_InjectedStart(ADC_HandleTypeDef* hadc);
HAL_StatusTypeDef       HAL_ADCEx_InjectedStop(ADC_HandleTypeDef* hadc);
HAL_StatusTypeDef       HAL_ADCEx_InjectedPollForConversion(ADC_HandleTypeDef* hadc, uint32_t Timeout);

 
HAL_StatusTypeDef       HAL_ADCEx_InjectedStart_IT(ADC_HandleTypeDef* hadc);
HAL_StatusTypeDef       HAL_ADCEx_InjectedStop_IT(ADC_HandleTypeDef* hadc);


 
HAL_StatusTypeDef       HAL_ADCEx_MultiModeStart_DMA(ADC_HandleTypeDef *hadc, uint32_t *pData, uint32_t Length);
HAL_StatusTypeDef       HAL_ADCEx_MultiModeStop_DMA(ADC_HandleTypeDef *hadc); 
uint32_t                HAL_ADCEx_MultiModeGetValue(ADC_HandleTypeDef *hadc);


 
uint32_t                HAL_ADCEx_InjectedGetValue(ADC_HandleTypeDef* hadc, uint32_t InjectedRank);

 
void                    HAL_ADCEx_InjectedConvCpltCallback(ADC_HandleTypeDef* hadc);
void                    HAL_ADCEx_InjectedQueueOverflowCallback(ADC_HandleTypeDef* hadc);
void                    HAL_ADCEx_LevelOutOfWindow2Callback(ADC_HandleTypeDef* hadc);
void                    HAL_ADCEx_LevelOutOfWindow3Callback(ADC_HandleTypeDef* hadc);
void                    HAL_ADCEx_EndOfSamplingCallback(ADC_HandleTypeDef* hadc);

 
HAL_StatusTypeDef HAL_ADCEx_RegularStop(ADC_HandleTypeDef* hadc);
HAL_StatusTypeDef HAL_ADCEx_RegularStop_IT(ADC_HandleTypeDef* hadc);
HAL_StatusTypeDef HAL_ADCEx_RegularStop_DMA(ADC_HandleTypeDef* hadc);

HAL_StatusTypeDef HAL_ADCEx_RegularMultiModeStop_DMA(ADC_HandleTypeDef* hadc);




 



 
 
HAL_StatusTypeDef       HAL_ADCEx_InjectedConfigChannel(ADC_HandleTypeDef* hadc,ADC_InjectionConfTypeDef* sConfigInjected);

HAL_StatusTypeDef       HAL_ADCEx_MultiModeConfigChannel(ADC_HandleTypeDef *hadc, ADC_MultiModeTypeDef *multimode);

HAL_StatusTypeDef       HAL_ADCEx_EnableInjectedQueue(ADC_HandleTypeDef* hadc);
HAL_StatusTypeDef       HAL_ADCEx_DisableInjectedQueue(ADC_HandleTypeDef* hadc);
HAL_StatusTypeDef       HAL_ADCEx_DisableVoltageRegulator(ADC_HandleTypeDef* hadc);
HAL_StatusTypeDef       HAL_ADCEx_EnterADCDeepPowerDownMode(ADC_HandleTypeDef* hadc);



 



 



 



 








 
#line 1723 "..\\HALLIB\\STM32L4xx_HAL_Driver\\Inc\\stm32l4xx_hal_adc.h"

 


 




 
 
HAL_StatusTypeDef       HAL_ADC_Init(ADC_HandleTypeDef* hadc);
HAL_StatusTypeDef       HAL_ADC_DeInit(ADC_HandleTypeDef *hadc);
void                    HAL_ADC_MspInit(ADC_HandleTypeDef* hadc);
void                    HAL_ADC_MspDeInit(ADC_HandleTypeDef* hadc);








 




 
 

 
HAL_StatusTypeDef       HAL_ADC_Start(ADC_HandleTypeDef* hadc);
HAL_StatusTypeDef       HAL_ADC_Stop(ADC_HandleTypeDef* hadc);
HAL_StatusTypeDef       HAL_ADC_PollForConversion(ADC_HandleTypeDef* hadc, uint32_t Timeout);
HAL_StatusTypeDef       HAL_ADC_PollForEvent(ADC_HandleTypeDef* hadc, uint32_t EventType, uint32_t Timeout);

 
HAL_StatusTypeDef       HAL_ADC_Start_IT(ADC_HandleTypeDef* hadc);
HAL_StatusTypeDef       HAL_ADC_Stop_IT(ADC_HandleTypeDef* hadc);

 
HAL_StatusTypeDef       HAL_ADC_Start_DMA(ADC_HandleTypeDef* hadc, uint32_t* pData, uint32_t Length);
HAL_StatusTypeDef       HAL_ADC_Stop_DMA(ADC_HandleTypeDef* hadc);

 
uint32_t                HAL_ADC_GetValue(ADC_HandleTypeDef* hadc);

 
void                    HAL_ADC_IRQHandler(ADC_HandleTypeDef* hadc);
void                    HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef* hadc);
void                    HAL_ADC_ConvHalfCpltCallback(ADC_HandleTypeDef* hadc);
void                    HAL_ADC_LevelOutOfWindowCallback(ADC_HandleTypeDef* hadc);
void                    HAL_ADC_ErrorCallback(ADC_HandleTypeDef *hadc);


 




 
 
HAL_StatusTypeDef       HAL_ADC_ConfigChannel(ADC_HandleTypeDef* hadc, ADC_ChannelConfTypeDef* sConfig);
HAL_StatusTypeDef       HAL_ADC_AnalogWDGConfig(ADC_HandleTypeDef* hadc, ADC_AnalogWDGConfTypeDef* AnalogWDGConfig);



 

 


 
uint32_t                HAL_ADC_GetState(ADC_HandleTypeDef* hadc);
uint32_t                HAL_ADC_GetError(ADC_HandleTypeDef *hadc);



 



 

 


 
HAL_StatusTypeDef ADC_ConversionStop(ADC_HandleTypeDef* hadc, uint32_t ConversionGroup);
HAL_StatusTypeDef ADC_Enable(ADC_HandleTypeDef* hadc);
HAL_StatusTypeDef ADC_Disable(ADC_HandleTypeDef* hadc);
void ADC_DMAConvCplt(DMA_HandleTypeDef *hdma);
void ADC_DMAHalfConvCplt(DMA_HandleTypeDef *hdma);
void ADC_DMAError(DMA_HandleTypeDef *hdma);



 



 



 








 
#line 250 "..\\USER\\stm32l4xx_hal_conf.h"


#line 1 "..\\HALLIB\\STM32L4xx_HAL_Driver\\Inc\\stm32l4xx_hal_can.h"

































 

 







 
#line 46 "..\\HALLIB\\STM32L4xx_HAL_Driver\\Inc\\stm32l4xx_hal_can.h"



 




 

 


 


 
typedef enum
{
  HAL_CAN_STATE_RESET             = 0x00U,   
  HAL_CAN_STATE_READY             = 0x01U,   
  HAL_CAN_STATE_LISTENING         = 0x02U,   
  HAL_CAN_STATE_SLEEP_PENDING     = 0x03U,   
  HAL_CAN_STATE_SLEEP_ACTIVE      = 0x04U,   
  HAL_CAN_STATE_ERROR             = 0x05U    

} HAL_CAN_StateTypeDef;



 
typedef struct
{
  uint32_t Prescaler;                  
 

  uint32_t Mode;                       
 

  uint32_t SyncJumpWidth;              

 

  uint32_t TimeSeg1;                   
 

  uint32_t TimeSeg2;                   
 

  FunctionalState TimeTriggeredMode;   
 

  FunctionalState AutoBusOff;          
 

  FunctionalState AutoWakeUp;          
 

  FunctionalState AutoRetransmission;  
 

  FunctionalState ReceiveFifoLocked;   
 

  FunctionalState TransmitFifoPriority;
 

} CAN_InitTypeDef;



 
typedef struct
{
  uint32_t FilterIdHigh;          

 

  uint32_t FilterIdLow;           

 

  uint32_t FilterMaskIdHigh;      


 

  uint32_t FilterMaskIdLow;       


 

  uint32_t FilterFIFOAssignment;  
 

  uint32_t FilterBank;            



 

  uint32_t FilterMode;            
 

  uint32_t FilterScale;           
 

  uint32_t FilterActivation;      
 

  uint32_t SlaveStartFilterBank;  




 

} CAN_FilterTypeDef;



 
typedef struct
{
  uint32_t StdId;    
 

  uint32_t ExtId;    
 

  uint32_t IDE;      
 

  uint32_t RTR;      
 

  uint32_t DLC;      
 

  FunctionalState TransmitGlobalTime; 



 

} CAN_TxHeaderTypeDef;



 
typedef struct
{
  uint32_t StdId;    
 

  uint32_t ExtId;    
 

  uint32_t IDE;      
 

  uint32_t RTR;      
 

  uint32_t DLC;      
 

  uint32_t Timestamp; 

 

  uint32_t FilterMatchIndex; 
 

} CAN_RxHeaderTypeDef;



 
typedef struct __CAN_HandleTypeDef
{
  CAN_TypeDef                 *Instance;                  

  CAN_InitTypeDef             Init;                       

  volatile HAL_CAN_StateTypeDef   State;                      

  volatile uint32_t               ErrorCode;                 
 

#line 255 "..\\HALLIB\\STM32L4xx_HAL_Driver\\Inc\\stm32l4xx_hal_can.h"
} CAN_HandleTypeDef;

#line 288 "..\\HALLIB\\STM32L4xx_HAL_Driver\\Inc\\stm32l4xx_hal_can.h"


 

 



 



 
#line 324 "..\\HALLIB\\STM32L4xx_HAL_Driver\\Inc\\stm32l4xx_hal_can.h"








 



 




 



 






 




 






 



 
#line 385 "..\\HALLIB\\STM32L4xx_HAL_Driver\\Inc\\stm32l4xx_hal_can.h"


 



 
#line 400 "..\\HALLIB\\STM32L4xx_HAL_Driver\\Inc\\stm32l4xx_hal_can.h"


 



 




 



 




 



 




 



 




 



 




 



 




 



 




 



 





 



 
 
#line 499 "..\\HALLIB\\STM32L4xx_HAL_Driver\\Inc\\stm32l4xx_hal_can.h"

 





 






 





 




 
 


 
#line 535 "..\\HALLIB\\STM32L4xx_HAL_Driver\\Inc\\stm32l4xx_hal_can.h"

 



 







 



 

 


 




 
#line 572 "..\\HALLIB\\STM32L4xx_HAL_Driver\\Inc\\stm32l4xx_hal_can.h"







 








 







 







 
#line 611 "..\\HALLIB\\STM32L4xx_HAL_Driver\\Inc\\stm32l4xx_hal_can.h"
























 








 

 


 




 

 
HAL_StatusTypeDef HAL_CAN_Init(CAN_HandleTypeDef *hcan);
HAL_StatusTypeDef HAL_CAN_DeInit(CAN_HandleTypeDef *hcan);
void HAL_CAN_MspInit(CAN_HandleTypeDef *hcan);
void HAL_CAN_MspDeInit(CAN_HandleTypeDef *hcan);

#line 668 "..\\HALLIB\\STM32L4xx_HAL_Driver\\Inc\\stm32l4xx_hal_can.h"


 




 

 
HAL_StatusTypeDef HAL_CAN_ConfigFilter(CAN_HandleTypeDef *hcan, CAN_FilterTypeDef *sFilterConfig);



 




 

 
HAL_StatusTypeDef HAL_CAN_Start(CAN_HandleTypeDef *hcan);
HAL_StatusTypeDef HAL_CAN_Stop(CAN_HandleTypeDef *hcan);
HAL_StatusTypeDef HAL_CAN_RequestSleep(CAN_HandleTypeDef *hcan);
HAL_StatusTypeDef HAL_CAN_WakeUp(CAN_HandleTypeDef *hcan);
uint32_t HAL_CAN_IsSleepActive(CAN_HandleTypeDef *hcan);
HAL_StatusTypeDef HAL_CAN_AddTxMessage(CAN_HandleTypeDef *hcan, CAN_TxHeaderTypeDef *pHeader, uint8_t aData[], uint32_t *pTxMailbox);
HAL_StatusTypeDef HAL_CAN_AbortTxRequest(CAN_HandleTypeDef *hcan, uint32_t TxMailboxes);
uint32_t HAL_CAN_GetTxMailboxesFreeLevel(CAN_HandleTypeDef *hcan);
uint32_t HAL_CAN_IsTxMessagePending(CAN_HandleTypeDef *hcan, uint32_t TxMailboxes);
uint32_t HAL_CAN_GetTxTimestamp(CAN_HandleTypeDef *hcan, uint32_t TxMailbox);
HAL_StatusTypeDef HAL_CAN_GetRxMessage(CAN_HandleTypeDef *hcan, uint32_t RxFifo, CAN_RxHeaderTypeDef *pHeader, uint8_t aData[]);
uint32_t HAL_CAN_GetRxFifoFillLevel(CAN_HandleTypeDef *hcan, uint32_t RxFifo);



 




 
 
HAL_StatusTypeDef HAL_CAN_ActivateNotification(CAN_HandleTypeDef *hcan, uint32_t ActiveITs);
HAL_StatusTypeDef HAL_CAN_DeactivateNotification(CAN_HandleTypeDef *hcan, uint32_t InactiveITs);
void HAL_CAN_IRQHandler(CAN_HandleTypeDef *hcan);



 




 
 

void HAL_CAN_TxMailbox0CompleteCallback(CAN_HandleTypeDef *hcan);
void HAL_CAN_TxMailbox1CompleteCallback(CAN_HandleTypeDef *hcan);
void HAL_CAN_TxMailbox2CompleteCallback(CAN_HandleTypeDef *hcan);
void HAL_CAN_TxMailbox0AbortCallback(CAN_HandleTypeDef *hcan);
void HAL_CAN_TxMailbox1AbortCallback(CAN_HandleTypeDef *hcan);
void HAL_CAN_TxMailbox2AbortCallback(CAN_HandleTypeDef *hcan);
void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan);
void HAL_CAN_RxFifo0FullCallback(CAN_HandleTypeDef *hcan);
void HAL_CAN_RxFifo1MsgPendingCallback(CAN_HandleTypeDef *hcan);
void HAL_CAN_RxFifo1FullCallback(CAN_HandleTypeDef *hcan);
void HAL_CAN_SleepCallback(CAN_HandleTypeDef *hcan);
void HAL_CAN_WakeUpFromRxMsgCallback(CAN_HandleTypeDef *hcan);
void HAL_CAN_ErrorCallback(CAN_HandleTypeDef *hcan);



 




 
 
HAL_CAN_StateTypeDef HAL_CAN_GetState(CAN_HandleTypeDef *hcan);
uint32_t HAL_CAN_GetError(CAN_HandleTypeDef *hcan);
HAL_StatusTypeDef HAL_CAN_ResetError(CAN_HandleTypeDef *hcan);



 



 

 


 



 

 


 



 

 


 



 

 


 

#line 843 "..\\HALLIB\\STM32L4xx_HAL_Driver\\Inc\\stm32l4xx_hal_can.h"



 
 



 





 








 
#line 254 "..\\USER\\stm32l4xx_hal_conf.h"


#line 1 "..\\HALLIB\\STM32L4xx_HAL_Driver\\Inc\\stm32l4xx_hal_comp.h"

































 

 







 
#line 46 "..\\HALLIB\\STM32L4xx_HAL_Driver\\Inc\\stm32l4xx_hal_comp.h"
#line 1 "..\\HALLIB\\STM32L4xx_HAL_Driver\\Inc\\stm32l4xx_ll_exti.h"

































 

 







 
#line 46 "..\\HALLIB\\STM32L4xx_HAL_Driver\\Inc\\stm32l4xx_ll_exti.h"



 





 

 
 
 
 
#line 69 "..\\HALLIB\\STM32L4xx_HAL_Driver\\Inc\\stm32l4xx_ll_exti.h"
 
#line 97 "..\\HALLIB\\STM32L4xx_HAL_Driver\\Inc\\stm32l4xx_ll_exti.h"

 


 



 
#line 165 "..\\HALLIB\\STM32L4xx_HAL_Driver\\Inc\\stm32l4xx_ll_exti.h"

#line 192 "..\\HALLIB\\STM32L4xx_HAL_Driver\\Inc\\stm32l4xx_ll_exti.h"










 


#line 231 "..\\HALLIB\\STM32L4xx_HAL_Driver\\Inc\\stm32l4xx_ll_exti.h"




 

 


 



 






 






 



 




 



 


 


 











































 
static __inline void LL_EXTI_EnableIT_0_31(uint32_t ExtiLine)
{
  ((((EXTI_TypeDef *) (((0x40000000UL) + 0x00010000UL) + 0x0400UL))->IMR1) |= (ExtiLine));
}



















 
static __inline void LL_EXTI_EnableIT_32_63(uint32_t ExtiLine)
{
  ((((EXTI_TypeDef *) (((0x40000000UL) + 0x00010000UL) + 0x0400UL))->IMR2) |= (ExtiLine));
}











































 
static __inline void LL_EXTI_DisableIT_0_31(uint32_t ExtiLine)
{
  ((((EXTI_TypeDef *) (((0x40000000UL) + 0x00010000UL) + 0x0400UL))->IMR1) &= ~(ExtiLine));
}




















 
static __inline void LL_EXTI_DisableIT_32_63(uint32_t ExtiLine)
{
  ((((EXTI_TypeDef *) (((0x40000000UL) + 0x00010000UL) + 0x0400UL))->IMR2) &= ~(ExtiLine));
}











































 
static __inline uint32_t LL_EXTI_IsEnabledIT_0_31(uint32_t ExtiLine)
{
  return ((((((EXTI_TypeDef *) (((0x40000000UL) + 0x00010000UL) + 0x0400UL))->IMR1) & (ExtiLine)) == (ExtiLine)) ? 1UL : 0UL);
}




















 
static __inline uint32_t LL_EXTI_IsEnabledIT_32_63(uint32_t ExtiLine)
{
  return ((((((EXTI_TypeDef *) (((0x40000000UL) + 0x00010000UL) + 0x0400UL))->IMR2) & (ExtiLine)) == (ExtiLine)) ? 1UL : 0UL);
}



 



 








































 
static __inline void LL_EXTI_EnableEvent_0_31(uint32_t ExtiLine)
{
  ((((EXTI_TypeDef *) (((0x40000000UL) + 0x00010000UL) + 0x0400UL))->EMR1) |= (ExtiLine));

}

















 
static __inline void LL_EXTI_EnableEvent_32_63(uint32_t ExtiLine)
{
  ((((EXTI_TypeDef *) (((0x40000000UL) + 0x00010000UL) + 0x0400UL))->EMR2) |= (ExtiLine));
}








































 
static __inline void LL_EXTI_DisableEvent_0_31(uint32_t ExtiLine)
{
  ((((EXTI_TypeDef *) (((0x40000000UL) + 0x00010000UL) + 0x0400UL))->EMR1) &= ~(ExtiLine));
}

















 
static __inline void LL_EXTI_DisableEvent_32_63(uint32_t ExtiLine)
{
  ((((EXTI_TypeDef *) (((0x40000000UL) + 0x00010000UL) + 0x0400UL))->EMR2) &= ~(ExtiLine));
}








































 
static __inline uint32_t LL_EXTI_IsEnabledEvent_0_31(uint32_t ExtiLine)
{
  return ((((((EXTI_TypeDef *) (((0x40000000UL) + 0x00010000UL) + 0x0400UL))->EMR1) & (ExtiLine)) == (ExtiLine)) ? 1UL : 0UL);

}

















 
static __inline uint32_t LL_EXTI_IsEnabledEvent_32_63(uint32_t ExtiLine)
{
  return ((((((EXTI_TypeDef *) (((0x40000000UL) + 0x00010000UL) + 0x0400UL))->EMR2) & (ExtiLine)) == (ExtiLine)) ? 1UL : 0UL);
}



 



 







































 
static __inline void LL_EXTI_EnableRisingTrig_0_31(uint32_t ExtiLine)
{
  ((((EXTI_TypeDef *) (((0x40000000UL) + 0x00010000UL) + 0x0400UL))->RTSR1) |= (ExtiLine));

}
















 
static __inline void LL_EXTI_EnableRisingTrig_32_63(uint32_t ExtiLine)
{
  ((((EXTI_TypeDef *) (((0x40000000UL) + 0x00010000UL) + 0x0400UL))->RTSR2) |= (ExtiLine));
}







































 
static __inline void LL_EXTI_DisableRisingTrig_0_31(uint32_t ExtiLine)
{
  ((((EXTI_TypeDef *) (((0x40000000UL) + 0x00010000UL) + 0x0400UL))->RTSR1) &= ~(ExtiLine));

}

















 
static __inline void LL_EXTI_DisableRisingTrig_32_63(uint32_t ExtiLine)
{
  ((((EXTI_TypeDef *) (((0x40000000UL) + 0x00010000UL) + 0x0400UL))->RTSR2) &= ~(ExtiLine));
}
































 
static __inline uint32_t LL_EXTI_IsEnabledRisingTrig_0_31(uint32_t ExtiLine)
{
  return ((((((EXTI_TypeDef *) (((0x40000000UL) + 0x00010000UL) + 0x0400UL))->RTSR1) & (ExtiLine)) == (ExtiLine)) ? 1UL : 0UL);
}










 
static __inline uint32_t LL_EXTI_IsEnabledRisingTrig_32_63(uint32_t ExtiLine)
{
  return ((((((EXTI_TypeDef *) (((0x40000000UL) + 0x00010000UL) + 0x0400UL))->RTSR2) & (ExtiLine)) == (ExtiLine)) ? 1UL : 0UL);
}



 



 







































 
static __inline void LL_EXTI_EnableFallingTrig_0_31(uint32_t ExtiLine)
{
  ((((EXTI_TypeDef *) (((0x40000000UL) + 0x00010000UL) + 0x0400UL))->FTSR1) |= (ExtiLine));
}

















 
static __inline void LL_EXTI_EnableFallingTrig_32_63(uint32_t ExtiLine)
{
  ((((EXTI_TypeDef *) (((0x40000000UL) + 0x00010000UL) + 0x0400UL))->FTSR2) |= (ExtiLine));
}






































 
static __inline void LL_EXTI_DisableFallingTrig_0_31(uint32_t ExtiLine)
{
  ((((EXTI_TypeDef *) (((0x40000000UL) + 0x00010000UL) + 0x0400UL))->FTSR1) &= ~(ExtiLine));
}
















 
static __inline void LL_EXTI_DisableFallingTrig_32_63(uint32_t ExtiLine)
{
  ((((EXTI_TypeDef *) (((0x40000000UL) + 0x00010000UL) + 0x0400UL))->FTSR2) &= ~(ExtiLine));
}
































 
static __inline uint32_t LL_EXTI_IsEnabledFallingTrig_0_31(uint32_t ExtiLine)
{
  return ((((((EXTI_TypeDef *) (((0x40000000UL) + 0x00010000UL) + 0x0400UL))->FTSR1) & (ExtiLine)) == (ExtiLine)) ? 1UL : 0UL);
}










 
static __inline uint32_t LL_EXTI_IsEnabledFallingTrig_32_63(uint32_t ExtiLine)
{
  return ((((((EXTI_TypeDef *) (((0x40000000UL) + 0x00010000UL) + 0x0400UL))->FTSR2) & (ExtiLine)) == (ExtiLine)) ? 1UL : 0UL);
}



 



 





































 
static __inline void LL_EXTI_GenerateSWI_0_31(uint32_t ExtiLine)
{
  ((((EXTI_TypeDef *) (((0x40000000UL) + 0x00010000UL) + 0x0400UL))->SWIER1) |= (ExtiLine));
}















 
static __inline void LL_EXTI_GenerateSWI_32_63(uint32_t ExtiLine)
{
  ((((EXTI_TypeDef *) (((0x40000000UL) + 0x00010000UL) + 0x0400UL))->SWIER2) |= (ExtiLine));
}



 



 


































 
static __inline uint32_t LL_EXTI_IsActiveFlag_0_31(uint32_t ExtiLine)
{
  return ((((((EXTI_TypeDef *) (((0x40000000UL) + 0x00010000UL) + 0x0400UL))->PR1) & (ExtiLine)) == (ExtiLine)) ? 1UL : 0UL);
}












 
static __inline uint32_t LL_EXTI_IsActiveFlag_32_63(uint32_t ExtiLine)
{
  return ((((((EXTI_TypeDef *) (((0x40000000UL) + 0x00010000UL) + 0x0400UL))->PR2) & (ExtiLine)) == (ExtiLine)) ? 1UL : 0UL);
}


































 
static __inline uint32_t LL_EXTI_ReadFlag_0_31(uint32_t ExtiLine)
{
  return (uint32_t)(((((EXTI_TypeDef *) (((0x40000000UL) + 0x00010000UL) + 0x0400UL))->PR1) & (ExtiLine)));
}












 
static __inline uint32_t LL_EXTI_ReadFlag_32_63(uint32_t ExtiLine)
{
  return (uint32_t)(((((EXTI_TypeDef *) (((0x40000000UL) + 0x00010000UL) + 0x0400UL))->PR2) & (ExtiLine)));
}


































 
static __inline void LL_EXTI_ClearFlag_0_31(uint32_t ExtiLine)
{
  ((((EXTI_TypeDef *) (((0x40000000UL) + 0x00010000UL) + 0x0400UL))->PR1) = (ExtiLine));
}












 
static __inline void LL_EXTI_ClearFlag_32_63(uint32_t ExtiLine)
{
  ((((EXTI_TypeDef *) (((0x40000000UL) + 0x00010000UL) + 0x0400UL))->PR2) = (ExtiLine));
}




 

#line 1356 "..\\HALLIB\\STM32L4xx_HAL_Driver\\Inc\\stm32l4xx_ll_exti.h"



 



 





 







 
#line 47 "..\\HALLIB\\STM32L4xx_HAL_Driver\\Inc\\stm32l4xx_hal_comp.h"



 




 

  


 



 
typedef struct
{


  uint32_t WindowMode;         


 


  uint32_t Mode;               


 

  uint32_t NonInvertingInput;  
 

  uint32_t InvertingInput;     
 

  uint32_t Hysteresis;         
 

  uint32_t OutputPol;          
 

  uint32_t BlankingSrce;       
 

  uint32_t TriggerMode;        
 

}COMP_InitTypeDef;



 

typedef enum
{
  HAL_COMP_STATE_RESET             = 0x00U,                                              
  HAL_COMP_STATE_RESET_LOCKED      = (HAL_COMP_STATE_RESET | (0x10U)),  
  HAL_COMP_STATE_READY             = 0x01U,                                              
  HAL_COMP_STATE_READY_LOCKED      = (HAL_COMP_STATE_READY | (0x10U)),  
  HAL_COMP_STATE_BUSY              = 0x02U,                                              
  HAL_COMP_STATE_BUSY_LOCKED       = (HAL_COMP_STATE_BUSY | (0x10U))    
}HAL_COMP_StateTypeDef;



 
typedef struct __COMP_HandleTypeDef
{
  COMP_TypeDef       *Instance;        
  COMP_InitTypeDef   Init;             
  HAL_LockTypeDef    Lock;             
  volatile HAL_COMP_StateTypeDef  State;   
  volatile uint32_t      ErrorCode;        





} COMP_HandleTypeDef;

#line 148 "..\\HALLIB\\STM32L4xx_HAL_Driver\\Inc\\stm32l4xx_hal_comp.h"



 

 


 



 






 




 




 




 
 
 
 





 



 







 



 
#line 223 "..\\HALLIB\\STM32L4xx_HAL_Driver\\Inc\\stm32l4xx_hal_comp.h"


 



 






 



 




 



 
#line 257 "..\\HALLIB\\STM32L4xx_HAL_Driver\\Inc\\stm32l4xx_hal_comp.h"


 



 
 
 
 


 


 



 



 
#line 288 "..\\HALLIB\\STM32L4xx_HAL_Driver\\Inc\\stm32l4xx_hal_comp.h"


 



 

 


 



 




 
#line 318 "..\\HALLIB\\STM32L4xx_HAL_Driver\\Inc\\stm32l4xx_hal_comp.h"





 






 






 










 






 




 



 




 





 





 





 





 








                                          








 





 





 





 





 





 





 






 





 





 





 





 








                                          








 





 





 





 





 





 





 





 



 


 
 


 



 






 



 






 



 

 


 



 




 
#line 596 "..\\HALLIB\\STM32L4xx_HAL_Driver\\Inc\\stm32l4xx_hal_comp.h"


 



 









#line 620 "..\\HALLIB\\STM32L4xx_HAL_Driver\\Inc\\stm32l4xx_hal_comp.h"

 
 
 
 
#line 666 "..\\HALLIB\\STM32L4xx_HAL_Driver\\Inc\\stm32l4xx_hal_comp.h"









#line 701 "..\\HALLIB\\STM32L4xx_HAL_Driver\\Inc\\stm32l4xx_hal_comp.h"

#line 731 "..\\HALLIB\\STM32L4xx_HAL_Driver\\Inc\\stm32l4xx_hal_comp.h"

#line 739 "..\\HALLIB\\STM32L4xx_HAL_Driver\\Inc\\stm32l4xx_hal_comp.h"






 



 


 


 



 

 
HAL_StatusTypeDef HAL_COMP_Init(COMP_HandleTypeDef *hcomp);
HAL_StatusTypeDef HAL_COMP_DeInit (COMP_HandleTypeDef *hcomp);
void              HAL_COMP_MspInit(COMP_HandleTypeDef *hcomp);
void              HAL_COMP_MspDeInit(COMP_HandleTypeDef *hcomp);








 

 


 
HAL_StatusTypeDef HAL_COMP_Start(COMP_HandleTypeDef *hcomp);
HAL_StatusTypeDef HAL_COMP_Stop(COMP_HandleTypeDef *hcomp);
void              HAL_COMP_IRQHandler(COMP_HandleTypeDef *hcomp);


 

 


 
HAL_StatusTypeDef HAL_COMP_Lock(COMP_HandleTypeDef *hcomp);
uint32_t          HAL_COMP_GetOutputLevel(COMP_HandleTypeDef *hcomp);
 
void              HAL_COMP_TriggerCallback(COMP_HandleTypeDef *hcomp);


 

 


 
HAL_COMP_StateTypeDef HAL_COMP_GetState(COMP_HandleTypeDef *hcomp);
uint32_t              HAL_COMP_GetError(COMP_HandleTypeDef *hcomp);


 



 



 



 







 
#line 258 "..\\USER\\stm32l4xx_hal_conf.h"


#line 1 "..\\HALLIB\\STM32L4xx_HAL_Driver\\Inc\\stm32l4xx_hal_crc.h"

































 

 







 
#line 46 "..\\HALLIB\\STM32L4xx_HAL_Driver\\Inc\\stm32l4xx_hal_crc.h"



 



 

 


 



 
typedef enum
{
  HAL_CRC_STATE_RESET     = 0x00U,   
  HAL_CRC_STATE_READY     = 0x01U,   
  HAL_CRC_STATE_BUSY      = 0x02U,   
  HAL_CRC_STATE_TIMEOUT   = 0x03U,   
  HAL_CRC_STATE_ERROR     = 0x04U    
} HAL_CRC_StateTypeDef;



 
typedef struct
{
  uint8_t DefaultPolynomialUse;       



 

  uint8_t DefaultInitValueUse;        


 

  uint32_t GeneratingPolynomial;      


 

  uint32_t CRCLength;                 




 

  uint32_t InitValue;                 
 

  uint32_t InputDataInversionMode;    




 

  uint32_t OutputDataInversionMode;   


 
} CRC_InitTypeDef;



 
typedef struct
{
  CRC_TypeDef                 *Instance;    

  CRC_InitTypeDef             Init;         

  HAL_LockTypeDef             Lock;         

  volatile HAL_CRC_StateTypeDef   State;        

  uint32_t InputDataFormat;                






 
} CRC_HandleTypeDef;


 

 


 



 



 



 



 



 




 



 




 



 






 



 






 



 



 






 



 




 



 

 


 




 






 







 








 







 



 


 


 



















 

 
#line 1 "..\\HALLIB\\STM32L4xx_HAL_Driver\\Inc\\stm32l4xx_hal_crc_ex.h"

































 

 







 
#line 46 "..\\HALLIB\\STM32L4xx_HAL_Driver\\Inc\\stm32l4xx_hal_crc_ex.h"



 



 

 
 


 



 






 



 




 



 

 


 





 






 







 




 

 


 











 

 



 



 
 
HAL_StatusTypeDef HAL_CRCEx_Polynomial_Set(CRC_HandleTypeDef *hcrc, uint32_t Pol, uint32_t PolyLength);
HAL_StatusTypeDef HAL_CRCEx_Input_Data_Reverse(CRC_HandleTypeDef *hcrc, uint32_t InputReverseMode);
HAL_StatusTypeDef HAL_CRCEx_Output_Data_Reverse(CRC_HandleTypeDef *hcrc, uint32_t OutputReverseMode);



 



 



 



 







 
#line 305 "..\\HALLIB\\STM32L4xx_HAL_Driver\\Inc\\stm32l4xx_hal_crc.h"

 


 

 


 
HAL_StatusTypeDef HAL_CRC_Init(CRC_HandleTypeDef *hcrc);
HAL_StatusTypeDef HAL_CRC_DeInit(CRC_HandleTypeDef *hcrc);
void HAL_CRC_MspInit(CRC_HandleTypeDef *hcrc);
void HAL_CRC_MspDeInit(CRC_HandleTypeDef *hcrc);


 

 


 
uint32_t HAL_CRC_Accumulate(CRC_HandleTypeDef *hcrc, uint32_t pBuffer[], uint32_t BufferLength);
uint32_t HAL_CRC_Calculate(CRC_HandleTypeDef *hcrc, uint32_t pBuffer[], uint32_t BufferLength);


 

 


 
HAL_CRC_StateTypeDef HAL_CRC_GetState(CRC_HandleTypeDef *hcrc);


 



 



 



 







 
#line 262 "..\\USER\\stm32l4xx_hal_conf.h"


#line 1 "..\\HALLIB\\STM32L4xx_HAL_Driver\\Inc\\stm32l4xx_hal_cryp.h"

































 

 







 
#line 46 "..\\HALLIB\\STM32L4xx_HAL_Driver\\Inc\\stm32l4xx_hal_cryp.h"



 

#line 738 "..\\HALLIB\\STM32L4xx_HAL_Driver\\Inc\\stm32l4xx_hal_cryp.h"



 
  






 
#line 266 "..\\USER\\stm32l4xx_hal_conf.h"


#line 1 "..\\HALLIB\\STM32L4xx_HAL_Driver\\Inc\\stm32l4xx_hal_dac.h"

































 

 









 

 
#line 50 "..\\HALLIB\\STM32L4xx_HAL_Driver\\Inc\\stm32l4xx_hal_dac.h"





 

 



 



 
typedef enum
{
  HAL_DAC_STATE_RESET             = 0x00U,   
  HAL_DAC_STATE_READY             = 0x01U,   
  HAL_DAC_STATE_BUSY              = 0x02U,   
  HAL_DAC_STATE_TIMEOUT           = 0x03U,   
  HAL_DAC_STATE_ERROR             = 0x04U    

}HAL_DAC_StateTypeDef;



 



typedef struct DAC_HandleTypeDef

{
  DAC_TypeDef                 *Instance;      

  volatile HAL_DAC_StateTypeDef   State;          

  HAL_LockTypeDef             Lock;           

  DMA_HandleTypeDef           *DMA_Handle1;   

  DMA_HandleTypeDef           *DMA_Handle2;   

  volatile uint32_t               ErrorCode;      

#line 110 "..\\HALLIB\\STM32L4xx_HAL_Driver\\Inc\\stm32l4xx_hal_dac.h"

}DAC_HandleTypeDef;



 
typedef struct
{
  uint32_t DAC_SampleTime ;          

 

  uint32_t DAC_HoldTime ;            

 

  uint32_t DAC_RefreshTime ;         

 
}
DAC_SampleAndHoldConfTypeDef;



 
typedef struct
{





  uint32_t DAC_SampleAndHold;            
 

  uint32_t DAC_Trigger;                  
 

  uint32_t DAC_OutputBuffer;             
 

  uint32_t DAC_ConnectOnChipPeripheral ; 
 

  uint32_t DAC_UserTrimming;             

 

  uint32_t DAC_TrimmingValue;             

 

  DAC_SampleAndHoldConfTypeDef  DAC_SampleAndHoldConfig;   

}DAC_ChannelConfTypeDef;

#line 190 "..\\HALLIB\\STM32L4xx_HAL_Driver\\Inc\\stm32l4xx_hal_dac.h"



 

 



 



 
#line 212 "..\\HALLIB\\STM32L4xx_HAL_Driver\\Inc\\stm32l4xx_hal_dac.h"



 



 

#line 230 "..\\HALLIB\\STM32L4xx_HAL_Driver\\Inc\\stm32l4xx_hal_dac.h"

#line 239 "..\\HALLIB\\STM32L4xx_HAL_Driver\\Inc\\stm32l4xx_hal_dac.h"

#line 252 "..\\HALLIB\\STM32L4xx_HAL_Driver\\Inc\\stm32l4xx_hal_dac.h"


#line 270 "..\\HALLIB\\STM32L4xx_HAL_Driver\\Inc\\stm32l4xx_hal_dac.h"




 



 





 



 
#line 295 "..\\HALLIB\\STM32L4xx_HAL_Driver\\Inc\\stm32l4xx_hal_dac.h"
         
         



 



 






 



 





 



 





 



 





 

  

 






 



 





 
#line 375 "..\\HALLIB\\STM32L4xx_HAL_Driver\\Inc\\stm32l4xx_hal_dac.h"



 

 



 




 
#line 399 "..\\HALLIB\\STM32L4xx_HAL_Driver\\Inc\\stm32l4xx_hal_dac.h"





 







 






 





 





 









 









 









 









 









 




 

 



 



#line 502 "..\\HALLIB\\STM32L4xx_HAL_Driver\\Inc\\stm32l4xx_hal_dac.h"
         
         















 

 
#line 1 "..\\HALLIB\\STM32L4xx_HAL_Driver\\Inc\\stm32l4xx_hal_dac_ex.h"

































 

 









 

 
#line 50 "..\\HALLIB\\STM32L4xx_HAL_Driver\\Inc\\stm32l4xx_hal_dac_ex.h"





 

 



 

 



 



 
#line 96 "..\\HALLIB\\STM32L4xx_HAL_Driver\\Inc\\stm32l4xx_hal_dac_ex.h"



 



 

 


 



 
#line 121 "..\\HALLIB\\STM32L4xx_HAL_Driver\\Inc\\stm32l4xx_hal_dac_ex.h"

#line 129 "..\\HALLIB\\STM32L4xx_HAL_Driver\\Inc\\stm32l4xx_hal_dac_ex.h"

#line 141 "..\\HALLIB\\STM32L4xx_HAL_Driver\\Inc\\stm32l4xx_hal_dac_ex.h"

#line 162 "..\\HALLIB\\STM32L4xx_HAL_Driver\\Inc\\stm32l4xx_hal_dac_ex.h"


















#line 204 "..\\HALLIB\\STM32L4xx_HAL_Driver\\Inc\\stm32l4xx_hal_dac_ex.h"


 

 
 



 



 
 

HAL_StatusTypeDef HAL_DACEx_TriangleWaveGenerate(DAC_HandleTypeDef* hdac, uint32_t Channel, uint32_t Amplitude);
HAL_StatusTypeDef HAL_DACEx_NoiseWaveGenerate(DAC_HandleTypeDef* hdac, uint32_t Channel, uint32_t Amplitude);




HAL_StatusTypeDef HAL_DACEx_DualSetValue(DAC_HandleTypeDef* hdac, uint32_t Alignment, uint32_t Data1, uint32_t Data2);
uint32_t HAL_DACEx_DualGetValue(DAC_HandleTypeDef* hdac);

         
         



void HAL_DACEx_ConvCpltCallbackCh2(DAC_HandleTypeDef* hdac);
void HAL_DACEx_ConvHalfCpltCallbackCh2(DAC_HandleTypeDef* hdac);
void HAL_DACEx_ErrorCallbackCh2(DAC_HandleTypeDef* hdac);
void HAL_DACEx_DMAUnderrunCallbackCh2(DAC_HandleTypeDef* hdac);

         
         




 



 
 

HAL_StatusTypeDef HAL_DACEx_SelfCalibrate(DAC_HandleTypeDef *hdac, DAC_ChannelConfTypeDef *sConfig, uint32_t Channel);
HAL_StatusTypeDef HAL_DACEx_SetUserTrimming(DAC_HandleTypeDef *hdac, DAC_ChannelConfTypeDef *sConfig, uint32_t Channel, uint32_t NewTrimmingValue);
uint32_t HAL_DACEx_GetTrimOffset (DAC_HandleTypeDef *hdac, uint32_t Channel);



 



 







 

 
 
void DAC_DMAConvCpltCh2(DMA_HandleTypeDef *hdma);
void DAC_DMAErrorCh2(DMA_HandleTypeDef *hdma);
void DAC_DMAHalfConvCpltCh2(DMA_HandleTypeDef *hdma);



 

         
         


 





 







 
#line 523 "..\\HALLIB\\STM32L4xx_HAL_Driver\\Inc\\stm32l4xx_hal_dac.h"

 



 



 
 
HAL_StatusTypeDef HAL_DAC_Init(DAC_HandleTypeDef* hdac);
HAL_StatusTypeDef HAL_DAC_DeInit(DAC_HandleTypeDef* hdac);
void HAL_DAC_MspInit(DAC_HandleTypeDef* hdac);
void HAL_DAC_MspDeInit(DAC_HandleTypeDef* hdac);



 



 
 
HAL_StatusTypeDef HAL_DAC_Start(DAC_HandleTypeDef* hdac, uint32_t Channel);
HAL_StatusTypeDef HAL_DAC_Stop(DAC_HandleTypeDef* hdac, uint32_t Channel);
HAL_StatusTypeDef HAL_DAC_Start_DMA(DAC_HandleTypeDef* hdac, uint32_t Channel, uint32_t* pData, uint32_t Length, uint32_t Alignment);
HAL_StatusTypeDef HAL_DAC_Stop_DMA(DAC_HandleTypeDef* hdac, uint32_t Channel);

void HAL_DAC_IRQHandler(DAC_HandleTypeDef* hdac);

HAL_StatusTypeDef HAL_DAC_SetValue(DAC_HandleTypeDef* hdac, uint32_t Channel, uint32_t Alignment, uint32_t Data);

void HAL_DAC_ConvCpltCallbackCh1(DAC_HandleTypeDef* hdac);
void HAL_DAC_ConvHalfCpltCallbackCh1(DAC_HandleTypeDef* hdac);
void HAL_DAC_ErrorCallbackCh1(DAC_HandleTypeDef *hdac);
void HAL_DAC_DMAUnderrunCallbackCh1(DAC_HandleTypeDef *hdac);









 



 
 
uint32_t HAL_DAC_GetValue(DAC_HandleTypeDef* hdac, uint32_t Channel);

HAL_StatusTypeDef HAL_DAC_ConfigChannel(DAC_HandleTypeDef* hdac, DAC_ChannelConfTypeDef* sConfig, uint32_t Channel);


 



 
 
HAL_DAC_StateTypeDef HAL_DAC_GetState(DAC_HandleTypeDef* hdac);
uint32_t HAL_DAC_GetError(DAC_HandleTypeDef *hdac);



 



 



 





 








 

#line 270 "..\\USER\\stm32l4xx_hal_conf.h"


#line 1 "..\\HALLIB\\STM32L4xx_HAL_Driver\\Inc\\stm32l4xx_hal_dcmi.h"

































 

 







 
#line 46 "..\\HALLIB\\STM32L4xx_HAL_Driver\\Inc\\stm32l4xx_hal_dcmi.h"

#line 686 "..\\HALLIB\\STM32L4xx_HAL_Driver\\Inc\\stm32l4xx_hal_dcmi.h"







 
#line 274 "..\\USER\\stm32l4xx_hal_conf.h"


#line 1 "..\\HALLIB\\STM32L4xx_HAL_Driver\\Inc\\stm32l4xx_hal_dma2d.h"

































 

 







#line 730 "..\\HALLIB\\STM32L4xx_HAL_Driver\\Inc\\stm32l4xx_hal_dma2d.h"








 
#line 278 "..\\USER\\stm32l4xx_hal_conf.h"


#line 1 "..\\HALLIB\\STM32L4xx_HAL_Driver\\Inc\\stm32l4xx_hal_dsi.h"

































 

 







#line 1338 "..\\HALLIB\\STM32L4xx_HAL_Driver\\Inc\\stm32l4xx_hal_dsi.h"







 
#line 282 "..\\USER\\stm32l4xx_hal_conf.h"


#line 1 "..\\HALLIB\\STM32L4xx_HAL_Driver\\Inc\\stm32l4xx_hal_firewall.h"

































 

 







 
#line 46 "..\\HALLIB\\STM32L4xx_HAL_Driver\\Inc\\stm32l4xx_hal_firewall.h"



 



  

  


  



  
typedef struct
{
  uint32_t CodeSegmentStartAddress;        
 

  uint32_t CodeSegmentLength;              
 

  uint32_t NonVDataSegmentStartAddress;    
 

  uint32_t NonVDataSegmentLength;          
 
 
  uint32_t VDataSegmentStartAddress;       
 

  uint32_t VDataSegmentLength;             
 
  
  uint32_t VolatileDataExecution;          

   
                                           
  uint32_t VolatileDataShared;             

   
                                                                                                                                     
}FIREWALL_InitTypeDef;




 

  
 


 



 




  



  




  



  





 



 
  
 


 








    
  


                                               




   


 


 



             












  
#line 190 "..\\HALLIB\\STM32L4xx_HAL_Driver\\Inc\\stm32l4xx_hal_firewall.h"


                    









  
#line 212 "..\\HALLIB\\STM32L4xx_HAL_Driver\\Inc\\stm32l4xx_hal_firewall.h"









  
#line 231 "..\\HALLIB\\STM32L4xx_HAL_Driver\\Inc\\stm32l4xx_hal_firewall.h"










  
#line 251 "..\\HALLIB\\STM32L4xx_HAL_Driver\\Inc\\stm32l4xx_hal_firewall.h"












  
#line 273 "..\\HALLIB\\STM32L4xx_HAL_Driver\\Inc\\stm32l4xx_hal_firewall.h"










 
#line 293 "..\\HALLIB\\STM32L4xx_HAL_Driver\\Inc\\stm32l4xx_hal_firewall.h"









 









 









 





 

 



 
  



   
  
 
HAL_StatusTypeDef HAL_FIREWALL_Config(FIREWALL_InitTypeDef * fw_init);
void HAL_FIREWALL_GetConfig(FIREWALL_InitTypeDef * fw_config);
void HAL_FIREWALL_EnableFirewall(void);
void HAL_FIREWALL_EnablePreArmFlag(void);
void HAL_FIREWALL_DisablePreArmFlag(void);



 
  


    



  



  
  






 
#line 286 "..\\USER\\stm32l4xx_hal_conf.h"


#line 1 "..\\HALLIB\\STM32L4xx_HAL_Driver\\Inc\\stm32l4xx_hal_flash.h"

































 

 







 
#line 46 "..\\HALLIB\\STM32L4xx_HAL_Driver\\Inc\\stm32l4xx_hal_flash.h"



 



 

 


 



 
typedef struct
{
  uint32_t TypeErase;   
 
  uint32_t Banks;       

 
  uint32_t Page;        

 
  uint32_t NbPages;     
 
} FLASH_EraseInitTypeDef;



 
typedef struct
{
  uint32_t OptionType;     
 
  uint32_t WRPArea;        

 
  uint32_t WRPStartOffset; 

 
  uint32_t WRPEndOffset;   
 
  uint32_t RDPLevel;       
 
  uint32_t USERType;       
 
  uint32_t USERConfig;     






 
  uint32_t PCROPConfig;    

 
  uint32_t PCROPStartAddr; 

 
  uint32_t PCROPEndAddr;   
 
} FLASH_OBProgramInitTypeDef;



 
typedef enum
{
  FLASH_PROC_NONE = 0,
  FLASH_PROC_PAGE_ERASE,
  FLASH_PROC_MASS_ERASE,
  FLASH_PROC_PROGRAM,
  FLASH_PROC_PROGRAM_LAST
} FLASH_ProcedureTypeDef;



 
typedef enum
{
  FLASH_CACHE_DISABLED = 0,
  FLASH_CACHE_ICACHE_ENABLED,
  FLASH_CACHE_DCACHE_ENABLED,
  FLASH_CACHE_ICACHE_DCACHE_ENABLED
} FLASH_CacheTypeDef;



 
typedef struct
{
  HAL_LockTypeDef             Lock;               
  volatile uint32_t               ErrorCode;          
  volatile FLASH_ProcedureTypeDef ProcedureOnGoing;   
  volatile uint32_t               Address;            
  volatile uint32_t               Bank;               
  volatile uint32_t               Page;               
  volatile uint32_t               NbPagesToErase;     
  volatile FLASH_CacheTypeDef     CacheToReactivate;  
}FLASH_ProcessTypeDef;



 

 


 



 
#line 181 "..\\HALLIB\\STM32L4xx_HAL_Driver\\Inc\\stm32l4xx_hal_flash.h"


 



 




 



 
#line 206 "..\\HALLIB\\STM32L4xx_HAL_Driver\\Inc\\stm32l4xx_hal_flash.h"


 




 







 



 






 



 
#line 245 "..\\HALLIB\\STM32L4xx_HAL_Driver\\Inc\\stm32l4xx_hal_flash.h"


 



 






 



 
#line 294 "..\\HALLIB\\STM32L4xx_HAL_Driver\\Inc\\stm32l4xx_hal_flash.h"


 



 







 



 




 



 




 



 




 



 




 



 




 



 




 



 




 






 




 
#line 394 "..\\HALLIB\\STM32L4xx_HAL_Driver\\Inc\\stm32l4xx_hal_flash.h"


 




 



#line 412 "..\\HALLIB\\STM32L4xx_HAL_Driver\\Inc\\stm32l4xx_hal_flash.h"


 


 




 



 




 



 




 

#line 464 "..\\HALLIB\\STM32L4xx_HAL_Driver\\Inc\\stm32l4xx_hal_flash.h"



 






 



 
#line 497 "..\\HALLIB\\STM32L4xx_HAL_Driver\\Inc\\stm32l4xx_hal_flash.h"


 



 













 



 
#line 550 "..\\HALLIB\\STM32L4xx_HAL_Driver\\Inc\\stm32l4xx_hal_flash.h"







 




 






 

 



 











 











 





 





 





 





 





 





 






 








 








 









 








 





 




 




 










 













 
























 























 





 

 
#line 1 "..\\HALLIB\\STM32L4xx_HAL_Driver\\Inc\\stm32l4xx_hal_flash_ex.h"

































 

 







 
#line 46 "..\\HALLIB\\STM32L4xx_HAL_Driver\\Inc\\stm32l4xx_hal_flash_ex.h"



 



 

 

 
#line 75 "..\\HALLIB\\STM32L4xx_HAL_Driver\\Inc\\stm32l4xx_hal_flash_ex.h"

 

 


 

 


 
HAL_StatusTypeDef HAL_FLASHEx_Erase(FLASH_EraseInitTypeDef *pEraseInit, uint32_t *PageError);
HAL_StatusTypeDef HAL_FLASHEx_Erase_IT(FLASH_EraseInitTypeDef *pEraseInit);
HAL_StatusTypeDef HAL_FLASHEx_OBProgram(FLASH_OBProgramInitTypeDef *pOBInit);
void              HAL_FLASHEx_OBGetConfig(FLASH_OBProgramInitTypeDef *pOBInit);


 

#line 104 "..\\HALLIB\\STM32L4xx_HAL_Driver\\Inc\\stm32l4xx_hal_flash_ex.h"



 

 


 





 



 



 







 
#line 779 "..\\HALLIB\\STM32L4xx_HAL_Driver\\Inc\\stm32l4xx_hal_flash.h"
#line 1 "..\\HALLIB\\STM32L4xx_HAL_Driver\\Inc\\stm32l4xx_hal_flash_ramfunc.h"

































  

 







 
#line 46 "..\\HALLIB\\STM32L4xx_HAL_Driver\\Inc\\stm32l4xx_hal_flash_ramfunc.h"



 



  

  
 
 


 



 
 
HAL_StatusTypeDef  HAL_FLASHEx_EnableRunPowerDown(void);
HAL_StatusTypeDef  HAL_FLASHEx_DisableRunPowerDown(void);





  



  



  



 







 
#line 780 "..\\HALLIB\\STM32L4xx_HAL_Driver\\Inc\\stm32l4xx_hal_flash.h"

 


 

 


 
HAL_StatusTypeDef  HAL_FLASH_Program(uint32_t TypeProgram, uint32_t Address, uint64_t Data);
HAL_StatusTypeDef  HAL_FLASH_Program_IT(uint32_t TypeProgram, uint32_t Address, uint64_t Data);
 
void               HAL_FLASH_IRQHandler(void);
 
void               HAL_FLASH_EndOfOperationCallback(uint32_t ReturnValue);
void               HAL_FLASH_OperationErrorCallback(uint32_t ReturnValue);


 

 


 
HAL_StatusTypeDef  HAL_FLASH_Unlock(void);
HAL_StatusTypeDef  HAL_FLASH_Lock(void);
 
HAL_StatusTypeDef  HAL_FLASH_OB_Unlock(void);
HAL_StatusTypeDef  HAL_FLASH_OB_Lock(void);
HAL_StatusTypeDef  HAL_FLASH_OB_Launch(void);


 

 


 
uint32_t HAL_FLASH_GetError(void);


 



 

 


 


#line 847 "..\\HALLIB\\STM32L4xx_HAL_Driver\\Inc\\stm32l4xx_hal_flash.h"

#line 855 "..\\HALLIB\\STM32L4xx_HAL_Driver\\Inc\\stm32l4xx_hal_flash.h"

#line 862 "..\\HALLIB\\STM32L4xx_HAL_Driver\\Inc\\stm32l4xx_hal_flash.h"




 

 


 




#line 882 "..\\HALLIB\\STM32L4xx_HAL_Driver\\Inc\\stm32l4xx_hal_flash.h"

#line 890 "..\\HALLIB\\STM32L4xx_HAL_Driver\\Inc\\stm32l4xx_hal_flash.h"





#line 904 "..\\HALLIB\\STM32L4xx_HAL_Driver\\Inc\\stm32l4xx_hal_flash.h"





#line 925 "..\\HALLIB\\STM32L4xx_HAL_Driver\\Inc\\stm32l4xx_hal_flash.h"



#line 936 "..\\HALLIB\\STM32L4xx_HAL_Driver\\Inc\\stm32l4xx_hal_flash.h"





#line 948 "..\\HALLIB\\STM32L4xx_HAL_Driver\\Inc\\stm32l4xx_hal_flash.h"





































#line 993 "..\\HALLIB\\STM32L4xx_HAL_Driver\\Inc\\stm32l4xx_hal_flash.h"



#line 1012 "..\\HALLIB\\STM32L4xx_HAL_Driver\\Inc\\stm32l4xx_hal_flash.h"


 



 



 



 







 
#line 290 "..\\USER\\stm32l4xx_hal_conf.h"


#line 1 "..\\HALLIB\\STM32L4xx_HAL_Driver\\Inc\\stm32l4xx_hal_hash.h"

































 

 







 
#line 46 "..\\HALLIB\\STM32L4xx_HAL_Driver\\Inc\\stm32l4xx_hal_hash.h"



 
#line 621 "..\\HALLIB\\STM32L4xx_HAL_Driver\\Inc\\stm32l4xx_hal_hash.h"


 









 
#line 294 "..\\USER\\stm32l4xx_hal_conf.h"


#line 1 "..\\HALLIB\\STM32L4xx_HAL_Driver\\Inc\\stm32l4xx_hal_sram.h"

































 

 









 
#line 1 "..\\HALLIB\\STM32L4xx_HAL_Driver\\Inc\\stm32l4xx_ll_fmc.h"

































 

 







 
#line 46 "..\\HALLIB\\STM32L4xx_HAL_Driver\\Inc\\stm32l4xx_ll_fmc.h"



 



 



 


#line 120 "..\\HALLIB\\STM32L4xx_HAL_Driver\\Inc\\stm32l4xx_ll_fmc.h"




#line 144 "..\\HALLIB\\STM32L4xx_HAL_Driver\\Inc\\stm32l4xx_ll_fmc.h"





 

 



 

#line 164 "..\\HALLIB\\STM32L4xx_HAL_Driver\\Inc\\stm32l4xx_ll_fmc.h"

#line 172 "..\\HALLIB\\STM32L4xx_HAL_Driver\\Inc\\stm32l4xx_ll_fmc.h"




 
typedef struct
{
  uint32_t NSBank;                       
 

  uint32_t DataAddressMux;               

 

  uint32_t MemoryType;                   

 

  uint32_t MemoryDataWidth;              
 

  uint32_t BurstAccessMode;              

 

  uint32_t WaitSignalPolarity;           

 

  uint32_t WaitSignalActive;             


 

  uint32_t WriteOperation;               
 

  uint32_t WaitSignal;                   

 

  uint32_t ExtendedMode;                 
 

  uint32_t AsynchronousWait;             

 

  uint32_t WriteBurst;                   
 

  uint32_t ContinuousClock;              


 

  uint32_t WriteFifo;                    



 

  uint32_t PageSize;                     
 






}FMC_NORSRAM_InitTypeDef;



 
typedef struct
{
  uint32_t AddressSetupTime;             


 

  uint32_t AddressHoldTime;              


 

  uint32_t DataSetupTime;                



 

#line 271 "..\\HALLIB\\STM32L4xx_HAL_Driver\\Inc\\stm32l4xx_ll_fmc.h"

  uint32_t BusTurnAroundDuration;        


 

  uint32_t CLKDivision;                  


 

  uint32_t DataLatency;                  





 

  uint32_t AccessMode;                   
 
}FMC_NORSRAM_TimingTypeDef;





 
typedef struct
{
  uint32_t NandBank;               
 

  uint32_t Waitfeature;            
 

  uint32_t MemoryDataWidth;        
 

  uint32_t EccComputation;         
 

  uint32_t ECCPageSize;            
 

  uint32_t TCLRSetupTime;          

 

  uint32_t TARSetupTime;           

 
}FMC_NAND_InitTypeDef;



 
typedef struct
{
  uint32_t SetupTime;            



 

  uint32_t WaitSetupTime;        



 

  uint32_t HoldSetupTime;        




 

  uint32_t HiZSetupTime;         



 
}FMC_NAND_PCC_TimingTypeDef;




 

 


 




 



 






 



 




 



 





 



 





 



 




 



 




 



 




 



 




 



 




 



 




 



 




 



 




 



 







 



 




 



 




 

#line 523 "..\\HALLIB\\STM32L4xx_HAL_Driver\\Inc\\stm32l4xx_ll_fmc.h"



 







 
#line 547 "..\\HALLIB\\STM32L4xx_HAL_Driver\\Inc\\stm32l4xx_ll_fmc.h"



 






 


 



 



 




 



 



 



 




 



 




 



 
#line 610 "..\\HALLIB\\STM32L4xx_HAL_Driver\\Inc\\stm32l4xx_ll_fmc.h"


 



 





 







 



 
#line 641 "..\\HALLIB\\STM32L4xx_HAL_Driver\\Inc\\stm32l4xx_ll_fmc.h"


 


 



 

 


 





 






 







 




 






 





 






 




 




 










 











 













 












 




 




 



 

 


 




 


 
HAL_StatusTypeDef  FMC_NORSRAM_Init(FMC_Bank1_TypeDef *Device, FMC_NORSRAM_InitTypeDef *Init);
HAL_StatusTypeDef  FMC_NORSRAM_Timing_Init(FMC_Bank1_TypeDef *Device, FMC_NORSRAM_TimingTypeDef *Timing, uint32_t Bank);
HAL_StatusTypeDef  FMC_NORSRAM_Extended_Timing_Init(FMC_Bank1E_TypeDef *Device, FMC_NORSRAM_TimingTypeDef *Timing, uint32_t Bank, uint32_t ExtendedMode);
HAL_StatusTypeDef  FMC_NORSRAM_DeInit(FMC_Bank1_TypeDef *Device, FMC_Bank1E_TypeDef *ExDevice, uint32_t Bank);


 



 
HAL_StatusTypeDef  FMC_NORSRAM_WriteOperation_Enable(FMC_Bank1_TypeDef *Device, uint32_t Bank);
HAL_StatusTypeDef  FMC_NORSRAM_WriteOperation_Disable(FMC_Bank1_TypeDef *Device, uint32_t Bank);


 


 





 


 
HAL_StatusTypeDef  FMC_NAND_Init(FMC_Bank3_TypeDef *Device, FMC_NAND_InitTypeDef *Init);
HAL_StatusTypeDef  FMC_NAND_CommonSpace_Timing_Init(FMC_Bank3_TypeDef *Device, FMC_NAND_PCC_TimingTypeDef *Timing, uint32_t Bank);
HAL_StatusTypeDef  FMC_NAND_AttributeSpace_Timing_Init(FMC_Bank3_TypeDef *Device, FMC_NAND_PCC_TimingTypeDef *Timing, uint32_t Bank);
HAL_StatusTypeDef  FMC_NAND_DeInit(FMC_Bank3_TypeDef *Device, uint32_t Bank);


 



 
HAL_StatusTypeDef  FMC_NAND_ECC_Enable(FMC_Bank3_TypeDef *Device, uint32_t Bank);
HAL_StatusTypeDef  FMC_NAND_ECC_Disable(FMC_Bank3_TypeDef *Device, uint32_t Bank);
HAL_StatusTypeDef  FMC_NAND_GetECC(FMC_Bank3_TypeDef *Device, uint32_t *ECCval, uint32_t Bank, uint32_t Timeout);


 


 






 



 



 







 
#line 48 "..\\HALLIB\\STM32L4xx_HAL_Driver\\Inc\\stm32l4xx_hal_sram.h"



 


 

 



 


 
typedef enum
{
  HAL_SRAM_STATE_RESET     = 0x00U,   
  HAL_SRAM_STATE_READY     = 0x01U,   
  HAL_SRAM_STATE_BUSY      = 0x02U,   
  HAL_SRAM_STATE_ERROR     = 0x03U,   
  HAL_SRAM_STATE_PROTECTED = 0x04U    
}HAL_SRAM_StateTypeDef;



 
typedef struct
{
  FMC_Bank1_TypeDef           *Instance;   

  FMC_Bank1E_TypeDef  *Extended;   

  FMC_NORSRAM_InitTypeDef       Init;        

  HAL_LockTypeDef               Lock;        

  volatile HAL_SRAM_StateTypeDef    State;       

  DMA_HandleTypeDef             *hdma;       
}SRAM_HandleTypeDef;



 

 
 



 




 




 

 


 



 

 
HAL_StatusTypeDef HAL_SRAM_Init(SRAM_HandleTypeDef *hsram, FMC_NORSRAM_TimingTypeDef *Timing, FMC_NORSRAM_TimingTypeDef *ExtTiming);
HAL_StatusTypeDef HAL_SRAM_DeInit(SRAM_HandleTypeDef *hsram);
void              HAL_SRAM_MspInit(SRAM_HandleTypeDef *hsram);
void              HAL_SRAM_MspDeInit(SRAM_HandleTypeDef *hsram);



 



 

 
HAL_StatusTypeDef HAL_SRAM_Read_8b(SRAM_HandleTypeDef *hsram, uint32_t *pAddress, uint8_t *pDstBuffer, uint32_t BufferSize);
HAL_StatusTypeDef HAL_SRAM_Write_8b(SRAM_HandleTypeDef *hsram, uint32_t *pAddress, uint8_t *pSrcBuffer, uint32_t BufferSize);
HAL_StatusTypeDef HAL_SRAM_Read_16b(SRAM_HandleTypeDef *hsram, uint32_t *pAddress, uint16_t *pDstBuffer, uint32_t BufferSize);
HAL_StatusTypeDef HAL_SRAM_Write_16b(SRAM_HandleTypeDef *hsram, uint32_t *pAddress, uint16_t *pSrcBuffer, uint32_t BufferSize);
HAL_StatusTypeDef HAL_SRAM_Read_32b(SRAM_HandleTypeDef *hsram, uint32_t *pAddress, uint32_t *pDstBuffer, uint32_t BufferSize);
HAL_StatusTypeDef HAL_SRAM_Write_32b(SRAM_HandleTypeDef *hsram, uint32_t *pAddress, uint32_t *pSrcBuffer, uint32_t BufferSize);
HAL_StatusTypeDef HAL_SRAM_Read_DMA(SRAM_HandleTypeDef *hsram, uint32_t *pAddress, uint32_t *pDstBuffer, uint32_t BufferSize);
HAL_StatusTypeDef HAL_SRAM_Write_DMA(SRAM_HandleTypeDef *hsram, uint32_t *pAddress, uint32_t *pSrcBuffer, uint32_t BufferSize);

void HAL_SRAM_DMA_XferCpltCallback(DMA_HandleTypeDef *hdma);
void HAL_SRAM_DMA_XferErrorCallback(DMA_HandleTypeDef *hdma);



 



 

 
HAL_StatusTypeDef HAL_SRAM_WriteOperation_Enable(SRAM_HandleTypeDef *hsram);
HAL_StatusTypeDef HAL_SRAM_WriteOperation_Disable(SRAM_HandleTypeDef *hsram);



 



 

 
HAL_SRAM_StateTypeDef HAL_SRAM_GetState(SRAM_HandleTypeDef *hsram);



 



 



 



 









 
#line 298 "..\\USER\\stm32l4xx_hal_conf.h"


#line 1 "..\\HALLIB\\STM32L4xx_HAL_Driver\\Inc\\stm32l4xx_hal_nor.h"

































 

 









 
#line 48 "..\\HALLIB\\STM32L4xx_HAL_Driver\\Inc\\stm32l4xx_hal_nor.h"




 



 

 


 



 
typedef enum
{
  HAL_NOR_STATE_RESET             = 0x00U,   
  HAL_NOR_STATE_READY             = 0x01U,   
  HAL_NOR_STATE_BUSY              = 0x02U,   
  HAL_NOR_STATE_ERROR             = 0x03U,   
  HAL_NOR_STATE_PROTECTED         = 0x04U    
}HAL_NOR_StateTypeDef;



 
typedef enum
{
  HAL_NOR_STATUS_SUCCESS  = 0U,
  HAL_NOR_STATUS_ONGOING,
  HAL_NOR_STATUS_ERROR,
  HAL_NOR_STATUS_TIMEOUT
}HAL_NOR_StatusTypeDef;



 
typedef struct
{
  uint16_t Manufacturer_Code;   

  uint16_t Device_Code1;

  uint16_t Device_Code2;

  uint16_t Device_Code3;       


 
}NOR_IDTypeDef;



 
typedef struct
{
  

 

  uint16_t CFI_1;

  uint16_t CFI_2;

  uint16_t CFI_3;

  uint16_t CFI_4;
}NOR_CFITypeDef;



 
typedef struct
{
  FMC_Bank1_TypeDef           *Instance;     

  FMC_Bank1E_TypeDef  *Extended;     

  FMC_NORSRAM_InitTypeDef       Init;          

  HAL_LockTypeDef               Lock;          

  volatile HAL_NOR_StateTypeDef     State;         
}NOR_HandleTypeDef;


 

 
 


 



 



 

 


 



 

 
HAL_StatusTypeDef HAL_NOR_Init(NOR_HandleTypeDef *hnor, FMC_NORSRAM_TimingTypeDef *Timing, FMC_NORSRAM_TimingTypeDef *ExtTiming);
HAL_StatusTypeDef HAL_NOR_DeInit(NOR_HandleTypeDef *hnor);
void HAL_NOR_MspInit(NOR_HandleTypeDef *hnor);
void HAL_NOR_MspDeInit(NOR_HandleTypeDef *hnor);
void HAL_NOR_MspWait(NOR_HandleTypeDef *hnor, uint32_t Timeout);


 



 

 
HAL_StatusTypeDef HAL_NOR_Read_ID(NOR_HandleTypeDef *hnor, NOR_IDTypeDef *pNOR_ID);
HAL_StatusTypeDef HAL_NOR_ReturnToReadMode(NOR_HandleTypeDef *hnor);
HAL_StatusTypeDef HAL_NOR_Read(NOR_HandleTypeDef *hnor, uint32_t *pAddress, uint16_t *pData);
HAL_StatusTypeDef HAL_NOR_Program(NOR_HandleTypeDef *hnor, uint32_t *pAddress, uint16_t *pData);

HAL_StatusTypeDef HAL_NOR_ReadBuffer(NOR_HandleTypeDef *hnor, uint32_t uwAddress, uint16_t *pData, uint32_t uwBufferSize);
HAL_StatusTypeDef HAL_NOR_ProgramBuffer(NOR_HandleTypeDef *hnor, uint32_t uwAddress, uint16_t *pData, uint32_t uwBufferSize);

HAL_StatusTypeDef HAL_NOR_Erase_Block(NOR_HandleTypeDef *hnor, uint32_t BlockAddress, uint32_t Address);
HAL_StatusTypeDef HAL_NOR_Erase_Chip(NOR_HandleTypeDef *hnor, uint32_t Address);
HAL_StatusTypeDef HAL_NOR_Read_CFI(NOR_HandleTypeDef *hnor, NOR_CFITypeDef *pNOR_CFI);


 



 

 
HAL_StatusTypeDef HAL_NOR_WriteOperation_Enable(NOR_HandleTypeDef *hnor);
HAL_StatusTypeDef HAL_NOR_WriteOperation_Disable(NOR_HandleTypeDef *hnor);


 



 

 
HAL_NOR_StateTypeDef HAL_NOR_GetState(NOR_HandleTypeDef *hnor);
HAL_NOR_StatusTypeDef HAL_NOR_GetStatus(NOR_HandleTypeDef *hnor, uint32_t Address, uint32_t Timeout);


 



 

 
 
 


 
 





 





 


 



 






 

 


 






 










 







 



 



 









 
#line 302 "..\\USER\\stm32l4xx_hal_conf.h"


#line 1 "..\\HALLIB\\STM32L4xx_HAL_Driver\\Inc\\stm32l4xx_hal_nand.h"

































 

 









 
#line 48 "..\\HALLIB\\STM32L4xx_HAL_Driver\\Inc\\stm32l4xx_hal_nand.h"



 



 

 
 


 



 
typedef enum
{
  HAL_NAND_STATE_RESET     = 0x00U,   
  HAL_NAND_STATE_READY     = 0x01U,   
  HAL_NAND_STATE_BUSY      = 0x02U,   
  HAL_NAND_STATE_ERROR     = 0x03U    
}HAL_NAND_StateTypeDef;



 
typedef struct
{
   
  uint8_t Maker_Id;

  uint8_t Device_Id;

  uint8_t Third_Id;

  uint8_t Fourth_Id;
}NAND_IDTypeDef;



 
typedef struct
{
  uint16_t Page;    

  uint16_t Plane;   

  uint16_t Block;   
}NAND_AddressTypeDef;



 
typedef struct
{
  uint32_t        PageSize;              
 

  uint32_t        SpareAreaSize;         
 

  uint32_t        BlockSize;              

  uint32_t        BlockNbr;               

  uint32_t        PlaneNbr;               

  uint32_t        PlaneSize;              

  FunctionalState ExtraCommandEnable;    




 
} NAND_DeviceConfigTypeDef;



 
typedef struct
{
  FMC_Bank3_TypeDef             *Instance;   

  FMC_NAND_InitTypeDef         Init;        

  HAL_LockTypeDef              Lock;        

  volatile HAL_NAND_StateTypeDef   State;       

  NAND_DeviceConfigTypeDef     Config;      

} NAND_HandleTypeDef;


 

 
 


 




 




 

 


 



 

 
HAL_StatusTypeDef  HAL_NAND_Init(NAND_HandleTypeDef *hnand, FMC_NAND_PCC_TimingTypeDef *ComSpace_Timing, FMC_NAND_PCC_TimingTypeDef *AttSpace_Timing);
HAL_StatusTypeDef  HAL_NAND_DeInit(NAND_HandleTypeDef *hnand);

HAL_StatusTypeDef  HAL_NAND_ConfigDevice(NAND_HandleTypeDef *hnand, NAND_DeviceConfigTypeDef *pDeviceConfig);

HAL_StatusTypeDef  HAL_NAND_Read_ID(NAND_HandleTypeDef *hnand, NAND_IDTypeDef *pNAND_ID);

void               HAL_NAND_MspInit(NAND_HandleTypeDef *hnand);
void               HAL_NAND_MspDeInit(NAND_HandleTypeDef *hnand);
void               HAL_NAND_IRQHandler(NAND_HandleTypeDef *hnand);
void               HAL_NAND_ITCallback(NAND_HandleTypeDef *hnand);



 



 

 

HAL_StatusTypeDef  HAL_NAND_Reset(NAND_HandleTypeDef *hnand);

HAL_StatusTypeDef  HAL_NAND_Read_Page_8b(NAND_HandleTypeDef *hnand, NAND_AddressTypeDef *pAddress, uint8_t *pBuffer, uint32_t NumPageToRead);
HAL_StatusTypeDef  HAL_NAND_Write_Page_8b(NAND_HandleTypeDef *hnand, NAND_AddressTypeDef *pAddress, uint8_t *pBuffer, uint32_t NumPageToWrite);
HAL_StatusTypeDef  HAL_NAND_Read_SpareArea_8b(NAND_HandleTypeDef *hnand, NAND_AddressTypeDef *pAddress, uint8_t *pBuffer, uint32_t NumSpareAreaToRead);
HAL_StatusTypeDef  HAL_NAND_Write_SpareArea_8b(NAND_HandleTypeDef *hnand, NAND_AddressTypeDef *pAddress, uint8_t *pBuffer, uint32_t NumSpareAreaTowrite);

HAL_StatusTypeDef  HAL_NAND_Read_Page_16b(NAND_HandleTypeDef *hnand, NAND_AddressTypeDef *pAddress, uint16_t *pBuffer, uint32_t NumPageToRead);
HAL_StatusTypeDef  HAL_NAND_Write_Page_16b(NAND_HandleTypeDef *hnand, NAND_AddressTypeDef *pAddress, uint16_t *pBuffer, uint32_t NumPageToWrite);
HAL_StatusTypeDef  HAL_NAND_Read_SpareArea_16b(NAND_HandleTypeDef *hnand, NAND_AddressTypeDef *pAddress, uint16_t *pBuffer, uint32_t NumSpareAreaToRead);
HAL_StatusTypeDef  HAL_NAND_Write_SpareArea_16b(NAND_HandleTypeDef *hnand, NAND_AddressTypeDef *pAddress, uint16_t *pBuffer, uint32_t NumSpareAreaTowrite);

HAL_StatusTypeDef  HAL_NAND_Erase_Block(NAND_HandleTypeDef *hnand, NAND_AddressTypeDef *pAddress);

uint32_t           HAL_NAND_Address_Inc(NAND_HandleTypeDef *hnand, NAND_AddressTypeDef *pAddress);



 



 

 
HAL_StatusTypeDef  HAL_NAND_ECC_Enable(NAND_HandleTypeDef *hnand);
HAL_StatusTypeDef  HAL_NAND_ECC_Disable(NAND_HandleTypeDef *hnand);
HAL_StatusTypeDef  HAL_NAND_GetECC(NAND_HandleTypeDef *hnand, uint32_t *ECCval, uint32_t Timeout);



 



 
 
HAL_NAND_StateTypeDef HAL_NAND_GetState(NAND_HandleTypeDef *hnand);
uint32_t              HAL_NAND_Read_Status(NAND_HandleTypeDef *hnand);


 



 
 
 
 


 











#line 267 "..\\HALLIB\\STM32L4xx_HAL_Driver\\Inc\\stm32l4xx_hal_nand.h"

 
#line 275 "..\\HALLIB\\STM32L4xx_HAL_Driver\\Inc\\stm32l4xx_hal_nand.h"


 

 


 






 









 









 





 



 



 



 









 
#line 306 "..\\USER\\stm32l4xx_hal_conf.h"


#line 1 "..\\HALLIB\\STM32L4xx_HAL_Driver\\Inc\\stm32l4xx_hal_i2c.h"

































 

 







 
#line 46 "..\\HALLIB\\STM32L4xx_HAL_Driver\\Inc\\stm32l4xx_hal_i2c.h"



 



 

 


 




 
typedef struct
{
  uint32_t Timing;              

 

  uint32_t OwnAddress1;         
 

  uint32_t AddressingMode;      
 

  uint32_t DualAddressMode;     
 

  uint32_t OwnAddress2;         
 

  uint32_t OwnAddress2Masks;    
 

  uint32_t GeneralCallMode;     
 

  uint32_t NoStretchMode;       
 

} I2C_InitTypeDef;



 



























 
typedef enum
{
  HAL_I2C_STATE_RESET             = 0x00U,    
  HAL_I2C_STATE_READY             = 0x20U,    
  HAL_I2C_STATE_BUSY              = 0x24U,    
  HAL_I2C_STATE_BUSY_TX           = 0x21U,    
  HAL_I2C_STATE_BUSY_RX           = 0x22U,    
  HAL_I2C_STATE_LISTEN            = 0x28U,    
  HAL_I2C_STATE_BUSY_TX_LISTEN    = 0x29U,   
 
  HAL_I2C_STATE_BUSY_RX_LISTEN    = 0x2AU,   
 
  HAL_I2C_STATE_ABORT             = 0x60U,    
  HAL_I2C_STATE_TIMEOUT           = 0xA0U,    
  HAL_I2C_STATE_ERROR             = 0xE0U     

} HAL_I2C_StateTypeDef;



 


















 
typedef enum
{
  HAL_I2C_MODE_NONE               = 0x00U,    
  HAL_I2C_MODE_MASTER             = 0x10U,    
  HAL_I2C_MODE_SLAVE              = 0x20U,    
  HAL_I2C_MODE_MEM                = 0x40U     

} HAL_I2C_ModeTypeDef;



 




 
#line 194 "..\\HALLIB\\STM32L4xx_HAL_Driver\\Inc\\stm32l4xx_hal_i2c.h"


 




 
typedef struct __I2C_HandleTypeDef
{
  I2C_TypeDef                *Instance;       

  I2C_InitTypeDef            Init;            

  uint8_t                    *pBuffPtr;       

  uint16_t                   XferSize;        

  volatile uint16_t              XferCount;       

  volatile uint32_t              XferOptions;    
 

  volatile uint32_t              PreviousState;   

  HAL_StatusTypeDef(*XferISR)(struct __I2C_HandleTypeDef *hi2c, uint32_t ITFlags, uint32_t ITSources);   

  DMA_HandleTypeDef          *hdmatx;         

  DMA_HandleTypeDef          *hdmarx;         

  HAL_LockTypeDef            Lock;            

  volatile HAL_I2C_StateTypeDef  State;           

  volatile HAL_I2C_ModeTypeDef   Mode;            

  volatile uint32_t              ErrorCode;       

  volatile uint32_t              AddrEventCount;  

#line 252 "..\\HALLIB\\STM32L4xx_HAL_Driver\\Inc\\stm32l4xx_hal_i2c.h"
} I2C_HandleTypeDef;

#line 282 "..\\HALLIB\\STM32L4xx_HAL_Driver\\Inc\\stm32l4xx_hal_i2c.h"


 



 
 



 



 
#line 304 "..\\HALLIB\\STM32L4xx_HAL_Driver\\Inc\\stm32l4xx_hal_i2c.h"



 




 



 




 



 




 



 
#line 343 "..\\HALLIB\\STM32L4xx_HAL_Driver\\Inc\\stm32l4xx_hal_i2c.h"


 



 




 



 




 



 




 



 




 



 





 



 






 






 
#line 417 "..\\HALLIB\\STM32L4xx_HAL_Driver\\Inc\\stm32l4xx_hal_i2c.h"


 



 
#line 440 "..\\HALLIB\\STM32L4xx_HAL_Driver\\Inc\\stm32l4xx_hal_i2c.h"


 



 

 



 




 
#line 467 "..\\HALLIB\\STM32L4xx_HAL_Driver\\Inc\\stm32l4xx_hal_i2c.h"














 















 















 
























 



















 






 





 





 



 

 
#line 1 "..\\HALLIB\\STM32L4xx_HAL_Driver\\Inc\\stm32l4xx_hal_i2c_ex.h"

































 

 







 
#line 46 "..\\HALLIB\\STM32L4xx_HAL_Driver\\Inc\\stm32l4xx_hal_i2c_ex.h"



 



 

 
 



 



 




 



 
#line 96 "..\\HALLIB\\STM32L4xx_HAL_Driver\\Inc\\stm32l4xx_hal_i2c_ex.h"


 



 

 
 



 




 

 
HAL_StatusTypeDef HAL_I2CEx_ConfigAnalogFilter(I2C_HandleTypeDef *hi2c, uint32_t AnalogFilter);
HAL_StatusTypeDef HAL_I2CEx_ConfigDigitalFilter(I2C_HandleTypeDef *hi2c, uint32_t DigitalFilter);
HAL_StatusTypeDef HAL_I2CEx_EnableWakeUp(I2C_HandleTypeDef *hi2c);
HAL_StatusTypeDef HAL_I2CEx_DisableWakeUp(I2C_HandleTypeDef *hi2c);
void HAL_I2CEx_EnableFastModePlus(uint32_t ConfigFastModePlus);
void HAL_I2CEx_DisableFastModePlus(uint32_t ConfigFastModePlus);

 


 



 

 


 





#line 151 "..\\HALLIB\\STM32L4xx_HAL_Driver\\Inc\\stm32l4xx_hal_i2c_ex.h"


 

 


 
 


 



 



 



 



 







 
#line 585 "..\\HALLIB\\STM32L4xx_HAL_Driver\\Inc\\stm32l4xx_hal_i2c.h"

 


 



 
 
HAL_StatusTypeDef HAL_I2C_Init(I2C_HandleTypeDef *hi2c);
HAL_StatusTypeDef HAL_I2C_DeInit(I2C_HandleTypeDef *hi2c);
void HAL_I2C_MspInit(I2C_HandleTypeDef *hi2c);
void HAL_I2C_MspDeInit(I2C_HandleTypeDef *hi2c);

 
#line 608 "..\\HALLIB\\STM32L4xx_HAL_Driver\\Inc\\stm32l4xx_hal_i2c.h"


 



 
 
 
HAL_StatusTypeDef HAL_I2C_Master_Transmit(I2C_HandleTypeDef *hi2c, uint16_t DevAddress, uint8_t *pData, uint16_t Size, uint32_t Timeout);
HAL_StatusTypeDef HAL_I2C_Master_Receive(I2C_HandleTypeDef *hi2c, uint16_t DevAddress, uint8_t *pData, uint16_t Size, uint32_t Timeout);
HAL_StatusTypeDef HAL_I2C_Slave_Transmit(I2C_HandleTypeDef *hi2c, uint8_t *pData, uint16_t Size, uint32_t Timeout);
HAL_StatusTypeDef HAL_I2C_Slave_Receive(I2C_HandleTypeDef *hi2c, uint8_t *pData, uint16_t Size, uint32_t Timeout);
HAL_StatusTypeDef HAL_I2C_Mem_Write(I2C_HandleTypeDef *hi2c, uint16_t DevAddress, uint16_t MemAddress, uint16_t MemAddSize, uint8_t *pData, uint16_t Size, uint32_t Timeout);
HAL_StatusTypeDef HAL_I2C_Mem_Read(I2C_HandleTypeDef *hi2c, uint16_t DevAddress, uint16_t MemAddress, uint16_t MemAddSize, uint8_t *pData, uint16_t Size, uint32_t Timeout);
HAL_StatusTypeDef HAL_I2C_IsDeviceReady(I2C_HandleTypeDef *hi2c, uint16_t DevAddress, uint32_t Trials, uint32_t Timeout);

 
HAL_StatusTypeDef HAL_I2C_Master_Transmit_IT(I2C_HandleTypeDef *hi2c, uint16_t DevAddress, uint8_t *pData, uint16_t Size);
HAL_StatusTypeDef HAL_I2C_Master_Receive_IT(I2C_HandleTypeDef *hi2c, uint16_t DevAddress, uint8_t *pData, uint16_t Size);
HAL_StatusTypeDef HAL_I2C_Slave_Transmit_IT(I2C_HandleTypeDef *hi2c, uint8_t *pData, uint16_t Size);
HAL_StatusTypeDef HAL_I2C_Slave_Receive_IT(I2C_HandleTypeDef *hi2c, uint8_t *pData, uint16_t Size);
HAL_StatusTypeDef HAL_I2C_Mem_Write_IT(I2C_HandleTypeDef *hi2c, uint16_t DevAddress, uint16_t MemAddress, uint16_t MemAddSize, uint8_t *pData, uint16_t Size);
HAL_StatusTypeDef HAL_I2C_Mem_Read_IT(I2C_HandleTypeDef *hi2c, uint16_t DevAddress, uint16_t MemAddress, uint16_t MemAddSize, uint8_t *pData, uint16_t Size);

HAL_StatusTypeDef HAL_I2C_Master_Sequential_Transmit_IT(I2C_HandleTypeDef *hi2c, uint16_t DevAddress, uint8_t *pData, uint16_t Size, uint32_t XferOptions);
HAL_StatusTypeDef HAL_I2C_Master_Sequential_Receive_IT(I2C_HandleTypeDef *hi2c, uint16_t DevAddress, uint8_t *pData, uint16_t Size, uint32_t XferOptions);
HAL_StatusTypeDef HAL_I2C_Slave_Sequential_Transmit_IT(I2C_HandleTypeDef *hi2c, uint8_t *pData, uint16_t Size, uint32_t XferOptions);
HAL_StatusTypeDef HAL_I2C_Slave_Sequential_Receive_IT(I2C_HandleTypeDef *hi2c, uint8_t *pData, uint16_t Size, uint32_t XferOptions);
HAL_StatusTypeDef HAL_I2C_EnableListen_IT(I2C_HandleTypeDef *hi2c);
HAL_StatusTypeDef HAL_I2C_DisableListen_IT(I2C_HandleTypeDef *hi2c);
HAL_StatusTypeDef HAL_I2C_Master_Abort_IT(I2C_HandleTypeDef *hi2c, uint16_t DevAddress);

 
HAL_StatusTypeDef HAL_I2C_Master_Transmit_DMA(I2C_HandleTypeDef *hi2c, uint16_t DevAddress, uint8_t *pData, uint16_t Size);
HAL_StatusTypeDef HAL_I2C_Master_Receive_DMA(I2C_HandleTypeDef *hi2c, uint16_t DevAddress, uint8_t *pData, uint16_t Size);
HAL_StatusTypeDef HAL_I2C_Slave_Transmit_DMA(I2C_HandleTypeDef *hi2c, uint8_t *pData, uint16_t Size);
HAL_StatusTypeDef HAL_I2C_Slave_Receive_DMA(I2C_HandleTypeDef *hi2c, uint8_t *pData, uint16_t Size);
HAL_StatusTypeDef HAL_I2C_Mem_Write_DMA(I2C_HandleTypeDef *hi2c, uint16_t DevAddress, uint16_t MemAddress, uint16_t MemAddSize, uint8_t *pData, uint16_t Size);
HAL_StatusTypeDef HAL_I2C_Mem_Read_DMA(I2C_HandleTypeDef *hi2c, uint16_t DevAddress, uint16_t MemAddress, uint16_t MemAddSize, uint8_t *pData, uint16_t Size);

HAL_StatusTypeDef HAL_I2C_Master_Sequential_Transmit_DMA(I2C_HandleTypeDef *hi2c, uint16_t DevAddress, uint8_t *pData, uint16_t Size, uint32_t XferOptions);
HAL_StatusTypeDef HAL_I2C_Master_Sequential_Receive_DMA(I2C_HandleTypeDef *hi2c, uint16_t DevAddress, uint8_t *pData, uint16_t Size, uint32_t XferOptions);
HAL_StatusTypeDef HAL_I2C_Slave_Sequential_Transmit_DMA(I2C_HandleTypeDef *hi2c, uint8_t *pData, uint16_t Size, uint32_t XferOptions);
HAL_StatusTypeDef HAL_I2C_Slave_Sequential_Receive_DMA(I2C_HandleTypeDef *hi2c, uint8_t *pData, uint16_t Size, uint32_t XferOptions);


 



 
 
void HAL_I2C_EV_IRQHandler(I2C_HandleTypeDef *hi2c);
void HAL_I2C_ER_IRQHandler(I2C_HandleTypeDef *hi2c);
void HAL_I2C_MasterTxCpltCallback(I2C_HandleTypeDef *hi2c);
void HAL_I2C_MasterRxCpltCallback(I2C_HandleTypeDef *hi2c);
void HAL_I2C_SlaveTxCpltCallback(I2C_HandleTypeDef *hi2c);
void HAL_I2C_SlaveRxCpltCallback(I2C_HandleTypeDef *hi2c);
void HAL_I2C_AddrCallback(I2C_HandleTypeDef *hi2c, uint8_t TransferDirection, uint16_t AddrMatchCode);
void HAL_I2C_ListenCpltCallback(I2C_HandleTypeDef *hi2c);
void HAL_I2C_MemTxCpltCallback(I2C_HandleTypeDef *hi2c);
void HAL_I2C_MemRxCpltCallback(I2C_HandleTypeDef *hi2c);
void HAL_I2C_ErrorCallback(I2C_HandleTypeDef *hi2c);
void HAL_I2C_AbortCpltCallback(I2C_HandleTypeDef *hi2c);


 



 
 
HAL_I2C_StateTypeDef HAL_I2C_GetState(I2C_HandleTypeDef *hi2c);
HAL_I2C_ModeTypeDef  HAL_I2C_GetMode(I2C_HandleTypeDef *hi2c);
uint32_t             HAL_I2C_GetError(I2C_HandleTypeDef *hi2c);



 



 

 


 



 

 


 







#line 721 "..\\HALLIB\\STM32L4xx_HAL_Driver\\Inc\\stm32l4xx_hal_i2c.h"



















#line 747 "..\\HALLIB\\STM32L4xx_HAL_Driver\\Inc\\stm32l4xx_hal_i2c.h"

























 

 


 
 


 



 



 








 
#line 310 "..\\USER\\stm32l4xx_hal_conf.h"


#line 1 "..\\HALLIB\\STM32L4xx_HAL_Driver\\Inc\\stm32l4xx_hal_iwdg.h"

































 

 







 
#line 46 "..\\HALLIB\\STM32L4xx_HAL_Driver\\Inc\\stm32l4xx_hal_iwdg.h"



 



 

 


 



 
typedef struct
{
  uint32_t Prescaler;  
 

  uint32_t Reload;     
 

  uint32_t Window;     
 

} IWDG_InitTypeDef;



 
typedef struct
{
  IWDG_TypeDef                 *Instance;   

  IWDG_InitTypeDef             Init;        

}IWDG_HandleTypeDef;



 

 


 



 
#line 106 "..\\HALLIB\\STM32L4xx_HAL_Driver\\Inc\\stm32l4xx_hal_iwdg.h"


 



 



 



 

 


 





 







 




 

 


 



 
 
HAL_StatusTypeDef HAL_IWDG_Init(IWDG_HandleTypeDef *hiwdg);


 



 
 
HAL_StatusTypeDef HAL_IWDG_Refresh(IWDG_HandleTypeDef *hiwdg);


 



 

 


 



 







 

 


 





 






 






 
#line 221 "..\\HALLIB\\STM32L4xx_HAL_Driver\\Inc\\stm32l4xx_hal_iwdg.h"





 






 




 



 



 








 
#line 314 "..\\USER\\stm32l4xx_hal_conf.h"


#line 1 "..\\HALLIB\\STM32L4xx_HAL_Driver\\Inc\\stm32l4xx_hal_lcd.h"

































 

 







#line 780 "..\\HALLIB\\STM32L4xx_HAL_Driver\\Inc\\stm32l4xx_hal_lcd.h"







 
#line 318 "..\\USER\\stm32l4xx_hal_conf.h"


#line 1 "..\\HALLIB\\STM32L4xx_HAL_Driver\\Inc\\stm32l4xx_hal_lptim.h"

































 

 







 
#line 46 "..\\HALLIB\\STM32L4xx_HAL_Driver\\Inc\\stm32l4xx_hal_lptim.h"



 



 

 


 



 
typedef struct
{
  uint32_t Source;         
 

  uint32_t Prescaler;      
 

}LPTIM_ClockConfigTypeDef;



 
typedef struct
{
  uint32_t Polarity;      




 

  uint32_t SampleTime;     

 

}LPTIM_ULPClockConfigTypeDef;



 
typedef struct
{
  uint32_t Source;        
 

  uint32_t ActiveEdge;    

 

  uint32_t SampleTime;    

 
}LPTIM_TriggerConfigTypeDef;



 
typedef struct
{
  LPTIM_ClockConfigTypeDef     Clock;                

  LPTIM_ULPClockConfigTypeDef  UltraLowPowerClock;   

  LPTIM_TriggerConfigTypeDef   Trigger;              

  uint32_t                     OutputPolarity;      
 

  uint32_t                     UpdateMode;          

 

  uint32_t                     CounterSource;       

 

  uint32_t                     Input1Source;        
 

  uint32_t                     Input2Source;        


 

#line 146 "..\\HALLIB\\STM32L4xx_HAL_Driver\\Inc\\stm32l4xx_hal_lptim.h"

}LPTIM_InitTypeDef;



 
typedef enum __HAL_LPTIM_StateTypeDef
{
  HAL_LPTIM_STATE_RESET            = 0x00,     
  HAL_LPTIM_STATE_READY            = 0x01,     
  HAL_LPTIM_STATE_BUSY             = 0x02,     
  HAL_LPTIM_STATE_TIMEOUT          = 0x03,     
  HAL_LPTIM_STATE_ERROR            = 0x04      
}HAL_LPTIM_StateTypeDef;



 
typedef struct __LPTIM_HandleTypeDef
{
      LPTIM_TypeDef              *Instance;          

      LPTIM_InitTypeDef           Init;              

      HAL_StatusTypeDef           Status;            

      HAL_LockTypeDef             Lock;              

   volatile  HAL_LPTIM_StateTypeDef   State;             

#line 191 "..\\HALLIB\\STM32L4xx_HAL_Driver\\Inc\\stm32l4xx_hal_lptim.h"
}LPTIM_HandleTypeDef;

#line 220 "..\\HALLIB\\STM32L4xx_HAL_Driver\\Inc\\stm32l4xx_hal_lptim.h"


 

 


 



 




 



 
#line 249 "..\\HALLIB\\STM32L4xx_HAL_Driver\\Inc\\stm32l4xx_hal_lptim.h"


 



 





 



 






 



 





 



 
#line 296 "..\\HALLIB\\STM32L4xx_HAL_Driver\\Inc\\stm32l4xx_hal_lptim.h"


 



 





 



 






 



 





 



 





 



 







 



 





 



 
#line 377 "..\\HALLIB\\STM32L4xx_HAL_Driver\\Inc\\stm32l4xx_hal_lptim.h"


 



 
#line 395 "..\\HALLIB\\STM32L4xx_HAL_Driver\\Inc\\stm32l4xx_hal_lptim.h"


 



 

 


 




 
#line 421 "..\\HALLIB\\STM32L4xx_HAL_Driver\\Inc\\stm32l4xx_hal_lptim.h"





 






 






 









 







 


#line 481 "..\\HALLIB\\STM32L4xx_HAL_Driver\\Inc\\stm32l4xx_hal_lptim.h"
















 

















 

















 


 












 


    












 





 

 


 

 
HAL_StatusTypeDef HAL_LPTIM_Init(LPTIM_HandleTypeDef *hlptim);
HAL_StatusTypeDef HAL_LPTIM_DeInit(LPTIM_HandleTypeDef *hlptim);

 
void HAL_LPTIM_MspInit(LPTIM_HandleTypeDef *hlptim);
void HAL_LPTIM_MspDeInit(LPTIM_HandleTypeDef *hlptim);

 
 
 
HAL_StatusTypeDef HAL_LPTIM_PWM_Start(LPTIM_HandleTypeDef *hlptim, uint32_t Period, uint32_t Pulse);
HAL_StatusTypeDef HAL_LPTIM_PWM_Stop(LPTIM_HandleTypeDef *hlptim);
 
HAL_StatusTypeDef HAL_LPTIM_PWM_Start_IT(LPTIM_HandleTypeDef *hlptim, uint32_t Period, uint32_t Pulse);
HAL_StatusTypeDef HAL_LPTIM_PWM_Stop_IT(LPTIM_HandleTypeDef *hlptim);

 
 
HAL_StatusTypeDef HAL_LPTIM_OnePulse_Start(LPTIM_HandleTypeDef *hlptim, uint32_t Period, uint32_t Pulse);
HAL_StatusTypeDef HAL_LPTIM_OnePulse_Stop(LPTIM_HandleTypeDef *hlptim);
 
HAL_StatusTypeDef HAL_LPTIM_OnePulse_Start_IT(LPTIM_HandleTypeDef *hlptim, uint32_t Period, uint32_t Pulse);
HAL_StatusTypeDef HAL_LPTIM_OnePulse_Stop_IT(LPTIM_HandleTypeDef *hlptim);

 
 
HAL_StatusTypeDef HAL_LPTIM_SetOnce_Start(LPTIM_HandleTypeDef *hlptim, uint32_t Period, uint32_t Pulse);
HAL_StatusTypeDef HAL_LPTIM_SetOnce_Stop(LPTIM_HandleTypeDef *hlptim);
 
HAL_StatusTypeDef HAL_LPTIM_SetOnce_Start_IT(LPTIM_HandleTypeDef *hlptim, uint32_t Period, uint32_t Pulse);
HAL_StatusTypeDef HAL_LPTIM_SetOnce_Stop_IT(LPTIM_HandleTypeDef *hlptim);

 
 
HAL_StatusTypeDef HAL_LPTIM_Encoder_Start(LPTIM_HandleTypeDef *hlptim, uint32_t Period);
HAL_StatusTypeDef HAL_LPTIM_Encoder_Stop(LPTIM_HandleTypeDef *hlptim);
 
HAL_StatusTypeDef HAL_LPTIM_Encoder_Start_IT(LPTIM_HandleTypeDef *hlptim, uint32_t Period);
HAL_StatusTypeDef HAL_LPTIM_Encoder_Stop_IT(LPTIM_HandleTypeDef *hlptim);

 
 
HAL_StatusTypeDef HAL_LPTIM_TimeOut_Start(LPTIM_HandleTypeDef *hlptim, uint32_t Period, uint32_t Timeout);
HAL_StatusTypeDef HAL_LPTIM_TimeOut_Stop(LPTIM_HandleTypeDef *hlptim);
 
HAL_StatusTypeDef HAL_LPTIM_TimeOut_Start_IT(LPTIM_HandleTypeDef *hlptim, uint32_t Period, uint32_t Timeout);
HAL_StatusTypeDef HAL_LPTIM_TimeOut_Stop_IT(LPTIM_HandleTypeDef *hlptim);

 
 
HAL_StatusTypeDef HAL_LPTIM_Counter_Start(LPTIM_HandleTypeDef *hlptim, uint32_t Period);
HAL_StatusTypeDef HAL_LPTIM_Counter_Stop(LPTIM_HandleTypeDef *hlptim);
 
HAL_StatusTypeDef HAL_LPTIM_Counter_Start_IT(LPTIM_HandleTypeDef *hlptim, uint32_t Period);
HAL_StatusTypeDef HAL_LPTIM_Counter_Stop_IT(LPTIM_HandleTypeDef *hlptim);

 
uint32_t HAL_LPTIM_ReadCounter(LPTIM_HandleTypeDef *hlptim);
uint32_t HAL_LPTIM_ReadAutoReload(LPTIM_HandleTypeDef *hlptim);
uint32_t HAL_LPTIM_ReadCompare(LPTIM_HandleTypeDef *hlptim);

 
void HAL_LPTIM_IRQHandler(LPTIM_HandleTypeDef *hlptim);

 
void HAL_LPTIM_CompareMatchCallback(LPTIM_HandleTypeDef *hlptim);
void HAL_LPTIM_AutoReloadMatchCallback(LPTIM_HandleTypeDef *hlptim);
void HAL_LPTIM_TriggerCallback(LPTIM_HandleTypeDef *hlptim);
void HAL_LPTIM_CompareWriteCallback(LPTIM_HandleTypeDef *hlptim);
void HAL_LPTIM_AutoReloadWriteCallback(LPTIM_HandleTypeDef *hlptim);
void HAL_LPTIM_DirectionUpCallback(LPTIM_HandleTypeDef *hlptim);
void HAL_LPTIM_DirectionDownCallback(LPTIM_HandleTypeDef *hlptim);





 





 
HAL_LPTIM_StateTypeDef HAL_LPTIM_GetState(LPTIM_HandleTypeDef *hlptim);



 

 


 



 

 


 



 

 


 



 

 


 





#line 713 "..\\HALLIB\\STM32L4xx_HAL_Driver\\Inc\\stm32l4xx_hal_lptim.h"















#line 737 "..\\HALLIB\\STM32L4xx_HAL_Driver\\Inc\\stm32l4xx_hal_lptim.h"




























#line 775 "..\\HALLIB\\STM32L4xx_HAL_Driver\\Inc\\stm32l4xx_hal_lptim.h"








 

 


 



 



 



 







 
#line 322 "..\\USER\\stm32l4xx_hal_conf.h"


#line 1 "..\\HALLIB\\STM32L4xx_HAL_Driver\\Inc\\stm32l4xx_hal_ltdc.h"

































 

 







#line 694 "..\\HALLIB\\STM32L4xx_HAL_Driver\\Inc\\stm32l4xx_hal_ltdc.h"







 
#line 326 "..\\USER\\stm32l4xx_hal_conf.h"


#line 1 "..\\HALLIB\\STM32L4xx_HAL_Driver\\Inc\\stm32l4xx_hal_opamp.h"

































 

 







 
#line 46 "..\\HALLIB\\STM32L4xx_HAL_Driver\\Inc\\stm32l4xx_hal_opamp.h"



 



 

 



 



 

typedef struct
{
  uint32_t PowerSupplyRange;            

 

  uint32_t PowerMode;                   
 

  uint32_t Mode;                        

 

  uint32_t InvertingInput;              




 

  uint32_t NonInvertingInput;           
 

  uint32_t PgaGain;                     

 

  uint32_t UserTrimming;                

 

  uint32_t TrimmingValueP;              


 

  uint32_t TrimmingValueN;              


 

  uint32_t TrimmingValuePLowPower;      


 

  uint32_t TrimmingValueNLowPower;      


 

}OPAMP_InitTypeDef;



 

typedef enum
{
  HAL_OPAMP_STATE_RESET               = 0x00000000,  

  HAL_OPAMP_STATE_READY               = 0x00000001,  
  HAL_OPAMP_STATE_CALIBBUSY           = 0x00000002,  

  HAL_OPAMP_STATE_BUSY                = 0x00000004,  
  HAL_OPAMP_STATE_BUSYLOCKED          = 0x00000005  
 

}HAL_OPAMP_StateTypeDef;



 



typedef struct

{
  OPAMP_TypeDef       *Instance;                     
  OPAMP_InitTypeDef   Init;                          
  HAL_StatusTypeDef Status;                          
  HAL_LockTypeDef   Lock;                            
  volatile HAL_OPAMP_StateTypeDef  State;                






}OPAMP_HandleTypeDef;



 

typedef  uint32_t HAL_OPAMP_TrimmingValueTypeDef;



 

#line 183 "..\\HALLIB\\STM32L4xx_HAL_Driver\\Inc\\stm32l4xx_hal_opamp.h"


 



 



 






 



 






 



 







 



 








 



 





 



 





 



 





 



 






 

 

 

 



 

 


 
 
 
 




 

 


 




 
#line 320 "..\\HALLIB\\STM32L4xx_HAL_Driver\\Inc\\stm32l4xx_hal_opamp.h"





 

 



 





#line 343 "..\\HALLIB\\STM32L4xx_HAL_Driver\\Inc\\stm32l4xx_hal_opamp.h"
        
        






        
        





#line 366 "..\\HALLIB\\STM32L4xx_HAL_Driver\\Inc\\stm32l4xx_hal_opamp.h"
        
        
        
        

#line 378 "..\\HALLIB\\STM32L4xx_HAL_Driver\\Inc\\stm32l4xx_hal_opamp.h"
        
        

#line 387 "..\\HALLIB\\STM32L4xx_HAL_Driver\\Inc\\stm32l4xx_hal_opamp.h"
        
        























 

 
#line 1 "..\\HALLIB\\STM32L4xx_HAL_Driver\\Inc\\stm32l4xx_hal_opamp_ex.h"

































 

 







 
#line 46 "..\\HALLIB\\STM32L4xx_HAL_Driver\\Inc\\stm32l4xx_hal_opamp_ex.h"



 



 
 
 
 
 


 






 


 

HAL_StatusTypeDef HAL_OPAMPEx_SelfCalibrateAll(OPAMP_HandleTypeDef *hopamp1, OPAMP_HandleTypeDef *hopamp2);



 


 


 
HAL_StatusTypeDef HAL_OPAMPEx_Unlock(OPAMP_HandleTypeDef *hopamp);


 



 



 



 







 
#line 416 "..\\HALLIB\\STM32L4xx_HAL_Driver\\Inc\\stm32l4xx_hal_opamp.h"

 


 



 
 
HAL_StatusTypeDef HAL_OPAMP_Init(OPAMP_HandleTypeDef *hopamp);
HAL_StatusTypeDef HAL_OPAMP_DeInit (OPAMP_HandleTypeDef *hopamp);
void HAL_OPAMP_MspInit(OPAMP_HandleTypeDef *hopamp);
void HAL_OPAMP_MspDeInit(OPAMP_HandleTypeDef *hopamp);


 



 

 
HAL_StatusTypeDef HAL_OPAMP_Start(OPAMP_HandleTypeDef *hopamp);
HAL_StatusTypeDef HAL_OPAMP_Stop(OPAMP_HandleTypeDef *hopamp);
HAL_StatusTypeDef HAL_OPAMP_SelfCalibrate(OPAMP_HandleTypeDef *hopamp);



 



 

 






HAL_StatusTypeDef HAL_OPAMP_Lock(OPAMP_HandleTypeDef *hopamp);
HAL_OPAMP_TrimmingValueTypeDef HAL_OPAMP_GetTrimOffset (OPAMP_HandleTypeDef *hopamp, uint32_t trimmingoffset);



 



 

 
HAL_OPAMP_StateTypeDef HAL_OPAMP_GetState(OPAMP_HandleTypeDef *hopamp);



 



 



 



 







 
#line 330 "..\\USER\\stm32l4xx_hal_conf.h"


#line 1 "..\\HALLIB\\STM32L4xx_HAL_Driver\\Inc\\stm32l4xx_hal_ospi.h"

































 

 







 
#line 46 "..\\HALLIB\\STM32L4xx_HAL_Driver\\Inc\\stm32l4xx_hal_ospi.h"

#line 1030 "..\\HALLIB\\STM32L4xx_HAL_Driver\\Inc\\stm32l4xx_hal_ospi.h"







 
#line 334 "..\\USER\\stm32l4xx_hal_conf.h"


#line 1 "..\\HALLIB\\STM32L4xx_HAL_Driver\\Inc\\stm32l4xx_hal_pwr.h"

































 

 







 
#line 46 "..\\HALLIB\\STM32L4xx_HAL_Driver\\Inc\\stm32l4xx_hal_pwr.h"



 



 

 



 



 
typedef struct
{
  uint32_t PVDLevel;   
 

  uint32_t Mode;      
 
}PWR_PVDTypeDef;




 

 



 




 
#line 96 "..\\HALLIB\\STM32L4xx_HAL_Driver\\Inc\\stm32l4xx_hal_pwr.h"


 



 
#line 110 "..\\HALLIB\\STM32L4xx_HAL_Driver\\Inc\\stm32l4xx_hal_pwr.h"


 






 




 



 




 



 




 




 



 



 



 



 

 


 









































 





















 






 





 





 





 





 





 





 






 






 









 









 





 





 




 


 


 






#line 346 "..\\HALLIB\\STM32L4xx_HAL_Driver\\Inc\\stm32l4xx_hal_pwr.h"










 

 
#line 1 "..\\HALLIB\\STM32L4xx_HAL_Driver\\Inc\\stm32l4xx_hal_pwr_ex.h"

































 

 







 
#line 46 "..\\HALLIB\\STM32L4xx_HAL_Driver\\Inc\\stm32l4xx_hal_pwr_ex.h"



 



 


 



 




 
typedef struct
{
  uint32_t PVMType;   






 

  uint32_t Mode;      
 
}PWR_PVMTypeDef;



 

 



 



 



 




 
#line 118 "..\\HALLIB\\STM32L4xx_HAL_Driver\\Inc\\stm32l4xx_hal_pwr_ex.h"


 



 
#line 133 "..\\HALLIB\\STM32L4xx_HAL_Driver\\Inc\\stm32l4xx_hal_pwr_ex.h"


 



 
#line 147 "..\\HALLIB\\STM32L4xx_HAL_Driver\\Inc\\stm32l4xx_hal_pwr_ex.h"


 





 







 




 




 



 




 



 
#line 203 "..\\HALLIB\\STM32L4xx_HAL_Driver\\Inc\\stm32l4xx_hal_pwr_ex.h"


 



 
#line 229 "..\\HALLIB\\STM32L4xx_HAL_Driver\\Inc\\stm32l4xx_hal_pwr_ex.h"


 



 
#line 244 "..\\HALLIB\\STM32L4xx_HAL_Driver\\Inc\\stm32l4xx_hal_pwr_ex.h"


 



 
#line 259 "..\\HALLIB\\STM32L4xx_HAL_Driver\\Inc\\stm32l4xx_hal_pwr_ex.h"


 










 
#line 284 "..\\HALLIB\\STM32L4xx_HAL_Driver\\Inc\\stm32l4xx_hal_pwr_ex.h"

#line 297 "..\\HALLIB\\STM32L4xx_HAL_Driver\\Inc\\stm32l4xx_hal_pwr_ex.h"


 



 

 


 





 





 





 





 





 





 





 






 






 









 









 





 





 









 





 





 





 





 





 





 






 






 









 









 





 





 








 





 





 





 





 





 





 






 






 









 









 





 





 








 





 





 





 





 





 





 






 






 









 









 





 





 


















 
#line 697 "..\\HALLIB\\STM32L4xx_HAL_Driver\\Inc\\stm32l4xx_hal_pwr_ex.h"



 

 


 

#line 722 "..\\HALLIB\\STM32L4xx_HAL_Driver\\Inc\\stm32l4xx_hal_pwr_ex.h"

#line 735 "..\\HALLIB\\STM32L4xx_HAL_Driver\\Inc\\stm32l4xx_hal_pwr_ex.h"

#line 744 "..\\HALLIB\\STM32L4xx_HAL_Driver\\Inc\\stm32l4xx_hal_pwr_ex.h"

#line 752 "..\\HALLIB\\STM32L4xx_HAL_Driver\\Inc\\stm32l4xx_hal_pwr_ex.h"

#line 761 "..\\HALLIB\\STM32L4xx_HAL_Driver\\Inc\\stm32l4xx_hal_pwr_ex.h"











#line 812 "..\\HALLIB\\STM32L4xx_HAL_Driver\\Inc\\stm32l4xx_hal_pwr_ex.h"




 




 



 


 
uint32_t HAL_PWREx_GetVoltageRange(void);
HAL_StatusTypeDef HAL_PWREx_ControlVoltageScaling(uint32_t VoltageScaling);
void HAL_PWREx_EnableBatteryCharging(uint32_t ResistorSelection);
void HAL_PWREx_DisableBatteryCharging(void);

void HAL_PWREx_EnableVddUSB(void);
void HAL_PWREx_DisableVddUSB(void);


void HAL_PWREx_EnableVddIO2(void);
void HAL_PWREx_DisableVddIO2(void);

void HAL_PWREx_EnableInternalWakeUpLine(void);
void HAL_PWREx_DisableInternalWakeUpLine(void);
HAL_StatusTypeDef HAL_PWREx_EnableGPIOPullUp(uint32_t GPIO, uint32_t GPIONumber);
HAL_StatusTypeDef HAL_PWREx_DisableGPIOPullUp(uint32_t GPIO, uint32_t GPIONumber);
HAL_StatusTypeDef HAL_PWREx_EnableGPIOPullDown(uint32_t GPIO, uint32_t GPIONumber);
HAL_StatusTypeDef HAL_PWREx_DisableGPIOPullDown(uint32_t GPIO, uint32_t GPIONumber);
void HAL_PWREx_EnablePullUpPullDownConfig(void);
void HAL_PWREx_DisablePullUpPullDownConfig(void);
void HAL_PWREx_EnableSRAM2ContentRetention(void);
void HAL_PWREx_DisableSRAM2ContentRetention(void);
#line 860 "..\\HALLIB\\STM32L4xx_HAL_Driver\\Inc\\stm32l4xx_hal_pwr_ex.h"
void HAL_PWREx_EnablePVM1(void);
void HAL_PWREx_DisablePVM1(void);


void HAL_PWREx_EnablePVM2(void);
void HAL_PWREx_DisablePVM2(void);

void HAL_PWREx_EnablePVM3(void);
void HAL_PWREx_DisablePVM3(void);
void HAL_PWREx_EnablePVM4(void);
void HAL_PWREx_DisablePVM4(void);
HAL_StatusTypeDef HAL_PWREx_ConfigPVM(PWR_PVMTypeDef *sConfigPVM);
#line 880 "..\\HALLIB\\STM32L4xx_HAL_Driver\\Inc\\stm32l4xx_hal_pwr_ex.h"


 
void HAL_PWREx_EnableLowPowerRunMode(void);
HAL_StatusTypeDef HAL_PWREx_DisableLowPowerRunMode(void);
void HAL_PWREx_EnterSTOP0Mode(uint8_t STOPEntry);
void HAL_PWREx_EnterSTOP1Mode(uint8_t STOPEntry);
void HAL_PWREx_EnterSTOP2Mode(uint8_t STOPEntry);
void HAL_PWREx_EnterSHUTDOWNMode(void);

void HAL_PWREx_PVD_PVM_IRQHandler(void);

void HAL_PWREx_PVM1Callback(void);


void HAL_PWREx_PVM2Callback(void);

void HAL_PWREx_PVM3Callback(void);
void HAL_PWREx_PVM4Callback(void);



 



 



 



 








 
#line 360 "..\\HALLIB\\STM32L4xx_HAL_Driver\\Inc\\stm32l4xx_hal_pwr.h"

 



 



 

 
void HAL_PWR_DeInit(void);
void HAL_PWR_EnableBkUpAccess(void);
void HAL_PWR_DisableBkUpAccess(void);



 



 

 
HAL_StatusTypeDef HAL_PWR_ConfigPVD(PWR_PVDTypeDef *sConfigPVD);
void HAL_PWR_EnablePVD(void);
void HAL_PWR_DisablePVD(void);


 
void HAL_PWR_EnableWakeUpPin(uint32_t WakeUpPinPolarity);
void HAL_PWR_DisableWakeUpPin(uint32_t WakeUpPinx);

 
void HAL_PWR_EnterSLEEPMode(uint32_t Regulator, uint8_t SLEEPEntry);
void HAL_PWR_EnterSTOPMode(uint32_t Regulator, uint8_t STOPEntry);
void HAL_PWR_EnterSTANDBYMode(void);

void HAL_PWR_EnableSleepOnExit(void);
void HAL_PWR_DisableSleepOnExit(void);
void HAL_PWR_EnableSEVOnPend(void);
void HAL_PWR_DisableSEVOnPend(void);

void HAL_PWR_PVDCallback(void);




 



 



 



 








 
#line 338 "..\\USER\\stm32l4xx_hal_conf.h"


#line 1 "..\\HALLIB\\STM32L4xx_HAL_Driver\\Inc\\stm32l4xx_hal_qspi.h"

































 

 







 
#line 46 "..\\HALLIB\\STM32L4xx_HAL_Driver\\Inc\\stm32l4xx_hal_qspi.h"





 



 

 


 



 
typedef struct
{
  uint32_t ClockPrescaler;     
 
  uint32_t FifoThreshold;      
 
  uint32_t SampleShifting;     

 
  uint32_t FlashSize;          



 
  uint32_t ChipSelectHighTime; 

 
  uint32_t ClockMode;          
 
#line 90 "..\\HALLIB\\STM32L4xx_HAL_Driver\\Inc\\stm32l4xx_hal_qspi.h"
}QSPI_InitTypeDef;



 
typedef enum
{
  HAL_QSPI_STATE_RESET             = 0x00U,     
  HAL_QSPI_STATE_READY             = 0x01U,     
  HAL_QSPI_STATE_BUSY              = 0x02U,     
  HAL_QSPI_STATE_BUSY_INDIRECT_TX  = 0x12U,     
  HAL_QSPI_STATE_BUSY_INDIRECT_RX  = 0x22U,     
  HAL_QSPI_STATE_BUSY_AUTO_POLLING = 0x42U,     
  HAL_QSPI_STATE_BUSY_MEM_MAPPED   = 0x82U,     
  HAL_QSPI_STATE_ABORT             = 0x08U,     
  HAL_QSPI_STATE_ERROR             = 0x04U      
}HAL_QSPI_StateTypeDef;



 
typedef struct __QSPI_HandleTypeDef
{
  QUADSPI_TypeDef            *Instance;         
  QSPI_InitTypeDef           Init;              
  uint8_t                    *pTxBuffPtr;       
  volatile uint32_t              TxXferSize;        
  volatile uint32_t              TxXferCount;       
  uint8_t                    *pRxBuffPtr;       
  volatile uint32_t              RxXferSize;        
  volatile uint32_t              RxXferCount;       
  DMA_HandleTypeDef          *hdma;             
  volatile HAL_LockTypeDef       Lock;              
  volatile HAL_QSPI_StateTypeDef State;             
  volatile uint32_t              ErrorCode;         
  uint32_t                   Timeout;           
#line 141 "..\\HALLIB\\STM32L4xx_HAL_Driver\\Inc\\stm32l4xx_hal_qspi.h"
}QSPI_HandleTypeDef;



 
typedef struct
{
  uint32_t Instruction;        
 
  uint32_t Address;            
 
  uint32_t AlternateBytes;     
 
  uint32_t AddressSize;        
 
  uint32_t AlternateBytesSize; 
 
  uint32_t DummyCycles;        
 
  uint32_t InstructionMode;    
 
  uint32_t AddressMode;        
 
  uint32_t AlternateByteMode;  
 
  uint32_t DataMode;           
 
  uint32_t NbData;             

 
  uint32_t DdrMode;            
 
  uint32_t DdrHoldHalfCycle;   


 
  uint32_t SIOOMode;           
 
}QSPI_CommandTypeDef;



 
typedef struct
{
  uint32_t Match;              
 
  uint32_t Mask;               
 
  uint32_t Interval;           
 
  uint32_t StatusBytesSize;    
 
  uint32_t MatchMode;          
 
  uint32_t AutomaticStop;      
 
}QSPI_AutoPollingTypeDef;



 
typedef struct
{
  uint32_t TimeOutPeriod;      
 
  uint32_t TimeOutActivation;  
 
}QSPI_MemoryMappedTypeDef;

#line 237 "..\\HALLIB\\STM32L4xx_HAL_Driver\\Inc\\stm32l4xx_hal_qspi.h"


 

 


 



 
#line 257 "..\\HALLIB\\STM32L4xx_HAL_Driver\\Inc\\stm32l4xx_hal_qspi.h"


 



 




 



 
#line 281 "..\\HALLIB\\STM32L4xx_HAL_Driver\\Inc\\stm32l4xx_hal_qspi.h"


 



 




 

#line 314 "..\\HALLIB\\STM32L4xx_HAL_Driver\\Inc\\stm32l4xx_hal_qspi.h"


 






 



 






 



 






 



 






 



 






 



 






 



 




 



 






 



 




 



 




 



 




 



 




 



 
#line 445 "..\\HALLIB\\STM32L4xx_HAL_Driver\\Inc\\stm32l4xx_hal_qspi.h"


 



 







 




 



 



 

 


 



 
#line 491 "..\\HALLIB\\STM32L4xx_HAL_Driver\\Inc\\stm32l4xx_hal_qspi.h"




 





 












 













 












 














 











 



 

 


 



 
 
HAL_StatusTypeDef     HAL_QSPI_Init     (QSPI_HandleTypeDef *hqspi);
HAL_StatusTypeDef     HAL_QSPI_DeInit   (QSPI_HandleTypeDef *hqspi);
void                  HAL_QSPI_MspInit  (QSPI_HandleTypeDef *hqspi);
void                  HAL_QSPI_MspDeInit(QSPI_HandleTypeDef *hqspi);


 



 
 
 
void                  HAL_QSPI_IRQHandler(QSPI_HandleTypeDef *hqspi);

 
HAL_StatusTypeDef     HAL_QSPI_Command      (QSPI_HandleTypeDef *hqspi, QSPI_CommandTypeDef *cmd, uint32_t Timeout);
HAL_StatusTypeDef     HAL_QSPI_Transmit     (QSPI_HandleTypeDef *hqspi, uint8_t *pData, uint32_t Timeout);
HAL_StatusTypeDef     HAL_QSPI_Receive      (QSPI_HandleTypeDef *hqspi, uint8_t *pData, uint32_t Timeout);
HAL_StatusTypeDef     HAL_QSPI_Command_IT   (QSPI_HandleTypeDef *hqspi, QSPI_CommandTypeDef *cmd);
HAL_StatusTypeDef     HAL_QSPI_Transmit_IT  (QSPI_HandleTypeDef *hqspi, uint8_t *pData);
HAL_StatusTypeDef     HAL_QSPI_Receive_IT   (QSPI_HandleTypeDef *hqspi, uint8_t *pData);
HAL_StatusTypeDef     HAL_QSPI_Transmit_DMA (QSPI_HandleTypeDef *hqspi, uint8_t *pData);
HAL_StatusTypeDef     HAL_QSPI_Receive_DMA  (QSPI_HandleTypeDef *hqspi, uint8_t *pData);

 
HAL_StatusTypeDef     HAL_QSPI_AutoPolling   (QSPI_HandleTypeDef *hqspi, QSPI_CommandTypeDef *cmd, QSPI_AutoPollingTypeDef *cfg, uint32_t Timeout);
HAL_StatusTypeDef     HAL_QSPI_AutoPolling_IT(QSPI_HandleTypeDef *hqspi, QSPI_CommandTypeDef *cmd, QSPI_AutoPollingTypeDef *cfg);

 
HAL_StatusTypeDef     HAL_QSPI_MemoryMapped(QSPI_HandleTypeDef *hqspi, QSPI_CommandTypeDef *cmd, QSPI_MemoryMappedTypeDef *cfg);

 
void                  HAL_QSPI_ErrorCallback        (QSPI_HandleTypeDef *hqspi);
void                  HAL_QSPI_AbortCpltCallback    (QSPI_HandleTypeDef *hqspi);
void                  HAL_QSPI_FifoThresholdCallback(QSPI_HandleTypeDef *hqspi);

 
void                  HAL_QSPI_CmdCpltCallback      (QSPI_HandleTypeDef *hqspi);
void                  HAL_QSPI_RxCpltCallback       (QSPI_HandleTypeDef *hqspi);
void                  HAL_QSPI_TxCpltCallback       (QSPI_HandleTypeDef *hqspi);
void                  HAL_QSPI_RxHalfCpltCallback   (QSPI_HandleTypeDef *hqspi);
void                  HAL_QSPI_TxHalfCpltCallback   (QSPI_HandleTypeDef *hqspi);

 
void                  HAL_QSPI_StatusMatchCallback  (QSPI_HandleTypeDef *hqspi);

 
void                  HAL_QSPI_TimeOutCallback      (QSPI_HandleTypeDef *hqspi);








 



 
 
HAL_QSPI_StateTypeDef HAL_QSPI_GetState        (QSPI_HandleTypeDef *hqspi);
uint32_t              HAL_QSPI_GetError        (QSPI_HandleTypeDef *hqspi);
HAL_StatusTypeDef     HAL_QSPI_Abort           (QSPI_HandleTypeDef *hqspi);
HAL_StatusTypeDef     HAL_QSPI_Abort_IT        (QSPI_HandleTypeDef *hqspi);
void                  HAL_QSPI_SetTimeout      (QSPI_HandleTypeDef *hqspi, uint32_t Timeout);
HAL_StatusTypeDef     HAL_QSPI_SetFifoThreshold(QSPI_HandleTypeDef *hqspi, uint32_t Threshold);
uint32_t              HAL_QSPI_GetFifoThreshold(QSPI_HandleTypeDef *hqspi);





 



 
 

 


 









#line 686 "..\\HALLIB\\STM32L4xx_HAL_Driver\\Inc\\stm32l4xx_hal_qspi.h"




#line 699 "..\\HALLIB\\STM32L4xx_HAL_Driver\\Inc\\stm32l4xx_hal_qspi.h"




































#line 741 "..\\HALLIB\\STM32L4xx_HAL_Driver\\Inc\\stm32l4xx_hal_qspi.h"





















 
 



 



 









 
#line 342 "..\\USER\\stm32l4xx_hal_conf.h"


#line 1 "..\\HALLIB\\STM32L4xx_HAL_Driver\\Inc\\stm32l4xx_hal_rng.h"

































  

 







 
#line 46 "..\\HALLIB\\STM32L4xx_HAL_Driver\\Inc\\stm32l4xx_hal_rng.h"



 



  

  


 

#line 69 "..\\HALLIB\\STM32L4xx_HAL_Driver\\Inc\\stm32l4xx_hal_rng.h"



  
typedef enum
{
  HAL_RNG_STATE_RESET     = 0x00,   
  HAL_RNG_STATE_READY     = 0x01,   
  HAL_RNG_STATE_BUSY      = 0x02,    
  HAL_RNG_STATE_TIMEOUT   = 0x03,   
  HAL_RNG_STATE_ERROR     = 0x04    

}HAL_RNG_StateTypeDef;



  
typedef struct __RNG_HandleTypeDef
{
  RNG_TypeDef                 *Instance;      





  HAL_LockTypeDef             Lock;           

  volatile HAL_RNG_StateTypeDef   State;          

  volatile  uint32_t              ErrorCode;      

  uint32_t                    RandomNumber;   

#line 109 "..\\HALLIB\\STM32L4xx_HAL_Driver\\Inc\\stm32l4xx_hal_rng.h"

}RNG_HandleTypeDef;

#line 132 "..\\HALLIB\\STM32L4xx_HAL_Driver\\Inc\\stm32l4xx_hal_rng.h"



 

 


 



 





 



  





 

#line 172 "..\\HALLIB\\STM32L4xx_HAL_Driver\\Inc\\stm32l4xx_hal_rng.h"



 






 



  
  
 


 




 
#line 206 "..\\HALLIB\\STM32L4xx_HAL_Driver\\Inc\\stm32l4xx_hal_rng.h"





 






 











 









 






 






 











 











 




  


 


 

 


   
HAL_StatusTypeDef HAL_RNG_Init(RNG_HandleTypeDef *hrng);
HAL_StatusTypeDef HAL_RNG_DeInit (RNG_HandleTypeDef *hrng);
void HAL_RNG_MspInit(RNG_HandleTypeDef *hrng);
void HAL_RNG_MspDeInit(RNG_HandleTypeDef *hrng);

 
#line 308 "..\\HALLIB\\STM32L4xx_HAL_Driver\\Inc\\stm32l4xx_hal_rng.h"



  

 


 
uint32_t HAL_RNG_GetRandomNumber(RNG_HandleTypeDef *hrng);     
uint32_t HAL_RNG_GetRandomNumber_IT(RNG_HandleTypeDef *hrng);  

HAL_StatusTypeDef HAL_RNG_GenerateRandomNumber(RNG_HandleTypeDef *hrng, uint32_t *random32bit);
HAL_StatusTypeDef HAL_RNG_GenerateRandomNumber_IT(RNG_HandleTypeDef *hrng);
uint32_t HAL_RNG_ReadLastRandomNumber(RNG_HandleTypeDef *hrng);

void HAL_RNG_IRQHandler(RNG_HandleTypeDef *hrng);
void HAL_RNG_ErrorCallback(RNG_HandleTypeDef *hrng);
void HAL_RNG_ReadyDataCallback(RNG_HandleTypeDef* hrng, uint32_t random32bit);


  

 


 
HAL_RNG_StateTypeDef HAL_RNG_GetState(RNG_HandleTypeDef *hrng);
uint32_t             HAL_RNG_GetError(RNG_HandleTypeDef *hrng);


 
  


 

 
 
 
 
 


 

#line 363 "..\\HALLIB\\STM32L4xx_HAL_Driver\\Inc\\stm32l4xx_hal_rng.h"



 
 



  



  







 
#line 346 "..\\USER\\stm32l4xx_hal_conf.h"


#line 1 "..\\HALLIB\\STM32L4xx_HAL_Driver\\Inc\\stm32l4xx_hal_rtc.h"

































 

 







 
#line 46 "..\\HALLIB\\STM32L4xx_HAL_Driver\\Inc\\stm32l4xx_hal_rtc.h"



 



 

 


 



 
typedef enum
{
  HAL_RTC_STATE_RESET             = 0x00U,   
  HAL_RTC_STATE_READY             = 0x01U,   
  HAL_RTC_STATE_BUSY              = 0x02U,   
  HAL_RTC_STATE_TIMEOUT           = 0x03U,   
  HAL_RTC_STATE_ERROR             = 0x04U    

}HAL_RTCStateTypeDef;



 
typedef struct
{
  uint32_t HourFormat;      
 

  uint32_t AsynchPrediv;    
 

  uint32_t SynchPrediv;     
 

  uint32_t OutPut;          
 

  uint32_t OutPutRemap;     
 

  uint32_t OutPutPolarity;  
 

  uint32_t OutPutType;      
 




}RTC_InitTypeDef;



 
typedef struct
{
  uint8_t Hours;            

 

  uint8_t Minutes;          
 

  uint8_t Seconds;          
 

  uint8_t TimeFormat;       
 

  uint32_t SubSeconds;     

 

  uint32_t SecondFraction;  



 

  uint32_t DayLightSaving;  
 

  uint32_t StoreOperation;  

 
}RTC_TimeTypeDef;



 
typedef struct
{
  uint8_t WeekDay;  
 

  uint8_t Month;    
 

  uint8_t Date;     
 

  uint8_t Year;     
 

}RTC_DateTypeDef;



 
typedef struct
{
  RTC_TimeTypeDef AlarmTime;      

  uint32_t AlarmMask;            
 

  uint32_t AlarmSubSecondMask;   
 

  uint32_t AlarmDateWeekDaySel;  
 

  uint8_t AlarmDateWeekDay;      

 

  uint32_t Alarm;                
 
}RTC_AlarmTypeDef;



 
typedef struct __RTC_HandleTypeDef
{
  RTC_TypeDef               *Instance;   




  RTC_InitTypeDef           Init;        

  HAL_LockTypeDef           Lock;        

  volatile HAL_RTCStateTypeDef  State;       

#line 214 "..\\HALLIB\\STM32L4xx_HAL_Driver\\Inc\\stm32l4xx_hal_rtc.h"

}RTC_HandleTypeDef;

#line 243 "..\\HALLIB\\STM32L4xx_HAL_Driver\\Inc\\stm32l4xx_hal_rtc.h"



 

 


 



 




 



 
#line 272 "..\\HALLIB\\STM32L4xx_HAL_Driver\\Inc\\stm32l4xx_hal_rtc.h"


 



 




 



 
#line 295 "..\\HALLIB\\STM32L4xx_HAL_Driver\\Inc\\stm32l4xx_hal_rtc.h"


 



 






 



 
#line 320 "..\\HALLIB\\STM32L4xx_HAL_Driver\\Inc\\stm32l4xx_hal_rtc.h"


 



 




 



 





 



 




 



 




 



 

 
#line 378 "..\\HALLIB\\STM32L4xx_HAL_Driver\\Inc\\stm32l4xx_hal_rtc.h"



 



 
#line 393 "..\\HALLIB\\STM32L4xx_HAL_Driver\\Inc\\stm32l4xx_hal_rtc.h"



 



 





 



 
#line 417 "..\\HALLIB\\STM32L4xx_HAL_Driver\\Inc\\stm32l4xx_hal_rtc.h"



 



 





 




 
#line 469 "..\\HALLIB\\STM32L4xx_HAL_Driver\\Inc\\stm32l4xx_hal_rtc.h"


 



 






 

#line 534 "..\\HALLIB\\STM32L4xx_HAL_Driver\\Inc\\stm32l4xx_hal_rtc.h"



 
#line 552 "..\\HALLIB\\STM32L4xx_HAL_Driver\\Inc\\stm32l4xx_hal_rtc.h"


 




 

 


 




 
#line 579 "..\\HALLIB\\STM32L4xx_HAL_Driver\\Inc\\stm32l4xx_hal_rtc.h"





 










 













 
#line 617 "..\\HALLIB\\STM32L4xx_HAL_Driver\\Inc\\stm32l4xx_hal_rtc.h"









 
#line 634 "..\\HALLIB\\STM32L4xx_HAL_Driver\\Inc\\stm32l4xx_hal_rtc.h"

 



 






 






 






 










 










 










 













 












 














 
#line 742 "..\\HALLIB\\STM32L4xx_HAL_Driver\\Inc\\stm32l4xx_hal_rtc.h"




 





 





 





 





 





 





 





 





 








 








 





 





 




 

 
#line 1 "..\\HALLIB\\STM32L4xx_HAL_Driver\\Inc\\stm32l4xx_hal_rtc_ex.h"

































 

 







 
#line 46 "..\\HALLIB\\STM32L4xx_HAL_Driver\\Inc\\stm32l4xx_hal_rtc_ex.h"



 



 

 


 



 
typedef struct
{
  uint32_t Tamper;                      
 

  uint32_t Interrupt;                   
 

  uint32_t Trigger;                     
 

  uint32_t NoErase;                     
 

  uint32_t MaskFlag;                    
 

  uint32_t Filter;                      
 

  uint32_t SamplingFrequency;           
 

  uint32_t PrechargeDuration;           
 

  uint32_t TamperPullUp;                
 

  uint32_t TimeStampOnTamperDetection;  
 
}RTC_TamperTypeDef;


 



 

 



 

 
 
 




 




 



 



 

 
 
 



 
#line 144 "..\\HALLIB\\STM32L4xx_HAL_Driver\\Inc\\stm32l4xx_hal_rtc_ex.h"


 

 
 
 



 
#line 161 "..\\HALLIB\\STM32L4xx_HAL_Driver\\Inc\\stm32l4xx_hal_rtc_ex.h"


 



 







 

#line 192 "..\\HALLIB\\STM32L4xx_HAL_Driver\\Inc\\stm32l4xx_hal_rtc_ex.h"



 




 



 




 


 
 
 



 
#line 237 "..\\HALLIB\\STM32L4xx_HAL_Driver\\Inc\\stm32l4xx_hal_rtc_ex.h"


 



 
#line 255 "..\\HALLIB\\STM32L4xx_HAL_Driver\\Inc\\stm32l4xx_hal_rtc_ex.h"


 



 
#line 269 "..\\HALLIB\\STM32L4xx_HAL_Driver\\Inc\\stm32l4xx_hal_rtc_ex.h"


 



 
#line 283 "..\\HALLIB\\STM32L4xx_HAL_Driver\\Inc\\stm32l4xx_hal_rtc_ex.h"


 



 
#line 301 "..\\HALLIB\\STM32L4xx_HAL_Driver\\Inc\\stm32l4xx_hal_rtc_ex.h"

#line 309 "..\\HALLIB\\STM32L4xx_HAL_Driver\\Inc\\stm32l4xx_hal_rtc_ex.h"



 



 
#line 337 "..\\HALLIB\\STM32L4xx_HAL_Driver\\Inc\\stm32l4xx_hal_rtc_ex.h"

#line 357 "..\\HALLIB\\STM32L4xx_HAL_Driver\\Inc\\stm32l4xx_hal_rtc_ex.h"


 



 
#line 374 "..\\HALLIB\\STM32L4xx_HAL_Driver\\Inc\\stm32l4xx_hal_rtc_ex.h"

#line 386 "..\\HALLIB\\STM32L4xx_HAL_Driver\\Inc\\stm32l4xx_hal_rtc_ex.h"


 



 
#line 400 "..\\HALLIB\\STM32L4xx_HAL_Driver\\Inc\\stm32l4xx_hal_rtc_ex.h"



 



 
#line 415 "..\\HALLIB\\STM32L4xx_HAL_Driver\\Inc\\stm32l4xx_hal_rtc_ex.h"


 



 
#line 434 "..\\HALLIB\\STM32L4xx_HAL_Driver\\Inc\\stm32l4xx_hal_rtc_ex.h"


 



 
#line 450 "..\\HALLIB\\STM32L4xx_HAL_Driver\\Inc\\stm32l4xx_hal_rtc_ex.h"


 

 
 
 



 
#line 467 "..\\HALLIB\\STM32L4xx_HAL_Driver\\Inc\\stm32l4xx_hal_rtc_ex.h"


 



 
#line 506 "..\\HALLIB\\STM32L4xx_HAL_Driver\\Inc\\stm32l4xx_hal_rtc_ex.h"


 






 
#line 529 "..\\HALLIB\\STM32L4xx_HAL_Driver\\Inc\\stm32l4xx_hal_rtc_ex.h"




 



 

 


 

#line 582 "..\\HALLIB\\STM32L4xx_HAL_Driver\\Inc\\stm32l4xx_hal_rtc_ex.h"

 


 




 






 









 









 










 













 










 













 







 
 



 





 





 





 





 





 





 





 





 









 








 





 





 




 

 


 




 






 









 









 









 












 










 














 
#line 858 "..\\HALLIB\\STM32L4xx_HAL_Driver\\Inc\\stm32l4xx_hal_rtc_ex.h"





 






 









 













 







#line 936 "..\\HALLIB\\STM32L4xx_HAL_Driver\\Inc\\stm32l4xx_hal_rtc_ex.h"

 


 





 






 






 






 









 








 


 


 





 










 
#line 1016 "..\\HALLIB\\STM32L4xx_HAL_Driver\\Inc\\stm32l4xx_hal_rtc_ex.h"





 










 











 






 



 










 















 







 










 















 















 














 









 





 





 





 





 





 





 





 





 








 








 





 





 




 



 

 



 

 
 
 

 



 
HAL_StatusTypeDef HAL_RTCEx_SetTimeStamp(RTC_HandleTypeDef *hrtc, uint32_t TimeStampEdge, uint32_t RTC_TimeStampPin);
HAL_StatusTypeDef HAL_RTCEx_SetTimeStamp_IT(RTC_HandleTypeDef *hrtc, uint32_t TimeStampEdge, uint32_t RTC_TimeStampPin);
HAL_StatusTypeDef HAL_RTCEx_DeactivateTimeStamp(RTC_HandleTypeDef *hrtc);
HAL_StatusTypeDef HAL_RTCEx_SetInternalTimeStamp(RTC_HandleTypeDef *hrtc);
HAL_StatusTypeDef HAL_RTCEx_DeactivateInternalTimeStamp(RTC_HandleTypeDef *hrtc);
HAL_StatusTypeDef HAL_RTCEx_GetTimeStamp(RTC_HandleTypeDef *hrtc, RTC_TimeTypeDef *sTimeStamp, RTC_DateTypeDef *sTimeStampDate, uint32_t Format);
void              HAL_RTCEx_TamperTimeStampIRQHandler(RTC_HandleTypeDef *hrtc);
void              HAL_RTCEx_TimeStampEventCallback(RTC_HandleTypeDef *hrtc);
HAL_StatusTypeDef HAL_RTCEx_PollForTimeStampEvent(RTC_HandleTypeDef *hrtc, uint32_t Timeout);


 

 
 
 

 



 
HAL_StatusTypeDef HAL_RTCEx_SetWakeUpTimer(RTC_HandleTypeDef *hrtc, uint32_t WakeUpCounter, uint32_t WakeUpClock);



HAL_StatusTypeDef HAL_RTCEx_SetWakeUpTimer_IT(RTC_HandleTypeDef *hrtc, uint32_t WakeUpCounter, uint32_t WakeUpClock);

HAL_StatusTypeDef HAL_RTCEx_DeactivateWakeUpTimer(RTC_HandleTypeDef *hrtc);
uint32_t          HAL_RTCEx_GetWakeUpTimer(RTC_HandleTypeDef *hrtc);
void              HAL_RTCEx_WakeUpTimerIRQHandler(RTC_HandleTypeDef *hrtc);
void              HAL_RTCEx_WakeUpTimerEventCallback(RTC_HandleTypeDef *hrtc);
HAL_StatusTypeDef HAL_RTCEx_PollForWakeUpTimerEvent(RTC_HandleTypeDef *hrtc, uint32_t Timeout);


 

 
 
 

 



 
HAL_StatusTypeDef HAL_RTCEx_SetSmoothCalib(RTC_HandleTypeDef *hrtc, uint32_t SmoothCalibPeriod, uint32_t SmoothCalibPlusPulses, uint32_t SmoothCalibMinusPulsesValue);



HAL_StatusTypeDef HAL_RTCEx_SetSynchroShift(RTC_HandleTypeDef *hrtc, uint32_t ShiftAdd1S, uint32_t ShiftSubFS);
HAL_StatusTypeDef HAL_RTCEx_SetCalibrationOutPut(RTC_HandleTypeDef *hrtc, uint32_t CalibOutput);
HAL_StatusTypeDef HAL_RTCEx_DeactivateCalibrationOutPut(RTC_HandleTypeDef *hrtc);
HAL_StatusTypeDef HAL_RTCEx_SetRefClock(RTC_HandleTypeDef *hrtc);
HAL_StatusTypeDef HAL_RTCEx_DeactivateRefClock(RTC_HandleTypeDef *hrtc);
HAL_StatusTypeDef HAL_RTCEx_EnableBypassShadow(RTC_HandleTypeDef *hrtc);
HAL_StatusTypeDef HAL_RTCEx_DisableBypassShadow(RTC_HandleTypeDef *hrtc);


 

 


 

void              HAL_RTCEx_AlarmBEventCallback(RTC_HandleTypeDef *hrtc);
HAL_StatusTypeDef HAL_RTCEx_PollForAlarmBEvent(RTC_HandleTypeDef *hrtc, uint32_t Timeout);


 



 
HAL_StatusTypeDef HAL_RTCEx_SetTamper(RTC_HandleTypeDef *hrtc, RTC_TamperTypeDef* sTamper);
HAL_StatusTypeDef HAL_RTCEx_SetTamper_IT(RTC_HandleTypeDef *hrtc, RTC_TamperTypeDef* sTamper);
HAL_StatusTypeDef HAL_RTCEx_DeactivateTamper(RTC_HandleTypeDef *hrtc, uint32_t Tamper);


HAL_StatusTypeDef HAL_RTCEx_PollForTamper1Event(RTC_HandleTypeDef *hrtc, uint32_t Timeout);

HAL_StatusTypeDef HAL_RTCEx_PollForTamper2Event(RTC_HandleTypeDef *hrtc, uint32_t Timeout);

HAL_StatusTypeDef HAL_RTCEx_PollForTamper3Event(RTC_HandleTypeDef *hrtc, uint32_t Timeout);



void              HAL_RTCEx_Tamper1EventCallback(RTC_HandleTypeDef *hrtc);

void              HAL_RTCEx_Tamper2EventCallback(RTC_HandleTypeDef *hrtc);

void              HAL_RTCEx_Tamper3EventCallback(RTC_HandleTypeDef *hrtc);





 



 
void              HAL_RTCEx_BKUPWrite(RTC_HandleTypeDef *hrtc, uint32_t BackupRegister, uint32_t Data);
uint32_t          HAL_RTCEx_BKUPRead(RTC_HandleTypeDef *hrtc, uint32_t BackupRegister);


 



 

 
 
 


 





 

 


 



 







#line 1409 "..\\HALLIB\\STM32L4xx_HAL_Driver\\Inc\\stm32l4xx_hal_rtc_ex.h"

















#line 1432 "..\\HALLIB\\STM32L4xx_HAL_Driver\\Inc\\stm32l4xx_hal_rtc_ex.h"


















#line 1458 "..\\HALLIB\\STM32L4xx_HAL_Driver\\Inc\\stm32l4xx_hal_rtc_ex.h"
























 



 



 



 







 
#line 833 "..\\HALLIB\\STM32L4xx_HAL_Driver\\Inc\\stm32l4xx_hal_rtc.h"

 


 



 
 
HAL_StatusTypeDef HAL_RTC_Init(RTC_HandleTypeDef *hrtc);
HAL_StatusTypeDef HAL_RTC_DeInit(RTC_HandleTypeDef *hrtc);

void HAL_RTC_MspInit(RTC_HandleTypeDef *hrtc);
void HAL_RTC_MspDeInit(RTC_HandleTypeDef *hrtc);

 






 



 
 
HAL_StatusTypeDef HAL_RTC_SetTime(RTC_HandleTypeDef *hrtc, RTC_TimeTypeDef *sTime, uint32_t Format);
HAL_StatusTypeDef HAL_RTC_GetTime(RTC_HandleTypeDef *hrtc, RTC_TimeTypeDef *sTime, uint32_t Format);
HAL_StatusTypeDef HAL_RTC_SetDate(RTC_HandleTypeDef *hrtc, RTC_DateTypeDef *sDate, uint32_t Format);
HAL_StatusTypeDef HAL_RTC_GetDate(RTC_HandleTypeDef *hrtc, RTC_DateTypeDef *sDate, uint32_t Format);


 



 
 
HAL_StatusTypeDef HAL_RTC_SetAlarm(RTC_HandleTypeDef *hrtc, RTC_AlarmTypeDef *sAlarm, uint32_t Format);
HAL_StatusTypeDef HAL_RTC_SetAlarm_IT(RTC_HandleTypeDef *hrtc, RTC_AlarmTypeDef *sAlarm, uint32_t Format);
HAL_StatusTypeDef HAL_RTC_DeactivateAlarm(RTC_HandleTypeDef *hrtc, uint32_t Alarm);
HAL_StatusTypeDef HAL_RTC_GetAlarm(RTC_HandleTypeDef *hrtc, RTC_AlarmTypeDef *sAlarm, uint32_t Alarm, uint32_t Format);
void              HAL_RTC_AlarmIRQHandler(RTC_HandleTypeDef *hrtc);
void              HAL_RTC_AlarmAEventCallback(RTC_HandleTypeDef *hrtc);
HAL_StatusTypeDef HAL_RTC_PollForAlarmAEvent(RTC_HandleTypeDef *hrtc, uint32_t Timeout);


 



 
 
HAL_StatusTypeDef   HAL_RTC_WaitForSynchro(RTC_HandleTypeDef* hrtc);


 



 
 
HAL_RTCStateTypeDef HAL_RTC_GetState(RTC_HandleTypeDef *hrtc);


 



 

 
 
 


 
 








#line 928 "..\\HALLIB\\STM32L4xx_HAL_Driver\\Inc\\stm32l4xx_hal_rtc.h"







 

 


 



 
#line 957 "..\\HALLIB\\STM32L4xx_HAL_Driver\\Inc\\stm32l4xx_hal_rtc.h"





































#line 1001 "..\\HALLIB\\STM32L4xx_HAL_Driver\\Inc\\stm32l4xx_hal_rtc.h"



#line 1011 "..\\HALLIB\\STM32L4xx_HAL_Driver\\Inc\\stm32l4xx_hal_rtc.h"




























 



 

 


 
HAL_StatusTypeDef  RTC_EnterInitMode(RTC_HandleTypeDef* hrtc);
uint8_t            RTC_ByteToBcd2(uint8_t Value);
uint8_t            RTC_Bcd2ToByte(uint8_t Value);


 




 



 



 







 

#line 350 "..\\USER\\stm32l4xx_hal_conf.h"


#line 1 "..\\HALLIB\\STM32L4xx_HAL_Driver\\Inc\\stm32l4xx_hal_sai.h"

































 

 









 
#line 48 "..\\HALLIB\\STM32L4xx_HAL_Driver\\Inc\\stm32l4xx_hal_sai.h"



 



 

 


 



 
typedef enum
{
  HAL_SAI_STATE_RESET   = 0x00U,  
  HAL_SAI_STATE_READY   = 0x01U,  
  HAL_SAI_STATE_BUSY    = 0x02U,  
  HAL_SAI_STATE_BUSY_TX = 0x12U,  
  HAL_SAI_STATE_BUSY_RX = 0x22U,  
} HAL_SAI_StateTypeDef;



 
typedef void (*SAIcallback)(void);

#line 96 "..\\HALLIB\\STM32L4xx_HAL_Driver\\Inc\\stm32l4xx_hal_sai.h"




 
typedef struct
{
  uint32_t AudioMode;           
 

  uint32_t Synchro;             
 

  uint32_t SynchroExt;          



 

  uint32_t OutputDrive;         


 

  uint32_t NoDivider;           













 

  uint32_t FIFOThreshold;       
 

  uint32_t AudioFrequency;      
 

  uint32_t Mckdiv;              



 






  uint32_t MonoStereoMode;      
 

  uint32_t CompandingMode;      
 

  uint32_t TriState;            
 





  
 

  uint32_t Protocol;        
 

  uint32_t DataSize;        
 

  uint32_t FirstBit;        
 

  uint32_t ClockStrobing;   
 
} SAI_InitTypeDef;


 




 
typedef struct
{

  uint32_t FrameLength;        



 

  uint32_t ActiveFrameLength;  


 

  uint32_t FSDefinition;       
 

  uint32_t FSPolarity;         
 

  uint32_t FSOffset;           
 

} SAI_FrameInitTypeDef;


 




 
typedef struct
{
  uint32_t FirstBitOffset;  
 

  uint32_t SlotSize;        
 

  uint32_t SlotNumber;      
 

  uint32_t SlotActive;      
 
} SAI_SlotInitTypeDef;


 




 
typedef struct __SAI_HandleTypeDef
{
  SAI_Block_TypeDef         *Instance;     

  SAI_InitTypeDef           Init;          

  SAI_FrameInitTypeDef      FrameInit;     

  SAI_SlotInitTypeDef       SlotInit;      

  uint8_t                  *pBuffPtr;      

  uint16_t                  XferSize;      

  uint16_t                  XferCount;     

  DMA_HandleTypeDef         *hdmatx;       

  DMA_HandleTypeDef         *hdmarx;       

  SAIcallback               mutecallback;  

  void (*InterruptServiceRoutine)(struct __SAI_HandleTypeDef *hsai);  

  HAL_LockTypeDef           Lock;          

  volatile HAL_SAI_StateTypeDef State;         

  volatile uint32_t             ErrorCode;     

#line 282 "..\\HALLIB\\STM32L4xx_HAL_Driver\\Inc\\stm32l4xx_hal_sai.h"
} SAI_HandleTypeDef;


 

#line 307 "..\\HALLIB\\STM32L4xx_HAL_Driver\\Inc\\stm32l4xx_hal_sai.h"



 

 


 



 
#line 332 "..\\HALLIB\\STM32L4xx_HAL_Driver\\Inc\\stm32l4xx_hal_sai.h"


 



 





 



 







 



 






 



 
#line 382 "..\\HALLIB\\STM32L4xx_HAL_Driver\\Inc\\stm32l4xx_hal_sai.h"


 

#line 405 "..\\HALLIB\\STM32L4xx_HAL_Driver\\Inc\\stm32l4xx_hal_sai.h"



 







 



 





 



 
#line 437 "..\\HALLIB\\STM32L4xx_HAL_Driver\\Inc\\stm32l4xx_hal_sai.h"


 



 




 



 




 



 






 



 




 



 
#line 488 "..\\HALLIB\\STM32L4xx_HAL_Driver\\Inc\\stm32l4xx_hal_sai.h"


 



 




 



 




 



 




 



 





 



 
#line 550 "..\\HALLIB\\STM32L4xx_HAL_Driver\\Inc\\stm32l4xx_hal_sai.h"


 



 




 



 




 



 







 



 







 



 




 



 
#line 615 "..\\HALLIB\\STM32L4xx_HAL_Driver\\Inc\\stm32l4xx_hal_sai.h"


 



 
#line 629 "..\\HALLIB\\STM32L4xx_HAL_Driver\\Inc\\stm32l4xx_hal_sai.h"


 



 
#line 642 "..\\HALLIB\\STM32L4xx_HAL_Driver\\Inc\\stm32l4xx_hal_sai.h"


 



 

 



 




 
#line 669 "..\\HALLIB\\STM32L4xx_HAL_Driver\\Inc\\stm32l4xx_hal_sai.h"













 














 














 














 















 





 





 




 






 


 

 


 
HAL_StatusTypeDef HAL_SAI_InitProtocol(SAI_HandleTypeDef *hsai, uint32_t protocol, uint32_t datasize, uint32_t nbslot);
HAL_StatusTypeDef HAL_SAI_Init(SAI_HandleTypeDef *hsai);
HAL_StatusTypeDef HAL_SAI_DeInit(SAI_HandleTypeDef *hsai);
void HAL_SAI_MspInit(SAI_HandleTypeDef *hsai);
void HAL_SAI_MspDeInit(SAI_HandleTypeDef *hsai);

#line 790 "..\\HALLIB\\STM32L4xx_HAL_Driver\\Inc\\stm32l4xx_hal_sai.h"


 

 


 
 
HAL_StatusTypeDef HAL_SAI_Transmit(SAI_HandleTypeDef *hsai, uint8_t *pData, uint16_t Size, uint32_t Timeout);
HAL_StatusTypeDef HAL_SAI_Receive(SAI_HandleTypeDef *hsai, uint8_t *pData, uint16_t Size, uint32_t Timeout);

 
HAL_StatusTypeDef HAL_SAI_Transmit_IT(SAI_HandleTypeDef *hsai, uint8_t *pData, uint16_t Size);
HAL_StatusTypeDef HAL_SAI_Receive_IT(SAI_HandleTypeDef *hsai, uint8_t *pData, uint16_t Size);

 
HAL_StatusTypeDef HAL_SAI_Transmit_DMA(SAI_HandleTypeDef *hsai, uint8_t *pData, uint16_t Size);
HAL_StatusTypeDef HAL_SAI_Receive_DMA(SAI_HandleTypeDef *hsai, uint8_t *pData, uint16_t Size);
HAL_StatusTypeDef HAL_SAI_DMAPause(SAI_HandleTypeDef *hsai);
HAL_StatusTypeDef HAL_SAI_DMAResume(SAI_HandleTypeDef *hsai);
HAL_StatusTypeDef HAL_SAI_DMAStop(SAI_HandleTypeDef *hsai);

 
HAL_StatusTypeDef HAL_SAI_Abort(SAI_HandleTypeDef *hsai);

 
HAL_StatusTypeDef HAL_SAI_EnableTxMuteMode(SAI_HandleTypeDef *hsai, uint16_t val);
HAL_StatusTypeDef HAL_SAI_DisableTxMuteMode(SAI_HandleTypeDef *hsai);
HAL_StatusTypeDef HAL_SAI_EnableRxMuteMode(SAI_HandleTypeDef *hsai, SAIcallback callback, uint16_t counter);
HAL_StatusTypeDef HAL_SAI_DisableRxMuteMode(SAI_HandleTypeDef *hsai);

 
void HAL_SAI_IRQHandler(SAI_HandleTypeDef *hsai);
void HAL_SAI_TxHalfCpltCallback(SAI_HandleTypeDef *hsai);
void HAL_SAI_TxCpltCallback(SAI_HandleTypeDef *hsai);
void HAL_SAI_RxHalfCpltCallback(SAI_HandleTypeDef *hsai);
void HAL_SAI_RxCpltCallback(SAI_HandleTypeDef *hsai);
void HAL_SAI_ErrorCallback(SAI_HandleTypeDef *hsai);


 



 
 
HAL_SAI_StateTypeDef HAL_SAI_GetState(SAI_HandleTypeDef *hsai);
uint32_t HAL_SAI_GetError(SAI_HandleTypeDef *hsai);


 



 

 


 





















#line 881 "..\\HALLIB\\STM32L4xx_HAL_Driver\\Inc\\stm32l4xx_hal_sai.h"










#line 897 "..\\HALLIB\\STM32L4xx_HAL_Driver\\Inc\\stm32l4xx_hal_sai.h"








































































 

 


 



 



 



 









 
#line 354 "..\\USER\\stm32l4xx_hal_conf.h"


#line 1 "..\\HALLIB\\STM32L4xx_HAL_Driver\\Inc\\stm32l4xx_hal_sd.h"

































  

 









 
#line 1 "..\\HALLIB\\STM32L4xx_HAL_Driver\\Inc\\stm32l4xx_ll_sdmmc.h"

































  

 









 
#line 48 "..\\HALLIB\\STM32L4xx_HAL_Driver\\Inc\\stm32l4xx_ll_sdmmc.h"



 



  

  


 
  


 
typedef struct
{
  uint32_t ClockEdge;            
 


  uint32_t ClockBypass;          

 


  uint32_t ClockPowerSave;       

 

  uint32_t BusWide;              
 

  uint32_t HardwareFlowControl;  
 

  uint32_t ClockDiv;             
   





  
}SDMMC_InitTypeDef;
  



 
typedef struct                                                                                            
{
  uint32_t Argument;            


 

  uint32_t CmdIndex;            
 

  uint32_t Response;            
 

  uint32_t WaitForInterrupt;    

 

  uint32_t CPSM;                

 
}SDMMC_CmdInitTypeDef;




 
typedef struct
{
  uint32_t DataTimeOut;          

  uint32_t DataLength;           
 
  uint32_t DataBlockSize;       
 
 
  uint32_t TransferDir;         

 
 
  uint32_t TransferMode;        
 
 
  uint32_t DPSM;                

 
}SDMMC_DataInitTypeDef;



 
  
 


 
#line 191 "..\\HALLIB\\STM32L4xx_HAL_Driver\\Inc\\stm32l4xx_ll_sdmmc.h"



 
#line 251 "..\\HALLIB\\STM32L4xx_HAL_Driver\\Inc\\stm32l4xx_ll_sdmmc.h"




 
#line 267 "..\\HALLIB\\STM32L4xx_HAL_Driver\\Inc\\stm32l4xx_ll_sdmmc.h"




 
#line 283 "..\\HALLIB\\STM32L4xx_HAL_Driver\\Inc\\stm32l4xx_ll_sdmmc.h"



 
#line 307 "..\\HALLIB\\STM32L4xx_HAL_Driver\\Inc\\stm32l4xx_ll_sdmmc.h"



 




#line 324 "..\\HALLIB\\STM32L4xx_HAL_Driver\\Inc\\stm32l4xx_ll_sdmmc.h"
























 








 







 




 







  




 







 



 









 



 







 
  


 
#line 428 "..\\HALLIB\\STM32L4xx_HAL_Driver\\Inc\\stm32l4xx_ll_sdmmc.h"


   
    

#line 446 "..\\HALLIB\\STM32L4xx_HAL_Driver\\Inc\\stm32l4xx_ll_sdmmc.h"



 



 



 









 



 









 



 







   



 










#line 521 "..\\HALLIB\\STM32L4xx_HAL_Driver\\Inc\\stm32l4xx_ll_sdmmc.h"


 



 



 



 
#line 551 "..\\HALLIB\\STM32L4xx_HAL_Driver\\Inc\\stm32l4xx_ll_sdmmc.h"

#line 567 "..\\HALLIB\\STM32L4xx_HAL_Driver\\Inc\\stm32l4xx_ll_sdmmc.h"


 



 







 



 
#line 592 "..\\HALLIB\\STM32L4xx_HAL_Driver\\Inc\\stm32l4xx_ll_sdmmc.h"





 



 







 
  


 







   



 
#line 659 "..\\HALLIB\\STM32L4xx_HAL_Driver\\Inc\\stm32l4xx_ll_sdmmc.h"


  



 
#line 703 "..\\HALLIB\\STM32L4xx_HAL_Driver\\Inc\\stm32l4xx_ll_sdmmc.h"

#line 726 "..\\HALLIB\\STM32L4xx_HAL_Driver\\Inc\\stm32l4xx_ll_sdmmc.h"










 



 
  
 


 
  



 
 
 
 
#line 763 "..\\HALLIB\\STM32L4xx_HAL_Driver\\Inc\\stm32l4xx_ll_sdmmc.h"

 
 



 
 
#line 780 "..\\HALLIB\\STM32L4xx_HAL_Driver\\Inc\\stm32l4xx_ll_sdmmc.h"

#line 788 "..\\HALLIB\\STM32L4xx_HAL_Driver\\Inc\\stm32l4xx_ll_sdmmc.h"
 


 





 




 
 





  






 






  





 


 




































 






































 







































 





























 






































 


























 






   






   






   






   






   






   






   










   










   






   



 



   

 


 
  
 


 
HAL_StatusTypeDef SDMMC_Init(SDMMC_TypeDef *SDMMCx, SDMMC_InitTypeDef Init);


 
  
 


 
uint32_t          SDMMC_ReadFIFO(SDMMC_TypeDef *SDMMCx);
HAL_StatusTypeDef SDMMC_WriteFIFO(SDMMC_TypeDef *SDMMCx, uint32_t *pWriteData);


 
  
 


 
HAL_StatusTypeDef SDMMC_PowerState_ON(SDMMC_TypeDef *SDMMCx);



HAL_StatusTypeDef SDMMC_PowerState_OFF(SDMMC_TypeDef *SDMMCx);
uint32_t          SDMMC_GetPowerState(SDMMC_TypeDef *SDMMCx);

 
HAL_StatusTypeDef SDMMC_SendCommand(SDMMC_TypeDef *SDMMCx, SDMMC_CmdInitTypeDef *Command);
uint8_t           SDMMC_GetCommandResponse(SDMMC_TypeDef *SDMMCx);
uint32_t          SDMMC_GetResponse(SDMMC_TypeDef *SDMMCx, uint32_t Response);

 
HAL_StatusTypeDef SDMMC_ConfigData(SDMMC_TypeDef *SDMMCx, SDMMC_DataInitTypeDef* Data);
uint32_t          SDMMC_GetDataCounter(SDMMC_TypeDef *SDMMCx);
uint32_t          SDMMC_GetFIFOCount(SDMMC_TypeDef *SDMMCx);

 
HAL_StatusTypeDef SDMMC_SetSDMMCReadWaitMode(SDMMC_TypeDef *SDMMCx, uint32_t SDMMC_ReadWaitMode);

 
uint32_t SDMMC_CmdBlockLength(SDMMC_TypeDef *SDMMCx, uint32_t BlockSize);
uint32_t SDMMC_CmdReadSingleBlock(SDMMC_TypeDef *SDMMCx, uint32_t ReadAdd);
uint32_t SDMMC_CmdReadMultiBlock(SDMMC_TypeDef *SDMMCx, uint32_t ReadAdd);
uint32_t SDMMC_CmdWriteSingleBlock(SDMMC_TypeDef *SDMMCx, uint32_t WriteAdd);
uint32_t SDMMC_CmdWriteMultiBlock(SDMMC_TypeDef *SDMMCx, uint32_t WriteAdd);
uint32_t SDMMC_CmdEraseStartAdd(SDMMC_TypeDef *SDMMCx, uint32_t StartAdd);
uint32_t SDMMC_CmdSDEraseStartAdd(SDMMC_TypeDef *SDMMCx, uint32_t StartAdd);
uint32_t SDMMC_CmdEraseEndAdd(SDMMC_TypeDef *SDMMCx, uint32_t EndAdd);
uint32_t SDMMC_CmdSDEraseEndAdd(SDMMC_TypeDef *SDMMCx, uint32_t EndAdd);
uint32_t SDMMC_CmdErase(SDMMC_TypeDef *SDMMCx);
uint32_t SDMMC_CmdStopTransfer(SDMMC_TypeDef *SDMMCx);
uint32_t SDMMC_CmdSelDesel(SDMMC_TypeDef *SDMMCx, uint64_t Addr);
uint32_t SDMMC_CmdGoIdleState(SDMMC_TypeDef *SDMMCx);
uint32_t SDMMC_CmdOperCond(SDMMC_TypeDef *SDMMCx);
uint32_t SDMMC_CmdAppCommand(SDMMC_TypeDef *SDMMCx, uint32_t Argument);
uint32_t SDMMC_CmdAppOperCommand(SDMMC_TypeDef *SDMMCx, uint32_t Argument);
uint32_t SDMMC_CmdBusWidth(SDMMC_TypeDef *SDMMCx, uint32_t BusWidth);
uint32_t SDMMC_CmdSendSCR(SDMMC_TypeDef *SDMMCx);
uint32_t SDMMC_CmdSendCID(SDMMC_TypeDef *SDMMCx);
uint32_t SDMMC_CmdSendCSD(SDMMC_TypeDef *SDMMCx, uint32_t Argument);
uint32_t SDMMC_CmdSetRelAdd(SDMMC_TypeDef *SDMMCx, uint16_t *pRCA);
uint32_t SDMMC_CmdSendStatus(SDMMC_TypeDef *SDMMCx, uint32_t Argument);
uint32_t SDMMC_CmdStatusRegister(SDMMC_TypeDef *SDMMCx);
uint32_t SDMMC_CmdOpCondition(SDMMC_TypeDef *SDMMCx, uint32_t Argument);
uint32_t SDMMC_CmdSwitch(SDMMC_TypeDef *SDMMCx, uint32_t Argument);






 
  


 
  


  



 









 
#line 48 "..\\HALLIB\\STM32L4xx_HAL_Driver\\Inc\\stm32l4xx_hal_sd.h"



 



 

  


 



    
typedef enum
{
  HAL_SD_STATE_RESET                  = ((uint32_t)0x00000000U),   
  HAL_SD_STATE_READY                  = ((uint32_t)0x00000001U),   
  HAL_SD_STATE_TIMEOUT                = ((uint32_t)0x00000002U),   
  HAL_SD_STATE_BUSY                   = ((uint32_t)0x00000003U),   
  HAL_SD_STATE_PROGRAMMING            = ((uint32_t)0x00000004U),   
  HAL_SD_STATE_RECEIVING              = ((uint32_t)0x00000005U),   
  HAL_SD_STATE_TRANSFER               = ((uint32_t)0x00000006U),   
  HAL_SD_STATE_ERROR                  = ((uint32_t)0x0000000FU)    
}HAL_SD_StateTypeDef;


 



    
typedef enum
{
  HAL_SD_CARD_READY                  = ((uint32_t)0x00000001U),   
  HAL_SD_CARD_IDENTIFICATION         = ((uint32_t)0x00000002U),   
  HAL_SD_CARD_STANDBY                = ((uint32_t)0x00000003U),   
  HAL_SD_CARD_TRANSFER               = ((uint32_t)0x00000004U),     
  HAL_SD_CARD_SENDING                = ((uint32_t)0x00000005U),   
  HAL_SD_CARD_RECEIVING              = ((uint32_t)0x00000006U),   
  HAL_SD_CARD_PROGRAMMING            = ((uint32_t)0x00000007U),   
  HAL_SD_CARD_DISCONNECTED           = ((uint32_t)0x00000008U),   
  HAL_SD_CARD_ERROR                  = ((uint32_t)0x000000FFU)    
}HAL_SD_CardStateTypedef;


 



 





  
typedef struct
{
  uint32_t CardType;                      
  
  uint32_t CardVersion;                   

  uint32_t Class;                         

  uint32_t RelCardAdd;                    
  
  uint32_t BlockNbr;                      

  uint32_t BlockSize;                     
  
  uint32_t LogBlockNbr;                   

  uint32_t LogBlockSize;                  






}HAL_SD_CardInfoTypeDef;



  
typedef struct __SD_HandleTypeDef
{
  SDMMC_TypeDef                   *Instance;         
  
  SDMMC_InitTypeDef               Init;              
  
  HAL_LockTypeDef              Lock;              
  
  uint8_t                      *pTxBuffPtr;       
  
  uint32_t                     TxXferSize;        
  
  uint8_t                      *pRxBuffPtr;       
  
  uint32_t                     RxXferSize;        
  
  volatile uint32_t                Context;           
  
  volatile HAL_SD_StateTypeDef     State;             
  
  volatile uint32_t                ErrorCode;           
  

  
  DMA_HandleTypeDef            *hdmatx;           
  
  DMA_HandleTypeDef            *hdmarx;           
  

  HAL_SD_CardInfoTypeDef       SdCard;            
  
  uint32_t                     CSD[4];            
  
  uint32_t                     CID[4];            

#line 188 "..\\HALLIB\\STM32L4xx_HAL_Driver\\Inc\\stm32l4xx_hal_sd.h"
}SD_HandleTypeDef;



 



  
typedef struct
{
  volatile uint8_t  CSDStruct;             
  volatile uint8_t  SysSpecVersion;        
  volatile uint8_t  Reserved1;             
  volatile uint8_t  TAAC;                  
  volatile uint8_t  NSAC;                  
  volatile uint8_t  MaxBusClkFrec;         
  volatile uint16_t CardComdClasses;       
  volatile uint8_t  RdBlockLen;            
  volatile uint8_t  PartBlockRead;         
  volatile uint8_t  WrBlockMisalign;       
  volatile uint8_t  RdBlockMisalign;       
  volatile uint8_t  DSRImpl;               
  volatile uint8_t  Reserved2;             
  volatile uint32_t DeviceSize;            
  volatile uint8_t  MaxRdCurrentVDDMin;    
  volatile uint8_t  MaxRdCurrentVDDMax;    
  volatile uint8_t  MaxWrCurrentVDDMin;    
  volatile uint8_t  MaxWrCurrentVDDMax;    
  volatile uint8_t  DeviceSizeMul;         
  volatile uint8_t  EraseGrSize;           
  volatile uint8_t  EraseGrMul;            
  volatile uint8_t  WrProtectGrSize;       
  volatile uint8_t  WrProtectGrEnable;     
  volatile uint8_t  ManDeflECC;            
  volatile uint8_t  WrSpeedFact;           
  volatile uint8_t  MaxWrBlockLen;         
  volatile uint8_t  WriteBlockPaPartial;   
  volatile uint8_t  Reserved3;             
  volatile uint8_t  ContentProtectAppli;   
  volatile uint8_t  FileFormatGroup;       
  volatile uint8_t  CopyFlag;              
  volatile uint8_t  PermWrProtect;         
  volatile uint8_t  TempWrProtect;         
  volatile uint8_t  FileFormat;            
  volatile uint8_t  ECC;                   
  volatile uint8_t  CSD_CRC;               
  volatile uint8_t  Reserved4;             

}HAL_SD_CardCSDTypedef;


 



 
typedef struct
{
  volatile uint8_t  ManufacturerID;   
  volatile uint16_t OEM_AppliID;      
  volatile uint32_t ProdName1;        
  volatile uint8_t  ProdName2;        
  volatile uint8_t  ProdRev;          
  volatile uint32_t ProdSN;           
  volatile uint8_t  Reserved1;        
  volatile uint16_t ManufactDate;     
  volatile uint8_t  CID_CRC;          
  volatile uint8_t  Reserved2;        

}HAL_SD_CardCIDTypedef;


 



 
typedef struct
{
  volatile uint8_t  DataBusWidth;            
  volatile uint8_t  SecuredMode;             
  volatile uint16_t CardType;                
  volatile uint32_t ProtectedAreaSize;       
  volatile uint8_t  SpeedClass;              
  volatile uint8_t  PerformanceMove;         
  volatile uint8_t  AllocationUnitSize;      
  volatile uint16_t EraseSize;               
  volatile uint8_t  EraseTimeout;            
  volatile uint8_t  EraseOffset;             

}HAL_SD_CardStatusTypedef;


 

#line 319 "..\\HALLIB\\STM32L4xx_HAL_Driver\\Inc\\stm32l4xx_hal_sd.h"


 

 


 





   
#line 369 "..\\HALLIB\\STM32L4xx_HAL_Driver\\Inc\\stm32l4xx_hal_sd.h"




                                                


 



  
#line 388 "..\\HALLIB\\STM32L4xx_HAL_Driver\\Inc\\stm32l4xx_hal_sd.h"



 



 
#line 402 "..\\HALLIB\\STM32L4xx_HAL_Driver\\Inc\\stm32l4xx_hal_sd.h"




    


 



 




 



 
  
 



 



 
#line 442 "..\\HALLIB\\STM32L4xx_HAL_Driver\\Inc\\stm32l4xx_hal_sd.h"
 




  





 





  





 


 





































 







































 











































 





























 











































 





























 




 
  




 


 
  


 
HAL_StatusTypeDef HAL_SD_Init     (SD_HandleTypeDef *hsd);
HAL_StatusTypeDef HAL_SD_InitCard (SD_HandleTypeDef *hsd);
HAL_StatusTypeDef HAL_SD_DeInit   (SD_HandleTypeDef *hsd);
void              HAL_SD_MspInit  (SD_HandleTypeDef *hsd);
void              HAL_SD_MspDeInit(SD_HandleTypeDef *hsd);


 
  


 
 
HAL_StatusTypeDef HAL_SD_ReadBlocks     (SD_HandleTypeDef *hsd, uint8_t *pData, uint32_t BlockAdd, uint32_t NumberOfBlocks, uint32_t Timeout);
HAL_StatusTypeDef HAL_SD_WriteBlocks    (SD_HandleTypeDef *hsd, uint8_t *pData, uint32_t BlockAdd, uint32_t NumberOfBlocks, uint32_t Timeout);
HAL_StatusTypeDef HAL_SD_Erase          (SD_HandleTypeDef *hsd, uint32_t BlockStartAdd, uint32_t BlockEndAdd);
 
HAL_StatusTypeDef HAL_SD_ReadBlocks_IT  (SD_HandleTypeDef *hsd, uint8_t *pData, uint32_t BlockAdd, uint32_t NumberOfBlocks);
HAL_StatusTypeDef HAL_SD_WriteBlocks_IT (SD_HandleTypeDef *hsd, uint8_t *pData, uint32_t BlockAdd, uint32_t NumberOfBlocks);
 
HAL_StatusTypeDef HAL_SD_ReadBlocks_DMA (SD_HandleTypeDef *hsd, uint8_t *pData, uint32_t BlockAdd, uint32_t NumberOfBlocks);
HAL_StatusTypeDef HAL_SD_WriteBlocks_DMA(SD_HandleTypeDef *hsd, uint8_t *pData, uint32_t BlockAdd, uint32_t NumberOfBlocks);

void              HAL_SD_IRQHandler     (SD_HandleTypeDef *hsd);

 
void              HAL_SD_TxCpltCallback (SD_HandleTypeDef *hsd);
void              HAL_SD_RxCpltCallback (SD_HandleTypeDef *hsd);
void              HAL_SD_ErrorCallback  (SD_HandleTypeDef *hsd);
void              HAL_SD_AbortCallback  (SD_HandleTypeDef *hsd);

#line 754 "..\\HALLIB\\STM32L4xx_HAL_Driver\\Inc\\stm32l4xx_hal_sd.h"



 
  


 
HAL_StatusTypeDef HAL_SD_ConfigWideBusOperation(SD_HandleTypeDef *hsd, uint32_t WideMode);


 
  


 
HAL_StatusTypeDef       HAL_SD_SendSDStatus (SD_HandleTypeDef *hsd, uint32_t *pSDstatus);
HAL_SD_CardStateTypedef HAL_SD_GetCardState (SD_HandleTypeDef *hsd);
HAL_StatusTypeDef       HAL_SD_GetCardCID   (SD_HandleTypeDef *hsd, HAL_SD_CardCIDTypedef *pCID);
HAL_StatusTypeDef       HAL_SD_GetCardCSD   (SD_HandleTypeDef *hsd, HAL_SD_CardCSDTypedef *pCSD);
HAL_StatusTypeDef       HAL_SD_GetCardStatus(SD_HandleTypeDef *hsd, HAL_SD_CardStatusTypedef *pStatus);
HAL_StatusTypeDef       HAL_SD_GetCardInfo  (SD_HandleTypeDef *hsd, HAL_SD_CardInfoTypeDef *pCardInfo);


 



 
HAL_SD_StateTypeDef HAL_SD_GetState(SD_HandleTypeDef *hsd);
uint32_t            HAL_SD_GetError(SD_HandleTypeDef *hsd);


 
  


 
HAL_StatusTypeDef HAL_SD_Abort   (SD_HandleTypeDef *hsd);
HAL_StatusTypeDef HAL_SD_Abort_IT(SD_HandleTypeDef *hsd);


 
    
 


 



  

 


 



  
          
 


 



  

 


 



  

 


 



 

 


 



 

 


 



 




  



  



 










 
#line 358 "..\\USER\\stm32l4xx_hal_conf.h"


#line 1 "..\\HALLIB\\STM32L4xx_HAL_Driver\\Inc\\stm32l4xx_hal_smbus.h"

































 

 







 
#line 46 "..\\HALLIB\\STM32L4xx_HAL_Driver\\Inc\\stm32l4xx_hal_smbus.h"



 



 

 


 




 
typedef struct
{
  uint32_t Timing;                 

 
  uint32_t AnalogFilter;           
 

  uint32_t OwnAddress1;            
 

  uint32_t AddressingMode;         
 

  uint32_t DualAddressMode;        
 

  uint32_t OwnAddress2;            
 

  uint32_t OwnAddress2Masks;       
 

  uint32_t GeneralCallMode;        
 

  uint32_t NoStretchMode;          
 

  uint32_t PacketErrorCheckMode;   
 

  uint32_t PeripheralMode;         
 

  uint32_t SMBusTimeout;           


 
} SMBUS_InitTypeDef;


 




 
#line 122 "..\\HALLIB\\STM32L4xx_HAL_Driver\\Inc\\stm32l4xx_hal_smbus.h"


 




 
#line 143 "..\\HALLIB\\STM32L4xx_HAL_Driver\\Inc\\stm32l4xx_hal_smbus.h"


 




 
typedef struct __SMBUS_HandleTypeDef
{
  I2C_TypeDef                  *Instance;        

  SMBUS_InitTypeDef            Init;             

  uint8_t                      *pBuffPtr;        

  uint16_t                     XferSize;         

  volatile uint16_t                XferCount;        

  volatile uint32_t                XferOptions;      

  volatile uint32_t                PreviousState;    

  HAL_LockTypeDef              Lock;             

  volatile uint32_t                State;            

  volatile uint32_t                ErrorCode;        

#line 187 "..\\HALLIB\\STM32L4xx_HAL_Driver\\Inc\\stm32l4xx_hal_smbus.h"
} SMBUS_HandleTypeDef;

#line 214 "..\\HALLIB\\STM32L4xx_HAL_Driver\\Inc\\stm32l4xx_hal_smbus.h"


 



 
 



 



 




 



 




 



 





 



 

#line 267 "..\\HALLIB\\STM32L4xx_HAL_Driver\\Inc\\stm32l4xx_hal_smbus.h"


 




 




 



 




 



 




 



 





 



 







 



 







 



 




 
#line 347 "..\\HALLIB\\STM32L4xx_HAL_Driver\\Inc\\stm32l4xx_hal_smbus.h"



 






 






 
#line 376 "..\\HALLIB\\STM32L4xx_HAL_Driver\\Inc\\stm32l4xx_hal_smbus.h"


 






 

#line 403 "..\\HALLIB\\STM32L4xx_HAL_Driver\\Inc\\stm32l4xx_hal_smbus.h"


 



 

 


 




 
#line 429 "..\\HALLIB\\STM32L4xx_HAL_Driver\\Inc\\stm32l4xx_hal_smbus.h"














 















 















 
























 


















 





 





 





 




 


 

 


 












#line 571 "..\\HALLIB\\STM32L4xx_HAL_Driver\\Inc\\stm32l4xx_hal_smbus.h"














#line 593 "..\\HALLIB\\STM32L4xx_HAL_Driver\\Inc\\stm32l4xx_hal_smbus.h"








#line 608 "..\\HALLIB\\STM32L4xx_HAL_Driver\\Inc\\stm32l4xx_hal_smbus.h"


























 

 


 



 

 
HAL_StatusTypeDef HAL_SMBUS_Init(SMBUS_HandleTypeDef *hsmbus);
HAL_StatusTypeDef HAL_SMBUS_DeInit(SMBUS_HandleTypeDef *hsmbus);
void HAL_SMBUS_MspInit(SMBUS_HandleTypeDef *hsmbus);
void HAL_SMBUS_MspDeInit(SMBUS_HandleTypeDef *hsmbus);
HAL_StatusTypeDef HAL_SMBUS_ConfigAnalogFilter(SMBUS_HandleTypeDef *hsmbus, uint32_t AnalogFilter);
HAL_StatusTypeDef HAL_SMBUS_ConfigDigitalFilter(SMBUS_HandleTypeDef *hsmbus, uint32_t DigitalFilter);

 
#line 661 "..\\HALLIB\\STM32L4xx_HAL_Driver\\Inc\\stm32l4xx_hal_smbus.h"


 



 

 


 
 
HAL_StatusTypeDef HAL_SMBUS_IsDeviceReady(SMBUS_HandleTypeDef *hsmbus, uint16_t DevAddress, uint32_t Trials, uint32_t Timeout);


 



 
 
HAL_StatusTypeDef HAL_SMBUS_Master_Transmit_IT(SMBUS_HandleTypeDef *hsmbus, uint16_t DevAddress, uint8_t *pData, uint16_t Size, uint32_t XferOptions);
HAL_StatusTypeDef HAL_SMBUS_Master_Receive_IT(SMBUS_HandleTypeDef *hsmbus, uint16_t DevAddress, uint8_t *pData, uint16_t Size, uint32_t XferOptions);
HAL_StatusTypeDef HAL_SMBUS_Master_Abort_IT(SMBUS_HandleTypeDef *hsmbus, uint16_t DevAddress);
HAL_StatusTypeDef HAL_SMBUS_Slave_Transmit_IT(SMBUS_HandleTypeDef *hsmbus, uint8_t *pData, uint16_t Size, uint32_t XferOptions);
HAL_StatusTypeDef HAL_SMBUS_Slave_Receive_IT(SMBUS_HandleTypeDef *hsmbus, uint8_t *pData, uint16_t Size, uint32_t XferOptions);

HAL_StatusTypeDef HAL_SMBUS_EnableAlert_IT(SMBUS_HandleTypeDef *hsmbus);
HAL_StatusTypeDef HAL_SMBUS_DisableAlert_IT(SMBUS_HandleTypeDef *hsmbus);
HAL_StatusTypeDef HAL_SMBUS_EnableListen_IT(SMBUS_HandleTypeDef *hsmbus);
HAL_StatusTypeDef HAL_SMBUS_DisableListen_IT(SMBUS_HandleTypeDef *hsmbus);


 



 
 
void HAL_SMBUS_EV_IRQHandler(SMBUS_HandleTypeDef *hsmbus);
void HAL_SMBUS_ER_IRQHandler(SMBUS_HandleTypeDef *hsmbus);
void HAL_SMBUS_MasterTxCpltCallback(SMBUS_HandleTypeDef *hsmbus);
void HAL_SMBUS_MasterRxCpltCallback(SMBUS_HandleTypeDef *hsmbus);
void HAL_SMBUS_SlaveTxCpltCallback(SMBUS_HandleTypeDef *hsmbus);
void HAL_SMBUS_SlaveRxCpltCallback(SMBUS_HandleTypeDef *hsmbus);
void HAL_SMBUS_AddrCallback(SMBUS_HandleTypeDef *hsmbus, uint8_t TransferDirection, uint16_t AddrMatchCode);
void HAL_SMBUS_ListenCpltCallback(SMBUS_HandleTypeDef *hsmbus);
void HAL_SMBUS_ErrorCallback(SMBUS_HandleTypeDef *hsmbus);



 



 

 
uint32_t HAL_SMBUS_GetState(SMBUS_HandleTypeDef *hsmbus);
uint32_t HAL_SMBUS_GetError(SMBUS_HandleTypeDef *hsmbus);



 



 

 


 
 


 



 



 



 








 
#line 362 "..\\USER\\stm32l4xx_hal_conf.h"


#line 1 "..\\HALLIB\\STM32L4xx_HAL_Driver\\Inc\\stm32l4xx_hal_spi.h"

































 

 







 
#line 46 "..\\HALLIB\\STM32L4xx_HAL_Driver\\Inc\\stm32l4xx_hal_spi.h"



 



 

 


 



 
typedef struct
{
  uint32_t Mode;                
 

  uint32_t Direction;           
 

  uint32_t DataSize;            
 

  uint32_t CLKPolarity;         
 

  uint32_t CLKPhase;            
 

  uint32_t NSS;                 

 

  uint32_t BaudRatePrescaler;   



 

  uint32_t FirstBit;            
 

  uint32_t TIMode;              
 

  uint32_t CRCCalculation;      
 

  uint32_t CRCPolynomial;       
 

  uint32_t CRCLength;           

 

  uint32_t NSSPMode;            




 
} SPI_InitTypeDef;



 
typedef enum
{
  HAL_SPI_STATE_RESET      = 0x00U,     
  HAL_SPI_STATE_READY      = 0x01U,     
  HAL_SPI_STATE_BUSY       = 0x02U,     
  HAL_SPI_STATE_BUSY_TX    = 0x03U,     
  HAL_SPI_STATE_BUSY_RX    = 0x04U,     
  HAL_SPI_STATE_BUSY_TX_RX = 0x05U,     
  HAL_SPI_STATE_ERROR      = 0x06U,     
  HAL_SPI_STATE_ABORT      = 0x07U      
} HAL_SPI_StateTypeDef;



 
typedef struct __SPI_HandleTypeDef
{
  SPI_TypeDef                *Instance;       

  SPI_InitTypeDef            Init;            

  uint8_t                    *pTxBuffPtr;     

  uint16_t                   TxXferSize;      

  volatile uint16_t              TxXferCount;     

  uint8_t                    *pRxBuffPtr;     

  uint16_t                   RxXferSize;      

  volatile uint16_t              RxXferCount;     

  uint32_t                   CRCSize;         

  void (*RxISR)(struct __SPI_HandleTypeDef *hspi);    

  void (*TxISR)(struct __SPI_HandleTypeDef *hspi);    

  DMA_HandleTypeDef          *hdmatx;         

  DMA_HandleTypeDef          *hdmarx;         

  HAL_LockTypeDef            Lock;            

  volatile HAL_SPI_StateTypeDef  State;           

  volatile uint32_t              ErrorCode;       

#line 179 "..\\HALLIB\\STM32L4xx_HAL_Driver\\Inc\\stm32l4xx_hal_spi.h"
} SPI_HandleTypeDef;

#line 206 "..\\HALLIB\\STM32L4xx_HAL_Driver\\Inc\\stm32l4xx_hal_spi.h"


 

 


 



 
#line 229 "..\\HALLIB\\STM32L4xx_HAL_Driver\\Inc\\stm32l4xx_hal_spi.h"


 



 




 



 





 



 
#line 268 "..\\HALLIB\\STM32L4xx_HAL_Driver\\Inc\\stm32l4xx_hal_spi.h"


 



 




 



 




 



 





 



 




 



 
#line 320 "..\\HALLIB\\STM32L4xx_HAL_Driver\\Inc\\stm32l4xx_hal_spi.h"


 



 




 



 




 



 




 







 





 








 





 



 





 



 
#line 403 "..\\HALLIB\\STM32L4xx_HAL_Driver\\Inc\\stm32l4xx_hal_spi.h"


 



 







 



 






 



 

 


 





 
#line 453 "..\\HALLIB\\STM32L4xx_HAL_Driver\\Inc\\stm32l4xx_hal_spi.h"










 











 











 

















 






 






 
#line 527 "..\\HALLIB\\STM32L4xx_HAL_Driver\\Inc\\stm32l4xx_hal_spi.h"





 
#line 540 "..\\HALLIB\\STM32L4xx_HAL_Driver\\Inc\\stm32l4xx_hal_spi.h"





 
#line 552 "..\\HALLIB\\STM32L4xx_HAL_Driver\\Inc\\stm32l4xx_hal_spi.h"





 






 




 

 


 





 






 






 

















 










 






 







 







 





 







 
#line 674 "..\\HALLIB\\STM32L4xx_HAL_Driver\\Inc\\stm32l4xx_hal_spi.h"





 







 







 








 







 
#line 721 "..\\HALLIB\\STM32L4xx_HAL_Driver\\Inc\\stm32l4xx_hal_spi.h"





 







 







 







 








 





 




 

 
#line 1 "..\\HALLIB\\STM32L4xx_HAL_Driver\\Inc\\stm32l4xx_hal_spi_ex.h"

































 

 







 
#line 46 "..\\HALLIB\\STM32L4xx_HAL_Driver\\Inc\\stm32l4xx_hal_spi_ex.h"



 



 

 
 
 
 


 

 
 


 
HAL_StatusTypeDef HAL_SPIEx_FlushRxFifo(SPI_HandleTypeDef *hspi);


 



 



 



 







 
#line 774 "..\\HALLIB\\STM32L4xx_HAL_Driver\\Inc\\stm32l4xx_hal_spi.h"

 


 



 
 
HAL_StatusTypeDef HAL_SPI_Init(SPI_HandleTypeDef *hspi);
HAL_StatusTypeDef HAL_SPI_DeInit(SPI_HandleTypeDef *hspi);
void HAL_SPI_MspInit(SPI_HandleTypeDef *hspi);
void HAL_SPI_MspDeInit(SPI_HandleTypeDef *hspi);

 






 



 
 
HAL_StatusTypeDef HAL_SPI_Transmit(SPI_HandleTypeDef *hspi, uint8_t *pData, uint16_t Size, uint32_t Timeout);
HAL_StatusTypeDef HAL_SPI_Receive(SPI_HandleTypeDef *hspi, uint8_t *pData, uint16_t Size, uint32_t Timeout);
HAL_StatusTypeDef HAL_SPI_TransmitReceive(SPI_HandleTypeDef *hspi, uint8_t *pTxData, uint8_t *pRxData, uint16_t Size,
                                          uint32_t Timeout);
HAL_StatusTypeDef HAL_SPI_Transmit_IT(SPI_HandleTypeDef *hspi, uint8_t *pData, uint16_t Size);
HAL_StatusTypeDef HAL_SPI_Receive_IT(SPI_HandleTypeDef *hspi, uint8_t *pData, uint16_t Size);
HAL_StatusTypeDef HAL_SPI_TransmitReceive_IT(SPI_HandleTypeDef *hspi, uint8_t *pTxData, uint8_t *pRxData,
                                             uint16_t Size);
HAL_StatusTypeDef HAL_SPI_Transmit_DMA(SPI_HandleTypeDef *hspi, uint8_t *pData, uint16_t Size);
HAL_StatusTypeDef HAL_SPI_Receive_DMA(SPI_HandleTypeDef *hspi, uint8_t *pData, uint16_t Size);
HAL_StatusTypeDef HAL_SPI_TransmitReceive_DMA(SPI_HandleTypeDef *hspi, uint8_t *pTxData, uint8_t *pRxData,
                                              uint16_t Size);
HAL_StatusTypeDef HAL_SPI_DMAPause(SPI_HandleTypeDef *hspi);
HAL_StatusTypeDef HAL_SPI_DMAResume(SPI_HandleTypeDef *hspi);
HAL_StatusTypeDef HAL_SPI_DMAStop(SPI_HandleTypeDef *hspi);
 
HAL_StatusTypeDef HAL_SPI_Abort(SPI_HandleTypeDef *hspi);
HAL_StatusTypeDef HAL_SPI_Abort_IT(SPI_HandleTypeDef *hspi);

void HAL_SPI_IRQHandler(SPI_HandleTypeDef *hspi);
void HAL_SPI_TxCpltCallback(SPI_HandleTypeDef *hspi);
void HAL_SPI_RxCpltCallback(SPI_HandleTypeDef *hspi);
void HAL_SPI_TxRxCpltCallback(SPI_HandleTypeDef *hspi);
void HAL_SPI_TxHalfCpltCallback(SPI_HandleTypeDef *hspi);
void HAL_SPI_RxHalfCpltCallback(SPI_HandleTypeDef *hspi);
void HAL_SPI_TxRxHalfCpltCallback(SPI_HandleTypeDef *hspi);
void HAL_SPI_ErrorCallback(SPI_HandleTypeDef *hspi);
void HAL_SPI_AbortCpltCallback(SPI_HandleTypeDef *hspi);


 



 
 
HAL_SPI_StateTypeDef HAL_SPI_GetState(SPI_HandleTypeDef *hspi);
uint32_t             HAL_SPI_GetError(SPI_HandleTypeDef *hspi);


 



 



 



 







 
#line 366 "..\\USER\\stm32l4xx_hal_conf.h"


#line 1 "..\\HALLIB\\STM32L4xx_HAL_Driver\\Inc\\stm32l4xx_hal_swpmi.h"

































 

 







 
#line 46 "..\\HALLIB\\STM32L4xx_HAL_Driver\\Inc\\stm32l4xx_hal_swpmi.h"



 





 

 


 



 
typedef struct
{
  uint32_t VoltageClass;             
 

  uint32_t BitRate;                  



 

  uint32_t TxBufferingMode;          
 

  uint32_t RxBufferingMode;          
 

}SWPMI_InitTypeDef;




 
typedef enum
{
  HAL_SWPMI_STATE_RESET             = 0x00,     
  HAL_SWPMI_STATE_READY             = 0x01,     
  HAL_SWPMI_STATE_BUSY              = 0x02,     
  HAL_SWPMI_STATE_BUSY_TX           = 0x12,     
  HAL_SWPMI_STATE_BUSY_RX           = 0x22,     
  HAL_SWPMI_STATE_BUSY_TX_RX        = 0x32,     
  HAL_SWPMI_STATE_TIMEOUT           = 0x03,     
  HAL_SWPMI_STATE_ERROR             = 0x04      
}HAL_SWPMI_StateTypeDef;



 
typedef struct __SWPMI_HandleTypeDef
{
  SWPMI_TypeDef                  *Instance;      

  SWPMI_InitTypeDef              Init;           

  uint32_t                       *pTxBuffPtr;    

  uint32_t                       TxXferSize;     

  uint32_t                       TxXferCount;    

  uint32_t                       *pRxBuffPtr;    

  uint32_t                       RxXferSize;     

  uint32_t                       RxXferCount;    

  DMA_HandleTypeDef              *hdmatx;        

  DMA_HandleTypeDef              *hdmarx;        

  HAL_LockTypeDef                Lock;           

  volatile HAL_SWPMI_StateTypeDef    State;          

  volatile uint32_t                  ErrorCode;      

#line 140 "..\\HALLIB\\STM32L4xx_HAL_Driver\\Inc\\stm32l4xx_hal_swpmi.h"

}SWPMI_HandleTypeDef;

#line 163 "..\\HALLIB\\STM32L4xx_HAL_Driver\\Inc\\stm32l4xx_hal_swpmi.h"



 

 


 




 
#line 187 "..\\HALLIB\\STM32L4xx_HAL_Driver\\Inc\\stm32l4xx_hal_swpmi.h"


 



 




 



 





 



 





 





 
#line 236 "..\\HALLIB\\STM32L4xx_HAL_Driver\\Inc\\stm32l4xx_hal_swpmi.h"


 





 
#line 254 "..\\HALLIB\\STM32L4xx_HAL_Driver\\Inc\\stm32l4xx_hal_swpmi.h"


 



 

 


 




 
#line 280 "..\\HALLIB\\STM32L4xx_HAL_Driver\\Inc\\stm32l4xx_hal_swpmi.h"





 






 


















 














 
















 
















 
















 
















 




 

 


 
 
HAL_StatusTypeDef HAL_SWPMI_Init(SWPMI_HandleTypeDef *hswpmi);
HAL_StatusTypeDef HAL_SWPMI_DeInit(SWPMI_HandleTypeDef *hswpmi);
void              HAL_SWPMI_MspInit(SWPMI_HandleTypeDef *hswpmi);
void              HAL_SWPMI_MspDeInit(SWPMI_HandleTypeDef *hswpmi);

#line 419 "..\\HALLIB\\STM32L4xx_HAL_Driver\\Inc\\stm32l4xx_hal_swpmi.h"

 
HAL_StatusTypeDef HAL_SWPMI_Transmit(SWPMI_HandleTypeDef *hswpmi, uint32_t *pData, uint16_t Size, uint32_t Timeout);
HAL_StatusTypeDef HAL_SWPMI_Receive(SWPMI_HandleTypeDef *hswpmi, uint32_t *pData, uint16_t Size, uint32_t Timeout);
HAL_StatusTypeDef HAL_SWPMI_Transmit_IT(SWPMI_HandleTypeDef *hswpmi, uint32_t *pData, uint16_t Size);
HAL_StatusTypeDef HAL_SWPMI_Receive_IT(SWPMI_HandleTypeDef *hswpmi, uint32_t *pData, uint16_t Size);
HAL_StatusTypeDef HAL_SWPMI_Transmit_DMA(SWPMI_HandleTypeDef *hswpmi, uint32_t *pData, uint16_t Size);
HAL_StatusTypeDef HAL_SWPMI_Receive_DMA(SWPMI_HandleTypeDef *hswpmi, uint32_t *pData, uint16_t Size);
HAL_StatusTypeDef HAL_SWPMI_DMAStop(SWPMI_HandleTypeDef *hswpmi);
HAL_StatusTypeDef HAL_SWPMI_EnableLoopback(SWPMI_HandleTypeDef *hswpmi);
HAL_StatusTypeDef HAL_SWPMI_DisableLoopback(SWPMI_HandleTypeDef *hswpmi);
void              HAL_SWPMI_IRQHandler(SWPMI_HandleTypeDef *hswpmi);
void              HAL_SWPMI_TxCpltCallback(SWPMI_HandleTypeDef *hswpmi);
void              HAL_SWPMI_TxHalfCpltCallback(SWPMI_HandleTypeDef *hswpmi);
void              HAL_SWPMI_RxCpltCallback(SWPMI_HandleTypeDef *hswpmi);
void              HAL_SWPMI_RxHalfCpltCallback(SWPMI_HandleTypeDef *hswpmi);
void              HAL_SWPMI_ErrorCallback(SWPMI_HandleTypeDef *hswpmi);

 
HAL_SWPMI_StateTypeDef HAL_SWPMI_GetState(SWPMI_HandleTypeDef *hswpmi);
uint32_t               HAL_SWPMI_GetError(SWPMI_HandleTypeDef *hswpmi);



 

 


 



 

 


 



 

 


 



 

 


 

















 



 





 







 
#line 370 "..\\USER\\stm32l4xx_hal_conf.h"


#line 1 "..\\HALLIB\\STM32L4xx_HAL_Driver\\Inc\\stm32l4xx_hal_tim.h"

































 

 







 
#line 46 "..\\HALLIB\\STM32L4xx_HAL_Driver\\Inc\\stm32l4xx_hal_tim.h"



 



 

 


 



 
typedef struct
{
  uint32_t Prescaler;         
 

  uint32_t CounterMode;       
 

  uint32_t Period;            

 

  uint32_t ClockDivision;     
 

  uint32_t RepetitionCounter;  






 

  uint32_t AutoReloadPreload;  
 
} TIM_Base_InitTypeDef;



 
typedef struct
{
  uint32_t OCMode;        
 

  uint32_t Pulse;         
 

  uint32_t OCPolarity;    
 

  uint32_t OCNPolarity;   

 

  uint32_t OCFastMode;    

 


  uint32_t OCIdleState;   

 

  uint32_t OCNIdleState;  

 
} TIM_OC_InitTypeDef;



 
typedef struct
{
  uint32_t OCMode;        
 

  uint32_t Pulse;         
 

  uint32_t OCPolarity;    
 

  uint32_t OCNPolarity;   

 

  uint32_t OCIdleState;   

 

  uint32_t OCNIdleState;  

 

  uint32_t ICPolarity;    
 

  uint32_t ICSelection;   
 

  uint32_t ICFilter;      
 
} TIM_OnePulse_InitTypeDef;



 
typedef struct
{
  uint32_t  ICPolarity;  
 

  uint32_t ICSelection;  
 

  uint32_t ICPrescaler;  
 

  uint32_t ICFilter;     
 
} TIM_IC_InitTypeDef;



 
typedef struct
{
  uint32_t EncoderMode;   
 

  uint32_t IC1Polarity;   
 

  uint32_t IC1Selection;  
 

  uint32_t IC1Prescaler;  
 

  uint32_t IC1Filter;     
 

  uint32_t IC2Polarity;   
 

  uint32_t IC2Selection;  
 

  uint32_t IC2Prescaler;  
 

  uint32_t IC2Filter;     
 
} TIM_Encoder_InitTypeDef;



 
typedef struct
{
  uint32_t ClockSource;     
 
  uint32_t ClockPolarity;   
 
  uint32_t ClockPrescaler;  
 
  uint32_t ClockFilter;     
 
} TIM_ClockConfigTypeDef;



 
typedef struct
{
  uint32_t ClearInputState;      
 
  uint32_t ClearInputSource;     
 
  uint32_t ClearInputPolarity;   
 
  uint32_t ClearInputPrescaler;  
 
  uint32_t ClearInputFilter;     
 
} TIM_ClearInputConfigTypeDef;





 
typedef struct
{
  uint32_t  MasterOutputTrigger;   
 
  uint32_t  MasterOutputTrigger2;  
 
  uint32_t  MasterSlaveMode;       
 
} TIM_MasterConfigTypeDef;



 
typedef struct
{
  uint32_t  SlaveMode;         
 
  uint32_t  InputTrigger;      
 
  uint32_t  TriggerPolarity;   
 
  uint32_t  TriggerPrescaler;  
 
  uint32_t  TriggerFilter;     
 

} TIM_SlaveConfigTypeDef;





 
typedef struct
{
  uint32_t OffStateRunMode;      
 
  uint32_t OffStateIDLEMode;     
 
  uint32_t LockLevel;            
 
  uint32_t DeadTime;             
 
  uint32_t BreakState;           
 
  uint32_t BreakPolarity;        
 
  uint32_t BreakFilter;          
 
  uint32_t Break2State;          
 
  uint32_t Break2Polarity;       
 
  uint32_t Break2Filter;         
 
  uint32_t AutomaticOutput;      
 
} TIM_BreakDeadTimeConfigTypeDef;



 
typedef enum
{
  HAL_TIM_STATE_RESET             = 0x00U,     
  HAL_TIM_STATE_READY             = 0x01U,     
  HAL_TIM_STATE_BUSY              = 0x02U,     
  HAL_TIM_STATE_TIMEOUT           = 0x03U,     
  HAL_TIM_STATE_ERROR             = 0x04U      
} HAL_TIM_StateTypeDef;



 
typedef enum
{
  HAL_TIM_ACTIVE_CHANNEL_1        = 0x01U,     
  HAL_TIM_ACTIVE_CHANNEL_2        = 0x02U,     
  HAL_TIM_ACTIVE_CHANNEL_3        = 0x04U,     
  HAL_TIM_ACTIVE_CHANNEL_4        = 0x08U,     
  HAL_TIM_ACTIVE_CHANNEL_5        = 0x10U,     
  HAL_TIM_ACTIVE_CHANNEL_6        = 0x20U,     
  HAL_TIM_ACTIVE_CHANNEL_CLEARED  = 0x00U      
} HAL_TIM_ActiveChannel;



 
typedef struct __TIM_HandleTypeDef
{
  TIM_TypeDef                 *Instance;      
  TIM_Base_InitTypeDef        Init;           
  HAL_TIM_ActiveChannel       Channel;        
  DMA_HandleTypeDef           *hdma[7];      
 
  HAL_LockTypeDef             Lock;           
  volatile HAL_TIM_StateTypeDef   State;          

#line 371 "..\\HALLIB\\STM32L4xx_HAL_Driver\\Inc\\stm32l4xx_hal_tim.h"
} TIM_HandleTypeDef;

#line 412 "..\\HALLIB\\STM32L4xx_HAL_Driver\\Inc\\stm32l4xx_hal_tim.h"



 
 

 


 



 





 



 
#line 462 "..\\HALLIB\\STM32L4xx_HAL_Driver\\Inc\\stm32l4xx_hal_tim.h"


 



 
#line 478 "..\\HALLIB\\STM32L4xx_HAL_Driver\\Inc\\stm32l4xx_hal_tim.h"


 



 





 



 




 



 






 



 







 



 





 



 




 



 





 



 




 



 




 



 




 



 




 



 




 



 




 



 





 



 







 



 






 



 




 



 





 



 
#line 670 "..\\HALLIB\\STM32L4xx_HAL_Driver\\Inc\\stm32l4xx_hal_tim.h"


 



 




 



 
#line 693 "..\\HALLIB\\STM32L4xx_HAL_Driver\\Inc\\stm32l4xx_hal_tim.h"


 



 
#line 716 "..\\HALLIB\\STM32L4xx_HAL_Driver\\Inc\\stm32l4xx_hal_tim.h"


 



 
#line 730 "..\\HALLIB\\STM32L4xx_HAL_Driver\\Inc\\stm32l4xx_hal_tim.h"


 



 
#line 747 "..\\HALLIB\\STM32L4xx_HAL_Driver\\Inc\\stm32l4xx_hal_tim.h"


 



 







 



 






 



 




 



 






 



 




 



 




 


 






 



 




 



 




 



 




 



 




 



 





 



 






 



 
#line 890 "..\\HALLIB\\STM32L4xx_HAL_Driver\\Inc\\stm32l4xx_hal_tim.h"


 



 
#line 913 "..\\HALLIB\\STM32L4xx_HAL_Driver\\Inc\\stm32l4xx_hal_tim.h"


 



 




 



 
#line 935 "..\\HALLIB\\STM32L4xx_HAL_Driver\\Inc\\stm32l4xx_hal_tim.h"


 



 
#line 956 "..\\HALLIB\\STM32L4xx_HAL_Driver\\Inc\\stm32l4xx_hal_tim.h"


 



 
#line 972 "..\\HALLIB\\STM32L4xx_HAL_Driver\\Inc\\stm32l4xx_hal_tim.h"


 



 







 



 






 



 




 



 
#line 1029 "..\\HALLIB\\STM32L4xx_HAL_Driver\\Inc\\stm32l4xx_hal_tim.h"


 



 
#line 1043 "..\\HALLIB\\STM32L4xx_HAL_Driver\\Inc\\stm32l4xx_hal_tim.h"


 



 






 



 






 



 
 

 


 




 
#line 1104 "..\\HALLIB\\STM32L4xx_HAL_Driver\\Inc\\stm32l4xx_hal_tim.h"





 






 






 
#line 1134 "..\\HALLIB\\STM32L4xx_HAL_Driver\\Inc\\stm32l4xx_hal_tim.h"






 
#line 1151 "..\\HALLIB\\STM32L4xx_HAL_Driver\\Inc\\stm32l4xx_hal_tim.h"






 















 















 














 














 























 























 
















 















 








 







 







 






 








 










 












 
#line 1372 "..\\HALLIB\\STM32L4xx_HAL_Driver\\Inc\\stm32l4xx_hal_tim.h"








 



















 




















 



















 
#line 1449 "..\\HALLIB\\STM32L4xx_HAL_Driver\\Inc\\stm32l4xx_hal_tim.h"













 
#line 1470 "..\\HALLIB\\STM32L4xx_HAL_Driver\\Inc\\stm32l4xx_hal_tim.h"













 
#line 1491 "..\\HALLIB\\STM32L4xx_HAL_Driver\\Inc\\stm32l4xx_hal_tim.h"













 
#line 1512 "..\\HALLIB\\STM32L4xx_HAL_Driver\\Inc\\stm32l4xx_hal_tim.h"








 













 

















 








 
 

 


 

 




 
 

 


 




#line 1609 "..\\HALLIB\\STM32L4xx_HAL_Driver\\Inc\\stm32l4xx_hal_tim.h"





















































#line 1669 "..\\HALLIB\\STM32L4xx_HAL_Driver\\Inc\\stm32l4xx_hal_tim.h"








#line 1687 "..\\HALLIB\\STM32L4xx_HAL_Driver\\Inc\\stm32l4xx_hal_tim.h"























































#line 1750 "..\\HALLIB\\STM32L4xx_HAL_Driver\\Inc\\stm32l4xx_hal_tim.h"

#line 1768 "..\\HALLIB\\STM32L4xx_HAL_Driver\\Inc\\stm32l4xx_hal_tim.h"




#line 1778 "..\\HALLIB\\STM32L4xx_HAL_Driver\\Inc\\stm32l4xx_hal_tim.h"

#line 1785 "..\\HALLIB\\STM32L4xx_HAL_Driver\\Inc\\stm32l4xx_hal_tim.h"

#line 1794 "..\\HALLIB\\STM32L4xx_HAL_Driver\\Inc\\stm32l4xx_hal_tim.h"

#line 1803 "..\\HALLIB\\STM32L4xx_HAL_Driver\\Inc\\stm32l4xx_hal_tim.h"























#line 1844 "..\\HALLIB\\STM32L4xx_HAL_Driver\\Inc\\stm32l4xx_hal_tim.h"







































 
 

 
#line 1 "..\\HALLIB\\STM32L4xx_HAL_Driver\\Inc\\stm32l4xx_hal_tim_ex.h"

































 

 







 
#line 46 "..\\HALLIB\\STM32L4xx_HAL_Driver\\Inc\\stm32l4xx_hal_tim_ex.h"



 



 

 


 



 

typedef struct
{
  uint32_t IC1Polarity;         
 

  uint32_t IC1Prescaler;        
 

  uint32_t IC1Filter;           
 

  uint32_t Commutation_Delay;   
 
} TIM_HallSensor_InitTypeDef;



 
typedef struct
{
  uint32_t Source;         
 
  uint32_t Enable;         
 
  uint32_t Polarity;       

 
}
TIMEx_BreakInputConfigTypeDef;



 
 

 


 



 
#line 124 "..\\HALLIB\\STM32L4xx_HAL_Driver\\Inc\\stm32l4xx_hal_tim_ex.h"

#line 149 "..\\HALLIB\\STM32L4xx_HAL_Driver\\Inc\\stm32l4xx_hal_tim_ex.h"

#line 158 "..\\HALLIB\\STM32L4xx_HAL_Driver\\Inc\\stm32l4xx_hal_tim_ex.h"

#line 170 "..\\HALLIB\\STM32L4xx_HAL_Driver\\Inc\\stm32l4xx_hal_tim_ex.h"

#line 177 "..\\HALLIB\\STM32L4xx_HAL_Driver\\Inc\\stm32l4xx_hal_tim_ex.h"

#line 188 "..\\HALLIB\\STM32L4xx_HAL_Driver\\Inc\\stm32l4xx_hal_tim_ex.h"

#line 198 "..\\HALLIB\\STM32L4xx_HAL_Driver\\Inc\\stm32l4xx_hal_tim_ex.h"

#line 205 "..\\HALLIB\\STM32L4xx_HAL_Driver\\Inc\\stm32l4xx_hal_tim_ex.h"


 



 




 



 
#line 227 "..\\HALLIB\\STM32L4xx_HAL_Driver\\Inc\\stm32l4xx_hal_tim_ex.h"


 



 




 



 




 



 
 

 


 



 
 

 


 





#line 283 "..\\HALLIB\\STM32L4xx_HAL_Driver\\Inc\\stm32l4xx_hal_tim_ex.h"









 
 

 


 




 
 
HAL_StatusTypeDef HAL_TIMEx_HallSensor_Init(TIM_HandleTypeDef *htim, TIM_HallSensor_InitTypeDef *sConfig);
HAL_StatusTypeDef HAL_TIMEx_HallSensor_DeInit(TIM_HandleTypeDef *htim);

void HAL_TIMEx_HallSensor_MspInit(TIM_HandleTypeDef *htim);
void HAL_TIMEx_HallSensor_MspDeInit(TIM_HandleTypeDef *htim);

 
HAL_StatusTypeDef HAL_TIMEx_HallSensor_Start(TIM_HandleTypeDef *htim);
HAL_StatusTypeDef HAL_TIMEx_HallSensor_Stop(TIM_HandleTypeDef *htim);
 
HAL_StatusTypeDef HAL_TIMEx_HallSensor_Start_IT(TIM_HandleTypeDef *htim);
HAL_StatusTypeDef HAL_TIMEx_HallSensor_Stop_IT(TIM_HandleTypeDef *htim);
 
HAL_StatusTypeDef HAL_TIMEx_HallSensor_Start_DMA(TIM_HandleTypeDef *htim, uint32_t *pData, uint16_t Length);
HAL_StatusTypeDef HAL_TIMEx_HallSensor_Stop_DMA(TIM_HandleTypeDef *htim);


 




 
 
 
HAL_StatusTypeDef HAL_TIMEx_OCN_Start(TIM_HandleTypeDef *htim, uint32_t Channel);
HAL_StatusTypeDef HAL_TIMEx_OCN_Stop(TIM_HandleTypeDef *htim, uint32_t Channel);

 
HAL_StatusTypeDef HAL_TIMEx_OCN_Start_IT(TIM_HandleTypeDef *htim, uint32_t Channel);
HAL_StatusTypeDef HAL_TIMEx_OCN_Stop_IT(TIM_HandleTypeDef *htim, uint32_t Channel);

 
HAL_StatusTypeDef HAL_TIMEx_OCN_Start_DMA(TIM_HandleTypeDef *htim, uint32_t Channel, uint32_t *pData, uint16_t Length);
HAL_StatusTypeDef HAL_TIMEx_OCN_Stop_DMA(TIM_HandleTypeDef *htim, uint32_t Channel);


 




 
 
 
HAL_StatusTypeDef HAL_TIMEx_PWMN_Start(TIM_HandleTypeDef *htim, uint32_t Channel);
HAL_StatusTypeDef HAL_TIMEx_PWMN_Stop(TIM_HandleTypeDef *htim, uint32_t Channel);

 
HAL_StatusTypeDef HAL_TIMEx_PWMN_Start_IT(TIM_HandleTypeDef *htim, uint32_t Channel);
HAL_StatusTypeDef HAL_TIMEx_PWMN_Stop_IT(TIM_HandleTypeDef *htim, uint32_t Channel);
 
HAL_StatusTypeDef HAL_TIMEx_PWMN_Start_DMA(TIM_HandleTypeDef *htim, uint32_t Channel, uint32_t *pData, uint16_t Length);
HAL_StatusTypeDef HAL_TIMEx_PWMN_Stop_DMA(TIM_HandleTypeDef *htim, uint32_t Channel);


 




 
 
 
HAL_StatusTypeDef HAL_TIMEx_OnePulseN_Start(TIM_HandleTypeDef *htim, uint32_t OutputChannel);
HAL_StatusTypeDef HAL_TIMEx_OnePulseN_Stop(TIM_HandleTypeDef *htim, uint32_t OutputChannel);

 
HAL_StatusTypeDef HAL_TIMEx_OnePulseN_Start_IT(TIM_HandleTypeDef *htim, uint32_t OutputChannel);
HAL_StatusTypeDef HAL_TIMEx_OnePulseN_Stop_IT(TIM_HandleTypeDef *htim, uint32_t OutputChannel);


 




 
 
HAL_StatusTypeDef HAL_TIMEx_ConfigCommutationEvent(TIM_HandleTypeDef *htim, uint32_t  InputTrigger, uint32_t  CommutationSource);
HAL_StatusTypeDef HAL_TIMEx_ConfigCommutationEvent_IT(TIM_HandleTypeDef *htim, uint32_t  InputTrigger, uint32_t  CommutationSource);
HAL_StatusTypeDef HAL_TIMEx_ConfigCommutationEvent_DMA(TIM_HandleTypeDef *htim, uint32_t  InputTrigger, uint32_t  CommutationSource);
HAL_StatusTypeDef HAL_TIMEx_MasterConfigSynchronization(TIM_HandleTypeDef *htim, TIM_MasterConfigTypeDef *sMasterConfig);
HAL_StatusTypeDef HAL_TIMEx_ConfigBreakDeadTime(TIM_HandleTypeDef *htim, TIM_BreakDeadTimeConfigTypeDef *sBreakDeadTimeConfig);
HAL_StatusTypeDef HAL_TIMEx_ConfigBreakInput(TIM_HandleTypeDef *htim, uint32_t BreakInput, TIMEx_BreakInputConfigTypeDef *sBreakInputConfig);
HAL_StatusTypeDef HAL_TIMEx_GroupChannel5(TIM_HandleTypeDef *htim, uint32_t Channels);
HAL_StatusTypeDef HAL_TIMEx_RemapConfig(TIM_HandleTypeDef *htim, uint32_t Remap);


 




 
 
void HAL_TIMEx_CommutationCallback(TIM_HandleTypeDef *htim);
void HAL_TIMEx_BreakCallback(TIM_HandleTypeDef *htim);
void HAL_TIMEx_Break2Callback(TIM_HandleTypeDef *htim);


 




 
 
HAL_TIM_StateTypeDef HAL_TIMEx_HallSensor_GetState(TIM_HandleTypeDef *htim);


 



 
 

 


 
void TIMEx_DMACommutationCplt(DMA_HandleTypeDef *hdma);


 
 



 



 








 
#line 1888 "..\\HALLIB\\STM32L4xx_HAL_Driver\\Inc\\stm32l4xx_hal_tim.h"

 


 




 
 
HAL_StatusTypeDef HAL_TIM_Base_Init(TIM_HandleTypeDef *htim);
HAL_StatusTypeDef HAL_TIM_Base_DeInit(TIM_HandleTypeDef *htim);
void HAL_TIM_Base_MspInit(TIM_HandleTypeDef *htim);
void HAL_TIM_Base_MspDeInit(TIM_HandleTypeDef *htim);
 
HAL_StatusTypeDef HAL_TIM_Base_Start(TIM_HandleTypeDef *htim);
HAL_StatusTypeDef HAL_TIM_Base_Stop(TIM_HandleTypeDef *htim);
 
HAL_StatusTypeDef HAL_TIM_Base_Start_IT(TIM_HandleTypeDef *htim);
HAL_StatusTypeDef HAL_TIM_Base_Stop_IT(TIM_HandleTypeDef *htim);
 
HAL_StatusTypeDef HAL_TIM_Base_Start_DMA(TIM_HandleTypeDef *htim, uint32_t *pData, uint16_t Length);
HAL_StatusTypeDef HAL_TIM_Base_Stop_DMA(TIM_HandleTypeDef *htim);


 




 
 
HAL_StatusTypeDef HAL_TIM_OC_Init(TIM_HandleTypeDef *htim);
HAL_StatusTypeDef HAL_TIM_OC_DeInit(TIM_HandleTypeDef *htim);
void HAL_TIM_OC_MspInit(TIM_HandleTypeDef *htim);
void HAL_TIM_OC_MspDeInit(TIM_HandleTypeDef *htim);
 
HAL_StatusTypeDef HAL_TIM_OC_Start(TIM_HandleTypeDef *htim, uint32_t Channel);
HAL_StatusTypeDef HAL_TIM_OC_Stop(TIM_HandleTypeDef *htim, uint32_t Channel);
 
HAL_StatusTypeDef HAL_TIM_OC_Start_IT(TIM_HandleTypeDef *htim, uint32_t Channel);
HAL_StatusTypeDef HAL_TIM_OC_Stop_IT(TIM_HandleTypeDef *htim, uint32_t Channel);
 
HAL_StatusTypeDef HAL_TIM_OC_Start_DMA(TIM_HandleTypeDef *htim, uint32_t Channel, uint32_t *pData, uint16_t Length);
HAL_StatusTypeDef HAL_TIM_OC_Stop_DMA(TIM_HandleTypeDef *htim, uint32_t Channel);


 




 
 
HAL_StatusTypeDef HAL_TIM_PWM_Init(TIM_HandleTypeDef *htim);
HAL_StatusTypeDef HAL_TIM_PWM_DeInit(TIM_HandleTypeDef *htim);
void HAL_TIM_PWM_MspInit(TIM_HandleTypeDef *htim);
void HAL_TIM_PWM_MspDeInit(TIM_HandleTypeDef *htim);
 
HAL_StatusTypeDef HAL_TIM_PWM_Start(TIM_HandleTypeDef *htim, uint32_t Channel);
HAL_StatusTypeDef HAL_TIM_PWM_Stop(TIM_HandleTypeDef *htim, uint32_t Channel);
 
HAL_StatusTypeDef HAL_TIM_PWM_Start_IT(TIM_HandleTypeDef *htim, uint32_t Channel);
HAL_StatusTypeDef HAL_TIM_PWM_Stop_IT(TIM_HandleTypeDef *htim, uint32_t Channel);
 
HAL_StatusTypeDef HAL_TIM_PWM_Start_DMA(TIM_HandleTypeDef *htim, uint32_t Channel, uint32_t *pData, uint16_t Length);
HAL_StatusTypeDef HAL_TIM_PWM_Stop_DMA(TIM_HandleTypeDef *htim, uint32_t Channel);


 




 
 
HAL_StatusTypeDef HAL_TIM_IC_Init(TIM_HandleTypeDef *htim);
HAL_StatusTypeDef HAL_TIM_IC_DeInit(TIM_HandleTypeDef *htim);
void HAL_TIM_IC_MspInit(TIM_HandleTypeDef *htim);
void HAL_TIM_IC_MspDeInit(TIM_HandleTypeDef *htim);
 
HAL_StatusTypeDef HAL_TIM_IC_Start(TIM_HandleTypeDef *htim, uint32_t Channel);
HAL_StatusTypeDef HAL_TIM_IC_Stop(TIM_HandleTypeDef *htim, uint32_t Channel);
 
HAL_StatusTypeDef HAL_TIM_IC_Start_IT(TIM_HandleTypeDef *htim, uint32_t Channel);
HAL_StatusTypeDef HAL_TIM_IC_Stop_IT(TIM_HandleTypeDef *htim, uint32_t Channel);
 
HAL_StatusTypeDef HAL_TIM_IC_Start_DMA(TIM_HandleTypeDef *htim, uint32_t Channel, uint32_t *pData, uint16_t Length);
HAL_StatusTypeDef HAL_TIM_IC_Stop_DMA(TIM_HandleTypeDef *htim, uint32_t Channel);


 




 
 
HAL_StatusTypeDef HAL_TIM_OnePulse_Init(TIM_HandleTypeDef *htim, uint32_t OnePulseMode);
HAL_StatusTypeDef HAL_TIM_OnePulse_DeInit(TIM_HandleTypeDef *htim);
void HAL_TIM_OnePulse_MspInit(TIM_HandleTypeDef *htim);
void HAL_TIM_OnePulse_MspDeInit(TIM_HandleTypeDef *htim);
 
HAL_StatusTypeDef HAL_TIM_OnePulse_Start(TIM_HandleTypeDef *htim, uint32_t OutputChannel);
HAL_StatusTypeDef HAL_TIM_OnePulse_Stop(TIM_HandleTypeDef *htim, uint32_t OutputChannel);
 
HAL_StatusTypeDef HAL_TIM_OnePulse_Start_IT(TIM_HandleTypeDef *htim, uint32_t OutputChannel);
HAL_StatusTypeDef HAL_TIM_OnePulse_Stop_IT(TIM_HandleTypeDef *htim, uint32_t OutputChannel);


 




 
 
HAL_StatusTypeDef HAL_TIM_Encoder_Init(TIM_HandleTypeDef *htim,  TIM_Encoder_InitTypeDef *sConfig);
HAL_StatusTypeDef HAL_TIM_Encoder_DeInit(TIM_HandleTypeDef *htim);
void HAL_TIM_Encoder_MspInit(TIM_HandleTypeDef *htim);
void HAL_TIM_Encoder_MspDeInit(TIM_HandleTypeDef *htim);
 
HAL_StatusTypeDef HAL_TIM_Encoder_Start(TIM_HandleTypeDef *htim, uint32_t Channel);
HAL_StatusTypeDef HAL_TIM_Encoder_Stop(TIM_HandleTypeDef *htim, uint32_t Channel);
 
HAL_StatusTypeDef HAL_TIM_Encoder_Start_IT(TIM_HandleTypeDef *htim, uint32_t Channel);
HAL_StatusTypeDef HAL_TIM_Encoder_Stop_IT(TIM_HandleTypeDef *htim, uint32_t Channel);
 
HAL_StatusTypeDef HAL_TIM_Encoder_Start_DMA(TIM_HandleTypeDef *htim, uint32_t Channel, uint32_t *pData1, uint32_t *pData2, uint16_t Length);
HAL_StatusTypeDef HAL_TIM_Encoder_Stop_DMA(TIM_HandleTypeDef *htim, uint32_t Channel);


 




 
 
void HAL_TIM_IRQHandler(TIM_HandleTypeDef *htim);


 




 
 
HAL_StatusTypeDef HAL_TIM_OC_ConfigChannel(TIM_HandleTypeDef *htim, TIM_OC_InitTypeDef *sConfig, uint32_t Channel);
HAL_StatusTypeDef HAL_TIM_PWM_ConfigChannel(TIM_HandleTypeDef *htim, TIM_OC_InitTypeDef *sConfig, uint32_t Channel);
HAL_StatusTypeDef HAL_TIM_IC_ConfigChannel(TIM_HandleTypeDef *htim, TIM_IC_InitTypeDef *sConfig, uint32_t Channel);
HAL_StatusTypeDef HAL_TIM_OnePulse_ConfigChannel(TIM_HandleTypeDef *htim, TIM_OnePulse_InitTypeDef *sConfig, uint32_t OutputChannel,  uint32_t InputChannel);
HAL_StatusTypeDef HAL_TIM_ConfigOCrefClear(TIM_HandleTypeDef *htim, TIM_ClearInputConfigTypeDef *sClearInputConfig, uint32_t Channel);
HAL_StatusTypeDef HAL_TIM_ConfigClockSource(TIM_HandleTypeDef *htim, TIM_ClockConfigTypeDef *sClockSourceConfig);
HAL_StatusTypeDef HAL_TIM_ConfigTI1Input(TIM_HandleTypeDef *htim, uint32_t TI1_Selection);
HAL_StatusTypeDef HAL_TIM_SlaveConfigSynchronization(TIM_HandleTypeDef *htim, TIM_SlaveConfigTypeDef *sSlaveConfig);
HAL_StatusTypeDef HAL_TIM_SlaveConfigSynchronization_IT(TIM_HandleTypeDef *htim, TIM_SlaveConfigTypeDef *sSlaveConfig);
HAL_StatusTypeDef HAL_TIM_DMABurst_WriteStart(TIM_HandleTypeDef *htim, uint32_t BurstBaseAddress, uint32_t BurstRequestSrc,                                               uint32_t  *BurstBuffer, uint32_t  BurstLength);

HAL_StatusTypeDef HAL_TIM_DMABurst_WriteStop(TIM_HandleTypeDef *htim, uint32_t BurstRequestSrc);
HAL_StatusTypeDef HAL_TIM_DMABurst_ReadStart(TIM_HandleTypeDef *htim, uint32_t BurstBaseAddress, uint32_t BurstRequestSrc,                                              uint32_t  *BurstBuffer, uint32_t  BurstLength);

HAL_StatusTypeDef HAL_TIM_DMABurst_ReadStop(TIM_HandleTypeDef *htim, uint32_t BurstRequestSrc);
HAL_StatusTypeDef HAL_TIM_GenerateEvent(TIM_HandleTypeDef *htim, uint32_t EventSource);
uint32_t HAL_TIM_ReadCapturedValue(TIM_HandleTypeDef *htim, uint32_t Channel);


 




 
 
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim);
void HAL_TIM_OC_DelayElapsedCallback(TIM_HandleTypeDef *htim);
void HAL_TIM_IC_CaptureCallback(TIM_HandleTypeDef *htim);
void HAL_TIM_PWM_PulseFinishedCallback(TIM_HandleTypeDef *htim);
void HAL_TIM_TriggerCallback(TIM_HandleTypeDef *htim);
void HAL_TIM_ErrorCallback(TIM_HandleTypeDef *htim);

 







 




 
 
HAL_TIM_StateTypeDef HAL_TIM_Base_GetState(TIM_HandleTypeDef *htim);
HAL_TIM_StateTypeDef HAL_TIM_OC_GetState(TIM_HandleTypeDef *htim);
HAL_TIM_StateTypeDef HAL_TIM_PWM_GetState(TIM_HandleTypeDef *htim);
HAL_TIM_StateTypeDef HAL_TIM_IC_GetState(TIM_HandleTypeDef *htim);
HAL_TIM_StateTypeDef HAL_TIM_OnePulse_GetState(TIM_HandleTypeDef *htim);
HAL_TIM_StateTypeDef HAL_TIM_Encoder_GetState(TIM_HandleTypeDef *htim);


 



 
 

 


 
void TIM_Base_SetConfig(TIM_TypeDef *TIMx, TIM_Base_InitTypeDef *Structure);
void TIM_TI1_SetConfig(TIM_TypeDef *TIMx, uint32_t TIM_ICPolarity, uint32_t TIM_ICSelection, uint32_t TIM_ICFilter);
void TIM_OC2_SetConfig(TIM_TypeDef *TIMx, TIM_OC_InitTypeDef *OC_Config);
void TIM_ETR_SetConfig(TIM_TypeDef *TIMx, uint32_t TIM_ExtTRGPrescaler,
                       uint32_t TIM_ExtTRGPolarity, uint32_t ExtTRGFilter);

void TIM_DMADelayPulseCplt(DMA_HandleTypeDef *hdma);
void TIM_DMAError(DMA_HandleTypeDef *hdma);
void TIM_DMACaptureCplt(DMA_HandleTypeDef *hdma);
void TIM_CCxChannelCmd(TIM_TypeDef *TIMx, uint32_t Channel, uint32_t ChannelState);







 
 



 



 







 
#line 374 "..\\USER\\stm32l4xx_hal_conf.h"


#line 1 "..\\HALLIB\\STM32L4xx_HAL_Driver\\Inc\\stm32l4xx_hal_tsc.h"

































 

 







 
#line 46 "..\\HALLIB\\STM32L4xx_HAL_Driver\\Inc\\stm32l4xx_hal_tsc.h"



 



 

 


 



 
typedef enum
{
  HAL_TSC_STATE_RESET  = 0x00UL,  
  HAL_TSC_STATE_READY  = 0x01UL,  
  HAL_TSC_STATE_BUSY   = 0x02UL,  
  HAL_TSC_STATE_ERROR  = 0x03UL   
} HAL_TSC_StateTypeDef;



 
typedef enum
{
  TSC_GROUP_ONGOING   = 0x00UL,  
  TSC_GROUP_COMPLETED = 0x01UL  
} TSC_GroupStatusTypeDef;



 
typedef struct
{
  uint32_t CTPulseHighLength;       
 
  uint32_t CTPulseLowLength;        
 
  uint32_t SpreadSpectrum;          
 
  uint32_t SpreadSpectrumDeviation; 
 
  uint32_t SpreadSpectrumPrescaler; 
 
  uint32_t PulseGeneratorPrescaler; 
 
  uint32_t MaxCountValue;           
 
  uint32_t IODefaultMode;           
 
  uint32_t SynchroPinPolarity;      
 
  uint32_t AcquisitionMode;         
 
  uint32_t MaxCountInterrupt;       
 
  uint32_t ChannelIOs;               
  uint32_t ShieldIOs;                
  uint32_t SamplingIOs;              
} TSC_InitTypeDef;



 
typedef struct
{
  uint32_t ChannelIOs;   
  uint32_t ShieldIOs;    
  uint32_t SamplingIOs;  
} TSC_IOConfigTypeDef;



 
typedef struct __TSC_HandleTypeDef
{
  TSC_TypeDef               *Instance;   
  TSC_InitTypeDef           Init;        
  volatile HAL_TSC_StateTypeDef State;       
  HAL_LockTypeDef           Lock;        
  volatile uint32_t             ErrorCode;   

#line 141 "..\\HALLIB\\STM32L4xx_HAL_Driver\\Inc\\stm32l4xx_hal_tsc.h"
} TSC_HandleTypeDef;



 
typedef enum
{
  TSC_GROUP1_IDX = 0x00UL,
  TSC_GROUP2_IDX,
  TSC_GROUP3_IDX,
  TSC_GROUP4_IDX,

  TSC_GROUP5_IDX,


  TSC_GROUP6_IDX,


  TSC_GROUP7_IDX,


  TSC_GROUP8_IDX,

  TSC_NB_OF_GROUPS
}TSC_GroupIndexTypeDef;

#line 187 "..\\HALLIB\\STM32L4xx_HAL_Driver\\Inc\\stm32l4xx_hal_tsc.h"



 

 


 




 






 



 
#line 228 "..\\HALLIB\\STM32L4xx_HAL_Driver\\Inc\\stm32l4xx_hal_tsc.h"


 



 
#line 251 "..\\HALLIB\\STM32L4xx_HAL_Driver\\Inc\\stm32l4xx_hal_tsc.h"


 



 




 



 
#line 275 "..\\HALLIB\\STM32L4xx_HAL_Driver\\Inc\\stm32l4xx_hal_tsc.h"


 



 
#line 289 "..\\HALLIB\\STM32L4xx_HAL_Driver\\Inc\\stm32l4xx_hal_tsc.h"


 



 




 



 




 



 




 



 




 



 




 



 
#line 357 "..\\HALLIB\\STM32L4xx_HAL_Driver\\Inc\\stm32l4xx_hal_tsc.h"
























#line 393 "..\\HALLIB\\STM32L4xx_HAL_Driver\\Inc\\stm32l4xx_hal_tsc.h"

#line 406 "..\\HALLIB\\STM32L4xx_HAL_Driver\\Inc\\stm32l4xx_hal_tsc.h"

#line 419 "..\\HALLIB\\STM32L4xx_HAL_Driver\\Inc\\stm32l4xx_hal_tsc.h"

#line 431 "..\\HALLIB\\STM32L4xx_HAL_Driver\\Inc\\stm32l4xx_hal_tsc.h"


 



 

 



 




 
#line 458 "..\\HALLIB\\STM32L4xx_HAL_Driver\\Inc\\stm32l4xx_hal_tsc.h"





 






 






 






 






 






 






 






 







 







 






 







 







 







 







 







 







 







 







 







 







 







 







 






 





 

 



 

#line 668 "..\\HALLIB\\STM32L4xx_HAL_Driver\\Inc\\stm32l4xx_hal_tsc.h"

#line 685 "..\\HALLIB\\STM32L4xx_HAL_Driver\\Inc\\stm32l4xx_hal_tsc.h"







#line 700 "..\\HALLIB\\STM32L4xx_HAL_Driver\\Inc\\stm32l4xx_hal_tsc.h"

#line 708 "..\\HALLIB\\STM32L4xx_HAL_Driver\\Inc\\stm32l4xx_hal_tsc.h"












#line 753 "..\\HALLIB\\STM32L4xx_HAL_Driver\\Inc\\stm32l4xx_hal_tsc.h"



 

 


 



 
 
HAL_StatusTypeDef HAL_TSC_Init(TSC_HandleTypeDef *htsc);
HAL_StatusTypeDef HAL_TSC_DeInit(TSC_HandleTypeDef *htsc);
void HAL_TSC_MspInit(TSC_HandleTypeDef *htsc);
void HAL_TSC_MspDeInit(TSC_HandleTypeDef *htsc);

 






 



 
 
HAL_StatusTypeDef HAL_TSC_Start(TSC_HandleTypeDef *htsc);
HAL_StatusTypeDef HAL_TSC_Start_IT(TSC_HandleTypeDef *htsc);
HAL_StatusTypeDef HAL_TSC_Stop(TSC_HandleTypeDef *htsc);
HAL_StatusTypeDef HAL_TSC_Stop_IT(TSC_HandleTypeDef *htsc);
HAL_StatusTypeDef HAL_TSC_PollForAcquisition(TSC_HandleTypeDef *htsc);
TSC_GroupStatusTypeDef HAL_TSC_GroupGetStatus(TSC_HandleTypeDef *htsc, uint32_t gx_index);
uint32_t HAL_TSC_GroupGetValue(TSC_HandleTypeDef *htsc, uint32_t gx_index);


 



 
 
HAL_StatusTypeDef HAL_TSC_IOConfig(TSC_HandleTypeDef *htsc, TSC_IOConfigTypeDef *config);
HAL_StatusTypeDef HAL_TSC_IODischarge(TSC_HandleTypeDef *htsc, uint32_t choice);


 



 
 
HAL_TSC_StateTypeDef HAL_TSC_GetState(TSC_HandleTypeDef *htsc);


 



 
 
void HAL_TSC_IRQHandler(TSC_HandleTypeDef *htsc);
void HAL_TSC_ConvCpltCallback(TSC_HandleTypeDef *htsc);
void HAL_TSC_ErrorCallback(TSC_HandleTypeDef *htsc);


 



 



 



 







 
#line 378 "..\\USER\\stm32l4xx_hal_conf.h"


#line 1 "..\\HALLIB\\STM32L4xx_HAL_Driver\\Inc\\stm32l4xx_hal_uart.h"

































 

 







 
#line 46 "..\\HALLIB\\STM32L4xx_HAL_Driver\\Inc\\stm32l4xx_hal_uart.h"



 



 

 


 



 
typedef struct
{
  uint32_t BaudRate;                  













 

  uint32_t WordLength;                
 

  uint32_t StopBits;                  
 

  uint32_t Parity;                    




 

  uint32_t Mode;                      
 

  uint32_t HwFlowCtl;                 

 

  uint32_t OverSampling;              
 

  uint32_t OneBitSampling;            

 






} UART_InitTypeDef;



 
typedef struct
{
  uint32_t AdvFeatureInit;        

 

  uint32_t TxPinLevelInvert;      
 

  uint32_t RxPinLevelInvert;      
 

  uint32_t DataInvert;            

 

  uint32_t Swap;                  
 

  uint32_t OverrunDisable;        
 

  uint32_t DMADisableonRxError;   
 

  uint32_t AutoBaudRateEnable;    
 

  uint32_t AutoBaudRateMode;      

 

  uint32_t MSBFirst;              
 
} UART_AdvFeatureInitTypeDef;









































 
typedef enum
{
  HAL_UART_STATE_RESET             = 0x00U,   
 
  HAL_UART_STATE_READY             = 0x20U,   
 
  HAL_UART_STATE_BUSY              = 0x24U,   
 
  HAL_UART_STATE_BUSY_TX           = 0x21U,   
 
  HAL_UART_STATE_BUSY_RX           = 0x22U,   
 
  HAL_UART_STATE_BUSY_TX_RX        = 0x23U,   

 
  HAL_UART_STATE_TIMEOUT           = 0xA0U,   
 
  HAL_UART_STATE_ERROR             = 0xE0U    
 
} HAL_UART_StateTypeDef;



 
typedef enum
{
  UART_CLOCKSOURCE_PCLK1      = 0x00U,     
  UART_CLOCKSOURCE_PCLK2      = 0x01U,     
  UART_CLOCKSOURCE_HSI        = 0x02U,     
  UART_CLOCKSOURCE_SYSCLK     = 0x04U,     
  UART_CLOCKSOURCE_LSE        = 0x08U,     
  UART_CLOCKSOURCE_UNDEFINED  = 0x10U      
} UART_ClockSourceTypeDef;



 
typedef struct __UART_HandleTypeDef
{
  USART_TypeDef            *Instance;                 

  UART_InitTypeDef         Init;                      

  UART_AdvFeatureInitTypeDef AdvancedInit;            

  uint8_t                  *pTxBuffPtr;               

  uint16_t                 TxXferSize;                

  volatile uint16_t            TxXferCount;               

  uint8_t                  *pRxBuffPtr;               

  uint16_t                 RxXferSize;                

  volatile uint16_t            RxXferCount;               

  uint16_t                 Mask;                      

#line 262 "..\\HALLIB\\STM32L4xx_HAL_Driver\\Inc\\stm32l4xx_hal_uart.h"

  void (*RxISR)(struct __UART_HandleTypeDef *huart);  

  void (*TxISR)(struct __UART_HandleTypeDef *huart);  

  DMA_HandleTypeDef        *hdmatx;                   

  DMA_HandleTypeDef        *hdmarx;                   

  HAL_LockTypeDef           Lock;                     

  volatile HAL_UART_StateTypeDef    gState;              

 

  volatile HAL_UART_StateTypeDef    RxState;             
 

  volatile uint32_t                 ErrorCode;            

#line 300 "..\\HALLIB\\STM32L4xx_HAL_Driver\\Inc\\stm32l4xx_hal_uart.h"

} UART_HandleTypeDef;

#line 334 "..\\HALLIB\\STM32L4xx_HAL_Driver\\Inc\\stm32l4xx_hal_uart.h"



 

 


 



 
#line 356 "..\\HALLIB\\STM32L4xx_HAL_Driver\\Inc\\stm32l4xx_hal_uart.h"


 



 






 



 





 



 






 



 





 



 




 



 




 



 




 

#line 450 "..\\HALLIB\\STM32L4xx_HAL_Driver\\Inc\\stm32l4xx_hal_uart.h"



 






 



 




 



 




 



 




 



 




 



 




 



 




 



 




 



 







 



 
#line 549 "..\\HALLIB\\STM32L4xx_HAL_Driver\\Inc\\stm32l4xx_hal_uart.h"


 



 




 



 




 



 




 



 




 



 




 



 




 



 




 



 




 



 




 



 




 



 



 



 





 



 




 



 



 



 



 



 



 



 



 





 
#line 741 "..\\HALLIB\\STM32L4xx_HAL_Driver\\Inc\\stm32l4xx_hal_uart.h"


 










 
#line 776 "..\\HALLIB\\STM32L4xx_HAL_Driver\\Inc\\stm32l4xx_hal_uart.h"






 



 





 



 
#line 810 "..\\HALLIB\\STM32L4xx_HAL_Driver\\Inc\\stm32l4xx_hal_uart.h"


 




 

 


 




 
#line 841 "..\\HALLIB\\STM32L4xx_HAL_Driver\\Inc\\stm32l4xx_hal_uart.h"




 






















 





 





 





 





 





 


#line 908 "..\\HALLIB\\STM32L4xx_HAL_Driver\\Inc\\stm32l4xx_hal_uart.h"































 























 


























 

























 























 





















 












 





 





 





 





 













 

















 

















 

















 







 

 


 
#line 1224 "..\\HALLIB\\STM32L4xx_HAL_Driver\\Inc\\stm32l4xx_hal_uart.h"





 






 






 






 








 









 





 






 









 







 








 










 






 







 







 







 









 







 







 







 







 







 







 







 










 
#line 1438 "..\\HALLIB\\STM32L4xx_HAL_Driver\\Inc\\stm32l4xx_hal_uart.h"





 







 







 







 







 







 







 







 







 







 







 








 



#line 1555 "..\\HALLIB\\STM32L4xx_HAL_Driver\\Inc\\stm32l4xx_hal_uart.h"



 

 
#line 1 "..\\HALLIB\\STM32L4xx_HAL_Driver\\Inc\\stm32l4xx_hal_uart_ex.h"

































 

 







 
#line 46 "..\\HALLIB\\STM32L4xx_HAL_Driver\\Inc\\stm32l4xx_hal_uart_ex.h"



 



 

 


 



 
typedef struct
{
  uint32_t WakeUpEvent;        


 

  uint16_t AddressLength;      
 

  uint8_t Address;              
} UART_WakeUpTypeDef;



 

 


 



 





 



 




 

#line 143 "..\\HALLIB\\STM32L4xx_HAL_Driver\\Inc\\stm32l4xx_hal_uart_ex.h"



 

 
 


 



 

 
HAL_StatusTypeDef HAL_RS485Ex_Init(UART_HandleTypeDef *huart, uint32_t Polarity, uint32_t AssertionTime, uint32_t DeassertionTime);



 



 

void HAL_UARTEx_WakeupCallback(UART_HandleTypeDef *huart);








 



 

 
HAL_StatusTypeDef HAL_UARTEx_StopModeWakeUpSourceConfig(UART_HandleTypeDef *huart, UART_WakeUpTypeDef WakeUpSelection);
HAL_StatusTypeDef HAL_UARTEx_EnableStopMode(UART_HandleTypeDef *huart);
HAL_StatusTypeDef HAL_UARTEx_DisableStopMode(UART_HandleTypeDef *huart);

HAL_StatusTypeDef HAL_UARTEx_EnableClockStopMode(UART_HandleTypeDef *huart);
HAL_StatusTypeDef HAL_UARTEx_DisableClockStopMode(UART_HandleTypeDef *huart);

HAL_StatusTypeDef HAL_MultiProcessorEx_AddressLength_Set(UART_HandleTypeDef *huart, uint32_t AddressLength);
#line 199 "..\\HALLIB\\STM32L4xx_HAL_Driver\\Inc\\stm32l4xx_hal_uart_ex.h"



 



 

 


 





 
#line 616 "..\\HALLIB\\STM32L4xx_HAL_Driver\\Inc\\stm32l4xx_hal_uart_ex.h"









 
#line 662 "..\\HALLIB\\STM32L4xx_HAL_Driver\\Inc\\stm32l4xx_hal_uart_ex.h"






 








 



#line 706 "..\\HALLIB\\STM32L4xx_HAL_Driver\\Inc\\stm32l4xx_hal_uart_ex.h"



 

 



 



 







 
#line 1562 "..\\HALLIB\\STM32L4xx_HAL_Driver\\Inc\\stm32l4xx_hal_uart.h"


 


 



 

 
HAL_StatusTypeDef HAL_UART_Init(UART_HandleTypeDef *huart);
HAL_StatusTypeDef HAL_HalfDuplex_Init(UART_HandleTypeDef *huart);
HAL_StatusTypeDef HAL_LIN_Init(UART_HandleTypeDef *huart, uint32_t BreakDetectLength);
HAL_StatusTypeDef HAL_MultiProcessor_Init(UART_HandleTypeDef *huart, uint8_t Address, uint32_t WakeUpMethod);
HAL_StatusTypeDef HAL_UART_DeInit(UART_HandleTypeDef *huart);
void HAL_UART_MspInit(UART_HandleTypeDef *huart);
void HAL_UART_MspDeInit(UART_HandleTypeDef *huart);

 







 



 

 
HAL_StatusTypeDef HAL_UART_Transmit(UART_HandleTypeDef *huart, uint8_t *pData, uint16_t Size, uint32_t Timeout);
HAL_StatusTypeDef HAL_UART_Receive(UART_HandleTypeDef *huart, uint8_t *pData, uint16_t Size, uint32_t Timeout);
HAL_StatusTypeDef HAL_UART_Transmit_IT(UART_HandleTypeDef *huart, uint8_t *pData, uint16_t Size);
HAL_StatusTypeDef HAL_UART_Receive_IT(UART_HandleTypeDef *huart, uint8_t *pData, uint16_t Size);
HAL_StatusTypeDef HAL_UART_Transmit_DMA(UART_HandleTypeDef *huart, uint8_t *pData, uint16_t Size);
HAL_StatusTypeDef HAL_UART_Receive_DMA(UART_HandleTypeDef *huart, uint8_t *pData, uint16_t Size);
HAL_StatusTypeDef HAL_UART_DMAPause(UART_HandleTypeDef *huart);
HAL_StatusTypeDef HAL_UART_DMAResume(UART_HandleTypeDef *huart);
HAL_StatusTypeDef HAL_UART_DMAStop(UART_HandleTypeDef *huart);
 
HAL_StatusTypeDef HAL_UART_Abort(UART_HandleTypeDef *huart);
HAL_StatusTypeDef HAL_UART_AbortTransmit(UART_HandleTypeDef *huart);
HAL_StatusTypeDef HAL_UART_AbortReceive(UART_HandleTypeDef *huart);
HAL_StatusTypeDef HAL_UART_Abort_IT(UART_HandleTypeDef *huart);
HAL_StatusTypeDef HAL_UART_AbortTransmit_IT(UART_HandleTypeDef *huart);
HAL_StatusTypeDef HAL_UART_AbortReceive_IT(UART_HandleTypeDef *huart);

void HAL_UART_IRQHandler(UART_HandleTypeDef *huart);
void HAL_UART_TxHalfCpltCallback(UART_HandleTypeDef *huart);
void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart);
void HAL_UART_RxHalfCpltCallback(UART_HandleTypeDef *huart);
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart);
void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart);
void HAL_UART_AbortCpltCallback(UART_HandleTypeDef *huart);
void HAL_UART_AbortTransmitCpltCallback(UART_HandleTypeDef *huart);
void HAL_UART_AbortReceiveCpltCallback(UART_HandleTypeDef *huart);



 



 

 
HAL_StatusTypeDef HAL_LIN_SendBreak(UART_HandleTypeDef *huart);
HAL_StatusTypeDef HAL_MultiProcessor_EnableMuteMode(UART_HandleTypeDef *huart);
HAL_StatusTypeDef HAL_MultiProcessor_DisableMuteMode(UART_HandleTypeDef *huart);
void HAL_MultiProcessor_EnterMuteMode(UART_HandleTypeDef *huart);
HAL_StatusTypeDef HAL_HalfDuplex_EnableTransmitter(UART_HandleTypeDef *huart);
HAL_StatusTypeDef HAL_HalfDuplex_EnableReceiver(UART_HandleTypeDef *huart);


 



 

 
HAL_UART_StateTypeDef HAL_UART_GetState(UART_HandleTypeDef *huart);
uint32_t              HAL_UART_GetError(UART_HandleTypeDef *huart);



 



 

 


 

HAL_StatusTypeDef UART_SetConfig(UART_HandleTypeDef *huart);
HAL_StatusTypeDef UART_CheckIdleState(UART_HandleTypeDef *huart);
HAL_StatusTypeDef UART_WaitOnFlagUntilTimeout(UART_HandleTypeDef *huart, uint32_t Flag, FlagStatus Status, uint32_t Tickstart, uint32_t Timeout);
void UART_AdvFeatureConfig(UART_HandleTypeDef *huart);



 



 



 







 
#line 382 "..\\USER\\stm32l4xx_hal_conf.h"


#line 1 "..\\HALLIB\\STM32L4xx_HAL_Driver\\Inc\\stm32l4xx_hal_usart.h"

































 

 







 
#line 46 "..\\HALLIB\\STM32L4xx_HAL_Driver\\Inc\\stm32l4xx_hal_usart.h"



 



 

 


 



 
typedef struct
{
  uint32_t BaudRate;                  





 

  uint32_t WordLength;                
 

  uint32_t StopBits;                  
 

  uint32_t Parity;                   




 

  uint32_t Mode;                      
 

  uint32_t CLKPolarity;               
 

  uint32_t CLKPhase;                  
 

  uint32_t CLKLastBit;                

 





} USART_InitTypeDef;



 
typedef enum
{
  HAL_USART_STATE_RESET             = 0x00U,     
  HAL_USART_STATE_READY             = 0x01U,     
  HAL_USART_STATE_BUSY              = 0x02U,     
  HAL_USART_STATE_BUSY_TX           = 0x12U,     
  HAL_USART_STATE_BUSY_RX           = 0x22U,     
  HAL_USART_STATE_BUSY_TX_RX        = 0x32U,     
  HAL_USART_STATE_TIMEOUT           = 0x03U,     
  HAL_USART_STATE_ERROR             = 0x04U      
} HAL_USART_StateTypeDef;



 
typedef enum
{
  USART_CLOCKSOURCE_PCLK1      = 0x00U,     
  USART_CLOCKSOURCE_PCLK2      = 0x01U,     
  USART_CLOCKSOURCE_HSI        = 0x02U,     
  USART_CLOCKSOURCE_SYSCLK     = 0x04U,     
  USART_CLOCKSOURCE_LSE        = 0x08U,     
  USART_CLOCKSOURCE_UNDEFINED  = 0x10U      
} USART_ClockSourceTypeDef;



 
typedef struct __USART_HandleTypeDef
{
  USART_TypeDef                 *Instance;                

  USART_InitTypeDef             Init;                     

  uint8_t                       *pTxBuffPtr;              

  uint16_t                      TxXferSize;               

  volatile uint16_t                 TxXferCount;              

  uint8_t                       *pRxBuffPtr;              

  uint16_t                      RxXferSize;               

  volatile uint16_t                 RxXferCount;              

  uint16_t                      Mask;                     

#line 172 "..\\HALLIB\\STM32L4xx_HAL_Driver\\Inc\\stm32l4xx_hal_usart.h"
  void (*RxISR)(struct __USART_HandleTypeDef *husart);    

  void (*TxISR)(struct __USART_HandleTypeDef *husart);    

  DMA_HandleTypeDef             *hdmatx;                  

  DMA_HandleTypeDef             *hdmarx;                  

  HAL_LockTypeDef               Lock;                     

  volatile HAL_USART_StateTypeDef   State;                    

  volatile uint32_t                 ErrorCode;                

#line 202 "..\\HALLIB\\STM32L4xx_HAL_Driver\\Inc\\stm32l4xx_hal_usart.h"

} USART_HandleTypeDef;

#line 234 "..\\HALLIB\\STM32L4xx_HAL_Driver\\Inc\\stm32l4xx_hal_usart.h"



 

 


 



 
#line 259 "..\\HALLIB\\STM32L4xx_HAL_Driver\\Inc\\stm32l4xx_hal_usart.h"


 



 






 



 





 



 





 



 




 



 




 



 




 



 




 



 




 

#line 360 "..\\HALLIB\\STM32L4xx_HAL_Driver\\Inc\\stm32l4xx_hal_usart.h"



 




 





 
#line 405 "..\\HALLIB\\STM32L4xx_HAL_Driver\\Inc\\stm32l4xx_hal_usart.h"


 










 

#line 441 "..\\HALLIB\\STM32L4xx_HAL_Driver\\Inc\\stm32l4xx_hal_usart.h"



 



 
#line 461 "..\\HALLIB\\STM32L4xx_HAL_Driver\\Inc\\stm32l4xx_hal_usart.h"


 



 







 



 

 


 




 
#line 499 "..\\HALLIB\\STM32L4xx_HAL_Driver\\Inc\\stm32l4xx_hal_usart.h"
























 















 





 





 





 





 





 


#line 579 "..\\HALLIB\\STM32L4xx_HAL_Driver\\Inc\\stm32l4xx_hal_usart.h"

#line 587 "..\\HALLIB\\STM32L4xx_HAL_Driver\\Inc\\stm32l4xx_hal_usart.h"


















 





















 
























 





















 


















 










 





 





 





 





 




 

 


 

#line 767 "..\\HALLIB\\STM32L4xx_HAL_Driver\\Inc\\stm32l4xx_hal_usart.h"




 









 










 









 








 






 







 







 






 






 







 



#line 880 "..\\HALLIB\\STM32L4xx_HAL_Driver\\Inc\\stm32l4xx_hal_usart.h"


 

 
#line 1 "..\\HALLIB\\STM32L4xx_HAL_Driver\\Inc\\stm32l4xx_hal_usart_ex.h"

































 

 







 
#line 46 "..\\HALLIB\\STM32L4xx_HAL_Driver\\Inc\\stm32l4xx_hal_usart_ex.h"



 



 

 
 


 



 





 

#line 92 "..\\HALLIB\\STM32L4xx_HAL_Driver\\Inc\\stm32l4xx_hal_usart_ex.h"

#line 132 "..\\HALLIB\\STM32L4xx_HAL_Driver\\Inc\\stm32l4xx_hal_usart_ex.h"



 

 


 





 
#line 269 "..\\HALLIB\\STM32L4xx_HAL_Driver\\Inc\\stm32l4xx_hal_usart_ex.h"









 
#line 319 "..\\HALLIB\\STM32L4xx_HAL_Driver\\Inc\\stm32l4xx_hal_usart_ex.h"






 




#line 347 "..\\HALLIB\\STM32L4xx_HAL_Driver\\Inc\\stm32l4xx_hal_usart_ex.h"

#line 381 "..\\HALLIB\\STM32L4xx_HAL_Driver\\Inc\\stm32l4xx_hal_usart_ex.h"


 

 


 



 

 







 



 

 
#line 420 "..\\HALLIB\\STM32L4xx_HAL_Driver\\Inc\\stm32l4xx_hal_usart_ex.h"



 



 



 



 







 
#line 886 "..\\HALLIB\\STM32L4xx_HAL_Driver\\Inc\\stm32l4xx_hal_usart.h"

 


 



 

 
HAL_StatusTypeDef HAL_USART_Init(USART_HandleTypeDef *husart);
HAL_StatusTypeDef HAL_USART_DeInit(USART_HandleTypeDef *husart);
void HAL_USART_MspInit(USART_HandleTypeDef *husart);
void HAL_USART_MspDeInit(USART_HandleTypeDef *husart);

 







 



 

 
HAL_StatusTypeDef HAL_USART_Transmit(USART_HandleTypeDef *husart, uint8_t *pTxData, uint16_t Size, uint32_t Timeout);
HAL_StatusTypeDef HAL_USART_Receive(USART_HandleTypeDef *husart, uint8_t *pRxData, uint16_t Size, uint32_t Timeout);
HAL_StatusTypeDef HAL_USART_TransmitReceive(USART_HandleTypeDef *husart, uint8_t *pTxData, uint8_t *pRxData, uint16_t Size, uint32_t Timeout);
HAL_StatusTypeDef HAL_USART_Transmit_IT(USART_HandleTypeDef *husart, uint8_t *pTxData, uint16_t Size);
HAL_StatusTypeDef HAL_USART_Receive_IT(USART_HandleTypeDef *husart, uint8_t *pRxData, uint16_t Size);
HAL_StatusTypeDef HAL_USART_TransmitReceive_IT(USART_HandleTypeDef *husart, uint8_t *pTxData, uint8_t *pRxData,  uint16_t Size);
HAL_StatusTypeDef HAL_USART_Transmit_DMA(USART_HandleTypeDef *husart, uint8_t *pTxData, uint16_t Size);
HAL_StatusTypeDef HAL_USART_Receive_DMA(USART_HandleTypeDef *husart, uint8_t *pRxData, uint16_t Size);
HAL_StatusTypeDef HAL_USART_TransmitReceive_DMA(USART_HandleTypeDef *husart, uint8_t *pTxData, uint8_t *pRxData, uint16_t Size);
HAL_StatusTypeDef HAL_USART_DMAPause(USART_HandleTypeDef *husart);
HAL_StatusTypeDef HAL_USART_DMAResume(USART_HandleTypeDef *husart);
HAL_StatusTypeDef HAL_USART_DMAStop(USART_HandleTypeDef *husart);
 
HAL_StatusTypeDef HAL_USART_Abort(USART_HandleTypeDef *husart);
HAL_StatusTypeDef HAL_USART_Abort_IT(USART_HandleTypeDef *husart);

void HAL_USART_IRQHandler(USART_HandleTypeDef *husart);
void HAL_USART_TxHalfCpltCallback(USART_HandleTypeDef *husart);
void HAL_USART_TxCpltCallback(USART_HandleTypeDef *husart);
void HAL_USART_RxCpltCallback(USART_HandleTypeDef *husart);
void HAL_USART_RxHalfCpltCallback(USART_HandleTypeDef *husart);
void HAL_USART_TxRxCpltCallback(USART_HandleTypeDef *husart);
void HAL_USART_ErrorCallback(USART_HandleTypeDef *husart);
void HAL_USART_AbortCpltCallback(USART_HandleTypeDef *husart);



 



 

 
HAL_USART_StateTypeDef HAL_USART_GetState(USART_HandleTypeDef *husart);
uint32_t               HAL_USART_GetError(USART_HandleTypeDef *husart);



 



 



 



 







 
#line 386 "..\\USER\\stm32l4xx_hal_conf.h"


#line 1 "..\\HALLIB\\STM32L4xx_HAL_Driver\\Inc\\stm32l4xx_hal_irda.h"

































 

 







 
#line 46 "..\\HALLIB\\STM32L4xx_HAL_Driver\\Inc\\stm32l4xx_hal_irda.h"



 



 

 


 



 
typedef struct
{
  uint32_t BaudRate;                  


 

  uint32_t WordLength;                
 

  uint32_t Parity;                    




 

  uint32_t Mode;                      
 

  uint8_t  Prescaler;                 

 

  uint16_t PowerMode;                 
 






} IRDA_InitTypeDef;







































 
typedef uint32_t HAL_IRDA_StateTypeDef;



 
typedef enum
{
  IRDA_CLOCKSOURCE_PCLK1      = 0x00U,     
  IRDA_CLOCKSOURCE_PCLK2      = 0x01U,     
  IRDA_CLOCKSOURCE_HSI        = 0x02U,     
  IRDA_CLOCKSOURCE_SYSCLK     = 0x04U,     
  IRDA_CLOCKSOURCE_LSE        = 0x10U,     
  IRDA_CLOCKSOURCE_UNDEFINED  = 0x20U      
} IRDA_ClockSourceTypeDef;



 



typedef struct

{
  USART_TypeDef            *Instance;         

  IRDA_InitTypeDef         Init;              

  uint8_t                  *pTxBuffPtr;       

  uint16_t                 TxXferSize;        

  volatile uint16_t            TxXferCount;       

  uint8_t                  *pRxBuffPtr;       

  uint16_t                 RxXferSize;        

  volatile uint16_t            RxXferCount;       

  uint16_t                 Mask;              

  DMA_HandleTypeDef        *hdmatx;           

  DMA_HandleTypeDef        *hdmarx;           

  HAL_LockTypeDef          Lock;              

  volatile HAL_IRDA_StateTypeDef    gState;      

 

  volatile HAL_IRDA_StateTypeDef    RxState;     
 

  uint32_t                 ErrorCode;         

#line 215 "..\\HALLIB\\STM32L4xx_HAL_Driver\\Inc\\stm32l4xx_hal_irda.h"

} IRDA_HandleTypeDef;

#line 244 "..\\HALLIB\\STM32L4xx_HAL_Driver\\Inc\\stm32l4xx_hal_irda.h"



 

 


 



 
#line 274 "..\\HALLIB\\STM32L4xx_HAL_Driver\\Inc\\stm32l4xx_hal_irda.h"


 



 
#line 291 "..\\HALLIB\\STM32L4xx_HAL_Driver\\Inc\\stm32l4xx_hal_irda.h"


 



 





 



 





 



 





 



 




 

#line 354 "..\\HALLIB\\STM32L4xx_HAL_Driver\\Inc\\stm32l4xx_hal_irda.h"



 




 



 




 



 




 



 




 



 




 



 





 





 
#line 435 "..\\HALLIB\\STM32L4xx_HAL_Driver\\Inc\\stm32l4xx_hal_irda.h"


 










 











 



 





 



 
#line 481 "..\\HALLIB\\STM32L4xx_HAL_Driver\\Inc\\stm32l4xx_hal_irda.h"


 



 







 



 


 


 




 
#line 524 "..\\HALLIB\\STM32L4xx_HAL_Driver\\Inc\\stm32l4xx_hal_irda.h"




 

















 





 






 





 





 





 



















 














 















 


















 













 















 












 





 





 





 





 




 

 


 





 
#line 771 "..\\HALLIB\\STM32L4xx_HAL_Driver\\Inc\\stm32l4xx_hal_irda.h"




 





 






 








 








 






 



#line 836 "..\\HALLIB\\STM32L4xx_HAL_Driver\\Inc\\stm32l4xx_hal_irda.h"





 







 







 







 







 







 





 

 
#line 1 "..\\HALLIB\\STM32L4xx_HAL_Driver\\Inc\\stm32l4xx_hal_irda_ex.h"

































 

 







 
#line 46 "..\\HALLIB\\STM32L4xx_HAL_Driver\\Inc\\stm32l4xx_hal_irda_ex.h"



 



 

 
 
 
 

 



 





 
#line 400 "..\\HALLIB\\STM32L4xx_HAL_Driver\\Inc\\stm32l4xx_hal_irda_ex.h"



 

 



 



 







 
#line 891 "..\\HALLIB\\STM32L4xx_HAL_Driver\\Inc\\stm32l4xx_hal_irda.h"

 


 



 

 
HAL_StatusTypeDef HAL_IRDA_Init(IRDA_HandleTypeDef *hirda);
HAL_StatusTypeDef HAL_IRDA_DeInit(IRDA_HandleTypeDef *hirda);
void HAL_IRDA_MspInit(IRDA_HandleTypeDef *hirda);
void HAL_IRDA_MspDeInit(IRDA_HandleTypeDef *hirda);









 



 

 
HAL_StatusTypeDef HAL_IRDA_Transmit(IRDA_HandleTypeDef *hirda, uint8_t *pData, uint16_t Size, uint32_t Timeout);
HAL_StatusTypeDef HAL_IRDA_Receive(IRDA_HandleTypeDef *hirda, uint8_t *pData, uint16_t Size, uint32_t Timeout);
HAL_StatusTypeDef HAL_IRDA_Transmit_IT(IRDA_HandleTypeDef *hirda, uint8_t *pData, uint16_t Size);
HAL_StatusTypeDef HAL_IRDA_Receive_IT(IRDA_HandleTypeDef *hirda, uint8_t *pData, uint16_t Size);
HAL_StatusTypeDef HAL_IRDA_Transmit_DMA(IRDA_HandleTypeDef *hirda, uint8_t *pData, uint16_t Size);
HAL_StatusTypeDef HAL_IRDA_Receive_DMA(IRDA_HandleTypeDef *hirda, uint8_t *pData, uint16_t Size);
HAL_StatusTypeDef HAL_IRDA_DMAPause(IRDA_HandleTypeDef *hirda);
HAL_StatusTypeDef HAL_IRDA_DMAResume(IRDA_HandleTypeDef *hirda);
HAL_StatusTypeDef HAL_IRDA_DMAStop(IRDA_HandleTypeDef *hirda);
 
HAL_StatusTypeDef HAL_IRDA_Abort(IRDA_HandleTypeDef *hirda);
HAL_StatusTypeDef HAL_IRDA_AbortTransmit(IRDA_HandleTypeDef *hirda);
HAL_StatusTypeDef HAL_IRDA_AbortReceive(IRDA_HandleTypeDef *hirda);
HAL_StatusTypeDef HAL_IRDA_Abort_IT(IRDA_HandleTypeDef *hirda);
HAL_StatusTypeDef HAL_IRDA_AbortTransmit_IT(IRDA_HandleTypeDef *hirda);
HAL_StatusTypeDef HAL_IRDA_AbortReceive_IT(IRDA_HandleTypeDef *hirda);

void HAL_IRDA_IRQHandler(IRDA_HandleTypeDef *hirda);
void HAL_IRDA_TxCpltCallback(IRDA_HandleTypeDef *hirda);
void HAL_IRDA_RxCpltCallback(IRDA_HandleTypeDef *hirda);
void HAL_IRDA_TxHalfCpltCallback(IRDA_HandleTypeDef *hirda);
void HAL_IRDA_RxHalfCpltCallback(IRDA_HandleTypeDef *hirda);
void HAL_IRDA_ErrorCallback(IRDA_HandleTypeDef *hirda);
void HAL_IRDA_AbortCpltCallback(IRDA_HandleTypeDef *hirda);
void HAL_IRDA_AbortTransmitCpltCallback(IRDA_HandleTypeDef *hirda);
void HAL_IRDA_AbortReceiveCpltCallback(IRDA_HandleTypeDef *hirda);



 

 



 

 
HAL_IRDA_StateTypeDef HAL_IRDA_GetState(IRDA_HandleTypeDef *hirda);
uint32_t              HAL_IRDA_GetError(IRDA_HandleTypeDef *hirda);



 



 



 



 







 
#line 390 "..\\USER\\stm32l4xx_hal_conf.h"


#line 1 "..\\HALLIB\\STM32L4xx_HAL_Driver\\Inc\\stm32l4xx_hal_smartcard.h"

































 

 







 
#line 46 "..\\HALLIB\\STM32L4xx_HAL_Driver\\Inc\\stm32l4xx_hal_smartcard.h"



 



 

 


 



 
typedef struct
{
  uint32_t BaudRate;                  


 

  uint32_t WordLength;                
 

  uint32_t StopBits;                  
 

  uint16_t Parity;                    



 

  uint16_t Mode;                      
 

  uint16_t CLKPolarity;               
 

  uint16_t CLKPhase;                  
 

  uint16_t CLKLastBit;                

 

  uint16_t OneBitSampling;            

 

  uint8_t  Prescaler;                 

 

  uint8_t  GuardTime;                  

  uint16_t NACKEnable;                

 

  uint32_t TimeOutEnable;             
 

  uint32_t TimeOutValue;              

 

  uint8_t BlockLength;                
 

  uint8_t AutoRetryCount;             


 






} SMARTCARD_InitTypeDef;



 
typedef struct
{
  uint32_t AdvFeatureInit;            

 

  uint32_t TxPinLevelInvert;          
 

  uint32_t RxPinLevelInvert;          
 

  uint32_t DataInvert;                

 

  uint32_t Swap;                      
 

  uint32_t OverrunDisable;            
 

  uint32_t DMADisableonRxError;       
 

  uint32_t MSBFirst;                  
 

  uint16_t TxCompletionIndication;     

 
} SMARTCARD_AdvFeatureInitTypeDef;







































 
typedef uint32_t HAL_SMARTCARD_StateTypeDef;



 
typedef struct __SMARTCARD_HandleTypeDef
{
  USART_TypeDef                     *Instance;              

  SMARTCARD_InitTypeDef             Init;                   

  SMARTCARD_AdvFeatureInitTypeDef   AdvancedInit;           

  uint8_t                           *pTxBuffPtr;            

  uint16_t                          TxXferSize;             

  volatile uint16_t                     TxXferCount;            

  uint8_t                           *pRxBuffPtr;            

  uint16_t                          RxXferSize;             

  volatile uint16_t                     RxXferCount;            

#line 239 "..\\HALLIB\\STM32L4xx_HAL_Driver\\Inc\\stm32l4xx_hal_smartcard.h"

  void (*RxISR)(struct __SMARTCARD_HandleTypeDef *huart);   

  void (*TxISR)(struct __SMARTCARD_HandleTypeDef *huart);   

  DMA_HandleTypeDef                 *hdmatx;                

  DMA_HandleTypeDef                 *hdmarx;                

  HAL_LockTypeDef                   Lock;                   

  volatile HAL_SMARTCARD_StateTypeDef   gState;                

 

  volatile HAL_SMARTCARD_StateTypeDef   RxState;               
 

  uint32_t                          ErrorCode;              

#line 281 "..\\HALLIB\\STM32L4xx_HAL_Driver\\Inc\\stm32l4xx_hal_smartcard.h"

} SMARTCARD_HandleTypeDef;

#line 310 "..\\HALLIB\\STM32L4xx_HAL_Driver\\Inc\\stm32l4xx_hal_smartcard.h"



 
typedef enum
{
  SMARTCARD_CLOCKSOURCE_PCLK1     = 0x00,  
  SMARTCARD_CLOCKSOURCE_PCLK2     = 0x01,  
  SMARTCARD_CLOCKSOURCE_HSI       = 0x02,  
  SMARTCARD_CLOCKSOURCE_SYSCLK    = 0x04,  
  SMARTCARD_CLOCKSOURCE_LSE       = 0x08,  
  SMARTCARD_CLOCKSOURCE_UNDEFINED = 0x10   
} SMARTCARD_ClockSourceTypeDef;



 

 


 



 
#line 353 "..\\HALLIB\\STM32L4xx_HAL_Driver\\Inc\\stm32l4xx_hal_smartcard.h"


 



 
#line 370 "..\\HALLIB\\STM32L4xx_HAL_Driver\\Inc\\stm32l4xx_hal_smartcard.h"


 



 



 



 




 



 




 



 





 



 




 



 




 



 




 



 




 




 




 



 




 

#line 486 "..\\HALLIB\\STM32L4xx_HAL_Driver\\Inc\\stm32l4xx_hal_smartcard.h"



 




 



 




 



 




 



 




 



 




 



 




 



 




 



 




 



 







 



 

 


 




 
#line 597 "..\\HALLIB\\STM32L4xx_HAL_Driver\\Inc\\stm32l4xx_hal_smartcard.h"




 





















 





 






 





 





 





 



























 






















 
























 

























 






















 





















 










 





 





 





 





 




 

 


 





 
#line 975 "..\\HALLIB\\STM32L4xx_HAL_Driver\\Inc\\stm32l4xx_hal_smartcard.h"






 






 






 






 






 






 







 







 






 






 






 







 







 







 



#line 1100 "..\\HALLIB\\STM32L4xx_HAL_Driver\\Inc\\stm32l4xx_hal_smartcard.h"





 
#line 1114 "..\\HALLIB\\STM32L4xx_HAL_Driver\\Inc\\stm32l4xx_hal_smartcard.h"





 







 







 







 







 







 







 







 





 

 
#line 1 "..\\HALLIB\\STM32L4xx_HAL_Driver\\Inc\\stm32l4xx_hal_smartcard_ex.h"

































 

 







 
#line 46 "..\\HALLIB\\STM32L4xx_HAL_Driver\\Inc\\stm32l4xx_hal_smartcard_ex.h"



 



 

 
 



 



 






 



 
#line 87 "..\\HALLIB\\STM32L4xx_HAL_Driver\\Inc\\stm32l4xx_hal_smartcard_ex.h"


 

#line 130 "..\\HALLIB\\STM32L4xx_HAL_Driver\\Inc\\stm32l4xx_hal_smartcard_ex.h"





 
#line 168 "..\\HALLIB\\STM32L4xx_HAL_Driver\\Inc\\stm32l4xx_hal_smartcard_ex.h"


 










 
#line 193 "..\\HALLIB\\STM32L4xx_HAL_Driver\\Inc\\stm32l4xx_hal_smartcard_ex.h"












#line 211 "..\\HALLIB\\STM32L4xx_HAL_Driver\\Inc\\stm32l4xx_hal_smartcard_ex.h"


 



 
#line 232 "..\\HALLIB\\STM32L4xx_HAL_Driver\\Inc\\stm32l4xx_hal_smartcard_ex.h"


 



 
 
 


 







 
#line 270 "..\\HALLIB\\STM32L4xx_HAL_Driver\\Inc\\stm32l4xx_hal_smartcard_ex.h"







 
#line 284 "..\\HALLIB\\STM32L4xx_HAL_Driver\\Inc\\stm32l4xx_hal_smartcard_ex.h"






 
#line 297 "..\\HALLIB\\STM32L4xx_HAL_Driver\\Inc\\stm32l4xx_hal_smartcard_ex.h"





 







 
#line 317 "..\\HALLIB\\STM32L4xx_HAL_Driver\\Inc\\stm32l4xx_hal_smartcard_ex.h"





 
#line 329 "..\\HALLIB\\STM32L4xx_HAL_Driver\\Inc\\stm32l4xx_hal_smartcard_ex.h"



 

 


 

 
 



 

 
void              HAL_SMARTCARDEx_BlockLength_Config(SMARTCARD_HandleTypeDef *hsmartcard, uint8_t BlockLength);
void              HAL_SMARTCARDEx_TimeOut_Config(SMARTCARD_HandleTypeDef *hsmartcard, uint32_t TimeOutValue);
HAL_StatusTypeDef HAL_SMARTCARDEx_EnableReceiverTimeOut(SMARTCARD_HandleTypeDef *hsmartcard);
HAL_StatusTypeDef HAL_SMARTCARDEx_DisableReceiverTimeOut(SMARTCARD_HandleTypeDef *hsmartcard);



 

 


 

 







 



 

 
#line 382 "..\\HALLIB\\STM32L4xx_HAL_Driver\\Inc\\stm32l4xx_hal_smartcard_ex.h"



 



 


 



 



 







 
#line 1185 "..\\HALLIB\\STM32L4xx_HAL_Driver\\Inc\\stm32l4xx_hal_smartcard.h"


 


 

 


 

HAL_StatusTypeDef HAL_SMARTCARD_Init(SMARTCARD_HandleTypeDef *hsmartcard);
HAL_StatusTypeDef HAL_SMARTCARD_DeInit(SMARTCARD_HandleTypeDef *hsmartcard);
void HAL_SMARTCARD_MspInit(SMARTCARD_HandleTypeDef *hsmartcard);
void HAL_SMARTCARD_MspDeInit(SMARTCARD_HandleTypeDef *hsmartcard);









 

 


 

HAL_StatusTypeDef HAL_SMARTCARD_Transmit(SMARTCARD_HandleTypeDef *hsmartcard, uint8_t *pData, uint16_t Size, uint32_t Timeout);
HAL_StatusTypeDef HAL_SMARTCARD_Receive(SMARTCARD_HandleTypeDef *hsmartcard, uint8_t *pData, uint16_t Size, uint32_t Timeout);
HAL_StatusTypeDef HAL_SMARTCARD_Transmit_IT(SMARTCARD_HandleTypeDef *hsmartcard, uint8_t *pData, uint16_t Size);
HAL_StatusTypeDef HAL_SMARTCARD_Receive_IT(SMARTCARD_HandleTypeDef *hsmartcard, uint8_t *pData, uint16_t Size);
HAL_StatusTypeDef HAL_SMARTCARD_Transmit_DMA(SMARTCARD_HandleTypeDef *hsmartcard, uint8_t *pData, uint16_t Size);
HAL_StatusTypeDef HAL_SMARTCARD_Receive_DMA(SMARTCARD_HandleTypeDef *hsmartcard, uint8_t *pData, uint16_t Size);
 
HAL_StatusTypeDef HAL_SMARTCARD_Abort(SMARTCARD_HandleTypeDef *hsmartcard);
HAL_StatusTypeDef HAL_SMARTCARD_AbortTransmit(SMARTCARD_HandleTypeDef *hsmartcard);
HAL_StatusTypeDef HAL_SMARTCARD_AbortReceive(SMARTCARD_HandleTypeDef *hsmartcard);
HAL_StatusTypeDef HAL_SMARTCARD_Abort_IT(SMARTCARD_HandleTypeDef *hsmartcard);
HAL_StatusTypeDef HAL_SMARTCARD_AbortTransmit_IT(SMARTCARD_HandleTypeDef *hsmartcard);
HAL_StatusTypeDef HAL_SMARTCARD_AbortReceive_IT(SMARTCARD_HandleTypeDef *hsmartcard);

void HAL_SMARTCARD_IRQHandler(SMARTCARD_HandleTypeDef *hsmartcard);
void HAL_SMARTCARD_TxCpltCallback(SMARTCARD_HandleTypeDef *hsmartcard);
void HAL_SMARTCARD_RxCpltCallback(SMARTCARD_HandleTypeDef *hsmartcard);
void HAL_SMARTCARD_ErrorCallback(SMARTCARD_HandleTypeDef *hsmartcard);
void HAL_SMARTCARD_AbortCpltCallback(SMARTCARD_HandleTypeDef *hsmartcard);
void HAL_SMARTCARD_AbortTransmitCpltCallback(SMARTCARD_HandleTypeDef *hsmartcard);
void HAL_SMARTCARD_AbortReceiveCpltCallback(SMARTCARD_HandleTypeDef *hsmartcard);



 

 


 

HAL_SMARTCARD_StateTypeDef HAL_SMARTCARD_GetState(SMARTCARD_HandleTypeDef *hsmartcard);
uint32_t                   HAL_SMARTCARD_GetError(SMARTCARD_HandleTypeDef *hsmartcard);



 



 



 



 







 
#line 394 "..\\USER\\stm32l4xx_hal_conf.h"


#line 1 "..\\HALLIB\\STM32L4xx_HAL_Driver\\Inc\\stm32l4xx_hal_wwdg.h"

































 

 







 
#line 46 "..\\HALLIB\\STM32L4xx_HAL_Driver\\Inc\\stm32l4xx_hal_wwdg.h"



 



 

 



 



 
typedef struct
{
  uint32_t Prescaler;     
 

  uint32_t Window;        
 

  uint32_t Counter;       
 

  uint32_t EWIMode ;      
 

} WWDG_InitTypeDef;



 
typedef struct __WWDG_HandleTypeDef
{
  WWDG_TypeDef      *Instance;   

  WWDG_InitTypeDef  Init;        






} WWDG_HandleTypeDef;

#line 112 "..\\HALLIB\\STM32L4xx_HAL_Driver\\Inc\\stm32l4xx_hal_wwdg.h"


 

 



 



 



 




 



 



 






 



 




 



 

 



 













 


 



 





 










 









 








 









 









 








 




 

 



 



 
 
HAL_StatusTypeDef     HAL_WWDG_Init(WWDG_HandleTypeDef *hwwdg);
void                  HAL_WWDG_MspInit(WWDG_HandleTypeDef *hwwdg);
 







 



 
 
HAL_StatusTypeDef     HAL_WWDG_Refresh(WWDG_HandleTypeDef *hwwdg);
void                  HAL_WWDG_IRQHandler(WWDG_HandleTypeDef *hwwdg);
void                  HAL_WWDG_EarlyWakeupCallback(WWDG_HandleTypeDef *hwwdg);


 



 



 



 







 
#line 398 "..\\USER\\stm32l4xx_hal_conf.h"


#line 1 "..\\HALLIB\\STM32L4xx_HAL_Driver\\Inc\\stm32l4xx_hal_pcd.h"

































 

 







 
#line 1 "..\\HALLIB\\STM32L4xx_HAL_Driver\\Inc\\stm32l4xx_ll_usb.h"

































 

 







 
#line 46 "..\\HALLIB\\STM32L4xx_HAL_Driver\\Inc\\stm32l4xx_ll_usb.h"





 



 

 



 
typedef enum
{
  USB_DEVICE_MODE  = 0,
  USB_HOST_MODE    = 1,
  USB_DRD_MODE     = 2
} USB_ModeTypeDef;




 
typedef enum
{
  URB_IDLE = 0,
  URB_DONE,
  URB_NOTREADY,
  URB_NYET,
  URB_ERROR,
  URB_STALL
} USB_OTG_URBStateTypeDef;



 
typedef enum
{
  HC_IDLE = 0,
  HC_XFRC,
  HC_HALTED,
  HC_NAK,
  HC_NYET,
  HC_STALL,
  HC_XACTERR,
  HC_BBLERR,
  HC_DATATGLERR
} USB_OTG_HCStateTypeDef;



 
typedef struct
{
  uint32_t dev_endpoints;        

 

  uint32_t Host_channels;        

 

  uint32_t speed;                
 

  uint32_t dma_enable;            

  uint32_t ep0_mps;              
 

  uint32_t phy_itface;           
 

  uint32_t Sof_enable;            

  uint32_t low_power_enable;      

  uint32_t lpm_enable;            

  uint32_t battery_charging_enable;  

  uint32_t vbus_sensing_enable;   

  uint32_t use_dedicated_ep1;     

  uint32_t use_external_vbus;     
} USB_OTG_CfgTypeDef;

typedef struct
{
  uint8_t   num;            
 

  uint8_t   is_in;          
 

  uint8_t   is_stall;       
 

  uint8_t   type;           
 

  uint8_t   data_pid_start; 
 

  uint8_t   even_odd_frame; 
 

  uint16_t  tx_fifo_num;    
 

  uint32_t  maxpacket;      
 

  uint8_t   *xfer_buff;      

  uint32_t  dma_addr;        

  uint32_t  xfer_len;        

  uint32_t  xfer_count;      
} USB_OTG_EPTypeDef;

typedef struct
{
  uint8_t   dev_addr ;     
 

  uint8_t   ch_num;        
 

  uint8_t   ep_num;        
 

  uint8_t   ep_is_in;      
 

  uint8_t   speed;         
 

  uint8_t   do_ping;        

  uint8_t   process_ping;   

  uint8_t   ep_type;       
 

  uint16_t  max_packet;    
 

  uint8_t   data_pid;      
 

  uint8_t   *xfer_buff;     

  uint32_t  xfer_len;       

  uint32_t  xfer_count;     

  uint8_t   toggle_in;     
 

  uint8_t   toggle_out;    
 

  uint32_t  dma_addr;       

  uint32_t  ErrCnt;         

  USB_OTG_URBStateTypeDef  urb_state;  
 

  USB_OTG_HCStateTypeDef   state;     
 
} USB_OTG_HCTypeDef;


#line 299 "..\\HALLIB\\STM32L4xx_HAL_Driver\\Inc\\stm32l4xx_ll_usb.h"

 



 




 





 



 




 



 




 



 




 



 






 



 






 



 






 



 





 



 







 



 







 



 





 



 





 




























#line 485 "..\\HALLIB\\STM32L4xx_HAL_Driver\\Inc\\stm32l4xx_ll_usb.h"


 

 


 









 

 


 

HAL_StatusTypeDef USB_CoreInit(USB_OTG_GlobalTypeDef *USBx, USB_OTG_CfgTypeDef Init);
HAL_StatusTypeDef USB_DevInit(USB_OTG_GlobalTypeDef *USBx, USB_OTG_CfgTypeDef Init);
HAL_StatusTypeDef USB_EnableGlobalInt(USB_OTG_GlobalTypeDef *USBx);
HAL_StatusTypeDef USB_DisableGlobalInt(USB_OTG_GlobalTypeDef *USBx);
HAL_StatusTypeDef USB_SetCurrentMode(USB_OTG_GlobalTypeDef *USBx, USB_ModeTypeDef mode);
HAL_StatusTypeDef USB_SetDevSpeed(USB_OTG_GlobalTypeDef *USBx, uint8_t speed);
HAL_StatusTypeDef USB_FlushRxFifo(USB_OTG_GlobalTypeDef *USBx);
HAL_StatusTypeDef USB_FlushTxFifo(USB_OTG_GlobalTypeDef *USBx, uint32_t num);
HAL_StatusTypeDef USB_ActivateEndpoint(USB_OTG_GlobalTypeDef *USBx, USB_OTG_EPTypeDef *ep);
HAL_StatusTypeDef USB_DeactivateEndpoint(USB_OTG_GlobalTypeDef *USBx, USB_OTG_EPTypeDef *ep);
HAL_StatusTypeDef USB_ActivateDedicatedEndpoint(USB_OTG_GlobalTypeDef *USBx, USB_OTG_EPTypeDef *ep);
HAL_StatusTypeDef USB_DeactivateDedicatedEndpoint(USB_OTG_GlobalTypeDef *USBx, USB_OTG_EPTypeDef *ep);
HAL_StatusTypeDef USB_EPStartXfer(USB_OTG_GlobalTypeDef *USBx, USB_OTG_EPTypeDef *ep);
HAL_StatusTypeDef USB_EP0StartXfer(USB_OTG_GlobalTypeDef *USBx, USB_OTG_EPTypeDef *ep);
HAL_StatusTypeDef USB_WritePacket(USB_OTG_GlobalTypeDef *USBx, uint8_t *src, uint8_t ch_ep_num, uint16_t len);
void             *USB_ReadPacket(USB_OTG_GlobalTypeDef *USBx, uint8_t *dest, uint16_t len);
HAL_StatusTypeDef USB_EPSetStall(USB_OTG_GlobalTypeDef *USBx, USB_OTG_EPTypeDef *ep);
HAL_StatusTypeDef USB_EPClearStall(USB_OTG_GlobalTypeDef *USBx, USB_OTG_EPTypeDef *ep);
HAL_StatusTypeDef USB_SetDevAddress(USB_OTG_GlobalTypeDef *USBx, uint8_t address);
HAL_StatusTypeDef USB_DevConnect(USB_OTG_GlobalTypeDef *USBx);
HAL_StatusTypeDef USB_DevDisconnect(USB_OTG_GlobalTypeDef *USBx);
HAL_StatusTypeDef USB_StopDevice(USB_OTG_GlobalTypeDef *USBx);
HAL_StatusTypeDef USB_ActivateSetup(USB_OTG_GlobalTypeDef *USBx);
HAL_StatusTypeDef USB_EP0_OutStart(USB_OTG_GlobalTypeDef *USBx, uint8_t *psetup);
uint8_t           USB_GetDevSpeed(USB_OTG_GlobalTypeDef *USBx);
uint32_t          USB_GetMode(USB_OTG_GlobalTypeDef *USBx);
uint32_t          USB_ReadInterrupts(USB_OTG_GlobalTypeDef *USBx);
uint32_t          USB_ReadDevAllOutEpInterrupt(USB_OTG_GlobalTypeDef *USBx);
uint32_t          USB_ReadDevOutEPInterrupt(USB_OTG_GlobalTypeDef *USBx, uint8_t epnum);
uint32_t          USB_ReadDevAllInEpInterrupt(USB_OTG_GlobalTypeDef *USBx);
uint32_t          USB_ReadDevInEPInterrupt(USB_OTG_GlobalTypeDef *USBx, uint8_t epnum);
void              USB_ClearInterrupts(USB_OTG_GlobalTypeDef *USBx, uint32_t interrupt);

HAL_StatusTypeDef USB_HostInit(USB_OTG_GlobalTypeDef *USBx, USB_OTG_CfgTypeDef cfg);
HAL_StatusTypeDef USB_InitFSLSPClkSel(USB_OTG_GlobalTypeDef *USBx, uint8_t freq);
HAL_StatusTypeDef USB_ResetPort(USB_OTG_GlobalTypeDef *USBx);
HAL_StatusTypeDef USB_DriveVbus(USB_OTG_GlobalTypeDef *USBx, uint8_t state);
uint32_t          USB_GetHostSpeed(USB_OTG_GlobalTypeDef *USBx);
uint32_t          USB_GetCurrentFrame(USB_OTG_GlobalTypeDef *USBx);
HAL_StatusTypeDef USB_HC_Init(USB_OTG_GlobalTypeDef *USBx,
                              uint8_t ch_num,
                              uint8_t epnum,
                              uint8_t dev_address,
                              uint8_t speed,
                              uint8_t ep_type,
                              uint16_t mps);
HAL_StatusTypeDef USB_HC_StartXfer(USB_OTG_GlobalTypeDef *USBx, USB_OTG_HCTypeDef *hc);
uint32_t          USB_HC_ReadInterrupt(USB_OTG_GlobalTypeDef *USBx);
HAL_StatusTypeDef USB_HC_Halt(USB_OTG_GlobalTypeDef *USBx, uint8_t hc_num);
HAL_StatusTypeDef USB_DoPing(USB_OTG_GlobalTypeDef *USBx, uint8_t ch_num);
HAL_StatusTypeDef USB_StopHost(USB_OTG_GlobalTypeDef *USBx);
HAL_StatusTypeDef USB_ActivateRemoteWakeup(USB_OTG_GlobalTypeDef *USBx);
HAL_StatusTypeDef USB_DeActivateRemoteWakeup(USB_OTG_GlobalTypeDef *USBx);


#line 597 "..\\HALLIB\\STM32L4xx_HAL_Driver\\Inc\\stm32l4xx_ll_usb.h"


 



 



 



 










 
#line 46 "..\\HALLIB\\STM32L4xx_HAL_Driver\\Inc\\stm32l4xx_hal_pcd.h"





 



 

 


 



 
typedef enum
{
  HAL_PCD_STATE_RESET   = 0x00,
  HAL_PCD_STATE_READY   = 0x01,
  HAL_PCD_STATE_ERROR   = 0x02,
  HAL_PCD_STATE_BUSY    = 0x03,
  HAL_PCD_STATE_TIMEOUT = 0x04
} PCD_StateTypeDef;

 
typedef enum
{
  LPM_L0 = 0x00,  
  LPM_L1 = 0x01,  
  LPM_L2 = 0x02,  
  LPM_L3 = 0x03,  
} PCD_LPM_StateTypeDef;

typedef enum
{
  PCD_LPM_L0_ACTIVE = 0x00,  
  PCD_LPM_L1_ACTIVE = 0x01,  
} PCD_LPM_MsgTypeDef;

typedef enum
{
  PCD_BCD_ERROR                     = 0xFF,
  PCD_BCD_CONTACT_DETECTION         = 0xFE,
  PCD_BCD_STD_DOWNSTREAM_PORT       = 0xFD,
  PCD_BCD_CHARGING_DOWNSTREAM_PORT  = 0xFC,
  PCD_BCD_DEDICATED_CHARGING_PORT   = 0xFB,
  PCD_BCD_DISCOVERY_COMPLETED       = 0x00,

} PCD_BCD_MsgTypeDef;

#line 121 "..\\HALLIB\\STM32L4xx_HAL_Driver\\Inc\\stm32l4xx_hal_pcd.h"


typedef USB_OTG_GlobalTypeDef  PCD_TypeDef;
typedef USB_OTG_CfgTypeDef     PCD_InitTypeDef;
typedef USB_OTG_EPTypeDef      PCD_EPTypeDef;










 
typedef struct __PCD_HandleTypeDef
{
  PCD_TypeDef             *Instance;    
  PCD_InitTypeDef         Init;         
  volatile uint8_t            USB_Address;  
  PCD_EPTypeDef           IN_ep[16];    
  PCD_EPTypeDef           OUT_ep[16];   
  HAL_LockTypeDef         Lock;         
  volatile PCD_StateTypeDef   State;        
  volatile  uint32_t          ErrorCode;    
  uint32_t                Setup[12];    
  PCD_LPM_StateTypeDef    LPM_State;    
  uint32_t                BESL;


  uint32_t lpm_active;                 
 

  uint32_t battery_charging_active;    
 
  void                    *pData;       

#line 178 "..\\HALLIB\\STM32L4xx_HAL_Driver\\Inc\\stm32l4xx_hal_pcd.h"
} PCD_HandleTypeDef;



 

 
#line 1 "..\\HALLIB\\STM32L4xx_HAL_Driver\\Inc\\stm32l4xx_hal_pcd_ex.h"

































 

 







 
#line 46 "..\\HALLIB\\STM32L4xx_HAL_Driver\\Inc\\stm32l4xx_hal_pcd_ex.h"





 



 
 
 
 
 


 


 


HAL_StatusTypeDef HAL_PCDEx_SetTxFiFo(PCD_HandleTypeDef *hpcd, uint8_t fifo, uint16_t size);
HAL_StatusTypeDef HAL_PCDEx_SetRxFiFo(PCD_HandleTypeDef *hpcd, uint16_t size);


#line 78 "..\\HALLIB\\STM32L4xx_HAL_Driver\\Inc\\stm32l4xx_hal_pcd_ex.h"
HAL_StatusTypeDef HAL_PCDEx_ActivateLPM(PCD_HandleTypeDef *hpcd);
HAL_StatusTypeDef HAL_PCDEx_DeActivateLPM(PCD_HandleTypeDef *hpcd);
HAL_StatusTypeDef HAL_PCDEx_ActivateBCD(PCD_HandleTypeDef *hpcd);
HAL_StatusTypeDef HAL_PCDEx_DeActivateBCD(PCD_HandleTypeDef *hpcd);
void HAL_PCDEx_BCD_VBUSDetect(PCD_HandleTypeDef *hpcd);
void HAL_PCDEx_LPM_Callback(PCD_HandleTypeDef *hpcd, PCD_LPM_MsgTypeDef msg);
void HAL_PCDEx_BCD_Callback(PCD_HandleTypeDef *hpcd, PCD_BCD_MsgTypeDef msg);



 



 



 



 










 
#line 186 "..\\HALLIB\\STM32L4xx_HAL_Driver\\Inc\\stm32l4xx_hal_pcd.h"

 


 



 







 



 





 



 





 




 






 



 

 



 





















#line 272 "..\\HALLIB\\STM32L4xx_HAL_Driver\\Inc\\stm32l4xx_hal_pcd.h"

#line 291 "..\\HALLIB\\STM32L4xx_HAL_Driver\\Inc\\stm32l4xx_hal_pcd.h"



 

 


 

 


 
HAL_StatusTypeDef HAL_PCD_Init(PCD_HandleTypeDef *hpcd);
HAL_StatusTypeDef HAL_PCD_DeInit(PCD_HandleTypeDef *hpcd);
void HAL_PCD_MspInit(PCD_HandleTypeDef *hpcd);
void HAL_PCD_MspDeInit(PCD_HandleTypeDef *hpcd);

#line 371 "..\\HALLIB\\STM32L4xx_HAL_Driver\\Inc\\stm32l4xx_hal_pcd.h"


 

 
 


 
HAL_StatusTypeDef HAL_PCD_Start(PCD_HandleTypeDef *hpcd);
HAL_StatusTypeDef HAL_PCD_Stop(PCD_HandleTypeDef *hpcd);
void HAL_PCD_IRQHandler(PCD_HandleTypeDef *hpcd);

void HAL_PCD_SOFCallback(PCD_HandleTypeDef *hpcd);
void HAL_PCD_SetupStageCallback(PCD_HandleTypeDef *hpcd);
void HAL_PCD_ResetCallback(PCD_HandleTypeDef *hpcd);
void HAL_PCD_SuspendCallback(PCD_HandleTypeDef *hpcd);
void HAL_PCD_ResumeCallback(PCD_HandleTypeDef *hpcd);
void HAL_PCD_ConnectCallback(PCD_HandleTypeDef *hpcd);
void HAL_PCD_DisconnectCallback(PCD_HandleTypeDef *hpcd);

void HAL_PCD_DataOutStageCallback(PCD_HandleTypeDef *hpcd, uint8_t epnum);
void HAL_PCD_DataInStageCallback(PCD_HandleTypeDef *hpcd, uint8_t epnum);
void HAL_PCD_ISOOUTIncompleteCallback(PCD_HandleTypeDef *hpcd, uint8_t epnum);
void HAL_PCD_ISOINIncompleteCallback(PCD_HandleTypeDef *hpcd, uint8_t epnum);


 

 


 
HAL_StatusTypeDef HAL_PCD_DevConnect(PCD_HandleTypeDef *hpcd);
HAL_StatusTypeDef HAL_PCD_DevDisconnect(PCD_HandleTypeDef *hpcd);
HAL_StatusTypeDef HAL_PCD_SetAddress(PCD_HandleTypeDef *hpcd, uint8_t address);
HAL_StatusTypeDef HAL_PCD_EP_Open(PCD_HandleTypeDef *hpcd, uint8_t ep_addr, uint16_t ep_mps, uint8_t ep_type);
HAL_StatusTypeDef HAL_PCD_EP_Close(PCD_HandleTypeDef *hpcd, uint8_t ep_addr);
HAL_StatusTypeDef HAL_PCD_EP_Receive(PCD_HandleTypeDef *hpcd, uint8_t ep_addr, uint8_t *pBuf, uint32_t len);
HAL_StatusTypeDef HAL_PCD_EP_Transmit(PCD_HandleTypeDef *hpcd, uint8_t ep_addr, uint8_t *pBuf, uint32_t len);
uint16_t          HAL_PCD_EP_GetRxCount(PCD_HandleTypeDef *hpcd, uint8_t ep_addr);
HAL_StatusTypeDef HAL_PCD_EP_SetStall(PCD_HandleTypeDef *hpcd, uint8_t ep_addr);
HAL_StatusTypeDef HAL_PCD_EP_ClrStall(PCD_HandleTypeDef *hpcd, uint8_t ep_addr);
HAL_StatusTypeDef HAL_PCD_EP_Flush(PCD_HandleTypeDef *hpcd, uint8_t ep_addr);
HAL_StatusTypeDef HAL_PCD_ActivateRemoteWakeup(PCD_HandleTypeDef *hpcd);
HAL_StatusTypeDef HAL_PCD_DeActivateRemoteWakeup(PCD_HandleTypeDef *hpcd);


 

 


 
PCD_StateTypeDef HAL_PCD_GetState(PCD_HandleTypeDef *hpcd);


 



 

 


 


 
















#line 464 "..\\HALLIB\\STM32L4xx_HAL_Driver\\Inc\\stm32l4xx_hal_pcd.h"



 

#line 505 "..\\HALLIB\\STM32L4xx_HAL_Driver\\Inc\\stm32l4xx_hal_pcd.h"


 

 


 
#line 1046 "..\\HALLIB\\STM32L4xx_HAL_Driver\\Inc\\stm32l4xx_hal_pcd.h"



 



 



 









 
#line 402 "..\\USER\\stm32l4xx_hal_conf.h"


#line 1 "..\\HALLIB\\STM32L4xx_HAL_Driver\\Inc\\stm32l4xx_hal_hcd.h"

































 

 







 
#line 46 "..\\HALLIB\\STM32L4xx_HAL_Driver\\Inc\\stm32l4xx_hal_hcd.h"





 



 

 


 



 
typedef enum
{
  HAL_HCD_STATE_RESET    = 0x00,
  HAL_HCD_STATE_READY    = 0x01,
  HAL_HCD_STATE_ERROR    = 0x02,
  HAL_HCD_STATE_BUSY     = 0x03,
  HAL_HCD_STATE_TIMEOUT  = 0x04
} HCD_StateTypeDef;

typedef USB_OTG_GlobalTypeDef   HCD_TypeDef;
typedef USB_OTG_CfgTypeDef      HCD_InitTypeDef;
typedef USB_OTG_HCTypeDef       HCD_HCTypeDef;
typedef USB_OTG_URBStateTypeDef HCD_URBStateTypeDef;
typedef USB_OTG_HCStateTypeDef  HCD_HCStateTypeDef;


 



 
typedef struct __HCD_HandleTypeDef
{
  HCD_TypeDef               *Instance;   
  HCD_InitTypeDef           Init;        
  HCD_HCTypeDef             hc[16];      
  HAL_LockTypeDef           Lock;        
  volatile HCD_StateTypeDef     State;       
  volatile  uint32_t            ErrorCode;   
  void                      *pData;      
#line 107 "..\\HALLIB\\STM32L4xx_HAL_Driver\\Inc\\stm32l4xx_hal_hcd.h"
} HCD_HandleTypeDef;


 



 

 


 



 





 



 




 




 






 



 

 



 














 

 


 



 
HAL_StatusTypeDef      HAL_HCD_Init(HCD_HandleTypeDef *hhcd);
HAL_StatusTypeDef      HAL_HCD_DeInit(HCD_HandleTypeDef *hhcd);
HAL_StatusTypeDef      HAL_HCD_HC_Init(HCD_HandleTypeDef *hhcd,
                                       uint8_t ch_num,
                                       uint8_t epnum,
                                       uint8_t dev_address,
                                       uint8_t speed,
                                       uint8_t ep_type,
                                       uint16_t mps);

HAL_StatusTypeDef     HAL_HCD_HC_Halt(HCD_HandleTypeDef *hhcd, uint8_t ch_num);
void                  HAL_HCD_MspInit(HCD_HandleTypeDef *hhcd);
void                  HAL_HCD_MspDeInit(HCD_HandleTypeDef *hhcd);

#line 239 "..\\HALLIB\\STM32L4xx_HAL_Driver\\Inc\\stm32l4xx_hal_hcd.h"


 

 


 
HAL_StatusTypeDef       HAL_HCD_HC_SubmitRequest(HCD_HandleTypeDef *hhcd,
                                                 uint8_t pipe,
                                                 uint8_t direction,
                                                 uint8_t ep_type,
                                                 uint8_t token,
                                                 uint8_t *pbuff,
                                                 uint16_t length,
                                                 uint8_t do_ping);

 
void             HAL_HCD_IRQHandler(HCD_HandleTypeDef *hhcd);
void             HAL_HCD_SOF_Callback(HCD_HandleTypeDef *hhcd);
void             HAL_HCD_Connect_Callback(HCD_HandleTypeDef *hhcd);
void             HAL_HCD_Disconnect_Callback(HCD_HandleTypeDef *hhcd);
void             HAL_HCD_PortEnabled_Callback(HCD_HandleTypeDef *hhcd);
void             HAL_HCD_PortDisabled_Callback(HCD_HandleTypeDef *hhcd);
void             HAL_HCD_HC_NotifyURBChange_Callback(HCD_HandleTypeDef *hhcd,
                                                     uint8_t chnum,
                                                     HCD_URBStateTypeDef urb_state);


 

 


 
HAL_StatusTypeDef       HAL_HCD_ResetPort(HCD_HandleTypeDef *hhcd);
HAL_StatusTypeDef       HAL_HCD_Start(HCD_HandleTypeDef *hhcd);
HAL_StatusTypeDef       HAL_HCD_Stop(HCD_HandleTypeDef *hhcd);


 

 


 
HCD_StateTypeDef        HAL_HCD_GetState(HCD_HandleTypeDef *hhcd);
HCD_URBStateTypeDef     HAL_HCD_HC_GetURBState(HCD_HandleTypeDef *hhcd, uint8_t chnum);
uint32_t                HAL_HCD_HC_GetXferCount(HCD_HandleTypeDef *hhcd, uint8_t chnum);
HCD_HCStateTypeDef      HAL_HCD_HC_GetState(HCD_HandleTypeDef *hhcd, uint8_t chnum);
uint32_t                HAL_HCD_GetCurrentFrame(HCD_HandleTypeDef *hhcd);
uint32_t                HAL_HCD_GetCurrentSpeed(HCD_HandleTypeDef *hhcd);


 



 

 


 



 

 


 



 

 


 



 



 



 









 
#line 406 "..\\USER\\stm32l4xx_hal_conf.h"


#line 1 "..\\HALLIB\\STM32L4xx_HAL_Driver\\Inc\\stm32l4xx_hal_gfxmmu.h"

































 

 







 
#line 46 "..\\HALLIB\\STM32L4xx_HAL_Driver\\Inc\\stm32l4xx_hal_gfxmmu.h"

#line 344 "..\\HALLIB\\STM32L4xx_HAL_Driver\\Inc\\stm32l4xx_hal_gfxmmu.h"



 
#line 410 "..\\USER\\stm32l4xx_hal_conf.h"


 
#line 428 "..\\USER\\stm32l4xx_hal_conf.h"







 
#line 47 "..\\HALLIB\\STM32L4xx_HAL_Driver\\Inc\\stm32l4xx_hal.h"



 



 

 
 


 



 








        
        



#line 84 "..\\HALLIB\\STM32L4xx_HAL_Driver\\Inc\\stm32l4xx_hal.h"



 



 
#line 98 "..\\HALLIB\\STM32L4xx_HAL_Driver\\Inc\\stm32l4xx_hal.h"



 



 
#line 140 "..\\HALLIB\\STM32L4xx_HAL_Driver\\Inc\\stm32l4xx_hal.h"



 

#line 186 "..\\HALLIB\\STM32L4xx_HAL_Driver\\Inc\\stm32l4xx_hal.h"




 





 



 





 




 






 



 


 
#line 234 "..\\HALLIB\\STM32L4xx_HAL_Driver\\Inc\\stm32l4xx_hal.h"



 



 

 



 


 
















































































































 



 


 



 



 







 



        
        

#line 401 "..\\HALLIB\\STM32L4xx_HAL_Driver\\Inc\\stm32l4xx_hal.h"


 















 





 




#line 439 "..\\HALLIB\\STM32L4xx_HAL_Driver\\Inc\\stm32l4xx_hal.h"



 






 




 











 





 





 





 








 



 








 










 

 


 

#line 530 "..\\HALLIB\\STM32L4xx_HAL_Driver\\Inc\\stm32l4xx_hal.h"


















#line 565 "..\\HALLIB\\STM32L4xx_HAL_Driver\\Inc\\stm32l4xx_hal.h"


 

 



 
extern volatile uint32_t uwTick;


 

 



 



 

 
HAL_StatusTypeDef HAL_Init(void);
HAL_StatusTypeDef HAL_DeInit(void);
void HAL_MspInit(void);
void HAL_MspDeInit(void);
HAL_StatusTypeDef HAL_InitTick (uint32_t TickPriority);



 



 

 
void HAL_IncTick(void);
void HAL_Delay(uint32_t Delay);
uint32_t HAL_GetTick(void);
void HAL_SuspendTick(void);
void HAL_ResumeTick(void);
uint32_t HAL_GetHalVersion(void);
uint32_t HAL_GetREVID(void);
uint32_t HAL_GetDEVID(void);
uint32_t HAL_GetUIDw0(void);
uint32_t HAL_GetUIDw1(void);
uint32_t HAL_GetUIDw2(void);



 



 

 
void HAL_DBGMCU_EnableDBGSleepMode(void);
void HAL_DBGMCU_DisableDBGSleepMode(void);
void HAL_DBGMCU_EnableDBGStopMode(void);
void HAL_DBGMCU_DisableDBGStopMode(void);
void HAL_DBGMCU_EnableDBGStandbyMode(void);
void HAL_DBGMCU_DisableDBGStandbyMode(void);



 



 

 
void HAL_SYSCFG_SRAM2Erase(void);
void HAL_SYSCFG_EnableMemorySwappingBank(void);
void HAL_SYSCFG_DisableMemorySwappingBank(void);


void HAL_SYSCFG_VREFBUF_VoltageScalingConfig(uint32_t VoltageScaling);
void HAL_SYSCFG_VREFBUF_HighImpedanceConfig(uint32_t Mode);
void HAL_SYSCFG_VREFBUF_TrimmingConfig(uint32_t TrimmingValue);
HAL_StatusTypeDef HAL_SYSCFG_EnableVREFBUF(void);
void HAL_SYSCFG_DisableVREFBUF(void);


void HAL_SYSCFG_EnableIOAnalogSwitchBooster(void);
void HAL_SYSCFG_DisableIOAnalogSwitchBooster(void);



 



 



 



 







 
#line 42 "main.h"

 
 
 
 



 
#line 40 "stm32l4xx_it.c"
#line 1 "stm32l4xx_it.h"

































 

 







 
 
 
 
 

void NMI_Handler(void);
void HardFault_Handler(void);
void MemManage_Handler(void);
void BusFault_Handler(void);
void UsageFault_Handler(void);
void SVC_Handler(void);
void DebugMon_Handler(void);
void PendSV_Handler(void);
void SysTick_Handler(void);







 
#line 41 "stm32l4xx_it.c"



 



 

 
 
 
 

 
 

 
 
 





 
void NMI_Handler(void)
{
}





 
void HardFault_Handler(void)
{
   
  while (1)
  {
  }
}





 
void MemManage_Handler(void)
{
   
  while (1)
  {
  }
}





 
void BusFault_Handler(void)
{
   
  while (1)
  {
  }
}





 
void UsageFault_Handler(void)
{
   
  while (1)
  {
  }
}





 
void SVC_Handler(void)
{
}





 
void DebugMon_Handler(void)
{
}





 
void PendSV_Handler(void)
{
}





 
void SysTick_Handler(void)
{
  HAL_IncTick();
}

 
 
 
 
 
 





 


 




  



 

 
