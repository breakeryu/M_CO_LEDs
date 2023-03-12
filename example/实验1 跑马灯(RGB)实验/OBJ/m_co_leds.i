#line 1 "..\\App\\src\\M_CO_LEDs.c"







 

#line 1 "..\\App\\src\\M_CO_LEDs.h"






 




#line 1 "..\\App\\src\\M_CO_driver.h"






 





#line 1 "F:\\KEIL5\\ARM\\ARMCC\\Bin\\..\\include\\string.h"
 
 
 
 




 








 












#line 38 "F:\\KEIL5\\ARM\\ARMCC\\Bin\\..\\include\\string.h"


  



    typedef unsigned int size_t;    
#line 54 "F:\\KEIL5\\ARM\\ARMCC\\Bin\\..\\include\\string.h"




extern __declspec(__nothrow) void *memcpy(void * __restrict  ,
                    const void * __restrict  , size_t  ) __attribute__((__nonnull__(1,2)));
   




 
extern __declspec(__nothrow) void *memmove(void *  ,
                    const void *  , size_t  ) __attribute__((__nonnull__(1,2)));
   







 
extern __declspec(__nothrow) char *strcpy(char * __restrict  , const char * __restrict  ) __attribute__((__nonnull__(1,2)));
   




 
extern __declspec(__nothrow) char *strncpy(char * __restrict  , const char * __restrict  , size_t  ) __attribute__((__nonnull__(1,2)));
   





 

extern __declspec(__nothrow) char *strcat(char * __restrict  , const char * __restrict  ) __attribute__((__nonnull__(1,2)));
   




 
extern __declspec(__nothrow) char *strncat(char * __restrict  , const char * __restrict  , size_t  ) __attribute__((__nonnull__(1,2)));
   






 






 

extern __declspec(__nothrow) int memcmp(const void *  , const void *  , size_t  ) __attribute__((__nonnull__(1,2)));
   





 
extern __declspec(__nothrow) int strcmp(const char *  , const char *  ) __attribute__((__nonnull__(1,2)));
   




 
extern __declspec(__nothrow) int strncmp(const char *  , const char *  , size_t  ) __attribute__((__nonnull__(1,2)));
   






 
extern __declspec(__nothrow) int strcasecmp(const char *  , const char *  ) __attribute__((__nonnull__(1,2)));
   





 
extern __declspec(__nothrow) int strncasecmp(const char *  , const char *  , size_t  ) __attribute__((__nonnull__(1,2)));
   






 
extern __declspec(__nothrow) int strcoll(const char *  , const char *  ) __attribute__((__nonnull__(1,2)));
   







 

extern __declspec(__nothrow) size_t strxfrm(char * __restrict  , const char * __restrict  , size_t  ) __attribute__((__nonnull__(2)));
   













 


#line 193 "F:\\KEIL5\\ARM\\ARMCC\\Bin\\..\\include\\string.h"
extern __declspec(__nothrow) void *memchr(const void *  , int  , size_t  ) __attribute__((__nonnull__(1)));

   





 

#line 209 "F:\\KEIL5\\ARM\\ARMCC\\Bin\\..\\include\\string.h"
extern __declspec(__nothrow) char *strchr(const char *  , int  ) __attribute__((__nonnull__(1)));

   




 

extern __declspec(__nothrow) size_t strcspn(const char *  , const char *  ) __attribute__((__nonnull__(1,2)));
   




 

#line 232 "F:\\KEIL5\\ARM\\ARMCC\\Bin\\..\\include\\string.h"
extern __declspec(__nothrow) char *strpbrk(const char *  , const char *  ) __attribute__((__nonnull__(1,2)));

   




 

#line 247 "F:\\KEIL5\\ARM\\ARMCC\\Bin\\..\\include\\string.h"
extern __declspec(__nothrow) char *strrchr(const char *  , int  ) __attribute__((__nonnull__(1)));

   





 

extern __declspec(__nothrow) size_t strspn(const char *  , const char *  ) __attribute__((__nonnull__(1,2)));
   



 

#line 270 "F:\\KEIL5\\ARM\\ARMCC\\Bin\\..\\include\\string.h"
extern __declspec(__nothrow) char *strstr(const char *  , const char *  ) __attribute__((__nonnull__(1,2)));

   





 

extern __declspec(__nothrow) char *strtok(char * __restrict  , const char * __restrict  ) __attribute__((__nonnull__(2)));
extern __declspec(__nothrow) char *_strtok_r(char *  , const char *  , char **  ) __attribute__((__nonnull__(2,3)));

extern __declspec(__nothrow) char *strtok_r(char *  , const char *  , char **  ) __attribute__((__nonnull__(2,3)));

   

































 

extern __declspec(__nothrow) void *memset(void *  , int  , size_t  ) __attribute__((__nonnull__(1)));
   



 
extern __declspec(__nothrow) char *strerror(int  );
   





 
extern __declspec(__nothrow) size_t strlen(const char *  ) __attribute__((__nonnull__(1)));
   



 

extern __declspec(__nothrow) size_t strlcpy(char *  , const char *  , size_t  ) __attribute__((__nonnull__(1,2)));
   
















 

extern __declspec(__nothrow) size_t strlcat(char *  , const char *  , size_t  ) __attribute__((__nonnull__(1,2)));
   






















 

extern __declspec(__nothrow) void _membitcpybl(void *  , const void *  , int  , int  , size_t  ) __attribute__((__nonnull__(1,2)));
extern __declspec(__nothrow) void _membitcpybb(void *  , const void *  , int  , int  , size_t  ) __attribute__((__nonnull__(1,2)));
extern __declspec(__nothrow) void _membitcpyhl(void *  , const void *  , int  , int  , size_t  ) __attribute__((__nonnull__(1,2)));
extern __declspec(__nothrow) void _membitcpyhb(void *  , const void *  , int  , int  , size_t  ) __attribute__((__nonnull__(1,2)));
extern __declspec(__nothrow) void _membitcpywl(void *  , const void *  , int  , int  , size_t  ) __attribute__((__nonnull__(1,2)));
extern __declspec(__nothrow) void _membitcpywb(void *  , const void *  , int  , int  , size_t  ) __attribute__((__nonnull__(1,2)));
extern __declspec(__nothrow) void _membitmovebl(void *  , const void *  , int  , int  , size_t  ) __attribute__((__nonnull__(1,2)));
extern __declspec(__nothrow) void _membitmovebb(void *  , const void *  , int  , int  , size_t  ) __attribute__((__nonnull__(1,2)));
extern __declspec(__nothrow) void _membitmovehl(void *  , const void *  , int  , int  , size_t  ) __attribute__((__nonnull__(1,2)));
extern __declspec(__nothrow) void _membitmovehb(void *  , const void *  , int  , int  , size_t  ) __attribute__((__nonnull__(1,2)));
extern __declspec(__nothrow) void _membitmovewl(void *  , const void *  , int  , int  , size_t  ) __attribute__((__nonnull__(1,2)));
extern __declspec(__nothrow) void _membitmovewb(void *  , const void *  , int  , int  , size_t  ) __attribute__((__nonnull__(1,2)));
    














































 







#line 502 "F:\\KEIL5\\ARM\\ARMCC\\Bin\\..\\include\\string.h"



 

#line 14 "..\\App\\src\\M_CO_driver.h"


 
 
 
typedef unsigned char           bool_t;
typedef float                   float32_t;
typedef double                  float64_t;




 
typedef enum {
     
    CO_SMT_UNKNOWN = -1,
     
    CO_SMT_INITIALIZING = 0,
     
    CO_SMT_PRE_OPERATIONAL = 127,
     
    CO_SMT_OPERATIONAL = 5,
     
    CO_SMT_STOPPED = 4
} M_CO_SMT_internalState_t;









 
typedef enum {
    CO_CAN_ERRTX_WARNING = 0x0001,   
    CO_CAN_ERRTX_PASSIVE = 0x0002,   
    CO_CAN_ERRTX_BUS_OFF = 0x0004,   
    CO_CAN_ERRTX_OVERFLOW = 0x0008,  

    CO_CAN_ERRTX_PDO_LATE = 0x0080,  

    CO_CAN_ERRRX_WARNING = 0x0100,   
    CO_CAN_ERRRX_PASSIVE = 0x0200,   
    CO_CAN_ERRRX_OVERFLOW = 0x0800,  

    CO_CAN_ERR_WARN_PASSIVE = 0x0303 
} M_CO_CAN_ERR_status_t;




 
typedef enum {
    CO_ERROR_NO = 0,                 
    CO_ERROR_ILLEGAL_ARGUMENT = -1,  
    CO_ERROR_OUT_OF_MEMORY = -2,     
    CO_ERROR_TIMEOUT = -3,           
    CO_ERROR_ILLEGAL_BAUDRATE = -4, 
 
    CO_ERROR_RX_OVERFLOW = -5,      
 
    CO_ERROR_RX_PDO_OVERFLOW = -6,   
    CO_ERROR_RX_MSG_LENGTH = -7,     
    CO_ERROR_RX_PDO_LENGTH = -8,     
    CO_ERROR_TX_OVERFLOW = -9,      
 
    CO_ERROR_TX_PDO_WINDOW = -10,    
    CO_ERROR_TX_UNCONFIGURED = -11, 
 
    CO_ERROR_OD_PARAMETERS = -12,    
    CO_ERROR_DATA_CORRUPT = -13,     
    CO_ERROR_CRC = -14,              
    CO_ERROR_TX_BUSY = -15,         
 
    CO_ERROR_WRONG_NMT_STATE = -16, 
 
    CO_ERROR_SYSCALL = -17,          
    CO_ERROR_INVALID_STATE = -18,    
    CO_ERROR_NODE_ID_UNCONFIGURED_LSS = -19 

 
} M_CO_ReturnError_t;






#line 13 "..\\App\\src\\M_CO_LEDs.h"
#line 1 "..\\App\\src\\M_CO_driver_target.h"






 




#line 1 "F:\\KEIL5\\ARM\\ARMCC\\Bin\\..\\include\\stddef.h"
 






 

 
 
 





 





#line 34 "F:\\KEIL5\\ARM\\ARMCC\\Bin\\..\\include\\stddef.h"




  typedef signed int ptrdiff_t;



  



    typedef unsigned int size_t;    
#line 57 "F:\\KEIL5\\ARM\\ARMCC\\Bin\\..\\include\\stddef.h"



   



      typedef unsigned short wchar_t;  
#line 82 "F:\\KEIL5\\ARM\\ARMCC\\Bin\\..\\include\\stddef.h"



    




   




  typedef long double max_align_t;









#line 114 "F:\\KEIL5\\ARM\\ARMCC\\Bin\\..\\include\\stddef.h"



 

#line 13 "..\\App\\src\\M_CO_driver_target.h"
#line 1 "F:\\KEIL5\\ARM\\ARMCC\\Bin\\..\\include\\stdbool.h"
 






 





#line 25 "F:\\KEIL5\\ARM\\ARMCC\\Bin\\..\\include\\stdbool.h"



#line 14 "..\\App\\src\\M_CO_driver_target.h"
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






 
#line 15 "..\\App\\src\\M_CO_driver_target.h"
#line 1 "..\\App\\src\\M_CO_LEDs_System.h"











 


#line 1 "F:\\KEIL5\\ARM\\ARMCC\\Bin\\..\\include\\limits.h"
 
 
 





 






     

     

     

     
#line 30 "F:\\KEIL5\\ARM\\ARMCC\\Bin\\..\\include\\limits.h"
       

       






#line 45 "F:\\KEIL5\\ARM\\ARMCC\\Bin\\..\\include\\limits.h"
     
     


     

     

     

     

     

     





     





     





     


       

       

       




 

#line 16 "..\\App\\src\\M_CO_LEDs_System.h"


#line 19 "..\\App\\src\\M_CO_LEDs_System.h"


#line 22 "..\\App\\src\\M_CO_LEDs_System.h"


typedef enum
{

  





 
  SES_OKAY = 0U,


  








 
  SES_FOUND = 1U,


  
















 
  SES_ACTIVE = 2U,


  






 
  SES_CONTRADICTION = 3U,


  

















 
  SES_RANGE_ERR = 4U,


  





 
  SES_TEXT_TOO_LONG = 5U,


  








 
  SES_TYPE_ERR = 6U,


  






 
  SES_EMPTY = 7U,


  





 
  SES_BUFFER_OVERFLOW = 8U,


  






 
  SES_SIGNAL_QUEUE_FULL = 9U,


  





 
  SES_NOT_INITIALIZED = 10U
} VSResult;


 
typedef enum { EVENT_TYPE = 0U, STATE_TYPE = 1U, ACTION_TYPE = 2U } IdentifierType;


char const *VSGetSignature (void);


#line 16 "..\\App\\src\\M_CO_driver_target.h"
#line 1 "..\\App\\src\\debugLEDs.h"











 


#line 16 "..\\App\\src\\debugLEDs.h"


#line 19 "..\\App\\src\\debugLEDs.h"




 
typedef uint8_t   SEM_EVENT_TYPE;
typedef uint8_t   SEM_ACTION_EXPRESSION_TYPE;
typedef uint8_t   SEM_GUARD_EXPRESSION_TYPE;
typedef uint8_t   SEM_EXPLANATION_TYPE;
typedef uint8_t   SEM_STATE_TYPE;
typedef uint8_t   SEM_STATE_MACHINE_TYPE;
typedef uint8_t   SEM_INSTANCE_TYPE;
typedef uint8_t   SEM_RULE_INDEX_TYPE;
typedef uint8_t   SEM_INTERNAL_TYPE;
typedef uint8_t   SEM_SIGNAL_QUEUE_TYPE;
typedef uint8_t   SEM_ACTION_FUNCTION_TYPE;
typedef uint8_t   SEM_EVENT_GROUP_TYPE;
typedef uint8_t   SEM_EGTI_TYPE;
typedef uint8_t   SEM_RULE_TABLE_INDEX_TYPE;




 









 





 





 





 





 




















 
void debugLEDsVSInitAll (void);













 
void debugLEDsSEM_Init (void);














































 
VSResult debugLEDsVSDeduct (SEM_EVENT_TYPE EventNo);

































 
VSResult debugLEDsVSElementName (IdentifierType IdentType, SEM_EXPLANATION_TYPE IdentNo, char const * * Text);

































 
VSResult debugLEDsVSElementExpl (IdentifierType IdentType, SEM_EXPLANATION_TYPE IdentNo, char const * * Text);























 
VSResult debugLEDsSEM_State (SEM_STATE_MACHINE_TYPE StateMachineNo, SEM_STATE_TYPE *StateNo);




 
void aLedsRun (void);


#line 17 "..\\App\\src\\M_CO_driver_target.h"
#line 1 "..\\App\\src\\M_CO_LEDs.h"






 

#line 18 "..\\App\\src\\M_CO_driver_target.h"
#line 19 "..\\App\\src\\M_CO_driver_target.h"







 
void debug_LEDs_System_Init(void);






 
void debug_LEDs_System_Run(void);




 
extern void aLedsRun (void) __attribute__((weak));


extern uint32_t M_CO_get_Time_Difference_us(void) __attribute__((weak));
extern M_CO_SMT_internalState_t M_CO_get_SMT_internalState_t(void) __attribute__((weak));
extern bool_t M_CO_get_SYSconfig(void) __attribute__((weak));
extern bool_t M_CO_get_ErrPriority1(void) __attribute__((weak));
extern bool_t M_CO_get_ErrPriority5(void) __attribute__((weak));
extern bool_t M_CO_get_ErrPriority4(void) __attribute__((weak));
extern bool_t M_CO_get_ErrPriority3(void) __attribute__((weak));
extern bool_t M_CO_get_ErrPriority6(void) __attribute__((weak));
extern bool_t M_CO_get_ErrOther(void) __attribute__((weak));
extern bool_t M_CO_get_SYSotherState(void) __attribute__((weak));





#line 14 "..\\App\\src\\M_CO_LEDs.h"




































 

 
typedef enum {
    CO_LED_flicker = 0x01,   
    CO_LED_blink   = 0x02,   
    CO_LED_flash_1 = 0x04,   
    CO_LED_flash_2 = 0x08,   
    CO_LED_flash_3 = 0x10,   
    CO_LED_flash_4 = 0x20,   
    CO_LED_CANopen = 0x80    
} M_CO_LED_BITFIELD_t;

 

 





 
typedef struct{
    uint32_t            LEDtmr50ms;      
    uint8_t             LEDtmr200ms;     
    uint8_t             LEDtmrflash_1;   
    uint8_t             LEDtmrflash_2;   
    uint8_t             LEDtmrflash_3;   
    uint8_t             LEDtmrflash_4;   
    uint8_t             LEDred;          
    uint8_t             LEDgreen;        
} M_CO_LEDs_t;









 
M_CO_ReturnError_t M_CO_LEDs_init(M_CO_LEDs_t *LEDs);






















 

void M_CO_LEDs_process(M_CO_LEDs_t *LEDs,
                     uint32_t timeDifference_us,
                     M_CO_SMT_internalState_t SMTstate,
                     bool_t SYSconfig,
                     bool_t ErrPriority1,
                     bool_t ErrPriority6,
                     bool_t ErrPriority3,
                     bool_t ErrPriority4,
                     bool_t ErrPriority5,
                     bool_t ErrOther,
                     bool_t SYSotherState
                     );

                     

#line 11 "..\\App\\src\\M_CO_LEDs.c"




 
M_CO_ReturnError_t M_CO_LEDs_init(M_CO_LEDs_t *LEDs) 
{
    M_CO_ReturnError_t ret = CO_ERROR_NO;

     
    if (LEDs == 0) {
        return CO_ERROR_ILLEGAL_ARGUMENT;
    }

     
    memset(LEDs, 0, sizeof(M_CO_LEDs_t));

    return ret;
}



 
void M_CO_LEDs_process(M_CO_LEDs_t *LEDs,
                     uint32_t timeDifference_us,
                     M_CO_SMT_internalState_t SMTstate,
                     bool_t SYSconfig,
                     bool_t ErrPriority1,
                     bool_t ErrPriority6,
                     bool_t ErrPriority3,
                     bool_t ErrPriority4,
                     bool_t ErrPriority5,
                     bool_t ErrOther,
                     bool_t SYSotherState
                     )
{

    uint8_t rd = 0;
    uint8_t gr = 0;
    bool_t tick = 0;

    LEDs->LEDtmr50ms += timeDifference_us;
    while (LEDs->LEDtmr50ms >= 50000) {
        bool_t rdFlickerNext = (LEDs->LEDred & CO_LED_flicker) == 0;

        tick = 1;
        LEDs->LEDtmr50ms -= 50000;

        if (++LEDs->LEDtmr200ms > 3) {
             
            LEDs->LEDtmr200ms = 0;
            rd = gr = 0;

            if ((LEDs->LEDred & CO_LED_blink) == 0) rd |= CO_LED_blink;
            else                                    gr |= CO_LED_blink;

            switch (++LEDs->LEDtmrflash_1) {
                case 1: rd |= CO_LED_flash_1; break;
                case 2: gr |= CO_LED_flash_1; break;
                case 6: LEDs->LEDtmrflash_1 = 0; break;
                default: break;
            }
            switch (++LEDs->LEDtmrflash_2) {
                case 1: case 3: rd |= CO_LED_flash_2; break;
                case 2: case 4: gr |= CO_LED_flash_2; break;
                case 8: LEDs->LEDtmrflash_2 = 0; break;
                default: break;
            }
            switch (++LEDs->LEDtmrflash_3) {
                case 1: case 3: case 5: rd |= CO_LED_flash_3; break;
                case 2: case 4: case 6: gr |= CO_LED_flash_3; break;
                case 10: LEDs->LEDtmrflash_3 = 0; break;
                default: break;
            }
            switch (++LEDs->LEDtmrflash_4) {
                case 1: case 3: case 5: case 7: rd |= CO_LED_flash_4; break;
                case 2: case 4: case 6: case 8: gr |= CO_LED_flash_4; break;
                case 12: LEDs->LEDtmrflash_4 = 0; break;
                default: break;
            }
        }
        else {
             
            rd = LEDs->LEDred & (0xFF ^ (CO_LED_flicker | CO_LED_CANopen));
            gr = LEDs->LEDgreen & (0xFF ^ (CO_LED_flicker | CO_LED_CANopen));
        }

         
        if (rdFlickerNext) rd |= CO_LED_flicker;
        else               gr |= CO_LED_flicker;

    }  

    if (tick) {
        uint8_t rd_co, gr_co;

         
        if      (ErrPriority1)                      rd_co = 1;
        else if (SMTstate == CO_SMT_INITIALIZING)   rd_co = rd & CO_LED_flicker;
        else if (ErrPriority3)                      rd_co = rd & CO_LED_flash_4;
        else if (ErrPriority4)                      rd_co = rd & CO_LED_flash_3;
        else if (ErrPriority5)                      rd_co = rd & CO_LED_flash_2;
        else if (ErrPriority6)                      rd_co = rd & CO_LED_flash_1;
        else if (ErrOther)                          rd_co = rd & CO_LED_blink;
        else                                        rd_co = 0;

         
        if      (SYSconfig)                         gr_co = gr & CO_LED_flicker;
        else if (SYSotherState)                     gr_co = gr & CO_LED_flash_3;
        else if (SMTstate == CO_SMT_STOPPED)        gr_co = gr & CO_LED_flash_1;
        else if (SMTstate == CO_SMT_PRE_OPERATIONAL)gr_co = gr & CO_LED_blink;
        else if (SMTstate == CO_SMT_OPERATIONAL)    gr_co = 1;
        else                                        gr_co = 0;

        if (rd_co != 0) rd |= CO_LED_CANopen;
        if (gr_co != 0) gr |= CO_LED_CANopen;
        LEDs->LEDred = rd;
        LEDs->LEDgreen = gr;
    }  

}
