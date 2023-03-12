#line 1 "..\\App\\src\\M_CO_LEDs_System.c"








 


#line 1 "..\\App\\src\\M_CO_LEDs_System.h"











 


#line 1 "F:\\KEIL5\\ARM\\ARMCC\\Bin\\..\\include\\limits.h"
 
 
 





 






     

     

     

     
#line 30 "F:\\KEIL5\\ARM\\ARMCC\\Bin\\..\\include\\limits.h"
       

       






#line 45 "F:\\KEIL5\\ARM\\ARMCC\\Bin\\..\\include\\limits.h"
     
     


     

     

     

     

     

     





     





     





     


       

       

       




 

#line 16 "..\\App\\src\\M_CO_LEDs_System.h"


#line 1 "F:\\KEIL5\\ARM\\ARMCC\\Bin\\..\\include\\stdbool.h"
 






 





#line 25 "F:\\KEIL5\\ARM\\ARMCC\\Bin\\..\\include\\stdbool.h"



#line 19 "..\\App\\src\\M_CO_LEDs_System.h"


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


#line 13 "..\\App\\src\\M_CO_LEDs_System.c"


char const *VSGetSignature (void)
{
  return "0c30841700540c0833abc5e61ab4b12a";
}
