#line 1 "..\\App\\src\\debugLEDs.c"








 


#line 1 "..\\App\\src\\debugLEDs.h"











 


#line 1 "F:\\KEIL5\\ARM\\ARMCC\\Bin\\..\\include\\stddef.h"
 






 

 
 
 





 





#line 34 "F:\\KEIL5\\ARM\\ARMCC\\Bin\\..\\include\\stddef.h"




  typedef signed int ptrdiff_t;



  



    typedef unsigned int size_t;    
#line 57 "F:\\KEIL5\\ARM\\ARMCC\\Bin\\..\\include\\stddef.h"



   



      typedef unsigned short wchar_t;  
#line 82 "F:\\KEIL5\\ARM\\ARMCC\\Bin\\..\\include\\stddef.h"



    




   




  typedef long double max_align_t;









#line 114 "F:\\KEIL5\\ARM\\ARMCC\\Bin\\..\\include\\stddef.h"



 

#line 16 "..\\App\\src\\debugLEDs.h"


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


#line 13 "..\\App\\src\\debugLEDs.c"



#line 17 "..\\App\\src\\debugLEDs.c"





 
enum debugLEDsSEMStateEnum
{
  STATE_SEM_NOT_INITIALIZED = 0U,
  STATE_SEM_INITIALIZED     = 1U,
  STATE_SEM_PREPARE         = 2U,
  STATE_SEM_OKAY            = 3U
};




 
struct debugLEDsSEMDATA
{
  enum debugLEDsSEMStateEnum                    State;
  SEM_EVENT_TYPE                                EventNo;
  SEM_STATE_TYPE                                CSV[1U];
  SEM_STATE_TYPE                                WSV[1U];
};




 
struct VSDATAdebugLEDs
{
  uint8_t        StateMachineIndex[0X002];
  uint8_t        RuleData[0X00c];
  uint8_t        RuleIndex[0X003];
  uint8_t        RuleTableIndex[0X004];
};




 
struct debugLEDsSEMDATA SEMdebugLEDs;




 
struct VSDATAdebugLEDs const debugLEDs = 
{
  {
    0x00U, 0x00U
  },
  {
    0x01U, 0x00U, 0x00U, 0x01U, 0x01U, 0x00U, 0x01U, 0x11U, 
    0x01U, 0x01U, 0x01U, 0x00U
  },
  {
    0x00U, 0x03U, 0x07U
  },
  {
    0x00U, 0x01U, 0x02U, 0x03U
  }
};


 



 





 
typedef _Bool (* debugLEDsVS_GUARDEXPR_TYPE) (void);


static void debugLEDsDeductChangeState (void);


static VSResult debugLEDsSEM_GetOutput (void);




 
typedef void (* debugLEDsVS_ACTIONEXPR_TYPE) (void);





 
static debugLEDsVS_ACTIONEXPR_TYPE const debugLEDsVSAction[1] = 
{
  &aLedsRun
};


void debugLEDsVSInitAll (void)
{
  debugLEDsSEM_Init();
}


void debugLEDsSEM_Init (void)
{
  {
    SEM_STATE_MACHINE_TYPE i;
    for (i = 0U; i < 1U; ++i)
    {
      SEMdebugLEDs .WSV[i] = 255U;
      SEMdebugLEDs .CSV[i] = 255U;
    }
  }
  SEMdebugLEDs .State = STATE_SEM_INITIALIZED;
}


static void debugLEDsDeductChangeState (void)
{
  SEM_STATE_MACHINE_TYPE i;
  for (i = 0U; i < 1U; ++i)
  {
    if (SEMdebugLEDs .WSV[i] != 255U)
    {
      SEMdebugLEDs .CSV[i] = SEMdebugLEDs .WSV[i];
      SEMdebugLEDs .WSV[i] = 255U;
    }
  }
}


VSResult debugLEDsVSDeduct (SEM_EVENT_TYPE EventNo)
{
  VSResult cc;
  if (SEMdebugLEDs .State == STATE_SEM_NOT_INITIALIZED)
  {
    return (SES_NOT_INITIALIZED);
  }
  if (3U <= EventNo)
  {
    return (SES_RANGE_ERR);
  }
  SEMdebugLEDs .EventNo = EventNo;
  SEMdebugLEDs .State = STATE_SEM_PREPARE;
  cc = debugLEDsSEM_GetOutput();
  if (cc == SES_OKAY)
  {
    debugLEDsDeductChangeState();
    SEMdebugLEDs.State = STATE_SEM_INITIALIZED;
  }
  return cc;
}


static VSResult debugLEDsSEM_GetOutput (void)
{

  uint_fast8_t iFirstR;
  uint_fast8_t iLastR;




  for(;;)
  {
    switch (SEMdebugLEDs .State)
    {
    case STATE_SEM_PREPARE:
      iFirstR = debugLEDs .RuleTableIndex[SEMdebugLEDs .EventNo];
      iLastR = debugLEDs .RuleTableIndex[SEMdebugLEDs .EventNo + 1U];
      while (iFirstR < iLastR)
      {

        uint_fast8_t i;
        uint_fast8_t nNo;
        uint_fast8_t nPos;
        uint_fast8_t nNxt;
        uint_fast8_t iRI;
        uint_fast8_t nAction;
#line 209 "..\\App\\src\\debugLEDs.c"

        iRI = debugLEDs .RuleIndex[iFirstR];
        ++iFirstR;
        i = debugLEDs .RuleData[iRI];
        ++iRI;
        nNxt = (unsigned char)(i & 0x0FU);
        nAction = (unsigned char)(i >> 4U);
        i = debugLEDs .RuleData[iRI];
        ++iRI;
        nPos = (unsigned char)(i & 0x0FU);

        for (nNo = 0U; nNo < nPos; ++nNo)
        {
          SEM_STATE_TYPE sa;
          sa = (SEM_STATE_TYPE)(debugLEDs .RuleData[iRI]);
          if (sa != SEMdebugLEDs .CSV[debugLEDs .StateMachineIndex[sa]])
          {
            goto NextRule;
          }
          else
          {
            ++iRI;
          }
        }


        for (nNo = 0U; nNo < nNxt; ++nNo)
        {
          SEM_STATE_TYPE sa;
          sa = (SEM_STATE_TYPE)(debugLEDs .RuleData[iRI]);
          ++iRI;
          i = debugLEDs .StateMachineIndex[sa];
          if (SEMdebugLEDs .WSV[i] == 255U)
          {
            SEMdebugLEDs .WSV[i] = sa;
          }
          else if (SEMdebugLEDs .WSV[i] != sa)
          {
            return (SES_CONTRADICTION);
          }
        }
        while (nAction != 0U)
        {
          SEM_ACTION_EXPRESSION_TYPE actionNo;
          actionNo = (SEM_ACTION_EXPRESSION_TYPE)(debugLEDs .RuleData[iRI]);
          ++iRI;
          nAction--;
          (*debugLEDsVSAction[actionNo])();
        }
NextRule:
        ;
      }
      SEMdebugLEDs.State = STATE_SEM_OKAY;
      return (SES_OKAY);

    case STATE_SEM_OKAY:
      return (SES_OKAY);

    default:
      return (SES_EMPTY);
    }
  }
}


VSResult debugLEDsVSElementName (IdentifierType IdentType, SEM_EXPLANATION_TYPE IdentNo, char const * * Text)
{
  VSResult ret = SES_OKAY;
  switch (IdentType)
  {
  default:
    ret = SES_TYPE_ERR;
    break;
  }
  return ret;
}


VSResult debugLEDsVSElementExpl (IdentifierType IdentType, SEM_EXPLANATION_TYPE IdentNo, char const * * Text)
{
  VSResult ret = SES_OKAY;
  switch (IdentType)
  {
  default:
    ret = SES_TYPE_ERR;
    break;
  }
  return ret;
}


VSResult debugLEDsSEM_State (SEM_STATE_MACHINE_TYPE StateMachineNo,
  SEM_STATE_TYPE *StateNo)
{
  if (1U <= StateMachineNo)
  {
    return (SES_RANGE_ERR);
  }
  *StateNo = SEMdebugLEDs .CSV[StateMachineNo];
  return (SES_FOUND);
}
