/*
 * Id:        debugLEDs.c
 *
 * Function:  Contains all API functions.
 *
 * This is an automatically generated file. It will be overwritten by the Coder.
 *
 * DO NOT EDIT THE FILE!
 */


#include "debugLEDs.h"
#if defined(__STDC_VERSION__) && (__STDC_VERSION__ >= 199901L)


#include <stdint.h>
#endif


/*
 * Used internally in the API
 */
enum debugLEDsSEMStateEnum
{
  STATE_SEM_NOT_INITIALIZED = 0U,
  STATE_SEM_INITIALIZED     = 1U,
  STATE_SEM_PREPARE         = 2U,
  STATE_SEM_OKAY            = 3U
};


/*
 * SEM Library Datatype Definition.
 */
struct debugLEDsSEMDATA
{
  enum debugLEDsSEMStateEnum                    State;
  SEM_EVENT_TYPE                                EventNo;
  SEM_STATE_TYPE                                CSV[VS_NOF_STATE_MACHINES];
  SEM_STATE_TYPE                                WSV[VS_NOF_STATE_MACHINES];
};


/*
 * VS System Datatype Definition.
 */
struct VSDATAdebugLEDs
{
  uint8_t        StateMachineIndex[0X002];
  uint8_t        RuleData[0X00c];
  uint8_t        RuleIndex[0X003];
  uint8_t        RuleTableIndex[0X004];
};




/*
 * VS System data definition and initialization. Rule data format number: 0
 */
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


/* Core model logic struct name */
#define VS debugLEDs


/* SEM data struct name */
#define SEM SEMdebugLEDs


/*
 * Guard expression type definition
 */
typedef _Bool (* debugLEDsVS_GUARDEXPR_TYPE) (void);


static void debugLEDsDeductChangeState (void);


static VSResult debugLEDsSEM_GetOutput (void);


/*
 * Action expression type definition
 */
typedef void (* debugLEDsVS_ACTIONEXPR_TYPE) (void);


/*
 * Action Expression Pointer Table Declaration.
 */
static debugLEDsVS_ACTIONEXPR_TYPE const debugLEDsVSAction[1];


/*
 * Action Expression Pointer Table Definition.
 */
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
      SEM.WSV[i] = STATE_UNDEFINED;
      SEM.CSV[i] = STATE_UNDEFINED;
    }
  }
  SEM.State = STATE_SEM_INITIALIZED;
}


static void debugLEDsDeductChangeState (void)
{
  SEM_STATE_MACHINE_TYPE i;
  for (i = 0U; i < 1U; ++i)
  {
    if (SEM.WSV[i] != STATE_UNDEFINED)
    {
      SEM.CSV[i] = SEM.WSV[i];
      SEM.WSV[i] = STATE_UNDEFINED;
    }
  }
}


VSResult debugLEDsVSDeduct (SEM_EVENT_TYPE EventNo)
{
  VSResult cc;
  if (SEM.State == STATE_SEM_NOT_INITIALIZED)
  {
    return (SES_NOT_INITIALIZED);
  }
  if (3U <= EventNo)
  {
    return (SES_RANGE_ERR);
  }
  SEM.EventNo = EventNo;
  SEM.State = STATE_SEM_PREPARE;
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
#if defined(__STDC_VERSION__) && (__STDC_VERSION__ >= 199901L)
  uint_fast8_t iFirstR;
  uint_fast8_t iLastR;
#else
  SEM_RULE_TABLE_INDEX_TYPE iFirstR;
  SEM_RULE_TABLE_INDEX_TYPE iLastR;
#endif
  for(;;)
  {
    switch (SEM.State)
    {
    case STATE_SEM_PREPARE:
      iFirstR = VS.RuleTableIndex[SEM.EventNo];
      iLastR = VS.RuleTableIndex[SEM.EventNo + 1U];
      while (iFirstR < iLastR)
      {
#if defined(__STDC_VERSION__) && (__STDC_VERSION__ >= 199901L)
        uint_fast8_t i;
        uint_fast8_t nNo;
        uint_fast8_t nPos;
        uint_fast8_t nNxt;
        uint_fast8_t iRI;
        uint_fast8_t nAction;
#else
        SEM_INTERNAL_TYPE i;
        uint8_t nNo;
        uint8_t nPos;
        uint8_t nNxt;
        SEM_RULE_INDEX_TYPE iRI;
        uint8_t nAction;
#endif

        iRI = VS.RuleIndex[iFirstR];
        ++iFirstR;
        i = VS.RuleData[iRI];
        ++iRI;
        nNxt = (unsigned char)(i & 0x0FU);
        nAction = (unsigned char)(i >> 4U);
        i = VS.RuleData[iRI];
        ++iRI;
        nPos = (unsigned char)(i & 0x0FU);

        for (nNo = 0U; nNo < nPos; ++nNo)
        {
          SEM_STATE_TYPE sa;
          sa = (SEM_STATE_TYPE)(VS.RuleData[iRI]);
          if (sa != SEM.CSV[VS.StateMachineIndex[sa]])
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
          sa = (SEM_STATE_TYPE)(VS.RuleData[iRI]);
          ++iRI;
          i = VS.StateMachineIndex[sa];
          if (SEM.WSV[i] == STATE_UNDEFINED)
          {
            SEM.WSV[i] = sa;
          }
          else if (SEM.WSV[i] != sa)
          {
            return (SES_CONTRADICTION);
          }
        }
        while (nAction != 0U)
        {
          SEM_ACTION_EXPRESSION_TYPE actionNo;
          actionNo = (SEM_ACTION_EXPRESSION_TYPE)(VS.RuleData[iRI]);
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
