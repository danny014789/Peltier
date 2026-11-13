/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    p2p_server_app.c
  * @author  MCD Application Team
  * @brief   Peer to peer Server Application
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2019-2021 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "app_common.h"
#include "dbg_trace.h"
#include "ble.h"
#include "p2p_server_app.h"
#include "stm32_seq.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "qb55_in_out.h"
#include "qb55_thermo.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
 typedef struct{
    uint8_t             Device_Led_Selection;
    uint8_t             Led1;
 }P2P_LedCharValue_t;

 typedef struct{
    uint8_t             Device_Button_Selection;
    uint8_t             ButtonStatus;
 }P2P_ButtonCharValue_t;

typedef struct
{
  uint8_t               Notification_Status; /* used to check if P2P Server is enabled to Notify */
  P2P_LedCharValue_t    LedControl;
  P2P_ButtonCharValue_t ButtonControl;
  uint16_t              ConnectionHandle;
} P2P_Server_App_Context_t;
/* USER CODE END PTD */

/* Private defines ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macros -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN PV */
/**
 * START of Section BLE_APP_CONTEXT
 */

static P2P_Server_App_Context_t P2P_Server_App_Context;

/**
 * END of Section BLE_APP_CONTEXT
 */
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN PFP */
static void P2PS_Send_Notification(void);
static void P2PS_APP_LED_BUTTON_context_Init(void);
static void QB55_Uart_Retry_TimerCb(void);
static void QB55_Uart_Drain_Task(void);
static void QB55_Uart_Command(const uint8_t *line, uint8_t len);
static void QB55_Uart_Send(const uint8_t *data, uint16_t len);
static void QB55_Uart_Rx(const uint8_t *data, uint8_t len);

/* Bytes waiting to go out as notifications. Only touched from sequencer tasks (never from an
   ISR), so no locking is needed. Sizes are powers of two: indices are free-running and masked. */
#define QB55_TXQ_SIZE     512U
#define QB55_RX_LINE_MAX  64U
static uint8_t  QB55_txq[QB55_TXQ_SIZE];
static uint16_t QB55_txq_head;   /* next byte to write */
static uint16_t QB55_txq_tail;   /* next byte to send  */
static uint8_t  QB55_rxline[QB55_RX_LINE_MAX];
static uint8_t  QB55_rxlen;

/* If the stack refuses a notification (its buffers are full) the bytes stay queued and this
   one-shot timer retries shortly. It gives up after QB55_RETRY_MAX tries so a dead link cannot
   spin forever. Nothing is sent unless the client wrote something first. */
#define QB55_RETRY_MS     20U
#define QB55_RETRY_TICKS  ((QB55_RETRY_MS * 1000U) / CFG_TS_TICK_VAL)
#define QB55_RETRY_MAX    50U
static uint8_t QB55_RetryTimerId;
static uint8_t QB55_retries;
/* non-static so they appear in the .map for SWD inspection */
volatile uint32_t dbgNotifyCount;
volatile uint32_t dbgNotifyOkCount;
volatile uint8_t  dbgNotifyStatus = 0xFF;
volatile uint32_t dbgRxBytes;    /* bytes written to the RX characteristic */
volatile uint32_t dbgRxLines;    /* complete lines assembled from them */
volatile uint32_t dbgTxDropped;  /* bytes not queued: no subscriber, or queue full */
/* USER CODE END PFP */

/* Functions Definition ------------------------------------------------------*/
void P2PS_STM_App_Notification(P2PS_STM_App_Notification_evt_t *pNotification)
{
/* USER CODE BEGIN P2PS_STM_App_Notification_1 */

/* USER CODE END P2PS_STM_App_Notification_1 */
  switch(pNotification->P2P_Evt_Opcode)
  {
/* USER CODE BEGIN P2PS_STM_App_Notification_P2P_Evt_Opcode */
#if(BLE_CFG_OTA_REBOOT_CHAR != 0)
    case P2PS_STM_BOOT_REQUEST_EVT:
      APP_DBG_MSG("-- P2P APPLICATION SERVER : BOOT REQUESTED\n");
      APP_DBG_MSG(" \n\r");

      *(uint32_t*)SRAM1_BASE = *(uint32_t*)pNotification->DataTransfered.pPayload;
      NVIC_SystemReset();
      break;
#endif
/* USER CODE END P2PS_STM_App_Notification_P2P_Evt_Opcode */

    case P2PS_STM__NOTIFY_ENABLED_EVT:
/* USER CODE BEGIN P2PS_STM__NOTIFY_ENABLED_EVT */
      P2P_Server_App_Context.Notification_Status = 1;
      QB55_txq_head = QB55_txq_tail = 0;   /* new subscriber: drop anything left from before */
      QB55_rxlen = 0;
      APP_DBG_MSG("-- P2P APPLICATION SERVER : NOTIFICATION ENABLED\n");
      APP_DBG_MSG(" \n\r");
/* USER CODE END P2PS_STM__NOTIFY_ENABLED_EVT */
      break;

    case P2PS_STM_NOTIFY_DISABLED_EVT:
/* USER CODE BEGIN P2PS_STM_NOTIFY_DISABLED_EVT */
      P2P_Server_App_Context.Notification_Status = 0;
      APP_DBG_MSG("-- P2P APPLICATION SERVER : NOTIFICATION DISABLED\n");
      APP_DBG_MSG(" \n\r");
/* USER CODE END P2PS_STM_NOTIFY_DISABLED_EVT */
      break;

    case P2PS_STM_WRITE_EVT:
/* USER CODE BEGIN P2PS_STM_WRITE_EVT */
      /* Bytes written to the NUS RX characteristic are UART data. The payload pointer is only
         valid inside this callback, so QB55_Uart_Rx copies it out. */
      QB55_Uart_Rx(pNotification->DataTransfered.pPayload, pNotification->DataTransfered.Length);
#if 0 /* ST's original P2P decoding: byte0 = device, byte1 = LED on/off. Not a UART. */
      if(pNotification->DataTransfered.pPayload[0] == 0x00){ /* ALL Deviceselected - may be necessary as LB Routeur informs all connection */
        if(pNotification->DataTransfered.pPayload[1] == 0x01)
        {
          BSP_LED_On(LED_BLUE);
          APP_DBG_MSG("-- P2P APPLICATION SERVER  : LED1 ON\n"); 
          APP_DBG_MSG(" \n\r");
          P2P_Server_App_Context.LedControl.Led1=0x01; /* LED1 ON */
        }
        if(pNotification->DataTransfered.pPayload[1] == 0x00)
        {
          BSP_LED_Off(LED_BLUE);
          APP_DBG_MSG("-- P2P APPLICATION SERVER  : LED1 OFF\n"); 
          APP_DBG_MSG(" \n\r");
          P2P_Server_App_Context.LedControl.Led1=0x00; /* LED1 OFF */
        }
      }
#if(P2P_SERVER1 != 0)  
      if(pNotification->DataTransfered.pPayload[0] == 0x01){ /* end device 1 selected - may be necessary as LB Routeur informs all connection */
        if(pNotification->DataTransfered.pPayload[1] == 0x01)
        {
          BSP_LED_On(LED_BLUE);
          APP_DBG_MSG("-- P2P APPLICATION SERVER 1 : LED1 ON\n"); 
          APP_DBG_MSG(" \n\r");
          P2P_Server_App_Context.LedControl.Led1=0x01; /* LED1 ON */
        }
        if(pNotification->DataTransfered.pPayload[1] == 0x00)
        {
          BSP_LED_Off(LED_BLUE);
          APP_DBG_MSG("-- P2P APPLICATION SERVER 1 : LED1 OFF\n"); 
          APP_DBG_MSG(" \n\r");
          P2P_Server_App_Context.LedControl.Led1=0x00; /* LED1 OFF */
        }
      }
#endif
#if(P2P_SERVER2 != 0)
      if(pNotification->DataTransfered.pPayload[0] == 0x02){ /* end device 2 selected */ 
        if(pNotification->DataTransfered.pPayload[1] == 0x01)
        {
          BSP_LED_On(LED_BLUE);
           APP_DBG_MSG("-- P2P APPLICATION SERVER 2 : LED1 ON\n"); 
          APP_DBG_MSG(" \n\r");
          P2P_Server_App_Context.LedControl.Led1=0x01; /* LED1 ON */
        }
        if(pNotification->DataTransfered.pPayload[1] == 0x00)
        {
          BSP_LED_Off(LED_BLUE);
          APP_DBG_MSG("-- P2P APPLICATION SERVER 2 : LED1 OFF\n"); 
          APP_DBG_MSG(" \n\r");
          P2P_Server_App_Context.LedControl.Led1=0x00; /* LED1 OFF */
        }   
      }
#endif      
#if(P2P_SERVER3 != 0)  
      if(pNotification->DataTransfered.pPayload[0] == 0x03){ /* end device 3 selected - may be necessary as LB Routeur informs all connection */
        if(pNotification->DataTransfered.pPayload[1] == 0x01)
        {
          BSP_LED_On(LED_BLUE);
          APP_DBG_MSG("-- P2P APPLICATION SERVER 3 : LED1 ON\n"); 
          APP_DBG_MSG(" \n\r");
          P2P_Server_App_Context.LedControl.Led1=0x01; /* LED1 ON */
        }
        if(pNotification->DataTransfered.pPayload[1] == 0x00)
        {
          BSP_LED_Off(LED_BLUE);
          APP_DBG_MSG("-- P2P APPLICATION SERVER 3 : LED1 OFF\n"); 
          APP_DBG_MSG(" \n\r");
          P2P_Server_App_Context.LedControl.Led1=0x00; /* LED1 OFF */
        }
      }
#endif
#if(P2P_SERVER4 != 0)
      if(pNotification->DataTransfered.pPayload[0] == 0x04){ /* end device 4 selected */ 
        if(pNotification->DataTransfered.pPayload[1] == 0x01)
        {
          BSP_LED_On(LED_BLUE);
           APP_DBG_MSG("-- P2P APPLICATION SERVER 2 : LED1 ON\n"); 
          APP_DBG_MSG(" \n\r");
          P2P_Server_App_Context.LedControl.Led1=0x01; /* LED1 ON */
        }
        if(pNotification->DataTransfered.pPayload[1] == 0x00)
        {
          BSP_LED_Off(LED_BLUE);
          APP_DBG_MSG("-- P2P APPLICATION SERVER 2 : LED1 OFF\n"); 
          APP_DBG_MSG(" \n\r");
          P2P_Server_App_Context.LedControl.Led1=0x00; /* LED1 OFF */
        }   
      }
#endif     
#if(P2P_SERVER5 != 0)  
      if(pNotification->DataTransfered.pPayload[0] == 0x05){ /* end device 5 selected - may be necessary as LB Routeur informs all connection */
        if(pNotification->DataTransfered.pPayload[1] == 0x01)
        {
          BSP_LED_On(LED_BLUE);
          APP_DBG_MSG("-- P2P APPLICATION SERVER 5 : LED1 ON\n"); 
          APP_DBG_MSG(" \n\r");
          P2P_Server_App_Context.LedControl.Led1=0x01; /* LED1 ON */
        }
        if(pNotification->DataTransfered.pPayload[1] == 0x00)
        {
          BSP_LED_Off(LED_BLUE);
          APP_DBG_MSG("-- P2P APPLICATION SERVER 5 : LED1 OFF\n"); 
          APP_DBG_MSG(" \n\r");
          P2P_Server_App_Context.LedControl.Led1=0x00; /* LED1 OFF */
        }
      }
#endif
#if(P2P_SERVER6 != 0)
      if(pNotification->DataTransfered.pPayload[0] == 0x06){ /* end device 6 selected */ 
        if(pNotification->DataTransfered.pPayload[1] == 0x01)
        {
          BSP_LED_On(LED_BLUE);
           APP_DBG_MSG("-- P2P APPLICATION SERVER 6 : LED1 ON\n"); 
          APP_DBG_MSG(" \n\r");
          P2P_Server_App_Context.LedControl.Led1=0x01; /* LED1 ON */
        }
        if(pNotification->DataTransfered.pPayload[1] == 0x00)
        {
          BSP_LED_Off(LED_BLUE);
          APP_DBG_MSG("-- P2P APPLICATION SERVER 6 : LED1 OFF\n"); 
          APP_DBG_MSG(" \n\r");
          P2P_Server_App_Context.LedControl.Led1=0x00; /* LED1 OFF */
        }   
      }
#endif 
#endif /* 0 */
/* USER CODE END P2PS_STM_WRITE_EVT */
      break;

    default:
/* USER CODE BEGIN P2PS_STM_App_Notification_default */
      
/* USER CODE END P2PS_STM_App_Notification_default */
      break;
  }
/* USER CODE BEGIN P2PS_STM_App_Notification_2 */

/* USER CODE END P2PS_STM_App_Notification_2 */
  return;
}

void P2PS_APP_Notification(P2PS_APP_ConnHandle_Not_evt_t *pNotification)
{
/* USER CODE BEGIN P2PS_APP_Notification_1 */

/* USER CODE END P2PS_APP_Notification_1 */
  switch(pNotification->P2P_Evt_Opcode)
  {
/* USER CODE BEGIN P2PS_APP_Notification_P2P_Evt_Opcode */

/* USER CODE END P2PS_APP_Notification_P2P_Evt_Opcode */
  case PEER_CONN_HANDLE_EVT :
/* USER CODE BEGIN PEER_CONN_HANDLE_EVT */
          
/* USER CODE END PEER_CONN_HANDLE_EVT */
    break;

    case PEER_DISCON_HANDLE_EVT :
/* USER CODE BEGIN PEER_DISCON_HANDLE_EVT */
      P2P_Server_App_Context.Notification_Status = 0;
      QB55_txq_head = QB55_txq_tail = 0;
      QB55_rxlen = 0;
      QB55_retries = 0;
      HW_TS_Stop(QB55_RetryTimerId);
       P2PS_APP_LED_BUTTON_context_Init();       
/* USER CODE END PEER_DISCON_HANDLE_EVT */
    break;

    default:
/* USER CODE BEGIN P2PS_APP_Notification_default */

/* USER CODE END P2PS_APP_Notification_default */
      break;
  }
/* USER CODE BEGIN P2PS_APP_Notification_2 */

/* USER CODE END P2PS_APP_Notification_2 */
  return;
}

void P2PS_APP_Init(void)
{
/* USER CODE BEGIN P2PS_APP_Init */
  UTIL_SEQ_RegTask( 1<< CFG_TASK_SW1_BUTTON_PUSHED_ID, UTIL_SEQ_RFU, P2PS_Send_Notification );

  /**
   * Initialize LedButton Service
   */
  P2P_Server_App_Context.Notification_Status=0;
  P2PS_APP_LED_BUTTON_context_Init();

  /* LD3 (red) is the LED the "led on" / "led off" commands control. LD1 is dead on this board and
     ST's code owns LD2 (advertising / radio activity), so LD3 is the free one. Start it off. */
  BSP_LED_Off(LED_RED);

  QB55_In_HoldPowerState();   /* after UTIL_LPM_Init(): no Stop mode, the PWM must keep running */
  QB55_Thermo_Init();         /* ADC + temperature controller; stays off until "pid on" */

  UTIL_SEQ_RegTask(1U << CFG_TASK_UART_DRAIN_ID, UTIL_SEQ_RFU, QB55_Uart_Drain_Task);
  HW_TS_Create(CFG_TIM_PROC_ID_ISR, &QB55_RetryTimerId, hw_ts_SingleShot, QB55_Uart_Retry_TimerCb);
/* USER CODE END P2PS_APP_Init */
  return;
}

/* USER CODE BEGIN QB55_UART_TX */
/* Runs in the timer-server ISR, so only hand the work to the sequencer: ACI commands must not
   be issued from an interrupt. */
static void QB55_Uart_Retry_TimerCb(void)
{
  UTIL_SEQ_SetTask(1U << CFG_TASK_UART_DRAIN_ID, CFG_SCH_PRIO_0);
}

/* Queue bytes for the connected client. All-or-nothing, so a reply is never sent torn. */
static void QB55_Uart_Send(const uint8_t *data, uint16_t len)
{
  uint16_t i;

  if (!P2P_Server_App_Context.Notification_Status ||
      len > (uint16_t)(QB55_TXQ_SIZE - (uint16_t)(QB55_txq_head - QB55_txq_tail)))
  {
    dbgTxDropped += len;
    return;
  }
  for (i = 0; i < len; i++)
  {
    QB55_txq[QB55_txq_head++ & (QB55_TXQ_SIZE - 1U)] = data[i];
  }
  UTIL_SEQ_SetTask(1U << CFG_TASK_UART_DRAIN_ID, CFG_SCH_PRIO_0);
}

/* Send at most one notification worth (20 bytes) per run, then reschedule while data remains.
   If the stack refuses (its buffers are full), keep the bytes and retry from a short timer. */
static void QB55_Uart_Drain_Task(void)
{
  uint8_t  chunk[P2P_UART_MAX_PAYLOAD];
  uint16_t n = (uint16_t)(QB55_txq_head - QB55_txq_tail);
  uint16_t i;

  if (n == 0U || !P2P_Server_App_Context.Notification_Status)
  {
    return;
  }
  if (n > P2P_UART_MAX_PAYLOAD)
  {
    n = P2P_UART_MAX_PAYLOAD;
  }
  for (i = 0; i < n; i++)
  {
    chunk[i] = QB55_txq[(QB55_txq_tail + i) & (QB55_TXQ_SIZE - 1U)];
  }

  /* Record the status, not just the attempt: a failed send is otherwise invisible. */
  dbgNotifyStatus = (uint8_t)P2PS_STM_Notify(chunk, (uint8_t)n);
  dbgNotifyCount++;
  if (dbgNotifyStatus == BLE_STATUS_SUCCESS)
  {
    dbgNotifyOkCount++;
    QB55_retries = 0;
    QB55_txq_tail = (uint16_t)(QB55_txq_tail + n);
    if (QB55_txq_head != QB55_txq_tail)
    {
      UTIL_SEQ_SetTask(1U << CFG_TASK_UART_DRAIN_ID, CFG_SCH_PRIO_0);
    }
  }
  else if (++QB55_retries <= QB55_RETRY_MAX)
  {
    HW_TS_Start(QB55_RetryTimerId, QB55_RETRY_TICKS);   /* stack busy: try again shortly */
  }
  else
  {
    dbgTxDropped += (uint16_t)(QB55_txq_head - QB55_txq_tail);   /* link is gone: give up */
    QB55_txq_head = QB55_txq_tail = 0;
    QB55_retries = 0;
  }
}

/* Called for every write to the RX characteristic. `data` is only valid during this call, so
   copy it out; and never issue BLE commands from here - the reply is queued and sent by the
   drain task. A line ends at '\n' ('\r' is ignored); an over-long line is flushed as-is. */
static void QB55_Uart_Rx(const uint8_t *data, uint8_t len)
{
  uint8_t i;

  dbgRxBytes += len;
  for (i = 0; i < len; i++)
  {
    const uint8_t c = data[i];
    const uint8_t eol = (c == '\n');

    if (c != '\n' && c != '\r')
    {
      QB55_rxline[QB55_rxlen++] = c;
    }
    if (eol || QB55_rxlen == QB55_RX_LINE_MAX)
    {
      uint8_t reply[6U + QB55_RX_LINE_MAX + 2U];
      uint8_t k = 0;
      static const char prefix[] = "echo: ";

      while (k < sizeof(prefix) - 1U)
      {
        reply[k] = (uint8_t)prefix[k];
        k++;
      }
      for (uint8_t j = 0; j < QB55_rxlen; j++)
      {
        reply[k++] = QB55_rxline[j];
      }
      reply[k++] = '\r';
      reply[k++] = '\n';
      dbgRxLines++;
      QB55_Uart_Send(reply, k);        /* every line is echoed ... */
      QB55_Uart_Command(QB55_rxline, QB55_rxlen);   /* ... and may also be a command */
      QB55_rxlen = 0;
    }
  }
}

/* Commands are whole lines, matched ignoring case and surrounding spaces. A recognised command
   acts and sends one extra confirmation line after the echo; anything else is only echoed. */
static void QB55_Uart_Command(const uint8_t *line, uint8_t len)
{
  char    cmd[QB55_RX_LINE_MAX + 1U];
  uint8_t start = 0;
  uint8_t end = len;
  uint8_t n = 0;

  while (start < end && line[start] == ' ')
  {
    start++;
  }
  while (end > start && line[end - 1U] == ' ')
  {
    end--;
  }
  for (; start < end; start++)
  {
    const char c = (char)line[start];
    cmd[n++] = (c >= 'A' && c <= 'Z') ? (char)(c + ('a' - 'A')) : c;
  }
  cmd[n] = '\0';

  if (strcmp(cmd, "led on") == 0)
  {
    static const uint8_t ok[] = "LED is ON\r\n";
    BSP_LED_On(LED_RED);
    QB55_Uart_Send(ok, sizeof(ok) - 1U);
  }
  else if (strcmp(cmd, "led off") == 0)
  {
    static const uint8_t ok[] = "LED is OFF\r\n";
    BSP_LED_Off(LED_RED);
    QB55_Uart_Send(ok, sizeof(ok) - 1U);
  }
  else
  {
    char reply[256];

    reply[0] = '\0';
    if (QB55_In_Command(cmd, reply, sizeof(reply)) ||        /* "IN+ 2V", "IN- 1.5V", "IN+" */
        QB55_Thermo_Command(cmd, reply, sizeof(reply)))      /* "set 40", "pid on", "status" */
    {
      QB55_Uart_Send((const uint8_t *)reply, (uint16_t)strlen(reply));
    }
  }
}

/* One line for the client, with the line ending added. Used by the controller for fault messages
   and the optional log; dropped silently when nobody is subscribed. */
void QB55_Uart_Print(const char *line)
{
  char buf[112];
  const int n = snprintf(buf, sizeof(buf), "%s\r\n", line);

  if (n > 0)
  {
    QB55_Uart_Send((const uint8_t *)buf, (uint16_t)((n < (int)sizeof(buf)) ? n : (int)sizeof(buf) - 1));
  }
}
/* USER CODE END QB55_UART_TX */

/* USER CODE BEGIN FD */
void P2PS_APP_LED_BUTTON_context_Init(void){
  
  BSP_LED_Off(LED_BLUE);
  APP_DBG_MSG("LED BLUE OFF\n");
  
  #if(P2P_SERVER1 != 0)
  P2P_Server_App_Context.LedControl.Device_Led_Selection=0x01; /* Device1 */
  P2P_Server_App_Context.LedControl.Led1=0x00; /* led OFF */
  P2P_Server_App_Context.ButtonControl.Device_Button_Selection=0x01;/* Device1 */
  P2P_Server_App_Context.ButtonControl.ButtonStatus=0x00;
#endif
#if(P2P_SERVER2 != 0)
  P2P_Server_App_Context.LedControl.Device_Led_Selection=0x02; /* Device2 */
  P2P_Server_App_Context.LedControl.Led1=0x00; /* led OFF */
  P2P_Server_App_Context.ButtonControl.Device_Button_Selection=0x02;/* Device2 */
  P2P_Server_App_Context.ButtonControl.ButtonStatus=0x00;
#endif  
#if(P2P_SERVER3 != 0)
  P2P_Server_App_Context.LedControl.Device_Led_Selection=0x03; /* Device3 */
  P2P_Server_App_Context.LedControl.Led1=0x00; /* led OFF */
  P2P_Server_App_Context.ButtonControl.Device_Button_Selection=0x03; /* Device3 */
  P2P_Server_App_Context.ButtonControl.ButtonStatus=0x00;
#endif
#if(P2P_SERVER4 != 0)
  P2P_Server_App_Context.LedControl.Device_Led_Selection=0x04; /* Device4 */
  P2P_Server_App_Context.LedControl.Led1=0x00; /* led OFF */
  P2P_Server_App_Context.ButtonControl.Device_Button_Selection=0x04; /* Device4 */
  P2P_Server_App_Context.ButtonControl.ButtonStatus=0x00;
#endif  
 #if(P2P_SERVER5 != 0)
  P2P_Server_App_Context.LedControl.Device_Led_Selection=0x05; /* Device5 */
  P2P_Server_App_Context.LedControl.Led1=0x00; /* led OFF */
  P2P_Server_App_Context.ButtonControl.Device_Button_Selection=0x05; /* Device5 */
  P2P_Server_App_Context.ButtonControl.ButtonStatus=0x00;
#endif
#if(P2P_SERVER6 != 0)
  P2P_Server_App_Context.LedControl.Device_Led_Selection=0x06; /* device6 */
  P2P_Server_App_Context.LedControl.Led1=0x00; /* led OFF */
  P2P_Server_App_Context.ButtonControl.Device_Button_Selection=0x06; /* Device6 */
  P2P_Server_App_Context.ButtonControl.ButtonStatus=0x00;
#endif  
}

void P2PS_APP_SW1_Button_Action(void)
{
  UTIL_SEQ_SetTask( 1<<CFG_TASK_SW1_BUTTON_PUSHED_ID, CFG_SCH_PRIO_0);

  return;
}
/* USER CODE END FD */

/*************************************************************
 *
 * LOCAL FUNCTIONS
 *
 *************************************************************/
/* USER CODE BEGIN FD_LOCAL_FUNCTIONS*/
void P2PS_Send_Notification(void)
{
 
  if(P2P_Server_App_Context.ButtonControl.ButtonStatus == 0x00){
    P2P_Server_App_Context.ButtonControl.ButtonStatus=0x01;
  } else {
    P2P_Server_App_Context.ButtonControl.ButtonStatus=0x00;
  }
  
   if(P2P_Server_App_Context.Notification_Status){ 
    APP_DBG_MSG("-- P2P APPLICATION SERVER  : INFORM CLIENT BUTTON 1 PUSHED \n ");
    APP_DBG_MSG(" \n\r");
    P2PS_STM_App_Update_Char(P2P_NOTIFY_CHAR_UUID, (uint8_t *)&P2P_Server_App_Context.ButtonControl);
   } else {
    APP_DBG_MSG("-- P2P APPLICATION SERVER : CAN'T INFORM CLIENT -  NOTIFICATION DISABLED\n "); 
   }

  return;
}

/* USER CODE END FD_LOCAL_FUNCTIONS*/
