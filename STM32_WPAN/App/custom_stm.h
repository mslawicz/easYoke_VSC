/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    App/custom_stm.h
  * @author  MCD Application Team
  * @brief   Header for custom_stm.c module.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2024 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef CUSTOM_STM_H
#define CUSTOM_STM_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
typedef enum
{
  /* Human_Interface_Device */
  CUSTOM_STM_PROT_MODE,
  CUSTOM_STM_REPORT,
  CUSTOM_STM_REP_MAP,
  CUSTOM_STM_INFO,
  CUSTOM_STM_CTRL_PT,
  /* Device_Information */
  CUSTOM_STM_PNP_ID,
  /* Battery */
  CUSTOM_STM_BAT_LVL,
} Custom_STM_Char_Opcode_t;

typedef enum
{
  /* ProtocolMode */
  CUSTOM_STM_PROT_MODE_READ_EVT,
  CUSTOM_STM_PROT_MODE_WRITE_NO_RESP_EVT,
  /* Report */
  CUSTOM_STM_REPORT_READ_EVT,
  CUSTOM_STM_REPORT_NOTIFY_ENABLED_EVT,
  CUSTOM_STM_REPORT_NOTIFY_DISABLED_EVT,
  /* ReportMap */
  CUSTOM_STM_REP_MAP_READ_EVT,
  /* Information */
  CUSTOM_STM_INFO_READ_EVT,
  /* ControlPoint */
  CUSTOM_STM_CTRL_PT_WRITE_NO_RESP_EVT,
  /* PnP_ID */
  CUSTOM_STM_PNP_ID_READ_EVT,
  /* BatteryLevel */
  CUSTOM_STM_BAT_LVL_READ_EVT,
  CUSTOM_STM_BAT_LVL_NOTIFY_ENABLED_EVT,
  CUSTOM_STM_BAT_LVL_NOTIFY_DISABLED_EVT,
  CUSTOM_STM_NOTIFICATION_COMPLETE_EVT,

  CUSTOM_STM_BOOT_REQUEST_EVT
} Custom_STM_Opcode_evt_t;

typedef struct
{
  uint8_t * pPayload;
  uint8_t   Length;
} Custom_STM_Data_t;

typedef struct
{
  Custom_STM_Opcode_evt_t       Custom_Evt_Opcode;
  Custom_STM_Data_t             DataTransfered;
  uint16_t                      ConnectionHandle;
  uint8_t                       ServiceInstance;
  uint16_t                      AttrHandle;
} Custom_STM_App_Notification_evt_t;

/* USER CODE BEGIN ET */

/* USER CODE END ET */

/* Exported constants --------------------------------------------------------*/
extern uint16_t SizeProt_Mode;
extern uint16_t SizeReport;
extern uint16_t SizeRep_Map;
extern uint16_t SizeInfo;
extern uint16_t SizeCtrl_Pt;
extern uint16_t SizePnp_Id;
extern uint16_t SizeBat_Lvl;

/* USER CODE BEGIN EC */

/* USER CODE END EC */

/* External variables --------------------------------------------------------*/
/* USER CODE BEGIN EV */

/* USER CODE END EV */

/* Exported macros -----------------------------------------------------------*/
/* USER CODE BEGIN EM */

/* USER CODE END EM */

/* Exported functions ------------------------------------------------------- */
void SVCCTL_InitCustomSvc(void);
void Custom_STM_App_Notification(Custom_STM_App_Notification_evt_t *pNotification);
tBleStatus Custom_STM_App_Update_Char(Custom_STM_Char_Opcode_t CharOpcode,  uint8_t *pPayload);
tBleStatus Custom_STM_App_Update_Char_Variable_Length(Custom_STM_Char_Opcode_t CharOpcode, uint8_t *pPayload, uint8_t size);
tBleStatus Custom_STM_App_Update_Char_Ext(uint16_t Connection_Handle, Custom_STM_Char_Opcode_t CharOpcode, uint8_t *pPayload);
/* USER CODE BEGIN EF */

/* USER CODE END EF */

#ifdef __cplusplus
}
#endif

#endif /*CUSTOM_STM_H */
