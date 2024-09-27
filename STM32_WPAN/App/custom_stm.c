/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    App/custom_stm.c
  * @author  MCD Application Team
  * @brief   Custom Example Service.
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

/* Includes ------------------------------------------------------------------*/
#include "common_blesvc.h"
#include "custom_stm.h"

/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
typedef struct{
  uint16_t  CustomHidHdle;                    /**< Human_Interface_Device handle */
  uint16_t  CustomProt_ModeHdle;                  /**< ProtocolMode handle */
  uint16_t  CustomReportHdle;                  /**< Report handle */
  uint16_t  CustomRep_MapHdle;                  /**< ReportMap handle */
  uint16_t  CustomInfoHdle;                  /**< Information handle */
  uint16_t  CustomCtrl_PtHdle;                  /**< ControlPoint handle */
  uint16_t  CustomDev_InfoHdle;                    /**< Device_Information handle */
  uint16_t  CustomPnp_IdHdle;                  /**< PnP_ID handle */
  uint16_t  CustomBatHdle;                    /**< Battery handle */
  uint16_t  CustomBat_LvlHdle;                  /**< BatteryLevel handle */
/* USER CODE BEGIN Context */
  /* Place holder for Characteristic Descriptors Handle*/

/* USER CODE END Context */
}CustomContext_t;

extern uint16_t Connection_Handle;
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private defines -----------------------------------------------------------*/
#define UUID_128_SUPPORTED  1

#if (UUID_128_SUPPORTED == 1)
#define BM_UUID_LENGTH  UUID_TYPE_128
#else
#define BM_UUID_LENGTH  UUID_TYPE_16
#endif

#define BM_REQ_CHAR_SIZE    (3)

/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macros ------------------------------------------------------------*/
#define CHARACTERISTIC_DESCRIPTOR_ATTRIBUTE_OFFSET         2
#define CHARACTERISTIC_VALUE_ATTRIBUTE_OFFSET              1
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
uint16_t SizeProt_Mode = 1;
uint16_t SizeReport = 64;
uint16_t SizeRep_Map = 256;
uint16_t SizeInfo = 4;
uint16_t SizeCtrl_Pt = 1;
uint16_t SizePnp_Id = 7;
uint16_t SizeBat_Lvl = 1;

/**
 * START of Section BLE_DRIVER_CONTEXT
 */
static CustomContext_t CustomContext;

/**
 * END of Section BLE_DRIVER_CONTEXT
 */

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
static SVCCTL_EvtAckStatus_t Custom_STM_Event_Handler(void *pckt);

static tBleStatus Generic_STM_App_Update_Char_Ext(uint16_t ConnectionHandle, uint16_t ServiceHandle, uint16_t CharHandle, uint16_t CharValueLen, uint8_t *pPayload);

/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Functions Definition ------------------------------------------------------*/
/* USER CODE BEGIN PFD */

/* USER CODE END PFD */

/* Private functions ----------------------------------------------------------*/

/* USER CODE BEGIN PF */

/* USER CODE END PF */

/**
 * @brief  Event handler
 * @param  Event: Address of the buffer holding the Event
 * @retval Ack: Return whether the Event has been managed or not
 */
static SVCCTL_EvtAckStatus_t Custom_STM_Event_Handler(void *Event)
{
  SVCCTL_EvtAckStatus_t return_value;
  hci_event_pckt *event_pckt;
  evt_blecore_aci *blecore_evt;
  aci_gatt_attribute_modified_event_rp0 *attribute_modified;
  aci_gatt_notification_complete_event_rp0    *notification_complete;
  Custom_STM_App_Notification_evt_t     Notification;
  /* USER CODE BEGIN Custom_STM_Event_Handler_1 */

  /* USER CODE END Custom_STM_Event_Handler_1 */

  return_value = SVCCTL_EvtNotAck;
  event_pckt = (hci_event_pckt *)(((hci_uart_pckt*)Event)->data);

  switch (event_pckt->evt)
  {
    case HCI_VENDOR_SPECIFIC_DEBUG_EVT_CODE:
      blecore_evt = (evt_blecore_aci*)event_pckt->data;
      switch (blecore_evt->ecode)
      {
        case ACI_GATT_ATTRIBUTE_MODIFIED_VSEVT_CODE:
          /* USER CODE BEGIN EVT_BLUE_GATT_ATTRIBUTE_MODIFIED_BEGIN */

          /* USER CODE END EVT_BLUE_GATT_ATTRIBUTE_MODIFIED_BEGIN */
          attribute_modified = (aci_gatt_attribute_modified_event_rp0*)blecore_evt->data;
          if (attribute_modified->Attr_Handle == (CustomContext.CustomReportHdle + CHARACTERISTIC_DESCRIPTOR_ATTRIBUTE_OFFSET))
          {
            return_value = SVCCTL_EvtAckFlowEnable;
            /* USER CODE BEGIN CUSTOM_STM_Service_1_Char_2 */

            /* USER CODE END CUSTOM_STM_Service_1_Char_2 */
            switch (attribute_modified->Attr_Data[0])
            {
              /* USER CODE BEGIN CUSTOM_STM_Service_1_Char_2_attribute_modified */

              /* USER CODE END CUSTOM_STM_Service_1_Char_2_attribute_modified */

              /* Disabled Notification management */
              case (!(COMSVC_Notification)):
                /* USER CODE BEGIN CUSTOM_STM_Service_1_Char_2_Disabled_BEGIN */

                /* USER CODE END CUSTOM_STM_Service_1_Char_2_Disabled_BEGIN */
                Notification.Custom_Evt_Opcode = CUSTOM_STM_REPORT_NOTIFY_DISABLED_EVT;
                Custom_STM_App_Notification(&Notification);
                /* USER CODE BEGIN CUSTOM_STM_Service_1_Char_2_Disabled_END */

                /* USER CODE END CUSTOM_STM_Service_1_Char_2_Disabled_END */
                break;

              /* Enabled Notification management */
              case COMSVC_Notification:
                /* USER CODE BEGIN CUSTOM_STM_Service_1_Char_2_COMSVC_Notification_BEGIN */

                /* USER CODE END CUSTOM_STM_Service_1_Char_2_COMSVC_Notification_BEGIN */
                Notification.Custom_Evt_Opcode = CUSTOM_STM_REPORT_NOTIFY_ENABLED_EVT;
                Custom_STM_App_Notification(&Notification);
                /* USER CODE BEGIN CUSTOM_STM_Service_1_Char_2_COMSVC_Notification_END */

                /* USER CODE END CUSTOM_STM_Service_1_Char_2_COMSVC_Notification_END */
                break;

              default:
                /* USER CODE BEGIN CUSTOM_STM_Service_1_Char_2_default */

                /* USER CODE END CUSTOM_STM_Service_1_Char_2_default */
              break;
            }
          }  /* if (attribute_modified->Attr_Handle == (CustomContext.CustomReportHdle + CHARACTERISTIC_DESCRIPTOR_ATTRIBUTE_OFFSET))*/

          else if (attribute_modified->Attr_Handle == (CustomContext.CustomBat_LvlHdle + CHARACTERISTIC_DESCRIPTOR_ATTRIBUTE_OFFSET))
          {
            return_value = SVCCTL_EvtAckFlowEnable;
            /* USER CODE BEGIN CUSTOM_STM_Service_3_Char_1 */

            /* USER CODE END CUSTOM_STM_Service_3_Char_1 */
            switch (attribute_modified->Attr_Data[0])
            {
              /* USER CODE BEGIN CUSTOM_STM_Service_3_Char_1_attribute_modified */

              /* USER CODE END CUSTOM_STM_Service_3_Char_1_attribute_modified */

              /* Disabled Notification management */
              case (!(COMSVC_Notification)):
                /* USER CODE BEGIN CUSTOM_STM_Service_3_Char_1_Disabled_BEGIN */

                /* USER CODE END CUSTOM_STM_Service_3_Char_1_Disabled_BEGIN */
                Notification.Custom_Evt_Opcode = CUSTOM_STM_BAT_LVL_NOTIFY_DISABLED_EVT;
                Custom_STM_App_Notification(&Notification);
                /* USER CODE BEGIN CUSTOM_STM_Service_3_Char_1_Disabled_END */

                /* USER CODE END CUSTOM_STM_Service_3_Char_1_Disabled_END */
                break;

              /* Enabled Notification management */
              case COMSVC_Notification:
                /* USER CODE BEGIN CUSTOM_STM_Service_3_Char_1_COMSVC_Notification_BEGIN */

                /* USER CODE END CUSTOM_STM_Service_3_Char_1_COMSVC_Notification_BEGIN */
                Notification.Custom_Evt_Opcode = CUSTOM_STM_BAT_LVL_NOTIFY_ENABLED_EVT;
                Custom_STM_App_Notification(&Notification);
                /* USER CODE BEGIN CUSTOM_STM_Service_3_Char_1_COMSVC_Notification_END */

                /* USER CODE END CUSTOM_STM_Service_3_Char_1_COMSVC_Notification_END */
                break;

              default:
                /* USER CODE BEGIN CUSTOM_STM_Service_3_Char_1_default */

                /* USER CODE END CUSTOM_STM_Service_3_Char_1_default */
              break;
            }
          }  /* if (attribute_modified->Attr_Handle == (CustomContext.CustomBat_LvlHdle + CHARACTERISTIC_DESCRIPTOR_ATTRIBUTE_OFFSET))*/

          else if (attribute_modified->Attr_Handle == (CustomContext.CustomProt_ModeHdle + CHARACTERISTIC_VALUE_ATTRIBUTE_OFFSET))
          {
            return_value = SVCCTL_EvtAckFlowEnable;
            /* USER CODE BEGIN CUSTOM_STM_Service_1_Char_1_ACI_GATT_ATTRIBUTE_MODIFIED_VSEVT_CODE */

            /* USER CODE END CUSTOM_STM_Service_1_Char_1_ACI_GATT_ATTRIBUTE_MODIFIED_VSEVT_CODE */
          } /* if (attribute_modified->Attr_Handle == (CustomContext.CustomProt_ModeHdle + CHARACTERISTIC_VALUE_ATTRIBUTE_OFFSET))*/
          else if (attribute_modified->Attr_Handle == (CustomContext.CustomCtrl_PtHdle + CHARACTERISTIC_VALUE_ATTRIBUTE_OFFSET))
          {
            return_value = SVCCTL_EvtAckFlowEnable;
            /* USER CODE BEGIN CUSTOM_STM_Service_1_Char_5_ACI_GATT_ATTRIBUTE_MODIFIED_VSEVT_CODE */

            /* USER CODE END CUSTOM_STM_Service_1_Char_5_ACI_GATT_ATTRIBUTE_MODIFIED_VSEVT_CODE */
          } /* if (attribute_modified->Attr_Handle == (CustomContext.CustomCtrl_PtHdle + CHARACTERISTIC_VALUE_ATTRIBUTE_OFFSET))*/
          /* USER CODE BEGIN EVT_BLUE_GATT_ATTRIBUTE_MODIFIED_END */

          /* USER CODE END EVT_BLUE_GATT_ATTRIBUTE_MODIFIED_END */
          break;

        case ACI_GATT_READ_PERMIT_REQ_VSEVT_CODE :
          /* USER CODE BEGIN EVT_BLUE_GATT_READ_PERMIT_REQ_BEGIN */

          /* USER CODE END EVT_BLUE_GATT_READ_PERMIT_REQ_BEGIN */
          /* USER CODE BEGIN EVT_BLUE_GATT_READ_PERMIT_REQ_END */

          /* USER CODE END EVT_BLUE_GATT_READ_PERMIT_REQ_END */
          break;

        case ACI_GATT_WRITE_PERMIT_REQ_VSEVT_CODE:
          /* USER CODE BEGIN EVT_BLUE_GATT_WRITE_PERMIT_REQ_BEGIN */

          /* USER CODE END EVT_BLUE_GATT_WRITE_PERMIT_REQ_BEGIN */
          /* USER CODE BEGIN EVT_BLUE_GATT_WRITE_PERMIT_REQ_END */

          /* USER CODE END EVT_BLUE_GATT_WRITE_PERMIT_REQ_END */
          break;

		case ACI_GATT_NOTIFICATION_COMPLETE_VSEVT_CODE:
        {
          /* USER CODE BEGIN EVT_BLUE_GATT_NOTIFICATION_COMPLETE_BEGIN */

          /* USER CODE END EVT_BLUE_GATT_NOTIFICATION_COMPLETE_BEGIN */
          notification_complete = (aci_gatt_notification_complete_event_rp0*)blecore_evt->data;
          Notification.Custom_Evt_Opcode = CUSTOM_STM_NOTIFICATION_COMPLETE_EVT;
          Notification.AttrHandle = notification_complete->Attr_Handle;
          Custom_STM_App_Notification(&Notification);
          /* USER CODE BEGIN EVT_BLUE_GATT_NOTIFICATION_COMPLETE_END */

          /* USER CODE END EVT_BLUE_GATT_NOTIFICATION_COMPLETE_END */
          break;
        }

        /* USER CODE BEGIN BLECORE_EVT */

        /* USER CODE END BLECORE_EVT */
        default:
          /* USER CODE BEGIN EVT_DEFAULT */

          /* USER CODE END EVT_DEFAULT */
          break;
      }
      /* USER CODE BEGIN EVT_VENDOR*/

      /* USER CODE END EVT_VENDOR*/
      break; /* HCI_VENDOR_SPECIFIC_DEBUG_EVT_CODE */

      /* USER CODE BEGIN EVENT_PCKT_CASES*/

      /* USER CODE END EVENT_PCKT_CASES*/

    default:
      /* USER CODE BEGIN EVENT_PCKT*/

      /* USER CODE END EVENT_PCKT*/
      break;
  }

  /* USER CODE BEGIN Custom_STM_Event_Handler_2 */

  /* USER CODE END Custom_STM_Event_Handler_2 */

  return(return_value);
}/* end Custom_STM_Event_Handler */

/* Public functions ----------------------------------------------------------*/

/**
 * @brief  Service initialization
 * @param  None
 * @retval None
 */
void SVCCTL_InitCustomSvc(void)
{

  Char_UUID_t  uuid;
  tBleStatus ret = BLE_STATUS_INVALID_PARAMS;
  uint8_t max_attr_record;

  /* USER CODE BEGIN SVCCTL_InitCustomSvc_1 */

  /* USER CODE END SVCCTL_InitCustomSvc_1 */

  /**
   *  Register the event handler to the BLE controller
   */
  SVCCTL_RegisterSvcHandler(Custom_STM_Event_Handler);

  /**
   *          Human_Interface_Device
   *
   * Max_Attribute_Records = 1 + 2*5 + 1*no_of_char_with_notify_or_indicate_property + 1*no_of_char_with_broadcast_property
   * service_max_attribute_record = 1 for Human_Interface_Device +
   *                                2 for ProtocolMode +
   *                                2 for Report +
   *                                2 for ReportMap +
   *                                2 for Information +
   *                                2 for ControlPoint +
   *                                1 for Report configuration descriptor +
   *                              = 12
   *
   * This value doesn't take into account number of descriptors manually added
   * In case of descriptors added, please update the max_attr_record value accordingly in the next SVCCTL_InitService User Section
   */
  max_attr_record = 12;

  /* USER CODE BEGIN SVCCTL_InitService */
  /* max_attr_record to be updated if descriptors have been added */

  /* USER CODE END SVCCTL_InitService */

  uuid.Char_UUID_16 = 0x1812;
  ret = aci_gatt_add_service(UUID_TYPE_16,
                             (Service_UUID_t *) &uuid,
                             PRIMARY_SERVICE,
                             max_attr_record,
                             &(CustomContext.CustomHidHdle));
  if (ret != BLE_STATUS_SUCCESS)
  {
    APP_DBG_MSG("  Fail   : aci_gatt_add_service command: HID, error code: 0x%x \n\r", ret);
  }
  else
  {
    APP_DBG_MSG("  Success: aci_gatt_add_service command: HID \n\r");
  }

  /**
   *  ProtocolMode
   */
  uuid.Char_UUID_16 = 0x2a4e;
  ret = aci_gatt_add_char(CustomContext.CustomHidHdle,
                          UUID_TYPE_16, &uuid,
                          SizeProt_Mode,
                          CHAR_PROP_READ | CHAR_PROP_WRITE_WITHOUT_RESP,
                          ATTR_PERMISSION_NONE,
                          GATT_NOTIFY_ATTRIBUTE_WRITE,
                          0x10,
                          CHAR_VALUE_LEN_CONSTANT,
                          &(CustomContext.CustomProt_ModeHdle));
  if (ret != BLE_STATUS_SUCCESS)
  {
    APP_DBG_MSG("  Fail   : aci_gatt_add_char command   : PROT_MODE, error code: 0x%x \n\r", ret);
  }
  else
  {
    APP_DBG_MSG("  Success: aci_gatt_add_char command   : PROT_MODE \n\r");
  }

  /* USER CODE BEGIN SVCCTL_Init_Service1_Char1 */
  /* Place holder for Characteristic Descriptors */

  /* USER CODE END SVCCTL_Init_Service1_Char1 */
  /**
   *  Report
   */
  uuid.Char_UUID_16 = 0x2a4d;
  ret = aci_gatt_add_char(CustomContext.CustomHidHdle,
                          UUID_TYPE_16, &uuid,
                          SizeReport,
                          CHAR_PROP_READ | CHAR_PROP_NOTIFY,
                          ATTR_PERMISSION_NONE,
                          GATT_DONT_NOTIFY_EVENTS,
                          0x10,
                          CHAR_VALUE_LEN_VARIABLE,
                          &(CustomContext.CustomReportHdle));
  if (ret != BLE_STATUS_SUCCESS)
  {
    APP_DBG_MSG("  Fail   : aci_gatt_add_char command   : REPORT, error code: 0x%x \n\r", ret);
  }
  else
  {
    APP_DBG_MSG("  Success: aci_gatt_add_char command   : REPORT \n\r");
  }

  /* USER CODE BEGIN SVCCTL_Init_Service1_Char2 */
  /* Place holder for Characteristic Descriptors */

  /* USER CODE END SVCCTL_Init_Service1_Char2 */
  /**
   *  ReportMap
   */
  uuid.Char_UUID_16 = 0x2a4b;
  ret = aci_gatt_add_char(CustomContext.CustomHidHdle,
                          UUID_TYPE_16, &uuid,
                          SizeRep_Map,
                          CHAR_PROP_READ,
                          ATTR_PERMISSION_NONE,
                          GATT_DONT_NOTIFY_EVENTS,
                          0x10,
                          CHAR_VALUE_LEN_VARIABLE,
                          &(CustomContext.CustomRep_MapHdle));
  if (ret != BLE_STATUS_SUCCESS)
  {
    APP_DBG_MSG("  Fail   : aci_gatt_add_char command   : REP_MAP, error code: 0x%x \n\r", ret);
  }
  else
  {
    APP_DBG_MSG("  Success: aci_gatt_add_char command   : REP_MAP \n\r");
  }

  /* USER CODE BEGIN SVCCTL_Init_Service1_Char3 */
  /* Place holder for Characteristic Descriptors */

  /* USER CODE END SVCCTL_Init_Service1_Char3 */
  /**
   *  Information
   */
  uuid.Char_UUID_16 = 0x2a4a;
  ret = aci_gatt_add_char(CustomContext.CustomHidHdle,
                          UUID_TYPE_16, &uuid,
                          SizeInfo,
                          CHAR_PROP_READ,
                          ATTR_PERMISSION_NONE,
                          GATT_DONT_NOTIFY_EVENTS,
                          0x10,
                          CHAR_VALUE_LEN_CONSTANT,
                          &(CustomContext.CustomInfoHdle));
  if (ret != BLE_STATUS_SUCCESS)
  {
    APP_DBG_MSG("  Fail   : aci_gatt_add_char command   : INFO, error code: 0x%x \n\r", ret);
  }
  else
  {
    APP_DBG_MSG("  Success: aci_gatt_add_char command   : INFO \n\r");
  }

  /* USER CODE BEGIN SVCCTL_Init_Service1_Char4 */
  /* Place holder for Characteristic Descriptors */

  /* USER CODE END SVCCTL_Init_Service1_Char4 */
  /**
   *  ControlPoint
   */
  uuid.Char_UUID_16 = 0x2a4c;
  ret = aci_gatt_add_char(CustomContext.CustomHidHdle,
                          UUID_TYPE_16, &uuid,
                          SizeCtrl_Pt,
                          CHAR_PROP_WRITE_WITHOUT_RESP,
                          ATTR_PERMISSION_NONE,
                          GATT_NOTIFY_ATTRIBUTE_WRITE,
                          0x10,
                          CHAR_VALUE_LEN_CONSTANT,
                          &(CustomContext.CustomCtrl_PtHdle));
  if (ret != BLE_STATUS_SUCCESS)
  {
    APP_DBG_MSG("  Fail   : aci_gatt_add_char command   : CTRL_PT, error code: 0x%x \n\r", ret);
  }
  else
  {
    APP_DBG_MSG("  Success: aci_gatt_add_char command   : CTRL_PT \n\r");
  }

  /* USER CODE BEGIN SVCCTL_Init_Service1_Char5 */
  /* Place holder for Characteristic Descriptors */

  /* USER CODE END SVCCTL_Init_Service1_Char5 */

  /**
   *          Device_Information
   *
   * Max_Attribute_Records = 1 + 2*1 + 1*no_of_char_with_notify_or_indicate_property + 1*no_of_char_with_broadcast_property
   * service_max_attribute_record = 1 for Device_Information +
   *                                2 for PnP_ID +
   *                              = 3
   *
   * This value doesn't take into account number of descriptors manually added
   * In case of descriptors added, please update the max_attr_record value accordingly in the next SVCCTL_InitService User Section
   */
  max_attr_record = 3;

  /* USER CODE BEGIN SVCCTL_InitService */
  /* max_attr_record to be updated if descriptors have been added */

  /* USER CODE END SVCCTL_InitService */

  uuid.Char_UUID_16 = 0x180a;
  ret = aci_gatt_add_service(UUID_TYPE_16,
                             (Service_UUID_t *) &uuid,
                             PRIMARY_SERVICE,
                             max_attr_record,
                             &(CustomContext.CustomDev_InfoHdle));
  if (ret != BLE_STATUS_SUCCESS)
  {
    APP_DBG_MSG("  Fail   : aci_gatt_add_service command: DEV_INFO, error code: 0x%x \n\r", ret);
  }
  else
  {
    APP_DBG_MSG("  Success: aci_gatt_add_service command: DEV_INFO \n\r");
  }

  /**
   *  PnP_ID
   */
  uuid.Char_UUID_16 = 0x2a50;
  ret = aci_gatt_add_char(CustomContext.CustomDev_InfoHdle,
                          UUID_TYPE_16, &uuid,
                          SizePnp_Id,
                          CHAR_PROP_READ,
                          ATTR_PERMISSION_NONE,
                          GATT_DONT_NOTIFY_EVENTS,
                          0x10,
                          CHAR_VALUE_LEN_CONSTANT,
                          &(CustomContext.CustomPnp_IdHdle));
  if (ret != BLE_STATUS_SUCCESS)
  {
    APP_DBG_MSG("  Fail   : aci_gatt_add_char command   : PNP_ID, error code: 0x%x \n\r", ret);
  }
  else
  {
    APP_DBG_MSG("  Success: aci_gatt_add_char command   : PNP_ID \n\r");
  }

  /* USER CODE BEGIN SVCCTL_Init_Service2_Char1 */
  /* Place holder for Characteristic Descriptors */

  /* USER CODE END SVCCTL_Init_Service2_Char1 */

  /**
   *          Battery
   *
   * Max_Attribute_Records = 1 + 2*1 + 1*no_of_char_with_notify_or_indicate_property + 1*no_of_char_with_broadcast_property
   * service_max_attribute_record = 1 for Battery +
   *                                2 for BatteryLevel +
   *                                1 for BatteryLevel configuration descriptor +
   *                              = 4
   *
   * This value doesn't take into account number of descriptors manually added
   * In case of descriptors added, please update the max_attr_record value accordingly in the next SVCCTL_InitService User Section
   */
  max_attr_record = 4;

  /* USER CODE BEGIN SVCCTL_InitService */
  /* max_attr_record to be updated if descriptors have been added */

  /* USER CODE END SVCCTL_InitService */

  uuid.Char_UUID_16 = 0x180f;
  ret = aci_gatt_add_service(UUID_TYPE_16,
                             (Service_UUID_t *) &uuid,
                             PRIMARY_SERVICE,
                             max_attr_record,
                             &(CustomContext.CustomBatHdle));
  if (ret != BLE_STATUS_SUCCESS)
  {
    APP_DBG_MSG("  Fail   : aci_gatt_add_service command: BAT, error code: 0x%x \n\r", ret);
  }
  else
  {
    APP_DBG_MSG("  Success: aci_gatt_add_service command: BAT \n\r");
  }

  /**
   *  BatteryLevel
   */
  uuid.Char_UUID_16 = 0x2a19;
  ret = aci_gatt_add_char(CustomContext.CustomBatHdle,
                          UUID_TYPE_16, &uuid,
                          SizeBat_Lvl,
                          CHAR_PROP_READ | CHAR_PROP_NOTIFY,
                          ATTR_PERMISSION_NONE,
                          GATT_DONT_NOTIFY_EVENTS,
                          0x10,
                          CHAR_VALUE_LEN_CONSTANT,
                          &(CustomContext.CustomBat_LvlHdle));
  if (ret != BLE_STATUS_SUCCESS)
  {
    APP_DBG_MSG("  Fail   : aci_gatt_add_char command   : BAT_LVL, error code: 0x%x \n\r", ret);
  }
  else
  {
    APP_DBG_MSG("  Success: aci_gatt_add_char command   : BAT_LVL \n\r");
  }

  /* USER CODE BEGIN SVCCTL_Init_Service3_Char1 */
  /* Place holder for Characteristic Descriptors */

  /* USER CODE END SVCCTL_Init_Service3_Char1 */

  /* USER CODE BEGIN SVCCTL_InitCustomSvc_2 */

  /* USER CODE END SVCCTL_InitCustomSvc_2 */

  return;
}

/**
 * @brief  Characteristic update
 * @param  CharOpcode: Characteristic identifier
 * @param  Service_Instance: Instance of the service to which the characteristic belongs
 *
 */
tBleStatus Custom_STM_App_Update_Char(Custom_STM_Char_Opcode_t CharOpcode, uint8_t *pPayload)
{
  tBleStatus ret = BLE_STATUS_INVALID_PARAMS;
  /* USER CODE BEGIN Custom_STM_App_Update_Char_1 */

  /* USER CODE END Custom_STM_App_Update_Char_1 */

  switch (CharOpcode)
  {

    case CUSTOM_STM_PROT_MODE:
      ret = aci_gatt_update_char_value(CustomContext.CustomHidHdle,
                                       CustomContext.CustomProt_ModeHdle,
                                       0, /* charValOffset */
                                       SizeProt_Mode, /* charValueLen */
                                       (uint8_t *)  pPayload);
      if (ret != BLE_STATUS_SUCCESS)
      {
        APP_DBG_MSG("  Fail   : aci_gatt_update_char_value PROT_MODE command, result : 0x%x \n\r", ret);
      }
      else
      {
        APP_DBG_MSG("  Success: aci_gatt_update_char_value PROT_MODE command\n\r");
      }
      /* USER CODE BEGIN CUSTOM_STM_App_Update_Service_1_Char_1*/

      /* USER CODE END CUSTOM_STM_App_Update_Service_1_Char_1*/
      break;

    case CUSTOM_STM_REPORT:
      ret = aci_gatt_update_char_value(CustomContext.CustomHidHdle,
                                       CustomContext.CustomReportHdle,
                                       0, /* charValOffset */
                                       SizeReport, /* charValueLen */
                                       (uint8_t *)  pPayload);
      if (ret != BLE_STATUS_SUCCESS)
      {
        APP_DBG_MSG("  Fail   : aci_gatt_update_char_value REPORT command, result : 0x%x \n\r", ret);
      }
      else
      {
        APP_DBG_MSG("  Success: aci_gatt_update_char_value REPORT command\n\r");
      }
      /* USER CODE BEGIN CUSTOM_STM_App_Update_Service_1_Char_2*/

      /* USER CODE END CUSTOM_STM_App_Update_Service_1_Char_2*/
      break;

    case CUSTOM_STM_REP_MAP:
      ret = aci_gatt_update_char_value(CustomContext.CustomHidHdle,
                                       CustomContext.CustomRep_MapHdle,
                                       0, /* charValOffset */
                                       SizeRep_Map, /* charValueLen */
                                       (uint8_t *)  pPayload);
      if (ret != BLE_STATUS_SUCCESS)
      {
        APP_DBG_MSG("  Fail   : aci_gatt_update_char_value REP_MAP command, result : 0x%x \n\r", ret);
      }
      else
      {
        APP_DBG_MSG("  Success: aci_gatt_update_char_value REP_MAP command\n\r");
      }
      /* USER CODE BEGIN CUSTOM_STM_App_Update_Service_1_Char_3*/

      /* USER CODE END CUSTOM_STM_App_Update_Service_1_Char_3*/
      break;

    case CUSTOM_STM_INFO:
      ret = aci_gatt_update_char_value(CustomContext.CustomHidHdle,
                                       CustomContext.CustomInfoHdle,
                                       0, /* charValOffset */
                                       SizeInfo, /* charValueLen */
                                       (uint8_t *)  pPayload);
      if (ret != BLE_STATUS_SUCCESS)
      {
        APP_DBG_MSG("  Fail   : aci_gatt_update_char_value INFO command, result : 0x%x \n\r", ret);
      }
      else
      {
        APP_DBG_MSG("  Success: aci_gatt_update_char_value INFO command\n\r");
      }
      /* USER CODE BEGIN CUSTOM_STM_App_Update_Service_1_Char_4*/

      /* USER CODE END CUSTOM_STM_App_Update_Service_1_Char_4*/
      break;

    case CUSTOM_STM_CTRL_PT:
      ret = aci_gatt_update_char_value(CustomContext.CustomHidHdle,
                                       CustomContext.CustomCtrl_PtHdle,
                                       0, /* charValOffset */
                                       SizeCtrl_Pt, /* charValueLen */
                                       (uint8_t *)  pPayload);
      if (ret != BLE_STATUS_SUCCESS)
      {
        APP_DBG_MSG("  Fail   : aci_gatt_update_char_value CTRL_PT command, result : 0x%x \n\r", ret);
      }
      else
      {
        APP_DBG_MSG("  Success: aci_gatt_update_char_value CTRL_PT command\n\r");
      }
      /* USER CODE BEGIN CUSTOM_STM_App_Update_Service_1_Char_5*/

      /* USER CODE END CUSTOM_STM_App_Update_Service_1_Char_5*/
      break;

    case CUSTOM_STM_PNP_ID:
      ret = aci_gatt_update_char_value(CustomContext.CustomDev_InfoHdle,
                                       CustomContext.CustomPnp_IdHdle,
                                       0, /* charValOffset */
                                       SizePnp_Id, /* charValueLen */
                                       (uint8_t *)  pPayload);
      if (ret != BLE_STATUS_SUCCESS)
      {
        APP_DBG_MSG("  Fail   : aci_gatt_update_char_value PNP_ID command, result : 0x%x \n\r", ret);
      }
      else
      {
        APP_DBG_MSG("  Success: aci_gatt_update_char_value PNP_ID command\n\r");
      }
      /* USER CODE BEGIN CUSTOM_STM_App_Update_Service_2_Char_1*/

      /* USER CODE END CUSTOM_STM_App_Update_Service_2_Char_1*/
      break;

    case CUSTOM_STM_BAT_LVL:
      ret = aci_gatt_update_char_value(CustomContext.CustomBatHdle,
                                       CustomContext.CustomBat_LvlHdle,
                                       0, /* charValOffset */
                                       SizeBat_Lvl, /* charValueLen */
                                       (uint8_t *)  pPayload);
      if (ret != BLE_STATUS_SUCCESS)
      {
        APP_DBG_MSG("  Fail   : aci_gatt_update_char_value BAT_LVL command, result : 0x%x \n\r", ret);
      }
      else
      {
        APP_DBG_MSG("  Success: aci_gatt_update_char_value BAT_LVL command\n\r");
      }
      /* USER CODE BEGIN CUSTOM_STM_App_Update_Service_3_Char_1*/

      /* USER CODE END CUSTOM_STM_App_Update_Service_3_Char_1*/
      break;

    default:
      break;
  }

  /* USER CODE BEGIN Custom_STM_App_Update_Char_2 */

  /* USER CODE END Custom_STM_App_Update_Char_2 */

  return ret;
}

/**
 * @brief  Characteristic update
 * @param  CharOpcode: Characteristic identifier
 * @param  pPayload: Characteristic value
 * @param  size: Length of the characteristic value in octets
 *
 */
tBleStatus Custom_STM_App_Update_Char_Variable_Length(Custom_STM_Char_Opcode_t CharOpcode, uint8_t *pPayload, uint8_t size)
{
  tBleStatus ret = BLE_STATUS_INVALID_PARAMS;
  /* USER CODE BEGIN Custom_STM_App_Update_Char_Variable_Length_1 */

  /* USER CODE END Custom_STM_App_Update_Char_Variable_Length_1 */

  switch (CharOpcode)
  {

    case CUSTOM_STM_PROT_MODE:
      ret = aci_gatt_update_char_value(CustomContext.CustomHidHdle,
                                       CustomContext.CustomProt_ModeHdle,
                                       0, /* charValOffset */
                                       size, /* charValueLen */
                                       (uint8_t *)  pPayload);
      if (ret != BLE_STATUS_SUCCESS)
      {
        APP_DBG_MSG("  Fail   : aci_gatt_update_char_value PROT_MODE command, result : 0x%x \n\r", ret);
      }
      else
      {
        APP_DBG_MSG("  Success: aci_gatt_update_char_value PROT_MODE command\n\r");
      }
      /* USER CODE BEGIN Custom_STM_App_Update_Char_Variable_Length_Service_1_Char_1*/

      /* USER CODE END Custom_STM_App_Update_Char_Variable_Length_Service_1_Char_1*/
      break;

    case CUSTOM_STM_REPORT:
      ret = aci_gatt_update_char_value(CustomContext.CustomHidHdle,
                                       CustomContext.CustomReportHdle,
                                       0, /* charValOffset */
                                       size, /* charValueLen */
                                       (uint8_t *)  pPayload);
      if (ret != BLE_STATUS_SUCCESS)
      {
        APP_DBG_MSG("  Fail   : aci_gatt_update_char_value REPORT command, result : 0x%x \n\r", ret);
      }
      else
      {
        APP_DBG_MSG("  Success: aci_gatt_update_char_value REPORT command\n\r");
      }
      /* USER CODE BEGIN Custom_STM_App_Update_Char_Variable_Length_Service_1_Char_2*/

      /* USER CODE END Custom_STM_App_Update_Char_Variable_Length_Service_1_Char_2*/
      break;

    case CUSTOM_STM_REP_MAP:
      ret = aci_gatt_update_char_value(CustomContext.CustomHidHdle,
                                       CustomContext.CustomRep_MapHdle,
                                       0, /* charValOffset */
                                       size, /* charValueLen */
                                       (uint8_t *)  pPayload);
      if (ret != BLE_STATUS_SUCCESS)
      {
        APP_DBG_MSG("  Fail   : aci_gatt_update_char_value REP_MAP command, result : 0x%x \n\r", ret);
      }
      else
      {
        APP_DBG_MSG("  Success: aci_gatt_update_char_value REP_MAP command\n\r");
      }
      /* USER CODE BEGIN Custom_STM_App_Update_Char_Variable_Length_Service_1_Char_3*/

      /* USER CODE END Custom_STM_App_Update_Char_Variable_Length_Service_1_Char_3*/
      break;

    case CUSTOM_STM_INFO:
      ret = aci_gatt_update_char_value(CustomContext.CustomHidHdle,
                                       CustomContext.CustomInfoHdle,
                                       0, /* charValOffset */
                                       size, /* charValueLen */
                                       (uint8_t *)  pPayload);
      if (ret != BLE_STATUS_SUCCESS)
      {
        APP_DBG_MSG("  Fail   : aci_gatt_update_char_value INFO command, result : 0x%x \n\r", ret);
      }
      else
      {
        APP_DBG_MSG("  Success: aci_gatt_update_char_value INFO command\n\r");
      }
      /* USER CODE BEGIN Custom_STM_App_Update_Char_Variable_Length_Service_1_Char_4*/

      /* USER CODE END Custom_STM_App_Update_Char_Variable_Length_Service_1_Char_4*/
      break;

    case CUSTOM_STM_CTRL_PT:
      ret = aci_gatt_update_char_value(CustomContext.CustomHidHdle,
                                       CustomContext.CustomCtrl_PtHdle,
                                       0, /* charValOffset */
                                       size, /* charValueLen */
                                       (uint8_t *)  pPayload);
      if (ret != BLE_STATUS_SUCCESS)
      {
        APP_DBG_MSG("  Fail   : aci_gatt_update_char_value CTRL_PT command, result : 0x%x \n\r", ret);
      }
      else
      {
        APP_DBG_MSG("  Success: aci_gatt_update_char_value CTRL_PT command\n\r");
      }
      /* USER CODE BEGIN Custom_STM_App_Update_Char_Variable_Length_Service_1_Char_5*/

      /* USER CODE END Custom_STM_App_Update_Char_Variable_Length_Service_1_Char_5*/
      break;

    case CUSTOM_STM_PNP_ID:
      ret = aci_gatt_update_char_value(CustomContext.CustomDev_InfoHdle,
                                       CustomContext.CustomPnp_IdHdle,
                                       0, /* charValOffset */
                                       size, /* charValueLen */
                                       (uint8_t *)  pPayload);
      if (ret != BLE_STATUS_SUCCESS)
      {
        APP_DBG_MSG("  Fail   : aci_gatt_update_char_value PNP_ID command, result : 0x%x \n\r", ret);
      }
      else
      {
        APP_DBG_MSG("  Success: aci_gatt_update_char_value PNP_ID command\n\r");
      }
      /* USER CODE BEGIN Custom_STM_App_Update_Char_Variable_Length_Service_2_Char_1*/

      /* USER CODE END Custom_STM_App_Update_Char_Variable_Length_Service_2_Char_1*/
      break;

    case CUSTOM_STM_BAT_LVL:
      ret = aci_gatt_update_char_value(CustomContext.CustomBatHdle,
                                       CustomContext.CustomBat_LvlHdle,
                                       0, /* charValOffset */
                                       size, /* charValueLen */
                                       (uint8_t *)  pPayload);
      if (ret != BLE_STATUS_SUCCESS)
      {
        APP_DBG_MSG("  Fail   : aci_gatt_update_char_value BAT_LVL command, result : 0x%x \n\r", ret);
      }
      else
      {
        APP_DBG_MSG("  Success: aci_gatt_update_char_value BAT_LVL command\n\r");
      }
      /* USER CODE BEGIN Custom_STM_App_Update_Char_Variable_Length_Service_3_Char_1*/

      /* USER CODE END Custom_STM_App_Update_Char_Variable_Length_Service_3_Char_1*/
      break;

    default:
      break;
  }

  /* USER CODE BEGIN Custom_STM_App_Update_Char_Variable_Length_2 */

  /* USER CODE END Custom_STM_App_Update_Char_Variable_Length_2 */

  return ret;
}

/**
 * @brief  Characteristic update
 * @param  Connection_Handle
 * @param  CharOpcode: Characteristic identifier
 * @param  pPayload: Characteristic value
 *
 */
tBleStatus Custom_STM_App_Update_Char_Ext(uint16_t Connection_Handle, Custom_STM_Char_Opcode_t CharOpcode, uint8_t *pPayload)
{
  tBleStatus ret = BLE_STATUS_INVALID_PARAMS;
  /* USER CODE BEGIN Custom_STM_App_Update_Char_Ext_1 */

  /* USER CODE END Custom_STM_App_Update_Char_Ext_1 */

  switch (CharOpcode)
  {

    case CUSTOM_STM_PROT_MODE:
      /* USER CODE BEGIN Updated_Length_Service_1_Char_1*/

      /* USER CODE END Updated_Length_Service_1_Char_1*/
	  Generic_STM_App_Update_Char_Ext(Connection_Handle, CustomContext.CustomHidHdle, CustomContext.CustomProt_ModeHdle, SizeProt_Mode, pPayload);

      break;

    case CUSTOM_STM_REPORT:
      /* USER CODE BEGIN Updated_Length_Service_1_Char_2*/

      /* USER CODE END Updated_Length_Service_1_Char_2*/
	  Generic_STM_App_Update_Char_Ext(Connection_Handle, CustomContext.CustomHidHdle, CustomContext.CustomReportHdle, SizeReport, pPayload);

      break;

    case CUSTOM_STM_REP_MAP:
      /* USER CODE BEGIN Updated_Length_Service_1_Char_3*/

      /* USER CODE END Updated_Length_Service_1_Char_3*/
	  Generic_STM_App_Update_Char_Ext(Connection_Handle, CustomContext.CustomHidHdle, CustomContext.CustomRep_MapHdle, SizeRep_Map, pPayload);

      break;

    case CUSTOM_STM_INFO:
      /* USER CODE BEGIN Updated_Length_Service_1_Char_4*/

      /* USER CODE END Updated_Length_Service_1_Char_4*/
	  Generic_STM_App_Update_Char_Ext(Connection_Handle, CustomContext.CustomHidHdle, CustomContext.CustomInfoHdle, SizeInfo, pPayload);

      break;

    case CUSTOM_STM_CTRL_PT:
      /* USER CODE BEGIN Updated_Length_Service_1_Char_5*/

      /* USER CODE END Updated_Length_Service_1_Char_5*/
	  Generic_STM_App_Update_Char_Ext(Connection_Handle, CustomContext.CustomHidHdle, CustomContext.CustomCtrl_PtHdle, SizeCtrl_Pt, pPayload);

      break;

    case CUSTOM_STM_PNP_ID:
      /* USER CODE BEGIN Updated_Length_Service_2_Char_1*/

      /* USER CODE END Updated_Length_Service_2_Char_1*/
	  Generic_STM_App_Update_Char_Ext(Connection_Handle, CustomContext.CustomDev_InfoHdle, CustomContext.CustomPnp_IdHdle, SizePnp_Id, pPayload);

      break;

    case CUSTOM_STM_BAT_LVL:
      /* USER CODE BEGIN Updated_Length_Service_3_Char_1*/

      /* USER CODE END Updated_Length_Service_3_Char_1*/
	  Generic_STM_App_Update_Char_Ext(Connection_Handle, CustomContext.CustomBatHdle, CustomContext.CustomBat_LvlHdle, SizeBat_Lvl, pPayload);

      break;

    default:
      break;
  }

  /* USER CODE BEGIN Custom_STM_App_Update_Char_Ext_2 */

  /* USER CODE END Custom_STM_App_Update_Char_Ext_2 */

  return ret;
}

static tBleStatus Generic_STM_App_Update_Char_Ext(uint16_t ConnectionHandle, uint16_t ServiceHandle, uint16_t CharHandle, uint16_t CharValueLen, uint8_t *pPayload)
{
  tBleStatus ret = BLE_STATUS_INVALID_PARAMS;

  ret = aci_gatt_update_char_value_ext(ConnectionHandle,
                                       ServiceHandle,
                                       CharHandle,
                                       0, /* update type:0 do not notify, 1 notify, 2 indicate */
                                       CharValueLen, /* charValueLen */
                                       0, /* value offset */
                                       243, /* value length */
                                       (uint8_t *)  pPayload);
  if (ret != BLE_STATUS_SUCCESS)
  {
    APP_DBG_MSG("  Fail   : aci_gatt_update_char_value_ext command, part 1, result : 0x%x \n\r", ret);
  }
  else
  {
    APP_DBG_MSG("  Success: aci_gatt_update_char_value_ext command, part 1\n\r");
  }
  /* USER CODE BEGIN Custom_STM_App_Update_Char_Ext_Service_1_Char_1*/

  if (CharValueLen-243<=243)
  {
    ret = aci_gatt_update_char_value_ext(ConnectionHandle,
                                         ServiceHandle,
                                         CharHandle,
                                         1, /* update type:0 do not notify, 1 notify, 2 indicate */
                                         CharValueLen, /* charValueLen */
                                         243, /* value offset */
                                         CharValueLen-243, /* value length */
                                         (uint8_t *)  ((pPayload)+243));
    if (ret != BLE_STATUS_SUCCESS)
    {
      APP_DBG_MSG("  Fail   : aci_gatt_update_char_value_ext command, part 2, result : 0x%x \n\r", ret);
    }
    else
    {
      APP_DBG_MSG("  Success: aci_gatt_update_char_value_ext command, part 2\n\r");
    }
  }
  else
  {
    ret = aci_gatt_update_char_value_ext(ConnectionHandle,
                                         ServiceHandle,
                                         CharHandle,
                                         0, /* update type:0 do not notify, 1 notify, 2 indicate */
                                         CharValueLen, /* charValueLen */
                                         243, /* value offset */
                                         243, /* value length */
                                         (uint8_t *)  ((pPayload)+243));
    if (ret != BLE_STATUS_SUCCESS)
    {
      APP_DBG_MSG("  Fail   : aci_gatt_update_char_value_ext command, part 3, result : 0x%x \n\r", ret);
    }
    else
    {
      APP_DBG_MSG("  Success: aci_gatt_update_char_value_ext command, part 3\n\r");
    }
    ret = aci_gatt_update_char_value_ext(ConnectionHandle,
                                         ServiceHandle,
                                         CharHandle,
                                         1, /* update type:0 do not notify, 1 notify, 2 indicate */
                                         CharValueLen, /* charValueLen */
                                         243+243, /* value offset */
                                         CharValueLen-243-243, /* value length */
                                         (uint8_t *)  ((pPayload)+243+243));
    if (ret != BLE_STATUS_SUCCESS)
    {
      APP_DBG_MSG("  Fail   : aci_gatt_update_char_value_ext command, part 4, result : 0x%x \n\r", ret);
    }
    else
    {
      APP_DBG_MSG("  Success: aci_gatt_update_char_value_ext command, part 4\n\r");
    }
  }
  return ret;
}

