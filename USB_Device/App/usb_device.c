/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : usb_device.c
  * @version        : v3.0_Cube
  * @brief          : This file implements the USB Device
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2025 STMicroelectronics.
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

#include "usb_device.h"
#include "usbd_core.h"
#include "usbd_desc.h"
#include "usbd_hid.h"

/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* USER CODE BEGIN PV */

/* Private variables ---------------------------------------------------------*/

#define HID_KEYBOARD_REPORT_DESC_SIZE 63U

__ALIGN_BEGIN static uint8_t
    HID_KEYBOARD_ReportDesc[HID_KEYBOARD_REPORT_DESC_SIZE] __ALIGN_END = {
        0x05, 0x01,  // USAGE_PAGE (Generic Desktop)
        0x09, 0x06,  // USAGE (Keyboard)
        0xa1, 0x01,  // COLLECTION (Application)
        0x05, 0x07,  //   USAGE_PAGE (Keyboard)
        0x19, 0xe0,  //   USAGE_MINIMUM (Keyboard LeftControl)
        0x29, 0xe7,  //   USAGE_MAXIMUM (Keyboard Right GUI)
        0x15, 0x00,  //   LOGICAL_MINIMUM (0)
        0x25, 0x01,  //   LOGICAL_MAXIMUM (1)
        0x75, 0x01,  //   REPORT_SIZE (1)
        0x95, 0x08,  //   REPORT_COUNT (8)
        0x81, 0x02,  //   INPUT (Data,Var,Abs)
        0x95, 0x01,  //   REPORT_COUNT (1)
        0x75, 0x08,  //   REPORT_SIZE (8)
        0x81, 0x03,  //   INPUT (Cnst,Var,Abs)
        0x95, 0x05,  //   REPORT_COUNT (5)
        0x75, 0x01,  //   REPORT_SIZE (1)
        0x05, 0x08,  //   USAGE_PAGE (LEDs)
        0x19, 0x01,  //   USAGE_MINIMUM (Num Lock)
        0x29, 0x05,  //   USAGE_MAXIMUM (Kana)
        0x91, 0x02,  //   OUTPUT (Data,Var,Abs)
        0x95, 0x01,  //   REPORT_COUNT (1)
        0x75, 0x03,  //   REPORT_SIZE (3)
        0x91, 0x03,  //   OUTPUT (Cnst,Var,Abs)
        0x95, 0x06,  //   REPORT_COUNT (6)
        0x75, 0x08,  //   REPORT_SIZE (8)
        0x15, 0x00,  //   LOGICAL_MINIMUM (0)
        0x25, 0x65,  //   LOGICAL_MAXIMUM (101)
        0x05, 0x07,  //   USAGE_PAGE (Keyboard)
        0x19, 0x00,  //   USAGE_MINIMUM (Reserved (no event indicated))
        0x29, 0x65,  //   USAGE_MAXIMUM (Keyboard Application)
        0x81, 0x00,  //   INPUT (Data,Ary,Abs)
        0xc0         // END_COLLECTION
};

static USBD_ClassTypeDef hid_class_def;

/* USB HID device FS Configuration Descriptor */
__ALIGN_BEGIN static uint8_t HID_CfgDesc[USB_HID_CONFIG_DESC_SIZ] __ALIGN_END =
    {
        0x09, /* bLength: Configuration Descriptor size */
        USB_DESC_TYPE_CONFIGURATION, /* bDescriptorType: Configuration */
        USB_HID_CONFIG_DESC_SIZ,     /* wTotalLength: Bytes returned */
        0x00, 0x01,                  /* bNumInterfaces: 1 interface */
        0x01, /* bConfigurationValue: Configuration value */
        0x00, /* iConfiguration: Index of string descriptor
                 describing the configuration */
#if (USBD_SELF_POWERED == 1U)
        0xE0, /* bmAttributes: Bus Powered according to user configuration */
#else
        0xA0, /* bmAttributes: Bus Powered according to user configuration */
#endif                  /* USBD_SELF_POWERED */
        USBD_MAX_POWER, /* MaxPower (mA) */

        /************** Descriptor of Joystick Mouse interface ****************/
        /* 09 */
        0x09,                    /* bLength: Interface Descriptor size */
        USB_DESC_TYPE_INTERFACE, /* bDescriptorType: Interface descriptor type
                                  */
        0x00,                    /* bInterfaceNumber: Number of Interface */
        0x00,                    /* bAlternateSetting: Alternate setting */
        0x01,                    /* bNumEndpoints */
        0x03,                    /* bInterfaceClass: HID */
        0x01,                    /* bInterfaceSubClass : 1=BOOT, 0=no boot */
        0x01, /* nInterfaceProtocol : 0=none, 1=keyboard, 2=mouse */
        0,    /* iInterface: Index of string descriptor */
        /******************** Descriptor of Joystick Mouse HID
           ********************/
        /* 18 */
        0x09,                /* bLength: HID Descriptor size */
        HID_DESCRIPTOR_TYPE, /* bDescriptorType: HID */
        0x11,                /* bcdHID: HID Class Spec release number */
        0x01, 0x00,          /* bCountryCode: Hardware target country */
        0x01, /* bNumDescriptors: Number of HID class descriptors to follow */
        0x22, /* bDescriptorType */
        HID_KEYBOARD_REPORT_DESC_SIZE, /* wItemLength: Total length of Report
                                          descriptor */
        0x00,
        /******************** Descriptor of Mouse endpoint ********************/
        /* 27 */
        0x07,                   /* bLength: Endpoint Descriptor size */
        USB_DESC_TYPE_ENDPOINT, /* bDescriptorType:*/

        HID_EPIN_ADDR,          /* bEndpointAddress: Endpoint Address (IN) */
        0x03,                   /* bmAttributes: Interrupt endpoint */
        HID_EPIN_SIZE,          /* wMaxPacketSize: 4 Bytes max */
        0x00, HID_FS_BINTERVAL, /* bInterval: Polling Interval */
        /* 34 */
};

/* USB HID device Configuration Descriptor */
__ALIGN_BEGIN static uint8_t USB_HID_Desc[USB_HID_DESC_SIZ] __ALIGN_END =
{
  /* 18 */
  0x09,                                               /* bLength: HID Descriptor size */
  HID_DESCRIPTOR_TYPE,                                /* bDescriptorType: HID */
  0x11,                                               /* bcdHID: HID Class Spec release number */
  0x01,
  0x00,                                               /* bCountryCode: Hardware target country */
  0x01,                                               /* bNumDescriptors: Number of HID class descriptors to follow */
  0x22,                                               /* bDescriptorType */
  HID_KEYBOARD_REPORT_DESC_SIZE,                         /* wItemLength: Total length of Report descriptor */
  0x00,
};

/* USER CODE END PV */

/* USER CODE BEGIN PFP */
/* Private function prototypes -----------------------------------------------*/
static uint8_t *HID_GetFSCfgDesc(uint16_t *length);

/* USER CODE END PFP */

extern void Error_Handler(void);
/* USB Device Core handle declaration. */
USBD_HandleTypeDef hUsbDeviceFS;
extern USBD_DescriptorsTypeDef HID_Desc;

/*
 * -- Insert your variables declaration here --
 */
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/*
 * -- Insert your external function declaration here --
 */
/* USER CODE BEGIN 1 */
/**
  * @brief  USBD_HID_Setup
  *         Handle the HID specific requests
  * @param  pdev: instance
  * @param  req: usb requests
  * @retval status
  */
 static uint8_t HID_Setup(USBD_HandleTypeDef *pdev, USBD_SetupReqTypedef *req)
 {
   USBD_HID_HandleTypeDef *hhid = (USBD_HID_HandleTypeDef *)pdev->pClassDataCmsit[pdev->classId];
   USBD_StatusTypeDef ret = USBD_OK;
   uint16_t len;
   uint8_t *pbuf;
   uint16_t status_info = 0U;
 
   if (hhid == NULL)
   {
     return (uint8_t)USBD_FAIL;
   }
 
   switch (req->bmRequest & USB_REQ_TYPE_MASK)
   {
     case USB_REQ_TYPE_CLASS :
       switch (req->bRequest)
       {
         case USBD_HID_REQ_SET_PROTOCOL:
           hhid->Protocol = (uint8_t)(req->wValue);
           break;
 
         case USBD_HID_REQ_GET_PROTOCOL:
           (void)USBD_CtlSendData(pdev, (uint8_t *)&hhid->Protocol, 1U);
           break;
 
         case USBD_HID_REQ_SET_IDLE:
           hhid->IdleState = (uint8_t)(req->wValue >> 8);
           break;
 
         case USBD_HID_REQ_GET_IDLE:
           (void)USBD_CtlSendData(pdev, (uint8_t *)&hhid->IdleState, 1U);
           break;
 
         default:
           USBD_CtlError(pdev, req);
           ret = USBD_FAIL;
           break;
       }
       break;
     case USB_REQ_TYPE_STANDARD:
       switch (req->bRequest)
       {
         case USB_REQ_GET_STATUS:
           if (pdev->dev_state == USBD_STATE_CONFIGURED)
           {
             (void)USBD_CtlSendData(pdev, (uint8_t *)&status_info, 2U);
           }
           else
           {
             USBD_CtlError(pdev, req);
             ret = USBD_FAIL;
           }
           break;
 
         case USB_REQ_GET_DESCRIPTOR:
           if ((req->wValue >> 8) == HID_REPORT_DESC)
           {
             len = MIN(HID_KEYBOARD_REPORT_DESC_SIZE, req->wLength);
             pbuf = HID_KEYBOARD_ReportDesc;
           }
           else if ((req->wValue >> 8) == HID_DESCRIPTOR_TYPE)
           {
             pbuf = USB_HID_Desc;
             len = MIN(USB_HID_DESC_SIZ, req->wLength);
           }
           else
           {
             USBD_CtlError(pdev, req);
             ret = USBD_FAIL;
             break;
           }
           (void)USBD_CtlSendData(pdev, pbuf, len);
           break;
 
         case USB_REQ_GET_INTERFACE :
           if (pdev->dev_state == USBD_STATE_CONFIGURED)
           {
             (void)USBD_CtlSendData(pdev, (uint8_t *)&hhid->AltSetting, 1U);
           }
           else
           {
             USBD_CtlError(pdev, req);
             ret = USBD_FAIL;
           }
           break;
 
         case USB_REQ_SET_INTERFACE:
           if (pdev->dev_state == USBD_STATE_CONFIGURED)
           {
             hhid->AltSetting = (uint8_t)(req->wValue);
           }
           else
           {
             USBD_CtlError(pdev, req);
             ret = USBD_FAIL;
           }
           break;
 
         case USB_REQ_CLEAR_FEATURE:
           break;
 
         default:
           USBD_CtlError(pdev, req);
           ret = USBD_FAIL;
           break;
       }
       break;
 
     default:
       USBD_CtlError(pdev, req);
       ret = USBD_FAIL;
       break;
   }
 
   return (uint8_t)ret;
 }
 

static uint8_t *HID_GetFSCfgDesc(uint16_t *length) {
  USBD_EpDescTypeDef *pEpDesc = USBD_GetEpDesc(HID_CfgDesc, HID_EPIN_ADDR);

  if (pEpDesc != NULL) {
    pEpDesc->bInterval = HID_FS_BINTERVAL;
  }

  *length = (uint16_t)sizeof(HID_CfgDesc);
  return HID_CfgDesc;
}

/* USER CODE END 1 */

/**
  * Init USB device Library, add supported class and start the library
  * @retval None
  */
void MX_USB_Device_Init(void)
{
  /* USER CODE BEGIN USB_Device_Init_PreTreatment */
  hid_class_def = USBD_HID;
  hid_class_def.Setup = HID_Setup;
  hid_class_def.GetFSConfigDescriptor = HID_GetFSCfgDesc;

  /* USER CODE END USB_Device_Init_PreTreatment */

  /* Init Device Library, add supported class and start the library. */
  if (USBD_Init(&hUsbDeviceFS, &HID_Desc, DEVICE_FS) != USBD_OK) {
    Error_Handler();
  }
  if (USBD_RegisterClass(&hUsbDeviceFS, &hid_class_def) != USBD_OK) {
    Error_Handler();
  }
  if (USBD_Start(&hUsbDeviceFS) != USBD_OK) {
    Error_Handler();
  }
  /* USER CODE BEGIN USB_Device_Init_PostTreatment */

  /* USER CODE END USB_Device_Init_PostTreatment */
}

/**
  * @}
  */

/**
  * @}
  */

