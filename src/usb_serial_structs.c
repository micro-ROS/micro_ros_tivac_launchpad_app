#include <stdint.h>
#include <stdbool.h>
#include "inc/hw_types.h"
#include "driverlib/usb.h"
#include "usblib/usblib.h"
#include "usblib/usbcdc.h"
#include "usblib/usb-ids.h"
#include "usblib/device/usbdevice.h"
#include "usblib/device/usbdcdc.h"
#include "usb_serial_structs.h"

const uint8_t g_pui8LangDescriptor[] =
{
  4,
  USB_DTYPE_STRING,
  USBShort(USB_LANG_EN_US)
};

const uint8_t g_pui8ManufacturerString[] =
{
  (17 + 1) * 2,
  USB_DTYPE_STRING,
  'T', 0, 'e', 0, 'x', 0, 'a', 0, 's', 0, ' ', 0, 'I', 0, 'n', 0, 's', 0,
  't', 0, 'r', 0, 'u', 0, 'm', 0, 'e', 0, 'n', 0, 't', 0, 's', 0,
};

const uint8_t g_pui8ProductString[] =
{
  2 + (16 * 2),
  USB_DTYPE_STRING,
  'V', 0, 'i', 0, 'r', 0, 't', 0, 'u', 0, 'a', 0, 'l', 0, ' ', 0,
  'C', 0, 'O', 0, 'M', 0, ' ', 0, 'P', 0, 'o', 0, 'r', 0, 't', 0
};

const uint8_t g_pui8SerialNumberString[] =
{
  2 + (8 * 2),
  USB_DTYPE_STRING,
  '1', 0, '2', 0, '3', 0, '4', 0, '5', 0, '6', 0, '7', 0, '8', 0
};

const uint8_t g_pui8ControlInterfaceString[] =
{
  2 + (21 * 2),
  USB_DTYPE_STRING,
  'A', 0, 'C', 0, 'M', 0, ' ', 0, 'C', 0, 'o', 0, 'n', 0, 't', 0,
  'r', 0, 'o', 0, 'l', 0, ' ', 0, 'I', 0, 'n', 0, 't', 0, 'e', 0,
  'r', 0, 'f', 0, 'a', 0, 'c', 0, 'e', 0
};

const uint8_t g_pui8ConfigString[] =
{
  2 + (26 * 2),
  USB_DTYPE_STRING,
  'S', 0, 'e', 0, 'l', 0, 'f', 0, ' ', 0, 'P', 0, 'o', 0, 'w', 0,
  'e', 0, 'r', 0, 'e', 0, 'd', 0, ' ', 0, 'C', 0, 'o', 0, 'n', 0,
  'f', 0, 'i', 0, 'g', 0, 'u', 0, 'r', 0, 'a', 0, 't', 0, 'i', 0,
  'o', 0, 'n', 0
};

const uint8_t * const g_ppui8StringDescriptors[] =
{
  g_pui8LangDescriptor,
  g_pui8ManufacturerString,
  g_pui8ProductString,
  g_pui8SerialNumberString,
  g_pui8ControlInterfaceString,
  g_pui8ConfigString
};

#define NUM_STRING_DESCRIPTORS (sizeof(g_ppui8StringDescriptors) / \
  sizeof(uint8_t *))

uint32_t RxHandler(
  void * pvCBData, uint32_t ui32Event,
  uint32_t ui32MsgValue, void * pvMsgData);
uint32_t TxHandler(
  void * pvCBData, uint32_t ui32Event,
  uint32_t ui32MsgValue, void * pvMsgData);
uint32_t ControlHandler(
  void * pvCBData, uint32_t ui32Event,
  uint32_t ui32MsgValue, void * pvMsgData);

tUSBDCDCDevice g_sCDCDevice =
{
  USB_VID_TI_1CBE,
  USB_PID_SERIAL,
  0,
  USB_CONF_ATTR_SELF_PWR,
  ControlHandler,
  (void *)&g_sCDCDevice,
  USBBufferEventCallback,
  (void *)&g_sRxBuffer,
  USBBufferEventCallback,
  (void *)&g_sTxBuffer,
  g_ppui8StringDescriptors,
  NUM_STRING_DESCRIPTORS
};

uint8_t g_pui8USBRxBuffer[UART_BUFFER_SIZE];
tUSBBuffer g_sRxBuffer =
{
  false,                            // This is a receive buffer.
  RxHandler,                        // pfnCallback
  (void *)&g_sCDCDevice,            // Callback data is our device pointer.
  USBDCDCPacketRead,                // pfnTransfer
  USBDCDCRxPacketAvailable,         // pfnAvailable
  (void *)&g_sCDCDevice,            // pvHandle
  g_pui8USBRxBuffer,                // pui8Buffer
  UART_BUFFER_SIZE,                 // ui32BufferSize
};

uint8_t g_pui8USBTxBuffer[UART_BUFFER_SIZE];
tUSBBuffer g_sTxBuffer =
{
  true,                             // This is a transmit buffer.
  TxHandler,                        // pfnCallback
  (void *)&g_sCDCDevice,            // Callback data is our device pointer.
  USBDCDCPacketWrite,               // pfnTransfer
  USBDCDCTxPacketAvailable,         // pfnAvailable
  (void *)&g_sCDCDevice,            // pvHandle
  g_pui8USBTxBuffer,                // pui8Buffer
  UART_BUFFER_SIZE,                 // ui32BufferSize
};
