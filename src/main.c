#include <stdbool.h>
#include <stdint.h>
#include <string.h>

#include "inc/hw_ints.h"
#include "inc/hw_memmap.h"
#include "inc/hw_types.h"
#include "inc/hw_gpio.h"
#include "inc/hw_uart.h"
#include "inc/hw_sysctl.h"
#include "driverlib/debug.h"
#include "driverlib/fpu.h"
#include "driverlib/gpio.h"
#include "driverlib/pin_map.h"
#include "driverlib/interrupt.h"
#include "driverlib/sysctl.h"
#include "driverlib/systick.h"
#include "driverlib/timer.h"
#include "driverlib/uart.h"
#include "driverlib/usb.h"
#include "driverlib/rom.h"
#include "usblib/usblib.h"
#include "usblib/usbcdc.h"
#include "usblib/usb-ids.h"
#include "usblib/device/usbdevice.h"
#include "usblib/device/usbdcdc.h"
#include "utils/ustdlib.h"
#include "usb_serial_structs.h"
#include "utils/uartstdio.h"

#include "./config.h"

// ##########################
// ## System tick  handler ##
// ##########################

volatile uint32_t g_ui32SysTickCount = 0;
void SysTickIntHandler(void)
{
  g_ui32SysTickCount++;
}

// #####################
// ## USB CDC handler ##
// #####################

#define COMMAND_PACKET_RECEIVED 0x00000001
#define COMMAND_STATUS_UPDATE   0x00000002

volatile uint32_t g_ui32Flags = 0;
static volatile bool g_bUSBConfigured = false;
static tLineCoding g_sLineCoding = {115200, USB_CDC_STOP_BITS_1, USB_CDC_PARITY_NONE, 8};

static bool SetLineCoding(tLineCoding * psLineCoding)
{
  memcpy(&g_sLineCoding, psLineCoding, sizeof(tLineCoding));
  return true;
}

static void GetLineCoding(tLineCoding * psLineCoding)
{
  memcpy(psLineCoding, &g_sLineCoding, sizeof(tLineCoding));
}


uint32_t ControlHandler(
  void * pvCBData, uint32_t ui32Event, uint32_t ui32MsgValue,
  void * pvMsgData)
{

  uint32_t ui32IntsOff;

  switch (ui32Event) {

    case USB_EVENT_CONNECTED:

      g_bUSBConfigured = true;

      USBBufferFlush(&g_sTxBuffer);
      USBBufferFlush(&g_sRxBuffer);

      ui32IntsOff = ROM_IntMasterDisable();
      g_ui32Flags |= COMMAND_STATUS_UPDATE;
      if (!ui32IntsOff) {ROM_IntMasterEnable();}

      break;

    case USB_EVENT_DISCONNECTED:

      g_bUSBConfigured = false;

      ui32IntsOff = ROM_IntMasterDisable();
      g_ui32Flags |= COMMAND_STATUS_UPDATE;

      if (!ui32IntsOff) {ROM_IntMasterEnable();}
      break;

    case USBD_CDC_EVENT_GET_LINE_CODING:
      GetLineCoding(pvMsgData);
      break;
    case USBD_CDC_EVENT_SET_LINE_CODING:
      SetLineCoding(pvMsgData);
      break;
    case USBD_CDC_EVENT_SET_CONTROL_LINE_STATE:
    case USBD_CDC_EVENT_SEND_BREAK:
    case USBD_CDC_EVENT_CLEAR_BREAK:
    case USB_EVENT_SUSPEND:
    case USB_EVENT_RESUME:
    default:
      break;
  }

  return 0;
}

uint32_t TxHandler(void * pvCBData, uint32_t ui32Event, uint32_t ui32MsgValue, void * pvMsgData)
{
  switch (ui32Event) {
    case USB_EVENT_TX_COMPLETE:
      break;
    default:
      break;
  }

  return 0;
}


uint32_t RxHandler(void * pvCBData, uint32_t ui32Event, uint32_t ui32MsgValue, void * pvMsgData)
{
  uint32_t ui32Count;
  switch (ui32Event) {
    case USB_EVENT_DATA_REMAINING: {
        ui32Count = USBBufferDataAvailable(&g_sRxBuffer);
        return ui32Count;
      }
    case USB_EVENT_RX_AVAILABLE:
    case USB_EVENT_REQUEST_BUFFER:
    default:
      break;
  }

  return 0;
}

// ###################################
// ## printf putchar implementation ##
// ###################################

int __io_putchar(int ch)
{
  uint8_t c[1];
  c[0] = ch & 0x00FF;
  USBBufferWrite(&g_sTxBuffer, c, 1);
  return ch;
}

// #################
// ## Entry point ##
// #################

void micro_ros_task(void);

int main(void)
{
  ROM_FPUEnable();
  ROM_FPULazyStackingEnable();
  ROM_SysCtlClockSet(SYSCTL_SYSDIV_4 | SYSCTL_USE_PLL | SYSCTL_OSC_MAIN | SYSCTL_XTAL_16MHZ);

  ROM_SysCtlPeripheralEnable(SYSCTL_PERIPH_USB0);
  ROM_SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOD);
  ROM_GPIOPinTypeUSBAnalog(GPIO_PORTD_BASE, GPIO_PIN_5 | GPIO_PIN_4);
  g_bUSBConfigured = false;

  USBBufferInit(&g_sTxBuffer);
  USBBufferInit(&g_sRxBuffer);

  USBStackModeSet(0, eUSBModeForceDevice, 0);

  USBDCDCInit(0, &g_sCDCDevice);

  ROM_IntMasterEnable();

  ROM_SysTickPeriodSet(ROM_SysCtlClockGet() / SYSTICKS_PER_SECOND);
  ROM_SysTickIntEnable();
  ROM_SysTickEnable();

  while (!g_bUSBConfigured) {}

  micro_ros_task();

  while (1) {}
}
