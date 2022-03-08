#include "./microros_usbcdc_transport.h"

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

#include "./usb_serial_structs.h"

#include "uxr/client/util/time.h"

bool tivac_usbcdc_transport_open(struct uxrCustomTransport * transport)
{
  return true;
}

bool tivac_usbcdc_transport_close(struct uxrCustomTransport * transport)
{
  return true;
}

size_t tivac_usbcdc_transport_write(
  struct uxrCustomTransport * transport, const uint8_t * buf,
  size_t len, uint8_t * errcode)
{
  uint32_t written = USBBufferWrite(&g_sTxBuffer, buf, len);
  USBRingBufFlush(&g_sTxBuffer);
  return written;
}

size_t tivac_usbcdc_transport_read(
  struct uxrCustomTransport * transport, uint8_t * buf, size_t len,
  int timeout, uint8_t * errcode)
{
  int64_t start_time = uxr_millis();

  uint32_t available = 0;

  do {
    available = USBBufferDataAvailable(&g_sRxBuffer);
  } while (available == 0 && (uxr_millis() - start_time) < timeout);

  size_t readed = 0;

  if (available > 0) {
    uint32_t to_read = (available > len) ? len : available;
    readed = USBBufferRead(&g_sRxBuffer, buf, to_read);
  }

  return readed;
}
