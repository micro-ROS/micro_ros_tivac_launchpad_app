#ifndef MICROROS_USBCDC_TRANSPORT_H_
#define MICROROS_USBCDC_TRANSPORT_H_

#include <stdbool.h>
#include <stdint.h>
#include <stdlib.h>

#include <rmw_microros/custom_transport.h>

bool tivac_usbcdc_transport_open(struct uxrCustomTransport * transport);
bool tivac_usbcdc_transport_close(struct uxrCustomTransport * transport);
size_t tivac_usbcdc_transport_write(
  struct uxrCustomTransport * transport, const uint8_t * buf,
  size_t len, uint8_t * errcode);
size_t tivac_usbcdc_transport_read(
  struct uxrCustomTransport * transport, uint8_t * buf, size_t len,
  int timeout, uint8_t * errcode);

#endif  // MICROROS_USBCDC_TRANSPORT_H_
