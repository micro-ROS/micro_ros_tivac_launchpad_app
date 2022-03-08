#ifndef _USB_SERIAL_STRUCTS_H_
#define _USB_SERIAL_STRUCTS_H_


#define UART_BUFFER_SIZE 256

extern uint32_t RxHandler(
  void * pvCBData, uint32_t ui32Event,
  uint32_t ui32MsgValue, void * pvMsgData);
extern uint32_t TxHandler(
  void * pvi32CBData, uint32_t ui32Event,
  uint32_t ui32MsgValue, void * pvMsgData);

extern tUSBBuffer g_sTxBuffer;
extern tUSBBuffer g_sRxBuffer;
extern tUSBDCDCDevice g_sCDCDevice;
extern uint8_t g_pui8USBTxBuffer[];
extern uint8_t g_pui8USBRxBuffer[];

#endif
