#ifndef INA219C_PROC_H_
#define INA219C_PROC_H_

#ifdef __cplusplus
extern "C"
{
#endif

void I2CReceive(uint32_t ui32WorkerAddress, uint8_t ui32WorkerRegister, uint8_t *pReceiveData, uint8_t ui8NumBytes);
void I2CSend(uint32_t ui32WorkerAddress, uint8_t ui32WorkerRegister, uint8_t *pTransmitData, uint8_t ui8NumBytes);
void I2CSingleByteSend(uint8_t ui32WorkerAddress, uint8_t byte);

#ifdef __cplusplus
}
#endif

#endif
