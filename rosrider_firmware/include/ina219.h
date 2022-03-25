#ifndef INA219_H_
#define INA219_H_

#ifdef __cplusplus
extern "C"
{
#endif

// DOC: 0x44 for all hardware revisions with ina219c

#define INA219_ADDRESS (0x40)                       // 1000000 (A0+A1=GND)
#define INA219_READ (0x01)

#define INA219_REG_CONFIG (0x00)                    // config register address
#define INA219_CONFIG_RESET (0x8000)                // reset Bit
#define INA219_CONFIG_BVOLTAGERANGE_MASK (0x2000)   // bus voltage range mask

typedef enum INA219_CONFIG_BVOLTAGERANGE {
  INA219_CONFIG_BVOLTAGERANGE_16V = (0x0000),       // 0-16V Range
  INA219_CONFIG_BVOLTAGERANGE_32V = (0x2000),       // 0-32V Range
} INA219_CONFIG_BVOLTAGERANGE;

#define INA219_CONFIG_GAIN_MASK (0x1800)            // mask for gain bits

typedef enum INA219_CONFIG_GAIN {
  INA219_CONFIG_GAIN_1_40MV = (0x0000),             // gain 1, 40mV range
  INA219_CONFIG_GAIN_2_80MV = (0x0800),             // gain 2, 80mV range
  INA219_CONFIG_GAIN_4_160MV = (0x1000),            // gain 4, 160mV range
  INA219_CONFIG_GAIN_8_320MV = (0x1800),            // gain 8, 320mV range
} INA219_CONFIG_GAIN;

#define INA219_CONFIG_BADCRES_MASK (0x0780)         // mask for bus ADC resolution bits

typedef enum INA219_CONFIG_BADCRES {
  INA219_CONFIG_BADCRES_9BIT = (0x0000),            // 9-bit bus res = 0..511
  INA219_CONFIG_BADCRES_10BIT = (0x0080),           // 10-bit bus res = 0..1023
  INA219_CONFIG_BADCRES_11BIT = (0x0100),           // 11-bit bus res = 0..2047
  INA219_CONFIG_BADCRES_12BIT = (0x0180),           // 12-bit bus res = 0..4097
  INA219_CONFIG_BADCRES_12BIT_2S_1060US = (0x0480), // 2 x 12-bit bus samples averaged together
  INA219_CONFIG_BADCRES_12BIT_4S_2130US = (0x0500), // 4 x 12-bit bus samples averaged together
  INA219_CONFIG_BADCRES_12BIT_8S_4260US = (0x0580), // 8 x 12-bit bus samples averaged together
  INA219_CONFIG_BADCRES_12BIT_16S_8510US = (0x0600),// 16 x 12-bit bus samples averaged together
  INA219_CONFIG_BADCRES_12BIT_32S_17MS = (0x0680),  // 32 x 12-bit bus samples averaged together
  INA219_CONFIG_BADCRES_12BIT_64S_34MS =  (0x0700), // 64 x 12-bit bus samples averaged together
  INA219_CONFIG_BADCRES_12BIT_128S_69MS = (0x0780), // 128 x 12-bit bus samples averaged together
} INA219_CONFIG_BADCRES;

#define INA219_CONFIG_SADCRES_MASK (0x0078)         // mask for shunt adc resolution and averaging

typedef enum INA219_CONFIG_SADCRES {
  INA219_CONFIG_SADCRES_9BIT_1S_84US = (0x0000),    // 1 x 9-bit shunt sample
  INA219_CONFIG_SADCRES_10BIT_1S_148US = (0x0008),  // 1 x 10-bit shunt sample
  INA219_CONFIG_SADCRES_11BIT_1S_276US = (0x0010),  // 1 x 11-bit shunt sample
  INA219_CONFIG_SADCRES_12BIT_1S_532US = (0x0018),  // 1 x 12-bit shunt sample
  INA219_CONFIG_SADCRES_12BIT_2S_1060US = (0x0048), // 2 x 12-bit shunt samples averaged together
  INA219_CONFIG_SADCRES_12BIT_4S_2130US = (0x0050), // 4 x 12-bit shunt samples averaged together
  INA219_CONFIG_SADCRES_12BIT_8S_4260US = (0x0058), // 8 x 12-bit shunt samples averaged together
  INA219_CONFIG_SADCRES_12BIT_16S_8510US = (0x0060),// 16 x 12-bit shunt samples averaged together
  INA219_CONFIG_SADCRES_12BIT_32S_17MS = (0x0068),  // 32 x 12-bit shunt samples averaged together
  INA219_CONFIG_SADCRES_12BIT_64S_34MS = (0x0070),  // 64 x 12-bit shunt samples averaged together
  INA219_CONFIG_SADCRES_12BIT_128S_69MS = (0x0078), // 128 x 12-bit shunt samples averaged together
} INA219_CONFIG_SADCRES;

#define INA219_CONFIG_MODE_MASK (0x0007)            // mask for operating mode bits

typedef enum INA219_CONFIG_MODE {
  INA219_CONFIG_MODE_POWERDOWN = 0x00,              // power down
  INA219_CONFIG_MODE_SVOLT_TRIGGERED = 0x01,        // shunt voltage triggered
  INA219_CONFIG_MODE_BVOLT_TRIGGERED = 0x02,        // bus voltage triggered
  INA219_CONFIG_MODE_SANDBVOLT_TRIGGERED = 0x03,    // shunt and bus voltage triggered
  INA219_CONFIG_MODE_ADCOFF = 0x04,                 // ADC off
  INA219_CONFIG_MODE_SVOLT_CONTINUOUS = 0x05,       // shunt voltage continuous
  INA219_CONFIG_MODE_BVOLT_CONTINUOUS = 0x06,       // bus voltage continuous
  INA219_CONFIG_MODE_SANDBVOLT_CONTINUOUS = 0x07,   // shunt and bus voltage continuous
} INA219_CONFIG_MODE;

#define INA219_REG_SHUNTVOLTAGE (0x01)              // shunt voltage register
#define INA219_REG_BUSVOLTAGE (0x02)                // bus voltage register
#define INA219_REG_POWER (0x03)                     // power register
#define INA219_REG_CURRENT (0x04)                   // current register
#define INA219_REG_CALIBRATION (0x05)               // calibration register

#include <stdint.h>
#include <stdbool.h>
#include <stdio.h>

uint32_t ina219_calValue;
uint8_t ui8CalRegister[2];
uint8_t ui8Register[2];
uint8_t ui8Command[1];

float ina219_currentDivider_mA;
float ina219_powerMultiplier_mW;

extern void I2CReceive(uint32_t ui32WorkerAddress, uint8_t ui32WorkerRegister, uint8_t *pReceiveData, uint8_t ui8NumBytes);
extern void I2CSend(uint32_t ui32WorkerAddress, uint8_t ui32WorkerRegister, uint8_t *pTransmitData, uint8_t ui8NumBytes);
extern void I2CSingleByteSend(uint8_t ui32WorkerAddress, uint8_t byte);

void setCalibration_32V_2A(uint32_t ui32WorkerAddress);

void sleep(uint32_t ui32WorkerAddress);

float getBusVoltage_mV(uint32_t ui32WorkerAddress);
float getShuntVoltage_mV(uint32_t ui32WorkerAddress);
float getCurrent_mA(uint32_t ui32WorkerAddress);
float getPower_mW(uint32_t ui32WorkerAddress);

uint16_t getBusVoltage_raw(uint32_t ui32WorkerAddress);
uint16_t getShuntVoltage_raw(uint32_t ui32WorkerAddress);
uint16_t getCurrent_raw(uint32_t ui32WorkerAddress);
uint16_t getPower_raw(uint32_t ui32WorkerAddress);

#ifdef __cplusplus
}
#endif

#endif