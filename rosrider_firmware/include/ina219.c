#include "ina219.h"

void setCalibration_32V_2A(uint32_t ui32WorkerAddress) {

  ina219_calValue = 4096;

  ina219_currentDivider_mA = 10.0f;        // Current LSB = 100uA per bit (1000/100 = 10)
  ina219_powerMultiplier_mW = 2.0f;        // Power LSB = 1mW per bit (2/1)

  // calculate ui8CalRegister
  ui8CalRegister[0] = (ina219_calValue >> 8) & 0xFF;
  ui8CalRegister[1] = ina219_calValue & 0xFF;

  // initial send of calibration register
  I2CSend(ui32WorkerAddress, INA219_REG_CALIBRATION, ui8CalRegister, 2);

  uint16_t config = INA219_CONFIG_BVOLTAGERANGE_32V |
                    INA219_CONFIG_GAIN_8_320MV |
                    INA219_CONFIG_BADCRES_12BIT_8S_4260US |
                    INA219_CONFIG_SADCRES_12BIT_8S_4260US |
                    INA219_CONFIG_MODE_SANDBVOLT_CONTINUOUS;

  /*
  uint16_t config = INA219_CONFIG_BVOLTAGERANGE_32V |
                    INA219_CONFIG_GAIN_8_320MV |
                    INA219_CONFIG_BADCRES_12BIT |
                    INA219_CONFIG_SADCRES_12BIT_1S_532US |
                    INA219_CONFIG_MODE_SANDBVOLT_CONTINUOUS;
                    */

  ui8Register[0] = (config >> 8) & 0xFF;
  ui8Register[1] = config & 0xFF;
  I2CSend(ui32WorkerAddress, INA219_REG_CONFIG, ui8Register, 2);

}

void sleep(uint32_t ui32WorkerAddress) {
  ui8Register[0] = INA219_CONFIG_MODE_POWERDOWN;
  I2CSend(ui32WorkerAddress, INA219_REG_CALIBRATION, ui8Register, 1);
}

uint16_t getBusVoltage_raw(uint32_t ui32WorkerAddress) {
  I2CReceive(ui32WorkerAddress, INA219_REG_BUSVOLTAGE, ui8Register, 2);
  return ((((ui8Register[0] << 8) | ui8Register[1]) >> 3) * 4);
}

uint16_t getShuntVoltage_raw(uint32_t ui32WorkerAddress) {
  I2CReceive(ui32WorkerAddress, INA219_REG_SHUNTVOLTAGE, ui8Register, 2);
  return ((ui8Register[0] << 8) | ui8Register[1]);
}

uint16_t getCurrent_raw(uint32_t ui32WorkerAddress) {
  // resend calibration register
  I2CSend(ui32WorkerAddress, INA219_REG_CALIBRATION, ui8CalRegister, 2);
  // receive op
  I2CReceive(ui32WorkerAddress, INA219_REG_CURRENT, ui8Register, 2);
  return ((ui8Register[0] << 8) | ui8Register[1]);
}

uint16_t getPower_raw(uint32_t ui32WorkerAddress) {
  // resend calibration register
  I2CSend(ui32WorkerAddress, INA219_REG_CALIBRATION, ui8CalRegister, 2);
  // receive op
  I2CReceive(ui32WorkerAddress, INA219_REG_POWER, ui8Register, 2);
  return ((ui8Register[0] << 8) | ui8Register[1]);
}

float getShuntVoltage_mV(uint32_t ui32WorkerAddress) {
  uint16_t value = getShuntVoltage_raw(ui32WorkerAddress);
  return value * 0.01;
}

float getBusVoltage_mV(uint32_t ui32WorkerAddress) {
  uint16_t value = getBusVoltage_raw(ui32WorkerAddress);
  return value * 1.0f;
}

float getCurrent_mA(uint32_t ui32WorkerAddress) {
  float valueDec = getCurrent_raw(ui32WorkerAddress);
  valueDec /= ina219_currentDivider_mA;
  return valueDec;
  // return getCurrent_raw(ui32WorkerAddress) / ina219_currentDivider_mA;
}

float getPower_mW(uint32_t ui32WorkerAddress) {
  uint16_t value = getPower_raw(ui32WorkerAddress);
  return value * ina219_powerMultiplier_mW;
  // return getPower_raw(ui32WorkerAddress) * ina219_powerMultiplier_mW;
}