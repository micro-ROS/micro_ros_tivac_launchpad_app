#ifndef ros_monitor_h
#define ros_monitor_h

#ifdef __cplusplus
extern "C"
{
#endif

#include "ina219.h"
#include "average.h"

bool drv_flt_left = false;
bool drv_flt_right = false;

uint32_t fifo_left[4];
uint32_t fifo_right[4];

uint16_t cs_left = 0;
uint16_t cs_right = 0;

float current_left = 0.0f;
float current_right = 0.0f;

float bus_voltage = 0.0f;
float bus_current = 0.0f;

void init_monitor() {

    // init ina219
    setCalibration_32V_2A(INA219_ADDRESS);

    // enable adc0
    SysCtlPeripheralEnable(SYSCTL_PERIPH_ADC0);

    // PE0, PE1
    GPIOPinTypeADC(GPIO_PORTE_BASE, GPIO_PIN_0 | GPIO_PIN_1);

    // configure two sequences
    ADCSequenceConfigure(ADC0_BASE, 1, ADC_TRIGGER_PROCESSOR, 0);
    ADCSequenceConfigure(ADC0_BASE, 2, ADC_TRIGGER_PROCESSOR, 1);

    // sequence 1 step configuration
    ADCSequenceStepConfigure(ADC0_BASE, 1, 0, ADC_CTL_CH2);
    ADCSequenceStepConfigure(ADC0_BASE, 1, 1, ADC_CTL_CH2);
    ADCSequenceStepConfigure(ADC0_BASE, 1, 2, ADC_CTL_CH2);
    ADCSequenceStepConfigure(ADC0_BASE, 1, 3, ADC_CTL_CH2 | ADC_CTL_IE | ADC_CTL_END);

    // sequence 2 step configuration
    ADCSequenceStepConfigure(ADC0_BASE, 2, 0, ADC_CTL_CH3);
    ADCSequenceStepConfigure(ADC0_BASE, 2, 1, ADC_CTL_CH3);
    ADCSequenceStepConfigure(ADC0_BASE, 2, 2, ADC_CTL_CH3);
    ADCSequenceStepConfigure(ADC0_BASE, 2, 3, ADC_CTL_CH3 | ADC_CTL_IE | ADC_CTL_END);

    // enable sequences
    ADCSequenceEnable(ADC0_BASE, 1);
    ADCSequenceEnable(ADC0_BASE, 2);

    // clear interrupt
    ADCIntClear(ADC0_BASE, 1);
    ADCIntClear(ADC0_BASE, 2);

    // PA6, PA7 : drv_flt_left, drv_flt_right
    GPIOPinTypeGPIOInput(GPIO_PORTA_BASE, GPIO_PIN_6);
    GPIOPinTypeGPIOInput(GPIO_PORTA_BASE, GPIO_PIN_7);

    // init arrays for rolling average
    init_average();

}

void update_monitor() {

    // trigger adc, wait for result, clear int, and get data to fifo
    ADCProcessorTrigger(ADC0_BASE, 1);
    while(!ADCIntStatus(ADC0_BASE, 1, false)) { }
    ADCIntClear(ADC0_BASE, 1);
    ADCSequenceDataGet(ADC0_BASE, 1, fifo_right);

    ADCProcessorTrigger(ADC0_BASE, 2);
    while(!ADCIntStatus(ADC0_BASE, 2, false)) { }
    ADCIntClear(ADC0_BASE, 2);
    ADCSequenceDataGet(ADC0_BASE, 2, fifo_left);

    // these are voltage readings from driver, and just average is taken here.
    cs_left =  (fifo_left[0] + fifo_left[1] + fifo_left[2] + fifo_left[3]) / 4;
    cs_right = (fifo_right[0] + fifo_right[1] + fifo_right[2] + fifo_right[3]) / 4;

    // rolling average for cs_left and cs_right - 0.5V per amp converted
    current_left = add_cs_left(cs_left) * (3.3f / 4095) * (1.0f / 0.5f);
    current_right = add_cs_right(cs_right) * (3.3f / 4095) * (1.0f / 0.5f);

    // PA6: left, PA7: right
    drv_flt_left = !(GPIOPinRead(GPIO_PORTA_BASE, GPIO_PIN_6) && GPIO_PIN_6);
    drv_flt_right = !(GPIOPinRead(GPIO_PORTA_BASE, GPIO_PIN_7) && GPIO_PIN_7);

    // left driver fault
    if(drv_flt_left) { motor_status |= 0b01000000; /* motor:bit6 set */ } else { motor_status &= 0b10111111; } // motor:bit6 unset
    // right driver fault
    if(drv_flt_right) { motor_status |= 0b10000000; /* motor:bit7 set */ } else { motor_status &= 0b01111111; } // motor:bit7 unset

    // left motor current limit check
    if(current_left > MT_AMP_LIMIT) { power_status |= 0b00010000; }  // power:bit4 set
    // right motor current limit check
    if(current_right > MT_AMP_LIMIT) { power_status |= 0b00100000; } // power:bit5 set

}

void power_check() {
    // bit1, undervoltage protection
    if(bus_voltage < BAT_VOLTS_LOW) { power_status |= 0b00000010; }     // power:bit1 set
    // bit2, over voltage protection
    if(bus_voltage > BAT_VOLTS_HIGH) { power_status |= 0b00000100; }    // power:bit2 set
    // bit3, main power fuse
    if(bus_current > MAIN_AMP_LIMIT) { power_status |= 0b00001000; }    // power:bit3 set
}

// notice update_ina219 must be followed by power_check, except at start
void update_ina219() {
    bus_voltage = getBusVoltage_mV(INA219_ADDRESS) / 1000.0f;           // volts
    bus_current = getCurrent_mA(INA219_ADDRESS) / 1000.0f;              // amps
}

void self_power_diagnostics() {
    update_ina219();
    // bit1, hard check for bus_voltage, undervoltage protection
    if(bus_voltage < 6.0f) { power_status |= 0b00000010; }              // power:bit1 set
    // bit2, hard check for bus_voltage, overvoltage protection
    if(bus_voltage > 14.4f) { power_status |= 0b00000100; }             // power:bit2 set
}

// PE4: SCL3, PE5: SDA3
void init_I2C2(void) {
    SysCtlPeripheralEnable(SYSCTL_PERIPH_I2C2);
    SysCtlPeripheralReset(SYSCTL_PERIPH_I2C2);
    //SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOE);
    GPIOPinConfigure(GPIO_PE4_I2C2SCL);
    GPIOPinConfigure(GPIO_PE5_I2C2SDA);
    GPIOPinTypeI2CSCL(GPIO_PORTE_BASE, GPIO_PIN_4);
    GPIOPinTypeI2C(GPIO_PORTE_BASE, GPIO_PIN_5);
    I2CMasterInitExpClk(I2C2_BASE, ui32SysClkFreq, true);
    HWREG(I2C2_BASE + I2C_O_FIFOCTL) = 80008000;
}

// void send_diagnostics_msg(float dt) {

//     diagnostics_msg.cs_left = current_left;
//     diagnostics_msg.cs_right = current_right;

//     diagnostics_msg.bus_voltage = bus_voltage;
//     diagnostics_msg.bus_current = bus_current;

//     diagnostics_msg.power_status = power_status;
//     diagnostics_msg.motor_status = motor_status;
//     diagnostics_msg.system_status = system_status;

//     if((power_status>1) || (system_status >= 128)) {
//         diagnostics_msg.drivers_disabled = true;
//     } else {
//         diagnostics_msg.drivers_disabled = false;
//     }

//     diagnostics_msg.dt = dt;

//     diagnostics_pub.publish(&diagnostics_msg);

// }

#ifdef __cplusplus
}
#endif

#endif