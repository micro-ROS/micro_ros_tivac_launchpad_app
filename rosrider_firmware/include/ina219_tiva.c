#include <stdint.h>
#include <stdbool.h>
#include <stdio.h>

#include "inc/hw_i2c.h"
#include "driverlib/i2c.h"
#include "inc/hw_memmap.h"
#include "ina219_proc.h"

void I2CReceive(uint32_t ui32WorkerAddress, uint8_t ui32WorkerRegister, uint8_t *pReceiveData, uint8_t ui8NumBytes) {

    // specify that we are writing (a register address) to the worker device
    I2CMasterSlaveAddrSet(I2C2_BASE, ui32WorkerAddress, false);

    // specify register to be read
    I2CMasterDataPut(I2C2_BASE, ui32WorkerRegister);

    // send control byte and register address byte to worker device
    I2CMasterControl(I2C2_BASE, I2C_MASTER_CMD_BURST_SEND_START);

    // wait for MCU to finish transaction
    while(I2CMasterBusy(I2C2_BASE));

    // specify that we are going to read from worker device
    I2CMasterSlaveAddrSet(I2C2_BASE, ui32WorkerAddress, true);

    // if there is only one argument, we only need to use the single send I2C function
    if(1 == ui8NumBytes) {

        // send control byte and read from the register we specified
        I2CMasterControl(I2C2_BASE, I2C_MASTER_CMD_SINGLE_RECEIVE);

        // wait for MCU to finish transaction
        while(I2CMasterBusy(I2C2_BASE));

        // return data pulled from the specified register
        pReceiveData[0] = I2CMasterDataGet(I2C2_BASE);

    } else {

        // initiate send of data from the MCU
        I2CMasterControl(I2C2_BASE, I2C_MASTER_CMD_BURST_RECEIVE_START);

        // wait until MCU is done transferring
        while(I2CMasterBusy(I2C2_BASE));

        // return data pulled from the specified register
        pReceiveData[0] = I2CMasterDataGet(I2C2_BASE);

        uint8_t ui8Counter;
        for(ui8Counter = 1; ui8Counter < (ui8NumBytes - 1); ui8Counter++) {

            // initiate send of data from the MCU
            I2CMasterControl(I2C2_BASE, I2C_MASTER_CMD_BURST_RECEIVE_CONT);

            // wait until MCU is done transferring.
            while(I2CMasterBusy(I2C2_BASE));

            // return data pulled from the specified register
            pReceiveData[ui8Counter] = I2CMasterDataGet(I2C2_BASE);
        }

        // initiate send of data from the MCU
        I2CMasterControl(I2C2_BASE, I2C_MASTER_CMD_BURST_RECEIVE_FINISH);

        // wait until MCU is done transferring.
        while(I2CMasterBusy(I2C2_BASE));

        // return data pulled from the specified register
        pReceiveData[ui8Counter] = I2CMasterDataGet(I2C2_BASE);

    }
}

// sends an I2C command to the specified worker
void I2CSend(uint32_t ui32WorkerAddress, uint8_t ui32WorkerRegister, uint8_t *pTransmitData, uint8_t ui8NumBytes) {

    // tell the master module what address it will place on the bus when communicating with the worker
    I2CMasterSlaveAddrSet(I2C2_BASE, ui32WorkerAddress, false);

    // put the register to be sent into FIFO
    I2CMasterDataPut(I2C2_BASE, ui32WorkerRegister);

    // initiate send of data from the MCU
    I2CMasterControl(I2C2_BASE, I2C_MASTER_CMD_BURST_SEND_START);

    // wait until MCU is done transferring
    while(I2CMasterBusy(I2C2_BASE));

    uint8_t ui8Counter;
    for(ui8Counter = 0; ui8Counter <= (ui8NumBytes - 2); ui8Counter++) {

        // put next piece of data into I2C FIFO
        I2CMasterDataPut(I2C2_BASE, pTransmitData[ui8Counter]);

        // send next data that was just placed into FIFO
        I2CMasterControl(I2C2_BASE, I2C_MASTER_CMD_BURST_SEND_CONT);

        // wait until MCU is done transferring.
        while(I2CMasterBusy(I2C2_BASE));
    }

    // put next piece of data into I2C FIFO
    I2CMasterDataPut(I2C2_BASE, pTransmitData[ui8NumBytes - 1]);

    // send next data that was just placed into FIFO
    I2CMasterControl(I2C2_BASE, I2C_MASTER_CMD_BURST_SEND_FINISH);

    // wait until MCU is done transferring.
    while(I2CMasterBusy(I2C2_BASE));
}

// send single byte to worker address
void I2CSingleByteSend(uint8_t ui32WorkerAddress, uint8_t byte) {
    I2CMasterSlaveAddrSet(I2C2_BASE, ui32WorkerAddress, false);
    I2CMasterDataPut(I2C2_BASE, byte);
    I2CMasterControl(I2C2_BASE, I2C_MASTER_CMD_SINGLE_SEND);
    while(I2CMasterBusy(I2C2_BASE));
}
