/*
Donovan Crowley for vtol
dxc825@case.edu
Feb 17, 2026
*/

#include <linux/types.h>
#include <linux/i2c-dev.h>
#include <sys/ioctl.h>
#include <fcntl.h>
#include <unistd.h>
#include <stdio.h>
#include "Lidar.h"

Lidar::Lidar(char* device){
    filename = device;
}

Lidar::~Lidar() {
    // Deconstructor: closes bus
    if (file_i2c >= 0) {
        close(file_i2c);
    }
}

__s32 Lidar::i2c_init (void)
{
    if ((file_i2c = open(filename, O_RDWR)) < 0)
    {
        // ERROR HANDLING
        printf("Failed to open the i2c bus");
        return -1;
    }
    else
    {
        return 0;
    }
}

__s32 Lidar::i2c_connect (__u8 lidarliteAddress)
{
    if (ioctl(file_i2c, I2C_SLAVE, lidarliteAddress) < 0)
    {
        printf("Failed to acquire bus access and/or talk to slave.\n");
        //ERROR HANDLING
        return -1;
    }
    else
    {
        return 0;
    }
}

void Lidar::configure(__u8 configuration, __u8 lidarliteAddress)
{
    __u8 sigCountMax;
    __u8 acqConfigReg;
    __u8 refCountMax;
    __u8 thresholdBypass;

    switch (configuration)
    {
        case 0: // Default mode, balanced performance
            sigCountMax = 0x80; // Default
            acqConfigReg = 0x08; // Default
            refCountMax = 0x05; // Default
            thresholdBypass = 0x00; // Default
            break;

        case 1: // Short range, high speed
            sigCountMax = 0x1d;
            acqConfigReg = 0x08; // Default
            refCountMax = 0x03;
            thresholdBypass = 0x00; // Default
            break;

        case 2: // Default range, higher speed short range
            sigCountMax = 0x80; // Default
            acqConfigReg = 0x00;
            refCountMax = 0x03;
            thresholdBypass = 0x00; // Default
            break;

        case 3: // Maximum range
            sigCountMax = 0xff;
            acqConfigReg = 0x08; // Default
            refCountMax = 0x05; // Default
            thresholdBypass = 0x00; // Default
            break;

        case 4: // High sensitivity detection, high erroneous measurements
            sigCountMax = 0x80; // Default
            acqConfigReg = 0x08; // Default
            refCountMax = 0x05; // Default
            thresholdBypass = 0x80;
            break;

        case 5: // Low sensitivity detection, low erroneous measurements
            sigCountMax = 0x80; // Default
            acqConfigReg = 0x08; // Default
            refCountMax = 0x05; // Default
            thresholdBypass = 0xb0;
            break;

        case 6: // Short range, high speed, higher error
            sigCountMax = 0x04;
            acqConfigReg = 0x01; // turn off short_sig, mode pin = status output mode
            refCountMax = 0x03;
            thresholdBypass = 0x00;
            break;

        default: // Default mode, balanced performance - same as configure(0)
            sigCountMax = 0x80; // Default
            acqConfigReg = 0x08; // Default
            refCountMax = 0x05; // Default
            thresholdBypass = 0x00; // Default
            break;
    }

    i2cWrite(LLv3_SIG_CNT_VAL, &sigCountMax, 1, lidarliteAddress);
    i2cWrite(LLv3_ACQ_CONFIG, &acqConfigReg, 1, lidarliteAddress);
    i2cWrite(LLv3_REF_CNT_VAL, &refCountMax, 1, lidarliteAddress);
    i2cWrite(LLv3_THRESH_BYPASS, &thresholdBypass, 1, lidarliteAddress);
}

void Lidar::setI2Caddr(__u8 newAddress, __u8 disableDefault, __u8 lidarliteAddress)
{
    __u8 dataBytes[2];

    // Read UNIT_ID serial number bytes and write them into I2C_ID byte locations
    i2cRead ((LLv3_UNIT_ID_HIGH | 0x80), dataBytes, 2, lidarliteAddress);
    i2cWrite(LLv3_I2C_ID_HIGH, dataBytes, 2, lidarliteAddress);

    // Write the new I2C device address to registers
    dataBytes[0] = newAddress;
    i2cWrite(LLv3_I2C_SEC_ADR, dataBytes, 1, lidarliteAddress);

    // Enable the new I2C device address using the default I2C device address
    dataBytes[0] = 0;
    i2cWrite(LLv3_I2C_CONFIG, dataBytes, 1, lidarliteAddress);

    // If desired, disable default I2C device address (using the new I2C device address)
    if (disableDefault)
    {
        dataBytes[0] = (1 << 3); // set bit to disable default address
        i2cWrite(LLv3_I2C_CONFIG, dataBytes, 1, newAddress);
    }
}

void Lidar::takeRange(__u8 lidarliteAddress)
{
    __u8 commandByte = 0x04;

    i2cWrite(LLv3_ACQ_CMD, &commandByte, 1, lidarliteAddress);
}

void Lidar::waitForBusy(__u8 lidarliteAddress)
{
    __u8  busyFlag;

    do  // Loop until device is not busy
    {
        busyFlag = getBusyFlag(lidarliteAddress);
    } while (busyFlag);
}

__u8 Lidar::getBusyFlag(__u8 lidarliteAddress)
{
    __u8  statusByte = 0;
    __u8  busyFlag; // busyFlag monitors when the device is done with a measurement

    // Read status register to check busy flag
    i2cRead(LLv3_STATUS, &statusByte, 1, lidarliteAddress);

    // STATUS bit 0 is busyFlag
    busyFlag = statusByte & 0x01;

    return busyFlag;
}

__u16 Lidar::readDistance(__u8 lidarliteAddress)
{
    __u8  distBytes[2] = {0};

    // Read two bytes from register 0x0f and 0x10 (autoincrement)
    i2cRead((LLv3_DISTANCE | 0x80), distBytes, 2, lidarliteAddress);

    // Shift high byte and OR in low byte
    return ((distBytes[0] << 8) | distBytes[1]);
}

__s32 Lidar::i2cWrite(__u8 regAddr,  __u8 * dataBytes,
                             __u8 numBytes, __u8 lidarliteAddress)
{
    __u8 buffer[2];
    __u8 i;
    __s32 result;

    i2c_connect(lidarliteAddress);

    for (i=0 ; i<numBytes ; i++)
    {
        buffer[0] = regAddr + i;
        buffer[1] = dataBytes[i];
        result   |= write(file_i2c, buffer, 2);
    }

    return result;
}

__s32 Lidar::i2cRead(__u8 regAddr,  __u8 * dataBytes,
                            __u8 numBytes, __u8 lidarliteAddress)
{
    __u8 buffer;

    i2c_connect(lidarliteAddress);

    buffer = regAddr;

    write(file_i2c, &buffer, 1);
    return read(file_i2c, dataBytes, numBytes);
}

void Lidar::correlationRecordRead(__s16 * correlationArray,
                                         __u16 numberOfReadings,
                                         __u8  lidarliteAddress)
{
    __u16  i = 0;
    __u8   dataBytes[2];
    __s16  correlationValue;
    __u8 * correlationValuePtr = (__u8 *) &correlationValue;

    //  Select memory bank
    dataBytes[0] = 0xc0;
    i2cWrite(LLv3_ACQ_SETTINGS, dataBytes, 1, lidarliteAddress);

    // Test mode enable
    dataBytes[0] = 0x07;
    i2cWrite(LLv3_COMMAND, dataBytes, 1, lidarliteAddress);

    for (i=0 ; i<numberOfReadings ; i++)
    {
        i2cRead((LLv3_CORR_DATA | 0x80), dataBytes, 2, lidarliteAddress);

        // First byte read is the magnitude of the data point
        correlationValuePtr[0] = dataBytes[0];

        // Second byte is the sign byte
        if (dataBytes[1])
            correlationValuePtr[1] = 0xff; // Artificially sign extend
        else
            correlationValuePtr[1] = 0x00;

        correlationArray[i] = correlationValue;
    }

    // Test mode disable
    dataBytes[0] = 0;
    i2cWrite(LLv3_COMMAND, dataBytes, 1, lidarliteAddress);
}