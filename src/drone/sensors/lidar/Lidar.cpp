/*
Donovan Crowley for vtol
dxc825@case.edu
Feb 17, 2026
*/

#include "Lidar.h"
#include <iostream>

Lidar::Lidar(const char* device) {
    /*
    Constructor: opens I2C bus path and sets address
    @param{device} string of I2C bus path
    */
    if ((file_i2c = open(device, O_RDWR)) < 0) {
        printf("Failed to open the i2c bus: %s\n", device);
        connected = false;
    } else{
        connected = true;
    }
}

Lidar::~Lidar() {
    // Deconstructor: closes bus
    if (connected && file_i2c >= 0) {
        close(file_i2c);
    }
}

bool Lidar::isConnected() const {
    /*
    Ensure Lidar was initialized on I2C bus
    @return: True when connection established; False otherwise
    */    
    return connected;
}

int Lidar::getDistance(__u8 address) {
    /*
    Trigger laser, wait, and read distance measurement
    @param{address} Lidar I2C address
    @return: distance in centimeters; -1 if not connected
    */
    if(!connected) return -1;

    takeRange(address);
    waitForBusy(address);
    return readDistance(address);
}

__s32 Lidar::i2c_connect(__u8 address) {
    /*
    Connects I2C slave address
    @param{address} I2C address of Lidar sensor
    @returns: 0 with successful connection, -1 if ioctl call fails
    */
    if (ioctl(file_i2c, I2C_SLAVE, address) < 0)
    {
        printf("Failed to acquire bus access and/or talk to slave.\n");
        return -1;
    }
    return 0;
}

void Lidar::configure(__u8 configuration, __u8 address) {
    /*
    Predefined configuration settings for the Lidar v3 Lite
    @param{configuration} configuration mode from 0 (default) to 5
    @param{address} Lidar I2C address
    */
    __u8 sigCountMax;
    __u8 acqConfigReg;
    __u8 refCountMax;
    __u8 thresholdBypass;

    switch (configuration)
    {
        case 1: // Short range, high speed
            sigCountMax = 0x1d;
            acqConfigReg = 0x08;
            refCountMax = 0x03;
            thresholdBypass = 0x00;
            break;
        case 2: // Default range, higher speed short range
            sigCountMax = 0x80;
            acqConfigReg = 0x00;
            refCountMax = 0x03;
            thresholdBypass = 0x00;
            break;
        case 3: // Maximum range
            sigCountMax = 0xff;
            acqConfigReg = 0x08;
            refCountMax = 0x05;
            thresholdBypass = 0x00;
            break;
        case 4: // High sensitivity detection, high erroneous measurements
            sigCountMax = 0x80;
            acqConfigReg = 0x08;
            refCountMax = 0x05;
            thresholdBypass = 0x80;
            break;
        case 5: // Low sensitivity detection, low erroneous measurements
            sigCountMax = 0x80;
            acqConfigReg = 0x08;
            refCountMax = 0x05;
            thresholdBypass = 0xb0;
            break;
        case 6: // Short range, high speed, higher error
            sigCountMax = 0x04;
            acqConfigReg = 0x01; // turn off short_sig, mode pin = status output mode
            refCountMax = 0x03;
            thresholdBypass = 0x00;
            break;
        default: // Default mode, balanced performance
            sigCountMax = 0x80;
            acqConfigReg = 0x08;
            refCountMax = 0x05;
            thresholdBypass = 0x00;
            break;
    }

    i2cWrite(LLv3_SIG_CNT_VAL, &sigCountMax, 1, address);
    i2cWrite(LLv3_ACQ_CONFIG, &acqConfigReg, 1, address);
    i2cWrite(LLv3_REF_CNT_VAL, &refCountMax, 1, address);
    i2cWrite(LLv3_THRESH_BYPASS, &thresholdBypass, 1, address);
}

void Lidar::setI2Caddr(__u8 newAddress, __u8 disableDefault, __u8 address) {
    /*
    Changes I2C address of Lidar
    @param{newAddress} new I2C address to assign
    @param{disableDefault} set to >0 to disable default 0x62 address, or 0 for active
    @param{address} current I2C address
    */
    __u8 dataBytes[2];

    i2cRead((LLv3_UNIT_ID_HIGH | 0x80), dataBytes, 2, address);
    i2cWrite(LLv3_I2C_ID_HIGH, dataBytes, 2, address);

    dataBytes[0] = newAddress;
    i2cWrite(LLv3_I2C_SEC_ADR, dataBytes, 1, address);

    dataBytes[0] = 0;
    i2cWrite(LLv3_I2C_CONFIG, dataBytes, 1, address);

    if (disableDefault)
    {
        dataBytes[0] = (1 << 3);
        i2cWrite(LLv3_I2C_CONFIG, dataBytes, 1, newAddress);
    }
}

void Lidar::takeRange(__u8 address) {
    /*
    Trigger laser from sensor
    @param{address} Lidar I2C address
    */
    __u8 commandByte = 0x04;
    i2cWrite(LLv3_ACQ_CMD, &commandByte, 1, address);
}

void Lidar::waitForBusy(__u8 address) {
    /*
    Disables distance reading until Lidar is finished with current measurement 
    @param{address} Lidar I2C address
    */
    __u8 busyFlag;
    do{
        busyFlag = getBusyFlag(address);
    } 
    while (busyFlag);
}

__u8 Lidar::getBusyFlag(__u8 address) {
    /*
    Read register to see if the sensor is busy
    @param{address} Lidar I2C address
    @return: 1 if busy, 0 if idle
    */
    __u8 statusByte = 0;
    i2cRead(LLv3_STATUS, &statusByte, 1, address);
    return (statusByte & 0x01);
}

__u16 Lidar::readDistance(__u8 address) {
    /*
    Reads 2-byte distance register from sensor
    @param{address} Lidar I2C address
    @return: Unsigned 16-bit integer of distance in meters
    */
    __u8 distBytes[2] = {0};
    i2cRead((LLv3_DISTANCE | 0x80), distBytes, 2, address);
    int distCm = ((distBytes[0] << 8) | distBytes[1]);
    float distance = distCm / 100.0f;
    return distance;
}

__s32 Lidar::i2cWrite(__u8 regAddr, __u8 *dataBytes, __u8 numBytes, __u8 address) {
    /*
    Write array of bytes to I2C device
    @param{regAddr} starting register address to write to
    @param{dataBytes} pointer to array of bytes to write
    @param{numBytes} number of bytes to write
    @param{address} Lidar I2C address
    @return: >= 0 for successful write operation; negative on error
    */
    __u8 buffer[2];
    __s32 result = 0;

    i2c_connect(address);

    for (__u8 i = 0 ; i < numBytes ; i++)
    {
        buffer[0] = regAddr + i;
        buffer[1] = dataBytes[i];
        result |= write(file_i2c, buffer, 2);
    }

    return result;
}

__s32 Lidar::i2cRead(__u8 regAddr, __u8 *dataBytes, __u8 numBytes, __u8 address) {
    /*
    Reads array of bytes from I2C device
    @param{regAddr} starting register address to read to
    @param{dataBytes} pointer to array of where to store read bytes
    @param{numBytes} number of bytes to read
    @param{address} Lidar I2C address
    @return: >= 0 for successful read operation; negative on error
    */
    __u8 buffer;
    i2c_connect(address);
    buffer = regAddr;
    write(file_i2c, &buffer, 1);
    return read(file_i2c, dataBytes, numBytes);
}

void Lidar::correlationRecordRead(__s16 *correlationArray, __u16 numberOfReadings, __u8 address) {
    /*
    Read correlation record (useful?)
    @param{correlationArray} pointer to signed 16-bit array to store correlation values
    @param{numberOfReadings} number of readings to take (default 256)
    @param{address} Lidar I2C address
    */
    __u8 dataBytes[2];
    __s16 correlationValue;
    __u8 * correlationValuePtr = (__u8 *) &correlationValue;

    dataBytes[0] = 0xc0;
    i2cWrite(LLv3_ACQ_SETTINGS, dataBytes, 1, address);

    dataBytes[0] = 0x07;
    i2cWrite(LLv3_COMMAND, dataBytes, 1, address);

    for (__u16 i = 0 ; i < numberOfReadings ; i++)
    {
        i2cRead((LLv3_CORR_DATA | 0x80), dataBytes, 2, address);
        correlationValuePtr[0] = dataBytes[0];

        if (dataBytes[1])
            correlationValuePtr[1] = 0xff; 
        else
            correlationValuePtr[1] = 0x00;

        correlationArray[i] = correlationValue;
    }

    dataBytes[0] = 0;
    i2cWrite(LLv3_COMMAND, dataBytes, 1, address);
}