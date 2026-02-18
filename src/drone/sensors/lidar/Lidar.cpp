/*
Donovan Crowley for vtol
dxc825@case.edu
Feb 17, 2026
*/

#include "Lidar.h"
#include <iostream>

Lidar::Lidar(const char* i2c_bus, uint8_t addr) : address(addr), connected(false) {
    /*
    Constructor: Opens I2C bus path and sets address
    @param: string of I2C bus path
    @param: 7 bit Lidar address
    */
    if ((i2c_fd = open(i2c_bus, O_RDWR)) < 0) {
        perror("Failed to open the i2c bus");
        return;
    }

    // Slave management
    if (ioctl(i2c_fd, I2C_SLAVE, address) < 0) {
        perror("Failed to acquire bus access");
        close(i2c_fd);
        return;
    }
    
    connected = true;
}

Lidar::~Lidar() {
    // Deconstructor: Closes bus
    if (i2c_fd >= 0) {
        close(i2c_fd);
    }
}

bool Lidar::isConnected() const {
    /*
    Ensure Lidar was initialized on I2C bus
    @return: True when connection established; False otherwise
    */    
    return connected;
}

void Lidar::writeReg(uint8_t reg, uint8_t value) {
    /*
    Write a byte to a register
    @param: register address
    @param: byte of data to write
    */
    uint8_t buffer[2] = {reg, value};
}

void Lidar::readRegs(uint8_t reg, uint8_t *dest, uint8_t bytes) {
    /*
    Read a byte to a register
    @param: register address
    @param: pointer to the array where the byte data will be read
    @param: number of bytes to read
    */
    if (write(i2c_fd, &reg, 1) != 1) return;
    read(i2c_fd, dest, bytes);
}

uint8_t Lidar::getBusyBit() {
    /*
    Read register to see if the sensor is busy
    @return: 1 if busy bit, 0 if ready to accept new data
    */
    uint8_t status = 0;
    readRegs(LL_STATUS, &status, 1);
    return status & 0x01; 
}

void Lidar::wait() {
    /*
    Disables distance reading until Lidar is finished with current measurement 
    */
    int timeout = 1000; 
    while (getBusyBit() && timeout > 0) {
        timeout--;
        usleep(100); // Add small delay to relieve CPU
    }
}

void Lidar::configure(uint8_t configMode) {
    /*
    Predefined configuration settings for the Lidar v3 Lite
    @param: configuration mode from 0 (or default) to 5
    */

    uint8_t sigCount, acqConfig, refCount, thresh;

    switch (configMode) {
        case 1: // Short range, high speed
            sigCount = 0x1d; 
            acqConfig = 0x08; 
            refCount = 0x03; 
            thresh = 0x00; 
            break;
        case 2: // Default range, higher speed
            sigCount = 0x80; 
            acqConfig = 0x00; 
            refCount = 0x03; 
            thresh = 0x00; 
            break;
        case 3: // Max range
            sigCount = 0xff; 
            acqConfig = 0x08; 
            refCount = 0x05; 
            thresh = 0x00; 
            break;
        case 4: // High sensitivity, high error
            sigCount = 0x80; 
            acqConfig = 0x08; 
            refCount = 0x05; 
            thresh = 0x80; 
            break;
        case 5: // Low sensitivity, low error
            sigCount = 0x80; 
            acqConfig = 0x08; 
            refCount = 0x05; 
            thresh = 0xb0; 
            break;
        default: // Default mode, balanced performance
            sigCount = 0x80; 
            acqConfig = 0x08; 
            refCount = 0x05; 
            thresh = 0x00; 
            break;
    }

    writeReg(LL_SIG_CNT_VAL, sigCount);
    writeReg(LL_ACQ_CONFIG, acqConfig);
    writeReg(LL_REF_CNT_VAL, refCount);
    writeReg(LL_THRESH_BYPASS, thresh);
}

int Lidar::readDistance() {
    /*
    Emits laser, delays, and calculates distance
    @param: distance value in meters 
    */

    if (!connected) return -1;

    // Trigger acquisition (write 0x04 to register 0x00)
    writeReg(LL_ACQ_CMD, 0x04);

    // Wait for device to finish
    wait();

    // Read 2 bytes from 0x8f (0x0f + 0x80 for auto-increment)
    uint8_t distBytes[2] = {0};
    readRegs(LL_DISTANCE | 0x80, distBytes, 2);

    int distCm = (distBytes[0] << 8) | distBytes[1];
    float distM = distCm / 100.0f;

    return distM;
}