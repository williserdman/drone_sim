#ifndef LIDAR_H
#define LIDAR_H

#include <linux/types.h>
#include <linux/i2c-dev.h>
#include <sys/ioctl.h>
#include <unistd.h>
#include <fcntl.h>
#include <cstdint>
#include <cstdio>

// Registers
#define LL_ACQ_CMD 0x00
#define LL_STATUS 0x01
#define LL_SIG_CNT_VAL 0x02
#define LL_ACQ_CONFIG 0x04
#define LL_DISTANCE 0x0f
#define LL_REF_CNT_VAL 0x12
#define LL_THRESH_BYPASS 0x1c
#define LL_ADDR_DEFAULT 0x62

class Lidar {
public:
    // I2C bus through path /dev/i2c-1 
    Lidar(const char* i2c_bus = "/dev/i2c-1", uint8_t address = LL_ADDR_DEFAULT);
    ~Lidar();
    bool isConnected() const;
    void configure(uint8_t configMode = 0);
    int readDistance();

private:
    int i2c_fd;
    bool connected;
    uint8_t address;

    void writeReg(uint8_t reg, uint8_t value);
    void readRegs(uint8_t reg, uint8_t *dest, uint8_t count);
    void wait();
    uint8_t getBusyBit();
};

#endif