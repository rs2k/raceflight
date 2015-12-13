/*
 * This file is part of Cleanflight.
 *
 * Cleanflight is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * Cleanflight is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with Cleanflight.  If not, see <http://www.gnu.org/licenses/>.
 */

#include <stdbool.h>
#include <stdint.h>

#include <math.h>

#include "build_config.h"

#include "platform.h"

#include "common/axis.h"
#include "common/maths.h"

#include "system.h"
#include "gpio.h"
#include "exti.h"
#include "bus_i2c.h"

#include "sensors/boardalignment.h"
#include "sensors/sensors.h"

#include "sensor.h"
#include "compass.h"

#include "accgyro_mpu.h"
#include "compass_ak8963.h"

// This sensor is available in MPU-9250.

// AK8963, mag sensor address
#define AK8963_MAG_I2C_ADDRESS      0x0C
#define AK8963_Device_ID            0x48

// Registers
#define AK8963_MAG_REG_WHO_AM_I     0x00
#define AK8963_MAG_REG_INFO         0x01
#define AK8963_MAG_REG_STATUS1      0x02
#define AK8963_MAG_REG_HXL          0x03
#define AK8963_MAG_REG_HXH          0x04
#define AK8963_MAG_REG_HYL          0x05
#define AK8963_MAG_REG_HYH          0x06
#define AK8963_MAG_REG_HZL          0x07
#define AK8963_MAG_REG_HZH          0x08
#define AK8963_MAG_REG_STATUS2      0x09
#define AK8963_MAG_REG_CNTL         0x0a
#define AK8963_MAG_REG_ASCT         0x0c // self test

#define AK8963A_ASAX                0x10 // Fuse ROM x-axis sensitivity adjustment value
#define AK8963A_ASAY                0x11 // Fuse ROM y-axis sensitivity adjustment value
#define AK8963A_ASAZ                0x12 // Fuse ROM z-axis sensitivity adjustment value

#define READ_FLAG                   0x80

#define BIT_STATUS1_REG_DATA_READY              (1 << 0)
#define BIT_STATUS2_REG_DATA_ERROR              (1 << 2)
#define BIT_STATUS2_REG_MAG_SENSOR_OVERFLOW     (1 << 3)

typedef bool (*ak8963ReadRegisterFunc)(uint8_t addr_, uint8_t reg_, uint8_t len_, uint8_t *buf);
typedef bool (*ak8963WriteRegisterFunc)(uint8_t addr_, uint8_t reg_, uint8_t data);

typedef struct ak8963Configuration_s {
    ak8963ReadRegisterFunc read;
    ak8963WriteRegisterFunc write;
} ak8963Configuration_t;

ak8963Configuration_t ak8963config;

#ifdef USE_SPI
bool ak8963SPIRead(uint8_t addr_, uint8_t reg_, uint8_t len_, uint8_t *buf)
{
    mpuConfiguration.write(MPU_RA_I2C_SLV0_ADDR, addr_ | READ_FLAG);         // set I2C slave address for read
    mpuConfiguration.write(MPU_RA_I2C_SLV0_REG, reg_);                       // set I2C slave register
    mpuConfiguration.write(MPU_RA_I2C_SLV0_CTRL, len_ | 0x80);               // read number of bytes
    delayMicroseconds(1);
    return mpuConfiguration.read(MPU_RA_EXT_SENS_DATA_00, len_, buf);        // read I2C
}

bool ak8963SPIWrite(uint8_t addr_, uint8_t reg_, uint8_t data)
{
    mpuConfiguration.write(MPU_RA_I2C_SLV0_ADDR, addr_ | READ_FLAG);         // set I2C slave address for read
    mpuConfiguration.write(MPU_RA_I2C_SLV0_REG, reg_);                       // set I2C slave register
    mpuConfiguration.write(MPU_RA_I2C_SLV0_DO, data);                        // set I2C salve value
    return mpuConfiguration.write(MPU_RA_I2C_SLV0_CTRL, 0x81);               // write 1 byte
}
#endif

bool ak8963Detect(mag_t *mag)
{
    bool ack = false;
    uint8_t sig = 0;

#ifdef USE_I2C
    // check for AK8963 on I2C bus
    ack = i2cRead(AK8963_MAG_I2C_ADDRESS, AK8963_MAG_REG_WHO_AM_I, 1, &sig);
    if (ack && sig == AK8963_Device_ID) // 0x48 / 01001000 / 'H'
    {
        ak8963config.read = i2cRead;
        ak8963config.write = i2cWrite;
        mag->init = ak8963Init;
        mag->read = ak8963Read;

        return true;
    }
#endif

#ifdef USE_SPI
    // check for AK8963 on I2C master via SPI bus (as part of MPU9250)
    ack = ak8963SPIRead(AK8963_MAG_I2C_ADDRESS, AK8963_MAG_REG_WHO_AM_I, 1, &sig);
    if (ack && sig == AK8963_Device_ID) // 0x48 / 01001000 / 'H'
    {
        ak8963config.read = ak8963SPIRead;
        ak8963config.write = ak8963SPIWrite;
        mag->init = ak8963Init;
        mag->read = ak8963Read;

        return true;
    }
#endif
    return false;
}

void ak8963Init()
{
    bool ack;
    uint8_t buffer[3];
    uint8_t status;

    UNUSED(ack);

    ack = ak8963config.write(AK8963_MAG_I2C_ADDRESS, AK8963_MAG_REG_CNTL, 0x00); // power down before entering fuse mode
    delay(20);

    ack = ak8963config.write(AK8963_MAG_I2C_ADDRESS, AK8963_MAG_REG_CNTL, 0x0F); // Enter Fuse ROM access mode
    delay(10);

    ack = ak8963config.read(AK8963_MAG_I2C_ADDRESS, AK8963A_ASAX, 3, &buffer[0]); // Read the x-, y-, and z-axis calibration values
    delay(10);

    ack = ak8963config.write(AK8963_MAG_I2C_ADDRESS, AK8963_MAG_REG_CNTL, 0x00); // power down after reading.
    delay(10);

    // Clear status registers
    ack = ak8963config.read(AK8963_MAG_I2C_ADDRESS, AK8963_MAG_REG_STATUS1, 1, &status);
    ack = ak8963config.read(AK8963_MAG_I2C_ADDRESS, AK8963_MAG_REG_STATUS2, 1, &status);

    // Trigger first measurement
    ack = ak8963config.write(AK8963_MAG_I2C_ADDRESS, AK8963_MAG_REG_CNTL, 0x01);
}

bool ak8963Read(int16_t *magData)
{
    bool ack;
    UNUSED(ack);
    uint8_t status;
    uint8_t buf[6];

    ack = ak8963config.read(AK8963_MAG_I2C_ADDRESS, AK8963_MAG_REG_STATUS1, 1, &status);
    if (!ack || (status & BIT_STATUS1_REG_DATA_READY) == 0) {
        return false;
    }

    ack = ak8963config.read(AK8963_MAG_I2C_ADDRESS, AK8963_MAG_REG_HXL, 6, buf); // read from AK8963_MAG_REG_HXL to AK8963_MAG_REG_HZH
    if (!ack) {
        return false;
    }

    ack = ak8963config.read(AK8963_MAG_I2C_ADDRESS, AK8963_MAG_REG_STATUS2, 1, &status);
    if (!ack) {
        return false;
    }

    if (status & BIT_STATUS2_REG_DATA_ERROR) {
        return false;
    }

    if (status & BIT_STATUS2_REG_MAG_SENSOR_OVERFLOW) {
        return false;
    }

    magData[X] = -(int16_t)(buf[1] << 8 | buf[0]) * 4;
    magData[Y] = -(int16_t)(buf[3] << 8 | buf[2]) * 4;
    magData[Z] = -(int16_t)(buf[5] << 8 | buf[4]) * 4;

    ack = ak8963config.write(AK8963_MAG_I2C_ADDRESS, AK8963_MAG_REG_CNTL, 0x01); // start reading again
    return true;
}
