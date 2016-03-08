
#pragma once

#define MPU6000_CONFIG		    	0x1A

#define BITS_DLPF_CFG_256HZ         0x00
#define BITS_DLPF_CFG_188HZ         0x01
#define BITS_DLPF_CFG_98HZ          0x02
#define BITS_DLPF_CFG_42HZ          0x03
#define BITS_DLPF_CFG_20HZ          0x04
#define BITS_DLPF_CFG_10HZ          0x05
#define BITS_DLPF_CFG_5HZ           0x06
#define BITS_DLPF_CFG_2100HZ_NOLPF  0x07

#define GYRO_SCALE_FACTOR  0.00053292f  // (4/131) * pi/180   (32.75 LSB = 1 DPS)

#define MPU6000_WHO_AM_I_CONST              (0x68)

// RF = Register Flag
#define MPU_RF_DATA_RDY_EN (1 << 0)

void mpu6000ResetGyro(void);

bool mpu6000SpiDetect(void);

bool mpu6000SpiAccDetect(acc_t *acc);
bool mpu6000SpiGyroDetect(gyro_t *gyro);

bool mpu6000WriteRegister(uint8_t reg, uint8_t data);
bool verifympu6000WriteRegister(uint8_t reg, uint8_t data);
bool mpu6000ReadRegister(uint8_t reg, uint8_t length, uint8_t *data);
bool mpu6000SlowReadRegister(uint8_t reg, uint8_t length, uint8_t *data);
