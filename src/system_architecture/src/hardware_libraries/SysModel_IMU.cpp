#include "SysModel_IMU.h"
#include <cstdint>
#include <time.h>
#include <linux/i2c-dev.h>
#include <sys/ioctl.h>
#include <fcntl.h>
#include <cstdio>
#include <stdio.h>
#include <unistd.h>
#include <stdlib.h>
#include <iostream>
#include <math.h>


SysModel_IMU::SysModel_IMU(): _i2caddr(IMU_I2C_ADDR) 
    {
	    begin();
    }

/*---------- Set the order for gyroscope and acceleration ------------------------------------------------
    //----------------------- Accelerometer --------------------------------------------
    //For enabling Accelerometer write to REG_ACCEL_CTRL(0x10) as follows:
    //By default it is in power down mode, and scale is at +/-2g.
    
    //8-Bit value legend
    //(4 bits for frequency) (2 bits for scale) (1 bit LPF filtering option) (last bit by default '0')
    
    //-------------- For Scale of +/-2g -------------------------------------
    //Low power mode
    //Power down:   0000 00 0 0 - 0x00
    //1.6Hz:        1011 00 0 0 - 0xB0
    //12.5Hz:       0001 00 0 0 - 0x10
    //26Hz:         0010 00 0 0 - 0x20
    //52Hz:         0011 00 0 0 - 0x30
    //104Hz:        0100 00 0 0 - 0x40
    //208Hz:        0101 00 0 0 - 0x50

    //High Performance mode
    //416Hz:        0110 00 0 0 - 0x60
    //833Hz:        0111 00 0 0 - 0x70
    //1.67kHz:      1000 00 0 0 - 0x80
    
    //-------------- For Scale of +/-4g -------------------------------------
    //Low power mode
    //Power down:   0000 00 0 0 - 0x00
    //1.6Hz:        1011 10 0 0 - 0xB8
    //12.5Hz:       0001 10 0 0 - 0x18
    //26Hz:         0010 10 0 0 - 0x28
    //52Hz:         0011 10 0 0 - 0x38
    //104Hz:        0100 10 0 0 - 0x48
    //208Hz:        0101 10 0 0 - 0x58

    //High Performance mode
    //416Hz:        0110 10 0 0 - 0x68
    //833Hz:        0111 10 0 0 - 0x78
    //1.67kHz:      1000 10 0 0 - 0x88

    //-------------- For Scale of +/-8g -------------------------------------
    //Low power mode
    //Power down:   0000 00 0 0 - 0x00
    //1.6Hz:        1011 11 0 0 - 0xBC
    //12.5Hz:       0001 11 0 0 - 0x1C
    //26Hz:         0010 11 0 0 - 0x2C
    //52Hz:         0011 11 0 0 - 0x3C
    //104Hz:        0100 11 0 0 - 0x4C
    //208Hz:        0101 11 0 0 - 0x5C

    //High Performance mode
    //416Hz:        0110 11 0 0 - 0x6C
    //833Hz:        0111 11 0 0 - 0x7C
    //1.67kHz:      1000 11 0 0 - 0x8C

    //-------------- For Scale of +/-16g -------------------------------------
    //Low power mode
    //Power down:   0000 00 0 0 - 0x00
    //1.6Hz:        1011 01 0 0 - 0xB4
    //12.5Hz:       0001 01 0 0 - 0x14
    //26Hz:         0010 01 0 0 - 0x24
    //52Hz:         0011 01 0 0 - 0x34
    //104Hz:        0100 01 0 0 - 0x44
    //208Hz:        0101 01 0 0 - 0x54

    //High Performance mode
    //416Hz:        0110 01 0 0 - 0x64
    //833Hz:        0111 01 0 0 - 0x74
    //1.67kHz:      1000 01 0 0 - 0x84
    //----------------------------------------------------------------------------

    //-------------------- Gryscope ---------------------------------------------
    //For enabling Gyroscope write to REG_GYRO_CTRL(0x11) as follows:
    //By default it is in 208Hz, and scale is at 500dps.
    
    //8-Bit value legend
    //(4 bits for frequency) (2 bits for scale) (1 bit LPF filtering option) (last bit by default '0')
    
    //-------------- For Scale of +/-250dps -------------------------------------
    //Low power mode
    //Power down:   0000 00 0 0 - 0x00
    //12.5Hz:       0001 00 0 0 - 0x10
    //26Hz:         0010 00 0 0 - 0x20
    //52Hz:         0011 00 0 0 - 0x30
    //104Hz:        0100 00 0 0 - 0x40
    //208Hz:        0101 00 0 0 - 0x50

    //High Performance mode
    //416Hz:        0110 00 0 0 - 0x60
    //833Hz:        0111 00 0 0 - 0x70
    //1.67kHz:      1000 00 0 0 - 0x80
    
    //-------------- For Scale of +/-500dps -------------------------------------
    //Low power mode
    //Power down:   0000 00 0 0 - 0x00
    //12.5Hz:       0001 01 0 0 - 0x14
    //26Hz:         0010 01 0 0 - 0x24
    //52Hz:         0011 01 0 0 - 0x34
    //104Hz:        0100 01 0 0 - 0x44
    //208Hz:        0101 01 0 0 - 0x54

    //High Performance mode
    //416Hz:        0110 01 0 0 - 0x64
    //833Hz:        0111 01 0 0 - 0x74
    //1.67kHz:      1000 01 0 0 - 0x84

    //-------------- For Scale of +/-1000dps -------------------------------------
    //Low power mode
    //Power down:   0000 10 0 0 - 0x00
    //12.5Hz:       0001 10 0 0 - 0x18
    //26Hz:         0010 10 0 0 - 0x28
    //52Hz:         0011 10 0 0 - 0x38
    //104Hz:        0100 10 0 0 - 0x48
    //208Hz:        0101 10 0 0 - 0x58

    //High Performance mode
    //416Hz:        0110 10 0 0 - 0x68
    //833Hz:        0111 10 0 0 - 0x78
    //1.67kHz:      1000 10 0 0 - 0x88

    //-------------- For Scale of +/-2000dps -------------------------------------
    //Low power mode
    //Power down:   0000 00 0 0 - 0x00
    //12.5Hz:       0001 11 0 0 - 0x1C
    //26Hz:         0010 11 0 0 - 0x2C
    //52Hz:         0011 11 0 0 - 0x3C
    //104Hz:        0100 11 0 0 - 0x4C
    //208Hz:        0101 11 0 0 - 0x5C

    //High Performance mode
    //416Hz:        0110 11 0 0 - 0x6C
    //833Hz:        0111 11 0 0 - 0x7C
    //1.67kHz:      1000 11 0 0 - 0x8C
//------------------------------------------------------------------------------*/

void SysModel_IMU::begin() {
    //initialize a default value for frequency and order for the gyroscope and accelerometer.
    //enable accelerometer
    uint8_t AccelOrder = 0x58; //208Hz and 4g
    i2c_write(REG_ACCEL_CTRL, AccelOrder);

    uint8_t GyroOrder = 0x54; //208Hz and 500dps
    i2c_write(REG_GYRO_CTRL, GyroOrder);

}

/*Read/Write to Registers*/
void SysModel_IMU::i2c_write(uint8_t reg_address, uint8_t val) {
    int file;
    char *filename = (char*)I2C_BUS;
    uint8_t buf[2];
	buf[0] = reg_address;
	buf[1] = val;

    if ((file = open(filename, O_RDWR)) < 0) {
        perror("Failed to open the i2c bus");
        exit(1);
    }

    if (ioctl(file, I2C_SLAVE, _i2caddr) < 0) {
        perror("Failed to acquire bus access and/or talk to slave");
        exit(1);
    }

	if (write(file, &buf, 2) != 2) {
		printf("Error, unable to write to i2c device\n");
		exit(1);
	}
    //printf("Value at reg %x should be: %x\n",buf[0], buf[1]);
    close(file);
}
uint8_t SysModel_IMU::i2c_read(uint8_t reg_address){
    int file;
    char *filename = (char*)I2C_BUS;
	uint8_t buf[1];

    if ((file = open(filename, O_RDWR)) < 0) {
        perror("Failed to open the i2c bus");
        exit(1);
    }

    if (ioctl(file, I2C_SLAVE, _i2caddr) < 0) {
        perror("Failed to acquire bus access and/or talk to slave");
        exit(1);
    }

	buf[0] = reg_address;

	if (write(file, &buf, 1) != 1) {
		printf("Error, unable to write to i2c device\n");
		exit(1);
	}

	if (read(file, &buf, 1) != 1) {
		printf("Error, unable to read from i2c device\n");
		exit(1);
	}

    usleep(100);  // 100 microseconds delay = 0.0001s
    close(file);
	return buf[0];
}

uint16_t SysModel_IMU::merge_bytes(uint8_t LSB, uint8_t MSB) 
{
    return (uint16_t)(((uint16_t)MSB << 8) | LSB);
}

int16_t SysModel_IMU::two_complement_to_int(uint8_t LSB, uint8_t MSB) 
{
    uint16_t word = merge_bytes(LSB, MSB);
    int16_t signed_int;

    // Check if the number is negative
    if (word & 0x8000) {
        // Convert from two's complement to int
        signed_int = (int16_t)(word | 0xFFFF0000);
    } else {
        signed_int = (int16_t)word;
    }

    return signed_int;
}


//Sourced below measurement functions from STM's github repo: https://github.com/STMicroelectronics/asm330lhb-pid.git
/**
  * @defgroup    ASM330LHB_Sensitivity
  * @brief       These functions convert raw-data into engineering units.
  * @{
  *
  */

float  SysModel_IMU::asm330lhb_from_fs2g_to_mg(int16_t lsb) {
  return ((float)lsb * 0.061f);
}
float  SysModel_IMU::asm330lhb_from_fs4g_to_mg(int16_t lsb) {
  return ((float)lsb * 0.122f);
}
float  SysModel_IMU::asm330lhb_from_fs8g_to_mg(int16_t lsb) {
  return ((float)lsb * 0.244f);
}
float  SysModel_IMU::asm330lhb_from_fs16g_to_mg(int16_t lsb) {
  return ((float)lsb * 0.488f);
}
float  SysModel_IMU::asm330lhb_from_fs125dps_to_mdps(int16_t lsb) {
  return ((float)lsb * 4.375f);
}
float  SysModel_IMU::asm330lhb_from_fs250dps_to_mdps(int16_t lsb) {
  return ((float)lsb * 8.75f);
}
float  SysModel_IMU::asm330lhb_from_fs500dps_to_mdps(int16_t lsb) {
  return ((float)lsb * 17.50f);
}
float  SysModel_IMU::asm330lhb_from_fs1000dps_to_mdps(int16_t lsb) {
  return ((float)lsb * 35.0f);
}
float  SysModel_IMU::asm330lhb_from_fs2000dps_to_mdps(int16_t lsb) {
  return ((float)lsb * 70.0f);
}
float  SysModel_IMU::asm330lhb_from_fs4000dps_to_mdps(int16_t lsb) {
  return ((float)lsb * 140.0f);
}
float  SysModel_IMU::asm330lhb_from_lsb_to_celsius(int16_t lsb) {
  return (((float)lsb / 256.0f) + 25.0f);
}
float  SysModel_IMU::asm330lhb_from_lsb_to_nsec(int32_t lsb) {
  return ((float)lsb * 25000.0f);
}

void  SysModel_IMU::read_and_convert_sensor_data() 
{
    //private
    accel_x_h = i2c_read(REG_ACCEL_X_OUT_H);
    accel_x_l = i2c_read(REG_ACCEL_X_OUT_L);
    accel_y_h = i2c_read(REG_ACCEL_Y_OUT_H);
    accel_y_l = i2c_read(REG_ACCEL_Y_OUT_L);
    accel_z_h = i2c_read(REG_ACCEL_Z_OUT_H);
    accel_z_l = i2c_read(REG_ACCEL_Z_OUT_L);

    gyro_x_h = i2c_read(REG_GYRO_X_OUT_H);
    gyro_x_l = i2c_read(REG_GYRO_X_OUT_L);
    gyro_y_h = i2c_read(REG_GYRO_Y_OUT_H);
    gyro_y_l = i2c_read(REG_GYRO_Y_OUT_L);
    gyro_z_h = i2c_read(REG_GYRO_Z_OUT_H);
    gyro_z_l = i2c_read(REG_GYRO_Z_OUT_L);

    temp_h = i2c_read(REG_TEMP_OUT_H);
    temp_l = i2c_read(REG_TEMP_OUT_L);

    x_accel = two_complement_to_int(accel_x_l, accel_x_h);
    y_accel = two_complement_to_int(accel_y_l, accel_y_h);
    z_accel = two_complement_to_int(accel_z_l, accel_z_h);
    x_gyro = two_complement_to_int(gyro_x_l, gyro_x_h);
    y_gyro = two_complement_to_int(gyro_y_l, gyro_y_h);
    z_gyro = two_complement_to_int(gyro_z_l, gyro_z_h);
    temp = two_complement_to_int(temp_l, temp_h);

    //public variables
    x_accel_g = asm330lhb_from_fs4g_to_mg(x_accel)/1000.0f; //divide by 1000 to go from mg to g.
    y_accel_g = asm330lhb_from_fs4g_to_mg(y_accel)/1000.0f;
    z_accel_g = asm330lhb_from_fs4g_to_mg(z_accel)/1000.0f;
    x_gyro_dps = asm330lhb_from_fs500dps_to_mdps(x_gyro)/1000.0f; //divide by 1000 to go from mdps to dps.
    y_gyro_dps = asm330lhb_from_fs500dps_to_mdps(y_gyro)/1000.0f;
    z_gyro_dps = asm330lhb_from_fs500dps_to_mdps(z_gyro)/1000.0f;
    temp_c = asm330lhb_from_lsb_to_celsius(temp);
}
