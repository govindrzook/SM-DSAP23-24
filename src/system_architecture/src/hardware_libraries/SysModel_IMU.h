#include <cstdint>

/* REGISTER ADDRESSES */
#define I2C_BUS "/dev/i2c-5" // Path to the I2C device file
#define IMU_I2C_ADDR 0x6A // I2C address of the device

#define REG_ACCEL_CTRL 0x10
#define REG_ACCEL_X_OUT_L 0x28
#define REG_ACCEL_X_OUT_H 0x29
#define REG_ACCEL_Y_OUT_L 0x2A
#define REG_ACCEL_Y_OUT_H 0x2B
#define REG_ACCEL_Z_OUT_L 0x2C
#define REG_ACCEL_Z_OUT_H 0x2D

#define REG_GYRO_CTRL 0x11
#define REG_GYRO_X_OUT_L 0x22
#define REG_GYRO_X_OUT_H 0x23
#define REG_GYRO_Y_OUT_L 0x24
#define REG_GYRO_Y_OUT_H 0x25
#define REG_GYRO_Z_OUT_L 0x26
#define REG_GYRO_Z_OUT_H 0x27

#define REG_TEMP_OUT_L 0x20
#define REG_TEMP_OUT_H 0x21

#define REG_WHO_AM_I 0x0F

/*!
 *  @brief  Class that stores functions for interacting with ASM330LHB IMU sensor.
 */

class SysModel_IMU {
    public:
        SysModel_IMU();
        void begin();

        float x_accel_g;
        float y_accel_g; 
        float z_accel_g;
        float x_gyro_dps;
        float y_gyro_dps;
        float z_gyro_dps;
        float temp_c;
        void read_and_convert_sensor_data(); 

    private:
        uint8_t _i2caddr;
        void i2c_write(uint8_t reg_address, uint8_t val);
        uint8_t i2c_read(uint8_t reg_address);
        uint16_t merge_bytes(uint8_t LSB, uint8_t MSB);
        int16_t two_complement_to_int(uint8_t LSB, uint8_t MSB);

        float asm330lhb_from_fs2g_to_mg(int16_t lsb);
        float asm330lhb_from_fs4g_to_mg(int16_t lsb);
        float asm330lhb_from_fs8g_to_mg(int16_t lsb);
        float asm330lhb_from_fs16g_to_mg(int16_t lsb);

        float asm330lhb_from_fs125dps_to_mdps(int16_t lsb);  
        float asm330lhb_from_fs250dps_to_mdps(int16_t lsb);
        float asm330lhb_from_fs500dps_to_mdps(int16_t lsb);
        float asm330lhb_from_fs1000dps_to_mdps(int16_t lsb);
        float asm330lhb_from_fs2000dps_to_mdps(int16_t lsb);
        float asm330lhb_from_fs4000dps_to_mdps(int16_t lsb);
        float asm330lhb_from_lsb_to_celsius(int16_t lsb);
        float asm330lhb_from_lsb_to_nsec(int32_t lsb);

        char accel_x_h,accel_x_l, accel_y_h,accel_y_l, accel_z_h,accel_z_l;
        char gyro_x_h,gyro_x_l, gyro_y_h,gyro_y_l, gyro_z_h,gyro_z_l;
        char temp_h, temp_l;
        int16_t x_accel = 0;
        int16_t y_accel = 0;
        int16_t z_accel = 0;
        int16_t x_gyro = 0;
        int16_t y_gyro = 0;
        int16_t z_gyro = 0;
        int16_t temp = 0;
    };
