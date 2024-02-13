#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <fcntl.h>
#include <sys/ioctl.h>
#include <linux/i2c-dev.h>

#define I2C_BUS "/dev/i2c-5" // Path to the I2C device file
#define I2C_ADDR 0x6A // I2C address of the device


char accel_x_h,accel_x_l, accel_y_h,accel_y_l, accel_z_h,accel_z_l;
char gyro_x_h,gyro_x_l, gyro_y_h,gyro_y_l, gyro_z_h,gyro_z_l;
char temp_h,temp_l;

uint16_t fifo_len = 0;

int16_t x_accel = 0;
int16_t y_accel = 0;
int16_t z_accel = 0;

int16_t x_gyro = 0;
int16_t y_gyro = 0;
int16_t z_gyro = 0;

int16_t temp = 0;

float x_accel_g, y_accel_g, z_accel_g, temp_f;
float x_gyro_dps, y_gyro_dps, z_gyro_dps;


int main() {
    int file;
    char *filename = (char*)I2C_BUS;
    int addr = I2C_ADDR;

    if ((file = open(filename, O_RDWR)) < 0) {
        perror("Failed to open the i2c bus");
        exit(1);
    }

    if (ioctl(file, I2C_SLAVE, addr) < 0) {
        perror("Failed to acquire bus access and/or talk to slave");
        exit(1);
    }

    __u8 reg = 0x0F; //WHO_AM_I register
    __u8 value = 0x2a;
/*
    // Example i2cset command: i2cset -y 1 0x53 0x10 0x2a
    // Set register 0x10 to value 0x2a
    __u8 reg = 0x10;
    __u8 value = 0x2a;
    if (write(file, &reg, 1) != 1) {
        perror("Failed to write to the i2c bus");
        exit(1);
    }

    if (write(file, &value, 1) != 1) {
        perror("Failed to write to the i2c bus");
        exit(1);
    }
*/

    // Example i2cget command: i2cget -y 5 0x6B 0x0F
    // Get value from register 0x0F
    if (write(file, &reg, 1) != 1) {
        perror("Failed to write to the i2c bus");
        exit(1);
    }

    __u8 data;

    if (read(file, &data, 1) != 1) {
        perror("Failed to read from the i2c bus");
        exit(1);
    }

    printf("Data read from register 0x0F: 0x%x\n", data);

    close(file);
    return 0;
}
