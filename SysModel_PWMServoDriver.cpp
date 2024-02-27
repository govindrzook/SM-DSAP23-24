
#include <SysModel_PWMServoDriver.h>
#include <cstdint>
#include <time.h>



SysModel_PWMServoDriver::SysModel_PWMServoDriver(){
    // allows the user to create an instance for a specific pwm output
    _i2caddr = PCA9685_I2C_ADDRESS;
    begin();

    // default freq: 50Hz
    /*NOT DONEEEE*/
    

}

SysModel_PWMServoDriver::SysModel_PWMServoDriver(const int servoNum, std::uint8_t freq){
    // allows the user to create an instance for a specific pwm output and freq
    /*NOT DONEEEE*/
}


void delay(int milliseconds)
{
	// Converting time into milli_seconds
	// int milli_seconds = 1000000 * number_of_seconds;
  int milli_seconds = 1000 * milliseconds;
  
	// Storing start time
	clock_t start_time = clock();

	// looping till required time is not achieved
	while (clock() < start_time + milli_seconds)
		;
}

void SysModel_PWMServoDriver::sleep() {
    uint8_t awake = readReg(PCA9685_MODE1);
    uint8_t sleep = awake | MODE1_SLEEP; // set sleep bit high
    writeReg(PCA9685_MODE1, sleep);
    // delay(5); // wait until cycle ends for sleep to be active
}

void SysModel_PWMServoDriver::wakeup() {
    uint8_t sleep = readReg(PCA9685_MODE1);
    uint8_t wakeup = sleep & ~MODE1_SLEEP; // set sleep bit low
    writeReg(PCA9685_MODE1, wakeup);
}

void SysModel_PWMServoDriver::setFreq(float freq){

    // Range output modulation frequency is dependant on oscillator
    if (freq < 1)
    freq = 1;
    if (freq > 3500)
    freq = 3500; // Datasheet limit is 3052=50MHz/(4*4096)

    float prescaleval = ((/*_oscillator_freq*/ FREQUENCY_OSCILLATOR / (freq * 4096.0)) + 0.5) - 1;
    if (prescaleval < PCA9685_PRESCALE_MIN){
    prescaleval = PCA9685_PRESCALE_MIN;
    }
    if (prescaleval > PCA9685_PRESCALE_MAX){
    prescaleval = PCA9685_PRESCALE_MAX;
    }
    uint8_t prescale = (uint8_t)prescaleval;


    uint8_t oldmode = readReg(PCA9685_MODE1);
    uint8_t newmode = (oldmode & ~MODE1_RESTART) | MODE1_SLEEP; // sleep
    writeReg(PCA9685_MODE1, newmode);                             // go to sleep
    writeReg(PCA9685_PRESCALE, prescale); // set the prescaler
    writeReg(PCA9685_MODE1, oldmode);
    delay(5);
    // This sets the MODE1 register to turn on auto increment.
    writeReg(PCA9685_MODE1, oldmode | MODE1_RESTART | MODE1_AI);

    _freq = freq;
}

void SysModel_PWMServoDriver::begin(){
    //initialize all LED registers to 0
    writeReg(PCA9685_ALLLED_ON_L,0);
    writeReg(PCA9685_ALLLED_ON_H,0);
    writeReg(PCA9685_ALLLED_OFF_L,0);
    writeReg(PCA9685_ALLLED_OFF_H,0);
    
    //set frequency to 50
    setFreq(50);
}

void SysModel_PWMServoDriver::setAngle(int servoNum, int angle){
    // float freq = 50; //change this so that freq is a private variable of servo
    float dutyCycle;
    uint16_t regValue;
    long pulseLength = map(angle,SERVO_ANGLE_MIN,SERVO_ANGLE_MAX,SERVO_PULSE_MIN,SERVO_PULSE_MAX); //in microseconds
    pulseLength *= 1000000; // convert pulselength to seconds
    regValue = ((pulseLength*_freq)*4096)-1;

    uint8_t buffer[4];
    buffer[0] = PCA9685_ALLLED_OFF_L + 4 * servoNum;
    buffer[1] = regValue && 0xFF; // to get the 8LSB
    buffer[2] = PCA9685_ALLLED_OFF_H + 4 * servoNum;
    buffer[3] = regValue >> 8; // to get the 4 MSB

    writeReg(buffer[0],buffer[1]);
    writeReg(buffer[2],buffer[3]);

}
/*Read/Write to Registers*/

void SysModel_PWMServoDriver::SysModel_PWMServoDriver::writeReg(uint8_t addr,uint8_t value){
    int file;
    char *filename = (char*)I2C_BUS;
    uint8_t data;

    if ((file = open(filename, O_RDWR)) < 0) {
        perror("Failed to open the i2c bus");
        exit(1);
    }

    if (ioctl(file, I2C_SLAVE, _i2caddr) < 0) {
        perror("Failed to acquire bus access and/or talk to slave");
        exit(1);
    }

    if (write(file, &addr, 1) != 1) {
        perror("Failed to write to the i2c bus");
        exit(1);
    }

    if (write(file, &value, 1) != 1) {
        perror("Failed to read from the i2c bus");
        exit(1);
    }

    close(file);

}

uint8_t SysModel_PWMServoDriver::readReg(uint8_t addr){
    int file;
    char *filename = (char*)I2C_BUS;
    uint8_t data;
 
    if ((file = open(filename, O_RDWR)) < 0) {
        perror("Failed to open the i2c bus");
        exit(1);
    }
 
    if (ioctl(file, I2C_SLAVE, _i2caddr) < 0) {
        perror("Failed to acquire bus access and/or talk to slave");
        exit(1);
    }

    if (write(file, &addr, 1) != 1) {
        perror("Failed to write to the i2c bus");
        exit(1);
    }
 
    if (read(file, &data, 1) != 1) {
        perror("Failed to read from the i2c bus");
        exit(1);
    }
 
    printf("Data read from register %x: 0x%x\n",addr, data);
 
    close(file);
    
    return data;

}

long SysModel_PWMServoDriver::map(long x, long in_min, long in_max, long out_min, long out_max) {
  return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}


