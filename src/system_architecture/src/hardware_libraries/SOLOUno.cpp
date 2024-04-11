#include </usr/include/libserial/SerialPort.h>
#include <cstdlib>
#include <fstream>
#include <iostream>
#include <chrono>
#include <thread>
#include <sstream>
#include <cmath>
#include <bitset>

// SOLO COMMAND CODES:_____________________________________________________________________________________________________
#define SET_COMMANDING_MODE 0x02
#define SET_MOTOR_DIRECTION 0x0C
#define SET_SPEED_REFERENCE 0x05
#define SET_TORQUE_REFERENCE 0x04

#define SET_CONTROL_MODE_TYPE 0x16
#define SET_TORQUE_PROPORTIONAL_GAIN 0x17
#define SET_TORQUE_INTEGRAL_GAIN 0x18

#define SET_SPEED_CONTROL_PROPORTIONAL_GAIN 0x0A
#define SET_SPEED_CONTROL_INTEGRAL_GAIN  0x0B
#define SET_MOTOR_TYPE 0x15
#define EMERGENCY_STOP 0x08

#define READ_SPEED_FEEDBACK 0x96
#define READ_ERROR_CODES 0xA1
#define CLEAR_ERROR_CODES 0x20

// SERIAL PORT NAMES:_____________________________________________________________________________________________________
#define PORT_NAME "/dev/ttyACM0"    // A SOLO UNO typically enumerates as ttyACM0.
#define PORT_NAME_2 "/dev/ttyACM1"  // A SOLO UNO may enumerate as ttyACM1 if a solo uno is already connected.

// COMMAND PARAMETERS: ___________________________________________________________________________________________
#define DC_BRUSHED 0x00
#define BLDC_MOTOR 0x01
#define ACIM 0x02
#define ULTRA_FAST_BLDC 0x03
#define DIGITAL_MODE 0x1
#define ANALOG_MODE 0x0 


class SoloUno {
public:
    SoloUno(char add) {
        this->address = add;
        initSolo();
    }
     void end(){ // Closes serial port.
        serial_port.Close();
    }

    void setupForTorqueControlHallSensors(){

        soloWriteSlow(0x15,0x01); // Brushless motor
        soloWriteSlow(0x16,0x01); // torque control
        soloWriteSlow(0x02, 0x01); // digital control
        soloWriteSlow(0x13, 0x02); // use hall effect sensors for feedback
        
    }

    int soloWriteFast(char cmd, int data) { // Fast version of a write, doesn't read back the echo'd data for validation.
     
        serial_port.FlushIOBuffers();
        serial_port.FlushInputBuffer();
        serial_port.FlushOutputBuffer();

        char initiator = 0xFF;
        char address = this->address;
        char command = cmd;
        int dataIn = data;

        char data0 = static_cast<char>((dataIn >> 24) & 0xFF);
        char data1 = static_cast<char>((dataIn >> 16) & 0xFF);
        char data2 = static_cast<char>((dataIn >> 8) & 0xFF);
        char data3 = static_cast<char>((dataIn & 0xFF));

        char crc = 0x00;
        char ending = 0xFE;

        char data_byte[] = {initiator, initiator, address, command, data0, data1, data2, data3, crc, ending};

        for (int x = 0; x < 10; x++) {

            serial_port.WriteByte(data_byte[x]);
            serial_port.DrainWriteBuffer();
        }
     
        return 0;
        
    }

    int soloWriteSlow(char cmd, int data) { // Slower because it checks for errors in transmission.

        serial_port.FlushIOBuffers();
        serial_port.FlushInputBuffer();
        serial_port.FlushOutputBuffer();

        char initiator = 0xFF;
        char address = this->address;
        char command = cmd;
        int dataIn = data;

        char data0 = static_cast<char>((dataIn >> 24) & 0xFF);
        char data1 = static_cast<char>((dataIn >> 16) & 0xFF);
        char data2 = static_cast<char>((dataIn >> 8) & 0xFF);
        char data3 = static_cast<char>((dataIn & 0xFF));

        char crc = 0x00;
        char ending = 0xFE;

        char data_byte[] = {initiator, initiator, address, command, data0, data1, data2, data3, crc, ending};


        for (int x = 0; x < 10; x++) {

            serial_port.WriteByte(data_byte[x]);
            serial_port.DrainWriteBuffer();

        }    

        std::string reading;
        std::string writtenValue = std::string(1, data_byte[0]) + std::string(1, data_byte[1]) + std::string(1, data_byte[2]) +
                                   std::string(1, data_byte[3]) + std::string(1, data_byte[4]) + std::string(1, data_byte[5]) +
                                   std::string(1, data_byte[6]) + std::string(1, data_byte[7]) + std::string(1, data_byte[8]) +
                                   std::string(1, data_byte[9]);

        try {
             serial_port.Read(reading, 10, 50);
         } catch (const LibSerial::ReadTimeout&) {
            std::cout << "SOLO UNO READ TIMEOUT" << std::endl;
            this->setSoloError("SOLO Read Timeout");
         }

         if (reading != writtenValue) {
             std::cout << "SOLO UNO WRITE ERROR" << std::endl;
             this->setSoloError("SOLO Write Error: Echo'd data not equal to sent data");
             return 1; // Write failure.
         }
         else{
             return 0; // Successful write.
         }
    }

    std::string soloRead(char cmd) {   

        serial_port.FlushIOBuffers();
        serial_port.FlushInputBuffer();
        serial_port.FlushOutputBuffer();

        char initiator = 0xFF;
        char address = this->address;
        char command = cmd;
        char crc = 0x00;
        char ending = 0xFE;
        char data = 0x00;

        char data_byte[] = {initiator, initiator, address, command, data, data, data, data, crc, ending};  

          for (int x = 0; x < 10; x++) {
              serial_port.WriteByte(data_byte[x]);
              serial_port.DrainWriteBuffer();
         }
  
        std::string reading;
        try {
             serial_port.Read(reading, 10, 50);
         } catch (const LibSerial::ReadTimeout&) {
            this->setSoloError("SOLO Read Timeout");
            std::cout << "SOLO UNO READ TIMEOUT" << std::endl;
         }
         
        return reading;

    }

    int readSpeed() {
        uint32_t soloData = getDataFromSoloPacket(soloRead(READ_SPEED_FEEDBACK)); // Get 8 data bytes from the returned packet.
        int velocity = soloInt32toInt(soloData); // Convert the 8 bytes to a usable Int.
        return velocity; // Positive is clockwise, negative is counter-clockwise.
    }
    void setSoloError(std::string error){ // Used within public functions to set the error field for the SOLO.
        this->error = error;
    } 

    std::string getSoloError(){ // Return the error associated with the SOLO and also reset it.
        std::string errorCode = this->error;
        this->error = "";
        return errorCode;
    }

    std::string readErrorRegister(){

        std::string errorCodes = "";
        uint32_t emergencyData = getDataFromSoloPacket(soloRead(READ_ERROR_CODES));

        if(emergencyData | 0x1){
            errorCodes += "Over-current error, ";
        }
        if(emergencyData | 0x10){
            errorCodes += "Over-voltage error, ";
        }

        if(emergencyData | 0x100){
            errorCodes += "Over-temperature error, ";
        }

        if(emergencyData | 0x1000){
            errorCodes += "Encoder calibration timeout - index pulse is missing, ";
        }

        if(emergencyData | 0x10000){
            errorCodes += "Hall sensors calibration timeout, ";
        }

        if(emergencyData | 0x100000){
            errorCodes += "CAN communication lost, ";
        }

        if(emergencyData | 0x1000000){
            errorCodes += "Stall timeout, ";
        }
        return errorCodes;
    }

    int clearErrorRegister(){
        int result = soloWriteSlow(CLEAR_ERROR_CODES, 0x00);
        return result;
    }
    int setCommandingMode(u_int32_t commandingMode){
        int result = soloWriteSlow(SET_COMMANDING_MODE, commandingMode);
    }
    void setTorqueFast(double data){
        int dat = doubleToFixedPoint(data); // Torque reference uses fixed point 32-17 for the data.
        soloWriteFast(SET_TORQUE_REFERENCE, dat);
    }

    int setTorqueSlow(double data){
        int dat = doubleToFixedPoint(data); // Torque reference uses fixed point 32-17 for the data.
        return soloWriteSlow(SET_TORQUE_REFERENCE, dat);
    }

    void setSpeedFast(double data){
        int dat = floor(data); // Speed reference uses unsigned int for the data.
        soloWriteFast(SET_SPEED_REFERENCE, dat);
    }

    int setSpeedSlow(double data){  
        int dat = floor(data); // Speed reference uses unsigned int for the data.
        return soloWriteSlow(SET_SPEED_REFERENCE, dat); 
    }
    void setDirectionFast(double data){ 
        int dat = floor(data); // Direction uses unsigned int for the data. (0 or 1)
        soloWriteFast(SET_MOTOR_DIRECTION, dat);  
    }

    int setDirectionSlow(double data){ // Inputs greater than 0 yield clockwise. Inputs less than 1 yield a counter-clockwise rotation.
        int dat;

        if(data >= 1){
            dat = 1;
        }
        else{
            dat = 0;
        }   
        return soloWriteSlow(SET_MOTOR_DIRECTION, dat);
    }

    int setMotorType(u_int32_t motorType){
        soloWriteSlow(SET_MOTOR_TYPE, motorType);
    }

    int setControlModeType(u_int32_t controlType){
        soloWriteSlow(SET_CONTROL_MODE_TYPE, controlType);
    }
    
    void emergencyStop(){
        for (int x = 0; x < 10; x++){
            soloWriteFast(EMERGENCY_STOP, 0x00000000); // Send 0's to Estop command 10 times to gaurantee* success.
        }
    }

    uint32_t getDataFromSoloPacket(std::string reading){ // Getting 8 DATA bytes from SOLO packet
        
        uint32_t data = 0;

        unsigned char b0 = static_cast<unsigned char>(reading[4]);
        unsigned char b1 = static_cast<unsigned char>(reading[5]);
        unsigned char b2 = static_cast<unsigned char>(reading[6]);
        unsigned char b3 = static_cast<unsigned char>(reading[7]);

        data = (static_cast<uint32_t>(b0) << 24) |
                       (static_cast<uint32_t>(b1) << 16) |
                       (static_cast<uint32_t>(b2) << 8) |
                       static_cast<uint32_t>(b3);
        //std::cout << "Formatted int: " << formattedInt << std::endl;

        return data;
    }

private:
 
    void initSolo() {
         try {
             serial_port.Open(PORT_NAME);
         } catch (const LibSerial::OpenFailed&) {
             try{
                 serial_port.Open(PORT_NAME_2);
             } catch(const LibSerial::OpenFailed&){
                 std::cerr << "The serial port did not open correctly." << std::endl;
                 this->setSoloError("Serial port failed to open");
                 return;
             }    
         }

        serial_port.SetBaudRate(LibSerial::BaudRate::BAUD_921600);
        serial_port.SetCharacterSize(LibSerial::CharacterSize::CHAR_SIZE_8);
        serial_port.SetFlowControl(LibSerial::FlowControl::FLOW_CONTROL_NONE);
        serial_port.SetParity(LibSerial::Parity::PARITY_NONE);
        serial_port.SetStopBits(LibSerial::StopBits::STOP_BITS_1);

        std::this_thread::sleep_for(std::chrono::seconds(1)); // Add delay to ensure the port is ready.

    }

    int soloInt32toInt(uint32_t soloData){
        // This function takes data from a solo packet and will return a usable integer from the signed 
        // data in a solo packet. This is intended to be used for reads such as Speed Feedback, which can be signed.
    
        if(soloData < 0x7FFFFFFF){ // If MSB of SOLO Data packet is 0: (data is positive)
            return (int) soloData; // The data calculated above will be correct.
        }
        else{
            int negativeData = 0xFFFFFFFF - soloData + 0x1; // Conversion from 2's compliment to integer.
            negativeData = negativeData*-1; // Invert result since it represents a negative number.
            return negativeData;
        }
    }
    double fixedPointToDouble(int data){
            // This process was built with guidance from the official SOLO UNO communication manual.

            double result;
            double den = 131072;

            if(data <= 0x7FFE0000){
                result = data/den;
            }
            else{
                result = data/den*-1;
            }
            return result;
    }

    int doubleToFixedPoint(double data){
         // This process was built with guidance from the official SOLO UNO communicatio manual.
         unsigned int result;
         double mult = 131072;

         if(data >= 0){
             double prod = data*mult;
             int roundedDown = static_cast<int>(floor(prod));
             result = roundedDown;
         }
         if(data < 0){
             double prod = data*mult;
             unsigned int roundedDown = abs(static_cast<int>(floor(prod)));
             unsigned int eightBytesF = 0xFFFFFFFF;
             result = eightBytesF - roundedDown + 1;

         }

         return result;
    }

private:
    LibSerial::SerialPort serial_port;
    std::string port_name;
    std::string error;
    char address;
};

