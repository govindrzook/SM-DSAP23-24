
#include </usr/include/libserial/SerialPort.h>
#include <cstdlib>
#include <fstream>
#include <iostream>

#include <chrono>
#include <thread>

constexpr const char* const SERIAL_PORT_2 = "/dev/ttyACM0" ;

/**
 * @brief This example reads the contents of a file and writes the entire 
 *        file to the serial port one character at a time. To use this
 *        example, simply utilize TestFile.txt or another file of your
 *        choosing as a command line argument.
 */
 
 std::string test(){
    LibSerial::SerialPort serial_port ;
 	return "test";
 }
// int main()
// {   
//     using namespace LibSerial ;
//     // Instantiate a SerialPort object.
//     SerialPort serial_port ;

//     try
//     {
//         // Open the Serial Port at the desired hardware port.
//         serial_port.Open(SERIAL_PORT_2) ;
//     }
//     catch (const OpenFailed&)
//     {
//         std::cerr << "The serial port did not open correctly." << std::endl ;
//         return EXIT_FAILURE ;
//     }


//     serial_port.FlushIOBuffers();
//     serial_port.FlushInputBuffer();
//     serial_port.FlushOutputBuffer();

//     // Set the baud rate of the serial port.
//     serial_port.SetBaudRate(BaudRate::BAUD_115200) ;

//     // Set the number of data bits.
//     serial_port.SetCharacterSize(CharacterSize::CHAR_SIZE_8) ;

//     // Turn off hardware flow control.
//     serial_port.SetFlowControl(FlowControl::FLOW_CONTROL_NONE) ;

  
//     serial_port.SetParity(Parity::PARITY_NONE) ;  // Disable parity.
    
//     // Set the number of stop bits.
//     serial_port.SetStopBits(StopBits::STOP_BITS_1) ;

//     // Read characters from the input file and write them to the serial port. 
//     std::cout << "Writing data to the serial port." << std::endl ;

//         std::ofstream outputFile("ascii.txt");

//         //char data_string[] = {'\xFF', '\xFF', '\x00','\x15','\x00','\x00','\x00','\x01','\x00','\xFE'};
//         //char data_byte = data_string[0];
        
//         char data_byte = 0xFF;
//         serial_port.WriteByte(data_byte);
//         serial_port.DrainWriteBuffer();
//         std::cout << "First byte written." << std::endl ;

//         //data_byte = data_string[1];
//         data_byte = 0xFF;
//         serial_port.WriteByte(data_byte);  
//         serial_port.DrainWriteBuffer() ;
//         std::cout << "Second byte written" << std::endl ;

//           //data_byte = data_string[2];
//           data_byte = 0x00;
//           serial_port.WriteByte(data_byte);
//          serial_port.DrainWriteBuffer() ;
//          std::cout << "Third byte written" << std::endl ;

//           //data_byte = data_string[3];
//           data_byte = 0x15;
//           serial_port.WriteByte(data_byte);
//          serial_port.DrainWriteBuffer() ;
//          std::cout << "4th byte written" << std::endl ;

//         //data_byte = data_string[4];
//         data_byte = 0x00;
//         serial_port.WriteByte(data_byte);
//         serial_port.DrainWriteBuffer() ;
//         std::cout << "5th byte written" << std::endl ;

//           //data_byte = data_string[5];
//           data_byte = 0x00;
//           serial_port.WriteByte(data_byte);
//          serial_port.DrainWriteBuffer() ;
//          std::cout << "6th byte written" << std::endl ;

//           //data_byte = data_string[6];
//           data_byte = 0x00;
//           serial_port.WriteByte(data_byte);
//          serial_port.DrainWriteBuffer() ;
//         std::cout << "7th byte written" << std::endl ;

//           //data_byte = data_string[7];
//           data_byte = 0x01;
//           serial_port.WriteByte(data_byte);
//          serial_port.DrainWriteBuffer() ;
//          std::cout << "8th byte written" << std::endl;

//          //data_byte = data_string[8];
//          data_byte = 0x00;
//          serial_port.WriteByte(data_byte);
//          serial_port.DrainWriteBuffer() ;
//          std::cout << "9th byte written" << std::endl;

//         //data_byte = data_string[9];
//         data_byte = 0xFE;
//         serial_port.WriteByte(data_byte);
//         serial_port.DrainWriteBuffer() ;
//         std::cout << "10th byte written" << std::endl;

//         std::string reading;
//         serial_port.Read(reading, 10, 5000);
//         std::cout << reading;
//         outputFile << reading;
//         outputFile.close();

//         serial_port.Close();

//     // Successful program completion.
//     std::cout << "The example program successfully completed!" << std::endl ;
//     return EXIT_SUCCESS ;
// }
