
#include <functional>
#include <memory>

#include <fstream>
#include <iostream>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "std_msgs/msg/float64.hpp"
#include "std_msgs/msg/u_int8.hpp"

using std::placeholders::_1;


class Logger : public rclcpp::Node
{
public:
  Logger(): Node("logger"){

    auto now = this->get_clock()->now();

  // Convert rclcpp::Time to std::chrono::system_clock::time_point
  std::chrono::time_point<std::chrono::system_clock> time_point(std::chrono::seconds(static_cast<int>(now.seconds())));

  // Convert the timepoint to a time_t
  std::time_t now_c = std::chrono::system_clock::to_time_t(time_point);

  // Format the time as a string
  std::stringstream ss;
  ss << std::put_time(std::localtime(&now_c), "%H%M%S"); // Example format: YYYYMMDDHHMMSS
  std::string str = ss.str();

file.open("log_" + str + ".log", std::ofstream::out | std::ofstream::app);


    ax_subscription = this->create_subscription<std_msgs::msg::Float64>(
      		"aX", 10, std::bind(&Logger::ax_callback, this, _1));
	ay_subscription = this->create_subscription<std_msgs::msg::Float64>(
      		"aY", 10, std::bind(&Logger::ay_callback, this, _1));
	az_subscription = this->create_subscription<std_msgs::msg::Float64>(
      		"aZ", 10, std::bind(&Logger::az_callback, this, _1));
	gx_subscription = this->create_subscription<std_msgs::msg::Float64>(
      		"gX", 10, std::bind(&Logger::gx_callback, this, _1));
	gy_subscription = this->create_subscription<std_msgs::msg::Float64>(
      		"gY", 10, std::bind(&Logger::gy_callback, this, _1));
	gz_subscription = this->create_subscription<std_msgs::msg::Float64>(
      		"gZ", 10, std::bind(&Logger::gz_callback, this, _1));	  
	temp_subscription = this->create_subscription<std_msgs::msg::Float64>(
      		"temp", 10, std::bind(&Logger::temp_callback, this, _1));
	  
	front_right_speed_subscription = this->create_subscription<std_msgs::msg::Float64>(
      		"frontRightSpeed", 10, std::bind(&Logger::front_right_speed_callback, this, _1));
    front_right_torque_subscription = this->create_subscription<std_msgs::msg::Float64>(
      		"frontRightTorque", 10, std::bind(&Logger::front_right_torque_callback, this, _1));

    front_right_steering_position_subscription = this->create_subscription<std_msgs::msg::UInt8>(
      		"frontRightSteerPosition", 10, std::bind(&Logger::fr_steering_position_callback, this, _1));
	
    front_right_brake_position_subscription = this->create_subscription<std_msgs::msg::UInt8>(
      		"frontRightBrakePosition", 10, std::bind(&Logger::fr_brake_position_callback, this, _1));

     front_left_steering_position_subscription = this->create_subscription<std_msgs::msg::UInt8>(
      		"frontLeftSteerPosition", 10, std::bind(&Logger::fl_steering_position_callback, this, _1));
	
    front_left_brake_position_subscription = this->create_subscription<std_msgs::msg::UInt8>(
      		"frontLeftBrakePosition", 10, std::bind(&Logger::fl_brake_position_callback, this, _1));

    

    error_subscription = this->create_subscription<std_msgs::msg::String>(
      		"error", 10, std::bind(&Logger::error_callback, this, _1));
	  
  }

private:

  const int loggerPeriod = 2; // Logger will wait until only execute every loggerPeriod callback.

  int frSoloSpeedReadingCount = 0;
  int frSoloTorqueCount = 0;

  int flSoloSpeedReadingCount = 0;
  int flSoloTorqueCount = 0;

  int brSoloSpeedReadingCount = 0;
  int brSoloTorqueCount = 0;

  int blSoloSpeedReadingCount = 0;
  int blSoloTorqueCount = 0;

  int axCount= 0;
  int ayCount = 0;
  int azCount = 0;
  int gxCount = 0;
  int gyCount= 0;
  int gzCount = 0;
  int tempCounter = 0;

  int frBrakeCounter = 0;
  int frSteerCounter = 0;
  int flBrakeCounter = 0;
  int flSteerCounter = 0;

   int brBrakeCounter = 0;
  int brSteerCounter = 0;
  int blBrakeCounter = 0;
  int blSteerCounter = 0;

  std::ofstream file;
  
	rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr front_right_torque_subscription;
  rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr front_right_speed_subscription;
  rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr front_left_torque_subscription;
  rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr front_left_speed_subscription;
  rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr back_right_torque_subscription;
  rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr back_right_speed_subscription;
  rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr back_left_torque_subscription;
  rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr back_left_speed_subscription;

	rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr ax_subscription;
	rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr ay_subscription;
	rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr az_subscription;
	rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr gx_subscription;
	rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr gy_subscription;
	rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr gz_subscription;
	rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr temp_subscription;
	
	rclcpp::Subscription<std_msgs::msg::UInt8>::SharedPtr front_right_steering_position_subscription;
	rclcpp::Subscription<std_msgs::msg::UInt8>::SharedPtr front_right_brake_position_subscription;
  rclcpp::Subscription<std_msgs::msg::UInt8>::SharedPtr front_left_steering_position_subscription;
	rclcpp::Subscription<std_msgs::msg::UInt8>::SharedPtr front_left_brake_position_subscription;

  rclcpp::Subscription<std_msgs::msg::UInt8>::SharedPtr back_right_steering_position_subscription;
	rclcpp::Subscription<std_msgs::msg::UInt8>::SharedPtr back_right_brake_position_subscription;
  rclcpp::Subscription<std_msgs::msg::UInt8>::SharedPtr back_left_steering_position_subscription;
	rclcpp::Subscription<std_msgs::msg::UInt8>::SharedPtr back_left_brake_position_subscription;

  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr error_subscription;

    std::string generateTimestamp(){

        auto now = std::chrono::system_clock::now();

        // Extract seconds, milliseconds, and nanoseconds
        auto seconds = std::chrono::time_point_cast<std::chrono::seconds>(now);
        auto milliseconds = std::chrono::duration_cast<std::chrono::milliseconds>(now - seconds).count();
        
        // Format the time as a string
        std::time_t now_c = std::chrono::system_clock::to_time_t(now);
        std::stringstream ss;
        

        ss << "[" << std::put_time(std::localtime(&now_c), "%Y-%m-%d %H:%M:%S") << "."
           << std::setw(3) << std::setfill('0') << milliseconds << "]";

           return ss.str();
    }
    void front_right_torque_callback(const std_msgs::msg::Float64 & msg)
    {
	    if(frSoloTorqueCount >= loggerPeriod){
		    
        RCLCPP_INFO(this->get_logger(), "FR-SOLO set torque command: '%f'", msg.data); 

        file << generateTimestamp() << " FR-SOLO torque command: " << msg.data << std::endl;
        frSoloTorqueCount = 0;

      }

      frSoloTorqueCount++;
        
    }

    void front_right_speed_callback(const std_msgs::msg::Float64 & msg)
    {
      if(frSoloSpeedReadingCount >= loggerPeriod){
        RCLCPP_INFO(this->get_logger(), "FR-BLDC speed reading: '%f'", msg.data);  
        file << "[" << generateTimestamp() << "] FR-SOLO speed feedback: " << msg.data << std::endl;


        frSoloSpeedReadingCount = 0;
      }
	    	
		    frSoloSpeedReadingCount++;
    }

    void front_left_torque_callback(const std_msgs::msg::Float64 & msg)
    {
	    if(flSoloTorqueCount >= loggerPeriod){
		    
        RCLCPP_INFO(this->get_logger(), "FL-SOLO set torque command: '%f'", msg.data); 

        file << generateTimestamp() << " FL-SOLO torque command: " << msg.data << std::endl;
        flSoloTorqueCount = 0;

      }

      flSoloTorqueCount++;
        
    }

    void front_left_speed_callback(const std_msgs::msg::Float64 & msg)
    {
      if(flSoloSpeedReadingCount >= loggerPeriod){
        RCLCPP_INFO(this->get_logger(), "FL-BLDC speed reading: '%f'", msg.data);  
        file << "[" << generateTimestamp() << "] FL-SOLO speed feedback: " << msg.data << std::endl;


        flSoloSpeedReadingCount = 0;
      }
	    	
		    flSoloSpeedReadingCount++;
    }

     void back_right_torque_callback(const std_msgs::msg::Float64 & msg)
    {
	    if(brSoloTorqueCount >= loggerPeriod){
		    
        RCLCPP_INFO(this->get_logger(), "BR-SOLO set torque command: '%f'", msg.data); 

        file << generateTimestamp() << " BR-SOLO torque command: " << msg.data << std::endl;
        brSoloTorqueCount = 0;

      }

      brSoloTorqueCount++;
        
    }

    void back_right_speed_callback(const std_msgs::msg::Float64 & msg)
    {
      if(brSoloSpeedReadingCount >= loggerPeriod){
        RCLCPP_INFO(this->get_logger(), "BR-BLDC speed reading: '%f'", msg.data);  
        file << "[" << generateTimestamp() << "] BR-SOLO speed feedback: " << msg.data << std::endl;


        brSoloSpeedReadingCount = 0;
      }
	    	
		    brSoloSpeedReadingCount++;
    }

    void back_left_torque_callback(const std_msgs::msg::Float64 & msg)
    {
	    if(blSoloTorqueCount >= loggerPeriod){
		    
        RCLCPP_INFO(this->get_logger(), "BL-SOLO set torque command: '%f'", msg.data); 

        file << generateTimestamp() << " BL-SOLO torque command: " << msg.data << std::endl;
        blSoloTorqueCount = 0;

      }

      blSoloTorqueCount++;
        
    }

    void back_left_speed_callback(const std_msgs::msg::Float64 & msg)
    {
      if(blSoloSpeedReadingCount >= loggerPeriod){
        RCLCPP_INFO(this->get_logger(), "BL-BLDC speed reading: '%f'", msg.data);  
        file << "[" << generateTimestamp() << "] BL-SOLO speed feedback: " << msg.data << std::endl;


        blSoloSpeedReadingCount = 0;
      }
	    	
		    blSoloSpeedReadingCount++;
    }
    void error_callback(const std_msgs::msg::String & msg) 
    {
        RCLCPP_INFO(this->get_logger(), "SOLO Error: ", msg.data);

        file << "[" << generateTimestamp() << "] SOLO Error: " << msg.data << std::endl;  
         
    }
    void ax_callback(const std_msgs::msg::Float64 & msg) 
    {
	     

      if(axCount >= loggerPeriod){
        RCLCPP_INFO(this->get_logger(), "X Accel (g): '%f'", msg.data);   
        
        file << "[" << generateTimestamp() << "] X Accel (g): " << msg.data << std::endl;
        axCount = 0;
      }
	    	
		    axCount++;
		  
    }
    void ay_callback(const std_msgs::msg::Float64 & msg) 
    {
	     

      if( ayCount >= loggerPeriod){
        RCLCPP_INFO(this->get_logger(), "Y Accel (g):'%f'", msg.data);

        file << "[" << generateTimestamp() << "] Y Accel (g): " << msg.data << std::endl;  
        ayCount = 0;

      }
	    	
		     ayCount++; 
		    
    }
    void az_callback(const std_msgs::msg::Float64 & msg) 
    {
	    

      if( azCount >= loggerPeriod){
        RCLCPP_INFO(this->get_logger(), "Z Accel (g): '%f'", msg.data);
        file << "[" << generateTimestamp() << "] Z Accel (g): " << msg.data << std::endl;

        azCount = 0;
      }
	    	
		     azCount++;

    }
    void gx_callback(const std_msgs::msg::Float64 & msg) 
    {
      if( gxCount >= loggerPeriod){
          RCLCPP_INFO(this->get_logger(), "X Gyro (dps): '%f'", msg.data);
          file << "[" << generateTimestamp() << "] X Gyro (dps): " << msg.data << std::endl; 
          gxCount = 0;
        }
          
          gxCount++;
		    
    }
    void gy_callback(const std_msgs::msg::Float64 & msg)
    {
      if(gyCount >= loggerPeriod){
        RCLCPP_INFO(this->get_logger(), "Y Gyro (dps): '%f'", msg.data);
        file << "[" << generateTimestamp() << "] Y Gyro (dps): " << msg.data << std::endl;    
        gyCount = 0;
      }  	
		    gyCount++;   
    }

    void gz_callback(const std_msgs::msg::Float64 & msg) 
    {
      if(gzCount >= loggerPeriod){
        RCLCPP_INFO(this->get_logger(), "Z Gyro (dps): '%f'", msg.data);
        file << "[" << generateTimestamp() << "] Z Gyro (dps): " << msg.data << std::endl;    
        gzCount = 0;
      }	
		    gzCount++;     
    }

	void temp_callback(const std_msgs::msg::Float64 & msg)
    {
      if(tempCounter >= loggerPeriod){
        RCLCPP_INFO(this->get_logger(), "Temperature (C): '%f'", msg.data);
        file << "[" << generateTimestamp() << "] IMU temperature (C): " << msg.data << std::endl;    
        tempCounter = 0;
      } 	
		    tempCounter++;     
    }

    void fr_steering_position_callback(const std_msgs::msg::UInt8 & msg)
    {
	    	
      if(frSteerCounter >= loggerPeriod){
        RCLCPP_INFO(this->get_logger(), "FR-Steering position: '%u'", msg.data);
        file << "[" << generateTimestamp() << "] FR-Steering servo position: " << static_cast<unsigned int>(msg.data)  << std::endl;     
        frSteerCounter = 0;
      }
	    	
		    frSteerCounter++; 
		    
    }
    void fr_brake_position_callback(const std_msgs::msg::UInt8 & msg)
    {	  
	     

      if(frBrakeCounter >= loggerPeriod){
        RCLCPP_INFO(this->get_logger(), "FR-Brake position: '%u'", msg.data);
        file << "[" << generateTimestamp() << "] FR-Braking servo position: " << static_cast<unsigned int>(msg.data) << std::endl; 
        frBrakeCounter = 0;
      }
	    	
		    frBrakeCounter++;  
		    
    }

    void fl_steering_position_callback(const std_msgs::msg::UInt8 & msg)
    {
	    	
      if(flSteerCounter >= loggerPeriod){
        RCLCPP_INFO(this->get_logger(), "FL-Steering position: '%u'", msg.data);
        file << "[" << generateTimestamp() << "] FL-Steering servo position: " << static_cast<unsigned int>(msg.data)  << std::endl;     
        flSteerCounter = 0;
      }
	    	
		    flSteerCounter++; 
		    
    }
    void fl_brake_position_callback(const std_msgs::msg::UInt8 & msg)
    {	  
	     

      if(flBrakeCounter >= loggerPeriod){
        RCLCPP_INFO(this->get_logger(), "FL-Brake position: '%u'", msg.data);
        file << "[" << generateTimestamp() << "] FL-Braking servo position: " << static_cast<unsigned int>(msg.data) << std::endl; 
        flBrakeCounter = 0;
      }
	    	
		    flBrakeCounter++;  
		    
    }

    void bl_steering_position_callback(const std_msgs::msg::UInt8 & msg)
    {
	    	
      if(blSteerCounter >= loggerPeriod){
        RCLCPP_INFO(this->get_logger(), "BL-Steering position: '%u'", msg.data);
        file << "[" << generateTimestamp() << "] BL-Steering servo position: " << static_cast<unsigned int>(msg.data)  << std::endl;     
        blSteerCounter = 0;
      }
	    	
		    blSteerCounter++; 
		    
    }
    void bl_brake_position_callback(const std_msgs::msg::UInt8 & msg)
    {	  
	     

      if(blBrakeCounter >= loggerPeriod){
        RCLCPP_INFO(this->get_logger(), "BL-Brake position: '%u'", msg.data);
        file << "[" << generateTimestamp() << "] BL-Braking servo position: " << static_cast<unsigned int>(msg.data) << std::endl; 
        blBrakeCounter = 0;
      }
	    	
		    blBrakeCounter++;  
		    
    }

    void br_steering_position_callback(const std_msgs::msg::UInt8 & msg)
    {
	    	
      if(brSteerCounter >= loggerPeriod){
        RCLCPP_INFO(this->get_logger(), "BR-Steering position: '%u'", msg.data);
        file << "[" << generateTimestamp() << "] BR-Steering servo position: " << static_cast<unsigned int>(msg.data)  << std::endl;     
        brSteerCounter = 0;
      }
	    	
		    brSteerCounter++; 
		    
    }
    void br_brake_position_callback(const std_msgs::msg::UInt8 & msg)
    {	  
	     

      if(brBrakeCounter >= loggerPeriod){
        RCLCPP_INFO(this->get_logger(), "BR-Brake position: '%u'", msg.data);
        file << "[" << generateTimestamp() << "] BR-Braking servo position: " << static_cast<unsigned int>(msg.data) << std::endl; 
        brBrakeCounter = 0;
      }
	    	
		    brBrakeCounter++;  
		    
    }

    
};

int main(int argc, char * argv[])
{

  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<Logger>());
  rclcpp::shutdown();

  return 0;
}
