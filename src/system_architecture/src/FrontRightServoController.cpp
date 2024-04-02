#include <functional>
#include <memory>
#include <fstream>
#include <iostream>
#include <stdio.h>
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "std_msgs/msg/u_int8.hpp"
#include "hardware_libraries/SysModel_PWMServoDriver.cpp"

using std::placeholders::_1;

class FrontRightServoController : public rclcpp::Node
{
public:
  FrontRightServoController()
  : Node("front_right_servo_controller")
  {
    front_right_steering_position_subscription = this->create_subscription<std_msgs::msg::UInt8>(
      		"frontRightSteerPosition", 10, std::bind(&FrontRightServoController::steering_position_callback, this, _1));
	
    front_right_brake_position_subscription = this->create_subscription<std_msgs::msg::UInt8>(
      		"frontRightBrakePosition", 10, std::bind(&FrontRightServoController::brake_position_callback, this, _1));

    pub_error = create_publisher<std_msgs::msg::String>("error", 10);
  }

private:
	SysModel_PWMServoDriver servo_steer;
	SysModel_PWMServoDriver servo_brake;
	
	size_t prevSteer = 0;
	size_t prevBrake = 0;

	size_t steerOutput = 0;
	size_t brakeOutput = 8;

	uint32_t maxBrake = 130;
	uint32_t minBrake = 60;

	uint32_t maxSteer = 180;
	uint32_t minSteer = 135;

  void steering_position_callback(const std_msgs::msg::UInt8 & msg)
  {
	if(prevSteer != msg.data){
		if(msg.data >= minSteer && msg.data <= maxSteer){
			servo_steer.setAngle(steerOutput,msg.data);  	
		}else{
			auto msg_error = std_msgs::msg::String();
    		msg_error.data = "FR-Steering angle is out of range.";
    		pub_error->publish(msg_error);
		}
		
	}else{
    	RCLCPP_INFO(this->get_logger(), "Steering angle input is the same as the previous one.");
	}
	prevSteer = msg.data;	    
  }
 void brake_position_callback(const std_msgs::msg::UInt8 & msg)
{
	if(prevBrake != msg.data){
		if(msg.data <= maxBrake && msg.data >= minBrake){
			servo_brake.setAngle(brakeOutput,msg.data);  
		}else{
			
			auto msg_error = std_msgs::msg::String();
    		msg_error.data = "FR-Brake angle input is out of range.";
    		pub_error->publish(msg_error);
		}
	}else{
    	RCLCPP_INFO(this->get_logger(), "FR-Brake angle input is the same as the previous one.");
	}

	prevBrake = msg.data;
		    
  }
 
	rclcpp::Subscription<std_msgs::msg::UInt8>::SharedPtr front_right_steering_position_subscription;
	rclcpp::Subscription<std_msgs::msg::UInt8>::SharedPtr front_right_brake_position_subscription;

	rclcpp::Publisher<std_msgs::msg::String>::SharedPtr pub_error;
};

int main(int argc, char * argv[])
{
	

  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<FrontRightServoController>());
  rclcpp::shutdown();
  return 0;
}
