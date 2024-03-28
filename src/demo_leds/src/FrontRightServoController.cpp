
#include <functional>
#include <memory>

#include <fstream>
#include <iostream>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "std_msgs/msg/u_int8.hpp"

#include "SysModel_PWMServoDriver.cpp"

using std::placeholders::_1;

size_t prev_steer = 0;
size_t prev_brake = 0;

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

    
  }

private:
	SysModel_PWMServoDriver servo_steer;
	SysModel_PWMServoDriver servo_brake;
	size_t steer_output = 0;
	size_t brake_output = 8;
	



  void steering_position_callback(const std_msgs::msg::UInt8 & msg)
  {
	if(prev_steer != msg.data){
		if(msg.data >= 130 && msg.data <= 180){
			servo_steer.setAngle(steer_output,msg.data);  	
			RCLCPP_INFO(this->get_logger(), "Steering position: '%u' , Prev: '%u' ", msg.data,prev_steer);
		}else{
			printf("Steering angle is out of range.\n");
		}
		
	}else{
		printf("Steering angle input is the same as the previous one.\n");
	}
	  
	prev_steer = msg.data;
	
		    
  }
 void brake_position_callback(const std_msgs::msg::UInt8 & msg)
{
	if(prev_brake != msg.data){
		if(msg.data <= 130 && msg.data >= 40){
			  servo_brake.setAngle(brake_output,msg.data);  
			  RCLCPP_INFO(this->get_logger(), "Brake position: '%u' , Prev: '%u' ", msg.data,prev_brake);  
		}else{
			printf("Braking angle is out of range.\n");
		}
	}else{
		printf("Braking angle input is the same as the previous one.\n");
	}

	prev_brake = msg.data;
		    
  }
 
	rclcpp::Subscription<std_msgs::msg::UInt8>::SharedPtr front_right_steering_position_subscription;
	rclcpp::Subscription<std_msgs::msg::UInt8>::SharedPtr front_right_brake_position_subscription;

};

int main(int argc, char * argv[])
{
	

  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<FrontRightServoController>());
  rclcpp::shutdown();
  return 0;
}
