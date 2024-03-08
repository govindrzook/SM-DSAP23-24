
#include <functional>
#include <memory>

#include <fstream>
#include <iostream>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

#include "SysModel_PWMServoDriver.cpp"

using std::placeholders::_1;

class FrontRightServoController : public rclcpp::Node
{
public:
  FrontRightServoController()
  : Node("front_right_servo_controller")
  {
    front_right_steering_position_subscription = this->create_subscription<std_msgs::msg::String>(
      		"frontRightSteerPosition", 10, std::bind(&FrontRightServoController::steering_position_callback, this, _1));
	
    front_right_brake_position_subscription = this->create_subscription<std_msgs::msg::String>(
      		"frontRightBrakePosition", 10, std::bind(&FrontRightServoController::brake_position_callback, this, _1));
    
  }

private:



  void steering_position_callback(const std_msgs::msg::String & msg) const
  {
	  // SysModel_PWMServoDriver servo_steer;
	  // servo_steer.begin();
	  double buffer = std::stod(msg.data);
	  servo_steer.setAngle(0,buffer);  	
	  RCLCPP_INFO(this->get_logger(), "Steering position: '%s'", msg.data.c_str());  
		    
  }
 void brake_position_callback(const std_msgs::msg::String & msg) const
  {	  
	  // SysModel_PWMServoDriver servo_brake;
	  // servo_brake.begin();
	  double buffer = std::stod(msg.data);
	  servo_brake.setAngle(1,buffer);  
	  RCLCPP_INFO(this->get_logger(), "Brake position: '%s'", msg.data.c_str());  
		    
  }
 
	rclcpp::Subscription<std_msgs::msg::String>::SharedPtr front_right_steering_position_subscription;
	rclcpp::Subscription<std_msgs::msg::String>::SharedPtr front_right_brake_position_subscription;

};

int main(int argc, char * argv[])
{
	SysModel_PWMServoDriver servo_steer;
SysModel_PWMServoDriver servo_brake;
	servo_steer.begin();
	servo_brake.begin();
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<FrontRightServoController>());
  rclcpp::shutdown();
  return 0;
}
