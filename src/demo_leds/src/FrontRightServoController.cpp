
#include <functional>
#include <memory>

#include <fstream>
#include <iostream>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

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
	    	RCLCPP_INFO(this->get_logger(), "Steering position: '%s'", msg.data.c_str());  
		    
  }
 void brake_position_callback(const std_msgs::msg::String & msg) const
  {
	    	RCLCPP_INFO(this->get_logger(), "Brake position: '%s'", msg.data.c_str());  
		    
  }
 
	rclcpp::Subscription<std_msgs::msg::String>::SharedPtr front_right_steering_position_subscription;
	rclcpp::Subscription<std_msgs::msg::String>::SharedPtr front_right_brake_position_subscription;

};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<FrontRightServoController>());
  rclcpp::shutdown();
  return 0;
}
