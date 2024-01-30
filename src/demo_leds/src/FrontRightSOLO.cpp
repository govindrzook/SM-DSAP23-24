
#include <functional>
#include <memory>

#include <fstream>
#include <iostream>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

using std::placeholders::_1;

class FrontRightSOLO : public rclcpp::Node
{
public:
  FrontRightSOLO()
  : Node("front_right_solo")
  {
    	front_right_torque_subscription = this->create_subscription<std_msgs::msg::String>(
      		"frontRightTorque", 10, std::bind(&FrontRightSOLO::front_right_torque_callback, this, _1));
	

	pub1_ = create_publisher<std_msgs::msg::String>("frontRightSpeed", 10);
    

	timer_ = create_wall_timer(std::chrono::milliseconds(1), std::bind(&FrontRightSOLO::timer_callback, this));

  }

private:

	void timer_callback()
  {
    // Create a message to publish
    auto msg = std_msgs::msg::String();
    msg.data = "500 RPM";  // Example data, replace with your actual data


    // Publish to each topic
    pub1_->publish(msg);
	
    
  }

  void front_right_torque_callback(const std_msgs::msg::String & msg) const
  {
	    	RCLCPP_INFO(this->get_logger(), "Received torque command: '%s'", msg.data.c_str());  
		    
  }
 
	rclcpp::Subscription<std_msgs::msg::String>::SharedPtr front_right_torque_subscription;
	rclcpp::Publisher<std_msgs::msg::String>::SharedPtr pub1_;
  	rclcpp::TimerBase::SharedPtr timer_;

};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<FrontRightSOLO>());
  rclcpp::shutdown();
  return 0;
}
