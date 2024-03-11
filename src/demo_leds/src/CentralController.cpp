
#include <functional>
#include <memory>

#include <fstream>
#include <iostream>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "std_msgs/msg/u_int8.hpp"

using std::placeholders::_1;

class CentralController : public rclcpp::Node
{
public:
  CentralController()
  : Node("central_controller"), steer_(0), brake_(180)
  {
    	ax_subscription = this->create_subscription<std_msgs::msg::String>(
      		"aX", 10, std::bind(&CentralController::ax_callback, this, _1));
	ay_subscription = this->create_subscription<std_msgs::msg::String>(
      		"aY", 10, std::bind(&CentralController::ay_callback, this, _1));
	az_subscription = this->create_subscription<std_msgs::msg::String>(
      		"aZ", 10, std::bind(&CentralController::az_callback, this, _1));
	gx_subscription = this->create_subscription<std_msgs::msg::String>(
      		"gX", 10, std::bind(&CentralController::gx_callback, this, _1));
	gy_subscription = this->create_subscription<std_msgs::msg::String>(
      		"gY", 10, std::bind(&CentralController::gy_callback, this, _1));
	gz_subscription = this->create_subscription<std_msgs::msg::String>(
      		"gZ", 10, std::bind(&CentralController::gz_callback, this, _1));
	
	front_right_speed_subscription = this->create_subscription<std_msgs::msg::String>(
      		"frontRightSpeed", 10, std::bind(&CentralController::front_right_speed_callback, this, _1));

	pub1_ = create_publisher<std_msgs::msg::UInt8>("frontRightSteerPosition", 10);
    pub2_ = create_publisher<std_msgs::msg::UInt8>("frontRightBrakePosition", 10);
	pub3_ = create_publisher<std_msgs::msg::String>("frontRightTorque", 10);

	timer_ = create_wall_timer(std::chrono::milliseconds(3000), std::bind(&CentralController::timer_callback, this));

  }

private:
	int steer_;
	int brake_;

	void timer_callback()
  {
    // Create a message to publish
    	auto msg = std_msgs::msg::UInt8();
	auto msg1 = std_msgs::msg::UInt8();
	auto msg2 = std_msgs::msg::String();

	msg.data = steer_ + 15;  // Static steering angle data
	msg1.data = brake_ - 15; // static brake angle data
	msg2.data = "1 Nm"; // static torque data

    // Publish to each topic
    pub1_->publish(msg); //front right steer position
	pub2_->publish(msg1); // front right brake position
	pub3_->publish(msg2); // front right torque

	if(msg.data == 180){
		steer_ = 0;
	}
	if(msg1.data == 0){
		brake_ = 180 ;
	}
    
  }

  void ax_callback(const std_msgs::msg::String & msg) const
  {
	    	RCLCPP_INFO(this->get_logger(), "aX: '%s'", msg.data.c_str());  
		    
  }
 void ay_callback(const std_msgs::msg::String & msg) const
  {
	    	RCLCPP_INFO(this->get_logger(), "aY: '%s'", msg.data.c_str());  
		    
  }
 void az_callback(const std_msgs::msg::String & msg) const
  {
	    	RCLCPP_INFO(this->get_logger(), "aZ: '%s'", msg.data.c_str());  
		    
  }
 void gx_callback(const std_msgs::msg::String & msg) const
  {
	    	RCLCPP_INFO(this->get_logger(), "gX: '%s'", msg.data.c_str());  
		    
  }
 void gy_callback(const std_msgs::msg::String & msg) const
  {
	    	RCLCPP_INFO(this->get_logger(), "gY: '%s'", msg.data.c_str());  
		    
  }
 void gz_callback(const std_msgs::msg::String & msg) const
  {
	    	RCLCPP_INFO(this->get_logger(), "gZ: '%s'", msg.data.c_str());  
		    
  }

  void front_right_speed_callback(const std_msgs::msg::String & msg) const
  {
	    	RCLCPP_INFO(this->get_logger(), "Read front right speed: '%s'", msg.data.c_str());  
		    
  }
	rclcpp::Subscription<std_msgs::msg::String>::SharedPtr ax_subscription;
	rclcpp::Subscription<std_msgs::msg::String>::SharedPtr ay_subscription;
	rclcpp::Subscription<std_msgs::msg::String>::SharedPtr az_subscription;

	rclcpp::Subscription<std_msgs::msg::String>::SharedPtr gx_subscription;
	rclcpp::Subscription<std_msgs::msg::String>::SharedPtr gy_subscription;
	rclcpp::Subscription<std_msgs::msg::String>::SharedPtr gz_subscription;

	rclcpp::Subscription<std_msgs::msg::String>::SharedPtr front_right_speed_subscription;

	rclcpp::Publisher<std_msgs::msg::UInt8>::SharedPtr pub1_;
	rclcpp::Publisher<std_msgs::msg::UInt8>::SharedPtr pub2_;
	rclcpp::Publisher<std_msgs::msg::String>::SharedPtr pub3_;
  	rclcpp::TimerBase::SharedPtr timer_;

};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<CentralController>());
  rclcpp::shutdown();
  return 0;
}
