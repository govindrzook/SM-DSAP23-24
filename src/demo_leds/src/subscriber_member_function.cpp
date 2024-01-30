
#include <functional>
#include <memory>

#include <fstream>
#include <iostream>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

using std::placeholders::_1;

class MinimalSubscriber : public rclcpp::Node
{
public:
  MinimalSubscriber()
  : Node("minimal_subscriber")
  {
    	ax_subscription = this->create_subscription<std_msgs::msg::String>(
      		"aX", 10, std::bind(&MinimalSubscriber::ax_callback, this, _1));
	ay_subscription = this->create_subscription<std_msgs::msg::String>(
      		"aY", 10, std::bind(&MinimalSubscriber::ay_callback, this, _1));
	az_subscription = this->create_subscription<std_msgs::msg::String>(
      		"aZ", 10, std::bind(&MinimalSubscriber::az_callback, this, _1));
	gx_subscription = this->create_subscription<std_msgs::msg::String>(
      		"gX", 10, std::bind(&MinimalSubscriber::gx_callback, this, _1));
	gy_subscription = this->create_subscription<std_msgs::msg::String>(
      		"gY", 10, std::bind(&MinimalSubscriber::gy_callback, this, _1));
	gz_subscription = this->create_subscription<std_msgs::msg::String>(
      		"gZ", 10, std::bind(&MinimalSubscriber::gz_callback, this, _1));
  }

private:

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
	rclcpp::Subscription<std_msgs::msg::String>::SharedPtr ax_subscription;
	rclcpp::Subscription<std_msgs::msg::String>::SharedPtr ay_subscription;
	rclcpp::Subscription<std_msgs::msg::String>::SharedPtr az_subscription;

	rclcpp::Subscription<std_msgs::msg::String>::SharedPtr gx_subscription;
	rclcpp::Subscription<std_msgs::msg::String>::SharedPtr gy_subscription;
	rclcpp::Subscription<std_msgs::msg::String>::SharedPtr gz_subscription;

};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<MinimalSubscriber>());
  rclcpp::shutdown();
  return 0;
}
