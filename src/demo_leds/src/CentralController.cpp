
#include <functional>
#include <memory>

#include <fstream>
#include <iostream>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "std_msgs/msg/float64.hpp"
#include "std_msgs/msg/u_int8.hpp"


using std::placeholders::_1;
int flag = 1;
int key = 5;
int speed = 0;
int steer = 155;
int brake = 130;


class CentralController : public rclcpp::Node
{
public:
  CentralController()
  : Node("central_controller")
  {
    	ax_subscription = this->create_subscription<std_msgs::msg::Float64>(
      		"aX", 10, std::bind(&CentralController::ax_callback, this, _1));
	ay_subscription = this->create_subscription<std_msgs::msg::Float64>(
      		"aY", 10, std::bind(&CentralController::ay_callback, this, _1));
	az_subscription = this->create_subscription<std_msgs::msg::Float64>(
      		"aZ", 10, std::bind(&CentralController::az_callback, this, _1));
	gx_subscription = this->create_subscription<std_msgs::msg::Float64>(
      		"gX", 10, std::bind(&CentralController::gx_callback, this, _1));
	gy_subscription = this->create_subscription<std_msgs::msg::Float64>(
      		"gY", 10, std::bind(&CentralController::gy_callback, this, _1));
	gz_subscription = this->create_subscription<std_msgs::msg::Float64>(
      		"gZ", 10, std::bind(&CentralController::gz_callback, this, _1));
	  
	temp_subscription = this->create_subscription<std_msgs::msg::Float64>(
      		"temp", 10, std::bind(&CentralController::temp_callback, this, _1));
	  
	front_right_speed_subscription = this->create_subscription<std_msgs::msg::Float64>(
      		"frontRightSpeed", 10, std::bind(&CentralController::front_right_speed_callback, this, _1));

	pub1_ = create_publisher<std_msgs::msg::UInt8>("frontRightSteerPosition", 10);
   	pub2_ = create_publisher<std_msgs::msg::UInt8>("frontRightBrakePosition", 10);
	pub3_ = create_publisher<std_msgs::msg::Float64>("frontRightTorque", 10);

	timer_ = create_wall_timer(std::chrono::seconds(5), std::bind(&CentralController::timer_callback, this));
	timer_ask = create_wall_timer(std::chrono::seconds(1), std::bind(&CentralController::timer_ask_callback, this));

  }

private:

	void timer_ask_callback(){
		//  while(flag){
			printf("Enter '1' if you want to change the speed.\n");
			printf("Enter '2' if you want to change the the steering angle.\n");
			printf("Enter '3' if you want to change the set the braking angle.\n");
			printf("Enter '4' if you want to display all the output values.\n");
			printf("Enter '0' if you want to stop\n");
			scanf(" %d",&key);
			
			switch(key){
				case 0:
					flag = 0;
					printf("STOP\n.")
					rclcpp::shutdown();
					break;

				case 1:
					printf("Please enter the speed in RPM [min - max]\n.");
					scanf(" %d",&speed);
					printf("Speed: %d\n",speed);

					// rclcpp::spin_some(std::make_shared<CentralController>());
					break;
				
				case 2:
					printf("Please enter the steering angle [130 to 180]\n.");
					scanf(" %d",&steer);
					printf("Steer: %d\n",steer);
					// rclcpp::spin_some(std::make_shared<CentralController>());
					break;

				case 3:
					printf("Please enter the braking angle [40 to 130]\n.");
					scanf(" %d",&brake);
					printf("Brake: %d\n",brake);
					// rclcpp::spin_some(std::make_shared<CentralController>());
					break;
				
				case 4:
					printf("OUTPUTS\n.");
					// rclcpp::spin_some(std::make_shared<CentralController>());
					break;

				default:
					printf("Invalid key. Please try again.\n");
					break;
			}
			


//   }
		
	}

	void timer_callback(){

	
	// Create a message to publish
	auto msg = std_msgs::msg::UInt8();
	auto msg1 = std_msgs::msg::UInt8();
	auto msg2 = std_msgs::msg::Float64();
		
	switch(key){
		case 1:
			msg2.data = speed;
			break;
		case 2:
			msg.data = steer;
			break;
		case 3:
			msg1.data = brake;
			break;
		default:
			printf("No changes.\n");
			break;

	}
	

    // Publish to each topic
    pub1_->publish(msg); //front right steer position
	pub2_->publish(msg1); // front right brake position
	pub3_->publish(msg2); // front right torque

	

  	}

  	void ax_callback(const std_msgs::msg::Float64 & msg) const
  	{
		if(key == 4){
			RCLCPP_INFO(this->get_logger(), "aX: '%f'", msg.data);  
		}
	    	    
  	}
 	void ay_callback(const std_msgs::msg::Float64 & msg) const
  	{
		if(key == 4){
	    RCLCPP_INFO(this->get_logger(), "aY: '%f'", msg.data);  
		}
  	}
 	void az_callback(const std_msgs::msg::Float64 & msg) const
  	{
		if(key == 4){
	    RCLCPP_INFO(this->get_logger(), "aZ: '%f'", msg.data);  
		}
  	}
 	void gx_callback(const std_msgs::msg::Float64 & msg) const
  	{
		if(key == 4){
	    RCLCPP_INFO(this->get_logger(), "gX: '%f'", msg.data);  
		}
  	}
 	void gy_callback(const std_msgs::msg::Float64 & msg) const
  	{
		if(key == 4){
	    RCLCPP_INFO(this->get_logger(), "gY: '%f'", msg.data);  
		}
  	}
 	void gz_callback(const std_msgs::msg::Float64 & msg) const
  	{
		if(key == 4){
	    RCLCPP_INFO(this->get_logger(), "gZ: '%f'", msg.data);  
		}
  	}

	void temp_callback(const std_msgs::msg::Float64 & msg) const
  	{
		if(key == 4){
	    RCLCPP_INFO(this->get_logger(), "temp: '%f'", msg.data); 
		}     
  	}
  	void front_right_speed_callback(const std_msgs::msg::Float64 & msg) const
  	{
		if(key == 4){
		RCLCPP_INFO(this->get_logger(), "Front right BLDC speed feedback: '%f'", msg.data); 
		} 	    
  	}

	rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr ax_subscription;
	rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr ay_subscription;
	rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr az_subscription;

	rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr gx_subscription;
	rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr gy_subscription;
	rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr gz_subscription;

	rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr temp_subscription;
	
	rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr front_right_speed_subscription;

	rclcpp::Publisher<std_msgs::msg::UInt8>::SharedPtr pub1_;
	rclcpp::Publisher<std_msgs::msg::UInt8>::SharedPtr pub2_;
	rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr pub3_;
  	rclcpp::TimerBase::SharedPtr timer_;
	rclcpp::TimerBase::SharedPtr timer_ask;

};

int main(int argc, char * argv[])
{

  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<CentralController>());
  rclcpp::shutdown();
  return 0;
}
