
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
int input;

// Create a message to publish
	auto msg = std_msgs::msg::UInt8();
	auto msg1 = std_msgs::msg::UInt8();
	auto msg2 = std_msgs::msg::Float64();


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

	timer_ = create_wall_timer(std::chrono::seconds(1), std::bind(&CentralController::timer_callback, this));
	// timer_ask = create_wall_timer(std::chrono::seconds(1), std::bind(&CentralController::timer_ask_callback, this));

  }

private:

	

	void timer_callback(){
	
		printf("Select from the following options:\n");
		printf("  '1' if you want to change the speed.\n");
		printf("  '2' if you want to change the the steering angle.\n");
		printf("  '3' if you want to change the set the braking angle.\n");
		printf("  '4' if you want to display all the output values.\n");
		printf("  '0' if you want to stop\n\n");
		flag = 1;
		scanf(" %d",&key);
		
		switch(key){
			
			case 0:
				printf("STOP\n");
				rclcpp::shutdown();
				break;

			case 1:
				while(flag){
					printf(">> Please enter the speed in RPM (CCW) [100 - 200 ] or (CW) [-100 to -200].\n");
					printf(">> To return to the previous menu, enter -1.\n");
					scanf(" %d",&input);

					if(input == -1){ 
						flag = 0;
						break; // speed is the same
					}else if((input >= 100 && input <=200) || (input <= -100 && input >= -200) || (input == 0)){
						speed = input;
					}else{
						printf("\nInput speed is out of range. Please try again.\n\n");//speed is the same
					}

					if(speed != 0){
						brake = 130;	//no brakes
					}else{
						brake = 40; //100%
					}

					pub();

					
					

				}

				break;
			
			case 2:
				while(flag){
					printf(">> Please enter the steering angle [180 (<--LEFT) to (RIGHT-->) 130].\n");
					printf(">> To return to the previous menu, enter -1.\n\n");
					scanf(" %d",&input);

					if(input == -1){
						flag = 0;
						break;
					}else if(input >= 130 && input <= 180){
						steer = input;
					}else{
						printf("\nInput steer is out of range. Please try again.\n\n");
					}
					
					pub();
					
					

				}
				break;

			case 3:
				while(flag){
					printf(">> Please enter the braking angle [130 (<--0%% braking) to (100%% braking-->)40 ].\n");
					printf(">> To return to the previous menu, enter -1.\n\n");
					scanf(" %d",&input);

					if(input == -1){
						flag = 0;
						break;
					}else if(input >= 40 && input <= 130){
						brake = input;
						speed = 0;
					}else{
						printf("\nInput brake is out of range. Please try again.\n\n");
					}	
					pub();
					
				}
				
				break;
			
			case 4:
				printf("Steering Angle: %d\n",steer);
				printf("Braking Angle: %d\n",brake);
				break;

			default:
				printf("\nInvalid key. Please try again.\n\n");
				break;
		}
	

  	}

	void pub(){
		msg.data = steer;
		msg1.data = brake;
		msg2.data = speed;
	
	pub1_->publish(msg); //front right steer position
	pub2_->publish(msg1); // front right brake position	
	pub3_->publish(msg2); // front right torque
	}

  	void ax_callback(const std_msgs::msg::Float64 & msg) const
  	{
		if(key == 4){
			RCLCPP_INFO(this->get_logger(), "X Accel (g): '%.2f'", msg.data);  
		}
	    	    
  	}
 	void ay_callback(const std_msgs::msg::Float64 & msg) const
  	{
		if(key == 4){
	    RCLCPP_INFO(this->get_logger(), "Y Accel (g): '%.2f'", msg.data);  
		}
  	}
 	void az_callback(const std_msgs::msg::Float64 & msg) const
  	{
		if(key == 4){
	    RCLCPP_INFO(this->get_logger(), "Z Accel (g): '%.2f'", msg.data);  
		}
  	}
 	void gx_callback(const std_msgs::msg::Float64 & msg) const
  	{
		if(key == 4){
	    RCLCPP_INFO(this->get_logger(), "X Gyro (dps): '%.2f'", msg.data);  
		}
  	}
 	void gy_callback(const std_msgs::msg::Float64 & msg) const
  	{
		if(key == 4){
	    RCLCPP_INFO(this->get_logger(), "Y Gyro (dps): '%.2f'", msg.data);  
		}
  	}
 	void gz_callback(const std_msgs::msg::Float64 & msg) const
  	{
		if(key == 4){
	    RCLCPP_INFO(this->get_logger(), "Z Gyro (dps): '%.2f'", msg.data);  
		}
  	}

	void temp_callback(const std_msgs::msg::Float64 & msg) const
  	{
		if(key == 4){
	    RCLCPP_INFO(this->get_logger(), "Temperature (C): '%.2f'", msg.data); 
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
	// rclcpp::TimerBase::SharedPtr timer_ask;

};

int main(int argc, char * argv[])
{

  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<CentralController>());
  rclcpp::shutdown();
  return 0;
}
