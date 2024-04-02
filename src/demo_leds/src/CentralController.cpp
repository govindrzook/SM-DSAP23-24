#include <functional>
#include <memory>
#include <fstream>
#include <iostream>
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "std_msgs/msg/float64.hpp"
#include "std_msgs/msg/u_int8.hpp"

using std::placeholders::_1;

double frTorque;
double frSpeedFeedback;
double flTorque;
double flSpeedFeedback;
double brTorque;
double brSpeedFeedback;
double blTorque;
double blSpeedFeedback;

double frSteer;
double frBrake;
double flSteer;
double flBrake;
double brSteer;
double brBrake;
double blSteer;
double blBrake;

double aX;
double aY;
double aZ;
double gX;
double gY;
double gZ;
double temperature;

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
	front_left_speed_subscription = this->create_subscription<std_msgs::msg::Float64>(
      		"frontLeftSpeed", 10, std::bind(&CentralController::front_left_speed_callback, this, _1));
	back_right_speed_subscription = this->create_subscription<std_msgs::msg::Float64>(
      		"backRightSpeed", 10, std::bind(&CentralController::back_right_speed_callback, this, _1));
	back_left_speed_subscription = this->create_subscription<std_msgs::msg::Float64>(
      		"backLeftSpeed", 10, std::bind(&CentralController::back_left_speed_callback, this, _1));

	pub_frSteer = create_publisher<std_msgs::msg::UInt8>("frontRightSteerPosition", 10);
   	pub_frBrake = create_publisher<std_msgs::msg::UInt8>("frontRightBrakePosition", 10);

	pub_flSteer = create_publisher<std_msgs::msg::UInt8>("frontLeftSteerPosition", 10);
   	pub_flBrake = create_publisher<std_msgs::msg::UInt8>("frontLeftBrakePosition", 10);

	pub_brSteer = create_publisher<std_msgs::msg::UInt8>("backRightSteerPosition", 10);
   	pub_brBrake = create_publisher<std_msgs::msg::UInt8>("backRightBrakePosition", 10);

	pub_blSteer = create_publisher<std_msgs::msg::UInt8>("backLeftSteerPosition", 10);
   	pub_blBrake = create_publisher<std_msgs::msg::UInt8>("backLeftBrakePosition", 10);
	
	pub_frTorque = create_publisher<std_msgs::msg::Float64>("frontRightTorque", 10);
	pub_flTorque = create_publisher<std_msgs::msg::Float64>("frontLeftTorque", 10);
	pub_brTorque = create_publisher<std_msgs::msg::Float64>("backRightTorque", 10);
	pub_blTorque = create_publisher<std_msgs::msg::Float64>("backLeftTorque", 10);

	pub_heartbeat = create_publisher<std_msgs::msg::UInt8>("heartbeat", 10); 
	pub_estop = create_publisher<std_msgs::msg::UInt8>("estop", 10);

	timer_ = create_wall_timer(std::chrono::seconds(1), std::bind(&CentralController::timer_callback, this));
	timer_heartbeat = create_wall_timer(std::chrono::milliseconds(250), std::bind(&CentralController::heartbeat_callback, this));

  }

private:

	void heartbeat_callback(){

		auto msg = std_msgs::msg::UInt8();
		msg.data = 1;
		pub_heartbeat->publish(msg);
		
	}

	void timer_callback(){ // This one timer callback sends all commands at once, meaning system tinme is defined by this timer. It could be useful to split these into multiple timers
						   // if some data points need to be updated faster than others.

	// Create messages to publish
	
		auto msg_frBrake = std_msgs::msg::UInt8();
		auto msg_frSteer = std_msgs::msg::UInt8();
		auto msg_flBrake = std_msgs::msg::UInt8();
		auto msg_flSteer = std_msgs::msg::UInt8();
		auto msg_brBrake = std_msgs::msg::UInt8();
		auto msg_brSteer = std_msgs::msg::UInt8();
		auto msg_blBrake = std_msgs::msg::UInt8();
		auto msg_blSteer = std_msgs::msg::UInt8();

		auto msg_frTorque = std_msgs::msg::Float64();
		auto msg_flTorque = std_msgs::msg::Float64();
		auto msg_brTorque = std_msgs::msg::Float64();
		auto msg_blTorque = std_msgs::msg::Float64();

	// Assign global variables to the message data.
		msg_frTorque.data = frTorque;
		msg_flTorque.data = flTorque;
		msg_brTorque.data = brTorque;
		msg_blTorque.data = blTorque;

		msg_frBrake.data = frBrake;
		msg_frSteer.data = frSteer;
		msg_flBrake.data = flBrake;
		msg_flSteer.data = flSteer; 
		msg_brBrake.data = brBrake;
		msg_brSteer.data = brSteer; 
		msg_blBrake.data = blBrake;
		msg_blSteer.data = blSteer; 

		// Publish the messages to each topic to control motors.
		
		pub_frTorque->publish(msg_frTorque); //  Publishes the front right torque
		pub_flTorque->publish(msg_flTorque); //  Publishes the front left torque
		pub_brTorque->publish(msg_brTorque); //  Publishes the back right torque
		pub_blTorque->publish(msg_blTorque); //  Publishes the back left torque

		pub_frBrake->publish(msg_frBrake); // front right brake position
		pub_frSteer->publish(msg_frSteer); //front right steer position
		pub_frBrake->publish(msg_flBrake); // front left brake position
		pub_frSteer->publish(msg_flSteer); //front left steer position
		pub_frBrake->publish(msg_brBrake); // back right brake position
		pub_frSteer->publish(msg_brSteer); //back right steer position
		pub_frBrake->publish(msg_blBrake); // back left brake position
		pub_frSteer->publish(msg_blSteer); //back left steer position

  	}

  	void ax_callback(const std_msgs::msg::Float64 & msg) const
  	{
	    aX = msg.data;   
		    
  	}
 	void ay_callback(const std_msgs::msg::Float64 & msg) const
  	{
	    aY = msg.data;   
		    
  	}
 	void az_callback(const std_msgs::msg::Float64 & msg) const
  	{
	    aZ = msg.data;   
		    
  	}
 	void gx_callback(const std_msgs::msg::Float64 & msg) const
  	{
	    gX = msg.data;   
		    
  	}
 	void gy_callback(const std_msgs::msg::Float64 & msg) const
  	{
	    gY = msg.data;   
		    
  	}
 	void gz_callback(const std_msgs::msg::Float64 & msg) const
  	{
	    gZ = msg.data; 
		    
  	}

	void temp_callback(const std_msgs::msg::Float64 & msg) const
  	{
	    temperature = msg.data;    
  	}
  	void front_right_speed_callback(const std_msgs::msg::Float64 & msg) const
  	{
		frSpeedFeedback = msg.data; 	    
  	}

	void front_left_speed_callback(const std_msgs::msg::Float64 & msg) const
  	{
		flSpeedFeedback = msg.data; 	    
  	}

	void back_right_speed_callback(const std_msgs::msg::Float64 & msg) const
  	{
		brSpeedFeedback = msg.data; 	    
  	}

	void back_left_speed_callback(const std_msgs::msg::Float64 & msg) const
  	{
		blSpeedFeedback = msg.data; 	    
  	}

	rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr ax_subscription;
	rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr ay_subscription;
	rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr az_subscription;

	rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr gx_subscription;
	rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr gy_subscription;
	rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr gz_subscription;

	rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr temp_subscription;
	
	rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr front_right_speed_subscription;
	rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr front_left_speed_subscription;
	rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr back_right_speed_subscription;
	rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr back_left_speed_subscription;

	rclcpp::Publisher<std_msgs::msg::UInt8>::SharedPtr pub_frSteer;
	rclcpp::Publisher<std_msgs::msg::UInt8>::SharedPtr pub_frBrake;
	rclcpp::Publisher<std_msgs::msg::UInt8>::SharedPtr pub_flSteer;
	rclcpp::Publisher<std_msgs::msg::UInt8>::SharedPtr pub_flBrake;
	rclcpp::Publisher<std_msgs::msg::UInt8>::SharedPtr pub_brSteer;
	rclcpp::Publisher<std_msgs::msg::UInt8>::SharedPtr pub_brBrake;
	rclcpp::Publisher<std_msgs::msg::UInt8>::SharedPtr pub_blSteer;
	rclcpp::Publisher<std_msgs::msg::UInt8>::SharedPtr pub_blBrake;

	rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr pub_frTorque;
	rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr pub_flTorque;
	rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr pub_brTorque;
	rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr pub_blTorque;

	rclcpp::Publisher<std_msgs::msg::UInt8>::SharedPtr pub_heartbeat;
	rclcpp::Publisher<std_msgs::msg::UInt8>::SharedPtr pub_estop;

  	rclcpp::TimerBase::SharedPtr timer_;
	rclcpp::TimerBase::SharedPtr timer_heartbeat;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<CentralController>());
  rclcpp::shutdown();
  return 0;
}
