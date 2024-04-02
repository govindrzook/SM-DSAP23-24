#include <functional>
#include <memory>
#include <fstream>
#include <iostream>
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "std_msgs/msg/float64.hpp"
#include "std_msgs/msg/u_int8.hpp"
#include </usr/include/libserial/SerialPort.h>
#include "hardware_libraries/SOLOUno.cpp"

using std::placeholders::_1;

int direction; // "Current" direction of the motor.
int lastTorqueCommand = 5000; // Impossible last speed for initial value.
int lastDirection = 2; // Impossible direction for initial value.

class FrontRightSOLO : public rclcpp::Node
{
public:
  FrontRightSOLO()
  : Node("front_right_solo")
  {
    front_right_torque_subscription = this->create_subscription<std_msgs::msg::Float64>(
      		"frontRightTorque", 10, std::bind(&FrontRightSOLO::front_right_torque_callback, this, _1));

    heartbeat_subscription = this->create_subscription<std_msgs::msg::UInt8>(
      		"heartbeat", 10, std::bind(&FrontRightSOLO::heartbeat_callback, this, _1));

    estop_subscription = this->create_subscription<std_msgs::msg::UInt8>(
      		"estop", 10, std::bind(&FrontRightSOLO::estop_callback, this, _1));

	  pub1_ = create_publisher<std_msgs::msg::Float64>("frontRightSpeed", 10);

    pub_error = create_publisher<std_msgs::msg::String>("error", 10);

	  timer_ = create_wall_timer(std::chrono::seconds(1), std::bind(&FrontRightSOLO::timer_callback, this));
    timer_heartbeat = create_wall_timer(std::chrono::seconds(1), std::bind(&FrontRightSOLO::heartbeat_timer_callback, this));

    soloPtr = new SoloUno(0x00); 

    //These three lines set the initial time for the e-stop mechanism.
    last = this->get_clock()->now();
    std::chrono::time_point<std::chrono::system_clock> time_point(std::chrono::seconds(static_cast<int>(last.seconds())));
    last_c = std::chrono::system_clock::to_time_t(time_point);
    
  }
  
private:

  SoloUno* soloPtr;
  
  rclcpp::Time last;
  std::time_t last_c;
  rclcpp::Time now;
  
  void updateLast(){
    last = this->get_clock()->now();
    std::chrono::time_point<std::chrono::system_clock> time_point(std::chrono::seconds(static_cast<int>(last.seconds())));
    last_c = std::chrono::system_clock::to_time_t(time_point);
  }
	void timer_callback()
  {
    auto msg = std_msgs::msg::Float64();
    msg.data = soloPtr->readSpeed();  
    pub1_->publish(msg); 
  }

  void estop_callback(const std_msgs::msg::UInt8 & msg){
    RCLCPP_INFO(this->get_logger(), "E-STOP");
    auto msg_error = std_msgs::msg::String();
    msg_error.data = "FR-SOLO E-stop activated";
    pub_error->publish(msg_error);
   
  }

  void heartbeat_timer_callback(){
    now = this->get_clock()->now();

    // Convert rclcpp::Time to std::chrono::system_clock::time_point
    std::chrono::time_point<std::chrono::system_clock> time_point(std::chrono::seconds(static_cast<int>(now.seconds())));

    // Convert the timepoint to a time_t
    std::time_t now_c = std::chrono::system_clock::to_time_t(time_point);
    std::time_t duration = now_c - last_c;
    if(duration > 1){
      RCLCPP_INFO(this->get_logger(), "E-STOP"); 
      auto msg_error = std_msgs::msg::String();
      msg_error.data = "FR-SOLO E-stop activated";
      pub_error->publish(msg_error);
      
      soloPtr->emergencyStop();  
    }
  }

  void heartbeat_callback(const std_msgs::msg::UInt8 & msg){
     this->updateLast();
  }

  void front_right_torque_callback(const std_msgs::msg::Float64 & msg) const{
	  RCLCPP_INFO(this->get_logger(), "SOLO received torque command: '%f'", msg.data); 

      if(msg.data >= 1){
        direction = 1; 
      }
      else if(msg.data < 0){ 
        direction = 0; 
      }    

      if(direction != lastDirection){ // Ensure that the current command is not the same as the last to avoid unneccessary writes.

        int directionResult = soloPtr->setDirectionSlow(direction);
        if(directionResult == 1){
          auto msg_error = std_msgs::msg::String();
          msg_error.data = "FR-SOLO direction setting failed";
          pub_error->publish(msg_error);
        }
        else{
          lastDirection = direction;
        }   
      }

      if(msg.data != lastTorqueCommand){ // Ensure that the current command is not the same as the last to avoid unneccessary writes.
        int result = soloPtr->setTorqueSlow(abs(msg.data));
        if(result == 1){
    
          auto msg_error = std_msgs::msg::String();
          msg_error.data = "FR-SOLO torque setting failed";
          pub_error->publish(msg_error);
          
        }
        else{
          lastTorqueCommand = (int) msg.data;
        }
        
      }
  
  }
 
	rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr front_right_torque_subscription;
  rclcpp::Subscription<std_msgs::msg::UInt8>::SharedPtr heartbeat_subscription;
  rclcpp::Subscription<std_msgs::msg::UInt8>::SharedPtr estop_subscription;
	rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr pub1_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr pub_error;
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::TimerBase::SharedPtr timer_heartbeat;

};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<FrontRightSOLO>());
  rclcpp::shutdown();
  return 0;
}
