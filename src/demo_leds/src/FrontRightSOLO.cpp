
#include <functional>
#include <memory>

#include <fstream>
#include <iostream>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "std_msgs/msg/float64.hpp"

// #include </usr/local/include/libserial/SerialPort.h>
#include </usr/include/libserial/SerialPort.h>
#include "SOLOUno.cpp"

using std::placeholders::_1;

int direction;
int lastSpeedCommand = 5000; // Impossible last speed for initial value.
int lastDirection = 2; // Impossible direction for initial value.


class FrontRightSOLO : public rclcpp::Node
{
public:
  FrontRightSOLO()
  : Node("front_right_solo")
  {
    	front_right_torque_subscription = this->create_subscription<std_msgs::msg::Float64>(
      		"frontRightTorque", 10, std::bind(&FrontRightSOLO::front_right_torque_callback, this, _1));

	  pub1_ = create_publisher<std_msgs::msg::Float64>("frontRightSpeed", 10);

	  timer_ = create_wall_timer(std::chrono::seconds(1), std::bind(&FrontRightSOLO::timer_callback, this));
    soloPtr = new SoloUno(0x00);
  }
  
private:

  SoloUno* soloPtr;
  
	void timer_callback()
  {
    auto msg = std_msgs::msg::Float64();

    msg.data = soloPtr->readSpeed();
 
    //RCLCPP_INFO(this->get_logger(), "Speed reading: '%f'", msg.data); 
    pub1_->publish(msg); 
  }

  void front_right_torque_callback(const std_msgs::msg::Float64 & msg) const
  {
	  RCLCPP_INFO(this->get_logger(), "SOLO received speed command: '%f'", msg.data); 

      if(msg.data >= 1){
        //printf("Direction is 1 from ths speed.\n");
        direction = 1;
      }
      else{
        //printf("Direction is 0 from ths speed.\n");
        direction = 0;
      }

      if(direction != lastDirection){ // Ensure that the current command is not the same as the last to avoid unneccessary writes.
        soloPtr->setDirectionSlow(direction);
        //printf("Unique direction has been written.\n");
        //soloPtr->setDirectionFast(direction);
        lastDirection = direction;
      }
      else{
        //RCLCPP_INFO(this->get_logger(), "SOLO received consecutive direction commands");
      }

      if(msg.data != lastSpeedCommand){ // Ensure that the current command is not the same as the last to avoid unneccessary writes.
        //int result = soloPtr->setSpeedSlow(msg.data);
        int result = soloPtr->setSpeedSlow(100);
        //soloPtr->setSpeedFast(msg.data);
        lastSpeedCommand = (int) msg.data;
        //printf("Unique speed has been written.\n");
      }
      else{
        //RCLCPP_INFO(this->get_logger(), "SOLO received consecutive speed command of: ", msg.data);
      }
  
  }
 
	rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr front_right_torque_subscription;
	rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr pub1_;
  rclcpp::TimerBase::SharedPtr timer_;

};

int main(int argc, char * argv[])
{

  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<FrontRightSOLO>());
  rclcpp::shutdown();

  return 0;
}
