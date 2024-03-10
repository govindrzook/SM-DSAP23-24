
#include <functional>
#include <memory>

#include <fstream>
#include <iostream>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "std_msgs/msg/float64.hpp"

#include </usr/local/include/libserial/SerialPort.h>
//#include </usr/include/libserial/SerialPort.h>
#include "SOLOUno.cpp"

using std::placeholders::_1;

class FrontRightSOLO : public rclcpp::Node
{
public:
  FrontRightSOLO()
  : Node("front_right_solo")
  {
    	front_right_torque_subscription = this->create_subscription<std_msgs::msg::Float64>(
      		"frontRightTorque", 10, std::bind(&FrontRightSOLO::front_right_torque_callback, this, _1));

	  pub1_ = create_publisher<std_msgs::msg::Float64>("frontRightSpeed", 10);

	  timer_ = create_wall_timer(std::chrono::seconds(3), std::bind(&FrontRightSOLO::timer_callback, this));
    soloPtr = new SoloUno(0x00);
  }

private:

  SoloUno* soloPtr;

	void timer_callback()
  {
    auto msg = std_msgs::msg::Float64();

    msg.data = rand() % 30000;  // Example data, replace with your actual data
    //msg.data = soloPtr->readSpeed();
    pub1_->publish(msg); 
  }

  void front_right_torque_callback(const std_msgs::msg::Float64 & msg) const
  {
	    	RCLCPP_INFO(this->get_logger(), "Received speed command: '%f'", msg.data); 
        //int result = soloPtr->setSpeedSlow(msg.data); 

        //if(result){
        //    RCLCPP_INFO(this->get_logger(), "Error code: '%d'", result);
        //}     
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
