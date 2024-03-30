
#include <functional>
#include <memory>

#include <fstream>
#include <iostream>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "std_msgs/msg/float64.hpp"
#include "std_msgs/msg/u_int8.hpp"

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

    heartbeat_subscription = this->create_subscription<std_msgs::msg::UInt8>(
      		"heartbeat", 10, std::bind(&FrontRightSOLO::heartbeat_callback, this, _1));

    estop_subscription = this->create_subscription<std_msgs::msg::UInt8>(
      		"estop", 10, std::bind(&FrontRightSOLO::estop_callback, this, _1));

	  pub1_ = create_publisher<std_msgs::msg::Float64>("frontRightSpeed", 10);

	  timer_ = create_wall_timer(std::chrono::seconds(1), std::bind(&FrontRightSOLO::timer_callback, this));
    timer_heartbeat = create_wall_timer(std::chrono::seconds(1), std::bind(&FrontRightSOLO::heartbeat_timer_callback, this));

    soloPtr = new SoloUno(0x00); 

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
 
    //RCLCPP_INFO(this->get_logger(), "Speed reading: '%f'", msg.data); 
    pub1_->publish(msg); 
  }
  void estop_callback(const std_msgs::msg::UInt8 & msg){

    RCLCPP_INFO(this->get_logger(), "E-STOP"); 
    soloPtr->emergencyStop();

  }
  void heartbeat_timer_callback()
  {

    now = this->get_clock()->now();

    // Convert rclcpp::Time to std::chrono::system_clock::time_point
    std::chrono::time_point<std::chrono::system_clock> time_point(std::chrono::seconds(static_cast<int>(now.seconds())));

    // Convert the timepoint to a time_t
    std::time_t now_c = std::chrono::system_clock::to_time_t(time_point);
    RCLCPP_INFO(this->get_logger(), "now '%ld'", now_c);
    RCLCPP_INFO(this->get_logger(), "last '%ld'", last_c);
    std::time_t duration = now_c - last_c;
    RCLCPP_INFO(this->get_logger(), "duration '%ld'", duration);
    if(duration > 1){
      RCLCPP_INFO(this->get_logger(), "E-STOP"); 
      soloPtr->emergencyStop();  
    }

  }

  void heartbeat_callback(const std_msgs::msg::UInt8 & msg){
    RCLCPP_INFO(this->get_logger(), "HEARTBEAT TOPIC CALLBACK");
     this->updateLast();
    RCLCPP_INFO(this->get_logger(), "last '%ld'", last_c);
  }

  void front_right_torque_callback(const std_msgs::msg::Float64 & msg) const
  {
	  RCLCPP_INFO(this->get_logger(), "SOLO received speed command: '%f'", msg.data); 

      if(msg.data >= 1){
        //printf("Direction is 1 from ths speed.\n");
        direction = 1;
        //soloPtr->setDirectionSlow(direction);
      }
      else if(msg.data < 0){
        //printf("Direction is 0 from ths speed.\n");
        direction = 0;
        //soloPtr->setDirectionSlow(direction);
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

        int result = soloPtr->setSpeedSlow(abs(msg.data)); 
        //soloPtr->setSpeedFast(msg.data);
        lastSpeedCommand = (int) msg.data;
        //printf("Unique speed has been written.\n");
      }
      else{
        //RCLCPP_INFO(this->get_logger(), "SOLO received consecutive speed command of: ", msg.data);
      }
  
  }
 
	rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr front_right_torque_subscription;
  rclcpp::Subscription<std_msgs::msg::UInt8>::SharedPtr heartbeat_subscription;
  rclcpp::Subscription<std_msgs::msg::UInt8>::SharedPtr estop_subscription;
	rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr pub1_;
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
