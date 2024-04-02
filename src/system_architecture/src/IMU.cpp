
#include <functional>
#include <memory>
#include <fstream>
#include <iostream>
#include <stdio.h>

#include "hardware_libraries/SysModel_IMU.cpp"

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/int32.hpp"
#include "std_msgs/msg/u_int8.hpp"
#include "std_msgs/msg/float64.hpp"
#include "std_msgs/msg/string.hpp"

/*struct SensorData {
    float x_accel_g;
    float y_accel_g;
    float z_accel_g;
    float x_gyro_dps;
    float y_gyro_dps;
    float z_gyro_dps;
    float temp_c;
};*/

class IMU : public rclcpp::Node
{
public:
  IMU(): Node("imu_node")
  {
    // Create publishers for six different topics
    //publisher_= this->create_publisher<std_msgs::msg::SensorData
    pub1_ = create_publisher<std_msgs::msg::Float64>("aX", 10);
    pub2_ = create_publisher<std_msgs::msg::Float64>("aY", 10);
    pub3_ = create_publisher<std_msgs::msg::Float64>("aZ", 10);
    pub4_ = create_publisher<std_msgs::msg::Float64>("gX", 10);
    pub5_ = create_publisher<std_msgs::msg::Float64>("gY", 10);
    pub6_ = create_publisher<std_msgs::msg::Float64>("gZ", 10);
    pub7_ = create_publisher<std_msgs::msg::Float64>("temp", 10);

    // Create a timer to update the topics every 1 ms
    timer_ = create_wall_timer(std::chrono::seconds(1), std::bind(&IMU::timer_callback, this));
    
  }

private: 
  SysModel_IMU imu_obj;

  void timer_callback()
  {
    // Create a message to publish
    auto msg = std_msgs::msg::Float64();
    auto msg1 = std_msgs::msg::Float64();
    auto msg2 = std_msgs::msg::Float64();
    auto msg3 = std_msgs::msg::Float64();
    auto msg4 = std_msgs::msg::Float64();
    auto msg5 = std_msgs::msg::Float64();
    auto msg6 = std_msgs::msg::Float64();

    //imu_obj

    imu_obj.read_and_convert_sensor_data();

    msg.data = imu_obj.x_accel_g;  // Example data, replace with your actual data
    msg1.data = imu_obj.y_accel_g;  // Example data, replace with your actual data
    msg2.data = imu_obj.z_accel_g;  // Example data, replace with your actual data
    msg3.data = imu_obj.x_gyro_dps;  // Example data, replace with your actual data
    msg4.data = imu_obj.y_gyro_dps;  // Example data, replace with your actual data
    msg5.data = imu_obj.z_gyro_dps;  // Example data, replace with your actual data
    msg6.data = imu_obj.temp_c;  // Example data, replace with your actual data

    // Publish to each topic
    pub1_->publish(msg);
    pub2_->publish(msg1);
    pub3_->publish(msg2);
    pub4_->publish(msg3);
    pub5_->publish(msg4);
    pub6_->publish(msg5);
    pub7_->publish(msg6);
  }

  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr pub1_;
  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr pub2_;
  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr pub3_;
  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr pub4_;
  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr pub5_;
  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr pub6_;
  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr pub7_;
  
  //RCLCPP_INFO(this->get_logger(), "Publishing sensor data");

  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char *argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<IMU>());
  rclcpp::shutdown();
  return 0;
}
