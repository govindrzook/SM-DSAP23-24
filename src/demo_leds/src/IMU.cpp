#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/int32.hpp"
#include "std_msgs/msg/string.hpp"

class IMU : public rclcpp::Node
{
public:
  IMU()
    : Node("imu_node")
  {
    // Create publishers for six different topics
    pub1_ = create_publisher<std_msgs::msg::String>("aX", 10);
    pub2_ = create_publisher<std_msgs::msg::String>("aY", 10);
    pub3_ = create_publisher<std_msgs::msg::String>("aZ", 10);
    pub4_ = create_publisher<std_msgs::msg::String>("gX", 10);
    pub5_ = create_publisher<std_msgs::msg::String>("gY", 10);
    pub6_ = create_publisher<std_msgs::msg::String>("gZ", 10);

    // Create a timer to update the topics every 1 ms
    timer_ = create_wall_timer(std::chrono::milliseconds(1), std::bind(&IMU::timer_callback, this));
  }

private:
  void timer_callback()
  {
    // Create a message to publish
    auto msg = std_msgs::msg::String();
    auto msg1 = std_msgs::msg::String();
    auto msg2 = std_msgs::msg::String();
    auto msg3 = std_msgs::msg::String();
    auto msg4 = std_msgs::msg::String();
    auto msg5 = std_msgs::msg::String();

    msg.data = "0g";  // Example data, replace with your actual data
    msg1.data = "1g";  // Example data, replace with your actual data
    msg2.data = "2g";  // Example data, replace with your actual data

    msg3.data = "3g";  // Example data, replace with your actual data
    msg4.data = "4g";  // Example data, replace with your actual data
    msg5.data = "5g";  // Example data, replace with your actual data

    // Publish to each topic
    pub1_->publish(msg);
    pub2_->publish(msg1);
    pub3_->publish(msg2);
    pub4_->publish(msg3);
    pub5_->publish(msg4);
    pub6_->publish(msg5);
  }

  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr pub1_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr pub2_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr pub3_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr pub4_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr pub5_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr pub6_;

  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char *argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<IMU>());
  rclcpp::shutdown();
  return 0;
}
