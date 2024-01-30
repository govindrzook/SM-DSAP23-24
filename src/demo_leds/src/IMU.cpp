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
    pub1_ = create_publisher<std_msgs::msg::Int32>("aX", 10);
    pub2_ = create_publisher<std_msgs::msg::Int32>("aY", 10);
    pub3_ = create_publisher<std_msgs::msg::Int32>("aZ", 10);
    pub4_ = create_publisher<std_msgs::msg::Int32>("gX", 10);
    pub5_ = create_publisher<std_msgs::msg::Int32>("gY", 10);
    pub6_ = create_publisher<std_msgs::msg::Int32>("gZ", 10);

    // Create a timer to update the topics every 1 ms
    timer_ = create_wall_timer(std::chrono::milliseconds(1), std::bind(&IMU::timer_callback, this));
  }

private:
  void timer_callback()
  {
    // Create a message to publish
    auto msg = std_msgs::msg::String()
    msg.data = "Sample IMU data";  // Example data, replace with your actual data

    // Publish to each topic
    pub1_->publish(msg);
    pub2_->publish(msg);
    pub3_->publish(msg);
    pub4_->publish(msg);
    pub5_->publish(msg);
    pub6_->publish(msg);
  }

  rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr pub1_;
  rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr pub2_;
  rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr pub3_;
  rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr pub4_;
  rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr pub5_;
  rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr pub6_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char *argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<IMU>());
  rclcpp::shutdown();
  return 0;
}
