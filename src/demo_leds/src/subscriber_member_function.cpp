// Copyright 2016 Open Source Robotics Foundation, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include <functional>
#include <memory>

#include <fstream>
#include <iostream>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

using std::placeholders::_1;

class MinimalSubscriber : public rclcpp::Node
{
public:
  MinimalSubscriber()
  : Node("minimal_subscriber")
  {
    subscription_ = this->create_subscription<std_msgs::msg::String>(
      "topic", 10, std::bind(&MinimalSubscriber::topic_callback, this, _1));
  }

private:

	
    	std::string pwm0_enable = "/sys/class/pwm/pwmchip0/pwm0/enable";
		std::string pwm0_duty_cycle = "/sys/class/pwm/pwmchip0/pwm0/duty_cycle";
		std::string pwm0_period = "/sys/class/pwm/pwmchip0/pwm0/period";
		
		std::string pwm1_enable = "/sys/class/pwm/pwmchip0/pwm1/enable";
		std::string pwm1_duty_cycle = "/sys/class/pwm/pwmchip0/pwm1/duty_cycle";
		std::string pwm1_period = "/sys/class/pwm/pwmchip0/pwm0/period";


		
    	 std::string output_file0 = "/sys/class/leds/beaglebone:green:usr0/brightness";
		 std::string output_file1 = "/sys/class/leds/beaglebone:green:usr1/brightness";
		 std::string output_file2 = "/sys/class/leds/beaglebone:green:usr2/brightness";
		 std::string output_file3 = "/sys/class/leds/beaglebone:green:usr3/brightness";
		 std::string output_file4 = "/sys/class/leds/beaglebone:green:usr4/brightness";

  void topic_callback(const std_msgs::msg::String & msg) const
  {

		std::ofstream file0(output_file0, std::ios::app);
        std::ofstream file1(output_file1, std::ios::app);
        std::ofstream file2(output_file2, std::ios::app);
        std::ofstream file3(output_file3, std::ios::app);
        std::ofstream file4(output_file4, std::ios::app);

		std::ofstream pwm0_en(pwm0_enable, std::ios::app);
		std::ofstream pwm0_dut(pwm0_duty_cycle, std::ios::app);
		std::ofstream pwm0_per(pwm0_period, std::ios::app);
		
		std::ofstream pwm1_en(pwm1_enable, std::ios::app);
        std::ofstream pwm1_dut(pwm1_duty_cycle, std::ios::app);
        std::ofstream pwm1_per(pwm1_period, std::ios::app);
		
		pwm0_en << 1;
		pwm1_en << 1;

		pwm0_per << 20000000;
		pwm1_per << 20000000;

  	if(std::stoi(msg.data) == 0){
  	
			pwm0_dut << 500000;
			pwm1_dut << 500000;
			
			pwm1_dut << 500;
			
			file0 << 1;
			file1 << 0;
			file2 << 0;
			file3 << 0;
			file4 << 0;

	    	RCLCPP_INFO(this->get_logger(), "Setting servo to position: '%s'", msg.data.c_str());
    
    	}
    	
    	if(std::stoi(msg.data) == 1){
  	
			pwm0_dut << 1000000;
			pwm1_dut << 1000000;

	    	file0 << 0;
            file1 << 1;
            file2 << 0;
            file3 << 0;
        	file4 << 0;

		RCLCPP_INFO(this->get_logger(), "Setting servo to position: '%s'", msg.data.c_str());
    
    	}
    	
    	if(std::stoi(msg.data) == 2){
			
			pwm0_dut << 1500000;
			pwm1_dut << 1500000;

	  		file0 << 0;
            file1 << 0;
            file2 << 1;
            file3 << 0;
            file4 << 0;
	  	
	    	RCLCPP_INFO(this->get_logger(), "Setting servo to position: '%s'", msg.data.c_str());
    
    	}
    	
    	if(std::stoi(msg.data) == 3){
			
			pwm0_dut << 2000000;
			pwm1_dut << 2000000;

			file0 << 0;
            file1 << 0;
            file2 << 0;
            file3 << 1;
            file4 << 0;
	  	
	    	RCLCPP_INFO(this->get_logger(), "Setting servo to position: '%s'", msg.data.c_str());
    
    	}

		if(std::stoi(msg.data) == 4){

			pwm0_dut << 2500000;
			pwm1_dut << 2500000;

		 	file0 << 0;
            file1 << 0;
            file2 << 0;
            file3 << 0;
            file4 << 1;

            RCLCPP_INFO(this->get_logger(), "Setting servo to position: '%s'", msg.data.c_str());

        }
  }

  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr subscription_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<MinimalSubscriber>());
  rclcpp::shutdown();
  return 0;
}
