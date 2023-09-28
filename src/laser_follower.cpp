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
#include <chrono>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "irobot_create_msgs/msg/hazard_detection_vector.hpp"
#include "geometry_msgs/msg/twist.hpp"

using std::placeholders::_1;
using namespace std::chrono_literals;

class Wanderer : public rclcpp::Node
{
public:
  Wanderer()
      : Node("wanderer")
  {
    subscription_wanderer = this->create_subscription<irobot_create_msgs::msg::HazardDetectionVector>(
        "yoshi/hazard_detection", rclcpp::SensorDataQoS(), std::bind(&Wanderer::hazard_callback, this, _1));
    publisher_ = this->create_publisher<geometry_msgs::msg::Twist>("yoshi/cmd_vel", 10);
    forward_timer_ = this->create_wall_timer(
        500ms, std::bind(&Wanderer::forward_callback, this));
    spin_timer_ = this->create_wall_timer(
        500ms, std::bind(&Wanderer::spin_callback, this));
    spin_timer_->cancel();
    reverse_timer_ = this->create_wall_timer(
        1.5s, std::bind(&Wanderer::reverse_callback, this));
    reverse_timer_->cancel();
    reverse_stop_timer_ = this->create_wall_timer(
        3s, std::bind(&Wanderer::reverse_stop_callback, this));
    reverse_stop_timer_->cancel();
    forward_stop_timer_ = this->create_wall_timer(
        3s, std::bind(&Wanderer::forward_stop_callback, this));
    forward_stop_timer_->cancel();

    in_process = false;
    reverse_done = false;
    spin_done = false;
    state = idle;
  }

private:
  int random_between_two_int(int min, int max)
  {
    return rand() % (max - min) + min + 1;
  }

  void topic_callback(const std_msgs::msg::String &msg) const
  {
    RCLCPP_INFO(this->get_logger(), "I heard: '%s'", msg.data.c_str());
  }

  void hazard_callback(const irobot_create_msgs::msg::HazardDetectionVector &msg)
  {
    if (state == idle)
    {
      bool hazard = false;
      RCLCPP_INFO(this->get_logger(), "Size of vector: '%ld'", msg.detections.size());
      for (irobot_create_msgs::msg::HazardDetection hd : msg.detections)
      {
        RCLCPP_INFO(this->get_logger(), "Bump: '%d'", hd.type);
        if (hd.type == hd.BUMP)
        {
          hazard = true;
        }
      }
      if (hazard)
      {
        forward_stop_callback();
        state = reverse;
      }
    }
    else if (state == reverse)
    {
      reverse_timer_->reset();
      reverse_stop_timer_->reset();
      state = during_process;
    }
    else if (state == spinn)
    {
      spin_timer_->reset();
      int duration = random_between_two_int(1, 5);
      spin_stop_timer_ = this->create_wall_timer(
          std::chrono::seconds(duration), std::bind(&Wanderer::spin_stop_callback, this));
      state = during_process;
    }
    else if (state == go_forward)
    {
      forward_timer_->reset();
      state = idle;
    }
    else
    {
      // do nothing during process
    }
  }
  void full_stop_callback()
  {
    auto twist_command = geometry_msgs::msg::Twist();
    twist_command.linear.x = 0.0;
    twist_command.linear.y = 0.0;
    twist_command.linear.z = 0.0;
    twist_command.angular.x = 0.0;
    twist_command.angular.y = 0.0;
    twist_command.angular.z = 0.0;
    publisher_->publish(twist_command);
    RCLCPP_INFO(this->get_logger(), "Twist command linear x '%f", twist_command.linear.x);
    RCLCPP_INFO(this->get_logger(), "Twist command linear y '%f", twist_command.linear.y);
    RCLCPP_INFO(this->get_logger(), "Twist command linear z '%f", twist_command.linear.z);
    RCLCPP_INFO(this->get_logger(), "Twist command angular x '%f", twist_command.angular.x);
    RCLCPP_INFO(this->get_logger(), "Twist command angular y '%f", twist_command.angular.y);
    RCLCPP_INFO(this->get_logger(), "Twist command angular z '%f", twist_command.angular.z);
  }

  void spin_callback()
  {
    auto twist_command = geometry_msgs::msg::Twist();
    twist_command.linear.x = 0.0;
    twist_command.linear.y = 0.0;
    twist_command.linear.z = 0.0;
    twist_command.angular.x = 0.0;
    twist_command.angular.y = 0.0;
    twist_command.angular.z = 0.5;
    publisher_->publish(twist_command);
    RCLCPP_INFO(this->get_logger(), "Twist command linear x '%f", twist_command.linear.x);
    RCLCPP_INFO(this->get_logger(), "Twist command linear y '%f", twist_command.linear.y);
    RCLCPP_INFO(this->get_logger(), "Twist command linear z '%f", twist_command.linear.z);
    RCLCPP_INFO(this->get_logger(), "Twist command angular x '%f", twist_command.angular.x);
    RCLCPP_INFO(this->get_logger(), "Twist command angular y '%f", twist_command.angular.y);
    RCLCPP_INFO(this->get_logger(), "Twist command angular z '%f", twist_command.angular.z);
  }

  void spin_stop_callback()
  {
    spin_timer_->cancel();
    spin_stop_timer_->cancel();
    // spin_done = true;
    state = go_forward;

    auto twist_command = geometry_msgs::msg::Twist();

    twist_command.linear.x = 0.0;
    twist_command.linear.y = 0.0;
    twist_command.linear.z = 0.0;
    twist_command.angular.x = 0.0;
    twist_command.angular.y = 0.0;
    twist_command.angular.z = 0.0;
    publisher_->publish(twist_command);
    RCLCPP_INFO(this->get_logger(), "Twist command linear x '%f", twist_command.linear.x);
    RCLCPP_INFO(this->get_logger(), "Twist command linear y '%f", twist_command.linear.y);
    RCLCPP_INFO(this->get_logger(), "Twist command linear z '%f", twist_command.linear.z);
    RCLCPP_INFO(this->get_logger(), "Twist command angular x '%f", twist_command.angular.x);
    RCLCPP_INFO(this->get_logger(), "Twist command angular y '%f", twist_command.angular.y);
    RCLCPP_INFO(this->get_logger(), "Twist command angular z '%f", twist_command.angular.z);
  }

  void reverse_callback()
  {

    auto twist_command = geometry_msgs::msg::Twist();
    twist_command.linear.x = -0.0001;
    twist_command.linear.y = 0.0;
    twist_command.linear.z = 0.0;
    twist_command.angular.x = 0.0;
    twist_command.angular.y = 0.0;
    twist_command.angular.z = 0.0;
    publisher_->publish(twist_command);
    RCLCPP_INFO(this->get_logger(), "Twist command linear x '%f", twist_command.linear.x);
    RCLCPP_INFO(this->get_logger(), "Twist command linear y '%f", twist_command.linear.y);
    RCLCPP_INFO(this->get_logger(), "Twist command linear z '%f", twist_command.linear.z);
    RCLCPP_INFO(this->get_logger(), "Twist command angular x '%f", twist_command.angular.x);
    RCLCPP_INFO(this->get_logger(), "Twist command angular y '%f", twist_command.angular.y);
    RCLCPP_INFO(this->get_logger(), "Twist command angular z '%f", twist_command.angular.z);
  }

  void reverse_stop_callback()
  {
    reverse_timer_->cancel();
    reverse_stop_timer_->cancel();

    state = spinn;

    auto twist_command = geometry_msgs::msg::Twist();
    twist_command.linear.x = 0.0;
    twist_command.linear.y = 0.0;
    twist_command.linear.z = 0.0;
    twist_command.angular.x = 0.0;
    twist_command.angular.y = 0.0;
    twist_command.angular.z = 0.0;
    publisher_->publish(twist_command);
    RCLCPP_INFO(this->get_logger(), "Twist command linear x '%f", twist_command.linear.x);
    RCLCPP_INFO(this->get_logger(), "Twist command linear y '%f", twist_command.linear.y);
    RCLCPP_INFO(this->get_logger(), "Twist command linear z '%f", twist_command.linear.z);
    RCLCPP_INFO(this->get_logger(), "Twist command angular x '%f", twist_command.angular.x);
    RCLCPP_INFO(this->get_logger(), "Twist command angular y '%f", twist_command.angular.y);
    RCLCPP_INFO(this->get_logger(), "Twist command angular z '%f", twist_command.angular.z);
  }

  void forward_callback()
  {

    auto twist_command = geometry_msgs::msg::Twist();
    twist_command.linear.x = 0.1;
    twist_command.linear.y = 0.0;
    twist_command.linear.z = 0.0;
    twist_command.angular.x = 0.0;
    twist_command.angular.y = 0.0;
    twist_command.angular.z = 0.0;
    publisher_->publish(twist_command);
    RCLCPP_INFO(this->get_logger(), "Twist command linear x '%f", twist_command.linear.x);
    RCLCPP_INFO(this->get_logger(), "Twist command linear y '%f", twist_command.linear.y);
    RCLCPP_INFO(this->get_logger(), "Twist command linear z '%f", twist_command.linear.z);
    RCLCPP_INFO(this->get_logger(), "Twist command angular x '%f", twist_command.angular.x);
    RCLCPP_INFO(this->get_logger(), "Twist command angular y '%f", twist_command.angular.y);
    RCLCPP_INFO(this->get_logger(), "Twist command angular z '%f", twist_command.angular.z);
  }

  void forward_stop_callback()
  {
    forward_timer_->cancel();

    auto twist_command = geometry_msgs::msg::Twist();
    twist_command.linear.x = 0.0;
    twist_command.linear.y = 0.0;
    twist_command.linear.z = 0.0;
    twist_command.angular.x = 0.0;
    twist_command.angular.y = 0.0;
    twist_command.angular.z = 0.0;
    publisher_->publish(twist_command);
    RCLCPP_INFO(this->get_logger(), "Twist command linear x '%f", twist_command.linear.x);
    RCLCPP_INFO(this->get_logger(), "Twist command linear y '%f", twist_command.linear.y);
    RCLCPP_INFO(this->get_logger(), "Twist command linear z '%f", twist_command.linear.z);
    RCLCPP_INFO(this->get_logger(), "Twist command angular x '%f", twist_command.angular.x);
    RCLCPP_INFO(this->get_logger(), "Twist command angular y '%f", twist_command.angular.y);
    RCLCPP_INFO(this->get_logger(), "Twist command angular z '%f", twist_command.angular.z);
  }

  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr subscription_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr publisher_;
  rclcpp::Subscription<irobot_create_msgs::msg::HazardDetectionVector>::SharedPtr subscription_wanderer;
  rclcpp::TimerBase::SharedPtr full_stop_timer_;
  rclcpp::TimerBase::SharedPtr spin_timer_;
  rclcpp::TimerBase::SharedPtr spin_stop_timer_;
  rclcpp::TimerBase::SharedPtr reverse_timer_;
  rclcpp::TimerBase::SharedPtr reverse_stop_timer_;
  rclcpp::TimerBase::SharedPtr forward_timer_;
  rclcpp::TimerBase::SharedPtr forward_stop_timer_;
  bool in_process;
  bool reverse_done;
  bool spin_done;

  int state;
  int idle = 0;
  int during_process = 1;
  int reverse = 2;
  int spinn = 3;
  int go_forward = 4;
};

int main(int argc, char *argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<Wanderer>());
  rclcpp::shutdown();
  return 0;
}
