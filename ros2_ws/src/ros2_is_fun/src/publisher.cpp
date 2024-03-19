#include <chrono>       // for std::chrono (time utilities)
#include <functional>   // for std::bind

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"


class PublisherNode : public rclcpp::Node
{
    public:
        PublisherNode() : Node("publisher_node")
        {
            count_ = 0;
            publisher_ = this->create_publisher<std_msgs::msg::String>("topic", 10);
            timer_ = this->create_wall_timer(
                std::chrono::milliseconds(1000),
                std::bind(&PublisherNode::timer_callback, this)
            );
        }
    private:
        void timer_callback()
        {
            auto message = std_msgs::msg::String();
            message.data = "(" + std::to_string(count_++) + ") Hello! ROS2 is fun";
            RCLCPP_INFO(this->get_logger(), "Publishing: '%s'", message.data.c_str());
            publisher_->publish(message);
        }
        rclcpp::TimerBase::SharedPtr timer_;
        rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_;
        size_t count_;
};

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  std::shared_ptr<PublisherNode> node = std::make_shared<PublisherNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
