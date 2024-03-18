# 'Hello! ROS2 is fun'

```{image} ../_static/gifs/talk-listen.gif
---
alt: talker-listener
align: center
width: 70%
---
```

This is introduction to creating a simple publisher and subscriber in ROS2. 

## Creating ROS2 package:
```bash
cd <path/to/ros2_ws>/src
ros2 pkg create ros2_is_fun --build-type ament_cmake --dependencies std_msgs rclcpp
```

This will create `ros2_is_fun` package with following files and directory inside it.
```bash
ros2_is_fun/
    CMakeLists.txt
    package.xml
    include/ros2_is_fun
    src/
```
For more info on the function of these files see *[What makes up a ROS 2 package?](https://docs.ros.org/en/foxy/Tutorials/Beginner-Client-Libraries/Creating-Your-First-ROS2-Package.html#what-makes-up-a-ros-2-package)*

The source code `publisher.cpp` and `subscriber.cpp` will be added in the `src` directory. 
The `CMakeLists.txt` will be edited to add the executable to the package and install them.

```cmake
add_executable(publisher src/publisher.cpp)
ament_target_dependencies(publisher rclcpp std_msgs)

add_executable(subscriber src/subscriber.cpp)
ament_target_dependencies(subscriber rclcpp std_msgs)

install(TARGETS 
  publisher
  subscriber
  DESTINATION lib/${PROJECT_NAME})
```

## Building the package

```bash
colcon build --packages-select ros2_is_fun
```

## Using the package

To be able to run the executables, first source the setup files from `install` directory.

```bash
source install/local_setup.bash
source install/setup.bash
```

Run the executables:

```bash
ros2 run ros2_is_fun publisher
```

```bash
ros2 run ros2_is_fun subscriber
```


## Source Code

``````{dropdown} Source Code
`````{tab-set}
````{tab-item} Publisher

```cpp
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

```

````

````{tab-item} Subscriber

```cpp
#include <functional>   // for std::bind

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

using std::placeholders::_1;

class SubscriberNode : public rclcpp::Node
{
    public:
        SubscriberNode() : Node("subscriber_node")
        {
            subscription_ = this->create_subscription<std_msgs::msg::String>(
                "topic", 10, std::bind(&SubscriberNode::topic_callback, this, _1)
            );
        }
    private:
        void topic_callback(const std_msgs::msg::String::SharedPtr message)
        {
            RCLCPP_INFO(this->get_logger(), "Msg Rcvd: '%s'", message->data.c_str());
        }
        rclcpp::Subscription<std_msgs::msg::String>::SharedPtr subscription_;
};

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  std::shared_ptr<SubscriberNode> node = std::make_shared<SubscriberNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}

```

````
`````
``````

## References

* ROS2 Foxy Tutorials: [Writing a simple publisher and subscriber (C++)](https://docs.ros.org/en/foxy/Tutorials/Beginner-Client-Libraries/Writing-A-Simple-Cpp-Publisher-And-Subscriber.html)
* `std::bind` function in C++: [Bind Function and Placeholders in C++](https://www.geeksforgeeks.org/bind-function-placeholders-c/)