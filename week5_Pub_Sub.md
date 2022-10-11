# Publisher-Subscriber

## 1 Create a packege

Navigate into ros2_ws/src, and run the package creation command:

```
ros2 pkg create --build-type ament_cmake cpp_pubsub
```

Navigate into ros2_ws/src/cpp_pubsub/src

## 2 Write the publisher node

Download the example talker code by entering the following command:

```
wget -O publisher_member_function.cpp https://raw.githubusercontent.com/ros2/examples/foxy/rclcpp/topics/minimal_publisher/member_function.cpp
```

Now there will be a new file named publisher_member_function.cpp. Open the file using your preferred text editor and edit:

```
#include <chrono>
#include <functional>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

using namespace std::chrono_literals;

/* This example creates a subclass of Node and uses std::bind() to register a
* member function as a callback from the timer. */

class MinimalPublisher : public rclcpp::Node
{
  public:
    MinimalPublisher()
    : Node("minimal_publisher"), count_(0)
    {
      publisher_ = this->create_publisher<std_msgs::msg::String>("topic", 10);
      timer_ = this->create_wall_timer(
      500ms, std::bind(&MinimalPublisher::timer_callback, this));
    }

  private:
    void timer_callback()
    {
      auto message = std_msgs::msg::String();
      message.data = "Hello, world! " + std::to_string(count_++);
      RCLCPP_INFO(this->get_logger(), "Publishing: '%s'", message.data.c_str());
      publisher_->publish(message);
    }
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_;
    size_t count_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<MinimalPublisher>());
  rclcpp::shutdown();
  return 0;
}
```
## 2.2 Add dependencies

Navigate one level back to the ros2_ws/src/cpp_pubsub directory, where the CMakeLists.txt and package.xml files have been created for you.

Open package.xml with your text editor and edit

```
<description>Examples of minimal publisher/subscriber using rclcpp</description>
<maintainer email="you@email.com">Your Name</maintainer>
<license>Apache License 2.0</license>
```

![image](https://user-images.githubusercontent.com/90166739/193400425-e84fc324-d121-406d-9ad8-0b89ec2588a3.png)

Add a new line after the ament_cmake buildtool dependency and paste the following dependencies corresponding to your node’s include statements:

```
<depend>rclcpp</depend>
<depend>std_msgs</depend>
```

![image](https://user-images.githubusercontent.com/90166739/193400604-116612a9-6e3f-4e3f-9ff9-dd70fc3375e5.png)

This declares the package needs rclcpp and std_msgs when its code is executed.
Make sure to save the file.

## 2.3 CMakeLists.txt

Make sure the file looks like following:

```
cmake_minimum_required(VERSION 3.5)
project(cpp_pubsub)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)

add_executable(talker src/publisher_member_function.cpp)
ament_target_dependencies(talker rclcpp std_msgs)

install(TARGETS
  talker
  DESTINATION lib/${PROJECT_NAME})

ament_package()
```

# 3 Write the subscriber node

Return to ros2_ws/src/cpp_pubsub/src to create the next node. Enter the following code in your terminal:

```
wget -O subscriber_member_function.cpp https://raw.githubusercontent.com/ros2/examples/foxy/rclcpp/topics/minimal_subscriber/member_function.cpp
```

Entering ls in the console will now return:

```
publisher_member_function.cpp  subscriber_member_function.cpp
```

Open the subscriber_member_function.cpp with your text editor.

```
#include <memory>

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
    void topic_callback(const std_msgs::msg::String::SharedPtr msg) const
    {
      RCLCPP_INFO(this->get_logger(), "I heard: '%s'", msg->data.c_str());
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
```

## 3.2 CMakeLists.txt

Reopen CMakeLists.txt and add the executable and target for the subscriber node below the publisher’s entries.

```
add_executable(listener src/subscriber_member_function.cpp)
ament_target_dependencies(listener rclcpp std_msgs)

install(TARGETS
  talker
  listener
  DESTINATION lib/${PROJECT_NAME})
```

# 4 Build and Run

It’s good practice to run rosdep in the root of your workspace (ros2_ws) to check for missing dependencies before building:

```
rosdep install -i --from-path src --rosdistro foxy -y
```

Still in the root of your workspace, ros2_ws, build your new package:

```
colcon build --packages-select cpp_pubsub
```

Open a new terminal, navigate to ros2_ws, and source the setup files:

```
. install/setup.bash
```

Now run the talker node:

```
ros2 run cpp_pubsub talker
```

```
[INFO] [minimal_publisher]: Publishing: "Hello World: 0"
[INFO] [minimal_publisher]: Publishing: "Hello World: 1"
[INFO] [minimal_publisher]: Publishing: "Hello World: 2"
[INFO] [minimal_publisher]: Publishing: "Hello World: 3"
[INFO] [minimal_publisher]: Publishing: "Hello World: 4"
```

![image](https://user-images.githubusercontent.com/90166739/193401313-a69a23af-fae1-4474-b080-409fca50a5a6.png)

Open another terminal, source the setup files from inside ros2_ws again, and then start the listener node:

```
ros2 run cpp_pubsub listener
```

```
[INFO] [minimal_subscriber]: I heard: "Hello World: 10"
[INFO] [minimal_subscriber]: I heard: "Hello World: 11"
[INFO] [minimal_subscriber]: I heard: "Hello World: 12"
[INFO] [minimal_subscriber]: I heard: "Hello World: 13"
[INFO] [minimal_subscriber]: I heard: "Hello World: 14"
```

![image](https://user-images.githubusercontent.com/90166739/193401418-6f192b1e-26b6-4b04-84c3-e0da0d6768dd.png)

Enter Ctrl+C in each terminal to stop the nodes from spinning

# Summary

You created two nodes to publish and subscribe to data over a topic. Before compiling and running them, you added their dependencies and executables to the package configuration files.
