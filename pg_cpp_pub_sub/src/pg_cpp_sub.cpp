#include <iostream>
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "steelcast_msgs/msg/server_response.hpp"
using std::placeholders::_1;

class MinimalSubscriber : public rclcpp::Node
{
public:
  MinimalSubscriber()
  : Node("minimal_subscriber")
  {
    // subscription_ = this->create_subscription<steelcast_msgs::msg::ServerResponse>(
    //   "~/steelcast_res", std::bind(&MinimalSubscriber::topic_callback, this, _1), rmw_qos_profile_sensor_data);
    subscription_ = this->create_subscription<std_msgs::msg::String>(
      "~/steelcast_compressed_img", std::bind(&MinimalSubscriber::topic_callback, this, _1), rmw_qos_profile_sensor_data);
  }

private:
  void topic_callback(const std_msgs::msg::String::SharedPtr msg)
  {
    // printf("I heard: \n");
    std::cout << "img = " << msg->data << "\n";
    // std::cout << msg->status << " " << msg->status_msg << " " << msg->width << " " << msg->height << " " << msg->frame_type << " ";
    // std::cout << "msg = " << msg->data << " size = " << sizeof(msg->data) << "\n";
  }
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr subscription_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<MinimalSubscriber>());
  return 0;
}
