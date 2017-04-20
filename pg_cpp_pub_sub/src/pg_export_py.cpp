#include <iostream>
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include <boost/python.hpp>

using namespace std;


class MinimalPublisher
{
public:
  rclcpp::Node::SharedPtr pub_node;
  std::string topic;
  MinimalPublisher(std::string topic_name)
  {
    int argc = 1;
    char *argv[] = {"pg_export_py_pub"};
    rclcpp::init(argc, argv);
    topic = topic_name;
    pub_node = rclcpp::node::Node::make_shared("minimal_subscriber");
    if (rclcpp::ok()) {
      rclcpp::spin_some(pub_node);
    }
  }

  void pub_msg(std::string str_msg) {
    auto publisher = pub_node->create_publisher<std_msgs::msg::String>(topic);
    auto ros_msg = std::make_shared<std_msgs::msg::String>();
    ros_msg->data = str_msg;
    for (int i = 0; i < 10 ; i++) {
      publisher->publish(ros_msg);
    }
    printf("Published: [%s]\n", ros_msg->data.c_str());
  }
};


class MinimalSubscriber
{
public:
  rclcpp::Node::SharedPtr sub_node;
  std::string ros_msg;
  MinimalSubscriber(std::string topic_name)
  {
    int argc = 1;
    char *argv[] = {"pg_export_py_sub"};
    rclcpp::init(argc, argv);
    sub_node = rclcpp::node::Node::make_shared("minimal_publisher");
    subscription_ = sub_node->create_subscription<std_msgs::msg::String>(
      "new", std::bind(&MinimalSubscriber::topic_callback, this, std::placeholders::_1));
      if (rclcpp::ok()) {
                 rclcpp::spin_some(sub_node);
                 rclcpp::utilities::sleep_for(500ms);
             }
  }

  void spin_ros_node() {
    if (rclcpp::ok()) {
        rclcpp::spin_some(sub_node);
        rclcpp::utilities::sleep_for(10ms);
    }
  }

  std::string get_msg() {
    return this->ros_msg;
  }

private:
  void topic_callback(const std_msgs::msg::String::SharedPtr msg)
  {
    this->ros_msg = msg->data;
    printf("I heard: [%s]\n", msg->data.c_str());
    std::cout << "I also heard: \n" << this->ros_msg << "\n";
  }
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr subscription_;
};


void hello() {
    std::cout << "Hello World" << '\n';
}

int main(int argc, char * argv[])
{
  // rclcpp::init(argc, argv);
  // rclcpp::spin(std::make_shared<MinimalSubscriber>());
  std::cout <<"kk, I'm done\n";
  return 0;
}

using namespace boost::python;
BOOST_PYTHON_MODULE(hi) {
    def("hello", hello);
    class_<MinimalPublisher>("Ros_Pub", init<std::string>())
    .def("pub_msg", &MinimalPublisher::pub_msg)
    ;
    class_<MinimalSubscriber>("Ros_Sub", init<std::string>())
    .def("spin_ros_node", &MinimalSubscriber::spin_ros_node)
    .def("get_msg", &MinimalSubscriber::get_msg)
    ;
}
