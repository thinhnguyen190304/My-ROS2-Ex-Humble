#include "rclcpp/rclcpp.hpp"
#include "example_interfaces/msg/int64.hpp"

using namespace std::chrono_literals;
using std::placeholders::_1;

class NumberPublisherNode : public rclcpp::Node
{
public:
  NumberPublisherNode() : Node("number_publisher")
  {
    this->declare_parameter<double>("timer_period", 1.0);
    this->declare_parameter<int>("number", 2);

    timer_period_ = this->get_parameter("timer_period").as_double();
    number_ = this->get_parameter("number").as_int();

    param_callback_handle_ =
      this->add_on_set_parameters_callback(
        std::bind(&NumberPublisherNode::parametersCallback, this, _1));

    publisher_ =
      this->create_publisher<example_interfaces::msg::Int64>("number", 10);

    timer_ =
      this->create_wall_timer(
        std::chrono::duration<double>(timer_period_),
        std::bind(&NumberPublisherNode::publishNumber, this));

    RCLCPP_INFO(this->get_logger(), "Number publisher has been started.");
  }

private:
  void publishNumber()
  {
    example_interfaces::msg::Int64 msg;
    msg.data = number_;
    publisher_->publish(msg);
  }

  rcl_interfaces::msg::SetParametersResult
  parametersCallback(const std::vector<rclcpp::Parameter> & params)
  {
    for (const auto & param : params)
    {
      if (param.get_name() == "timer_period")
      {
        timer_period_ = param.as_double();
        timer_->reset();
      }
      else if (param.get_name() == "number")
      {
        number_ = param.as_int();
      }
    }

    rcl_interfaces::msg::SetParametersResult result;
    result.successful = true;
    return result;
  }

  double timer_period_;
  int number_;

  rclcpp::Publisher<example_interfaces::msg::Int64>::SharedPtr publisher_;
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr
    param_callback_handle_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<NumberPublisherNode>());
  rclcpp::shutdown();
  return 0;
}
