#include "rclcpp/rclcpp.hpp"
#include "example_interfaces/msg/int64.hpp"

using namespace std::chrono_literals;
using std::placeholders::_1;

class NumberPublisherNode : public rclcpp::Node
{
public:
    NumberPublisherNode() : Node("number_publisher")
    {
        this->declare_parameter("number", 2);
        this->declare_parameter("publish_frequency", 1.0);

        number_ = this->get_parameter("number").as_int();
        publish_frequency_ = this->get_parameter("publish_frequency").as_double();

        number_publisher_ =
            this->create_publisher<example_interfaces::msg::Int64>("number", 10);

        createTimer();

        param_callback_handle_ =
            this->add_on_set_parameters_callback(
                std::bind(&NumberPublisherNode::parametersCallback, this, _1));

        RCLCPP_INFO(this->get_logger(), "Number publisher has been started.");
    }

private:
    void createTimer()
    {
        auto period = std::chrono::duration<double>(1.0 / publish_frequency_);
        number_timer_ = this->create_wall_timer(
            period, std::bind(&NumberPublisherNode::publishNumber, this));
    }

    rcl_interfaces::msg::SetParametersResult
    parametersCallback(const std::vector<rclcpp::Parameter> &params)
    {
        for (const auto &param : params)
        {
            if (param.get_name() == "publish_frequency")
            {
                publish_frequency_ = param.as_double();
                number_timer_->cancel();
                createTimer();

                RCLCPP_INFO(this->get_logger(),
                            "Publish frequency changed to %.2f Hz",
                            publish_frequency_);
            }
        }

        rcl_interfaces::msg::SetParametersResult result;
        result.successful = true;
        return result;
    }

    void publishNumber()
    {
        example_interfaces::msg::Int64 msg;
        msg.data = number_;
        number_publisher_->publish(msg);
    }

    int number_;
    double publish_frequency_;

    rclcpp::Publisher<example_interfaces::msg::Int64>::SharedPtr number_publisher_;
    rclcpp::TimerBase::SharedPtr number_timer_;
    OnSetParametersCallbackHandle::SharedPtr param_callback_handle_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<NumberPublisherNode>());
    rclcpp::shutdown();
    return 0;
}
