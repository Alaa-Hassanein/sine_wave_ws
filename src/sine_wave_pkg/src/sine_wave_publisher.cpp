#include "sine_wave_pkg/sine_wave_publisher.hpp"

SineWavePublisher::SineWavePublisher()
    : Node("sine_wave_publisher"), angle_(0.0)
{
    param_listener_ = std::make_shared<sine_wave_params::ParamListener>(get_node_parameters_interface());
    params_ = param_listener_->get_params();

    publisher_ = this->create_publisher<std_msgs::msg::Float64>(params_.topic_name, 10);
    update_timer();

    // Register the parameter callback
    param_callback_handle_ = this->add_on_set_parameters_callback(
        std::bind(&SineWavePublisher::parameter_callback, this, std::placeholders::_1));
}

void SineWavePublisher::publish_sine_wave()
{
    auto message = std_msgs::msg::Float64();
    message.data = params_.amplitude * std::sin(angle_ * params_.angular_frequency + params_.phase);

    RCLCPP_INFO(this->get_logger(), "Publishing: '%f'", message.data);
    publisher_->publish(message);
    angle_ += 0.1;
}

rcl_interfaces::msg::SetParametersResult SineWavePublisher::parameter_callback(const std::vector<rclcpp::Parameter> &params)
{
    rcl_interfaces::msg::SetParametersResult result;
    result.successful = true;

    for (const auto &param : params)
    {
        if (param.get_name() == "amplitude")
        {
            params_.amplitude = param.as_double();
        }
        else if (param.get_name() == "phase")
        {
            params_.phase = param.as_double();
        }
        else if (param.get_name() == "angular_frequency")
        {
            params_.angular_frequency = param.as_double();
        }
        else if (param.get_name() == "frequency")
        {
            params_.frequency = param.as_double();
            update_timer(); // Update the timer when frequency changes
        }
        RCLCPP_INFO(this->get_logger(), "Parameter %s has been set to: %f", param.get_name().c_str(), param.as_double());
    }

    return result;
}

void SineWavePublisher::update_timer()
{
    timer_ = this->create_wall_timer(
        std::chrono::duration<double>(1.0 / params_.frequency), std::bind(&SineWavePublisher::publish_sine_wave, this));
}

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<SineWavePublisher>());
    rclcpp::shutdown();
    return 0;
}
