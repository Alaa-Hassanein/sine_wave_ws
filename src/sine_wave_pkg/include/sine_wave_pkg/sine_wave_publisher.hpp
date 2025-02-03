#ifndef SINE_WAVE_PUBLISHER_HPP
#define SINE_WAVE_PUBLISHER_HPP

#include <chrono>
#include <cmath>
#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/float64.hpp>
#include <sine_wave_pkg/sine_wave_parameters.hpp>
#include "rcl_interfaces/msg/set_parameters_result.hpp"

using namespace std::chrono_literals;

class SineWavePublisher : public rclcpp::Node
{
public:
    SineWavePublisher();

private:
    void publish_sine_wave();
    rcl_interfaces::msg::SetParametersResult parameter_callback(const std::vector<rclcpp::Parameter> &params);
    void update_timer();

    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr publisher_;
    rclcpp::TimerBase::SharedPtr timer_;
    double angle_;
    std::shared_ptr<sine_wave_params::ParamListener> param_listener_;
    sine_wave_params::Params params_;
    rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr param_callback_handle_;
};

#endif // SINE_WAVE_PUBLISHER_HPP
