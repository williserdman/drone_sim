#include "ClockDecimator.hh"

#include <cstdint>
#include <memory>

#include <rclcpp/rclcpp.hpp>
#include <rosgraph_msgs/msg/clock.hpp>

namespace drone_sim
{

class ClockDecimatorNode final : public rclcpp::Node
{
public:
  ClockDecimatorNode()
  : Node("drone_sim_clock_decimator")
  {
    const auto qos = rclcpp::QoS(rclcpp::KeepLast(1000)).reliable();
    this->publisher = this->create_publisher<rosgraph_msgs::msg::Clock>(
      "/gazebo/private/clock", qos);
    this->subscription = this->create_subscription<rosgraph_msgs::msg::Clock>(
      "/gazebo/native/clock", qos,
      [this](const rosgraph_msgs::msg::Clock::SharedPtr message) {
        const std::int64_t stampNanoseconds =
          static_cast<std::int64_t>(message->clock.sec) * 1'000'000'000LL +
          static_cast<std::int64_t>(message->clock.nanosec);
        if (this->decimator.ShouldPublish(stampNanoseconds))
        {
          this->publisher->publish(*message);
        }
      });
  }

private:
  ClockDecimator decimator;
  rclcpp::Publisher<rosgraph_msgs::msg::Clock>::SharedPtr publisher;
  rclcpp::Subscription<rosgraph_msgs::msg::Clock>::SharedPtr subscription;
};

}  // namespace drone_sim

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<drone_sim::ClockDecimatorNode>());
  rclcpp::shutdown();
  return 0;
}
