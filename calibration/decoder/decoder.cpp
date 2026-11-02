#include <rclcpp/rclcpp.hpp>
#include <image_transport/image_transport.hpp>
#include <vector>

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<rclcpp::Node>("calibration_decoder");
  std::vector<image_transport::Publisher> publishers;
  std::vector<image_transport::Subscriber> subscribers;
  for (const std::string side : {"left", "right"}) {
    auto publisher = image_transport::create_publisher(
        node.get(), "/calibration/" + side + "/image_raw");
    publishers.push_back(publisher);
    subscribers.push_back(image_transport::create_subscription(
        node.get(), "/stereo/" + side + "/camera/image_raw",
        [publisher](const sensor_msgs::msg::Image::ConstSharedPtr &image) {
          publisher.publish(image);
        }, "compressed", rmw_qos_profile_sensor_data));
  }
  rclcpp::spin(node);
  rclcpp::shutdown();
}
