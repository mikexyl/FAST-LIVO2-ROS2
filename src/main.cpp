#include "LIVMapper.h"

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  rclcpp::NodeOptions options;
  auto nh = std::make_shared<rclcpp::Node>("laserMapping");
  image_transport::ImageTransport it_(nh);
  LIVMapper mapper(nh, "laserMapping");
  mapper.initializeSubscribersAndPublishers(nh, it_);
  mapper.run(nh);
  rclcpp::shutdown();
  return 0;
}
