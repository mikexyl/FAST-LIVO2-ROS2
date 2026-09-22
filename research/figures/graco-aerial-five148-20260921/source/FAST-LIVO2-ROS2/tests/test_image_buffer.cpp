#include "LIVMapper.h"
#include <algorithm>
#include <iostream>

int main()
{
  auto message = std::make_shared<sensor_msgs::msg::Image>();
  message->width = 32;
  message->height = 24;
  message->encoding = "bgr8";
  message->step = message->width * 3;
  message->data.assign(message->step * message->height, 73);
  cv::Mat buffered = LIVMapper::getImageFromMsg(message);

  // A later ROS callback may reuse the original message allocation. Buffered
  // pixels must remain unchanged and valid after that message is destroyed.
  std::fill(message->data.begin(), message->data.end(), 0);
  message.reset();
  cv::Mat resized;
  cv::resize(buffered, resized, cv::Size(16, 12));
  if (buffered.rows != 24 || buffered.cols != 32 ||
      cv::countNonZero(resized.reshape(1) != 73) != 0)
  {
    std::cerr << "Buffered image still aliases the ROS message storage\n";
    return 1;
  }
  return 0;
}
