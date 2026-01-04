#include <memory>
#include <string>
#include <iostream>
#include <unistd.h>
#include <fcntl.h>
#include <termios.h>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

using std::placeholders::_1;

class UartSubscriber : public rclcpp::Node
{
public:
  UartSubscriber()
  : Node("uart_subscriber"), counter_(0)
  {
    // Open UART
    fd_ = open("/dev/ttyUSB0", O_RDWR | O_NOCTTY | O_SYNC);
    if (fd_ < 0) {
      RCLCPP_ERROR(this->get_logger(), "Failed to open /dev/ttyUSB0: %s", strerror(errno));
      return;
    }

    // Configure UART
    struct termios tty;
    if (tcgetattr(fd_, &tty) != 0) {
      RCLCPP_ERROR(this->get_logger(), "Error getting termios settings");
      close(fd_);
      fd_ = -1;
      return;
    }

    cfsetospeed(&tty, B115200);
    cfsetispeed(&tty, B115200);

    tty.c_cflag = (tty.c_cflag & ~CSIZE) | CS8; // 8-bit chars
    tty.c_iflag &= ~IGNBRK;                     // disable break processing
    tty.c_lflag = 0;                            // no signaling chars, no echo, no canonical processing
    tty.c_oflag = 0;                            // no remapping, no delays
    tty.c_cc[VMIN]  = 0;                        // read doesn't block
    tty.c_cc[VTIME] = 5;                        // 0.5 seconds read timeout

    tty.c_iflag &= ~(IXON | IXOFF | IXANY);     // shut off xon/xoff ctrl
    tty.c_cflag |= (CLOCAL | CREAD);            // ignore modem controls, enable reading
    tty.c_cflag &= ~(PARENB | PARODD);          // no parity
    tty.c_cflag &= ~CSTOPB;                     // one stop bit
    tty.c_cflag &= ~CRTSCTS;                    // no hardware flow control

    if (tcsetattr(fd_, TCSANOW, &tty) != 0) {
      RCLCPP_ERROR(this->get_logger(), "Error setting termios settings");
      close(fd_);
      fd_ = -1;
      return;
    }

    RCLCPP_INFO(this->get_logger(), "UART connected on /dev/ttyUSB0");

    // ROS 2 subscriber
    subscription_ = this->create_subscription<std_msgs::msg::String>(
      "topic", 10, std::bind(&UartSubscriber::topic_callback, this, _1));
  }

  ~UartSubscriber() {
    if (fd_ >= 0) {
      close(fd_);
    }
  }

private:
  void topic_callback(const std_msgs::msg::String::SharedPtr msg)
  {
    if (fd_ < 0) {
      RCLCPP_ERROR(this->get_logger(), "UART not available");
      return;
    }

    counter_++;
    std::string command = "vel: 20; accel: 10; brake: 10";

    ssize_t written = write(fd_, command.c_str(), command.size());
    if (written < 0) {
      RCLCPP_ERROR(this->get_logger(), "Failed to write to UART");
    } else {
      RCLCPP_INFO(this->get_logger(), "I heard: '%s' -> Sent: %s",
                  msg->data.c_str(), command.c_str());
    }
  }

  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr subscription_;
  int fd_{-1};
  int counter_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<UartSubscriber>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}