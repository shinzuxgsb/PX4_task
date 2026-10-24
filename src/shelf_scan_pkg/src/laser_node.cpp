// 激光笔控制节点（2024 电赛 D 题：盘点时点亮 0.5 秒左右，其余时间必须关闭）
//
// 订阅：/shelf_scan/laser (std_msgs/UInt8，1=点亮，0=关闭)
// 支持的驱动方式（参数 driver）：
//   topic  ：把电平转发到 /laser_control，由外部执行器订阅
//   gpio   ：直接用香橙派 GPIO（sysfs 或 gpiod 命令）驱动激光笔 MOS 管
//   serial ：往指定串口写 "L1\n" / "L0\n"（外接小 MCU / 串口继电器）
//   none   ：只打印日志（纯软件联调）
// 安全：超过 max_on_sec 自动关闭，节点启动时强制关闭。
#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdlib>
#include <fstream>
#include <memory>
#include <sstream>
#include <string>

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/u_int8.hpp>

#include "shelf_scan_pkg/simple_serial.hpp"

namespace
{

class LaserNode : public rclcpp::Node
{
public:
  LaserNode() : rclcpp::Node("shelf_laser_node")
  {
    laser_topic_ = declare_parameter<std::string>("laser_topic", "/shelf_scan/laser");
    control_topic_ = declare_parameter<std::string>("control_topic", "/laser_control");
    driver_ = declare_parameter<std::string>("driver", "topic");
    max_on_sec_ = declare_parameter<double>("max_on_sec", 3.0);

    gpio_mode_ = declare_parameter<std::string>("gpio_mode", "sysfs");
    gpio_chip_ = declare_parameter<std::string>("gpio_chip", "gpiochip0");
    gpio_line_ = declare_parameter<int>("gpio_line", 7);
    gpiod_hold_sec_ = declare_parameter<double>("gpiod_hold_sec", 0.10);
    active_high_ = declare_parameter<bool>("active_high", true);

    serial_port_ = declare_parameter<std::string>("serial_port", "/dev/ttyUSB1");
    serial_baud_ = declare_parameter<int>("serial_baud", 115200);
    on_command_ = declare_parameter<std::string>("on_command", "L1\n");
    off_command_ = declare_parameter<std::string>("off_command", "L0\n");

    control_pub_ = create_publisher<std_msgs::msg::UInt8>(control_topic_, rclcpp::QoS(10).reliable());
    laser_sub_ = create_subscription<std_msgs::msg::UInt8>(
      laser_topic_, rclcpp::QoS(10).reliable(),
      std::bind(&LaserNode::laserCallback, this, std::placeholders::_1));

    if (driver_ == "serial") {
      serial_ = std::make_shared<shelf_scan::SimpleSerial>();
      if (!serial_->open(serial_port_, serial_baud_)) {
        RCLCPP_ERROR(
          get_logger(), "打开激光串口 %s 失败：%s", serial_port_.c_str(),
          serial_->last_error().c_str());
        serial_.reset();
      } else {
        RCLCPP_INFO(get_logger(), "激光串口 %s 已打开", serial_port_.c_str());
      }
    } else if (driver_ == "gpio") {
      configureGpio();
    } else if (driver_ != "topic" && driver_ != "none") {
      RCLCPP_WARN(get_logger(), "未知激光驱动 %s，按 none 处理", driver_.c_str());
    }

    setOutput(false);
    watchdog_ = create_wall_timer(
      std::chrono::milliseconds(200), std::bind(&LaserNode::watchdogCallback, this));
    RCLCPP_INFO(
      get_logger(), "激光节点已启动：driver=%s 最长连续点亮=%.1fs",
      driver_.c_str(), max_on_sec_);
  }

  ~LaserNode() override
  {
    setOutput(false);
    if (serial_) serial_->close();
  }

private:
  void configureGpio()
  {
    if (gpio_mode_ == "gpiod") {
      RCLCPP_INFO(
        get_logger(), "使用 gpiod 命令控制 %s 第 %d 脚（需要已安装 libgpiod）",
        gpio_chip_.c_str(), gpio_line_);
      gpio_ready_ = true;
      writeGpio(false);          // 上电立刻关灯，不能等第一条指令
      return;
    }
    const std::string base = "/sys/class/gpio/gpio" + std::to_string(gpio_line_);
    // 导出可能瞬时失败（刚 unexport 完、或上一次进程还没释放），重试几次更稳
    for (int attempt = 1; attempt <= 5; ++attempt) {
      {
        std::ofstream probe(base + "/direction");
        if (probe.is_open()) break;
      }
      std::ofstream export_file("/sys/class/gpio/export");
      if (!export_file.is_open()) {
        RCLCPP_ERROR(
          get_logger(),
          "GPIO sysfs 不可用（/sys/class/gpio/export）。"
          "可改 gpio_mode:=gpiod 或 driver:=topic");
        return;
      }
      export_file << gpio_line_;
      export_file.close();
      rclcpp::sleep_for(std::chrono::milliseconds(150));
    }
    std::ofstream direction(base + "/direction");
    if (!direction.is_open()) {
      RCLCPP_ERROR(
        get_logger(), "GPIO %d 导出失败（已被占用或引脚号不对），激光将不可控", gpio_line_);
      return;
    }
    direction << "out";
    direction.close();
    gpio_ready_ = true;
    writeGpio(false);            // 上电立刻关灯（sysfs 默认是低电平，
                                 // 低电平导通时如果不管就会一启动就亮）
    RCLCPP_INFO(get_logger(), "GPIO %d 已配置为输出（active_high=%s）",
      gpio_line_, active_high_ ? "true" : "false");
  }

  void writeGpio(bool on)
  {
    if (!gpio_ready_) return;
    const int level = (on == active_high_) ? 1 : 0;
    if (gpio_mode_ == "gpiod") {
      // Orange Pi 5 Max ships without GPIO sysfs. libgpiod's default "exit"
      // mode releases the line immediately, so a laser pulse can disappear
      // before the 80 ms mission pulse. Keep the line driven for a bounded
      // interval, then release it.
      const long hold_us = std::lround(std::max(0.01, gpiod_hold_sec_) * 1.0e6);
      std::ostringstream command;
      command << "gpioset --mode=time --sec=" << hold_us / 1000000L
              << " --usec=" << hold_us % 1000000L
              << " " << gpio_chip_ << " " << gpio_line_ << "=" << level
              << " >/dev/null 2>&1";
      if (std::system(command.str().c_str()) != 0) {
        RCLCPP_WARN_THROTTLE(
          get_logger(), *get_clock(), 5000, "gpioset 执行失败：%s",
          command.str().c_str());
      }
      return;
    }
    std::ofstream value_file("/sys/class/gpio/gpio" + std::to_string(gpio_line_) + "/value");
    if (value_file.is_open()) value_file << level;
  }

  void setOutput(bool on)
  {
    if (!on && !laser_on_) return;   // 已经是关闭状态，无需重复下发
    laser_on_ = on;
    laser_on_time_ = now();

    if (driver_ == "topic" || driver_ == "none") {
      std_msgs::msg::UInt8 message;
      message.data = on ? 1 : 0;
      control_pub_->publish(message);
    } else if (driver_ == "gpio") {
      writeGpio(on);
    } else if (driver_ == "serial" && serial_ && serial_->is_open()) {
      serial_->write(on ? on_command_ : off_command_);
    }
    if (on) {
      RCLCPP_INFO(get_logger(), "激光点亮（driver=%s）", driver_.c_str());
    } else {
      RCLCPP_DEBUG(get_logger(), "激光关闭（driver=%s）", driver_.c_str());
    }
  }

  void laserCallback(const std_msgs::msg::UInt8::SharedPtr message)
  {
    setOutput(message->data != 0);
  }

  void watchdogCallback()
  {
    if (laser_on_ && (now() - laser_on_time_).seconds() > max_on_sec_) {
      RCLCPP_WARN(get_logger(), "激光连续点亮超过 %.1fs，强制关闭", max_on_sec_);
      setOutput(false);
    }
  }

  rclcpp::Publisher<std_msgs::msg::UInt8>::SharedPtr control_pub_;
  rclcpp::Subscription<std_msgs::msg::UInt8>::SharedPtr laser_sub_;
  rclcpp::TimerBase::SharedPtr watchdog_;
  std::shared_ptr<shelf_scan::SimpleSerial> serial_;

  std::string laser_topic_;
  std::string control_topic_;
  std::string driver_;
  std::string gpio_mode_;
  std::string gpio_chip_;
  std::string serial_port_;
  std::string on_command_;
  std::string off_command_;
  int gpio_line_ = 7;
  int serial_baud_ = 115200;
  double gpiod_hold_sec_ = 0.10;
  double max_on_sec_ = 3.0;
  bool active_high_ = true;
  bool gpio_ready_ = false;
  bool laser_on_ = false;
  rclcpp::Time laser_on_time_{0, 0, RCL_ROS_TIME};
};

}  // namespace

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<LaserNode>());
  rclcpp::shutdown();
  return 0;
}
