// 轻量串口（Linux / POSIX）
//
// 为什么不用工程里现成的 serial_comm：它基于 boost::asio + 定时器实现读超时，
// 超时/取消后会在 io_service 队列里残留待处理的读操作，重复调用 read() 时可能
// 出现“上一次的读还没收尾、又发起一次读”的情况，丢数据风险较大。
// 这里用 open + termios + poll 实现，读写各自独立、超时精确、退出干净。
//
// 用于：与地面站通信的无线数传串口、激光串口（需要串口版驱动时）。
#pragma once

#include <cstddef>
#include <cstdint>
#include <string>
#include <vector>

namespace shelf_scan
{

class SimpleSerial
{
public:
  SimpleSerial() = default;
  ~SimpleSerial();

  SimpleSerial(const SimpleSerial &) = delete;
  SimpleSerial & operator=(const SimpleSerial &) = delete;

  bool open(const std::string & device, int baud);
  void close();
  bool is_open() const { return fd_ >= 0; }

  // 返回：>0 读到的字节数；0 超时（无数据）；-1 错误
  int read(uint8_t * buffer, std::size_t max_size, int timeout_ms);
  bool write(const std::string & text);

  const std::string & last_error() const { return last_error_; }

  // 列出 /dev 下常见的串口设备
  static std::vector<std::string> available_ports();

private:
  int fd_ = -1;
  std::string last_error_;
};

}  // namespace shelf_scan
