#include "shelf_scan_pkg/simple_serial.hpp"

#ifndef _WIN32
#include <cerrno>
#include <cstring>
#include <dirent.h>
#include <fcntl.h>
#include <poll.h>
#include <termios.h>
#include <unistd.h>
#endif

#include <algorithm>

namespace shelf_scan
{
namespace
{

#ifndef _WIN32
speed_t baud_to_speed(int baud)
{
  switch (baud) {
    case 9600: return B9600;
    case 19200: return B19200;
    case 38400: return B38400;
    case 57600: return B57600;
    case 115200: return B115200;
#ifdef B230400
    case 230400: return B230400;
#endif
#ifdef B460800
    case 460800: return B460800;
#endif
#ifdef B921600
    case 921600: return B921600;
#endif
    default: return B115200;
  }
}
#endif

}  // namespace

SimpleSerial::~SimpleSerial()
{
  close();
}

bool SimpleSerial::open(const std::string & device, int baud)
{
  close();
#ifdef _WIN32
  last_error_ = "本实现仅支持 Linux（香橙派）";
  return false;
#else
  const int fd = ::open(device.c_str(), O_RDWR | O_NOCTTY | O_NONBLOCK);
  if (fd < 0) {
    last_error_ = "打开 " + device + " 失败：" + std::strerror(errno);
    return false;
  }

  termios settings{};
  if (::tcgetattr(fd, &settings) != 0) {
    last_error_ = "读取串口属性失败：" + std::string(std::strerror(errno));
    ::close(fd);
    return false;
  }

  // 8N1、无流控、原始模式（不接受任何字符转换）
  settings.c_iflag &= ~(IGNBRK | BRKINT | PARMRK | ISTRIP | INLCR | IGNCR | ICRNL | IXON);
  settings.c_oflag &= ~OPOST;
  settings.c_lflag &= ~(ECHO | ECHONL | ICANON | ISIG | IEXTEN);
  settings.c_cflag &= ~(CSIZE | PARENB | CSTOPB);
  settings.c_cflag |= (CS8 | CLOCAL | CREAD);
  settings.c_cflag &= ~CRTSCTS;
  settings.c_cc[VMIN] = 0;
  settings.c_cc[VTIME] = 0;
  const speed_t speed = baud_to_speed(baud);
  ::cfsetispeed(&settings, speed);
  ::cfsetospeed(&settings, speed);

  if (::tcsetattr(fd, TCSANOW, &settings) != 0) {
    last_error_ = "配置串口失败：" + std::string(std::strerror(errno));
    ::close(fd);
    return false;
  }
  ::tcflush(fd, TCIOFLUSH);
  fd_ = fd;
  last_error_.clear();
  return true;
#endif
}

void SimpleSerial::close()
{
#ifndef _WIN32
  if (fd_ >= 0) {
    ::close(fd_);
    fd_ = -1;
  }
#endif
}

int SimpleSerial::read(uint8_t * buffer, std::size_t max_size, int timeout_ms)
{
#ifdef _WIN32
  (void)buffer;
  (void)max_size;
  (void)timeout_ms;
  return -1;
#else
  if (fd_ < 0) {
    last_error_ = "串口未打开";
    return -1;
  }
  pollfd descriptor{};
  descriptor.fd = fd_;
  descriptor.events = POLLIN;
  const int ready = ::poll(&descriptor, 1, timeout_ms);
  if (ready < 0) {
    if (errno == EINTR) return 0;
    last_error_ = std::string("poll 失败：") + std::strerror(errno);
    return -1;
  }
  if (ready == 0) return 0;   // 超时

  const ssize_t count = ::read(fd_, buffer, max_size);
  if (count < 0) {
    if (errno == EAGAIN || errno == EWOULDBLOCK || errno == EINTR) return 0;
    last_error_ = std::string("读串口失败：") + std::strerror(errno);
    return -1;
  }
  return static_cast<int>(count);
#endif
}

bool SimpleSerial::write(const std::string & text)
{
#ifdef _WIN32
  (void)text;
  return false;
#else
  if (fd_ < 0) {
    last_error_ = "串口未打开";
    return false;
  }
  std::size_t written = 0;
  const int deadline_ms = 500;
  while (written < text.size()) {
    pollfd descriptor{};
    descriptor.fd = fd_;
    descriptor.events = POLLOUT;
    const int ready = ::poll(&descriptor, 1, deadline_ms);
    if (ready <= 0) {
      last_error_ = ready == 0 ? "写串口超时" : std::string("poll(POLLOUT) 失败：") +
        std::strerror(errno);
      return false;
    }
    const ssize_t count = ::write(fd_, text.data() + written, text.size() - written);
    if (count < 0) {
      if (errno == EAGAIN || errno == EWOULDBLOCK || errno == EINTR) continue;
      last_error_ = std::string("写串口失败：") + std::strerror(errno);
      return false;
    }
    written += static_cast<std::size_t>(count);
  }
  return true;
#endif
}

std::vector<std::string> SimpleSerial::available_ports()
{
  std::vector<std::string> ports;
#ifndef _WIN32
  DIR * directory = ::opendir("/dev");
  if (directory == nullptr) return ports;
  while (dirent * entry = ::readdir(directory)) {
    const std::string name(entry->d_name);
    const bool candidate =
      name.compare(0, 6, "ttyUSB") == 0 || name.compare(0, 6, "ttyACM") == 0 ||
      name.compare(0, 6, "ttyAMA") == 0;
    if (candidate) ports.push_back("/dev/" + name);
  }
  ::closedir(directory);
  std::sort(ports.begin(), ports.end());
#endif
  return ports;
}

}  // namespace shelf_scan
