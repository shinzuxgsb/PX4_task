// =====================================================================
//  tofsense_m_node.cpp  —— Nooploop TOFSense-M 激光定高器串口驱动
//
//  协议：NLink_TOFSense_M_Frame0（见 TOFSense-M 用户手册 7.2.1 / 7.3.1）
//    串口默认 921600, 8N1, 无校验/流控, 小端
//    一帧 = Frame Header(0x57) + Function Mark(0x01 输出 / 0x10 查询)
//          + reserved(1) + id(1) + system_time(u32) + zone_map(u8)
//          + N x { dis(u24, 单位mm*1000), dis_status(u8), signal(u16) }
//          + reserved(6) + SumCheck(1)
//    整帧长度 = 16 + N*6；zone_map=0x40 → 8x8(N=64, 400字节, 15Hz)
//                        zone_map=0x10 → 4x4(N=16, 112字节, 60Hz)
//    校验和 = 从帧头到 Data 所有字节求和, 取低 8 位
//
//  本节点：读串口 → 解析 → 发布高度
//    /tofsense/range   sensor_msgs/Range   （高度，米）
//    /tofsense/height  std_msgs/Float32    （同一数值，方便直接喂桥接）
//    /tofsense/zones   std_msgs/Float32MultiArray（各像素距离，调试用）
//
//  权限：/dev/ttySx 属于 root:dialout，先把用户加进去：
//        sudo usermod -aG dialout orangepi   （然后重新登录）
//
//  注意：这是"读"，不改变模块配置。模块要保持 UART 主动输出模式。
// =====================================================================

#include <fcntl.h>
#include <termios.h>
#include <unistd.h>

#include <atomic>
#include <algorithm>
#include <chrono>
#include <cstdint>
#include <cstring>
#include <string>
#include <thread>
#include <vector>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/range.hpp>
#include <std_msgs/msg/float32.hpp>
#include <std_msgs/msg/float32_multi_array.hpp>

namespace {
constexpr uint8_t kHeader = 0x57;
constexpr uint8_t kFuncOutput = 0x01;
constexpr uint8_t kZoneMap8x8 = 0x40;
constexpr uint8_t kZoneMap4x4 = 0x10;
}  // namespace

class TofSenseM : public rclcpp::Node
{
public:
  TofSenseM() : Node("tofsense_m")
  {
    port_        = declare_parameter<std::string>("port", "/dev/ttyS3");
    baud_        = declare_parameter<int>("baud", 921600);
    frame_id_    = declare_parameter<std::string>("frame_id", "tof_link");
    range_mode_  = declare_parameter<std::string>("range_mode", "min");  // min | center | mean
    // 2026-10-10：下限从 0.03 放到 0.02 —— 实测静止时读到 0.031m，被 0.03 的门限卡得死死的。
    min_range_   = declare_parameter<double>("min_range", 0.02);
    max_range_   = declare_parameter<double>("max_range", 8.0);
    fov_         = declare_parameter<double>("field_of_view", 0.05);
    status_ok_max_ = declare_parameter<int>("status_ok_max", 0);  // dis_status > 该值认为无效
    // 距离换算：meters = (int24 值) / distance_scale
    //   1000    → 手册示例的“492”按 492 直接当米（0.492m 合理）
    //   1000000 → 手册示例的“492”当成 492mm
    distance_scale_ = declare_parameter<double>("distance_scale", 1000000.0);
    // 2026-10-10 新增：三道"抗抽风"闸 —— 原来一个都没有，所以 TOF 能报出 0.003m / 0.709m
    // 这种乱值，而它直接就是 EV 的高度。另外 min_range/max_range 原来只写进 Range 消息、
    // **根本没参与过滤**。
    // 默认 4：实测这台静止时常常只有 6/16 个区有效，设 6 会把大部分帧丢掉（高度只剩 ~1Hz）。
    min_valid_zones_ = declare_parameter<int>("min_valid_zones", 4);  // 有效区少于此数 → 整帧丢弃
    min_signal_      = declare_parameter<int>("min_signal", 0);       // 信号强度下限，0 = 不过滤
    median_window_   = declare_parameter<int>("median_window", 5);    // 输出做 N 点中值滤波，1 = 关

    range_pub_  = create_publisher<sensor_msgs::msg::Range>("/tofsense/range", 10);
    height_pub_ = create_publisher<std_msgs::msg::Float32>("/tofsense/height", 10);
    zones_pub_  = create_publisher<std_msgs::msg::Float32MultiArray>("/tofsense/zones", 10);

    running_ = true;
    thread_ = std::thread(&TofSenseM::readLoop, this);

    last_stat_ = now();
    RCLCPP_INFO(get_logger(), "TOFSense-M 驱动启动：%s @ %d（range_mode=%s）",
                port_.c_str(), baud_, range_mode_.c_str());
  }

  ~TofSenseM() override
  {
    running_ = false;
    if (thread_.joinable()) {
      thread_.join();
    }
    if (fd_ >= 0) {
      ::close(fd_);
    }
  }

private:
  uint64_t now_us()
  {
    return static_cast<uint64_t>(get_clock()->now().nanoseconds() / 1000);
  }

  // 按用户手册 7.1 的写法解 int24（先左移再除 256，保持符号）
  static float decodeInt24(const uint8_t *p, float scale)
  {
    const int32_t raw = static_cast<int32_t>(
        (static_cast<uint32_t>(p[0]) << 8) |
        (static_cast<uint32_t>(p[1]) << 16) |
        (static_cast<uint32_t>(p[2]) << 24));
    return static_cast<float>(raw / 256) / scale;
  }

  bool openPort()
  {
    const int fd = ::open(port_.c_str(), O_RDWR | O_NOCTTY | O_NONBLOCK);
    if (fd < 0) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000,
                           "打开 %s 失败：%s（检查接线和 dialout 组权限）",
                           port_.c_str(), std::strerror(errno));
      return false;
    }
    termios tio{};
    if (tcgetattr(fd, &tio) != 0) {
      ::close(fd);
      return false;
    }
    cfmakeraw(&tio);
    tio.c_cflag |= (CLOCAL | CREAD);
    tio.c_cflag &= ~CRTSCTS;
    tio.c_cc[VMIN] = 0;
    tio.c_cc[VTIME] = 1;
    speed_t sp = B921600;
    switch (baud_) {
      case 115200:  sp = B115200;  break;
      case 230400:  sp = B230400;  break;
      case 460800:  sp = B460800;  break;
      case 921600:  sp = B921600;  break;
      case 1000000: sp = B1000000; break;
      case 1500000: sp = B1500000; break;
      case 2000000: sp = B2000000; break;
      case 3000000: sp = B3000000; break;
      default: break;
    }
    cfsetispeed(&tio, sp);
    cfsetospeed(&tio, sp);
    if (tcsetattr(fd, TCSANOW, &tio) != 0) {
      ::close(fd);
      return false;
    }
    tcflush(fd, TCIFLUSH);
    fd_ = fd;
    RCLCPP_INFO(get_logger(), "%s 打开成功 @ %d 8N1", port_.c_str(), baud_);
    return true;
  }

  void readLoop()
  {
    std::vector<uint8_t> buf;
    buf.reserve(2048);
    uint8_t chunk[1024];

    while (running_ && rclcpp::ok()) {
      if (fd_ < 0 && !openPort()) {
        std::this_thread::sleep_for(std::chrono::seconds(2));
        continue;
      }
      const ssize_t n = ::read(fd_, chunk, sizeof(chunk));
      if (n < 0) {
        if (errno != EAGAIN && errno != EWOULDBLOCK && errno != EINTR) {
          RCLCPP_ERROR(get_logger(), "读串口出错：%s", std::strerror(errno));
          ::close(fd_);
          fd_ = -1;
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(2));
        continue;
      }
      buf.insert(buf.end(), chunk, chunk + n);
      consume(buf);
      if (buf.size() > 8192) {
        buf.erase(buf.begin(), buf.begin() + (buf.size() - 1024));  // 防止乱码堆积
      }
    }
  }

  // 从缓冲区里反复取帧
  void consume(std::vector<uint8_t> &buf)
  {
    size_t i = 0;
    while (i + 10 <= buf.size()) {
      if (buf[i] != kHeader || buf[i + 1] != kFuncOutput) {
        ++i;
        continue;
      }
      const uint8_t zone_map = buf[i + 8];
      int zones = 0;
      if (zone_map == kZoneMap8x8) {
        zones = 64;
      } else if (zone_map == kZoneMap4x4) {
        zones = 16;
      } else {
        ++i;
        continue;
      }
      const size_t total = 16 + static_cast<size_t>(zones) * 6;
      if (buf.size() - i < total) {
        break;  // 帧还没收全
      }
      uint8_t sum = 0;
      for (size_t k = 0; k + 1 < total; ++k) {
        sum = static_cast<uint8_t>(sum + buf[i + k]);
      }
      if (sum != buf[i + total - 1]) {
        ++checksum_errors_;
        ++i;  // 校验不过 → 往后挪一个字节重新同步
        continue;
      }
      parseAndPublish(&buf[i], total, zones);
      ++frames_;
      i += total;
    }
    if (i > 0) {
      buf.erase(buf.begin(), buf.begin() + static_cast<long>(i));
    }
  }

  void parseAndPublish(const uint8_t *f, size_t total, int zones)
  {
    const uint8_t id = f[3];
    const uint32_t t_ms = static_cast<uint32_t>(f[4]) | (static_cast<uint32_t>(f[5]) << 8) |
                          (static_cast<uint32_t>(f[6]) << 16) | (static_cast<uint32_t>(f[7]) << 24);
    (void)total;

    std::vector<float> dists(static_cast<size_t>(zones), 0.0f);
    double best = 0.0;
    double sum = 0.0;
    int valid = 0;

    for (int z = 0; z < zones; ++z) {
      const uint8_t *p = f + 9 + z * 6;
      const float d = decodeInt24(p, static_cast<float>(distance_scale_));  // p[0..2] → 米
      const uint8_t status = p[3];
      const uint16_t signal = static_cast<uint16_t>(p[4] | (p[5] << 8));
      dists[static_cast<size_t>(z)] = d;
      // 2026-10-10：有效判据补齐 —— 状态 + 量程 + 信号强度一起看
      if (d > 0.0f && status <= status_ok_max_ &&
          d >= static_cast<float>(min_range_) && d <= static_cast<float>(max_range_) &&
          (min_signal_ <= 0 || signal >= min_signal_)) {
        if (valid == 0 || d < best) {
          best = d;
        }
        sum += d;
        ++valid;
      }
    }

    // 2026-10-10：有效区太少就整帧丢弃（近地/桨流下 TOF 常常只有几个区是真的）
    if (valid < min_valid_zones_) {
      return;
    }

    double height = 0.0;
    if (valid > 0) {
      if (range_mode_ == "center") {
        // 中心区本身也要有效且在量程内（原来直接取 dists[c]，中心区无效时会得到 0）
        const int c = zones == 64 ? (4 * 8 + 4) : (2 * 4 + 2);
        const float dc = dists[static_cast<size_t>(c)];
        height = (dc >= static_cast<float>(min_range_) && dc <= static_cast<float>(max_range_))
                     ? dc : (sum / valid);
      } else if (range_mode_ == "mean") {
        height = sum / valid;
      } else {
        height = best;  // 默认：取最近的有效像素（正下方地面）
      }
    }
    if (height <= 0.0) {
      return;  // 这一帧没有有效距离
    }

    // 2026-10-10：输出做 N 点中值滤波 —— 单点尖峰（0.003 / 0.7 那种）直接被压掉，
    // 真实高度是连续变化的，中值只带来约 N/2 帧延迟（45Hz 下 ~0.05s，可忽略）。
    if (median_window_ > 1) {
      med_buf_.push_back(static_cast<float>(height));
      if (static_cast<int>(med_buf_.size()) > median_window_) {
        med_buf_.erase(med_buf_.begin());
      }
      std::vector<float> tmp(med_buf_);
      std::sort(tmp.begin(), tmp.end());
      height = tmp[tmp.size() / 2];
    }

    const auto stamp = this->now();

    sensor_msgs::msg::Range r;
    r.header.stamp = stamp;
    r.header.frame_id = frame_id_;
    r.radiation_type = sensor_msgs::msg::Range::INFRARED;
    r.field_of_view = static_cast<float>(fov_);
    r.min_range = static_cast<float>(min_range_);
    r.max_range = static_cast<float>(max_range_);
    r.range = static_cast<float>(height);
    range_pub_->publish(r);

    std_msgs::msg::Float32 h;
    h.data = static_cast<float>(height);
    height_pub_->publish(h);

    std_msgs::msg::Float32MultiArray z;
    z.data = dists;
    zones_pub_->publish(z);

    last_id_ = id;
    last_t_ms_ = t_ms;
    reportStats(zones, valid, height);
  }

  void reportStats(int zones, int valid, double height)
  {
    const auto t = now();
    const double dt = (t - last_stat_).seconds();
    if (dt < 5.0) {
      return;
    }
    RCLCPP_INFO(get_logger(),
                "帧 %llu（校验错 %llu）| %dx%d 有效 %d | 高度 %.3f m | id=%u t=%ums",
                static_cast<unsigned long long>(frames_.load()),
                static_cast<unsigned long long>(checksum_errors_.load()),
                zones == 64 ? 8 : 4, zones == 64 ? 8 : 4, valid, height,
                static_cast<unsigned>(last_id_), static_cast<unsigned>(last_t_ms_));
    last_stat_ = t;
  }

  // ---- 参数 ----
  std::string port_, frame_id_, range_mode_;
  int baud_{921600};
  double min_range_{0.02}, max_range_{8.0}, fov_{0.05};
  int status_ok_max_{0};
  double distance_scale_{1000.0};
  int min_valid_zones_{4}, min_signal_{0}, median_window_{5};
  std::vector<float> med_buf_;      // 中值滤波的滑动窗口

  // ---- 状态 ----
  int fd_{-1};
  std::atomic<bool> running_{false};
  std::thread thread_;
  std::atomic<uint64_t> frames_{0};
  std::atomic<uint64_t> checksum_errors_{0};
  rclcpp::Time last_stat_{0, 0, RCL_ROS_TIME};
  uint8_t last_id_{0};
  uint32_t last_t_ms_{0};

  // ---- ROS 接口 ----
  rclcpp::Publisher<sensor_msgs::msg::Range>::SharedPtr range_pub_;
  rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr height_pub_;
  rclcpp::Publisher<std_msgs::msg::Float32MultiArray>::SharedPtr zones_pub_;
};

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<TofSenseM>());
  rclcpp::shutdown();
  return 0;
}
