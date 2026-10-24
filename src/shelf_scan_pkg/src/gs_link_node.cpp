// 机载端 <-> 地面站 链路桥
//
// 作用：把 ROS 话题上的一行一条 JSON 与真实链路对接：
//   - 局域网 WebSocket（推荐：同一 WiFi 下直连地面站，不用数传模块）
//   - 无线数传串口
//   - 局域网 UDP
//
//   /shelf_scan/gs_out  (std_msgs/String)  -->  链路发出
//   链路收到的一行 JSON                    -->  /shelf_scan/gs_in (std_msgs/String)
//
// 地面站侧配置见 groundstation/config/runtime.json：
//   websocket 0.0.0.0:8890/ws（本文件做客户端）/ 数传 COM9@115200 / UDP 8888
// 串口用自带的 SimpleSerial（poll 实现）：读写互不干扰、支持大报文（整条航线约 10KB）、
// 退出干净（读超时 200ms）。
// WebSocket 用 Boost.Beast（异步 + 自动重连）：航线条 JSON 走单帧 TCP，不会像 UDP 那样丢包。
#include <atomic>
#include <algorithm>
#include <chrono>
#include <condition_variable>
#include <cstring>
#include <deque>
#include <memory>
#include <mutex>
#include <string>
#include <thread>
#include <vector>

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>

#include "shelf_scan_pkg/simple_serial.hpp"

// Boost.Beast 是头文件库：CMake 打开 SHELF_ENABLE_WEBSOCKET 且头文件存在时
// 才编译 WebSocket 链路，否则只编译串口/UDP 链路（不影响其它功能）
#if defined(SHELF_WEBSOCKET_ENABLED) && defined(__has_include)
#  if __has_include(<boost/beast/core.hpp>) && __has_include(<boost/asio.hpp>)
#    define SHELF_HAS_BEAST 1
#    include <boost/asio/connect.hpp>
#    include <boost/asio/ip/tcp.hpp>
#    include <boost/asio/socket_base.hpp>
#    include <boost/asio/steady_timer.hpp>
#    include <boost/beast/core.hpp>
#    include <boost/beast/websocket.hpp>
#    include <boost/system/error_code.hpp>
#  endif
#endif

#ifndef _WIN32
#include <arpa/inet.h>
#include <netinet/in.h>
#include <sys/socket.h>
#include <unistd.h>
#endif

namespace
{

using SimpleSerial = shelf_scan::SimpleSerial;

// 单调时钟（秒）：给 WebSocket 看门狗计时，不依赖 ROS 时间
double steadySeconds()
{
  return std::chrono::duration<double>(
    std::chrono::steady_clock::now().time_since_epoch()).count();
}

#ifdef SHELF_HAS_BEAST
namespace beast = boost::beast;
namespace websocket = beast::websocket;
namespace asio = boost::asio;
using tcp = asio::ip::tcp;
using BoostError = boost::system::error_code;
#endif

// ws://host[:port][/path] -> (scheme, host, port, target)
bool parseWsUrl(const std::string & url, std::string & scheme, std::string & host,
                std::string & port, std::string & target)
{
  const std::size_t split = url.find("://");
  if (split == std::string::npos) return false;
  scheme = url.substr(0, split);
  for (char & ch : scheme) {
    if (ch >= 'A' && ch <= 'Z') ch = static_cast<char>(ch - 'A' + 'a');
  }
  std::string rest = url.substr(split + 3);
  const std::size_t slash = rest.find('/');
  std::string authority = rest;
  target = "/";
  if (slash != std::string::npos) {
    authority = rest.substr(0, slash);
    target = rest.substr(slash);
  }
  if (authority.empty()) return false;

  const std::size_t colon = authority.rfind(':');
  if (colon != std::string::npos && authority.find(']') == std::string::npos) {
    host = authority.substr(0, colon);
    port = authority.substr(colon + 1);
  } else if (authority.front() == '[') {           // IPv6 字面量
    const std::size_t close = authority.find(']');
    if (close == std::string::npos) return false;
    host = authority.substr(1, close - 1);
    port = (close + 2 <= authority.size()) ? authority.substr(close + 2) : "80";
  } else {
    host = authority;
    port = "80";
  }
  if (host.empty()) return false;
  if (port.empty()) port = "80";
  return true;
}

class GsLinkNode : public rclcpp::Node
{
public:
  GsLinkNode() : rclcpp::Node("shelf_gs_link_node")
  {
    device_id_ = declare_parameter<std::string>("device_id", "drone-01");
    firmware_ = declare_parameter<std::string>("firmware", "shelf-scan-1.0");
    gs_in_topic_ = declare_parameter<std::string>("gs_in_topic", "/shelf_scan/gs_in");
    gs_out_topic_ = declare_parameter<std::string>("gs_out_topic", "/shelf_scan/gs_out");
    status_topic_ =
      declare_parameter<std::string>("link_status_topic", "/shelf_scan/link_status");

    serial_enabled_ = declare_parameter<bool>("serial_enabled", true);
    serial_port_ = declare_parameter<std::string>("serial_port", "/dev/ttyUSB0");
    serial_baud_ = declare_parameter<int>("serial_baud", 115200);
    read_timeout_ms_ = declare_parameter<int>("serial_read_timeout_ms", 200);

    udp_enabled_ = declare_parameter<bool>("udp_enabled", false);
    udp_bind_ = declare_parameter<std::string>("udp_bind", "0.0.0.0");
    udp_port_ = declare_parameter<int>("udp_port", 8888);

    ws_enabled_ = declare_parameter<bool>("ws_enabled", false);
    ws_url_ = declare_parameter<std::string>("ws_url", "ws://192.168.1.100:8890/ws");
    ws_token_ = declare_parameter<std::string>("ws_token", "");
    ws_reconnect_sec_ = declare_parameter<double>("ws_reconnect_sec", 2.0);

    hello_period_sec_ = declare_parameter<double>("hello_period_sec", 2.0);
    stats_period_sec_ = declare_parameter<double>("stats_period_sec", 5.0);

    auto latched = rclcpp::QoS(rclcpp::KeepLast(1)).transient_local().reliable();
    gs_in_pub_ = create_publisher<std_msgs::msg::String>(gs_in_topic_, rclcpp::QoS(50).reliable());
    gs_out_pub_ = create_publisher<std_msgs::msg::String>(gs_out_topic_, rclcpp::QoS(100).reliable());
    status_pub_ = create_publisher<std_msgs::msg::String>(status_topic_, latched);
    gs_out_sub_ = create_subscription<std_msgs::msg::String>(
      gs_out_topic_, rclcpp::QoS(100).reliable(),
      std::bind(&GsLinkNode::outboundCallback, this, std::placeholders::_1));

    timer_ = create_wall_timer(
      std::chrono::duration<double>(1.0), std::bind(&GsLinkNode::timerCallback, this));

    if (serial_enabled_) {
      serial_ = std::make_shared<SimpleSerial>();
      reader_thread_ = std::thread([this]() { serialLoop(); });
    }
    if (udp_enabled_) {
      udp_thread_ = std::thread([this]() { udpLoop(); });
    }
    if (ws_enabled_) {
#ifdef SHELF_HAS_BEAST
      std::string scheme, host, port, target;
      if (parseWsUrl(ws_url_, scheme, host, port, target) && scheme == "ws") {
        ws_host_ = host;
        ws_port_ = port;
        ws_target_ = target;
        if (!ws_token_.empty()) {          // token 走查询串，地面站两种方式都认
          const bool has_query = ws_target_.find('?') != std::string::npos;
          ws_target_ += has_query ? "&" : "?";
          ws_target_ += "token=" + ws_token_;
        }
        ws_thread_ = std::thread([this]() { wsLoop(); });
      } else {
        ws_enabled_ = false;
        RCLCPP_ERROR(
          get_logger(), "ws_url 无效：%s（应形如 ws://192.168.1.100:8890/ws），"
          "WebSocket 链路已停用", ws_url_.c_str());
      }
#else
      ws_enabled_ = false;
      RCLCPP_ERROR(
        get_logger(),
        "本机编译时未启用 WebSocket（缺 Boost.Beast 或被 SHELF_NO_WEBSOCKET 关掉）："
        "sudo apt install libboost-dev 后重新编译；"
        "临时可改用 udp_enabled:=true（配 tools/ws_bridge.py）或数传串口");
#endif
    }
    writer_thread_ = std::thread([this]() { writeLoop(); });

    RCLCPP_INFO(
      get_logger(),
      "链路桥已启动：串口=%s(%s@%d) UDP=%s(%s:%d) WebSocket=%s(%s)",
      serial_enabled_ ? "启用" : "关闭", serial_port_.c_str(), serial_baud_,
      udp_enabled_ ? "启用" : "关闭", udp_bind_.c_str(), udp_port_,
      ws_enabled_ ? "启用" : "关闭", ws_url_.c_str());
    RCLCPP_INFO(
      get_logger(), "下发 %s -> 链路，链路 -> %s",
      gs_out_topic_.c_str(), gs_in_topic_.c_str());
  }

  ~GsLinkNode() override
  {
    running_ = false;
    tx_cv_.notify_all();
    if (writer_thread_.joinable()) writer_thread_.join();
    if (reader_thread_.joinable()) reader_thread_.join();
#ifndef _WIN32
    if (udp_thread_.joinable()) udp_thread_.join();
    if (udp_socket_ >= 0) ::close(udp_socket_);
#endif
#ifdef SHELF_HAS_BEAST
    if (ws_thread_.joinable()) {
      ws_ioc_.stop();            // 停 IO 线程，未完成的异步操作随之取消
      ws_thread_.join();
    }
#endif
    if (serial_) serial_->close();
  }

private:
  // ------------------------------------------------------------------ 发送
  void outboundCallback(const std_msgs::msg::String::SharedPtr message)
  {
    if (message->data.empty()) return;
    std::string line = message->data;
    if (line.back() != '\n') line.push_back('\n');
    {
      // 只入队，不在 ROS 回调里做串口写：避免链路拥塞时卡住飞控节拍
      std::lock_guard<std::mutex> lock(tx_mutex_);
      if (tx_queue_.size() > 500) tx_queue_.pop_front();
      tx_queue_.push_back(std::move(line));
    }
    tx_cv_.notify_one();
  }

  void writeLoop()
  {
    while (running_ && rclcpp::ok()) {
      std::string line;
      {
        std::unique_lock<std::mutex> lock(tx_mutex_);
        tx_cv_.wait_for(lock, std::chrono::milliseconds(200), [this]() {
          return !tx_queue_.empty() || !running_;
        });
        wsWatchdog();          // 空转时也每 200ms 检查一次链路是否已死
        if (tx_queue_.empty()) continue;
        line = std::move(tx_queue_.front());
        tx_queue_.pop_front();
      }
      if (serial_) {
        std::lock_guard<std::mutex> lock(serial_write_mutex_);
        if (serial_->is_open()) {
          if (serial_->write(line)) {
            tx_count_++;
          } else {
            RCLCPP_WARN_THROTTLE(
              get_logger(), *get_clock(), 3000, "串口发送失败：%s",
              serial_->last_error().c_str());
          }
        }
      }
      if (udp_enabled_) sendUdp(line);
      if (ws_enabled_) postWsLine(line);
    }
  }

  void sendUdp(const std::string & line)
  {
#ifndef _WIN32
    std::lock_guard<std::mutex> lock(udp_mutex_);
    if (udp_socket_ < 0 || !udp_peer_valid_) return;
    const ssize_t sent = ::sendto(
      udp_socket_, line.data(), line.size(), 0,
      reinterpret_cast<const sockaddr *>(&udp_peer_), sizeof(udp_peer_));
    if (sent > 0) tx_count_++;
#else
    (void)line;
#endif
  }

  // ------------------------------------------------------------------ 接收
  void serialLoop()
  {
    std::string buffer;
    std::vector<uint8_t> chunk(1024);
    while (running_ && rclcpp::ok()) {
      if (!serial_->is_open()) {
        {
          std::lock_guard<std::mutex> lock(serial_read_mutex_);
          if (!serial_->open(serial_port_, serial_baud_)) {
            RCLCPP_WARN_THROTTLE(
              get_logger(), *get_clock(), 5000,
              "打开串口 %s 失败：%s（5 秒后重试）", serial_port_.c_str(),
              serial_->last_error().c_str());
            if (first_open_failure_) {
              first_open_failure_ = false;
              std::string list;
              for (const auto & port : SimpleSerial::available_ports()) list += port + " ";
              RCLCPP_WARN(
                get_logger(), "当前可用串口：%s（数传模块常见为 /dev/ttyUSB0 或 /dev/ttyACM0）",
                list.empty() ? "（无）" : list.c_str());
            }
          } else {
            RCLCPP_INFO(
              get_logger(), "串口 %s 已打开（%d baud），开始接收地面站数据",
              serial_port_.c_str(), serial_baud_);
          }
        }
        if (!serial_->is_open()) {
          for (int i = 0; i < 50 && running_ && rclcpp::ok(); ++i) {
            std::this_thread::sleep_for(std::chrono::milliseconds(100));
          }
          continue;
        }
      }

      int bytes = 0;
      {
        std::lock_guard<std::mutex> lock(serial_read_mutex_);
        bytes = serial_->read(chunk.data(), chunk.size(), read_timeout_ms_);
      }
      if (bytes <= 0) {
        continue;
      }
      buffer.append(reinterpret_cast<const char *>(chunk.data()), static_cast<std::size_t>(bytes));
      std::size_t position = 0;
      while ((position = buffer.find('\n')) != std::string::npos) {
        const std::string line = buffer.substr(0, position);
        buffer.erase(0, position + 1);
        handleInboundLine(line);
      }
      // 地面站下发的整条 start_route 可达 30KB 以上（一行一条 JSON），
      // 缓冲区必须留足；只有在明显异常（超过 256KB 仍未换行）时才丢弃。
      if (buffer.size() > 262144) {
        RCLCPP_WARN(get_logger(), "接收缓冲区异常增大（%zu 字节），已清空", buffer.size());
        buffer.clear();
      }
    }
  }

  void udpLoop()
  {
#ifndef _WIN32
    udp_socket_ = ::socket(AF_INET, SOCK_DGRAM, 0);
    if (udp_socket_ < 0) {
      RCLCPP_ERROR(get_logger(), "创建 UDP 套接字失败");
      return;
    }
    sockaddr_in address;
    std::memset(&address, 0, sizeof(address));
    address.sin_family = AF_INET;
    address.sin_port = htons(static_cast<uint16_t>(udp_port_));
    if (udp_bind_ == "0.0.0.0") {
      address.sin_addr.s_addr = INADDR_ANY;
    } else {
      ::inet_pton(AF_INET, udp_bind_.c_str(), &address.sin_addr);
    }
    if (::bind(udp_socket_, reinterpret_cast<sockaddr *>(&address), sizeof(address)) < 0) {
      RCLCPP_ERROR(get_logger(), "UDP 绑定 %s:%d 失败", udp_bind_.c_str(), udp_port_);
      return;
    }
    RCLCPP_INFO(
      get_logger(), "UDP 已监听 %s:%d，收到第一个包后自动回发", udp_bind_.c_str(), udp_port_);

    while (running_ && rclcpp::ok()) {
      // 缓冲区要放得下整条 start_route（约 30KB 的 JSON），否则 UDP 方式
      // 会把航线截断成半条，机载端解析失败、飞机不动。
      char buffer[65536];
      sockaddr_in peer;
      socklen_t peer_length = sizeof(peer);
      const ssize_t bytes = ::recvfrom(
        udp_socket_, buffer, sizeof(buffer) - 1, 0,
        reinterpret_cast<sockaddr *>(&peer), &peer_length);
      if (bytes <= 0) continue;
      {
        std::lock_guard<std::mutex> lock(udp_mutex_);
        udp_peer_ = peer;
        udp_peer_valid_ = true;
      }
      std::string text(buffer, static_cast<std::size_t>(bytes));
      std::size_t position = 0;
      while (position < text.size()) {
        std::size_t next = text.find('\n', position);
        if (next == std::string::npos) next = text.size();
        const std::string line = text.substr(position, next - position);
        position = next + 1;
        if (!line.empty()) handleInboundLine(line);
      }
    }
#endif
  }

  // ------------------------------------------------------------- WebSocket
  // 同一 WiFi 下直连地面站（地面站是服务端，本节点是客户端）。
  // 所有 socket 操作都在 ws_thread_ 的 io_context 里跑，别的线程只 post 任务，
  // 断线后按 ws_reconnect_sec_ 自动重连，地面站那边不用做任何操作。
  // 看门狗：连着但连续 30s 一次都没发出去，说明链路已死（WiFi 掉了/地面站关机），
  // 主动断开重连，避免“socket 还开着、数据谁也收不到”的假连接。
  void wsWatchdog()
  {
#ifdef SHELF_HAS_BEAST
    if (!ws_enabled_ || !ws_connected_) return;
    const double last = ws_last_write_ok_.load();
    if (last < 0.0 || steadySeconds() - last <= 30.0) return;
    wsForceReconnect();
#endif
  }

  void wsForceReconnect()
  {
#ifdef SHELF_HAS_BEAST
    if (!ws_thread_.joinable()) return;
    asio::post(ws_ioc_, [this]() {
      if (!ws_connected_) return;
      wsFail("无响应", "连续 30s 一次都没发出去（看门狗判定链路已死）", ws_generation_);
    });
#endif
  }

  void postWsLine(const std::string & line)
  {
#ifdef SHELF_HAS_BEAST
    if (!ws_thread_.joinable() || !ws_connected_) return;
    asio::post(ws_ioc_, [this, line]() {
      std::string text = line;
      while (!text.empty() && (text.back() == '\n' || text.back() == '\r')) text.pop_back();
      if (text.empty() || !ws_connected_) return;
      if (ws_tx_queue_.size() > 500) ws_tx_queue_.pop_front();
      ws_tx_queue_.push_back(std::move(text));
      wsStartWrite(ws_generation_);
    });
#else
    (void)line;
#endif
  }

#ifdef SHELF_HAS_BEAST
  void wsLoop()
  {
    RCLCPP_INFO(
      get_logger(), "WebSocket 客户端启动：%s（断线自动重连，间隔 %.1fs）",
      ws_url_.c_str(), ws_reconnect_sec_);
    asio::post(ws_ioc_, [this]() { wsStartConnect(); });
    ws_ioc_.run();
  }

  void wsStartConnect()
  {
    if (!running_ || !rclcpp::ok()) return;
    const unsigned long generation = ++ws_generation_;   // 丢弃旧连接的迟到回调
    ws_stream_.reset();
    ws_buffer_.consume(ws_buffer_.size());
    ws_tx_queue_.clear();
    ws_writing_ = false;

    BoostError resolve_error;
    const tcp::resolver::results_type endpoints =
      ws_resolver_.resolve(ws_host_, ws_port_, resolve_error);
    if (resolve_error) {
      wsFail("解析地址", resolve_error.message(), generation);
      return;
    }
    ws_stream_ = std::make_unique<websocket::stream<tcp::socket>>(ws_ioc_);
    asio::async_connect(
      ws_stream_->next_layer(), endpoints,
      [this, generation](BoostError error, const tcp::endpoint &) {
        wsOnConnect(error, generation);
      });
  }

  void wsOnConnect(BoostError error, unsigned long generation)
  {
    if (generation != ws_generation_) return;
    if (error) {
      wsFail("连接", error.message(), generation);
      return;
    }
    // TCP 保活 + 应用层看门狗（见 writeLoop 里的 wsWatchdog）双重保障：
    // 地面站断线/WiFi 掉线时本端最多 30s 就会发现并重连。
    BoostError keepalive_error;
    ws_stream_->next_layer().set_option(asio::socket_base::keep_alive(true), keepalive_error);
    ws_stream_->async_handshake(
      ws_host_, ws_target_,
      [this, generation](BoostError handshake_error) {
        wsOnHandshake(handshake_error, generation);
      });
  }

  void wsOnHandshake(BoostError error, unsigned long generation)
  {
    if (generation != ws_generation_) return;
    if (error) {
      wsFail("握手", error.message(), generation);
      return;
    }
    ws_connected_ = true;
    ws_last_write_ok_.store(steadySeconds());      // 看门狗基准时间
    ws_stream_->text(true);          // 我们只发文本帧（一行一条 JSON）
    RCLCPP_INFO(get_logger(), "已连接地面站 WebSocket：%s", ws_url_.c_str());
    wsStartRead(generation);
  }

  void wsStartRead(unsigned long generation)
  {
    if (!ws_stream_ || !ws_connected_ || generation != ws_generation_) return;
    ws_buffer_.consume(ws_buffer_.size());
    ws_stream_->async_read(
      ws_buffer_, [this, generation](BoostError error, std::size_t) {
        wsOnRead(error, generation);
      });
  }

  void wsOnRead(BoostError error, unsigned long generation)
  {
    if (generation != ws_generation_) return;
    if (error) {
      wsFail("接收", error.message(), generation);
      return;
    }
    const std::string text = beast::buffers_to_string(ws_buffer_.data());
    ws_buffer_.consume(ws_buffer_.size());
    std::size_t position = 0;
    while (position <= text.size()) {
      std::size_t next = text.find('\n', position);
      if (next == std::string::npos) next = text.size();
      const std::string line = text.substr(position, next - position);
      position = next + 1;
      if (!line.empty()) handleInboundLine(line);
      if (next >= text.size()) break;
    }
    wsStartRead(generation);
  }

  void wsStartWrite(unsigned long generation)
  {
    if (!ws_stream_ || !ws_connected_ || generation != ws_generation_ ||
      ws_writing_ || ws_tx_queue_.empty())
    {
      return;
    }
    ws_writing_ = true;
    ws_stream_->async_write(
      asio::buffer(ws_tx_queue_.front()),
      [this, generation](BoostError error, std::size_t) {
        if (generation != ws_generation_) return;
        ws_writing_ = false;
        if (error) {
          wsFail("发送", error.message(), generation);
          return;
        }
        tx_count_++;
        ws_last_write_ok_.store(steadySeconds());
        if (!ws_tx_queue_.empty()) ws_tx_queue_.pop_front();
        wsStartWrite(generation);
      });
  }

  // detail 直接用字符串：避免依赖 Boost.System 的 errc 头
  //（本机 boost 1.74 精简包里没有 boost/system/errc.hpp）
  void wsFail(const char * what, const std::string & detail, unsigned long generation)
  {
    if (generation != ws_generation_) return;
    const bool was_connected = ws_connected_.exchange(false);
    ws_last_write_ok_.store(-1.0);
    ws_writing_ = false;
    ws_tx_queue_.clear();
    if (ws_stream_) {
      BoostError ignored;
      ws_stream_->next_layer().close(ignored);
      ws_stream_.reset();
    }
    if (was_connected) {
      RCLCPP_WARN(
        get_logger(), "WebSocket %s失败：%s（%.1fs 后重连）",
        what, detail.c_str(), ws_reconnect_sec_);
    } else {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 10000,
        "WebSocket 未连接：%s失败：%s（重试中，检查地面站地址/防火墙：%s）",
        what, detail.c_str(), ws_url_.c_str());
    }
    if (!running_ || !rclcpp::ok()) return;
    ws_timer_.expires_after(std::chrono::milliseconds(
      static_cast<int>(std::max(0.5, ws_reconnect_sec_) * 1000.0)));
    ws_timer_.async_wait([this](BoostError timer_error) {
      if (!timer_error && running_ && rclcpp::ok()) wsStartConnect();
    });
  }
#endif  // SHELF_HAS_BEAST

  void handleInboundLine(const std::string & raw_line)
  {
    std::string line = raw_line;
    while (!line.empty() && (line.back() == '\r' || line.back() == ' ')) line.pop_back();
    if (line.empty()) return;

    rx_count_++;
    last_rx_time_ = now();
    has_rx_ = true;
    if (line.front() != '{') {
      RCLCPP_DEBUG(get_logger(), "忽略非 JSON 数据：%s", line.c_str());
      return;
    }
    std_msgs::msg::String message;
    message.data = line;
    gs_in_pub_->publish(message);
    RCLCPP_DEBUG(get_logger(), "<- 地面站：%s", line.c_str());
  }

  // ------------------------------------------------------------------ 周期
  void timerCallback()
  {
    const rclcpp::Time now_time = now();
    const double hello_interval = has_rx_ ? 10.0 : std::max(0.5, hello_period_sec_);
    if ((now_time - last_hello_time_).seconds() >= hello_interval) {
      last_hello_time_ = now_time;
      publishLine(
        "{\"type\":\"hello\",\"device\":\"" + device_id_ + "\",\"fw\":\"" + firmware_ + "\"}");
    }
    if ((now_time - last_stats_time_).seconds() >= stats_period_sec_) {
      last_stats_time_ = now_time;
      const bool serial_ok = serial_ && serial_->is_open();
#ifdef SHELF_HAS_BEAST
      const bool ws_ok = ws_enabled_ && ws_connected_;
#else
      const bool ws_ok = false;
#endif
      std_msgs::msg::String message;
      message.data = std::string("{\"rx\":") + std::to_string(rx_count_) +
        ",\"tx\":" + std::to_string(tx_count_) +
        ",\"serial\":" + (serial_ok ? "true" : "false") +
        ",\"udp\":" + (udp_enabled_ ? "true" : "false") +
        ",\"ws\":" + (ws_ok ? "true" : "false") + "}";
      status_pub_->publish(message);
      if (!has_rx_) {
        RCLCPP_INFO_THROTTLE(
          get_logger(), *get_clock(), 30000,
          "链路统计：接收 %lu 条，发送 %lu 条（尚未收到地面站数据："
          "检查 ws_url/数传端口、是否与地面站同一网段、防火墙）",
          static_cast<unsigned long>(rx_count_), static_cast<unsigned long>(tx_count_));
      } else {
        RCLCPP_DEBUG(
          get_logger(), "链路统计：接收 %lu 条，发送 %lu 条",
          static_cast<unsigned long>(rx_count_), static_cast<unsigned long>(tx_count_));
      }
    }
  }

  void publishLine(const std::string & line)
  {
    std_msgs::msg::String message;
    message.data = line;
    gs_out_pub_->publish(message);
  }

  std::shared_ptr<SimpleSerial> serial_;
  std::thread writer_thread_;
  std::thread reader_thread_;
  std::thread udp_thread_;
#ifdef SHELF_HAS_BEAST
  // WebSocket 成员：io_context / socket 只允许 ws_thread_ 触碰，
  // 其它线程通过 asio::post 投递任务（ws_tx_queue_ 也只在 IO 线程里改）
  asio::io_context ws_ioc_;
  tcp::resolver ws_resolver_{ws_ioc_};
  asio::steady_timer ws_timer_{ws_ioc_};
  std::unique_ptr<websocket::stream<tcp::socket>> ws_stream_;
  beast::flat_buffer ws_buffer_;
  std::deque<std::string> ws_tx_queue_;
  unsigned long ws_generation_ = 0;
  bool ws_writing_ = false;
  std::atomic<bool> ws_connected_{false};
  std::atomic<double> ws_last_write_ok_{-1.0};   // 单调时钟秒；-1 = 尚未连接
  std::thread ws_thread_;
  std::string ws_host_;
  std::string ws_port_;
  std::string ws_target_;
#endif
  std::mutex serial_read_mutex_;
  std::mutex serial_write_mutex_;
  std::mutex udp_mutex_;
  std::mutex tx_mutex_;
  std::condition_variable tx_cv_;
  std::deque<std::string> tx_queue_;
  std::atomic<bool> running_{true};
  std::atomic<bool> has_rx_{false};
  std::atomic<unsigned long> rx_count_{0};
  std::atomic<unsigned long> tx_count_{0};
  bool first_open_failure_ = true;
#ifndef _WIN32
  int udp_socket_ = -1;
  bool udp_peer_valid_ = false;
  sockaddr_in udp_peer_{};
#endif

  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr gs_in_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr gs_out_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr status_pub_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr gs_out_sub_;
  rclcpp::TimerBase::SharedPtr timer_;

  std::string device_id_;
  std::string firmware_;
  std::string gs_in_topic_;
  std::string gs_out_topic_;
  std::string status_topic_;
  std::string serial_port_;
  std::string udp_bind_;
  std::string ws_url_;
  std::string ws_token_;
  int serial_baud_ = 115200;
  int read_timeout_ms_ = 200;
  int udp_port_ = 8888;
  bool serial_enabled_ = true;
  bool udp_enabled_ = false;
  bool ws_enabled_ = false;
  double ws_reconnect_sec_ = 2.0;
  double hello_period_sec_ = 2.0;
  double stats_period_sec_ = 5.0;
  rclcpp::Time last_hello_time_{0, 0, RCL_ROS_TIME};
  rclcpp::Time last_stats_time_{0, 0, RCL_ROS_TIME};
  rclcpp::Time last_rx_time_{0, 0, RCL_ROS_TIME};
};

}  // namespace

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<GsLinkNode>());
  rclcpp::shutdown();
  return 0;
}
