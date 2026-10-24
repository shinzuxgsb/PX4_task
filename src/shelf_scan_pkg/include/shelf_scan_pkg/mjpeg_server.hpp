// 机载 MJPEG 预览服务（2026-09-19 新增）
//
// 用途：让识别节点把「带检测框的画面」直接以 MJPEG over HTTP 吐出去，电脑浏览器
// 打开 http://<香橙派IP>:<port>/ 就能看实时画面，电脑上不用装任何软件。
//
// 为什么放在识别节点里，而不是另起一个进程去读相机：
//   UVC 相机（/dev/videoX）同一时刻只允许一个进程打开，识别节点已经占着它。
//   在本节点里顺带把「已经画好标注」的帧发出去，既不用抢占相机，也不需要
//   v4l2loopback 之类的转发层。
//
// 实现要点：
//   · 只认 /（极简 HTML 页面）和 /stream（MJPEG）；其余返回 404。
//   · 没有任何客户端连接时完全不编码、不发送，对识别主流程零开销。
//   · 客户端断开（关标签页）就地摘除，不抛异常、不影响识别。
//   · 收发都带超时 + MSG_NOSIGNAL：慢客户端最多拖 1 秒就被摘掉，
//     对方断开也不会用 SIGPIPE 把整个识别节点打死。
#ifndef SHELF_SCAN_PKG__MJPEG_SERVER_HPP_
#define SHELF_SCAN_PKG__MJPEG_SERVER_HPP_

#include <arpa/inet.h>
#include <netinet/in.h>
#include <netinet/tcp.h>
#include <sys/socket.h>
#include <sys/time.h>
#include <unistd.h>

#include <atomic>
#include <cerrno>
#include <cstddef>
#include <cstdint>
#include <mutex>
#include <string>
#include <thread>
#include <vector>

#include <opencv2/core.hpp>
#include <opencv2/imgcodecs.hpp>

namespace shelf_scan_pkg
{

class MjpegServer
{
public:
  MjpegServer() = default;
  ~MjpegServer() { stop(); }

  MjpegServer(const MjpegServer &) = delete;
  MjpegServer & operator=(const MjpegServer &) = delete;

  // 返回 false 表示端口被占用/权限不足。调用方只应打警告，不要让节点启动失败。
  bool start(int port, const std::string & title)
  {
    title_ = title;
    listen_fd_ = ::socket(AF_INET, SOCK_STREAM, 0);
    if (listen_fd_ < 0) return false;

    int reuse = 1;
    ::setsockopt(listen_fd_, SOL_SOCKET, SO_REUSEADDR, &reuse, sizeof(reuse));

    sockaddr_in address{};
    address.sin_family = AF_INET;
    address.sin_addr.s_addr = htonl(INADDR_ANY);
    address.sin_port = htons(static_cast<std::uint16_t>(port));

    if (::bind(listen_fd_, reinterpret_cast<sockaddr *>(&address), sizeof(address)) < 0 ||
      ::listen(listen_fd_, 8) < 0)
    {
      ::close(listen_fd_);
      listen_fd_ = -1;
      return false;
    }

    port_ = port;
    running_ = true;
    accept_thread_ = std::thread(&MjpegServer::acceptLoop, this);
    return true;
  }

  void stop()
  {
    if (!running_.exchange(false)) return;
    if (listen_fd_ >= 0) {
      // shutdown 一个监听中的 socket 会让阻塞在 accept() 的线程立刻返回。
      ::shutdown(listen_fd_, SHUT_RDWR);
      ::close(listen_fd_);
      listen_fd_ = -1;
    }
    if (accept_thread_.joinable()) accept_thread_.join();

    std::lock_guard<std::mutex> lock(mutex_);
    for (const int fd : clients_) ::close(fd);
    clients_.clear();
  }

  std::size_t clientCount() const
  {
    std::lock_guard<std::mutex> lock(mutex_);
    return clients_.size();
  }

  bool hasClients() const { return clientCount() > 0; }

  int port() const { return port_; }

  // 把一帧 BGR 图编码成 JPEG 推给所有已连接的浏览器。
  // 没有客户端时直接返回，不做编码 —— 这是「不影响识别」的关键。
  bool publish(const cv::Mat & frame, int quality)
  {
    if (frame.empty()) return false;
    if (!hasClients()) return false;

    std::vector<unsigned char> jpeg;
    const std::vector<int> parameters{cv::IMWRITE_JPEG_QUALITY, quality};
    if (!cv::imencode(".jpg", frame, jpeg, parameters)) return false;

    const std::string head =
      "--" + std::string(boundary()) + "\r\n"
      "Content-Type: image/jpeg\r\n"
      "Content-Length: " + std::to_string(jpeg.size()) + "\r\n\r\n";

    std::vector<unsigned char> chunk;
    chunk.reserve(head.size() + jpeg.size() + 2);
    chunk.insert(chunk.end(), head.begin(), head.end());
    chunk.insert(chunk.end(), jpeg.begin(), jpeg.end());
    chunk.push_back('\r');
    chunk.push_back('\n');

    std::lock_guard<std::mutex> lock(mutex_);
    for (auto it = clients_.begin(); it != clients_.end(); ) {
      if (sendAll(*it, chunk.data(), chunk.size())) {
        ++it;
      } else {
        ::close(*it);
        it = clients_.erase(it);
      }
    }
    return true;
  }

private:
  static const char * boundary() { return "shelfscanframe"; }

  void acceptLoop()
  {
    while (running_) {
      sockaddr_in peer{};
      socklen_t peer_length = sizeof(peer);
      const int fd = ::accept(listen_fd_, reinterpret_cast<sockaddr *>(&peer), &peer_length);
      if (fd < 0) {
        if (!running_) break;
        if (errno == EINTR) continue;
        break;
      }
      handleClient(fd);
    }
  }

  void handleClient(int fd)
  {
    int nodelay = 1;
    ::setsockopt(fd, IPPROTO_TCP, TCP_NODELAY, &nodelay, sizeof(nodelay));
    // 读写超时：慢客户端最多拖住 1 秒就被摘掉，不会把识别主循环卡死。
    timeval timeout{1, 0};
    ::setsockopt(fd, SOL_SOCKET, SO_RCVTIMEO, &timeout, sizeof(timeout));
    ::setsockopt(fd, SOL_SOCKET, SO_SNDTIMEO, &timeout, sizeof(timeout));

    char buffer[1024] = {0};
    const ssize_t received = ::recv(fd, buffer, sizeof(buffer) - 1, 0);
    if (received <= 0) {
      ::close(fd);
      return;
    }

    const std::string path =
      requestPath(std::string(buffer, static_cast<std::size_t>(received)));

    if (path == "/" || path == "/index.html") {
      const std::string page = htmlPage();
      sendAll(fd, page.data(), page.size());
      ::close(fd);
      return;
    }

    if (path == "/stream" || path == "/stream.mjpg" || path == "/cam.mjpg") {
      const std::string header =
        "HTTP/1.1 200 OK\r\n"
        "Content-Type: multipart/x-mixed-replace; boundary=" + std::string(boundary()) +
        "\r\n"
        "Cache-Control: no-store, no-cache, must-revalidate\r\n"
        "Pragma: no-cache\r\n"
        "Connection: close\r\n"
        "\r\n";
      if (!sendAll(fd, header.data(), header.size())) {
        ::close(fd);
        return;
      }
      std::lock_guard<std::mutex> lock(mutex_);
      clients_.push_back(fd);
      return;
    }

    const std::string not_found =
      "HTTP/1.1 404 Not Found\r\n"
      "Content-Type: text/plain; charset=utf-8\r\n"
      "Content-Length: 15\r\n"
      "Connection: close\r\n"
      "\r\n"
      "404 not found\r\n";
    sendAll(fd, not_found.data(), not_found.size());
    ::close(fd);
  }

  static std::string requestPath(const std::string & request)
  {
    const std::size_t first = request.find(' ');
    if (first == std::string::npos) return "/";
    const std::size_t second = request.find(' ', first + 1);
    if (second == std::string::npos) return "/";
    std::string path = request.substr(first + 1, second - first - 1);
    const std::size_t query = path.find('?');
    if (query != std::string::npos) path.resize(query);
    return path;
  }

  static bool sendAll(int fd, const void * data, std::size_t size)
  {
    const char * bytes = static_cast<const char *>(data);
    std::size_t sent = 0;
    while (sent < size) {
      const ssize_t written = ::send(fd, bytes + sent, size - sent, MSG_NOSIGNAL);
      if (written > 0) {
        sent += static_cast<std::size_t>(written);
        continue;
      }
      if (written < 0 && errno == EINTR) continue;
      return false;
    }
    return true;
  }

  static std::string httpResponse(
    const std::string & content_type, const std::string & body)
  {
    return "HTTP/1.1 200 OK\r\nContent-Type: " + content_type +
      "\r\nContent-Length: " + std::to_string(body.size()) +
      "\r\nCache-Control: no-store\r\nConnection: close\r\n\r\n" + body;
  }

  std::string htmlPage() const
  {
    const std::string body =
      "<!doctype html>\n"
      "<html lang=\"zh\"><head><meta charset=\"utf-8\">\n"
      "<meta name=\"viewport\" content=\"width=device-width,initial-scale=1\">\n"
      "<title>" + title_ + "</title>\n"
      "<style>\n"
      "  html,body{margin:0;height:100%;background:#0d0d0d;color:#cfcfcf;"
      "font:14px/1.5 system-ui,'Microsoft YaHei',sans-serif}\n"
      "  header{padding:8px 14px;border-bottom:1px solid #262626;display:flex;"
      "justify-content:space-between;align-items:center}\n"
      "  #state{color:#8f8}\n"
      "  .stage{height:calc(100% - 43px);display:flex;align-items:center;"
      "justify-content:center}\n"
      "  img{max-width:100%;max-height:100%;background:#000}\n"
      "</style></head><body>\n"
      "<header><span>" + title_ + "</span><span id=\"state\">连接中…</span></header>\n"
      "<div class=\"stage\"><img id=\"view\" src=\"/stream\" alt=\"识别画面\"></div>\n"
      "<script>\n"
      "var v=document.getElementById('view'),s=document.getElementById('state');\n"
      "v.onload=function(){s.textContent='已连接';};\n"
      "v.onerror=function(){s.textContent='已断开，重连中…';\n"
      "  setTimeout(function(){v.src='/stream?t='+Date.now();},1000);};\n"
      "</script></body></html>\n";
    return httpResponse("text/html; charset=utf-8", body);
  }

  std::string title_ = "预览";
  int port_ = 0;
  int listen_fd_ = -1;
  std::atomic<bool> running_{false};
  std::thread accept_thread_;
  mutable std::mutex mutex_;
  std::vector<int> clients_;
};

}  // namespace shelf_scan_pkg

#endif  // SHELF_SCAN_PKG__MJPEG_SERVER_HPP_
