// 二维码 / AprilTag 识别节点（2024 电赛 D 题）
//
// 两种工作方式：
//   1) 连续识别：飞行中每帧检测货架上的二维码，把货物编号发布到
//      /shelf_scan/code  ->  std_msgs/Float32MultiArray = [编号, 置信度, x误差px, y误差px]
//      同时（可选）按原工程约定发布 /fine_data，供 pid_control_pkg 视觉对准使用。
//   2) 单次抓取（要求 2-1）：起飞前识别抽到的那张二维码，把编号发布到
//      /shelf_scan/target_id  ->  std_msgs/Int32（latched），任务节点会转发给地面站。
//      触发方式三选一：
//        · 启动参数 auto_capture_target:=true（节点启动后自动识别一次）
//        · 服务：ros2 service call /shelf_scan/capture_target std_srvs/srv/Trigger
//        · 话题：ros2 topic pub --once /shelf_scan/capture_trigger std_msgs/msg/Empty
//        · 图片文件：参数 target_image_path:=/path/to/picked.png
#include <algorithm>
#include <atomic>
#include <chrono>
#include <cmath>
#include <condition_variable>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <map>
#include <memory>
#include <mutex>
#include <stdexcept>
#include <string>
#include <thread>
#include <utility>
#include <vector>

#include <opencv2/aruco.hpp>
#include <opencv2/core.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/videoio.hpp>
#include <zbar.h>

#include "shelf_scan_pkg/mjpeg_server.hpp"

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/empty.hpp>
#include <std_msgs/msg/float32_multi_array.hpp>
#include <std_msgs/msg/int32.hpp>
#include <std_msgs/msg/int32_multi_array.hpp>
#include <std_srvs/srv/trigger.hpp>

namespace
{

using namespace zbar;

class CodeReaderNode : public rclcpp::Node
{
public:
  CodeReaderNode() : rclcpp::Node("shelf_code_reader_node")
  {
    zbar_scanner_ = zbar_image_scanner_create();
    if (zbar_scanner_ == nullptr) {
      throw std::runtime_error("无法创建 ZBar 二维码扫描器");
    }
    zbar_image_scanner_set_config(zbar_scanner_, ZBAR_QRCODE, ZBAR_CFG_ENABLE, 1);

    camera_device_ = declare_parameter<std::string>("camera_device", "/dev/video0");
    frame_width_ = declare_parameter<int>("frame_width", 640);
    frame_height_ = declare_parameter<int>("frame_height", 480);
    fps_ = declare_parameter<double>("fps", 120.0);
    detect_fps_ = declare_parameter<double>("detect_fps", 15.0);
    show_preview_ = declare_parameter<bool>("show_preview", false);
    code_mode_ = declare_parameter<std::string>("code_mode", "both");
    apriltag_dictionary_name_ =
      declare_parameter<std::string>("apriltag_dictionary", "DICT_APRILTAG_36h11");
    tag_id_as_number_ = declare_parameter<bool>("tag_id_as_number", true);
    number_min_ = declare_parameter<int>("number_min", 1);
    number_max_ = declare_parameter<int>("number_max", 24);
    publish_fine_data_ = declare_parameter<bool>("publish_fine_data", true);

    // ---- 电脑浏览器实时预览（MJPEG over HTTP）----
    preview_enabled_ = declare_parameter<bool>("preview_enabled", false);
    preview_port_ = declare_parameter<int>("preview_port", 8080);
    preview_fps_ = declare_parameter<double>("preview_fps", 30.0);
    preview_quality_ = declare_parameter<int>("preview_quality", 70);
    preview_draw_ = declare_parameter<bool>("preview_draw", true);

    // 二维码的物理边长（题目：打印在 A4 纸上，19cm×19cm 左右）。
    // 用它把"实测像素边长"换算成比例尺 px/cm，从而把画面偏差换成厘米——
    // 这样不管飞机离货架多远都能算，不需要现场标定。
    qr_size_cm_ = declare_parameter<double>("qr_size_cm", 19.0);
    // 摄像头倒装（上下颠倒）：图像被旋转了 180°。
    // 直接在读数之后把整帧转回来，这样识别、dx/dy、容差圆、预览、以及下游
    // （任务节点的 vscan 对准、视觉伺服）全部自动按"正装"约定工作，下游一行都不用改。
    // 两个开关独立，方便处理"只反了一个方向"的情况（物理倒装通常两个都是 true）。
    camera_flip_ud_ = declare_parameter<bool>("camera_flip_ud", true);   // 上下反
    camera_flip_lr_ = declare_parameter<bool>("camera_flip_lr", true);   // 左右反
    // 预览上画的容差圆半径（cm），取值应与任务节点的 laser_align_radius_cm 一致，
    // 只影响画出来的圈，不参与识别。
    align_radius_cm_ = declare_parameter<double>("align_radius_cm", 5.0);

    code_topic_ = declare_parameter<std::string>("code_topic", "/shelf_scan/code");
    target_id_topic_ =
      declare_parameter<std::string>("target_id_topic", "/shelf_scan/target_id");
    fine_data_topic_ = declare_parameter<std::string>("fine_data_topic", "/fine_data");
    trigger_topic_ =
      declare_parameter<std::string>("capture_trigger_topic", "/shelf_scan/capture_trigger");
    service_name_ =
      declare_parameter<std::string>("capture_service", "/shelf_scan/capture_target");

    target_image_path_ = declare_parameter<std::string>("target_image_path", "");
    auto_capture_target_ = declare_parameter<bool>("auto_capture_target", false);
    auto_capture_delay_sec_ = declare_parameter<double>("auto_capture_delay_sec", 3.0);
    capture_frames_ = declare_parameter<int>("capture_frames", 12);
    capture_interval_sec_ = declare_parameter<double>("capture_interval_sec", 0.08);

    if (code_mode_ != "qr" && code_mode_ != "apriltag" && code_mode_ != "both") {
      throw std::invalid_argument("code_mode 必须是 qr / apriltag / both");
    }
    if (apriltag_dictionary_name_ != "DICT_APRILTAG_36h11") {
      RCLCPP_WARN(
        get_logger(), "仅验证过 DICT_APRILTAG_36h11，当前为 %s",
        apriltag_dictionary_name_.c_str());
    }
    apriltag_dictionary_ = cv::aruco::getPredefinedDictionary(cv::aruco::DICT_APRILTAG_36h11);
    apriltag_parameters_ = cv::aruco::DetectorParameters::create();
    apriltag_parameters_->adaptiveThreshWinSizeMin = 3;
    apriltag_parameters_->adaptiveThreshWinSizeMax = 53;
    apriltag_parameters_->adaptiveThreshWinSizeStep = 4;
    apriltag_parameters_->minMarkerPerimeterRate = 0.02;
    apriltag_parameters_->cornerRefinementMethod = 3;
    apriltag_parameters_->errorCorrectionRate = 0.6;

    code_pub_ = create_publisher<std_msgs::msg::Float32MultiArray>(code_topic_, rclcpp::QoS(10));
    auto latched = rclcpp::QoS(rclcpp::KeepLast(1)).transient_local().reliable();
    target_id_pub_ = create_publisher<std_msgs::msg::Int32>(target_id_topic_, latched);
    fine_data_pub_ = create_publisher<std_msgs::msg::Int32MultiArray>(
      fine_data_topic_, rclcpp::QoS(10));
    trigger_sub_ = create_subscription<std_msgs::msg::Empty>(
      trigger_topic_, rclcpp::QoS(10),
      [this](std_msgs::msg::Empty::SharedPtr) { captureTarget("触发话题"); });
    capture_service_ = create_service<std_srvs::srv::Trigger>(
      service_name_,
      [this](
        const std::shared_ptr<std_srvs::srv::Trigger::Request>,
        std::shared_ptr<std_srvs::srv::Trigger::Response> response) {
        const int number = captureTarget("服务调用");
        response->success = number >= number_min_;
        response->message = number >= number_min_ ?
          ("识别到货物编号 " + std::to_string(number)) : "未识别到二维码";
      });

    if (!openCamera()) {
      throw std::runtime_error("相机打开失败：" + camera_device_);
    }

    worker_running_ = true;
    capture_thread_ = std::thread(&CodeReaderNode::captureLoop, this);
    detect_thread_ = std::thread(&CodeReaderNode::detectLoop, this);
    preview_thread_ = std::thread(&CodeReaderNode::previewLoop, this);

    if (auto_capture_target_) {
      capture_timer_ = create_wall_timer(
        std::chrono::duration<double>(std::max(0.5, auto_capture_delay_sec_)),
        [this]() {
          capture_timer_->cancel();
          captureTarget("启动自动识别");
        });
    }

    RCLCPP_INFO(
      get_logger(),
      "二维码识别节点已启动：设备=%s 模式=%s 编号范围=[%d,%d] 连续识别话题=%s",
      camera_device_.c_str(), code_mode_.c_str(), number_min_, number_max_, code_topic_.c_str());
    RCLCPP_INFO(
      get_logger(),
      "异步图像管线：采集目标=%.0f fps，识别目标=%.0f fps，地面站预览=%.0f fps",
      fps_, detect_fps_, preview_fps_);
    RCLCPP_INFO(
      get_logger(),
      "要求 2-1 单次识别：服务 %s 或话题 %s 或参数 auto_capture_target:=true",
      service_name_.c_str(), trigger_topic_.c_str());
    RCLCPP_INFO(
      get_logger(),
      "比例尺标定：二维码物理边长 %.1fcm -> code 话题额外发布 px/cm 和厘米偏差（第 6~8 个值）",
      qr_size_cm_);
    RCLCPP_INFO(
      get_logger(), "图像方向：上下翻转=%s 左右翻转=%s（倒装时读帧后立即转正）",
      camera_flip_ud_ ? "是" : "否", camera_flip_lr_ ? "是" : "否");

    if (preview_enabled_) {
      if (preview_.start(preview_port_, "D题货架盘点 · 机载识别画面")) {
        RCLCPP_INFO(
          get_logger(),
          "MJPEG 预览已开启：电脑浏览器打开 http://<香橙派IP>:%d/ （画面带检测框，"
          "%.0f fps，JPEG 质量 %d）",
          preview_port_, preview_fps_, preview_quality_);
      } else {
        RCLCPP_WARN(
          get_logger(), "MJPEG 预览启动失败（端口 %d 被占用？），本次不提供预览",
          preview_port_);
        preview_enabled_ = false;
      }
    }
  }

  ~CodeReaderNode() override
  {
    worker_running_ = false;
    frame_ready_cv_.notify_all();
    if (capture_thread_.joinable()) capture_thread_.join();
    if (detect_thread_.joinable()) detect_thread_.join();
    if (preview_thread_.joinable()) preview_thread_.join();
    preview_.stop();
    if (zbar_scanner_ != nullptr) zbar_image_scanner_destroy(zbar_scanner_);
    if (camera_.isOpened()) camera_.release();
    if (show_preview_) cv::destroyAllWindows();
  }

private:
  bool openCamera()
  {
    if (!camera_.open(camera_device_, cv::CAP_V4L2)) {
      RCLCPP_ERROR(
        get_logger(),
        "无法打开相机 %s（是否被 drone_camera_pkg 占用？用 fuser %s 检查）",
        camera_device_.c_str(), camera_device_.c_str());
      return false;
    }
    camera_.set(
      cv::CAP_PROP_FOURCC,
      cv::VideoWriter::fourcc('M', 'J', 'P', 'G'));
    if (frame_width_ > 0) camera_.set(cv::CAP_PROP_FRAME_WIDTH, frame_width_);
    if (frame_height_ > 0) camera_.set(cv::CAP_PROP_FRAME_HEIGHT, frame_height_);
    if (fps_ > 0.0) camera_.set(cv::CAP_PROP_FPS, fps_);
    const int fourcc = static_cast<int>(camera_.get(cv::CAP_PROP_FOURCC));
    RCLCPP_INFO(
      get_logger(),
      "相机实际参数：%.0fx%.0f @ %.1f fps，fourcc=%c%c%c%c",
      camera_.get(cv::CAP_PROP_FRAME_WIDTH), camera_.get(cv::CAP_PROP_FRAME_HEIGHT),
      camera_.get(cv::CAP_PROP_FPS),
      static_cast<char>(fourcc & 0xff),
      static_cast<char>((fourcc >> 8) & 0xff),
      static_cast<char>((fourcc >> 16) & 0xff),
      static_cast<char>((fourcc >> 24) & 0xff));
    return true;
  }

  cv::Mat preprocess(const cv::Mat & frame) const
  {
    cv::Mat gray;
    cv::cvtColor(frame, gray, cv::COLOR_BGR2GRAY);
    cv::Mat equalized;
    auto clahe = cv::createCLAHE(2.0, cv::Size(8, 8));
    clahe->apply(gray, equalized);
    cv::Mat blurred;
    cv::Mat sharpened;
    cv::GaussianBlur(equalized, blurred, cv::Size(0, 0), 1.0);
    cv::addWeighted(equalized, 2.0, blurred, -1.0, 0.0, sharpened);
    return sharpened;
  }

  // 摄像头倒装时把整帧转回"正装"方向。放在读帧之后立刻调用，
  // 这样后面所有环节（识别 / dx dy / 容差圆 / 预览 / 下游任务与伺服）看到的
  // 都是正常方向，符号约定天然一致。
  void applyOrientation(cv::Mat & frame) const
  {
    if (frame.empty()) return;
    if (camera_flip_ud_ && camera_flip_lr_) {
      cv::rotate(frame, frame, cv::ROTATE_180);
    } else if (camera_flip_ud_) {
      cv::flip(frame, frame, 0);
    } else if (camera_flip_lr_) {
      cv::flip(frame, frame, 1);
    }
  }

  bool parseNumber(const std::string & text, int & number) const
  {
    int value = 0;
    bool has_digit = false;
    for (char ch : text) {
      if (ch >= '0' && ch <= '9') {
        value = value * 10 + (ch - '0');
        has_digit = true;
        if (value > 9999) break;
      } else if (has_digit) {
        break;   // 取第一段连续数字，形如 "12" / "NO.7" / "7 "
      }
    }
    if (!has_digit) return false;
    if (value < number_min_ || value > number_max_) return false;
    number = value;
    return true;
  }

  bool decodeQr(
    const cv::Mat & frame, int & number, double & confidence, cv::Point2f & center,
    std::vector<cv::Point2f> & box)
  {
    if (code_mode_ == "apriltag") return false;
    return decodeQrWithZbar(frame, number, confidence, center, box);
  }

  bool decodeQrWithZbar(
    const cv::Mat & frame, int & number, double & confidence, cv::Point2f & center,
    std::vector<cv::Point2f> & box)
  {
    if (zbar_scanner_ == nullptr || frame.empty()) return false;
    cv::Mat gray;
    if (frame.channels() == 1) {
      gray = frame;
    } else {
      cv::cvtColor(frame, gray, cv::COLOR_BGR2GRAY);
    }

    zbar_image_t * image = zbar_image_create();
    zbar_image_set_format(image, zbar_fourcc('Y', '8', '0', '0'));
    zbar_image_set_size(image, gray.cols, gray.rows);
    zbar_image_set_data(image, gray.data, gray.total(), nullptr);
    const int symbols_found = zbar_scan_image(zbar_scanner_, image);
    bool decoded = false;
    int valid_symbols = 0;      // 本帧里能解析成 1~24 编号的二维码个数
    double best_distance = 0.0;
    int best_number = 0;
    cv::Point2f best_center(0.0F, 0.0F);
    // 胜出那个二维码的四个角点，仅用于预览画框（识别逻辑不使用）
    std::vector<cv::Point2f> best_box;
    if (symbols_found > 0) {
      // 一帧里可能同时出现上下两排的二维码（行距 40cm、视场也差不多 40cm）。
      // 取"离画面中心最近"的那个，纵向扫描时才能确定现在对准的是哪一排。
      const double frame_cx = static_cast<double>(gray.cols) / 2.0;
      const double frame_cy = static_cast<double>(gray.rows) / 2.0;
      for (const zbar_symbol_t * symbol = zbar_image_first_symbol(image);
        symbol != nullptr; symbol = zbar_symbol_next(symbol))
      {
        if (zbar_symbol_get_type(symbol) != ZBAR_QRCODE) continue;
        int parsed = 0;
        const std::string text = zbar_symbol_get_data(symbol);
        if (!parseNumber(text, parsed)) continue;
        ++valid_symbols;
        const int point_count = zbar_symbol_get_loc_size(symbol);
        cv::Point2f symbol_center(0.0F, 0.0F);
        for (int i = 0; i < point_count; ++i) {
          symbol_center.x += static_cast<float>(zbar_symbol_get_loc_x(symbol, i));
          symbol_center.y += static_cast<float>(zbar_symbol_get_loc_y(symbol, i));
        }
        if (point_count > 0) symbol_center *= 1.0F / static_cast<float>(point_count);
        const double distance = std::hypot(
          static_cast<double>(symbol_center.x) - frame_cx,
          static_cast<double>(symbol_center.y) - frame_cy);
        if (!decoded || distance < best_distance) {
          decoded = true;
          best_distance = distance;
          best_number = parsed;
          best_center = symbol_center;
          best_box.clear();
          for (int i = 0; i < point_count; ++i) {
            best_box.emplace_back(
              static_cast<float>(zbar_symbol_get_loc_x(symbol, i)),
              static_cast<float>(zbar_symbol_get_loc_y(symbol, i)));
          }
        }
      }
    }
    zbar_image_destroy(image);
    last_qr_count_.store(valid_symbols);
    if (decoded) {
      number = best_number;
      confidence = 0.95;
      center = best_center;
      box = best_box;
    }
    return decoded;
  }

  bool decodeAprilTag(
    const cv::Mat & frame, int & number, double & confidence, cv::Point2f & center,
    std::vector<cv::Point2f> & box)
  {
    if (code_mode_ == "qr") return false;
    std::vector<int> ids;
    std::vector<std::vector<cv::Point2f>> corners;
    cv::aruco::detectMarkers(
      frame, apriltag_dictionary_, corners, ids, apriltag_parameters_);
    if (ids.empty()) return false;

    int best_index = -1;
    double best_area = -1.0;
    for (std::size_t i = 0; i < ids.size(); ++i) {
      const int candidate = tag_id_as_number_ ? ids[i] : ids[i] + 1;
      if (candidate < number_min_ || candidate > number_max_) continue;
      if (corners[i].size() != 4) continue;
      const double area = cv::contourArea(corners[i]);
      if (area > best_area) {
        best_area = area;
        best_index = static_cast<int>(i);
      }
    }
    if (best_index < 0) return false;
    number = tag_id_as_number_ ?
      ids[static_cast<std::size_t>(best_index)] :
      ids[static_cast<std::size_t>(best_index)] + 1;
    confidence = 0.95;
    center = cv::Point2f(0.0F, 0.0F);
    for (const auto & corner : corners[static_cast<std::size_t>(best_index)]) center += corner;
    center *= 0.25F;
    box = corners[static_cast<std::size_t>(best_index)];
    return true;
  }

 struct Detection
 {
    bool valid = false;
    int number = 0;
    double confidence = 0.0;
    cv::Point2f center;
    std::vector<cv::Point2f> corners;   // 二维码/标记的四个角点（预览画框用）
    double px_per_cm = 0.0;             // 由实测像素边长 / 物理边长(qr_size_cm)推得
    std::string source;
    int qr_count = 0;        // 这一帧里一共看到几个二维码（>1 说明两行同框了）
 };

  // 由二维码四个角点的实测像素边长和已知物理边长，反推比例尺 px/cm。
  // 取四条边的平均值，抗一点透视变形。算不出来就返回 0（调用方会退化处理）。
  double scaleFromCorners(const std::vector<cv::Point2f> & corners) const
  {
    if (qr_size_cm_ <= 0.0 || corners.size() < 4) return 0.0;
    double total = 0.0;
    for (std::size_t i = 0; i < corners.size(); ++i) {
      total += cv::norm(corners[(i + 1) % corners.size()] - corners[i]);
    }
    const double edge_px = total / static_cast<double>(corners.size());
    if (edge_px < 2.0) return 0.0;
    return edge_px / qr_size_cm_;
  }

  Detection detect(const cv::Mat & frame)
  {
    Detection detection;
    // Contrast-enhanced and raw frames are complementary. Running both on
    // every frame halved the practical detection rate, so alternate variants
    // between consecutive detections while preserving both as fallbacks.
    const bool use_raw = raw_variant_;
    raw_variant_ = !raw_variant_;
    cv::Mat processed;
    const cv::Mat * candidate_ptr = &frame;
    if (!use_raw) {
      processed = preprocess(frame);
      candidate_ptr = &processed;
    }
    const cv::Mat & candidate = *candidate_ptr;
    int number = 0;
    double confidence = 0.0;
    cv::Point2f center;
    std::vector<cv::Point2f> box;

    if (decodeAprilTag(candidate, number, confidence, center, box))
    {
      detection.valid = true;
      detection.number = number;
      detection.confidence = confidence;
      detection.center = center;
      detection.corners = box;
      detection.px_per_cm = scaleFromCorners(box);
      detection.source = "apriltag";
      return detection;
    }
    if (decodeQr(candidate, number, confidence, center, box))
    {
      detection.valid = true;
      detection.number = number;
      detection.confidence = confidence;
      detection.center = center;
      detection.corners = box;
      detection.px_per_cm = scaleFromCorners(box);
      detection.source = use_raw ? "qr.raw" : "qr.enhanced";
      detection.qr_count = last_qr_count_.load();
      return detection;
    }
    return detection;
  }

  void publishDetection(const Detection & detection, const cv::Mat & frame)
  {
    const float error_x = static_cast<float>(frame.cols) / 2.0F - detection.center.x;
    const float error_y = static_cast<float>(frame.rows) / 2.0F - detection.center.y;
    // 用二维码的实测像素边长推出的比例尺，把画面偏差换算成货架面上的厘米偏差。
    // 主题布局（前 5 个字段是原工程约定，不能动，新字段一律往后加）：
    //   [0] 编号  [1] 置信度  [2] x 偏差(px)  [3] y 偏差(px)  [4] 本帧二维码个数
    //   [5] px_per_cm  [6] x 偏差(cm)  [7] y 偏差(cm)
    const double px_per_cm = detection.px_per_cm;
    const double error_x_cm = px_per_cm > 1e-6 ? error_x / px_per_cm : 0.0;
    const double error_y_cm = px_per_cm > 1e-6 ? error_y / px_per_cm : 0.0;
    std_msgs::msg::Float32MultiArray message;
    message.data = {
      static_cast<float>(detection.number),
      static_cast<float>(detection.confidence),
      error_x,
      error_y,
      // 第 5 个值：这一帧一共看到几个二维码。任务节点靠它判断
      // "两行同框"（>1）→ 应该贴近货架缩小视场。
      static_cast<float>(detection.qr_count),
      static_cast<float>(px_per_cm),
      static_cast<float>(error_x_cm),
      static_cast<float>(error_y_cm)};
    code_pub_->publish(message);

    if (publish_fine_data_) {
      // 与原工程 drone_camera_pkg 完全一致的顺序：[y 偏差, x 偏差]
      std_msgs::msg::Int32MultiArray fine;
      fine.data = {
        static_cast<int>(std::lround(error_y)),
        static_cast<int>(std::lround(error_x))};
      fine_data_pub_->publish(fine);
    }
    RCLCPP_DEBUG(
      get_logger(), "识别到货物编号 %d（%s，置信度 %.2f，偏差 %.0f,%.0f px）",
      detection.number, detection.source.c_str(), detection.confidence, error_x, error_y);
  }

  // 把检测结果画到帧上：二维码边框、中心点、到画面中心的偏差连线、底部状态条。
  // 画面中心的十字就是激光同轴基准，显示的 dx/dy 与 /shelf_scan/code 第 3、4 个值一致，
  // 现场调"光斑有没有打在码上"时可以直接照着看。
  void drawOverlay(cv::Mat & frame, const Detection & detection) const
  {
    if (frame.empty()) return;
    const int cx = frame.cols / 2;
    const int cy = frame.rows / 2;

    cv::line(
      frame, cv::Point(cx - 20, cy), cv::Point(cx + 20, cy),
      cv::Scalar(130, 130, 130), 1, cv::LINE_AA);
    cv::line(
      frame, cv::Point(cx, cy - 20), cv::Point(cx, cy + 20),
      cv::Scalar(130, 130, 130), 1, cv::LINE_AA);
    cv::rectangle(
      frame, cv::Rect(cx - 62, cy - 62, 124, 124), cv::Scalar(80, 80, 80), 1, cv::LINE_AA);

    if (detection.valid) {
      if (detection.corners.size() >= 4) {
        std::vector<cv::Point> polygon;
        polygon.reserve(detection.corners.size());
        for (const auto & corner : detection.corners) {
          polygon.emplace_back(
            static_cast<int>(std::lround(corner.x)),
            static_cast<int>(std::lround(corner.y)));
        }
        const std::vector<std::vector<cv::Point>> polygons{polygon};
        cv::polylines(frame, polygons, true, cv::Scalar(0, 255, 0), 2, cv::LINE_AA);
      }

      const cv::Point center(
        static_cast<int>(std::lround(detection.center.x)),
        static_cast<int>(std::lround(detection.center.y)));

      // 以二维码中心为圆心的容差圆：半径 = 5cm × 比例尺。
      // 画面中心（十字准星＝激光光斑）落进这个圈，才算"光斑压在码上"。
      const double px_per_cm = detection.px_per_cm;
      const int radius_px = px_per_cm > 1e-6 ?
        static_cast<int>(std::lround(align_radius_cm_ * px_per_cm)) : 0;
      if (radius_px >= 2) {
        cv::circle(
          frame, center, radius_px, cv::Scalar(0, 215, 255), 1, cv::LINE_AA);
      }

      cv::circle(frame, center, 4, cv::Scalar(0, 0, 255), cv::FILLED, cv::LINE_AA);
      cv::line(frame, cv::Point(cx, cy), center, cv::Scalar(0, 170, 255), 1, cv::LINE_AA);

      // 偏差一律显示厘米——任务节点就是按厘米判"该不该打激光"的
      const double dx_cm = px_per_cm > 1e-6 ?
        (cx - center.x) / px_per_cm : 0.0;
      const double dy_cm = px_per_cm > 1e-6 ?
        (cy - center.y) / px_per_cm : 0.0;
      const double dist_cm = std::hypot(dx_cm, dy_cm);
      const bool inside = px_per_cm > 1e-6 && dist_cm <= align_radius_cm_;

      char label[192];
      std::snprintf(
        label, sizeof(label), "No.%d %s dx=%+.1f dy=%+.1f cm  d=%.1f  %s",
        detection.number, detection.source.c_str(), dx_cm, dy_cm, dist_cm,
        px_per_cm > 1e-6 ? (inside ? "IN" : "OUT") : "NO-SCALE");
      const cv::Point origin(std::max(8, center.x - 130), std::max(32, center.y - radius_px - 10));
      const cv::Scalar label_color = inside ? cv::Scalar(0, 255, 0) : cv::Scalar(0, 215, 255);
      // 先描黑边再写字：货架是浅色的，纯绿字在亮背景上会看不清
      cv::putText(
        frame, label, origin, cv::FONT_HERSHEY_SIMPLEX, 0.52,
        cv::Scalar(0, 0, 0), 4, cv::LINE_AA);
      cv::putText(
        frame, label, origin, cv::FONT_HERSHEY_SIMPLEX, 0.52,
        label_color, 1, cv::LINE_AA);
    }

    const int bar_height = 28;
    if (frame.rows > bar_height) {
      cv::Mat bar = frame(cv::Rect(0, frame.rows - bar_height, frame.cols, bar_height));
      const cv::Mat shade(bar.size(), bar.type(), cv::Scalar(0, 0, 0));
      cv::addWeighted(bar, 0.30, shade, 0.70, 0.0, bar);
    }
    char status[160];
    std::snprintf(
      status, sizeof(status), "mode=%s  qr_count=%d  %s",
      code_mode_.c_str(),
      detection.valid ? detection.qr_count : last_qr_count_.load(),
      detection.valid ? "TRACKING" : "searching");
    cv::putText(
      frame, status, cv::Point(10, frame.rows - 9),
      cv::FONT_HERSHEY_SIMPLEX, 0.55, cv::Scalar(225, 225, 225), 1, cv::LINE_AA);
  }

  // 推送一帧预览。没有浏览器连接时立刻返回，连 JPEG 都不编码。
  // 独立预览线程按 preview_fps_ 调用这里，识别耗时不会拖慢画面。
  void updatePreview(cv::Mat & frame, const Detection & detection)
  {
    if (!preview_enabled_ || frame.empty()) return;
    if (!preview_.hasClients()) return;

    if (preview_draw_) drawOverlay(frame, detection);
    preview_.publish(frame, preview_quality_);
  }

  bool getLatestFrame(cv::Mat & frame, std::uint64_t & sequence)
  {
    std::unique_lock<std::mutex> lock(frame_mutex_);
    const bool ready = frame_ready_cv_.wait_for(
      lock, std::chrono::milliseconds(250),
      [this, sequence]() {
        return !worker_running_.load() || latest_frame_seq_ > sequence;
      });
    if (!ready || latest_frame_seq_ <= sequence || latest_frame_.empty()) {
      return false;
    }
    sequence = latest_frame_seq_;
    frame = latest_frame_.clone();
    return !frame.empty();
  }

  void captureLoop()
  {
    while (worker_running_.load()) {
      cv::Mat frame;
      if (!camera_.read(frame) || frame.empty()) {
        RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 3000, "读取相机帧失败");
        std::this_thread::sleep_for(std::chrono::milliseconds(2));
        continue;
      }
      {
        std::lock_guard<std::mutex> lock(frame_mutex_);
        latest_frame_ = std::move(frame);
        ++latest_frame_seq_;
      }
      ++capture_count_;
      frame_ready_cv_.notify_all();
    }
  }

  void detectLoop()
  {
    std::uint64_t sequence = 0;
    const auto period = std::chrono::duration<double>(
      1.0 / std::max(1.0, detect_fps_));
    while (worker_running_.load()) {
      const auto started = std::chrono::steady_clock::now();
      cv::Mat frame;
      if (getLatestFrame(frame, sequence)) {
        applyOrientation(frame);
        Detection detection;
        {
          std::lock_guard<std::mutex> detect_lock(detect_mutex_);
          detection = detect(frame);
        }
        {
          std::lock_guard<std::mutex> result_lock(detection_mutex_);
          latest_detection_ = detection;
          latest_detection_time_ = std::chrono::steady_clock::now();
        }
        if (detection.valid) publishDetection(detection, frame);
        ++detect_count_;
      }
      std::this_thread::sleep_until(
        started + std::chrono::duration_cast<std::chrono::steady_clock::duration>(period));
    }
  }

  void previewLoop()
  {
    std::uint64_t sequence = 0;
    const auto period = std::chrono::duration<double>(
      1.0 / std::max(1.0, preview_fps_));
    auto next_frame = std::chrono::steady_clock::now();
    auto last_stats = next_frame;
    while (worker_running_.load()) {
      cv::Mat frame;
      if (getLatestFrame(frame, sequence)) {
        applyOrientation(frame);
        Detection detection;
        {
          std::lock_guard<std::mutex> lock(detection_mutex_);
          const bool fresh = latest_detection_time_.time_since_epoch().count() != 0 &&
            std::chrono::steady_clock::now() - latest_detection_time_ <=
            std::chrono::milliseconds(600);
          if (fresh) detection = latest_detection_;
        }
        if (preview_.hasClients()) {
          updatePreview(frame, detection);
          ++preview_count_;
        }
        if (show_preview_) {
          cv::Mat canvas = frame.clone();
          drawOverlay(canvas, detection);
          cv::imshow("shelf_code_reader", canvas);
          cv::waitKey(1);
        }
      }

      const auto now = std::chrono::steady_clock::now();
      if (now - last_stats >= std::chrono::seconds(2)) {
        const double elapsed = std::chrono::duration<double>(now - last_stats).count();
        const auto captures = capture_count_.exchange(0);
        const auto detects = detect_count_.exchange(0);
        const auto previews = preview_count_.exchange(0);
        RCLCPP_INFO(
          get_logger(),
          "CAMERA_RATE capture=%.1f fps detect=%.1f fps preview=%.1f fps",
          captures / elapsed, detects / elapsed, previews / elapsed);
        last_stats = now;
      }

      next_frame += std::chrono::duration_cast<std::chrono::steady_clock::duration>(period);
      const auto after_work = std::chrono::steady_clock::now();
      if (next_frame < after_work) next_frame = after_work + std::chrono::milliseconds(1);
      std::this_thread::sleep_until(next_frame);
    }
  }

  // 要求 2-1：起飞前识别抽取的那张二维码
  int captureTarget(const std::string & reason)
  {
    std::map<int, int> votes;

    if (!target_image_path_.empty()) {
      const cv::Mat image = cv::imread(target_image_path_);
      if (image.empty()) {
        RCLCPP_ERROR(get_logger(), "无法读取图片文件：%s", target_image_path_.c_str());
      } else {
        Detection detection = detect(image);
        if (!detection.valid) detection = detect(image);
        if (detection.valid) votes[detection.number] += 100;
        if (preview_enabled_) {
          // 静帧直接推，不受 preview_fps_ 节流影响
          cv::Mat still = image;
          if (preview_draw_) drawOverlay(still, detection);
          preview_.publish(still, preview_quality_);
        }
        const std::string decoded =
          detection.valid ? std::to_string(detection.number) : std::string("失败");
        RCLCPP_INFO(
          get_logger(), "从图片 %s 识别：%s", target_image_path_.c_str(), decoded.c_str());
      }
    } else {
      const int frames = std::max(1, capture_frames_);
      RCLCPP_INFO(
        get_logger(), "%s：开始连续采集 %d 帧识别抽取的二维码，请保持对准…",
        reason.c_str(), frames);
      std::uint64_t sequence = 0;
      for (int i = 0; i < frames; ++i) {
        cv::Mat frame;
        if (getLatestFrame(frame, sequence)) {
          applyOrientation(frame);      // 摄像头倒装时先转回来
          Detection detection;
          {
            std::lock_guard<std::mutex> detect_lock(detect_mutex_);
            detection = detect(frame);
          }
          if (detection.valid) ++votes[detection.number];
          updatePreview(frame, detection);
          if (show_preview_) {
            cv::Mat canvas = frame.clone();
            drawOverlay(canvas, detection);
            cv::imshow("shelf_code_reader", canvas);
            cv::waitKey(1);
          }
        }
        std::this_thread::sleep_for(
          std::chrono::duration<double>(std::max(0.0, capture_interval_sec_)));
      }
    }

    int best_number = 0;
    int best_votes = 0;
    for (const auto & entry : votes) {
      if (entry.second > best_votes) {
        best_votes = entry.second;
        best_number = entry.first;
      }
    }

    if (best_number < number_min_) {
      RCLCPP_ERROR(get_logger(), "%s：未能识别到有效二维码编号（请对准二维码、补光后重试）", reason.c_str());
      return 0;
    }
    std_msgs::msg::Int32 message;
    message.data = best_number;
    target_id_pub_->publish(message);
    RCLCPP_INFO(
      get_logger(), "%s：识别到抽取的二维码，货物编号 = %d（%d 票），已上报",
      reason.c_str(), best_number, best_votes);
    return best_number;
  }

  cv::VideoCapture camera_;
  zbar_image_scanner_t * zbar_scanner_ = nullptr;
  std::atomic<int> last_qr_count_{0};  // decodeQrWithZbar 最近一帧二维码个数
  bool raw_variant_ = false;
  cv::Ptr<cv::aruco::Dictionary> apriltag_dictionary_;
  cv::Ptr<cv::aruco::DetectorParameters> apriltag_parameters_;

  shelf_scan_pkg::MjpegServer preview_;
  bool preview_enabled_ = false;
  int preview_port_ = 8080;
  double preview_fps_ = 10.0;
  int preview_quality_ = 70;
  bool preview_draw_ = true;

  double qr_size_cm_ = 19.0;        // 二维码物理边长，用于 px <-> cm 换算
  double align_radius_cm_ = 5.0;    // 预览上容差圆的半径（cm）
  bool camera_flip_ud_ = true;      // 摄像头上下倒装
  bool camera_flip_lr_ = true;      // 摄像头左右倒装

  std::atomic<bool> worker_running_{false};
  std::thread capture_thread_;
  std::thread detect_thread_;
  std::thread preview_thread_;
  std::mutex frame_mutex_;
  std::condition_variable frame_ready_cv_;
  cv::Mat latest_frame_;
  std::uint64_t latest_frame_seq_ = 0;
  std::mutex detect_mutex_;
  std::mutex detection_mutex_;
  Detection latest_detection_;
  std::chrono::steady_clock::time_point latest_detection_time_{};
  std::atomic<std::uint64_t> capture_count_{0};
  std::atomic<std::uint64_t> detect_count_{0};
  std::atomic<std::uint64_t> preview_count_{0};

  rclcpp::Publisher<std_msgs::msg::Float32MultiArray>::SharedPtr code_pub_;
  rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr target_id_pub_;
  rclcpp::Publisher<std_msgs::msg::Int32MultiArray>::SharedPtr fine_data_pub_;
  rclcpp::Subscription<std_msgs::msg::Empty>::SharedPtr trigger_sub_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr capture_service_;
  rclcpp::TimerBase::SharedPtr capture_timer_;

  std::string camera_device_;
  std::string code_mode_;
  std::string apriltag_dictionary_name_;
  std::string code_topic_;
  std::string target_id_topic_;
  std::string fine_data_topic_;
  std::string trigger_topic_;
  std::string service_name_;
  std::string target_image_path_;
  int frame_width_ = 640;
  int frame_height_ = 480;
  double fps_ = 120.0;
  double detect_fps_ = 15.0;
  bool show_preview_ = false;
  bool tag_id_as_number_ = true;
  bool publish_fine_data_ = true;
  bool auto_capture_target_ = false;
  double auto_capture_delay_sec_ = 3.0;
  double capture_interval_sec_ = 0.08;
  int capture_frames_ = 12;
  int number_min_ = 1;
  int number_max_ = 24;
};

}  // namespace

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  try {
    auto node = std::make_shared<CodeReaderNode>();
    rclcpp::spin(node);
  } catch (const std::exception & error) {
    fprintf(stderr, "shelf_code_reader_node 启动失败：%s\n", error.what());
    rclcpp::shutdown();
    return 1;
  }
  rclcpp::shutdown();
  return 0;
}
