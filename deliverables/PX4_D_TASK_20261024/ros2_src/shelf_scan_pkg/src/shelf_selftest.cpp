// 机载端逻辑自测（不需要无人机、不需要 ROS 运行、不需要飞控）
//
// 校验内容：
//   1) 24 个点位坐标 / 悬停点 / 朝向 与地面站模型（test/golden.json）完全一致；
//   2) 最近邻 + 2-opt 航线顺序、航程、预计用时与地面站一致；
//   3) 地面站真实下发的 start_route 报文能被正确解析；
//   4) 场地坐标 <-> 地图坐标 标定与换算正确；
//   5) JSON 解析 / 生成正确。
//
// 用法：ros2 run shelf_scan_pkg shelf_selftest
//      或 ./install/shelf_scan_pkg/lib/shelf_scan_pkg/shelf_selftest
#include <cmath>
#include <cstdio>
#include <fstream>
#include <iostream>
#include <algorithm>
#include <iterator>
#include <string>
#include <vector>

#include <ament_index_cpp/get_package_share_directory.hpp>

#include "shelf_scan_pkg/field_geometry.hpp"
#include "shelf_scan_pkg/gs_protocol.hpp"
#include "shelf_scan_pkg/json_lite.hpp"
#include "shelf_scan_pkg/route_planner.hpp"

namespace
{

int g_failures = 0;
int g_checks = 0;

void check(bool condition, const std::string & description)
{
  ++g_checks;
  if (condition) {
    std::cout << "  [通过] " << description << "\n";
  } else {
    ++g_failures;
    std::cout << "  [失败] " << description << "\n";
  }
}

bool nearlyEqual(double a, double b, double tolerance)
{
  return std::fabs(a - b) <= tolerance;
}

double angularDifference(double a, double b)
{
  double value = std::fmod(a - b + 180.0, 360.0);
  if (value < 0.0) value += 360.0;
  return std::fabs(value - 180.0);
}

std::string readFile(const std::string & path, bool & ok)
{
  std::ifstream input(path);
  if (!input.is_open()) {
    ok = false;
    return std::string();
  }
  ok = true;
  return std::string(
    (std::istreambuf_iterator<char>(input)), std::istreambuf_iterator<char>());
}

}  // namespace

int main(int argc, char ** argv)
{
  std::string share_directory;
  try {
    share_directory = ament_index_cpp::get_package_share_directory("shelf_scan_pkg");
  } catch (const std::exception & error) {
    std::cerr << "无法定位 shelf_scan_pkg 的安装目录：" << error.what() << "\n";
    return 2;
  }
  const std::string field_path = argc > 1 ? argv[1] : share_directory + "/config/field.json";
  const std::string golden_path = argc > 2 ? argv[2] : share_directory + "/test/golden.json";

  std::cout << "机载端逻辑自测\n";
  std::cout << "  场地配置：" << field_path << "\n";
  std::cout << "  黄金数据：" << golden_path << "\n";

  shelf_scan::FieldGeometry field;
  std::string error;
  if (!field.loadFile(field_path, error)) {
    std::cerr << "加载场地配置失败：" << error << "\n";
    return 2;
  }
  bool file_ok = false;
  const std::string golden_text = readFile(golden_path, file_ok);
  if (!file_ok) {
    std::cerr << "读取黄金数据失败：" << golden_path << "\n";
    return 2;
  }
  shelf_scan::json::Value golden;
  if (!shelf_scan::json::parse(golden_text, golden)) {
    std::cerr << "黄金数据不是合法 JSON\n";
    return 2;
  }

  // ---------------------------------------------------------------- 1 点位
  std::cout << "\n[1] 24 个盘点点位几何\n";
  const shelf_scan::json::Value & golden_slots = golden["slots"];
  check(
    golden_slots.is_array() && golden_slots.size() == field.slots().size(),
    "点位数一致（" + std::to_string(field.slots().size()) + "）");
  double worst_position = 0.0;
  double worst_yaw = 0.0;
  for (std::size_t i = 0; i < golden_slots.size(); ++i) {
    const shelf_scan::json::Value & expected = golden_slots[i];
    const std::string id = expected["id"].as_string();
    const shelf_scan::SlotInfo * info = field.slot(id);
    if (info == nullptr) {
      check(false, "缺少点位 " + id);
      continue;
    }
    worst_position = std::max(
      worst_position,
      std::max(
        std::fabs(info->qr.x - expected["x"].as_double()),
        std::max(
          std::fabs(info->qr.y - expected["y"].as_double()),
          std::max(
            std::fabs(info->qr.z - expected["z"].as_double()),
            std::max(
              std::fabs(info->hover.x - expected["hover_x"].as_double()),
              std::fabs(info->hover.y - expected["hover_y"].as_double()))))));
    worst_yaw = std::max(
      worst_yaw, angularDifference(info->yaw, expected["yaw"].as_double()));
  }
  check(
    worst_position < 1e-6,
    "全部点位坐标与地面站一致（最大偏差 " + std::to_string(worst_position) + "cm）");
  check(
    worst_yaw < 1e-6,
    "全部点位朝向与地面站一致（最大偏差 " + std::to_string(worst_yaw) + "deg）");

  // ---------------------------------------------------------------- 2 航线
  std::cout << "\n[2] 遍历盘点航线（两排弓字形）\n";
  shelf_scan::RoutePlanner planner(field);
  const shelf_scan::Route sweep = planner.planSweep();

  // 机载端生成的点位访问顺序
  std::vector<std::string> order;
  for (const auto & leg : sweep.legs) {
    if (leg.type == "scan") order.push_back(leg.slot);
  }
  const shelf_scan::json::Value & golden_order = golden["sweep_order"];
  bool order_matches = golden_order.is_array() && golden_order.size() == order.size();
  if (order_matches) {
    std::string produced;
    for (std::size_t i = 0; i < order.size(); ++i) {
      produced += order[i] + " ";
      if (order[i] != golden_order[i].as_string()) order_matches = false;
    }
    if (!order_matches) std::cout << "      机载顺序：" << produced << "\n";
  }
  check(order_matches, "盘点顺序与地面站一致");

  const double expected_length = golden["sweep_length_cm"].as_double();
  const double expected_eta = golden["sweep_eta_s"].as_double();
  check(
    nearlyEqual(sweep.length_cm, expected_length, 0.5),
    "航程一致（机载 " + std::to_string(sweep.length_cm) + "cm / 地面站 " +
    std::to_string(expected_length) + "cm）");
  check(
    nearlyEqual(sweep.eta_s, expected_eta, 0.3),
    "预计用时一致（机载 " + std::to_string(sweep.eta_s) + "s / 地面站 " +
    std::to_string(expected_eta) + "s）");
  std::size_t scan_legs = 0;
  for (const auto & leg : sweep.legs) {
    if (leg.type == "scan") ++scan_legs;
  }
  check(scan_legs == field.slots().size(), "扫描航段数等于点位数");

  // ------------------------------------------------------ 3 地面站报文解析
  std::cout << "\n[3] 地面站 start_route 报文解析\n";
  const std::string route_text = golden["sample_route"].dump();
  shelf_scan::Route parsed;
  const bool route_parsed = shelf_scan::parseRouteJson(route_text, parsed, error);
  check(route_parsed, "能解析完整航线报文" + (route_parsed ? std::string() : ("（" + error + "）")));
  if (route_parsed) {
    check(parsed.legs.size() == sweep.legs.size(), "航段数与本地规划一致");
    std::size_t mismatch = 0;
    double worst_hover = 0.0;
    for (std::size_t i = 0; i < std::min(parsed.legs.size(), sweep.legs.size()); ++i) {
      if (parsed.legs[i].type != sweep.legs[i].type) ++mismatch;
      if (parsed.legs[i].type == "scan") {
        worst_hover = std::max(
          worst_hover,
          std::hypot(
            parsed.legs[i].hover.x - sweep.legs[i].hover.x,
            parsed.legs[i].hover.y - sweep.legs[i].hover.y));
      } else if (parsed.legs[i].type == "vscan") {
        // 纵向扫描段：上下两排编号 + 列坐标 + 起止高度都要和本地一致
        if (parsed.legs[i].slots.size() != sweep.legs[i].slots.size()) {
          worst_hover = 1e9;
        } else {
          for (std::size_t k = 0; k < parsed.legs[i].slots.size(); ++k) {
            if (parsed.legs[i].slots[k] != sweep.legs[i].slots[k]) worst_hover = 1e9;
          }
        }
        if (!parsed.legs[i].points.empty() && !sweep.legs[i].points.empty()) {
          worst_hover = std::max(
            worst_hover,
            std::hypot(
              parsed.legs[i].points.front().x - sweep.legs[i].points.front().x,
              parsed.legs[i].points.front().y - sweep.legs[i].points.front().y));
          worst_hover = std::max(
            worst_hover,
            std::fabs(parsed.legs[i].points.back().z - sweep.legs[i].points.back().z));
        }
      }
    }
    check(mismatch == 0, "航段类型序列一致");
    check(worst_hover < 0.01, "扫描段与本地几何一致（最大偏差 " + std::to_string(worst_hover) + "cm）");

    const std::string command_text =
      "{\"type\":\"cmd\",\"cmd\":\"start_route\",\"mode\":\"sweep\",\"route\":" + route_text + "}";
    shelf_scan::json::Value command_value;
    shelf_scan::json::parse(command_text, command_value);
    const shelf_scan::InboundCommand command = shelf_scan::parseCommand(command_value);
    check(
      command.kind == shelf_scan::InboundCommand::Kind::StartRoute && command.has_route,
      "能解析 type=cmd / cmd=start_route 报文");
    const std::string target_command_text =
      "{\"type\":\"cmd\",\"cmd\":\"start_route\",\"mode\":\"target\",\"target_no\":13,"
      "\"route\":" + route_text + "}";
    shelf_scan::json::Value target_value;
    shelf_scan::json::parse(target_command_text, target_value);
    const shelf_scan::InboundCommand target_command = shelf_scan::parseCommand(target_value);
    check(
      target_command.kind == shelf_scan::InboundCommand::Kind::StartRoute &&
      target_command.target_no == 13 && target_command.mode == "target",
      "能解析定向盘点指令（target_no）");
  }

  // ------------------------------------------------------------ 4 坐标变换
  std::cout << "\n[4] 坐标标定与换算\n";
  const shelf_scan::FrameTransform transform = shelf_scan::FrameTransform::fromCalibration(
    0.0, 0.0, 0.0, field.takeoff.x, field.takeoff.y, field.takeoff_yaw);
  const shelf_scan::SlotInfo * slot_a1 = field.slot("A1");
  check(slot_a1 != nullptr, "存在点位 A1");
  if (slot_a1 != nullptr) {
    const shelf_scan::Vec2 map_point = transform.fieldToMap(slot_a1->hover.x, slot_a1->hover.y);
    // 手工按 yaw_offset 旋转一次做独立校验（起飞朝向现在是 0 = 正对货架）
    const double rel_x = slot_a1->hover.x - field.takeoff.x;
    const double rel_y = slot_a1->hover.y - field.takeoff.y;
    const double angle = -field.takeoff_yaw * M_PI / 180.0;
    const double expect_x = std::cos(angle) * rel_x - std::sin(angle) * rel_y;
    const double expect_y = std::sin(angle) * rel_x + std::cos(angle) * rel_y;
    check(
      nearlyEqual(map_point.x, expect_x, 0.01) && nearlyEqual(map_point.y, expect_y, 0.01),
      "A1 悬停点换算到地图坐标 (" + std::to_string(expect_x) + "," +
      std::to_string(expect_y) + ")cm（实际 " +
      std::to_string(map_point.x) + "," + std::to_string(map_point.y) + "）");
  }
  const shelf_scan::Vec2 round_trip = transform.mapToField(
    transform.fieldToMap(123.4, 234.5).x, transform.fieldToMap(123.4, 234.5).y);
  check(
    nearlyEqual(round_trip.x, 123.4, 1e-6) && nearlyEqual(round_trip.y, 234.5, 1e-6),
    "正反变换可逆");
  check(
    nearlyEqual(transform.fieldYawToMap(field.takeoff_yaw), 0.0, 1e-9) &&
    nearlyEqual(transform.mapYawToField(0.0), field.takeoff_yaw, 1e-9),
    "机头朝向换算正确（场地起飞朝向 <-> 地图 0deg）");
  const shelf_scan::FrameTransform shifted = shelf_scan::FrameTransform::fromCalibration(
    12.0, -30.0, 45.0, field.takeoff.x, field.takeoff.y, field.takeoff_yaw);
  const shelf_scan::Vec2 home = shifted.fieldToMap(field.takeoff.x, field.takeoff.y);
  check(
    nearlyEqual(home.x, 12.0, 1e-9) && nearlyEqual(home.y, -30.0, 1e-9),
    "任意起飞位姿下起飞点都能映射到当前地图位置");

  // ------------------------------------------------------------------ 5 JSON
  std::cout << "\n[5] JSON 解析与生成\n";
  const std::string telemetry =
    "{\"type\":\"telemetry\",\"t\":12.4,\"pos\":{\"x\":128.5,\"y\":187.5,\"z\":150.0},"
    "\"yaw\":90.0,\"batt\":76.5,\"armed\":true,\"laser\":false,\"state\":\"flying\","
    "\"slot\":\"A3\"}";
  shelf_scan::json::Value telemetry_value;
  const bool telemetry_ok = shelf_scan::json::parse(telemetry, telemetry_value);
  check(
    telemetry_ok && telemetry_value["pos"]["x"].as_double() == 128.5 &&
    telemetry_value["slot"].as_string() == "A3",
    "解析地面站示例遥测报文");
  const std::string unicode_text = "{\"title\":\"遍历盘点航线\"}";
  shelf_scan::json::Value unicode_value;
  shelf_scan::json::parse(unicode_text, unicode_value);
  check(unicode_value["title"].as_string() == "遍历盘点航线", "支持中文（UTF-8）字符串");
  shelf_scan::json::Value escaped;
  shelf_scan::json::parse("{\"title\":\"\\u4e2d\\u6587\"}", escaped);
  check(escaped["title"].as_string() == "中文", "支持 \\uXXXX 转义");

  shelf_scan::TelemetryFields fields;
  fields.t = 12.34;
  fields.position_field = shelf_scan::Vec3{128.5, 187.5, 150.0};
  fields.yaw_field = 90.0;
  fields.armed = true;
  fields.laser = false;
  fields.state = "flying";
  fields.slot = "A3";
  shelf_scan::json::Value built;
  shelf_scan::json::parse(shelf_scan::buildTelemetry(fields), built);
  check(
    built["pos"]["x"].as_double() == 128.5 && built["slot"].as_string() == "A3" &&
    built["type"].as_string() == "telemetry",
    "生成遥测报文可被再次解析");
  shelf_scan::json::Value detect;
  shelf_scan::json::parse(
    shelf_scan::buildDetect(7, "B3", true, 0.97, shelf_scan::Vec3{152.5, 250.0, 140.0}, "", 33.2),
    detect);
  check(
    detect["no"].as_int() == 7 && detect["slot"].as_string() == "B3" &&
    detect["ok"].as_bool(),
    "生成盘点结果报文格式正确");

  std::cout << "\n汇总：" << (g_checks - g_failures) << "/" << g_checks << " 项通过\n";
  if (g_failures == 0) {
    std::cout << "全部通过：机载端几何、规划与协议解析与地面站一致。\n";
    return 0;
  }
  std::cout << "存在 " << g_failures << " 项失败，请检查场地配置与版本一致性。\n";
  return 1;
}
