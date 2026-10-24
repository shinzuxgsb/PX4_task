// 机载端 <-> 地面站 的消息协议（与 groundstation/PROTOCOL.md 一致）
//
// 上行（飞机 -> 地面站）：hello / telemetry / detect / target_id / target_hit / mission
// 下行（地面站 -> 飞机）：{"type":"cmd","cmd":"start_route|land|estop|...","route":{...}}
#pragma once

#include <map>
#include <string>

#include "shelf_scan_pkg/field_geometry.hpp"
#include "shelf_scan_pkg/json_lite.hpp"
#include "shelf_scan_pkg/route_planner.hpp"

namespace shelf_scan
{

// ---------------------------------------------------------------------------
// 场地坐标系 <-> 无人机地图坐标系
//
// 地面站使用“场地坐标”：原点在作业区左下角，X 向右、Y 向上、单位 cm。
// 无人机使用 cartographer 的 map 坐标系：原点是起飞瞬间的位置，朝向由建图起点决定。
// 两者之间是一个平面旋转 + 平移：
//     field = R(yaw_offset) * map + t
// 起飞前无人机静止在起飞点，因此可以用“起飞点”一次标定出 yaw_offset 与 t。
// ---------------------------------------------------------------------------
struct FrameTransform
{
  bool calibrated = false;
  double yaw_offset_deg = 0.0;   // 地图坐标 -> 场地坐标 的旋转角（度）
  double translation_x = 0.0;    // 场地坐标下的平移量（cm）
  double translation_y = 0.0;

  Vec2 fieldToMap(double field_x, double field_y) const;
  Vec3 fieldToMap(double field_x, double field_y, double field_z) const;
  Vec2 mapToField(double map_x, double map_y) const;
  double fieldYawToMap(double field_yaw_deg) const;
  double mapYawToField(double map_yaw_deg) const;

  // 用一次实测标定：无人机此刻在地图坐标系下的位姿 (map_x, map_y, map_yaw)
  // 等于场地坐标下的 (field_x, field_y, field_yaw)（默认起飞点 75,75,90）
  static FrameTransform fromCalibration(
    double map_x, double map_y, double map_yaw_deg,
    double field_x, double field_y, double field_yaw_deg);

  // 直接给定偏移量（手工标定）
  static FrameTransform fromOffsets(
    double yaw_offset_deg, double translation_x, double translation_y);
};

// ---------------------------------------------------------------------------
// 报文构造
// ---------------------------------------------------------------------------
struct TelemetryFields
{
  double t = 0.0;             // 任务用时（s，从收到 start_route 起算）
  Vec3 position_field;        // 场地坐标（cm）
  double yaw_field = 0.0;     // 场地坐标下的机头朝向（度）
  bool has_height = false;    // /height 是否有真实数据（false 时 z 不可信）
  bool has_battery = false;
  double battery = 0.0;
  bool armed = false;
  bool laser = false;
  std::string state = "idle";
  std::string slot;
};

std::string buildHello(const std::string & device, const std::string & firmware);
std::string buildTelemetry(const TelemetryFields & fields);
std::string buildDetect(
  int number, const std::string & slot, bool ok, double confidence,
  const Vec3 & qr_field, const std::string & reason, double elapsed_s);
std::string buildTargetId(int number, double confidence);
std::string buildTargetHit(int number, const std::string & slot);
std::string buildMission(const std::string & phase, const std::string & note);
std::string buildAck(const std::string & command, bool ok, const std::string & detail);

// ---------------------------------------------------------------------------
// 航线解析
// ---------------------------------------------------------------------------
bool parseRoute(const json::Value & route_value, Route & out, std::string & error);
bool parseRouteJson(const std::string & text, Route & out, std::string & error);
std::string routeToJson(const Route & route);

// 命令解析结果 --------------------------------------------------------------
struct InboundCommand
{
  enum class Kind { Unknown, StartRoute, Land, Estop, Calibrate, GotoSlot, SetTarget, Ping };
  Kind kind = Kind::Unknown;
  std::string raw_command;
  std::string mode;             // sweep / target
  int target_no = 0;
  std::string slot;             // goto 命令的目标点位
  Route route;                  // 携带的航线（可为空）
  bool has_route = false;
};

InboundCommand parseCommand(const json::Value & message);

}  // namespace shelf_scan
