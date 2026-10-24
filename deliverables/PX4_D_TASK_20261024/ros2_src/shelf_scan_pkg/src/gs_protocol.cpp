#include "shelf_scan_pkg/gs_protocol.hpp"

#include <algorithm>
#include <cmath>

namespace shelf_scan
{
namespace
{

constexpr double kPi = 3.14159265358979323846;

double degToRad(double degrees)
{
  return degrees * kPi / 180.0;
}

double normalizeDeg(double angle)
{
  double value = std::fmod(angle + 180.0, 360.0);
  if (value < 0.0) value += 360.0;
  return value - 180.0;
}

double round1(double value)
{
  return std::round(value * 10.0) / 10.0;
}

bool readPoint3(const json::Value & value, Vec3 & out)
{
  if (!value.is_array() || value.size() < 2) return false;
  out.x = value[0].as_double();
  out.y = value[1].as_double();
  out.z = value.size() >= 3 ? value[2].as_double(0.0) : 0.0;
  return true;
}

json::Value point3ToJson(const Vec3 & point)
{
  json::Value value = json::Value::array();
  value.push_back(json::Value(point.x));
  value.push_back(json::Value(point.y));
  value.push_back(json::Value(point.z));
  return value;
}

}  // namespace

// --------------------------------------------------------------------------
// FrameTransform
// --------------------------------------------------------------------------
Vec2 FrameTransform::fieldToMap(double field_x, double field_y) const
{
  const double dx = field_x - translation_x;
  const double dy = field_y - translation_y;
  const double angle = -degToRad(yaw_offset_deg);
  const double cosine = std::cos(angle);
  const double sine = std::sin(angle);
  return Vec2{cosine * dx - sine * dy, sine * dx + cosine * dy};
}

Vec3 FrameTransform::fieldToMap(double field_x, double field_y, double field_z) const
{
  const Vec2 point = fieldToMap(field_x, field_y);
  return Vec3{point.x, point.y, field_z};
}

Vec2 FrameTransform::mapToField(double map_x, double map_y) const
{
  const double angle = degToRad(yaw_offset_deg);
  const double cosine = std::cos(angle);
  const double sine = std::sin(angle);
  return Vec2{
    cosine * map_x - sine * map_y + translation_x,
    sine * map_x + cosine * map_y + translation_y};
}

double FrameTransform::fieldYawToMap(double field_yaw_deg) const
{
  return normalizeDeg(field_yaw_deg - yaw_offset_deg);
}

double FrameTransform::mapYawToField(double map_yaw_deg) const
{
  return normalizeDeg(map_yaw_deg + yaw_offset_deg);
}

FrameTransform FrameTransform::fromCalibration(
  double map_x, double map_y, double map_yaw_deg,
  double field_x, double field_y, double field_yaw_deg)
{
  FrameTransform transform;
  transform.calibrated = true;
  transform.yaw_offset_deg = normalizeDeg(field_yaw_deg - map_yaw_deg);
  const double angle = degToRad(transform.yaw_offset_deg);
  const double cosine = std::cos(angle);
  const double sine = std::sin(angle);
  transform.translation_x = field_x - (cosine * map_x - sine * map_y);
  transform.translation_y = field_y - (sine * map_x + cosine * map_y);
  return transform;
}

FrameTransform FrameTransform::fromOffsets(
  double yaw_offset_deg, double translation_x, double translation_y)
{
  FrameTransform transform;
  transform.calibrated = true;
  transform.yaw_offset_deg = normalizeDeg(yaw_offset_deg);
  transform.translation_x = translation_x;
  transform.translation_y = translation_y;
  return transform;
}

// --------------------------------------------------------------------------
// 报文构造
// --------------------------------------------------------------------------
std::string buildHello(const std::string & device, const std::string & firmware)
{
  json::Value message = json::Value::object();
  message.set("type", json::Value("hello"));
  message.set("device", json::Value(device));
  message.set("fw", json::Value(firmware));
  return message.dump();
}

std::string buildTelemetry(const TelemetryFields & fields)
{
  json::Value message = json::Value::object();
  message.set("type", json::Value("telemetry"));
  message.set("t", json::Value(round1(fields.t)));
  json::Value position = json::Value::object();
  position.set("x", json::Value(round1(fields.position_field.x)));
  position.set("y", json::Value(round1(fields.position_field.y)));
  position.set("z", json::Value(round1(fields.position_field.z)));
  message.set("pos", position);
  message.set("yaw", json::Value(round1(fields.yaw_field)));
  // 没有 /height 时 pos.z 报的是 0（地面），地面站靠这个标志区分
  // “真的在地面” 和 “飞控没数据”，避免把 0cm 误当成真实高度
  message.set("has_height", json::Value(fields.has_height));
  if (fields.has_battery) {
    message.set("batt", json::Value(round1(fields.battery)));
  }
  message.set("armed", json::Value(fields.armed));
  message.set("laser", json::Value(fields.laser));
  message.set("state", json::Value(fields.state));
  if (!fields.slot.empty()) {
    message.set("slot", json::Value(fields.slot));
  }
  return message.dump();
}

std::string buildDetect(
  int number, const std::string & slot, bool ok, double confidence,
  const Vec3 & qr_field, const std::string & reason, double elapsed_s)
{
  json::Value message = json::Value::object();
  message.set("type", json::Value("detect"));
  if (ok) {
    message.set("no", json::Value(number));
  } else {
    message.set("no", json::Value());
  }
  message.set("slot", json::Value(slot));
  message.set("ok", json::Value(ok));
  if (ok) {
    message.set("x", json::Value(round1(qr_field.x)));
    message.set("y", json::Value(round1(qr_field.y)));
    message.set("z", json::Value(round1(qr_field.z)));
    message.set("conf", json::Value(round1(confidence * 1000.0) / 1000.0));
  } else if (!reason.empty()) {
    message.set("reason", json::Value(reason));
  }
  message.set("t", json::Value(round1(elapsed_s)));
  return message.dump();
}

std::string buildTargetId(int number, double confidence)
{
  json::Value message = json::Value::object();
  message.set("type", json::Value("target_id"));
  message.set("no", json::Value(number));
  if (confidence > 0.0) {
    message.set("conf", json::Value(round1(confidence * 1000.0) / 1000.0));
  }
  return message.dump();
}

std::string buildTargetHit(int number, const std::string & slot)
{
  json::Value message = json::Value::object();
  message.set("type", json::Value("target_hit"));
  message.set("no", json::Value(number));
  message.set("slot", json::Value(slot));
  return message.dump();
}

std::string buildMission(const std::string & phase, const std::string & note)
{
  json::Value message = json::Value::object();
  message.set("type", json::Value("mission"));
  message.set("phase", json::Value(phase));
  if (!note.empty()) message.set("note", json::Value(note));
  return message.dump();
}

std::string buildAck(const std::string & command, bool ok, const std::string & detail)
{
  json::Value message = json::Value::object();
  message.set("type", json::Value("ack"));
  message.set("cmd", json::Value(command));
  message.set("ok", json::Value(ok));
  if (!detail.empty()) message.set("detail", json::Value(detail));
  return message.dump();
}

// --------------------------------------------------------------------------
// 航线解析 / 生成
// --------------------------------------------------------------------------
bool parseRoute(const json::Value & route_value, Route & out, std::string & error)
{
  if (!route_value.is_object()) {
    error = "route 必须是对象";
    return false;
  }
  const json::Value & legs = route_value["legs"];
  if (!legs.is_array() || legs.size() == 0) {
    error = "route.legs 不能为空";
    return false;
  }

  out = Route();
  out.kind = route_value["kind"].as_string("sweep");
  out.title = route_value["title"].as_string();
  out.mode = route_value["mode"].as_string();
  out.target_no = static_cast<int>(route_value["target_no"].as_int(0));
  out.target_slot = route_value["target_slot"].as_string();
  out.length_cm = route_value["length_cm"].as_double(0.0);
  out.eta_s = route_value["eta_s"].as_double(0.0);

  for (std::size_t i = 0; i < legs.size(); ++i) {
    const json::Value & leg_value = legs[i];
    RouteLeg leg;
    leg.type = leg_value["type"].as_string();
    if (leg.type.empty()) {
      error = "route.legs[" + std::to_string(i) + "].type 为空";
      return false;
    }
    leg.slot = leg_value["slot"].as_string();
    leg.order = static_cast<int>(leg_value["order"].as_int(0));
    leg.label = leg_value["label"].as_string();
    leg.laser_s = leg_value["laser_s"].as_double(0.5);
    // vscan（纵向扫描）段：一列 = 上下两排
    leg.face = leg_value["face"].as_string();
    leg.column = static_cast<int>(leg_value["column"].as_int(0));
    const json::Value & slots_value = leg_value["slots"];
    if (slots_value.is_array()) {
      for (std::size_t k = 0; k < slots_value.size(); ++k) {
        leg.slots.push_back(slots_value[k].as_string());
      }
    }
    const json::Value & rows_value = leg_value["rows"];
    if (rows_value.is_array()) {
      for (std::size_t k = 0; k < rows_value.size(); ++k) {
        leg.rows.push_back(rows_value[k].as_double());
      }
    }
    if (leg_value.has("yaw") && leg_value["yaw"].is_number()) {
      leg.yaw = leg_value["yaw"].as_double();
      leg.has_yaw = true;
    }
    const json::Value & points = leg_value["points"];
    if (points.is_array()) {
      for (std::size_t k = 0; k < points.size(); ++k) {
        Vec3 point;
        if (!readPoint3(points[k], point)) {
          error = "route.legs[" + std::to_string(i) + "].points 元素格式错误";
          return false;
        }
        leg.points.push_back(point);
      }
    }
    if (leg_value.has("hover")) {
      readPoint3(leg_value["hover"], leg.hover);
      if (leg.points.empty()) leg.points.push_back(leg.hover);
    }
    if (leg_value.has("qr")) {
      const json::Value & qr = leg_value["qr"];
      if (qr.is_array() && qr.size() >= 2) {
        leg.qr = Vec2{qr[0].as_double(), qr[1].as_double()};
      }
    }
    if (leg.type == "scan" && leg.points.empty()) {
      error = "scan 航段缺少 hover / points";
      return false;
    }
    out.legs.push_back(leg);
  }
  return true;
}

bool parseRouteJson(const std::string & text, Route & out, std::string & error)
{
  json::Value value;
  if (!json::parse(text, value)) {
    error = "route 不是合法 JSON";
    return false;
  }
  return parseRoute(value, out, error);
}

std::string routeToJson(const Route & route)
{
  json::Value value = json::Value::object();
  value.set("kind", json::Value(route.kind));
  value.set("title", json::Value(route.title));
  value.set("length_cm", json::Value(route.length_cm));
  value.set("eta_s", json::Value(route.eta_s));
  json::Value legs = json::Value::array();
  for (const RouteLeg & leg : route.legs) {
    json::Value leg_value = json::Value::object();
    leg_value.set("type", json::Value(leg.type));
    if (!leg.slot.empty()) leg_value.set("slot", json::Value(leg.slot));
    if (leg.order > 0) leg_value.set("order", json::Value(leg.order));
    if (leg.has_yaw) leg_value.set("yaw", json::Value(leg.yaw));
    if (leg.type == "scan") leg_value.set("laser_s", json::Value(leg.laser_s));
    json::Value points = json::Value::array();
    for (const Vec3 & point : leg.points) points.push_back(point3ToJson(point));
    leg_value.set("points", points);
    legs.push_back(leg_value);
  }
  value.set("legs", legs);
  return value.dump();
}

// --------------------------------------------------------------------------
// 命令解析
// --------------------------------------------------------------------------
InboundCommand parseCommand(const json::Value & message)
{
  InboundCommand command;
  if (!message.is_object()) return command;
  const std::string type = message["type"].as_string();
  if (type == "hello" || type == "hello_gs") {
    command.kind = InboundCommand::Kind::Ping;
    return command;
  }
  if (type != "cmd") return command;

  command.raw_command = message["cmd"].as_string();
  const std::string & name = command.raw_command;
  if (name == "start_route") {
    command.kind = InboundCommand::Kind::StartRoute;
    command.mode = message["mode"].as_string("sweep");
    command.target_no = static_cast<int>(message["target_no"].as_int(0));
    if (message.has("route")) {
      std::string error;
      if (parseRoute(message["route"], command.route, error)) {
        command.has_route = true;
      }
    }
  } else if (name == "land") {
    command.kind = InboundCommand::Kind::Land;
  } else if (name == "estop" || name == "stop") {
    command.kind = InboundCommand::Kind::Estop;
  } else if (name == "calibrate" || name == "set_origin") {
    command.kind = InboundCommand::Kind::Calibrate;
  } else if (name == "goto" || name == "scan_slot") {
    command.kind = InboundCommand::Kind::GotoSlot;
    command.slot = message["slot"].as_string();
  } else if (name == "set_target" || name == "target_id") {
    command.kind = InboundCommand::Kind::SetTarget;
    command.target_no = static_cast<int>(message["no"].as_int(0));
  }
  return command;
}

}  // namespace shelf_scan
