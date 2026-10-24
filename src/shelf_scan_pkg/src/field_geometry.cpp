#include "shelf_scan_pkg/field_geometry.hpp"

#include <algorithm>
#include <cctype>
#include <cmath>
#include <cstdio>
#include <fstream>
#include <functional>
#include <sstream>

#include "shelf_scan_pkg/json_lite.hpp"

namespace shelf_scan
{
namespace
{

constexpr double kPi = 3.14159265358979323846;

Vec2 normalized(double x, double y)
{
  const double length = std::hypot(x, y);
  if (length <= 1e-9) return Vec2{1.0, 0.0};
  return Vec2{x / length, y / length};
}

double yawOf(const Vec2 & direction)
{
  return std::atan2(direction.y, direction.x) * 180.0 / kPi;
}

double readNumber(const json::Value & object, const char * key, double fallback)
{
  const json::Value & value = object[key];
  if (value.is_number()) return value.as_double(fallback);
  if (value.is_string()) return value.as_double(fallback);
  return fallback;
}

bool readNumberRequired(
  const json::Value & object, const char * key, double & out, std::string & error,
  const char * context)
{
  const json::Value & value = object[key];
  if (!value.is_number()) {
    error = std::string(context) + " 缺少数值字段 " + key;
    return false;
  }
  out = value.as_double();
  return true;
}

}  // namespace

bool FieldGeometry::loadFile(const std::string & path, std::string & error)
{
  std::ifstream input(path);
  if (!input.is_open()) {
    error = "无法打开场地配置文件：" + path;
    return false;
  }
  std::ostringstream buffer;
  buffer << input.rdbuf();
  return loadText(buffer.str(), error);
}

bool FieldGeometry::loadText(const std::string & text, std::string & error)
{
  json::Value root;
  if (!json::parse(text, root)) {
    error = "场地配置不是合法 JSON";
    return false;
  }
  if (!root.is_object()) {
    error = "场地配置根节点必须是对象";
    return false;
  }

  // ---- 作业区 ----
  const json::Value & area = root["area"];
  width = readNumber(area, "w", 500.0);
  height = readNumber(area, "h", 400.0);
  wall_margin = readNumber(area, "wall_margin", 5.0);

  name = root["name"].as_string("shelf field");
  safety_margin = readNumber(root, "safety_margin_cm", 30.0);
  standoff = readNumber(root, "standoff_cm", 55.0);
  // 现场实测的悬停点覆盖：{"A3": [100.0, 157.0], ...}
  slot_hover_xy.clear();
  slot_hover_z.clear();
  slot_fly_overshoot.clear();
  for (const auto & entry : root["slot_hover_xy_cm"].members()) {
    const json::Value & pair = entry.second;
    if (pair.size() < 2) continue;
    slot_hover_xy[entry.first] = Vec2{pair[0].as_double(0.0), pair[1].as_double(0.0)};
    // 可选的第 3 项 = 该点位的高度覆盖（写 [x, y, z] 时生效）
    if (pair.size() >= 3) slot_hover_z[entry.first] = pair[2].as_double(0.0);
  }
  // 飞行过冲（只影响飞机往哪飞）：{"A1": [0.0, 10.0, 0.0], ...}
  for (const auto & entry : root["slot_fly_overshoot_cm"].members()) {
    const json::Value & v = entry.second;
    if (v.size() < 3) continue;
    slot_fly_overshoot[entry.first] =
      Vec3{v[0].as_double(0.0), v[1].as_double(0.0), v[2].as_double(0.0)};
  }
  cruise_alt = readNumber(root, "cruise_alt_cm", 150.0);
  // 换面缓冲点 + 纵向扫描范围
  const json::Value & buffer_value = root["buffer"];
  buffer.x = readNumber(buffer_value, "x", 75.0);
  buffer.y = readNumber(buffer_value, "y", 350.0);
  buffer_yaw = readNumber(buffer_value, "yaw", 90.0);
  vscan_top = readNumber(root, "vscan_top_cm", cruise_alt);
  vscan_floor = readNumber(root, "vscan_floor_cm", 70.0);
  vscan_speed = readNumber(root, "vscan_speed_cm_s", 10.0);
  laser_dwell_s = readNumber(root, "laser_dwell_s", 0.5);
  rack_length = readNumber(root, "rack_length_cm", 200.0);
  rack_height = readNumber(root, "rack_height_cm", 190.0);
  board_bottom = readNumber(root, "board_bottom_cm", 60.0);
  aisle_switch_penalty_cm = readNumber(root, "aisle_switch_penalty_cm", 120.0);

  // 注意：这里的局部变量不能叫 takeoff / landing，否则会遮蔽同名的类成员
  const json::Value & takeoff_value = root["takeoff"];
  takeoff.x = readNumber(takeoff_value, "x", 75.0);
  takeoff.y = readNumber(takeoff_value, "y", 75.0);
  takeoff_yaw = readNumber(takeoff_value, "yaw", 90.0);

  const json::Value & landing_value = root["landing"];
  landing.x = readNumber(landing_value, "x", 425.0);
  landing.y = readNumber(landing_value, "y", 325.0);
  landing_yaw = readNumber(landing_value, "yaw", 90.0);
  landing_radius = readNumber(landing_value, "radius", 30.0);

  const json::Value & rack_rows = root["slot_rows_cm"];
  rows.clear();
  for (std::size_t i = 0; i < rack_rows.size(); ++i) {
    rows.push_back(rack_rows[i].as_double());
  }
  if (rows.empty()) rows = {140.0, 100.0};

  const json::Value & column_offsets = root["slot_cols_offset_cm"];
  cols_offset.clear();
  for (std::size_t i = 0; i < column_offsets.size(); ++i) {
    cols_offset.push_back(column_offsets[i].as_double());
  }
  if (cols_offset.empty()) cols_offset = {-50.0, 0.0, 50.0};

  const json::Value & speed_values = root["speeds"];
  speeds.horizontal = readNumber(speed_values, "horizontal", 100.0);
  speeds.vertical = readNumber(speed_values, "vertical", 50.0);
  speeds.yaw = readNumber(speed_values, "yaw", 90.0);
  speeds.land = readNumber(speed_values, "land", 30.0);

  const json::Value & timing_values = root["timing"];
  timing.climb_s = readNumber(timing_values, "climb_s", 4.0);
  timing.settle_s = readNumber(timing_values, "settle_s", 1.5);
  timing.scan_s = readNumber(timing_values, "scan_s", 1.2);
  timing.land_s = readNumber(timing_values, "land_s", 6.0);

  const json::Value & limits = root["time_limits_s"];
  time_limit_sweep_s = readNumber(limits, "sweep", 270.0);
  time_limit_target_s = readNumber(limits, "target", 180.0);

  // ---- 货架与板面 ----
  faces_.clear();
  rack_rects_.clear();
  obstacles_.clear();
  band_planes_.clear();
  band_axis_x_ = true;

  const json::Value & racks = root["racks"];
  for (std::size_t i = 0; i < racks.size(); ++i) {
    const json::Value & rack = racks[i];
    const std::string rack_id = rack["id"].as_string("S" + std::to_string(i + 1));
    std::string axis = rack["axis"].as_string("y");
    for (char & ch : axis) ch = static_cast<char>(std::tolower(static_cast<unsigned char>(ch)));
    const double thickness = readNumber(rack, "thickness", 5.0);

    Vec2 center;
    Vec2 u;
    Rect rect;
    if (axis == "y") {
      double board_x = 0.0;
      double y_from = 0.0;
      double y_to = 0.0;
      if (!readNumberRequired(rack, "board_x", board_x, error, "rack")) return false;
      y_from = readNumber(rack, "y_from", 0.0);
      y_to = readNumber(rack, "y_to", rack_length);
      center = Vec2{board_x, (y_from + y_to) / 2.0};
      u = Vec2{0.0, 1.0};
      rect = Rect{board_x - thickness / 2.0, y_from, board_x + thickness / 2.0, y_to};
      band_axis_x_ = true;
    } else {
      double board_y = 0.0;
      double x_from = 0.0;
      double x_to = 0.0;
      if (!readNumberRequired(rack, "board_y", board_y, error, "rack")) return false;
      x_from = readNumber(rack, "x_from", 0.0);
      x_to = readNumber(rack, "x_to", rack_length);
      center = Vec2{(x_from + x_to) / 2.0, board_y};
      u = Vec2{1.0, 0.0};
      rect = Rect{x_from, board_y - thickness / 2.0, x_to, board_y + thickness / 2.0};
      band_axis_x_ = false;
    }
    const double length = std::hypot(
      (rect.x1 - rect.x0) * u.x, (rect.y1 - rect.y0) * u.y);
    rack_rects_.push_back(rect);
    obstacles_.push_back(rect);
    band_planes_.push_back(band_axis_x_ ? center.x : center.y);

    const json::Value & rack_faces = rack["faces"];
    for (std::size_t f = 0; f < rack_faces.size(); ++f) {
      const json::Value & face = rack_faces[f];
      FaceInfo info;
      info.id = face["id"].as_string(std::string("F") + std::to_string(f + 1));
      info.rack = rack_id;
      info.center = center;
      info.u = u;
      info.thickness = thickness;
      info.length = length > 1e-6 ? length : rack_length;
      const json::Value & normal = face["normal"];
      if (normal.size() >= 2) {
        info.normal = normalized(normal[0].as_double(1.0), normal[1].as_double(0.0));
      } else {
        info.normal = Vec2{1.0, 0.0};
      }
      const json::Value & row_delta = face["row_delta_cm"];
      for (std::size_t k = 0; k < row_delta.size(); ++k) {
        info.row_delta.push_back(row_delta[k].as_double(0.0));
      }
      info.y_offset = readNumber(face, "y_offset_cm", 0.0);
      // 该面的悬停距离修正（负数 = 更贴近板面）。缺省 0 = 用全局 standoff_cm。
      info.standoff_delta = readNumber(face, "standoff_delta_cm", 0.0);
      faces_.push_back(info);
    }
  }
  if (faces_.empty()) {
    error = "场地配置中没有任何货架面";
    return false;
  }

  build();
  return true;
}

void FieldGeometry::build()
{
  slots_.clear();
  for (const FaceInfo & face : faces_) {
    buildFaceSlots(face);
  }
  buildWarnings();
}

// 场地配置的一致性提示（只提示、不改数据）。
//
// 2026-09-27：列的配对已改成"按列号"，所以同一列上下两排的悬停 y **允许**不同
// （悬停点是举机实测的，上下两排高度不同、激光落点会差几厘米）。
// 但差得太多往往意味着量测出了问题，或者某一排的 y 填错了 —— 这里把差异
// 记下来，启动时由任务节点用 WARN 打出来，免得又变成"航线默默变了"才发现。
void FieldGeometry::buildWarnings()
{
  warnings_.clear();
  std::map<std::pair<std::string, int>, std::vector<const SlotInfo *>> groups;
  for (const SlotInfo & info : slots_) {
    groups[std::make_pair(info.face, info.col)].push_back(&info);
  }
  for (const auto & entry : groups) {
    const std::vector<const SlotInfo *> & members = entry.second;
    if (members.size() < 2) continue;                  // 单点列（缺一个点）不提示
    double lo = members.front()->hover.y;
    double hi = lo;
    for (const SlotInfo * member : members) {
      lo = std::min(lo, member->hover.y);
      hi = std::max(hi, member->hover.y);
    }
    if (hi - lo <= 1.0) continue;                       // 1cm 以内属正常实测误差
    char spread[32];
    std::snprintf(spread, sizeof(spread), "%.1f", hi - lo);
    std::string text = entry.first.first + " 面第 " +
      std::to_string(entry.first.second) + " 列上下两排的悬停 y 相差 " +
      spread + "cm（";
    for (std::size_t k = 0; k < members.size(); ++k) {
      if (k > 0) text += "、";
      char value[32];
      std::snprintf(value, sizeof(value), "%.1f", members[k]->hover.y);
      text += members[k]->id + "=" + value;
    }
    text += "）——列配对按列号，航线不受影响（没有重复点位）";
    warnings_.push_back(text);
  }
}

void FieldGeometry::buildFaceSlots(const FaceInfo & face)
{
  // 站立在该面正前方时的“左手方向”：视线 f = -normal，向上 u = +Z，
  // 右手 = f x u = (fy, -fx) —— 与地面站 field.py 的 _left_vector 一致。
  const Vec2 left{face.normal.y, -face.normal.x};
  const std::size_t ncols = cols_offset.size();
  const double projection = left.x * face.u.x + left.y * face.u.y;

  std::vector<double> offsets = cols_offset;
  if (projection > 0.0) {
    std::sort(offsets.begin(), offsets.end(), std::greater<double>());
  } else {
    std::sort(offsets.begin(), offsets.end());
  }

  for (std::size_t r = 0; r < rows.size(); ++r) {
    // 该面的行高微调（field.json 的 faces[].row_delta_cm）
    const double row_z = rows[r] + (r < face.row_delta.size() ? face.row_delta[r] : 0.0);
    // 该面的悬停距离：全局 standoff 加上本面的修正量（可正可负）。
    // 现场某一面贴得比模型更远/更近时，用它单独补偿，其它面不受影响。
    const double face_standoff = standoff + face.standoff_delta;
    for (std::size_t c = 0; c < offsets.size(); ++c) {
      SlotInfo info;
      info.row = static_cast<int>(r) + 1;
      info.col = static_cast<int>(c) + 1;
      info.index = info.row * static_cast<int>(ncols) - static_cast<int>(ncols) + info.col;
      info.id = face.id + std::to_string(info.index);
      info.face = face.id;
      info.rack = face.rack;
      info.normal = face.normal;
      const double offset = offsets[c];
      const double qx = face.center.x + face.normal.x * face.thickness / 2.0 + face.u.x * offset;
      const double qy = face.center.y + face.normal.y * face.thickness / 2.0 +
        face.u.y * offset + face.y_offset;
      info.qr = Vec3{qx, qy, row_z};
      info.hover = Vec3{
        qx + face.normal.x * face_standoff,
        qy + face.normal.y * face_standoff,
        row_z};
      // 实测覆盖优先：模型的行/列/悬停距离都可能差几厘米，实测值直接顶替 (x,y)。
      const auto override_it = slot_hover_xy.find(info.id);
      if (override_it != slot_hover_xy.end()) {
        info.hover.x = override_it->second.x;
        info.hover.y = override_it->second.y;
        // 实测 y 同时也是**二维码自己的 y**：激光打在码中心时飞机就在这个 y 上，
        // 所以"码的 y"和"悬停点的 y"必须是同一个值。不这么做的话，现场编号
        // 与模型列序相反的那一面（现在是 B 面）上报的货物坐标会是镜像的。
        info.qr.y = override_it->second.y;
      }
      // 可选的高度覆盖（[x, y, z] 的第三项）
      // 注意：地面站的 field.py 是"一个 z_eff 同时用于二维码和悬停点"，
      // 这里必须跟它保持一致，否则 shelf_selftest 的"点位坐标与地面站一致"会失败。
      const auto z_it = slot_hover_z.find(info.id);
      if (z_it != slot_hover_z.end()) {
        info.hover.z = z_it->second;
        info.qr.z = z_it->second;
      }
      // 飞行目标：默认就等于悬停点；有现场过冲时再加（只改"往哪飞"）
      info.fly_hover = info.hover;
      const auto ov_it = slot_fly_overshoot.find(info.id);
      if (ov_it != slot_fly_overshoot.end()) {
        info.fly_hover.x += ov_it->second.x;
        info.fly_hover.y += ov_it->second.y;
        info.fly_hover.z += ov_it->second.z;
      }
      info.yaw = yawOf(Vec2{-face.normal.x, -face.normal.y});
      slots_.push_back(info);
    }
  }
}

const SlotInfo * FieldGeometry::slot(const std::string & id) const
{
  for (const SlotInfo & info : slots_) {
    if (info.id == id) return &info;
  }
  return nullptr;
}

SlotInfo * FieldGeometry::mutableSlot(const std::string & id)
{
  for (SlotInfo & info : slots_) {
    if (info.id == id) return &info;
  }
  return nullptr;
}

std::vector<std::string> FieldGeometry::slotIdsInOrder() const
{
  std::vector<const SlotInfo *> ordered;
  ordered.reserve(slots_.size());
  for (const SlotInfo & info : slots_) ordered.push_back(&info);
  std::sort(
    ordered.begin(), ordered.end(),
    [](const SlotInfo * a, const SlotInfo * b) {
      if (a->face != b->face) return a->face < b->face;
      if (a->row != b->row) return a->row < b->row;
      return a->col < b->col;
    });
  std::vector<std::string> ids;
  ids.reserve(ordered.size());
  for (const SlotInfo * info : ordered) ids.push_back(info->id);
  return ids;
}

std::string FieldGeometry::nearestSlot(double x, double y, double z) const
{
  std::string best;
  double best_distance = 1e18;
  for (const SlotInfo & info : slots_) {
    const double distance =
      std::hypot(info.qr.x - x, info.qr.y - y) + std::fabs(info.qr.z - z) * 0.5;
    if (distance < best_distance) {
      best_distance = distance;
      best = info.id;
    }
  }
  return best;
}

std::string FieldGeometry::aisleOf(const SlotInfo & info) const
{
  const double position = band_axis_x_ ? info.hover.x : info.hover.y;
  std::vector<double> planes = band_planes_;
  std::sort(planes.begin(), planes.end());
  int index = 0;
  for (double plane : planes) {
    if (position > plane) ++index;
  }
  return "band" + std::to_string(index);
}

Rect FieldGeometry::inflated(const Rect & rect) const
{
  return Rect{
    rect.x0 - safety_margin, rect.y0 - safety_margin,
    rect.x1 + safety_margin, rect.y1 + safety_margin};
}

bool FieldGeometry::pointFree(double x, double y) const
{
  if (x < wall_margin || x > width - wall_margin) return false;
  if (y < wall_margin || y > height - wall_margin) return false;
  for (const Rect & rect : obstacles_) {
    const Rect area = inflated(rect);
    if (x >= area.x0 && x <= area.x1 && y >= area.y0 && y <= area.y1) return false;
  }
  return true;
}

bool FieldGeometry::segmentFree(
  double x0, double y0, double x1, double y1, int samples) const
{
  if (!pointFree(x0, y0) || !pointFree(x1, y1)) return false;
  const int count = std::max(2, samples);
  for (int i = 1; i < count; ++i) {
    const double t = static_cast<double>(i) / static_cast<double>(count);
    if (!pointFree(x0 + (x1 - x0) * t, y0 + (y1 - y0) * t)) return false;
  }
  return true;
}

Vec2 FieldGeometry::clampToSafeStandoff(
  const Vec2 & point, double min_clearance, bool * clamped) const
{
  Vec2 result = point;
  if (clamped != nullptr) *clamped = false;
  for (const Rect & rect : rack_rects_) {
    if (band_axis_x_) {
      // 板面沿 Y 竖放：只在板面自身的 y 范围内做 X 方向推离
      if (result.y < rect.y0 || result.y > rect.y1) continue;
      const double left = rect.x0 - min_clearance;
      const double right = rect.x1 + min_clearance;
      if (result.x > left && result.x < right) {
        result.x = (result.x < (rect.x0 + rect.x1) / 2.0) ? left : right;
        if (clamped != nullptr) *clamped = true;
      }
    } else {
      if (result.x < rect.x0 || result.x > rect.x1) continue;
      const double bottom = rect.y0 - min_clearance;
      const double top = rect.y1 + min_clearance;
      if (result.y > bottom && result.y < top) {
        result.y = (result.y < (rect.y0 + rect.y1) / 2.0) ? bottom : top;
        if (clamped != nullptr) *clamped = true;
      }
    }
  }
  return result;
}

}  // namespace shelf_scan
