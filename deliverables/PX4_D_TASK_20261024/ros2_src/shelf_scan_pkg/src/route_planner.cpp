#include "shelf_scan_pkg/route_planner.hpp"

#include <algorithm>
#include <cmath>
#include <functional>
#include <limits>
#include <map>
#include <queue>
#include <set>
#include <tuple>

#include "shelf_scan_pkg/json_lite.hpp"

namespace shelf_scan
{
namespace
{

// 可见图节点在膨胀矩形角点外再外扩的量（与地面站一致）。
// 原来是 10cm，等于在 safety_margin 之外又多留 10cm；改成 2cm 后
// 绕行路线正好贴着配置的安全余量走，绕货架两端的"下去再上来"能短 ~30cm。
constexpr double kEdgeOffset = 2.0;
constexpr double kEpsilon = 1e-9;

double round2(double value)
{
  return std::round(value * 100.0) / 100.0;
}

double distance2d(const Vec2 & a, const Vec2 & b)
{
  return std::hypot(a.x - b.x, a.y - b.y);
}

double normalizeDeg(double angle)
{
  double value = std::fmod(angle, 360.0);
  if (value < 0.0) value += 360.0;
  return value;
}

double angleDiffDeg(double a, double b)
{
  return std::fabs(normalizeDeg(a - b + 180.0) - 180.0);
}

struct GraphNode
{
  Vec2 point;
};

// 一列：同一 **列号**（SlotInfo::col）上的上下两个点位
struct ColumnInfo
{
  double x = 0.0;
  double y = 0.0;      // 该列的平均悬停 y：只用来给列排序
  int col = 0;         // 列号（1/2/3），y 相同时的稳定排序键
  double yaw = 0.0;
  const SlotInfo * top = nullptr;
  const SlotInfo * bottom = nullptr;
};

// 2026-09-27：列的配对改成按 **列号**（SlotInfo::col），不再要求"上下两排 hover_y 完全相等"。
//
// 老逻辑是按 hover_y 相等配对：两个点 hover_y 一样才算一列。但悬停点是现场
// "举着飞机让激光打到二维码中心"实测出来的，同一列上下两排高度不同、激光落点会
// 差几厘米，于是 A6(154) 与 A3(157)、B6(142) 与 B3(146) 不再相等，那一列被拆成
// 两个"只有 1 个点"的列（top == bottom），serpentineOrder() 的
// "上排一趟 + 下排折返"就把这两个点各排了两次 —— 航线上多出 4 个重复点位
// （机载端自测的「扫描航段数等于点位数」16 != 12 就是这么来的）。
// 现在按列号配对：上下两排的 y 可以各自不同，列顺序按该列的平均 y 从小到大。
std::vector<ColumnInfo> columnsOfFace(const FieldGeometry & field, const std::string & face_id)
{
  std::map<std::pair<std::string, int>, std::vector<const SlotInfo *>> grouped;
  for (const SlotInfo & info : field.slots()) {
    if (info.face != face_id) continue;
    grouped[std::make_pair(info.rack, info.col)].push_back(&info);
  }
  std::vector<ColumnInfo> columns;
  columns.reserve(grouped.size());
  for (auto & entry : grouped) {
    std::vector<const SlotInfo *> & members = entry.second;
    // 高的那排当 top；高度相同时按编号定序，保证两端结果一致
    std::sort(members.begin(), members.end(), [](const SlotInfo * a, const SlotInfo * b) {
      if (std::fabs(a->hover.z - b->hover.z) > 1e-9) return a->hover.z > b->hover.z;
      return a->id < b->id;
    });
    ColumnInfo col;
    col.x = members.front()->hover.x;
    col.col = entry.first.second;
    col.yaw = members.front()->yaw;
    col.top = members.front();
    col.bottom = members.back();
    double sum = 0.0;
    for (const SlotInfo * member : members) sum += member->hover.y;
    col.y = sum / static_cast<double>(members.size());
    columns.push_back(col);
  }
  std::sort(columns.begin(), columns.end(), [](const ColumnInfo & a, const ColumnInfo & b) {
    if (std::fabs(a.y - b.y) > 1e-9) return a.y < b.y;
    return a.col < b.col;
  });
  return columns;
}

}  // namespace

std::vector<Vec2> RoutePlanner::path(const Vec2 & from, const Vec2 & to) const
{
  if (field_.segmentFree(from.x, from.y, to.x, to.y)) {
    return {from, to};
  }

  std::vector<GraphNode> nodes;
  nodes.push_back(GraphNode{from});
  for (const Rect & rect : field_.obstacleRects()) {
    const Rect area = field_.inflated(rect);
    nodes.push_back(GraphNode{Vec2{area.x0 - kEdgeOffset, area.y0 - kEdgeOffset}});
    nodes.push_back(GraphNode{Vec2{area.x0 - kEdgeOffset, area.y1 + kEdgeOffset}});
    nodes.push_back(GraphNode{Vec2{area.x1 + kEdgeOffset, area.y0 - kEdgeOffset}});
    nodes.push_back(GraphNode{Vec2{area.x1 + kEdgeOffset, area.y1 + kEdgeOffset}});
  }
  nodes.push_back(GraphNode{to});

  const std::size_t count = nodes.size();
  const std::size_t goal = count - 1;

  std::vector<std::vector<std::pair<std::size_t, double>>> adjacency(count);
  for (std::size_t i = 0; i < count; ++i) {
    for (std::size_t j = i + 1; j < count; ++j) {
      if (field_.segmentFree(
          nodes[i].point.x, nodes[i].point.y,
          nodes[j].point.x, nodes[j].point.y, 30))
      {
        const double weight = distance2d(nodes[i].point, nodes[j].point);
        adjacency[i].emplace_back(j, weight);
        adjacency[j].emplace_back(i, weight);
      }
    }
  }

  std::vector<double> best(count, std::numeric_limits<double>::infinity());
  std::vector<long> previous(count, -1);
  using Entry = std::pair<double, std::size_t>;
  std::priority_queue<Entry, std::vector<Entry>, std::greater<Entry>> queue;
  best[0] = 0.0;
  queue.emplace(0.0, 0);
  while (!queue.empty()) {
    const Entry entry = queue.top();
    queue.pop();
    if (entry.first > best[entry.second] + kEpsilon) continue;
    if (entry.second == goal) break;
    for (const auto & edge : adjacency[entry.second]) {
      const double candidate = entry.first + edge.second;
      if (candidate < best[edge.first] - kEpsilon) {
        best[edge.first] = candidate;
        previous[edge.first] = static_cast<long>(entry.second);
        queue.emplace(candidate, edge.first);
      }
    }
  }

  std::vector<Vec2> chain;
  if (std::isfinite(best[goal])) {
    long index = static_cast<long>(goal);
    while (index != -1) {
      chain.push_back(nodes[static_cast<std::size_t>(index)].point);
      index = previous[static_cast<std::size_t>(index)];
    }
    std::reverse(chain.begin(), chain.end());
  } else {
    chain = {from, to};  // 退化处理：直达
  }

  // 去掉共线 / 可直达的中间点
  std::vector<Vec2> simplified;
  if (chain.size() <= 2) {
    simplified = chain;
  } else {
    simplified.push_back(chain.front());
    std::size_t i = 0;
    while (i + 1 < chain.size()) {
      std::size_t j = chain.size() - 1;
      while (j > i + 1 &&
        !field_.segmentFree(chain[i].x, chain[i].y, chain[j].x, chain[j].y, 40))
      {
        --j;
      }
      simplified.push_back(chain[j]);
      i = j;
    }
  }
  return simplified;
}

double RoutePlanner::eta(const std::vector<RouteLeg> & legs) const
{
  const double vh = field_.speeds.horizontal > 0.0 ? field_.speeds.horizontal : 100.0;
  const double vv = field_.speeds.vertical > 0.0 ? field_.speeds.vertical : 50.0;
  const double vy = field_.speeds.yaw > 0.0 ? field_.speeds.yaw : 90.0;
  const double vs = field_.vscan_speed > 0.0 ? field_.vscan_speed : 10.0;
  double total = 0.0;
  bool has_last_yaw = false;
  double last_yaw = 0.0;
  for (const RouteLeg & leg : legs) {
    if (leg.type == "takeoff" || leg.type == "land") {
      total += leg.type == "takeoff" ? field_.timing.climb_s : field_.timing.land_s;
      if (leg.has_yaw) {
        last_yaw = leg.yaw;
        has_last_yaw = true;
      }
      continue;
    }
    // 纵向扫描的下降是慢速扫描，不能按常规垂直速度算
    const double vdown = (leg.type == "vscan") ? vs : vv;
    for (std::size_t k = 1; k < leg.points.size(); ++k) {
      const Vec3 & p = leg.points[k - 1];
      const Vec3 & q = leg.points[k];
      total += std::hypot(p.x - q.x, p.y - q.y) / vh;
      total += std::fabs(q.z - p.z) / vdown;
    }
    if (leg.type == "scan" || leg.type == "vscan") {
      // vscan 一列要上下两排各停留一次
      total += (leg.type == "vscan" ? 2.0 : 1.0) *
        (field_.timing.settle_s + field_.timing.scan_s);
      if (leg.has_yaw && has_last_yaw) {
        total += angleDiffDeg(leg.yaw, last_yaw) / vy;
      }
      if (leg.has_yaw) {
        last_yaw = leg.yaw;
        has_last_yaw = true;
      }
    }
  }
  return std::round(total * 10.0) / 10.0;
}

RouteLeg RoutePlanner::buildTransitLeg(
  const Vec2 & from, double from_z, const SlotInfo & target, int order) const
{
  RouteLeg leg;
  leg.type = "transit";
  leg.slot = target.id;
  leg.order = order;
  leg.has_yaw = true;
  leg.yaw = target.yaw;   // 巡航途中就转到盘点朝向，节省到位后的转向时间
  leg.label = "飞往 " + target.id;

  // 转场段的终点用 fly_hover（= hover + 现场过冲）：飞机要飞到的是这里
  const std::vector<Vec2> points = path(from, Vec2{target.fly_hover.x, target.fly_hover.y});
  const double segments = static_cast<double>(points.size() > 1 ? points.size() - 1 : 1);
  for (std::size_t i = 0; i < points.size(); ++i) {
    const double t = static_cast<double>(i) / segments;
    leg.points.push_back(Vec3{
      round2(points[i].x), round2(points[i].y),
      round2(from_z + (target.fly_hover.z - from_z) * t)});
  }
  return leg;
}

std::vector<std::string> RoutePlanner::orderSlots(
  const std::vector<std::string> & slot_ids) const
{
  std::vector<const SlotInfo *> remaining;
  remaining.reserve(slot_ids.size());
  for (const std::string & id : slot_ids) {
    const SlotInfo * info = field_.slot(id);
    if (info != nullptr) remaining.push_back(info);
  }

  const double horizontal = field_.speeds.horizontal > 0.0 ? field_.speeds.horizontal : 100.0;
  const double vertical = field_.speeds.vertical > 0.0 ? field_.speeds.vertical : 50.0;
  const double z_weight = horizontal / std::max(1e-6, vertical);
  const double penalty = field_.aisle_switch_penalty_cm;

  // 起点：起飞点（巡航高度）
  SlotInfo start;
  start.id = "TAKEOFF";
  start.hover = Vec3{field_.takeoff.x, field_.takeoff.y, field_.cruise_alt};
  SlotInfo land;
  land.id = "LAND";
  land.hover = Vec3{field_.landing.x, field_.landing.y, 0.0};

  std::vector<const SlotInfo *> order;
  order.reserve(remaining.size());
  const SlotInfo * current = &start;
  double current_z = field_.cruise_alt;
  while (!remaining.empty()) {
    std::size_t best_index = 0;
    double best_cost = std::numeric_limits<double>::infinity();
    for (std::size_t i = 0; i < remaining.size(); ++i) {
      const double d = std::hypot(
        current->hover.x - remaining[i]->hover.x,
        current->hover.y - remaining[i]->hover.y) +
        z_weight * std::fabs(current_z - remaining[i]->hover.z);
      const double cost = field_.aisleOf(*current) == field_.aisleOf(*remaining[i]) ?
        d : d + penalty;
      if (cost < best_cost) {
        best_cost = cost;
        best_index = i;
      }
    }
    current = remaining[best_index];
    current_z = current->hover.z;
    order.push_back(current);
    remaining.erase(remaining.begin() + static_cast<long>(best_index));
  }

  const auto total_cost = [&](const std::vector<const SlotInfo *> & sequence) {
      double total = 0.0;
      const SlotInfo * previous_point = &start;
      double previous_z = field_.cruise_alt;
      for (const SlotInfo * item : sequence) {
        const double d = std::hypot(
          previous_point->hover.x - item->hover.x,
          previous_point->hover.y - item->hover.y) +
          z_weight * std::fabs(previous_z - item->hover.z);
        total += field_.aisleOf(*previous_point) == field_.aisleOf(*item) ? d : d + penalty;
        previous_point = item;
        previous_z = item->hover.z;
      }
      const double d = std::hypot(
        previous_point->hover.x - land.hover.x, previous_point->hover.y - land.hover.y) +
        z_weight * std::fabs(previous_z - land.hover.z);
      total += field_.aisleOf(*previous_point) == field_.aisleOf(land) ? d : d + penalty;
      return total;
    };

  double best_total = total_cost(order);
  bool improved = true;
  while (improved) {
    improved = false;
    for (std::size_t i = 0; i + 1 < order.size(); ++i) {
      for (std::size_t j = i + 1; j < order.size(); ++j) {
        std::vector<const SlotInfo *> candidate = order;
        std::reverse(
          candidate.begin() + static_cast<long>(i),
          candidate.begin() + static_cast<long>(j) + 1);
        const double value = total_cost(candidate);
        if (value + 1e-6 < best_total) {
          order = candidate;
          best_total = value;
          improved = true;
        }
      }
    }
  }

  std::vector<std::string> ids;
  ids.reserve(order.size());
  for (const SlotInfo * item : order) ids.push_back(item->id);
  return ids;
}

// 遍历盘点航线：两个固定高度的"弓字形"逐点扫描。
//
// 遍历盘点航线（今天以前的老逻辑）：一串悬停点组成的链。
//
// 12 个悬停点按"最近邻 + 2-opt"排好序组成一条链，逐个飞过去悬停扫描；
// 某个点没扫到时，机载端会自己沿板面法线做微调重试（见 mission_node）。
// 与 groundstation/gs/planner.py 的 plan_sweep 完全等价。
Route RoutePlanner::planSweep(const std::vector<std::string> & order) const
{
  return planPointwise(order.empty() ? serpentineOrder() : order);
}

std::vector<std::string> RoutePlanner::serpentineOrder() const
{
  std::vector<std::string> ids;
  for (const FaceInfo & face : field_.faces()) {
    const std::vector<ColumnInfo> columns = columnsOfFace(field_, face.id);   // 按 y 升序
    for (const ColumnInfo & col : columns) {
      if (col.top != nullptr) ids.push_back(col.top->id);
    }
    for (auto it = columns.rbegin(); it != columns.rend(); ++it) {
      if (it->bottom != nullptr) ids.push_back(it->bottom->id);
    }
  }
  return ids;
}

Route RoutePlanner::planPointwise(const std::vector<std::string> & order) const
{
  const std::vector<std::string> slot_ids =
    order.empty() ? orderSlots(field_.slotIdsInOrder()) : order;

  Route route;
  route.kind = "sweep";

  RouteLeg takeoff;
  takeoff.type = "takeoff";
  takeoff.has_yaw = true;
  takeoff.yaw = field_.takeoff_yaw;
  takeoff.points.push_back(Vec3{field_.takeoff.x, field_.takeoff.y, 0.0});
  takeoff.points.push_back(Vec3{field_.takeoff.x, field_.takeoff.y, field_.cruise_alt});
  takeoff.label = "起飞上升至 " + json::Value::formatNumber(field_.cruise_alt) + "cm";
  route.legs.push_back(takeoff);

  Vec2 previous{field_.takeoff.x, field_.takeoff.y};
  double current_z = field_.cruise_alt;
  int index = 0;
  for (const std::string & id : slot_ids) {
    const SlotInfo * target = field_.slot(id);
    if (target == nullptr) continue;
    ++index;
    route.legs.push_back(buildTransitLeg(previous, current_z, *target, index));

    RouteLeg scan;
    scan.type = "scan";
    scan.slot = target->id;
    scan.order = index;
    scan.hover = target->hover;
    scan.qr = Vec2{target->qr.x, target->qr.y};
    scan.yaw = target->yaw;
    scan.has_yaw = true;
    scan.laser_s = field_.laser_dwell_s;
    // 航段的目标点用 fly_hover（= hover + 现场过冲）：飞机实际要飞到的是这里。
    // scan.hover 仍保留标定悬停点，供激光判据/上报使用。
    scan.points.push_back(target->fly_hover);
    scan.label = "盘点 " + target->id;
    route.legs.push_back(scan);

    previous = Vec2{target->fly_hover.x, target->fly_hover.y};
    current_z = target->fly_hover.z;
  }

  RouteLeg back;
  back.type = "return";
  back.label = "返航";
  const std::vector<Vec2> back_path =
    path(previous, Vec2{field_.landing.x, field_.landing.y});
  for (const Vec2 & point : back_path) {
    back.points.push_back(Vec3{round2(point.x), round2(point.y), round2(current_z)});
  }
  route.legs.push_back(back);

  RouteLeg land;
  land.type = "land";
  land.has_yaw = true;
  land.yaw = field_.landing_yaw;
  land.label = "降落";
  land.points.push_back(Vec3{field_.landing.x, field_.landing.y, current_z});
  land.points.push_back(Vec3{field_.landing.x, field_.landing.y, 0.0});
  route.legs.push_back(land);

  finalize(route, "sweep", "遍历盘点航线（" + std::to_string(index) + " 个点）");
  return route;
}

// 已知货物所在点位：直接飞过去悬停扫描（定向盘点用）
Route RoutePlanner::planDirect(const std::string & slot_id, int number) const
{
  Route route;
  const SlotInfo * info = field_.slot(slot_id);
  if (info == nullptr) return route;   // 空航线，调用方按 invalid 处理

  const double hx = info->hover.x;
  const double hy = info->hover.y;
  const double hz = info->hover.z;

  RouteLeg takeoff;
  takeoff.type = "takeoff";
  takeoff.has_yaw = true;
  takeoff.yaw = field_.takeoff_yaw;
  takeoff.points.push_back(Vec3{field_.takeoff.x, field_.takeoff.y, 0.0});
  takeoff.points.push_back(Vec3{field_.takeoff.x, field_.takeoff.y, field_.cruise_alt});
  takeoff.label = "起飞上升至 " + json::Value::formatNumber(field_.cruise_alt) + "cm";
  route.legs.push_back(takeoff);

  RouteLeg leg;
  leg.type = "transit";
  leg.slot = slot_id;
  leg.order = 1;
  leg.has_yaw = true;
  leg.yaw = info->yaw;
  leg.label = "直达 " + slot_id;
  const std::vector<Vec2> pts =
    path(Vec2{field_.takeoff.x, field_.takeoff.y}, Vec2{hx, hy});
  const double segs = static_cast<double>(pts.size() > 1 ? pts.size() - 1 : 1);
  for (std::size_t i = 0; i < pts.size(); ++i) {
    const double t = static_cast<double>(i) / segs;
    leg.points.push_back(Vec3{
      round2(pts[i].x), round2(pts[i].y), round2(field_.cruise_alt + (hz - field_.cruise_alt) * t)});
  }
  route.legs.push_back(leg);

  RouteLeg scan;
  scan.type = "scan";
  scan.slot = slot_id;
  scan.order = 1;
  scan.hover = Vec3{hx, hy, hz};
  scan.qr = Vec2{info->qr.x, info->qr.y};
  scan.has_yaw = true;
  scan.yaw = info->yaw;
  scan.laser_s = field_.laser_dwell_s;
  scan.points.push_back(Vec3{hx, hy, hz});
  scan.label = "盘点 " + slot_id;
  route.legs.push_back(scan);

  RouteLeg back;
  back.type = "return";
  back.label = "返航";
  const std::vector<Vec2> back_path = path(Vec2{hx, hy}, Vec2{field_.landing.x, field_.landing.y});
  for (const Vec2 & point : back_path) {
    back.points.push_back(Vec3{round2(point.x), round2(point.y), round2(hz)});
  }
  route.legs.push_back(back);

  RouteLeg land;
  land.type = "land";
  land.has_yaw = true;
  land.yaw = field_.landing_yaw;
  land.label = "降落";
  land.points.push_back(Vec3{field_.landing.x, field_.landing.y, hz});
  land.points.push_back(Vec3{field_.landing.x, field_.landing.y, 0.0});
  route.legs.push_back(land);

  finalize(route, "target",
           "定向盘点航线：货物 " + std::to_string(number) + " -> " + slot_id + "（直达）");
  route.target_no = number;
  route.target_slot = slot_id;
  route.mode = "direct";
  return route;
}

Route RoutePlanner::planTarget(
  int number, const std::map<std::string, int> & mapping, bool search_all) const
{
  std::string slot_id;
  for (const auto & entry : mapping) {
    if (entry.second == number) {
      slot_id = entry.first;
      break;
    }
  }

  if (!slot_id.empty() && !search_all) {
    return planDirect(slot_id, number);
  }

  Route route = planSweep();
  route.kind = "target";
  route.mode = "search";
  route.target_no = number;
  route.title = number > 0 ?
    "定向盘点航线：货物 " + std::to_string(number) + " 位置未知，按优化搜索航线逐点识别" :
    "遍历盘点航线";
  return route;
}

void RoutePlanner::finalize(Route & route, const char * kind, const std::string & title) const
{
  route.kind = kind;
  route.title = title;
  double length = 0.0;
  for (const RouteLeg & leg : route.legs) {
    for (std::size_t k = 1; k < leg.points.size(); ++k) {
      // 与地面站一致：航程只统计水平距离
      length += std::hypot(
        leg.points[k - 1].x - leg.points[k].x,
        leg.points[k - 1].y - leg.points[k].y);
    }
  }
  route.length_cm = std::round(length * 10.0) / 10.0;
  route.eta_s = eta(route.legs);
}

}  // namespace shelf_scan
