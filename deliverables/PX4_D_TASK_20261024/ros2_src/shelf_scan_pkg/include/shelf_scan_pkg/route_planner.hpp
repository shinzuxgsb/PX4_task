// 航线规划（机载备份）：可见图避障 + 最近邻 + 2-opt。
//
// 正常比赛时航线由地面站规划后通过 start_route 下发，机载端只负责执行；
// 这里的实现与 groundstation/gs/planner.py 完全一致，用于：
//   1) 地面站未下发航线时的本地兜底（例如只做机载自测）；
//   2) 用 test/golden.json 做交叉验证。
#pragma once

#include <array>
#include <map>
#include <string>
#include <vector>

#include "shelf_scan_pkg/field_geometry.hpp"

namespace shelf_scan
{

struct RouteLeg
{
  std::string type;              // takeoff / transit / scan / vscan / return / land
  std::vector<Vec3> points;      // 该航段的航点（场地坐标，cm）
  std::string slot;              // scan 段对应的坐标编号
  int order = 0;                 // scan 段的次序，从 1 开始
  Vec3 hover;                    // scan 段悬停点
  Vec2 qr;                       // scan 段二维码中心
  double yaw = 0.0;              // 目标机头朝向（度）
  bool has_yaw = false;
  double laser_s = 0.5;          // 激光点亮时长
  std::string label;             // 中文说明（仅日志用）

  // ---- vscan（纵向扫描）专用：一列 = 同一悬停 x/y 上的上下两排 ----
  std::string face;                    // "A" / "B"
  int column = 0;                      // 该面第几列，从 1 开始
  std::vector<std::string> slots;      // [上排编号, 下排编号]
  std::vector<double> rows;            // [上排行高, 下排行高]（模型值，用于按高度归属）
};

struct Route
{
  std::string kind = "sweep";    // sweep / target
  std::string title;
  std::string mode;              // direct / search（定向盘点）
  int target_no = 0;             // 定向盘点的目标编号（0 表示未指定）
  std::string target_slot;
  double length_cm = 0.0;
  double eta_s = 0.0;
  std::vector<RouteLeg> legs;
  bool valid() const { return !legs.empty(); }
};

class RoutePlanner
{
public:
  explicit RoutePlanner(const FieldGeometry & field) : field_(field) {}

  // 遍历盘点航线；order 为空时自动按最近邻 + 2-opt 排序
  Route planSweep(const std::vector<std::string> & order = std::vector<std::string>()) const;

  // 定向盘点航线：已知目标位置则直达，否则按优化顺序逐点搜索
  Route planTarget(
    int number, const std::map<std::string, int> & mapping, bool search_all = false) const;

  std::vector<std::string> orderSlots(const std::vector<std::string> & slot_ids) const;

  // 两点间避障路径（场地坐标）
  std::vector<Vec2> path(const Vec2 & from, const Vec2 & to) const;

  // 按地面站同一套参数估算用时
  double eta(const std::vector<RouteLeg> & legs) const;

private:
  RouteLeg buildTransitLeg(const Vec2 & from, double from_z, const SlotInfo & target, int order) const;
  // 老的"逐点悬停扫描"（指定点位时用）
  Route planPointwise(const std::vector<std::string> & order) const;
  // 每面"上排正向 + 下排折返"的固定顺序（与地面站 gs/planner.py 的
  // _serpentine_order 完全一致）：A3 A2 A1 A4 A5 A6 / B1 B2 B3 B6 B5 B4
  std::vector<std::string> serpentineOrder() const;
  // 已知货物所在点位：直接飞过去悬停扫描
  Route planDirect(const std::string & slot_id, int number) const;
  void finalize(Route & route, const char * kind, const std::string & title) const;

  const FieldGeometry & field_;
};

}  // namespace shelf_scan
