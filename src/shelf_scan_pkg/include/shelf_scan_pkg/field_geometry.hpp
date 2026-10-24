// D 题场地几何：由 field.json 推导 24 个盘点点位、障碍物与通行判断。
//
// 本文件的算法与地面站 groundstation/gs/field.py 一一对应，保证机载端
// 与地面站算出的坐标、朝向完全一致（可直接用 test/golden.json 校验）。
#pragma once

#include <string>
#include <map>
#include <utility>
#include <vector>

namespace shelf_scan
{

struct Vec2
{
  double x = 0.0;
  double y = 0.0;
};

struct Vec3
{
  double x = 0.0;
  double y = 0.0;
  double z = 0.0;
};

struct Rect
{
  double x0 = 0.0;
  double y0 = 0.0;
  double x1 = 0.0;
  double y1 = 0.0;
};

struct FaceInfo
{
  std::string id;
  std::string rack;
  Vec2 normal;      // 面法线（单位向量，指向板面外侧）
  Vec2 center;      // 板面中心
  Vec2 u;           // 板面长边方向（单位向量）
  double thickness = 5.0;
  double length = 200.0;
  // 按面微调（field.json 的 faces[].row_delta_cm / y_offset_cm / standoff_delta_cm）：
  // 每个面的二维码未必正好在模型行高/列位上，这里允许逐面修正。
  std::vector<double> row_delta;   // [上排增量, 下排增量]（cm，可缺省）
  double y_offset = 0.0;           // 该面所有点位沿 y 的整体偏移（cm）
  // 该面的悬停距离修正（cm）：负数 = 比全局 standoff_cm 更贴近板面。
  // 现场若某一面贴得更远/更近需要单独补偿，就填这里，不影响其它面。
  double standoff_delta = 0.0;
};

struct SlotInfo
{
  std::string id;       // A1 ~ D6
  std::string face;     // A / B / C / D
  std::string rack;     // S1 / S2
  int row = 1;          // 1 = 上排(140cm)，2 = 下排(100cm)
  int col = 1;          // 1 = 左列
  int index = 1;        // 面内序号 1~6
  Vec3 qr;              // 二维码中心（场地坐标，cm）
  Vec3 hover;           // 无人机悬停点（场地坐标，cm）
  // 飞行目标 = hover + 现场过冲（field.json 的 slot_fly_overshoot_cm）。
  // 只影响"飞机往哪飞"，不影响点位坐标、上报的货物坐标、以及按 hover_y 分列。
  // 现场发现某些航段末端飞机提前停住（差几厘米到不了码）时用它补。
  Vec3 fly_hover;       // 无人机实际飞向的目标（场地坐标，cm）
  double yaw = 0.0;     // 机头朝向（度，指向板面）
  Vec2 normal;          // 该面法线
};

struct Speeds
{
  double horizontal = 100.0;  // cm/s
  double vertical = 50.0;     // cm/s
  double yaw = 90.0;          // deg/s
  double land = 30.0;         // cm/s
};

struct Timing
{
  double climb_s = 4.0;
  double settle_s = 1.5;
  double scan_s = 1.2;
  double land_s = 6.0;
};

class FieldGeometry
{
public:
  // 从与地面站同格式的 field.json 载入（init 时调用一次）
  bool loadFile(const std::string & path, std::string & error);
  bool loadText(const std::string & text, std::string & error);

  const std::vector<SlotInfo> & slots() const { return slots_; }
  const SlotInfo * slot(const std::string & id) const;
  SlotInfo * mutableSlot(const std::string & id);

  // 按 A1..A6,B1..B6,C1..C6,D1..D6 的顺序返回点位编号
  std::vector<std::string> slotIdsInOrder() const;

  // 按实测坐标反查最近的坐标编号（只上报 x/y/z 时使用）
  std::string nearestSlot(double x, double y, double z) const;

  // 点位所在通道（按货架板面分隔出的条带）：band0 / band1 / band2
  std::string aisleOf(const SlotInfo & slot) const;

  Rect inflated(const Rect & rect) const;
  bool pointFree(double x, double y) const;
  bool segmentFree(double x0, double y0, double x1, double y1, int samples = 40) const;

  // 硬安全包络：把任何落进货架板面 min_clearance 以内的目标点推出去。
  // 这是飞行时的最后一道保险——一旦坐标系/定位出问题，宁可不飞过去也不撞货架。
  // 返回是否发生过修正。
  Vec2 clampToSafeStandoff(const Vec2 & point, double min_clearance, bool * clamped = nullptr) const;

  const std::vector<Rect> & obstacleRects() const { return obstacles_; }
  const std::vector<FaceInfo> & faces() const { return faces_; }
  // 场地配置的一致性提示（例如"同一列上下两排的悬停 y 差了几厘米"）。
  // 只做提示、不影响任何计算；任务节点启动时会把它们打到日志里。
  const std::vector<std::string> & warnings() const { return warnings_; }

  // 场地参数
  std::string name;
  double width = 500.0;
  double height = 400.0;
  double wall_margin = 5.0;
  double safety_margin = 30.0;
  double standoff = 55.0;
  // 现场实测的悬停点 (x,y) 覆盖（field.json 的 slot_hover_xy_cm）：
  // 键是点位编号（A1~D6），值是 [x, y]。举着飞机让激光打到二维码中心、
  // 读地面站上飞机当时的位置，得到的就是这个。
  // 模型（行高/列位/悬停距离）算不准时用它直接顶替——现场贴码不规整，
  // 靠调 standoff / y_offset 是凑不出来的（每列偏差都不一样）。
  std::map<std::string, Vec2> slot_hover_xy;
  // 同样的键，可选的高度覆盖（field.json 里写成 [x, y, z] 三项时才有值）。
  // 现场发现某个点位的下降没走够、需要单独往下压一点时用它。
  std::map<std::string, double> slot_hover_z;
  // 飞行过冲（field.json 的 slot_fly_overshoot_cm）：只加在 fly_hover 上
  std::map<std::string, Vec3> slot_fly_overshoot;
  double cruise_alt = 150.0;
  // 换面缓冲点 + 纵向扫描的高度范围（与地面站 field.json 同一份）
  Vec2 buffer;
  double buffer_yaw = 90.0;
  double vscan_top = 150.0;
  double vscan_floor = 70.0;
  double vscan_speed = 10.0;     // 纵向扫描下降速度（cm/s，仅用于估算用时）
  double laser_dwell_s = 0.5;
  double rack_length = 200.0;
  double rack_height = 190.0;
  double board_bottom = 60.0;
  double aisle_switch_penalty_cm = 120.0;
  Vec2 takeoff;
  double takeoff_yaw = 90.0;
  Vec2 landing;
  double landing_yaw = 90.0;
  double landing_radius = 30.0;
  std::vector<double> rows;
  std::vector<double> cols_offset;
  Speeds speeds;
  Timing timing;
  double time_limit_sweep_s = 270.0;
  double time_limit_target_s = 180.0;

private:
  void build();
  void buildFaceSlots(const FaceInfo & face);
  void buildWarnings();
  std::vector<FaceInfo> faces_;
  std::vector<SlotInfo> slots_;
  std::vector<Rect> obstacles_;
  std::vector<Rect> rack_rects_;
  std::vector<double> band_planes_;
  bool band_axis_x_ = true;
  std::vector<std::string> warnings_;
};

}  // namespace shelf_scan
