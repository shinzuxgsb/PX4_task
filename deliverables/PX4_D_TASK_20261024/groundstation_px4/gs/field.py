"""由场地配置推导出全部盘点点位的三维坐标、避障障碍与绘图信息。

场地模型（与题目图1 一致）：
    - 作业区 500cm(X) x 400cm(Y)，原点在左下角
    - 每个货架是一块竖放的板（沿 Y 方向，长 200cm，厚 5cm），两面各贴 6 个二维码
    - 每个面 2 行 x 3 列：行高 140cm / 100cm，列距 50cm
    - 编号：站在该面正前方，从左上角开始按行计数（A1 左上 ... A6 右下）
"""

from __future__ import annotations

import math
from typing import Any, Dict, List, Tuple

Point = Tuple[float, float]


def _norm(v) -> Point:
    x, y = float(v[0]), float(v[1])
    n = math.hypot(x, y) or 1.0
    return (x / n, y / n)


def _yaw_of(direction: Point) -> float:
    return math.degrees(math.atan2(direction[1], direction[0]))


class Field:
    """场地几何模型：点位查询、障碍物、通行判断、绘图数据。"""

    def __init__(self, cfg: Dict[str, Any]):
        self.cfg = cfg
        self.w = float(cfg["area"]["w"])
        self.h = float(cfg["area"]["h"])
        self.wall_margin = float(cfg["area"].get("wall_margin", 5.0))
        self.margin = float(cfg.get("safety_margin_cm", 30.0))
        self.takeoff = dict(cfg["takeoff"])
        self.landing = dict(cfg["landing"])
        self.standoff = float(cfg.get("standoff_cm", 55.0))
        # 现场实测的悬停点覆盖：{"A3": [100.0, 157.0], ...}
        # 举着飞机让激光打到二维码中心、读地面站上飞机当时的位置，得到的就是这个。
        # 模型算不准（现场贴码不规整、每列偏差都不一样）时直接顶替 (x,y)。
        self.slot_hover_xy = {
            str(k): (float(v[0]), float(v[1]))
            for k, v in (cfg.get("slot_hover_xy_cm", {}) or {}).items()
        }
        # 可选的第 3 项 = 该点位的高度覆盖（写 [x, y, z] 时生效）
        self.slot_hover_z = {
            str(k): float(v[2])
            for k, v in (cfg.get("slot_hover_xy_cm", {}) or {}).items()
            if len(v) >= 3
        }
        # 飞行过冲（只影响飞机往哪飞，不影响点位坐标与上报坐标）
        self.slot_fly_overshoot = {
            str(k): (float(v[0]), float(v[1]), float(v[2]))
            for k, v in (cfg.get("slot_fly_overshoot_cm", {}) or {}).items()
            if len(v) >= 3
        }
        self.cruise_alt = float(cfg.get("cruise_alt_cm", 150.0))
        # 换面前的缓冲点（纵向扫描路线用它绕开货架顶端）
        self.buffer = dict(cfg.get("buffer", {"x": 75.0, "y": 350.0, "yaw": 90.0}))
        # 纵向扫描：每列从 top 缓慢降到 floor，边降边识别；floor 是硬地板
        self.vscan_top = float(cfg.get("vscan_top_cm", self.cruise_alt))
        self.vscan_floor = float(cfg.get("vscan_floor_cm", 70.0))
        self.rows = [float(z) for z in cfg.get("slot_rows_cm", [])]
        self.cols_off = [float(v) for v in cfg.get("slot_cols_offset_cm", [])]
        self.per_face = len(self.rows) * len(self.cols_off)
        self.total_slots = 0

        self.faces: List[Dict[str, Any]] = []
        self.slots: Dict[str, Dict[str, Any]] = {}
        self.obstacles: List[Tuple[float, float, float, float]] = []
        self.racks: List[Dict[str, Any]] = []
        self._band_planes: List[float] = []
        self._band_axis = "x"
        self._build()
        # 场地配置的一致性提示（只提示、不影响计算），由 server 打到界面日志里。
        # 例："A 面第 3 列上下两排的悬停 y 相差 3.0cm（A3=157.0、A6=154.0）…"
        self.warnings: List[str] = self._build_warnings()

    # ---------------------------------------------------------------- build
    def _build(self) -> None:
        for rack in self.cfg.get("racks", []):
            axis = str(rack.get("axis", "y")).lower()
            th = float(rack.get("thickness", 5.0))
            if axis == "y":
                cx = float(rack["board_x"])
                y0, y1 = float(rack["y_from"]), float(rack["y_to"])
                cy = (y0 + y1) / 2.0
                length = y1 - y0
                u = (0.0, 1.0)
                rect = (cx - th / 2.0, y0, cx + th / 2.0, y1)
                self._band_axis = "x"
            else:
                cy = float(rack["board_y"])
                x0, x1 = float(rack["x_from"]), float(rack["x_to"])
                cx = (x0 + x1) / 2.0
                length = x1 - x0
                u = (1.0, 0.0)
                rect = (x0, cy - th / 2.0, x1, cy + th / 2.0)
                self._band_axis = "y"
            self.racks.append({
                "id": rack["id"], "axis": axis, "center": [cx, cy],
                "length": length, "thickness": th, "rect": list(rect),
            })
            self.obstacles.append(rect)
            self._band_planes.append(cx if axis == "y" else cy)
            for face in rack.get("faces", []):
                normal = _norm(face["normal"])
                self.faces.append({
                    "id": face["id"],
                    "rack": rack["id"],
                    "normal": normal,
                    "center": (cx, cy),
                    "u": u,
                    "thickness": th,
                    "length": length,
                    # 按面微调：行的额外高度增量（[上排, 下排]）、整面 y 偏移、
                    # 以及该面的悬停距离修正（负数 = 比全局 standoff_cm 更贴近板面）
                    "row_delta_cm": [float(v) for v in face.get("row_delta_cm", [])],
                    "y_offset_cm": float(face.get("y_offset_cm", 0.0)),
                    "standoff_delta_cm": float(face.get("standoff_delta_cm", 0.0)),
                })
        for face in self.faces:
            self._build_face_slots(face)
        self.total_slots = len(self.slots)

    def _build_warnings(self) -> List[str]:
        """同一列的上下两排悬停 y 差了多少（2026-09-27 起列的配对改成按列号，
        所以 y 允许不同 —— 这里只留个提示，免得"某一排 y 填错"看不出来）。"""
        groups: Dict[Tuple[str, int], List[Dict[str, Any]]] = {}
        for s in self.all_slots():
            groups.setdefault((str(s["face"]), int(s["col"])), []).append(s)
        notes: List[str] = []
        for (face_id, col_index), members in groups.items():
            if len(members) < 2:
                continue                        # 单点列（缺一个点）不提示
            ys = [float(s["hover_y"]) for s in members]
            spread = max(ys) - min(ys)
            if spread <= 1.0:
                continue                        # 1cm 以内属正常实测误差
            detail = "、".join(f"{s['slot']}={float(s['hover_y']):.1f}" for s in members)
            notes.append(
                f"{face_id} 面第 {col_index} 列上下两排的悬停 y 相差 {spread:.1f}cm"
                f"（{detail}）——列配对按列号，航线不受影响（没有重复点位）")
        return notes

    def _left_vector(self, normal: Point) -> Point:
        """站立在该面正前方时的“左手方向”（用于从左上角起编号）。"""
        nx, ny = normal
        # 视线方向 f = -normal，向上 u = +Z，右手 = f x u = (fy, -fx)
        return (ny, -nx)

    def _build_face_slots(self, face: Dict[str, Any]) -> None:
        nx, ny = face["normal"]
        cx, cy = face["center"]
        th = face["thickness"]
        lx, ly = self._left_vector(face["normal"])
        ncols = len(self.cols_off)
        # 列顺序：站在该面正前方，最左边的一列编号最小
        ux, uy = face["u"]
        proj = lx * ux + ly * uy
        offsets = sorted(self.cols_off, reverse=(proj > 0))
        row_delta = face.get("row_delta_cm", [])
        y_off = float(face.get("y_offset_cm", 0.0))
        # 该面的悬停距离：全局 standoff 加上本面修正量（可正可负），其它面不受影响
        so = self.standoff + float(face.get("standoff_delta_cm", 0.0))
        for r, z in enumerate(self.rows, start=1):            # 第 1 行 = 最上面一行
            row_z = z + (row_delta[r - 1] if r - 1 < len(row_delta) else 0.0)
            for c, off in enumerate(offsets, start=1):        # 第 1 列 = 最左一列
                idx = (r - 1) * ncols + c
                slot = f"{face['id']}{idx}"
                # off 是沿板长方向的偏移量
                qx = cx + nx * th / 2.0 + ux * off
                qy = cy + ny * th / 2.0 + uy * off + y_off
                hx = qx + nx * so
                hy = qy + ny * so
                # 实测覆盖优先：模型的行/列/悬停距离都可能差几厘米
                if slot in self.slot_hover_xy:
                    hx, hy = self.slot_hover_xy[slot]
                    # 实测 y 同时也是二维码自己的 y：激光打在码中心时飞机就在这个 y 上，
                    # 两者必须一致，否则现场编号与模型列序相反的那一面（B 面）上报的
                    # 货物坐标会是镜像的。
                    qy = hy
                # 高度：默认用本排的行高；写了实测覆盖（[x, y, z]）就只用该点位自己的值。
                # 2026-09-27：以前这里是"在列循环里改 row 级变量"，于是一个点位的高度
                # 覆盖会**泄漏给同一排后面的列**（A4 写 107 → A5/A6 也变 107），
                # 而机载端 C++ 是逐点设置的 —— 两端会不一致、shelf_selftest 直接挂。
                z_eff = self.slot_hover_z.get(slot, row_z)
                ov = self.slot_fly_overshoot.get(slot, (0.0, 0.0, 0.0))
                self.slots[slot] = {
                    "slot": slot,
                    "face": face["id"],
                    "rack": face["rack"],
                    "row": r,
                    "col": c,
                    "index": idx,
                    "x": round(qx, 2), "y": round(qy, 2), "z": round(z_eff, 2),
                    "hover_x": round(hx, 2), "hover_y": round(hy, 2),
                    "hover_z": round(z_eff, 2),
                    # 飞机实际要飞到的目标 = 悬停点 + 现场过冲
                    "fly_hover_x": round(hx + ov[0], 2),
                    "fly_hover_y": round(hy + ov[1], 2),
                    "fly_hover_z": round(z_eff + ov[2], 2),
                    "yaw": round(_yaw_of((-nx, -ny)), 2),
                    "normal": [nx, ny],
                }

    # ------------------------------------------------------------- geometry
    def inflate(self, rect: Tuple[float, float, float, float]):
        x0, y0, x1, y1 = rect
        m = self.margin
        return (x0 - m, y0 - m, x1 + m, y1 + m)

    def point_free(self, p: Point) -> bool:
        x, y = p
        if not (self.wall_margin <= x <= self.w - self.wall_margin):
            return False
        if not (self.wall_margin <= y <= self.h - self.wall_margin):
            return False
        for rect in self.obstacles:
            x0, y0, x1, y1 = self.inflate(rect)
            if x0 <= x <= x1 and y0 <= y <= y1:
                return False
        return True

    def segment_free(self, a: Point, b: Point, samples: int = 40) -> bool:
        if not self.point_free(a) or not self.point_free(b):
            return False
        for i in range(1, samples):
            t = i / samples
            if not self.point_free((a[0] + (b[0] - a[0]) * t,
                                    a[1] + (b[1] - a[1]) * t)):
                return False
        return True

    # ------------------------------------------------------------ accessors
    def slot(self, slot_id: str) -> Dict[str, Any]:
        return self.slots[slot_id]

    def all_slots(self) -> List[Dict[str, Any]]:
        return sorted(self.slots.values(),
                      key=lambda s: (s["face"], s["row"], s["col"]))

    def face_groups(self) -> Dict[str, List[Dict[str, Any]]]:
        groups: Dict[str, List[Dict[str, Any]]] = {}
        for face in self.faces:
            groups[face["id"]] = [s for s in self.all_slots()
                                  if s["face"] == face["id"]]
        return groups

    def aisle_of(self, slot: Dict[str, Any]) -> str:
        """点位所在通道（按货架板面分隔出的条带）。"""
        p = slot["hover_x"] if self._band_axis == "x" else slot["hover_y"]
        idx = sum(1 for v in sorted(self._band_planes) if p > v)
        return f"band{idx}"

    def slot_from_xyz(self, x: float, y: float, z: float) -> str:
        """按实测坐标反查最近的坐标编号（飞机只上报 x/y/z 时使用）。"""
        best, best_d = "", 1e9
        for s in self.slots.values():
            d = math.hypot(s["x"] - x, s["y"] - y) + abs(s["z"] - z) * 0.5
            if d < best_d:
                best, best_d = s["slot"], d
        return best

    # --------------------------------------------------------------- export
    def to_dict(self) -> Dict[str, Any]:
        faces = []
        for f in self.faces:
            nx, ny = f["normal"]
            cx, cy = f["center"]
            ux, uy = f["u"]
            half = f["length"] / 2.0
            off = f["thickness"] / 2.0
            faces.append({
                "id": f["id"], "rack": f["rack"],
                "normal": [nx, ny],
                "from": [round(cx + ux * half + nx * off, 2),
                         round(cy + uy * half + ny * off, 2)],
                "to": [round(cx - ux * half + nx * off, 2),
                       round(cy - uy * half + ny * off, 2)],
                "label": [round(cx + nx * 34.0, 2), round(cy + ny * 34.0, 2)],
            })
        return {
            "name": self.cfg.get("name", ""),
            "area": {"w": self.w, "h": self.h, "wall_margin": self.wall_margin},
            "takeoff": self.takeoff,
            "landing": self.landing,
            "racks": self.racks,
            "faces": faces,
            "slots": self.all_slots(),
            "rows": self.rows,
            "cols_off": self.cols_off,
            "rows_count": len(self.rows),
            "cols_count": len(self.cols_off),
            "per_face": self.per_face,
            "total_slots": self.total_slots,
            "cruise_alt": self.cruise_alt,
            "standoff_cm": self.standoff,
            "buffer": self.buffer,
            "vscan_top_cm": self.vscan_top,
            "vscan_floor_cm": self.vscan_floor,
            "safety_margin_cm": self.margin,
            "faces_order": [f["id"] for f in self.faces],
        }
