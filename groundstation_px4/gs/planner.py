"""航线规划：遍历盘点航线 + 指定货物定点盘点航线。

规划思路
    1) 用可见图(visibility graph) + Dijkstra 求两点间避开货架的最短路径；
    2) 用最近邻 + 2-opt 求 24 个盘点的访问顺序（带换通道惩罚，使航线按面连续）；
    3) 输出结构化 legs（起飞 / 巡航 / 识别 / 返航 / 降落），供地面站绘图、
       无人机执行、以及用时估算(ETA)。
"""

from __future__ import annotations

import heapq
import math
import time
from typing import Any, Dict, List, Optional, Sequence, Tuple

Point = Tuple[float, float]


def _dist(a: Point, b: Point) -> float:
    return math.hypot(a[0] - b[0], a[1] - b[1])


class PathFinder:
    """基于可见图的最短避障路径。"""

    def __init__(self, field):
        self.field = field
        self._nodes: List[Point] = []
        for rect in field.obstacles:
            x0, y0, x1, y1 = field.inflate(rect)
            # 在膨胀矩形角点外再外扩的量（与机载端 route_planner.cpp 的
            # kEdgeOffset 必须一致）。2cm 表示绕行路线贴着配置的安全余量走。
            e = 2.0
            self._nodes += [
                (x0 - e, y0 - e), (x0 - e, y1 + e),
                (x1 + e, y0 - e), (x1 + e, y1 + e),
            ]
        self._cache: Dict[Tuple[Point, Point], List[Point]] = {}

    def path(self, a: Point, b: Point) -> List[Point]:
        key = (a, b)
        if key in self._cache:
            return self._cache[key]
        f = self.field
        if f.segment_free(a, b):
            out = [a, b]
        else:
            nodes = [a] + self._nodes + [b]
            n = len(nodes)
            adj: List[List[Tuple[int, float]]] = [[] for _ in range(n)]
            for i in range(n):
                for j in range(i + 1, n):
                    if f.segment_free(nodes[i], nodes[j], samples=30):
                        d = _dist(nodes[i], nodes[j])
                        adj[i].append((j, d))
                        adj[j].append((i, d))
            dist = [float("inf")] * n
            prev = [-1] * n
            dist[0] = 0.0
            pq = [(0.0, 0)]
            while pq:
                d, u = heapq.heappop(pq)
                if d > dist[u] + 1e-9:
                    continue
                if u == n - 1:
                    break
                for v, w in adj[u]:
                    if d + w < dist[v] - 1e-9:
                        dist[v] = d + w
                        prev[v] = u
                        heapq.heappush(pq, (dist[v], v))
            if dist[n - 1] == float("inf"):
                out = [a, b]  # 退化处理：直达
            else:
                chain = []
                cur = n - 1
                while cur != -1:
                    chain.append(nodes[cur])
                    cur = prev[cur]
                out = list(reversed(chain))
        out = _simplify(self.field, out)
        self._cache[key] = out
        return out


def _simplify(field, pts: List[Point]) -> List[Point]:
    """去掉共线/可直达的中间点。"""
    if len(pts) <= 2:
        return pts
    out = [pts[0]]
    i = 0
    while i < len(pts) - 1:
        j = len(pts) - 1
        while j > i + 1 and not field.segment_free(pts[i], pts[j], samples=40):
            j -= 1
        out.append(pts[j])
        i = j
    return out


def _eta(field, legs: Sequence[Dict[str, Any]]) -> float:
    sp = field.cfg.get("speeds", {})
    vh = float(sp.get("horizontal", 100.0)) or 100.0
    vv = float(sp.get("vertical", 50.0)) or 50.0
    vy = float(sp.get("yaw", 90.0)) or 90.0
    vs = float(field.cfg.get("vscan_speed_cm_s", 10.0)) or 10.0
    tg = field.cfg.get("timing", {})
    settle = float(tg.get("settle_s", 1.5))
    scan = float(tg.get("scan_s", 1.2))
    climb = float(tg.get("climb_s", 4.0))
    land = float(tg.get("land_s", 6.0))
    total = 0.0
    last_yaw = None
    for leg in legs:
        kind = leg["type"]
        if kind in ("takeoff", "land"):
            total += climb if kind == "takeoff" else land
            last_yaw = leg.get("yaw", last_yaw)
            continue
        pts = leg.get("points") or [leg.get("hover")]
        # 纵向扫描的下降是慢速扫描，不能按常规垂直速度算
        vdown = vs if kind == "vscan" else vv
        for k in range(1, len(pts)):
            p, q = pts[k - 1], pts[k]
            total += _dist((p[0], p[1]), (q[0], q[1])) / vh
            total += abs(float(q[2]) - float(p[2])) / vdown
        if kind == "scan":
            total += settle + scan
            yaw = leg.get("yaw")
            if yaw is not None and last_yaw is not None:
                delta = abs((yaw - last_yaw + 180.0) % 360.0 - 180.0)
                total += delta / vy
            if yaw is not None:
                last_yaw = yaw
        elif kind == "vscan":
            # 每列上下两排各停留一次（识别 + 记录）
            total += 2.0 * (settle + scan)
            yaw = leg.get("yaw")
            if yaw is not None and last_yaw is not None:
                delta = abs((yaw - last_yaw + 180.0) % 360.0 - 180.0)
                total += delta / vy
            if yaw is not None:
                last_yaw = yaw
    return round(total, 1)


def _leg_transit(field, pf: PathFinder, cur_z: float, target: Dict[str, Any]):
    a = (target["prev"][0], target["prev"][1])
    # 转场段的终点用 fly_hover（= 悬停点 + 现场过冲）：飞机实际要飞到的是这里
    b = (target.get("fly_hover_x", target["hover_x"]),
         target.get("fly_hover_y", target["hover_y"]))
    pts = pf.path(a, b)
    z0, z1 = cur_z, target.get("fly_hover_z", target["hover_z"])
    n = max(len(pts) - 1, 1)
    path3 = []
    for i, p in enumerate(pts):
        t = i / n
        path3.append([round(p[0], 2), round(p[1], 2),
                      round(z0 + (z1 - z0) * t, 2)])
    return {"type": "transit", "points": path3,
            "slot": target["slot"], "label": f"飞往 {target['slot']}"}


def _leg_transit_to(field, pf: PathFinder, cur_z: float, cur_xy: Point,
                    dest: Sequence[float], label: str, slot=None) -> Dict[str, Any]:
    """从 cur_xy@cur_z 飞到 dest=(x,y,z)，途中绕开货架。"""
    pts = pf.path((cur_xy[0], cur_xy[1]), (dest[0], dest[1]))
    z0, z1 = float(cur_z), float(dest[2])
    n = max(len(pts) - 1, 1)
    path3 = [[round(p[0], 2), round(p[1], 2), round(z0 + (z1 - z0) * i / n, 2)]
             for i, p in enumerate(pts)]
    return {"type": "transit", "points": path3, "slot": slot, "label": label}


def _leg_climb(x: float, y: float, z_from: float, z_to: float,
               label: str) -> Dict[str, Any]:
    """同一列上原地升降（长城状路线里的"竖边"）。"""
    return {"type": "transit", "slot": None, "label": label,
            "points": [[round(x, 2), round(y, 2), round(z_from, 2)],
                       [round(x, 2), round(y, 2), round(z_to, 2)]]}


def _leg_vscan(field, face_id: str, column_index: int, col: Dict[str, Any],
               order: int) -> Dict[str, Any]:
    """一列的纵向扫描段：从 vscan_top 缓慢降到 vscan_floor，边降边识别。"""
    top_z = float(field.vscan_top)
    floor_z = float(field.vscan_floor)
    x, y = float(col["x"]), float(col["y"])
    return {
        "type": "vscan",
        "face": face_id,
        "column": column_index,
        "order": order,
        "slots": [col["top"]["slot"], col["bottom"]["slot"]],
        "rows": [float(col["top"]["hover_z"]), float(col["bottom"]["hover_z"])],
        "points": [[x, y, top_z], [x, y, floor_z]],
        "hover": [x, y, top_z],
        "yaw": float(col["yaw"]),
        "laser_s": float(field.cfg.get("laser_dwell_s", 0.5)),
        "label": (f"纵向扫描 {face_id}面第{column_index}列"
                  f"（{col['top']['slot']}/{col['bottom']['slot']}）"),
    }


def face_columns(field, face_id: str) -> List[Dict[str, Any]]:
    """把某个面的点位按"列"分组：同一 **列号**（``col``，1/2/3）上的上下两个点为一列。

    2026-09-27：以前按"hover_y 完全相等"配对。但悬停点是现场"举着飞机让激光打到
    二维码中心"实测出来的，同一列上下两排高度不同、激光落点会差几厘米，于是
    A6(154) 与 A3(157)、B6(142) 与 B3(146) 不再相等，那一列被拆成两个"只有 1 个点"
    的列（top == bottom），`_serpentine_order()` 的"上排一趟 + 下排折返"就把这两个
    点各排了两次 —— 航线上多出 4 个重复点位。现在按列号配对：上下两排的 y 可以
    各自不同，列顺序按该列的平均 y 从小到大。

    返回按 y 从小到大排序，每项 {'x','col','y','yaw','top','bottom'}，top 是高的那个。
    """
    grouped: Dict[Tuple[str, int], List[Dict[str, Any]]] = {}
    for s in field.all_slots():
        if s["face"] != face_id:
            continue
        grouped.setdefault((str(s.get("rack", "")), int(s["col"])), []).append(s)
    columns: List[Dict[str, Any]] = []
    for (_rack, col_index), members in grouped.items():
        # 高的那排当 top；高度相同时按编号定序，保证与机载端 C++ 结果一致
        members = sorted(members, key=lambda s: (-float(s["hover_z"]), str(s["slot"])))
        columns.append({
            "x": float(members[0]["hover_x"]),
            "col": col_index,
            "y": sum(float(s["hover_y"]) for s in members) / len(members),
            "yaw": float(members[0]["yaw"]),
            "top": members[0],
            "bottom": members[-1],
        })
    columns.sort(key=lambda c: (c["y"], c["col"]))
    return columns


def plan_sweep(field, order: Optional[Sequence[str]] = None,
               slots: Optional[Sequence[str]] = None) -> Dict[str, Any]:
    """遍历盘点航线（今天以前的老逻辑）：一串悬停点组成的链。

    顺序固定为"每个面先从上排一端扫到另一端、再折返扫下排"：
        A 面 A3 → A2 → A1 → A4 → A5 → A6
        B 面 B1 → B2 → B3 → B6 → B5 → B4
    然后返航降落。某个点没扫到时，机载端会自己沿板面法线做微调重试。
    """
    if not order and not slots:
        order = _serpentine_order(field)
    return _plan_pointwise(field, order, slots)


def _serpentine_order(field) -> List[str]:
    """每个面：上排按 y 从小到大，然后折返按下排 y 从大到小。"""
    ids: List[str] = []
    for face in field.faces:
        columns = face_columns(field, face["id"])
        ids += [c["top"]["slot"] for c in columns]
        ids += [c["bottom"]["slot"] for c in reversed(columns)]
    return ids


def _plan_pointwise(field, order: Optional[Sequence[str]],
                    slots: Optional[Sequence[str]]) -> Dict[str, Any]:
    """逐点悬停扫描（老的逻辑），仅在显式指定点位时使用。"""
    pf = PathFinder(field)
    slot_ids = [s["slot"] for s in field.all_slots()]
    if slots:
        slot_ids = [s for s in slot_ids if s in set(slots)]
    targets = [field.slot(s) for s in (order or _order_slots(field, slot_ids))]

    legs: List[Dict[str, Any]] = []
    tf = field.takeoff
    legs.append({
        "type": "takeoff",
        "points": [[tf["x"], tf["y"], 0.0], [tf["x"], tf["y"], field.cruise_alt]],
        "yaw": tf.get("yaw", 90.0), "slot": None,
        "label": f"起飞上升至 {field.cruise_alt:.0f}cm",
    })
    prev: Point = (tf["x"], tf["y"])
    cur_z = field.cruise_alt
    for idx, target in enumerate(targets, start=1):
        tgt = dict(target)
        tgt["prev"] = prev
        tgt["order"] = idx
        legs.append(_leg_transit(field, pf, cur_z, tgt))
        legs.append({
            "type": "scan", "slot": target["slot"], "order": idx,
            "points": [[target.get("fly_hover_x", target["hover_x"]),
                        target.get("fly_hover_y", target["hover_y"]),
                        target.get("fly_hover_z", target["hover_z"])]],
            "hover": [target["hover_x"], target["hover_y"], target["hover_z"]],
            "qr": [target["x"], target["y"]],
            "yaw": target["yaw"],
            "laser_s": float(field.cfg.get("laser_dwell_s", 0.5)),
            "label": f"盘点 {target['slot']}",
        })
        prev = (target.get("fly_hover_x", target["hover_x"]),
                target.get("fly_hover_y", target["hover_y"]))
        cur_z = target.get("fly_hover_z", target["hover_z"])

    land = field.landing
    pts = pf.path(prev, (land["x"], land["y"]))
    path3 = [[round(p[0], 2), round(p[1], 2), round(cur_z, 2)] for p in pts]
    legs.append({"type": "return", "points": path3, "slot": None, "label": "返航"})
    legs.append({
        "type": "land",
        "points": [[land["x"], land["y"], cur_z], [land["x"], land["y"], 0.0]],
        "yaw": land.get("yaw", 90.0), "slot": None, "label": "降落",
    })
    return _finalize(field, legs, "sweep",
                     f"遍历盘点航线（{len(targets)} 个点）")


def _plan_direct(field, slot_id: str, target_no: Optional[int],
                 measured_z: Optional[float] = None) -> Dict[str, Any]:
    """已知货物位置：直接飞到该点位悬停扫描。

    measured_z 为遍历盘点时实测到的高度（纵向扫描记下来的），
    有就用实测值，没有就退回模型行高。
    """
    pf = PathFinder(field)
    info = field.slot(slot_id)
    z = float(measured_z) if measured_z is not None else float(info["hover_z"])
    hx, hy = float(info["hover_x"]), float(info["hover_y"])
    tf = field.takeoff
    legs: List[Dict[str, Any]] = [{
        "type": "takeoff",
        "points": [[tf["x"], tf["y"], 0.0], [tf["x"], tf["y"], field.cruise_alt]],
        "yaw": tf.get("yaw", 90.0), "slot": None,
        "label": f"起飞上升至 {field.cruise_alt:.0f}cm",
    }]
    legs.append(_leg_transit_to(field, pf, field.cruise_alt,
                                (float(tf["x"]), float(tf["y"])),
                                (hx, hy, z), f"直达 {slot_id}"))
    legs.append({
        "type": "scan", "slot": slot_id, "order": 1,
        "points": [[hx, hy, z]],
        "hover": [hx, hy, z],
        "qr": [info["x"], info["y"]],
        "yaw": info["yaw"],
        "laser_s": float(field.cfg.get("laser_dwell_s", 0.5)),
        "label": f"盘点 {slot_id}",
    })
    land = field.landing
    pts = pf.path((hx, hy), (land["x"], land["y"]))
    legs.append({"type": "return", "slot": None, "label": "返航",
                 "points": [[round(p[0], 2), round(p[1], 2), round(z, 2)]
                            for p in pts]})
    legs.append({
        "type": "land",
        "points": [[land["x"], land["y"], z], [land["x"], land["y"], 0.0]],
        "yaw": land.get("yaw", 90.0), "slot": None, "label": "降落",
    })
    route = _finalize(field, legs, "target",
                      f"定向盘点航线：货物 {target_no} -> {slot_id}（直达"
                      + (f"，实测高度 {z:.0f}cm" if measured_z is not None else "") + "）")
    route["target_no"] = target_no
    route["target_slot"] = slot_id
    route["mode"] = "direct"
    return route


def plan_target(field, target_no: Optional[int], mapping: Dict[str, int],
                search_all: bool = False,
                measured_z: Optional[float] = None) -> Dict[str, Any]:
    """定点盘点航线。mapping 为已识别的 坐标->编号 表。

    已知目标位置：直达该点后返航（优先用遍历时实测的高度）；
    未知目标位置：生成按最近邻优化的搜索航线（到点即识别，命中后返航）。
    """
    slot_id = None
    if target_no is not None:
        for slot, no in mapping.items():
            if int(no) == int(target_no):
                slot_id = slot
                break
    if slot_id and not search_all:
        return _plan_direct(field, slot_id, target_no, measured_z)

    route = plan_sweep(field)
    route["kind"] = "target"
    route["mode"] = "search"
    route["target_no"] = target_no
    route["target_slot"] = None
    route["title"] = (f"定向盘点航线：货物 {target_no} 位置未知，"
                      f"按优化搜索航线逐点识别" if target_no is not None
                      else "遍历盘点航线")
    return route


def _order_slots(field, slot_ids: Sequence[str]) -> List[str]:
    """最近邻 + 2-opt 排序，带换通道惩罚。"""
    slots = [field.slot(s) for s in slot_ids]
    pen = float(field.cfg.get("aisle_switch_penalty_cm", 120.0))
    sp = field.cfg.get("speeds", {})
    # 竖直方向与水平方向的耗时等价系数：上下 40cm 相当于水平 80cm
    zw = float(sp.get("horizontal", 100.0)) / max(1e-6, float(sp.get("vertical", 50.0)))
    start = (field.takeoff["x"], field.takeoff["y"])

    def point_of(s):
        return (s["hover_x"], s["hover_y"])

    def cost_pts(a, b) -> float:
        d = math.hypot(a[0] - b[0], a[1] - b[1]) + zw * abs(a[2] - b[2])
        return d

    def cost(a, b) -> float:
        d = cost_pts((a["hover_x"], a["hover_y"], a.get("hover_z", field.cruise_alt)),
                     (b["hover_x"], b["hover_y"], b.get("hover_z", field.cruise_alt)))
        if field.aisle_of(a) != field.aisle_of(b):
            d += pen
        return d

    remaining = list(slots)
    order: List[Dict[str, Any]] = []
    cur = {"hover_x": start[0], "hover_y": start[1], "hover_z": field.cruise_alt,
           "face": None, "slot": "TAKEOFF"}
    while remaining:
        nxt = min(remaining, key=lambda s: cost(cur, s))
        remaining.remove(nxt)
        order.append(nxt)
        cur = nxt

    # 2-opt 改进（含返航点到降落点的收敛项）
    land = {"hover_x": field.landing["x"], "hover_y": field.landing["y"],
            "hover_z": 0.0, "face": None, "slot": "LAND"}

    def total(seq) -> float:
        t = 0.0
        prev = {"hover_x": start[0], "hover_y": start[1],
                "hover_z": field.cruise_alt, "face": None, "slot": "TAKEOFF"}
        for s in seq:
            t += cost(prev, s)
            prev = s
        t += cost(prev, land)
        return t

    improved = True
    best = total(order)
    while improved:
        improved = False
        for i in range(len(order) - 1):
            for j in range(i + 1, len(order)):
                cand = order[:i] + order[i:j + 1][::-1] + order[j + 1:]
                val = total(cand)
                if val + 1e-6 < best:
                    order, best, improved = cand, val, True
    return [s["slot"] for s in order]


def _finalize(field, legs: List[Dict[str, Any]], kind: str,
              title: str) -> Dict[str, Any]:
    polyline: List[List[float]] = []
    for leg in legs:
        for p in leg.get("points", []):
            if not polyline or (abs(polyline[-1][0] - p[0]) > 1e-6 or
                                abs(polyline[-1][1] - p[1]) > 1e-6):
                polyline.append([p[0], p[1]])
    waypoints: List[Dict[str, Any]] = []
    for leg in legs:
        if leg["type"] == "scan":
            waypoints.append({
                "order": leg.get("order"), "slot": leg["slot"],
                "x": leg["hover"][0], "y": leg["hover"][1], "z": leg["hover"][2],
                "yaw": leg.get("yaw"), "qr": leg.get("qr"),
            })
        elif leg["type"] == "vscan":
            # 一列 = 上下两排两个点，都在同一个悬停 x/y 上，高度不同
            for sid, rz in zip(leg["slots"], leg["rows"]):
                info = field.slot(sid)
                waypoints.append({
                    "order": leg.get("order"), "slot": sid,
                    "x": info["hover_x"], "y": info["hover_y"], "z": rz,
                    "yaw": leg.get("yaw"), "qr": [info["x"], info["y"]],
                })
    length = 0.0
    for leg in legs:
        pts = leg.get("points") or []
        for k in range(1, len(pts)):
            length += _dist((pts[k - 1][0], pts[k - 1][1]),
                            (pts[k][0], pts[k][1]))
    return {
        "kind": kind,
        "title": title,
        "created": time.time(),
        "legs": legs,
        "waypoints": waypoints,
        "polyline": polyline,
        "length_cm": round(length, 1),
        "eta_s": _eta(field, legs),
    }
