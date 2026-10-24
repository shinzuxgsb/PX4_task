"""内置虚拟无人机：没有真机时驱动整套地面站流程与演示。

它按地面站下发的航线飞行，并在到达盘点点时点亮激光、上报识别结果；
上报的消息格式与真机完全一致（见 PROTOCOL.md），因此地面站无需区分。
"""

from __future__ import annotations

import math
import random
import threading
import time
from typing import Any, Dict, List, Optional

from . import config as cfgmod


class VirtualDrone:
    def __init__(self, field, publish, ground_truth: Dict[str, int],
                 time_scale: float = 1.0, seed: int = 20240801):
        self.field = field
        self.publish = publish          # 回调：publish(dict)
        self.gt = ground_truth          # 坐标 -> 货物编号（真值）
        self.time_scale = max(0.1, float(time_scale))
        self.rng = random.Random(seed)
        self.lock = threading.RLock()
        self.thread: Optional[threading.Thread] = None
        self.stop_flag = threading.Event()

        self.x = float(field.takeoff["x"])
        self.y = float(field.takeoff["y"])
        self.z = 0.0
        self.yaw = float(field.takeoff.get("yaw", 90.0))
        self.batt = 100.0
        self.armed = False
        self.laser = False
        self.state = "idle"
        self.route: Optional[Dict[str, Any]] = None
        self.leg_index = 0
        self.seg_index = 0
        self.dwell_until = 0.0
        self.scan_done_leg = -1
        self.vscan_done: set = set()
        self.vscan_leg = -1
        self.target_no: Optional[int] = None
        self.t0 = time.time()
        self.last_tick = time.time()
        self.last_pub = 0.0

    # ------------------------------------------------------------- lifecycle
    def start(self) -> None:
        if self.thread and self.thread.is_alive():
            return
        self.stop_flag.clear()
        self.publish({"type": "hello", "device": "virtual-drone",
                      "fw": "mock-1.0", "source": "mock"})
        self.thread = threading.Thread(target=self._loop, name="virtual-drone",
                                       daemon=True)
        self.thread.start()

    def stop(self) -> None:
        self.stop_flag.set()

    # ---------------------------------------------------------------- commands
    def set_route(self, route: Dict[str, Any], target_no: Optional[int] = None) -> None:
        with self.lock:
            self.route = route
            self.target_no = target_no
            self.leg_index = 0
            self.seg_index = 0
            self.scan_done_leg = -1
            self.vscan_done = set()
            self.vscan_leg = -1
            self.dwell_until = 0.0
            self.armed = True
            self.state = "flying"
            self.t0 = time.time()

    def land_now(self) -> None:
        with self.lock:
            self.route = None
            self.state = "landing"
            self.armed = False
            self.laser = False

    def emergency_stop(self) -> None:
        # 与机载端语义一致：急停 = 就地悬停（保留当前高度，只是不再沿航线飞）
        with self.lock:
            self.route = None
            self.state = "estop"
            self.armed = False
            self.laser = False

    def reset(self) -> None:
        """清空数据：把虚拟无人机收回起飞点，方便立刻演示下一轮。

        必须同时重置位置和航线状态——只清 GroundState 的话，虚拟无人机
        下一帧遥测（100ms 后）就会把界面上的位置又拉回原来的地方。
        """
        with self.lock:
            self.route = None
            self.leg_index = 0
            self.seg_index = 0
            self.dwell_until = 0.0
            self.scan_done_leg = -1
            self.target_no = None
            self.x = float(self.field.takeoff["x"])
            self.y = float(self.field.takeoff["y"])
            self.z = 0.0
            self.yaw = float(self.field.takeoff.get("yaw", 90.0))
            self.armed = False
            self.laser = False
            self.state = "idle"
            self.t0 = time.time()
            self.last_pub = 0.0        # 下一拍立刻补发一帧遥测
        self.publish(self.telemetry())

    def telemetry(self) -> Dict[str, Any]:
        with self.lock:
            return {
                "type": "telemetry", "source": "mock",
                "t": round(time.time() - self.t0, 2),
                "pos": {"x": round(self.x, 1), "y": round(self.y, 1),
                        "z": round(self.z, 1)},
                "yaw": round(self.yaw, 1),
                "batt": round(self.batt, 1),
                "armed": self.armed,
                "laser": self.laser,
                "state": self.state,
                "slot": self._current_slot(),
            }

    def _current_slot(self) -> Optional[str]:
        if not self.route:
            return None
        if self.leg_index < len(self.route["legs"]):
            return self.route["legs"][self.leg_index].get("slot")
        return None

    # ------------------------------------------------------------------- loop
    def _loop(self) -> None:
        while not self.stop_flag.is_set():
            now = time.time()
            dt = min(0.2, now - self.last_tick) * self.time_scale
            self.last_tick = now
            with self.lock:
                self._step(dt)
            if now - self.last_pub >= 0.1:
                self.last_pub = now
                self.publish(self.telemetry())
            time.sleep(0.05)
        self.publish({"type": "bye", "source": "mock"})

    def _step(self, dt: float) -> None:
        sp = self.field.cfg.get("speeds", {})
        vh = float(sp.get("horizontal", 100.0))
        vv = float(sp.get("vertical", 50.0))
        vy = float(sp.get("yaw", 90.0))
        self.batt = max(5.0, self.batt - dt * 0.05)
        if self.route is None:
            if self.state == "landing" and self.z > 0:
                self.z = max(0.0, self.z - vv * dt)
            return
        legs = self.route["legs"]
        if self.leg_index >= len(legs):
            self.route = None
            self.state = "landed" if self.z <= 0.5 else "hover"
            self.armed = False
            self.publish({
                "type": "px4_status", "state": "FINISHED",
                "mode": "OFFBOARD", "armed": False, "ekf_ok": True,
                "link_ok": True, "ready": 0, "mission_active": False,
                "takeoff_available": False,
            })
            self.publish({"type": "mission", "phase": "done",
                          "source": "mock"})
            return

        leg = legs[self.leg_index]
        kind = leg["type"]
        pts = leg.get("points") or []

        if kind == "vscan":
            # 纵向扫描：边降边看，降到某一排的高度时就识别那一排
            if self.vscan_leg != self.leg_index:
                self.vscan_leg = self.leg_index
                self.vscan_done = set()
            for sid, rz in zip(leg.get("slots", []), leg.get("rows", [])):
                if sid in self.vscan_done:
                    continue
                if self.z <= float(rz) + 2.0:
                    self.vscan_done.add(sid)
                    self.laser = True
                    self.state = "scanning"
                    self._emit_detect({"slot": sid,
                                       "laser_s": float(leg.get("laser_s", 0.5))})
                    self.laser = False

        if kind == "scan" and self.dwell_until == 0.0 and self.seg_index == 0:
            # 到位：点亮激光并识别
            self.laser = True
            self.state = "scanning"
            self.dwell_until = time.time() + float(leg.get("laser_s", 0.5)) \
                / self.time_scale
            self._emit_detect(leg)

        if self.dwell_until:
            self.laser = True
            if time.time() >= self.dwell_until:
                self.laser = False
                self.dwell_until = 0.0
                self.leg_index += 1
                self.seg_index = 0
                self.state = "flying"
            return

        if self.seg_index >= len(pts) - 1:
            self.leg_index += 1
            self.seg_index = 0
            if self.leg_index < len(legs) and legs[self.leg_index]["type"] == "scan":
                pass
            return

        target = pts[self.seg_index + 1]
        tx, ty, tz = float(target[0]), float(target[1]), float(target[2])
        dx, dy, dz = tx - self.x, ty - self.y, tz - self.z
        dh = math.hypot(dx, dy)
        step_h = vh * dt
        step_v = vv * dt
        if dh > 1e-6:
            k = min(1.0, step_h / dh)
            self.x += dx * k
            self.y += dy * k
            desired = math.degrees(math.atan2(dy, dx))
            self._turn_to(desired, vy * dt)
        if abs(dz) > 1e-6:
            kz = min(1.0, step_v / abs(dz))
            self.z += dz * kz
        if dh <= 1.0 and abs(dz) < 1.0:
            self.seg_index += 1

    def _turn_to(self, desired: float, max_step: float) -> None:
        delta = (desired - self.yaw + 180.0) % 360.0 - 180.0
        self.yaw = (self.yaw + max(-max_step, min(max_step, delta))) % 360.0

    def _emit_detect(self, leg: Dict[str, Any]) -> None:
        slot = leg["slot"]
        no = self.gt.get(slot)
        info = self.field.slot(slot)
        if no is None:
            self.publish({"type": "detect", "no": None, "slot": slot,
                          "ok": False, "reason": "no_qr", "source": "mock"})
            return
        self.publish({
            "type": "detect", "no": int(no), "slot": slot, "ok": True,
            "x": info["x"], "y": info["y"], "z": info["z"],
            "conf": round(0.93 + 0.06 * self.rng.random(), 3),
            "source": "mock",
        })
        if self.target_no is not None and int(no) == int(self.target_no):
            self.publish({"type": "target_hit", "no": int(no), "slot": slot,
                          "source": "mock"})
            # 命中目标后直接切到返航段
            for i, lg in enumerate(self.route["legs"]):
                if lg["type"] in ("return", "land"):
                    self.leg_index = i
                    self.seg_index = 0
                    break


def make_ground_truth(field, seed: int = 20240801) -> Dict[str, int]:
    """生成（或复现）二维码张贴位置的随机真值：坐标 -> 货物编号。"""
    path = None
    try:
        import json
        import os
        path = os.path.join(cfgmod.DATA_DIR, "ground_truth.json")
        if os.path.exists(path):
            with open(path, "r", encoding="utf-8") as fh:
                data = json.load(fh)
            slots = {s["slot"] for s in field.all_slots()}
            if set(data.keys()) == slots:
                return {k: int(v) for k, v in data.items()}
    except Exception:
        data = None
    rng = random.Random(seed)
    slots = [s["slot"] for s in field.all_slots()]
    # 货物编号就是 1..点位数（点位减少时自动跟着变，不要写死 24）
    numbers = list(range(1, len(slots) + 1))
    rng.shuffle(numbers)
    gt = {slot: numbers[i % len(numbers)] for i, slot in enumerate(slots)}
    try:
        import json
        import os
        if path:
            os.makedirs(os.path.dirname(path), exist_ok=True)
            with open(path, "w", encoding="utf-8") as fh:
                json.dump(gt, fh, ensure_ascii=False, indent=2)
    except Exception:
        pass
    return gt
