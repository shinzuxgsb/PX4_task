"""地面站状态机：汇总无人机上报信息、盘点结果、任务计时与查询。"""

from __future__ import annotations

import csv
import io
import json
import os
import threading
import time
from collections import deque
from typing import Any, Callable, Deque, Dict, List, Optional

from . import config as cfgmod
from .planner import plan_sweep, plan_target


class GroundState:
    def __init__(self, field, runtime: Dict[str, Any]):
        self.field = field
        self.runtime = runtime
        self.lock = threading.RLock()
        self.send: Callable[[Dict[str, Any]], None] = lambda _m: None
        self.on_led: Callable[[bool], None] = lambda _on: None

        self.reset_session(announce=False)
        self.link_status: Dict[str, Any] = {}
        self.started_at = time.time()
        self.hello_seen: Dict[str, float] = {}

    # ------------------------------------------------------------- sessions
    def reset_session(self, announce: bool = True) -> None:
        with self.lock:
            self.records: Dict[str, Dict[str, Any]] = {}   # slot -> record
            self.mapping: Dict[str, int] = {}              # slot -> 货物编号
            self.by_number: Dict[int, str] = {}            # 货物编号 -> slot
            self.heights: Dict[str, float] = {}            # slot -> 实测高度（纵向扫描记录）
            self.events: Deque[Dict[str, Any]] = deque(maxlen=500)
            self.detects: Deque[Dict[str, Any]] = deque(maxlen=200)
            self.led_seq = 0
            self.led_until = 0.0
            self.led_state = False
            self.query: Dict[str, Any] = {}
            self.mission: Dict[str, Any] = {
                "mode": None, "state": "idle", "t0": None, "elapsed": 0.0,
                "limit": None, "finished": False, "over_limit": False,
                "route": None, "phase": "待命",
            }
            self.target: Dict[str, Any] = {
                "no": None, "slot": None, "identified": False,
                "hit": False, "route_title": None,
            }
            self.px4: Dict[str, Any] = {
                "state": "UNKNOWN", "mode": "", "armed": False,
                "ekf_ok": False, "link_ok": False, "ready": 0,
                "mission_active": False, "error": "", "message": "",
                "takeoff_available": False, "last_rx": None,
            }
            self.telemetry: Dict[str, Any] = {
                "x": self.field.takeoff["x"], "y": self.field.takeoff["y"],
                "z": 0.0, "yaw": self.field.takeoff.get("yaw", 90.0),
                "batt": None, "armed": False, "laser": False,
                "has_height": False,
                "state": "idle", "slot": None, "last_rx": None, "age": None,
            }
            self.trail: List[List[float]] = []
        if announce:
            self.log("info", "已清空本次盘点数据，开始新的一轮")

    # ------------------------------------------------------------------ log
    def log(self, level: str, text: str, **extra) -> None:
        evt = {"t": time.time(), "level": level, "text": text}
        evt.update(extra)
        with self.lock:
            self.events.append(evt)

    # ------------------------------------------------------------- messages
    def handle(self, msg: Dict[str, Any], _source: str = "?") -> None:
        mtype = str(msg.get("type", "")).lower()
        if mtype == "telemetry":
            self._on_telemetry(msg)
        elif mtype == "detect":
            self._on_detect(msg)
        elif mtype == "target_id":
            self._on_target_id(msg)
        elif mtype == "target_hit":
            self._on_target_hit(msg)
        elif mtype == "mission":
            self._on_mission(msg)
        elif mtype == "px4_status":
            self._on_px4_status(msg)
        elif mtype == "hello":
            device = str(msg.get("device", "未知设备"))
            now = time.time()
            if now - self.hello_seen.get(device, 0.0) >= 30.0:
                self.log("info", f"链路已连接：{device}")
                self.hello_seen[device] = now
        elif mtype == "bye":
            self.hello_seen.pop(str(msg.get("device", "未知设备")), None)
            self.log("warn", "链路断开")
        elif mtype == "hello_gs":
            pass
        else:
            self.log("debug", f"未识别消息：{json.dumps(msg, ensure_ascii=False)[:160]}")

    def _on_telemetry(self, msg: Dict[str, Any]) -> None:
        pos = msg.get("pos") or {}
        with self.lock:
            t = self.telemetry
            if pos:
                t["x"] = float(pos.get("x", t["x"]))
                t["y"] = float(pos.get("y", t["y"]))
                t["z"] = float(pos.get("z", t["z"]))
                if not self.trail or (abs(self.trail[-1][0] - t["x"]) > 2.0 or
                                      abs(self.trail[-1][1] - t["y"]) > 2.0):
                    self.trail.append([round(t["x"], 1), round(t["y"], 1)])
                    if len(self.trail) > 4000:
                        self.trail = self.trail[-2000:]
            t["yaw"] = float(msg.get("yaw", t["yaw"]))
            # 机载端会带 has_height：飞控没上电时高度不可信，界面显示 "--"
            t["has_height"] = bool(msg.get("has_height", t.get("has_height", True)))
            if msg.get("batt") is not None:
                t["batt"] = float(msg["batt"])
            t["armed"] = bool(msg.get("armed", t["armed"]))
            t["laser"] = bool(msg.get("laser", t["laser"]))
            t["state"] = msg.get("state", t["state"])
            t["slot"] = msg.get("slot", t["slot"])
            t["last_rx"] = time.time()
            t["age"] = 0.0

    def _on_detect(self, msg: Dict[str, Any]) -> None:
        slot = str(msg.get("slot") or msg.get("coord") or "").strip()
        no = msg.get("no")
        if not slot and msg.get("x") is not None:
            # 飞机只上报坐标时，按坐标反查最近的坐标编号
            try:
                slot = self.field.slot_from_xyz(float(msg["x"]),
                                                float(msg.get("y", 0.0)),
                                                float(msg.get("z", 0.0)))
            except (TypeError, ValueError):
                slot = ""
        if not msg.get("ok", True) or no is None:
            self.log("warn", f"{slot} 未能识别二维码"
                              f"（{msg.get('reason', 'unknown')}）")
            return
        no = int(no)
        info = self.field.slots.get(slot)
        def _num(v, default=0.0):
            try:
                return round(float(v), 1)
            except (TypeError, ValueError):
                return default
        # 坐标优先用机载端上报的实测值（纵向扫描会带回实测高度 z）；
        # 没带就用模型值兜底。x/y 是货架几何量，实测与模型一致。
        qr_x = _num(info["x"]) if info else 0.0
        qr_y = _num(info["y"]) if info else 0.0
        qr_z = _num(info["z"]) if info else 0.0
        if msg.get("x") is not None:
            qr_x = _num(msg.get("x"), qr_x)
        if msg.get("y") is not None:
            qr_y = _num(msg.get("y"), qr_y)
        if msg.get("z") is not None:
            qr_z = _num(msg.get("z"), qr_z)
        rec = {
            "no": no, "slot": slot,
            "face": info["face"] if info else slot[:1],
            "row": info["row"] if info else None,
            "x": qr_x, "y": qr_y, "z": qr_z,
            "conf": msg.get("conf"),
            "t": msg.get("t", None),
            "wall": time.time(),
            "elapsed": self.elapsed(),
        }
        with self.lock:
            old = self.mapping.get(slot)
            self.records[slot] = rec
            self.mapping[slot] = no
            self.by_number[no] = slot
            self.heights[slot] = qr_z      # 定向盘点直达时用实测高度
            self.detects.append(rec)
            # LED：每盘点到一个货物亮灭一次（亮 1 秒左右）
            self.led_seq += 1
            self.led_until = time.time() + 1.0
            self.led_state = True
        if old is not None and int(old) != no:
            self.log("warn", f"{slot} 编号变化：{old} -> {no}")
        self.log("ok", f"盘点成功：货物 {no} 位于 {slot}"
                       f"（{rec['x']:.0f},{rec['y']:.0f},{rec['z']:.0f} cm）")
        self.on_led(True)

    def _on_target_id(self, msg: Dict[str, Any]) -> None:
        no = msg.get("no")
        if no is None:
            return
        with self.lock:
            self.target["no"] = int(no)
            self.target["identified"] = True
        self.log("info", f"无人机识别到抽取的二维码：货物编号 {int(no)}")

    def _on_target_hit(self, msg: Dict[str, Any]) -> None:
        with self.lock:
            self.target["hit"] = True
            if msg.get("slot"):
                self.target["slot"] = str(msg["slot"])
        self.log("ok", f"定向盘点命中：货物 {msg.get('no')} -> {msg.get('slot')}")

    def _on_mission(self, msg: Dict[str, Any]) -> None:
        phase = msg.get("phase")
        with self.lock:
            if phase in ("start", "begin"):
                if self.mission["t0"] is None:
                    self.mission["t0"] = time.time()
                    self.mission["state"] = "running"
            elif phase in ("done", "end"):
                elapsed = self.elapsed()
                self.mission["state"] = "finished"
                self.mission["finished"] = True
                self.mission["elapsed"] = elapsed
                if not msg.get("note"):
                    self.mission["phase"] = "任务完成"
            elif phase in ("abort", "estop", "stopped"):
                # 无人机侧主动中止（含急停）：同样冻结计时，并让界面显示为“急停”。
                if not self.mission["finished"] and self.mission["t0"] is not None:
                    self.mission["elapsed"] = self.elapsed()
                    self.mission["finished"] = True
                self.mission["state"] = "estop"
                if not msg.get("note"):
                    self.mission["phase"] = "任务已停止"
            if msg.get("note"):
                self.mission["phase"] = msg["note"]
        if phase in ("done", "end"):
            self.log("info", f"任务结束，用时 {self.elapsed():.1f} 秒")
        elif phase in ("abort", "estop", "stopped"):
            self.log("error",
                     f"无人机中止任务：{msg.get('note') or '未说明原因'}，已就地悬停")

    def _on_px4_status(self, msg: Dict[str, Any]) -> None:
        with self.lock:
            p = self.px4
            old_state = p["state"]
            p["state"] = str(msg.get("state", p["state"]))
            p["mode"] = str(msg.get("mode", p["mode"]))
            p["armed"] = bool(msg.get("armed", p["armed"]))
            p["ekf_ok"] = bool(msg.get("ekf_ok", p["ekf_ok"]))
            p["link_ok"] = bool(msg.get("link_ok", p["link_ok"]))
            if msg.get("battery_pct") is not None:
                p["battery_pct"] = int(msg["battery_pct"])
            p["ready"] = int(msg.get("ready", p["ready"]))
            p["mission_active"] = bool(
                msg.get("mission_active", p["mission_active"])
            )
            p["error"] = str(msg.get("error", p["error"]))
            p["message"] = str(msg.get("message", p["message"]))
            p["takeoff_available"] = bool(
                msg.get("takeoff_available", p["takeoff_available"])
            )
            p["last_rx"] = time.time()
            if p["state"] == "WAIT_GO" and self.mission["state"] != "running":
                self.mission["phase"] = "待命"
        if p["state"] != old_state:
            message = {
                "WAIT_STABLE": "机载端正在检查定位稳定性",
                "WAIT_OFFBOARD": "等待 PX4 切换 OFFBOARD",
                "WAIT_GO": "预检通过，等待地面站起飞指令",
                "ARMING": "PX4 正在解锁",
                "START_MISSION": "正在启动盘点任务",
                "FLIGHT": "盘点任务执行中",
                "LAND": "PX4 正在降落",
                "FINISHED": "PX4 任务流程已结束",
                "HANDOVER": "已退出 Offboard，交回人工控制",
            }.get(p["state"], p["state"])
            self.log("info", f"PX4 状态：{message}")

    # ------------------------------------------------------------- timing
    def elapsed(self) -> float:
        with self.lock:
            m = self.mission
            if m["t0"] is None:
                return 0.0
            if m["finished"]:
                return float(m["elapsed"])
            return time.time() - m["t0"]

    def start_mission(self, mode: str) -> None:
        with self.lock:
            self.mission["t0"] = time.time()
            self.mission["state"] = "running"
            self.mission["finished"] = False
            self.mission["over_limit"] = False
            limit = self.field.cfg.get("time_limits_s", {}).get(mode)
            self.mission["limit"] = float(limit) if limit else None
            self.mission["mode"] = mode

    # ------------------------------------------------------------ commands
    def start_sweep(self) -> Dict[str, Any]:
        route = plan_sweep(self.field)
        with self.lock:
            self.reset_data_keep_route()
            self.mission["mode"] = "sweep"
            self.mission["route"] = route
            self.mission["phase"] = "遍历盘点中"
            self.target["route_title"] = route["title"]
        self.start_mission("sweep")
        self.log("info", f"下发遍历盘点航线：{route['title']}，"
                         f"预计用时 {route['eta_s']:.0f}s，"
                         f"航程 {route['length_cm'] / 100:.1f}m")
        self.send({"type": "cmd", "cmd": "px4_start", "mode": "sweep",
                   "route": route})
        return route

    def reset_data_keep_route(self) -> None:
        with self.lock:
            self.records.clear()
            self.mapping.clear()
            self.by_number.clear()
            self.heights.clear()
            self.detects.clear()
            self.trail = []
            self.led_seq = 0
            self.led_until = 0.0

    def start_target(self, no: Optional[int]) -> Dict[str, Any]:
        with self.lock:
            mapping = dict(self.mapping)
            heights = dict(self.heights)
        # 目标位置已知时直达；高度用遍历盘点时实测的值（纵向扫描记录的）
        measured_z = None
        if no is not None:
            for slot, num in mapping.items():
                if int(num) == int(no):
                    measured_z = heights.get(slot)
                    break
        route = plan_target(self.field, no, mapping, measured_z=measured_z)
        with self.lock:
            self.target["no"] = int(no) if no is not None else None
            self.target["slot"] = route.get("target_slot")
            self.target["hit"] = False
            self.target["route_title"] = route["title"]
            self.mission["mode"] = "target"
            self.mission["route"] = route
            self.mission["phase"] = "定向盘点中"
        self.start_mission("target")
        self.log("info", f"下发定向盘点航线：{route['title']}，"
                         f"预计用时 {route['eta_s']:.0f}s")
        self.send({"type": "cmd", "cmd": "px4_start", "mode": "target",
                   "target_no": no, "route": route})
        return route

    def stop_and_land(self) -> None:
        self.mission["phase"] = "返航降落中"
        self.send({"type": "cmd", "cmd": "land"})
        self.log("warn", "已下发降落指令")

    def estop(self) -> None:
        # 急停 = 无人机就地悬停（机载端会把 PID 安全门置为无效、立刻下发零速度）。
        # 计时到此为止：不再继续累加，否则界面会一直走表甚至误报“已超时”。
        with self.lock:
            self.mission["state"] = "estop"
            self.mission["phase"] = "紧急停止（就地悬停）"
            if not self.mission["finished"] and self.mission["t0"] is not None:
                self.mission["elapsed"] = self.elapsed()
                self.mission["finished"] = True
        self.send({"type": "cmd", "cmd": "estop"})
        self.log("error", "紧急停止：已下发就地悬停，任务计时停止")

    def query_number(self, no: int) -> Dict[str, Any]:
        with self.lock:
            slot = self.by_number.get(int(no))
            rec = self.records.get(slot) if slot else None
            result = {"no": int(no), "found": rec is not None,
                      "slot": slot, "record": rec}
            self.query = result
        if rec:
            self.log("info", f"查询：货物 {no} 在 {slot} "
                             f"({rec['x']:.0f},{rec['y']:.0f},{rec['z']:.0f}cm)")
        else:
            self.log("warn", f"查询：货物 {no} 尚未盘点")
        return result

    def set_mapping(self, slot: str, no: int) -> None:
        with self.lock:
            self.mapping[slot] = int(no)
            self.by_number[int(no)] = slot
        self.log("info", f"手工标定：{slot} -> 货物 {no}")

    # -------------------------------------------------------------- exports
    def export_csv(self) -> str:
        buf = io.StringIO()
        writer = csv.writer(buf)
        writer.writerow(["序号", "货物编号", "坐标", "货架面", "行",
                         "X(cm)", "Y(cm)", "Z(cm)", "置信度", "任务用时(s)", "记录时间"])
        with self.lock:
            records = sorted(self.records.values(), key=lambda r: (r["slot"]))
        for i, r in enumerate(records, 1):
            writer.writerow([
                i, r["no"], r["slot"], r["face"], r["row"],
                f"{r['x']:.1f}", f"{r['y']:.1f}", f"{r['z']:.1f}",
                r.get("conf") if r.get("conf") is not None else "",
                f"{r['elapsed']:.1f}",
                time.strftime("%Y-%m-%d %H:%M:%S", time.localtime(r["wall"])),
            ])
        missing = self.missing_numbers()
        writer.writerow([])
        writer.writerow(["未盘点货物编号", ", ".join(str(m) for m in missing)])
        return buf.getvalue()

    def save_session(self) -> str:
        cfgmod.ensure_dirs()
        snap = self.snapshot()
        name = time.strftime("session_%Y%m%d_%H%M%S.json")
        path = os.path.join(cfgmod.SESSION_DIR, name)
        with open(path, "w", encoding="utf-8") as fh:
            json.dump(snap, fh, ensure_ascii=False, indent=2)
        latest = os.path.join(cfgmod.SESSION_DIR, "latest.json")
        with open(latest, "w", encoding="utf-8") as fh:
            json.dump(snap, fh, ensure_ascii=False, indent=2)
        return path

    def missing_numbers(self) -> List[int]:
        with self.lock:
            found = {int(v) for v in self.mapping.values()}
        return [n for n in range(1, self.field.total_slots + 1)
                if n not in found]

    # ------------------------------------------------------------- snapshot
    def snapshot(self) -> Dict[str, Any]:
        now = time.time()
        with self.lock:
            tel = dict(self.telemetry)
            if tel.get("last_rx"):
                tel["age"] = round(now - tel["last_rx"], 2)
            elapsed = self.elapsed()
            mission = dict(self.mission)
            mission["elapsed"] = round(elapsed, 2)
            limit = mission.get("limit")
            mission["remaining"] = (round(limit - elapsed, 1)
                                    if limit else None)
            mission["over_limit"] = bool(limit and elapsed > limit)
            records = sorted(self.records.values(), key=lambda r: r["slot"])
            snap = {
                "now": now,
                "uptime": round(now - self.started_at, 1),
                "mission": mission,
                "px4": dict(self.px4),
                "target": self.target,
                "telemetry": tel,
                "trail": list(self.trail[-1200:]),
                "records": records,
                "detects": list(self.detects)[-40:],
                "mapping": {k: int(v) for k, v in self.mapping.items()},
                "missing": self.missing_numbers(),
                "total_slots": self.field.per_face * len(self.field.faces),
                "led": {"seq": self.led_seq, "on": now < self.led_until,
                        "remaining": round(max(0.0, self.led_until - now), 2)},
                "query": self.query,
                "events": list(self.events)[-120:],
                "links": self.link_status,
            }
        return snap
