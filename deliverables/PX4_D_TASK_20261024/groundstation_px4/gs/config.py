"""场地几何与地面站运行配置。

所有几何量单位统一为厘米(cm)，角度单位为度(deg)，坐标系定义：
    - 原点在作业区左下角
    - X 轴向右，Y 轴向上（俯视图）
    - Z 轴向上，地面为 0
    - yaw 采用数学约定：yaw=0 指向 +X，逆时针为正

注意：仓库俯视图（题目图1）中的起飞点/降落点/货架实际摆放位置需要按赛区
真实场地标定，这里给出的是可运行的默认值，请在 config/field.json 中修改。
"""

from __future__ import annotations

import copy
import json
import os
from typing import Any, Dict

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.dirname(HERE)
CONFIG_DIR = os.path.join(ROOT, "config")
FIELD_PATH = os.path.join(CONFIG_DIR, "field.json")
RUNTIME_PATH = os.path.join(CONFIG_DIR, "runtime.json")
DATA_DIR = os.path.join(ROOT, "data")
SESSION_DIR = os.path.join(DATA_DIR, "sessions")


def _face(face_id: str, normal) -> Dict[str, Any]:
    return {"id": face_id, "normal": list(normal)}


DEFAULT_FIELD: Dict[str, Any] = {
    "version": 2,
    "name": "赛区场地 500cm x 400cm",
    "note": "坐标由题目图1 标定（用起飞点/降落点校核，误差<1cm）。"
            "如现场有出入，请修改本文件中的 board_x / y_from / y_to。",
    # 作业区：X 向右 0~500，Y 向上 0~400，原点在左下角
    "area": {"w": 500.0, "h": 400.0, "wall_margin": 5.0},
    "takeoff": {"x": 75.0, "y": 75.0, "yaw": 90.0, "size": 40.0},
    "landing": {"x": 425.0, "y": 325.0, "yaw": 90.0, "radius": 30.0},
    # 无人机安全半径：规划航线时以此膨胀货架障碍
    "safety_margin_cm": 30.0,
    # 一个双面货架（离起飞点近的那个）：板面沿 Y 方向竖放，面法线为 ±X
    # S1 提供 A/B 两面，共 12 个点位
    "racks": [
        {
            "id": "S1",
            "axis": "y",
            "board_x": 150.0,
            "y_from": 100.0,
            "y_to": 300.0,
            "thickness": 5.0,
            "faces": [
                _face("A", (-1.0, 0.0)),
                _face("B", (1.0, 0.0)),
            ],
        },
    ],
    # 货架尺寸：长 200cm、板面高 100cm、板面下沿离地 70cm（即板面 70~170cm）
    "rack_length_cm": 200.0,
    "rack_height_cm": 100.0,
    "board_bottom_cm": 70.0,
    # 每个面 2 行 x 3 列 = 6 个二维码
    # 行高：从上到下第 1 行 140cm、第 2 行 100cm（行距 40cm）
    "slot_rows_cm": [140.0, 100.0],
    # 列位置：相对板面中心（沿板长方向）的偏移，列距 50cm
    "slot_cols_offset_cm": [-50.0, 0.0, 50.0],
    # 编号方式：站在该面正前方，从左上角开始按行计数
    "slot_numbering": "viewer_rowmajor",
    "standoff_cm": 55.0,              # 无人机相对货架面的悬停距离
    "cruise_alt_cm": 150.0,           # 题目要求 150±10cm
    "laser_dwell_s": 0.5,             # 激光笔点亮时长（题目：0.5s 左右）
    "speeds": {
        "horizontal": 100.0,          # cm/s
        "vertical": 50.0,             # cm/s
        "yaw": 90.0,                  # deg/s
        "land": 30.0,                 # cm/s
    },
    "timing": {
        "climb_s": 4.0,               # 起飞到 150cm 的点位稳定时间
        "settle_s": 1.5,              # 每个盘点点稳定时间
        "scan_s": 1.2,                # 单点识别耗时（含激光点亮）
        "land_s": 6.0,                # 降落过程耗时
    },
    "time_limits_s": {"sweep": 270.0, "target": 180.0},
    # 规划偏置：换通道(aisle)的额外代价，越大越倾向于按面连续盘点
    "aisle_switch_penalty_cm": 120.0,
}


def _deep_merge(base: Dict[str, Any], override: Dict[str, Any]) -> Dict[str, Any]:
    out = copy.deepcopy(base)
    for key, value in (override or {}).items():
        if isinstance(value, dict) and isinstance(out.get(key), dict):
            out[key] = _deep_merge(out[key], value)
        else:
            out[key] = copy.deepcopy(value)
    return out


def ensure_dirs() -> None:
    for path in (CONFIG_DIR, DATA_DIR, SESSION_DIR):
        os.makedirs(path, exist_ok=True)


def load_field() -> Dict[str, Any]:
    ensure_dirs()
    if os.path.exists(FIELD_PATH):
        with open(FIELD_PATH, "r", encoding="utf-8") as fh:
            return _deep_merge(DEFAULT_FIELD, json.load(fh))
    save_field(DEFAULT_FIELD)
    return copy.deepcopy(DEFAULT_FIELD)


def save_field(field: Dict[str, Any]) -> Dict[str, Any]:
    ensure_dirs()
    merged = _deep_merge(DEFAULT_FIELD, field)
    with open(FIELD_PATH, "w", encoding="utf-8") as fh:
        json.dump(merged, fh, ensure_ascii=False, indent=2)
    return merged


DEFAULT_RUNTIME: Dict[str, Any] = {
    # 与无人机通讯方式：websocket | udp | tcp | serial | mock（可多种同时开）
    "links": {
        # 局域网直连（推荐）：机载端做客户端连到 ws://<电脑IP>:8890/ws
        "websocket": {"enabled": True, "bind": "0.0.0.0", "port": 8890,
                      "path": "/ws", "token": "", "idle_timeout_sec": 45.0},
        "udp": {"enabled": False, "bind": "0.0.0.0", "port": 8888},
        "tcp": {"enabled": False, "bind": "0.0.0.0", "port": 8889},
        # 无线数传：串口 COM9 @ 115200
        "serial": {"enabled": False, "port": "COM9", "baud": 115200},
        "mock": {"enabled": False, "time_scale": 1.0},
    },
    # 可选：用一个真实 LED 灯板（单片机）代替屏幕上的 LED 指示
    "led_output": {"enabled": False, "port": "COM5", "baud": 115200,
                   "on_cmd": "LED1\n", "off_cmd": "LED0\n"},
    "http": {"host": "127.0.0.1", "port": 8765},
    "camera": {
        "enabled": True,
        "url": "http://10.249.123.10:8080/stream",
        "label": "机载摄像头 /dev/video0",
    },
    "audio_beep": True,
}


def load_runtime() -> Dict[str, Any]:
    ensure_dirs()
    if os.path.exists(RUNTIME_PATH):
        with open(RUNTIME_PATH, "r", encoding="utf-8") as fh:
            runtime = _deep_merge(DEFAULT_RUNTIME, json.load(fh))
    else:
        runtime = copy.deepcopy(DEFAULT_RUNTIME)
        save_runtime(runtime)
    # 允许用环境变量/命令行覆盖 HTTP 端口
    env_port = os.environ.get("GS_PORT")
    if env_port:
        runtime["http"]["port"] = int(env_port)
    env_camera = os.environ.get("GS_CAMERA_URL")
    if env_camera:
        runtime["camera"]["url"] = env_camera
    return runtime


def save_runtime(runtime: Dict[str, Any]) -> Dict[str, Any]:
    ensure_dirs()
    merged = _deep_merge(DEFAULT_RUNTIME, runtime)
    with open(RUNTIME_PATH, "w", encoding="utf-8") as fh:
        json.dump(merged, fh, ensure_ascii=False, indent=2)
    return merged
