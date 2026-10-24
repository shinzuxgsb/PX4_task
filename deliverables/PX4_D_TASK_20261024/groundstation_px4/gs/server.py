"""地面站 HTTP 服务：本地网页界面 + JSON API。"""

from __future__ import annotations

import json
import mimetypes
import os
import posixpath
import threading
import time
import urllib.parse
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from typing import Any, Dict, Optional

from . import __version__, config as cfgmod
from .field import Field
from .links import (HAVE_SERIAL, LinkManager, MockLink, SerialLink, TcpLink,
                    UdpLink, WebSocketLink)
from .mock import VirtualDrone, make_ground_truth
from .planner import plan_sweep, plan_target
from .state import GroundState

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
WEB_DIR = os.path.join(ROOT, "web")

#: 真实链路最近有数据时，内置虚拟无人机静默的窗口（秒）。
#: 真机遥测 10Hz，5s 足够覆盖偶发丢包；真机真的断了，虚拟无人机再恢复。
REAL_PRIORITY_WINDOW_S = 5.0


class GroundStation:
    def __init__(self, runtime: Dict[str, Any], field_cfg: Optional[Dict[str, Any]] = None,
                 force_mock: Optional[bool] = None, time_scale: Optional[float] = None):
        self.runtime = runtime
        self.field_cfg = field_cfg or cfgmod.load_field()
        self.field = Field(self.field_cfg)
        self.state = GroundState(self.field, runtime)
        self.links = LinkManager(self.state.handle)
        self.state.send = self.links.send
        self.state.on_led = self.links.led
        self.ground_truth: Dict[str, int] = {}

        mock_cfg = dict(runtime["links"].get("mock", {}))
        if force_mock is not None:
            mock_cfg["enabled"] = force_mock
        if time_scale is not None:
            mock_cfg["time_scale"] = time_scale
        self.mock_cfg = mock_cfg
        self.mock_link: Optional[MockLink] = None
        self.last_real_rx = 0.0
        self._mock_suppressed = False

    # -------------------------------------------------------------- startup
    def build_links(self) -> None:
        rt = self.runtime["links"]
        if rt.get("websocket", {}).get("enabled"):
            ws_cfg = rt["websocket"]
            self.links.add(WebSocketLink(
                self._on_real_message,
                ws_cfg.get("bind", "0.0.0.0"),
                ws_cfg.get("port", 8890),
                ws_cfg.get("path", "/ws"),
                ws_cfg.get("token", ""),
                ws_cfg.get("idle_timeout_sec", 45.0)))
        if rt.get("udp", {}).get("enabled"):
            self.links.add(UdpLink(self._on_real_message,
                                   rt["udp"].get("bind", "0.0.0.0"),
                                   rt["udp"].get("port", 8888)))
        if rt.get("tcp", {}).get("enabled"):
            self.links.add(TcpLink(self._on_real_message,
                                   rt["tcp"].get("bind", "0.0.0.0"),
                                   rt["tcp"].get("port", 8889)))
        if rt.get("serial", {}).get("enabled"):
            self.links.add(SerialLink(self._on_real_message,
                                      rt["serial"].get("port", "COM3"),
                                      rt["serial"].get("baud", 115200)))
        if self.mock_cfg.get("enabled"):
            self.ground_truth = make_ground_truth(self.field)
            drone = VirtualDrone(self.field, self._publish,
                                 self.ground_truth,
                                 self.mock_cfg.get("time_scale", 1.0))
            self.mock_link = MockLink(self.state.handle, drone)
            self.links.add(self.mock_link)
        self.links.configure_led(self.runtime.get("led_output", {}))
        for link in self.links.links:          # 链路日志接到界面事件流
            link.on_log = self.state.log
        self.links.start()
        self.state.link_status = self.links.status()
        self.state.log("info", f"地面站 v{__version__} 启动，场地：{self.field_cfg.get('name')}")
        self.log_field_warnings()
        real_links = [link.name for link in self.links.links if link.name != "mock"]
        if self.mock_cfg.get("enabled") and real_links:
            self.state.log(
                "warn",
                "内置虚拟无人机与真实链路同时开着（" + "/".join(real_links) +
                "）：演练数据会和真机数据混在一起，接真机请用 run_real.bat "
                "或 python server.py --no-mock")
        if not HAVE_SERIAL:
            self.state.log("debug", "未安装 pyserial：串口链路不可用（如需请 pip install pyserial）")

    def _publish(self, msg: Dict[str, Any]) -> None:
        # 真机优先：真实链路有数据时，虚拟无人机整条静默。
        # 否则两条遥测流会互相覆盖（真机在起飞点、虚拟机在降落点），
        # 界面上就是“在两个点之间来回瞬移”。
        if time.time() - self.last_real_rx < REAL_PRIORITY_WINDOW_S:
            if not self._mock_suppressed:
                self._mock_suppressed = True
                self.state.log(
                    "warn",
                    "检测到真实链路数据：内置虚拟无人机已静默"
                    "（避免真机与虚拟机的位置互相覆盖）")
            return
        if self._mock_suppressed:
            self._mock_suppressed = False
            self.state.log("info", "真实链路已停止上报：内置虚拟无人机恢复运行")
        self.state.handle(msg, "mock")

    def _on_real_message(self, msg: Dict[str, Any], source: str = "link") -> None:
        """真实链路（WebSocket/UDP/TCP/串口）来的消息，兼作“真机在线”标志。"""
        self.last_real_rx = time.time()
        self.state.handle(msg, source)

    def reset_mock(self) -> None:
        """清空数据时把内置虚拟无人机收回起飞点。"""
        if self.mock_link is not None:
            self.mock_link.drone.reset()

    def refresh_link_status(self) -> None:
        self.state.link_status = self.links.status()

    # ------------------------------------------------------------ operations
    def reload_field(self, cfg: Dict[str, Any]) -> None:
        self.field_cfg = cfgmod.save_field(cfg)
        self.field = Field(self.field_cfg)
        self.state.field = self.field
        if self.mock_link is not None:
            self.mock_link.drone.field = self.field
        self.state.log("info", "场地几何已更新，重新生成航线")
        self.log_field_warnings()

    def log_field_warnings(self) -> None:
        """把场地配置的一致性提示打到界面日志里。

        目前只有一条：同一列上下两排的悬停 y 差了多少（2026-09-27 起列的配对
        改成按列号，所以 y 允许不同——这里只是留个提示，防止某一排 y 填错）。
        """
        for note in getattr(self.field, "warnings", []) or []:
            self.state.log("warn", f"场地检查：{note}")

    def plan_preview(self, mode: str, no: Optional[int] = None) -> Dict[str, Any]:
        if mode == "target":
            mapping = dict(self.state.mapping)
            measured_z = None
            if no is not None:
                for slot, num in mapping.items():
                    if int(num) == int(no):
                        measured_z = self.state.heights.get(slot)
                        break
            return plan_target(self.field, no, mapping, measured_z=measured_z)
        return plan_sweep(self.field)


class Handler(BaseHTTPRequestHandler):
    server_version = "ShelfInventoryGS/0.1"
    gs: GroundStation = None  # type: ignore

    # ------------------------------------------------------------- helpers
    def log_message(self, fmt, *args):  # 静默
        return

    def _send(self, code: int, body: bytes, ctype: str) -> None:
        self.send_response(code)
        self.send_header("Content-Type", ctype)
        self.send_header("Content-Length", str(len(body)))
        self.send_header("Cache-Control", "no-store")
        self.end_headers()
        try:
            self.wfile.write(body)
        except (BrokenPipeError, ConnectionResetError):
            pass

    def _json(self, obj: Any, code: int = 200) -> None:
        body = json.dumps(obj, ensure_ascii=False).encode("utf-8")
        self._send(code, body, "application/json; charset=utf-8")

    def _body(self) -> Dict[str, Any]:
        length = int(self.headers.get("Content-Length") or 0)
        if not length:
            return {}
        raw = self.rfile.read(length)
        try:
            return json.loads(raw.decode("utf-8"))
        except Exception:
            return {}

    # ----------------------------------------------------------- GET/POST
    def do_GET(self) -> None:
        self.gs.refresh_link_status()
        parsed = urllib.parse.urlparse(self.path)
        path = parsed.path
        query = urllib.parse.parse_qs(parsed.query)
        if path == "/api/state":
            self._json(self.gs.state.snapshot())
            return
        if path == "/api/field":
            self._json({"field": self.gs.field.to_dict(),
                        "raw": self.gs.field_cfg})
            return
        if path == "/api/config":
            self._json({"runtime": self.gs.runtime,
                        "serial_available": HAVE_SERIAL,
                        "version": __version__})
            return
        if path == "/api/route":
            mode = (query.get("mode") or ["sweep"])[0]
            no = query.get("no")
            no = int(no[0]) if no else None
            self._json(self.gs.plan_preview(mode, no))
            return
        if path == "/api/export.csv":
            body = ("\ufeff" + self.gs.state.export_csv()).encode("utf-8")
            self.send_response(200)
            self.send_header("Content-Type", "text/csv; charset=utf-8")
            self.send_header("Content-Disposition",
                             "attachment; filename=inventory.csv")
            self.send_header("Content-Length", str(len(body)))
            self.end_headers()
            self.wfile.write(body)
            return
        if path == "/api/ground_truth":
            self._json(self.gs.ground_truth)
            return
        self._static(path)

    def do_POST(self) -> None:
        self.gs.refresh_link_status()
        path = urllib.parse.urlparse(self.path).path
        body = self._body()
        st = self.gs.state
        if path == "/api/cmd":
            cmd = body.get("cmd")
            if cmd == "start_sweep":
                if not st.px4.get("takeoff_available"):
                    self._json({
                        "ok": False,
                        "error": "PX4 preflight is not in WAIT_GO",
                    }, 409)
                    return
                self._json({"ok": True, "route": st.start_sweep()})
                return
            if cmd == "start_target":
                if not st.px4.get("takeoff_available"):
                    self._json({
                        "ok": False,
                        "error": "PX4 preflight is not in WAIT_GO",
                    }, 409)
                    return
                no = body.get("no")
                self._json({"ok": True,
                            "route": st.start_target(int(no) if no else None)})
                return
            if cmd == "land":
                st.stop_and_land()
                self._json({"ok": True})
                return
            if cmd == "estop":
                st.estop()
                self._json({"ok": True})
                return
            if cmd == "reset":
                st.reset_session()
                self.gs.reset_mock()
                self._json({"ok": True})
                return
            if cmd == "query":
                self._json(st.query_number(int(body.get("no", 0))))
                return
            if cmd == "set_mapping":
                st.set_mapping(str(body.get("slot")), int(body.get("no")))
                self._json({"ok": True})
                return
            if cmd == "target_id":
                st.handle({"type": "target_id", "no": body.get("no")})
                self._json({"ok": True})
                return
            if cmd == "save_session":
                self._json({"ok": True, "path": st.save_session()})
                return
            self._json({"ok": False, "error": f"未知指令 {cmd}"}, 400)
            return
        if path == "/api/field":
            self.gs.reload_field(body.get("field") or {})
            self._json({"ok": True, "field": self.gs.field.to_dict()})
            return
        if path == "/api/runtime":
            self.gs.runtime = cfgmod.save_runtime(body.get("runtime") or {})
            self._json({"ok": True, "runtime": self.gs.runtime})
            return
        self._json({"ok": False, "error": "not found"}, 404)

    # ----------------------------------------------------------- static files
    def _static(self, path: str) -> None:
        if path in ("/", ""):
            path = "/index.html"
        safe = posixpath.normpath(path).lstrip("/")
        full = os.path.join(WEB_DIR, safe)
        if not os.path.abspath(full).startswith(os.path.abspath(WEB_DIR)):
            self._json({"error": "forbidden"}, 403)
            return
        if not os.path.isfile(full):
            self._json({"error": "not found", "path": path}, 404)
            return
        ctype, _ = mimetypes.guess_type(full)
        if full.endswith((".js", ".css", ".html")):
            ctype = (ctype or "text/plain") + "; charset=utf-8"
        with open(full, "rb") as fh:
            body = fh.read()
        self._send(200, body, ctype or "application/octet-stream")


def serve(runtime: Dict[str, Any], field_cfg: Optional[Dict[str, Any]] = None,
          force_mock: Optional[bool] = None,
          time_scale: Optional[float] = None,
          announce=None) -> ThreadingHTTPServer:
    gs = GroundStation(runtime, field_cfg, force_mock, time_scale)
    gs.build_links()
    Handler.gs = gs
    host = runtime["http"].get("host", "127.0.0.1")
    port = int(runtime["http"].get("port", 8765))
    httpd = ThreadingHTTPServer((host, port), Handler)
    httpd.daemon_threads = True
    if announce:
        announce(gs, host, port)
    return httpd
