"""与无人机的通信链路：UDP / TCP / WebSocket / 串口 / 内置虚拟无人机。"""

from __future__ import annotations

import json
import socket
import threading
import time
from typing import Any, Callable, Dict, List, Optional

from .ws import (
    OP_BINARY,
    OP_CLOSE,
    OP_CONTINUATION,
    OP_PING,
    OP_PONG,
    OP_TEXT,
    WebSocketConnection,
    WebSocketError,
)

try:  # 串口为可选依赖
    import serial  # type: ignore
    from serial.tools import list_ports  # type: ignore
    HAVE_SERIAL = True
except Exception:  # pragma: no cover
    serial = None
    list_ports = None
    HAVE_SERIAL = False


def _decode(data: bytes) -> Optional[Dict[str, Any]]:
    try:
        text = data.decode("utf-8", errors="ignore").strip()
    except Exception:
        return None
    if not text:
        return None
    # 兼容一行一条 / 一包一条 JSON
    for line in text.splitlines():
        line = line.strip().rstrip(",")
        if not line:
            continue
        if not line.startswith("{"):
            continue
        try:
            return json.loads(line)
        except Exception:
            continue
    return None


class BaseLink:
    name = "base"

    def __init__(self, on_message: Callable[[Dict[str, Any], str], None]):
        self.on_message = on_message
        # 可选：界面日志回调 (level, text)，由 GroundStation.build_links 注入
        self.on_log: Optional[Callable[[str, str], None]] = None
        self.status: Dict[str, Any] = {"name": self.name, "ok": False,
                                       "detail": "未启动", "rx": 0, "tx": 0,
                                       "last_rx": None}
        self._stop = threading.Event()

    def log(self, level: str, text: str) -> None:
        if self.on_log is not None:
            try:
                self.on_log(level, text)
            except Exception:
                pass

    def start(self) -> None:
        raise NotImplementedError

    def stop(self) -> None:
        self._stop.set()

    def send(self, payload: Dict[str, Any]) -> bool:
        return False


class UdpLink(BaseLink):
    name = "udp"

    def __init__(self, on_message, bind: str, port: int):
        super().__init__(on_message)
        self.bind = bind
        self.port = int(port)
        self.sock: Optional[socket.socket] = None
        self.peer: Optional[tuple] = None
        self.thread: Optional[threading.Thread] = None

    def start(self) -> None:
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self.sock.bind((self.bind, self.port))
        self.sock.settimeout(0.3)
        self.status.update({"ok": True, "detail": f"监听 UDP {self.bind}:{self.port}"})
        self.thread = threading.Thread(target=self._loop, name="link-udp",
                                       daemon=True)
        self.thread.start()

    def _loop(self) -> None:
        while not self._stop.is_set():
            try:
                data, addr = self.sock.recvfrom(8192)
            except socket.timeout:
                continue
            except OSError:
                break
            self.peer = addr
            msg = _decode(data)
            if msg is not None:
                self.status["rx"] += 1
                self.status["last_rx"] = time.time()
                self.status["peer"] = f"{addr[0]}:{addr[1]}"
                self.on_message(msg, self.name)

    def send(self, payload: Dict[str, Any]) -> bool:
        if not self.sock or not self.peer:
            return False
        try:
            data = (json.dumps(payload, ensure_ascii=False) + "\n").encode("utf-8")
            self.sock.sendto(data, self.peer)
            self.status["tx"] += 1
            return True
        except OSError:
            return False


class TcpLink(BaseLink):
    name = "tcp"

    def __init__(self, on_message, bind: str, port: int):
        super().__init__(on_message)
        self.bind = bind
        self.port = int(port)
        self.server: Optional[socket.socket] = None
        self.clients: List[socket.socket] = []
        self.thread: Optional[threading.Thread] = None

    def start(self) -> None:
        self.server = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.server.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self.server.bind((self.bind, self.port))
        self.server.listen(4)
        self.server.settimeout(0.3)
        self.status.update({"ok": True, "detail": f"监听 TCP {self.bind}:{self.port}"})
        self.thread = threading.Thread(target=self._loop, name="link-tcp",
                                       daemon=True)
        self.thread.start()

    def _loop(self) -> None:
        while not self._stop.is_set():
            try:
                conn, addr = self.server.accept()
            except socket.timeout:
                continue
            except OSError:
                break
            self.clients.append(conn)
            self.status["detail"] = f"TCP {self.bind}:{self.port} 已连接 {len(self.clients)} 台"
            threading.Thread(target=self._client_loop, args=(conn, addr),
                             daemon=True).start()

    def _client_loop(self, conn: socket.socket, addr) -> None:
        buf = b""
        conn.settimeout(0.5)
        while not self._stop.is_set():
            try:
                data = conn.recv(4096)
            except socket.timeout:
                continue
            except OSError:
                break
            if not data:
                break
            buf += data
            while b"\n" in buf:
                line, buf = buf.split(b"\n", 1)
                msg = _decode(line)
                if msg is not None:
                    self.status["rx"] += 1
                    self.status["last_rx"] = time.time()
                    self.status["peer"] = f"{addr[0]}:{addr[1]}"
                    self.on_message(msg, self.name)
        try:
            conn.close()
        except OSError:
            pass
        if conn in self.clients:
            self.clients.remove(conn)

    def send(self, payload: Dict[str, Any]) -> bool:
        if not self.clients:
            return False
        data = (json.dumps(payload, ensure_ascii=False) + "\n").encode("utf-8")
        ok = False
        for conn in list(self.clients):
            try:
                conn.sendall(data)
                ok = True
            except OSError:
                pass
        if ok:
            self.status["tx"] += 1
        return ok


class WebSocketLink(BaseLink):
    """局域网 WebSocket 链路（地面站做服务端，机载端做客户端）。

    机载端用 WiFi 连上后与数传串口完全等价：一条 JSON 一个文本帧。
    相比 UDP，WebSocket 走 TCP，航线（约 30KB）不会被分片丢掉，
    也不会出现“先收到半条 JSON”的问题。
    """

    name = "ws"

    def __init__(self, on_message, bind: str = "0.0.0.0", port: int = 8890,
                 path: str = "/ws", token: str = "",
                 idle_timeout_sec: float = 45.0):
        super().__init__(on_message)
        self.bind = bind
        self.port = int(port)
        self.path = path or ""
        self.token = token or ""
        self.idle_timeout_sec = float(idle_timeout_sec or 45.0)
        self.server: Optional[socket.socket] = None
        self.clients: List[WebSocketConnection] = []
        self.thread: Optional[threading.Thread] = None
        self._clients_lock = threading.Lock()

    # ------------------------------------------------------------ 生命周期
    def start(self) -> None:
        self.server = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.server.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self.server.bind((self.bind, self.port))
        self.server.listen(8)
        self.server.settimeout(0.3)
        self._refresh_detail(announce=True)
        self.status.update({"ok": True, "clients": 0})
        self.thread = threading.Thread(target=self._accept_loop, name="link-ws",
                                       daemon=True)
        self.thread.start()

    def stop(self) -> None:
        super().stop()
        if self.server is not None:
            try:
                self.server.close()
            except OSError:
                pass
        with self._clients_lock:
            clients = list(self.clients)
            self.clients.clear()
        for client in clients:
            client.send_close()
            client.close()
        self.status.update({"ok": False, "clients": 0, "detail": "已停止"})

    def _refresh_detail(self, announce: bool = False) -> None:
        with self._clients_lock:
            count = len(self.clients)
            peers = [c.peer for c in self.clients]
        url = f"ws://{self.bind}:{self.port}{self.path}"
        if count:
            self.status["detail"] = f"{url} 已连接 {count} 台（{', '.join(peers)}）"
        elif announce:
            self.status["detail"] = f"监听 {url}"
        else:
            self.status["detail"] = f"{url} 等待机载端连接"
        self.status["clients"] = count

    # --------------------------------------------------------------- 接入
    def _accept_loop(self) -> None:
        while not self._stop.is_set():
            try:
                conn, addr = self.server.accept()
            except socket.timeout:
                continue
            except OSError:
                break
            threading.Thread(target=self._client_loop, args=(conn, addr),
                             name="link-ws-client", daemon=True).start()

    def _client_loop(self, conn: socket.socket, addr) -> None:
        ws = WebSocketConnection(conn, addr)
        conn.settimeout(5.0)          # 握手给 5s，之后按 0.5s 轮询收发帧
        try:
            if not ws.handshake(self.path, self.token):
                self.log("warn", f"WebSocket 握手失败/被拒绝：{ws.peer}")
                ws.close()
                return
        except OSError:
            ws.close()
            return

        conn.settimeout(0.5)
        with self._clients_lock:
            self.clients.append(ws)
        self._refresh_detail()
        self.log("ok", f"WebSocket 机载端已连接：{ws.peer}（{ws.path}）")

        fragment_buffer = b""
        try:
            while not self._stop.is_set():
                try:
                    opcode, payload, fin = ws.read_frame()
                except socket.timeout:
                    idle = time.time() - ws.last_rx
                    if idle >= self.idle_timeout_sec:
                        self.log("warn", f"WebSocket {ws.peer} 已 {int(idle)}s 无数据，断开")
                        break
                    if idle >= self.idle_timeout_sec / 3.0:
                        ws.send_ping()          # 保活，顺便试探链路是否还通
                    continue
                except (WebSocketError, OSError):
                    break

                ws.last_rx = time.time()
                self.status["last_rx"] = time.time()
                self.status["peer"] = ws.peer

                if opcode == OP_PING:
                    ws.send_frame(OP_PONG, payload)
                    continue
                if opcode == OP_PONG:
                    continue
                if opcode == OP_CLOSE:
                    ws.send_close()
                    break
                if opcode == OP_CONTINUATION:
                    fragment_buffer += payload
                elif opcode in (OP_TEXT, OP_BINARY):
                    fragment_buffer = payload
                else:
                    continue
                if not fin:
                    continue
                text = fragment_buffer.decode("utf-8", errors="ignore")
                fragment_buffer = b""
                self._handle_text(text)
        finally:
            with self._clients_lock:
                if ws in self.clients:
                    self.clients.remove(ws)
            ws.close()
            self._refresh_detail()
            self.log("info", f"WebSocket 机载端已断开：{ws.peer}")

    def _handle_text(self, text: str) -> None:
        for raw in (text.splitlines() or [text]):
            line = raw.strip()
            if not line:
                continue
            msg = _decode(line.encode("utf-8"))
            if msg is None:
                continue
            self.status["rx"] += 1
            self.status["last_rx"] = time.time()
            self.on_message(msg, self.name)

    # --------------------------------------------------------------- 发送
    def send(self, payload: Dict[str, Any]) -> bool:
        with self._clients_lock:
            clients = list(self.clients)
        if not clients:
            return False
        text = json.dumps(payload, ensure_ascii=False)
        ok = False
        for client in clients:
            if client.send_text(text):
                ok = True
            else:
                client.close()
                with self._clients_lock:
                    if client in self.clients:
                        self.clients.remove(client)
                self._refresh_detail()
        if ok:
            self.status["tx"] += 1
        return ok


class SerialLink(BaseLink):
    name = "serial"

    def __init__(self, on_message, port: str, baud: int):
        super().__init__(on_message)
        self.port = port
        self.baud = int(baud)
        self.ser = None
        self.thread: Optional[threading.Thread] = None

    def start(self) -> None:
        if not HAVE_SERIAL:
            self.status.update({"ok": False,
                                "detail": "缺少 pyserial（pip install pyserial）"})
            return
        try:
            self.ser = serial.Serial(self.port, self.baud, timeout=0.2)
        except Exception as exc:
            self.status.update({"ok": False, "detail": f"打开串口失败：{exc}"})
            return
        self.status.update({"ok": True, "detail": f"串口 {self.port} @{self.baud}"})
        self.thread = threading.Thread(target=self._loop, name="link-serial",
                                       daemon=True)
        self.thread.start()

    def _loop(self) -> None:
        buf = b""
        while not self._stop.is_set():
            try:
                data = self.ser.read(4096)
            except Exception:
                break
            if not data:
                continue
            buf += data
            while b"\n" in buf:
                line, buf = buf.split(b"\n", 1)
                msg = _decode(line)
                if msg is not None:
                    self.status["rx"] += 1
                    self.status["last_rx"] = time.time()
                    self.on_message(msg, self.name)

    def send(self, payload: Dict[str, Any]) -> bool:
        if not self.ser:
            return False
        try:
            self.ser.write((json.dumps(payload, ensure_ascii=False) + "\n")
                           .encode("utf-8"))
            self.status["tx"] += 1
            return True
        except Exception:
            return False


class MockLink(BaseLink):
    """把内置虚拟无人机接到同一条消息总线上。"""

    name = "mock"

    def __init__(self, on_message, drone):
        super().__init__(on_message)
        self.drone = drone
        self.status.update({"ok": False, "detail": "虚拟无人机未启动"})

    def start(self) -> None:
        self.drone.start()
        self.status.update({"ok": True, "detail": "内置虚拟无人机运行中"})
        self.drone.publish({
            "type": "px4_status", "state": "WAIT_GO",
            "mode": "OFFBOARD", "armed": False, "ekf_ok": True,
            "link_ok": True, "ready": 0, "mission_active": False,
            "takeoff_available": True,
        })

    def stop(self) -> None:
        super().stop()
        self.drone.stop()
        self.status.update({"ok": False, "detail": "已停止"})

    def send(self, payload: Dict[str, Any]) -> bool:
        if str(payload.get("type")) != "cmd":
            return False
        cmd = payload.get("cmd")
        if cmd in ("start_route", "px4_start"):
            self.drone.set_route(payload["route"], payload.get("target_no"))
            self.drone.publish({
                "type": "px4_status", "state": "FLIGHT",
                "mode": "OFFBOARD", "armed": True, "ekf_ok": True,
                "link_ok": True, "ready": 1, "mission_active": True,
                "takeoff_available": False,
            })
            return True
        if cmd == "land":
            self.drone.land_now()
            return True
        if cmd == "estop":
            self.drone.emergency_stop()
            return True
        return False


class LinkManager:
    def __init__(self, on_message: Callable[[Dict[str, Any], str], None]):
        self.on_message = on_message
        self.links: List[BaseLink] = []
        self.led_output: Optional[SerialLink] = None
        self.led_cfg: Dict[str, Any] = {}
        self._led_timer: Optional[threading.Timer] = None

    def add(self, link: BaseLink) -> None:
        self.links.append(link)

    def start(self) -> None:
        for link in self.links:
            try:
                link.start()
            except Exception as exc:  # 端口占用等
                link.status.update({"ok": False, "detail": f"启动失败：{exc}"})

    def stop(self) -> None:
        for link in self.links:
            link.stop()

    def send(self, payload: Dict[str, Any]) -> None:
        text = json.dumps(payload, ensure_ascii=False)
        if len(text) > 400:
            text = text[:400] + f"...(共{len(text)}字符)"
        for link in self.links:
            try:
                link.send(payload)
            except Exception:
                pass

    def status(self) -> Dict[str, Any]:
        return {link.name: dict(link.status) for link in self.links}

    # 可选：真实 LED 灯板
    def configure_led(self, cfg: Dict[str, Any]) -> None:
        self.led_cfg = cfg or {}
        if not self.led_cfg.get("enabled"):
            return
        if not HAVE_SERIAL:
            return
        try:
            self.led_output = SerialLink(lambda *_a: None,
                                         self.led_cfg["port"],
                                         self.led_cfg.get("baud", 115200))
            self.led_output.start()
        except Exception:
            self.led_output = None

    def led(self, on: bool) -> None:
        if self._led_timer:
            self._led_timer.cancel()
            self._led_timer = None
        if not self.led_output:
            return
        cmd = self.led_cfg.get("on_cmd", "LED1\n") if on \
            else self.led_cfg.get("off_cmd", "LED0\n")
        try:
            self.led_output.ser.write(cmd.encode("utf-8"))
        except Exception:
            pass
        if on:
            self._led_timer = threading.Timer(
                1.0, self.led, args=(False,))
            self._led_timer.daemon = True
            self._led_timer.start()
