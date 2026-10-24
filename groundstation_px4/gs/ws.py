"""WebSocket（RFC 6455）服务端：标准库实现，不依赖第三方包。

地面站做服务端、机载端做客户端：机载端连上后，双方各发“一条 JSON = 一个文本帧”，
与 UDP / TCP / 数传串口 完全等价（见 PROTOCOL.md）。

只实现项目用得到的部分：
    - 握手（Sec-WebSocket-Key -> Sec-WebSocket-Accept）
    - 文本/二进制帧收发、分片续帧
    - ping/pong 保活、close 收尾
不实现：TLS(wss)、扩展协商（permessage-deflate）。
"""

from __future__ import annotations

import base64
import hashlib
import socket
import struct
import threading
import time
from typing import Optional, Tuple

WS_GUID = "258EAFA5-E914-47DA-95CA-C5AB0DC85B11"

OP_CONTINUATION = 0x0
OP_TEXT = 0x1
OP_BINARY = 0x2
OP_CLOSE = 0x8
OP_PING = 0x9
OP_PONG = 0xA

#: 单帧上限：整条 start_route（含 legs/waypoints）约 30KB，4MB 足够宽松
MAX_FRAME_BYTES = 4 * 1024 * 1024
#: 握手请求头上限
MAX_HEADER_BYTES = 16 * 1024


class WebSocketError(Exception):
    """协议错误 / 连接中断。"""


def accept_key(client_key: str) -> str:
    """按 RFC 6455 由 Sec-WebSocket-Key 计算 Sec-WebSocket-Accept。"""
    digest = hashlib.sha1((client_key + WS_GUID).encode("ascii", "ignore")).digest()
    return base64.b64encode(digest).decode("ascii")


def _xor_mask(data: bytes, key: bytes) -> bytes:
    if not key:
        return data
    size = len(key)
    return bytes(byte ^ key[index % size] for index, byte in enumerate(data))


def _parse_query(query: str) -> dict:
    out = {}
    for item in (query or "").split("&"):
        if not item or "=" not in item:
            continue
        name, _, value = item.partition("=")
        out[name.strip().lower()] = value.strip()
    return out


class WebSocketConnection:
    """一条已建立（或待建立）的 WebSocket 连接，服务端角色。"""

    def __init__(self, conn: socket.socket, addr) -> None:
        self.conn = conn
        self.addr = addr
        self.peer = f"{addr[0]}:{addr[1]}"
        self.path = ""
        self.closed = False
        self.last_rx = time.time()
        self._buf = b""
        self._send_lock = threading.Lock()

    # ------------------------------------------------------------- 握手
    def handshake(self, expect_path: str = "", token: str = "") -> bool:
        header = self._read_headers()
        if header is None:
            return False
        text = header.decode("latin-1", errors="ignore")
        lines = text.split("\r\n")
        if not lines or not lines[0].upper().startswith("GET "):
            self._reject(400, "Bad Request")
            return False
        parts = lines[0].split()
        if len(parts) < 2:
            self._reject(400, "Bad Request")
            return False
        target = parts[1]
        path, _, query = target.partition("?")
        self.path = path

        headers = {}
        for line in lines[1:]:
            if ":" in line:
                name, _, value = line.partition(":")
                headers[name.strip().lower()] = value.strip()

        if "websocket" not in headers.get("upgrade", "").lower():
            self._reject(400, "Not a WebSocket handshake")
            return False
        key = headers.get("sec-websocket-key")
        if not key:
            self._reject(400, "Missing Sec-WebSocket-Key")
            return False
        if expect_path and path != expect_path:
            self._reject(404, "Not Found")
            return False
        if token:
            supplied = headers.get("x-token", "") or _parse_query(query).get("token", "")
            if supplied != token:
                self._reject(401, "Unauthorized")
                return False

        response = (
            "HTTP/1.1 101 Switching Protocols\r\n"
            "Upgrade: websocket\r\n"
            "Connection: Upgrade\r\n"
            f"Sec-WebSocket-Accept: {accept_key(key)}\r\n"
            "\r\n"
        )
        try:
            self.conn.sendall(response.encode("latin-1"))
        except OSError:
            return False
        return True

    def _read_headers(self) -> Optional[bytes]:
        buf = self._buf
        while b"\r\n\r\n" not in buf:
            try:
                chunk = self.conn.recv(4096)
            except (socket.timeout, OSError):
                return None
            if not chunk:
                return None
            buf += chunk
            if len(buf) > MAX_HEADER_BYTES:
                self._reject(431, "Request Header Fields Too Large")
                return None
        header, _, rest = buf.partition(b"\r\n\r\n")
        self._buf = rest          # 同一包里可能已经带着第一帧
        return header

    def _reject(self, code: int, reason: str) -> None:
        body = f"{code} {reason}\n".encode("utf-8")
        try:
            self.conn.sendall(
                f"HTTP/1.1 {code} {reason}\r\n"
                "Connection: close\r\n"
                "Content-Type: text/plain; charset=utf-8\r\n"
                f"Content-Length: {len(body)}\r\n\r\n".encode("latin-1") + body
            )
        except OSError:
            pass

    # ------------------------------------------------------------- 收帧
    def _read_exact(self, count: int) -> bytes:
        while len(self._buf) < count:
            try:
                chunk = self.conn.recv(max(4096, count - len(self._buf)))
            except socket.timeout:
                raise
            except OSError as exc:
                raise WebSocketError(str(exc)) from exc
            if not chunk:
                raise WebSocketError("连接已关闭")
            self._buf += chunk
        data, self._buf = self._buf[:count], self._buf[count:]
        return data

    def read_frame(self) -> Tuple[int, bytes, bool]:
        """读一帧，返回 (opcode, payload, fin)。

        连接中断抛 WebSocketError；读超时抛 socket.timeout
        （已收到的半帧数据保留在缓冲区，下次继续）。
        """
        head = self._read_exact(2)
        first, second = head[0], head[1]
        fin = bool(first & 0x80)
        opcode = first & 0x0F
        masked = bool(second & 0x80)
        length = second & 0x7F
        if length == 126:
            length = struct.unpack("!H", self._read_exact(2))[0]
        elif length == 127:
            length = struct.unpack("!Q", self._read_exact(8))[0]
        if length > MAX_FRAME_BYTES:
            raise WebSocketError(f"单帧过大：{length} 字节")
        if masked:
            key = self._read_exact(4)
        else:
            # 客户端 -> 服务端必须掩码，否则按协议错误断开
            raise WebSocketError("客户端帧未掩码")
        payload = self._read_exact(length) if length else b""
        if masked:
            payload = _xor_mask(payload, key)
        return opcode, payload, fin

    # ------------------------------------------------------------- 发帧
    def send_frame(self, opcode: int, payload: bytes = b"") -> bool:
        size = len(payload)
        header = bytearray()
        header.append(0x80 | (opcode & 0x0F))
        if size < 126:
            header.append(size)
        elif size < 65536:
            header.append(126)
            header += struct.pack("!H", size)
        else:
            header.append(127)
            header += struct.pack("!Q", size)
        with self._send_lock:
            if self.closed:
                return False
            try:
                self.conn.sendall(bytes(header) + payload)
                return True
            except OSError:
                self.closed = True
                return False

    def send_text(self, text: str) -> bool:
        return self.send_frame(OP_TEXT, text.encode("utf-8"))

    def send_ping(self, payload: bytes = b"") -> bool:
        return self.send_frame(OP_PING, payload)

    def send_close(self, code: int = 1000) -> bool:
        try:
            return self.send_frame(OP_CLOSE, struct.pack("!H", code))
        except struct.error:  # pragma: no cover
            return False

    def close(self) -> None:
        with self._send_lock:
            self.closed = True
        try:
            self.conn.shutdown(socket.SHUT_RDWR)
        except OSError:
            pass
        try:
            self.conn.close()
        except OSError:
            pass
