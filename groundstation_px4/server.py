#!/usr/bin/env python3
"""PX4 立体货架盘点无人机 - 地面站主程序。

用法：
    python server.py                 # 使用 config/ 下的配置启动
    python server.py --mock          # 同时启动内置虚拟无人机（演示/自测）
    python server.py --port 8765 --mock --time-scale 4
"""

from __future__ import annotations

import argparse
import os
import socket
import subprocess
import sys
import threading
import time
import webbrowser

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

from gs import config as cfgmod  # noqa: E402
from gs.server import serve  # noqa: E402


BROWSER_CANDIDATES = [
    r"C:\Program Files (x86)\Microsoft\Edge\Application\msedge.exe",
    r"C:\Program Files\Microsoft\Edge\Application\msedge.exe",
    r"C:\Program Files\Google\Chrome\Application\chrome.exe",
    r"C:\Program Files (x86)\Google\Chrome\Application\chrome.exe",
]


def local_ip() -> str:
    """本机在局域网里的地址（不会真的发包，只用来选路由）。"""
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    try:
        sock.connect(("8.8.8.8", 80))
        return sock.getsockname()[0]
    except OSError:
        try:
            return socket.gethostbyname(socket.gethostname())
        except OSError:
            return "127.0.0.1"
    finally:
        sock.close()


def open_browser(url: str) -> str:
    """优先用 Edge/Chrome 打开界面；失败时回退到系统默认程序。"""
    for path in BROWSER_CANDIDATES:
        if os.path.exists(path):
            try:
                subprocess.Popen([path, url], close_fds=True)
                return os.path.basename(path)
            except Exception:
                continue
    try:
        webbrowser.open(url)
        return "default"
    except Exception as exc:  # pragma: no cover
        print(f"  请手动在浏览器打开：{url}（自动打开失败：{exc}）")
        return "manual"


def main() -> int:
    try:
        sys.stdout.reconfigure(encoding="utf-8")  # type: ignore[attr-defined]
    except Exception:
        pass
    parser = argparse.ArgumentParser(description="立体货架盘点无人机系统 - 地面站")
    parser.add_argument("--host", default=None, help="HTTP 监听地址")
    parser.add_argument("--port", type=int, default=None, help="HTTP 端口")
    parser.add_argument("--mock", action="store_true", help="启用内置虚拟无人机")
    parser.add_argument("--no-mock", action="store_true", help="关闭内置虚拟无人机")
    parser.add_argument("--time-scale", type=float, default=None,
                        help="虚拟无人机时间倍率（演示用，1 为真实速度）")
    parser.add_argument("--camera-url", default=None,
                        help="机载 MJPEG 地址，例如 http://10.249.123.10:8080/stream")
    parser.add_argument("--open", action="store_true", help="启动后打开浏览器")
    args = parser.parse_args()

    runtime = cfgmod.load_runtime()
    if args.host:
        runtime["http"]["host"] = args.host
    if args.port:
        runtime["http"]["port"] = int(args.port)
    if args.camera_url:
        runtime.setdefault("camera", {})["url"] = args.camera_url
        runtime["camera"]["enabled"] = True
    force_mock = True if args.mock else (False if args.no_mock else None)

    def announce(gs, host, port):
        url = f"http://{host}:{port}/"
        print("=" * 62)
        print("  PX4 货架盘点地面站")
        print(f"  界面地址 : {url}")
        print(f"  场地     : {gs.field_cfg.get('name')}")
        print(f"  盘点点位 : {len(gs.field.slots)} 个 "
              f"({','.join(f['id'] for f in gs.field.faces)})")
        links = [k for k, v in gs.links.status().items() if v.get("ok")]
        print(f"  通信链路 : {', '.join(links) if links else '无'}")
        camera_cfg = gs.runtime.get("camera", {})
        print(f"  相机画面 : {camera_cfg.get('url', '未配置')}")
        ws_cfg = gs.runtime.get("links", {}).get("websocket", {})
        if ws_cfg.get("enabled"):
            path = ws_cfg.get("path", "/ws") or "/ws"
            print(f"  WebSocket: ws://{local_ip()}:{ws_cfg.get('port', 8890)}{path}"
                  "   <- 机载端 ws_url 填这个")
        print(f"  虚拟无人机: {'开' if gs.mock_cfg.get('enabled') else '关'}")
        print("=" * 62)
        print("  Ctrl+C 退出")
        if args.open:
            def _open():
                who = open_browser(url)
                if who == "manual":
                    print(f"  >>> 请手动打开浏览器访问 {url}")
            threading.Timer(0.6, _open).start()

    httpd = serve(runtime, force_mock=force_mock, time_scale=args.time_scale,
                  announce=announce)
    try:
        httpd.serve_forever(poll_interval=0.2)
    except KeyboardInterrupt:
        print("\n正在保存本次数据 ...")
        try:
            path = httpd.RequestHandlerClass.gs.state.save_session()
            print(f"已保存：{path}")
        except Exception as exc:
            print(f"保存失败：{exc}")
    finally:
        try:
            httpd.RequestHandlerClass.gs.links.stop()
        except Exception:
            pass
        httpd.server_close()
        time.sleep(0.1)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
