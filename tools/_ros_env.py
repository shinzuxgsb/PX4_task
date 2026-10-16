#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""让同目录下的 python 工具「不用先 source 也能跑」。

为什么需要它（2026-10-10 用户实际踩到）
---------------------------------------
新开的 shell 里直接跑
    python3 ~/px4_ws/tools/watch_fc_link.py --minutes 15
得到
    ModuleNotFoundError: No module named 'px4_msgs'
当时这台机器的 ~/.bashrc 不会自动 source ROS，必须自己先
    source /opt/ros/humble/setup.bash
    source ~/px4_ws/install/setup.bash
（**19:20 起 ~/.bashrc 已经自动 source 了**，所以交互式 shell 不再需要手敲；
  但下面这些场合本来就不读 .bashrc，仍然靠本模块兜底）
    · `ssh orangepi@… 'python3 …'`（非交互非登录 shell，不读 .bashrc）
    · systemd / cron / 别的脚本拉起来的 python
    · 万一 .bashrc 被改坏 / 工作区被挪走

做什么
------
* 环境已就绪（`AMENT_PREFIX_PATH` 有值、`sys.path` 里有 px4_msgs）→ 什么都不做；
* 否则用 bash **重新起一次自己**（命令行参数原样带过去，只重试一次，用环境变量防递归），
  失败时先打一句人话提示，而不是甩一屏 traceback；
* 顺手保证 FastDDS 走 UDP（如果没设 profile）——免得又踩"话题看得见、数据收不到"的坑。

用法：在其它脚本的最前面（任何 ROS 相关 import 之前）加一行
    import _ros_env  # noqa: F401
"""

import os
import shlex
import sys

_WS = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))   # …/px4_ws
_ROS_SETUP = '/opt/ros/humble/setup.bash'
_WS_SETUP = os.path.join(_WS, 'install', 'setup.bash')
_UDP_PROFILE = os.path.join(_WS, 'tools', 'fastdds_udp_only.xml')
_WRAP_FLAG = '_ROS_ENV_WRAPPED'


def env_ok() -> bool:
    """ROS 环境（含工作区里的 px4_msgs）是否已经可用。"""
    if not os.environ.get('AMENT_PREFIX_PATH'):
        return False
    return any('px4_msgs' in p for p in sys.path)


def _force_udp() -> None:
    if not os.environ.get('FASTRTPS_DEFAULT_PROFILES_FILE') and os.path.exists(_UDP_PROFILE):
        os.environ['FASTRTPS_DEFAULT_PROFILES_FILE'] = _UDP_PROFILE
        print(f'[环境] 未设 FastDDS profile → 本进程强制 UDP：{_UDP_PROFILE}', file=sys.stderr)


def ensure() -> None:
    """环境不对就用 bash 重起本脚本（参数保留）；否则什么都不做。"""
    if env_ok():
        _force_udp()
        return

    if os.environ.get(_WRAP_FLAG) == '1':
        print('[环境] 还是找不到 px4_msgs。请先手动执行：\n'
              f'    source {_ROS_SETUP}\n'
              f'    source {_WS_SETUP}\n'
              '然后再跑本脚本。', file=sys.stderr)
        return

    if not os.path.exists(_WS_SETUP):
        print(f'[环境] 找不到 {_WS_SETUP}（工作区路径不对？）', file=sys.stderr)
        return

    script = os.path.abspath(sys.argv[0])
    cmd = (f'source {shlex.quote(_ROS_SETUP)} >/dev/null 2>&1; '
           f'source {shlex.quote(_WS_SETUP)} >/dev/null 2>&1; '
           f'export {_WRAP_FLAG}=1; '
           f'exec python3 {shlex.quote(script)} "$@"')
    print('[环境] 当前 shell 没有 ROS 环境，自动 source 后重启本脚本…', file=sys.stderr)
    os.execvp('bash', ['bash', '-c', cmd, 'bash', *sys.argv[1:]])


ensure()
