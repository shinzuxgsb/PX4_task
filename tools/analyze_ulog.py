#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""飞控 ulog 复盘工具（重点：一次"炸机/异常"前后到底发生了什么）。

为什么要写它（2026-10-10）
--------------------------
那天下午的两次炸机，SD 卡里**只有早上的记录**（下午的没写进去）。所以早上那份
`log_70_*.ulg` 反而成了唯一一手证据，而且它和下午那次死在**同一个位置**
（前进段跑完、刚要掉头）。这个脚本就是把那份日志里最关键的几样东西直接摆出来：

  1. 日志里的**文字消息**（logged_messages）——"Preflight Fail / Armed by / Failsafe /
     Takeoff detected / Landing detected / Critical ..." 这些是飞控自己说的话，最值钱；
  2. **日志是怎么结束的**：正常是"落地→上锁"后收尾；硬掉电/复位是**戛然而止**；
  3. 最后 N 秒的**电池电压/电流、电机输出、姿态、飞控状态（nav_state/armed/failsafe）**；
  4. **掉帧/丢包**（dropouts）与数据话题清单（哪个话题先停）。

用法（香橙派上；pyulog 是纯 python，不用装 pip）：
    PYTHONPATH=/tmp/pyulog_pkg python3 tools/analyze_ulog.py /tmp/xxx.ulg
    PYTHONPATH=/tmp/pyulog_pkg python3 tools/analyze_ulog.py /tmp/xxx.ulg --tail 5

（pyulog 的获取方式见 tools/README 或 CHANGELOG：从 PyPI 拉 wheel、解压到 /tmp 即可，
  本脚本会自动把 /tmp/pyulog_pkg 加进 sys.path。）
"""

import argparse
import os
import sys

for _p in ('/tmp/pyulog_pkg',):
    if os.path.isdir(_p) and _p not in sys.path:
        sys.path.insert(0, _p)

try:
    from pyulog import ULog
except ImportError:  # pragma: no cover
    print('找不到 pyulog。香橙派上可以这样准备（不装 pip、不动系统包）：\n'
          '  mkdir -p /tmp/pyulog_pkg && cd /tmp\n'
          '  url=$(curl -s https://pypi.org/pypi/pyulog/json | python3 -c "import sys,json;'
          'print([u[\'url\'] for u in json.load(sys.stdin)[\'urls\'] if u[\'filename\'].endswith(\'.whl\')][0])")\n'
          '  curl -sL -o /tmp/pyulog.whl "$url" && python3 -m zipfile -e /tmp/pyulog.whl /tmp/pyulog_pkg\n'
          '  PYTHONPATH=/tmp/pyulog_pkg python3 tools/analyze_ulog.py <ulog>', file=sys.stderr)
    sys.exit(2)

# 想看的话题（有就打印，没有就跳过）
WATCH = [
    ('battery_status',        ['voltage_v', 'current_a', 'remaining', 'warning', 'cell_count']),
    ('vehicle_status',        ['nav_state', 'arming_state', 'failsafe', 'pre_flight_checks_pass']),
    ('vehicle_control_mode',  ['flag_armed', 'flag_control_offboard_enabled', 'flag_control_position_enabled']),
    ('vehicle_local_position', ['x', 'y', 'z', 'vz', 'dist_bottom', 'xy_valid', 'v_xy_valid', 'heading']),
    ('vehicle_attitude',      ['q[0]', 'q[1]', 'q[2]', 'q[3]']),
    ('actuator_motors',       ['control[0]', 'control[1]', 'control[2]', 'control[3]']),
    ('actuator_outputs',      ['output[0]', 'output[1]', 'output[2]', 'output[3]']),
    ('failsafe_flags',        ['offboard_control_signal_lost', 'manual_control_signal_lost',
                               'gcs_connection_lost', 'auto_mission_missing', 'battery_warning']),
]


def rpy_from_q(w, x, y, z):
    import math
    roll = math.atan2(2 * (w * x + y * z), 1 - 2 * (x * x + y * y))
    pitch = math.asin(max(-1.0, min(1.0, 2 * (w * y - z * x))))
    yaw = math.atan2(2 * (w * z + x * y), 1 - 2 * (y * y + z * z))
    return [v * 180.0 / math.pi for v in (roll, pitch, yaw)]


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('ulog')
    ap.add_argument('--tail', type=float, default=3.0, help='打印最后多少秒（默认 3s）')
    ap.add_argument('--step', type=int, default=10, help='每 N 帧取一行（默认 10）')
    args = ap.parse_args()

    u = ULog(args.ulog)
    dur = (u.last_timestamp - u.start_timestamp) / 1e6

    print('================ 基本信息 ================')
    print(f'文件: {args.ulog}')
    print(f'日志时长: {dur:.2f} s')
    for k in ('sys_name', 'ver_sw', 'ver_hw', 'time_ref_utc', 'flight_uuid'):
        v = u.msg_info_dict.get(k) if hasattr(u, 'msg_info_dict') else None
        if v is not None:
            print(f'  {k} = {v}')
    print(f'数据话题: {len(u.data_list)} 个；dropout(丢包/卡顿) 段: {len(u.dropouts)}')
    for d in u.dropouts:
        print(f'  dropout 持续时间 {d[0]/1e6:.2f}s @ t={(d[1]-u.start_timestamp)/1e6:.2f}s')

    print('\n================ 日志里的文字消息（飞控自己说的）================')
    if not u.logged_messages:
        print('  （无）')
    for m in u.logged_messages:
        rel = (m.timestamp - u.start_timestamp) / 1e6
        lvl = m.log_level_str() if hasattr(m, 'log_level_str') else str(getattr(m, 'log_level', ''))
        print(f'  [{rel:7.2f}s] {lvl:8s} {m.message}')

    print('\n================ 话题清单（末帧时刻，看谁先停）================')
    rows = []
    for d in u.data_list:
        ts = d.data.get('timestamp')
        if ts is None or len(ts) == 0:
            continue
        rows.append((d.name, d.multi_id, len(ts), (ts[0] - u.start_timestamp) / 1e6,
                     (ts[-1] - u.start_timestamp) / 1e6))
    for r in sorted(rows):
        print(f'  {r[0]:28s} id={r[1]}  {r[2]:6d} 帧   [{r[3]:7.2f}s .. {r[4]:7.2f}s]')

    print(f'\n================ 最后 {args.tail:g} 秒的关键量 ================')
    t_end = u.last_timestamp
    t0 = t_end - int(args.tail * 1e6)
    for name, fields in WATCH:
        d = next((x for x in u.data_list if x.name == name), None)
        if d is None:
            continue
        ts = d.data['timestamp']
        idx = [i for i in range(len(ts)) if ts[i] >= t0]
        if not idx:
            continue
        have = [f for f in fields if f in d.data]
        print(f'\n--- {name}（{have and "有" or "无"}关注字段；共 {len(idx)} 帧，每 {args.step} 帧取一行）---')
        print('   t(s)  ' + '  '.join(f'{f[:14]:>14s}' for f in have))
        for i in idx[::args.step]:
            vals = []
            for f in have:
                v = d.data[f][i]
                vals.append(f'{float(v):14.4f}')
            print(f'  {(ts[i]-u.start_timestamp)/1e6:7.2f}  ' + '  '.join(vals))
        # 姿态额外换算成角度
        if name == 'vehicle_attitude' and all(k in d.data for k in ('q[0]', 'q[1]', 'q[2]', 'q[3]')):
            print('   最后 3 帧的 横滚/俯仰/航向(deg):')
            for i in idx[-3::1]:
                r, p, y = rpy_from_q(*(float(d.data[k][i]) for k in ('q[0]', 'q[1]', 'q[2]', 'q[3]')))
                print(f'    t={(ts[i]-u.start_timestamp)/1e6:7.2f}s  roll={r:8.2f}  pitch={p:8.2f}  yaw={y:8.2f}')


if __name__ == '__main__':
    main()
