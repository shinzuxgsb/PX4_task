#!/usr/bin/env python3
"""bag_crash_report.py —— 直接从 rosbag2 的 .db3 读，**不需要 metadata.yaml**。

背景：机上"黑匣子"（run_p2p.sh → tools/record_flight.sh）在服务被 kill/重启时来不及写
metadata.yaml，于是 `ros2 bag info/play` 会报 "Could not find metadata in bag directory"。
本工具用 sqlite3 直接打开 .db3、用 rclpy 反序列化，照样能复盘。

用法：
    source /opt/ros/humble/setup.bash; source ~/px4_ws/install/setup.bash
    python3 ~/px4_ws/tools/bag_crash_report.py ~/flights/20261013_180742
    python3 ~/px4_ws/tools/bag_crash_report.py ~/flights/xxx.db3 --csv /tmp/bag.csv

输出：① 话题清单（条数/首末墙钟时间）② 状态变化时间线 ③ 每类关键量的极值/末值。
"""
import argparse
import csv
import datetime
import glob
import os
import sqlite3
import sys

import rclpy.serialization
from rosidl_runtime_py.utilities import get_message


def wall(ns):
    return datetime.datetime.fromtimestamp(ns / 1e9).strftime('%H:%M:%S.%f')[:-3]


def find_db3(p):
    if os.path.isdir(p):
        f = sorted(glob.glob(os.path.join(p, '*.db3')))
        if not f:
            sys.exit('目录里没有 .db3：' + p)
        return f[0]
    return p


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('bag')
    ap.add_argument('--csv', default=None)
    args = ap.parse_args()

    db = find_db3(args.bag)
    con = sqlite3.connect('file:%s?mode=ro' % db, uri=True)
    cur = con.cursor()
    tcols = [r[1] for r in cur.execute('PRAGMA table_info(topics)')]
    sel = 'id,name,type' + (',offered_qos_profiles' if 'offered_qos_profiles' in tcols else '')
    topics, rows = {}, []
    for r in cur.execute('select %s from topics' % sel):
        topics[r[1]] = (r[0], r[2])
        n, t0, t1 = cur.execute(
            'select count(*),min(timestamp),max(timestamp) from messages where topic_id=?',
            (r[0],)).fetchone()
        rows.append((r[1], r[2], n or 0, t0, t1))

    print('== 黑匣子 %s' % db)
    print('== 话题清单（条数 / 首 → 末）')
    for name, typ, n, t0, t1 in rows:
        if n:
            print('  %-38s %7d  %s → %s' % (name, n, wall(t0), wall(t1)))
        else:
            print('  %-38s %7d' % (name, 0))

    def load(topic):
        if topic not in topics:
            return []
        tid, typ = topics[topic]
        cls = get_message(typ)
        out = []
        for ts, data in cur.execute(
                'select timestamp,data from messages where topic_id=? order by timestamp', (tid,)):
            try:
                out.append((ts, rclpy.serialization.deserialize_message(data, cls)))
            except Exception:
                pass
        return out

    events = []
    csv_rows = []

    st = load('/fmu/out/vehicle_status')
    last = None
    for ts, m in st:
        key = (getattr(m, 'arming_state', None), getattr(m, 'nav_state', None),
               getattr(m, 'failsafe', None), getattr(m, 'pre_flight_checks_pass', None))
        if key != last:
            events.append((ts, 'vehicle_status',
                           'arming=%s nav=%s failsafe=%s pre_chk=%s' % key))
            last = key
    if st:
        m = st[-1][1]
        print('\n== vehicle_status 末值：arming=%s nav=%s failsafe=%s pre_chk=%s' % (
            getattr(m, 'arming_state', None), getattr(m, 'nav_state', None),
            getattr(m, 'failsafe', None), getattr(m, 'pre_flight_checks_pass', None)))

    ff = load('/fmu/out/failsafe_flags')
    if ff:
        fields = [f for f in ff[0][1].get_fields_and_field_types()]
        seen = set()
        for ts, m in ff:
            on = tuple(f for f in fields if getattr(m, f, 0) in (True, 1))
            if on != seen:
                if on:
                    events.append((ts, 'failsafe_flags', ','.join(on)))
                seen = on
        print('== failsafe_flags 出现过的标志：%s' % (sorted(seen) or '（全程无）'))

    bat = load('/fmu/out/battery_status')
    if bat:
        vs = [m.voltage_v for _, m in bat if hasattr(m, 'voltage_v')]
        cs = [m.current_a for _, m in bat if hasattr(m, 'current_a')]
        print('== 电池：电压 min %.2f / max %.2f V；电流 max %.1f A；末值 %.2f V %.1f A' % (
            min(vs), max(vs), max(cs), vs[-1], cs[-1]))
        print('   放电最低点出现在 %s' % wall(bat[[m.voltage_v for _, m in bat].index(min(vs))][0]))

    lp = load('/fmu/out/vehicle_local_position')
    if lp:
        zs = [m.z for _, m in lp]
        zmin, zmax = min(zs), max(zs)
        print('== local_position：z 范围 %.2f → %.2f m；末值 z=%.2f v_z=%.2f xy_valid=%s z_valid=%s' % (
            zmin, zmax, lp[-1][1].z, getattr(lp[-1][1], 'vz', float('nan')),
            getattr(lp[-1][1], 'xy_valid', None), getattr(lp[-1][1], 'z_valid', None)))
        for ts, m in lp:
            if m.z >= zmax - 1e-6:
                print('   最高点 %.2f m 出现在 %s' % (zmax, wall(ts)))
                break
        csv_rows += [(ts, 'lpos', m.x, m.y, m.z) for ts, m in lp]

    mcs = load('/fmu/out/manual_control_setpoint')
    if mcs:
        th = [m.throttle for _, m in mcs]
        print('== manual_control_setpoint：throttle 范围 %.2f → %.2f；末值 %.2f；valid=%s' % (
            min(th), max(th), th[-1], getattr(mcs[-1][1], 'valid', None)))
        for f in ('kill_switch', 'mode_slot', 'switches'):
            if hasattr(mcs[0][1], f):
                print('   %s：%s' % (f, [getattr(m, f) for _, m in mcs[-3:]]))

    att = load('/fmu/out/vehicle_attitude')
    if att:
        import math
        def rp(m):
            q = m.q
            roll = math.atan2(2 * (q[0] * q[1] + q[2] * q[3]), 1 - 2 * (q[1] ** 2 + q[2] ** 2))
            pitch = math.asin(max(-1, min(1, 2 * (q[0] * q[2] - q[3] * q[1]))))
            return math.degrees(roll), math.degrees(pitch)
        rps = [rp(m) for _, m in att]
        print('== 姿态：最大 |roll| %.0f° / |pitch| %.0f°；末值 roll %.0f° pitch %.0f°' % (
            max(abs(r) for r, _ in rps), max(abs(p) for _, p in rps), *rps[-1]))

    tof = load('/tofsense/height')
    if tof:
        hs = [getattr(m, 'data', None) if hasattr(m, 'data') else getattr(m, 'height', None)
              for _, m in tof]
        hs = [h for h in hs if h is not None]
        if hs:
            print('== /tofsense/height：%s 帧；min %.3f max %.3f' % (len(hs), min(hs), max(hs)))

    ev = load('/fmu/in/vehicle_visual_odometry')
    if ev:
        print('== EV：%d 帧（%.1f Hz），pose_frame=%s quality=%s' % (
            len(ev),
            len(ev) / max(1e-9, (ev[-1][0] - ev[0][0]) / 1e9),
            getattr(ev[-1][1], 'pose_frame', None), getattr(ev[-1][1], 'quality', None)))

    print('\n== 状态变化时间线（vehicle_status / failsafe_flags）')
    for ts, who, what in events[:200]:
        print('  %s  %-16s %s' % (wall(ts), who, what))

    if args.csv:
        with open(args.csv, 'w', newline='') as fh:
            w = csv.writer(fh)
            w.writerow(['wall', 'ns', 'what', 'a', 'b', 'c'])
            for ts, what, *vals in csv_rows:
                w.writerow([wall(ts), ts, what] + list(vals))
        print('\n（明细已写 %s）' % args.csv)


if __name__ == '__main__':
    main()