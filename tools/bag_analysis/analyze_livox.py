#!/usr/bin/env python3
"""Анализ /livox/lidar (CustomMsg) и /livox/imu из rosbag2 без ROS.

    python3 tools/bag_analysis/analyze_livox.py <папка_bag>

Проверяет: point_num против длины массива, долю точек нулевой дальности, теги, частоту и пропуски
IMU, расхождение header.stamp со временем записи.
"""

import argparse
import collections
import os
import sys

import numpy as np

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from bagreader import open_bag, parse_custommsg, parse_imu  # noqa: E402


def analyze_lidar(bag_dir, topic):
    print(f'--- {topic}')
    count = mismatches = total = zero = 0
    zero_by_tag = collections.Counter()
    stamps, bag_times = [], []
    frames = set()
    for stamp_ns, buf in open_bag(bag_dir, topic):
        msg = parse_custommsg(buf)
        pts = np.array(msg['points'], dtype=float).reshape(-1, 7)
        count += 1
        mismatches += int(msg['point_num'] != len(pts))
        ranges = np.linalg.norm(pts[:, 1:4], axis=1)
        total += len(pts)
        zero += int((ranges == 0).sum())
        for tag in pts[ranges == 0][:, 5].astype(int):
            zero_by_tag[int(tag)] += 1
        stamps.append(msg['stamp'])
        bag_times.append(stamp_ns * 1e-9)
        frames.add(msg['frame'])
    if not count:
        print('сообщений нет')
        return
    print(f'сообщений: {count}; frame_id: {sorted(frames)}; point_num != len(points): {mismatches}')
    print(f'точек: {total}, нулевой дальности: {zero} ({100 * zero / max(total, 1):.1f}%), по тегам: '
          f'{dict(sorted(zero_by_tag.items()))}')
    stamps, bag_times = np.array(stamps), np.array(bag_times)
    print(f'паузы между кадрами по header.stamp, с: {np.diff(stamps).round(3)}')
    lag = bag_times - stamps
    print(f'время записи - header.stamp, с: от {lag.min():.2f} до {lag.max():.2f}')


def analyze_imu(bag_dir, topic):
    print(f'--- {topic}')
    stamps, bag_times, accs = [], [], []
    frames = set()
    for stamp_ns, buf in open_bag(bag_dir, topic):
        msg = parse_imu(buf)
        stamps.append(msg['stamp'])
        bag_times.append(stamp_ns * 1e-9)
        accs.append(msg['acc'])
        frames.add(msg['frame'])
    if not stamps:
        print('сообщений нет')
        return
    stamps, bag_times, accs = np.array(stamps), np.array(bag_times), np.array(accs)
    gaps = np.diff(stamps)
    rate = (len(stamps) - 1) / (stamps[-1] - stamps[0])
    print(f'сообщений: {len(stamps)}; frame_id: {sorted(frames)}; частота по header.stamp: {rate:.1f} Гц')
    print(f'пауз длиннее 15 мс: {int((gaps > 0.015).sum())}, самая длинная {gaps.max() * 1e3:.0f} мс')
    lag = bag_times - stamps
    print(f'время записи - header.stamp, с: от {lag.min():.2f} до {lag.max():.2f}')
    mean_acc = accs.mean(axis=0)
    print(f'среднее ускорение по осям: {mean_acc.round(3)} (модуль {np.linalg.norm(mean_acc):.3f}: '
          'около 1 значит единицы g, около 9.8 значит м/с^2)')


def main():
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawTextHelpFormatter)
    parser.add_argument('bag', help='папка rosbag2')
    parser.add_argument('--lidar-topic', default='/livox/lidar')
    parser.add_argument('--imu-topic', default='/livox/imu')
    args = parser.parse_args()
    analyze_lidar(args.bag, args.lidar_topic)
    analyze_imu(args.bag, args.imu_topic)


if __name__ == '__main__':
    main()
