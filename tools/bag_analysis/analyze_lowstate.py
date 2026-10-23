#!/usr/bin/env python3
"""Анализ IMU внутри /lowstate из rosbag2 без ROS.

    python3 tools/bag_analysis/analyze_lowstate.py [--static] <папка_bag> [<папка_bag> ...]

Проверяет: частоту, норму кватерниона, нулевые кватернионы, порядок компонент (сравнение
с полем rpy из того же сообщения) и даёт грубую оценку шума по самому «тихому» окну в 1 с.

Ключ --static: запись сделана при неподвижном роботе; печатает среднее и дисперсию по всей записи
(гироскоп, акселерометр, углы) и строки с готовыми значениями для параметров imu_converter.
Углы крена, тангажа и рыскания берутся из поля rpy; дисперсия угла включает дрейф за время записи.
"""

import argparse
import math
import os
import sys

import numpy as np

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from bagreader import open_bag, parse_lowstate  # noqa: E402


def euler_from_wxyz(w, x, y, z):
    """Крен, тангаж, рыскание (рад) из кватерниона (w, x, y, z)."""
    roll = math.atan2(2 * (w * x + y * z), 1 - 2 * (x * x + y * y))
    pitch = math.asin(max(-1.0, min(1.0, 2 * (w * y - z * x))))
    yaw = math.atan2(2 * (w * z + x * y), 1 - 2 * (y * y + z * z))
    return roll, pitch, yaw


def max_angle_error(euler, rpy):
    diff = (euler - rpy + np.pi) % (2 * np.pi) - np.pi
    return np.abs(diff).max(axis=0)


def static_report(gyros, accs, rpys):
    """Среднее и дисперсия по всей записи неподвижного робота + строки для параметров."""
    gyro_var = gyros.var(axis=0, ddof=1)
    acc_var = accs.var(axis=0, ddof=1)
    angle_var = np.unwrap(rpys, axis=0).var(axis=0, ddof=1)
    print('--- статика: вся запись')
    print(f'гироскоп: среднее (смещение) {gyros.mean(axis=0).round(5)} рад/с')
    print(f'  дисперсия {gyro_var} (рад/с)^2')
    acc_mean = accs.mean(axis=0)
    print(f'акселерометр: среднее {acc_mean.round(4)} м/с^2 (модуль {np.linalg.norm(acc_mean):.3f})')
    print(f'  дисперсия {acc_var} (м/с^2)^2')
    print(f'углы крен/тангаж/рыскание, дисперсия {angle_var} рад^2 (включает дрейф)')
    print('значения для параметров (максимум по осям, округление до 2 значащих цифр):')
    for name, value in (('orientation_variance', angle_var.max()),
                        ('angular_velocity_variance', gyro_var.max()),
                        ('linear_acceleration_variance', acc_var.max())):
        print(f'  {name}: {float(f"{value:.2g}")}')
    print('Робот должен был быть неподвижен всю запись; если нет, значения завышены.')


def analyze(bag_dir, window=500, static=False):
    stamps, ticks, quats, gyros, accs, rpys = [], [], [], [], [], []
    for stamp_ns, buf in open_bag(bag_dir, '/lowstate'):
        msg = parse_lowstate(buf)
        stamps.append(stamp_ns * 1e-9)
        ticks.append(msg['tick'])
        quats.append(msg['quat'])
        gyros.append(msg['gyro'])
        accs.append(msg['acc'])
        rpys.append(msg['rpy'])
    arrays = (stamps, ticks, quats, gyros, accs, rpys)
    stamps, ticks, quats, gyros, accs, rpys = map(np.array, arrays)
    print(f'== {bag_dir}: {len(stamps)} сообщений')

    dt = np.diff(stamps)
    print(f'частота (время записи): {1 / np.median(dt):.1f} Гц, макс. пауза {dt.max() * 1e3:.0f} мс')
    print(f'шаг tick: {np.median(np.diff(ticks)):.0f}, повторов tick: {int((np.diff(ticks) == 0).sum())}')

    norm = np.linalg.norm(quats, axis=1)
    print(f'норма кватерниона: {norm.min():.6f} .. {norm.max():.6f}; нулевых: {int((norm < 1e-3).sum())}; '
          f'NaN: {int(np.isnan(quats).sum())}')

    as_wxyz = np.array([euler_from_wxyz(*q) for q in quats])
    as_xyzw = np.array([euler_from_wxyz(q[3], q[0], q[1], q[2]) for q in quats])
    print('макс. расхождение с полем rpy, рад (крен, тангаж, рыскание):')
    print(f'  кватернион как (w,x,y,z): {max_angle_error(as_wxyz, rpys)}')
    print(f'  кватернион как (x,y,z,w): {max_angle_error(as_xyzw, rpys)}')

    best = None
    for start in range(0, len(gyros) - window, 50):
        part = gyros[start:start + window]
        score = part.std(axis=0).sum() + np.abs(part.mean(axis=0)).sum()
        if best is None or score < best[0]:
            best = (score, start)
    start = best[1]
    print(f'самое тихое окно 1 с: с {stamps[start] - stamps[0]:.2f} с от начала записи')
    print(f'  гироскоп: среднее {gyros[start:start + window].mean(axis=0).round(4)}, '
          f'СКО {gyros[start:start + window].std(axis=0).round(4)} рад/с')
    print(f'  акселерометр: СКО {accs[start:start + window].std(axis=0).round(4)} м/с^2')
    print('  Это верхняя оценка шума: робот в этот момент мог стоять и балансировать, а не быть неподвижным.')
    if static:
        static_report(gyros, accs, rpys)


def main():
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawTextHelpFormatter)
    parser.add_argument('bags', nargs='+', help='папки rosbag2 с топиком /lowstate')
    parser.add_argument('--static', action='store_true',
                        help='запись неподвижного робота: дисперсии по всей записи')
    args = parser.parse_args()
    for bag_dir in args.bags:
        analyze(bag_dir, static=args.static)


if __name__ == '__main__':
    main()
