#!/usr/bin/env python3
# Copyright (c) 2025 Alice Zenina and Alexander Grachev RTU MIREA (Russia)
# SPDX-License-Identifier: MIT

"""Оценка наклона пола в облаке лидара и высоты лидара над полом.

Робот стоит на ровном полу, вокруг свободное место. Скрипт собирает несколько облаков
`sensor_msgs/PointCloud2`, берёт точки ниже лидара, находит плоскость пола и печатает:
  * наклон пола относительно осей x и y кадра облака (в нём пол должен быть горизонтален);
  * расстояние от начала координат до плоскости (высота лидара над полом).

Если наклон заметно больше 1-2 градусов, подберите `lidar_tf_roll` / `lidar_tf_pitch` в `lidar.launch.py`
и запустите скрипт снова: наклон должен уменьшаться. Если растёт, поменяйте знак поправки.

Запуск:
    python3 check_lidar_floor.py [--cloud /sensors/lidar/livox/point_cloud2] [--clouds 10] [--timeout 30]
"""

import argparse
import math
import sys

import numpy as np

MIN_RANGE = 0.3         # м, ближе этого точки считаем шумом корпуса
MAX_RANGE = 8.0         # м, дальше пол плохо виден
MAX_HEIGHT = -0.2       # м, берём точки ниже этого z (пол под лидаром)
INLIER_DISTANCE = 0.03  # м, допуск точки до плоскости


def cloud_to_xyz(msg):
    """PointCloud2 -> массив N x 3 (float64) без NaN и нулевых точек."""
    offsets = {field.name: field.offset for field in msg.fields}
    dtype = np.dtype({
        'names': ['x', 'y', 'z'],
        'formats': ['<f4', '<f4', '<f4'],
        'offsets': [offsets['x'], offsets['y'], offsets['z']],
        'itemsize': msg.point_step,
    })
    raw = np.frombuffer(bytes(msg.data), dtype=dtype, count=msg.width * msg.height)
    xyz = np.stack([raw['x'], raw['y'], raw['z']], axis=1).astype(np.float64)
    return xyz[np.isfinite(xyz).all(axis=1)]


def select_floor_candidates(xyz):
    """Оставляет точки ниже лидара в разумном диапазоне дальности."""
    distance = np.linalg.norm(xyz, axis=1)
    mask = (distance > MIN_RANGE) & (distance < MAX_RANGE) & (xyz[:, 2] < MAX_HEIGHT)
    return xyz[mask]


def fit_plane(points, iterations=300, seed=0):
    """RANSAC + уточнение МНК. Возвращает (нормаль n с n_z > 0, d, число inliers): n . p = d."""
    rng = np.random.default_rng(seed)
    best_inliers = None
    for _ in range(iterations):
        sample = points[rng.choice(len(points), 3, replace=False)]
        normal = np.cross(sample[1] - sample[0], sample[2] - sample[0])
        length = np.linalg.norm(normal)
        if length < 1e-9:
            continue
        normal /= length
        distances = np.abs((points - sample[0]) @ normal)
        inliers = distances < INLIER_DISTANCE
        if best_inliers is None or inliers.sum() > best_inliers.sum():
            best_inliers = inliers
    if best_inliers is None or best_inliers.sum() < 3:
        raise ValueError('не удалось найти плоскость')
    chosen = points[best_inliers]
    centroid = chosen.mean(axis=0)
    _, _, vt = np.linalg.svd(chosen - centroid)
    normal = vt[-1]
    if normal[2] < 0:
        normal = -normal
    return normal, float(normal @ centroid), int(best_inliers.sum())


def tilt_angles_deg(normal):
    """(угол вокруг оси x, угол вокруг оси y, полный угол) нормали от вертикали, градусы."""
    about_x = math.degrees(math.atan2(-normal[1], normal[2]))
    about_y = math.degrees(math.atan2(normal[0], normal[2]))
    total = math.degrees(math.acos(max(-1.0, min(1.0, normal[2]))))
    return about_x, about_y, total


def main():
    parser = argparse.ArgumentParser(description=__doc__.split('\n')[0])
    parser.add_argument('--cloud', default='/sensors/lidar/livox/point_cloud2')
    parser.add_argument('--clouds', type=int, default=10, help='сколько облаков собрать')
    parser.add_argument('--timeout', type=float, default=30.0)
    args = parser.parse_args()

    import rclpy
    from rclpy.qos import qos_profile_sensor_data
    from sensor_msgs.msg import PointCloud2

    rclpy.init()
    node = rclpy.create_node('check_lidar_floor')
    collected = []
    frame_ids = set()

    def on_cloud(msg):
        frame_ids.add(msg.header.frame_id)
        collected.append(select_floor_candidates(cloud_to_xyz(msg)))

    node.create_subscription(PointCloud2, args.cloud, on_cloud, qos_profile_sensor_data)
    deadline = node.get_clock().now().nanoseconds + int(args.timeout * 1e9)
    while rclpy.ok() and len(collected) < args.clouds and node.get_clock().now().nanoseconds < deadline:
        rclpy.spin_once(node, timeout_sec=0.2)
    node.destroy_node()
    rclpy.shutdown()

    if len(collected) < args.clouds:
        print(f'FAIL получено только {len(collected)} облаков из {args.clouds} (топик {args.cloud})')
        return 1
    points = np.concatenate(collected)
    if len(points) < 100:
        print(f'FAIL мало точек пола ({len(points)}): лидар не видит пол ниже z < {MAX_HEIGHT} м')
        return 1
    normal, offset, inliers = fit_plane(points)
    about_x, about_y, total = tilt_angles_deg(normal)
    print(f'кадр облака: {sorted(frame_ids)}; точек пола: {len(points)}, на плоскости: {inliers} '
          f'({100 * inliers / len(points):.0f}%)')
    print(f'нормаль пола: ({normal[0]:.4f}, {normal[1]:.4f}, {normal[2]:.4f})')
    print(f'наклон пола: вокруг x {about_x:+.2f} град, вокруг y {about_y:+.2f} град, полный {total:.2f} град')
    print(f'высота лидара над полом: {abs(offset):.3f} м (сравните с рулеткой и с URDF)')
    print('ИТОГ:', 'PASS' if total <= 2.0 else 'FAIL', '(допуск полного наклона 2 град)')
    return 0 if total <= 2.0 else 1


if __name__ == '__main__':
    sys.exit(main())
