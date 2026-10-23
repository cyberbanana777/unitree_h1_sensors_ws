#!/usr/bin/env python3
"""Минимальное чтение rosbag2 (sqlite3, CDR little-endian) без установленного ROS.

Умеет разбирать: unitree_go/msg/LowState (только нужные поля), sensor_msgs/msg/Imu,
livox_ros_driver2/msg/CustomMsg. Нужны только стандартная библиотека и numpy (в анализаторах).
Раскладка полей взята из `unitree_ros2/.../unitree_go/msg` и `livox_ros_driver2/msg`;
если вы меняли версии сообщений, сверьте её.
"""

import glob
import sqlite3
import struct


def open_bag(bag_dir, topic):
    """Итератор (время записи в нс, сырые байты CDR) по топику, по возрастанию времени."""
    db_files = sorted(glob.glob(bag_dir.rstrip('/') + '/*.db3'))
    if not db_files:
        raise FileNotFoundError(f'В {bag_dir} нет файла .db3')
    for db_file in db_files:
        # immutable=1: файл не меняется, -wal/-shm рядом не нужны
        con = sqlite3.connect(f'file:{db_file}?mode=ro&immutable=1', uri=True)
        row = con.execute('select id from topics where name=?', (topic,)).fetchone()
        if row is None:
            continue
        query = 'select timestamp, data from messages where topic_id=? order by timestamp'
        for stamp_ns, data in con.execute(query, (row[0],)):
            yield stamp_ns, bytes(data)


class Cdr:
    """Читатель CDR: выравнивание считается от начала данных (после 4-байтного заголовка)."""

    def __init__(self, buf):
        if buf[1] != 1:
            raise ValueError('Поддерживается только little-endian CDR')
        self.buf = buf
        self.base = 4
        self.pos = 4

    def get(self, fmt, align):
        remainder = (self.pos - self.base) % align
        if remainder:
            self.pos += align - remainder
        values = struct.unpack_from('<' + fmt, self.buf, self.pos)
        self.pos += struct.calcsize('<' + fmt)
        return values

    def string(self):
        length = self.get('I', 4)[0]
        text = self.buf[self.pos:self.pos + length - 1].decode()
        self.pos += length
        return text


def parse_lowstate(buf):
    """unitree_go/LowState -> dict (quat в порядке Unitree w,x,y,z)."""
    c = Cdr(buf)
    c.get('2B', 1)
    c.get('2B', 1)                       # head, level_flag, frame_reserve
    c.get('2I', 4)
    c.get('2I', 4)
    c.get('H', 2)                        # sn, version, bandwidth
    quat = c.get('4f', 4)
    gyro = c.get('3f', 4)
    acc = c.get('3f', 4)
    rpy = c.get('3f', 4)
    c.get('b', 1)                        # IMUState.temperature
    for _ in range(20):                  # MotorState[20]
        c.get('B', 1)
        c.get('7f', 4)
        c.get('b', 1)
        c.get('I', 4)
        c.get('2I', 4)
    c.get('4B', 1)
    c.get('i', 4)
    c.get('H', 2)                        # BmsState
    c.get('2b', 1)
    c.get('2b', 1)
    c.get('15H', 2)
    c.get('4h', 2)
    c.get('4h', 2)                       # foot_force, foot_force_est
    tick = c.get('I', 4)[0]
    return {'quat': quat, 'gyro': gyro, 'acc': acc, 'rpy': rpy, 'tick': tick}


def parse_imu(buf):
    """sensor_msgs/Imu -> dict."""
    c = Cdr(buf)
    sec, nsec = c.get('iI', 4)
    frame = c.string()
    quat = c.get('4d', 8)
    quat_cov = c.get('9d', 8)
    gyro = c.get('3d', 8)
    gyro_cov = c.get('9d', 8)
    acc = c.get('3d', 8)
    acc_cov = c.get('9d', 8)
    return {'stamp': sec + nsec * 1e-9, 'frame': frame, 'quat': quat, 'quat_cov': quat_cov,
            'gyro': gyro, 'gyro_cov': gyro_cov, 'acc': acc, 'acc_cov': acc_cov}


def parse_custommsg(buf):
    """livox_ros_driver2/CustomMsg -> dict; points: (offset_time, x, y, z, reflectivity, tag, line)."""
    c = Cdr(buf)
    sec, nsec = c.get('iI', 4)
    frame = c.string()
    timebase = c.get('Q', 8)[0]
    point_num = c.get('I', 4)[0]
    lidar_id = c.get('B', 1)[0]
    c.get('3B', 1)                       # rsvd
    count = c.get('I', 4)[0]
    points = []
    for _ in range(count):
        offset = c.get('I', 4)[0]
        x, y, z = c.get('3f', 4)
        reflectivity, tag, line = c.get('3B', 1)
        points.append((offset, x, y, z, reflectivity, tag, line))
    return {'stamp': sec + nsec * 1e-9, 'frame': frame, 'timebase': timebase, 'point_num': point_num,
            'lidar_id': lidar_id, 'points': points}
