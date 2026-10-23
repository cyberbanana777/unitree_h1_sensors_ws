# unitree_h1_sensors_ws

ROS 2 пакеты для работы с сенсорами робота Unitree H1: лидар Livox MID360, IMU и одометрия из
состояния робота, камера Intel RealSense D435i.

> Репозиторий в разработке. Состояние пакетов — в [docs/PACKAGES.md](docs/PACKAGES.md),
> проверка на железе — в [docs/HARDWARE_TESTING.md](docs/HARDWARE_TESTING.md),
> журнал рефакторинга — в [TODO.md](TODO.md), изменения по пакетам — в `CHANGELOG.md` внутри каждого пакета.

## Пакеты

| Пакет | Назначение |
|---|---|
| `livox_ros_driver2` | Драйвер Livox (upstream + наши правки, см. `MODIFICATIONS.md`) |
| `livox_to_pointcloud2` | `livox_ros_driver2/CustomMsg` → `sensor_msgs/PointCloud2` |
| `pointcloud_to_laserscan` | `PointCloud2` → `LaserScan` (вендорный пакет, см. его README) |
| `imu_converter` | `unitree_go/LowState` → `sensor_msgs/Imu` |
| `odom_converter` | `SportModeState` + `LowState` → `nav_msgs/Odometry` |
| `realsense_d435i_configs` | Пресеты и launch для камеры RealSense D435i (`realsense2_camera`), MIT |
| `completed_scripts_sensors` | Launch-файлы, запускающие наборы сенсоров |

Скрипты проверки (`check_imu.py`, `check_odom.py`, `check_pointcloud.py`, `check_lidar_floor.py`), сбор окружения (`collect_env.sh`) и разбор bag без ROS (`tools/bag_analysis/`, результаты — [docs/BAG_ANALYSIS.md](docs/BAG_ANALYSIS.md)) лежат в `tools/`, порядок работы — в [docs/HARDWARE_TESTING.md](docs/HARDWARE_TESTING.md).

## Установка

1. ROS 2 Foxy или Humble.
2. [Livox-SDK2](https://github.com/Livox-SDK/Livox-SDK2) — собрать и установить (нужна `liblivox_lidar_sdk_shared.so`).
3. `unitree_ros2` и `unitree_sdk2py` (для `imu_converter` и `odom_converter`);
   `high_level_control` с `OdomClient` из workspace `unitree_h1_control_ws` (для `odom_converter`).
4. Сборка:

```bash
cd <workspace>
./src/install_dependencies.bash          # rosdep + проверка Livox-SDK2, unitree_sdk2py, unitree_go (--dry-run: только показать)
colcon build   # Humble определяется по ROS_DISTRO; на Foxy тоже просто colcon build
source install/setup.bash
```

## Запуск

```bash
ros2 launch completed_scripts_sensors lidar.launch.py   # лидар + IMU + TF
ros2 run odom_converter odom_converter_node             # одометрия робота
```

Топики лидара (namespace `sensors`):

```
/sensors/lidar/livox/custom_msg    livox_ros_driver2/CustomMsg   (драйвер)
/sensors/lidar/livox/point_cloud2  sensor_msgs/PointCloud2       (livox_to_pointcloud2)
/sensors/lidar/livox/scan          sensor_msgs/LaserScan         (pointcloud_to_laserscan)
/sensors/imu/livox                 sensor_msgs/Imu               (IMU самого лидара)
/sensors/imu/unitree_h1            sensor_msgs/Imu               (imu_converter, из /lowstate)
```

Положение `livox_frame` относительно `mid360_link` настраивается аргументами `lidar_tf_x|y|z|yaw|pitch|roll`
(по умолчанию 0: драйвер уже применяет `extrinsic_parameter` из `MID360_config.json`).

## TODO

Потенциальные улучшения (что ещё стоит сделать). Журнал уже сделанного рефакторинга ведётся
отдельно в [TODO.md](TODO.md).

- Проверить камеру на роботе: применяется ли профиль, имена фреймов (`camera_name`), загрузка CPU на `rgb_usb3`.
- IMU камеры D435i (гироскоп и акселерометр) сейчас выключен во всех пресетах; при необходимости добавить пресет
  (`enable_gyro`, `enable_accel`, `unite_imu_method`), см. `realsense_d435i_configs/README.md`.
- `livox_to_pointcloud2`: параметр, отбрасывающий точки нулевой дальности (в записях около 19 % точек без возврата).
- `odom_converter`: параметр, пропускающий `OdomClient` (запуск в симуляции; по умолчанию поведение на роботе не меняется).
- Измерить реальные ковариации IMU и одометрии (запись статики в `ros2 bag`) и задать их параметрами.
- Взять штамп времени IMU и одометрии из `LowState.tick` вместо времени приёма на хосте.
- Определить, в какой системе координат приходит `SportModeState.velocity`, и при необходимости пересчитывать в `pelvis`.
- Перейти на `SensorDataQoS` для облака точек, если RViz или SLAM начнут отставать.
- Добавить в облако точек время каждой точки (`offset_time`) для компенсации движения (deskew).
- Описать в README единую схему фреймов (`odom` → `pelvis` → `mid360_link` → `livox_frame`) и кто какой TF публикует.
- Хранить `livox_ros_driver2` как патч поверх upstream вместо полной копии (убрать ROS1-часть и `3rdparty`).
- Собирать и тестировать все пакеты в CI (Docker на Foxy и Humble), добавить `launch_testing` для launch-файлов.

## Лицензии

Собственные пакеты — MIT. Сторонний код сохраняет свои лицензии: `livox_ros_driver2` (MIT),
`livox_to_pointcloud2` (Apache-2.0), `pointcloud_to_laserscan` (BSD).
