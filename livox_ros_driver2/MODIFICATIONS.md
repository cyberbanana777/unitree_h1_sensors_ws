# Изменения относительно upstream `Livox-SDK/livox_ros_driver2`

Драйвер взят из https://github.com/Livox-SDK/livox_ros_driver2 (MIT) почти без изменений.
Все наши правки перечислены здесь.

## `config/MID360_config.json`

Настроен под сеть робота Unitree H1 (формат с массивом `host_net_info`):

| Параметр | было (`MID360_config.json.backup`) | стало |
|---|---|---|
| IP хоста (`host_ip`) | `192.168.1.120` | `192.168.123.162` |
| IP лидара (`lidar_ip`, `lidar_configs[].ip`) | `192.168.1.120` | `192.168.123.120` |
| `extrinsic_parameter` | `roll/pitch/yaw = 0` | `roll = 180`, `pitch = 16`, `yaw = 0` |

Драйвер **применяет** `extrinsic_parameter` к точкам (см. `src/comm/pub_handler.cpp`,
`LidarPubHandler`), то есть облако в `livox_frame` уже повёрнуто на эти углы. Поэтому в TF
`mid360_link -> livox_frame` (см. `completed_scripts_sensors/launch/lidar.launch.py`) то же самое
вращение добавлять не нужно.

Если у вас другой IP на компьютере (`ip a`), поменяйте `host_ip` под свою сеть. Файл
`MID360_config.json.backup` оставлен в `config/` без изменений (старый конфиг для сети `192.168.1.x`);
драйвер его не читает.

## `launch_ROS2/msg_MID360_launch.py`

* `publish_freq = 50.0`, `xfer_format = 1` (CustomMsg вместо PointCloud2);
* `namespace='sensors'`;
* ремапы `livox/lidar -> lidar/livox/custom_msg` и `livox/imu -> imu/livox`. Они **относительные**:
  в namespace `sensors` топики драйвера называются `/sensors/livox/lidar` и `/sensors/livox/imu`,
  и абсолютное правило `/livox/lidar` их не находило.

Итоговые топики: `/sensors/lidar/livox/custom_msg`, `/sensors/imu/livox`.

## `launch_ROS2/rviz_MID360_launch.py`

* `publish_freq = 20.0`, `xfer_format = 1`. Namespace и ремапы здесь не заданы — этот launch нужен
  для отладки с RViz.

## Что нужно для сборки

* [Livox-SDK2](https://github.com/Livox-SDK/Livox-SDK2) должен быть установлен в `/usr/local/lib`
  (`liblivox_lidar_sdk_shared.so`);
* на ROS 2 Humble — флаг `-DHUMBLE_ROS=humble` (см. `README.md` этого репозитория).
