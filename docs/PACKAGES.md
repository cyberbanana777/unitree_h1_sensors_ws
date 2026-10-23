# Карта пакетов

Состояние на 2026-10-23. Версии — из `package.xml`.

## Как читать статусы

| Статус | Значение |
|---|---|
| 🟢 **Рефакторен, проверен на железе** | Правки внесены, пройден [план проверки](HARDWARE_TESTING.md). *Пока таких нет.* |
| 🟡 **Рефакторен, не проверен** | Правки внесены; логика проверена юнит-тестами или сборкой с заглушками, но настоящая сборка в ROS и запуск на роботе не выполнялись |
| ⚪ **Не тронут** | Пакет в том виде, в каком его прислали; известные проблемы перечислены |
| 🔵 **Вендорный** | Чужой код; наши правки (если есть) перечислены отдельно |

## Сводная таблица

| Пакет | Версия | Что делает | Статус |
|---|---|---|---|
| `livox_ros_driver2` | 1.1.1 (драйвер 1.2.4) | Драйвер лидара Livox MID360 | 🔵🟡 вендорный, правки в launch и документации, не проверен |
| `livox_to_pointcloud2` | 0.1.1 | `CustomMsg` → `PointCloud2` | 🟡 рефакторен, не проверен |
| `pointcloud_to_laserscan` | 2.0.1 | `PointCloud2` → `LaserScan` | 🔵 вендорный; launch правился вами ранее, мы заменили только include `tf2_sensor_msgs` (устаревший `.h`) |
| `imu_converter` | 1.1.0 | IMU из `LowState` → `sensor_msgs/Imu` | 🟡 рефакторен, не проверен |
| `odom_converter` | 2.0.0 | `SportModeState` + `LowState` → `nav_msgs/Odometry` | 🟡 рефакторен, не проверен |
| `completed_scripts_sensors` | 1.1.0 | Launch-файлы наборов сенсоров | 🟡 оба launch-файла рефакторены, не проверены |
| `realsense_d435i_configs` | 0.3.0 | Пресеты и launch для RealSense D435i | 🟡 рефакторен, не проверен (MIT) |

## Как пакеты связаны

```
Лидар Livox MID360 ──UDP──▶ livox_ros_driver2
                                   │ /sensors/lidar/livox/custom_msg   (CustomMsg)
                                   ▼
                          livox_to_pointcloud2
                                   │ /sensors/lidar/livox/point_cloud2 (PointCloud2)
                                   ▼
                        pointcloud_to_laserscan ──▶ /sensors/lidar/livox/scan (LaserScan)

Робот Unitree H1 ──DDS──▶ /lowstate ──▶ imu_converter ──▶ /sensors/imu/unitree_h1 (Imu)
                  └─────▶ /lowstate + /odommodestate ──▶ odom_converter ──▶ /odom (Odometry)

completed_scripts_sensors/lidar.launch.py запускает всю цепочку лидара, imu_converter
и статический TF mid360_link → livox_frame.
```

`odom_converter` запускается отдельно (`ros2 run`), в `lidar.launch.py` его нет.

## Подробно по пакетам

### `livox_ros_driver2` — 🔵🟡
- **Что делает:** принимает UDP-пакеты лидара MID360 и публикует облако точек и IMU лидара.
- **Откуда:** upstream [Livox-SDK/livox_ros_driver2](https://github.com/Livox-SDK/livox_ros_driver2), MIT.
- **Что изменено нами:** сетевые IP и `extrinsic` в `config/MID360_config.json`; `publish_freq`, тип сообщения,
  namespace и ремапы в `launch_ROS2/msg_MID360_launch.py` (ремапы исправлены на относительные). Код `src/` не менялся.
  Подробности: `MODIFICATIONS.md`, `CHANGELOG.md`.
- **Не проверено:** сборка с Livox-SDK2, подключение к лидару, итоговые имена топиков.
- **Известно:** вместе с пакетом лежат неиспользуемые ROS1-файлы, HAP-конфиги и `3rdparty/rapidjson`.

### `livox_to_pointcloud2` — 🟡
- **Что делает:** переупаковывает `CustomMsg` в `PointCloud2` (поля `x, y, z, intensity, tag, line`, шаг 18 байт).
- **Что сделано:** исправлено число точек и невыровненная запись, удалены мёртвые файлы, добавлены launch и README.
- **Чем проверено:** сборка с `-Wall -Wextra -Wpedantic` и ASan/UBSan против заглушек `rclcpp`; тест с заведомо неверным `point_num`.
- **Не проверено:** сборка с настоящим `rclcpp` (Foxy и Humble), работа с живым драйвером.

### `pointcloud_to_laserscan` — 🔵⚪
- **Что делает:** срез облака по высоте в `LaserScan`.
- **Откуда:** upstream `ros-perception/pointcloud_to_laserscan` (BSD), launch-пример изменён вами под Livox.
- **Правки только в `launch/sample_pointcloud_to_laserscan_launch.py`:** `transform_tolerance` стал аргументом (по умолчанию 0.1 вместо 5.01),
  `angle_increment` 0.007 → 0.00873 (≈π/360, 720 лучей). Код узла не менялся; `.h` → `.hpp` для `tf2_sensor_msgs` см. выше.

### `imu_converter` — 🟡
- **Что делает:** `LowState.imu_state` → `sensor_msgs/Imu` (`/sensors/imu/unitree_h1`).
- **Что сделано:** ковариации `-1` заменены на нули, параметры дисперсий, пропуск нулевого кватерниона, тесты.
- **Чем проверено:** 5 юнит-тестов против заглушек сообщений ROS, flake8.
- **Не проверено:** сборка и запуск в настоящем ROS, поведение с реальным `/lowstate`, реальные значения дисперсий.

### `odom_converter` — 🟡
- **Что делает:** собирает `nav_msgs/Odometry` из `SportModeState` (положение, скорость) и `LowState` (ориентация, угловая скорость).
- **Что сделано:** исправлен порядок кватерниона, параметры, опциональный TF, безопасная остановка, переименован
  исполняемый файл (старое имя удалено, версия 2.0.0), тесты.
- **Чем проверено:** 4 юнит-теста против заглушек сообщений ROS, flake8.
- **Не проверено:** сборка, `OdomClient`/`unitree_sdk2py`, система координат скорости, реальное поведение на роботе.

### `completed_scripts_sensors` — 🟡
- **Что делает:** `lidar.launch.py` запускает цепочку лидара, IMU и TF; `realsense.launch.py` запускает камеру.
- **`lidar.launch.py` 🟡:** согласованы топики, TF превращён в аргументы `lidar_tf_*` (по умолчанию нули — предположение).
- **`realsense.launch.py` 🟡:** теперь включает `realsense_d435i_configs/d435i.launch.py` (`mode` по умолчанию `rgb_depth_pointcloud`,
  TF `realsense_link → camera_link` через `parent_frame`); `config/realsense.yaml` удалён. Поведение отличается от старого конфига
  (там было облако точек) — подтвердите режим по умолчанию.
- Версия `1.1.0`; изменения — в `CHANGELOG.md`.

### `realsense_d435i_configs` — 🟡
- **Что делает:** четыре пресета (`rgb_usb3`, `rgb_usb2`, `rgb_depth`, `rgb_depth_pointcloud`) и `d435i.launch.py` для `realsense2_camera`.
- **Что сделано:** оба имени параметров профиля в каждом пресете, исправлена таблица README, `camera_name` передаётся в ноду,
  аргументы launch (`serial_no`, `initial_reset`, `clip_distance`, `use_sim_time`, `log_level`, `parent_frame`, `tf_*`),
  метаданные `package.xml`, лицензия MIT, `CHANGELOG.md`.
- **Чем проверено:** 26 проверок (`test_launch_params.py`, `test_presets.py`) на чистой логике и yaml, запущены через мой
  упрощённый запуск без настоящего pytest; flake8.
- **Не проверено:** `colcon build`, настоящий `launch`, `realsense2_camera` и сама камера (применение профиля, имена фреймов, CPU).

## Удалённые пакеты

| Пакет | Дата | Почему |
|---|---|---|
| `cmd_to_odom_package` | 2026-10-23 | Заглушка, написанная до того, как производитель добавил штатную одометрию; её заменяет `odom_converter`. Последняя версия перед удалением — 0.1.0 (там исправлена потеря $\Delta t$ в интегрировании). Исходная версия есть в вашем исходном архиве `sensors.zip` |
| `camera_package` | 2026-10-23 | Flask-трансляция RealSense (MJPEG), не ROS-узлы; больше не нужен, камеру запускает `realsense_d435i_configs`. Исходная версия есть в вашем исходном архиве `sensors.zip` |

## Что ещё не покрыто

- В `src/` нет пакета для связи с URDF робота (TF `pelvis → mid360_link`).
- Нет автоматической сборки и CI. Сборка проверена вручную 2026-10-23 (см. ниже), на роботе ничего не запускалось.

## Проверено без робота (2026-10-23, ПК разработчика)

Окружение: Ubuntu 22.04.5, ROS 2 Humble, `rmw_cyclonedds_cpp`, Python 3.10.12, Livox-SDK2 в `/usr/local/lib`. Версии: `completed_scripts_sensors` 1.1.0, `imu_converter` 1.1.0,
`livox_ros_driver2` 1.1.1, `livox_to_pointcloud2` 0.1.1, `odom_converter` 2.0.0, `pointcloud_to_laserscan` 2.0.1,
`realsense_d435i_configs` 0.3.0.

- Чистая сборка 7 пакетов без предупреждений (кроме служебного вывода CMake драйвера Livox про APR).
- `test_conversion`: 9 из 9 (`imu_converter` 5, `odom_converter` 4). Линтеры (`flake8`, `pep257`, `copyright`, C++) красные, на работу не влияют.
- Аргументы launch-файлов и защита от опечатки в `mode` соответствуют ожиданию.
- IMU на записи ходьбы: 499.9 Гц, порядок кватерниона правильный.
- Лидар на записи `lidar-imu`: `check_pointcloud.py` PASS (20 пар), облако и скан с ожидаемыми полями.
- Ссылок на удалённые пакеты в других workspace нет.

Статусы 🟡 остаются: сеть, частоты драйвера, геометрия, одометрия и камера на роботе не проверялись.
