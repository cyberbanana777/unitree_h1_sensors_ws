# Changelog

Формат основан на [Keep a Changelog](https://keepachangelog.com/ru/1.1.0/), версии — [SemVer](https://semver.org/lang/ru/).

## [1.1.0] - 2026-10-23

> Рефакторинг `lidar.launch.py`. **Не запускалось на роботе** (см. `docs/HARDWARE_TESTING.md`).
> `lidar.launch.py` и `realsense.launch.py` рефакторены; **ни один не запускался на роботе**.

### Fixed
- `setup.cfg`: `script-dir` / `install-scripts` заменены на `script_dir` / `install_scripts` (предупреждение setuptools
  `Usage of dash-separated ... will not be supported in future versions`).
- `lidar.launch.py`: цепочка топиков была разорвана. Конвертер публиковал `/sensors/livox/point_cloud2`,
  а `pointcloud_to_laserscan` подписывался на `/sensors/lidar/livox/point_cloud2` (ремап в launch был закомментирован).
  Теперь у `livox_to_pointcloud2` явные ремапы, вся цепочка идёт по схеме
  `/sensors/lidar/livox/{custom_msg, point_cloud2, scan}`.
- Статический TF `mid360_link → livox_frame` был задан семью числами (формат «x y z qx qy qz qw») с неединичным
  кватернионом $(0,\,-0.28,\,0,\,0)$. Заменён на шесть launch-аргументов.

### Changed
- `lidar.launch.py`: аргумент `transform_tolerance` (по умолчанию 0.1 с, раньше в `pointcloud_to_laserscan` было зашито 5.01 с);
  значение передаётся во включаемый launch `pointcloud_to_laserscan`. **Не запускалось в ROS.**
- **`realsense.launch.py` теперь запускает `realsense_d435i_configs`** (`d435i.launch.py`) вместо прямого включения
  `rs_launch.py`. Раньше путь к `config/realsense.yaml` указывал на неустановленный файл, а переменную
  `REALSENSE_CONFIG_FILE` `rs_launch.py`, насколько известно, не читает, так что конфиг, по-видимому, не применялся.
  Аргумент `enable_front_camera` сохранён; добавлены `mode` (по умолчанию `rgb_depth_pointcloud`: решение владельца, облако точек включено), `serial_no`,
  `parent_frame` (по умолчанию `realsense_link`, как и раньше: статический TF `realsense_link → camera_link`
  с нулевым смещением), `use_sim_time` теперь действительно передаётся в камеру.
- **Поведение отличается от старого `config/realsense.yaml`** (RGB и глубина 640x480 @ 15, облако точек, выравнивание
  глубины, `camera_name: accuracy_test_camera`): теперь используются пресеты `realsense_d435i_configs`. Режим
  по умолчанию включает облако, как и в старом конфиге; без облака — `mode:=rgb_depth`.
- TF задаётся аргументами `lidar_tf_x`, `lidar_tf_y`, `lidar_tf_z`, `lidar_tf_yaw`, `lidar_tf_pitch`, `lidar_tf_roll`
  (формат `x y z yaw pitch roll parent child`). **По умолчанию все нули** — это предположение: драйвер уже применяет
  `extrinsic_parameter` (roll 180°, pitch 16°) из `MID360_config.json`, поэтому дополнительного поворота быть не должно.
  Реальное крепление лидара не проверено: подберите значения по RViz.
- Узел статического TF переименован в `livox_static_transform_publisher` (раньше имя совпадало с TF камеры).
- Добавлена лицензионная шапка и docstring с описанием цепочки топиков; вспомогательная функция `_include()`.

### Added
- В `package.xml` зависимости `imu_converter`, `tf2_ros` и `realsense_d435i_configs` (launch-файлы их используют).

### Removed
- `config/realsense.yaml` (не устанавливался; заменён пресетами `realsense_d435i_configs`).
