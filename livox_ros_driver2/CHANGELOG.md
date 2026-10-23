# Changelog

All notable changes to this project will be documented in this file.

## [package 1.1.1, H1 fork] - 2026-10-23

### Fixed
- `CMakeLists.txt`: обычный `colcon build` без `-DHUMBLE_ROS=humble` падал на Humble с
  `LIVOX_INTERFACES_INCLUDE_DIRECTORIES-NOTFOUND` (выбиралась ветка для Foxy). Теперь, если `HUMBLE_ROS` не задан,
  ветка выбирается по `ROS_DISTRO` (всё, кроме `foxy` и `galactic`, идёт по пути Humble). Явный `-DHUMBLE_ROS=humble`
  и `build.sh` работают как раньше. Логика выбора проверена отдельным скриптом `cmake -P`; **сборка в настоящем ROS
  этой правкой ещё не проверена**.

## [package 1.1.0, H1 fork] - 2026-10-23

Версия пакета в `package.xml` (`1.1.0`) относится к нашему форку и не совпадает с версией самого драйвера
(`1.2.4`, из `src/include/livox_ros_driver2.h`; её мы не меняли). Исходный код драйвера (`src/`) не менялся,
все правки — в launch-файлах, конфигах и документации (в 1.1.1 — ещё в `CMakeLists.txt`); подробности — в [MODIFICATIONS.md](MODIFICATIONS.md).
**Не собиралось и не запускалось на роботе** (см. `docs/HARDWARE_TESTING.md`).

### Fixed
- `launch_ROS2/msg_MID360_launch.py`: ремапы были абсолютными (`/livox/lidar`, `/livox/imu`), а при
  `namespace='sensors'` драйвер публикует в `/sensors/livox/lidar` и `/sensors/livox/imu`; правило не совпадало
  и не применялось. Теперь ремапы относительные (`livox/lidar`, `livox/imu`), итоговые топики —
  `/sensors/lidar/livox/custom_msg` и `/sensors/imu/livox`.

### Changed
- `MODIFICATIONS.md` переписан: таблица отличий конфига от `MID360_config.json.backup`
  (IP, extrinsic), описание launch-правок, пояснение, что драйвер применяет `extrinsic_parameter` к точкам.
- Версия пакета `1.0.0` → `1.1.0` в `package.xml` и `package_ROS2.xml`
  (`build.sh` копирует второй поверх первого, поэтому правятся оба).

### Unchanged
- `config/MID360_config.json.backup` оставлен как есть (старый конфиг с IP `192.168.1.120`).

---

# Upstream changelog (Livox-SDK/livox_ros_driver2)

## [1.2.4]
### Fixed
- Optimize framing performance

## [1.2.3]
### Fixed
- Optimize framing logic and reduce CPU usage
- Fixed some known issues

## [1.2.1]
### Fixed
- Fix offset time error regarding CustomMsg format message publishment.

## [1.2.0]
### Added
- Revise the frame segmentation logic.
- (Notice!!!) Add Timestamp to each point in Livox pointcloud2 (PointXYZRTLT) format. The PointXYZRTL format has been updated to PointXYZRTLT format. Compatibility needs to be considered.
### Fixed
- Improve support for gPTP and GPS synchronizations.

--- 
## [1.1.3]
### Fixed
- Improve performance when running in ROS2 Humble.

--- 
## [1.1.2]
### Changed
- Change publish frequency range to [0.5Hz, 10 Hz].
### Fixed
- Fix a high CPU-usage problem.

--- 
## [1.1.1]
### Added
- Offer valid line-number info in the point cloud data of MID-360 Lidar.
- Enable IMU by default.
### Changed
- Update the README slightly.

--- 
## [1.0.0]
### Added
- Support Mid-360 Lidar.
- Support for Ubuntu 22.04 ROS2 humble.
- Support multi-topic fuction, the suffix of the topic name corresponds to the ip address of each Lidar. 
### Changed
- Remove the embedded SDK.
- Constraint: Livox ROS Driver 2 for ROS2 does not support message passing with PCL native data types.
### Fixed
- Fix IMU packet loss.
- Fix some conflicts with livox ros driver.
- Fixed HAP Lidar publishing PointCloud2 and CustomMsg format point clouds with no line number.
