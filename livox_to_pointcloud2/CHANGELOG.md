# Changelog

Формат основан на [Keep a Changelog](https://keepachangelog.com/ru/1.1.0/), версии — [SemVer](https://semver.org/lang/ru/).

Пакет основан на [porizou/livox_to_pointcloud2](https://github.com/porizou/livox_to_pointcloud2) (Apache-2.0).

## [0.1.1] - 2026-10-23

### Исправлено
- Предупреждение `headers install destination is set to include by ament_auto_package` из лога сборки: заголовок
  `livox_to_pointcloud2.hpp` нужен только самому узлу, поэтому перенесён из `include/` в `src/` (каталога `include/` больше нет,
  заголовки не устанавливаются). Код не менялся. Не собиралось в настоящем ROS: убедитесь, что предупреждение пропало.

## [0.1.0] - 2026-10-23

> Рефакторинг. Логика конвертации собрана с `-Wall -Wextra -Wpedantic` и ASan/UBSan **против заглушек rclcpp**.
> С настоящим ROS и на роботе не собиралось и не запускалось (см. `docs/HARDWARE_TESTING.md`).

### Исправлено
- Число точек теперь `min(point_num, points.size())`. Раньше размер буфера считался по `point_num`,
  а цикл шёл по `points`: при `point_num < points.size()` запись выходила за буфер, при `point_num > points.size()`
  в облаке оставался неинициализированный «хвост». При расхождении выводится предупреждение (не чаще раза в 5 с).
- Запись полей через `std::memcpy` вместо `reinterpret_cast<float*>`: при `point_step = 18` поля
  `float` не выровнены, прямое разыменование было неопределённым поведением.

### Изменено
- Заполнение `PointField` вынесено в вспомогательную функцию `addField()`; раскладка полей не менялась
  (`x, y, z, intensity` — float32; `tag, line` — uint8; `point_step = 18`).
- В колбэке `ConstSharedPtr`; в конструкторе в лог пишутся реальные имена топиков.
- `CMakeLists.txt`: `cmake_minimum_required 3.8`, лишние комментарии убраны.
- `package.xml`: версия `0.1.0`, корректные `buildtool_depend`, лицензия `Apache-2.0`, убрана зависимость `std_msgs`.

### Добавлено
- `launch/livox_to_pointcloud2.launch.py`, `README.md` с таблицей топиков.

### Удалено
- `msg/CustomMsg.msg`, `msg/CustomPoint.msg`: нигде не собирались, настоящие типы лежат в `livox_ros_driver2`.
- `launch/livox_to_pointcloud2.launch.yml`: ремапил несуществующие топики (`/livox_pointcloud`, `/converted_pointcloud2`).

## [0.0.1]
- Исходная версия (форк porizou).
