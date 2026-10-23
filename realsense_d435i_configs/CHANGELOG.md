# Changelog

Формат основан на [Keep a Changelog](https://keepachangelog.com/ru/1.1.0/), версии — [SemVer](https://semver.org/lang/ru/).

## [0.3.0] - 2026-10-23

### Изменено
- Режим по умолчанию в `d435i.launch.py` теперь `rgb_depth_pointcloud` (раньше `rgb_depth`): решение владельца,
  облако точек должно быть включено.
- `clip_distance` в `rgb_depth_pointcloud.yaml`: 3.0 → 30.0 (практически без отсечки). **Не проверено с камерой:**
  если `realsense2_camera` не примет значение, использовать -1.0.

### Добавлено
- В README заметка о возможности включить IMU камеры (пока не нужен).

## [0.2.0] - 2026-10-23

> Пакет добавлен в репозиторий и доработан. **Не запускался с настоящей камерой и `realsense2_camera`**
> (проверены только тесты на чистом Python/YAML, см. `docs/HARDWARE_TESTING.md`).

### Исправлено
- `rgb_depth_pointcloud.yaml` задавал только `rgb_camera.color_profile` и `depth_module.depth_profile`. В версиях
  `realsense2_camera` до 4.51.x эти параметры называются `*.profile`, и нода молча игнорировала незнакомые имена,
  оставляя профиль по умолчанию. Теперь указаны оба имени, как в остальных пресетах.
- README: у режима `rgb_depth_pointcloud` был указан RGB 1280x720, а в yaml 848x480; указано, что глубина после
  прореживания 424x240. Совпадение таблицы с yaml проверяется тестом.
- `camera_name` не передавался в ноду параметром (только как имя узла), из-за чего, по-видимому, фреймы всегда
  назывались `camera_*`. Теперь передаётся параметром `camera_name`.
- `package.xml`: настоящий maintainer вместо `user@example.com`, лицензия MIT (как в остальном репозитории),
  `xml-model`, зависимости `ament_index_python`, `tf2_ros`, тестовые зависимости.

### Добавлено
- Лицензия MIT (файл `LICENSE`).
- Аргументы launch: `initial_reset`, `clip_distance`, `use_sim_time`, `log_level`, `parent_frame`, `tf_x … tf_roll`.
- Статический TF `parent_frame → <camera_name>_link` (раньше привязку к URDF делал `realsense.launch.py`).
- Проверки конфигурации: файл `config_file` должен существовать, `initial_reset` / `use_sim_time` — `true`/`false`.
- Логику launch вынесена в функции `build_parameters()`, `build_tf_arguments()`, `parse_bool()`; тесты
  `test/test_launch_params.py` и `test/test_presets.py`.
- Заметки о версиях `realsense2_camera`, нагрузке на CPU и `clip_distance` в README.

### Изменено
- Лицензионная шапка в launch-файле, `cmake_minimum_required 3.8`.

## [0.1.0]
- Исходная версия (присланный архив `realsense_d435i_configs`).
