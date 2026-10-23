# realsense_d435i_configs

Четыре пресета для `realsense2_camera` + один launch-файл для Intel RealSense D435i.
IMU камеры во всех пресетах выключен.

| mode                   | RGB            | Depth        | PointCloud   | USB  |
|------------------------|----------------|--------------|--------------|------|
| `rgb_usb3`             | 1920x1080 @ 30 | -            | -            | 3.x  |
| `rgb_usb2`             | 640x480 @ 30   | -            | -            | 2.0  |
| `rgb_depth`            | 1280x720 @ 30  | 848x480 @ 30 | -            | 3.x  |
| `rgb_depth_pointcloud` | 848x480 @ 30   | 848x480 @ 30 | да (цветное) | 3.x  |

В `rgb_depth` глубина выровнена в кадр цветной камеры. В `rgb_depth_pointcloud` включено прореживание (`decimation_filter`, ×2),
поэтому реальное разрешение глубины и облака 424x240; выравнивание выключено, текстура облака берётся по экстринсикам.

## Запуск

```bash
ros2 launch realsense_d435i_configs d435i.launch.py mode:=rgb_usb3
ros2 launch realsense_d435i_configs d435i.launch.py mode:=rgb_usb2
ros2 launch realsense_d435i_configs d435i.launch.py mode:=rgb_depth
ros2 launch realsense_d435i_configs d435i.launch.py mode:=rgb_depth_pointcloud serial_no:=123456789
```

Топики по умолчанию (`camera_name:=camera`, `camera_namespace:=camera`): `/camera/camera/color/image_raw`,
`/camera/camera/depth/image_rect_raw`, `/camera/camera/aligned_depth_to_color/image_raw` (`rgb_depth`),
`/camera/camera/depth/color/points` (`rgb_depth_pointcloud`).

## Аргументы launch

| Аргумент | По умолчанию | Описание |
|---|---|---|
| `mode` | `rgb_depth_pointcloud` | Пресет: `rgb_usb3`, `rgb_usb2`, `rgb_depth`, `rgb_depth_pointcloud` |
| `camera_name` | `camera` | Имя узла и префикс фреймов (`camera_link`, `camera_color_optical_frame`, ...) |
| `camera_namespace` | `camera` | Namespace узла |
| `serial_no` | пусто | Серийный номер, если камер несколько (префикс `_` добавляется сам) |
| `config_file` | пусто | Свой yaml вместо пресета |
| `initial_reset` | `true` | Перезагружать камеру при старте (занимает несколько секунд, USB переподключается) |
| `clip_distance` | пусто | Отсечка глубины, м; пусто = значение пресета (в облачном пресете 30.0, то есть практически без отсечки) |
| `use_sim_time` | `false` | |
| `log_level` | `info` | |
| `parent_frame` | пусто | Если задан, публикуется статический TF `parent_frame → <camera_name>_link` |
| `tf_x`, `tf_y`, `tf_z`, `tf_yaw`, `tf_pitch`, `tf_roll` | `0.0` | Положение и поворот камеры в `parent_frame` (м, рад) |

Привязка к роботу: камера сама публикует только внутренние фреймы (`camera_link` → оптические фреймы).
Чтобы связать её с URDF, задайте `parent_frame` (например, `realsense_link`) и при необходимости `tf_*`:

```bash
ros2 launch realsense_d435i_configs d435i.launch.py mode:=rgb_depth parent_frame:=realsense_link
```

## Версии `realsense2_camera`

Имена параметров профиля зависят от версии: до 4.51.x это `rgb_camera.profile` и `depth_module.profile`,
с 4.54.x — `rgb_camera.color_profile` и `depth_module.depth_profile`. Незнакомый параметр из yaml нода молча
игнорирует, поэтому во всех пресетах указаны оба имени (тест `test_presets.py` это проверяет).
Проверить, что профиль применился: `ros2 topic echo --once /camera/camera/color/camera_info --no-arr` (поля `width`, `height`).

Имена параметров в закомментированных строках пресетов (`depth_module.visual_preset`, `pointcloud.pointcloud_qos`
и т. п.) перед включением сверьте через `ros2 param list /camera/camera` для вашей версии.

## Заметки

* `rgb_usb3` (1080p, 30 к/с): конвертация YUYV → RGB8 идёт на CPU, на Jetson это заметная нагрузка.
* `clip_distance: 30.0` в облачном пресете практически не отсекает точки (полезная дальность D435i около 10 м). Если нода не примет значение, используйте `-1.0` (отсечка выключена) или аргумент launch `clip_distance`.
* **IMU камеры (потенциальное улучшение, сейчас не нужен).** D435i содержит гироскоп и акселерометр, во всех пресетах они выключены. Если понадобятся, добавьте пресет с `enable_gyro: true`, `enable_accel: true` и `unite_imu_method: 2` (линейная интерполяция в один топик `imu`); имена параметров сверьте через `ros2 param list` для вашей версии `realsense2_camera`.
* Облачный пресет публикует облако с обычным QoS. Если включите `pointcloud.pointcloud_qos: SENSOR_DATA`,
  подписчик тоже должен быть best effort (в RViz: Reliability = Best Effort).
* `hole_filling_filter` выключен намеренно: он дорисовывает точки интерполяцией, то есть добавляет выдуманную геометрию.

## Тесты

```bash
colcon test --packages-select realsense_d435i_configs && colcon test-result --verbose
```

Проверяются: параметры, которые строит launch (серийник, `camera_name`, ошибки), структура пресетов
(оба имени профиля, согласованность потоков) и совпадение таблицы выше с yaml.
