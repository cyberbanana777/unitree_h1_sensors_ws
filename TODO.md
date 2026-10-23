# TODO: журнал рефакторинга unitree_h1_sensors_ws

Этот файл — список **изменений по рефакторингу** (что сделано и что осталось довести). Потенциальные
улучшения проекта ведутся в разделе «TODO» главного [README.md](README.md), не путайте их.
Состояние пакетов — [docs/PACKAGES.md](docs/PACKAGES.md), проверка на железе — [docs/HARDWARE_TESTING.md](docs/HARDWARE_TESTING.md),
подробные изменения — `CHANGELOG.md` в каждом пакете.

Легенда: `[x]` сделано в этом проходе (проверено только на заглушках, не на роботе), `[ ]` осталось,
**[робот]** — можно проверить только на железе.

Приоритет сейчас: `livox_ros_driver2`, `livox_to_pointcloud2`, `imu_converter`, `odom_converter`.

---

## 0. Сначала проверить на роботе (сборка + первый запуск)

Первая сборка (старый вид workspace, SDK не стоял): `livox_ros_driver2` не собрался (нет Livox-SDK2), остальное не проверено.

Вторая сборка (SDK установлен, `./build.sh humble`): **собрались все 8 пакетов**, включая `livox_to_pointcloud2`
(настоящая C++-сборка с rclcpp) и `completed_scripts_sensors`. Обычный `colcon build` без флага падал:
`LIVOX_INTERFACES_INCLUDE_DIRECTORIES-NOTFOUND`, потому что без `-DHUMBLE_ROS=humble` CMake драйвера шёл по ветке Foxy.
- [x] `livox_ros_driver2/CMakeLists.txt`: ветка Humble выбирается по `ROS_DISTRO`, если флаг не задан (версия 1.1.1).
  **[робот/ПК]** Проверьте: `rm -rf build install log && colcon build` без флагов должен пройти.
- [x] Предупреждение `tf2_sensor_msgs.h is obsolete` (`pointcloud_to_laserscan`): include через `__has_include`
  (`.hpp` на Humble, `.h` на Foxy).
- [x] Предупреждение `headers install destination is set to include` (`livox_to_pointcloud2`, версия 0.1.1): заголовок перенесён в `src/`.
- Предупреждение `Manually-specified variables were not used: HUMBLE_ROS, ROS_EDITION` появляется только из-за флагов `build.sh`
  и безвредно; при обычном `colcon build` его нет.
- Строки `The path '.../install/odom_converter' ... doesn't exist` — не из пакетов: терминал помнит старый `install/`.
  Откройте новый терминал (или `unset AMENT_PREFIX_PATH CMAKE_PREFIX_PATH` и заново `source /opt/ros/humble/setup.bash`).
- В том логе ещё были `cmd_to_odom_package` и `camera_package`: в новом архиве их нет; удалите папки в своём `src/`.
- Осталось: `colcon build` по новому архиву, затем `docs/HARDWARE_TESTING.md`.

Полный план с командами и критериями PASS — [docs/HARDWARE_TESTING.md](docs/HARDWARE_TESTING.md).


- [x] Livox-SDK2 установлен (по вашему сообщению).
- [ ] **[робот]** Собрать workspace. на Humble и Foxy достаточно `colcon build` (флаг `-DHUMBLE_ROS=humble` по-прежнему работает).
- [ ] **[робот]** Запустить `ros2 launch completed_scripts_sensors lidar.launch.py` и убедиться, что есть топики:
  `/sensors/lidar/livox/custom_msg`, `/sensors/lidar/livox/point_cloud2`, `/sensors/lidar/livox/scan`,
  `/sensors/imu/livox`, `/sensors/imu/unitree_h1`.
- [ ] **[робот]** `ros2 topic hz` на каждом из них; `custom_msg` должен идти с частотой около `publish_freq` (50 Гц).
- [ ] **[робот]** Прогнать `colcon test --packages-select imu_converter odom_converter` (тесты написаны, но
  запускались только против заглушек сообщений).

## 1. livox_ros_driver2

- [x] Ремапы в `msg_MID360_launch.py` сделаны относительными (абсолютное `/livox/lidar` не совпадало с
  `/sensors/livox/lidar`, ремап молча не работал).
- [x] `MODIFICATIONS.md` переписан: что именно поменяно в конфиге и launch, и почему.
- [x] Версия пакета `1.0.0` → `1.1.0` (`package.xml` и `package_ROS2.xml`), `CHANGELOG.md` дополнен.
- [x] `MID360_config.json.backup` оставлен без изменений (по вашему решению).
- [ ] **[робот]** Проверить, что `ping 192.168.123.120` проходит и `host_ip` в `MID360_config.json`
  совпадает с IP компьютера в сети робота.
- [ ] Решить, нужен ли в репозитории весь upstream: `launch_ROS1/`, `package_ROS1.xml`, HAP-конфиги,
  `3rdparty/rapidjson` (можно подтягивать из upstream и хранить только патч).
- [ ] `rviz_MID360_launch.py`: добавить тот же namespace/ремапы или явно пометить как отладочный.

## 2. livox_to_pointcloud2

- [x] Число точек = `min(point_num, points.size())` (раньше расхождение портило сообщение).
- [x] Запись полей через `memcpy` (шаг 18 байт, невыровненный доступ).
- [x] Удалены неиспользуемые `msg/*.msg` и `launch/*.yml` с ремапами несуществующих топиков.
- [x] Новый `launch/livox_to_pointcloud2.launch.py`, README, поправлен `package.xml`.
- [x] Версия `0.0.1` → `0.1.0`, добавлен `CHANGELOG.md`.
- [x] Собран с `-Wall -Wextra -Wpedantic` и ASan/UBSan против заглушек rclcpp (проверена логика конвертации).
- [ ] **[робот]** Собрать с настоящим rclcpp (проверить `ConstSharedPtr` в колбэке на Foxy).
- [ ] Решить по QoS: драйвер публикует с надёжным QoS и очередью, RViz по умолчанию подписывается
  надёжно. Если облако будет «тормозить» — перейти на `SensorDataQoS` с обеих сторон.
- [ ] Добавить в облако поле времени точки (`offset_time`), если понадобится компенсации движения (deskew) для SLAM.

## 3. imu_converter

- [x] Ковариации: вместо `-1` (значит «данных нет») по умолчанию нули («ковариация неизвестна»);
  дисперсии задаются параметрами.
- [x] Пропуск сообщений с нулевым кватернионом.
- [x] Убраны `numpy`, `pip_requirements.txt`, неиспользуемые импорты; `package.xml` исправлен.
- [x] Логика вынесена в `fill_imu_message`, добавлены тесты порядка кватерниона и ковариаций.
- [x] Версия `1.0.0` → `1.1.0`, добавлен `CHANGELOG.md`.
- [ ] **[робот]** Задать реальные дисперсии (посчитать по статике: `ros2 bag` + дисперсия гироскопа и акселерометра).
- [ ] **[робот]** Проверить знак/ось: при наклоне вперёд `linear_acceleration` и ориентация меняются как в REP-103.
- [ ] Решить, брать ли штамп из `LowState.tick` (время робота) вместо `now()`.

## 4. odom_converter

- [x] **Исправлен порядок кватерниона**: Unitree $(w,x,y,z)$ → ROS $(x,y,z,w)$ (раньше `w` попадал в `x`).
- [x] Файл и исполняемый файл переименованы `odom_convereter_node` → `odom_converter_node`;
  старое имя с опечаткой **удалено** (без алиаса).
- [x] Версия `1.0.0` → `2.0.0` (ломающее изменение: имя исполняемого файла), добавлен `CHANGELOG.md`.
- [ ] Проверить, что `meta_launch_ws` и другие ваши launch-файлы не используют старое имя `odom_convereter_node`.
- [x] Параметры: топики, фреймы, `publish_tf` (по умолчанию выкл.), ковариации, `network_interface`.
- [x] Убраны мёртвые ветки `hasattr`; предупреждение «Waiting for LowState» с троттлингом.
- [x] Безопасное завершение (`DisableOdom` в `try`), понятная ошибка при отсутствии `high_level_control`.
- [x] `package.xml`: зависимость `tf2_ros`, убран лишний `std_msgs`; тесты на кватернион.
- [ ] **[робот]** Проверить, в какой системе (мировой/связанной) приходит `SportModeState.velocity`;
  в `nav_msgs/Odometry` twist должен быть в `child_frame` (`pelvis`).
- [ ] **[робот]** Убедиться, что `w` в `/odom` ≈ 1 на неподвижном роботе и совпадает с `/sensors/imu/unitree_h1`.
- [ ] Задать честные ковариации (иначе нули читаются как «абсолютная точность»).
- [ ] Занести `high_level_control` (`OdomClient`) и `unitree_sdk2py` в документацию установки workspace.

## 5. completed_scripts_sensors (связующий launch)

- [x] `lidar.launch.py`: ремапы конвертера приведены к одной схеме топиков; цепочка драйвер → PointCloud2 → LaserScan замкнута.
- [x] Статический TF `mid360_link → livox_frame` был некорректным (7 чисел = «x y z qx qy qz qw»
  с неединичным кватернионом `(0, -0.28, 0, 0)`). Теперь это 6 launch-аргументов (`lidar_tf_*`), по умолчанию
  единичное преобразование: драйвер сам применяет `extrinsic_parameter` (roll 180°, pitch 16°).
- [x] В `package.xml` добавлены `imu_converter`, `tf2_ros`.
- [x] Начат `CHANGELOG.md` (раздел Unreleased); версия пакета повышена до `1.1.0` (решение владельца).
- [ ] **[робот]** Проверить TF в RViz (Fixed Frame = `mid360_link`): облако должно стоять ровно, пол — горизонтально.
  Если нет — подобрать `lidar_tf_*`. Не дублируйте вращение, уже заданное в `extrinsic_parameter`.
- [x] `transform_tolerance` в `pointcloud_to_laserscan` теперь аргумент launch (`lidar.launch.py` передаёт его дальше), по умолчанию 0.1 с вместо 5.01.
  Обоснование: TF `mid360_link → livox_frame` статический, для него допуск не нужен. **[робот]** Если `scan` не появляется, а в логе
  `Message Filter dropping message`, попробуйте `transform_tolerance:=5.01` (старое поведение) и сообщите, это даст находку про TF и время (см. T54).
- [x] `realsense.launch.py` переписан: включает `realsense_d435i_configs/d435i.launch.py` (аргументы `enable_front_camera`,
  `mode`, `serial_no`, `parent_frame:=realsense_link`, `use_sim_time`); `config/realsense.yaml` удалён.
  Режим по умолчанию — `rgb_depth_pointcloud` (облако включено, решение владельца); без облака — `mode:=rgb_depth`.
  **[робот]** Подтвердите, что нужен именно этот режим по умолчанию (раздел 5a в `HARDWARE_TESTING.md`).

## 6. Остальные пакеты (потом)

- [x] **`cmd_to_odom_package` удалён** из репозитория (решение владельца: заглушка, написанная до штатной одометрии
  производителя; её заменяет `odom_converter`). Перед удалением в нём был исправлен потерянный `dt` и версия
  поднята до `0.1.0`, но эти правки в репозиторий не вошли. Проверьте, что ничего в `meta_launch_ws` и других
  ваших launch-файлах не запускает `cmd_to_odom_node`.
- [x] **`camera_package` удалён** из репозитория (решение владельца: больше не нужен). Для справки, что в нём было не так:
  камера открывалась при импорте, один `pipeline` на два генератора, `pipeline.stop()` останавливал поток для всех,
  IP зашит в коде, нет авторизации. Проверьте, что ничего в ваших launch-файлах не запускает его узлы.
- [x] `install_dependensies.bash` (523 строки, опечатка в имени, искал несуществующие `pip_requirements.txt`/`apt_requirements.txt`
  и ссылался на несуществующий `install_additions.bash`, то есть фактически только обновлял pip) заменён на `install_dependencies.bash`:
  `rosdep install` плюс проверка Livox-SDK2, `unitree_sdk2py`, `high_level_control`, `unitree_go`; есть `--dry-run`.
  Проверено только с подставными `rosdep`/`ros2`; настоящий `rosdep` не запускался.
- [x] `pointcloud_to_laserscan`: `angle_increment` изменён с 0.007 на 0.00873 (≈π/360, 0.5°, 720 лучей вместо 898) по решению владельца.

## 7. realsense_d435i_configs (добавлен в `src/`, версия 0.2.0, MIT)

Что хорошо: понятные пресеты, комментарии объясняют компромиссы (почему `hole_filling_filter` выключен),
режим проверяется, можно подставить свой `config_file`, серийник с `_` учтён, `/**:` работает с любым именем узла.

Замечания к исходной версии 0.1.0 и что с ними сделано (отмечено `[x]`; осталось — только то, что видно на роботе):

- [x] **Имена параметров профиля.** Во всех пресетах указаны оба имени (`*.profile` и `*.color_profile`/`*.depth_profile`),
  тест `test_presets.py` это проверяет. **[робот]** `ros2 param list` и проверка разрешения по `camera_info`.
- [x] **README расходился с yaml** (RGB в `rgb_depth_pointcloud` — 848x480, не 1280x720; реальная глубина 424x240 из-за
  `decimation_filter`). Таблица исправлена, тест сверяет колонки RGB и Depth с yaml.
- [x] `camera_name` теперь передаётся в `parameters`. **[робот]** проверить `ros2 run tf2_tools view_frames`:
  действительно ли фреймы называются `<camera_name>_*` (я это по документации предполагаю, но не видел).
- [x] TF `realsense_link → camera_link`: аргумент `parent_frame` (+ `tf_*`), `completed_scripts_sensors/realsense.launch.py`
  передаёт `parent_frame:=realsense_link`, как делал старый launch.
- [x] `package.xml`: maintainer, лицензия MIT (+ файл `LICENSE`), `exec_depend` на `ament_index_python`, `tf2_ros`,
  `test_depend`.
- [x] `initial_reset`, `clip_distance`, `use_sim_time`, `log_level` вынесены в аргументы launch.
- [x] Тесты: `test_launch_params.py`, `test_presets.py` (26 проверок чистой логики и yaml, запущены через мой
  упрощённый запуск без настоящего pytest). Настоящий `launch`/`realsense2_camera` не запускались.
- [ ] `clip_distance: 3.0` в облачном пресете по-прежнему режет всё дальше 3 м (значение пресета не менял) —
  для навигации человекоподобного робота мало; решите, поднять ли, или передавайте `clip_distance:=...`.
- [ ] 1920x1080@30 (`rgb_usb3`) — YUYV→RGB8 на CPU, на Jetson это ощутимая нагрузка. **[робот]** проверить загрузку CPU.
- [ ] Проверить имена закомментированных параметров (`depth_module.visual_preset`, `pointcloud.pointcloud_qos`)
  через `ros2 param list` на вашей версии, прежде чем раскомментировать.
- [ ] IMU камеры выключен везде, хотя это D435i. Если нужен — добавить пресет (`enable_gyro/accel`, `unite_imu_method`).
- [x] Старый `config/realsense.yaml` (имя камеры `accuracy_test_camera`) и `camera_package` удалены.

## 8. Гигиена репозитория

- [x] Добавлены `docs/PACKAGES.md` (карта пакетов и статусы), `docs/HARDWARE_TESTING.md` (план проверки с командами и критериями)
  и скрипты `tools/check_imu.py`, `tools/check_odom.py`, `tools/check_pointcloud.py`.
- [x] В главный README добавлен раздел «TODO» (потенциальные улучшения).
- [x] Корневой `README.md` переписан (старый был копией README другого репозитория — телеуправление УКТ).
- [x] Удалены закоммиченные `__pycache__/*.pyc`.
- [x] Добавлен `.gitignore`.
- [x] `setup.cfg` во всех Python-пакетах: `script-dir`/`install-scripts` → `script_dir`/`install_scripts`
  (убирает предупреждения setuptools из лога сборки; проверено на копиях пакетов, предупреждение исчезает).
- [ ] Единая лицензия: сейчас MIT (ваши пакеты и `realsense_d435i_configs`), Apache-2.0 (`livox_to_pointcloud2`),
  BSD (`pointcloud_to_laserscan`), MIT (Livox). Это нормально, но нужен раздел в README.
- [ ] Лицензионные шапки в оставшихся файлах (`lidar.launch.py` теперь с шапкой, `realsense.launch.py` — есть).
- [ ] Шаблонные `test_copyright/flake8/pep257` в Python-пакетах: прогнать `colcon test`, исправить стиль или убрать пустые тесты.

## 9. По результатам анализа bag (подробности — [docs/BAG_ANALYSIS.md](docs/BAG_ANALYSIS.md))

- [x] Порядок кватерниона Unitree $(w,x,y,z)$ подтверждён на реальных записях (расхождение с `rpy` до $1.5\cdot10^{-7}$ рад).
- [ ] Защита от нулевого кватерниона в `imu_converter` ничем не подтверждена: в 22 479 сообщениях нулей нет. Оставить или убрать,
  решится только записью старта робота.
- [ ] Задать дисперсии IMU: ориентир `angular_velocity_variance` $\approx 10^{-4}$, `linear_acceleration_variance` $\approx 10^{-2}$
  (верхняя оценка по bag). В код значения по умолчанию не вносились.
- [x] Отброс точек нулевой дальности в `livox_to_pointcloud2` и параметр `odom_converter` для запуска без `OdomClient`
  (симуляция): решено не делать сейчас, перенесены в раздел «TODO» главного README как потенциальные улучшения.
- [ ] **[робот]** T54: header.stamp лидара опережал время записи на 8–10 с. Выяснить причину (часы лидара или другой компьютер).
- [ ] `/sensors/imu/livox` в единицах g, не м/с² (нарушение REP-145). Если этот IMU пойдёт в SLAM или фильтр — масштабировать на $9.81$.

## 10. Документы для проверок (добавлено 2026-10-23)

- [x] Проверки без робота (сборка, тесты, launch, IMU и лидар на записях) выполнены 2026-10-23, итоги в [docs/PACKAGES.md](docs/PACKAGES.md).
- [x] [docs/ROBOT_CHECK_GUIDE.md](docs/ROBOT_CHECK_GUIDE.md): методичка для исполнителя проверок на роботе (блоки A–H, протокол результатов).
- [x] [docs/ROBOT_OPEN_QUESTIONS.md](docs/ROBOT_OPEN_QUESTIONS.md): всё, что решается только на роботе (Q1–Q15), и решения владельца.
- [x] Инструменты: `tools/check_lidar_floor.py` (наклон пола и высота лидара), `tools/collect_env.sh` (сведения об окружении),
  `analyze_lowstate.py --static` (дисперсии IMU по записи неподвижного робота). Проверены на синтетических данных и записях, не на роботе.
- [x] Решения владельца: режим камеры по умолчанию `rgb_depth_pointcloud`, `clip_distance` 30.0, дисперсии IMU по умолчанию не задаём
  (эксперимент позже, Q7), IMU камеры не нужен (записано в README), версии подняты (`completed_scripts_sensors` 1.1.0, `realsense_d435i_configs` 0.3.0).
- [x] `angle_increment` в `pointcloud_to_laserscan`: владелец выбрал круглое значение 0.00873 (≈π/360, 720 лучей), было 0.007.
