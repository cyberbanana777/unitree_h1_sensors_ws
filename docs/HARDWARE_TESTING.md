# Проверка софта на железе

Цель: убедиться, что пакеты, прошедшие рефакторинг, работают на реальном Unitree H1 с лидаром MID360.
Документ — чек-лист: у каждой проверки есть команда и критерий **PASS**. Пошаговая версия для исполнителя с процедурами
и протоколом результатов — [ROBOT_CHECK_GUIDE.md](ROBOT_CHECK_GUIDE.md). Если все проверки раздела 9 зелёные,
можно считать, что эта часть софта исправна и работает.

> До этой проверки рефакторинг подтверждён только юнит-тестами на заглушках и сборкой логики конвертера с заглушками
> `rclcpp`. Настоящая сборка в ROS и запуск на роботе не выполнялись, поэтому пункты ниже могут выявить
> проблемы, которых я не видел. Если что-то не сходится с «ожидается», зафиксируйте вывод команды: это и есть находка.

## 0. Безопасность и условия

- Проверки 1–7 выполняются **с неподвижным роботом** в безопасном состоянии (по инструкции производителя).
- Проверки движения (раздел 6.4) выполняйте только под присмотром оператора с пультом, на свободной площадке.
  В этом документе нет команд, которые двигают робота: движение задаётся штатным пультом.
- Значения «ожидается» в таблицах (частоты, допуски) — ориентиры из конфигурации и характеристик датчиков, а не
  гарантии. Если у вас стабильно другие значения, сначала разберитесь почему, потом меняйте допуск.
- Все команды выполняются из окружения, где сделан `source` ROS и workspace:
  ```bash
  source /opt/ros/<foxy|humble>/setup.bash
  source ~/unitree_h1_sensors_ws/install/setup.bash        # ваш путь
  ```
- Скрипты проверок лежат в `src/tools/` (запуск: `python3 src/tools/<имя>.py`).

Обозначения: **[T1]**, **[T2]**… — номера проверок для итоговой таблицы.

## 1. Сеть и железо

| № | Команда | PASS |
|---|---|---|
| T1 | `ip -4 addr \| grep 192.168.123.162` | у компьютера есть адрес `192.168.123.162` (тот же, что `host_ip` в `MID360_config.json`) |
| T2 | `ping -c 5 192.168.123.120` | 0% потерь (лидар отвечает) |
| T3 | `ros2 topic list \| grep -E '^/(lowstate\|odommodestate)$'` | оба топика есть (робот публикует состояние) |
| T4 | `ros2 topic hz /lowstate` (остановить через 10 с) | идёт стабильный поток; **запишите частоту** — нужна в T21 |

Если T2 не проходит: проверьте кабель, питание лидара, что сеть компьютера `192.168.123.x/24`. Если T3 не проходит:
проверьте `unitree_ros2` (`source ~/unitree_ros2/cyclonedds_ws/install/local_setup.bash`) и сетевой интерфейс DDS.

## 2. Сборка и юнит-тесты

```bash
cd ~/unitree_h1_sensors_ws
./src/install_dependencies.bash
colcon build        # Humble определяется по ROS_DISTRO (флаг -DHUMBLE_ROS=humble тоже работает)
source install/setup.bash
```

| № | Команда | PASS |
|---|---|---|
| T5 | `colcon build 2>&1 \| tail -20` | `Summary: N packages finished`, **нет** `failed` и `stderr:` с ошибками (предупреждения `-Wpedantic` в `livox_ros_driver2` допустимы, они upstream); предупреждений setuptools `Usage of dash-separated 'script-dir' ...` быть не должно |
| T6 | `colcon test --packages-select imu_converter odom_converter --pytest-args -k test_conversion && colcon test-result --verbose` | `test_conversion` в обоих пакетах пройден (9 тестов), 0 failures |
| T7 | `ros2 interface show livox_ros_driver2/msg/CustomMsg` | печатается тип с полями `header`, `timebase`, `point_num`, `lidar_id`, `points[]` (сообщения драйвера собрались; у `livox_to_pointcloud2` автотестов C++ нет, его логику проверяет `check_pointcloud.py` в разделе 3) |

Шаблонные тесты `test_copyright`, `test_flake8`, `test_pep257` в Python-пакетах могут падать из-за стиля — это
не ошибка работы узлов. Запишите, какие падают, но на итог T6 они не влияют (см. `TODO.md`, раздел 8).

Проверьте, что новые исполняемые файлы на месте, а старого с опечаткой нет:

| № | Команда | PASS |
|---|---|---|
| T8 | `ros2 pkg executables odom_converter` | выводит только `odom_converter odom_converter_node` |
| T9 | `ros2 pkg executables imu_converter && ros2 pkg executables livox_to_pointcloud2` | `imu_converter_node` и `livox_to_pointcloud2_node` |

## 3. Лидар: драйвер → PointCloud2 → LaserScan

Запуск (оставьте терминал открытым, лог пишется в файл):

```bash
ros2 launch completed_scripts_sensors lidar.launch.py 2>&1 | tee /tmp/lidar_launch.log
```

В другом терминале:

| № | Команда | PASS |
|---|---|---|
| T10 | `ros2 launch completed_scripts_sensors lidar.launch.py --show-args` | в списке аргументов есть `lidar_tf_x … lidar_tf_roll` |
| T11 | `ros2 node list` | есть `/sensors/livox_lidar_publisher`, `/sensors/livox_to_pointcloud2`, `/sensors/pointcloud_to_laserscan_node`, `/sensors/imu_converter_node`, `/livox_static_transform_publisher` |
| T12 | `ros2 topic list \| grep -E 'livox\|imu'` | есть `/sensors/lidar/livox/custom_msg`, `/sensors/lidar/livox/point_cloud2`, `/sensors/lidar/livox/scan`, `/sensors/imu/livox`, `/sensors/imu/unitree_h1` |
| T13 | `ros2 topic list \| grep -E '^/sensors/livox/'` | **пусто** (если топики `/sensors/livox/lidar` или `/sensors/livox/imu` есть, ремапы драйвера не сработали) |
| T14 | `ros2 topic hz /sensors/lidar/livox/custom_msg` (10 с) | около 50 Гц (`publish_freq` в `msg_MID360_launch.py`), разброс не больше нескольких Гц |
| T15 | `ros2 topic hz /sensors/lidar/livox/point_cloud2` | совпадает с T14 |
| T16 | `ros2 topic hz /sensors/lidar/livox/scan` | совпадает с T14 по порядку величины (скан публикуется на каждое облако) |

**T15a. Содержимое сообщения:**

```bash
ros2 topic echo --once /sensors/lidar/livox/point_cloud2 --no-arr
```

PASS: `header.frame_id: livox_frame`, `height: 1`, `width` порядка нескольких тысяч (200 000 точек/с ÷ 50 Гц ≈ 4000),
`point_step: 18`, `row_step = 18 × width`, поля `x, y, z, intensity, tag, line`.

**T15b. Сравнение с исходным сообщением** (главная проверка исправленного конвертера):

```bash
python3 src/tools/check_pointcloud.py --pairs 20
```

PASS: `ИТОГ: PASS`, 20 пар совпали по `width == point_num`, `point_step`, `row_step`, полям и первой/последней точке.

**T15c. Логи** (после минуты работы):

```bash
grep -E "WARN|ERROR|point_num" /tmp/lidar_launch.log | head
```

PASS: нет `point_num ... differs from points.size()` и нет `ERROR`. Единичное предупреждение при старте допустимо.

## 3a. Время и потери данных лидара

Проверки по итогам анализа записанных bag ([BAG_ANALYSIS.md](BAG_ANALYSIS.md)), пункты 4–6. Номера T54–T55 продолжают сквозную нумерацию.

| № | Команда | PASS |
|---|---|---|
| T54 | `ros2 topic echo --once --field header.stamp /sensors/lidar/livox/custom_msg` и сразу `date +%s` | разница секунд не больше 1 с. Если больше (в bag было 8–10 с), часы лидара не синхронизированы с компьютером: зафиксируйте разницу, это влияет на TF и SLAM |
| T55 | `ros2 bag record -o lidar_check --duration 60 /sensors/lidar/livox/custom_msg /sensors/imu/livox`, затем `python3 src/tools/bag_analysis/analyze_livox.py lidar_check --lidar-topic /sensors/lidar/livox/custom_msg --imu-topic /sensors/imu/livox` | кадры лидара идут без пауз около 20 мс, IMU около 200 Гц и пауз длиннее 15 мс меньше 1%. Если в записи потери, а `ros2 topic hz` в T14 в норме, проблема в записи, а не в драйвере |

## 4. Геометрия: TF и облако в RViz

| № | Команда | PASS |
|---|---|---|
| T17a | `ros2 run tf2_ros tf2_echo mid360_link livox_frame` | преобразование печатается (по умолчанию нули — единичное) |
| T17b | `ros2 run tf2_tools view_frames` (Foxy: `view_frames.py`) и откройте `frames.pdf` | `livox_frame` соединён с `mid360_link`, нет второго родителя у `livox_frame` |
| T17c | `ros2 run tf2_ros tf2_echo pelvis mid360_link` | преобразование есть (если нет — `robot_state_publisher` с URDF H1 не запущен, это не вина пакетов) |
| T17d | `ros2 run tf2_ros tf2_echo pelvis imu_link` и `ros2 run tf2_ros tf2_echo pelvis realsense_link` | преобразования есть: фрейм IMU (`imu_link`) и камеры (`realsense_link`) должны приходить из URDF робота, эти пакеты их не публикуют. Если нет, данные IMU и камеры нельзя привязать к роботу в RViz и SLAM |

Визуальная проверка в RViz (`rviz2`): `Fixed Frame = mid360_link`, добавьте `PointCloud2`
(`/sensors/lidar/livox/point_cloud2`, Reliability = Reliable, при проблемах попробуйте Best Effort) и `LaserScan`
(`/sensors/lidar/livox/scan`).

| № | Что смотреть | PASS |
|---|---|---|
| T18 | Пол в облаке; численно: `python3 src/tools/check_lidar_floor.py` | пол горизонтален (нет наклона и переворота), полный наклон не больше 2° |
| T19 | Стена на известном расстоянии (рулетка) | расстояние до стены в облаке по оси $x$ `livox_frame` отличается от рулетки не больше чем на 5 см |
| T20 | Вертикаль | дверной проём или колонна вертикальны |

Если T18 или T20 не проходят — подберите аргументы `lidar_tf_*` и повторите:

```bash
ros2 launch completed_scripts_sensors lidar.launch.py lidar_tf_pitch:=0.28
```

(0.28 — только пример значения в радианах.) Не дублируйте вращение, уже заданное `extrinsic_parameter`
(roll 180°, pitch 16°) в `MID360_config.json`. Найденные значения запишите в `README.md`/launch.

## 5. IMU: `imu_converter`

Робот неподвижен. Узел уже запущен из `lidar.launch.py` (если нет: `ros2 run imu_converter imu_converter_node`).

| № | Команда | PASS |
|---|---|---|
| T21 | `ros2 topic hz /sensors/imu/unitree_h1` | частота не ниже 95% от частоты `/lowstate` из T4 |
| T22 | `ros2 topic echo --once /sensors/imu/unitree_h1` | `frame_id: imu_link`; в матрицах `*_covariance` все нули (по умолчанию) |
| T23 | `ros2 param get /sensors/imu_converter_node frame_id` | `imu_link` |
| T24 | `python3 src/tools/check_imu.py --duration 10` | `ИТОГ: PASS` |

Что проверяет T24 (по пунктам, чтобы понимать смысл):

1. Порядок кватерниона: сравнивает `LowState.imu_state.quaternion` (порядок $w,x,y,z$) с `Imu.orientation` (порядок $x,y,z,w$).
2. Норма кватерниона $\sqrt{w^2+x^2+y^2+z^2} \approx 1$.
3. Модуль ускорения в покое $\|a\| \approx 9.81$ м/с² (допуск ±0.5).
4. Средняя угловая скорость ≈ 0 (допуск 0.05 рад/с).
5. Ковариации нулевые.

Скрипт печатает среднее и дисперсию по осям в покое. Они нужны для параметров
`orientation_variance`, `angular_velocity_variance`, `linear_acceleration_variance` (проверка 5 при этом перестанет
проходить — это нормально, значит ковариации заданы).

Для дисперсий лучше записать более длинный отрезок:

```bash
ros2 bag record -o ~/bags/imu_static /lowstate /sensors/imu/unitree_h1     # 60 с, затем Ctrl+C
python3 src/tools/check_imu.py --duration 60
```

Опциональная проверка направления (только если робот в безопасном состоянии и это допускает инструкция
производителя): медленно наклоните корпус и убедитесь в RViz (дисплей `Imu`, `Fixed Frame = imu_link`), что ориентация
поворачивается в ту же сторону, что корпус.

## 5a. Камера: `realsense_d435i_configs`

Камера подключена к USB 3 (`lsusb -t` показывает 5000M). Нужен установленный `realsense2_camera`. Номера T44–T53 продолжают
сквозную нумерацию, чтобы не менять остальные.

| № | Команда | PASS |
|---|---|---|
| T44 | `colcon test --packages-select realsense_d435i_configs && colcon test-result --verbose` | оба набора тестов пройдены (настоящим pytest; до этого они гонялись только упрощённым запуском) |
| T45 | `rs-enumerate-devices -s` (или `lsusb \| grep -i intel`) | D435i виден; запишите серийный номер |
| T46 | `ros2 launch realsense_d435i_configs d435i.launch.py mode:=rgb_depth` | камера стартует за ~10 с, в логе нет `ERROR`; нода `/camera/camera` |
| T47 | `ros2 param get /camera/camera rgb_camera.color_profile` (на старой версии `rgb_camera.profile`) | `1280x720x30` |
| T48 | `ros2 topic echo --once /camera/camera/color/camera_info --no-arr \| grep -E 'width\|height'` | `1280` и `720` (профиль применился, а не взят по умолчанию) |
| T49 | `ros2 topic hz /camera/camera/color/image_raw` и `.../aligned_depth_to_color/image_raw` | около 30 Гц у обоих |
| T50 | Остановить, запустить `mode:=rgb_depth_pointcloud` (он же режим по умолчанию; в логе нет ошибки про диапазон `clip_distance`, значение 30.0), затем `ros2 topic hz /camera/camera/depth/color/points` | около 30 Гц; `width x height` облака 424x240 (из-за `decimation_filter`) |
| T51 | `ros2 run tf2_tools view_frames` (Foxy: `view_frames.py`) при запуске с `camera_name:=front_cam` | фреймы называются `front_cam_*`, а не `camera_*` (проверка, что `camera_name` доходит до ноды) |
| T52 | `ros2 launch completed_scripts_sensors realsense.launch.py` (по умолчанию режим с облаком), затем `ros2 topic list \| grep points` и `ros2 run tf2_ros tf2_echo realsense_link camera_link` | есть `/camera/camera/depth/color/points`; преобразование печатается (нули — единичное); с `enable_front_camera:=False` камера не запускается |
| T53 | Запуск `mode:=rgb_usb3`, `top -b -n 1 \| grep realsense` | запишите загрузку CPU (на Jetson 1920x1080 может быть заметной) |

Пресеты `rgb_usb2` и `rgb_usb3` — по необходимости (T46–T49 с другим `mode`).

## 6. Одометрия: `odom_converter`

Нужны `unitree_sdk2py` и `high_level_control` (из `unitree_h1_control_ws`) в окружении.

```bash
ros2 run odom_converter odom_converter_node                       # или с интерфейсом DDS:
ros2 run odom_converter odom_converter_node --ros-args -p network_interface:=<интерфейс>
```

### 6.1 Запуск

| № | Что смотреть | PASS |
|---|---|---|
| T25 | Лог после старта | строка `Odometry converter started: odommodestate + lowstate -> odom (odom -> pelvis, publish_tf=False)`, **нет** traceback |
| T26 | Если `high_level_control` специально убрать из окружения | узел завершается с сообщением `Cannot import unitree_sdk2py / high_level_control ...`, а не просто `ModuleNotFoundError` (необязательная проверка) |
| T27 | `ros2 topic hz /odom` | частота совпадает с `ros2 topic hz /odommodestate` |

### 6.2 Содержимое и главная регрессия (порядок кватерниона)

| № | Команда | PASS |
|---|---|---|
| T28 | `python3 src/tools/check_odom.py --duration 10` | `ИТОГ: PASS`: `frame_id = odom`, `child_frame_id = pelvis`, норма кватерниона ≈ 1, угол между ориентацией `/odom` и `/sensors/imu/unitree_h1` не больше 2° |

Если T28 даёт угол около 180°, порядок компонент снова перепутан (именно это было ошибкой в версии 1.0.0).
Для ручной проверки: у неподвижного робота `ros2 topic echo --once /odom --no-arr` показывает
`orientation.w` близким к $\cos(\psi/2)$, где $\psi$ — рыскание (если крен и тангаж малы); при $\psi \approx 0$
получаем $w \approx 1$, $x \approx y \approx z \approx 0$.

### 6.3 Параметры и TF

| № | Команда | PASS |
|---|---|---|
| T29 | `ros2 param get /odometry_converter odom_frame` и `... child_frame` | `odom` и `pelvis` |
| T30 | `ros2 run odom_converter odom_converter_node --ros-args -p publish_tf:=true`, затем `ros2 run tf2_ros tf2_echo odom pelvis` | преобразование `odom → pelvis` печатается, положение совпадает с `/odom` |
| T31 | В лог при запуске без `LowState` (остановите публикацию `/lowstate`) | предупреждение `Waiting for LowState data...` не чаще раза в 5 с |

В T30 убедитесь, что TF `odom → pelvis` больше никто не публикует (`ros2 run tf2_tools view_frames`), иначе будет конфликт.

### 6.4 Движение (под присмотром оператора)

Команды движения подаёт оператор штатным пультом. Во время движения в отдельном терминале запустите
`python3 src/tools/check_odom.py --duration <время движения>`.

| № | Сценарий | PASS |
|---|---|---|
| T32 | Пройти по прямой примерно 1 м (измерьте рулеткой) | «по плоскости» в выводе скрипта отличается от рулетки не больше чем на 10% (ориентир) |
| T33 | Повернуться на месте на 90° влево | рыскание в выводе скрипта изменилось примерно на $+90^\circ$ (знак: против часовой стрелки — плюс) |
| T34 | Остановиться | положение перестаёт меняться, скорость ≈ 0 |

**T35: система координат скорости** (открытый вопрос рефакторинга). Для `nav_msgs/Odometry` скорость `twist.linear`
должна быть выражена в `child_frame` (`pelvis`). Проверка: повернитесь на 90° и пройдите вперёд.

- Если средняя скорость в выводе скрипта $v_x > 0$, $v_y \approx 0$ при любом курсе — скорость в связанной системе, всё
  правильно.
- Если после поворота на 90° получается $v_x \approx 0$, $v_y > 0$ — скорость в мировой системе, и узел нужно
  доработать: повернуть вектор в `pelvis`, то есть $\mathbf{v}_{body} = R(\psi)^{\top}\mathbf{v}_{world}$.

Результат T35 запишите в `README.md` пакета `odom_converter` и в `CHANGELOG.md`.

### 6.5 Остановка

| № | Действие | PASS |
|---|---|---|
| T36 | Ctrl+C в терминале узла | завершение без traceback; в логе нет `Failed to disable odometry` |
| T37 | Сразу запустить узел снова | запускается без ошибок (повторное включение одометрии проходит) |

## 7. Стабильность

Запустите `lidar.launch.py` и `odom_converter_node` минимум на 30 минут.

| № | Команда | PASS |
|---|---|---|
| T38 | `top -b -n 1 -o %CPU \| head -15` в начале и в конце | CPU не растёт заметно со временем; запишите нагрузку узлов `livox_ros_driver2_node`, `livox_to_pointcloud2_node` |
| T39 | `ros2 topic hz /sensors/lidar/livox/custom_msg` в конце | частота та же, что в T14 |
| T40 | `ros2 node list` в конце | все узлы из T11 на месте (ни один не упал) |
| T41 | `grep -cE "ERROR" /tmp/lidar_launch.log` | 0 |

Устойчивость к сбоям (необязательно, но показательно):

| № | Действие | Ожидание |
|---|---|---|
| T42 | Отключить кабель лидара на 10 с, затем вернуть | узлы не падают; конвертер просто перестаёт получать данные. Возобновится ли поток — это поведение драйвера, зафиксируйте результат |
| T43 | Запустить `imu_converter_node` до запуска робота/`/lowstate` | узел ждёт, не падает, после появления `/lowstate` публикует |

## 8. Если проверка не прошла: куда смотреть

| Симптом | Где искать |
|---|---|
| T13 не пусто | ремапы в `livox_ros_driver2/launch_ROS2/msg_MID360_launch.py` (должны быть относительными) |
| Нет `/sensors/lidar/livox/point_cloud2` | `lidar.launch.py` (ремапы конвертера), лог `livox_to_pointcloud2` |
| Есть cloud, нет scan | TF `mid360_link → livox_frame` (T17); затем `ros2 launch completed_scripts_sensors lidar.launch.py transform_tolerance:=5.01` (старое значение: если помогло, причина во времени или TF, см. T54) |
| `check_pointcloud.py`: «слишком мало пар» | не совпадают `header.stamp` (конвертер копирует заголовок, значит, проблема в QoS/потерях) или нет данных на одном топике |
| `check_imu.py`: расхождение кватерниона | порядок компонент, версия `unitree_go`; сравните ручной `echo` `/lowstate` и `Imu` |
| `check_odom.py`: угол ≈ 180° | порядок кватерниона в `fill_odometry_message` |
| Топик есть, данных нет | QoS: `ros2 topic info -v <топик>` (Reliability публикатора и подписчика должны быть совместимы) |

## 9. Итоговая таблица

Копируйте её в отчёт и отмечайте результаты. Пакеты считаются исправными, если выполнены все обязательные пункты.

| Пакет | Обязательные проверки | Результат |
|---|---|---|
| сеть и железо | T1, T2, T3, T4 | ☐ |
| сборка и тесты | T5, T6, T7, T8, T9 | ☐ |
| `livox_ros_driver2` | T11 (драйвер), T12, T13, T14, T54 | ☐ |
| `livox_to_pointcloud2` | T15, T15a, T15b (`check_pointcloud.py`), T15c (лог без `point_num`) | ☐ |
| `pointcloud_to_laserscan` + TF | T16, T17a, T17b, T18, T19, T20 | ☐ |
| `imu_converter` | T21, T22, T23, T24 | ☐ |
| `odom_converter` | T25, T27, T28, T29, T32, T33, T35, T36, T37 | ☐ |
| `completed_scripts_sensors` | T10, T11, T12, T52 | ☐ |
| `realsense_d435i_configs` | T44, T45, T46, T47, T48, T49, T51 | ☐ |
| стабильность | T38, T39, T40, T41 | ☐ |

Необязательные: T26, T30, T31, T34, T42, T43, T17c, T17d, T50, T53, T55.

После прохождения: обновите статусы в [PACKAGES.md](PACKAGES.md) (🟡 → 🟢), впишите результаты и найденные значения
(`lidar_tf_*`, дисперсии IMU, вывод по T35) в `CHANGELOG.md` соответствующих пакетов и снимите пометки
**[робот]** в `TODO.md`.
