# Методичка: проверка сенсорики Unitree H1 на реальном роботе

Для кого: для исполнителя проверок (лаборатория, доступ к роботу). Что проверяем: пакеты рабочего пространства
`unitree_h1_sensors_ws` — лидар Livox MID360, IMU и одометрия Unitree, камера RealSense D435i. Почему это нужно:
весь код проверен только на заглушках и на записанных bag, на настоящем роботе и в настоящем ROS-запуске он ещё
не работал (подробности — в [PACKAGES.md](PACKAGES.md)).

Версии на момент написания: `livox_ros_driver2` 1.1.1, `livox_to_pointcloud2` 0.1.1, `imu_converter` 1.1.0,
`odom_converter` 2.0.0, `completed_scripts_sensors` 1.1.0, `realsense_d435i_configs` 0.3.0, `pointcloud_to_laserscan` 2.0.1.

Время: примерно 4–5 часов с перерывами (блоки A–C около часа, D около часа, E около 1.5 с экспериментами, F около часа, G около часа, H 40 минут).
Блоки можно делать в разные дни, но каждый целиком.

## 0. Как работать с этим документом

1. Идите по блокам A → H по порядку. Внутри блока шаги идут по номерам проверок (**T1**, **T2** …).
2. У каждой проверки есть **команда**, **ожидается** (критерий PASS) и, если не получилось, **что делать**.
3. **Ничего не чините по ходу.** Не меняйте код, конфиги и параметры, кроме тех, что указаны в шаге. Найденное — это результат
   проверки: запишите его и покажите Claude. Исключение: аргументы командной строки, которые методичка просит подобрать.
4. Копируйте вывод команд **целиком**, а не пересказывайте. Выводы сохраняйте в файлы (см. раздел 3).
5. В конце заполните протокол и отправьте его Claude вместе с файлами (раздел 12).
6. «Допуски» в таблицах — ориентиры из конфигурации и характеристик датчиков, а не гарантии. Если у вас стабильно другие
   значения, это не обязательно ошибка, но это надо записать.

Сопутствующие документы: список проверок в виде таблиц — [HARDWARE_TESTING.md](HARDWARE_TESTING.md) (те же номера T),
открытые вопросы, которые закрываются по результатам, — [ROBOT_OPEN_QUESTIONS.md](ROBOT_OPEN_QUESTIONS.md) (номера Q).

## 1. Безопасность

- В методичке **нет** ни одной команды, которая двигает робота. Движение задаёт только оператор штатным пультом.
- Все проверки, кроме блока F (шаги T32–T35), выполняются с **неподвижным** роботом в безопасном состоянии
  (по инструкции производителя: на подвесе, подставке или стоящим без команд движения).
- Для шагов с движением нужны: обученный оператор с пультом, свободная площадка не меньше 3×3 м, второй человек рядом,
  доступная аварийная остановка. Если чего-то из этого нет, шаги T32–T35 пропустите и отметьте «пропущено, нет условий».
- Узел `odom_converter` при старте включает одометрию робота (`OdomClient.EnableOdom()`), а при остановке выключает.
  Это не движение, но вызов сервиса робота. Запускайте его только при готовом к работе роботе.
- Лидар излучает лазер класса 1 (безопасен для глаз при штатной работе). Не смотрите на него с близкого расстояния через оптику.
- Если что-то ведёт себя странно (робот реагирует на ваши команды, неожиданные звуки), остановите узлы (Ctrl+C), сообщите оператору.

## 2. Что нужно

| Что | Зачем |
|---|---|
| Unitree H1 с включённым питанием и сетью, доступ по `ssh` к его вычислителю или ноутбук, подключённый к сети робота | источники данных |
| Лидар MID360 на роботе, адрес `192.168.123.120`, у компьютера адрес `192.168.123.162` | блоки A–D |
| ROS 2 Humble (или Foxy), собранный `unitree_ros2` (сообщения `unitree_go`), `unitree_sdk2py`, `high_level_control` из `unitree_h1_control_ws` | блоки E, F |
| Livox-SDK2 (`liblivox_lidar_sdk_shared.so`) | драйвер лидара |
| Камера D435i на USB 3, пакет `realsense2_camera` | блок G |
| Рулетка, ровный пол без предметов 3 м вокруг робота, стена с известным расстоянием | блок D |
| Обновлённый `src/` из архива `unitree_h1_sensors_ws.zip` | код |
| Ноутбук с `rviz2` | визуальные проверки |

Если чего-то нет, это не повод останавливаться: пропустите блок, где оно нужно, и запишите причину.

## 3. Подготовка (один раз)

```bash
source /opt/ros/humble/setup.bash                                    # или foxy
source ~/unitree_ros2/cyclonedds_ws/install/local_setup.bash         # поправьте путь, даёт unitree_go
export WS=~/unitree_h1_sensors_ws                                    # поправьте путь к workspace
export RUN=~/h1_check_$(date +%F)                                    # папка для всех выводов
mkdir -p $RUN
cd $WS
```

Все команды ниже предполагают, что переменные `WS` и `RUN` заданы и выполнен `source`. Если вы открыли новый терминал,
повторите эти пять строк (последние две: `export` и `cd`).

Если сеть робота требует отдельного DDS-интерфейса, настройте его так, как вы делаете для остальных проектов
(`CYCLONEDDS_URI` и т. п.), и запишите в протокол, как.

Сохраните сведения об окружении (приложите к отчёту):

```bash
$WS/src/tools/collect_env.sh $RUN/env.txt
```

Как сохранять вывод любой команды: `команда 2>&1 | tee $RUN/<имя>.log`. Для запуска, который работает долго,
вывод пишется в файл, а проверки идут в соседних терминалах.

## 4. Блок A. Сеть и железо (T1–T4)

| № | Команда | Ожидается |
|---|---|---|
| T1 | `ip -4 addr \| grep 192.168.123.162` | у компьютера есть адрес `192.168.123.162` (тот же, что `host_ip` в `MID360_config.json`) |
| T2 | `ping -c 5 192.168.123.120` | 0% потерь (лидар отвечает) |
| T3 | `ros2 topic list \| grep -E '^/(lowstate\|odommodestate)$'` | оба топика есть |
| T4 | `ros2 topic hz /lowstate` (остановить через 10 с) | стабильный поток; **запишите частоту** (около 500 Гц), она нужна в T21 |

Если T1 не проходит: проверьте IP сетевой карты и маску (`192.168.123.x/24`). Если T2: кабель и питание лидара. Если T3:
`source` из раздела 3 и сетевой интерфейс DDS. Без T1–T4 дальнейшие блоки не имеют смысла: остановитесь и сообщите Claude.

## 5. Блок B. Сборка и тесты (T5–T9)

Если на этом же компьютере и workspace уже были сделаны чистая сборка без ошибок и `test_conversion` (9 из 9 тестов проходят),
отметьте T5–T9 как «выполнено ранее» и перейдите к блоку C. Иначе:

```bash
cd $WS && rm -rf build install log
./src/install_dependencies.bash 2>&1 | tee $RUN/deps.log
colcon build 2>&1 | tee $RUN/build.log
source install/setup.bash
```

| № | Команда | Ожидается |
|---|---|---|
| T5 | `tail -15 $RUN/build.log` и `grep -nE "warning\|error" $RUN/build.log` | `Summary: 7 packages finished`, нет `failed`, нет предупреждений про `tf2_sensor_msgs.h` и `dash-separated` |
| T6 | `colcon test --packages-select imu_converter odom_converter --pytest-args -k test_conversion && colcon test-result --verbose` | 9 тестов пройдено, 0 ошибок |
| T7 | `ros2 interface show livox_ros_driver2/msg/CustomMsg` | поля `header`, `timebase`, `point_num`, `lidar_id`, `points[]` |
| T8 | `ros2 pkg executables odom_converter` | только `odom_converter odom_converter_node` |
| T9 | `ros2 pkg executables imu_converter && ros2 pkg executables livox_to_pointcloud2` | `imu_converter_node` и `livox_to_pointcloud2_node` |

Шаблонные тесты `test_flake8`, `test_pep257`, `test_copyright` могут падать из-за стиля: запишите, какие, это не ошибка работы узлов.

## 6. Блок C. Лидар: драйвер → PointCloud2 → LaserScan (T10–T16)

Терминал 1 (оставьте открытым, вывод в файл):

```bash
ros2 launch completed_scripts_sensors lidar.launch.py 2>&1 | tee $RUN/lidar_launch.log
```

Терминал 2:

| № | Команда | Ожидается |
|---|---|---|
| T10 | `ros2 launch completed_scripts_sensors lidar.launch.py --show-args` | есть `lidar_tf_x … lidar_tf_roll` и `transform_tolerance` |
| T11 | `ros2 node list` | `/sensors/livox_lidar_publisher`, `/sensors/livox_to_pointcloud2`, `/sensors/pointcloud_to_laserscan_node`, `/sensors/imu_converter_node`, `/livox_static_transform_publisher` |
| T12 | `ros2 topic list \| grep -E 'livox\|imu'` | `/sensors/lidar/livox/custom_msg`, `/sensors/lidar/livox/point_cloud2`, `/sensors/lidar/livox/scan`, `/sensors/imu/livox`, `/sensors/imu/unitree_h1` |
| T13 | `ros2 topic list \| grep -E '^/sensors/livox/'` | **пусто** (иначе ремапы драйвера не сработали) |
| T14 | `ros2 topic hz /sensors/lidar/livox/custom_msg` (10 с) | около 50 Гц, разброс не больше нескольких Гц; запишите среднее |
| T15 | `ros2 topic hz /sensors/lidar/livox/point_cloud2` | совпадает с T14 |
| T16 | `ros2 topic hz /sensors/lidar/livox/scan` | совпадает с T14 по порядку величины |

**T15a. Содержимое облака:**

```bash
ros2 topic echo --once /sensors/lidar/livox/point_cloud2 --no-arr | tee $RUN/cloud_once.txt | grep -E "frame_id|height|width|point_step|row_step"
```

Ожидается: `frame_id: livox_frame`, `height: 1`, `width` порядка 4000, `point_step: 18`, `row_step` = 18 × `width`.

**T15b. Сверка с исходным сообщением** (главная проверка исправленного конвертера):

```bash
python3 $WS/src/tools/check_pointcloud.py --pairs 20 2>&1 | tee $RUN/check_pointcloud.log
```

Ожидается: `ИТОГ: PASS`.

**T15c. Логи (после минуты работы):**

```bash
grep -E "WARN|ERROR|point_num" $RUN/lidar_launch.log | head -20
```

Ожидается: нет `ERROR` и нет `point_num ... differs from points.size()`. Единичное предупреждение при старте допустимо.

**T54. Согласованность часов лидара** (в записи, сделанной раньше, штамп лидара опережал время компьютера на 8–10 с):

```bash
ros2 topic echo --once --field header.stamp /sensors/lidar/livox/custom_msg; date +%s
```

Ожидается: поле `sec` отличается от `date +%s` не больше чем на 1. Если больше, **запишите разницу** (вопрос Q2); дальше всё равно продолжайте.

**T55. Потери данных** (60 секунд):

```bash
ros2 bag record -o $RUN/lidar_check --duration 60 /sensors/lidar/livox/custom_msg /sensors/imu/livox
python3 $WS/src/tools/bag_analysis/analyze_livox.py $RUN/lidar_check \
  --lidar-topic /sensors/lidar/livox/custom_msg --imu-topic /sensors/imu/livox | tee $RUN/lidar_check.txt
```

Ожидается: паузы между кадрами лидара около 0.02 с, IMU около 200 Гц и пауз длиннее 15 мс меньше 1%. Если в записи потери, а T14 в норме,
значит проблема записи (сеть, диск), а не драйвера; запишите оба результата.

Пока лидар работает, снимите **Q5** (единицы IMU лидара):

```bash
ros2 topic echo --once /sensors/imu/livox --field linear_acceleration
```

На неподвижном лидаре модуль вектора $\sqrt{x^2+y^2+z^2}$ около 1 означает единицы g, около 9.8 — м/с². Запишите числа.

Не останавливайте launch: он нужен в блоках D и E.

## 7. Блок D. Геометрия: TF и положение лидара (T17–T20)

### 7.1 TF

| № | Команда | Ожидается |
|---|---|---|
| T17a | `ros2 run tf2_ros tf2_echo mid360_link livox_frame` | преобразование печатается (по умолчанию нули) |
| T17b | `ros2 run tf2_tools view_frames` (Foxy: `view_frames.py`), откройте `frames.pdf` | `livox_frame` соединён с `mid360_link`, второго родителя у `livox_frame` нет |
| T17c | `ros2 run tf2_ros tf2_echo pelvis mid360_link` | преобразование есть |
| T17d | `ros2 run tf2_ros tf2_echo pelvis imu_link` и `ros2 run tf2_ros tf2_echo pelvis realsense_link` | преобразования есть |

T17c и T17d зависят от URDF робота (`robot_state_publisher`), а не от этих пакетов. Если их нет, запишите, какие именно фреймы
отсутствуют (вопрос Q4); дальше можно продолжать, но шаги RViz делайте в кадре `mid360_link`.

### 7.2 Облако в RViz

Запустите `rviz2`. `Fixed Frame` = `mid360_link`. Добавьте дисплеи:
`PointCloud2` (`/sensors/lidar/livox/point_cloud2`, Reliability = Reliable; если облака нет, попробуйте Best Effort) и
`LaserScan` (`/sensors/lidar/livox/scan`). Кроме того, добавьте инструмент **Publish Point** (панель инструментов): клик по точке
печатает её координаты в `/clicked_point`:

```bash
ros2 topic echo /clicked_point
```

| № | Что делать | Ожидается |
|---|---|---|
| T18 | Посмотрите на пол сбоку (вид Orbit). Затем запустите оценку наклона (см. 7.3) | пол горизонтален, полный наклон не больше 2° |
| T19 | Выберите стену на известном расстоянии (измерьте рулеткой от центра лидара по горизонтали вдоль оси $x$). Кликните Publish Point по стене прямо перед роботом | $x$ из `/clicked_point` отличается от рулетки не больше чем на 5 см |
| T20 | Посмотрите на вертикальные предметы (дверной проём, колонна) | вертикальны, не наклонены |

Заметка: если URDF задаёт `mid360_link` с наклоном, пол в этом кадре будет наклонён. Тогда повторите T18 с `Fixed Frame = pelvis`
(нужен T17c), где пол обязан быть горизонтальным; запишите, в каком кадре проверяли.

### 7.3 Наклон пола численно

Робот стоит на ровном полу, вокруг свободное место.

```bash
python3 $WS/src/tools/check_lidar_floor.py 2>&1 | tee $RUN/floor_before.txt
```

Скрипт печатает наклон пола вокруг осей $x$ и $y$ и высоту лидара над полом. Ожидается `ИТОГ: PASS` (полный наклон не больше 2°),
высота должна совпасть с рулеткой с точностью нескольких сантиметров.

Если наклон больше 2°, подберите поправку с обратной связью. Пример для наклона вокруг оси $y$ (значения в радианах, 1° ≈ 0.0175 рад):

```bash
# остановите launch (Ctrl+C в терминале 1) и запустите с поправкой, например:
ros2 launch completed_scripts_sensors lidar.launch.py lidar_tf_pitch:=0.0873 2>&1 | tee $RUN/lidar_launch_tilt.log
# в другом терминале:
python3 $WS/src/tools/check_lidar_floor.py 2>&1 | tee $RUN/floor_after.txt
```

Наклон должен уменьшиться. Если вырос, смените знак поправки (`-0.0873`). Для наклона вокруг оси $x$ используйте `lidar_tf_roll`.
Повторяйте, пока полный наклон не станет не больше 1–2°. **Запишите итоговые значения `lidar_tf_*`** (вопрос Q1) — сами в файлы
их не вносите, это сделает Claude вместе с вами.

Не дублируйте вращение, уже заданное в драйвере (`extrinsic_parameter`: roll 180°, pitch 16° в `MID360_config.json`).

## 8. Блок E. IMU: `imu_converter` (T21–T24) и эксперименты

Узел уже запущен из `lidar.launch.py`. Робот неподвижен.

| № | Команда | Ожидается |
|---|---|---|
| T21 | `ros2 topic hz /sensors/imu/unitree_h1` | частота не ниже 95% от частоты `/lowstate` из T4 |
| T22 | `ros2 topic echo --once /sensors/imu/unitree_h1` | `frame_id: imu_link`; все `*_covariance` нулевые |
| T23 | `ros2 param get /sensors/imu_converter_node frame_id` | `imu_link` |
| T24 | `python3 $WS/src/tools/check_imu.py --duration 10 2>&1 \| tee $RUN/check_imu.log` | `ИТОГ: PASS` |

Что проверяет T24: порядок компонент кватерниона ($w,x,y,z$ у Unitree → $x,y,z,w$ у ROS), его норму ≈ 1, модуль ускорения ≈ 9.81 м/с²,
среднюю угловую скорость ≈ 0, нулевые ковариации. Главное из них — порядок кватерниона. Если FAIL из-за ускорения или
угловой скорости, проверьте, что робот действительно неподвижен, и повторите.

Необязательно (только если инструкция производителя это допускает): медленно наклоните корпус и убедитесь в RViz (дисплей `Imu`,
`Fixed Frame = imu_link`), что ориентация поворачивается в ту же сторону.

### 8.1 Эксперимент «старт робота» (вопрос Q6)

Цель: узнать, присылает ли робот нулевой кватернион в первые секунды после включения (от этого зависит защита в `imu_converter`).

1. Ноутбук подключён к сети робота, `source` выполнен. Робот **выключен** (или перезапускается по инструкции).
2. Запустите запись **до** включения робота (она пока ничего не пишет, ждёт топик):
   ```bash
   ros2 bag record -o $RUN/lowstate_boot /lowstate
   ```
3. Включите робота обычным порядком, дождитесь его готовности (около минуты после появления `/lowstate`). Остановите запись (Ctrl+C).
4. Разберите запись:
   ```bash
   python3 $WS/src/tools/bag_analysis/analyze_lowstate.py $RUN/lowstate_boot | tee $RUN/lowstate_boot.txt
   ```
   Смотрите строку «нулевых». Если нулевых кватернионов нет, защита не нужна, если есть — нужна.
5. Повторите 2–3 раза, если время позволяет. Если включение робота вы делать не можете, отметьте «пропущено».

### 8.2 Эксперимент «шум IMU» (вопрос Q7)

Цель: получить дисперсии для параметров `orientation_variance`, `angular_velocity_variance`, `linear_acceleration_variance`.
Робот в безопасном **неподвижном** состоянии, рядом никто не ходит и ничего не вибрирует (не стучите по столу).

```bash
ros2 bag record -o $RUN/imu_static --duration 600 /lowstate
python3 $WS/src/tools/bag_analysis/analyze_lowstate.py --static $RUN/imu_static | tee $RUN/imu_static.txt
```

Ожидается: смещение гироскопа порядка сотых рад/с, дисперсии заметно меньше 1. В конце скрипт печатает три значения для параметров.
**В код их не вносите**: отправьте файл `imu_static.txt` Claude.

## 9. Блок F. Одометрия: `odom_converter` (T25–T37)

Нужны `unitree_sdk2py` и `high_level_control`. Узел при старте включает одометрию робота (см. раздел 1).

```bash
ros2 run odom_converter odom_converter_node 2>&1 | tee $RUN/odom_node.log
# если нужен интерфейс DDS:
ros2 run odom_converter odom_converter_node --ros-args -p network_interface:=<интерфейс> 2>&1 | tee $RUN/odom_node.log
```

### 9.1 Запуск и содержимое

| № | Что смотреть | Ожидается |
|---|---|---|
| T25 | Лог после старта | строка `Odometry converter started: odommodestate + lowstate -> odom (odom -> pelvis, publish_tf=False)`, **нет** traceback |
| T27 | `ros2 topic hz /odom` и `ros2 topic hz /odommodestate` | частоты совпадают |
| T28 | `python3 $WS/src/tools/check_odom.py --duration 10 2>&1 \| tee $RUN/check_odom.log` | `ИТОГ: PASS`: `frame_id = odom`, `child_frame_id = pelvis`, норма кватерниона ≈ 1, угол между ориентацией `/odom` и `/sensors/imu/unitree_h1` не больше 2° |
| T29 | `ros2 param get /odometry_converter odom_frame` и `... child_frame` | `odom` и `pelvis` |

Если T28 даёт угол около 180°, порядок компонент перепутан (так было в исходной версии) — это серьёзная находка.

Необязательно: T26 (без `high_level_control` узел должен сообщить `Cannot import unitree_sdk2py / high_level_control ...`, а не `ModuleNotFoundError`),
T30 (`-p publish_tf:=true`, затем `ros2 run tf2_ros tf2_echo odom pelvis`; убедитесь, что TF `odom → pelvis` никто больше не публикует),
T31 (без `/lowstate` в логе предупреждение `Waiting for LowState data...` не чаще раза в 5 с).

### 9.2 Движение под присмотром оператора (T32–T35)

Только при выполнении условий из раздела 1. Команды движения подаёт оператор пультом. На время движения запустите
`python3 $WS/src/tools/check_odom.py --duration <секунд>` с запасом по времени.

| № | Сценарий | Ожидается |
|---|---|---|
| T32 | Пройти по прямой примерно 1 м (замерьте рулеткой) | «по плоскости» в выводе скрипта отличается от рулетки не больше чем на 10% |
| T33 | Повернуться на месте на 90° влево | рыскание изменилось примерно на $+90^\circ$ (против часовой стрелки — плюс) |
| T34 | Остановиться | положение перестаёт меняться, скорость ≈ 0 |
| T35 | Повернуться на 90° и пройти вперёд ~1 м | см. ниже |

**T35 (вопрос Q8): в какой системе координат скорость.** `Odometry.twist.linear` по стандарту должна быть в `child_frame` (`pelvis`).
Смотрите среднюю скорость, которую печатает `check_odom.py`:

- если $v_x > 0$ и $v_y \approx 0$ при любом курсе — скорость в связанной системе, всё правильно;
- если после поворота на 90° получается $v_x \approx 0$, $v_y > 0$ — скорость в мировой системе, узел нужно доработать (Claude сделает по вашим числам).

Запишите оба случая: до поворота и после.

### 9.3 Остановка

| № | Действие | Ожидается |
|---|---|---|
| T36 | Ctrl+C в терминале узла | завершение без traceback; в логе нет `Failed to disable odometry` |
| T37 | Сразу запустить узел снова | запускается без ошибок |

## 10. Блок G. Камера D435i (T44–T53)

Камера подключена к USB 3 (`lsusb -t` показывает `5000M`). Остановите прочие запуски камеры.

| № | Команда | Ожидается |
|---|---|---|
| T44 | `colcon test --packages-select realsense_d435i_configs && colcon test-result --verbose` | все тесты пройдены |
| T45 | `rs-enumerate-devices -s` (или `lsusb \| grep -i intel`) | D435i виден; запишите серийный номер |
| T46 | `ros2 launch realsense_d435i_configs d435i.launch.py mode:=rgb_depth 2>&1 \| tee $RUN/cam_rgb_depth.log` | камера стартует за ~10 с, нет `ERROR` в логе, нода `/camera/camera` |
| T47 | `ros2 param get /camera/camera rgb_camera.color_profile` (на старой версии пакета `rgb_camera.profile`) | `1280x720x30` |
| T48 | `ros2 topic echo --once /camera/camera/color/camera_info --no-arr \| grep -E 'width\|height'` | `1280` и `720` |
| T49 | `ros2 topic hz /camera/camera/color/image_raw` и `.../aligned_depth_to_color/image_raw` | около 30 Гц у обоих |

Дальше остановите запуск (Ctrl+C) и проверьте режим с облаком (он же режим по умолчанию):

| № | Команда | Ожидается |
|---|---|---|
| T50 | `ros2 launch realsense_d435i_configs d435i.launch.py mode:=rgb_depth_pointcloud 2>&1 \| tee $RUN/cam_pointcloud.log`, затем `ros2 topic hz /camera/camera/depth/color/points` и `ros2 param get /camera/camera clip_distance` | около 30 Гц; `clip_distance` = 30.0; **в логе нет ошибки про диапазон параметра** (вопрос Q11) |
| T51 | остановить, запустить с `camera_name:=front_cam`, затем `ros2 run tf2_tools view_frames` (Foxy: `view_frames.py`) | фреймы называются `front_cam_*`, а не `camera_*` (вопрос Q13) |
| T52 | остановить; `ros2 launch completed_scripts_sensors realsense.launch.py`, затем `ros2 topic list \| grep points` и `ros2 run tf2_ros tf2_echo realsense_link camera_link` | есть `/camera/camera/depth/color/points`; преобразование печатается (нули — единичное); с `enable_front_camera:=False` камера не запускается |
| T53 | `ros2 launch realsense_d435i_configs d435i.launch.py mode:=rgb_usb3` и в другом терминале `top -b -n 1 \| grep -i realsense` | запишите загрузку CPU (вопрос Q14) |

Если в T50 нода отвергла `clip_distance: 30.0`, **не правьте файл сами**: сохраните лог (`cam_pointcloud.log`) и покажите Claude, он предложит значение.

Визуально: запустите `ros2 run rqt_image_view rqt_image_view` (картинка цветной камеры) и в RViz добавьте `PointCloud2`
`/camera/camera/depth/color/points` (кадр `camera_color_optical_frame` или `realsense_link`). Облако должно быть цветным и видеть
предметы дальше 3 м (раньше обрезалось). Опишите, что видите.

Для вопроса Q12 запишите версию драйвера камеры: `dpkg -l | grep realsense2-camera`.

## 11. Блок H. Стабильность (T38–T43)

Запустите вместе `lidar.launch.py`, `odom_converter_node` и камеру (`realsense.launch.py`) минимум на 30 минут, робот неподвижен.

| № | Команда | Ожидается |
|---|---|---|
| T38 | `top -b -n 1 -o %CPU \| head -15` в начале и в конце | CPU не растёт заметно; запишите нагрузку `livox_ros_driver2_node`, `livox_to_pointcloud2_node` |
| T39 | `ros2 topic hz /sensors/lidar/livox/custom_msg` в конце | частота та же, что в T14 |
| T40 | `ros2 node list` в конце | все узлы из T11 на месте (никто не упал) |
| T41 | `grep -cE "ERROR" $RUN/lidar_launch.log` | 0 |

Устойчивость к сбоям (необязательно):

| № | Действие | Ожидание |
|---|---|---|
| T42 | Отключить кабель лидара на 10 с, затем вернуть | узлы не падают; поток возобновляется или нет — зафиксируйте |
| T43 | Запустить `imu_converter_node` до запуска `/lowstate` | узел ждёт, не падает, после появления `/lowstate` публикует |

## 12. Протокол результатов и отчёт

Скопируйте таблицу, заполните столбец «Результат» (`PASS`, `FAIL`, `пропущено: причина`) и «Значение / комментарий».

| № | Проверка | Результат | Значение / комментарий |
|---|---|---|---|
| T1–T4 | сеть, топики `/lowstate`, `/odommodestate`; частота `/lowstate` | | |
| T5–T9 | сборка, тесты, исполняемые файлы | | |
| T10–T13 | узлы и топики лидара | | |
| T14 | частота `custom_msg` | | |
| T15, T15a–c | облако, `check_pointcloud.py`, логи | | |
| T16 | частота `scan` | | |
| T54 | разница часов лидара и компьютера (с) | | |
| T55 | потери данных (паузы, частота IMU) | | |
| Q5 | модуль ускорения IMU лидара | | |
| T17a–d | TF | | какие фреймы отсутствуют |
| T18–T20, floor | пол, стена, вертикаль; наклон до и после | | итоговые `lidar_tf_*` |
| T21–T24 | IMU, `check_imu.py` | | |
| Q6 | нулевые кватернионы при старте | | |
| Q7 | шум IMU (файл `imu_static.txt`) | | |
| T25–T29 | запуск и содержимое одометрии, `check_odom.py` | | |
| T32–T34 | движение | | |
| T35 | система координат скорости | | $v_x, v_y$ до и после поворота |
| T36–T37 | остановка и повторный запуск | | |
| T44–T49 | камера, режим `rgb_depth` | | версия `realsense2_camera` |
| T50–T53 | камера, облако, `camera_name`, CPU | | `clip_distance`, нагрузка CPU |
| T38–T41 | стабильность (30 мин) | | CPU в начале и в конце |

Отправьте Claude:

1. заполненную таблицу;
2. архив папки `$RUN`: `cd ~ && tar czf h1_check_results.tgz $(basename $RUN)` (там `env.txt`, все логи, записи `lidar_check`, `lowstate_boot`, `imu_static`; если они крупные, приложите хотя бы `*.txt` и `*.log`);
3. для каждого **FAIL** — вывод команды целиком и что вы делали непосредственно перед этим.

Пример сообщения Claude:

> Проверки блока C. T14 не прошёл: `ros2 topic hz /sensors/lidar/livox/custom_msg` показывает 12 Гц вместо 50. T11–T13 прошли.
> Окружение в `env.txt`, лог запуска в `lidar_launch.log` (прикрепила). Что проверить дальше?

Что делает Claude с отчётом: сопоставит FAIL с таблицей «симптом → причина» ниже и с [ROBOT_OPEN_QUESTIONS.md](ROBOT_OPEN_QUESTIONS.md),
предложит правку (или скажет, что проблема вне кода) и обновит статусы пакетов в [PACKAGES.md](PACKAGES.md) (🟡 → 🟢 только после прохождения обязательных проверок).

## 13. Если не получилось: куда смотреть

| Симптом | Что проверить |
|---|---|
| T13 не пусто (`/sensors/livox/lidar` есть) | ремапы в `livox_ros_driver2/launch_ROS2/msg_MID360_launch.py` должны быть относительными |
| Нет `/sensors/lidar/livox/point_cloud2` | лог `livox_to_pointcloud2`, ремапы в `lidar.launch.py` |
| Облако есть, `scan` нет, в логе `Message Filter dropping message` | TF `mid360_link → livox_frame` (T17a); затем запуск `lidar.launch.py transform_tolerance:=5.01` (старое значение) и сообщите, помогло ли (вопрос Q2/Q3) |
| `check_pointcloud.py`: «слишком мало пар» | потери данных или QoS; `ros2 topic info -v <топик>` |
| `check_imu.py`: расхождение кватерниона | порядок компонент, версия `unitree_go`; сравните ручной `echo` `/lowstate` и `Imu` |
| `check_odom.py`: угол ≈ 180° | порядок кватерниона в `fill_odometry_message` |
| Топик есть, данных нет | QoS: `ros2 topic info -v <топик>` (Reliability публикатора и подписчика совместимы?) |
| Драйвер лидара сразу завершается | нет адреса `192.168.123.162` (T1), нет `liblivox_lidar_sdk_shared.so`, лидар не отвечает на `ping` (T2) |
| Камера: `No RealSense devices were found` | USB 3 кабель, питание, `rs-enumerate-devices`, права на `/dev` (udev-правила librealsense) |
| `odom_converter`: ошибка при старте | `unitree_sdk2py`, `high_level_control`, `network_interface`; сообщение `Cannot import ...` — чего не хватает |

## 14. Шпаргалка команд ROS 2

| Что | Команда |
|---|---|
| список узлов, топиков | `ros2 node list`, `ros2 topic list` |
| частота топика | `ros2 topic hz <топик>` |
| одно сообщение | `ros2 topic echo --once <топик>` (`--no-arr` скрывает длинные массивы, `--field a.b` печатает поле) |
| информация о топике и QoS | `ros2 topic info -v <топик>` |
| параметр узла | `ros2 param get <узел> <параметр>` |
| список параметров | `ros2 param list <узел>` |
| преобразование между кадрами | `ros2 run tf2_ros tf2_echo <родитель> <ребёнок>` |
| дерево TF | `ros2 run tf2_tools view_frames` |
| запись | `ros2 bag record -o <папка> <топики>` (остановка Ctrl+C) |
| воспроизведение | `ros2 bag play <папка>` |
