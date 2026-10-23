# odom_converter

Узел `odom_converter_node` собирает `nav_msgs/Odometry` из двух источников Unitree:

* `unitree_go/SportModeState` (топик `odommodestate`) — положение и линейная скорость;
* `unitree_go/LowState` (топик `lowstate`) — ориентация и угловая скорость из IMU.

При старте узел включает одометрию робота (`OdomClient.EnableOdom()`), при остановке выключает.

```bash
ros2 run odom_converter odom_converter_node
```

> Исполняемый файл называется `odom_converter_node`. Старое имя с опечаткой `odom_convereter_node`
> удалено в версии 2.0.0 (см. [CHANGELOG.md](CHANGELOG.md)) — поправьте свои launch-файлы.

## Зависимости

* `unitree_go` (из `unitree_ros2`);
* `unitree_sdk2py`;
* `high_level_control` с `OdomClient` — из workspace `unitree_h1_control_ws`.

Если SDK не найден, узел завершится с понятным сообщением об ошибке.

## Параметры

| Параметр | По умолчанию | Описание |
|---|---|---|
| `sport_topic` | `odommodestate` | Топик `SportModeState` |
| `lowstate_topic` | `lowstate` | Топик `LowState` |
| `odom_topic` | `odom` | Выходной топик |
| `odom_frame` | `odom` | `header.frame_id` |
| `child_frame` | `pelvis` | `child_frame_id` |
| `publish_tf` | `false` | Публиковать TF `odom_frame -> child_frame` |
| `pose_variance` | `0.0` | Диагональ ковариации позы (0 = «неизвестна») |
| `twist_variance` | `0.0` | Диагональ ковариации скорости |
| `network_interface` | `` | Сетевой интерфейс для DDS Unitree SDK (например `eth0`) |

`publish_tf` выключен по умолчанию, чтобы не конфликтовать с другими источниками TF (например,
`robot_state_publisher`). Включайте, только если `odom -> pelvis` больше никто не публикует.

## Что нужно проверить на роботе

* В какой системе координат приходит `SportModeState.velocity` (мировой или связанной с телом).
  Для `nav_msgs/Odometry` twist должен быть в `child_frame`. Узел копирует значения как есть.
* Что `w` в `/odom` близко к 1 при неподвижном роботе и совпадает с `/sensors/imu/unitree_h1`.
