# imu_converter

Узел `imu_converter_node` берёт IMU из `unitree_go/LowState` (`/lowstate`) и публикует
стандартный `sensor_msgs/Imu`.

```bash
ros2 run imu_converter imu_converter_node
```

| Параметр | По умолчанию | Описание |
|---|---|---|
| `input_topic` | `/lowstate` | Откуда читать `unitree_go/LowState` |
| `output_topic` | `/sensors/imu/unitree_h1` | Куда публиковать `sensor_msgs/Imu` |
| `frame_id` | `imu_link` | `header.frame_id` |
| `orientation_variance` | `0.0` | Значение на диагонали ковариации ориентации |
| `angular_velocity_variance` | `0.0` | То же для угловой скорости |
| `linear_acceleration_variance` | `0.0` | То же для ускорения |

Что важно знать:

* Кватернион Unitree приходит в порядке $(w, x, y, z)$, в ROS нужен $(x, y, z, w)$ — узел переупаковывает.
* Нулевые ковариации по REP-145 значат «ковариация неизвестна», данные при этом считаются валидными.
  Значение `-1` в `[0]` означало бы «этих данных нет», поэтому оно не используется.
* Штамп времени — время приёма на хосте (`now()`), а не время измерения в IMU.
* Пока робот присылает нулевой кватернион, сообщения пропускаются (предупреждение раз в 5 с).

Тесты логики конвертации: `colcon test --packages-select imu_converter`.
