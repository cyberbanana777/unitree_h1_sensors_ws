# livox_to_pointcloud2

Узел `livox_to_pointcloud2_node` конвертирует `livox_ros_driver2/CustomMsg` в
`sensor_msgs/PointCloud2`. Основан на пакете [porizou/livox_to_pointcloud2](https://github.com/porizou/livox_to_pointcloud2)
(Apache-2.0) и доработан под этот репозиторий.

| | Топик (относительный) | Тип |
|---|---|---|
| подписка | `livox/lidar` | `livox_ros_driver2/msg/CustomMsg` |
| публикация | `livox/point_cloud2` | `sensor_msgs/msg/PointCloud2` |

Имена топиков относительные: итоговое имя зависит от namespace узла, а подмена делается ремапом
(см. `completed_scripts_sensors/launch/lidar.launch.py`).

Поля точки (`point_step = 18`): `x, y, z` (float32), `intensity` (float32, берётся из `reflectivity`),
`tag`, `line` (uint8).

```bash
ros2 run livox_to_pointcloud2 livox_to_pointcloud2_node
# или
ros2 launch livox_to_pointcloud2 livox_to_pointcloud2.launch.py
```

## Что изменено относительно оригинала

* число точек берётся как `min(point_num, points.size())` — раньше расхождение давало запись за границей буфера;
* значения пишутся через `memcpy` (невыровненный доступ при шаге 18 байт);
* убраны неиспользуемые `msg/CustomMsg.msg`, `CustomPoint.msg` и launch-файл с несуществующими ремапами;
* `package.xml`: корректный `buildtool_depend`, лицензия `Apache-2.0`.
