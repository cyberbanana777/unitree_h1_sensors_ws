# Changelog

Формат основан на [Keep a Changelog](https://keepachangelog.com/ru/1.1.0/), версии — [SemVer](https://semver.org/lang/ru/).

## [2.0.0] - 2026-10-23

> Рефакторинг. **Проверено только юнит-тестами на заглушках сообщений, на роботе не запускалось**
> (см. `docs/HARDWARE_TESTING.md`).

### Изменено (ломает совместимость)
- Исполняемый файл `odom_convereter_node` (с опечаткой) переименован в `odom_converter_node`,
  файл модуля — `odom_convereter_node.py` → `odom_converter_node.py`. Старое имя удалено,
  без алиаса: обновите свои launch-файлы.
- Фреймы, топики и ковариации больше не зашиты в код, а задаются параметрами
  (значения по умолчанию совпадают со старым поведением: `odom` → `pelvis`, топики `odommodestate`, `lowstate`, `odom`).
- Ошибки в `main()` больше не проглатываются в лог: исключение доходит до traceback.

### Исправлено
- `setup.cfg`: `script-dir` / `install-scripts` заменены на `script_dir` / `install_scripts` (предупреждение setuptools
  `Usage of dash-separated ... will not be supported in future versions`).
- Порядок кватерниона: Unitree присылает $(w, x, y, z)$, ROS ждёт $(x, y, z, w)$. Раньше `quaternion[0]`
  (то есть $w$) писалось в `x`, а `quaternion[3]` — в `w`; для неподвижного робота `/odom`
  показывал поворот на $180^\circ$.
- `DisableOdom()` при остановке обёрнут в `try/except`, повторный вызов безопасен.
- Предупреждение «Waiting for LowState data...» печаталось на каждое сообщение, теперь не чаще раза в 5 с.

### Добавлено
- Параметры: `sport_topic`, `lowstate_topic`, `odom_topic`, `odom_frame`, `child_frame`,
  `publish_tf` (по умолчанию `false`), `pose_variance`, `twist_variance`, `network_interface`.
- Опциональная публикация TF `odom_frame → child_frame`.
- Понятное сообщение об ошибке, если не найдены `unitree_sdk2py` или `high_level_control`.
- Функция `fill_odometry_message()` и тесты `test/test_conversion.py`.
- `README.md` с параметрами и списком того, что проверить на роботе.

### Удалено
- Мёртвые ветки `hasattr(imu_state, 'quaternion' | 'gyroscope')` (поля есть всегда).
- Неиспользуемая зависимость `std_msgs` в `package.xml`.

### Известные вопросы
- Система координат `SportModeState.velocity` (мировая или связанная) не проверена;
  для `nav_msgs/Odometry` скорость должна быть в `child_frame`.
- Ковариации по умолчанию нулевые («неизвестны»), реальные значения не измерены.

## [1.0.0]
- Исходная версия.
