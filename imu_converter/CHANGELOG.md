# Changelog

Формат основан на [Keep a Changelog](https://keepachangelog.com/ru/1.1.0/), версии — [SemVer](https://semver.org/lang/ru/).

## [1.1.0] - 2026-10-23

> Рефакторинг. **Проверено только юнит-тестами на заглушках сообщений, на роботе не запускалось**
> (см. `docs/HARDWARE_TESTING.md`).

### Изменено
- Ковариации по умолчанию нулевые вместо `-1.0`. По описанию `sensor_msgs/Imu` значение `-1` в элементе `[0]`
  означает «этих данных нет, игнорируйте», тогда как ориентация, гироскоп и акселерометр публикуются.
  Нули означают «данные есть, ковариация неизвестна». **Потребители, которые раньше игнорировали
  ориентацию из-за `-1`, теперь начнут её использовать.**
- `main()`: без `except Exception`, корректное завершение при уже остановленном `rclpy`.

### Добавлено
- Параметры `orientation_variance`, `angular_velocity_variance`, `linear_acceleration_variance`
  (значение на диагонали ковариации, по умолчанию `0.0`).
- Пропуск сообщений с нулевым кватернионом (предупреждение не чаще раза в 5 с). Это защитная мера,
  встречаются ли такие сообщения на реальном роботе — не проверено.
- Функции `fill_imu_message()` и `quaternion_is_valid()`, тесты `test/test_conversion.py`.
- `README.md` с таблицей параметров.

### Удалено
- `import numpy` и `pip_requirements.txt` (содержал только неиспользуемый `numpy`).
- Неиспользуемые импорты `IMUState`, `Quaternion`, `Vector3`; зависимость `geometry_msgs` в `package.xml`.

### Исправлено
- `setup.cfg`: `script-dir` / `install-scripts` заменены на `script_dir` / `install_scripts` (предупреждение setuptools
  `Usage of dash-separated ... will not be supported in future versions`).
- `package.xml`/`setup.py`: описание и опечатка `sensors_msgs/msg/IMU` → `sensor_msgs/msg/Imu`.

## [1.0.0]
- Исходная версия.
