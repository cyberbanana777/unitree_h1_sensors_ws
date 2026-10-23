#!/usr/bin/env bash
# Copyright (c) 2025 Alice Zenina and Alexander Grachev RTU MIREA (Russia)
# SPDX-License-Identifier: MIT
#
# Установка зависимостей workspace через rosdep и проверка того, что rosdep поставить не может.
#
#   ./install_dependencies.bash          # поставить зависимости
#   ./install_dependencies.bash --dry-run  # только показать, что поставил бы rosdep
#
# Скрипт можно запускать из любой папки. Workspace считается родителем папки src,
# в которой лежит этот файл. Скрипт сам ничего не собирает и не ставит вручную:
# Livox-SDK2, unitree_ros2, unitree_sdk2py и high_level_control он только проверяет и
# при отсутствии подсказывает, что делать.

set -u

DRY_RUN=0
case "${1:-}" in
    "") ;;
    --dry-run|-n) DRY_RUN=1 ;;
    -h|--help) sed -n 5,14p "$0"; exit 0 ;;
    *) echo "Неизвестный аргумент: $1" >&2; exit 2 ;;
esac

SRC_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WS_DIR="$(dirname "$SRC_DIR")"

if [ -z "${ROS_DISTRO:-}" ]; then
    echo "ROS_DISTRO не задан: выполните source /opt/ros/<distro>/setup.bash" >&2
    exit 1
fi
if ! command -v rosdep >/dev/null 2>&1; then
    echo "rosdep не найден. Установите: sudo apt install python3-rosdep2 (или python3-rosdep)" >&2
    exit 1
fi

echo "== rosdep для $SRC_DIR (ROS_DISTRO=$ROS_DISTRO)"
# unitree_go ставится вместе с unitree_ros2, livox_ros_driver2 лежит в этом же workspace:
# для rosdep это неизвестные ключи, поэтому они пропускаются и проверяются ниже.
ROSDEP_ARGS=(--from-paths "$SRC_DIR" --ignore-src -r -y --rosdistro "$ROS_DISTRO"
             --skip-keys "unitree_go livox_ros_driver2")
if [ "$DRY_RUN" -eq 1 ]; then
    ROSDEP_ARGS+=(--simulate)
fi
rosdep install "${ROSDEP_ARGS[@]}"
ROSDEP_STATUS=$?

echo
echo "== Проверка того, что rosdep не ставит"
MISSING=0

LIVOX_LIB=$(ls /usr/local/lib/liblivox_lidar_sdk_shared.so* /usr/lib/liblivox_lidar_sdk_shared.so* \
    /usr/lib/*/liblivox_lidar_sdk_shared.so* 2>/dev/null | head -n 1)
if ldconfig -p 2>/dev/null | grep -q liblivox_lidar_sdk_shared; then
    echo "[ok]   Livox-SDK2 (liblivox_lidar_sdk_shared.so)"
elif [ -n "$LIVOX_LIB" ]; then
    echo "[ok]   Livox-SDK2: $LIVOX_LIB (ldconfig кэш не обновлён: выполните 'sudo ldconfig')"
else
    echo "[НЕТ]  Livox-SDK2: соберите и установите https://github.com/Livox-SDK/Livox-SDK2 (нужен sudo make install)"
    MISSING=1
fi

if python3 -c "import unitree_sdk2py" >/dev/null 2>&1; then
    echo "[ok]   unitree_sdk2py (нужен odom_converter)"
else
    echo "[НЕТ]  unitree_sdk2py: установите по инструкции https://github.com/unitreerobotics/unitree_sdk2_python"
    MISSING=1
fi

if python3 -c "import high_level_control" >/dev/null 2>&1; then
    echo "[ok]   high_level_control (OdomClient, нужен odom_converter)"
else
    echo "[НЕТ]  high_level_control: соберите workspace unitree_h1_control_ws и выполните его source"
    MISSING=1
fi

if ros2 interface show unitree_go/msg/LowState >/dev/null 2>&1; then
    echo "[ok]   unitree_go (сообщения Unitree)"
else
    echo "[НЕТ]  unitree_go: соберите unitree_ros2 и выполните его source (нужны imu_converter и odom_converter)"
    MISSING=1
fi

echo
if [ "$ROSDEP_STATUS" -ne 0 ]; then
    echo "rosdep завершился с ошибкой (код $ROSDEP_STATUS), см. сообщения выше."
fi
if [ "$MISSING" -ne 0 ]; then
    echo "Не всё из списка выше установлено: пакеты, которые от этого зависят, не соберутся или не запустятся."
fi
[ "$ROSDEP_STATUS" -eq 0 ] && [ "$MISSING" -eq 0 ]
