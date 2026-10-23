#!/usr/bin/env bash
# Copyright (c) 2025 Alice Zenina and Alexander Grachev RTU MIREA (Russia)
# SPDX-License-Identifier: MIT
#
# Собирает сведения об окружении в один текстовый файл, чтобы приложить его к отчёту о проверке.
#
#   ./src/tools/collect_env.sh                 # файл h1_env_<дата>.txt в текущей папке
#   ./src/tools/collect_env.sh /путь/к/файлу   # свой путь
#
# Скрипт только читает: ничего не устанавливает и не запускает на роботе.

OUT="${1:-h1_env_$(date +%F_%H%M).txt}"
SRC_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"

section() { printf '\n===== %s =====\n' "$1"; }

{
    echo "Дата: $(date -Is)"
    echo "Хост: $(hostname)"

    section "Система"
    uname -a
    grep -E '^(PRETTY_NAME|VERSION)=' /etc/os-release 2>/dev/null
    python3 --version 2>&1

    section "ROS"
    echo "ROS_DISTRO=${ROS_DISTRO:-<не задан>}"
    echo "RMW_IMPLEMENTATION=${RMW_IMPLEMENTATION:-<не задан>}"
    echo "ROS_DOMAIN_ID=${ROS_DOMAIN_ID:-<не задан>}"
    echo "CYCLONEDDS_URI=${CYCLONEDDS_URI:-<не задан>}"
    echo "AMENT_PREFIX_PATH=${AMENT_PREFIX_PATH:-<не задан>}" | tr ':' '\n'

    section "Версии пакетов workspace (из package.xml)"
    for pkg_xml in "$SRC_DIR"/*/package.xml; do
        name=$(sed -n 's:.*<name>\(.*\)</name>.*:\1:p' "$pkg_xml" | head -1)
        version=$(sed -n 's:.*<version>\(.*\)</version>.*:\1:p' "$pkg_xml" | head -1)
        printf '%-28s %s\n' "$name" "$version"
    done

    section "Установленные пакеты ROS по теме"
    if command -v dpkg >/dev/null 2>&1; then
        dpkg -l 2>/dev/null | grep -E 'ros-.*(realsense|librealsense|tf2-sensor|laser-geometry)|librealsense' | awk '{print $2, $3}'
    fi
    ros2 pkg list 2>/dev/null | grep -E '^(unitree_go|unitree_hg|livox_ros_driver2|realsense2_camera|high_level_control)$' | sort

    section "Python"
    for mod in unitree_sdk2py high_level_control numpy; do
        if python3 -c "import $mod" >/dev/null 2>&1; then echo "[ok]  $mod"; else echo "[нет] $mod"; fi
    done

    section "Livox SDK2"
    ldconfig -p 2>/dev/null | grep liblivox || echo "[нет] liblivox_lidar_sdk_shared"

    section "Сеть (IPv4)"
    ip -4 -brief addr 2>/dev/null

    section "Git (если workspace под git)"
    if git -C "$SRC_DIR" rev-parse --is-inside-work-tree >/dev/null 2>&1; then
        git -C "$SRC_DIR" log -1 --format='%h %ad %s' --date=short
        git -C "$SRC_DIR" status --short | head -20
    else
        echo "не под git"
    fi

    section "Камера RealSense (если подключена)"
    lsusb 2>/dev/null | grep -i intel || echo "Intel-устройств USB не найдено"
    command -v rs-enumerate-devices >/dev/null 2>&1 && rs-enumerate-devices -s 2>&1 | head -10
} > "$OUT" 2>&1

echo "Сохранено: $OUT"
