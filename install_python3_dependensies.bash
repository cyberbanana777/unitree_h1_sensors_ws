#!/bin/bash

# Функция для вывода сообщений с префиксом
log() {
    echo "[Installer] $1"
}

# Переменные для сбора статистики
found=0
not_found=0
found_dirs=()
not_found_dirs=()

# Перебираем все поддиректории в текущей директории
for dir in */; do
    # Убираем слеш из имени директории
    dir=${dir%/}
    
    # Проверяем наличие файла requirements.txt в директории
    if [ -f "$dir/requirements.txt" ]; then
        ((found++))
        found_dirs+=("$dir")
        log "Найден requirements.txt в $dir. Начинаю установку зависимостей..."
        
        # Создаем временный файл для логов
        temp_log=$(mktemp)
        
        # Устанавливаем зависимости с выводом в временный файл
        pip install -r "$dir/requirements.txt" > "$temp_log" 2>&1
        status=$?
        
        # Анализируем результат установки
        if [ $status -eq 0 ]; then
            log "УСПЕХ: Все зависимости из $dir установлены успешно"
            log "Установленные пакеты:"
            grep '^Successfully installed' "$temp_log" | sed 's/Successfully installed/[Installer]   - /'
        else
            log "ОШИБКА: Проблемы при установке зависимостей из $dir"
            log "Подробности ошибок:"
            
            # Выводим ошибки с отступом
            grep -E 'ERROR:|Could not install' "$temp_log" | sed 's/^/[Installer]   - /'
            
            # Выводим успешно установленные пакеты, если такие были
            if grep -q '^Successfully installed' "$temp_log"; then
                log "Часть пакетов установлена успешно:"
                grep '^Successfully installed' "$temp_log" | sed 's/Successfully installed/[Installer]   - /'
            fi
        fi
        
        # Удаляем временный файл
        rm -f "$temp_log"
    else
        ((not_found++))
        not_found_dirs+=("$dir")
        log "Пропускаем $dir - requirements.txt не найден"
    fi
done

# Выводим итоговую статистику
log ""
log "===== ИТОГОВАЯ СТАТИСТИКА ====="
log "Найдено requirements.txt в ${found} папках:"
for d in "${found_dirs[@]}"; do
    log "  - $d"
done

log ""
log "Не найдено requirements.txt в ${not_found} папках:"
for d in "${not_found_dirs[@]}"; do
    log "  - $d"
done

log ""
log "Процесс установки завершен"
