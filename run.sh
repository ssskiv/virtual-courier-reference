#!/bin/bash
# Запуск эталонного решения в среде «Виртуальный курьер» с окнами (Webots, RViz, камера).
#
#   ./run.sh [сценарий] [опции]
#
#   сценарий     имя без .yaml: final_1..3 (scenarios/ здесь) или public_1/2 (из публичного репо).
#                По умолчанию final_1.
#   --bag        записать все топики в bags/<сценарий>_<время>/ (ros2 bag, сжатие zstd)
#   --headless   без окон (быстрее; только лог в терминале)
#   --fast       Webots в режиме fast (быстрее реального времени, если хватает CPU)
#   --list       показать доступные сценарии
#
# Переменные окружения:
#   VC_REPO      путь к публичному репозиторию среды (по умолчанию ../virtual-courier)
#
# Ctrl+C — остановить заезд и контейнеры.
set -euo pipefail

REF=$(cd "$(dirname "$0")" && pwd)
VC_REPO=${VC_REPO:-$(cd "$REF/../virtual-courier" 2>/dev/null && pwd || true)}
SCENARIO=final_1
BAG=false
GUI=true
MODE=realtime

for arg in "$@"; do
    case "$arg" in
        --bag) BAG=true ;;
        --headless) GUI=false ;;
        --fast) MODE=fast ;;
        --list)
            echo "закрытые:  $(ls "$REF/scenarios" | sed 's/\.yaml//' | tr '\n' ' ')"
            [ -n "$VC_REPO" ] && echo "публичные: $(ls "$VC_REPO/scene/scenarios" | sed 's/\.yaml//' | tr '\n' ' ')"
            exit 0 ;;
        -h|--help) sed -n '2,16p' "$0"; exit 0 ;;
        -*) echo "неизвестная опция $arg" >&2; exit 1 ;;
        *) SCENARIO=$arg ;;
    esac
done

if [ -z "$VC_REPO" ] || [ ! -f "$VC_REPO/docker-compose.yaml" ]; then
    echo "!!! не найден публичный репозиторий среды. Укажите VC_REPO=/путь/к/virtual-courier" >&2
    exit 1
fi
if [ -f "$REF/scenarios/$SCENARIO.yaml" ]; then
    SCENARIO_DIR="$REF/scenarios"
elif [ -f "$VC_REPO/scene/scenarios/$SCENARIO.yaml" ]; then
    SCENARIO_DIR="$VC_REPO/scene/scenarios"
else
    echo "!!! сценарий $SCENARIO не найден (./run.sh --list)" >&2
    exit 1
fi

# Окна из контейнеров (root) на ваш X-сервер.
if [ "$GUI" = true ]; then
    if ! xhost 2>/dev/null | grep -q "localuser:root"; then
        echo ">>> разрешаю окна контейнеров: xhost +SI:localuser:root"
        xhost +SI:localuser:root >/dev/null
    fi
fi

mkdir -p "$REF/bags"
OVERRIDE=$(mktemp --suffix=.yaml)
cat > "$OVERRIDE" <<EOF
services:
  scene:
    environment:
      WEBOTS_GUI: "$GUI"
      WEBOTS_MODE: "$MODE"
      VC_TEAM: reference
  solution:
    environment:
      GUI: "$GUI"
    volumes:
      - $REF/src/solution:/ros_ws/src/solution
      - $REF/bags:/bags
EOF

export SCENARIO SCENARIO_DIR COMPOSE_PROJECT_NAME=vc_reference
COMPOSE=(docker compose --project-directory "$VC_REPO" -f "$VC_REPO/docker-compose.yaml" -f "$OVERRIDE")
if command -v nvidia-smi >/dev/null 2>&1 && docker info 2>/dev/null | grep -qi nvidia; then
    COMPOSE+=(-f "$VC_REPO/docker-compose.nvidia.yaml")
fi

cleanup() {
    trap - INT TERM EXIT
    echo
    echo ">>> остановка…"
    docker exec vc_solution pkill -INT -f "ros2 bag record" >/dev/null 2>&1 && sleep 2 || true
    # Файлы создаёт root контейнера; открываем их, чтобы можно было удалить без sudo.
    docker exec vc_solution chmod -R a+rwX /bags >/dev/null 2>&1 || true
    "${COMPOSE[@]}" down --remove-orphans >/dev/null 2>&1 || true
    rm -f "$OVERRIDE"
    if [ "$BAG" = true ]; then echo ">>> запись: $(ls -td "$REF"/bags/*/ 2>/dev/null | head -1)"; fi
    echo ">>> результаты заездов: $VC_REPO/results/"
}
trap cleanup INT TERM EXIT

echo ">>> сценарий $SCENARIO ($SCENARIO_DIR), окна: $GUI, режим Webots: $MODE"
"${COMPOSE[@]}" down --remove-orphans >/dev/null 2>&1 || true
docker rm -f vc_scene vc_solution >/dev/null 2>&1 || true
"${COMPOSE[@]}" up -d

if [ "$BAG" = true ]; then
    NAME="${SCENARIO}_$(date +%Y%m%d-%H%M%S)"
    echo ">>> ros2 bag → bags/$NAME"
    docker exec -d vc_solution bash -c \
        "source /opt/ros/humble/setup.bash && ros2 bag record -a -o /bags/$NAME \\
         --compression-mode file --compression-format zstd --max-bag-duration 60 > /bags/$NAME.log 2>&1"
fi

echo ">>> ход заезда (Ctrl+C — остановить):"
"${COMPOSE[@]}" logs -f --no-log-prefix scene solution 2>/dev/null \
    | grep --line-buffered -E "\[referee\]|\[reference\]: \[|стена выгрузки|найдена|уточнена|неактивна|пора домой|Traceback|ОШИБКА" \
    | sed -u 's/\[INFO\] \[[0-9.]*\] //; s/^\[reference_node.py-1\] //'
