#!/usr/bin/env bash
# 一键启动全部视觉服务：一个终端、输出带 [模块] 前缀、Ctrl-C 全部退出。
#
# 节点列表来自 scripts/vision_nodes.conf（每行一个，不含 .py），
# 或环境变量 VISION_NODES="node1 node2"。
#
# 用法：
#   roslaunch nav777 等先启动好（roscore / 相机 / 导航）
#   ~/bit_ws/src/nav_demo/scripts/start_vision.sh
#
# 可用环境变量覆盖：
#   AI_VISION_DIR=~/AI视觉  AI_VISION_VENV=~/baidu-aip-env  BIT_WS=~/bit_ws
#   VISION_NODES_CONF=...  VISION_NODES="..."
set -u

AI_DIR="${AI_VISION_DIR:-$HOME/AI视觉}"
VENV="${AI_VISION_VENV:-$HOME/baidu-aip-env}"
WS="${BIT_WS:-$HOME/bit_ws}"
CONF="${VISION_NODES_CONF:-$WS/src/nav_demo/scripts/vision_nodes.conf}"

# shellcheck disable=SC1091
[ -f /opt/ros/noetic/setup.bash ] && source /opt/ros/noetic/setup.bash
# shellcheck disable=SC1091
[ -f "$WS/devel/setup.bash" ] && source "$WS/devel/setup.bash"

if [ ! -d "$AI_DIR" ]; then
    echo "[start_vision] 找不到视觉目录：$AI_DIR" >&2
    exit 1
fi
if [ ! -x "$VENV/bin/python3" ]; then
    echo "[start_vision] 找不到虚拟环境：$VENV" >&2
    exit 1
fi

NODES=()
if [ -n "${VISION_NODES:-}" ]; then
    read -r -a NODES <<< "$VISION_NODES"
elif [ -f "$CONF" ]; then
    while IFS= read -r line; do
        line="${line%%#*}"
        line="$(echo "$line" | xargs)"
        [ -n "$line" ] && NODES+=("$line")
    done < "$CONF"
fi

if [ "${#NODES[@]}" -eq 0 ]; then
    echo "[start_vision] 还没有配置视觉节点：请编辑 $CONF（每行一个节点名）" >&2
    exit 1
fi

cd "$AI_DIR" || exit 1
# shellcheck disable=SC1091
source "$VENV/bin/activate"

PY="$VENV/bin/python3"
pids=()

cleanup() {
    trap - INT TERM
    echo
    echo "[start_vision] 停止全部视觉服务 ..."
    for pid in "${pids[@]:-}"; do
        kill "$pid" 2>/dev/null || true
    done
    wait 2>/dev/null || true
    exit 0
}
trap cleanup INT TERM

for name in "${NODES[@]}"; do
    if [ ! -f "$name.py" ]; then
        echo "[start_vision] 缺少 $name.py，跳过" >&2
        continue
    fi
    "$PY" "$name.py" > >(sed -u "s/^/[$name] /") 2>&1 &
    pids+=($!)
done

if [ "${#pids[@]}" -eq 0 ]; then
    echo "[start_vision] 没有启动任何视觉服务" >&2
    exit 1
fi

echo "[start_vision] 已启动 ${#pids[@]} 个视觉服务（$(IFS=,; echo "${NODES[*]}")），Ctrl-C 全部退出"
wait
