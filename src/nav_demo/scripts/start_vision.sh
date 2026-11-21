#!/usr/bin/env bash
# 一键启动全部视觉服务：一个终端、输出带 [模块] 前缀、Ctrl-C 全部退出。
#
# 节点列表来自 scripts/vision_nodes.conf，每行：
#   <脚本名> [解释器]
# 脚本名按顺序在 ~/AI视觉、bit_ws/src/photo_capture/scripts、
# bit_ws/src/nav_demo/scripts 下找；也可以直接写绝对/相对路径。
# 解释器可选 venv（默认，~$AI_VISION_VENV）或 system（/usr/bin/python3）或绝对路径。
#   inspection_vision system     # 统一视觉节点：需要系统 python3（lap/hyperlpr3）
#
# 也可用环境变量 VISION_NODES="node1 node2" 临时覆盖。
#
# 用法：
#   roslaunch nav777 等先启动好（roscore / 相机 / 导航）
#   ~/bit_ws/src/nav_demo/scripts/start_vision.sh
#
# 可用环境变量覆盖：
#   AI_VISION_DIR=~/AI视觉  AI_VISION_VENV=~/baidu-aip-env  BIT_WS=~/bit_ws
#   SYSTEM_PYTHON=/usr/bin/python3  VISION_NODES_CONF=...  VISION_NODES="..."
#   VISION_PY=venv|system|/abs/python  （VISION_NODES 临时覆盖时的默认解释器）
set -u

AI_DIR="${AI_VISION_DIR:-$HOME/AI视觉}"
VENV="${AI_VISION_VENV:-$HOME/baidu-aip-env}"
SYSTEM_PY="${SYSTEM_PYTHON:-/usr/bin/python3}"
WS="${BIT_WS:-$HOME/bit_ws}"
CONF="${VISION_NODES_CONF:-$WS/src/nav_demo/scripts/vision_nodes.conf}"
DEFAULT_PY="${VISION_PY:-venv}"

# ROS 的 setup.bash 会读一些可能未定义的变量，source 期间临时关掉 set -u
set +u
# shellcheck disable=SC1091
[ -f /opt/ros/noetic/setup.bash ] && source /opt/ros/noetic/setup.bash
# shellcheck disable=SC1091
[ -f "$WS/devel/setup.bash" ] && source "$WS/devel/setup.bash"
set -u

if [ ! -d "$AI_DIR" ]; then
    echo "[start_vision] 提示：找不到视觉目录 $AI_DIR（只用仓库内节点时无影响）" >&2
fi
if [ ! -x "$VENV/bin/python3" ]; then
    echo "[start_vision] 提示：找不到虚拟环境 $VENV（只有 venv 节点会受影响）" >&2
fi

NODES=()
NODE_PY=()
if [ -n "${VISION_NODES:-}" ]; then
    read -r -a NODES <<< "$VISION_NODES"
    for _ in "${NODES[@]}"; do NODE_PY+=("$DEFAULT_PY"); done
elif [ -f "$CONF" ]; then
    while IFS= read -r line; do
        line="${line%%#*}"
        line="$(echo "$line" | xargs)"
        [ -z "$line" ] && continue
        read -r name py <<< "$line"
        NODES+=("$name")
        NODE_PY+=("${py:-$DEFAULT_PY}")
    done < "$CONF"
fi

if [ "${#NODES[@]}" -eq 0 ]; then
    echo "[start_vision] 还没有配置视觉节点：请编辑 $CONF（每行一个节点名）" >&2
    exit 1
fi

cd "$AI_DIR" 2>/dev/null || cd "$WS" || { echo "[start_vision] 无法进入工作目录" >&2; exit 1; }
# shellcheck disable=SC1091
[ -x "$VENV/bin/python3" ] && source "$VENV/bin/activate"

resolve_script() {
    local name="$1"
    case "$name" in
        */*) [ -f "$name" ] && { echo "$name"; return 0; } ;;
    esac
    for base in "$AI_DIR" "$WS/src/photo_capture/scripts" "$WS/src/nav_demo/scripts"; do
        if [ -f "$base/$name.py" ]; then
            echo "$base/$name.py"
            return 0
        fi
    done
    return 1
}

resolve_python() {
    case "${1:-venv}" in
        venv) echo "$VENV/bin/python3" ;;
        system) echo "$SYSTEM_PY" ;;
        *) echo "$1" ;;
    esac
}

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

for index in "${!NODES[@]}"; do
    name="${NODES[$index]}"
    spec="${NODE_PY[$index]:-venv}"
    py="$(resolve_python "$spec")"
    script="$(resolve_script "$name")"
    if [ -z "$script" ]; then
        echo "[start_vision] 找不到 $name.py（在 $AI_DIR / $WS/src 下），跳过" >&2
        continue
    fi
    if [ ! -x "$py" ]; then
        echo "[start_vision] 解释器不存在：$py（节点 $name），跳过" >&2
        continue
    fi
    echo "[start_vision] 启动 $name：$py $script"
    "$py" "$script" > >(sed -u "s/^/[$name] /") 2>&1 &
    pids+=($!)
done

if [ "${#pids[@]}" -eq 0 ]; then
    echo "[start_vision] 没有启动任何视觉服务" >&2
    exit 1
fi

echo "[start_vision] 已启动 ${#pids[@]} 个视觉服务（$(IFS=,; echo "${NODES[*]}")），Ctrl-C 全部退出"
wait
