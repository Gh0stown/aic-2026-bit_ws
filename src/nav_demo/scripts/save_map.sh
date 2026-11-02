#!/bin/bash
# 保存当前地图：默认按时间戳命名，不会覆盖已有地图
# 用法： ./save_map.sh             -> map/mymap_20261102_1713.pgm / .yaml
#        ./save_map.sh mymap_new   -> map/mymap_new.pgm / .yaml
set -e
MAP_DIR="$(rospack find nav_demo)/map"
NAME="${1:-mymap_$(date +%Y%m%d_%H%M)}"
if [ -e "$MAP_DIR/$NAME.pgm" ]; then
  echo "已存在: $MAP_DIR/$NAME.pgm，换一个名字" >&2
  exit 1
fi
rosrun map_server map_saver -f "$MAP_DIR/$NAME"
# map_saver 会把绝对路径写进 yaml，改成相对路径，换机器/换目录也能加载
sed -i "s#^image: .*/#image: #" "$MAP_DIR/$NAME.yaml"
echo "完成: $MAP_DIR/$NAME.pgm"
