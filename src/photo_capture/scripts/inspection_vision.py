#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""inspection_vision：导航到点后调用的统一视觉识别服务。

提供 ROS1 服务 `/inspection_vision`（nav_demo/InspectionVision.srv）：
一次调用做一件识别，返回播报文本、结构化 extra（JSON 字符串）和带框图片路径。
节点只读相机，不发布 `/cmd_vel`，不控制底盘。

用法（车上，先启动 roscore / 相机 / 导航）：

    source /opt/ros/noetic/setup.bash
    source ~/bit_ws/devel/setup.bash
    /usr/bin/python3 ~/bit_ws/src/photo_capture/scripts/inspection_vision.py

一键启动：编辑 nav_demo/scripts/vision_nodes.conf 后运行 start_vision.sh。

离线自测（不需要 ROS master，可以用图片/视频代替相机）：

    /usr/bin/python3 inspection_vision.py --list-tasks
    /usr/bin/python3 inspection_vision.py --self-test light \
        --device ~/AI视觉/data/camera_photos/xxx.jpg --point 1 --sub-id 红绿灯1

注意：相机只能有一个采集进程；节点运行时不要再开 photo_capture GUI。
"""

from __future__ import annotations

import argparse
import json
import os
import sys
import time
import traceback
from pathlib import Path

MODULE_DIR = Path(__file__).resolve().parent
if str(MODULE_DIR) not in sys.path:
    sys.path.insert(0, str(MODULE_DIR))

from iv_common import (DEFAULT_GAUGE_MODEL, CameraUnavailable, FrameSource,
                       ModelCache, VisionContext, VisionError, load_config)
from iv_service import TASK_HANDLERS, VisionService

SERVICE_NAME = "/inspection_vision"
NODE_NAME = "inspection_vision"


# ---------------------------------------------------------------------------
# 参数 / 配置
# ---------------------------------------------------------------------------

def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        description="统一视觉识别服务：/inspection_vision（不发布 /cmd_vel）")
    parser.add_argument("--config", default=None,
                        help="配置文件；默认 src/photo_capture/config/inspection_vision.yaml")
    # 帧源 / 相机
    parser.add_argument("--device", default=None,
                        help="0 / /dev/video0 / 图片目录 / 视频文件（后两者用于离线自测）")
    parser.add_argument("--width", type=int, default=None, help="采集宽度")
    parser.add_argument("--height", type=int, default=None, help="采集高度")
    parser.add_argument("--fps", type=float, default=None, help="采集帧率")
    parser.add_argument("--pixel-format", choices=("auto", "mjpg", "yuyv"),
                        default=None, help="UVC 取流格式")
    # 推理 / 输出
    parser.add_argument("--infer-device", choices=("cpu", "cuda", "auto"),
                        default="cpu", help="YOLO 推理设备（默认 cpu）")
    parser.add_argument("--model-dir", default=None, help="模型目录（默认 ~/AI视觉）")
    parser.add_argument("--output-dir", default=None,
                        help="识别图片/日志根目录（默认 ~/Pictures/inspection_2026）")
    parser.add_argument("--thermal-port", default=None,
                        help="MLX90640 串口（默认用配置 thermal.port）")
    parser.add_argument("--thermal-capture", default=None,
                        help="离线回放 capture.bin（设置后忽略 thermal.port）")
    parser.add_argument("--service-name", default=SERVICE_NAME,
                        help="ROS 服务名（默认 %s）" % SERVICE_NAME)
    parser.add_argument("--warmup", dest="warmup", action="store_true", default=True,
                        help="启动时预加载常用模型并预热相机（默认开启）")
    parser.add_argument("--no-warmup", dest="warmup", action="store_false",
                        help="启动时不预加载模型")
    # 离线自测
    parser.add_argument("--self-test", default=None,
                        help="不启动 ROS，直接跑一次识别；值为 task 或 all")
    parser.add_argument("--list-tasks", action="store_true", help="列出支持的任务后退出")
    parser.add_argument("--point", type=int, default=0, help="自测用的点位号")
    parser.add_argument("--sub-id", default="", help="自测用的 sub_id，例如 楼宇A / 1号停车位")
    parser.add_argument("--params", default="{}", help="自测用 params JSON")
    parser.add_argument("--json", dest="json_output", action="store_true",
                        help="自测结果只输出一行 JSON（方便脚本解析）")
    return parser


def apply_overrides(config: dict, args: argparse.Namespace) -> dict:
    if args.device is not None:
        config["camera"]["device"] = args.device
    for key in ("width", "height", "fps", "pixel_format"):
        value = getattr(args, key)
        if value is not None:
            config["camera"][key] = value
    if args.model_dir:
        config["models"]["dir"] = args.model_dir
    if args.output_dir:
        config["output"]["dir"] = str(Path(args.output_dir).expanduser().resolve())
    if args.thermal_port is not None:
        config["thermal"]["port"] = args.thermal_port
    if args.thermal_capture is not None:
        config["thermal"]["capture"] = args.thermal_capture
        if args.thermal_capture:
            config["thermal"]["port"] = ""
    return config


def create_context(config: dict, infer_device: str, logger=None) -> VisionContext:
    log = logger or (lambda text: print(text, flush=True))
    camera = config["camera"]
    frames = FrameSource(
        camera.get("device", 0),
        width=camera.get("width", 1280),
        height=camera.get("height", 720),
        fps=camera.get("fps", 30),
        mirror=camera.get("mirror", False),
        sharpness=camera.get("sharpness", 32),
        pixel_format=camera.get("pixel_format", "mjpg"),
        warmup=camera.get("read_warmup", 2),
        logger=log,
    )
    models = ModelCache(config["models"].get("dir"), device=infer_device, logger=log)
    return VisionContext(config, models, frames, log=log)


def warm_up(service: VisionService, log) -> None:
    """预打开相机 + 预加载常用 YOLO 模型，避免第一次识别吃掉 2-3 秒。"""
    context = service.context
    try:
        context.frames.open()
    except Exception as error:  # noqa: BLE001
        log("[vision] 预热相机失败（不影响服务启动，首次请求会重试）：%s" % error)
    yolo_keys = ("crowd", "light", "sign", "trash_bins", "trash_items",
                 "fire", "nameplate", "plate", "bike_count")
    for key in yolo_keys:
        try:
            started = time.monotonic()
            context.model(context.model_path(key), imgsz=640)
            log("[vision] 预热 %s 完成（%.2f s）" % (key, time.monotonic() - started))
        except Exception as error:  # noqa: BLE001
            log("[vision] 预热 %s 失败：%s" % (key, error))
    # PaddleOCR / HyperLPR 首次初始化也要 1-3 s，提前拉起来
    try:
        started = time.monotonic()
        import iv_tasks_ocr
        chinese = iv_tasks_ocr.chinese_ocr()
        iv_tasks_ocr.plate_ocr()
        # ChineseOcr 是懒加载：显式触发一次 PaddleOCR 构造（约 1-3 s）
        engine = getattr(chinese, "_engine", None)
        if callable(engine):
            engine()
        log("[vision] 预热 OCR 引擎完成（%.2f s）" % (time.monotonic() - started))
    except Exception as error:  # noqa: BLE001
        log("[vision] 预热 OCR 失败（第一次 OCR 会慢）：%s" % error)
    if "meter" in service.handlers:
        try:
            started = time.monotonic()
            import iv_tasks_meter
            iv_tasks_meter.warmup(context)
            log("[vision] 预热 meter 完成（%.2f s）" % (time.monotonic() - started))
        except Exception as error:  # noqa: BLE001
            log("[vision] 预热 meter 失败（第一次识别会慢一些）：%s" % error)


def check_thermal_config(config: dict, log) -> None:
    """启动时提示红外串口/回放路径，避免 12 号点才发现没配置。"""
    thermal = config.get("thermal", {}) or {}
    port = str(thermal.get("port") or "").strip()
    capture = str(thermal.get("capture") or "").strip()
    if capture:
        path = Path(capture).expanduser()
        if not path.is_file():
            log("[vision] 警告：thermal.capture 不存在：%s" % path)
        return
    if port and not os.path.exists(port):
        log("[vision] 警告：thermal.port=%s 不存在；temperature 任务前请按 "
            "ls -l /dev/serial/by-id/ 填实际设备" % port)


def check_meter_model(config: dict, log) -> None:
    """站房仪表的权重不在 Git，启动时提示一次路径，避免第一次识别才发现。"""
    meter = config.get("meter", {})
    path = Path(str(meter.get("model") or DEFAULT_GAUGE_MODEL)).expanduser()
    if path.is_file():
        log("[vision] Exp16 仪表模型：%s" % path)
    else:
        log("[vision] 警告：找不到 Exp16 仪表模型 %s，task=meter 会失败" % path)


# ---------------------------------------------------------------------------
# 离线自测
# ---------------------------------------------------------------------------

def run_self_test(args: argparse.Namespace, config: dict) -> int:
    tasks = list(TASK_HANDLERS) if args.self_test in ("all", "ALL") else \
        [str(args.self_test).strip().lower()]
    quiet = lambda *_args: None  # noqa: E731
    context = create_context(config, args.infer_device,
                             logger=quiet if args.json_output else None)
    service = VisionService(context, logger=quiet if args.json_output else None)
    if args.self_test in ("all", "ALL") and not args.json_output:
        for task in list(tasks):
            if task not in service.handlers:
                print("[self-test] %s 缺依赖：%s" % (task, service.load_errors.get(task, "?")))
    needs_camera = any(task != "temperature" for task in tasks)
    if needs_camera:
        try:
            context.frames.open()
        except Exception as error:  # noqa: BLE001
            print("[self-test] 帧源打开失败：%s" % error, file=sys.stderr)
            return 1
        if not args.json_output:
            print("[self-test] 帧源：%s" % context.frames.description)
    elif not args.json_output:
        print("[self-test] temperature 使用红外相机/回放，不需要白光帧源")
    if not args.json_output:
        print("[self-test] 输出目录：%s" % context.run_dir)
    exit_code = 0
    for task in tasks:
        if task == "light":
            # 红绿灯自测会连拍多帧，deadline 放宽一点
            params = args.params if args.params != "{}" else '{"deadline": 5.0}'
        else:
            params = args.params
        started = time.monotonic()
        result = service.handle(task, args.point, args.sub_id, params)
        payload = {
            "task": task,
            "success": result.ok,
            "result": result.result,
            "extra": result.extra,
            "images": result.images,
            "elapsed": round(time.monotonic() - started, 3),
        }
        if args.json_output:
            print(json.dumps(payload, ensure_ascii=False, default=str))
        else:
            print("\n[self-test] %s → success=%s %.2fs"
                  % (task, result.ok, payload["elapsed"]))
            print("  result: %s" % (result.result or "(空)"))
            print("  extra : %s" % json.dumps(result.extra, ensure_ascii=False))
            for path in result.images:
                print("  image : %s" % path)
        exit_code = exit_code or (0 if result.ok else 2)
    summary = context.summary()
    if not args.json_output:
        print("[self-test] 日志：%s" % summary["log"])
    context.frames.close()
    return exit_code


# ---------------------------------------------------------------------------
# ROS 服务
# ---------------------------------------------------------------------------

def serve(args: argparse.Namespace, config: dict) -> int:
    import threading

    import rospy
    from nav_demo.srv import InspectionVision, InspectionVisionResponse

    rospy.init_node(NODE_NAME, anonymous=False, disable_signals=True)
    log = lambda text: rospy.loginfo(text)  # noqa: E731
    context = create_context(config, args.infer_device, logger=log)
    service = VisionService(context, logger=log)
    lock = threading.Lock()

    log("[vision] 配置：%s" % config.get("_config_path", "(默认)"))
    log("[vision] 模型目录：%s" % Path(config["models"]["dir"]).expanduser())
    log("[vision] 输出目录：%s" % context.run_dir)
    log("[vision] 可用任务：%s" % service.available_summary())
    check_meter_model(config, log)
    check_thermal_config(config, log)
    if args.warmup:
        warm_up(service, log)

    def handle(request):
        response = InspectionVisionResponse()
        try:
            with lock:  # ROS1 服务默认多线程回调；识别和相机都只能串行
                result = service.handle(request.task, request.point_id,
                                        request.sub_id, request.params)
        except Exception as error:  # noqa: BLE001
            rospy.logerr("[vision] 服务内部异常：\n%s" % traceback.format_exc(limit=5))
            response.success = False
            response.result = ""
            response.extra = json.dumps(
                {"request_id": "", "error": "%s: %s" % (type(error).__name__, error)},
                ensure_ascii=False, default=str)
            response.images = []
            return response
        response.success = bool(result.ok)
        response.result = result.result
        response.extra = json.dumps(result.extra, ensure_ascii=False, default=str)
        response.images = list(result.images)
        return response

    rospy.Service(args.service_name, InspectionVision, handle)
    log("[vision] 服务已启动：%s" % args.service_name)

    def shutdown():
        try:
            import iv_tasks_thermal
            iv_tasks_thermal.close_session()
        except Exception:  # noqa: BLE001
            pass
        try:
            context.frames.close()
        except Exception:  # noqa: BLE001
            pass
        try:
            summary = context.summary()
            log("[vision] 本轮汇总：total=%s log=%s by_task=%s"
                % (summary["total"], summary["log"],
                   json.dumps(summary["by_task"], ensure_ascii=False)))
        except Exception:  # noqa: BLE001
            pass

    rospy.on_shutdown(shutdown)
    try:
        rospy.spin()
    except KeyboardInterrupt:
        pass
    return 0


# ---------------------------------------------------------------------------
# main
# ---------------------------------------------------------------------------

def main(argv=None) -> int:
    args = build_parser().parse_args(argv)
    if args.list_tasks:
        print("支持的任务：")
        for task in TASK_HANDLERS:
            print("  %-12s %s" % (task, TASK_HANDLERS[task][0]))
        print("\n离线自测示例：")
        print("  %s %s --self-test light --device <图片目录或视频> --point 1 "
              "--sub-id 红绿灯1" % (sys.executable, __file__))
        return 0

    try:
        config = apply_overrides(load_config(args.config), args)
    except VisionError as error:
        print("[vision] %s" % error, file=sys.stderr)
        return 1
    if not config.get("_config_found"):
        print("[vision] 提示：没找到配置文件，使用内置默认值", file=sys.stderr)

    if args.self_test:
        return run_self_test(args, config)
    try:
        return serve(args, config)
    except ImportError as error:
        print("[vision] 缺少 ROS/服务依赖：%s\n"
              "请先 source /opt/ros/noetic/setup.bash 和 "
              "~/bit_ws/devel/setup.bash，并用 /usr/bin/python3 运行。" % error,
              file=sys.stderr)
        return 1
    except CameraUnavailable as error:
        print("[vision] %s" % error, file=sys.stderr)
        return 1


if __name__ == "__main__":
    sys.exit(main())
