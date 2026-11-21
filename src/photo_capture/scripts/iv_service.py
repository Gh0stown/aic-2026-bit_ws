#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""inspection_vision 的任务分发层（不依赖 ROS，方便单元测试）。

`VisionService.handle()` 接收 srv 的四个字段，返回 TaskResult；节点主程序只
负责把 TaskResult 填进 InspectionVisionResponse。各个 task 的识别代码放在
`iv_tasks_*.py`，按需 import：某个任务的可选依赖缺失（例如没装 PaddleOCR）
只影响该任务，节点其余任务照常工作。
"""

from __future__ import annotations

import importlib
import time
import traceback
from datetime import datetime
from typing import Any, Dict, List, Optional, Tuple

from iv_common import (CameraUnavailable, ModelUnavailable, TaskResult,
                       VisionContext, VisionError, VisionTimeout, as_float,
                       parse_params)

# task -> (模块, 处理函数)。顺序也是 --self-test 列出任务的顺序。
TASK_HANDLERS: Dict[str, Tuple[str, str]] = {
    "crowd": ("iv_tasks_detect", "handle_crowd"),
    "sign": ("iv_tasks_detect", "handle_sign"),
    "light": ("iv_tasks_detect", "handle_light"),
    "trash": ("iv_tasks_trash", "handle"),
    "fire": ("iv_tasks_ocr", "handle_fire"),
    "plate": ("iv_tasks_ocr", "handle_plate"),
    "temperature": ("iv_tasks_thermal", "handle"),
    "meter": ("iv_tasks_meter", "handle"),
    "ebike": ("iv_tasks_ebike", "handle"),
}

TASK_ALIASES = {
    "person": "crowd",
    "people": "crowd",
    "red_light": "light",
    "traffic_light": "light",
    "gauge": "meter",
    "thermal": "temperature",
    "trash_can": "trash",
}

DEFAULT_MAX_TIMEOUT = 300.0


class VisionService:
    """把一次服务请求分发给对应 handler，并保证请求串行、结果必落日志。"""

    def __init__(self, context: VisionContext, logger=None):
        self.context = context
        self.log = logger or context.log
        self.handlers: Dict[str, Any] = {}
        self.load_errors: Dict[str, str] = {}
        self._load_all()

    # -- handler 加载 -------------------------------------------------
    def _load_all(self) -> None:
        for task, (module_name, function_name) in TASK_HANDLERS.items():
            try:
                module = importlib.import_module(module_name)
                handler = getattr(module, function_name)
            except Exception as error:  # noqa: BLE001
                self.load_errors[task] = "{}: {}".format(type(error).__name__, error)
                self.log("[vision] 任务 %s 不可用：%s" % (task, self.load_errors[task]))
                continue
            self.handlers[task] = handler

    @property
    def available_tasks(self) -> List[str]:
        return [task for task in TASK_HANDLERS if task in self.handlers]

    def available_summary(self) -> str:
        parts = []
        for task in TASK_HANDLERS:
            parts.append(task if task in self.handlers else task + "(缺依赖)")
        return ", ".join(parts)

    # -- 请求参数 -----------------------------------------------------
    def timeout_for(self, task: str, params: Dict[str, Any]) -> float:
        timeouts = self.context.config.get("timeouts", {})
        default = as_float(timeouts.get(task, timeouts.get("default", 6.0)), 6.0)
        # 兼容三种写法：deadline / timeout 是“本次允许的秒数”，
        # deadline_unix 是绝对 Unix 时间戳。
        value = params.get("deadline", params.get("timeout"))
        timeout = as_float(value, default) if value is not None else default
        if "deadline_unix" in params:
            timeout = min(timeout, as_float(params.get("deadline_unix"), 0.0) - time.time())
        if task == "ebike" and params.get("pan"):
            pan_timeout = as_float(
                params.get("duration"),
                as_float(self.context.config.get("timeouts", {}).get("ebike_pan", 150.0),
                         150.0)) + 20.0
            timeout = max(timeout, pan_timeout)
        return max(0.5, min(DEFAULT_MAX_TIMEOUT, timeout))

    @staticmethod
    def request_id_for(params: Dict[str, Any], task: str, point_id: int) -> str:
        value = params.get("request_id")
        if value:
            return str(value)[:120]
        return "auto-%s-p%s-%s" % (task, point_id,
                                   datetime.now().strftime("%H%M%S_%f")[:-3])

    # -- 主入口 -------------------------------------------------------
    def handle(self, task: str, point_id: int, sub_id: str, params_text: str) -> TaskResult:
        raw_task = str(task or "").strip().lower()
        normalized = TASK_ALIASES.get(raw_task, raw_task)
        params, params_error = parse_params(params_text)
        if params_error:
            params = {}
        request_id = "auto-invalid-%s" % datetime.now().strftime("%H%M%S_%f")[:-3]
        timeout = self.timeout_for(normalized, params)
        self.context.begin_request(normalized or "unknown", point_id, sub_id,
                                   params, request_id, timeout)
        if not normalized:
            return self.context.finish(self.context.fail("task 为空"))
        if normalized not in TASK_HANDLERS:
            return self.context.finish(self.context.fail(
                "未知任务：{}（支持：{}）".format(task, ", ".join(TASK_HANDLERS))))
        if params_error:
            return self.context.finish(self.context.fail(params_error))
        handler = self.handlers.get(normalized)
        if handler is None:
            reason = self.load_errors.get(normalized, "handler 未加载")
            return self.context.finish(self.context.fail("任务不可用：{}".format(reason)))

        # params 里的 request_id 可能晚于 begin_request 设置（这里统一覆盖）
        self.context.request_id = request_id = self.request_id_for(
            params, normalized, point_id)
        try:
            result = handler(self.context, params)
            if not isinstance(result, TaskResult):
                result = self.context.fail(
                    "handler 返回值类型错误：{}".format(type(result).__name__))
        except VisionTimeout as error:
            result = self.context.fail(str(error), result="识别超时")
        except CameraUnavailable as error:
            result = self.context.fail("相机不可用：{}".format(error), result="相机不可用")
        except ModelUnavailable as error:
            result = self.context.fail("模型不可用：{}".format(error), result="模型不可用")
        except VisionError as error:
            result = self.context.fail(str(error))
        except Exception as error:  # noqa: BLE001
            detail = traceback.format_exc(limit=4)
            self.log("[vision] %s 异常：\n%s" % (normalized, detail))
            result = self.context.fail("{}: {}".format(type(error).__name__, error))
        return self.context.finish(result)
