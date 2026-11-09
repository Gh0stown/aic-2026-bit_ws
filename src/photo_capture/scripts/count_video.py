#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""对 sweep.avi 做 YOLO 跟踪，按唯一 track ID 统计目标数量。

给导航脚本的接口有两种：

1. Python 直接调用：
       from count_video import count_video
       result = count_video(video=".../sweep.avi", model=".../Cycle.pt",
                            min_frames=20)
       print(result.count, result.by_class)

2. 子进程 / CLI（--json 只输出 JSON，适合导航脚本解析）：
       python3 count_video.py --video ... --json

思路：
  - 逐帧 YOLO + ByteTrack，同一目标在多帧里只保留一个 track ID；
  - 过滤少于 min_frames 的短轨迹（重复框/误检/碎片）；
  - 每条轨迹按类别投票，避免单帧类别抖动；
  - 输出唯一数量、标注视频、每条轨迹裁剪图和 summary JSON。

注意：这不是把每帧检测数相加；没有跟踪的逐帧计数一定会重复。
"""

import argparse
import collections
import json
import sys
import time
from dataclasses import asdict, dataclass, field
from datetime import datetime
from pathlib import Path
from typing import Callable, Dict, Iterable, List, Optional, Union

import cv2
import numpy as np


DEFAULT_MODEL = str(Path.home() / "AI视觉" / "Cycle.pt")
SCHEMA_VERSION = 1


@dataclass
class CountResult:
    """count_video() 的稳定返回结构；导航脚本不要解析 stdout。"""

    ok: bool
    count: int = 0
    by_class: Dict[str, int] = field(default_factory=dict)
    tracks: List[dict] = field(default_factory=list)
    total_frames: int = 0
    fps: float = 0.0
    video: str = ""
    model: str = ""
    output_dir: str = ""
    summary_path: str = ""
    tracked_video: str = ""
    contact_sheet: str = ""
    message: str = ""

    def to_dict(self) -> dict:
        payload = asdict(self)
        payload["schema_version"] = SCHEMA_VERSION
        return payload


def _normalize_classes(classes: Union[str, Iterable[str], None]) -> set:
    if classes is None:
        return set()
    if isinstance(classes, str):
        return {name.strip() for name in classes.split(",") if name.strip()}
    return {str(name).strip() for name in classes if str(name).strip()}


def _safe_name(name: str) -> str:
    return "".join(
        ch if ch.isalnum() or ch in "-_" else "_" for ch in str(name)) or "obj"


def count_video(
    video: Union[str, Path],
    model: Optional[Union[str, Path]] = None,
    conf: float = 0.25,
    imgsz: int = 640,
    device: str = "cpu",
    tracker: str = "bytetrack.yaml",
    min_frames: int = 20,
    classes: Union[str, Iterable[str], None] = "",
    output_dir: Optional[Union[str, Path]] = None,
    save_video: bool = True,
    progress: Optional[Callable[[int, int], None]] = None,
) -> CountResult:
    """统计视频里的唯一目标数量。

    参数：
      video       输入 sweep.avi
      model       YOLO 权重，默认 ~/AI视觉/Cycle.pt
      min_frames  轨迹至少出现多少帧才计数；15fps 下默认 20 约 1.3 秒
      classes     只统计这些类别，如 "UP,DOWN"；空表示全部
      output_dir  输出目录；空则放在视频旁边的 识别_<时间戳>_<视频名>/
      save_video  是否写 sweep_tracked.avi

    返回 CountResult；不抛业务异常，失败时 ok=False、message 里写原因。
    """
    video = Path(video).expanduser()
    model_path = Path(model).expanduser() if model else Path(DEFAULT_MODEL)
    base = CountResult(
        ok=False, video=str(video), model=str(model_path))

    if not video.is_file():
        base.message = "视频不存在：{}".format(video)
        return base
    if not model_path.is_file():
        base.message = "模型不存在：{}".format(model_path)
        return base

    try:
        import lap  # noqa: F401  ByteTrack 需要
    except Exception:
        base.message = (
            "缺少 lap，ByteTrack 无法运行；本机已装到 "
            "~/.local/lib/python3.8/site-packages，其他机器可 pip install lap")
        return base
    try:
        from ultralytics import YOLO
    except Exception as error:
        base.message = "无法导入 ultralytics：{}".format(error)
        return base

    if output_dir is None:
        output_dir = video.parent / "识别_{}_{}".format(
            datetime.now().strftime("%Y%m%d_%H%M%S"), video.stem)
    output_dir = Path(output_dir).expanduser()
    output_dir.mkdir(parents=True, exist_ok=True)

    allowed = _normalize_classes(classes)
    cap = cv2.VideoCapture(str(video))
    if not cap.isOpened():
        base.message = "打不开视频：{}".format(video)
        return base
    fps = float(cap.get(cv2.CAP_PROP_FPS) or 15.0)
    width = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH))
    height = int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
    total_expected = int(cap.get(cv2.CAP_PROP_FRAME_COUNT) or 0)
    base.fps = round(fps, 3)
    base.output_dir = str(output_dir)

    model_obj = YOLO(str(model_path))
    tracks = {}
    per_frame = []
    frame_no = 0
    started = time.time()

    try:
        while True:
            ok, frame = cap.read()
            if not ok:
                break
            frame_no += 1
            results = model_obj.track(
                frame, persist=True, tracker=tracker,
                conf=max(0.01, min(0.99, float(conf))),
                imgsz=max(64, int(imgsz)), device=device, verbose=False)
            result = results[0] if results else None
            dets = []
            if (result is not None and result.boxes is not None
                    and result.boxes.id is not None):
                ids = result.boxes.id.cpu().numpy().astype(int)
                cls_ids = result.boxes.cls.cpu().numpy().astype(int)
                confs = result.boxes.conf.cpu().numpy()
                boxes = result.boxes.xyxy.cpu().numpy()
                for tid, cls_id, score, box in zip(ids, cls_ids, confs, boxes):
                    tid = int(tid)
                    cls_id = int(cls_id)
                    x1, y1, x2, y2 = [int(round(v)) for v in box]
                    x1 = max(0, min(width - 1, x1))
                    y1 = max(0, min(height - 1, y1))
                    x2 = max(0, min(width, x2))
                    y2 = max(0, min(height, y2))
                    if x2 <= x1 or y2 <= y1:
                        continue
                    rec = tracks.setdefault(tid, {
                        "votes": collections.Counter(),
                        "frames": 0,
                        "first": frame_no,
                        "last": frame_no,
                        "max_conf": 0.0,
                        "best_area": 0,
                        "crop": None,
                        "best_frame": frame_no,
                    })
                    rec["votes"][cls_id] += 1
                    rec["frames"] += 1
                    rec["last"] = frame_no
                    rec["max_conf"] = max(rec["max_conf"], float(score))
                    area = (x2 - x1) * (y2 - y1)
                    if area > rec["best_area"]:
                        rec["best_area"] = area
                        rec["best_frame"] = frame_no
                        rec["crop"] = frame[y1:y2, x1:x2].copy()
                    dets.append((tid, (x1, y1, x2, y2), float(score)))
            per_frame.append(dets)
            if progress is not None and (frame_no % 50 == 0):
                try:
                    progress(frame_no, total_expected)
                except Exception:
                    pass
    except Exception as error:
        base.message = "跟踪失败：{}".format(error)
        return base
    finally:
        cap.release()

    names = model_obj.names
    selected = {}
    for tid, rec in tracks.items():
        if rec["frames"] < max(1, int(min_frames)):
            continue
        class_id = rec["votes"].most_common(1)[0][0]
        class_name = str(names.get(class_id, class_id))
        if allowed and class_name not in allowed:
            continue
        rec["class_id"] = class_id
        rec["class_name"] = class_name
        selected[tid] = rec

    by_class = collections.Counter(rec["class_name"] for rec in selected.values())
    summary = {
        "schema_version": SCHEMA_VERSION,
        "ok": True,
        "video": str(video),
        "model": str(model_path),
        "fps": round(fps, 3),
        "total_frames": frame_no,
        "min_frames": int(min_frames),
        "conf": float(conf),
        "count": len(selected),
        "by_class": dict(sorted(by_class.items())),
        "tracks": [],
    }

    for tid, rec in sorted(selected.items(), key=lambda kv: -kv[1]["frames"]):
        summary["tracks"].append({
            "id": tid,
            "class": rec["class_name"],
            "class_votes": {str(names.get(k, k)): v
                            for k, v in rec["votes"].items()},
            "frames": rec["frames"],
            "first_frame": rec["first"],
            "last_frame": rec["last"],
            "max_conf": round(rec["max_conf"], 4),
            "best_frame": rec["best_frame"],
        })
        crop = rec["crop"]
        if crop is not None and crop.size:
            cv2.imwrite(
                str(output_dir / "{}_{:02d}.jpg".format(
                    _safe_name(rec["class_name"]), tid)),
                crop, [int(cv2.IMWRITE_JPEG_QUALITY), 95])

    summary_path = output_dir / "track_summary.json"
    with summary_path.open("w", encoding="utf-8") as handle:
        json.dump(summary, handle, ensure_ascii=False, indent=2)
    (output_dir / "count.txt").write_text(
        "{} 辆\n{}\n".format(
            len(selected),
            "，".join("{} {}".format(k, v) for k, v in sorted(by_class.items()))),
        encoding="utf-8")

    tracked_video = ""
    if save_video:
        cap = cv2.VideoCapture(str(video))
        video_path = output_dir / "sweep_tracked.avi"
        writer = cv2.VideoWriter(
            str(video_path), cv2.VideoWriter_fourcc(*"MJPG"),
            fps, (width, height))
        for dets in per_frame:
            ok, frame = cap.read()
            if not ok:
                break
            for tid, box, _score in dets:
                rec = selected.get(tid)
                if rec is None:
                    continue
                x1, y1, x2, y2 = box
                color = ((0, 255, 0) if rec["class_name"].upper() == "UP"
                         else (0, 165, 255))
                cv2.rectangle(frame, (x1, y1), (x2, y2), color, 2)
                cv2.putText(
                    frame, "{} #{}".format(rec["class_name"], tid),
                    (x1, max(20, y1 - 6)),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.65, color, 2)
            cv2.putText(
                frame, "unique: {}".format(len(selected)),
                (20, 34), cv2.FONT_HERSHEY_SIMPLEX, 0.9, (0, 255, 0), 2)
            writer.write(frame)
        cap.release()
        writer.release()
        tracked_video = str(video_path)

    contact_sheet = ""
    crops = []
    for tid, rec in sorted(selected.items(),
                           key=lambda kv: (kv[1]["class_name"], kv[0])):
        if rec["crop"] is not None and rec["crop"].size:
            crops.append((tid, rec["class_name"], rec["crop"]))
    if crops:
        tw, th = 300, 220
        cols = min(3, len(crops))
        rows = (len(crops) + cols - 1) // cols
        canvas = np.full(
            (rows * (th + 34) + 10, cols * (tw + 10) + 10, 3),
            30, dtype=np.uint8)
        for index, (tid, class_name, crop) in enumerate(crops):
            row, col = divmod(index, cols)
            x = 10 + col * (tw + 10)
            y = 30 + row * (th + 34)
            thumb = cv2.resize(crop, (tw, th), interpolation=cv2.INTER_AREA)
            canvas[y:y + th, x:x + tw] = thumb
            cv2.putText(
                canvas, "{} #{}".format(class_name, tid),
                (x, y - 8), cv2.FONT_HERSHEY_SIMPLEX, 0.6,
                (255, 255, 255), 1)
        sheet_path = output_dir / "tracks_contact.jpg"
        cv2.imwrite(str(sheet_path), canvas,
                    [int(cv2.IMWRITE_JPEG_QUALITY), 95])
        contact_sheet = str(sheet_path)

    return CountResult(
        ok=True,
        count=len(selected),
        by_class=dict(sorted(by_class.items())),
        tracks=summary["tracks"],
        total_frames=frame_no,
        fps=round(fps, 3),
        video=str(video),
        model=str(model_path),
        output_dir=str(output_dir),
        summary_path=str(summary_path),
        tracked_video=tracked_video,
        contact_sheet=contact_sheet,
        message="",
    )


def parse_args(argv=None):
    parser = argparse.ArgumentParser(
        description="用 YOLO + ByteTrack 统计 sweep.avi 里的唯一目标数量")
    parser.add_argument("--video", required=True, help="输入 sweep.avi 路径")
    parser.add_argument("--model", default=DEFAULT_MODEL,
                        help="YOLO 权重，默认 ~/AI视觉/Cycle.pt")
    parser.add_argument("--conf", type=float, default=0.25,
                        help="检测置信度（默认 0.25）")
    parser.add_argument("--imgsz", type=int, default=640,
                        help="推理尺寸（默认 640）")
    parser.add_argument("--device", default="cpu",
                        help="推理设备，cpu 或 cuda（默认 cpu）")
    parser.add_argument("--tracker", default="bytetrack.yaml",
                        help="Ultralytics tracker 配置（默认 bytetrack.yaml）")
    parser.add_argument("--min-frames", type=int, default=20,
                        help="轨迹至少出现多少帧才计数（默认 20，15fps 约 1.3s）")
    parser.add_argument("--classes", default="",
                        help="只统计这些类别，逗号分隔；默认全部")
    parser.add_argument("--output-dir", default="",
                        help="输出目录；默认在视频同目录下建 识别_<时间戳>")
    parser.add_argument("--no-video", action="store_true",
                        help="不写标注视频，只出数量/裁剪图/JSON")
    parser.add_argument("--json", action="store_true",
                        help="只向 stdout 输出 JSON（供导航脚本解析）")
    parser.add_argument("--json-out", default="",
                        help="把结果 JSON 额外写到这个路径")
    return parser.parse_args(argv)


def main(argv=None) -> int:
    args = parse_args(argv)
    progress = None
    if not args.json:
        def progress(done: int, total: int) -> None:
            print("[count_video] {}/{} 帧".format(done, total))
    result = count_video(
        video=args.video,
        model=args.model,
        conf=args.conf,
        imgsz=args.imgsz,
        device=args.device,
        tracker=args.tracker,
        min_frames=args.min_frames,
        classes=args.classes,
        output_dir=args.output_dir or None,
        save_video=not args.no_video,
        progress=progress,
    )
    payload = result.to_dict()
    if args.json_out:
        Path(args.json_out).expanduser().write_text(
            json.dumps(payload, ensure_ascii=False, indent=2),
            encoding="utf-8")
    if args.json:
        print(json.dumps(payload, ensure_ascii=False))
    else:
        if result.ok:
            print("[count_video] 完成：{} 辆（{}）".format(
                result.count,
                "，".join("{} {}".format(k, v)
                         for k, v in sorted(result.by_class.items())) or "无"))
            print("[count_video] 结果目录：{}".format(result.output_dir))
        else:
            print("[count_video] 失败：{}".format(result.message),
                  file=sys.stderr)
    return 0 if result.ok else 2


if __name__ == "__main__":
    sys.exit(main())
