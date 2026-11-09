#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""对 sweep.avi 做 YOLO 跟踪，按唯一 track ID 统计目标数量。

思路：
  - 逐帧 YOLO + ByteTrack，同一辆车在多帧里只保留一个 track ID；
  - 过滤少于 --min-frames 的短轨迹（重复框/误检/碎片）；
  - 每条轨迹按类别投票，避免单帧类别抖动；
  - 输出唯一数量、标注视频、每条轨迹的裁剪图和 summary JSON。

注意：这不是把每帧检测数相加。没有跟踪的照片逐帧计数一定会重复。
"""

import argparse
import collections
import json
import os
import sys
import time
from datetime import datetime
from pathlib import Path

import cv2
import numpy as np


DEFAULT_MODEL = str(Path.home() / "AI视觉" / "Cycle.pt")


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
    return parser.parse_args(argv)


def main(argv=None) -> int:
    args = parse_args(argv)
    video = Path(args.video).expanduser()
    model_path = Path(args.model).expanduser()
    if not video.is_file():
        print("[count_video] 视频不存在：{}".format(video), file=sys.stderr)
        return 2
    if not model_path.is_file():
        print("[count_video] 模型不存在：{}".format(model_path), file=sys.stderr)
        return 2

    try:
        import lap  # noqa: F401  ByteTrack 需要
    except Exception:
        print("[count_video] 缺少 lap，ByteTrack 无法运行。\n"
              "本机已手动装到 ~/.local/lib/python3.8/site-packages；"
              "其他机器可用 pip install lap（需要能正常联网/编译）。",
              file=sys.stderr)
        return 2
    try:
        from ultralytics import YOLO
    except Exception as error:
        print("[count_video] 无法导入 ultralytics：{}".format(error), file=sys.stderr)
        return 2

    output_dir = Path(args.output_dir).expanduser() if args.output_dir else (
        video.parent / "识别_{}_{}".format(
            datetime.now().strftime("%Y%m%d_%H%M%S"), video.stem))
    output_dir.mkdir(parents=True, exist_ok=True)

    allowed = {name.strip() for name in args.classes.split(",") if name.strip()}
    model = YOLO(str(model_path))
    cap = cv2.VideoCapture(str(video))
    if not cap.isOpened():
        print("[count_video] 打不开视频：{}".format(video), file=sys.stderr)
        return 2
    fps = cap.get(cv2.CAP_PROP_FPS) or 15.0
    width = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH))
    height = int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
    total_expected = int(cap.get(cv2.CAP_PROP_FRAME_COUNT))

    tracks = {}
    per_frame = []  # [(tid, (x1,y1,x2,y2), conf)]
    frame_no = 0
    started = time.time()
    while True:
        ok, frame = cap.read()
        if not ok:
            break
        frame_no += 1
        results = model.track(
            frame, persist=True, tracker=args.tracker,
            conf=max(0.01, min(0.99, args.conf)), imgsz=args.imgsz,
            device=args.device, verbose=False)
        result = results[0] if results else None
        dets = []
        if (result is not None and result.boxes is not None
                and result.boxes.id is not None):
            ids = result.boxes.id.cpu().numpy().astype(int)
            classes = result.boxes.cls.cpu().numpy().astype(int)
            confs = result.boxes.conf.cpu().numpy()
            boxes = result.boxes.xyxy.cpu().numpy()
            for tid, cls, conf, box in zip(ids, classes, confs, boxes):
                tid = int(tid)
                cls = int(cls)
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
                rec["votes"][cls] += 1
                rec["frames"] += 1
                rec["last"] = frame_no
                rec["max_conf"] = max(rec["max_conf"], float(conf))
                area = (x2 - x1) * (y2 - y1)
                if area > rec["best_area"]:
                    rec["best_area"] = area
                    rec["best_frame"] = frame_no
                    rec["crop"] = frame[y1:y2, x1:x2].copy()
                dets.append((tid, (x1, y1, x2, y2), float(conf)))
        per_frame.append(dets)
        if frame_no % 50 == 0:
            print("[count_video] {}/{} 帧".format(frame_no, total_expected))
    cap.release()

    names = model.names
    selected = {}
    for tid, rec in tracks.items():
        if rec["frames"] < max(1, args.min_frames):
            continue
        class_id, _ = rec["votes"].most_common(1)[0]
        class_name = str(names.get(class_id, class_id))
        if allowed and class_name not in allowed:
            continue
        rec["class_id"] = class_id
        rec["class_name"] = class_name
        selected[tid] = rec

    by_class = collections.Counter(rec["class_name"] for rec in selected.values())
    summary = {
        "video": str(video),
        "model": str(model_path),
        "fps": round(float(fps), 3),
        "total_frames": frame_no,
        "min_frames": args.min_frames,
        "conf": args.conf,
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
            safe = "".join(ch if ch.isalnum() or ch in "-_" else "_"
                           for ch in rec["class_name"]) or "obj"
            cv2.imwrite(str(output_dir / "{}_{:02d}.jpg".format(safe, tid)),
                        crop, [int(cv2.IMWRITE_JPEG_QUALITY), 95])

    with (output_dir / "track_summary.json").open("w", encoding="utf-8") as handle:
        json.dump(summary, handle, ensure_ascii=False, indent=2)
    (output_dir / "count.txt").write_text(
        "{} 辆\n{}\n".format(
            len(selected),
            "，".join("{} {}".format(k, v) for k, v in sorted(by_class.items()))),
        encoding="utf-8")

    # 标注视频：只画最终选中的 track，避免重复框/短碎片干扰
    if not args.no_video:
        cap = cv2.VideoCapture(str(video))
        writer = cv2.VideoWriter(
            str(output_dir / "sweep_tracked.avi"),
            cv2.VideoWriter_fourcc(*"MJPG"), fps, (width, height))
        for dets in per_frame:
            ok, frame = cap.read()
            if not ok:
                break
            for tid, box, conf in dets:
                rec = selected.get(tid)
                if rec is None:
                    continue
                x1, y1, x2, y2 = box
                color = (0, 255, 0) if rec["class_name"].upper() == "UP" else (0, 165, 255)
                cv2.rectangle(frame, (x1, y1), (x2, y2), color, 2)
                label = "{} #{}".format(rec["class_name"], tid)
                cv2.putText(frame, label, (x1, max(20, y1 - 6)),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.65, color, 2)
            cv2.putText(frame, "unique: {}".format(len(selected)),
                        (20, 34), cv2.FONT_HERSHEY_SIMPLEX, 0.9, (0, 255, 0), 2)
            writer.write(frame)
        cap.release()
        writer.release()

    # 所有唯一轨迹的缩略图
    crops = []
    for tid, rec in sorted(selected.items(), key=lambda kv: (kv[1]["class_name"], kv[0])):
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
            cv2.putText(canvas, "{} #{}".format(class_name, tid),
                        (x, y - 8), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 1)
        cv2.imwrite(str(output_dir / "tracks_contact.jpg"), canvas,
                    [int(cv2.IMWRITE_JPEG_QUALITY), 95])

    print("[count_video] 完成：{} 辆（{}）".format(
        len(selected),
        "，".join("{} {}".format(k, v) for k, v in sorted(by_class.items())) or "无"))
    print("[count_video] 结果目录：{}".format(output_dir))
    print("[count_video] 耗时 {:.1f}s".format(time.time() - started))
    return 0


if __name__ == "__main__":
    sys.exit(main())
