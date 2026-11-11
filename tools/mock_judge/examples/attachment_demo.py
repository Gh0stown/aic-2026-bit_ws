"""选手端完整上报 Demo（可直接运行）。

来源：上传案例.zip / 上传案例/测试代码.py；仅将地址、队名改为 argparse 配置。
用法：
    python tools/mock_judge/examples/attachment_demo.py
    python tools/mock_judge/examples/attachment_demo.py --url http://127.0.0.1:8080 --team 示例战队

依赖：只依赖 requests。示意图片用标准库（zlib）现场生成 PNG，
      不需要 Pillow、不需要 OpenCV，拷到任何一台装了 requests 的机器都能跑。
本 Demo 演示：
    - /api/hello 赛前连通自检
    - 单张图片上报（车牌 1/2/3 号位，三次，各自在页面上可切换）
    - 多张图片一次上报（人群识别，多视角）
"""

import argparse
import struct
import zlib

import requests

JUDGE_URL = "http://127.0.0.1:8080"
TEAM = "示例战队"


def _make_png(color=(235, 242, 252), box=(47, 107, 255)) -> bytes:
    """纯标准库生成一张带蓝色边框的示意 PNG（640×360），不依赖任何第三方库。"""
    w, h, border = 640, 360, 8
    line = bytes(box) * w
    mid = bytes(box) * border + bytes(color) * (w - 2 * border) + bytes(box) * border
    raw = b"".join(b"\x00" + r for r in ([line] * border + [mid] * (h - 2 * border) + [line] * border))

    def chunk(tag: bytes, data: bytes) -> bytes:
        return (struct.pack(">I", len(data)) + tag + data
                + struct.pack(">I", zlib.crc32(tag + data) & 0xFFFFFFFF))

    ihdr = struct.pack(">IIBBBBB", w, h, 8, 2, 0, 0, 0)   # 8bit RGB
    return (b"\x89PNG\r\n\x1a\n" + chunk(b"IHDR", ihdr)
            + chunk(b"IDAT", zlib.compress(raw, 6)) + chunk(b"IEND", b""))


def hello() -> None:
    """赛前准备阶段：测试与裁判机连通。不会触发计时。"""
    r = requests.post(f"{JUDGE_URL}/api/hello", json={"team": TEAM}, timeout=3)
    r.raise_for_status()
    print("[hello]", r.json())


def report_single_image(task: str, result: str, sub_id: str = "", extra: str = "") -> dict:
    """单张图片上报（字段名 image）。"""
    files = {"image": ("capture.png", _make_png(), "image/png")}
    r = requests.post(
        f"{JUDGE_URL}/api/report",
        data={"team": TEAM, "task": task, "sub_id": sub_id, "result": result, "extra": extra},
        files=files,
        timeout=8,
    )
    r.raise_for_status()
    return r.json()


def report_multi_images(task: str, result: str, labels, sub_id: str = "", extra: str = "") -> dict:
    """多张图片一次上报（同一字段名 images 重复提交）。"""
    palette = [(235, 242, 252), (232, 246, 238), (252, 240, 232)]   # 每张颜色不同，便于区分
    files = []
    for i, _lab in enumerate(labels):
        files.append(("images", (f"view{i + 1}.png",
                                 _make_png(palette[i % len(palette)]), "image/png")))
    r = requests.post(
        f"{JUDGE_URL}/api/report",
        data={"team": TEAM, "task": task, "sub_id": sub_id, "result": result, "extra": extra},
        files=files or None,
        timeout=10,
    )
    r.raise_for_status()
    return r.json()


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="附件原始上传演示（无相机、无 ROS）")
    parser.add_argument("--url", default=JUDGE_URL, help="裁判地址，默认 http://127.0.0.1:8080")
    parser.add_argument("--team", default=TEAM, help="队伍名")
    args = parser.parse_args()
    JUDGE_URL, TEAM = args.url.rstrip("/"), args.team
    print(f"目标裁判机: {JUDGE_URL} | 队伍: {TEAM}\n")

    # 1) 通信自检
    hello()

    # 2) 车牌：1/2/3 号位各上报一次（单图），页面会显示 3 个可切换标签
    for i in (1, 2, 3):
        res = report_single_image(
            task="plate",
            sub_id=f"{i}号停车位",
            result=f"{i}号停车场车牌号为苏A1234{i}",
        )
        print(f"[plate {i}号位] ok, 图片数={res.get('image_count')}")

    # 3) 人群：一次上报 3 张图（多视角/多街区），页面显示缩略图网格 + 画廊翻页
    res = report_multi_images(
        task="crowd",
        sub_id="A街区",
        result="社区内共有16人，A街8人，B街8人，发现2名非社区人员在A街",
        labels=["A街-视角1", "A街-视角2", "B街-视角"],
    )
    print(f"[crowd 多图] ok, 图片数={res.get('image_count')}")

    print("\n全部上报完成，请刷新裁判大屏查看效果。")
