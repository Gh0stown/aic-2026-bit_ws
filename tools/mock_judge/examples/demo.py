"""八任务纯模拟上传：文字、单图、多图；不连接模型、相机或 ROS。

运行：PYTHONDONTWRITEBYTECODE=1 python3 tools/mock_judge/examples/demo.py \
    --url http://127.0.0.1:8080 --team 联调队

只进行 hello/report，不启动计时、不改变管理状态。图片为附件中的标准库示意 PNG。
"""

import argparse

import requests

try:
    from .attachment_demo import _make_png
except ImportError:  # 支持直接执行脚本。
    from attachment_demo import _make_png


TASKS = (
    ("crowd", "社区共16人，发现2名非社区人员", "A街区", 3),
    ("sign", "发现禁止通行标志", "路口1", 1),
    ("trash", "垃圾桶周围无散落垃圾", "垃圾桶1", 0),
    ("fire", "未发现火情", "楼栋1", 1),
    ("plate", "车牌号为苏A12345", "1号停车位", 1),
    ("temperature", "温度为26.5℃", "测温点1", 0),
    ("meter", "仪表示数为0.42 MPa", "仪表1", 1),
    ("ebike", "发现1辆电动车违规停放", "楼道1", 2),
)


def run(url, team):
    """以真实 HTTP 发送固定模拟数据，返回每次成功上报的 JSON。"""
    url = url.rstrip("/")
    with requests.Session() as client:
        response = client.post(url + "/api/hello", json={"team": team}, timeout=5)
        response.raise_for_status()
        print("[hello]", response.json())
        uploaded = []
        for task, result, sub_id, count in TASKS:
            data = {"team": team, "task": task, "result": result,
                    "sub_id": sub_id, "extra": "模拟数据，仅用于接口联调，非真实识别结果"}
            files = [
                ("image" if count == 1 else "images",
                 ("%s_%d.png" % (task, index + 1),
                  _make_png(color=(230 + index * 5, 240, 250)), "image/png"))
                for index in range(count)
            ]
            response = client.post(url + "/api/report", data=data,
                                   files=files or None, timeout=15)
            response.raise_for_status()
            payload = response.json()
            if not payload.get("ok"):
                raise RuntimeError("上报失败: %r" % payload)
            uploaded.append(payload)
            print("[%s] report_id=%s image_count=%s" %
                  (task, payload.get("report_id"), payload.get("image_count")))
    return uploaded


def main():
    parser = argparse.ArgumentParser(description="八任务无硬件、无模型上传演示")
    parser.add_argument("--url", default="http://127.0.0.1:8080", help="裁判服务地址")
    parser.add_argument("--team", default="模拟联调队", help="队伍名")
    args = parser.parse_args()
    run(args.url, args.team)
    print("完成：8条模拟记录、9张示意图片；hello/report 不会自动开始计时。")


if __name__ == "__main__":
    main()
