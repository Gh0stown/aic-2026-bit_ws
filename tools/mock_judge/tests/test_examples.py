"""示例客户端回归；HTTP 在此 mock，真实 HTTP 验收由服务集成检查执行。"""

import contextlib
import io
import runpy
import sys
import unittest
from pathlib import Path
from unittest.mock import patch

from PIL import Image

from tools.mock_judge.examples import attachment_demo, demo


class Response:
    def __init__(self, count):
        self.count = count

    def raise_for_status(self):
        pass

    def json(self):
        return {"ok": True, "report_id": "example-id", "image_count": self.count}


class ExampleTests(unittest.TestCase):
    def test_attachment_png_is_valid_without_camera(self):
        data = attachment_demo._make_png()
        with Image.open(io.BytesIO(data)) as image:
            self.assertEqual(image.format, "PNG")
            self.assertEqual(image.size, (640, 360))
            image.verify()

    def test_attachment_cli_sends_original_four_reports_six_images(self):
        requests = []

        def post(url, **kwargs):
            requests.append((url, kwargs))
            return Response(len(kwargs.get("files") or []))

        # run_path 不复用已导入模块，直接覆盖 argparse 的真实脚本入口。
        source = str(Path(attachment_demo.__file__).resolve())
        with patch.object(sys, "argv", [source, "--url", "http://127.0.0.1:19080/", "--team", "CLI队"]):
            with patch("requests.post", side_effect=post), contextlib.redirect_stdout(io.StringIO()):
                runpy.run_path(source, run_name="__main__")
        self.assertEqual(len(requests), 5)
        self.assertEqual(requests[0][0], "http://127.0.0.1:19080/api/hello")
        self.assertEqual(requests[0][1]["json"], {"team": "CLI队"})
        reports = requests[1:]
        self.assertTrue(all(url == "http://127.0.0.1:19080/api/report" for url, kwargs in reports))
        self.assertTrue(all(kwargs["data"]["team"] == "CLI队" for url, kwargs in reports))
        self.assertEqual([kwargs["data"]["task"] for url, kwargs in reports], ["plate"] * 3 + ["crowd"])
        self.assertEqual(sum(len(kwargs["files"]) for url, kwargs in reports), 6)
        self.assertEqual([kwargs["data"]["sub_id"] for url, kwargs in reports[:3]],
                         ["1号停车位", "2号停车位", "3号停车位"])

    def test_eight_task_demo_has_text_single_multiple_images(self):
        calls = []

        class Client:
            def __enter__(self):
                return self

            def __exit__(self, *args):
                return False

            def post(self, url, **kwargs):
                calls.append((url, kwargs))
                return Response(len(kwargs.get("files") or []))

        with patch.object(demo.requests, "Session", Client), contextlib.redirect_stdout(io.StringIO()):
            responses = demo.run("http://127.0.0.1:19080/", "八任务队")
        self.assertEqual(len(responses), 8)
        self.assertEqual(len(calls), 9)
        reports = calls[1:]
        self.assertEqual({kwargs["data"]["task"] for url, kwargs in reports},
                         {"crowd", "sign", "trash", "fire", "plate", "temperature", "meter", "ebike"})
        counts = [len(kwargs.get("files") or []) for url, kwargs in reports]
        self.assertEqual(sum(counts), 9)
        self.assertTrue({0, 1, 2, 3} <= set(counts))
        self.assertTrue(all(url == "http://127.0.0.1:19080/api/report" for url, kwargs in reports))
        self.assertTrue(all(kwargs["data"]["team"] == "八任务队" for url, kwargs in reports))
        for url, kwargs in reports:
            self.assertIn("模拟数据", kwargs["data"]["extra"])
            for field, (name, data, mime) in kwargs.get("files") or []:
                self.assertIn(field, ("image", "images"))
                self.assertEqual(mime, "image/png")
                with Image.open(io.BytesIO(data)) as image:
                    image.verify()


if __name__ == "__main__":
    unittest.main()
