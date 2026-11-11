"""HTTP 合约回归：PYTHONDONTWRITEBYTECODE=1 python3 -m unittest discover -s tools/mock_judge/tests -v"""

import csv
import io
import json
import sqlite3
import stat
import tempfile
import unittest
import zipfile
from concurrent.futures import ThreadPoolExecutor
from datetime import datetime
from pathlib import Path
from unittest.mock import patch

from PIL import Image

from tools.mock_judge.app import create_app


TASKS = {"crowd", "sign", "trash", "fire", "plate", "temperature", "meter", "ebike"}


def image_bytes(fmt="PNG", size=(8, 6), color=(45, 100, 200)):
    buffer = io.BytesIO()
    Image.new("RGB", size, color).save(buffer, format=fmt)
    return buffer.getvalue()


def timestamp(value):
    """响应允许 UTC ISO8601；注入时钟始终使用 Unix seconds。"""
    if isinstance(value, (float, int)):
        return value
    return datetime.fromisoformat(value.replace("Z", "+00:00")).timestamp()


class Clock:
    def __init__(self):
        self.now = 1_700_000_000.0

    def __call__(self):
        return self.now


class JudgeAPI(unittest.TestCase):
    def setUp(self):
        self.temp = tempfile.TemporaryDirectory(prefix="mock-judge-test-")
        self.addCleanup(self.temp.cleanup)
        self.clock = Clock()
        self.config = {
            "TESTING": True, "DATA_DIR": self.temp.name, "ADMIN_TOKEN": "test-token",
            "MAX_CONTENT_LENGTH": 32 * 1024 * 1024, "MAX_IMAGES": 16,
            "MAX_IMAGE_BYTES": 8 * 1024 * 1024, "MAX_IMAGE_PIXELS": 20_000_000,
        }
        self.app = create_app(self.config, clock=self.clock)
        self.client = self.app.test_client()
        self.auth = {"Authorization": "Bearer test-token"}

    def assert_ok(self, response, status=200):
        self.assertEqual(response.status_code, status, response.get_data(as_text=True))
        data = response.get_json()
        self.assertIsInstance(data, dict)
        self.assertIs(data.get("ok"), True)
        return data

    def assert_error(self, response, status):
        self.assertEqual(response.status_code, status, response.get_data(as_text=True))
        self.assertTrue(response.is_json)
        self.assertIs(response.get_json().get("ok"), False)

    def report(self, **overrides):
        data = {"team": "测试队", "task": "plate", "result": "苏A12345"}
        data.update(overrides)
        return self.client.post("/api/report", data=data)

    def reports(self, **query):
        return self.assert_ok(self.client.get("/api/reports", query_string=query))

    def state(self):
        return self.assert_ok(self.client.get("/api/state"))

    def admin(self, action, payload=None):
        return self.client.post("/api/admin/" + action,
                                json={} if payload is None else payload, headers=self.auth)

    def multipart(self, files, **overrides):
        """直接构造 wire multipart，保留 image/images 交错顺序（MultiDict 会分组）。"""
        boundary = "judge-test-unique-boundary"
        data = {"team": "测试队", "task": "crowd", "result": "模拟人群"}
        data.update(overrides)
        parts = []
        for name, value in data.items():
            parts.append(("--%s\r\nContent-Disposition: form-data; name=\"%s\"\r\n\r\n%s\r\n" %
                          (boundary, name, value)).encode())
        for field, filename, content, mime in files:
            parts.append(("--%s\r\nContent-Disposition: form-data; name=\"%s\"; filename=\"%s\"\r\n"
                          "Content-Type: %s\r\n\r\n" %
                          (boundary, field, filename, mime)).encode() + content + b"\r\n")
        parts.append(("--%s--\r\n" % boundary).encode())
        return self.client.post("/api/report", data=b"".join(parts),
                                content_type="multipart/form-data; boundary=" + boundary)

    def test_initial_state_schema_and_task_list(self):
        state = self.state()
        self.assertTrue({"schema_version", "server_time", "session", "sessions", "teams", "tasks"} <= state.keys())
        session = state["session"]
        self.assertTrue({"id", "created_at", "status", "locked_team", "started_at", "finished_at", "elapsed_seconds"} <= session.keys())
        self.assertEqual(session["status"], "ready")
        self.assertIsNone(session["started_at"])
        self.assertEqual(session["elapsed_seconds"], 0)
        self.assertEqual(timestamp(state["server_time"]), self.clock.now)
        self.assertEqual({task["id"] for task in state["tasks"]}, TASKS)
        self.assertTrue(all(task["label"] for task in state["tasks"]))
        self.assertEqual(state["sessions"][0]["id"], session["id"])
        self.assertIn("judge_store", self.app.extensions)

    def test_hello_json_and_report_do_not_start_timer(self):
        hello = self.assert_ok(self.client.post("/api/hello", json={"team": "测试队"}))
        self.assertEqual(hello["team"], "测试队")
        self.assertEqual(hello["message"], "通信正常")
        self.clock.now += 5
        self.assert_ok(self.report())
        state = self.state()
        self.assertEqual(state["session"]["status"], "ready")
        self.assertIsNone(state["session"]["started_at"])
        team = next(t for t in state["teams"] if t["team"] == "测试队")
        self.assertEqual(team["report_count"], 1)
        self.assertEqual(timestamp(team["last_hello_at"]), self.clock.now - 5)
        self.assertEqual(timestamp(team["last_report_at"]), self.clock.now)

    def test_all_eight_tasks_and_unregistered_team(self):
        for task in sorted(TASKS):
            with self.subTest(task=task):
                self.assertEqual(self.assert_ok(self.report(task=task))["image_count"], 0)
        self.assertEqual(self.reports()["total"], 8)

    def test_report_shape_preserves_raw_text_and_optional_empty_fields(self):
        text = '<script>alert("x")</script>\n=SUM(A1:A2)'
        response = self.assert_ok(self.report(result=text, extra="<img src=x onerror=alert(1)>", sub_id=""))
        row = self.reports()["reports"][0]
        self.assertTrue({"id", "session_id", "received_at", "team", "task", "result", "sub_id", "extra", "image_count", "images"} <= row.keys())
        self.assertEqual(row["id"], response["report_id"])
        self.assertEqual(row["result"], text)
        self.assertEqual(row["extra"], "<img src=x onerror=alert(1)>")
        self.assertEqual(row["sub_id"], "")
        self.assertEqual(row["images"], [])

    def test_single_png_and_jpeg_content_not_extension_or_mime(self):
        for fmt, expected in (("PNG", "image/png"), ("JPEG", "image/jpeg")):
            with self.subTest(fmt=fmt):
                content = image_bytes(fmt)
                response = self.multipart([("image", "misleading.txt", content, "application/octet-stream")])
                self.assertEqual(self.assert_ok(response)["image_count"], 1)
                image = self.reports()["reports"][0]["images"][0]
                self.assertTrue({"id", "url", "original_name", "media_type", "width", "height", "position"} <= image.keys())
                self.assertEqual(image["media_type"], expected)
                self.assertEqual((image["width"], image["height"]), (8, 6))
                with self.client.get(image["url"]) as fetched:
                    self.assertEqual(fetched.status_code, 200)
                    self.assertEqual(fetched.mimetype, expected)
                    self.assertEqual(fetched.data, content)

    def test_mixed_image_images_preserve_wire_order(self):
        names = ["one.png", "two.png", "three.png", "four.png"]
        files = [(field, name, image_bytes(color=(index * 30, 20, 70)), "image/png")
                 for index, (field, name) in enumerate(zip(("image", "images", "image", "images"), names))]
        self.assertEqual(self.assert_ok(self.multipart(files))["image_count"], 4)
        images = self.reports()["reports"][0]["images"]
        self.assertEqual([image["original_name"] for image in images], names)
        self.assertEqual([image["position"] for image in images], list(range(4)))

    def test_filename_path_traversal_cannot_escape_data_directory(self):
        response = self.multipart([("image", "../../escape-test.png", image_bytes(), "image/png")])
        self.assert_ok(response)
        image = self.reports()["reports"][0]["images"][0]
        self.assertNotIn("..", image["url"])
        files = [p for p in Path(self.temp.name).rglob("*") if p.is_file()]
        self.assertFalse(any(p.name == "escape-test.png" for p in files))
        with self.client.get(image["url"]) as fetched:
            self.assertEqual(fetched.status_code, 200)
        self.assertEqual(self.client.get("/api/images/not-an-image-id").status_code, 404)

    def test_required_fields_whitespace_and_unknown_task(self):
        for field in ("team", "task", "result"):
            for bad in (None, "", " \t\n"):
                with self.subTest(field=field, bad=bad):
                    data = {"team": "测试队", "task": "plate", "result": "result"}
                    if bad is None:
                        del data[field]
                    else:
                        data[field] = bad
                    self.assert_error(self.client.post("/api/report", data=data), 400)
        self.assert_error(self.report(task="unknown"), 400)
        self.assertEqual(self.reports()["total"], 0)

    def test_hello_rejects_invalid_json_and_team(self):
        for value in ({}, {"team": " "}, {"team": None}, {"team": 123}, {"team": []}, {"team": "x" * 129}, []):
            with self.subTest(value=value):
                self.assert_error(self.client.post("/api/hello", json=value), 400)
        self.assert_error(self.client.post("/api/hello", data="{", content_type="application/json"), 400)

    def test_string_length_boundaries(self):
        self.assert_ok(self.report(team="队" * 128, sub_id="位" * 128,
                                   result="文" * 10000, extra="注" * 10000))
        for field, length in (("team", 129), ("sub_id", 129), ("result", 10001), ("extra", 10001)):
            with self.subTest(field=field):
                self.assert_error(self.report(**{field: "文" * length}), 400)
        self.assertEqual(self.reports()["total"], 1)

    def test_invalid_and_damaged_media_rejected_without_partial_report(self):
        for content, mime in ((b"not a PNG", "image/png"), (image_bytes("GIF"), "image/gif"),
                              (image_bytes()[:35], "image/png"),
                              (image_bytes("JPEG")[:-20], "image/jpeg")):
            with self.subTest(mime=mime, size=len(content)):
                files = [("image", "valid.png", image_bytes(), "image/png"),
                         ("images", "invalid.png", content, mime)]
                self.assert_error(self.multipart(files), 415)
                self.assertEqual(self.reports()["total"], 0)
        self.assertFalse(list(Path(self.temp.name).rglob("*.png")))

    def test_image_count_limit(self):
        png = image_bytes()
        files = [("images", "p%d.png" % i, png, "image/png") for i in range(16)]
        self.assertEqual(self.assert_ok(self.multipart(files))["image_count"], 16)
        self.assert_error(self.multipart(files + [("images", "extra.png", png, "image/png")]), 413)
        self.assertEqual(self.reports()["total"], 1)

    def test_configurable_image_bytes_limit(self):
        self.app.config["MAX_IMAGE_BYTES"] = len(image_bytes()) - 1
        self.assert_error(self.multipart([("image", "too-large.png", image_bytes(), "image/png")]), 413)
        self.assertEqual(self.reports()["total"], 0)

    def test_configurable_image_pixels_limit(self):
        self.app.config["MAX_IMAGE_PIXELS"] = 47
        self.assert_error(self.multipart([("image", "too-many-pixels.png", image_bytes(), "image/png")]), 413)
        self.assertEqual(self.reports()["total"], 0)

    def test_request_size_limit_returns_json(self):
        self.app.config["MAX_CONTENT_LENGTH"] = 1024
        self.assert_error(self.report(extra="x" * 2048), 413)
        self.assertEqual(self.reports()["total"], 0)

    def test_reports_filters_pagination_newest_first(self):
        expected = []
        for i in range(5):
            self.clock.now += 1
            expected.append(self.assert_ok(self.report(team="A" if i % 2 else "B",
                                                     task="plate" if i < 3 else "crowd",
                                                     sub_id="" if i < 2 else "slot"))["report_id"])
        all_reports = self.reports()
        self.assertEqual([row["id"] for row in all_reports["reports"]], expected[::-1])
        page = self.reports(limit=2, offset=1)
        self.assertEqual((page["total"], page["limit"], page["offset"]), (5, 2, 1))
        self.assertEqual([row["id"] for row in page["reports"]], expected[::-1][1:3])
        for query, count in (({"team": "A"}, 2), ({"task": "crowd"}, 2),
                             ({"sub_id": ""}, 2), ({"sub_id": "slot"}, 3),
                             ({"team": "B", "task": "plate", "sub_id": ""}, 1)):
            with self.subTest(query=query):
                self.assertEqual(self.reports(**query)["total"], count)
        self.assertEqual(self.reports(offset=99)["reports"], [])
        self.assertEqual(self.reports(team="not-present")["total"], 0)

    def test_invalid_pagination_returns_400(self):
        for query in ({"limit": "abc"}, {"offset": "abc"}, {"limit": -1}, {"offset": -1}, {"limit": 0}):
            with self.subTest(query=query):
                self.assert_error(self.client.get("/api/reports", query_string=query), 400)

    def test_admin_endpoints_require_bearer(self):
        for path in ("team-lock", "session/start", "session/finish", "session/new"):
            for headers in ({}, {"Authorization": "Bearer wrong"}):
                with self.subTest(path=path, headers=headers):
                    response = self.client.post("/api/admin/" + path, json={}, headers=headers)
                    self.assertIn(response.status_code, (401, 403))
                    self.assertTrue(response.is_json)
        self.assertIn(self.client.get("/api/admin/export").status_code, (401, 403))
        self.assertEqual(self.state()["session"]["status"], "ready")

    def test_generated_admin_token_is_private_persistent_and_not_in_state(self):
        config = dict(self.config)
        del config["ADMIN_TOKEN"]
        first = create_app(config, clock=self.clock)
        token = first.config["ADMIN_TOKEN"]
        self.assertIsInstance(token, str)
        self.assertGreaterEqual(len(token), 24)
        token_files = [path for path in Path(self.temp.name).iterdir()
                       if path.is_file() and "token" in path.name]
        self.assertEqual(len(token_files), 1)
        self.assertEqual(stat.S_IMODE(token_files[0].stat().st_mode), 0o600)
        second = create_app(config, clock=self.clock)
        self.assertEqual(second.config["ADMIN_TOKEN"], token)
        for app in (first, second):
            with app.test_client() as client:
                state = client.get("/api/state")
                self.assertEqual(state.status_code, 200)
                self.assertNotIn(token, state.get_data(as_text=True))
                response = client.post("/api/admin/team-lock", json={"team": None},
                                       headers={"Authorization": "Bearer " + token})
                self.assert_ok(response)

    def test_unknown_database_schema_refuses_startup(self):
        store = self.app.extensions["judge_store"]
        with sqlite3.connect(str(store.db_path)) as db:
            db.execute("PRAGMA user_version=99")
        with self.assertRaises(RuntimeError):
            create_app(self.config, clock=self.clock)

    def test_invalid_session_transitions_return_conflict(self):
        self.assert_error(self.admin("session/finish"), 409)
        self.assertEqual(self.state()["session"]["status"], "ready")
        started = self.assert_ok(self.admin("session/start"))["session"]
        self.clock.now += 5
        self.assert_error(self.admin("session/start"), 409)
        self.assert_error(self.admin("session/new"), 409)
        self.assertEqual(self.state()["session"]["id"], started["id"])
        self.assertEqual(self.state()["session"]["started_at"], started["started_at"])
        self.assert_ok(self.admin("session/finish"))
        self.assert_error(self.admin("session/start"), 409)
        self.assert_error(self.admin("session/finish"), 409)

    def test_admin_cross_origin_rejected(self):
        headers = dict(self.auth, Origin="http://untrusted.invalid")
        response = self.client.post("/api/admin/session/start", json={}, headers=headers)
        self.assert_error(response, 403)
        self.assertEqual(self.state()["session"]["status"], "ready")

    def test_team_lock_unlock_and_validation(self):
        self.assert_ok(self.admin("team-lock", {"team": "锁定队"}))
        self.assertEqual(self.state()["session"]["locked_team"], "锁定队")
        self.assert_error(self.client.post("/api/hello", json={"team": "其他队"}), 403)
        self.assert_error(self.report(team="其他队"), 403)
        self.assert_ok(self.client.post("/api/hello", json={"team": "锁定队"}))
        self.assert_ok(self.report(team="锁定队"))
        for payload in ({}, {"team": " "}, {"team": 123}, {"team": "x" * 129}):
            with self.subTest(payload=payload):
                self.assert_error(self.admin("team-lock", payload), 400)
        self.assert_ok(self.admin("team-lock", {"team": None}))
        self.assert_ok(self.report(team="其他队"))

    def test_session_manual_timing_finish_and_new_preserve_history(self):
        initial_id = self.state()["session"]["id"]
        ready_report = self.assert_ok(self.report())["report_id"]
        start = self.assert_ok(self.admin("session/start"))["session"]
        self.assertEqual(start["status"], "running")
        self.assertEqual(timestamp(start["started_at"]), self.clock.now)
        self.clock.now += 12.5
        self.assertAlmostEqual(self.state()["session"]["elapsed_seconds"], 12.5)
        self.assert_ok(self.report(result="运行期间"))
        finished = self.assert_ok(self.admin("session/finish"))["session"]
        self.assertEqual(finished["status"], "finished")
        self.assertAlmostEqual(finished["elapsed_seconds"], 12.5)
        self.clock.now += 100
        self.assertAlmostEqual(self.state()["session"]["elapsed_seconds"], 12.5)
        self.assert_error(self.report(), 409)
        new = self.assert_ok(self.admin("session/new"))["session"]
        self.assertNotEqual(new["id"], initial_id)
        self.assertEqual(new["status"], "ready")
        self.assertEqual(new["elapsed_seconds"], 0)
        self.assert_ok(self.report(result="新场次"))
        old = self.reports(session_id=initial_id)
        self.assertEqual(old["total"], 2)
        self.assertIn(ready_report, [row["id"] for row in old["reports"]])
        self.assertEqual(self.reports(session_id=new["id"])["total"], 1)
        self.assertEqual(len(self.state()["sessions"]), 2)

    def test_restart_preserves_timer_lock_reports_and_image(self):
        self.assert_ok(self.admin("team-lock", {"team": "测试队"}))
        self.assert_ok(self.admin("session/start"))
        self.assert_ok(self.multipart([("image", "persist.png", image_bytes(), "image/png")]))
        before = self.state()
        image_url = self.reports()["reports"][0]["images"][0]["url"]
        self.clock.now += 123.25
        self.app = create_app(self.config, clock=self.clock)
        self.client = self.app.test_client()
        after = self.state()
        self.assertEqual(after["session"]["id"], before["session"]["id"])
        self.assertEqual(after["session"]["locked_team"], "测试队")
        self.assertEqual(after["session"]["status"], "running")
        self.assertAlmostEqual(after["session"]["elapsed_seconds"], 123.25)
        self.assertEqual(self.reports()["total"], 1)
        with self.client.get(image_url) as fetched:
            self.assertEqual(fetched.data, image_bytes())

    def test_export_contains_manifest_csv_images_and_safe_csv(self):
        self.assert_ok(self.multipart([("image", "../../capture.png", image_bytes(), "image/png")],
                                     team="=2+2", result="+SUM(1,2)", sub_id="@slot", extra="-formula"))
        session_id = self.state()["session"]["id"]
        response = self.client.get("/api/admin/export", query_string={"session_id": session_id}, headers=self.auth)
        self.assertEqual(response.status_code, 200)
        with zipfile.ZipFile(io.BytesIO(response.data)) as archive:
            names = archive.namelist()
            self.assertIn("manifest.json", names)
            self.assertIn("reports.csv", names)
            self.assertFalse(any(name.startswith("/") or ".." in Path(name).parts for name in names))
            images = [name for name in names if name.startswith("images/") and not name.endswith("/")]
            self.assertEqual(len(images), 1)
            self.assertEqual(archive.read(images[0]), image_bytes())
            manifest = json.loads(archive.read("manifest.json"))
            self.assertIn("schema_version", manifest)
            self.assertIn("=2+2", json.dumps(manifest, ensure_ascii=False))
            rows = list(csv.DictReader(io.StringIO(archive.read("reports.csv").decode("utf-8-sig"))))
            self.assertEqual(len(rows), 1)
            for field in ("team", "result", "sub_id", "extra"):
                self.assertFalse(rows[0][field].startswith(("=", "+", "-", "@")), (field, rows[0][field]))

    def test_export_is_session_scoped(self):
        old_session = self.state()["session"]["id"]
        self.assert_ok(self.report(result="old-only-marker"))
        self.assert_ok(self.admin("session/new"))
        self.assert_ok(self.report(result="new-only-marker"))
        response = self.client.get("/api/admin/export", query_string={"session_id": old_session}, headers=self.auth)
        self.assertEqual(response.status_code, 200)
        with zipfile.ZipFile(io.BytesIO(response.data)) as archive:
            csv_text = archive.read("reports.csv").decode("utf-8-sig")
            self.assertIn("old-only-marker", csv_text)
            self.assertNotIn("new-only-marker", csv_text)

    def test_database_connection_failure_returns_json_and_recovers(self):
        with patch("tools.mock_judge.store.sqlite3.connect",
                   side_effect=sqlite3.OperationalError("simulated database unavailable")):
            with self.assertLogs(self.app.logger, level="ERROR"):
                self.assert_error(self.report(), 500)
        self.assertEqual(self.reports()["total"], 0)
        self.assertEqual(list(self.app.extensions["judge_store"].staging_dir.iterdir()), [])
        self.assert_ok(self.report())

    def test_storage_move_failure_rolls_back_files_and_database(self):
        from tools.mock_judge import store as store_module
        original_replace = store_module.os.replace
        calls = []

        def fail_second_move(source, destination):
            calls.append(destination)
            if len(calls) == 2:
                raise OSError("simulated disk failure")
            return original_replace(source, destination)

        files = [("images", "%d.png" % i, image_bytes(), "image/png") for i in range(2)]
        with patch("tools.mock_judge.store.os.replace", side_effect=fail_second_move):
            with self.assertLogs(self.app.logger, level="ERROR"):
                self.assert_error(self.multipart(files), 500)
        self.assertEqual(len(calls), 2)
        self.assertEqual(self.reports()["total"], 0)
        store = self.app.extensions["judge_store"]
        self.assertEqual(list(store.images_dir.iterdir()), [])
        self.assertEqual(list(store.staging_dir.iterdir()), [])
        self.assert_ok(self.multipart(files))
        self.assertEqual(self.reports()["total"], 1)

    def test_concurrent_uploads_append_unique_records(self):
        def upload(index):
            with self.app.test_client() as client:
                response = client.post("/api/report", data={"team": "并发队", "task": "plate",
                                                            "result": str(index), "sub_id": "同点位"})
                return response.status_code, response.get_json()
        with ThreadPoolExecutor(max_workers=6) as executor:
            results = list(executor.map(upload, range(18)))
        self.assertTrue(all(status == 200 for status, data in results), results)
        ids = [data["report_id"] for status, data in results]
        self.assertEqual(len(set(ids)), 18)
        reports = self.reports(team="并发队", sub_id="同点位", limit=100)
        self.assertEqual(reports["total"], 18)
        self.assertEqual({row["result"] for row in reports["reports"]}, {str(i) for i in range(18)})


if __name__ == "__main__":
    unittest.main()
