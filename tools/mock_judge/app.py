"""HTTP contract for the local-network mock judge (no ROS dependencies)."""
import csv
import hmac
import io
import json
import os
import secrets
import sqlite3
import tempfile
import warnings
import zipfile
from functools import wraps
from pathlib import Path

from flask import Flask, Request, jsonify, render_template, request, send_file
from PIL import Image, UnidentifiedImageError
from werkzeug.datastructures import ImmutableOrderedMultiDict
from werkzeug.exceptions import HTTPException

from .store import JudgeError, Store, TASKS


class OrderedRequest(Request):
    # Werkzeug 3.0's ordered form storage preserves interleaved image/images.
    parameter_storage_class = ImmutableOrderedMultiDict


def _text(value, name, max_length, required=False):
    if not isinstance(value, str):
        raise JudgeError("%s 必须是字符串" % name)
    if required and not value.strip():
        raise JudgeError("%s 不能为空" % name)
    if len(value) > max_length:
        raise JudgeError("%s 超过 %s 字符" % (name, max_length))
    if "\x00" in value:
        raise JudgeError("%s 不能包含空字符" % name)
    return value


def _json_object():
    value = request.get_json(silent=True)
    if not isinstance(value, dict):
        raise JudgeError("请求必须包含 JSON 对象")
    return value


def _token(root, configured):
    if configured is not None:
        if not isinstance(configured, str) or not configured.strip():
            raise ValueError("ADMIN_TOKEN 不能为空")
        return configured
    path = root / "admin_token"
    try:
        fd = os.open(str(path), os.O_WRONLY | os.O_CREAT | os.O_EXCL, 0o600)
    except FileExistsError:
        token = path.read_text(encoding="utf-8").strip()
        if not token:
            raise RuntimeError("管理令牌文件为空：%s" % path)
        return token
    with os.fdopen(fd, "w", encoding="utf-8") as handle:
        token = secrets.token_urlsafe(32)
        handle.write(token + "\n")
    return token


def create_app(config=None, clock=None):
    app = Flask(__name__)
    app.request_class = OrderedRequest
    app.config.from_mapping(
        DATA_DIR=str(Path.home() / ".local/share/bit_ws/mock_judge"),
        ADMIN_TOKEN=None, MAX_CONTENT_LENGTH=32 * 1024 * 1024,
        MAX_IMAGES=16, MAX_IMAGE_BYTES=8 * 1024 * 1024,
        MAX_IMAGE_PIXELS=20_000_000, MAX_FORM_MEMORY_SIZE=1024 * 1024,
    )
    if config:
        app.config.update(config)
    for name in ("MAX_CONTENT_LENGTH", "MAX_IMAGES", "MAX_IMAGE_BYTES", "MAX_IMAGE_PIXELS"):
        if not isinstance(app.config[name], int) or app.config[name] <= 0:
            raise ValueError("%s 必须为正整数" % name)
    store = Store(app.config["DATA_DIR"], clock)
    app.extensions["judge_store"] = store
    app.config["ADMIN_TOKEN"] = _token(store.root, app.config["ADMIN_TOKEN"])
    task_ids = {key for key, _ in TASKS}

    def admin(function):
        @wraps(function)
        def wrapped(*args, **kwargs):
            origin = request.headers.get("Origin")
            if origin is not None and origin.rstrip("/") != request.host_url.rstrip("/"):
                raise JudgeError("不允许跨域管理操作", 403)
            authorization = request.headers.get("Authorization", "")
            expected = "Bearer " + app.config["ADMIN_TOKEN"]
            if not hmac.compare_digest(authorization.encode("utf-8"), expected.encode("utf-8")):
                raise JudgeError("需要正确的管理令牌", 401)
            return function(*args, **kwargs)
        return wrapped

    @app.errorhandler(JudgeError)
    def judge_error(error):
        return jsonify(ok=False, error=str(error)), error.status

    @app.errorhandler(HTTPException)
    def http_error(error):
        messages = {400: "请求格式错误", 404: "接口不存在", 405: "请求方法不支持",
                    413: "请求超过允许大小", 415: "请求媒体类型不支持", 500: "服务内部错误"}
        return jsonify(ok=False, error=messages.get(error.code, error.description)), error.code

    @app.errorhandler(OSError)
    @app.errorhandler(sqlite3.Error)
    def storage_error(error):
        app.logger.exception("裁判数据存储失败")
        return jsonify(ok=False, error="数据存储失败，请检查裁判机日志及磁盘空间"), 500

    @app.after_request
    def headers(response):
        response.headers["X-Content-Type-Options"] = "nosniff"
        response.headers["Referrer-Policy"] = "same-origin"
        response.headers["Content-Security-Policy"] = (
            "default-src 'self'; script-src 'self'; style-src 'self'; "
            "img-src 'self' blob:; connect-src 'self'; object-src 'none'; "
            "frame-ancestors 'none'; base-uri 'none'; form-action 'self'"
        )
        if request.path.startswith("/api/"):
            response.headers["Cache-Control"] = "no-store"
        return response

    @app.get("/")
    def index():
        return render_template("index.html")

    @app.post("/api/hello")
    def hello():
        body = _json_object()
        team = _text(body.get("team"), "team", 128, True)
        return jsonify(store.hello(team))

    def stage_image(upload, staging, position):
        data = upload.stream.read(app.config["MAX_IMAGE_BYTES"] + 1)
        if len(data) > app.config["MAX_IMAGE_BYTES"]:
            raise JudgeError("单张图片超过允许大小", 413)
        try:
            with warnings.catch_warnings():
                warnings.simplefilter("error", Image.DecompressionBombWarning)
                with Image.open(io.BytesIO(data)) as image:
                    if image.format not in ("JPEG", "PNG"):
                        raise JudgeError("仅支持 JPEG/PNG 图片", 415)
                    width, height = image.size
                    if width * height > app.config["MAX_IMAGE_PIXELS"]:
                        raise JudgeError("图片像素数超过允许上限", 413)
                    kind = image.format
                    image.verify()
                # verify() alone does not decode JPEG pixels; reject truncated payloads.
                with Image.open(io.BytesIO(data)) as decoded:
                    decoded.load()
        except (Image.DecompressionBombError, Image.DecompressionBombWarning):
            raise JudgeError("图片像素数超过安全上限", 413)
        except (UnidentifiedImageError, OSError, ValueError, SyntaxError):
            raise JudgeError("图片损坏或不是有效的 JPEG/PNG", 415)
        extension, media_type = (".jpg", "image/jpeg") if kind == "JPEG" else (".png", "image/png")
        path = Path(staging) / (str(position) + extension)
        path.write_bytes(data)
        # Client names are metadata only, never paths or response headers.
        return dict(path=path, extension=extension, media_type=media_type,
                    width=width, height=height, original_name=(upload.filename or "")[:512])

    @app.post("/api/report")
    def report():
        if request.mimetype not in ("application/x-www-form-urlencoded", "multipart/form-data"):
            raise JudgeError("上报必须使用表单编码或 multipart/form-data", 415)
        form = request.form
        payload = {}
        for name, maximum, required in (("team", 128, True), ("task", 32, True),
                                         ("result", 10000, True), ("sub_id", 128, False),
                                         ("extra", 10000, False)):
            values = form.getlist(name)
            if len(values) > 1:
                raise JudgeError("字段 %s 不能重复" % name)
            payload[name] = _text(values[0] if values else "", name, maximum, required)
        if payload["task"] not in task_ids:
            raise JudgeError("未知 task，请使用规定的八类任务 ID")
        uploads = []
        for name, upload in request.files.items(multi=True):
            if name not in ("image", "images"):
                raise JudgeError("图片字段必须使用 image 或 images")
            uploads.append(upload)
        if len(uploads) > app.config["MAX_IMAGES"]:
            raise JudgeError("图片数量超过允许上限", 413)
        with tempfile.TemporaryDirectory(prefix="report-", dir=str(store.staging_dir)) as staging:
            images = [stage_image(upload, staging, position) for position, upload in enumerate(uploads)]
            return jsonify(store.report(payload, images))

    @app.get("/api/state")
    def state():
        return jsonify(store.state(request.args.get("session_id")))

    @app.get("/api/reports")
    def reports():
        try:
            limit = int(request.args.get("limit", "100"))
            offset = int(request.args.get("offset", "0"))
        except ValueError:
            raise JudgeError("limit/offset 必须为整数")
        if not 1 <= limit <= 200 or offset < 0:
            raise JudgeError("limit 应为1到200，offset不得为负数")
        filters = {name: request.args[name] for name in ("team", "task", "sub_id") if name in request.args}
        if "task" in filters and filters["task"] not in task_ids:
            raise JudgeError("未知 task")
        return jsonify(store.reports(request.args.get("session_id"), filters, limit, offset))

    @app.get("/api/images/<image_id>")
    def image(image_id):
        path, media_type = store.image(image_id)
        return send_file(str(path), mimetype=media_type, conditional=True)

    @app.post("/api/admin/team-lock")
    @admin
    def team_lock():
        body = _json_object()
        if "team" not in body:
            raise JudgeError("缺少 team；解除锁定请传 null")
        team = body["team"]
        if team is not None:
            team = _text(team, "team", 128, True)
        return jsonify(ok=True, session=store.manage("team-lock", team))

    @app.post("/api/admin/session/<action>")
    @admin
    def session_action(action):
        _json_object()
        if action not in ("start", "finish", "new"):
            raise JudgeError("未知场次操作", 404)
        return jsonify(ok=True, session=store.manage(action))

    @app.get("/api/admin/export")
    @admin
    def export():
        snapshot = store.export_snapshot(request.args.get("session_id"))
        # Spill large exports to disk rather than holding every image in RAM.
        output = tempfile.SpooledTemporaryFile(max_size=8 * 1024 * 1024, mode="w+b")
        try:
            with zipfile.ZipFile(output, "w", compression=zipfile.ZIP_DEFLATED) as archive:
                for item in snapshot["reports"]:
                    for info in item["images"]:
                        path, _ = store.image(info["id"])
                        member = "images/" + path.name
                        info["archive_path"] = member
                        archive.write(str(path), member)
                archive.writestr("manifest.json", json.dumps(snapshot, ensure_ascii=False, indent=2))
                csv_output = io.StringIO(newline="")
                writer = csv.writer(csv_output)
                fields = ("id", "session_id", "received_at", "team", "task", "sub_id", "result", "extra", "image_count")
                writer.writerow(fields)
                for item in snapshot["reports"]:
                    cells = []
                    for field in fields:
                        value = str(item[field])
                        if value.lstrip().startswith(("=", "+", "-", "@")) or value.startswith(("\t", "\r", "\n")):
                            value = "'" + value
                        cells.append(value)
                    writer.writerow(cells)
                archive.writestr("reports.csv", "\ufeff" + csv_output.getvalue())
            output.seek(0)
            response = send_file(output, mimetype="application/zip", as_attachment=True,
                                 download_name="mock-judge-" + snapshot["session"]["id"] + ".zip")
            response.call_on_close(output.close)
            return response
        except BaseException:
            output.close()
            raise

    return app
