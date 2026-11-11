"""SQLite persistence and serialized simulated-session transitions."""
import os
import sqlite3
import threading
import time
import uuid
from contextlib import contextmanager
from datetime import datetime, timezone
from pathlib import Path

TASKS = (
    ("crowd", "人群数量"), ("sign", "指示牌"), ("trash", "垃圾桶状态"),
    ("fire", "楼宇火灾"), ("plate", "车辆车牌"),
    ("temperature", "异常温度"), ("meter", "站房仪表"), ("ebike", "电动车状态"),
)


class JudgeError(Exception):
    def __init__(self, message, status=400):
        super().__init__(message)
        self.status = status


def iso(timestamp):
    if timestamp is None:
        return None
    return datetime.fromtimestamp(timestamp, timezone.utc).isoformat().replace("+00:00", "Z")


class Store:
    def __init__(self, data_dir, clock=None):
        self.root = Path(data_dir).expanduser().resolve()
        self.root.mkdir(parents=True, exist_ok=True)
        self.images_dir = self.root / "images"
        self.staging_dir = self.root / "staging"
        self.images_dir.mkdir(exist_ok=True)
        self.staging_dir.mkdir(exist_ok=True)
        self.db_path = self.root / "judge.sqlite3"
        self.clock = clock or time.time
        self.lock = threading.RLock()
        with self.transaction() as db:
            version = db.execute("PRAGMA user_version").fetchone()[0]
            if version not in (0, 1):
                raise RuntimeError("不支持的数据版本：%s" % version)
            statements = (
                "CREATE TABLE IF NOT EXISTS sessions (id TEXT PRIMARY KEY, created_at REAL NOT NULL, status TEXT NOT NULL, locked_team TEXT, started_at REAL, finished_at REAL)",
                "CREATE TABLE IF NOT EXISTS hellos (session_id TEXT NOT NULL REFERENCES sessions(id), team TEXT NOT NULL, received_at REAL NOT NULL, PRIMARY KEY(session_id, team))",
                "CREATE TABLE IF NOT EXISTS reports (id TEXT PRIMARY KEY, session_id TEXT NOT NULL REFERENCES sessions(id), received_at REAL NOT NULL, team TEXT NOT NULL, task TEXT NOT NULL, sub_id TEXT NOT NULL, result TEXT NOT NULL, extra TEXT NOT NULL)",
                "CREATE TABLE IF NOT EXISTS images (id TEXT PRIMARY KEY, report_id TEXT NOT NULL REFERENCES reports(id), position INTEGER NOT NULL, original_name TEXT NOT NULL, media_type TEXT NOT NULL, width INTEGER NOT NULL, height INTEGER NOT NULL, filename TEXT NOT NULL UNIQUE)",
                "CREATE TABLE IF NOT EXISTS events (id INTEGER PRIMARY KEY, session_id TEXT NOT NULL REFERENCES sessions(id), received_at REAL NOT NULL, action TEXT NOT NULL, detail TEXT NOT NULL)",
                "CREATE INDEX IF NOT EXISTS report_group ON reports(session_id, team, task, sub_id)",
                "CREATE INDEX IF NOT EXISTS image_report ON images(report_id, position)",
            )
            for statement in statements:
                db.execute(statement)
            db.execute("PRAGMA user_version=1")
            if db.execute("SELECT 1 FROM sessions LIMIT 1").fetchone() is None:
                self._new_session(db)

    @contextmanager
    def transaction(self, write=True):
        # One connection per operation: safe with Flask threaded requests and restart.
        with self.lock:
            db = sqlite3.connect(str(self.db_path), timeout=30, isolation_level=None)
            db.row_factory = sqlite3.Row
            try:
                db.execute("PRAGMA foreign_keys=ON")
                db.execute("BEGIN IMMEDIATE" if write else "BEGIN")
                yield db
                db.commit()
            except BaseException:
                db.rollback()
                raise
            finally:
                db.close()

    def _new_session(self, db):
        sid = uuid.uuid4().hex
        db.execute("INSERT INTO sessions(id,created_at,status) VALUES(?,?,'ready')", (sid, self.clock()))
        self._event(db, sid, "new", "")
        return self._session(db, sid)

    def _session(self, db, session_id=None):
        if session_id is None:
            row = db.execute("SELECT * FROM sessions ORDER BY rowid DESC LIMIT 1").fetchone()
        else:
            row = db.execute("SELECT * FROM sessions WHERE id=?", (session_id,)).fetchone()
        if row is None:
            raise JudgeError("场次不存在", 404)
        return row

    def _session_json(self, row):
        end = row["finished_at"] if row["finished_at"] is not None else self.clock()
        elapsed = max(0.0, end - row["started_at"]) if row["started_at"] is not None else 0.0
        return dict(id=row["id"], created_at=iso(row["created_at"]), status=row["status"],
                    locked_team=row["locked_team"], started_at=iso(row["started_at"]),
                    finished_at=iso(row["finished_at"]), elapsed_seconds=elapsed)

    def _event(self, db, sid, action, detail):
        db.execute("INSERT INTO events(session_id,received_at,action,detail) VALUES(?,?,?,?)",
                   (sid, self.clock(), action, detail))

    @staticmethod
    def _check_team(session, team):
        if session["locked_team"] is not None and session["locked_team"] != team:
            raise JudgeError("裁判台已锁定其他队伍，请确认队名", 403)

    def hello(self, team):
        with self.transaction() as db:
            session = self._session(db)
            self._check_team(session, team)
            db.execute("INSERT INTO hellos(session_id,team,received_at) VALUES(?,?,?) "
                       "ON CONFLICT(session_id,team) DO UPDATE SET received_at=excluded.received_at",
                       (session["id"], team, self.clock()))
        return {"ok": True, "message": "通信正常", "team": team}

    def report(self, payload, staged_images):
        """Move already validated temporary images; roll back files on SQL errors."""
        moved = []
        report_id = uuid.uuid4().hex
        try:
            with self.transaction() as db:
                session = self._session(db)
                self._check_team(session, payload["team"])
                if session["status"] == "finished":
                    raise JudgeError("当前场次已结束，请新建场次后上报", 409)
                db.execute("INSERT INTO reports VALUES(?,?,?,?,?,?,?,?)",
                           (report_id, session["id"], self.clock(), payload["team"], payload["task"],
                            payload["sub_id"], payload["result"], payload["extra"]))
                for position, image in enumerate(staged_images):
                    image_id = uuid.uuid4().hex
                    filename = image_id + image["extension"]
                    destination = self.images_dir / filename
                    os.replace(str(image["path"]), str(destination))
                    moved.append(destination)
                    db.execute("INSERT INTO images VALUES(?,?,?,?,?,?,?,?)",
                               (image_id, report_id, position, image["original_name"], image["media_type"],
                                image["width"], image["height"], filename))
        except BaseException:
            for path in moved:
                try:
                    path.unlink()
                except FileNotFoundError:
                    pass
            raise
        return {"ok": True, "report_id": report_id, "image_count": len(staged_images)}

    def manage(self, action, team=None):
        with self.transaction() as db:
            session = self._session(db)
            sid = session["id"]
            if action == "new":
                if session["status"] == "running":
                    raise JudgeError("请先结束正在计时的场次，再新建场次", 409)
                session = self._new_session(db)
                return self._session_json(session)
            if action == "team-lock":
                db.execute("UPDATE sessions SET locked_team=? WHERE id=?", (team, sid))
                detail = team or ""
            elif action == "start":
                if session["status"] != "ready":
                    raise JudgeError("仅准备中的场次可以开始", 409)
                db.execute("UPDATE sessions SET status='running',started_at=? WHERE id=?", (self.clock(), sid))
                detail = ""
            elif action == "finish":
                if session["status"] != "running":
                    raise JudgeError("仅计时中的场次可以结束", 409)
                db.execute("UPDATE sessions SET status='finished',finished_at=? WHERE id=?", (self.clock(), sid))
                detail = ""
            else:
                raise JudgeError("未知管理操作", 404)
            self._event(db, sid, action, detail)
            return self._session_json(self._session(db, sid))

    def state(self, session_id=None):
        with self.transaction(write=False) as db:
            session = self._session(db, session_id)
            sessions = [self._session_json(r) for r in db.execute("SELECT * FROM sessions ORDER BY rowid DESC")]
            teams = {}
            for row in db.execute("SELECT team,received_at FROM hellos WHERE session_id=?", (session["id"],)):
                teams[row["team"]] = dict(team=row["team"], last_hello_at=iso(row["received_at"]),
                                          last_report_at=None, report_count=0)
            for row in db.execute("SELECT team,MAX(received_at) AS last,COUNT(*) AS count FROM reports WHERE session_id=? GROUP BY team", (session["id"],)):
                info = teams.setdefault(row["team"], dict(team=row["team"], last_hello_at=None))
                info.update(last_report_at=iso(row["last"]), report_count=row["count"])
            return dict(ok=True, schema_version=1, server_time=iso(self.clock()),
                        session=self._session_json(session), sessions=sessions,
                        teams=sorted(teams.values(), key=lambda t: t["team"]),
                        tasks=[dict(id=key, label=label) for key, label in TASKS])

    def _reports(self, db, session_id, filters, limit=None, offset=0):
        where = ["session_id=?"]
        args = [session_id]
        for field in ("team", "task", "sub_id"):
            if field in filters:
                where.append(field + "=?")
                args.append(filters[field])
        clause = " AND ".join(where)
        total = db.execute("SELECT COUNT(*) FROM reports WHERE " + clause, args).fetchone()[0]
        sql = "SELECT * FROM reports WHERE " + clause + " ORDER BY rowid DESC"
        if limit is not None:
            sql += " LIMIT ? OFFSET ?"
            args += [limit, offset]
        reports = []
        for row in db.execute(sql, args).fetchall():
            item = dict(row)
            item["received_at"] = iso(item["received_at"])
            images = []
            for image in db.execute("SELECT * FROM images WHERE report_id=? ORDER BY position", (row["id"],)):
                info = dict(image)
                info.pop("filename")
                info.pop("report_id")
                info["url"] = "/api/images/" + info["id"]
                images.append(info)
            item.update(images=images, image_count=len(images))
            reports.append(item)
        return reports, total

    def reports(self, session_id=None, filters=None, limit=100, offset=0):
        with self.transaction(write=False) as db:
            session = self._session(db, session_id)
            reports, total = self._reports(db, session["id"], filters or {}, limit, offset)
            return dict(ok=True, reports=reports, total=total, limit=limit, offset=offset)

    def image(self, image_id):
        with self.transaction(write=False) as db:
            row = db.execute("SELECT * FROM images WHERE id=?", (image_id,)).fetchone()
            if row is None:
                raise JudgeError("图片不存在", 404)
            path = self.images_dir / row["filename"]
            if not path.is_file():
                raise JudgeError("图片文件缺失，请检查数据目录", 404)
            return path, row["media_type"]

    def export_snapshot(self, session_id=None):
        with self.transaction(write=False) as db:
            session = self._session(db, session_id)
            reports, total = self._reports(db, session["id"], {})
            events = []
            for row in db.execute("SELECT * FROM events WHERE session_id=? ORDER BY id", (session["id"],)):
                event = dict(row)
                event["received_at"] = iso(event["received_at"])
                events.append(event)
            hellos = [dict(team=r["team"], received_at=iso(r["received_at"])) for r in
                      db.execute("SELECT team,received_at FROM hellos WHERE session_id=? ORDER BY team", (session["id"],))]
            return dict(schema_version=1, exported_at=iso(self.clock()),
                        session=self._session_json(session), reports=reports, report_count=total,
                        events=events, hellos=hellos)
