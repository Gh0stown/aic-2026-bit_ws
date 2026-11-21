#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""裁判系统上报客户端。

协议来源：`tools/mock_judge/README.md`、`上传案例/选手接入说明.md`。
- hello()：赛前连通测试，不触发计时；
- report()：后台线程异步上报，不阻塞导航/识别；
- 任务：crowd / sign / trash / fire / plate / temperature / meter / ebike；
  红绿灯、泊车入位不上传。
"""

import argparse
import os
import queue
import threading

try:
    import requests
except Exception:  # noqa: BLE001
    requests = None


ASSESSMENT_TASKS = ('crowd', 'sign', 'trash', 'fire', 'plate',
                    'temperature', 'meter', 'ebike')


class JudgeClient(object):
    def __init__(self, base_url, team, timeout=3.0, log=None):
        if requests is None:
            raise RuntimeError('缺少 requests：python3 -m pip install requests')
        self.base_url = str(base_url).rstrip('/')
        self.team = str(team)
        self.timeout = float(timeout)
        self.log = log or (lambda _text: None)
        self._queue = queue.Queue()
        self._stop = threading.Event()
        self._thread = threading.Thread(target=self._worker, daemon=True)
        self._thread.start()

    def hello(self):
        try:
            resp = requests.post('%s/api/hello' % self.base_url,
                                 json={'team': self.team}, timeout=self.timeout)
            data = resp.json()
            return bool(data.get('ok')), str(data.get('message') or resp.text)
        except Exception as exc:  # noqa: BLE001
            return False, str(exc)

    def report(self, task, result, sub_id='', extra='', images=None):
        if task not in ASSESSMENT_TASKS:
            raise ValueError('不支持的任务：%s' % task)
        self._queue.put((task, result, sub_id, extra, list(images or [])))

    def _worker(self):
        while not self._stop.is_set():
            try:
                item = self._queue.get(timeout=0.2)
            except queue.Empty:
                continue
            if item is None:
                break
            task, result, sub_id, extra, images = item
            opened = []
            files = []
            try:
                for path in images:
                    if not os.path.isfile(path):
                        continue
                    fh = open(path, 'rb')
                    opened.append(fh)
                    mime = ('image/png' if path.lower().endswith('.png')
                            else 'image/jpeg')
                    files.append(('images', (os.path.basename(path), fh, mime)))
                data = {'team': self.team, 'task': task, 'result': result,
                        'sub_id': sub_id, 'extra': extra}
                resp = requests.post('%s/api/report' % self.base_url, data=data,
                                     files=files or None,
                                     timeout=max(6.0, self.timeout))
                try:
                    body = resp.json()
                except Exception:  # noqa: BLE001
                    body = {}
                if resp.ok and body.get('ok'):
                    self.log('[裁判] %s 上报成功 id=%s 图片=%s'
                             % (task, body.get('report_id'), body.get('image_count')))
                else:
                    self.log('[裁判] %s 上报失败 HTTP %s：%s'
                             % (task, resp.status_code,
                                body.get('error') or resp.text[:200]))
            except Exception as exc:  # noqa: BLE001
                self.log('[裁判] %s 上报异常：%s' % (task, exc))
            finally:
                for fh in opened:
                    try:
                        fh.close()
                    except Exception:  # noqa: BLE001
                        pass

    def close(self, wait=True):
        self._stop.set()
        self._queue.put(None)
        if wait:
            self._thread.join(timeout=5.0)


def main():
    ap = argparse.ArgumentParser(description='裁判系统连通/上报测试')
    ap.add_argument('--url', default='http://127.0.0.1:8080')
    ap.add_argument('--team', required=True)
    ap.add_argument('--hello', action='store_true')
    ap.add_argument('--task', choices=ASSESSMENT_TASKS)
    ap.add_argument('--result', default='')
    ap.add_argument('--sub-id', default='')
    ap.add_argument('--extra', default='')
    ap.add_argument('--image', action='append', default=[])
    args = ap.parse_args()
    client = JudgeClient(args.url, args.team, log=print)
    try:
        if args.hello:
            ok, message = client.hello()
            print('hello:', 'OK' if ok else 'FAIL', message)
        if args.task:
            client.report(args.task, args.result, args.sub_id, args.extra, args.image)
    finally:
        client.close()
    print('done')


if __name__ == '__main__':
    main()
