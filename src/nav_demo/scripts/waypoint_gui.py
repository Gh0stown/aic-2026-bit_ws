#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""巡检点位图形化记录工具（Tkinter）。

功能：
    - 左侧列出 `config/inspection_points.yaml` 里的预设点位（0~19 号）；
    - 右侧显示场地地图，实时画出小车当前位姿（红色箭头）；
    - 遥控把车开到位置，选中某个点位，点“记录当前位姿”，
      就把 map 坐标系下的 (x, y, yaw) 存到该点位；
    - 预设里没有的点位，可以在底部输入名字“添加并记录”；
    - 记录结果写到 `nav_demo/waypoints/inspection_waypoints.yaml`。

使用前提：
    roslaunch ~/bit_ws/src/nav777.launch      # 提供 map / AMCL / TF
    roslaunch mbot_teleop mbot_teleop.launch  # 遥控
    rosrun nav_demo waypoint_gui.py

参数：
    ~map      地图 yaml，默认 mymap_20261106_1916_clean.yaml
    ~presets  预设点位文件
    ~output   输出航点文件
"""

import argparse
import datetime
import math
import os
import threading

import tkinter as tk
import tkinter.font as tkfont
from tkinter import messagebox, ttk

import yaml
from PIL import Image, ImageTk

import rospy
import tf
import tf.transformations as tf_trans

PKG_DIR = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
DEFAULT_MAP = os.path.join(PKG_DIR, 'map', 'mymap_20261106_1916_clean.yaml')
DEFAULT_PRESETS = os.path.join(PKG_DIR, 'config', 'inspection_points.yaml')
DEFAULT_OUTPUT = os.path.join(PKG_DIR, 'waypoints', 'inspection_waypoints.yaml')

MAP_FRAME = 'map'
BASE_FRAME = 'base_footprint'
CANVAS_MAX_W = 520
CANVAS_MAX_H = 470


def load_yaml(path):
    with open(path, 'r') as f:
        return yaml.safe_load(f) or {}


def load_map(yaml_path):
    meta = load_yaml(yaml_path)
    img_path = meta.get('image')
    if not img_path:
        raise IOError('地图 yaml 里没有 image 字段: %s' % yaml_path)
    if not os.path.isabs(img_path):
        img_path = os.path.join(os.path.dirname(yaml_path), img_path)
    img = Image.open(img_path).convert('RGB')
    return img, meta


class PoseSource(object):
    """后台按 10Hz 读取 map -> base_footprint 的位姿。"""

    def __init__(self, map_frame, base_frame):
        self.map_frame = map_frame
        self.base_frame = base_frame
        self.listener = tf.TransformListener()
        self.lock = threading.Lock()
        self.pose = None
        self.ok = False
        rospy.Timer(rospy.Duration(0.1), self._tick)

    def _tick(self, _event):
        try:
            trans, rot = self.listener.lookupTransform(
                self.map_frame, self.base_frame, rospy.Time(0))
            yaw = tf_trans.euler_from_quaternion(rot)[2]
            with self.lock:
                self.pose = (trans[0], trans[1], yaw)
                self.ok = True
        except Exception:  # noqa: BLE001
            with self.lock:
                self.ok = False

    def get(self):
        with self.lock:
            return self.pose, self.ok


class WaypointGui(object):
    def __init__(self, root, map_img, map_meta, presets, recorded, output):
        self.root = root
        self.map_img = map_img
        self.map_meta = map_meta
        self.res = float(map_meta.get('resolution', 0.05))
        self.ox, self.oy = float(map_meta['origin'][0]), float(map_meta['origin'][1])
        self.output = output

        self.presets = presets
        self.preset_ids = [int(p['id']) for p in presets]
        self.preset_by_id = {int(p['id']): p for p in presets}
        self.recorded = recorded  # {int id: entry}

        self.pose_source = PoseSource(MAP_FRAME, BASE_FRAME)
        self.pose_items = {}
        self.arrow = None

        self._setup_fonts()
        self._build_ui()
        self._draw_map()
        self.refresh()
        children = self.tree.get_children()
        if children:
            self.tree.selection_set(children[0])
            self.tree.focus(children[0])
        self.root.after(100, self._tick)

    # ---------- 界面 ----------
    def _setup_fonts(self):
        families = set(tkfont.families())
        for name in ('Noto Sans CJK SC', 'Noto Sans CJK JP', 'WenQuanYi Micro Hei',
                     'Source Han Sans SC'):
            if name in families:
                for key in ('TkDefaultFont', 'TkTextFont', 'TkMenuFont', 'TkHeadingFont'):
                    tkfont.nametofont(key).configure(family=name, size=10)
                break
        self.font_family = tkfont.nametofont('TkDefaultFont').actual('family')

    def _build_ui(self):
        self.root.title('巡检点位记录')
        self.root.resizable(False, False)

        left = ttk.Frame(self.root, padding=6)
        left.grid(row=0, column=0, sticky='ns')
        right = ttk.Frame(self.root, padding=6)
        right.grid(row=0, column=1, sticky='nsew')

        # --- 左侧点位表 ---
        cols = ('status', 'id', 'name', 'note')
        self.tree = ttk.Treeview(left, columns=cols, show='headings', height=20,
                                 selectmode='browse')
        for col, text, width, anchor in (
                ('status', '状态', 40, 'center'),
                ('id', '序号', 40, 'center'),
                ('name', '点位名称', 205, 'w'),
                ('note', '备注', 170, 'w')):
            self.tree.heading(col, text=text)
            self.tree.column(col, width=width, anchor=anchor, stretch=False)
        sb = ttk.Scrollbar(left, orient='vertical', command=self.tree.yview)
        self.tree.configure(yscrollcommand=sb.set)
        self.tree.grid(row=0, column=0, columnspan=2, sticky='nsew')
        sb.grid(row=0, column=1, sticky='ns')
        self.tree.bind('<Double-1>', lambda e: self.record_selected())
        self.tree.bind('<<TreeviewSelect>>', self._on_select)

        self.detail_var = tk.StringVar()
        ttk.Label(left, textvariable=self.detail_var, wraplength=455,
                  foreground='#333').grid(row=1, column=0, columnspan=2,
                                          sticky='w', pady=(4, 6))

        ttk.Button(left, text='记录当前位姿（到选中点位）',
                   command=self.record_selected).grid(
            row=2, column=0, columnspan=2, sticky='ew', pady=(4, 2))
        ttk.Button(left, text='清除选中点位坐标', command=self.clear_selected).grid(
            row=3, column=0, columnspan=2, sticky='ew', pady=2)

        custom = ttk.LabelFrame(left, text='预设里没有的点位', padding=4)
        custom.grid(row=4, column=0, columnspan=2, sticky='ew', pady=(10, 0))
        self.custom_var = tk.StringVar()
        entry = ttk.Entry(custom, textvariable=self.custom_var, width=30)
        entry.grid(row=0, column=0, sticky='ew', padx=(0, 4))
        ttk.Button(custom, text='添加并记录', command=self.add_custom).grid(
            row=0, column=1)
        custom.columnconfigure(0, weight=1)

        self.status_var = tk.StringVar()
        ttk.Label(left, textvariable=self.status_var, foreground='#555').grid(
            row=5, column=0, columnspan=2, sticky='w', pady=(8, 0))

        # --- 右侧地图 ---
        self.scale = min(CANVAS_MAX_W / self.map_img.width,
                         CANVAS_MAX_H / self.map_img.height)
        disp = self.map_img.resize(
            (int(self.map_img.width * self.scale),
             int(self.map_img.height * self.scale)), Image.NEAREST)
        self.photo = ImageTk.PhotoImage(disp)
        self.canvas = tk.Canvas(right, width=disp.width, height=disp.height,
                                highlightthickness=1, highlightbackground='#555')
        self.canvas.pack()
        self.canvas.create_image(0, 0, anchor='nw', image=self.photo)

        self.pose_var = tk.StringVar(value='等待 map -> base_footprint ...')
        ttk.Label(right, textvariable=self.pose_var, font=(self.font_family, 10)).pack(
            anchor='w', pady=(6, 2))
        ttk.Label(right, text='绿色点 = 已记录；红色箭头 = 小车当前位姿',
                  foreground='#555').pack(anchor='w')

        # --- 底部状态 ---
        self.root.protocol('WM_DELETE_WINDOW', self.on_close)

    def world_to_canvas(self, x, y):
        col = (x - self.ox) / self.res
        row = self.map_img.height - (y - self.oy) / self.res
        return col * self.scale, row * self.scale

    def canvas_to_world(self, cx, cy):
        x = self.ox + (cx / self.scale) * self.res
        y = self.oy + (self.map_img.height - cy / self.scale) * self.res
        return x, y

    def _draw_map(self):
        self.canvas.delete('points')
        self.pose_items = {}
        for pid, entry in sorted(self.recorded.items()):
            if 'x' not in entry or 'y' not in entry:
                continue
            px, py = self.world_to_canvas(entry['x'], entry['y'])
            oval = self.canvas.create_oval(px - 6, py - 6, px + 6, py + 6,
                                           fill='#1a9e1a', outline='white', width=2,
                                           tags='points')
            label = self.canvas.create_text(px, py - 14, text=str(pid),
                                            fill='#0a6b0a', tags='points',
                                            font=(self.font_family, 9, 'bold'))
            self.pose_items[pid] = (oval, label)

    def _on_select(self, _event=None):
        selection = self.tree.selection()
        if not selection:
            self.detail_var.set('')
            return
        pid = int(selection[0])
        if pid in self.preset_by_id:
            p = self.preset_by_id[pid]
            name = p['name']
            note = p.get('note', '') or ''
            if p.get('guess') and '推测' not in note:
                note = ('推测同一位置；' + note).strip('；')
        else:
            e = self.recorded.get(pid, {})
            name = e.get('name', '自定义_%d' % pid)
            note = '自定义点位'
        entry = self.recorded.get(pid)
        if entry:
            note = '%s    坐标 x=%.3f  y=%.3f  yaw=%.1f°' % (
                note, entry['x'], entry['y'], entry.get('yaw_deg', 0.0))
        else:
            note = (note + '    （未记录）').strip()
        self.detail_var.set('%s：%s' % (name, note))

    # ---------- 数据 ----------
    def _insert_row(self, pid, name, note, custom=False):
        entry = self.recorded.get(pid)
        status = '已记' if entry and 'x' in entry else ''
        values = (status, str(pid), name, note)
        self.tree.insert('', 'end', iid=str(pid), values=values)

    def refresh(self):
        selection = self.tree.selection()
        for item in self.tree.get_children():
            self.tree.delete(item)

        for p in self.presets:
            note = p.get('note', '') or ''
            if p.get('guess'):
                note = ('推测：' + note) if note else '推测同一位置'
            self._insert_row(int(p['id']), p['name'], note)

        # 自定义点位（不在预设里的记录）
        for pid, entry in sorted(self.recorded.items()):
            if pid in self.preset_by_id:
                continue
            self._insert_row(pid, entry.get('name', '自定义_%d' % pid), '自定义点位',
                             custom=True)

        if selection:
            try:
                self.tree.selection_set(selection)
            except tk.TclError:
                pass
        self._draw_map()
        total = len(self.presets)
        done = sum(1 for p in self.presets if int(p['id']) in self.recorded)
        self.status_var.set('已记录 %d / %d 个预设点位   输出：%s'
                            % (done, total, os.path.basename(self.output)))

    def _save(self):
        out_dir = os.path.dirname(self.output)
        if out_dir and not os.path.isdir(out_dir):
            os.makedirs(out_dir)
        data = {
            'frame_id': MAP_FRAME,
            'map': os.path.basename(self.map_meta.get('image', '')),
            'points': [self.recorded[pid] for pid in sorted(self.recorded)],
        }
        header = ('# 巡检点位坐标（waypoint_gui.py 生成，单位米/弧度）\n')
        text = header + yaml.dump(data, allow_unicode=True,
                                  default_flow_style=False, sort_keys=False)
        tmp = self.output + '.tmp'
        with open(tmp, 'w') as f:
            f.write(text)
        os.replace(tmp, self.output)

    def _record(self, pid, name, confirm=True):
        pose, ok = self.pose_source.get()
        if not ok or pose is None:
            messagebox.showerror('没有位姿', '读不到 map -> base_footprint，\n'
                                             '请先确认导航 / AMCL 已启动。')
            return False
        if pid in self.recorded and confirm:
            if not messagebox.askyesno('覆盖', '%s 已经记录过，覆盖吗？' % name):
                return False
        entry = {
            'id': pid,
            'name': name,
            'x': round(pose[0], 3),
            'y': round(pose[1], 3),
            'yaw': round(pose[2], 4),
            'yaw_deg': round(math.degrees(pose[2]), 1),
            'stamp': datetime.datetime.now().strftime('%Y-%m-%d %H:%M:%S'),
        }
        self.recorded[pid] = entry
        try:
            self._save()
        except Exception as exc:  # noqa: BLE001
            del self.recorded[pid]
            messagebox.showerror('保存失败', str(exc))
            return False
        self.refresh()
        try:
            self.tree.selection_set(str(pid))
            self.tree.see(str(pid))
        except tk.TclError:
            pass
        return True

    # ---------- 操作 ----------
    def record_selected(self):
        selection = self.tree.selection()
        if not selection:
            messagebox.showinfo('提示', '先在左边选一个点位')
            return
        pid = int(selection[0])
        if pid in self.preset_by_id:
            name = self.preset_by_id[pid]['name']
        else:
            name = self.recorded[pid].get('name', '自定义_%d' % pid)
        self._record(pid, name)

    def clear_selected(self):
        selection = self.tree.selection()
        if not selection:
            return
        pid = int(selection[0])
        if pid not in self.recorded:
            return
        if not messagebox.askyesno('清除', '清除 %s 的坐标？' % self.recorded[pid].get('name')):
            return
        del self.recorded[pid]
        self._save()
        self.refresh()

    def add_custom(self):
        name = self.custom_var.get().strip()
        if not name:
            messagebox.showinfo('提示', '先输入点位名字')
            return
        pid = max([99] + list(self.recorded) + self.preset_ids) + 1
        if self._record(pid, name, confirm=False):
            self.custom_var.set('')

    # ---------- 定时刷新位姿 ----------
    def _tick(self):
        pose, ok = self.pose_source.get()
        if ok and pose is not None:
            self.pose_var.set('当前位姿  x=%+.3f  y=%+.3f  yaw=%+.4f rad (%+.1f°)'
                              % (pose[0], pose[1], pose[2], math.degrees(pose[2])))
            self._draw_arrow(*pose)
        else:
            self.pose_var.set('等待 map -> base_footprint ...')
        self.root.after(100, self._tick)

    def _draw_arrow(self, x, y, yaw):
        px, py = self.world_to_canvas(x, y)
        a = -yaw
        c, s = math.cos(a), math.sin(a)
        tip = (px + 16 * c, py + 16 * s)
        b1 = (px - 6 * c - 7 * s, py - 6 * s + 7 * c)
        b2 = (px - 6 * c + 7 * s, py - 6 * s - 7 * c)
        coords = [tip[0], tip[1], b1[0], b1[1], b2[0], b2[1]]
        if self.arrow is None:
            self.arrow = self.canvas.create_polygon(
                coords, fill='#e02020', outline='white', width=1)
        else:
            self.canvas.coords(self.arrow, *coords)
        self.canvas.tag_raise(self.arrow)

    def on_close(self):
        rospy.signal_shutdown('gui closed')
        self.root.destroy()


def main():
    ap = argparse.ArgumentParser(description='巡检点位图形化记录')
    ap.add_argument('--map', default=DEFAULT_MAP)
    ap.add_argument('--presets', default=DEFAULT_PRESETS)
    ap.add_argument('--output', default=DEFAULT_OUTPUT)
    ap.add_argument('--withdraw', action='store_true', help=argparse.SUPPRESS)
    ap.add_argument('--self-test', action='store_true', help=argparse.SUPPRESS)
    args, _ = ap.parse_known_args()

    try:
        rospy.init_node('waypoint_gui', disable_signals=True)
    except Exception as exc:  # noqa: BLE001
        raise SystemExit('连不上 ROS master，请先启动导航：roslaunch ~/bit_ws/src/nav777.launch\n%s' % exc)
    threading.Thread(target=rospy.spin, daemon=True).start()

    map_img, map_meta = load_map(args.map)
    presets = load_yaml(args.presets).get('points', [])
    if not presets:
        raise SystemExit('预设点位为空: %s' % args.presets)

    recorded = {}
    if os.path.isfile(args.output):
        for p in load_yaml(args.output).get('points', []):
            if 'id' in p and 'x' in p and 'y' in p:
                recorded[int(p['id'])] = p

    root = tk.Tk()
    app = WaypointGui(root, map_img, map_meta, presets, recorded, args.output)
    if args.withdraw:
        root.withdraw()

    if args.self_test:
        def _selftest():
            first = app.tree.get_children()[0]
            app.tree.selection_set(first)
            ok = app._record(int(first), app.preset_by_id[int(first)]['name'],
                             confirm=False)
            print('SELF-TEST %s -> %s' % ('OK' if ok else 'FAIL', args.output))
            root.after(200, root.destroy)
        root.after(1500, _selftest)

    root.mainloop()


if __name__ == '__main__':
    main()
