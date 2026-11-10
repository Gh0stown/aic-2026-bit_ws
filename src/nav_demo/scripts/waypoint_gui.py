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
        family = tkfont.nametofont('TkDefaultFont').actual('family')
        for name in ('Noto Sans CJK SC', 'Noto Sans CJK JP', 'WenQuanYi Micro Hei',
                     'Source Han Sans SC'):
            if name in families:
                family = name
                break
        for key in ('TkDefaultFont', 'TkTextFont', 'TkMenuFont', 'TkHeadingFont'):
            tkfont.nametofont(key).configure(family=family, size=10)
        self.font_family = family
        self.colors = {'bg': '#edf2f6', 'ink': '#203247', 'muted': '#687b8e',
                       'line': '#dbe4ec', 'accent': '#087f8c', 'navy': '#172b40'}
        c = self.colors
        style = ttk.Style(self.root)
        style.theme_use('clam')
        style.configure('.', font=(family, 10), foreground=c['ink'], background=c['bg'])
        style.configure('TFrame', background=c['bg'])
        style.configure('Card.TFrame', background='white')
        style.configure('TLabel', background='white')
        style.configure('Muted.TLabel', foreground=c['muted'])
        style.configure('Section.TLabel', font=(family, 11, 'bold'))
        style.configure('TButton', padding=(10, 5), background='#f1f5f9',
                        bordercolor=c['line'], focuscolor=c['accent'])
        style.map('TButton', background=[('active', '#e3edf4')],
                  foreground=[('disabled', '#9ba9b6')])
        style.configure('Accent.TButton', background=c['accent'], foreground='white',
                        bordercolor=c['accent'], font=(family, 11, 'bold'), padding=(12, 9))
        style.map('Accent.TButton', background=[('active', '#066b77')])
        style.configure('Danger.TButton', foreground='#b83d4c')
        style.configure('TEntry', padding=5, fieldbackground='white', bordercolor=c['line'])
        style.configure('TCheckbutton', background='white', padding=4)
        style.map('TCheckbutton', background=[('active', 'white')])
        style.configure('Treeview', background='white', fieldbackground='white',
                        rowheight=32, borderwidth=0)
        style.configure('Treeview.Heading', background='#edf3f7', foreground=c['muted'],
                        font=(family, 9, 'bold'), padding=(5, 7))
        style.map('Treeview', background=[('selected', '#cce9ec')],
                  foreground=[('selected', '#075b65')])
        style.configure('Horizontal.TProgressbar', background=c['accent'],
                        troughcolor='#e7eef3', borderwidth=0, thickness=5)
        style.configure('TSeparator', background=c['line'])

    def _build_ui(self):
        c = self.colors
        self.root.title('巡检点位记录 · 点位工作台')
        width = min(1320, self.root.winfo_screenwidth() - 60)
        height = min(900, self.root.winfo_screenheight() - 90)
        self.root.geometry('%dx%d' % (width, height))
        self.root.minsize(1100, 760)
        self.root.resizable(True, True)
        self.root.configure(background=c['bg'])
        self.view_zoom = 1.0
        self.view_pan_x = self.view_pan_y = 0.0
        self.map_offset_x = self.map_offset_y = 0.0
        self._map_resize_job = None
        self._hidden_rows = set()
        self._map_drag = None

        header = tk.Frame(self.root, background=c['navy'], padx=20, pady=12)
        header.pack(fill='x')
        tk.Label(header, text='巡检点位记录', background=c['navy'], foreground='white',
                 font=(self.font_family, 19, 'bold')).pack(side='left')
        tk.Label(header, text=' /  WAYPOINT STUDIO', background=c['navy'],
                 foreground='#9fb5c9', font=(self.font_family, 9)).pack(side='left', padx=12)
        self.connection_var = tk.StringVar(value='等待定位')
        self.connection_badge = tk.Label(header, textvariable=self.connection_var,
                                         background='#294357', foreground='#dbe8f2',
                                         font=(self.font_family, 10, 'bold'), padx=12, pady=4)
        self.connection_badge.pack(side='right')

        footer = ttk.Frame(self.root, style='Card.TFrame', padding=(16, 8))
        footer.pack(side='bottom', fill='x', padx=14, pady=(0, 12))
        self.status_var = tk.StringVar()
        ttk.Label(footer, textvariable=self.status_var, style='Muted.TLabel').pack(anchor='w')
        self.output_label = ttk.Label(footer, text='保存位置：' + self.output,
                                      style='Muted.TLabel', font=(self.font_family, 9))
        self.output_label.pack(anchor='w', fill='x')
        footer.bind('<Configure>', lambda e: self.output_label.configure(wraplength=e.width - 32))

        body = ttk.Panedwindow(self.root, orient='horizontal')
        body.pack(fill='both', expand=True, padx=14, pady=12)
        left = ttk.Frame(body, style='Card.TFrame', padding=14)
        right = ttk.Frame(body, style='Card.TFrame', padding=14)
        body.add(left, weight=5)
        body.add(right, weight=6)
        self.root.after_idle(lambda: body.sashpos(0, int(width * 0.46)))
        left.columnconfigure(0, weight=1)
        left.rowconfigure(3, weight=1)

        list_header = ttk.Frame(left, style='Card.TFrame')
        list_header.grid(row=0, column=0, sticky='ew')
        ttk.Label(list_header, text='点位档案', style='Section.TLabel').pack(side='left')
        self.summary_var = tk.StringVar(value='等待载入')
        ttk.Label(list_header, textvariable=self.summary_var, style='Muted.TLabel').pack(side='right')
        self.progress = ttk.Progressbar(left, maximum=max(1, len(self.presets)))
        self.progress.grid(row=1, column=0, sticky='ew', pady=(8, 10))
        filters = ttk.Frame(left, style='Card.TFrame')
        filters.grid(row=2, column=0, sticky='ew', pady=(0, 10))
        filters.columnconfigure(1, weight=1)
        ttk.Label(filters, text='查找', style='Muted.TLabel').grid(row=0, column=0, padx=(0, 8))
        self.search_var = tk.StringVar()
        self.search_entry = ttk.Entry(filters, textvariable=self.search_var, width=16)
        self.search_entry.grid(row=0, column=1, sticky='ew')
        self.missing_only_var = tk.BooleanVar(value=False)
        ttk.Checkbutton(filters, text='仅看未记录', variable=self.missing_only_var,
                        command=self._apply_filter).grid(row=0, column=2, padx=(8, 0))
        self.search_var.trace_add('write', lambda *_: self._apply_filter())

        table = ttk.Frame(left, style='Card.TFrame')
        table.grid(row=3, column=0, sticky='nsew')
        table.rowconfigure(0, weight=1)
        table.columnconfigure(0, weight=1)
        cols = ('status', 'id', 'name', 'tasks', 'note')
        self.tree = ttk.Treeview(table, columns=cols, show='headings', height=7,
                                 selectmode='browse')
        for col, text, col_width, minimum, anchor in (
                ('status', '状态', 60, 58, 'center'), ('id', 'ID', 40, 36, 'center'),
                ('name', '点位名称', 166, 120, 'w'), ('tasks', '识别朝向', 94, 90, 'w'),
                ('note', '备注', 160, 120, 'w')):
            self.tree.heading(col, text=text)
            self.tree.column(col, width=col_width, minwidth=minimum, anchor=anchor,
                             stretch=col in ('name', 'note'))
        self.tree.grid(row=0, column=0, sticky='nsew')
        sb = ttk.Scrollbar(table, orient='vertical', command=self.tree.yview)
        sb.grid(row=0, column=1, sticky='ns')
        hsb = ttk.Scrollbar(table, orient='horizontal', command=self.tree.xview)
        hsb.grid(row=1, column=0, sticky='ew')
        self.tree.configure(yscrollcommand=sb.set, xscrollcommand=hsb.set)
        self.tree.tag_configure('recorded', background='#f0f8f8', foreground='#176772')
        self.tree.tag_configure('missing', background='#fff8ed', foreground='#8b5d27')
        self.tree.bind('<Double-1>', lambda e: self.record_selected())
        self.tree.bind('<<TreeviewSelect>>', self._on_select)

        selection = ttk.Frame(left, style='Card.TFrame')
        selection.grid(row=4, column=0, sticky='ew', pady=(10, 8))
        ttk.Label(selection, text='当前选中', style='Section.TLabel').pack(anchor='w')
        self.detail_var = tk.StringVar(value='选择左侧点位，或点击地图中的已记录点')
        self.detail_label = ttk.Label(selection, textvariable=self.detail_var,
                                      wraplength=480, justify='left')
        self.detail_label.pack(fill='x', pady=(4, 2))
        self.task_detail_var = tk.StringVar(value='识别任务：只读展示，不会修改任务朝向')
        self.task_label = ttk.Label(selection, textvariable=self.task_detail_var,
                                    style='Muted.TLabel', wraplength=480, justify='left')
        self.task_label.pack(fill='x')
        selection.bind('<Configure>', lambda e: self._wrap_details(e.width))

        actions = ttk.Frame(left, style='Card.TFrame')
        actions.grid(row=5, column=0, sticky='ew')
        actions.columnconfigure(0, weight=1)
        actions.columnconfigure(1, weight=1)
        self.record_btn = ttk.Button(actions, text='记录当前位姿 → 选中点位',
                                      command=self.record_selected, style='Accent.TButton')
        self.record_btn.grid(row=0, column=0, columnspan=2, sticky='ew', pady=(0, 7))
        self.next_btn = ttk.Button(actions, text='选择下一个未记录点', command=self._select_next_missing)
        self.next_btn.grid(row=1, column=0, sticky='ew', padx=(0, 6))
        self.clear_btn = ttk.Button(actions, text='清除选中坐标…', command=self.clear_selected,
                                     style='Danger.TButton')
        self.clear_btn.grid(row=1, column=1, sticky='ew')
        custom = ttk.Frame(left, style='Card.TFrame')
        custom.grid(row=6, column=0, sticky='ew', pady=(10, 0))
        custom.columnconfigure(0, weight=1)
        ttk.Label(custom, text='自定义点位', style='Muted.TLabel').grid(
            row=0, column=0, columnspan=2, sticky='w', pady=(0, 4))
        self.custom_var = tk.StringVar()
        self.custom_entry = ttk.Entry(custom, textvariable=self.custom_var, width=24)
        self.custom_entry.grid(row=1, column=0, sticky='ew', padx=(0, 7))
        self.add_btn = ttk.Button(custom, text='添加并记录', command=self.add_custom)
        self.add_btn.grid(row=1, column=1, sticky='ew')

        map_header = ttk.Frame(right, style='Card.TFrame')
        map_header.pack(fill='x', pady=(0, 8))
        ttk.Label(map_header, text='场地地图', style='Section.TLabel').pack(side='left')
        self.zoom_var = tk.StringVar(value='100%')
        ttk.Button(map_header, text='适应窗口', command=self._fit_map).pack(side='right')
        ttk.Button(map_header, text='＋', width=3, command=lambda: self._zoom_map(1.25)).pack(
            side='right', padx=4)
        ttk.Label(map_header, textvariable=self.zoom_var, style='Muted.TLabel').pack(side='right')
        ttk.Button(map_header, text='−', width=3, command=lambda: self._zoom_map(0.8)).pack(
            side='right', padx=4)
        map_footer = ttk.Frame(right, style='Card.TFrame')
        map_footer.pack(side='bottom', fill='x')
        self.pose_var = tk.StringVar(value='等待 map -> base_footprint ...')
        self.pose_label = ttk.Label(map_footer, textvariable=self.pose_var,
                                    wraplength=500, font=(self.font_family, 10))
        self.pose_label.pack(anchor='w', fill='x', pady=(10, 8))
        ttk.Separator(map_footer).pack(fill='x', pady=(0, 8))
        ttk.Label(map_footer, text='● 已记录点位    ◯ 当前选中    ▲ 机器人',
                  style='Muted.TLabel').pack(anchor='w')
        ttk.Label(map_footer, text='滚轮缩放 · 拖动平移 · 单击点位联动列表',
                  style='Muted.TLabel', font=(self.font_family, 9)).pack(anchor='w', pady=(3, 0))
        ttk.Label(map_footer, text='地图操作只改变视图，不写入坐标；记录仍取实时 TF。',
                  style='Muted.TLabel', font=(self.font_family, 9)).pack(anchor='w', pady=(3, 0))
        map_footer.bind('<Configure>', lambda e: self.pose_label.configure(wraplength=e.width))
        self.canvas = tk.Canvas(right, width=CANVAS_MAX_W, height=CANVAS_MAX_H,
                                background='#f4f7fa', highlightthickness=0, cursor='hand2')
        self.canvas.pack(fill='both', expand=True)
        self.canvas.bind('<Configure>', self._on_map_resize)
        self.canvas.bind('<ButtonPress-1>', self._start_map_drag)
        self.canvas.bind('<B1-Motion>', self._drag_map)
        self.canvas.bind('<ButtonRelease-1>', self._end_map_drag)
        self.canvas.bind('<Button-4>', lambda e: self._zoom_map(1.25, e.x, e.y))
        self.canvas.bind('<Button-5>', lambda e: self._zoom_map(0.8, e.x, e.y))
        self.canvas.bind('<MouseWheel>', lambda e: self._zoom_map(
            1.25 if e.delta > 0 else 0.8, e.x, e.y) if e.delta else None)
        self._render_map()
        self.root.protocol('WM_DELETE_WINDOW', self.on_close)

    def _wrap_details(self, width):
        for label in (self.detail_label, self.task_label):
            label.configure(wraplength=max(160, width))

    def _task_text(self, pid, names=False):
        tasks = self.recorded.get(pid, {}).get('tasks') or []
        return ' / '.join(('%s ' % task.get('name', '') if names else '') +
                          ('%.0f°' % math.degrees(float(task.get('yaw', 0.0))))
                          for task in tasks) or '—'

    def _apply_filter(self):
        query = self.search_var.get().strip().casefold()
        ids = self.preset_ids + [pid for pid in sorted(self.recorded) if pid not in self.preset_by_id]
        for pid in ids:
            iid = str(pid)
            if not self.tree.exists(iid):
                continue
            values = self.tree.item(iid, 'values')
            matches = (not query or query in ' '.join(str(v) for v in values).casefold())
            matches = matches and (not self.missing_only_var.get() or pid not in self.recorded)
            if matches:
                self.tree.move(iid, '', 'end')
                self._hidden_rows.discard(iid)
            else:
                if iid in self.tree.selection():
                    self.tree.selection_remove(iid)
                self.tree.detach(iid)
                self._hidden_rows.add(iid)
        self._on_select()

    def _select_next_missing(self):
        missing = [pid for pid in self.preset_ids if pid not in self.recorded]
        if not missing:
            messagebox.showinfo('点位已齐全', '所有预设点位都已记录。')
            return
        selected = self.tree.selection()
        current = int(selected[0]) if selected else None
        start = self.preset_ids.index(current) + 1 if current in self.preset_ids else 0
        candidates = self.preset_ids[start:] + self.preset_ids[:start]
        pid = next(pid for pid in candidates if pid in missing)
        self.search_var.set('')
        self.missing_only_var.set(False)
        self._apply_filter()
        self.tree.selection_set(str(pid))
        self.tree.focus(str(pid))
        self.tree.see(str(pid))
        self._on_select()

    def _on_map_resize(self, _event):
        if self._map_resize_job is not None:
            self.root.after_cancel(self._map_resize_job)
        self._map_resize_job = self.root.after(80, self._resize_map)

    def _resize_map(self):
        self._map_resize_job = None
        self._render_map()

    def _map_transform(self):
        width = max(1, self.canvas.winfo_width())
        height = max(1, self.canvas.winfo_height())
        if width <= 1 or height <= 1:
            width, height = CANVAS_MAX_W, CANVAS_MAX_H
        self.scale = min(max(1, width - 40) / self.map_img.width,
                         max(1, height - 40) / self.map_img.height) * self.view_zoom
        self.map_offset_x = (width - self.map_img.width * self.scale) / 2 + self.view_pan_x
        self.map_offset_y = (height - self.map_img.height * self.scale) / 2 + self.view_pan_y

    def _render_map(self):
        self._map_transform()
        disp = self.map_img.resize((max(1, int(self.map_img.width * self.scale)),
                                    max(1, int(self.map_img.height * self.scale))), Image.NEAREST)
        self.photo = ImageTk.PhotoImage(disp, master=self.root)
        self.canvas.delete('all')
        self.canvas.create_image(self.map_offset_x, self.map_offset_y,
                                 anchor='nw', image=self.photo, tags='map')
        self.arrow = None
        self.zoom_var.set('%d%%' % round(self.view_zoom * 100))
        self._draw_map()
        pose, ok = self.pose_source.get()
        if ok and pose is not None:
            self._draw_arrow(*pose)

    def _fit_map(self):
        self.view_zoom = 1.0
        self.view_pan_x = self.view_pan_y = 0.0
        self._render_map()

    def _zoom_map(self, factor, cx=None, cy=None):
        cx = self.canvas.winfo_width() / 2 if cx is None else cx
        cy = self.canvas.winfo_height() / 2 if cy is None else cy
        world = self.canvas_to_world(cx, cy)
        self.view_zoom = max(0.5, min(5.0, self.view_zoom * factor))
        self._map_transform()
        nx, ny = self.world_to_canvas(*world)
        self.view_pan_x += cx - nx
        self.view_pan_y += cy - ny
        self._render_map()
        return 'break'

    def _start_map_drag(self, event):
        self._map_drag = (event.x, event.y, event.x, event.y, False)

    def _drag_map(self, event):
        if self._map_drag is None:
            return
        sx, sy, px, py, moved = self._map_drag
        moved = moved or math.hypot(event.x - sx, event.y - sy) > 4
        if moved:
            dx, dy = event.x - px, event.y - py
            self.view_pan_x += dx
            self.view_pan_y += dy
            self.map_offset_x += dx
            self.map_offset_y += dy
            self.canvas.move('all', dx, dy)
        self._map_drag = (sx, sy, event.x, event.y, moved)

    def _end_map_drag(self, event):
        if self._map_drag is not None and not self._map_drag[4]:
            self._select_map_point(event.x, event.y)
        self._map_drag = None

    def _select_map_point(self, cx, cy):
        candidates = []
        # 编号文字优先命中，便于选择坐标重合、但标签已错开的点位。
        for pid, (_oval, label) in self.pose_items.items():
            box = self.canvas.bbox(label)
            if box and box[0] <= cx <= box[2] and box[1] <= cy <= box[3]:
                candidates.append((-1, pid))
        for pid, entry in self.recorded.items():
            if 'x' in entry and 'y' in entry:
                px, py = self.world_to_canvas(entry['x'], entry['y'])
                distance = math.hypot(cx - px, cy - py)
                if distance <= 18:
                    candidates.append((distance, pid))
        if not candidates:
            return
        pid = min(candidates)[1]
        self.search_var.set('')
        self.missing_only_var.set(False)
        self._apply_filter()
        self.tree.selection_set(str(pid))
        self.tree.focus(str(pid))
        self.tree.see(str(pid))
        self._on_select()

    def _highlight_selected(self):
        self.canvas.delete('selected')
        selection = self.tree.selection()
        if not selection:
            return
        entry = self.recorded.get(int(selection[0]), {})
        if 'x' not in entry or 'y' not in entry:
            return
        px, py = self.world_to_canvas(entry['x'], entry['y'])
        self.canvas.create_oval(px - 14, py - 14, px + 14, py + 14,
                                outline='#db8c25', width=3, tags='selected')

    def world_to_canvas(self, x, y):
        col = (x - self.ox) / self.res
        row = self.map_img.height - (y - self.oy) / self.res
        return self.map_offset_x + col * self.scale, self.map_offset_y + row * self.scale

    def canvas_to_world(self, cx, cy):
        x = self.ox + ((cx - self.map_offset_x) / self.scale) * self.res
        y = self.oy + (self.map_img.height - (cy - self.map_offset_y) / self.scale) * self.res
        return x, y

    def _draw_map(self):
        self.canvas.delete('points')
        self.pose_items = {}
        label_boxes = []
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
            # 只错开文字，不移动实际点位；0/19 等同坐标点也能分别看到编号。
            for dx, dy in ((0, -17), (19, -6), (-19, -6), (0, 19),
                           (22, -23), (-22, -23), (22, 22), (-22, 22)):
                self.canvas.coords(label, px + dx, py + dy)
                box = self.canvas.bbox(label)
                if not any(box[0] < b[2] + 3 and box[2] > b[0] - 3 and
                           box[1] < b[3] + 3 and box[3] > b[1] - 3 for b in label_boxes):
                    break
            label_boxes.append(box)
            self.pose_items[pid] = (oval, label)
        self._highlight_selected()

    def _on_select(self, _event=None):
        selection = self.tree.selection()
        if selection and selection[0] in self._hidden_rows:
            # 记录后原回调会重新选中该项；若它已被“仅看未记录”隐藏，清掉隐形选择。
            self.tree.selection_remove(selection)
            selection = ()
        if not selection:
            self.detail_var.set('选择左侧点位，或点击地图中的已记录点')
            self.task_detail_var.set('识别任务：只读展示，不会修改任务朝向')
            self._highlight_selected()
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
        self.task_detail_var.set('识别任务（只读）：' + self._task_text(pid, names=True))
        self._highlight_selected()

    # ---------- 数据 ----------
    def _insert_row(self, pid, name, note, custom=False):
        entry = self.recorded.get(pid)
        status = '● 已录' if entry and 'x' in entry else '○ 待录'
        values = (status, str(pid), name, self._task_text(pid), note)
        self.tree.insert('', 'end', iid=str(pid), values=values,
                         tags=('recorded' if entry and 'x' in entry else 'missing',))

    def refresh(self):
        selection = self.tree.selection()
        for item in set(self.tree.get_children()) | self._hidden_rows:
            if self.tree.exists(item):
                self.tree.delete(item)
        self._hidden_rows.clear()

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
        self.summary_var.set('已录 %02d / %02d · 缺失 %d' % (done, total, total - done))
        self.progress.configure(maximum=max(1, total), value=done)
        self._apply_filter()

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
        # 重新录点时保留已有的识别任务/备注，避免覆盖掉
        old = self.recorded.get(pid)
        if old:
            for key in ('tasks', 'note'):
                if key in old:
                    entry[key] = old[key]
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
            self.connection_var.set('定位已连接')
            self.connection_badge.configure(background='#105c63', foreground='#a2eee1')
        else:
            self.pose_var.set('等待 map -> base_footprint ...')
            self.connection_var.set('等待定位 · 请检查导航 / TF')
            self.connection_badge.configure(background='#72501d', foreground='#ffe0a6')
            if self.arrow is not None:
                self.canvas.delete(self.arrow)
                self.arrow = None
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
