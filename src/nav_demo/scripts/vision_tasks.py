#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""点位识别任务 → 统一视觉服务 task / 裁判 sub_id 的映射。"""

# 识别任务名（inspection_waypoints.yaml 里的 tasks[].name）→ (task, sub_id)
TASKS = {
    '红绿灯 1 识别': ('light', '红绿灯1'),
    '红绿灯 2 识别': ('light', '红绿灯2'),
    'A 街区北方识别': ('crowd', 'A街区'),
    'A 街区西方识别': ('crowd', 'A街区'),
    'A 街区南方识别': ('crowd', 'A街区'),
    'B 街区北方识别': ('crowd', 'B街区'),
    'B 街区西方识别': ('crowd', 'B街区'),
    'B 街区东方识别': ('crowd', 'B街区'),
    '禁止通行指示牌识别': ('sign', 'A街区'),
    '楼宇 A 识别': ('fire', '楼宇A'),
    '楼宇 B 识别': ('fire', '楼宇B'),
    '楼宇 C 识别': ('fire', '楼宇C'),
    '楼宇 D 识别': ('temperature', '楼宇D'),
    '站房识别': ('meter', '站房'),
    '1 号停车位识别': ('plate', '1号停车位'),
    '2 号停车位识别': ('plate', '2号停车位'),
    '3 号停车位识别': ('plate', '3号停车位'),
    '垃圾桶识别': ('trash', '垃圾桶'),
    '电动车识别起始': ('ebike', '停车区'),
}

LIGHT = 'light'
JUDGE_TASKS = ('crowd', 'sign', 'trash', 'fire', 'plate', 'temperature', 'meter', 'ebike')
GREEN_WORDS = ('绿', 'green', 'Green', 'GREEN')


def lookup(name):
    """返回 (task, sub_id)，未知任务返回 None。"""
    return TASKS.get(name)


def is_green(text):
    text = text or ''
    return any(word in text for word in GREEN_WORDS)
