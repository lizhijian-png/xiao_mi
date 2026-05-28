"""第四段路线控制。

当前策略以坐标触发为主，视觉识别只作为提前确认和日志排查手段。
这样即使可乐瓶、小球、足球识别不稳定，机械狗也会在指定坐标执行动作，
不会因为没有识别到目标而一直向前走。
"""

import math
import os
import shutil
import subprocess
import time

import cv2
import numpy as np


# 赛道中心线坐标：三条竖向赛道分别对应第一段、第二段、第三段。
LEFT_LANE_X = -0.10
MID_LANE_X = 1.00
RIGHT_LANE_X = 2.10

# y=7.10 是第四段底部公共横向通道；y=8.90 是三条竖道之间换道用的上方横线。
LANE_SWITCH_Y = 7.10
ROUTE_SWITCH_Y = 8.90

# 第四段入口、可乐撞击偏移点、独木桥入口相关坐标。
START_POINT = (3.10, 7.10)
COLA_APPROACH_X = 0.00
BRIDGE_SWITCH_POINT = (RIGHT_LANE_X, LANE_SWITCH_Y)
BRIDGE_APPROACH_POINT = (3.75, LANE_SWITCH_Y)
BRIDGE_START = (3.75, 7.60)

# 左侧限高杆范围；右侧第三段固定在 y=10.00 开始低姿态。
LEFT_BAR = {"x": LEFT_LANE_X, "y_min": 8.95, "y_max": 9.95}
RIGHT_LOW_START_Y = 10.00

# 目标物坐标和动作结束坐标。
# 可乐：到 y=11.10 播报，到 y=11.15 开始倒退。
# 小球：到 y=11.00 播报，到 y=11.05 开始倒退。
# 足球：到 y=11.20 播报，低姿态走到 y=11.40 后开始回退。
COLA = {"x": LEFT_LANE_X, "y": 11.10, "backup_y": 11.15}
ORANGE_BALL = {"x": 0.94, "y": 11.00, "backup_y": 11.05}
FOOTBALL = {"x": RIGHT_LANE_X, "announce_y": 11.20, "backup_y": 11.25}

# 限高杆检测和动作距离参数。
LEFT_BAR_DETECT_UP_Y = LEFT_BAR["y_min"] - 0.20
LEFT_BAR_DETECT_DOWN_Y = LEFT_BAR["y_max"] + 0.20
BAR_CLEAR_MARGIN = 0.30
TARGET_BACKUP_DIST = 0.10


# 步态编号和朝向角度。
# 1: 普通前进；5: 低姿态/蹲下通过限高杆；6: 后退；7/8: 左/右平移校正。
LOW_BAR_GAIT = 5
FOOTBALL_GAIT = 28
HEADING_EAST = 0
HEADING_NORTH = 90
HEADING_SOUTH = 270
FAST_DEG = 18
SLOW_DEG = 6
XY_TOL = 0.08


LOG_PATH = os.path.abspath(
    os.path.join(os.path.dirname(__file__), "..", "log", "segment4_log.txt")
)


# 路线状态名。状态机按左侧可乐 -> 中间小球 -> 右侧足球 -> 独木桥的顺序推进。
S = {
    "TO_START": "S4_TO_START",
    "LEFT_TO_LANE": "S4_LEFT_TO_LANE",
    "LEFT_TURN_UP": "S4_LEFT_TURN_UP",
    "LEFT_ALIGN_UP": "S4_LEFT_ALIGN_UP",
    "LEFT_BAR_UP": "S4_LEFT_BAR_UP",
    "LEFT_ALIGN_COLA": "S4_LEFT_ALIGN_COLA",
    "LEFT_FIND_COLA": "S4_LEFT_FIND_COLA",
    "LEFT_BACKUP_COLA": "S4_LEFT_BACKUP_COLA",
    "LEFT_TURN_BACK": "S4_LEFT_TURN_BACK",
    "LEFT_ALIGN_DOWN": "S4_LEFT_ALIGN_DOWN",
    "LEFT_BAR_DOWN": "S4_LEFT_BAR_DOWN",
    "LEFT_TURN_EAST": "S4_LEFT_TURN_EAST",
    "MID_TO_LANE": "S4_MID_TO_LANE",
    "MID_TURN_UP": "S4_MID_TURN_UP",
    "MID_ALIGN_UP": "S4_MID_ALIGN_UP",
    "MID_FIND_ORANGE": "S4_MID_FIND_ORANGE",
    "MID_BACKUP_ORANGE": "S4_MID_BACKUP_ORANGE",
    "MID_TURN_BACK": "S4_MID_TURN_BACK",
    "MID_ALIGN_DOWN": "S4_MID_ALIGN_DOWN",
    "MID_CHECK_BLOCK": "S4_MID_CHECK_BLOCK",
    "MID_RETURN_Y": "S4_MID_RETURN_Y",
    "MID_TURN_EAST": "S4_MID_TURN_EAST",
    "RIGHT_TO_LANE": "S4_RIGHT_TO_LANE",
    "RIGHT_TURN_UP": "S4_RIGHT_TURN_UP",
    "RIGHT_ALIGN_UP": "S4_RIGHT_ALIGN_UP",
    "RIGHT_TO_LOW_START": "S4_RIGHT_TO_LOW_START",
    "RIGHT_LOW_FORWARD": "S4_RIGHT_LOW_FORWARD",
    "RIGHT_ALIGN_AFTER_BALL": "S4_RIGHT_ALIGN_AFTER_BALL",
    "RIGHT_BACKUP_LOW": "S4_RIGHT_BACKUP_LOW",
    "RIGHT_STAND": "S4_RIGHT_STAND",
    "RIGHT_TURN_DOWN": "S4_RIGHT_TURN_DOWN",
    "RIGHT_ALIGN_DOWN": "S4_RIGHT_ALIGN_DOWN",
    "RIGHT_RETURN_Y": "S4_RIGHT_RETURN_Y",
    "RIGHT_TURN_EAST": "S4_RIGHT_TURN_EAST",
    "BRIDGE_TO_X": "S4_BRIDGE_TO_X",
    "BRIDGE_TURN_UP": "S4_BRIDGE_TURN_UP",
    "BRIDGE_APPROACH": "S4_BRIDGE_APPROACH",
    "DONE": "S4_DONE",
}

LOW_STATES = {
    S["LEFT_BAR_UP"],
    S["LEFT_BAR_DOWN"],
    S["RIGHT_LOW_FORWARD"],
    S["RIGHT_ALIGN_AFTER_BALL"],
    S["RIGHT_BACKUP_LOW"],
}


# 全局状态。_obstacle_idx/_target_idx 仅用于兼容 test4.py 的打印线程。
_state = S["TO_START"]
_obstacle_idx = 0
_target_idx = 0
_motion_start = None
_announced = set()
_last_log_time = 0.0
_last_log_signature = None
_right_low_start = None


def _log_event(event, **fields):
    """追加关键状态日志；失败时静默，避免影响比赛控制。"""
    try:
        os.makedirs(os.path.dirname(LOG_PATH), exist_ok=True)
        stamp = time.strftime("%Y-%m-%d %H:%M:%S")
        detail = " ".join(f"{k}={v}" for k, v in fields.items())
        with open(LOG_PATH, "a", encoding="utf-8") as f:
            f.write(f"{stamp} {event} {detail}\n")
    except Exception:
        pass


def _reset_log_file():
    """每次 reset 时清空本轮第四段日志，便于只看当前实验。"""
    try:
        os.makedirs(os.path.dirname(LOG_PATH), exist_ok=True)
        with open(LOG_PATH, "w", encoding="utf-8") as f:
            f.write("")
    except Exception:
        pass


def _fmt_pos(position):
    return f"({position[0]:.3f},{position[1]:.3f},{position[2]:.3f})"


def _set_state(new_state, reason, position=None):
    """切换状态并记录原因。"""
    global _state
    old_state = _state
    _state = new_state
    _log_event(
        "STATE",
        old=old_state,
        new=new_state,
        reason=reason,
        pos=_fmt_pos(position) if position is not None else "-",
    )


def _return_step(step, reason, position=None, rpy=None):
    """返回步态编号前统一记录日志。"""
    _log_event(
        "STEP",
        step=step,
        reason=reason,
        state=_state,
        pos=_fmt_pos(position) if position is not None else "-",
        rpy=f"{rpy:.1f}" if rpy is not None else "-",
    )
    return step


def reset_segment4():
    """重置第四段状态机，test.py/test4.py 开始第四段前会调用。"""
    global _state, _obstacle_idx, _target_idx, _motion_start, _announced
    global _last_log_time, _last_log_signature, _right_low_start

    _reset_log_file()
    _state = S["TO_START"]
    _obstacle_idx = 0
    _target_idx = 0
    _motion_start = None
    _announced = set()
    _last_log_time = 0.0
    _last_log_signature = None
    _right_low_start = None
    _log_event("RESET", state=_state)


def _norm(angle):
    while angle > 180:
        angle -= 360
    while angle <= -180:
        angle += 360
    return angle


def _turn_to(rpy, target_hdg):
    """根据当前航向 rpy 调整到目标角度；对准后返回 1 表示可前进。"""
    d = _norm(rpy - (target_hdg % 360))
    if d > FAST_DEG:
        return 15
    if d > SLOW_DEG:
        return 3
    if d < -FAST_DEG:
        return 14
    if d < -SLOW_DEG:
        return 2
    return 1


def _distance(a, b):
    return math.hypot(a[0] - b[0], a[1] - b[1])


def _dist_from_start(position):
    if _motion_start is None:
        return 0.0
    return _distance(position, _motion_start)


def _motion_start_reset(position):
    """记录当前动作起点，用于计算倒退距离。"""
    global _motion_start
    _motion_start = [position[0], position[1]]


def _speak(text):
    """语音播报；没有 TTS 工具时至少打印文本。"""
    print(f"语音播报：识别到{text}")
    cmd = shutil.which("spd-say") or shutil.which("espeak")
    if cmd:
        try:
            subprocess.Popen([cmd, f"识别到{text}"])
        except Exception:
            pass


def _announce_once(key, text):
    """同一目标只播报一次，避免循环内反复播报。"""
    if key in _announced:
        return
    _announced.add(key)
    _speak(text)
    _log_event("ANNOUNCE", key=key, text=text)


def _forward_step(rpy, heading, gait=1):
    """先对准 heading，已对准则执行指定前进步态。"""
    step = _turn_to(rpy, heading)
    return step if step != 1 else gait


def _turn_state(rpy, heading, next_state, reason, position):
    """原地转向到指定朝向，完成后进入下一个状态。"""
    step = _turn_to(rpy, heading)
    if step == 1:
        _set_state(next_state, reason, position)
        return _return_step(0, reason, position, rpy)
    return _return_step(step, f"{reason}_turn", position, rpy)


def _go_x(position, rpy, target_x, heading, next_state, reason):
    """沿 x 方向走到目标 x。"""
    x = position[0]
    reached = x <= target_x + XY_TOL if heading == 180 else x >= target_x - XY_TOL
    if reached:
        _set_state(next_state, reason, position)
        return _return_step(0, reason, position, rpy)
    return _return_step(_forward_step(rpy, heading), f"{reason}_move_x", position, rpy)


def _go_y(position, rpy, target_y, heading, next_state, reason, gait=1):
    """沿 y 方向走到目标 y。"""
    y = position[1]
    reached = y >= target_y - XY_TOL if heading == HEADING_NORTH else y <= target_y + XY_TOL
    if reached:
        _set_state(next_state, reason, position)
        return _return_step(0, reason, position, rpy)
    return _return_step(_forward_step(rpy, heading, gait), f"{reason}_move_y", position, rpy)


def _adjust_x(position, rpy, lane_x, heading, next_state, reason):
    """保持当前朝向，用左右平移步态把 x 调整到赛道中心线。"""
    x = position[0]
    step = _turn_to(rpy, heading)
    if step != 1:
        return _return_step(step, f"{reason}_align_heading", position, rpy)

    x_err = x - lane_x
    _log_event("ROUTE_X_ADJUST", lane_x=f"{lane_x:.2f}", x_err=f"{x_err:.3f}", pos=_fmt_pos(position))
    if abs(x_err) <= XY_TOL:
        _set_state(next_state, reason, position)
        return _return_step(0, reason, position, rpy)

    if heading == HEADING_NORTH:
        lateral_step = 7 if x_err > 0 else 8
    elif heading == HEADING_SOUTH:
        lateral_step = 8 if x_err > 0 else 7
    else:
        lateral_step = 7 if x_err > 0 else 8
    return _return_step(lateral_step, f"{reason}_strafe_x", position, rpy)


def _backup_to_distance(position, next_state, reason):
    """后退固定距离后切换状态。"""
    if _dist_from_start(position) >= TARGET_BACKUP_DIST:
        _set_state(next_state, reason, position)
        _motion_start_reset(position)
        return 0
    return 6


def _central_roi(frame):
    h, w = frame.shape[:2]
    return frame[h // 5: 4 * h // 5, w // 4: 3 * w // 4]


def _detect_cola(frame):
    """检测可乐瓶：兼容撕掉包装后偏深色/棕色的瓶身。"""
    if frame is None:
        return False
    roi = _central_roi(frame)
    hsv = cv2.cvtColor(roi, cv2.COLOR_BGR2HSV)
    dark = cv2.inRange(hsv, np.array([0, 0, 0]), np.array([180, 255, 95]))
    brown = cv2.inRange(hsv, np.array([5, 35, 20]), np.array([35, 255, 140]))
    mask = cv2.bitwise_or(dark, brown)
    mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, np.ones((5, 5), np.uint8))
    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    for cnt in contours:
        area = cv2.contourArea(cnt)
        if area < 140:
            continue
        _, _, bw, bh = cv2.boundingRect(cnt)
        if bw and bh and (bh / bw > 1.15 or area > 650):
            return True
    return False


def _detect_orange_ball(frame):
    """检测橙色小球：HSV 橙色阈值 + 圆度过滤。"""
    if frame is None:
        return False
    roi = _central_roi(frame)
    hsv = cv2.cvtColor(roi, cv2.COLOR_BGR2HSV)
    mask = cv2.inRange(hsv, np.array([8, 100, 80]), np.array([28, 255, 255]))
    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    contours = [c for c in contours if cv2.contourArea(c) > 350]
    if not contours:
        return False
    cnt = max(contours, key=cv2.contourArea)
    area = cv2.contourArea(cnt)
    perimeter = cv2.arcLength(cnt, True)
    return perimeter > 0 and 4 * math.pi * area / (perimeter * perimeter) > 0.45


def _detect_football(frame):
    """检测足球：黑白区域 + 圆度过滤。"""
    if frame is None:
        return False
    roi = _central_roi(frame)
    gray = cv2.cvtColor(roi, cv2.COLOR_BGR2GRAY)
    dark = cv2.inRange(gray, 0, 80)
    light = cv2.inRange(gray, 170, 255)
    mask = cv2.bitwise_or(dark, light)
    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    contours = [c for c in contours if cv2.contourArea(c) > 250]
    if not contours:
        return False
    cnt = max(contours, key=cv2.contourArea)
    area = cv2.contourArea(cnt)
    perimeter = cv2.arcLength(cnt, True)
    return perimeter > 0 and 4 * math.pi * area / (perimeter * perimeter) > 0.35


def _detect_obstacle(frame):
    """检测中间不可跨越障碍，主要用于第二段返程时播报。"""
    if frame is None:
        return False
    roi = _central_roi(frame)
    hsv = cv2.cvtColor(roi, cv2.COLOR_BGR2HSV)
    blue = cv2.inRange(hsv, np.array([90, 80, 60]), np.array([130, 255, 255]))
    red1 = cv2.inRange(hsv, np.array([0, 80, 60]), np.array([10, 255, 255]))
    red2 = cv2.inRange(hsv, np.array([170, 80, 60]), np.array([180, 255, 255]))
    mask = cv2.bitwise_or(blue, cv2.bitwise_or(red1, red2))
    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    return any(cv2.contourArea(c) > 300 for c in contours)


def _detect_limit_bar_ahead(frame):
    """检测前方近距离红色限高杆，作为坐标触发的补充。"""
    if frame is None:
        return False
    h, w = frame.shape[:2]
    roi = frame[h // 3: 5 * h // 6, w // 5: 4 * w // 5]
    hsv = cv2.cvtColor(roi, cv2.COLOR_BGR2HSV)
    red1 = cv2.inRange(hsv, np.array([0, 90, 70]), np.array([10, 255, 255]))
    red2 = cv2.inRange(hsv, np.array([170, 90, 70]), np.array([180, 255, 255]))
    mask = cv2.bitwise_or(red1, red2)
    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    for cnt in contours:
        if cv2.contourArea(cnt) < 250:
            continue
        _, y, bw, bh = cv2.boundingRect(cnt)
        if bw > bh * 3 and bw > roi.shape[1] * 0.18 and y > roi.shape[0] * 0.15:
            return True
    return False


def _detect_target(frame, kind):
    """按目标类型调用对应视觉检测函数。"""
    if kind == "cola":
        return _detect_cola(frame)
    if kind == "orange_ball":
        return _detect_orange_ball(frame)
    if kind == "football":
        return _detect_football(frame)
    return False


def _route(position, rpy, frame):
    """第四段主路线状态机。"""
    global _motion_start, _right_low_start

    x, y, _ = position
    _log_event("ROUTE", state=_state, pos=_fmt_pos(position), rpy=f"{rpy:.1f}")

    # 入口：先到第四段起点，再沿 y=7.10m 的底部通道进入最左侧赛道。
    if _state == S["TO_START"]:
        if abs(y - START_POINT[1]) > XY_TOL:
            heading = HEADING_NORTH if START_POINT[1] > y else HEADING_SOUTH
            return _return_step(_forward_step(rpy, heading), "route_to_start_fix_y", position, rpy)
        return _go_x(position, rpy, START_POINT[0], 180, S["LEFT_TO_LANE"], "route_start_reached")

    # 第一段：转入 x=-0.10m 赛道，低姿态过限高杆，偏到 x=0.00m 撞可乐。
    if _state == S["LEFT_TO_LANE"]:
        return _go_x(position, rpy, LEFT_LANE_X, 180, S["LEFT_TURN_UP"], "route_left_lane_reached")
    if _state == S["LEFT_TURN_UP"]:
        return _turn_state(rpy, HEADING_NORTH, S["LEFT_ALIGN_UP"], "route_left_turn_up_done", position)
    if _state == S["LEFT_ALIGN_UP"]:
        return _adjust_x(position, rpy, LEFT_LANE_X, HEADING_NORTH, S["LEFT_BAR_UP"], "route_left_align_up_done")
    if _state == S["LEFT_BAR_UP"]:
        if _motion_start is not None:
            if y >= LEFT_BAR["y_max"] + BAR_CLEAR_MARGIN:
                _motion_start = None
                _set_state(S["LEFT_ALIGN_COLA"], "route_left_bar_up_clear", position)
                return _return_step(0, "route_left_bar_up_clear", position, rpy)
            return _return_step(_forward_step(rpy, HEADING_NORTH, LOW_BAR_GAIT), "route_left_bar_up_keep_low", position, rpy)
        if y >= LEFT_BAR_DETECT_UP_Y or _detect_limit_bar_ahead(frame):
            _announce_once("left_bar_up", "限高杆")
            _motion_start_reset(position)
            return _return_step(LOW_BAR_GAIT, "route_left_bar_up_enter_low", position, rpy)
        return _return_step(_forward_step(rpy, HEADING_NORTH), "route_left_bar_up_forward", position, rpy)

    if _state == S["LEFT_ALIGN_COLA"]:
        return _adjust_x(position, rpy, COLA_APPROACH_X, HEADING_NORTH, S["LEFT_FIND_COLA"], "route_left_align_cola_done")
    if _state == S["LEFT_FIND_COLA"]:
        detected = _detect_target(frame, "cola")
        _log_event("ROUTE_TARGET_SCAN", target="cola", detected=detected, pos=_fmt_pos(position))
        # 坐标触发兜底：到 y=11.10m 播报可乐，继续到 y=11.15m 后倒退。
        if detected or y >= COLA["y"]:
            _announce_once("cola", "可乐瓶")
        if y >= COLA["backup_y"]:
            _motion_start_reset(position)
            _set_state(S["LEFT_BACKUP_COLA"], "route_cola_backup_start", position)
            return _return_step(0, "route_cola_backup_start", position, rpy)
        return _return_step(_forward_step(rpy, HEADING_NORTH), "route_left_find_cola_forward", position, rpy)
    if _state == S["LEFT_BACKUP_COLA"]:
        step = _backup_to_distance(position, S["LEFT_TURN_BACK"], "route_cola_backup_done")
        return _return_step(step, "route_cola_backup", position, rpy)
    if _state == S["LEFT_TURN_BACK"]:
        return _turn_state(rpy, HEADING_SOUTH, S["LEFT_ALIGN_DOWN"], "route_left_turn_back_done", position)
    if _state == S["LEFT_ALIGN_DOWN"]:
        return _adjust_x(position, rpy, LEFT_LANE_X, HEADING_SOUTH, S["LEFT_BAR_DOWN"], "route_left_align_down_done")
    if _state == S["LEFT_BAR_DOWN"]:
        if y <= ROUTE_SWITCH_Y + XY_TOL:
            _motion_start = None
            _set_state(S["LEFT_TURN_EAST"], "route_left_return_y_reached", position)
            return _return_step(0, "route_left_return_y_reached", position, rpy)
        if _motion_start is not None or y <= LEFT_BAR_DETECT_DOWN_Y:
            _motion_start = _motion_start or [x, y]
            _announce_once("left_bar_down", "限高杆")
            return _return_step(_forward_step(rpy, HEADING_SOUTH, LOW_BAR_GAIT), "route_left_bar_down_keep_low", position, rpy)
        return _return_step(_forward_step(rpy, HEADING_SOUTH), "route_left_bar_down_forward", position, rpy)
    if _state == S["LEFT_TURN_EAST"]:
        return _turn_state(rpy, HEADING_EAST, S["MID_TO_LANE"], "route_left_turn_east_done", position)

    # 第二段：先走到 y=8.90m 的换道线，再进入 x=1.00m 赛道寻找橙色小球。
    if _state == S["MID_TO_LANE"]:
        return _go_x(position, rpy, MID_LANE_X, HEADING_EAST, S["MID_TURN_UP"], "route_mid_lane_reached")
    if _state == S["MID_TURN_UP"]:
        return _turn_state(rpy, HEADING_NORTH, S["MID_ALIGN_UP"], "route_mid_turn_up_done", position)
    if _state == S["MID_ALIGN_UP"]:
        return _adjust_x(position, rpy, MID_LANE_X, HEADING_NORTH, S["MID_FIND_ORANGE"], "route_mid_align_up_done")
    if _state == S["MID_FIND_ORANGE"]:
        detected = _detect_target(frame, "orange_ball")
        _log_event("ROUTE_TARGET_SCAN", target="orange_ball", detected=detected, pos=_fmt_pos(position))
        # 到 y=11.00m 播报小球，继续到 y=11.05m 后倒退，避免目标卡住转身。
        if detected or y >= ORANGE_BALL["y"]:
            _announce_once("orange_ball", "橙色小球")
        if y >= ORANGE_BALL["backup_y"]:
            _motion_start_reset(position)
            _set_state(S["MID_BACKUP_ORANGE"], "route_orange_backup_start", position)
            return _return_step(0, "route_orange_backup_start", position, rpy)
        return _return_step(_forward_step(rpy, HEADING_NORTH), "route_mid_find_orange_forward", position, rpy)
    if _state == S["MID_BACKUP_ORANGE"]:
        step = _backup_to_distance(position, S["MID_TURN_BACK"], "route_orange_backup_done")
        return _return_step(step, "route_orange_backup", position, rpy)
    if _state == S["MID_TURN_BACK"]:
        return _turn_state(rpy, HEADING_SOUTH, S["MID_ALIGN_DOWN"], "route_mid_turn_back_done", position)
    if _state == S["MID_ALIGN_DOWN"]:
        return _adjust_x(position, rpy, MID_LANE_X, HEADING_SOUTH, S["MID_CHECK_BLOCK"], "route_mid_align_down_done")
    if _state == S["MID_CHECK_BLOCK"]:
        if y <= 9.95 or _detect_obstacle(frame):
            _announce_once("block", "无法跨越障碍")
            _set_state(S["MID_RETURN_Y"], "route_mid_block_checked", position)
            return _return_step(0, "route_mid_block_checked", position, rpy)
        return _return_step(_forward_step(rpy, HEADING_SOUTH), "route_mid_check_block_forward", position, rpy)
    if _state == S["MID_RETURN_Y"]:
        return _go_y(position, rpy, ROUTE_SWITCH_Y, HEADING_SOUTH, S["MID_TURN_EAST"], "route_mid_return_y_reached")
    if _state == S["MID_TURN_EAST"]:
        return _turn_state(rpy, HEADING_EAST, S["RIGHT_TO_LANE"], "route_mid_turn_east_done", position)

    # 第三段：进入 x=2.10m 赛道，y=10.00m 后一直蹲着前进并完成足球射门。
    if _state == S["RIGHT_TO_LANE"]:
        return _go_x(position, rpy, RIGHT_LANE_X, HEADING_EAST, S["RIGHT_TURN_UP"], "route_right_lane_reached")
    if _state == S["RIGHT_TURN_UP"]:
        return _turn_state(rpy, HEADING_NORTH, S["RIGHT_ALIGN_UP"], "route_right_turn_up_done", position)
    if _state == S["RIGHT_ALIGN_UP"]:
        return _adjust_x(position, rpy, RIGHT_LANE_X, HEADING_NORTH, S["RIGHT_TO_LOW_START"], "route_right_align_up_done")
    if _state == S["RIGHT_TO_LOW_START"]:
        # 从 y=10.00m 开始低姿态，后续直到倒退回该位置才站起。
        if y >= RIGHT_LOW_START_Y:
            _right_low_start = [RIGHT_LANE_X, RIGHT_LOW_START_Y]
            _announce_once("right_bar", "限高杆")
            _set_state(S["RIGHT_LOW_FORWARD"], "route_right_enter_low_at_10m", position)
            return _return_step(LOW_BAR_GAIT, "route_right_enter_low_at_10m", position, rpy)
        return _return_step(_forward_step(rpy, HEADING_NORTH), "route_right_forward_to_low_start", position, rpy)
    if _state == S["RIGHT_LOW_FORWARD"]:
        detected = _detect_target(frame, "football")
        _log_event("ROUTE_TARGET_SCAN", target="football", detected=detected, pos=_fmt_pos(position))
        # 到 y=11.20m 播报足球，继续推到 y=11.40m 后低姿态倒退。
        if detected or y >= FOOTBALL["announce_y"]:
            _announce_once("football", "足球")
        if y >= FOOTBALL["backup_y"]:
            _set_state(S["RIGHT_ALIGN_AFTER_BALL"], "route_football_reach_11_4", position)
            return _return_step(0, "route_football_reach_11_4", position, rpy)
        return _return_step(_forward_step(rpy, HEADING_NORTH, LOW_BAR_GAIT), "route_right_low_forward", position, rpy)
    if _state == S["RIGHT_ALIGN_AFTER_BALL"]:
        return _adjust_x(position, rpy, RIGHT_LANE_X, HEADING_NORTH, S["RIGHT_BACKUP_LOW"], "route_right_align_after_ball_done")
    if _state == S["RIGHT_BACKUP_LOW"]:
        if _right_low_start is None or y <= _right_low_start[1] + XY_TOL:
            _set_state(S["RIGHT_STAND"], "route_right_backup_to_10m_done", position)
            return _return_step(0, "route_right_backup_to_10m_done", position, rpy)
        return _return_step(6, "route_right_backup_low", position, rpy)
    if _state == S["RIGHT_STAND"]:
        _motion_start = None
        _set_state(S["RIGHT_TURN_DOWN"], "route_right_stand_done", position)
        return _return_step(0, "route_right_stand_done", position, rpy)
    if _state == S["RIGHT_TURN_DOWN"]:
        return _turn_state(rpy, HEADING_SOUTH, S["RIGHT_ALIGN_DOWN"], "route_right_turn_180_done", position)
    if _state == S["RIGHT_ALIGN_DOWN"]:
        _right_low_start = None
        return _adjust_x(position, rpy, RIGHT_LANE_X, HEADING_SOUTH, S["RIGHT_RETURN_Y"], "route_right_align_down_done")

    # 收尾：回到底部通道，再走到独木桥前，让前脚搭上独木桥后结束第四段。
    if _state == S["RIGHT_RETURN_Y"]:
        return _go_y(position, rpy, BRIDGE_SWITCH_POINT[1], HEADING_SOUTH, S["RIGHT_TURN_EAST"], "route_right_return_switch_y_reached")
    if _state == S["RIGHT_TURN_EAST"]:
        return _turn_state(rpy, HEADING_EAST, S["BRIDGE_TO_X"], "route_right_turn_east_done", position)
    if _state == S["BRIDGE_TO_X"]:
        return _go_x(position, rpy, BRIDGE_APPROACH_POINT[0], HEADING_EAST, S["BRIDGE_TURN_UP"], "route_bridge_x_reached")
    if _state == S["BRIDGE_TURN_UP"]:
        return _turn_state(rpy, HEADING_NORTH, S["BRIDGE_APPROACH"], "route_bridge_turn_up_done", position)
    if _state == S["BRIDGE_APPROACH"]:
        if y >= BRIDGE_START[1] - XY_TOL:
            _set_state(S["DONE"], "route_bridge_front_feet_on", position)
            return _return_step(-1, "segment4_done_bridge_front_feet_on", position, rpy)
        return _return_step(_forward_step(rpy, HEADING_NORTH), "route_bridge_approach_forward", position, rpy)

    _set_state(S["DONE"], "route_unknown_state_done", position)
    return _return_step(-1, "route_unknown_state_done", position, rpy)


def segment4_control(position, gait_mode, rpy, frame=None):
    """对外接口：返回当前应执行的步态编号，返回 -1 表示第四段完成。"""
    global _last_log_time, _last_log_signature

    x, y, _ = position
    gait, mode = gait_mode
    now = time.time()
    signature = (_state, round(x, 1), round(y, 1), int(rpy // 10))
    if signature != _last_log_signature or now - _last_log_time >= 0.5:
        _log_event("TICK", state=_state, pos=_fmt_pos(position), rpy=f"{rpy:.1f}", gait=gait, mode=mode)
        _last_log_signature = signature
        _last_log_time = now

    if _state == S["DONE"]:
        return _return_step(-1, "segment4_done", position, rpy)

    switching_gait = (gait == 0 and mode == 0) or (gait == 1 and mode == 9)
    if switching_gait:
        if _state in LOW_STATES:
            return _return_step(LOW_BAR_GAIT, "gait_switch_keep_low", position, rpy)
        return _return_step(0, "gait_switch_wait", position, rpy)

    return _route(position, rpy, frame)
