"""
第四赛段：深隧寻珍。

要求：
- 识别限高杆，播报“识别到限高杆”，并从杆下通过，全程不能碰撞限高杆。
- 识别无法跨越障碍，播报“识别到无法跨越障碍”，并绕过障碍物。
- 识别可乐瓶、橙色小球、足球，播报后分别完成撞倒、晃动、踢入球门。
- 最后前腿足底碰到独木桥起始端，第四赛段结束。

当前坐标使用题目给定全局坐标系：原点在赛段一石板区，x 轴向右，y 轴向上。
"""

import math
import os
import shutil
import subprocess
import time

import cv2
import numpy as np


# 第四赛段全局坐标换算：
# 三条竖向通道之间的黄色分隔线宽 10cm，需要计入通道中心坐标。
LEFT_LANE_X = -0.10
MID_LANE_X = 1.00
RIGHT_LANE_X = 2.10
BRIDGE_LANE_X = 3.15

# 第四段底部黄线内侧 y=6.60m，公共横向通道中心约为 y=7.10m。
LANE_SWITCH_Y = 7.10

TARGETS = [
    {
        "name": "可乐瓶",
        "kind": "cola",
        "x": LEFT_LANE_X,
        "y": 11.10,
        "y_min": 10.95,
        "y_max": 11.25,
        "strike": 0.1,
        "gait": 1,
    },
    {
        "name": "橙色小球",
        "kind": "orange_ball",
        "x": 0.94,
        "y": 11.00,
        "y_min": 10.90,
        "y_max": 11.10,
        "strike": 0.05,
        "gait": 1,
    },
    {
        "name": "足球",
        "kind": "football",
        "x": RIGHT_LANE_X,
        "y": 10.80,
        "y_min": 10.65,
        "y_max": 10.95,
        "strike": 0.40,
        "gait": 28,
    },
]

FOOTBALL_GOAL = (RIGHT_LANE_X, 11.20)

OBSTACLES = [
    {
        "name": "左侧限高杆",
        "speak": "限高杆",
        "type": "bar",
        "x": LEFT_LANE_X,
        "y_min": 8.95,
        "y_max": 9.95,
    },
    {
        "name": "中部无法跨越障碍",
        "speak": "无法跨越障碍",
        "type": "block",
        "x": MID_LANE_X,
        "y_min": 8.45,
        "y_max": 9.45,
    },
    {
        "name": "右侧限高杆",
        "speak": "限高杆",
        "type": "bar",
        "x": RIGHT_LANE_X,
        "y_min": 9.20,
        "y_max": 10.20,
    },
]

BRIDGE_SWITCH_POINT = (RIGHT_LANE_X, LANE_SWITCH_Y)
BRIDGE_APPROACH_POINT = (3.75, LANE_SWITCH_Y)
BRIDGE_START = (3.75, 7.60)

HEADING_FRONT = 90
FAST_DEG = 18
SLOW_DEG = 6
XY_TOL = 0.08
SAME_LANE_TOL = 0.35
SCAN_LIMIT = 20
SCAN_STEP = 1
BLOCK_SCAN_LIMIT = 3

BAR_APPROACH_DIST = 0.45
BAR_DETECT_DIST = 0.20
BAR_PASS_DIST = 1.10
BAR_CLEAR_MARGIN = 0.30
BLOCK_APPROACH_DIST = 0.45
BLOCK_CLEAR_DIST = 0.75

LOW_BAR_GAIT = 5
LOW_BAR_DETECT_CLEAR_DIST = 1.00
BAR_LANE_TOL = 0.35
BAR_FORWARD_CLEAR = 0.35

_ST_GOTO_OBSTACLE = "S4_GOTO_OBSTACLE"
_ST_SCAN_OBSTACLE = "S4_SCAN_OBSTACLE"
_ST_PASS_LOW_BAR = "S4_PASS_LOW_BAR"
_ST_BYPASS_BLOCK = "S4_BYPASS_BLOCK"
_ST_GOTO_TARGET = "S4_GOTO_TARGET"
_ST_SCAN_TARGET = "S4_SCAN_TARGET"
_ST_STRIKE = "S4_STRIKE"
_ST_SHAKE = "S4_SHAKE"
_ST_BACKUP = "S4_BACKUP"
_ST_GOTO_BRIDGE = "S4_GOTO_BRIDGE"
_ST_DONE = "S4_DONE"

_ST_ROUTE_TO_START = "S4_ROUTE_TO_START"
_ST_ROUTE_LEFT_TO_LANE = "S4_ROUTE_LEFT_TO_LANE"
_ST_ROUTE_LEFT_TURN_UP = "S4_ROUTE_LEFT_TURN_UP"
_ST_ROUTE_LEFT_ALIGN_UP = "S4_ROUTE_LEFT_ALIGN_UP"
_ST_ROUTE_LEFT_BAR_UP = "S4_ROUTE_LEFT_BAR_UP"
_ST_ROUTE_LEFT_ALIGN_AFTER_BAR = "S4_ROUTE_LEFT_ALIGN_AFTER_BAR"
_ST_ROUTE_LEFT_FIND_COLA = "S4_ROUTE_LEFT_FIND_COLA"
_ST_ROUTE_LEFT_STRIKE_COLA = "S4_ROUTE_LEFT_STRIKE_COLA"
_ST_ROUTE_LEFT_BACKUP_AFTER_COLA = "S4_ROUTE_LEFT_BACKUP_AFTER_COLA"
_ST_ROUTE_LEFT_TURN_BACK = "S4_ROUTE_LEFT_TURN_BACK"
_ST_ROUTE_LEFT_ALIGN_DOWN = "S4_ROUTE_LEFT_ALIGN_DOWN"
_ST_ROUTE_LEFT_BAR_DOWN = "S4_ROUTE_LEFT_BAR_DOWN"
_ST_ROUTE_LEFT_RETURN_Y = "S4_ROUTE_LEFT_RETURN_Y"
_ST_ROUTE_LEFT_TURN_EAST = "S4_ROUTE_LEFT_TURN_EAST"
_ST_ROUTE_TO_MID = "S4_ROUTE_TO_MID"
_ST_ROUTE_MID_TURN_UP = "S4_ROUTE_MID_TURN_UP"
_ST_ROUTE_MID_ALIGN_UP = "S4_ROUTE_MID_ALIGN_UP"
_ST_ROUTE_MID_FIND_ORANGE = "S4_ROUTE_MID_FIND_ORANGE"
_ST_ROUTE_MID_SHAKE_ORANGE = "S4_ROUTE_MID_SHAKE_ORANGE"
_ST_ROUTE_MID_BACKUP_AFTER_ORANGE = "S4_ROUTE_MID_BACKUP_AFTER_ORANGE"
_ST_ROUTE_MID_TURN_BACK = "S4_ROUTE_MID_TURN_BACK"
_ST_ROUTE_MID_ALIGN_DOWN = "S4_ROUTE_MID_ALIGN_DOWN"
_ST_ROUTE_MID_CHECK_BLOCK = "S4_ROUTE_MID_CHECK_BLOCK"
_ST_ROUTE_MID_RETURN_Y = "S4_ROUTE_MID_RETURN_Y"
_ST_ROUTE_MID_TURN_EAST = "S4_ROUTE_MID_TURN_EAST"
_ST_ROUTE_TO_RIGHT = "S4_ROUTE_TO_RIGHT"
_ST_ROUTE_RIGHT_TURN_UP = "S4_ROUTE_RIGHT_TURN_UP"
_ST_ROUTE_RIGHT_ALIGN_UP = "S4_ROUTE_RIGHT_ALIGN_UP"
_ST_ROUTE_RIGHT_BAR_UP = "S4_ROUTE_RIGHT_BAR_UP"
_ST_ROUTE_RIGHT_FIND_FOOTBALL_LOW = "S4_ROUTE_RIGHT_FIND_FOOTBALL_LOW"
_ST_ROUTE_RIGHT_KICK_FOOTBALL_LOW = "S4_ROUTE_RIGHT_KICK_FOOTBALL_LOW"
_ST_ROUTE_RIGHT_ALIGN_AFTER_KICK_LOW = "S4_ROUTE_RIGHT_ALIGN_AFTER_KICK_LOW"
_ST_ROUTE_RIGHT_BACKUP_LOW = "S4_ROUTE_RIGHT_BACKUP_LOW"
_ST_ROUTE_RIGHT_STAND_AFTER_LOW = "S4_ROUTE_RIGHT_STAND_AFTER_LOW"
_ST_ROUTE_RIGHT_TURN_DOWN_AFTER_LOW = "S4_ROUTE_RIGHT_TURN_DOWN_AFTER_LOW"
_ST_ROUTE_RIGHT_ALIGN_DOWN_AFTER_LOW = "S4_ROUTE_RIGHT_ALIGN_DOWN_AFTER_LOW"
_ST_ROUTE_RIGHT_RETURN_Y = "S4_ROUTE_RIGHT_RETURN_Y"
_ST_ROUTE_RIGHT_TURN_EAST = "S4_ROUTE_RIGHT_TURN_EAST"
_ST_ROUTE_TO_BRIDGE_X = "S4_ROUTE_TO_BRIDGE_X"
_ST_ROUTE_BRIDGE_TURN_UP = "S4_ROUTE_BRIDGE_TURN_UP"
_ST_ROUTE_BRIDGE_APPROACH = "S4_ROUTE_BRIDGE_APPROACH"

_ROUTE_STATES = (
    _ST_ROUTE_TO_START,
    _ST_ROUTE_LEFT_TO_LANE,
    _ST_ROUTE_LEFT_TURN_UP,
    _ST_ROUTE_LEFT_ALIGN_UP,
    _ST_ROUTE_LEFT_BAR_UP,
    _ST_ROUTE_LEFT_ALIGN_AFTER_BAR,
    _ST_ROUTE_LEFT_FIND_COLA,
    _ST_ROUTE_LEFT_STRIKE_COLA,
    _ST_ROUTE_LEFT_BACKUP_AFTER_COLA,
    _ST_ROUTE_LEFT_TURN_BACK,
    _ST_ROUTE_LEFT_ALIGN_DOWN,
    _ST_ROUTE_LEFT_BAR_DOWN,
    _ST_ROUTE_LEFT_RETURN_Y,
    _ST_ROUTE_LEFT_TURN_EAST,
    _ST_ROUTE_TO_MID,
    _ST_ROUTE_MID_TURN_UP,
    _ST_ROUTE_MID_ALIGN_UP,
    _ST_ROUTE_MID_FIND_ORANGE,
    _ST_ROUTE_MID_SHAKE_ORANGE,
    _ST_ROUTE_MID_BACKUP_AFTER_ORANGE,
    _ST_ROUTE_MID_TURN_BACK,
    _ST_ROUTE_MID_ALIGN_DOWN,
    _ST_ROUTE_MID_CHECK_BLOCK,
    _ST_ROUTE_MID_RETURN_Y,
    _ST_ROUTE_MID_TURN_EAST,
    _ST_ROUTE_TO_RIGHT,
    _ST_ROUTE_RIGHT_TURN_UP,
    _ST_ROUTE_RIGHT_ALIGN_UP,
    _ST_ROUTE_RIGHT_BAR_UP,
    _ST_ROUTE_RIGHT_FIND_FOOTBALL_LOW,
    _ST_ROUTE_RIGHT_KICK_FOOTBALL_LOW,
    _ST_ROUTE_RIGHT_ALIGN_AFTER_KICK_LOW,
    _ST_ROUTE_RIGHT_BACKUP_LOW,
    _ST_ROUTE_RIGHT_STAND_AFTER_LOW,
    _ST_ROUTE_RIGHT_TURN_DOWN_AFTER_LOW,
    _ST_ROUTE_RIGHT_ALIGN_DOWN_AFTER_LOW,
    _ST_ROUTE_RIGHT_RETURN_Y,
    _ST_ROUTE_RIGHT_TURN_EAST,
    _ST_ROUTE_TO_BRIDGE_X,
    _ST_ROUTE_BRIDGE_TURN_UP,
    _ST_ROUTE_BRIDGE_APPROACH,
)

_state = _ST_ROUTE_TO_START
_obstacle_idx = 0
_target_idx = 0
_scan_count = 0
_motion_start = None
_announced = set()
_bypass_points = []
_bypass_idx = 0
_front_bar_low_active = False
_front_bar_low_start = None
_front_bar_low_heading = HEADING_FRONT
_shake_phase = 0
_shake_start = None
_last_log_time = 0.0
_last_log_signature = None
_right_low_start = None

START_POINT = (3.10, 7.10)
ROUTE_SWITCH_Y = 8.90
LEFT_RETURN_Y = ROUTE_SWITCH_Y
COLA_APPROACH_X = 0.00
LEFT_BAR_DETECT_UP_Y = OBSTACLES[0]["y_min"] - BAR_DETECT_DIST
LEFT_BAR_DETECT_DOWN_Y = OBSTACLES[0]["y_max"] + BAR_DETECT_DIST
MID_BLOCK_DETECT_DOWN_Y = OBSTACLES[1]["y_max"] + 0.50
COLA_STRIKE_DIST = TARGETS[0]["strike"]
SEGMENT4_MAX_Y = 11.60
COLA_FORCE_STRIKE_Y = TARGETS[0]["y"]
COLA_BACKUP_START_Y = 11.15
ORANGE_FORCE_SHAKE_Y = TARGETS[1]["y"]
ORANGE_BACKUP_START_Y = 11.05
FOOTBALL_FORCE_KICK_Y = TARGETS[2]["y"]
RIGHT_BAR_DETECT_UP_Y = OBSTACLES[2]["y_min"] - BAR_DETECT_DIST
RIGHT_LOW_START_Y = 10.00
TARGET_BACKUP_DIST = 0.10


LOG_PATH = os.path.abspath(
    os.path.join(os.path.dirname(__file__), "..", "log", "segment4_log.txt")
)


def _log_event(event, **fields):
    try:
        os.makedirs(os.path.dirname(LOG_PATH), exist_ok=True)
        stamp = time.strftime("%Y-%m-%d %H:%M:%S")
        detail = " ".join(f"{k}={v}" for k, v in fields.items())
        with open(LOG_PATH, "a", encoding="utf-8") as f:
            f.write(f"{stamp} {event} {detail}\n")
    except Exception:
        pass


def _reset_log_file():
    try:
        os.makedirs(os.path.dirname(LOG_PATH), exist_ok=True)
        with open(LOG_PATH, "w", encoding="utf-8") as f:
            f.write("")
    except Exception:
        pass


def _fmt_pos(position):
    return f"({position[0]:.3f},{position[1]:.3f},{position[2]:.3f})"


def _set_state(new_state, reason, position=None):
    global _state
    old_state = _state
    _state = new_state
    _log_event(
        "STATE",
        old=old_state,
        new=new_state,
        reason=reason,
        obs=_obstacle_idx,
        target=_target_idx,
        pos=_fmt_pos(position) if position is not None else "-",
    )


def _return_step(step, reason, position=None, rpy=None):
    _log_event(
        "STEP",
        step=step,
        reason=reason,
        state=_state,
        obs=_obstacle_idx,
        target=_target_idx,
        pos=_fmt_pos(position) if position is not None else "-",
        rpy=f"{rpy:.1f}" if rpy is not None else "-",
    )
    return step


def reset_segment4():
    """重置第四赛段状态。"""
    global _state, _obstacle_idx, _target_idx, _scan_count, _motion_start, _announced
    global _bypass_points, _bypass_idx, _front_bar_low_active
    global _front_bar_low_start, _front_bar_low_heading
    global _shake_phase, _shake_start, _last_log_time, _last_log_signature, _right_low_start
    _reset_log_file()
    _state = _ST_ROUTE_TO_START
    _obstacle_idx = 0
    _target_idx = 0
    _scan_count = 0
    _motion_start = None
    _announced = set()
    _bypass_points = []
    _bypass_idx = 0
    _front_bar_low_active = False
    _front_bar_low_start = None
    _front_bar_low_heading = HEADING_FRONT
    _shake_phase = 0
    _shake_start = None
    _last_log_time = 0.0
    _last_log_signature = None
    _right_low_start = None
    _log_event("RESET", state=_state, obs=_obstacle_idx, target=_target_idx)


def _norm(a):
    while a > 180:
        a -= 360
    while a <= -180:
        a += 360
    return a


def _nearest_cardinal(rpy):
    return round((rpy % 360) / 90) * 90 % 360


def _turn_to(rpy, target_hdg):
    """朝向目标角度，完成后返回普通前进步态编号 1。"""
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


def _goto_xy(position, rpy, target):
    """
    到达目标点。

    默认不同竖向通道之间换道时，先回到底部公共通道，再横向换道。
    """
    x, y, _ = position
    tx, ty = target

    if abs(x - tx) <= SAME_LANE_TOL and y > LANE_SWITCH_Y + XY_TOL:
        if _near_limit_bar(position):
            _log_event("GOTO_XY", branch="same_lane_near_bar", pos=_fmt_pos(position), target=target)
            return LOW_BAR_GAIT
        if abs(y - ty) > XY_TOL:
            _log_event("GOTO_XY", branch="same_lane_forward_y", pos=_fmt_pos(position), target=target)
            return _goto_xy_direct(position, rpy, (x, ty))
        _log_event("GOTO_XY", branch="same_lane_reached", pos=_fmt_pos(position), target=target)
        return 0

    if abs(x - tx) > XY_TOL and y < LANE_SWITCH_Y - XY_TOL:
        _log_event("GOTO_XY", branch="below_switch_forward_to_switch_y", pos=_fmt_pos(position), target=target)
        step = _turn_to(rpy, 90)
        return step if step != 1 else 1

    if abs(x - tx) > XY_TOL and y > LANE_SWITCH_Y + XY_TOL:
        _log_event(
            "GOTO_XY",
            branch="above_switch_return_to_switch_y",
            pos=_fmt_pos(position),
            target=target,
            x_err=f"{x - tx:.3f}",
            y=f"{y:.3f}",
        )
        return _goto_xy_direct(position, rpy, (x, LANE_SWITCH_Y))

    _log_event("GOTO_XY", branch="direct", pos=_fmt_pos(position), target=target)
    return _goto_xy_direct(position, rpy, target)


def _goto_obstacle(position, rpy, obstacle):
    x, y, _ = position
    tx, ty = _obstacle_approach_point(obstacle)

    if _obstacle_idx == 0:
        if y < LANE_SWITCH_Y - XY_TOL:
            _log_event("GOTO_OBSTACLE", lane="left", branch="enter_forward_to_switch", pos=_fmt_pos(position), target=(tx, ty))
            step = _turn_to(rpy, 90)
            return step if step != 1 else 1
        if abs(x - tx) > XY_TOL:
            _log_event("GOTO_OBSTACLE", lane="left", branch="shift_to_left_lane", pos=_fmt_pos(position), target=(tx, ty), x_err=f"{x - tx:.3f}")
            target_hdg = 0 if tx > x else 180
            step = _turn_to(rpy, target_hdg)
            return step if step != 1 else 1
        if abs(y - ty) > XY_TOL:
            _log_event("GOTO_OBSTACLE", lane="left", branch="forward_to_bar_detect_y", pos=_fmt_pos(position), target=(tx, ty))
            step = _turn_to(rpy, 90)
            return step if step != 1 else 1
        _log_event("GOTO_OBSTACLE", lane="left", branch="reached_bar_detect_y", pos=_fmt_pos(position), target=(tx, ty))
        return 0

    _log_event("GOTO_OBSTACLE", lane="other", branch="generic_goto_xy", pos=_fmt_pos(position), target=(tx, ty), obs=_obstacle_idx)
    return _goto_xy(position, rpy, (tx, ty))


def _goto_xy_direct(position, rpy, target):
    """直接走向目标点，用于绕障过程中的虚线借道。"""
    x, y, _ = position
    tx, ty = target

    if abs(x - tx) > XY_TOL:
        target_hdg = 0 if tx > x else 180
        step = _turn_to(rpy, target_hdg)
        _log_event("GOTO_DIRECT", branch="fix_x", pos=_fmt_pos(position), target=target, step=step, x_err=f"{x - tx:.3f}")
        return step if step != 1 else 1

    if abs(y - ty) > XY_TOL:
        low_bar_step = _low_bar_crossing_step(position, rpy, target)
        if low_bar_step is not None:
            _log_event("GOTO_DIRECT", branch="low_bar_crossing", pos=_fmt_pos(position), target=target, step=low_bar_step)
            return low_bar_step
        target_hdg = 90 if ty > y else 270
        step = _turn_to(rpy, target_hdg)
        _log_event("GOTO_DIRECT", branch="fix_y", pos=_fmt_pos(position), target=target, step=step, y_err=f"{y - ty:.3f}")
        return step if step != 1 else 1

    _log_event("GOTO_DIRECT", branch="reached", pos=_fmt_pos(position), target=target)
    return 0


def _walk_front(rpy, gait=1):
    step = _turn_to(rpy, HEADING_FRONT)
    return step if step != 1 else gait


def _left_lane_cola_goto(position, rpy, target):
    x, y, _ = position
    left_bar = OBSTACLES[0]
    clear_y = left_bar["y_max"] + BAR_CLEAR_MARGIN

    if y < clear_y:
        _log_event("LEFT_COLA", phase="goto", branch="keep_low_until_clear", pos=_fmt_pos(position), clear_y=f"{clear_y:.3f}", x=f"{x:.3f}")
        return _return_step(LOW_BAR_GAIT, "left_cola_goto_keep_low_until_clear", position, rpy)
    if y < target["y_min"]:
        _log_event("LEFT_COLA", phase="goto", branch="forward_to_cola_y", pos=_fmt_pos(position), target_y=f"{target['y_min']:.3f}", x=f"{x:.3f}")
        return _return_step(_walk_front(rpy), "left_cola_goto_forward_to_cola_y", position, rpy)
    _log_event("LEFT_COLA", phase="goto", branch="reached_scan_y", pos=_fmt_pos(position), x=f"{x:.3f}")
    return _return_step(0, "left_cola_goto_reached_scan_y", position, rpy)


def _left_lane_cola_scan(position, rpy, y_max):
    x, y, _ = position
    if y >= y_max:
        _log_event("LEFT_COLA", phase="scan", branch="range_finished", pos=_fmt_pos(position), y_max=f"{y_max:.3f}", x=f"{x:.3f}")
        return _return_step(0, "left_cola_scan_range_finished", position, rpy)
    _log_event("LEFT_COLA", phase="scan", branch="forward_scan", pos=_fmt_pos(position), y_max=f"{y_max:.3f}", x=f"{x:.3f}")
    return _return_step(_walk_front(rpy, SCAN_STEP), "left_cola_scan_forward", position, rpy)


def _near_limit_bar(position):
    x, y, _ = position
    for obstacle in OBSTACLES:
        if obstacle["type"] != "bar":
            continue
        if abs(x - obstacle["x"]) > BAR_LANE_TOL:
            continue
        if obstacle["y_min"] - 0.10 <= y <= obstacle["y_max"] + BAR_FORWARD_CLEAR:
            return True
    return False


def _speak(text):
    """语音播报；没有 TTS 时至少打印播报文本。"""
    print(f"语音播报：识别到{text}")
    cmd = shutil.which("spd-say") or shutil.which("espeak")
    if cmd:
        try:
            subprocess.Popen([cmd, f"识别到{text}"])
        except Exception:
            pass


def _central_roi(frame):
    h, w = frame.shape[:2]
    return frame[h // 5: 4 * h // 5, w // 4: 3 * w // 4]


def _detect_orange_ball(frame):
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
    if perimeter == 0:
        return False
    circularity = 4 * math.pi * area / (perimeter * perimeter)
    return circularity > 0.45


def _detect_cola(frame):
    roi = _central_roi(frame)
    hsv = cv2.cvtColor(roi, cv2.COLOR_BGR2HSV)
    dark = cv2.inRange(hsv, np.array([0, 0, 0]), np.array([180, 255, 95]))
    cola_brown = cv2.inRange(hsv, np.array([5, 35, 20]), np.array([35, 255, 140]))
    mask = cv2.bitwise_or(dark, cola_brown)
    mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, np.ones((5, 5), np.uint8))
    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    best = None
    for cnt in contours:
        area = cv2.contourArea(cnt)
        if area < 140:
            continue
        _, _, w, h = cv2.boundingRect(cnt)
        if w == 0 or h == 0:
            continue
        ratio = h / w
        fill = area / (w * h)
        if best is None or area > best[0]:
            best = (area, w, h, ratio, fill)
        bottle_like = ratio > 1.15 and fill > 0.22
        large_dark_body = area > 650 and ratio > 0.70
        if bottle_like or large_dark_body:
            _log_event(
                "COLA_DETECT",
                detected=True,
                area=f"{area:.1f}",
                w=w,
                h=h,
                ratio=f"{ratio:.2f}",
                fill=f"{fill:.2f}",
            )
            return True
    if best is not None:
        area, w, h, ratio, fill = best
        _log_event(
            "COLA_DETECT",
            detected=False,
            best_area=f"{area:.1f}",
            best_w=w,
            best_h=h,
            best_ratio=f"{ratio:.2f}",
            best_fill=f"{fill:.2f}",
        )
    else:
        _log_event("COLA_DETECT", detected=False, best_area=0)
    return False


def _detect_football(frame):
    roi = _central_roi(frame)
    gray = cv2.cvtColor(roi, cv2.COLOR_BGR2GRAY)
    dark = cv2.inRange(gray, 0, 80)
    light = cv2.inRange(gray, 170, 255)
    combined = cv2.bitwise_or(dark, light)
    contours, _ = cv2.findContours(combined, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    contours = [c for c in contours if cv2.contourArea(c) > 250]
    if not contours:
        return False
    cnt = max(contours, key=cv2.contourArea)
    area = cv2.contourArea(cnt)
    perimeter = cv2.arcLength(cnt, True)
    if perimeter == 0:
        return False
    circularity = 4 * math.pi * area / (perimeter * perimeter)
    return circularity > 0.35


def _detect_obstacle(frame):
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
    """识别前方近距离限高杆，命中后进入低姿态保护。"""
    if frame is None:
        return False

    h, w = frame.shape[:2]
    roi = frame[h // 3: 5 * h // 6, w // 5: 4 * w // 5]
    hsv = cv2.cvtColor(roi, cv2.COLOR_BGR2HSV)

    red1 = cv2.inRange(hsv, np.array([0, 90, 70]), np.array([10, 255, 255]))
    red2 = cv2.inRange(hsv, np.array([170, 90, 70]), np.array([180, 255, 255]))
    mask = cv2.bitwise_or(red1, red2)
    mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, np.ones((5, 5), np.uint8))

    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    for cnt in contours:
        if cv2.contourArea(cnt) < 250:
            continue
        _, y, bw, bh = cv2.boundingRect(cnt)
        horizontal_bar = bw > bh * 3 and bw > roi.shape[1] * 0.18
        close_enough = y > roi.shape[0] * 0.15
        if horizontal_bar and close_enough:
            return True
    return False


def _detect_target(frame, kind):
    if frame is None:
        return False
    if kind == "cola":
        return _detect_cola(frame)
    if kind == "orange_ball":
        return _detect_orange_ball(frame)
    if kind == "football":
        return _detect_football(frame)
    return False


def _low_bar_crossing_step(position, rpy, target):
    """经过已知限高杆区域时，强制使用低姿态步态。"""
    x, y, _ = position
    tx, ty = target

    if abs(x - tx) > XY_TOL:
        return None

    for obstacle in OBSTACLES:
        if obstacle["type"] != "bar" or abs(x - obstacle["x"]) > BAR_LANE_TOL:
            continue

        crosses_bar = min(y, ty) <= obstacle["y_max"] and max(y, ty) >= obstacle["y_min"]
        if not crosses_bar:
            continue

        target_hdg = 90 if ty > y else 270
        step = _turn_to(rpy, target_hdg)
        return step if step != 1 else LOW_BAR_GAIT

    return None


def _obstacle_approach_point(obstacle):
    dist = BAR_DETECT_DIST if obstacle["type"] == "bar" else BLOCK_APPROACH_DIST
    return (obstacle["x"], obstacle["y_min"] - dist)


def _scan_forward(position, rpy, lane_x, y_max):
    if abs(position[0] - lane_x) <= SAME_LANE_TOL and position[1] > LANE_SWITCH_Y + XY_TOL:
        if position[1] >= y_max:
            return 0
        turn_step = _turn_to(rpy, HEADING_FRONT)
        return turn_step if turn_step != 1 else SCAN_STEP

    step = _goto_xy_direct(position, rpy, (lane_x, y_max))
    if step == 0:
        return 0
    if abs(position[0] - lane_x) <= XY_TOL:
        turn_step = _turn_to(rpy, HEADING_FRONT)
        return turn_step if turn_step != 1 else SCAN_STEP
    return step


def _make_bypass_points(obstacle):
    """
    为无法跨越障碍生成绕行点。

    优先借左侧相邻通道；若障碍已经在最左通道，则借右侧相邻通道。
    """
    lane_x = obstacle["x"]
    front_y = obstacle["y_min"] - BLOCK_APPROACH_DIST
    clear_y = obstacle["y_max"] + BLOCK_CLEAR_DIST
    side_x = lane_x - 1.10 if lane_x > LEFT_LANE_X + 0.05 else lane_x + 1.10
    return [
        (side_x, front_y),
        (side_x, clear_y),
        (lane_x, clear_y),
    ]


def _shake_step(position):
    global _shake_phase, _shake_start

    if _shake_start is None:
        _shake_start = [position[0], position[1]]

    dist = _dist_from_start(position)
    if dist >= 0.22:
        _shake_phase += 1
        _motion_start_reset(position)
        if _shake_phase >= 4:
            return 0

    return 7 if _shake_phase % 2 == 0 else 8


def _motion_start_reset(position):
    global _motion_start
    _motion_start = [position[0], position[1]]


def _announce_once(key, text):
    if key in _announced:
        return
    _announced.add(key)
    _speak(text)


def _route_step_forward(rpy, heading, gait=1):
    step = _turn_to(rpy, heading)
    return step if step != 1 else gait


def _route_go_x(position, rpy, target_x, heading, next_state, reason):
    x, _, _ = position
    reached = x <= target_x + XY_TOL if heading == 180 else x >= target_x - XY_TOL
    if reached:
        _set_state(next_state, reason, position)
        return _return_step(0, reason, position, rpy)
    return _return_step(_route_step_forward(rpy, heading), f"{reason}_move_x", position, rpy)


def _route_go_y(position, rpy, target_y, heading, next_state, reason, gait=1):
    _, y, _ = position
    reached = y >= target_y - XY_TOL if heading == 90 else y <= target_y + XY_TOL
    if reached:
        _set_state(next_state, reason, position)
        return _return_step(0, reason, position, rpy)
    return _return_step(_route_step_forward(rpy, heading, gait), f"{reason}_move_y", position, rpy)


def _route_turn(rpy, heading, next_state, reason, position):
    step = _turn_to(rpy, heading)
    if step == 1:
        _set_state(next_state, reason, position)
        return _return_step(0, reason, position, rpy)
    return _return_step(step, f"{reason}_turn", position, rpy)


def _route_adjust_x_strafe(position, rpy, lane_x, heading, next_state, reason):
    x, _, _ = position
    step = _turn_to(rpy, heading)
    if step != 1:
        return _return_step(step, f"{reason}_align_heading", position, rpy)

    x_err = x - lane_x
    _log_event(
        "ROUTE_X_ADJUST",
        lane_x=f"{lane_x:.2f}",
        x_err=f"{x_err:.3f}",
        heading=heading,
        pos=_fmt_pos(position),
    )
    if abs(x_err) <= XY_TOL:
        _set_state(next_state, reason, position)
        return _return_step(0, reason, position, rpy)

    if heading == 90:
        lateral_step = 7 if x_err > 0 else 8
    elif heading == 270:
        lateral_step = 8 if x_err > 0 else 7
    else:
        lateral_step = 7 if x_err > 0 else 8
    return _return_step(lateral_step, f"{reason}_strafe_x", position, rpy)


def _route_detect_low_bar(frame, position, direction):
    _, y, _ = position
    if direction == "up" and y >= LEFT_BAR_DETECT_UP_Y:
        return True
    if direction == "down" and y <= LEFT_BAR_DETECT_DOWN_Y + XY_TOL:
        return True
    return _detect_limit_bar_ahead(frame)


def _right_bar_detected(frame, position):
    _, y, _ = position
    return y >= RIGHT_BAR_DETECT_UP_Y or _detect_limit_bar_ahead(frame)


def _route_control(position, gait_mode, rpy, frame=None):
    global _state, _obstacle_idx, _target_idx, _motion_start
    global _shake_phase, _shake_start, _right_low_start

    x, y, _ = position
    _log_event("ROUTE", state=_state, pos=_fmt_pos(position), rpy=f"{rpy:.1f}")

    if _state == _ST_ROUTE_TO_START:
        if abs(y - START_POINT[1]) > XY_TOL:
            heading = 90 if START_POINT[1] > y else 270
            return _return_step(_route_step_forward(rpy, heading), "route_to_start_fix_y", position, rpy)
        return _route_go_x(position, rpy, START_POINT[0], 180, _ST_ROUTE_LEFT_TO_LANE, "route_start_reached")

    if _state == _ST_ROUTE_LEFT_TO_LANE:
        return _route_go_x(position, rpy, LEFT_LANE_X, 180, _ST_ROUTE_LEFT_TURN_UP, "route_left_lane_reached")

    if _state == _ST_ROUTE_LEFT_TURN_UP:
        return _route_turn(rpy, 90, _ST_ROUTE_LEFT_ALIGN_UP, "route_left_turn_up_done", position)

    if _state == _ST_ROUTE_LEFT_ALIGN_UP:
        return _route_adjust_x_strafe(
            position,
            rpy,
            LEFT_LANE_X,
            90,
            _ST_ROUTE_LEFT_BAR_UP,
            "route_left_align_up_done",
        )

    if _state == _ST_ROUTE_LEFT_BAR_UP:
        if _motion_start is not None:
            if y >= OBSTACLES[0]["y_max"] + BAR_CLEAR_MARGIN:
                _set_state(_ST_ROUTE_LEFT_ALIGN_AFTER_BAR, "route_left_bar_up_clear", position)
                _motion_start = None
                return _return_step(0, "route_left_bar_up_clear", position, rpy)
            return _return_step(_route_step_forward(rpy, 90, LOW_BAR_GAIT), "route_left_bar_up_keep_low", position, rpy)
        if y >= OBSTACLES[0]["y_max"] + BAR_CLEAR_MARGIN:
            _set_state(_ST_ROUTE_LEFT_ALIGN_AFTER_BAR, "route_left_bar_up_already_clear", position)
            return _return_step(0, "route_left_bar_up_already_clear", position, rpy)
        if _route_detect_low_bar(frame, position, "up"):
            _announce_once("route_left_bar_up", "限高杆")
            _motion_start = [x, y]
            _log_event("ROUTE_BAR", lane="left", direction="up", action="enter_low", pos=_fmt_pos(position))
            return _return_step(LOW_BAR_GAIT, "route_left_bar_up_enter_low", position, rpy)
        return _return_step(_route_step_forward(rpy, 90), "route_left_bar_up_approach_20cm", position, rpy)

    if _state == _ST_ROUTE_LEFT_ALIGN_AFTER_BAR:
        # Approach the cola slightly to the right, so the fallen bottle does not block the return turn.
        return _route_adjust_x_strafe(
            position,
            rpy,
            COLA_APPROACH_X,
            90,
            _ST_ROUTE_LEFT_FIND_COLA,
            "route_left_align_after_bar_done",
        )

    if _state == _ST_ROUTE_LEFT_FIND_COLA:
        cola_detected = _detect_target(frame, "cola")
        _log_event(
            "ROUTE_TARGET_SCAN",
            target="cola",
            detected=cola_detected,
            pos=_fmt_pos(position),
            backup_y=f"{COLA_BACKUP_START_Y:.2f}",
        )
        if cola_detected:
            _announce_once("route_cola", "可乐瓶")
        if y >= COLA_BACKUP_START_Y:
            _motion_start = [x, y]
            _announce_once("route_cola_forced", "可乐瓶")
            _log_event(
                "ROUTE_TARGET_COORD_BACKUP",
                target="cola",
                pos=_fmt_pos(position),
                backup_y=f"{COLA_BACKUP_START_Y:.2f}",
                reason="reach_cola_backup_y",
            )
            _set_state(_ST_ROUTE_LEFT_BACKUP_AFTER_COLA, "route_cola_coord_backup", position)
            return _return_step(0, "route_cola_coord_backup", position, rpy)
        return _return_step(_route_step_forward(rpy, 90), "route_left_find_cola_forward", position, rpy)

    if _state == _ST_ROUTE_LEFT_STRIKE_COLA:
        if y >= SEGMENT4_MAX_Y:
            _log_event(
                "ROUTE_STRIKE_BOUNDARY_STOP",
                target="cola",
                pos=_fmt_pos(position),
                boundary_y=f"{SEGMENT4_MAX_Y:.2f}",
                dist=f"{_dist_from_start(position):.2f}",
            )
            _set_state(_ST_ROUTE_LEFT_BACKUP_AFTER_COLA, "route_cola_strike_boundary_stop", position)
            _motion_start = [x, y]
            return _return_step(0, "route_cola_strike_boundary_stop", position, rpy)
        if _dist_from_start(position) >= COLA_STRIKE_DIST:
            _set_state(_ST_ROUTE_LEFT_BACKUP_AFTER_COLA, "route_cola_strike_done", position)
            _motion_start = [x, y]
            return _return_step(0, "route_cola_strike_done", position, rpy)
        return _return_step(_route_step_forward(rpy, 90), "route_cola_strike_forward", position, rpy)

    if _state == _ST_ROUTE_LEFT_BACKUP_AFTER_COLA:
        if _dist_from_start(position) >= TARGET_BACKUP_DIST:
            _set_state(_ST_ROUTE_LEFT_TURN_BACK, "route_cola_backup_done", position)
            _motion_start = None
            return _return_step(0, "route_cola_backup_done", position, rpy)
        return _return_step(6, "route_cola_backup_before_turn", position, rpy)

    if _state == _ST_ROUTE_LEFT_TURN_BACK:
        # Turn clockwise back to the left lane, then re-center before crossing the low bar again.
        return _route_turn(rpy, 270, _ST_ROUTE_LEFT_ALIGN_DOWN, "route_left_turn_back_done", position)

    if _state == _ST_ROUTE_LEFT_ALIGN_DOWN:
        return _route_adjust_x_strafe(
            position,
            rpy,
            LEFT_LANE_X,
            270,
            _ST_ROUTE_LEFT_BAR_DOWN,
            "route_left_align_down_done",
        )

    if _state == _ST_ROUTE_LEFT_BAR_DOWN:
        if y <= LEFT_RETURN_Y + XY_TOL:
            _set_state(_ST_ROUTE_LEFT_TURN_EAST, "route_left_return_y_reached", position)
            _motion_start = None
            return _return_step(0, "route_left_return_y_reached", position, rpy)
        if _motion_start is not None:
            return _return_step(_route_step_forward(rpy, 270, LOW_BAR_GAIT), "route_left_bar_down_keep_low", position, rpy)
        if _route_detect_low_bar(frame, position, "down"):
            _announce_once("route_left_bar_down", "限高杆")
            _motion_start = [x, y]
            return _return_step(LOW_BAR_GAIT, "route_left_bar_down_enter_low", position, rpy)
        return _return_step(_route_step_forward(rpy, 270), "route_left_bar_down_approach_20cm", position, rpy)

    if _state == _ST_ROUTE_LEFT_TURN_EAST:
        return _route_turn(rpy, 0, _ST_ROUTE_TO_MID, "route_left_turn_east_done", position)

    if _state == _ST_ROUTE_TO_MID:
        return _route_go_x(position, rpy, MID_LANE_X, 0, _ST_ROUTE_MID_TURN_UP, "route_mid_lane_reached")

    if _state == _ST_ROUTE_MID_TURN_UP:
        return _route_turn(rpy, 90, _ST_ROUTE_MID_ALIGN_UP, "route_mid_turn_up_done", position)

    if _state == _ST_ROUTE_MID_ALIGN_UP:
        return _route_adjust_x_strafe(
            position,
            rpy,
            MID_LANE_X,
            90,
            _ST_ROUTE_MID_FIND_ORANGE,
            "route_mid_align_up_done",
        )

    if _state == _ST_ROUTE_MID_FIND_ORANGE:
        orange_detected = _detect_target(frame, "orange_ball")
        _log_event(
            "ROUTE_TARGET_SCAN",
            target="orange_ball",
            detected=orange_detected,
            pos=_fmt_pos(position),
            target_y=f"{TARGETS[1]['y']:.2f}",
            backup_y=f"{ORANGE_BACKUP_START_Y:.2f}",
        )
        if orange_detected:
            _announce_once("route_orange_ball", "橙色小球")
        if y >= ORANGE_BACKUP_START_Y:
            _announce_once("route_orange_ball", "橙色小球")
            _motion_start = [x, y]
            _shake_phase = 0
            _shake_start = None
            _set_state(_ST_ROUTE_MID_BACKUP_AFTER_ORANGE, "route_orange_coord_backup", position)
            return _return_step(0, "route_orange_coord_backup", position, rpy)
        return _return_step(_route_step_forward(rpy, 90), "route_mid_find_orange_forward", position, rpy)

    if _state == _ST_ROUTE_MID_SHAKE_ORANGE:
        step = _shake_step(position)
        if step == 0:
            _shake_phase = 0
            _shake_start = None
            _motion_start = [x, y]
            _set_state(_ST_ROUTE_MID_BACKUP_AFTER_ORANGE, "route_orange_shake_done", position)
            return _return_step(0, "route_orange_shake_done", position, rpy)
        return _return_step(step, "route_orange_shake", position, rpy)

    if _state == _ST_ROUTE_MID_BACKUP_AFTER_ORANGE:
        if _dist_from_start(position) >= TARGET_BACKUP_DIST:
            _set_state(_ST_ROUTE_MID_TURN_BACK, "route_orange_backup_done", position)
            _motion_start = None
            return _return_step(0, "route_orange_backup_done", position, rpy)
        return _return_step(6, "route_orange_backup_before_turn", position, rpy)

    if _state == _ST_ROUTE_MID_TURN_BACK:
        return _route_turn(rpy, 270, _ST_ROUTE_MID_ALIGN_DOWN, "route_mid_turn_back_done", position)

    if _state == _ST_ROUTE_MID_ALIGN_DOWN:
        return _route_adjust_x_strafe(
            position,
            rpy,
            MID_LANE_X,
            270,
            _ST_ROUTE_MID_CHECK_BLOCK,
            "route_mid_align_down_done",
        )

    if _state == _ST_ROUTE_MID_CHECK_BLOCK:
        block_near = y <= MID_BLOCK_DETECT_DOWN_Y or _detect_obstacle(frame)
        if block_near:
            _announce_once("route_mid_block", "无法跨越障碍")
            _set_state(_ST_ROUTE_MID_RETURN_Y, "route_mid_block_checked", position)
            return _return_step(0, "route_mid_block_checked", position, rpy)
        return _return_step(_route_step_forward(rpy, 270), "route_mid_check_block_forward", position, rpy)

    if _state == _ST_ROUTE_MID_RETURN_Y:
        return _route_go_y(position, rpy, LEFT_RETURN_Y, 270, _ST_ROUTE_MID_TURN_EAST, "route_mid_return_y_reached")

    if _state == _ST_ROUTE_MID_TURN_EAST:
        return _route_turn(rpy, 0, _ST_ROUTE_TO_RIGHT, "route_mid_turn_east_done", position)

    if _state == _ST_ROUTE_TO_RIGHT:
        return _route_go_x(position, rpy, RIGHT_LANE_X, 0, _ST_ROUTE_RIGHT_TURN_UP, "route_right_lane_reached")

    if _state == _ST_ROUTE_RIGHT_TURN_UP:
        return _route_turn(rpy, 90, _ST_ROUTE_RIGHT_ALIGN_UP, "route_right_turn_up_done", position)

    if _state == _ST_ROUTE_RIGHT_ALIGN_UP:
        return _route_adjust_x_strafe(
            position,
            rpy,
            RIGHT_LANE_X,
            90,
            _ST_ROUTE_RIGHT_BAR_UP,
            "route_right_align_up_done",
        )

    if _state == _ST_ROUTE_RIGHT_BAR_UP:
        if _right_low_start is not None:
            _set_state(_ST_ROUTE_RIGHT_FIND_FOOTBALL_LOW, "route_right_low_start_done", position)
            return _return_step(LOW_BAR_GAIT, "route_right_low_start_done", position, rpy)
        if y >= RIGHT_LOW_START_Y:
            _announce_once("route_right_bar_up", "限高杆")
            _right_low_start = [RIGHT_LANE_X, RIGHT_LOW_START_Y]
            _motion_start = [x, y]
            _log_event(
                "ROUTE_BAR",
                lane="right",
                direction="up",
                action="enter_low_at_fixed_y_until_football_done",
                low_start=f"({RIGHT_LANE_X:.2f},{RIGHT_LOW_START_Y:.2f})",
                pos=_fmt_pos(position),
            )
            return _return_step(LOW_BAR_GAIT, "route_right_enter_low_at_10m", position, rpy)
        return _return_step(_route_step_forward(rpy, 90), "route_right_forward_to_low_start_y", position, rpy)

    if _state == _ST_ROUTE_RIGHT_FIND_FOOTBALL_LOW:
        football_detected = _detect_target(frame, "football")
        _log_event(
            "ROUTE_TARGET_SCAN",
            target="football",
            detected=football_detected,
            pos=_fmt_pos(position),
            target_y=f"{FOOTBALL_FORCE_KICK_Y:.2f}",
        )
        if football_detected or y >= FOOTBALL_FORCE_KICK_Y:
            _announce_once("route_football", "足球")
            _motion_start = [x, y]
            reason = "route_football_detected" if football_detected else "route_football_force_kick"
            _set_state(_ST_ROUTE_RIGHT_KICK_FOOTBALL_LOW, reason, position)
            return _return_step(TARGETS[2]["gait"], f"{reason}_start", position, rpy)
        return _return_step(_route_step_forward(rpy, 90, LOW_BAR_GAIT), "route_right_find_football_low_forward", position, rpy)

    if _state == _ST_ROUTE_RIGHT_KICK_FOOTBALL_LOW:
        if y >= FOOTBALL_GOAL[1] or _dist_from_start(position) >= TARGETS[2]["strike"]:
            _set_state(_ST_ROUTE_RIGHT_ALIGN_AFTER_KICK_LOW, "route_football_kick_done_align_x_low", position)
            return _return_step(0, "route_football_kick_done_align_x_low", position, rpy)
        return _return_step(TARGETS[2]["gait"], "route_football_kick_forward", position, rpy)

    if _state == _ST_ROUTE_RIGHT_ALIGN_AFTER_KICK_LOW:
        return _route_adjust_x_strafe(
            position,
            rpy,
            RIGHT_LANE_X,
            90,
            _ST_ROUTE_RIGHT_BACKUP_LOW,
            "route_right_align_after_kick_low_done",
        )

    if _state == _ST_ROUTE_RIGHT_BACKUP_LOW:
        if _right_low_start is None or y <= _right_low_start[1] + XY_TOL:
            _set_state(_ST_ROUTE_RIGHT_STAND_AFTER_LOW, "route_right_backup_to_low_start_done", position)
            return _return_step(0, "route_right_backup_to_low_start_done", position, rpy)
        return _return_step(6, "route_right_backup_low_no_turn", position, rpy)

    if _state == _ST_ROUTE_RIGHT_STAND_AFTER_LOW:
        _motion_start = None
        _set_state(_ST_ROUTE_RIGHT_TURN_DOWN_AFTER_LOW, "route_right_stand_after_low_done", position)
        return _return_step(0, "route_right_stand_after_low_done", position, rpy)

    if _state == _ST_ROUTE_RIGHT_TURN_DOWN_AFTER_LOW:
        return _route_turn(rpy, 270, _ST_ROUTE_RIGHT_ALIGN_DOWN_AFTER_LOW, "route_right_turn_180_after_low_done", position)

    if _state == _ST_ROUTE_RIGHT_ALIGN_DOWN_AFTER_LOW:
        _right_low_start = None
        return _route_adjust_x_strafe(
            position,
            rpy,
            RIGHT_LANE_X,
            270,
            _ST_ROUTE_RIGHT_RETURN_Y,
            "route_right_align_down_after_low_done",
        )

    if _state == _ST_ROUTE_RIGHT_RETURN_Y:
        return _route_go_y(
            position,
            rpy,
            BRIDGE_SWITCH_POINT[1],
            270,
            _ST_ROUTE_RIGHT_TURN_EAST,
            "route_right_return_switch_y_reached",
        )

    if _state == _ST_ROUTE_RIGHT_TURN_EAST:
        return _route_turn(rpy, 0, _ST_ROUTE_TO_BRIDGE_X, "route_right_turn_east_done", position)

    if _state == _ST_ROUTE_TO_BRIDGE_X:
        return _route_go_x(
            position,
            rpy,
            BRIDGE_APPROACH_POINT[0],
            0,
            _ST_ROUTE_BRIDGE_TURN_UP,
            "route_bridge_x_reached",
        )

    if _state == _ST_ROUTE_BRIDGE_TURN_UP:
        return _route_turn(rpy, 90, _ST_ROUTE_BRIDGE_APPROACH, "route_bridge_turn_up_done", position)

    if _state == _ST_ROUTE_BRIDGE_APPROACH:
        if y >= BRIDGE_START[1] - XY_TOL:
            _set_state(_ST_DONE, "route_bridge_front_feet_on", position)
            return _return_step(-1, "segment4_done_bridge_front_feet_on", position, rpy)
        return _return_step(_route_step_forward(rpy, 90), "route_bridge_approach_forward", position, rpy)

    _set_state(_ST_GOTO_OBSTACLE, "route_unknown_fallback", position)
    return _return_step(0, "route_unknown_fallback", position, rpy)


def _finish_obstacle():
    global _state, _obstacle_idx
    _obstacle_idx += 1
    _set_state(
        _ST_GOTO_TARGET if _target_idx < len(TARGETS) else _ST_GOTO_OBSTACLE,
        "finish_obstacle",
    )
    return 0


def _finish_target():
    global _state, _target_idx, _motion_start
    _target_idx += 1
    _motion_start = None
    if _target_idx >= len(TARGETS):
        _set_state(_ST_ROUTE_RIGHT_RETURN_Y, "finish_all_targets_route_to_bridge", None)
    else:
        _set_state(
            _ST_GOTO_OBSTACLE if _obstacle_idx < len(OBSTACLES) else _ST_GOTO_BRIDGE,
            "finish_target",
        )
    return 0


def segment4_control(position, gait_mode, rpy, frame=None):
    """
    返回第四赛段当前应该执行的步态编号。

    返回 -1 表示第四赛段完成，可以进入第五赛段。
    """
    global _state, _obstacle_idx, _target_idx, _scan_count, _motion_start
    global _bypass_points, _bypass_idx, _front_bar_low_active
    global _front_bar_low_start, _front_bar_low_heading
    global _shake_phase, _shake_start, _last_log_time, _last_log_signature

    x, y, _ = position
    gait, mode = gait_mode
    now = time.time()
    signature = (_state, _obstacle_idx, _target_idx, round(x, 1), round(y, 1), int(rpy // 10))
    if signature != _last_log_signature or now - _last_log_time >= 0.5:
        _log_event(
            "TICK",
            state=_state,
            obs=_obstacle_idx,
            target=_target_idx,
            pos=_fmt_pos(position),
            rpy=f"{rpy:.1f}",
            gait=gait,
            mode=mode,
        )
        _last_log_signature = signature
        _last_log_time = now

    if _state == _ST_DONE:
        return _return_step(-1, "segment4_done", position, rpy)

    route_low_states = (
        _ST_ROUTE_LEFT_BAR_UP,
        _ST_ROUTE_LEFT_BAR_DOWN,
        _ST_ROUTE_RIGHT_BAR_UP,
        _ST_ROUTE_RIGHT_FIND_FOOTBALL_LOW,
        _ST_ROUTE_RIGHT_KICK_FOOTBALL_LOW,
        _ST_ROUTE_RIGHT_ALIGN_AFTER_KICK_LOW,
        _ST_ROUTE_RIGHT_BACKUP_LOW,
    )
    if (
        (gait == 0 and mode == 0) or (gait == 1 and mode == 9)
    ) and (_state == _ST_PASS_LOW_BAR or (_state in route_low_states and _motion_start is not None) or _front_bar_low_active):
        return _return_step(LOW_BAR_GAIT, "gait_switch_keep_low_bar", position, rpy)

    if (gait == 0 and mode == 0) or (gait == 1 and mode == 9):
        return _return_step(0, "gait_switch_wait", position, rpy)

    if _state in _ROUTE_STATES:
        return _route_control(position, gait_mode, rpy, frame)

    if _front_bar_low_active:
        if _front_bar_low_start is not None and _distance(position, _front_bar_low_start) >= LOW_BAR_DETECT_CLEAR_DIST:
            _front_bar_low_active = False
            _front_bar_low_start = None
            return _return_step(1, "front_bar_visual_clear", position, rpy)
        return _return_step(LOW_BAR_GAIT, "front_bar_visual_low_active", position, rpy)

    expected_bar = (
        _obstacle_idx < len(OBSTACLES)
        and OBSTACLES[_obstacle_idx]["type"] == "bar"
        and _state in (_ST_GOTO_OBSTACLE, _ST_SCAN_OBSTACLE)
    )
    bar_close_enough = expected_bar and y >= OBSTACLES[_obstacle_idx]["y_min"] - BAR_DETECT_DIST

    if bar_close_enough and _detect_limit_bar_ahead(frame):
        _log_event("DETECT_BAR", state=_state, pos=_fmt_pos(position), rpy=f"{rpy:.1f}", expected=expected_bar)
        if "视觉限高杆" not in _announced:
            _announced.add("视觉限高杆")
            _speak("限高杆")
        if expected_bar:
            _set_state(_ST_PASS_LOW_BAR, "vision_bar_detected", position)
            _motion_start = [x, y]
            return _return_step(LOW_BAR_GAIT, "vision_bar_enter_low", position, rpy)
        _front_bar_low_active = True
        _front_bar_low_start = [x, y]
        _front_bar_low_heading = _nearest_cardinal(rpy)
        return _return_step(LOW_BAR_GAIT, "vision_bar_generic_low", position, rpy)

    if _obstacle_idx < len(OBSTACLES):
        obstacle = OBSTACLES[_obstacle_idx]

        if (
            _obstacle_idx == 0
            and obstacle["type"] == "bar"
            and y >= obstacle["y_max"] + BAR_CLEAR_MARGIN
            and _state in (_ST_GOTO_OBSTACLE, _ST_SCAN_OBSTACLE)
        ):
            _log_event(
                "RECOVER_LEFT_BAR_ALREADY_PASSED",
                state=_state,
                pos=_fmt_pos(position),
                clear_y=f"{obstacle['y_max'] + BAR_CLEAR_MARGIN:.3f}",
            )
            _finish_obstacle()
            return _return_step(1, "recover_left_bar_already_passed_go_cola", position, rpy)

        if _state == _ST_GOTO_OBSTACLE:
            step = _goto_obstacle(position, rpy, obstacle)
            if step == 0:
                _set_state(_ST_SCAN_OBSTACLE, "reached_obstacle_approach", position)
                _scan_count = 0
                if obstacle["type"] == "bar":
                    _set_state(_ST_PASS_LOW_BAR, "bar_approach_reached", position)
                    _motion_start = [x, y]
                    if obstacle["name"] not in _announced:
                        _announced.add(obstacle["name"])
                        _speak(obstacle["speak"])
                    return _return_step(LOW_BAR_GAIT, "bar_approach_enter_low", position, rpy)
                return _return_step(0, "obstacle_scan_start", position, rpy)
            return _return_step(step, "goto_obstacle", position, rpy)

        if _state == _ST_SCAN_OBSTACLE:
            _scan_count += 1
            detected = _detect_obstacle(frame)
            if obstacle["type"] == "block":
                fallback_confirmed = _scan_count >= BLOCK_SCAN_LIMIT
            else:
                range_finished = y >= obstacle["y_max"]
                fallback_confirmed = range_finished or _scan_count >= SCAN_LIMIT * 8
            _log_event(
                "SCAN_OBSTACLE",
                name=obstacle["name"],
                detected=detected,
                fallback=fallback_confirmed,
                count=_scan_count,
                pos=_fmt_pos(position),
            )
            if detected or fallback_confirmed:
                if detected and obstacle["name"] not in _announced:
                    _announced.add(obstacle["name"])
                    _speak(obstacle["speak"])
                if obstacle["type"] == "bar":
                    _set_state(_ST_PASS_LOW_BAR, "scan_bar_confirmed", position)
                    _motion_start = [x, y]
                else:
                    _set_state(_ST_BYPASS_BLOCK, "scan_block_confirmed", position)
                    _bypass_points = _make_bypass_points(obstacle)
                    _bypass_idx = 0
                return _return_step(0, "obstacle_confirmed", position, rpy)
            if obstacle["type"] == "block":
                return _return_step(0, "block_scan_wait", position, rpy)
            return _return_step(_scan_forward(position, rpy, obstacle["x"], obstacle["y_max"]), "bar_scan_forward", position, rpy)

        if _state == _ST_PASS_LOW_BAR:
            clear_y = obstacle["y_max"] + BAR_CLEAR_MARGIN
            if y >= clear_y or _dist_from_start(position) >= BAR_PASS_DIST + BAR_CLEAR_MARGIN:
                _log_event(
                    "PASS_LOW_BAR_DONE",
                    obstacle=obstacle["name"],
                    pos=_fmt_pos(position),
                    dist=f"{_dist_from_start(position):.2f}",
                    clear_y=f"{clear_y:.2f}",
                )
                _finish_obstacle()
                return _return_step(1 if _target_idx == 0 else 0, "pass_low_bar_done", position, rpy)
            return _return_step(LOW_BAR_GAIT, "pass_low_bar_keep_low", position, rpy)

        if _state == _ST_BYPASS_BLOCK:
            if _bypass_idx >= len(_bypass_points):
                _log_event("BYPASS_DONE", pos=_fmt_pos(position), obs=_obstacle_idx)
                return _return_step(_finish_obstacle(), "bypass_done", position, rpy)
            step = _goto_xy_direct(position, rpy, _bypass_points[_bypass_idx])
            if step == 0:
                _log_event("BYPASS_POINT_REACHED", idx=_bypass_idx, point=_bypass_points[_bypass_idx], pos=_fmt_pos(position))
                _bypass_idx += 1
                return _return_step(0, "bypass_point_reached", position, rpy)
            return _return_step(step, "bypass_move", position, rpy)

    if _target_idx < len(TARGETS):
        target = TARGETS[_target_idx]
        target_start = (target["x"], target["y_min"])

        if _state == _ST_GOTO_TARGET:
            if _target_idx == 0:
                step = _left_lane_cola_goto(position, rpy, target)
            else:
                step = _goto_xy(position, rpy, target_start)
            if step == 0:
                _set_state(_ST_SCAN_TARGET, "reached_target_scan_start", position)
                _scan_count = 0
                return _return_step(0, "target_scan_start", position, rpy)
            return _return_step(step, "goto_target", position, rpy)

        if _state == _ST_SCAN_TARGET:
            _scan_count += 1
            detected = _detect_target(frame, target["kind"])
            coord_reached = y >= target.get("y", target["y_min"])
            range_finished = y >= target["y_max"]
            fallback_confirmed = range_finished or _scan_count >= SCAN_LIMIT * 8
            _log_event(
                "SCAN_TARGET",
                name=target["name"],
                kind=target["kind"],
                detected=detected,
                coord_reached=coord_reached,
                fallback=fallback_confirmed,
                count=_scan_count,
                pos=_fmt_pos(position),
            )
            if detected or coord_reached:
                _announce_once(target["name"], target["name"])
                next_state = _ST_SHAKE if target["kind"] == "orange_ball" else _ST_STRIKE
                reason = "target_detected" if detected else "target_coord_reached"
                _set_state(next_state, reason, position)
                _motion_start = [x, y]
                _shake_phase = 0
                _shake_start = None
                return _return_step(0, f"{reason}_prepare_action", position, rpy)
            if fallback_confirmed:
                _announce_once(target["name"], target["name"])
                next_state = _ST_SHAKE if target["kind"] == "orange_ball" else _ST_STRIKE
                _log_event("TARGET_FALLBACK_FORCE_ACTION", name=target["name"], pos=_fmt_pos(position))
                _set_state(next_state, "target_fallback_force_action", position)
                _motion_start = [x, y]
                _shake_phase = 0
                _shake_start = None
                return _return_step(0, "target_fallback_force_action_prepare", position, rpy)
            if _target_idx == 0:
                return _left_lane_cola_scan(position, rpy, target["y_max"])
            return _scan_forward(position, rpy, target["x"], target["y_max"])

        if _state == _ST_STRIKE:
            step = _turn_to(rpy, HEADING_FRONT)
            if step != 1:
                return _return_step(step, "strike_align_front", position, rpy)
            reached_goal = target["kind"] == "football" and y >= FOOTBALL_GOAL[1]
            if reached_goal or _dist_from_start(position) >= target["strike"]:
                _set_state(_ST_BACKUP, "strike_distance_done", position)
                _motion_start = [x, y]
                return _return_step(0, "strike_done_start_backup", position, rpy)
            return _return_step(target["gait"], "strike_forward", position, rpy)

        if _state == _ST_SHAKE:
            step = _turn_to(rpy, HEADING_FRONT)
            if step != 1:
                return _return_step(step, "shake_align_front", position, rpy)
            step = _shake_step(position)
            if step == 0:
                _set_state(_ST_BACKUP, "shake_done", position)
                _motion_start = [x, y]
                _shake_phase = 0
                _shake_start = None
                return _return_step(0, "shake_done_start_backup", position, rpy)
            return _return_step(step, "shake_move", position, rpy)

        if _state == _ST_BACKUP:
            if _dist_from_start(position) >= 0.28:
                return _return_step(_finish_target(), "backup_done", position, rpy)
            return _return_step(6, "backup", position, rpy)

    if _state != _ST_GOTO_BRIDGE:
        _set_state(_ST_ROUTE_RIGHT_RETURN_Y, "all_targets_done_route_to_bridge", position)
        return _return_step(0, "all_targets_done_route_to_bridge", position, rpy)

    _set_state(_ST_ROUTE_RIGHT_RETURN_Y, "goto_bridge_legacy_redirect", position)
    return _return_step(0, "goto_bridge_legacy_redirect", position, rpy)
