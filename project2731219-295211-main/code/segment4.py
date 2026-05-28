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

# y=7.10 是第四段底部公共横向通道；第一段到第二段改为斜向过渡。
LANE_SWITCH_Y = 7.10
ROUTE_SWITCH_Y = 8.90
LEFT_TO_MID_START_Y = 8.40
LEFT_TO_MID_TARGET = (MID_LANE_X, 9.50)

# 第四段入口、可乐撞击偏移点、第五段入口相关坐标。
START_POINT = (3.10, 7.10)
COLA_APPROACH_X = 0.00
BRIDGE_SWITCH_POINT = (RIGHT_LANE_X, LANE_SWITCH_Y)
BRIDGE_APPROACH_POINT = (3.15, LANE_SWITCH_Y)
SEG5_ENTRY_HEADING = 90

# 左侧限高杆固定在 y=9.40m，第一段低姿态覆盖 y=9.00m 到 y=10.00m。
LEFT_BAR = {"x": LEFT_LANE_X, "y": 9.40, "y_min": 9.00, "y_max": 9.80}
LEFT_BAR_LOW_END_Y = 10.00

# 右侧第三段限高杆固定在 y=10.40m；y>10.00m 后保持低姿态，回退到 y=10.40m 才站起。
RIGHT_BAR_Y = 10.40
RIGHT_LOW_START_Y = 10.00
RIGHT_STAND_Y = RIGHT_BAR_Y

# 目标物坐标和动作结束坐标。
# 可乐：到 y=11.10 播报，到 y=11.15 开始倒退。
# 小球：到 y=11.00 播报，到 y=11.05 开始倒退。
# 足球：到 y=11.10 播报并开始回退。
COLA = {"x": LEFT_LANE_X, "y": 11.10, "backup_y": 11.15}
ORANGE_BALL = {"x": 0.94, "y": 11.00, "backup_y": 11.05}
FOOTBALL = {"x": RIGHT_LANE_X, "announce_y": 11.10, "backup_y": 11.10}

# 限高杆检测和动作距离参数。
LEFT_BAR_DETECT_UP_Y = LEFT_BAR["y_min"]
LEFT_BAR_DETECT_DOWN_Y = LEFT_BAR_LOW_END_Y
BAR_CLEAR_MARGIN = 0.00
TARGET_BACKUP_DIST = 0.10
COLA_BACKUP_DIST = 0.15
ORANGE_BACKUP_DIST = 0.15
FOOTBALL_BACKUP_AFTER_KICK_DIST = 0.10


# 步态编号和朝向角度。
# 1: 普通前进；5: 低姿态/蹲下通过限高杆；6: 后退；7/8: 左/右平移校正。
LOW_BAR_GAIT = 5
FOOTBALL_GAIT = 28
HEADING_EAST = 0
HEADING_NORTHEAST = 45
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
    "LEFT_BAR_UP_LOW": "S4_LEFT_BAR_UP_LOW",
    "LEFT_STAND_AFTER_BAR": "S4_LEFT_STAND_AFTER_BAR",
    "LEFT_ALIGN_COLA": "S4_LEFT_ALIGN_COLA",
    "LEFT_FIND_COLA": "S4_LEFT_FIND_COLA",
    "LEFT_BACKUP_COLA": "S4_LEFT_BACKUP_COLA",
    "LEFT_TURN_BACK": "S4_LEFT_TURN_BACK",
    "LEFT_ALIGN_DOWN": "S4_LEFT_ALIGN_DOWN",
    "LEFT_BAR_DOWN": "S4_LEFT_BAR_DOWN",
    "LEFT_BAR_DOWN_LOW": "S4_LEFT_BAR_DOWN_LOW",
    "LEFT_RETURN_Y": "S4_LEFT_RETURN_Y",
    "LEFT_TURN_DIAG": "S4_LEFT_TURN_DIAG",
    "LEFT_DIAG_TO_MID": "S4_LEFT_DIAG_TO_MID",
    "LEFT_TURN_MID_UP": "S4_LEFT_TURN_MID_UP",
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
    "RIGHT_TURN_BACK_LOW": "S4_RIGHT_TURN_BACK_LOW",
    "RIGHT_ALIGN_BACK_LOW": "S4_RIGHT_ALIGN_BACK_LOW",
    "RIGHT_BACKUP_LOW": "S4_RIGHT_BACKUP_LOW",
    "RIGHT_STAND": "S4_RIGHT_STAND",
    "RIGHT_TURN_DOWN": "S4_RIGHT_TURN_DOWN",
    "RIGHT_ALIGN_DOWN": "S4_RIGHT_ALIGN_DOWN",
    "RIGHT_RETURN_Y": "S4_RIGHT_RETURN_Y",
    "RIGHT_TURN_EAST": "S4_RIGHT_TURN_EAST",
    "BRIDGE_TO_X": "S4_BRIDGE_TO_X",
    "BRIDGE_TURN_UP": "S4_BRIDGE_TURN_UP",
    "DONE": "S4_DONE",
}

LOW_STATES = {
    S["LEFT_BAR_UP_LOW"],
    S["LEFT_BAR_DOWN_LOW"],
    S["RIGHT_LOW_FORWARD"],
    S["RIGHT_ALIGN_AFTER_BALL"],
    S["RIGHT_TURN_BACK_LOW"],
    S["RIGHT_ALIGN_BACK_LOW"],
    S["RIGHT_BACKUP_LOW"],
}

RIGHT_LOW_REQUIRED_STATES = {
    S["RIGHT_LOW_FORWARD"],
    S["RIGHT_ALIGN_AFTER_BALL"],
    S["RIGHT_TURN_BACK_LOW"],
    S["RIGHT_ALIGN_BACK_LOW"],
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


def _go_xy(position, rpy, target_xy, heading, next_state, reason, gait=1):
    """沿指定航向走到目标坐标附近，主要用于第一段到第二段的 45 度斜向过渡。"""
    x, y, _ = position
    target_x, target_y = target_xy
    if heading == HEADING_NORTHEAST:
        reached_x = x >= target_x - XY_TOL
        reached_y = y >= target_y - XY_TOL
    else:
        reached_x = abs(x - target_x) <= XY_TOL
        reached_y = abs(y - target_y) <= XY_TOL
    if reached_x and reached_y:
        _set_state(next_state, reason, position)
        return _return_step(0, reason, position, rpy)
    return _return_step(_forward_step(rpy, heading, gait), f"{reason}_move_xy", position, rpy)


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


def _backup_to_distance(position, next_state, reason, distance=TARGET_BACKUP_DIST):
    """后退固定距离后切换状态。"""
    if _dist_from_start(position) >= distance:
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


def _route_entry(position, rpy):
    """入口段：回到第四段底部横向通道，并切入第一条竖道。"""
    _, y, _ = position

    if abs(y - START_POINT[1]) > XY_TOL:
        heading = HEADING_NORTH if START_POINT[1] > y else HEADING_SOUTH
        return _return_step(_forward_step(rpy, heading), "route_to_start_fix_y", position, rpy)
    return _go_x(position, rpy, START_POINT[0], 180, S["LEFT_TO_LANE"], "route_start_reached")


def _route_left_lane(position, rpy, frame):
    """第一条竖道：过左侧限高杆，撞倒可乐瓶，再返回换道线。"""
    global _motion_start

    x, y, _ = position

    # 进入左侧竖道并调整到向上。
    if _state == S["LEFT_TO_LANE"]:
        return _go_x(position, rpy, LEFT_LANE_X, 180, S["LEFT_TURN_UP"], "route_left_lane_reached")
    if _state == S["LEFT_TURN_UP"]:
        return _turn_state(rpy, HEADING_NORTH, S["LEFT_ALIGN_UP"], "route_left_turn_up_done", position)
    if _state == S["LEFT_ALIGN_UP"]:
        return _adjust_x(position, rpy, LEFT_LANE_X, HEADING_NORTH, S["LEFT_BAR_UP"], "route_left_align_up_done")

    # 上行过限高杆：到 y=9.00 后先切入低姿态状态，再锁定低姿态走到 y=9.80。
    if _state == S["LEFT_BAR_UP"]:
        if y >= LEFT_BAR_DETECT_UP_Y:
            _announce_once("left_bar_up", "限高杆")
            _motion_start = None
            _set_state(S["LEFT_BAR_UP_LOW"], "route_left_bar_up_enter_low", position)
            return _return_step(LOW_BAR_GAIT, "route_left_bar_up_enter_low", position, rpy)
        return _return_step(_forward_step(rpy, HEADING_NORTH), "route_left_bar_up_forward", position, rpy)
    if _state == S["LEFT_BAR_UP_LOW"]:
        if y >= LEFT_BAR_LOW_END_Y:
            _motion_start = None
            _set_state(S["LEFT_STAND_AFTER_BAR"], "route_left_bar_up_clear", position)
            return _return_step(0, "route_left_bar_up_clear", position, rpy)
        return _return_step(_forward_step(rpy, HEADING_NORTH, LOW_BAR_GAIT), "route_left_bar_up_keep_low", position, rpy)

    # 起身后再进入撞可乐前的横向对齐，避免蹲起和横移叠在一起。
    if _state == S["LEFT_STAND_AFTER_BAR"]:
        _motion_start = None
        _set_state(S["LEFT_ALIGN_COLA"], "route_left_stand_after_bar_done", position)
        return _return_step(0, "route_left_stand_after_bar_done", position, rpy)
    if _state == S["LEFT_ALIGN_COLA"]:
        return _adjust_x(position, rpy, COLA_APPROACH_X, HEADING_NORTH, S["LEFT_FIND_COLA"], "route_left_align_cola_done")

    # 可乐瓶：识别或到达固定坐标后播报，继续向前撞倒，再后退 0.15m。
    if _state == S["LEFT_FIND_COLA"]:
        detected = _detect_target(frame, "cola")
        _log_event("ROUTE_TARGET_SCAN", target="cola", detected=detected, pos=_fmt_pos(position))
        if detected or y >= COLA["y"]:
            _announce_once("cola", "可乐瓶")
        if y >= COLA["backup_y"]:
            _motion_start_reset(position)
            _set_state(S["LEFT_BACKUP_COLA"], "route_cola_backup_start", position)
            return _return_step(0, "route_cola_backup_start", position, rpy)
        return _return_step(_forward_step(rpy, HEADING_NORTH), "route_left_find_cola_forward", position, rpy)
    if _state == S["LEFT_BACKUP_COLA"]:
        step = _backup_to_distance(position, S["LEFT_TURN_BACK"], "route_cola_backup_done", COLA_BACKUP_DIST)
        return _return_step(step, "route_cola_backup", position, rpy)

    # 返回左侧竖道中心线，下行时先正常走，到 10.00-9.00 区间才锁定低姿态。
    if _state == S["LEFT_TURN_BACK"]:
        _motion_start = None
        return _turn_state(rpy, HEADING_SOUTH, S["LEFT_ALIGN_DOWN"], "route_left_turn_back_done", position)
    if _state == S["LEFT_ALIGN_DOWN"]:
        _motion_start = None
        return _adjust_x(position, rpy, LEFT_LANE_X, HEADING_SOUTH, S["LEFT_BAR_DOWN"], "route_left_align_down_done")
    if _state == S["LEFT_BAR_DOWN"]:
        if y <= LEFT_TO_MID_START_Y:
            _motion_start = None
            _set_state(S["LEFT_TURN_DIAG"], "route_left_diag_start_y_reached", position)
            return _return_step(0, "route_left_diag_start_y_reached", position, rpy)
        if y <= LEFT_BAR_DETECT_DOWN_Y:
            _announce_once("left_bar_down", "限高杆")
            _set_state(S["LEFT_BAR_DOWN_LOW"], "route_left_bar_down_enter_low", position)
            return _return_step(LOW_BAR_GAIT, "route_left_bar_down_enter_low", position, rpy)
        return _return_step(_forward_step(rpy, HEADING_SOUTH), "route_left_bar_down_forward", position, rpy)
    if _state == S["LEFT_BAR_DOWN_LOW"]:
        if y <= LEFT_BAR["y_min"]:
            _motion_start = None
            _set_state(S["LEFT_RETURN_Y"], "route_left_bar_down_clear", position)
            return _return_step(0, "route_left_bar_down_clear", position, rpy)
        return _return_step(_forward_step(rpy, HEADING_SOUTH, LOW_BAR_GAIT), "route_left_bar_down_keep_low", position, rpy)
    if _state == S["LEFT_RETURN_Y"]:
        if y <= LEFT_TO_MID_START_Y:
            _set_state(S["LEFT_TURN_DIAG"], "route_left_diag_start_y_reached", position)
            return _return_step(0, "route_left_diag_start_y_reached", position, rpy)
        return _return_step(_forward_step(rpy, HEADING_SOUTH), "route_left_return_to_8_40", position, rpy)
    if _state == S["LEFT_TURN_DIAG"]:
        return _turn_state(rpy, HEADING_NORTHEAST, S["LEFT_DIAG_TO_MID"], "route_left_turn_diag_done", position)
    if _state == S["LEFT_DIAG_TO_MID"]:
        return _go_xy(position, rpy, LEFT_TO_MID_TARGET, HEADING_NORTHEAST, S["LEFT_TURN_MID_UP"], "route_left_diag_mid_reached")
    if _state == S["LEFT_TURN_MID_UP"]:
        return _turn_state(rpy, HEADING_NORTH, S["MID_ALIGN_UP"], "route_left_turn_mid_up_done", position)

    return None


def _route_mid_lane(position, rpy, frame):
    """第二条竖道：撞击橙色小球，返程识别不可跨越障碍，再切到第三条竖道。"""
    _, y, _ = position

    # 进入中间竖道。
    if _state == S["MID_TO_LANE"]:
        return _go_x(position, rpy, MID_LANE_X, HEADING_EAST, S["MID_TURN_UP"], "route_mid_lane_reached")
    if _state == S["MID_TURN_UP"]:
        return _turn_state(rpy, HEADING_NORTH, S["MID_ALIGN_UP"], "route_mid_turn_up_done", position)
    if _state == S["MID_ALIGN_UP"]:
        return _adjust_x(position, rpy, MID_LANE_X, HEADING_NORTH, S["MID_FIND_ORANGE"], "route_mid_align_up_done")

    # 橙色小球：识别或到达固定坐标后播报，继续向前撞击，再后退 0.15m。
    if _state == S["MID_FIND_ORANGE"]:
        detected = _detect_target(frame, "orange_ball")
        _log_event("ROUTE_TARGET_SCAN", target="orange_ball", detected=detected, pos=_fmt_pos(position))
        if detected or y >= ORANGE_BALL["y"]:
            _announce_once("orange_ball", "橙色小球")
        if y >= ORANGE_BALL["backup_y"]:
            _motion_start_reset(position)
            _set_state(S["MID_BACKUP_ORANGE"], "route_orange_backup_start", position)
            return _return_step(0, "route_orange_backup_start", position, rpy)
        return _return_step(_forward_step(rpy, HEADING_NORTH), "route_mid_find_orange_forward", position, rpy)
    if _state == S["MID_BACKUP_ORANGE"]:
        step = _backup_to_distance(position, S["MID_TURN_BACK"], "route_orange_backup_done", ORANGE_BACKUP_DIST)
        return _return_step(step, "route_orange_backup", position, rpy)

    # 返程：回到中线并检查不可跨越障碍，播报后回到换道线。
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

    return None


def _route_right_lane(position, rpy, frame):
    """第三条竖道：蹲姿过右侧限高杆，撞足球入门，再蹲姿返回到 y=10.40 后站起。"""
    global _motion_start, _right_low_start

    _, y, _ = position

    # 进入右侧竖道。
    if _state == S["RIGHT_TO_LANE"]:
        return _go_x(position, rpy, RIGHT_LANE_X, HEADING_EAST, S["RIGHT_TURN_UP"], "route_right_lane_reached")
    if _state == S["RIGHT_TURN_UP"]:
        return _turn_state(rpy, HEADING_NORTH, S["RIGHT_ALIGN_UP"], "route_right_turn_up_done", position)
    if _state == S["RIGHT_ALIGN_UP"]:
        return _adjust_x(position, rpy, RIGHT_LANE_X, HEADING_NORTH, S["RIGHT_TO_LOW_START"], "route_right_align_up_done")

    # 到 y=10.00 后进入低姿态区间；站起点固定为右侧限高杆 y=10.40。
    if _state == S["RIGHT_TO_LOW_START"]:
        if y >= RIGHT_LOW_START_Y:
            _right_low_start = [RIGHT_LANE_X, RIGHT_STAND_Y]
            _set_state(S["RIGHT_LOW_FORWARD"], "route_right_enter_low_at_10m", position)
            return _return_step(LOW_BAR_GAIT, "route_right_enter_low_at_10m", position, rpy)
        return _return_step(_forward_step(rpy, HEADING_NORTH), "route_right_forward_to_low_start", position, rpy)

    # 低姿态前进：过限高杆后继续推足球，到 y=11.10 认为足球已进门。
    if _state == S["RIGHT_LOW_FORWARD"]:
        detected = _detect_target(frame, "football")
        _log_event("ROUTE_TARGET_SCAN", target="football", detected=detected, pos=_fmt_pos(position))
        if y >= RIGHT_BAR_Y:
            _announce_once("right_bar", "限高杆")
        if detected or y >= FOOTBALL["announce_y"]:
            _announce_once("football", "足球")
        if y >= FOOTBALL["backup_y"]:
            _motion_start_reset(position)
            _set_state(S["RIGHT_ALIGN_AFTER_BALL"], "route_football_reach_11_10", position)
            return _return_step(LOW_BAR_GAIT, "route_football_reach_11_10", position, rpy)
        return _return_step(_forward_step(rpy, HEADING_NORTH, LOW_BAR_GAIT), "route_right_low_forward", position, rpy)

    # 足球到位后先短退 0.10m，再转身，先纠正回 x=2.10，再低姿态向前走回 y=10.40。
    if _state == S["RIGHT_ALIGN_AFTER_BALL"]:
        if _dist_from_start(position) >= FOOTBALL_BACKUP_AFTER_KICK_DIST:
            _set_state(S["RIGHT_TURN_BACK_LOW"], "route_right_backup_after_ball_done", position)
            _motion_start_reset(position)
            return _return_step(LOW_BAR_GAIT, "route_right_backup_after_ball_done", position, rpy)
        return _return_step(6, "route_right_backup_after_ball", position, rpy)
    if _state == S["RIGHT_TURN_BACK_LOW"]:
        step = _turn_to(rpy, HEADING_SOUTH)
        if step == 1:
            _set_state(S["RIGHT_ALIGN_BACK_LOW"], "route_right_turn_back_low_done", position)
            return _return_step(LOW_BAR_GAIT, "route_right_turn_back_low_done", position, rpy)
        return _return_step(step, "route_right_turn_back_low_done_turn", position, rpy)
    if _state == S["RIGHT_ALIGN_BACK_LOW"]:
        return _adjust_x(position, rpy, RIGHT_LANE_X, HEADING_SOUTH, S["RIGHT_BACKUP_LOW"], "route_right_align_back_x_done")
    if _state == S["RIGHT_BACKUP_LOW"]:
        if _right_low_start is None or y <= _right_low_start[1]:
            _set_state(S["RIGHT_STAND"], "route_right_low_return_to_10_4m_done", position)
            return _return_step(0, "route_right_low_return_to_10_4m_done", position, rpy)
        return _return_step(_forward_step(rpy, HEADING_SOUTH, LOW_BAR_GAIT), "route_right_low_forward_back_to_bar", position, rpy)

    # 站起后回到右侧竖道中心线，准备从底部通道去第五段入口。
    if _state == S["RIGHT_STAND"]:
        _motion_start = None
        _set_state(S["RIGHT_TURN_DOWN"], "route_right_stand_done", position)
        return _return_step(0, "route_right_stand_done", position, rpy)
    if _state == S["RIGHT_TURN_DOWN"]:
        return _turn_state(rpy, HEADING_SOUTH, S["RIGHT_ALIGN_DOWN"], "route_right_turn_180_done", position)
    if _state == S["RIGHT_ALIGN_DOWN"]:
        _right_low_start = None
        return _adjust_x(position, rpy, RIGHT_LANE_X, HEADING_SOUTH, S["RIGHT_RETURN_Y"], "route_right_align_down_done")

    return None


def _route_bridge_exit(position, rpy):
    """收尾段：回到底部横向通道，在第五段入口中心线转正后交给 segment5。"""

    if _state == S["RIGHT_RETURN_Y"]:
        return _go_y(position, rpy, BRIDGE_SWITCH_POINT[1], HEADING_SOUTH, S["RIGHT_TURN_EAST"], "route_right_return_switch_y_reached")
    if _state == S["RIGHT_TURN_EAST"]:
        return _turn_state(rpy, HEADING_EAST, S["BRIDGE_TO_X"], "route_right_turn_east_done", position)
    if _state == S["BRIDGE_TO_X"]:
        return _go_x(position, rpy, BRIDGE_APPROACH_POINT[0], HEADING_EAST, S["BRIDGE_TURN_UP"], "route_bridge_x_reached")
    if _state == S["BRIDGE_TURN_UP"]:
        step = _turn_to(rpy, SEG5_ENTRY_HEADING)
        if step == 1:
            _set_state(S["DONE"], "route_seg5_entry_aligned", position)
            return _return_step(-1, "segment4_done_seg5_entry_aligned", position, rpy)
        return _return_step(step, "route_seg5_entry_align_turn", position, rpy)

    return None


def _route(position, rpy, frame):
    """第四段总调度：按状态把控制权分发给各条竖道的小状态机。"""
    _log_event("ROUTE", state=_state, pos=_fmt_pos(position), rpy=f"{rpy:.1f}")

    if _state == S["TO_START"]:
        return _route_entry(position, rpy)

    for handler in (
        _route_left_lane,
        _route_mid_lane,
        _route_right_lane,
        _route_bridge_exit,
    ):
        step = handler(position, rpy, frame) if handler is not _route_bridge_exit else handler(position, rpy)
        if step is not None:
            return step

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

    if _state == S["RIGHT_TO_LOW_START"]:
        return _route(position, rpy, frame)

    switching_gait = (gait == 0 and mode == 0) or (gait == 1 and mode == 9)
    if switching_gait:
        if _state == S["LEFT_BAR_UP"]:
            if y >= LEFT_BAR_DETECT_UP_Y:
                return _route(position, rpy, frame)
            return _return_step(_forward_step(rpy, HEADING_NORTH), "gait_switch_left_bar_forward", position, rpy)
        if _state == S["LEFT_BAR_UP_LOW"]:
            if y >= LEFT_BAR_LOW_END_Y:
                return _route(position, rpy, frame)
            return _return_step(LOW_BAR_GAIT, "gait_switch_keep_left_bar_up_low", position, rpy)
        if _state == S["LEFT_BAR_DOWN"]:
            if y <= LEFT_BAR_DETECT_DOWN_Y:
                return _route(position, rpy, frame)
            return _return_step(_forward_step(rpy, HEADING_SOUTH), "gait_switch_left_bar_down_forward", position, rpy)
        if _state == S["LEFT_BAR_DOWN_LOW"]:
            if y <= LEFT_BAR["y_min"]:
                return _route(position, rpy, frame)
            return _return_step(LOW_BAR_GAIT, "gait_switch_keep_left_bar_down_low", position, rpy)
        if _state == S["LEFT_RETURN_Y"]:
            if y <= LEFT_TO_MID_START_Y:
                return _route(position, rpy, frame)
            return _return_step(_forward_step(rpy, HEADING_SOUTH), "gait_switch_left_return_to_8_40", position, rpy)
        if _state == S["RIGHT_BACKUP_LOW"]:
            return _return_step(LOW_BAR_GAIT, "gait_switch_keep_low_return", position, rpy)
        if _state in RIGHT_LOW_REQUIRED_STATES and y > RIGHT_LOW_START_Y:
            return _return_step(LOW_BAR_GAIT, "gait_switch_keep_right_low", position, rpy)
        if _state in LOW_STATES:
            return _return_step(LOW_BAR_GAIT, "gait_switch_keep_low", position, rpy)
        return _return_step(0, "gait_switch_wait", position, rpy)

    return _route(position, rpy, frame)
