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
import shutil
import subprocess

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
        "y_min": 10.60,
        "y_max": 11.60,
        "strike": 0.45,
        "gait": 1,
    },
    {
        "name": "橙色小球",
        "kind": "orange_ball",
        "x": MID_LANE_X,
        "y_min": 10.60,
        "y_max": 11.60,
        "strike": 0.45,
        "gait": 1,
    },
    {
        "name": "足球",
        "kind": "football",
        "x": RIGHT_LANE_X,
        "y_min": 10.10,
        "y_max": 11.10,
        "strike": 0.85,
        "gait": 28,
    },
]

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

BRIDGE_START = (BRIDGE_LANE_X, 7.60)

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

_state = _ST_GOTO_OBSTACLE
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


def reset_segment4():
    """重置第四赛段状态。"""
    global _state, _obstacle_idx, _target_idx, _scan_count, _motion_start, _announced
    global _bypass_points, _bypass_idx, _front_bar_low_active
    global _front_bar_low_start, _front_bar_low_heading
    global _shake_phase, _shake_start
    _state = _ST_GOTO_OBSTACLE
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
            return LOW_BAR_GAIT
        if abs(y - ty) > XY_TOL:
            return _goto_xy_direct(position, rpy, (x, ty))
        return 0

    if abs(x - tx) > XY_TOL and y < LANE_SWITCH_Y - XY_TOL:
        step = _turn_to(rpy, 90)
        return step if step != 1 else 1

    if abs(x - tx) > XY_TOL and y > LANE_SWITCH_Y + XY_TOL:
        return _goto_xy_direct(position, rpy, (x, LANE_SWITCH_Y))

    return _goto_xy_direct(position, rpy, target)


def _goto_obstacle(position, rpy, obstacle):
    x, y, _ = position
    tx, ty = _obstacle_approach_point(obstacle)

    if _obstacle_idx == 0:
        if y < LANE_SWITCH_Y - XY_TOL:
            step = _turn_to(rpy, 90)
            return step if step != 1 else 1
        if abs(x - tx) > XY_TOL:
            target_hdg = 0 if tx > x else 180
            step = _turn_to(rpy, target_hdg)
            return step if step != 1 else 1
        if abs(y - ty) > XY_TOL:
            step = _turn_to(rpy, 90)
            return step if step != 1 else 1
        return 0

    return _goto_xy(position, rpy, (tx, ty))


def _goto_xy_direct(position, rpy, target):
    """直接走向目标点，用于绕障过程中的虚线借道。"""
    x, y, _ = position
    tx, ty = target

    if abs(x - tx) > XY_TOL:
        target_hdg = 0 if tx > x else 180
        step = _turn_to(rpy, target_hdg)
        return step if step != 1 else 1

    if abs(y - ty) > XY_TOL:
        low_bar_step = _low_bar_crossing_step(position, rpy, target)
        if low_bar_step is not None:
            return low_bar_step
        target_hdg = 90 if ty > y else 270
        step = _turn_to(rpy, target_hdg)
        return step if step != 1 else 1

    return 0


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
    mask = cv2.inRange(hsv, np.array([0, 0, 0]), np.array([180, 255, 70]))
    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    for cnt in contours:
        if cv2.contourArea(cnt) < 180:
            continue
        _, _, w, h = cv2.boundingRect(cnt)
        if h > w * 1.6:
            return True
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


def _finish_obstacle():
    global _state, _obstacle_idx
    _obstacle_idx += 1
    _state = _ST_GOTO_TARGET if _target_idx < len(TARGETS) else _ST_GOTO_OBSTACLE
    return 0


def _finish_target():
    global _state, _target_idx, _motion_start
    _target_idx += 1
    _motion_start = None
    _state = _ST_GOTO_OBSTACLE if _obstacle_idx < len(OBSTACLES) else _ST_GOTO_BRIDGE
    return 0


def segment4_control(position, gait_mode, rpy, frame=None):
    """
    返回第四赛段当前应该执行的步态编号。

    返回 -1 表示第四赛段完成，可以进入第五赛段。
    """
    global _state, _obstacle_idx, _target_idx, _scan_count, _motion_start
    global _bypass_points, _bypass_idx, _front_bar_low_active
    global _front_bar_low_start, _front_bar_low_heading
    global _shake_phase, _shake_start

    x, y, _ = position
    gait, mode = gait_mode

    if (
        (gait == 0 and mode == 0) or (gait == 1 and mode == 9)
    ) and (_state == _ST_PASS_LOW_BAR or _front_bar_low_active):
        return LOW_BAR_GAIT

    if (gait == 0 and mode == 0) or (gait == 1 and mode == 9):
        return 0

    if _front_bar_low_active:
        if _front_bar_low_start is not None and _distance(position, _front_bar_low_start) >= LOW_BAR_DETECT_CLEAR_DIST:
            _front_bar_low_active = False
            _front_bar_low_start = None
            return 1
        return LOW_BAR_GAIT

    expected_bar = (
        _obstacle_idx < len(OBSTACLES)
        and OBSTACLES[_obstacle_idx]["type"] == "bar"
        and _state in (_ST_GOTO_OBSTACLE, _ST_SCAN_OBSTACLE)
    )
    bar_close_enough = expected_bar and y >= OBSTACLES[_obstacle_idx]["y_min"] - BAR_DETECT_DIST

    if bar_close_enough and _detect_limit_bar_ahead(frame):
        if "视觉限高杆" not in _announced:
            _announced.add("视觉限高杆")
            _speak("限高杆")
        if expected_bar:
            _state = _ST_PASS_LOW_BAR
            _motion_start = [x, y]
            return LOW_BAR_GAIT
        _front_bar_low_active = True
        _front_bar_low_start = [x, y]
        _front_bar_low_heading = _nearest_cardinal(rpy)
        return LOW_BAR_GAIT

    if _obstacle_idx < len(OBSTACLES):
        obstacle = OBSTACLES[_obstacle_idx]

        if _state == _ST_GOTO_OBSTACLE:
            step = _goto_obstacle(position, rpy, obstacle)
            if step == 0:
                _state = _ST_SCAN_OBSTACLE
                _scan_count = 0
                if obstacle["type"] == "bar":
                    _state = _ST_PASS_LOW_BAR
                    _motion_start = [x, y]
                    if obstacle["name"] not in _announced:
                        _announced.add(obstacle["name"])
                        _speak(obstacle["speak"])
                    return LOW_BAR_GAIT
                return 0
            return step

        if _state == _ST_SCAN_OBSTACLE:
            _scan_count += 1
            detected = _detect_obstacle(frame)
            if obstacle["type"] == "block":
                fallback_confirmed = _scan_count >= BLOCK_SCAN_LIMIT
            else:
                range_finished = y >= obstacle["y_max"]
                fallback_confirmed = range_finished or _scan_count >= SCAN_LIMIT * 8
            if detected or fallback_confirmed:
                if detected and obstacle["name"] not in _announced:
                    _announced.add(obstacle["name"])
                    _speak(obstacle["speak"])
                if obstacle["type"] == "bar":
                    _state = _ST_PASS_LOW_BAR
                    _motion_start = [x, y]
                else:
                    _state = _ST_BYPASS_BLOCK
                    _bypass_points = _make_bypass_points(obstacle)
                    _bypass_idx = 0
                return 0
            if obstacle["type"] == "block":
                return 0
            return _scan_forward(position, rpy, obstacle["x"], obstacle["y_max"])

        if _state == _ST_PASS_LOW_BAR:
            if _dist_from_start(position) >= BAR_PASS_DIST:
                return _finish_obstacle()
            return LOW_BAR_GAIT

        if _state == _ST_BYPASS_BLOCK:
            if _bypass_idx >= len(_bypass_points):
                return _finish_obstacle()
            step = _goto_xy_direct(position, rpy, _bypass_points[_bypass_idx])
            if step == 0:
                _bypass_idx += 1
                return 0
            return step

    if _target_idx < len(TARGETS):
        target = TARGETS[_target_idx]
        target_start = (target["x"], target["y_min"])

        if _state == _ST_GOTO_TARGET:
            step = _goto_xy(position, rpy, target_start)
            if step == 0:
                _state = _ST_SCAN_TARGET
                _scan_count = 0
                return 0
            return step

        if _state == _ST_SCAN_TARGET:
            _scan_count += 1
            detected = _detect_target(frame, target["kind"])
            range_finished = y >= target["y_max"]
            fallback_confirmed = range_finished or _scan_count >= SCAN_LIMIT * 8
            if detected:
                if target["name"] not in _announced:
                    _announced.add(target["name"])
                    _speak(target["name"])
                _state = _ST_SHAKE if target["kind"] == "orange_ball" else _ST_STRIKE
                _motion_start = [x, y]
                _shake_phase = 0
                _shake_start = None
                return 0
            if fallback_confirmed:
                return _finish_target()
            return _scan_forward(position, rpy, target["x"], target["y_max"])

        if _state == _ST_STRIKE:
            step = _turn_to(rpy, HEADING_FRONT)
            if step != 1:
                return step
            if _dist_from_start(position) >= target["strike"]:
                _state = _ST_BACKUP
                _motion_start = [x, y]
                return 0
            return target["gait"]

        if _state == _ST_SHAKE:
            step = _turn_to(rpy, HEADING_FRONT)
            if step != 1:
                return step
            step = _shake_step(position)
            if step == 0:
                _state = _ST_BACKUP
                _motion_start = [x, y]
                _shake_phase = 0
                _shake_start = None
                return 0
            return step

        if _state == _ST_BACKUP:
            if _dist_from_start(position) >= 0.28:
                return _finish_target()
            return 6

    if _state != _ST_GOTO_BRIDGE:
        _state = _ST_GOTO_BRIDGE

    step = _goto_xy(position, rpy, BRIDGE_START)
    if step == 0:
        _state = _ST_DONE
        return -1
    return step
