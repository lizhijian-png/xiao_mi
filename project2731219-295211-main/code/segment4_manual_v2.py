"""第四赛段（人工输入版 V2）：新规则 - 限高杆可能在任意两道。

新规则改动：
  - 限高杆不一定只在左道和右道，可能出现在中间道
  - 两个限高杆随机分布在三条道中的任意两条
  - 没有限高杆的道只能通过 y=8.6~9.1 的小口进出
  - 当限高杆出现在中间道时，左道或右道之一没有限高杆，该道通过小口进出

用法：
  python3 segment4_manual_v2.py <mid_open> <cola_pos> <football_pos> <orange_pos> <no_bar_lane>

命令行 5 个整数：
  mid_open       开口侧（当无杆道为中道时）  0=左侧，1=右侧
  cola_pos       可乐所在道                  1=左，2=中，3=右
  football_pos   足球所在道                  1=左，2=中，3=右
  orange_pos     橙色小球所在道              1=左，2=中，3=右
  no_bar_lane    没有限高杆的道              1=左，2=中，3=右

例：`python3 segment4_manual_v2.py 0 1 2 3 2`
  = 中道没有限高杆（通过小口进出），可乐在左道，足球在中道，橙球在右道
    中道开口侧=0（左侧）表示小口在中道左侧，即左道可以直接通过小口进中道

规则推导：
  - no_bar_lane=1 → 左道无杆，左道通过小口进出（小口在左道右侧=中道左侧）
  - no_bar_lane=2 → 中道无杆，mid_open决定小口位置（0=左，1=右）
  - no_bar_lane=3 → 右道无杆，右道通过小口进出（小口在右道左侧=中道右侧）

访问策略：
  - 有杆的道：正常通过（蹲姿过杆）
  - 无杆的道：必须通过小口进出（y=8.85附近）
"""

import math
import os
import shutil
import subprocess
import sys
import time

try:
    import cv2
    import numpy as np
except ImportError:
    cv2 = None
    np = None


# ── 场地几何 ──────────────────────────────────────────────────────
LEFT_LANE_X = -0.10
MID_LANE_X = 1.00
RIGHT_LANE_X = 2.10

# 踢球偏移：足球放在道路中心线上，机器狗需偏移 10cm 让脚能踢到球
# 偏移方向：左道向右偏（向中道），右道向左偏（向中道），中道向左偏（可根据效果调整）
# 原理：让球在机器狗身体一侧而非中间，避免球被推过去而是被脚踢开
LEFT_LANE_FOOTBALL_OFFSET = +0.10   # 向右偏：-0.10 + 0.10 = 0.00
MID_LANE_FOOTBALL_OFFSET = -0.10    # 向左偏：1.00 - 0.10 = 0.90 (可改为 +0.10 向右偏)
RIGHT_LANE_FOOTBALL_OFFSET = -0.10  # 向左偏：2.10 - 0.10 = 2.00

LANE_SWITCH_Y = 7.20          # 底部横向通道
START_POINT = (3.10, 7.10)    # 第四段入口

# 小口开口：y ∈ [8.6, 9.1]，进出点取 8.90（给下方留更多余量）
OPENING_Y = 8.90

# 小口物理中心位置：0.45(左侧小口) / 1.55(右侧小口)
# 判定 x 值需要根据机器狗通过方向偏移 10cm：
#   - 中道无杆（从中道内出小口）：偏向道外 10cm，方便机器狗完全出小口
#   - 左/右道无杆（从边道进/出小口）：偏向中道 10cm，方便机器狗从边道横穿到中道侧
MID_OPENING_LEFT_EXIT_X = 0.35    # 中道无杆+小口左：中道内→道外 (0.45-0.10)
MID_OPENING_RIGHT_EXIT_X = 1.65   # 中道无杆+小口右：中道内→道外 (1.55+0.10)
LEFT_LANE_OPENING_X = 0.55        # 左道无杆：小口在中道一侧的判定点 (0.45+0.10)
RIGHT_LANE_OPENING_X = 1.45       # 右道无杆：小口在中道一侧的判定点 (1.55-0.10)

# 限高杆：随机范围 9.1~10.1，有杆道才需要蹲
BAR_Y_MIN = 9.10
BAR_Y_MAX = 10.10
LOW_START_Y = 9.00
LOW_END_Y = 10.30

# 目标物坐标
TARGET_Y = {
    "cola": 11.10,
    "football": 11.00,
    "orange": 11.00,
}
ANNOUNCE_Y = {
    "cola": 10.90,
    "football": 10.90,
    "orange": 10.80,
}
BACKUP_DIST = {
    "cola": 0.15,
    "football": 0.13,
    "orange": 0.20,
}
TARGET_NAME_CN = {
    "cola": "可乐瓶",
    "football": "足球",
    "orange": "橙色小球",
}

# 步态编号
S4_FORWARD_LEFT_GAIT = 42
S4_FORWARD_RIGHT_GAIT = 43
S4_FAST_FORWARD_GAIT = 44
S4_BACKUP_GAIT = 45
S4_STRAFE_LEFT_GAIT = 46
S4_STRAFE_RIGHT_GAIT = 47
LOW_BAR_GAIT = 5
ORANGE_JUMP_GAIT = 57
JUMP_FRAMES = 8

HEADING_EAST = 0
HEADING_NORTH = 90
HEADING_WEST = 180
HEADING_SOUTH = 270

FAST_DEG = 18
SLOW_DEG = 6
XY_TOL = 0.08
LAT_TOLERANCE = 0.02
INERTIA_MARGIN = 0.02
LOW_STAND_SETTLE_FRAMES = 3

LOG_PATH = os.path.abspath(
    os.path.join(os.path.dirname(__file__), "..", "log", "segment4_v2_log.txt")
)


# ── 状态名 ────────────────────────────────────────────────────────
S = {
    "TO_START": "S4V2_TO_START",
    # 有杆道（左/右/中任一有杆的道）
    "LANE_ENTER_X": "S4V2_LANE_ENTER_X",
    "LANE_TURN_NORTH": "S4V2_LANE_TURN_NORTH",
    "LANE_ALIGN_UP": "S4V2_LANE_ALIGN_UP",
    "LANE_UP_BEFORE_BAR": "S4V2_LANE_UP_BEFORE_BAR",
    "LANE_UP_LOW": "S4V2_LANE_UP_LOW",
    "LANE_STAND_AFTER_BAR": "S4V2_LANE_STAND_AFTER_BAR",
    "LANE_ADVANCE_TO_TARGET": "S4V2_LANE_ADVANCE_TO_TARGET",
    "LANE_BACKUP_TARGET": "S4V2_LANE_BACKUP_TARGET",
    "LANE_JUMP": "S4V2_LANE_JUMP",
    # 足球专用（有杆道）
    "LANE_FOOTBALL_LOW_BACKUP": "S4V2_LANE_FOOTBALL_LOW_BACKUP",
    "LANE_FOOTBALL_TURN_SOUTH": "S4V2_LANE_FOOTBALL_TURN_SOUTH",
    "LANE_FOOTBALL_ALIGN_DOWN": "S4V2_LANE_FOOTBALL_ALIGN_DOWN",
    "LANE_FOOTBALL_DOWN_LOW": "S4V2_LANE_FOOTBALL_DOWN_LOW",
    # 有杆道下行
    "LANE_TURN_SOUTH": "S4V2_LANE_TURN_SOUTH",
    "LANE_ALIGN_DOWN": "S4V2_LANE_ALIGN_DOWN",
    "LANE_DOWN_BEFORE_BAR": "S4V2_LANE_DOWN_BEFORE_BAR",
    "LANE_DOWN_LOW": "S4V2_LANE_DOWN_LOW",
    "LANE_STAND_AFTER_DOWN": "S4V2_LANE_STAND_AFTER_DOWN",
    "LANE_RETURN_Y": "S4V2_LANE_RETURN_Y",
    # 无杆道（通过小口进出）
    "OPENING_PREPARE_X": "S4V2_OPENING_PREPARE_X",     # 先横移到小口 x 位置（避开墙）
    "OPENING_PREPARE_TURN": "S4V2_OPENING_PREPARE_TURN",  # 转向北
    "OPENING_ENTER_Y": "S4V2_OPENING_ENTER_Y",         # 向北到小口高度
    "OPENING_CROSS_IN": "S4V2_OPENING_CROSS_IN",
    "OPENING_TURN_NORTH": "S4V2_OPENING_TURN_NORTH",
    "OPENING_ALIGN_UP": "S4V2_OPENING_ALIGN_UP",
    "OPENING_ADVANCE": "S4V2_OPENING_ADVANCE",
    "OPENING_BACKUP": "S4V2_OPENING_BACKUP",
    "OPENING_JUMP": "S4V2_OPENING_JUMP",
    "OPENING_TURN_SOUTH": "S4V2_OPENING_TURN_SOUTH",
    "OPENING_ALIGN_DOWN": "S4V2_OPENING_ALIGN_DOWN",
    "OPENING_RETURN": "S4V2_OPENING_RETURN",
    "OPENING_CROSS_OUT": "S4V2_OPENING_CROSS_OUT",
    "OPENING_EXIT_SOUTH": "S4V2_OPENING_EXIT_SOUTH",
    "OPENING_TO_BOTTOM": "S4V2_OPENING_TO_BOTTOM",
    # 桥
    "BRIDGE_TO_X": "S4V2_BRIDGE_TO_X",
    "BRIDGE_TURN_UP": "S4V2_BRIDGE_TURN_UP",
    "DONE": "S4V2_DONE",
}

LOW_STATES = {
    S["LANE_UP_LOW"],
    S["LANE_DOWN_LOW"],
    S["LANE_FOOTBALL_ALIGN_DOWN"],
    S["LANE_FOOTBALL_DOWN_LOW"],
}

BRIDGE_APPROACH_POINT = (3.15, LANE_SWITCH_Y)
SEG5_ENTRY_HEADING = 90


# ── 全局状态 ──────────────────────────────────────────────────────
_state = S["TO_START"]
_lane_order = []
_lane_iter = 0
_current_lane = None
_current_target = None
_current_has_bar = False
_mid_open = "left"
_preset = None

_motion_start = None
_stand_count = 0
_jump_frames = 0
_announced = set()
_last_log_time = 0.0
_last_log_signature = None


# ── 参数解析 ──────────────────────────────────────────────────────
def parse_preset_args(argv):
    """解析 5 个整数参数，返回 preset dict。

    返回：
      {
        "mid_open": "left" | "right",
        "no_bar_lane": "left" | "mid" | "right",
        "lane_of": {"cola": "left"/"mid"/"right",
                    "football": ...,
                    "orange": ...},
      }
    """
    if len(argv) != 5:
        raise ValueError(
            f"需要 5 个整数(mid_open cola football orange no_bar_lane)，实际收到 {len(argv)} 个: {argv}"
        )
    try:
        nums = [int(a) for a in argv]
    except ValueError as e:
        raise ValueError(f"参数必须是整数，收到: {argv}") from e

    mid_open, cola, football, orange, no_bar_lane = nums
    if mid_open not in (0, 1):
        raise ValueError(f"mid_open 必须是 0(左) 或 1(右)，收到: {mid_open}")
    if no_bar_lane not in (1, 2, 3):
        raise ValueError(f"no_bar_lane 必须是 1(左)、2(中) 或 3(右)，收到: {no_bar_lane}")
    for name, v in (("cola", cola), ("football", football), ("orange", orange)):
        if v not in (1, 2, 3):
            raise ValueError(f"{name} 位置必须在 1~3 之间，收到: {v}")
    if len({cola, football, orange}) != 3:
        raise ValueError(
            f"三个目标物位置必须互不相同，收到 cola={cola} football={football} orange={orange}"
        )

    lane_name = {1: "left", 2: "mid", 3: "right"}
    return {
        "mid_open": "left" if mid_open == 0 else "right",
        "no_bar_lane": lane_name[no_bar_lane],
        "lane_of": {
            "cola": lane_name[cola],
            "football": lane_name[football],
            "orange": lane_name[orange],
        },
    }


def reset_segment4_v2(preset):
    """重置状态机。preset 由 parse_preset_args 生成。"""
    global _state, _lane_order, _lane_iter, _current_lane, _current_target
    global _current_has_bar, _mid_open, _preset, _motion_start, _stand_count
    global _jump_frames, _announced, _last_log_time, _last_log_signature

    _preset = preset
    _mid_open = preset["mid_open"]
    no_bar = preset["no_bar_lane"]

    # 决定访问顺序
    if no_bar == "left":
        _lane_order = ["left", "mid", "right"]
    elif no_bar == "right":
        _lane_order = ["left", "mid", "right"]
    elif no_bar == "mid":
        _lane_order = ["left", "mid", "right"] if _mid_open == "left" else ["left", "right", "mid"]

    _lane_iter = 0
    _current_lane = None
    _current_target = None
    _current_has_bar = False
    _motion_start = None
    _stand_count = 0
    _jump_frames = 0
    _announced = set()
    _last_log_time = 0.0
    _last_log_signature = None
    _state = S["TO_START"]
# ── 工具函数 ──────────────────────────────────────────────────────
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
    old = _state
    _state = new_state
    _log_event(
        "STATE",
        old=old, new=new_state, reason=reason,
        pos=_fmt_pos(position) if position is not None else "-",
    )


def _return_step(step, reason, position=None, rpy=None):
    _log_event(
        "STEP",
        step=step, reason=reason, state=_state,
        pos=_fmt_pos(position) if position is not None else "-",
        rpy=f"{rpy:.1f}" if rpy is not None else "-",
    )
    return step


def _norm(angle):
    while angle > 180:
        angle -= 360
    while angle <= -180:
        angle += 360
    return angle


def _turn_to(rpy, target_hdg):
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
    global _motion_start
    _motion_start = [position[0], position[1]]


def _speak(text):
    print(f"语音播报：识别到{text}")
    cmd = shutil.which("spd-say") or shutil.which("espeak")
    if cmd:
        try:
            subprocess.Popen([cmd, f"识别到{text}"])
        except Exception:
            pass


def _announce_once(key, text):
    if key in _announced:
        return
    _announced.add(key)
    _speak(text)
    _log_event("ANNOUNCE", key=key, text=text)


def _forward_step(rpy, heading, gait=S4_FAST_FORWARD_GAIT):
    step = _turn_to(rpy, heading)
    return step if step != 1 else gait


def _forward_with_lateral(position, rpy, heading, center_val, axis, gait=S4_FAST_FORWARD_GAIT, tolerance=LAT_TOLERANCE):
    """带横向纠偏的前进控制（参考 segment5._forward_with_lateral）。

    axis 指定横向纠偏轴：'x'（沿 y 行进，纠 x 偏移）或 'y'（沿 x 行进，纠 y 偏移）。
    朝向未对准则先转向；对准则按横向偏移发前进+左/右纠偏步态。
    """
    d = _norm(rpy - (heading % 360))
    if d > FAST_DEG:
        return 15
    if d > SLOW_DEG:
        return 3
    if d < -FAST_DEG:
        return 14
    if d < -SLOW_DEG:
        return 2

    current_val = position[0] if axis == 'x' else position[1]
    offset = current_val - center_val

    # 世界坐标偏移 → 机身左右方向：东(0°)右=y-，北(90°)右=x+，西(180°)右=y+，南(270°)右=x-
    if heading == HEADING_EAST:
        lateral = -offset
    elif heading == HEADING_NORTH:
        lateral = offset
    elif heading == HEADING_WEST:
        lateral = offset
    elif heading == HEADING_SOUTH:
        lateral = -offset
    else:
        lateral = 0.0

    if lateral > tolerance:
        return S4_FORWARD_LEFT_GAIT    # 偏右 → 左纠偏
    if lateral < -tolerance:
        return S4_FORWARD_RIGHT_GAIT   # 偏左 → 右纠偏
    return gait


def _forward_lane_step(position, rpy, heading, lane_x, gait=S4_FAST_FORWARD_GAIT):
    if gait == LOW_BAR_GAIT:
        return gait  # 低姿态区间不纠偏，避免蹲走中机身抬高
    return _forward_with_lateral(position, rpy, heading, lane_x, 'x', gait)


def _turn_state(rpy, heading, next_state, reason, position):
    step = _turn_to(rpy, heading)
    if step == 1:
        _set_state(next_state, reason, position)
        return _return_step(0, reason, position, rpy)
    return _return_step(step, f"{reason}_turn", position, rpy)


def _stand_then(next_state, reason, position, rpy, frames=LOW_STAND_SETTLE_FRAMES):
    global _stand_count
    _stand_count += 1
    if _stand_count >= frames:
        _stand_count = 0
        _set_state(next_state, reason, position)
    return _return_step(0, reason, position, rpy)


def _go_x(position, rpy, target_x, heading, next_state, reason, center_y=None):
    """沿 x 方向走到目标 x；传 center_y 时同时做 y 方向横向纠偏。"""
    x = position[0]
    tol = XY_TOL + INERTIA_MARGIN
    reached = x <= target_x + tol if heading == HEADING_WEST else x >= target_x - tol
    if reached:
        _set_state(next_state, reason, position)
        return _return_step(0, reason, position, rpy)
    if center_y is not None:
        step = _forward_with_lateral(position, rpy, heading, center_y, 'y')
    else:
        step = _forward_step(rpy, heading)
    return _return_step(step, f"{reason}_move_x", position, rpy)


def _adjust_x(position, rpy, lane_x, heading, next_state, reason):
    x = position[0]
    step = _turn_to(rpy, heading)
    if step != 1:
        return _return_step(step, f"{reason}_align_heading", position, rpy)
    x_err = x - lane_x
    if abs(x_err) <= XY_TOL:
        _set_state(next_state, reason, position)
        return _return_step(0, reason, position, rpy)
    if heading == HEADING_NORTH:
        lateral = S4_STRAFE_LEFT_GAIT if x_err > 0 else S4_STRAFE_RIGHT_GAIT
    elif heading == HEADING_SOUTH:
        lateral = S4_STRAFE_RIGHT_GAIT if x_err > 0 else S4_STRAFE_LEFT_GAIT
    else:
        lateral = S4_STRAFE_LEFT_GAIT if x_err > 0 else S4_STRAFE_RIGHT_GAIT
    return _return_step(lateral, f"{reason}_strafe_x", position, rpy)


def _backup_to_distance(position, next_state, reason, distance):
    if _dist_from_start(position) >= distance:
        _set_state(next_state, reason, position)
        _motion_start_reset(position)
        return _return_step(0, reason, position)
    return _return_step(S4_BACKUP_GAIT, reason, position)


# ── 视觉检测（兼容无 opencv 环境） ────────────────────────────────
def _central_roi(frame):
    h, w = frame.shape[:2]
    return frame[h // 5: 4 * h // 5, w // 4: 3 * w // 4]


def _detect_cola(frame):
    if frame is None or cv2 is None:
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
    if frame is None or cv2 is None:
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
    if frame is None or cv2 is None:
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


def _detect_limit_bar_ahead(frame):
    if frame is None or cv2 is None:
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
    if kind == "cola":
        return _detect_cola(frame)
    if kind == "orange":
        return _detect_orange_ball(frame)
    if kind == "football":
        return _detect_football(frame)
    return False


# ── V2 特有辅助函数 ────────────────────────────────────────────────
def _has_bar(lane):
    """判断指定道是否有限高杆"""
    return lane != _preset["no_bar_lane"]


def _needs_opening(lane):
    """判断指定道是否需要通过小口进出"""
    return lane == _preset["no_bar_lane"]


def _get_opening_x(lane):
    """获取指定道的小口判定 x 坐标

    小口物理中心：0.45(左) / 1.55(右)
    判定 x 根据机器狗通过方向偏移 10cm：

    返回：
      - 左道无杆：小口中道侧判定点 0.55 (物理中心+10cm，方便左道横穿到中道侧)
      - 中道无杆+小口左：中道外出口 0.35 (物理中心-10cm，方便中道内出小口)
      - 中道无杆+小口右：中道外出口 1.65 (物理中心+10cm，方便中道内出小口)
      - 右道无杆：小口中道侧判定点 1.45 (物理中心-10cm，方便右道横穿到中道侧)
    """
    no_bar = _preset["no_bar_lane"]

    if no_bar == "left":
        return LEFT_LANE_OPENING_X  # 0.55
    elif no_bar == "right":
        return RIGHT_LANE_OPENING_X  # 1.45
    elif no_bar == "mid":
        mid_open = _preset["mid_open"]
        return MID_OPENING_LEFT_EXIT_X if mid_open == "left" else MID_OPENING_RIGHT_EXIT_X
    return MID_LANE_X


def _lane_x(lane):
    """获取指定道的原始 x 坐标（中心线）"""
    return {"left": LEFT_LANE_X, "mid": MID_LANE_X, "right": RIGHT_LANE_X}[lane]


def _get_effective_lane_x(lane, target):
    """获取实际使用的道路 x 坐标（考虑足球踢球偏移）

    足球在道路中心，机器狗需偏离中心让脚能踢到球：
      - 左道 + 足球：0.00 (原始-0.10 + 偏移+0.10，向中道/右偏)
      - 中道 + 足球：0.90 (原始 1.00 + 偏移-0.10，向左偏，可改为+0.10向右偏)
      - 右道 + 足球：2.00 (原始 2.10 + 偏移-0.10，向中道/左偏)

    原理：
      - 偏移让球在机器狗身体一侧（而非中间两腿之间）
      - 避免球被推过去，而是被外侧脚踢开
      - 左道和右道都向中道方向偏，让球在场地外侧

    其他目标物（可乐、橙球）：使用道路原始中心线
    """
    if target == "football":
        base_x = _lane_x(lane)
        if lane == "left":
            return base_x + LEFT_LANE_FOOTBALL_OFFSET   # 0.00
        elif lane == "mid":
            return base_x + MID_LANE_FOOTBALL_OFFSET    # 0.90
        elif lane == "right":
            return base_x + RIGHT_LANE_FOOTBALL_OFFSET  # 2.00
    return _lane_x(lane)


# ── 路由辅助 ──────────────────────────────────────────────────────
def _begin_lane():
    """切换到下一条竖道，初始化该道的全局配置。"""
    global _lane_iter, _current_lane, _current_target, _current_has_bar, _state
    global _motion_start, _stand_count

    lane = _lane_order[_lane_iter]
    _current_lane = lane

    # 找到该道的目标物
    for target, ln in _preset["lane_of"].items():
        if ln == lane:
            _current_target = target
            break

    # 判断该道是否有杆
    _current_has_bar = _has_bar(lane)
    _motion_start = None
    _stand_count = 0

    # 根据是否需要小口进入不同状态
    if _needs_opening(lane):
        _set_state(S["OPENING_PREPARE_X"], f"begin_opening_lane_{lane}")
    else:
        _set_state(S["LANE_ENTER_X"], f"begin_bar_lane_{lane}")


def _route_entry(position, rpy):
    """入口：先修正 y 到底部横道，然后进入第一条竖道。"""
    _, y, _ = position
    if abs(y - START_POINT[1]) > XY_TOL:
        heading = HEADING_NORTH if START_POINT[1] > y else HEADING_SOUTH
        step = _forward_with_lateral(position, rpy, heading, START_POINT[0], 'x')
        return _return_step(step, "route_to_start_fix_y", position, rpy)
    _begin_lane()
    return _return_step(0, "route_start_reached", position, rpy)


def _route_bridge(position, rpy):
    """收尾：走底部横道到桥入口，转正交接给第五段。"""
    if _state == S["BRIDGE_TO_X"]:
        return _go_x(position, rpy, BRIDGE_APPROACH_POINT[0], HEADING_EAST,
                     S["BRIDGE_TURN_UP"], "bridge_x_reached", center_y=LANE_SWITCH_Y)
    if _state == S["BRIDGE_TURN_UP"]:
        step = _turn_to(rpy, SEG5_ENTRY_HEADING)
        if step == 1:
            _set_state(S["DONE"], "bridge_turn_up_done", position)
            return _return_step(-1, "segment4_v2_done", position, rpy)
        return _return_step(step, "bridge_turn_up_align", position, rpy)
    return None
def _route_general_lane(position, rpy, frame):
    """左/右竖道状态机（必有杆）。"""
    global _stand_count, _motion_start, _lane_iter, _jump_frames

    x, y, _ = position
    target = _current_target
    # 考虑足球踢球偏移：右道+足球时使用 2.00 而非 2.10
    lane_x = _get_effective_lane_x(_current_lane, target)
    target_y = TARGET_Y[target]
    announce_y = ANNOUNCE_Y[target]
    backup_dist = BACKUP_DIST[target]
    backup_y = target_y + 0.05  # 撞击结束线（过目标物后多走 5cm）

    # ── 进入 + 上行 ──
    if _state == S["LANE_ENTER_X"]:
        heading = HEADING_WEST if lane_x < x else HEADING_EAST
        return _go_x(position, rpy, lane_x, heading, S["LANE_TURN_NORTH"], "lane_enter_x_reached", center_y=LANE_SWITCH_Y)
    if _state == S["LANE_TURN_NORTH"]:
        return _turn_state(rpy, HEADING_NORTH, S["LANE_ALIGN_UP"], "lane_turn_north_done", position)
    if _state == S["LANE_ALIGN_UP"]:
        return _adjust_x(position, rpy, lane_x, HEADING_NORTH, S["LANE_UP_BEFORE_BAR"], "lane_align_up_done")

    if _state == S["LANE_UP_BEFORE_BAR"]:
        if y >= LOW_START_Y:
            _announce_once(f"{_current_lane}_bar_up", "限高杆")
            _stand_count = 0
            _set_state(S["LANE_UP_LOW"], "lane_up_enter_low", position)
            return _return_step(LOW_BAR_GAIT, "lane_up_enter_low", position, rpy)
        return _return_step(_forward_lane_step(position, rpy, HEADING_NORTH, lane_x), "lane_up_before_bar", position, rpy)

    if _state == S["LANE_UP_LOW"]:
        if y >= LOW_END_Y:
            _stand_count = 0
            if target == "football":
                _set_state(S["LANE_ADVANCE_TO_TARGET"], "lane_up_low_clear_football", position)
                return _return_step(LOW_BAR_GAIT, "lane_up_low_clear_football", position, rpy)
            _set_state(S["LANE_STAND_AFTER_BAR"], "lane_up_low_clear", position)
            return _return_step(0, "lane_up_low_clear", position, rpy)
        return _return_step(_forward_lane_step(position, rpy, HEADING_NORTH, lane_x, LOW_BAR_GAIT), "lane_up_low", position, rpy)

    if _state == S["LANE_STAND_AFTER_BAR"]:
        return _stand_then(S["LANE_ADVANCE_TO_TARGET"], "lane_stand_after_bar_done", position, rpy)

    if _state == S["LANE_ADVANCE_TO_TARGET"]:
        gait = LOW_BAR_GAIT if target == "football" else S4_FAST_FORWARD_GAIT
        detected = _detect_target(frame, target)
        _log_event("ROUTE_TARGET_SCAN", target=target, detected=detected, pos=_fmt_pos(position))
        if detected or y >= announce_y:
            _announce_once(target, TARGET_NAME_CN[target])
        if y >= backup_y:
            _motion_start_reset(position)
            if target == "orange":
                _jump_frames = 0
                _set_state(S["LANE_JUMP"], "lane_orange_jump_start", position)
                return _return_step(ORANGE_JUMP_GAIT, "lane_orange_jump_start", position, rpy)
            _set_state(S["LANE_BACKUP_TARGET"], "lane_target_reach_backup", position)
            return _return_step(0, "lane_target_reach_backup", position, rpy)
        return _return_step(_forward_lane_step(position, rpy, HEADING_NORTH, lane_x, gait), "lane_advance_to_target", position, rpy)

    # 悬挂小球：走到球正下方后原地跳高顶撞，落地后转南返程。
    if _state == S["LANE_JUMP"]:
        _jump_frames += 1
        if _jump_frames >= JUMP_FRAMES:
            _set_state(S["LANE_TURN_SOUTH"], "lane_orange_jump_done", position)
            return _return_step(0, "lane_orange_jump_done", position, rpy)
        return _return_step(ORANGE_JUMP_GAIT, "lane_orange_jump", position, rpy)

    if _state == S["LANE_BACKUP_TARGET"]:
        if target == "football":
            _motion_start_reset(position)
            _set_state(S["LANE_FOOTBALL_LOW_BACKUP"], "lane_football_low_backup_start", position)
            return _return_step(0, "lane_football_low_backup_start", position, rpy)
        return _backup_to_distance(position, S["LANE_TURN_SOUTH"], "lane_backup_target_done", backup_dist)

    # ── 足球专用（有杆）：撞完保持低姿态短退 0.13m，再转身低姿态退回过杆 ──
    if _state == S["LANE_FOOTBALL_LOW_BACKUP"]:
        return _backup_to_distance(position, S["LANE_FOOTBALL_TURN_SOUTH"], "lane_football_low_backup_done", FOOTBALL_BACKUP_AFTER_KICK_DIST)
    if _state == S["LANE_FOOTBALL_TURN_SOUTH"]:
        step = _turn_to(rpy, HEADING_SOUTH)
        if step == 1:
            _set_state(S["LANE_FOOTBALL_ALIGN_DOWN"], "lane_football_turn_south_done", position)
            return _return_step(LOW_BAR_GAIT, "lane_football_turn_south_done", position, rpy)
        return _return_step(step, "lane_football_turn_south_turn", position, rpy)
    if _state == S["LANE_FOOTBALL_ALIGN_DOWN"]:
        return _adjust_x(position, rpy, lane_x, HEADING_SOUTH, S["LANE_FOOTBALL_DOWN_LOW"], "lane_football_align_down_done")
    if _state == S["LANE_FOOTBALL_DOWN_LOW"]:
        _announce_once(f"{_current_lane}_bar_down", "限高杆")
        if y <= LOW_START_Y:
            _stand_count = 0
            _set_state(S["LANE_STAND_AFTER_DOWN"], "lane_football_down_low_done", position)
            return _return_step(0, "lane_football_down_low_done", position, rpy)
        return _return_step(_forward_lane_step(position, rpy, HEADING_SOUTH, lane_x, LOW_BAR_GAIT), "lane_football_down_low", position, rpy)

    # ── 下行 + 换道 ──
    if _state == S["LANE_TURN_SOUTH"]:
        return _turn_state(rpy, HEADING_SOUTH, S["LANE_ALIGN_DOWN"], "lane_turn_south_done", position)
    if _state == S["LANE_ALIGN_DOWN"]:
        return _adjust_x(position, rpy, lane_x, HEADING_SOUTH, S["LANE_DOWN_BEFORE_BAR"], "lane_align_down_done")

    if _state == S["LANE_DOWN_BEFORE_BAR"]:
        if y <= LOW_END_Y:
            _announce_once(f"{_current_lane}_bar_down", "限高杆")
            _stand_count = 0
            _set_state(S["LANE_DOWN_LOW"], "lane_down_enter_low", position)
            return _return_step(LOW_BAR_GAIT, "lane_down_enter_low", position, rpy)
        return _return_step(_forward_lane_step(position, rpy, HEADING_SOUTH, lane_x), "lane_down_before_bar", position, rpy)

    if _state == S["LANE_DOWN_LOW"]:
        if y <= LOW_START_Y:
            _stand_count = 0
            _set_state(S["LANE_STAND_AFTER_DOWN"], "lane_down_low_clear", position)
            return _return_step(0, "lane_down_low_clear", position, rpy)
        return _return_step(_forward_lane_step(position, rpy, HEADING_SOUTH, lane_x, LOW_BAR_GAIT), "lane_down_low", position, rpy)

    if _state == S["LANE_STAND_AFTER_DOWN"]:
        return _stand_then(S["LANE_RETURN_Y"], "lane_stand_after_down_done", position, rpy)

    if _state == S["LANE_RETURN_Y"]:
        # V2: 下一条道如果需要小口，下到开口高度 y=8.85 就直接横穿进去；
        # 否则下到底部横道 y=7.20 再换道。
        next_lane_idx = _lane_iter + 1
        if next_lane_idx < len(_lane_order):
            next_lane = _lane_order[next_lane_idx]
            next_needs_opening = _needs_opening(next_lane)
        else:
            next_needs_opening = False
        target_y = OPENING_Y if next_needs_opening else LANE_SWITCH_Y
        if y <= target_y:
            _lane_iter += 1
            if _lane_iter >= len(_lane_order):
                _set_state(S["BRIDGE_TO_X"], "lane_return_all_done", position)
                return _return_step(0, "lane_return_all_done", position, rpy)
            _begin_lane()
            return _return_step(0, "lane_return_next_lane", position, rpy)
        return _return_step(_forward_lane_step(position, rpy, HEADING_SOUTH, lane_x), "lane_return_y", position, rpy)

    return None


def _route_opening_lane(position, rpy, frame):
    """无杆道状态机：通过小口 (y=8.85) 进出，无需蹲姿。"""
    global _lane_iter, _jump_frames

    x, y, _ = position
    target = _current_target
    # 考虑足球踢球偏移：右道+足球时使用 2.00 而非 2.10
    lane_x = _get_effective_lane_x(_current_lane, target)
    opening_x = _get_opening_x(_current_lane)
    target_y = TARGET_Y[target]
    announce_y = ANNOUNCE_Y[target]
    backup_dist = BACKUP_DIST[target]
    backup_y = target_y + 0.05

    # ── 准备进入：先横移到小口 x 位置（避开墙），再转北，再向北到小口高度 ──
    # 关键：墙的存在意味着不能直接向北走，必须先确保 x 到达安全位置

    if _state == S["OPENING_PREPARE_X"]:
        # 步骤1：在底部横道横移到 opening_x（避开中道的墙）
        heading = HEADING_EAST if opening_x > x else HEADING_WEST
        return _go_x(position, rpy, opening_x, heading,
                    S["OPENING_PREPARE_TURN"], "opening_prepare_x_done",
                    center_y=LANE_SWITCH_Y)

    if _state == S["OPENING_PREPARE_TURN"]:
        # 步骤2：转向北
        return _turn_state(rpy, HEADING_NORTH, S["OPENING_ENTER_Y"],
                          "opening_prepare_turn_done", position)

    if _state == S["OPENING_ENTER_Y"]:
        # 步骤3：沿 opening_x 向北到小口高度 y=8.90
        if y >= OPENING_Y - XY_TOL:
            _set_state(S["OPENING_CROSS_IN"], "opening_enter_y_done", position)
            return _return_step(0, "opening_enter_y_done", position, rpy)
        # 向北前进，同时保持 x=opening_x
        step = _forward_lane_step(position, rpy, HEADING_NORTH, opening_x)
        return _return_step(step, "opening_enter_y", position, rpy)

    if _state == S["OPENING_CROSS_IN"]:
        # 横穿进入该道中心线
        heading = HEADING_EAST if lane_x > x else HEADING_WEST
        return _go_x(position, rpy, lane_x, heading,
                    S["OPENING_TURN_NORTH"], "opening_cross_in_done",
                    center_y=OPENING_Y)

    if _state == S["OPENING_TURN_NORTH"]:
        return _turn_state(rpy, HEADING_NORTH, S["OPENING_ALIGN_UP"],
                          "opening_turn_north_done", position)

    if _state == S["OPENING_ALIGN_UP"]:
        return _adjust_x(position, rpy, lane_x, HEADING_NORTH,
                        S["OPENING_ADVANCE"], "opening_align_up_done")

    # ── 向北到目标物 ──
    if _state == S["OPENING_ADVANCE"]:
        detected = _detect_target(frame, target)
        _log_event("ROUTE_TARGET_SCAN", target=target, detected=detected, pos=_fmt_pos(position))
        if detected or y >= announce_y:
            _announce_once(target, TARGET_NAME_CN[target])
        if y >= backup_y:
            _motion_start_reset(position)
            if target == "orange":
                _jump_frames = 0
                _set_state(S["OPENING_JUMP"], "opening_orange_jump_start", position)
                return _return_step(ORANGE_JUMP_GAIT, "opening_orange_jump_start", position, rpy)
            _set_state(S["OPENING_BACKUP"], "opening_target_reach", position)
            return _return_step(0, "opening_target_reach", position, rpy)
        step = _forward_lane_step(position, rpy, HEADING_NORTH, lane_x)
        return _return_step(step, "opening_advance", position, rpy)

    if _state == S["OPENING_BACKUP"]:
        return _backup_to_distance(position, S["OPENING_TURN_SOUTH"],
                                   "opening_backup_done", backup_dist)

    if _state == S["OPENING_JUMP"]:
        _jump_frames += 1
        if _jump_frames >= JUMP_FRAMES:
            _set_state(S["OPENING_TURN_SOUTH"], "opening_jump_done", position)
            return _return_step(0, "opening_jump_done", position, rpy)
        return _return_step(ORANGE_JUMP_GAIT, "opening_jump", position, rpy)

    # ── 返回小口出去 ──
    if _state == S["OPENING_TURN_SOUTH"]:
        return _turn_state(rpy, HEADING_SOUTH, S["OPENING_ALIGN_DOWN"],
                          "opening_turn_south_done", position)

    if _state == S["OPENING_ALIGN_DOWN"]:
        return _adjust_x(position, rpy, lane_x, HEADING_SOUTH,
                        S["OPENING_RETURN"], "opening_align_down_done")

    if _state == S["OPENING_RETURN"]:
        # 向南返回到小口高度
        if y <= OPENING_Y + XY_TOL:
            _set_state(S["OPENING_CROSS_OUT"], "opening_return_to_y", position)
            return _return_step(0, "opening_return_to_y", position, rpy)
        step = _forward_lane_step(position, rpy, HEADING_SOUTH, lane_x)
        return _return_step(step, "opening_return", position, rpy)

    if _state == S["OPENING_CROSS_OUT"]:
        # 横穿出小口
        heading = HEADING_EAST if opening_x > x else HEADING_WEST
        return _go_x(position, rpy, opening_x, heading,
                    S["OPENING_EXIT_SOUTH"], "opening_cross_out_done",
                    center_y=OPENING_Y)

    if _state == S["OPENING_EXIT_SOUTH"]:
        return _turn_state(rpy, HEADING_SOUTH, S["OPENING_TO_BOTTOM"],
                          "opening_exit_south_done", position)

    if _state == S["OPENING_TO_BOTTOM"]:
        # 继续向南到底部横道或进入下一道
        target_y = LANE_SWITCH_Y
        next_lane_idx = _lane_iter + 1

        # V2: 检查下一道是否也需要小口（且能直接在当前高度横穿）
        if next_lane_idx < len(_lane_order):
            next_lane = _lane_order[next_lane_idx]
            if _needs_opening(next_lane) and y <= OPENING_Y + 0.1:
                # 下一道也是无杆道，在小口高度直接横穿过去
                target_y = OPENING_Y

        if y <= target_y:
            _lane_iter += 1
            if _lane_iter >= len(_lane_order):
                _set_state(S["BRIDGE_TO_X"], "opening_all_done", position)
                return _return_step(0, "opening_all_done", position, rpy)
            _begin_lane()
            return _return_step(0, "opening_next_lane", position, rpy)
        step = _forward_lane_step(position, rpy, HEADING_SOUTH, opening_x)
        return _return_step(step, "opening_to_bottom", position, rpy)

    return None


def _route(position, rpy, frame):
    """第四段V2总调度。"""
    if _state == S["TO_START"]:
        return _route_entry(position, rpy)

    if _state in (S["BRIDGE_TO_X"], S["BRIDGE_TURN_UP"]):
        return _route_bridge(position, rpy)

    # 判断当前道是否需要小口
    if _needs_opening(_current_lane):
        result = _route_opening_lane(position, rpy, frame)
        if result is not None:
            return result

    # 有杆道，使用原有状态机
    return _route_general_lane(position, rpy, frame)


# ── 对外接口 ──────────────────────────────────────────────────────
def segment4_v2_control(position, gait_mode, rpy, frame=None):
    """每帧调用一次。返回步态编号，-1 表示第四段完成。"""
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
        return -1

    if mode == 7:
        return _return_step(0, "recover_from_down", position, rpy)

    # 跳跃撞球状态：跳过步态切换等待，直接推进跳跃流程
    if _state in (S["LANE_JUMP"], S["OPENING_JUMP"]):
        return _route(position, rpy, frame)

    switching_gait = (gait == 0 and mode == 0) or (gait == 1 and mode == 9)
    if switching_gait:
        if _state in LOW_STATES:
            return _return_step(LOW_BAR_GAIT, "gait_switch_keep_low", position, rpy)
        return _return_step(0, "gait_switch_wait", position, rpy)

    return _route(position, rpy, frame)


# ── 硬件运行入口 ──────────────────────────────────────────────────
if __name__ == "__main__":
    preset = parse_preset_args(sys.argv[1:])
    print(f"[第四段V2] 预设: mid_open={preset['mid_open']}  no_bar_lane={preset['no_bar_lane']}  lane_of={preset['lane_of']}")
    order = ["left", "right", "mid"] if preset["mid_open"] == "right" and preset["no_bar_lane"] == "mid" else ["left", "mid", "right"]
    print(f"[第四段V2] 访问顺序: {order}")

    sys.path.insert(0, '/usr/local/lib/python3.8/site-packages')
    sys.path.append("./lcm")

    import threading
    import lcm
    from Robot_Ctrl import Robot_Ctrl
    from Msg_receive import Pos_msg, Gait_msg
    from user_pub import user_pub
    from robot_control_cmd_lcmt import robot_control_cmd_lcmt

    def main():
        reset_segment4_v2(preset)
        _reset_log_file()
        lcm_cmd = lcm.LCM("udpm://239.255.76.67:7671?ttl=255")
        cmd_msg = robot_control_cmd_lcmt()
        data_lock = threading.Lock()

        user_pub()
        my_ctrl = Robot_Ctrl()
        pos_msg = Pos_msg(data_lock)
        gait_msg = Gait_msg(data_lock)

        ctrl_thread = threading.Thread(target=my_ctrl.run, daemon=True)
        rec_thread = threading.Thread(target=pos_msg.run, daemon=True)
        gait_thread = threading.Thread(target=gait_msg.run, daemon=True)

        ctrl_thread.start()
        time.sleep(4)
        rec_thread.start()
        gait_thread.start()

        print("=== 赛段四V2 开始 ===")
        try:
            while True:
                with data_lock:
                    pos = list(pos_msg.position)
                    gait = list(gait_msg.gait_mode)
                    yaw = pos_msg.rpy[2]

                step = segment4_v2_control(pos, gait, yaw)

                if step == -1:
                    print("=== 赛段四V2 完成 ===")
                    break

                my_ctrl.num = step
                my_ctrl.msg.life_count = (my_ctrl.msg.life_count + 1) % 127

                print(
                    f"pos={[round(v, 2) for v in pos]}  yaw={yaw:.1f}°  "
                    f"state={_state}  lane={_current_lane}  target={_current_target}  has_bar={_current_has_bar}  step={step}"
                )

                if step == 0:
                    time.sleep(0.5)

        except KeyboardInterrupt:
            pass
        finally:
            cmd_msg.mode = 7
            cmd_msg.gait_id = 0
            cmd_msg.duration = 0
            cmd_msg.life_count += 1
            lcm_cmd.publish("robot_control_cmd", cmd_msg.encode())
            sys.exit()

    main()
