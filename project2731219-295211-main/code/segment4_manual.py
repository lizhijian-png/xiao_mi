"""第四赛段（人工输入版）：随机摆放 + 随机限高杆 + 中道随机开口侧。

用法：
  python3 segment4_manual.py <mid_open> <cola_pos> <football_pos> <orange_pos>

命令行 4 个整数：
  mid_open      中道开口侧      0=左侧，1=右侧
  cola_pos      可乐所在道      1=左，2=中，3=右
  football_pos  足球所在道      1=左，2=中，3=右
  orange_pos    橙色小球所在道  1=左，2=中，3=右

例：`python3 segment4_manual.py 0 1 2 3`
  = 中道从左侧进出，可乐在左道，足球在中道，橙球在右道。

规则要点（详见 segment4_规则与设计.md）：
  - 三个目标物随机分占左/中/右三条竖道。
  - 限高杆 y 在 9.1~10.1 内随机，仅左、右两道有限高杆，中道没有。
  - 中道四面被墙包住，只有开放侧在 y=8.6~9.1 有一个开口，进出都必须穿此口。
  - 竖道访问顺序：mid_open=left 走 L→M→R→桥；mid_open=right 走 L→R→M→桥。

坐标系沿用 segment4.py：入口 (3.10, 7.10)，底部横向通道 y=7.20。
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
except ImportError:  # 逻辑测试环境可能没有 opencv，视觉检测直接返回 False
    cv2 = None
    np = None


# ── 场地几何 ──────────────────────────────────────────────────────
LEFT_LANE_X = -0.10
MID_LANE_X = 1.00
RIGHT_LANE_X = 2.10

LANE_SWITCH_Y = 7.20          # 底部横向通道，切换竖道
START_POINT = (3.10, 7.10)    # 第四段入口

# 中道开口：y ∈ [8.6, 9.1]，进出点取 8.85；开口两侧横穿出口 x。
MID_OPEN_Y = 8.85
MID_OPEN_LEFT_EXIT_X = 0.35   # 开口左侧横穿出中道后的 x
MID_OPEN_RIGHT_EXIT_X = 1.65  # 开口右侧横穿出中道后的 x

# 限高杆：随机范围 9.1~10.1，仅左右两道有。低姿态区间覆盖并留余量。
BAR_Y_MIN = 9.10
BAR_Y_MAX = 10.10
LOW_START_Y = 9.00    # 从该 y 开始进入低姿态准备
LOW_END_Y = 10.30     # 走低姿态到该 y 站起（覆盖最大杆位 + 0.2 余量）

# 目标物坐标（固定 y，只有所在道 x 变化）。
TARGET_Y = {
    "cola": 11.10,     # 可乐撞击位置（终点 y = 11.15）
    "football": 11.00,  # 足球（终点 y = 11.05，对齐原 backup_y）
    "orange": 11.00,    # 橙球（终点 y = 11.05）
}
ANNOUNCE_Y = {
    "cola": 10.90,
    "football": 10.90,
    "orange": 10.80,
}
BACKUP_DIST = {
    "cola": 0.15,
    "football": 0.13,   # 足球过门后退
    "orange": 0.20,
}
TARGET_NAME_CN = {
    "cola": "可乐瓶",
    "football": "足球",
    "orange": "橙色小球",
}

# 步态编号（与 segment4.py 一致）。
S4_FORWARD_LEFT_GAIT = 42    # 前进同时左修正
S4_FORWARD_RIGHT_GAIT = 43   # 前进同时右修正
S4_FAST_FORWARD_GAIT = 44    # 快走
S4_BACKUP_GAIT = 45          # 快退
S4_STRAFE_LEFT_GAIT = 46     # 左平移校正
S4_STRAFE_RIGHT_GAIT = 47    # 右平移校正
LOW_BAR_GAIT = 5             # 低姿态/蹲下
FOOTBALL_BACKUP_AFTER_KICK_DIST = 0.13
ORANGE_JUMP_GAIT = 57        # usergait.toml 索引：原地跳高撞悬挂小球（mode=22 FORCE_JUMP）
JUMP_FRAMES = 8              # 发跳跃步态的帧数（每帧约 0.2s，覆盖跳跃+落地，需现场标定）

HEADING_EAST = 0
HEADING_NORTHEAST = 45
HEADING_NORTH = 90
HEADING_WEST = 180
HEADING_SOUTH = 270

FAST_DEG = 18
SLOW_DEG = 6
XY_TOL = 0.08
LAT_TOLERANCE = 0.02   # 横向纠偏容差：偏移超 2cm 就发纠偏步态（参考 segment5 提前纠偏）
INERTIA_MARGIN = 0.02
LOW_STAND_SETTLE_FRAMES = 3

LOG_PATH = os.path.abspath(
    os.path.join(os.path.dirname(__file__), "..", "log", "segment4_manual_log.txt")
)


# ── 状态名 ────────────────────────────────────────────────────────
S = {
    "TO_START": "S4M_TO_START",
    # 通用竖道（左/右两道，必有杆）
    "LANE_ENTER_X": "S4M_LANE_ENTER_X",
    "LANE_TURN_NORTH": "S4M_LANE_TURN_NORTH",
    "LANE_ALIGN_UP": "S4M_LANE_ALIGN_UP",
    "LANE_UP_BEFORE_BAR": "S4M_LANE_UP_BEFORE_BAR",
    "LANE_UP_LOW": "S4M_LANE_UP_LOW",
    "LANE_STAND_AFTER_BAR": "S4M_LANE_STAND_AFTER_BAR",
    "LANE_ADVANCE_TO_TARGET": "S4M_LANE_ADVANCE_TO_TARGET",
    "LANE_BACKUP_TARGET": "S4M_LANE_BACKUP_TARGET",
    "LANE_JUMP": "S4M_LANE_JUMP",
    # 足球专用（有杆的左右道）：撞完保持低姿态短退 0.13m 再转身、原路低姿态退回过杆
    "LANE_FOOTBALL_LOW_BACKUP": "S4M_LANE_FOOTBALL_LOW_BACKUP",
    "LANE_FOOTBALL_TURN_SOUTH": "S4M_LANE_FOOTBALL_TURN_SOUTH",
    "LANE_FOOTBALL_ALIGN_DOWN": "S4M_LANE_FOOTBALL_ALIGN_DOWN",
    "LANE_FOOTBALL_DOWN_LOW": "S4M_LANE_FOOTBALL_DOWN_LOW",
    # 通用竖道：下行 + 换道
    "LANE_TURN_SOUTH": "S4M_LANE_TURN_SOUTH",
    "LANE_ALIGN_DOWN": "S4M_LANE_ALIGN_DOWN",
    "LANE_DOWN_BEFORE_BAR": "S4M_LANE_DOWN_BEFORE_BAR",
    "LANE_DOWN_LOW": "S4M_LANE_DOWN_LOW",
    "LANE_STAND_AFTER_DOWN": "S4M_LANE_STAND_AFTER_DOWN",
    "LANE_RETURN_Y": "S4M_LANE_RETURN_Y",
    # 中道（无杆，四面墙，只有开放侧开口 y=8.85 可进出）
    "MID_ENTER_OPENING": "S4M_MID_ENTER_OPENING",
    "MID_ENTER_TURN_NORTH": "S4M_MID_ENTER_TURN_NORTH",
    "MID_ALIGN_UP": "S4M_MID_ALIGN_UP",
    "MID_ADVANCE": "S4M_MID_ADVANCE",
    "MID_BACKUP": "S4M_MID_BACKUP",
    "MID_JUMP": "S4M_MID_JUMP",
    "MID_TURN_SOUTH": "S4M_MID_TURN_SOUTH",
    "MID_ALIGN_DOWN": "S4M_MID_ALIGN_DOWN",
    "MID_RETURN_OPENING": "S4M_MID_RETURN_OPENING",
    "MID_EXIT_OPENING": "S4M_MID_EXIT_OPENING",
    "MID_EXIT_TURN_SOUTH": "S4M_MID_EXIT_TURN_SOUTH",
    "MID_RETURN_BOTTOM": "S4M_MID_RETURN_BOTTOM",
    # 桥
    "BRIDGE_TO_X": "S4M_BRIDGE_TO_X",
    "BRIDGE_TURN_UP": "S4M_BRIDGE_TURN_UP",
    "DONE": "S4M_DONE",
}

LOW_STATES = {
    S["LANE_UP_LOW"],
    S["LANE_DOWN_LOW"],
    S["LANE_FOOTBALL_ALIGN_DOWN"],
    S["LANE_FOOTBALL_DOWN_LOW"],
}

# 桥入口与第五段朝向
BRIDGE_APPROACH_POINT = (3.15, LANE_SWITCH_Y)
SEG5_ENTRY_HEADING = 90


# ── 全局状态 ──────────────────────────────────────────────────────
_state = S["TO_START"]
_lane_order = []          # 访问顺序，例如 ["left", "mid", "right"]
_lane_iter = 0
_current_lane = None      # 当前道名 "left"/"mid"/"right"
_current_target = None    # 当前道上的目标 "cola"/"football"/"orange"
_current_has_bar = False  # 当前道是否有杆（左/右道 True，中道 False）
_mid_open = "left"        # "left" 或 "right"
_preset = None

_motion_start = None
_stand_count = 0
_jump_frames = 0
_announced = set()
_last_log_time = 0.0
_last_log_signature = None


# ── 参数解析 ──────────────────────────────────────────────────────
def parse_preset_args(argv):
    """解析 4 个整数参数，返回 preset dict。

    返回：
      {
        "mid_open": "left" | "right",
        "lane_of": {"cola": "left"/"mid"/"right",
                    "football": ...,
                    "orange": ...},
      }

    异常：参数数量不对 / 非整数 / mid_open 越界 / 三目标位置非法或重复。
    """
    if len(argv) != 4:
        raise ValueError(
            f"需要 4 个整数(mid_open cola football orange)，实际收到 {len(argv)} 个: {argv}"
        )
    try:
        nums = [int(a) for a in argv]
    except ValueError as e:
        raise ValueError(f"参数必须是整数，收到: {argv}") from e

    mid_open, cola, football, orange = nums
    if mid_open not in (0, 1):
        raise ValueError(f"mid_open 必须是 0(左) 或 1(右)，收到: {mid_open}")
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
        "lane_of": {
            "cola": lane_name[cola],
            "football": lane_name[football],
            "orange": lane_name[orange],
        },
    }


def reset_segment4_manual(preset):
    """重置状态机。preset 由 parse_preset_args 生成。"""
    global _state, _lane_order, _lane_iter, _current_lane, _current_target
    global _current_has_bar, _mid_open, _preset, _motion_start, _stand_count
    global _jump_frames, _announced, _last_log_time, _last_log_signature

    _preset = preset
    _mid_open = preset["mid_open"]
    lane_of = preset["lane_of"]
    _lane_order = ["left", "right", "mid"] if _mid_open == "right" else ["left", "mid", "right"]
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


# ── 路由辅助 ──────────────────────────────────────────────────────
def _lane_x(lane):
    return {"left": LEFT_LANE_X, "mid": MID_LANE_X, "right": RIGHT_LANE_X}[lane]


def _has_bar(lane):
    return lane in ("left", "right")


def _begin_lane():
    """切换到下一条竖道，初始化该道的全局配置。"""
    global _lane_iter, _current_lane, _current_target, _current_has_bar, _state
    global _motion_start, _stand_count
    lane = _lane_order[_lane_iter]
    _current_lane = lane
    for target, ln in _preset["lane_of"].items():
        if ln == lane:
            _current_target = target
            break
    _current_has_bar = _has_bar(lane)
    _motion_start = None
    _stand_count = 0
    if lane == "mid":
        _set_state(S["MID_ENTER_OPENING"], "begin_mid_lane")
    else:
        _set_state(S["LANE_ENTER_X"], "begin_lane")


def _route_entry(position, rpy):
    """入口：先修正 y 到底部横道 y=7.10（同时纠 x 到入口 3.10），然后进入第一条竖道。"""
    _, y, _ = position
    if abs(y - START_POINT[1]) > XY_TOL:
        heading = HEADING_NORTH if START_POINT[1] > y else HEADING_SOUTH
        step = _forward_with_lateral(position, rpy, heading, START_POINT[0], 'x')
        return _return_step(step, "route_to_start_fix_y", position, rpy)
    _begin_lane()
    return _return_step(0, "route_start_reached", position, rpy)


def _route_general_lane(position, rpy, frame):
    """左/右竖道状态机（必有杆）。"""
    global _stand_count, _motion_start, _lane_iter, _jump_frames

    x, y, _ = position
    lane_x = _lane_x(_current_lane)
    target = _current_target
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
        # 下一条是中道时，下到开口高度 y=8.85 就直接横穿进中道；
        # 否则下到底部横道 y=7.20 再换道。
        next_is_mid = (_lane_iter + 1 < len(_lane_order)) and (_lane_order[_lane_iter + 1] == "mid")
        target_y = MID_OPEN_Y if next_is_mid else LANE_SWITCH_Y
        if y <= target_y:
            _lane_iter += 1
            if _lane_iter >= len(_lane_order):
                _set_state(S["BRIDGE_TO_X"], "lane_return_all_done", position)
                return _return_step(0, "lane_return_all_done", position, rpy)
            _begin_lane()
            return _return_step(0, "lane_return_next_lane", position, rpy)
        return _return_step(_forward_lane_step(position, rpy, HEADING_SOUTH, lane_x), "lane_return_y", position, rpy)

    return None


def _route_mid(position, rpy, frame):
    """中道状态机（无杆，四面墙，只有开放侧开口 y=8.85 可进出）。"""
    global _lane_iter, _jump_frames
    _, y, _ = position
    target = _current_target
    target_y = TARGET_Y[target]
    announce_y = ANNOUNCE_Y[target]
    backup_dist = BACKUP_DIST[target]
    backup_y = target_y + 0.05

    enter_from_left = (_mid_open == "left")
    enter_heading = HEADING_EAST if enter_from_left else HEADING_WEST
    exit_heading = HEADING_WEST if enter_from_left else HEADING_EAST
    open_x = MID_OPEN_LEFT_EXIT_X if enter_from_left else MID_OPEN_RIGHT_EXIT_X

    # ── 进入：上一道下行到 y=8.85 时已直接切到本状态，横穿开口进中道 ──
    if _state == S["MID_ENTER_OPENING"]:
        return _go_x(position, rpy, MID_LANE_X, enter_heading, S["MID_ENTER_TURN_NORTH"], "mid_enter_opening_cross", center_y=MID_OPEN_Y)
    if _state == S["MID_ENTER_TURN_NORTH"]:
        return _turn_state(rpy, HEADING_NORTH, S["MID_ALIGN_UP"], "mid_enter_turn_north_done", position)

    # ── 上行撞目标 ──
    if _state == S["MID_ALIGN_UP"]:
        return _adjust_x(position, rpy, MID_LANE_X, HEADING_NORTH, S["MID_ADVANCE"], "mid_align_up_done")
    if _state == S["MID_ADVANCE"]:
        detected = _detect_target(frame, target)
        _log_event("ROUTE_TARGET_SCAN", target=target, detected=detected, pos=_fmt_pos(position))
        if detected or y >= announce_y:
            _announce_once(target, TARGET_NAME_CN[target])
        if y >= backup_y:
            _motion_start_reset(position)
            if target == "orange":
                _jump_frames = 0
                _set_state(S["MID_JUMP"], "mid_orange_jump_start", position)
                return _return_step(ORANGE_JUMP_GAIT, "mid_orange_jump_start", position, rpy)
            _set_state(S["MID_BACKUP"], "mid_target_reach_backup", position)
            return _return_step(0, "mid_target_reach_backup", position, rpy)
        return _return_step(_forward_lane_step(position, rpy, HEADING_NORTH, MID_LANE_X), "mid_advance", position, rpy)
    if _state == S["MID_BACKUP"]:
        return _backup_to_distance(position, S["MID_TURN_SOUTH"], "mid_backup_done", backup_dist)
    if _state == S["MID_JUMP"]:
        _jump_frames += 1
        if _jump_frames >= JUMP_FRAMES:
            _set_state(S["MID_TURN_SOUTH"], "mid_orange_jump_done", position)
            return _return_step(0, "mid_orange_jump_done", position, rpy)
        return _return_step(ORANGE_JUMP_GAIT, "mid_orange_jump", position, rpy)

    # ── 下行出中道 ──
    if _state == S["MID_TURN_SOUTH"]:
        return _turn_state(rpy, HEADING_SOUTH, S["MID_ALIGN_DOWN"], "mid_turn_south_done", position)
    if _state == S["MID_ALIGN_DOWN"]:
        return _adjust_x(position, rpy, MID_LANE_X, HEADING_SOUTH, S["MID_RETURN_OPENING"], "mid_align_down_done")
    if _state == S["MID_RETURN_OPENING"]:
        if y <= MID_OPEN_Y:
            _set_state(S["MID_EXIT_OPENING"], "mid_return_to_opening_y", position)
            return _return_step(0, "mid_return_to_opening_y", position, rpy)
        return _return_step(_forward_lane_step(position, rpy, HEADING_SOUTH, MID_LANE_X), "mid_return_opening", position, rpy)
    if _state == S["MID_EXIT_OPENING"]:
        return _go_x(position, rpy, open_x, exit_heading, S["MID_EXIT_TURN_SOUTH"], "mid_exit_opening_cross", center_y=MID_OPEN_Y)
    if _state == S["MID_EXIT_TURN_SOUTH"]:
        return _turn_state(rpy, HEADING_SOUTH, S["MID_RETURN_BOTTOM"], "mid_exit_turn_south_done", position)

    # ── 出开口后回到底部横道 ──
    if _state == S["MID_RETURN_BOTTOM"]:
        if y <= LANE_SWITCH_Y:
            _lane_iter += 1
            if _lane_iter >= len(_lane_order):
                _set_state(S["BRIDGE_TO_X"], "mid_return_all_done", position)
                return _return_step(0, "mid_return_all_done", position, rpy)
            _begin_lane()
            return _return_step(0, "mid_return_next_lane", position, rpy)
        return _return_step(_forward_lane_step(position, rpy, HEADING_SOUTH, open_x), "mid_return_bottom", position, rpy)

    return None


def _route_bridge(position, rpy):
    """收尾：走底部横道到桥入口，转正交接给第五段。"""
    if _state == S["BRIDGE_TO_X"]:
        return _go_x(position, rpy, BRIDGE_APPROACH_POINT[0], HEADING_EAST, S["BRIDGE_TURN_UP"], "bridge_x_reached", center_y=LANE_SWITCH_Y)
    if _state == S["BRIDGE_TURN_UP"]:
        step = _turn_to(rpy, SEG5_ENTRY_HEADING)
        if step == 1:
            _set_state(S["DONE"], "bridge_turn_up_done", position)
            return _return_step(-1, "segment4_manual_done", position, rpy)
        return _return_step(step, "bridge_turn_up_align", position, rpy)
    return None


def _route(position, rpy, frame):
    """第四段总调度。"""
    if _state == S["TO_START"]:
        return _route_entry(position, rpy)

    if _state in (S["BRIDGE_TO_X"], S["BRIDGE_TURN_UP"]):
        return _route_bridge(position, rpy)

    if _current_lane == "mid":
        return _route_mid(position, rpy, frame)

    return _route_general_lane(position, rpy, frame)


# ── 对外接口 ──────────────────────────────────────────────────────
def segment4_manual_control(position, gait_mode, rpy, frame=None):
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

    # 跳跃撞球状态：跳过步态切换等待，直接推进跳跃流程。
    if _state in (S["LANE_JUMP"], S["MID_JUMP"]):
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
    print(f"[第四段人工输入版] 预设: mid_open={preset['mid_open']}  lane_of={preset['lane_of']}")
    order = ["left", "right", "mid"] if preset["mid_open"] == "right" else ["left", "mid", "right"]
    print(f"[第四段人工输入版] 访问顺序: {order}")

    sys.path.insert(0, '/usr/local/lib/python3.8/site-packages')
    sys.path.append("./lcm")

    import threading
    import lcm
    from Robot_Ctrl import Robot_Ctrl
    from Msg_receive import Pos_msg, Gait_msg
    from user_pub import user_pub
    from robot_control_cmd_lcmt import robot_control_cmd_lcmt

    def main():
        reset_segment4_manual(preset)
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

        print("=== 赛段四（人工输入版）开始 ===")
        try:
            while True:
                with data_lock:
                    pos = list(pos_msg.position)
                    gait = list(gait_msg.gait_mode)
                    yaw = pos_msg.rpy[2]

                step = segment4_manual_control(pos, gait, yaw)

                if step == -1:
                    print("=== 赛段四（人工输入版）完成 ===")
                    break

                my_ctrl.num = step
                my_ctrl.msg.life_count = (my_ctrl.msg.life_count + 1) % 127

                print(
                    f"pos={[round(v, 2) for v in pos]}  yaw={yaw:.1f}°  "
                    f"state={_state}  lane={_current_lane}  target={_current_target}  step={step}"
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
