"""
第三赛段：S型弯道

入口：(-0.3, 4.7)，朝向 90°（y+ 方向）
出口：(3.1,  6.5)，转回朝向 90°

几何（两段等半径圆弧，圆心角各 45°，R=2.5132m）：
  弧1：顺时针，圆心 (1.636, 3.098)
       (-0.3, 4.7) → (1.4, 5.6)，切线方向 50.4° → 5.4°
  弧2：逆时针，圆心 (1.164, 8.102)
       (1.4, 5.6)  → (3.1, 6.5)，切线方向 5.4° → 50.4°
  两弧在 (1.4, 5.6) 处切线连续（均为 5.4°）

流程：前进到入口 y=4.7 → 转向 50.4° → 跑弧1（含径向纠偏）
      → 跑弧2（含径向纠偏）→ 原地转回 90°
"""

import math

# ── 弧段几何参数 ────────────────────────────────────────────────
_ARC1_CX, _ARC1_CY = 1.636, 3.098   # 弧1圆心（顺时针）
_ARC2_CX, _ARC2_CY = 1.164, 8.102   # 弧2圆心（逆时针）
_ARC_R = 2.5132

# 弧1入口切线方向
_ARC1_ENTRY_HDG = 50.4

# 纠偏增益：偏离理想轨迹 1m 时的朝向修正角（度），建议 10~20
_CORR_GAIN = 15.0

# ── 入口等待坐标 ────────────────────────────────────────────────
_ENTRY_Y = 4.7

# ── 状态转换阈值 ────────────────────────────────────────────────
_ARC1_DONE_X = 1.3    # 弧1完成：x 超过此值切换弧2
_ARC2_DONE_X = 3.0    # 弧2完成：x 超过此值切换出口转向

# ── 朝向控制阈值 ────────────────────────────────────────────────
_FAST_DEG = 15
_SLOW_DEG = 5

# ── 状态 ────────────────────────────────────────────────────────
_ST_APPROACH = "S3_APPROACH"
_ST_ALIGN    = "S3_ALIGN"
_ST_ARC1     = "S3_ARC1"
_ST_ARC2     = "S3_ARC2"
_ST_TURN_90  = "S3_TURN_90"
_ST_DONE     = "S3_DONE"

_state = _ST_APPROACH


def reset_segment3():
    global _state
    _state = _ST_APPROACH


def _norm(a):
    while a >  180: a -= 360
    while a <= -180: a += 360
    return a


def _turn(rpy, target_hdg):
    d = _norm(rpy - (target_hdg % 360))
    if   d >  _FAST_DEG: return 15
    elif d >  _SLOW_DEG: return 3
    elif d < -_FAST_DEG: return 14
    elif d < -_SLOW_DEG: return 2
    return 1


def _arc_turn(rpy, cx, cy, x, y, clockwise):
    """
    弧线跟随步态，含径向纠偏。
    clockwise=True  → 顺时针，切线 = atan2(dy,dx) - 90，偏外时切线角减小
    clockwise=False → 逆时针，切线 = atan2(dy,dx) + 90，偏外时切线角增大
    """
    dx, dy = x - cx, y - cy
    r_actual = math.sqrt(dx*dx + dy*dy)
    dr = r_actual - _ARC_R

    tangent = math.degrees(math.atan2(dy, dx)) + (-90 if clockwise else 90)
    sign = -1 if clockwise else 1
    correction = sign * math.degrees(math.atan(dr * _CORR_GAIN / _ARC_R))

    return _turn(rpy, tangent + correction)


def segment3_control(position, gait_mode, rpy):
    """
    赛段3（S型弯道）控制逻辑，每帧调用一次。

    Returns:
        int: 步态索引；-1 表示赛段完成
    """
    global _state

    x, y, _ = position
    gait, mode = gait_mode

    if (gait == 0 and mode == 0) or (gait == 1 and mode == 9):
        return 0

    # ── APPROACH：朝向 90° 前进到入口 ────────────────────────────
    if _state == _ST_APPROACH:
        if y >= _ENTRY_Y:
            _state = _ST_ALIGN
            return 0
        return _turn(rpy, 90)

    # ── ALIGN：原地转向至弧1切线方向 50.4° ───────────────────────
    elif _state == _ST_ALIGN:
        step = _turn(rpy, _ARC1_ENTRY_HDG)
        if step == 1:
            _state = _ST_ARC1
        return step

    # ── 弧1：顺时针，含径向纠偏 ──────────────────────────────────
    elif _state == _ST_ARC1:
        if x >= _ARC1_DONE_X:
            _state = _ST_ARC2
            return 1
        return _arc_turn(rpy, _ARC1_CX, _ARC1_CY, x, y, clockwise=True)

    # ── 弧2：逆时针，含径向纠偏 ──────────────────────────────────
    elif _state == _ST_ARC2:
        if x >= _ARC2_DONE_X:
            _state = _ST_TURN_90
            return 0
        return _arc_turn(rpy, _ARC2_CX, _ARC2_CY, x, y, clockwise=False)

    # ── TURN_90：出口原地转回朝向 90° ────────────────────────────
    elif _state == _ST_TURN_90:
        step = _turn(rpy, 90)
        if step == 1:
            _state = _ST_DONE
            return -1
        return step

    return -1
