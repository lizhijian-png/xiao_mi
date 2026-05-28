"""
第五赛段：螺旋爬坡 + 不平整路面 + 跳下

赛道几何（绝对坐标）：
  分段1：y+ (90°)  中心线 x=3.15    起点(3.15, 7.0)  → 终点(3.15, 12.35)  长5.35m
      ├─ 台阶在 y≈7.6
  分段2：x- (180°) 中心线 y=12.35   起点(3.15, 12.35) → 终点(-0.35, 12.35) 长3.5m
  分段3：y+ (90°)  中心线 x=-0.35   起点(-0.35, 12.35) → 终点(-0.35, 15.35) 长3.0m
  分段4：x+ (0°)   中心线 y=15.35   起点(-0.35, 15.35) → 终点(3.15, 15.35)  长3.5m
  分段5：y- (270°) 中心线 x=3.15    起点(3.15, 15.35) → 平整1.5m+跳下0.5m  长2.0m

usergait.toml 步态索引：
  共享（赛段1-4不变）：
    0:站立 1:前进 2:左转 3:右转 4:趴下 9:高抬腿 10:斜坡
    14:快左转 15:快右转 28:快前进 29:左纠偏 30:右纠偏
  赛段5专用（台阶和第一段直坡不下压机身，保留后脚上台阶空间）：
    31:高抬腿(step_h=0.14,pitch=-0.08,z=-0.05) 32:斜坡(step_h=0.10,pitch=-0.05,z=0.0)
    33:左纠偏(step_h=0.12,pitch=-0.03,z=-0.03) 34:右纠偏(step_h=0.12,pitch=-0.03,z=-0.03)
    35:上台阶高抬脚(step_h=0.18,pitch=-0.08,z=0.0)
"""

import math

# ── 赛道几何参数（绝对坐标，米）───────────────────────────────────
# 中心线坐标
CENTER_X_SEG1 = 3.15    # 分段1、5 中心线 x
CENTER_Y_SEG2 = 12.35   # 分段2 中心线 y
CENTER_X_SEG3 = -0.35   # 分段3 中心线 x
CENTER_Y_SEG4 = 15.35   # 分段4 中心线 y

# 连接点（转弯触发位置）
TURN1_Y = 12.35   # SEG1→SEG2: 到达 y≥12.35 触发左转
TURN1_EARLY_Y = 12.10  # SEG1→SEG2: 在y=12.10先转45°，避免继续靠右边缘前进
TURN1_FORWARD_DIST = 0.20  # SEG1→SEG2: 第一次45°转完后再实际前进约0.2m，再进入第二次45°转向
TURN2_X = -0.35   # SEG2→SEG3: 到达 x≤-0.35 触发右转
TURN3_Y = 15.35   # SEG3→SEG4: 到达 y≥15.35 触发右转
TURN4_X = 3.15    # SEG4→SEG5: 到达 x≥3.15 触发右转
SEG5_END_Y = 13.35  # SEG5 跳下区结束 (15.35 - 2.0)

# 台阶位置
STEP_Y = 7.6       # 台阶在 y=7.6
STEP_APPROACH_Y = 7.5   # 台阶前切换到爬升步态
STEP_CLEAR_Y = 8.0      # 台阶后继续保持高抬脚到 y=8.0，给后脚留出完整上台阶距离

# ── 纠偏参数 ──────────────────────────────────────────────────────
LAT_TOLERANCE = 0.03   # 横向偏移容忍度 3cm（路宽50cm）

# ── 朝向角度 ──────────────────────────────────────────────────────
HDG_YP = 90    # y+
HDG_TURN1_MID = 135  # 第一交接处先从90°转到135°，走约0.2m后再转到180°
HDG_XN = 180   # x-
HDG_XP = 0     # x+
HDG_YN = 270   # y-

# ── 转向阈值 ──────────────────────────────────────────────────────
FAST_DEG = 20
SLOW_DEG = 8

# ── 状态机 ────────────────────────────────────────────────────────
_ST_SEG1_APPROACH = "SEG1_APPROACH"
_ST_SEG1_STEP     = "SEG1_STEP"
_ST_SEG1_UPHILL   = "SEG1_UPHILL"
_ST_PRE_TURN1     = "PRE_TURN1"
_ST_TURN1_FORWARD = "TURN1_FORWARD"
_ST_TURN1         = "TURN1"
_ST_SEG2          = "SEG2"
_ST_PRE_TURN2     = "PRE_TURN2"
_ST_TURN2         = "TURN2"
_ST_SEG3          = "SEG3"
_ST_PRE_TURN3     = "PRE_TURN3"
_ST_TURN3         = "TURN3"
_ST_SEG4          = "SEG4"
_ST_PRE_TURN4     = "PRE_TURN4"
_ST_TURN4         = "TURN4"
_ST_SEG5_FLAT     = "SEG5_FLAT"
_ST_SEG5_JUMP     = "SEG5_JUMP"
_ST_DONE          = "DONE"

_state = _ST_SEG1_APPROACH
_stand_count = 0  # 站立帧计数（复用：台阶后稳定、转弯前稳定）
_turn1_forward_start = None  # 第一交接处第一次45°转完后的起走位置，用于计算实际前进距离


def reset_segment5():
    global _state, _stand_count, _turn1_forward_start
    _state = _ST_SEG1_APPROACH
    _stand_count = 0
    _turn1_forward_start = None


def _norm(a):
    while a > 180: a -= 360
    while a <= -180: a += 360
    return a


def _turn_step(rpy, target_hdg):
    """纯转向：返回转向步态，对准后返回 0（站立）"""
    d = _norm(rpy - (target_hdg % 360))
    if d > FAST_DEG:      return 15
    elif d > SLOW_DEG:    return 3
    elif d < -FAST_DEG:   return 14
    elif d < -SLOW_DEG:   return 2
    return 0  # 对准 → 站立


def _forward_with_lateral(rpy, target_hdg, center_val, current_val, axis, gait_forward, gait_left, gait_right):
    """
    带横向纠偏的前进控制。

    Args:
        rpy:            当前朝向
        target_hdg:     目标朝向
        center_val:     中心线坐标值
        current_val:    当前坐标值
        axis:           'x' 或 'y'，表示横向纠偏轴
        gait_forward:   前进步态
        gait_left:      前进+左纠偏步态
        gait_right:     前进+右纠偏步态

    Returns:
        int: 步态索引
    """
    # 1. 朝向对准
    d = _norm(rpy - (target_hdg % 360))
    if d > FAST_DEG:      return 15
    elif d > SLOW_DEG:    return 3
    elif d < -FAST_DEG:   return 14
    elif d < -SLOW_DEG:   return 2

    # 2. 横向纠偏：计算当前坐标偏离中心线的方向和大小
    offset = current_val - center_val

    # 将世界坐标偏移转换为机身左右方向
    # 朝向 0°(x+)→右=y-  90°(y+)→右=x+  180°(x-)→右=y+  270°(y-)→右=x-
    if target_hdg == HDG_XP:    # 0°,  右=y-
        lateral = -offset
    elif target_hdg == HDG_YP:  # 90°, 右=x+
        lateral = offset
    elif target_hdg == HDG_XN:  # 180°, 右=y+
        lateral = offset
    elif target_hdg == HDG_YN:  # 270°, 右=x-
        lateral = -offset
    else:
        lateral = 0.0

    if lateral > LAT_TOLERANCE:
        return gait_left    # 偏右 → 左纠偏
    elif lateral < -LAT_TOLERANCE:
        return gait_right   # 偏左 → 右纠偏

    return gait_forward     # 居中直行


def _stand_then(next_state, required_frames):
    """站立若干帧后切换到下一状态"""
    global _state, _stand_count
    _stand_count += 1
    if _stand_count >= required_frames:
        _state = next_state
        _stand_count = 0
        return 0
    return 0  # 站立


def segment5_control(position, gait_mode, rpy):
    """
    第五赛段控制逻辑，每帧（0.2s）调用一次。

    Returns:
        int: 步态索引；-1 表示赛段5完成
    """
    global _state, _stand_count, _turn1_forward_start

    x, y, _ = position
    gait, mode = gait_mode

    # 步态切换中等待 / 趴下后站起
    if (gait == 0 and mode == 0) or (gait == 1 and mode == 9):
        return 0
    if mode == 7:
        return 0

    # ═══════════════════════════════════════════════════════════════
    # 分段1：y+ (90°) 中心线 x=3.15  (3.15,7.0)→(3.15,12.35)
    # ═══════════════════════════════════════════════════════════════

    # ── 接近台阶：y < 7.5 ────────────────────────────────────────
    if _state == _ST_SEG1_APPROACH:
        if y >= STEP_APPROACH_Y:
            _state = _ST_SEG1_STEP
            return 0
        return _forward_with_lateral(
            rpy, HDG_YP, CENTER_X_SEG1, x, 'x',
            gait_forward=32, gait_left=33, gait_right=34)

    # ── 翻越台阶：7.5 ≤ y < 8.0，延长高抬脚区间，确保后脚也越过5cm台阶 ───
    elif _state == _ST_SEG1_STEP:
        if y >= STEP_CLEAR_Y:
            # 台阶翻越完成，站立稳定后进入上坡
            step = _stand_then(_ST_SEG1_UPHILL, 3)
            return step
        return _forward_with_lateral(
            rpy, HDG_YP, CENTER_X_SEG1, x, 'x',
            gait_forward=35, gait_left=33, gait_right=34)

    # ── 继续上坡：8.0 ≤ y < 12.10 ────────────────────────────────
    elif _state == _ST_SEG1_UPHILL:
        if y >= TURN1_EARLY_Y:
            _state = _ST_PRE_TURN1
            _stand_count = 0
            return 0
        return _forward_with_lateral(
            rpy, HDG_YP, CENTER_X_SEG1, x, 'x',
            gait_forward=32, gait_left=33, gait_right=34)

    # ── 第一次提前转45°：先转到135°，减少第一交接处踩到右边边缘的风险 ─────
    elif _state == _ST_PRE_TURN1:
        step = _turn_step(rpy, HDG_TURN1_MID)
        if step == 0:
            # 第一次45°转向完成后，不马上转到180°，先向前走约0.2m避开右侧边缘。
            _state = _ST_TURN1_FORWARD
            _turn1_forward_start = (x, y)
            return 0
        return step

    # ── 斜向前进约0.2m：按实际位移计算距离，再进入第二次45°转向 ──────────
    elif _state == _ST_TURN1_FORWARD:
        if _turn1_forward_start is None:
            # 正常流程会在第一次45°转完时记录起点；这里补记一次，避免状态恢复后距离无法计算。
            _turn1_forward_start = (x, y)
        dx = x - _turn1_forward_start[0]
        dy = y - _turn1_forward_start[1]
        if math.sqrt(dx * dx + dy * dy) >= TURN1_FORWARD_DIST:
            _state = _ST_TURN1
            _stand_count = 0
            _turn1_forward_start = None
            return 0
        return _forward_with_lateral(
            rpy, HDG_TURN1_MID, CENTER_X_SEG1, x, 'x',
            gait_forward=32, gait_left=33, gait_right=34)

    # ── 第二次转45°：由135°转到180°，对准第二段中心线方向 ───────────────
    elif _state == _ST_TURN1:
        step = _turn_step(rpy, HDG_XN)
        if step == 0:
            _state = _ST_SEG2
            return 0
        return step

    # ═══════════════════════════════════════════════════════════════
    # 分段2：x- (180°) 中心线 y=12.35  (3.15,12.35)→(-0.35,12.35)
    # ═══════════════════════════════════════════════════════════════
    elif _state == _ST_SEG2:
        if x <= TURN2_X:
            _state = _ST_PRE_TURN2
            _stand_count = 0
            return 0
        return _forward_with_lateral(
            rpy, HDG_XN, CENTER_Y_SEG2, y, 'y',
            gait_forward=31, gait_left=33, gait_right=34)

    # ── 转弯前稳定 ──────────────────────────────────────────────
    elif _state == _ST_PRE_TURN2:
        return _stand_then(_ST_TURN2, 5)

    # ── 右转 180°→90° ───────────────────────────────────────────
    elif _state == _ST_TURN2:
        step = _turn_step(rpy, HDG_YP)
        if step == 0:
            _state = _ST_SEG3
            return 0
        return step

    # ═══════════════════════════════════════════════════════════════
    # 分段3：y+ (90°) 中心线 x=-0.35  (-0.35,12.35)→(-0.35,15.35)
    # ═══════════════════════════════════════════════════════════════
    elif _state == _ST_SEG3:
        if y >= TURN3_Y:
            _state = _ST_PRE_TURN3
            _stand_count = 0
            return 0
        return _forward_with_lateral(
            rpy, HDG_YP, CENTER_X_SEG3, x, 'x',
            gait_forward=31, gait_left=33, gait_right=34)

    # ── 转弯前稳定 ──────────────────────────────────────────────
    elif _state == _ST_PRE_TURN3:
        return _stand_then(_ST_TURN3, 5)

    # ── 右转 90°→0° ────────────────────────────────────────────
    elif _state == _ST_TURN3:
        step = _turn_step(rpy, HDG_XP)
        if step == 0:
            _state = _ST_SEG4
            return 0
        return step

    # ═══════════════════════════════════════════════════════════════
    # 分段4：x+ (0°) 中心线 y=15.35  (-0.35,15.35)→(3.15,15.35)
    # ═══════════════════════════════════════════════════════════════
    elif _state == _ST_SEG4:
        if x >= TURN4_X:
            _state = _ST_PRE_TURN4
            _stand_count = 0
            return 0
        return _forward_with_lateral(
            rpy, HDG_XP, CENTER_Y_SEG4, y, 'y',
            gait_forward=31, gait_left=33, gait_right=34)

    # ── 转弯前稳定 ──────────────────────────────────────────────
    elif _state == _ST_PRE_TURN4:
        return _stand_then(_ST_TURN4, 5)

    # ── 右转 0°→270° ───────────────────────────────────────────
    elif _state == _ST_TURN4:
        step = _turn_step(rpy, HDG_YN)
        if step == 0:
            _state = _ST_SEG5_FLAT
            return 0
        return step

    # ═══════════════════════════════════════════════════════════════
    # 分段5：y- (270°) 中心线 x=3.15  (3.15,15.35)→(3.15,13.35)
    # ═══════════════════════════════════════════════════════════════
    elif _state == _ST_SEG5_FLAT:
        if y <= SEG5_END_Y + 0.5:  # 进入跳下区（最后0.5m）
            _state = _ST_SEG5_JUMP
            return 0
        return _forward_with_lateral(
            rpy, HDG_YN, CENTER_X_SEG1, x, 'x',
            gait_forward=28, gait_left=33, gait_right=34)

    # ── 跳下区 ──────────────────────────────────────────────────
    elif _state == _ST_SEG5_JUMP:
        if y <= SEG5_END_Y:
            _state = _ST_DONE
            return -1
        return _forward_with_lateral(
            rpy, HDG_YN, CENTER_X_SEG1, x, 'x',
            gait_forward=1, gait_left=33, gait_right=34)

    return -1


# ────────────────────────────────────────────────────────────────────
# 独立测试入口
# ────────────────────────────────────────────────────────────────────
if __name__ == "__main__":
    import sys
    sys.path.insert(0, '/usr/local/lib/python3.8/site-packages')
    sys.path.append("./lcm")

    import time
    import threading
    import lcm
    from Robot_Ctrl import Robot_Ctrl
    from Msg_receive import Pos_msg, Gait_msg
    from user_pub import user_pub
    from robot_control_cmd_lcmt import robot_control_cmd_lcmt

    def main():
        reset_segment5()

        lcm_cmd = lcm.LCM("udpm://239.255.76.67:7671?ttl=255")
        cmd_msg = robot_control_cmd_lcmt()
        data_lock = threading.Lock()

        user_pub()
        my_ctrl = Robot_Ctrl()
        pos_msg = Pos_msg(data_lock)
        gait_msg = Gait_msg(data_lock)

        ctrl_thread = threading.Thread(target=my_ctrl.run, daemon=True)
        rec_thread  = threading.Thread(target=pos_msg.run, daemon=True)
        gait_thread = threading.Thread(target=gait_msg.run, daemon=True)

        ctrl_thread.start()
        time.sleep(4)

        my_ctrl.num = 2
        my_ctrl.msg.life_count = (my_ctrl.msg.life_count + 1) % 127
        time.sleep(0.5)

        rec_thread.start()
        gait_thread.start()

        print("=== 第五赛段：螺旋爬坡+不平整路面+跳下 开始 ===")
        try:
            while True:
                with data_lock:
                    pos  = list(pos_msg.position)
                    gait = list(gait_msg.gait_mode)
                    yaw  = pos_msg.rpy[2]

                step = segment5_control(pos, gait, yaw)

                if step == -1:
                    print("=== 第五赛段完成 ===")
                    break

                my_ctrl.num = step
                my_ctrl.msg.life_count = (my_ctrl.msg.life_count + 1) % 127

                print(
                    f"pos={[round(v, 2) for v in pos]}  "
                    f"yaw={yaw:.1f}°  step={step}  state={_state}"
                )

                if step == 0:
                    time.sleep(4)

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
