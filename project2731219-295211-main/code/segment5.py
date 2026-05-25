"""
第五赛段：螺旋爬坡 + 不平整路面 + 跳下

赛道描述：
  分段1：y+ (90°) 上坡 4.5m（起点高5cm→终点高20cm）
  转弯1：左转 90°→180°
  分段2：x- (180°) 不平整路面 4m（左高20cm/右高10cm，宽50cm）
  转弯2：右转 180°→90°
  分段3：y+ (90°) 不平整路面 4m
  转弯3：右转 90°→0°
  分段4：x+ (0°) 不平整路面 4m
  转弯4：右转 0°→270°
  分段5：y- (270°) 平整路面 2m（1.5m + 0.5m跳下区）

坐标系（与 test.py 一致）：
  rpy[2]=0°→x+, 90°→y+, 180°→x-, 270°→y-

usergait.toml 步态索引：
  0:站立 1:前进 2:左转 3:右转 4:趴下
  9:高抬腿(step_h=0.12) 10:斜坡(vel=0.25) 14:快左转 15:快右转 28:快前进
"""

import math

# ── 子分段长度（米）────────────────────────────────────────────────
SEG1_LENGTH = 4.5    # 上坡 450cm
SEG2_LENGTH = 4.0    # 不平整 x- 400cm（外侧）
SEG3_LENGTH = 4.0    # 不平整 y+ 400cm（外侧）
SEG4_LENGTH = 4.0    # 不平整 x+ 400cm（外侧）
SEG5_FLAT   = 1.5    # 平整（虚线前）150cm
SEG5_JUMP   = 0.5    # 跳下区 50cm

# ── 朝向角度 ──────────────────────────────────────────────────────
HDG_YP = 90    # y+ 方向
HDG_XN = 180   # x- 方向
HDG_XP = 0     # x+ 方向
HDG_YN = 270   # y- 方向

# ── 转向阈值 ──────────────────────────────────────────────────────
FAST_DEG = 20
SLOW_DEG = 8

# ── 状态机 ────────────────────────────────────────────────────────
_ST_SEG1      = "SEG1"
_ST_TURN1     = "TURN1"
_ST_SEG2      = "SEG2"
_ST_TURN2     = "TURN2"
_ST_SEG3      = "SEG3"
_ST_TURN3     = "TURN3"
_ST_SEG4      = "SEG4"
_ST_TURN4     = "TURN4"
_ST_SEG5_FLAT = "SEG5_FLAT"
_ST_SEG5_JUMP = "SEG5_JUMP"
_ST_DONE      = "DONE"

_state = _ST_SEG1
_entry_pos = None  # [x, y]，每个子分段的入口坐标


def reset_segment5():
    global _state, _entry_pos
    _state = _ST_SEG1
    _entry_pos = None


def _norm(a):
    while a > 180: a -= 360
    while a <= -180: a += 360
    return a


def _forward_at_heading(rpy, target_hdg, gait_idx):
    """转向对准目标朝向，对准后返回指定前进步态"""
    d = _norm(rpy - (target_hdg % 360))
    if d > FAST_DEG:      return 15  # 快右转
    elif d > SLOW_DEG:    return 3   # 慢右转
    elif d < -FAST_DEG:   return 14  # 快左转
    elif d < -SLOW_DEG:   return 2   # 慢左转
    return gait_idx


def _dist_along(position, direction):
    """计算从入口沿指定方向的行进距离"""
    global _entry_pos
    if _entry_pos is None:
        return 0.0
    dx = position[0] - _entry_pos[0]
    dy = position[1] - _entry_pos[1]
    if direction == HDG_YP:   return dy
    elif direction == HDG_YN: return -dy
    elif direction == HDG_XP: return dx
    elif direction == HDG_XN: return -dx
    return 0.0


def segment5_control(position, gait_mode, rpy):
    """
    第五赛段控制逻辑，每帧（0.2s）调用一次。

    Args:
        position:  [x, y, z]
        gait_mode: [gait_id, mode]
        rpy:       float 机身朝向角（度）

    Returns:
        int: 步态索引；-1 表示赛段5完成
    """
    global _state, _entry_pos

    x, y, _ = position
    gait, mode = gait_mode

    # 步态切换中等待
    if (gait == 0 and mode == 0) or (gait == 1 and mode == 9):
        return 0

    # ── 分段1：y+ (90°) 上坡 4.5m ──────────────────────────────────
    if _state == _ST_SEG1:
        if _entry_pos is None:
            _entry_pos = [x, y]
        if _dist_along(position, HDG_YP) >= SEG1_LENGTH:
            _state = _ST_TURN1
            return 0
        return _forward_at_heading(rpy, HDG_YP, 10)  # 斜坡步态 #10

    # ── 转弯1：左转 90°→180° ──────────────────────────────────────
    elif _state == _ST_TURN1:
        step = _forward_at_heading(rpy, HDG_XN, 9)
        if step == 9:  # 对准后切换
            _state = _ST_SEG2
            _entry_pos = [x, y]
        return step

    # ── 分段2：x- (180°) 不平整 4m ─────────────────────────────────
    elif _state == _ST_SEG2:
        if _dist_along(position, HDG_XN) >= SEG2_LENGTH:
            _state = _ST_TURN2
            return 0
        return _forward_at_heading(rpy, HDG_XN, 9)  # 高抬腿 #9

    # ── 转弯2：右转 180°→90° ──────────────────────────────────────
    elif _state == _ST_TURN2:
        step = _forward_at_heading(rpy, HDG_YP, 9)
        if step == 9:
            _state = _ST_SEG3
            _entry_pos = [x, y]
        return step

    # ── 分段3：y+ (90°) 不平整 4m ─────────────────────────────────
    elif _state == _ST_SEG3:
        if _dist_along(position, HDG_YP) >= SEG3_LENGTH:
            _state = _ST_TURN3
            return 0
        return _forward_at_heading(rpy, HDG_YP, 9)

    # ── 转弯3：右转 90°→0° ────────────────────────────────────────
    elif _state == _ST_TURN3:
        step = _forward_at_heading(rpy, HDG_XP, 9)
        if step == 9:
            _state = _ST_SEG4
            _entry_pos = [x, y]
        return step

    # ── 分段4：x+ (0°) 不平整 4m ──────────────────────────────────
    elif _state == _ST_SEG4:
        if _dist_along(position, HDG_XP) >= SEG4_LENGTH:
            _state = _ST_TURN4
            return 0
        return _forward_at_heading(rpy, HDG_XP, 9)

    # ── 转弯4：右转 0°→270° ───────────────────────────────────────
    elif _state == _ST_TURN4:
        step = _forward_at_heading(rpy, HDG_YN, 28)
        if step == 28:
            _state = _ST_SEG5_FLAT
            _entry_pos = [x, y]
        return step

    # ── 分段5：y- (270°) 平整 1.5m（虚线前）───────────────────────
    elif _state == _ST_SEG5_FLAT:
        if _dist_along(position, HDG_YN) >= SEG5_FLAT:
            _state = _ST_SEG5_JUMP
            _entry_pos = [x, y]
            return 0
        return _forward_at_heading(rpy, HDG_YN, 28)  # 快速前进 #28

    # ── 分段5：跳下区 y- (270°) 0.5m ──────────────────────────────
    elif _state == _ST_SEG5_JUMP:
        if _dist_along(position, HDG_YN) >= SEG5_JUMP:
            _state = _ST_DONE
            return -1  # 赛段5完成
        return _forward_at_heading(rpy, HDG_YN, 1)  # 慢速前进 #1

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

        my_ctrl.num = 2  # 起步微调
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
