"""
第二赛段（人工输入版）：荒野寻珠 - 手动预设列直撞版

用法：
  python3 segment2_manual.py 1 2 3

命令行后 3 个整数表示 R4/R3/R2 行橙色小球所在列（1~4 对应 C1~C4）。
R1 行由排除法自动推出（每行每列各一个橙球）。

策略：跳过视觉扫描，直接按预设列去撞。原状态机（入场清石板、
     行间预横移、冻结、侧向对准、冲击、虚拟出口）全部保留，
     仅将 SCAN_ROW 从流程中移除。

坐标系（以赛段2入口虚线中点为原点）：
  x+ 向左，y+ 向前（朝出口），入口 x=3.1 y=0 朝向90°
"""

import sys
import time

# ── 场地几何 ────────────────────────────────────────────────────
COL_X = {"C1": -0.4, "C2": 0.8, "C3": 2.0, "C4": 3.2}
ROW_Y = {"R4": 1.34, "R3": 2.18, "R2": 3.02, "R1": 3.86}

# 固定蓝球（禁止触碰）——输入若违反将被 parse_preset_args 拦下
FIXED_BLUE = {("R4", "C3"), ("R4", "C4"), ("R3", "C4")}
ALL_ROWS = ["R4", "R3", "R2", "R1"]
ROWS_INPUT_ORDER = ["R4", "R3", "R2"]   # 命令行 3 个数字对应的行

# 虚拟出口目标（复用导航流程驶出赛段）
EXIT_TARGET = {"x": -0.3, "y": 4.7, "strike_y": 4.50}

# ── 控制参数 ────────────────────────────────────────────────────
STONE_SAFE_Y        = 0.90   # 低于此 y 禁止横移（石板安全线）
PRE_SHIFT_DIST      = 0.10   # 预横移距离（m）
FREEZE_OFFSET       = 0.42   # row_y - FREEZE_OFFSET：停止横移开始纯前进
STRIKE_OFFSET       = 0.20   # row_y - STRIKE_OFFSET：冲击终止线
ALIGN_X_TOL         = 0.05   # 对准 x 容差（m）
EXIT_TURN_THRESHOLD = 0.30   # 出口对准距离阈值：≥此值转向直行，否则侧移

# ── 状态定义 ────────────────────────────────────────────────────
_ST_CLEAR_STONE        = "CLEAR_STONE"
_ST_INTER_ROW_SHIFT    = "INTER_ROW_SHIFT"
_ST_ADVANCE_FREEZE     = "ADVANCE_TO_FREEZE"
_ST_TURN_SIDE          = "TURN_SIDE"
_ST_WALK_TO_COL        = "WALK_TO_COL"
_ST_TURN_FRONT         = "TURN_FRONT"
_ST_LATERAL_SHIFT_EXIT = "LATERAL_SHIFT_EXIT"
_ST_STRIKE             = "STRIKE"
_ST_DONE               = "DONE"

# ── 状态机变量 ──────────────────────────────────────────────────
_state            = _ST_CLEAR_STONE
_target_idx       = 0
_shift_start_x    = None
_targets          = []
_turn_target_hdg  = None
_walk_target_x    = None
_walk_target_y    = None
_after_turn_state = None
_preset_cols      = {}   # {"R4":"C1", "R3":"C2", "R2":"C3", "R1":"C4"}


# ══════════════════════════════════════════════════════════════
# 命令行参数解析
# ══════════════════════════════════════════════════════════════
def parse_preset_args(argv):
    """
    解析命令行参数（不含程序名），返回 4 行完整预设 dict。

    Args:
        argv: 长度为 3 的字符串列表，如 ["1","2","3"]
              分别表示 R4/R3/R2 行橙球所在列号（1~4）

    Returns:
        {"R4":"C?","R3":"C?","R2":"C?","R1":"C?"} —— R1 由排除法推出

    Raises:
        ValueError: 参数数量不对 / 非1~4整数 / 重复 / 触碰蓝球
    """
    if len(argv) != 3:
        raise ValueError(
            f"需要 3 个整数(R4/R3/R2 的列号 1~4)，实际收到 {len(argv)} 个: {argv}"
        )
    try:
        nums = [int(a) for a in argv]
    except ValueError as e:
        raise ValueError(f"参数必须是整数，收到: {argv}") from e
    for n in nums:
        if n < 1 or n > 4:
            raise ValueError(f"列号必须在 1~4 之间，收到: {n}")
    if len(set(nums)) != 3:
        raise ValueError(f"三个列号必须互不相同（每列一个橙球），收到: {nums}")

    remaining = list({1, 2, 3, 4} - set(nums))
    assert len(remaining) == 1
    nums.append(remaining[0])

    preset = {row: f"C{n}" for row, n in zip(ALL_ROWS, nums)}

    violations = [(r, c) for r, c in preset.items() if (r, c) in FIXED_BLUE]
    if violations:
        raise ValueError(
            f"输入的橙球位置与固定蓝球冲突: {violations}; "
            f"FIXED_BLUE = {sorted(FIXED_BLUE)}"
        )
    return preset


# ══════════════════════════════════════════════════════════════
# 状态机
# ══════════════════════════════════════════════════════════════
def _build_targets():
    """构建目标队列：R4→R3→R2→R1→虚拟出口。"""
    return [
        {"row": "R4", "y": ROW_Y["R4"], "is_exit": False},
        {"row": "R3", "y": ROW_Y["R3"], "is_exit": False},
        {"row": "R2", "y": ROW_Y["R2"], "is_exit": False},
        {"row": "R1", "y": ROW_Y["R1"], "is_exit": False, "strike_y": 4.20},
        {"row": None, "y": EXIT_TARGET["y"], "is_exit": True,
         "x": EXIT_TARGET["x"], "strike_y": EXIT_TARGET["strike_y"]},
    ]


def reset_segment2_manual(preset):
    """
    重置状态机。preset 必须为 4 行完整字典，
    通常由 parse_preset_args 生成。
    """
    global _state, _target_idx, _shift_start_x, _targets, \
           _turn_target_hdg, _walk_target_x, _walk_target_y, \
           _after_turn_state, _preset_cols
    missing = [r for r in ALL_ROWS if r not in preset]
    if missing:
        raise ValueError(f"preset 缺少行 {missing}，收到 {preset}")
    _state            = _ST_CLEAR_STONE
    _target_idx       = 0
    _shift_start_x    = None
    _turn_target_hdg  = None
    _walk_target_x    = None
    _walk_target_y    = None
    _after_turn_state = None
    _targets          = _build_targets()
    _preset_cols      = dict(preset)


def _target_col_x(target):
    """返回目标对准 x：普通行取预设列 x，虚拟出口取 EXIT_TARGET["x"]。"""
    if target["is_exit"]:
        return target["x"]
    return COL_X[_preset_cols[target["row"]]]


_FAST_DEG = 15
_SLOW_DEG = 5


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


def segment2_manual_control(position, gait_mode, rpy, frame=None):
    """
    赛段2（人工输入版）控制逻辑，每帧调用一次。

    Args:
        position:  [x, y, z]
        gait_mode: [gait_id, mode]
        rpy:       float  机身朝向角（度）
        frame:     忽略（保留签名以兼容总编排器）

    Returns:
        int: 步态索引；-1 表示赛段完成
    """
    global _state, _target_idx, _shift_start_x, \
           _turn_target_hdg, _walk_target_x, _walk_target_y, _after_turn_state

    if not _targets:
        raise RuntimeError("请先调用 reset_segment2_manual(preset) 初始化")

    x, y, _ = position
    gait, mode = gait_mode

    if (gait == 0 and mode == 0) or (gait == 1 and mode == 9):
        return 0

    # ── CLEAR_STONE：入场纯前进，脱离石板区 ─────────────────────
    if _state == _ST_CLEAR_STONE:
        if y >= STONE_SAFE_Y:
            _state = _ST_ADVANCE_FREEZE
            return 0
        return 1

    if _target_idx >= len(_targets):
        _state = _ST_DONE
        return -1
    target = _targets[_target_idx]
    row_y  = target["y"]

    # ── INTER_ROW_SHIFT：行间预横移 ─────────────────────────────
    if _state == _ST_INTER_ROW_SHIFT:
        if target["is_exit"]:
            dist = abs(target["x"] - x)
            if dist >= EXIT_TURN_THRESHOLD:
                _turn_target_hdg  = 0 if target["x"] > x else 180
                _walk_target_x    = target["x"]
                _walk_target_y    = 4.25
                _after_turn_state = _ST_STRIKE
                _state = _ST_TURN_SIDE
            else:
                _state = _ST_LATERAL_SHIFT_EXIT
            return 0
        if _shift_start_x is None:
            _shift_start_x = x
        tx      = _target_col_x(target)
        shifted = abs(x - _shift_start_x)
        if shifted >= PRE_SHIFT_DIST:
            _state = _ST_ADVANCE_FREEZE
            return 0
        return 7 if tx < x else 8

    # ── ADVANCE_TO_FREEZE：前进到冻结线 ────────────────────────
    elif _state == _ST_ADVANCE_FREEZE:
        if y >= row_y - FREEZE_OFFSET:
            tx = _target_col_x(target)
            if abs(x - tx) <= ALIGN_X_TOL:
                _after_turn_state = _ST_STRIKE
                _state = _ST_STRIKE
            else:
                _turn_target_hdg  = 0 if tx > x else 180
                _walk_target_x    = tx
                _walk_target_y    = row_y - FREEZE_OFFSET
                _after_turn_state = _ST_STRIKE
                _state = _ST_TURN_SIDE
            return 0
        return 1

    # ── TURN_SIDE：转向 0° 或 180° 面向目标列 ──────────────────
    elif _state == _ST_TURN_SIDE:
        step = _turn(rpy, _turn_target_hdg)
        if step == 1:
            _state = _ST_WALK_TO_COL
            return 0
        return step

    # ── WALK_TO_COL：朝 0°/180° 直行到目标列 x ────────────────
    # step 7 = body-left (+body_y), step 8 = body-right (-body_y)
    # 映射到世界 y：
    #   yaw=180° → step7=世界 -y, step8=世界 +y
    #   yaw=  0° → step7=世界 +y, step8=世界 -y  (与 180° 相反！)
    elif _state == _ST_WALK_TO_COL:
        if abs(x - _walk_target_x) <= ALIGN_X_TOL:
            _state = _ST_TURN_FRONT
            return 0
        hdg_step = _turn(rpy, _turn_target_hdg)
        if hdg_step != 1:
            return hdg_step
        if _walk_target_y is not None:
            dy = y - _walk_target_y
            if _turn_target_hdg == 180:
                if dy > ALIGN_X_TOL:
                    return 7   # 需要 -y，180° 下 body-left = 世界 -y
                if dy < -ALIGN_X_TOL:
                    return 8   # 需要 +y，180° 下 body-right = 世界 +y
            else:  # _turn_target_hdg == 0
                if dy > ALIGN_X_TOL:
                    return 8   # 需要 -y，0° 下 body-right = 世界 -y
                if dy < -ALIGN_X_TOL:
                    return 7   # 需要 +y，0° 下 body-left = 世界 +y
        return 1

    # ── TURN_FRONT：转回朝向 90° ────────────────────────────────
    elif _state == _ST_TURN_FRONT:
        step = _turn(rpy, 90)
        if step == 1:
            _state = _after_turn_state
            return 0
        return step

    # ── LATERAL_SHIFT_EXIT：出口短距离侧移 ─────────────────────
    elif _state == _ST_LATERAL_SHIFT_EXIT:
        tx = target["x"]
        if abs(x - tx) <= ALIGN_X_TOL:
            _state = _ST_STRIKE
            return 0
        return 7 if tx < x else 8

    # ── STRIKE：前进冲击到 strike_y ────────────────────────────
    elif _state == _ST_STRIKE:
        strike_end = target.get("strike_y", row_y - STRIKE_OFFSET)
        if y >= strike_end:
            if target["is_exit"]:
                _state = _ST_DONE
                return -1
            _target_idx   += 1
            _shift_start_x = x
            _state         = _ST_INTER_ROW_SHIFT
            return 0
        return 1

    else:
        return -1


# ══════════════════════════════════════════════════════════════
# 硬件运行入口
# ══════════════════════════════════════════════════════════════
if __name__ == "__main__":
    preset = parse_preset_args(sys.argv[1:])
    print(f"预设橙球位置: {preset}")

    sys.path.insert(0, '/usr/local/lib/python3.8/site-packages')
    sys.path.append("./lcm")

    import threading
    import lcm
    from Robot_Ctrl import Robot_Ctrl
    from Msg_receive import Pos_msg, Gait_msg
    from user_pub import user_pub
    from robot_control_cmd_lcmt import robot_control_cmd_lcmt

    def main():
        reset_segment2_manual(preset)
        lcm_cmd   = lcm.LCM("udpm://239.255.76.67:7671?ttl=255")
        cmd_msg   = robot_control_cmd_lcmt()
        data_lock = threading.Lock()

        user_pub()
        my_ctrl  = Robot_Ctrl()
        pos_msg  = Pos_msg(data_lock)
        gait_msg = Gait_msg(data_lock)

        ctrl_thread = threading.Thread(target=my_ctrl.run,  daemon=True)
        rec_thread  = threading.Thread(target=pos_msg.run,  daemon=True)
        gait_thread = threading.Thread(target=gait_msg.run, daemon=True)

        ctrl_thread.start()
        time.sleep(4)
        rec_thread.start()
        gait_thread.start()

        print("=== 赛段二（人工输入版）开始 ===")
        try:
            while True:
                with data_lock:
                    pos  = list(pos_msg.position)
                    gait = list(gait_msg.gait_mode)
                    yaw  = pos_msg.rpy[2]

                step = segment2_manual_control(pos, gait, yaw)

                if step == -1:
                    print("=== 赛段二（人工输入版）完成 ===")
                    break

                my_ctrl.num = step
                my_ctrl.msg.life_count = (my_ctrl.msg.life_count + 1) % 127

                print(
                    f"pos={[round(v,2) for v in pos]}  yaw={yaw:.1f}°  "
                    f"state={_state}  target={_target_idx}  step={step}  "
                    f"preset={_preset_cols}"
                )

                if step == 0:
                    time.sleep(0.5)

        except KeyboardInterrupt:
            pass
        finally:
            cmd_msg.mode       = 7
            cmd_msg.gait_id    = 0
            cmd_msg.duration   = 0
            cmd_msg.life_count += 1
            lcm_cmd.publish("robot_control_cmd", cmd_msg.encode())
            sys.exit()

    main()
