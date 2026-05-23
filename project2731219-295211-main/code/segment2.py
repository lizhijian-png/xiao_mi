"""
第二赛段：荒野寻珠

策略：数独约束推断候选列 + 预横移 + 逐行扫击 + 虚拟出口复用
坐标系（以赛段2入口虚线中点为原点）：
  x+ 向左，y+ 向前（朝出口），入口 x=3.1 y=0 朝向90°
"""

import time
import cv2
import numpy as np

# ── 场地几何 ────────────────────────────────────────────────────
COL_X = {"C1": -0.4, "C2": 0.8, "C3": 2.0, "C4": 3.2}
ROW_Y = {"R4": 1.34, "R3": 2.18, "R2": 3.02, "R1": 3.86}

# 固定蓝球（禁止触碰）
FIXED_BLUE = {("R4", "C3"), ("R4", "C4"), ("R3", "C4")}
ALL_COLS   = ["C1", "C2", "C3", "C4"]   # x 从小(左)到大(右)排列

# 虚拟出口目标（复用导航流程驶出赛段）
EXIT_TARGET = {"x": -0.3, "y": 4.7, "strike_y": 4.50}

# ── 控制参数 ────────────────────────────────────────────────────
STONE_SAFE_Y   = 0.90   # 低于此 y 禁止横移（石板安全线）
PRE_SHIFT_DIST = 0.10   # 预横移距离（m）
FREEZE_OFFSET  = 0.40   # row_y - FREEZE_OFFSET：停止横移开始纯前进
STRIKE_OFFSET  = 0.20   # row_y - STRIKE_OFFSET：冲击终止线
ALIGN_X_TOL    = 0.05   # 对准 x 容差（m）
ALIGN_PX_TOL   = 30     # 视觉对准像素容差（px）
EXIT_TURN_THRESHOLD = 0.30  # 出口对准距离阈值：≥此值转向直行，否则侧移

# ── 视觉参数 ────────────────────────────────────────────────────
ORANGE_HSV_LOWER = np.array([8,  120, 80])
ORANGE_HSV_UPPER = np.array([28, 255, 255])
# 近距离目标球（0.2~0.4m前方）的最小面积阈值，过滤掉远处其他行的橙球
ORANGE_MIN_AREA_NEAR = 1500  # px²  ← 近行目标球；远处球面积远小于此值

# ── 数独推断 ────────────────────────────────────────────────────
_hit_cols    = {}   # {row_name: col_name}
_locked_x    = None # SCAN_ROW 锁定目标球时的机器人 x 坐标，供 _record_hit 使用

def _candidates(row):
    """返回本行橙球候选列列表（x 从小到大），已排除蓝球列和已命中列。"""
    used = set(_hit_cols.values())
    blue = {c for (r, c) in FIXED_BLUE if r == row}
    return [c for c in ALL_COLS if c not in used and c not in blue]


def detect_orange_ball(frame, min_area=ORANGE_MIN_AREA_NEAR, near=False):
    """
    检测摄像头正前方是否有橙色球。

    Args:
        min_area: 最小轮廓面积（px²）。
        near:     True 时只看图像下半区（y>h*5/8），用于 SCAN_ROW 期间过滤
                  投影在画面上方的远处球（3m+ 外）。近处球（0.4m）投影偏下。

    Returns:
        (found: bool, pixel_offset: float)
        pixel_offset: 球中心偏离图像中心的像素数（正=偏右/x增大，负=偏左/x减小）
    """
    h, w = frame.shape[:2]
    # near=True：只取下半部分，排除远处行的球（远处球投影在画面中上方）
    roi_top = h * 5 // 8 if near else h // 4
    roi = frame[roi_top: 3 * h // 4, :]

    hsv  = cv2.cvtColor(roi, cv2.COLOR_BGR2HSV)
    mask = cv2.inRange(hsv, ORANGE_HSV_LOWER, ORANGE_HSV_UPPER)

    kernel = np.ones((5, 5), np.uint8)
    mask   = cv2.morphologyEx(mask, cv2.MORPH_OPEN,  kernel)
    mask   = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)

    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    contours     = [c for c in contours if cv2.contourArea(c) > min_area]

    if not contours:
        return False, 0.0

    largest = max(contours, key=cv2.contourArea)
    M = cv2.moments(largest)
    if M["m00"] == 0:
        return False, 0.0

    cx = int(M["m10"] / M["m00"])
    return True, float(cx - w / 2)   # 正=偏右，负=偏左


# ── 状态定义 ────────────────────────────────────────────────────
_ST_CLEAR_STONE        = "CLEAR_STONE"
_ST_INTER_ROW_SHIFT    = "INTER_ROW_SHIFT"
_ST_ADVANCE_FREEZE     = "ADVANCE_TO_FREEZE"
_ST_TURN_SIDE          = "TURN_SIDE"
_ST_WALK_TO_COL        = "WALK_TO_COL"
_ST_TURN_FRONT         = "TURN_FRONT"
_ST_LATERAL_SHIFT_EXIT = "LATERAL_SHIFT_EXIT"
_ST_SCAN_ROW           = "SCAN_ROW"
_ST_STRIKE             = "STRIKE"
_ST_DONE               = "DONE"

# ── 状态机变量 ──────────────────────────────────────────────────
_state            = _ST_CLEAR_STONE
_target_idx       = 0
_shift_start_x    = None
_targets          = []
_turn_target_hdg  = None   # 当前转向目标朝向角（度）
_walk_target_x    = None   # WALK_TO_COL 目标列 x
_walk_target_y    = None   # WALK_TO_COL 目标 y（冻结线，防止横走时 y 漂移）
_after_turn_state = None   # TURN_FRONT 完成后跳转的状态
_scan_hit_x       = None   # SCAN_ROW 识别到球时的 x，用于侧移 0.15m 后记录命中


def _build_targets():
    """构建目标队列：R4→R3→R2→R1→虚拟出口。"""
    return [
        {"row": "R4", "y": ROW_Y["R4"], "is_exit": False},
        {"row": "R3", "y": ROW_Y["R3"], "is_exit": False},
        {"row": "R2", "y": ROW_Y["R2"], "is_exit": False},
        {"row": "R1", "y": ROW_Y["R1"], "is_exit": False},
        {"row": None, "y": EXIT_TARGET["y"], "is_exit": True,
         "x": EXIT_TARGET["x"], "strike_y": EXIT_TARGET["strike_y"]},
    ]


def reset_segment2():
    global _state, _target_idx, _shift_start_x, _targets, _hit_cols, _locked_x, \
           _turn_target_hdg, _walk_target_x, _after_turn_state
    _state            = _ST_CLEAR_STONE
    _target_idx       = 0
    _shift_start_x    = None
    _locked_x         = None
    _hit_cols         = {}
    _targets          = _build_targets()
    _turn_target_hdg  = None
    _walk_target_x    = None
    _walk_target_y    = None
    _after_turn_state = None
    _scan_hit_x       = None


def _target_col_x(target):
    """
    返回本目标的对准列 x 坐标。
    - 普通行：取候选列中 x 最大（最右）的列，从右向左扫避免被下一行同列橙球干扰
    - 虚拟出口：直接用 EXIT_TARGET["x"]
    """
    if target["is_exit"]:
        return target["x"]
    cands = _candidates(target["row"])
    if not cands:
        return COL_X["C1"]
    return COL_X[cands[-1]]


def _needs_scan(target):
    """是否需要视觉扫描：出口和候选列唯一时跳过。"""
    if target["is_exit"]:
        return False
    return len(_candidates(target["row"])) > 1


def _record_hit(target, x_curr):
    """命中后记录列到 _hit_cols。
    优先用 _locked_x（SCAN_ROW 锁定时的坐标），其次用 x_curr（STRIKE 结束时坐标）。
    """
    if target["is_exit"]:
        return
    cands = _candidates(target["row"])
    if not cands:
        return
    ref_x = _locked_x if _locked_x is not None else x_curr
    hit_col = min(cands, key=lambda c: abs(COL_X[c] - ref_x))
    _hit_cols[target["row"]] = hit_col


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


def segment2_control(position, gait_mode, rpy, frame=None):
    """
    赛段2（荒野寻珠）控制逻辑，每帧调用一次。

    Args:
        position:  [x, y, z]
        gait_mode: [gait_id, mode]
        rpy:       float  机身朝向角（度）
        frame:     np.ndarray or None

    Returns:
        int: 步态索引；-1 表示赛段完成
    """
    global _state, _target_idx, _shift_start_x, _locked_x, \
           _turn_target_hdg, _walk_target_x, _walk_target_y, _after_turn_state, _scan_hit_x

    if not _targets:
        reset_segment2()

    x, y, _ = position
    gait, mode = gait_mode

    if (gait == 0 and mode == 0) or (gait == 1 and mode == 9):
        return 0

    # ── CLEAR_STONE：入场纯前进，脱离石板区 ─────────────────────
    if _state == _ST_CLEAR_STONE:
        if y >= STONE_SAFE_Y:
            _state = _ST_INTER_ROW_SHIFT
            _shift_start_x = x
            return 0
        return 1

    # ── 取当前目标 ───────────────────────────────────────────────
    if _target_idx >= len(_targets):
        _state = _ST_DONE
        return -1
    target = _targets[_target_idx]
    row_y  = target["y"]

    # ── INTER_ROW_SHIFT：预横移，进入列间过道 ────────────────────
    # 虚拟出口跳过预横移，直接进入对准流程
    if _state == _ST_INTER_ROW_SHIFT:
        if target["is_exit"]:
            dist = abs(target["x"] - x)
            if dist >= EXIT_TURN_THRESHOLD:
                _turn_target_hdg  = 0 if target["x"] > x else 180
                _walk_target_x    = target["x"]
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
        if tx < x:
            return 7
        else:
            return 8

    # ── ADVANCE_TO_FREEZE：前进到冻结线 ──────────────────────────
    elif _state == _ST_ADVANCE_FREEZE:
        if y >= row_y - FREEZE_OFFSET:
            tx = _target_col_x(target)
            if abs(x - tx) <= ALIGN_X_TOL:
                _locked_x = None
                _after_turn_state = _ST_SCAN_ROW if _needs_scan(target) else _ST_STRIKE
                _state = _after_turn_state
            else:
                _turn_target_hdg  = 0 if tx > x else 180
                _walk_target_x    = tx
                _walk_target_y    = row_y - FREEZE_OFFSET   # 横走期间保持在冻结线 y
                _after_turn_state = _ST_SCAN_ROW if _needs_scan(target) else _ST_STRIKE
                _state = _ST_TURN_SIDE
            return 0
        return 1

    # ── TURN_SIDE：转向 0° 或 180° 面向目标列，每帧持续校准角度 ──
    elif _state == _ST_TURN_SIDE:
        step = _turn(rpy, _turn_target_hdg)
        if step == 1:
            _state = _ST_WALK_TO_COL
            return 0
        return step

    # ── WALK_TO_COL：朝 0°/180° 直行到目标列 x，每帧校准 y 偏差 ──
    # 机器人朝向 0°/180°，y 方向对应侧移（7=左/y减小，8=右/y增大）
    elif _state == _ST_WALK_TO_COL:
        if abs(x - _walk_target_x) <= ALIGN_X_TOL:
            _state = _ST_TURN_FRONT
            return 0
        hdg_step = _turn(rpy, _turn_target_hdg)
        if hdg_step != 1:
            return hdg_step
        if _walk_target_y is not None:
            dy = y - _walk_target_y
            if dy > ALIGN_X_TOL:
                return 7   # y 偏大，向左侧移（y 减小方向）
            if dy < -ALIGN_X_TOL:
                return 8   # y 偏小，向右侧移（y 增大方向）
        return 1

    # ── TURN_FRONT：转回朝向 90° ──────────────────────────────────
    elif _state == _ST_TURN_FRONT:
        step = _turn(rpy, 90)
        if step == 1:
            _locked_x = None
            _state = _after_turn_state
            return 0
        return step

    # ── LATERAL_SHIFT_EXIT：出口短距离侧移（dist < 0.3m）────────
    elif _state == _ST_LATERAL_SHIFT_EXIT:
        tx = target["x"]
        if abs(x - tx) <= ALIGN_X_TOL:
            _state = _ST_STRIKE
            return 0
        return 7 if tx < x else 8

    # ── SCAN_ROW：视觉扫描，从最右候选列向左找橙球 ───────────────
    # 识别到球后先侧移 0.15m（让身体对齐球心），再记录命中进入下一行
    elif _state == _ST_SCAN_ROW:
        cands      = _candidates(target["row"])
        min_scan_x = COL_X[cands[0]] if cands else x

        # 已锁定球位，正在侧移 0.15m
        if _scan_hit_x is not None:
            if abs(x - _scan_hit_x) >= 0.15:
                _locked_x      = x
                _scan_hit_x    = None
                _record_hit(target, x)
                _target_idx   += 1
                _shift_start_x = x
                _state = _ST_INTER_ROW_SHIFT
                return 0
            return 7 if _scan_hit_x > x else 8   # 朝锁定方向侧移

        if frame is not None:
            found, offset = detect_orange_ball(frame, min_area=ORANGE_MIN_AREA_NEAR, near=True)
            if found:
                if abs(offset) <= ALIGN_PX_TOL:
                    # 像素对准，记录此刻 x 并开始侧移 0.15m
                    _scan_hit_x = x + 0.15   # 向左（x 减小方向）侧移目标
                    return 7
                return 8 if offset > ALIGN_PX_TOL else 7
            else:
                if x >= min_scan_x - ALIGN_X_TOL:
                    return 7
                if abs(x - min_scan_x) > ALIGN_X_TOL:
                    return 7 if min_scan_x < x else 8
                _record_hit(target, x)
                _target_idx   += 1
                _shift_start_x = x
                _state = _ST_INTER_ROW_SHIFT
                return 0
        else:
            _record_hit(target, x)
            _target_idx   += 1
            _shift_start_x = x
            _state = _ST_INTER_ROW_SHIFT
            return 0

    # ── STRIKE（仅出口）：前进冲击到 strike_y ────────────────────
    elif _state == _ST_STRIKE:
        strike_end = target.get("strike_y", row_y - STRIKE_OFFSET)
        if y >= strike_end:
            _record_hit(target, x)
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


# ── 独立测试入口 ──────────────────────────────────────────────
if __name__ == "__main__":
    import sys
    sys.path.insert(0, '/usr/local/lib/python3.8/site-packages')
    sys.path.append("./lcm")

    import threading
    import lcm
    from Robot_Ctrl import Robot_Ctrl
    from Msg_receive import Pos_msg, Gait_msg
    from user_pub import user_pub
    from robot_control_cmd_lcmt import robot_control_cmd_lcmt
    import rclpy
    from rclpy.node import Node
    from sensor_msgs.msg import Image
    from cv_bridge import CvBridge
    from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy

    class CameraNode(Node):
        def __init__(self):
            super().__init__("seg2_camera")
            self.bridge = CvBridge()
            self.frame  = None
            qos = QoSProfile(
                reliability=QoSReliabilityPolicy.BEST_EFFORT,
                history=QoSHistoryPolicy.KEEP_LAST,
                depth=1,
            )
            self.create_subscription(Image, "/rgb_camera/image_raw", self._cb, qos)

        def _cb(self, msg):
            self.frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")

    def main():
        reset_segment2()
        lcm_cmd   = lcm.LCM("udpm://239.255.76.67:7671?ttl=255")
        cmd_msg   = robot_control_cmd_lcmt()
        data_lock = threading.Lock()

        user_pub()
        my_ctrl  = Robot_Ctrl()
        pos_msg  = Pos_msg(data_lock)
        gait_msg = Gait_msg(data_lock)

        rclpy.init(args=None)
        cam_node = CameraNode()

        ctrl_thread = threading.Thread(target=my_ctrl.run,  daemon=True)
        rec_thread  = threading.Thread(target=pos_msg.run,  daemon=True)
        gait_thread = threading.Thread(target=gait_msg.run, daemon=True)
        ros_thread  = threading.Thread(target=lambda: rclpy.spin(cam_node), daemon=True)

        ctrl_thread.start()
        time.sleep(4)
        rec_thread.start()
        gait_thread.start()
        ros_thread.start()

        print("=== 赛段二：荒野寻珠 开始 ===")
        try:
            while True:
                with data_lock:
                    pos  = list(pos_msg.position)
                    gait = list(gait_msg.gait_mode)
                    yaw  = pos_msg.rpy[2]

                step = segment2_control(pos, gait, yaw, cam_node.frame)

                if step == -1:
                    print("=== 赛段二完成 ===")
                    break

                my_ctrl.num = step
                my_ctrl.msg.life_count = (my_ctrl.msg.life_count + 1) % 127

                print(
                    f"pos={[round(v,2) for v in pos]}  yaw={yaw:.1f}°  "
                    f"state={_state}  target={_target_idx}  step={step}  "
                    f"hit={_hit_cols}"
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
            rclpy.shutdown()
            sys.exit()

    main()
