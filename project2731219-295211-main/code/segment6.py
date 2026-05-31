"""
第六赛段：撷金建功（角落倒顶 → 转身前推 → 趴下）

方案：贴墙到角自定位 → 角落倒顶顺球 → 转身前推护球进缺口。
设计依据 docs/superpowers/specs/2026-05-31-segment6-corner-nudge-dribble-design.md

统一坐标系（与赛段1-5一致）：原点(0,0)在第一赛段中轴线、距左黄线0.6m、
距下黄线0.5m。x 向右为正(0°)，y 向上为正(90°)，rpy[2] 为机身朝向角(°)。

场地（已确认）：可行驶矩形 x∈[-0.10,2.90]、y∈[12.60,15.10]，本段完全平整无障碍，
  左上角(-0.1,15.1)开阔。边界可触碰不可越。
  足球中心(0.50,14.50) r0.10；右下缺口 x=2.90 y∈[12.60,13.10] 通终点圈，
  圈心(3.15,12.85) r0.25。

核心：贴右墙上行→贴顶墙左行到左上角(两墙钉死x/y自定位)→转尾让头朝148°、尾朝328°→
  后退步态让后体扫过球把球沿328°顶出角落→转身180°头朝328°正对球→低重心前推护球
  穿缝进圈→趴下。328°直线从球(0.5,14.5)穿缺口直达圈心(3.15,12.85)，推球段几何天然成立。
"""

# ── 场地几何（绝对坐标，米）──
LEFT_WALL_X, RIGHT_WALL_X = -0.10, 2.90
BOT_WALL_Y,  TOP_WALL_Y   = 12.60, 15.10
BALL_X, BALL_Y, BALL_R    = 0.50, 14.50, 0.10
FINISH_CX, FINISH_CY, FINISH_R = 3.15, 12.85, 0.25
GAP_X = 2.90

# ── 路径航点（狗机身中心目标值，绝对坐标）──
TOP_Y        = 14.85   # A 退出：狗中心到此贴顶墙（继续贴墙后中心≈14.95、下缘≈14.80>球顶14.60）
CORNER_X     = 0.20    # B 退出：狗中心到此即到左上角（左墙x=-0.10挡停）
NUDGE_EXIT_X = 0.35    # D 退出：狗中心x（后体已扫过球0.50、球被顶离角）
KICK_TRIGGER_X = 2.40  # F→G：狗到此x改快速步态
FINISH_STOP_X  = FINISH_CX  # G：随球停在圈心x（不留余量，确保后脚进缺口）
XY_TOL = 0.08          # 航点到达容差

# ── 朝向角度 ──
HDG_UP      = 90    # A 朝向：+y 上行
HDG_LEFT    = 180   # B 朝向：-x 左行
HDG_HEAD_IN = 148   # C 目标头朝向：扎进左上角（尾朝328°对准缺口）
HDG_PUSH    = 328   # E/F/G 头朝向：球→缺口→圈心方向（atan2(-1.65,2.65)≈-31.9°→328.1°）
FAST_DEG, SLOW_DEG = 20, 8

# ── 步态下标（toml 实测，按数组下标取）──
G_STAND  = 0
G_NAV    = 1     # 前进0.20
G_TURN_L, G_TURN_R   = 2, 3       # 慢转 ±0.25
G_LAY    = 4     # 趴下
G_BACK_SLOW, G_BACK  = 6, 26      # 后退 -0.10 / -0.20
G_FTURN_L, G_FTURN_R = 14, 15     # 快转 ±0.60
G_KICK   = 28    # 快前进0.30
G_PUSH   = 43    # 推球低重心前进0.20

# ── 踢球退路开关（倒顶若仿真失败，置 True 切踢射方案）──
USE_KICK_FALLBACK = False

# ── 状态机状态（八阶段）──
_ST_A_GO_TOP      = "A_GO_TOP"       # 转90°贴右墙上行到顶墙
_ST_B_GO_CORNER   = "B_GO_CORNER"    # 转180°贴顶墙左行到左上角（两墙自定位）
_ST_C_AIM_TAIL    = "C_AIM_TAIL"     # 转头到148°（尾朝328°对准缺口）
_ST_D_NUDGE       = "D_NUDGE"        # 后退步态让后体扫过球，把球沿328°顶出角落
_ST_E_FACE_PUSH   = "E_FACE_PUSH"    # 原地转身头朝328°正对被顶出的球
_ST_F_DRIBBLE     = "F_DRIBBLE"      # 锁328°低重心前推带球到缺口前
_ST_G_THROUGH_GAP = "G_THROUGH_GAP"  # 换快步态送球穿缝，狗随球进圈
_ST_H_LAYDOWN     = "H_LAYDOWN"      # 圈内趴下
_ST_DONE          = "DONE"
# 踢球退路状态（USE_KICK_FALLBACK=True 时启用，收尾复用 H/DONE）
_ST_K_AIM  = "K_AIM"    # 角落外对准缺口方向 328°
_ST_K_KICK = "K_KICK"   # 快步态把球踢/推过缺口，狗随球进圈


# ── 状态机全局变量 ──
_state = None
_laydown_count = 0


def reset_segment6():
    """每次比赛/测试前重置赛段6状态。"""
    global _state, _laydown_count
    _state = _ST_A_GO_TOP
    _laydown_count = 0


def _norm(a):
    """把角度归一化到 (-180, 180]。"""
    while a > 180:   a -= 360
    while a <= -180: a += 360
    return a


def _turn_step(cur_hdg, target_hdg):
    """纯转向选步：对准 target 返回 0（站立），否则返回转向步态下标。"""
    d = _norm(cur_hdg - (target_hdg % 360))
    if d > FAST_DEG:    return G_FTURN_R
    elif d > SLOW_DEG:  return G_TURN_R
    elif d < -FAST_DEG: return G_FTURN_L
    elif d < -SLOW_DEG: return G_TURN_L
    return 0


def _walk(cur_hdg, target_hdg, walk_gait):
    """先对准朝向，对准后返回 walk_gait 前进。"""
    step = _turn_step(cur_hdg, target_hdg)
    return walk_gait if step == 0 else step


def _arrived(cur, target):
    """里程计到达判据：|cur-target| <= XY_TOL。"""
    return abs(cur - target) <= XY_TOL


def detect_ball(frame):
    """识别相机帧中足球，返回球心相对画面中心的横向像素偏移（正=偏右）。

    默认主控传 frame=None → 返回 0.0，走纯里程计（确定性优先）。
    现场需纠偏时传入真实帧：HSV 阈值分割足球颜色 + 最小外接圆求球心 u。
    """
    if frame is None:
        return 0.0
    import cv2, numpy as np
    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
    # 足球颜色阈值现场标定（此处示意白色，需按实际球色调整）
    mask = cv2.inRange(hsv, np.array([0, 0, 200]), np.array([180, 40, 255]))
    cnts, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    if not cnts:
        return 0.0
    (u, _v), _r = cv2.minEnclosingCircle(max(cnts, key=cv2.contourArea))
    return float(u - frame.shape[1] / 2.0)   # 正=球偏右


def segment6_control(position, gait_mode, rpy, frame=None):
    """赛段6控制，每帧(~0.2s)调用。返回步态下标；-1=完成。

    position: [x,y,z] 来自 Pos_msg.position（兼容 [x,y]，忽略 z）
    gait_mode: [gait_id, mode] 来自 Gait_msg.gait_mode
    rpy: float 机身朝向角(°) 来自 Pos_msg.rpy[2]
    frame: 相机帧或 None（视觉钩子，默认关闭）
    """
    global _state, _laydown_count
    x, y = position[0], position[1]
    gait, mode = gait_mode

    # 步态切换中/趴下中等待，避免重复发指令打断动作（与赛段5一致）。
    # H_LAYDOWN / DONE 例外：H 让趴下帧计数确定性推进到 DONE（否则 G_LAY 发出后
    # mode 变7会被此判据锁死，计数卡在1、趴下↔站立抖动）；DONE 是终止态须无条件
    # 返回 -1（否则趴下中 mode==7 会把 -1 拦成 G_STAND，赛段永不报完成）。
    if _state not in (_ST_H_LAYDOWN, _ST_DONE) and (
        (gait == 0 and mode == 0) or (gait == 1 and mode == 9) or mode == 7
    ):
        return G_STAND

    _ = detect_ball(frame)   # 视觉钩子，默认 frame=None 返回 0，不影响里程计主控

    if USE_KICK_FALLBACK:
        return _kick_fallback_control(x, y, rpy)   # Task 3 定义

    # ── A：转90°贴右墙上行到顶墙 ──
    if _state == _ST_A_GO_TOP:
        if y >= TOP_Y:
            _state = _ST_B_GO_CORNER
            return G_STAND
        return _walk(rpy, HDG_UP, G_NAV)

    # ── B：转180°贴顶墙左行到左上角（左墙挡停，两墙自定位）──
    elif _state == _ST_B_GO_CORNER:
        if x <= CORNER_X:
            _state = _ST_C_AIM_TAIL
            return G_STAND
        return _walk(rpy, HDG_LEFT, G_NAV)

    # ── C：转头到148°（尾朝328°对准缺口）──
    elif _state == _ST_C_AIM_TAIL:
        ts = _turn_step(rpy, HDG_HEAD_IN)
        if ts != 0:
            return ts
        _state = _ST_D_NUDGE
        return G_BACK

    # ── D：后退步态让后体扫过球，把球沿328°顶出角落 ──
    elif _state == _ST_D_NUDGE:
        if x >= NUDGE_EXIT_X:
            _state = _ST_E_FACE_PUSH
            return G_STAND
        ts = _turn_step(rpy, HDG_HEAD_IN)   # 保持头朝148°，漂移先转回再退
        if ts != 0:
            return ts
        return G_BACK

    # ── E：原地转身头朝328°正对被顶出的球 ──
    elif _state == _ST_E_FACE_PUSH:
        ts = _turn_step(rpy, HDG_PUSH)
        if ts != 0:
            return ts
        _state = _ST_F_DRIBBLE
        return G_PUSH

    # ── F：锁328°低重心前推，带球到缺口前 ──
    elif _state == _ST_F_DRIBBLE:
        if x >= KICK_TRIGGER_X:
            _state = _ST_G_THROUGH_GAP
            return G_KICK
        return _walk(rpy, HDG_PUSH, G_PUSH)

    # ── G：换快步态送球穿缝，狗随球进圈（不留余量，确保后脚进缺口）──
    elif _state == _ST_G_THROUGH_GAP:
        if x >= FINISH_STOP_X:
            _state = _ST_H_LAYDOWN
            return G_STAND
        return _walk(rpy, HDG_PUSH, G_KICK)

    # ── H：圈内趴下，计3帧后完成 ──
    elif _state == _ST_H_LAYDOWN:
        _laydown_count += 1
        if _laydown_count >= 3:
            _state = _ST_DONE
            return -1
        return G_LAY

    elif _state == _ST_DONE:
        return -1

    return -1


def _kick_fallback_control(x, y, rpy):
    """踢球退路：角落外对准328°→快步态踢射→追球进圈→趴下。

    复用主线朝向/步态/状态骨架，去掉 C/D 倒退环节。
    倒顶若仿真反复顶不到球/顶歪时，置 USE_KICK_FALLBACK=True 启用。
    收尾自带 H/DONE 计数（与主线同形）。
    """
    global _state, _laydown_count

    # 退路首帧（_state 仍是 reset 后的 _ST_A_GO_TOP）→ 切入对准态
    if _state == _ST_A_GO_TOP:
        _state = _ST_K_AIM

    # ── K_AIM：原地转到328°（对准缺口）──
    if _state == _ST_K_AIM:
        ts = _turn_step(rpy, HDG_PUSH)
        if ts != 0:
            return ts
        _state = _ST_K_KICK
        return G_KICK

    # ── K_KICK：快步态踢/推球过缺口，狗随球进圈（锁328°）──
    elif _state == _ST_K_KICK:
        if x >= FINISH_STOP_X:
            _state = _ST_H_LAYDOWN
            return G_STAND
        return _walk(rpy, HDG_PUSH, G_KICK)

    # ── 收尾趴下：本函数自带 H/DONE 副本完成计数（主控仅贡献等待判据对 H/DONE 的豁免）──
    elif _state == _ST_H_LAYDOWN:
        _laydown_count += 1
        if _laydown_count >= 3:
            _state = _ST_DONE
            return -1
        return G_LAY

    elif _state == _ST_DONE:
        return -1

    return -1
