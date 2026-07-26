"""
第六赛段：撷金建功（角落横移顶球 → 验球闭环重试 → 推球进圈 → 趴下）

方案：贴墙到角自定位 → 转头到225°→低重心左横移把球顶出角落 → 转身让头朝328°
      →【视觉验球】球出角落则前推护球进缺口；未出角落则退回角落换参数重顶（最多3次尝试）。
设计依据 docs/superpowers/specs/2026-07-26-segment6-realrobot-closed-loop-design.md
（前置设计 docs/superpowers/specs/2026-05-31-segment6-corner-sidestep-sweep-design.md）

统一坐标系（与赛段1-5一致）：原点(0,0)在第一赛段中轴线、距左黄线0.6m、
距下黄线0.5m。x 向右为正(0°)，y 向上为正(90°)，rpy[2] 为机身朝向角(°)。

场地（已确认）：可行驶矩形 x∈[0.0,2.8]、y∈[12.7,15.0]（黄线内侧），本段完全平整无障碍，
  左上角(0.0,15.0)开阔。边界可触碰不可越。
  足球中心(0.50,14.50) r0.10；右下缺口 x=2.80 y∈[12.7,13.10] 通终点圈，
  圈心(3.15,12.85) r0.25。

真机化三点加固（本次改动）：
  1. A/B 贴墙退出加「位移停滞」兜底判据 —— 全程连跑不重置里程计，seg6 拿到的坐标
     带前五段累积漂移；漂移偏大会让狗顶墙打滑而坐标阈值永不满足（卡死），偏小会
     提前转向顶空。墙的物理位置可靠：撞上就走不动，「走不动」不依赖绝对坐标。
  2. E 段转身后插入 D_VERIFY 视觉验球 —— 顶球时头朝225°而球在世界315°方向（差90°），
     顶球全程相机看不到球；转到328°后球才落在画面中央偏右，故验球只能放在 E 之后。
     只判是非（球在不在视野）不测距：单目估距对机身俯仰极敏感，是非判断则免疫。
  3. 验球失败经 R_RETURN 复用贴墙机制退回角落重顶，每次换侧移量与朝向（小型搜索）；
     尝试用尽自动切踢球退路。三层超时看护确保任何路径都不会永久卡死。
"""

import time

# ── 场地几何（绝对坐标，米）──
LEFT_WALL_X, RIGHT_WALL_X = 0.0, 2.8     # 黄线内侧实际边界（原 -0.10/2.90）
BOT_WALL_Y,  TOP_WALL_Y   = 12.7, 15.0   # 黄线内侧实际边界（原 12.60/15.10）
BALL_X, BALL_Y, BALL_R    = 0.50, 14.50, 0.10   # 物理位置不变
FINISH_CX, FINISH_CY, FINISH_R = 3.15, 12.85, 0.25   # 物理位置不变（矩形外）
GAP_X = 2.80   # 缺口随右边界收紧（原 2.90）

# ── 路径航点（狗机身中心目标值，绝对坐标）──
TOP_Y        = 14.80   # A 退出：贴顶墙（顶墙15.0，继续贴墙后中心≈14.85、下缘≈14.70>球顶14.60）
CORNER_X     = 0.20    # B 退出：狗中心到此即到左上角（左墙x=0.0挡停）
KICK_TRIGGER_X = 2.40  # F→G：狗到此x改快速步态
FINISH_STOP_X  = FINISH_CX  # G：随球停在圈心x（不留余量，确保后脚进缺口）
XY_TOL = 0.08          # 航点到达容差

# ── 朝向角度 ──
HDG_UP      = 90    # A 朝向：+y 上行
HDG_LEFT    = 180   # B 朝向：-x 左行
HDG_SWEEP   = 225   # C 目标头朝向基准（左侧身体朝右下顶球）；实际按 RETRY_PROFILE 取
SWEEP_L_DIST = 0.25  # D1 退出：左扫里程位移阈值基准；实际按 RETRY_PROFILE 取
SWEEP_R_DIST = 0.15  # D2 退出：右退里程位移阈值（拉开间隙便于转身不碰球，约15cm）
HDG_PUSH    = 328   # E/F/G 头朝向：球→缺口→圈心方向（atan2(-1.65,2.65)≈-31.9°→328.1°）
HDG_FINISH  = 0     # 终点圈内趴下前转身朝向：正对 +x
FAST_DEG, SLOW_DEG = 20, 8

# ── 步态下标（toml 实测，按数组下标取）──
G_STAND  = 0
G_NAV    = 1     # 前进0.20
G_TURN_L, G_TURN_R   = 2, 3       # 慢转 ±0.25
G_LAY    = 4     # 趴下
G_BACK_SLOW, G_BACK  = 6, 26      # 后退 -0.10 / -0.20
G_FTURN_L, G_FTURN_R = 14, 15     # 快转 ±0.60
G_KICK   = 28    # 快前进0.30
G_PUSH   = 51    # 推球低重心前进0.20（usergait.toml 真实下标51）
G_SWEEP_L = 52   # 低重心左横移 vel_y+0.08、posZ-0.05（usergait.toml 真实下标52）
G_SWEEP_R = 53   # 低重心右横移 vel_y-0.10（usergait.toml 真实下标53）

# ── 顶球尝试参数表：(D1 左扫位移阈值 m, C 段目标头朝向 °) ──
# 顶球失败两大主因：扫得不够远（球没出角）、扫的位置偏了（从球旁边过去）。
# 侧移量递增解决前者；朝向在 235°/215° 两侧各试一次解决后者 —— 这是小型搜索，
# 而不是把同一个失败动作重复三遍。
RETRY_PROFILE = [
    (0.25, 225),   # 第1次尝试（首顶）：基准
    (0.32, 235),   # 第2次尝试：更朝下，扫得更靠外
    (0.40, 215),   # 第3次尝试：更朝左，扫得更靠里
]
# MAX_ATTEMPT 计的是顶球尝试总次数（含首顶）：1 次首顶 + 2 次重顶 = 3 次；
# 第 4 次不再尝试，直接切踢球退路。
MAX_ATTEMPT = len(RETRY_PROFILE)

# ── 视觉验球参数 ──
# HSV 阈值是现场标定的唯一必改项（拍几帧角落里的球，调这两行到稳定命中即可）。
BALL_HSV_LO = (0, 0, 200)      # 默认白球下界（H,S,V）
BALL_HSV_HI = (180, 40, 255)   # 默认白球上界
# 备用：亮色球（橙/黄）阈值，现场若为彩色球改用这组
# BALL_HSV_LO = (10, 120, 120)
# BALL_HSV_HI = (35, 255, 255)
BALL_CIRCULARITY_MIN = 0.6   # 圆度下界，滤掉细长的黄线/墙缝/阴影带
BALL_MIN_R = 8               # 像素半径下界，滤掉小噪点
BALL_MAX_R = 220             # 像素半径上界，滤掉大片高光墙面
VERIFY_WINDOW     = 5    # 判定窗口帧数
VERIFY_HITS       = 3    # 窗口内命中帧数达此值判「球已出角落」
VERIFY_MAX_FRAMES = 10   # 验球总帧数上限（约2秒），到此仍未凑够按失败处理

# ── 贴墙停滞判据参数 ──
STALL_EPS    = 0.015   # 单帧位移阈值1.5cm（正常步态 0.20m/s × 0.2s ≈ 4cm）
STALL_FRAMES = 8       # 连续8帧≈1.6s。低重心步态有迈步周期、单帧位移会周期性接近零，
                       # 8帧跨越多个完整周期，只有真被墙挡住才会连续8帧不动。

# ── 三层超时看护（秒，time.monotonic 计时）──
STATE_TIMEOUT = 15.0   # 单状态超时：强制推进，兜住「停滞检测也失效」的极端情况
PUSH_TIMEOUT  = 25.0   # 推球段(F+G)超时：F/G 无墙可撞只能靠坐标，超时就地趴下保完赛
SEG_TIMEOUT   = 120.0  # 赛段总超时：放弃球，前推后趴下（唯一允许放弃球的地方——
                       # 卡死在场上不趴下比球没进圈更糟，后者只丢本段分，前者可能让
                       # 整条赛道无法收尾）

# ── 踢球退路开关（置 True 强制走踢射方案；验球尝试用尽也会自动切入）──
USE_KICK_FALLBACK = False

# ── 状态机状态 ──
_ST_A_GO_TOP      = "A_GO_TOP"       # 转90°贴右墙上行到顶墙
_ST_B_GO_CORNER   = "B_GO_CORNER"    # 转180°贴顶墙左行到左上角（两墙自定位）
_ST_C_AIM_SWEEP   = "C_AIM_SWEEP"    # 转头到本次尝试的顶球朝向（默认225°）
_ST_D1_SWEEP      = "D1_SWEEP"       # 低重心左横移，把球顶出角落
_ST_D2_CLEAR      = "D2_CLEAR"       # 低重心右横移，退开拉间隙便于转身不碰球
_ST_E_FACE_PUSH   = "E_FACE_PUSH"    # 原地转身头朝328°（转完球才进视野，供验球）
_ST_D_VERIFY      = "D_VERIFY"       # 视觉验球：球是否已离开角落
_ST_R_RETURN      = "R_RETURN"       # 验球失败：退回角落重顶（复用A/B贴墙机制）
_ST_F_DRIBBLE     = "F_DRIBBLE"      # 锁328°低重心前推带球到缺口前
_ST_G_THROUGH_GAP = "G_THROUGH_GAP"  # 低重心推球穿缝，狗随球进圈
_ST_ABANDON       = "ABANDON"        # 赛段总超时：放弃球，前推到停滞后收尾
_ST_TURN_FINISH   = "TURN_FINISH"    # 圈内原地转身，头从328°转到正对+x(0°)
_ST_H_LAYDOWN     = "H_LAYDOWN"      # 圈内趴下
_ST_DONE          = "DONE"
# 踢球退路状态（USE_KICK_FALLBACK=True 或验球尝试用尽时启用，收尾复用 H/DONE）
_ST_K_AIM  = "K_AIM"    # 角落外对准缺口方向 328°
_ST_K_KICK = "K_KICK"   # 快步态把球踢/推过缺口，狗随球进圈

# 单状态超时强制推进的目标（兜底用，不参与正常流转）
_FORCE_NEXT = {
    _ST_A_GO_TOP:      _ST_B_GO_CORNER,
    _ST_B_GO_CORNER:   _ST_C_AIM_SWEEP,
    _ST_C_AIM_SWEEP:   _ST_D1_SWEEP,
    _ST_D1_SWEEP:      _ST_D2_CLEAR,
    _ST_D2_CLEAR:      _ST_E_FACE_PUSH,
    _ST_E_FACE_PUSH:   _ST_D_VERIFY,
    _ST_R_RETURN:      _ST_C_AIM_SWEEP,
    _ST_F_DRIBBLE:     _ST_G_THROUGH_GAP,
    _ST_G_THROUGH_GAP: _ST_TURN_FINISH,
    _ST_ABANDON:       _ST_TURN_FINISH,
    _ST_TURN_FINISH:   _ST_H_LAYDOWN,
    _ST_K_AIM:         _ST_K_KICK,
    _ST_K_KICK:        _ST_TURN_FINISH,
}

# 需要跨帧计数、必须绕过「步态切换中等待」判据的状态
# （D_VERIFY 要累计验球帧，H/DONE 要让趴下计数确定性推进到完成）
_NO_WAIT_STATES = (_ST_D_VERIFY, _ST_H_LAYDOWN, _ST_DONE)


# ── 状态机全局变量 ──
_state = None
_laydown_count = 0
_sweep_x0 = None   # D1/D2 段横移起点 x（进 D1/D2 首帧记录）
_sweep_y0 = None   # D1/D2 段横移起点 y
_attempt = 0       # 顶球尝试序号（0-based，索引 RETRY_PROFILE）
_verify_hit = 0    # D_VERIFY 命中帧数
_verify_frames = 0 # D_VERIFY 已取帧数
_stall = {}        # 停滞检测器：key -> [上帧值, 连续停滞帧数]
_seg_start_t = None    # 赛段起始时刻（monotonic）
_state_enter_t = None  # 当前状态进入时刻（monotonic）
_push_start_t = None   # 推球段(F)进入时刻（monotonic）


def reset_segment6():
    """每次比赛/测试前重置赛段6状态。"""
    global _state, _laydown_count, _sweep_x0, _sweep_y0
    global _attempt, _verify_hit, _verify_frames, _stall
    global _seg_start_t, _state_enter_t, _push_start_t
    _state = _ST_A_GO_TOP
    _laydown_count = 0
    _sweep_x0 = None
    _sweep_y0 = None
    _attempt = 0
    _verify_hit = 0
    _verify_frames = 0
    _stall = {}
    _seg_start_t = time.monotonic()
    _state_enter_t = _seg_start_t
    _push_start_t = None


def _set_state(new_state):
    """切状态并重置与状态绑定的计时器/计数器。

    停滞检测器按状态清空：每个状态的「走不动」判断必须从头计，
    否则上一状态残留的计数会让新状态一进来就误判停滞。
    """
    global _state, _state_enter_t, _stall, _push_start_t
    _state = new_state
    _state_enter_t = time.monotonic()
    _stall = {}
    if new_state == _ST_F_DRIBBLE and _push_start_t is None:
        _push_start_t = _state_enter_t   # 推球段计时覆盖 F+G 全程，只在进F时起表


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


def _dist(x, y, x0, y0):
    """两点欧氏距离，横移位移判据用。"""
    return ((x - x0) ** 2 + (y - y0) ** 2) ** 0.5


def _is_stalled(cur_val, key):
    """撞墙判据：连续 STALL_FRAMES 帧位移增量都 < STALL_EPS 则返回 True。

    不依赖绝对坐标 —— 墙的物理位置可靠，撞上就走不动。用于 A/B 段贴墙退出的
    兜底判据（与坐标判据取「或」，谁先满足谁生效）。key 使各状态独立计数。
    """
    rec = _stall.get(key)
    if rec is None:
        _stall[key] = [cur_val, 0]
        return False
    if abs(cur_val - rec[0]) < STALL_EPS:
        rec[1] += 1
    else:
        rec[1] = 0
    rec[0] = cur_val
    return rec[1] >= STALL_FRAMES


def _sweep_params():
    """取本次尝试的 (D1左扫位移阈值, C段目标头朝向)，越界则退化到最后一档。"""
    idx = min(_attempt, len(RETRY_PROFILE) - 1)
    return RETRY_PROFILE[idx]


def _ball_from_candidates(candidates, frame_width):
    """从候选圆里挑出足球（纯函数，不依赖 cv2，可离线单测）。

    candidates: [(area, cx, radius), ...]  面积/圆心x/半径，均为像素
    frame_width: 画面宽度，用于算相对中心的横向偏移
    返回 (found, u_offset, radius_px)：u_offset 正=球偏右，仅用于微调朝向；
    radius_px 仅用于过滤噪声，不用于测距。
    """
    best = None
    for area, cx, radius in candidates:
        if radius <= 0:
            continue
        if not (BALL_MIN_R < radius < BALL_MAX_R):
            continue
        circularity = area / (3.141592653589793 * radius * radius)
        if circularity <= BALL_CIRCULARITY_MIN:
            continue
        if best is None or area > best[0]:
            best = (area, cx, radius)
    if best is None:
        return False, 0.0, 0.0
    return True, float(best[1] - frame_width / 2.0), float(best[2])


def find_ball(frame):
    """在画面下半区找足球，返回 (found, u_offset, radius_px)。

    只判是非不测距：单目估距靠「已知球径+像素半径」反解，误差对相机俯仰角极敏感，
    而狗在低重心步态下机身俯仰持续摆动；「画面里有没有一个足球大小的圆」这个
    判断对俯仰摆动几乎免疫，且恰好就是所需答案。

    frame=None（主控未传帧/相机故障）→ (False, 0.0, 0.0)，调用方据此降级走纯里程计。
    cv2/numpy 延迟导入：保证 frame=None 路径只依赖标准库，无相机环境也能跑单测。
    """
    if frame is None:
        return False, 0.0, 0.0
    import cv2
    import numpy as np
    h, w = frame.shape[:2]
    lower = frame[h // 2:, :]        # 只看下半区：球在地面，天然排除灯光/场外杂物/远景
    hsv = cv2.cvtColor(lower, cv2.COLOR_BGR2HSV)
    mask = cv2.inRange(hsv, np.array(BALL_HSV_LO), np.array(BALL_HSV_HI))
    cnts, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    candidates = []
    for c in cnts:
        (cx, _cy), r = cv2.minEnclosingCircle(c)
        candidates.append((float(cv2.contourArea(c)), float(cx), float(r)))
    return _ball_from_candidates(candidates, w)


def segment6_control(position, gait_mode, rpy, frame=None):
    """赛段6控制，每帧(~0.2s)调用。返回步态下标；-1=完成。

    position: [x,y,z] 来自 Pos_msg.position（兼容 [x,y]，忽略 z）
    gait_mode: [gait_id, mode] 来自 Gait_msg.gait_mode
    rpy: float 机身朝向角(°) 来自 Pos_msg.rpy[2]
    frame: 相机帧或 None（None 时验球降级放行，走纯里程计）
    """
    global _state, _laydown_count, _sweep_x0, _sweep_y0
    global _attempt, _verify_hit, _verify_frames
    x, y = position[0], position[1]
    gait, mode = gait_mode

    if _state is None:          # 未 reset 就被调用：按首帧处理，避免 None 状态穿透
        reset_segment6()

    # ── 赛段总超时：放弃球保完赛（唯一允许放弃球的地方）──
    if (_state not in (_ST_ABANDON, _ST_TURN_FINISH, _ST_H_LAYDOWN, _ST_DONE)
            and time.monotonic() - _seg_start_t > SEG_TIMEOUT):
        _set_state(_ST_ABANDON)
        return G_STAND

    # ── 推球段(F+G)超时：无墙可撞只能靠坐标，超时就地收尾 ──
    if (_state in (_ST_F_DRIBBLE, _ST_G_THROUGH_GAP)
            and _push_start_t is not None
            and time.monotonic() - _push_start_t > PUSH_TIMEOUT):
        _set_state(_ST_TURN_FINISH)
        return G_STAND

    # ── 单状态超时：强制推进，兜住停滞检测也失效的极端情况 ──
    if (_state in _FORCE_NEXT
            and time.monotonic() - _state_enter_t > STATE_TIMEOUT):
        _set_state(_FORCE_NEXT[_state])
        return G_STAND

    # 步态切换中/趴下中等待，避免重复发指令打断动作（与赛段5一致）。
    # _NO_WAIT_STATES 例外：D_VERIFY 要累计验球帧（站立时 gait/mode 会命中本判据，
    # 一旦被拦住帧数永远攒不够）；H_LAYDOWN 让趴下帧计数确定性推进到 DONE（否则
    # G_LAY 发出后 mode 变7会被此判据锁死，计数卡在1、趴下↔站立抖动）；DONE 是
    # 终止态须无条件返回 -1（否则趴下中 mode==7 会把 -1 拦成 G_STAND，赛段永不报完成）。
    if _state not in _NO_WAIT_STATES and (
        (gait == 0 and mode == 0) or (gait == 1 and mode == 9) or mode == 7
    ):
        return G_STAND

    if USE_KICK_FALLBACK:
        return _kick_fallback_control(x, y, rpy)

    sweep_dist, sweep_hdg = _sweep_params()

    # ── A：转90°贴右墙上行到顶墙（坐标到达 或 撞墙停滞）──
    if _state == _ST_A_GO_TOP:
        if y >= TOP_Y or _is_stalled(y, "A"):
            _set_state(_ST_B_GO_CORNER)
            return G_STAND
        return _walk(rpy, HDG_UP, G_NAV)

    # ── B：转180°贴顶墙左行到左上角（坐标到达 或 撞墙停滞，两墙自定位）──
    elif _state == _ST_B_GO_CORNER:
        if x <= CORNER_X or _is_stalled(x, "B"):
            _set_state(_ST_C_AIM_SWEEP)
            return G_STAND
        return _walk(rpy, HDG_LEFT, G_NAV)

    # ── C：原地转头到本次尝试的顶球朝向（首顶225°，重试时换角度）──
    elif _state == _ST_C_AIM_SWEEP:
        ts = _turn_step(rpy, sweep_hdg)
        if ts != 0:
            return ts
        _set_state(_ST_D1_SWEEP)
        return G_SWEEP_L

    # ── D1：低重心左横移把球扫出角落，里程位移≥本次阈值 → 转 D2 ──
    elif _state == _ST_D1_SWEEP:
        if _sweep_x0 is None:               # 进 D1 首帧记起点
            _sweep_x0, _sweep_y0 = x, y
        if _dist(x, y, _sweep_x0, _sweep_y0) >= sweep_dist:
            _sweep_x0 = None                # 清起点，留给 D2 重记
            _sweep_y0 = None
            _set_state(_ST_D2_CLEAR)
            return G_SWEEP_R
        ts = _turn_step(rpy, sweep_hdg)     # 漂移先转回顶球朝向再横移
        if ts != 0:
            return ts
        return G_SWEEP_L

    # ── D2：低重心右横移退开拉间隙，里程位移≥SWEEP_R_DIST → 转 E（此时转身不碰球）──
    elif _state == _ST_D2_CLEAR:
        if _sweep_x0 is None:               # 进 D2 首帧重记起点
            _sweep_x0, _sweep_y0 = x, y
        if _dist(x, y, _sweep_x0, _sweep_y0) >= SWEEP_R_DIST:
            _set_state(_ST_E_FACE_PUSH)
            return G_STAND
        ts = _turn_step(rpy, sweep_hdg)
        if ts != 0:
            return ts
        return G_SWEEP_R

    # ── E：原地转身头朝328°（转到此朝向后球才进入视野，供 D_VERIFY 验球）──
    elif _state == _ST_E_FACE_PUSH:
        ts = _turn_step(rpy, HDG_PUSH)
        if ts != 0:
            return ts
        _verify_hit = 0
        _verify_frames = 0
        _set_state(_ST_D_VERIFY)
        return G_STAND

    # ── D_VERIFY：连续取帧判断球是否已离开角落 ──
    elif _state == _ST_D_VERIFY:
        found, u_offset, _r = find_ball(frame)
        if frame is None:
            _set_state(_ST_F_DRIBBLE)   # 相机故障 → 降级放行，走原纯里程计流程
            return G_PUSH
        _verify_frames += 1
        if found:
            _verify_hit += 1
        if _verify_frames >= VERIFY_WINDOW and _verify_hit >= VERIFY_HITS:
            _set_state(_ST_F_DRIBBLE)
            # 用横向像素偏移微调一次朝向，让球更居中；已居中则直接推
            if u_offset > 0:
                return G_TURN_R
            elif u_offset < 0:
                return G_TURN_L
            return G_PUSH
        if _verify_frames >= VERIFY_WINDOW or _verify_frames >= VERIFY_MAX_FRAMES:
            if _attempt + 1 >= MAX_ATTEMPT:
                _set_state(_ST_K_AIM)       # 尝试用尽 → 自动切踢球退路
            else:
                _set_state(_ST_R_RETURN)
            return G_STAND
        return G_STAND

    # ── R_RETURN：验球失败，退回角落重顶。不按坐标走回（坐标不可信），而是把状态
    #    置回 A_GO_TOP 复用「贴墙走到走不动为止」机制，每次重试都收敛到同一角落。──
    elif _state == _ST_R_RETURN:
        _attempt += 1
        _verify_hit = 0
        _verify_frames = 0
        _sweep_x0 = None
        _sweep_y0 = None
        _set_state(_ST_A_GO_TOP)
        return G_STAND

    # ── F：锁328°低重心前推，带球到缺口前 ──
    elif _state == _ST_F_DRIBBLE:
        if x >= KICK_TRIGGER_X:
            _set_state(_ST_G_THROUGH_GAP)
            return G_PUSH
        return _walk(rpy, HDG_PUSH, G_PUSH)

    # ── G：保持低重心推球穿缝，狗随球进圈（不留余量，确保后脚进缺口）──
    # 用 G_PUSH(51, posZ=-0.08) 全程压住球，避免高步态(28)抬高机身致球从身下漏走。
    elif _state == _ST_G_THROUGH_GAP:
        if x >= FINISH_STOP_X:
            _set_state(_ST_TURN_FINISH)
            return G_STAND
        return _walk(rpy, HDG_PUSH, G_PUSH)

    # ── ABANDON：赛段总超时，放弃球，锁328°前推到停滞后收尾趴下 ──
    elif _state == _ST_ABANDON:
        if _is_stalled(x, "ABANDON"):
            _set_state(_ST_TURN_FINISH)
            return G_STAND
        return _walk(rpy, HDG_PUSH, G_PUSH)

    # ── TURN_FINISH：圈内原地转身，头从328°转到正对+x(0°)，对准后进趴下 ──
    elif _state == _ST_TURN_FINISH:
        ts = _turn_step(rpy, HDG_FINISH)
        if ts != 0:
            return ts
        _set_state(_ST_H_LAYDOWN)
        return G_STAND

    # ── H：圈内趴下，计3帧后完成 ──
    elif _state == _ST_H_LAYDOWN:
        _laydown_count += 1
        if _laydown_count >= 3:
            _set_state(_ST_DONE)
            return -1
        return G_LAY

    elif _state == _ST_DONE:
        return -1

    # ── 踢球退路状态（验球尝试用尽后由主线切入）──
    elif _state in (_ST_K_AIM, _ST_K_KICK):
        return _kick_fallback_control(x, y, rpy)

    return -1


def _kick_fallback_control(x, y, rpy):
    """踢球退路：角落外对准328°→快步态踢射→追球进圈→趴下。

    复用主线朝向/步态/状态骨架，去掉 C/D 环节。两种进入方式：
    1) 手动置 USE_KICK_FALLBACK=True 强制走退路；
    2) 主线验球尝试用尽（_attempt+1 >= MAX_ATTEMPT）自动切入 K_AIM。
    收尾自带 H/DONE 计数（与主线同形，防两条路径漂移）。
    """
    global _state, _laydown_count

    # 退路首帧（_state 仍是 reset 后的 _ST_A_GO_TOP）→ 切入对准态
    if _state == _ST_A_GO_TOP:
        _set_state(_ST_K_AIM)

    # ── K_AIM：原地转到328°（对准缺口）──
    if _state == _ST_K_AIM:
        ts = _turn_step(rpy, HDG_PUSH)
        if ts != 0:
            return ts
        _set_state(_ST_K_KICK)
        return G_KICK

    # ── K_KICK：快步态踢/推球过缺口，狗随球进圈（锁328°）──
    elif _state == _ST_K_KICK:
        if x >= FINISH_STOP_X:
            _set_state(_ST_TURN_FINISH)
            return G_STAND
        return _walk(rpy, HDG_PUSH, G_KICK)

    # ── TURN_FINISH：圈内原地转身头朝+x(0°)，对准后进趴下（与主线同形）──
    elif _state == _ST_TURN_FINISH:
        ts = _turn_step(rpy, HDG_FINISH)
        if ts != 0:
            return ts
        _set_state(_ST_H_LAYDOWN)
        return G_STAND

    # ── 收尾趴下：本函数自带 H/DONE 副本完成计数（主控仅贡献等待判据对 H/DONE 的豁免）──
    elif _state == _ST_H_LAYDOWN:
        _laydown_count += 1
        if _laydown_count >= 3:
            _set_state(_ST_DONE)
            return -1
        return G_LAY

    elif _state == _ST_DONE:
        return -1

    return -1
