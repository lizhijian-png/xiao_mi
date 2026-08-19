"""
第六赛段：撷金建功（角落横移顶球 → 转身前推 → 趴下）

方案：贴墙到角自定位 → 转头到225°→低重心左横移把球顶出角落 → 再转身让头朝328°前推护球进缺口。
设计依据 docs/superpowers/specs/2026-05-31-segment6-corner-sidestep-sweep-design.md

统一坐标系（与赛段1-5一致）：原点(0,0)在第一赛段中轴线、距左黄线0.6m、
距下黄线0.5m。x 向右为正(0°)，y 向上为正(90°)，rpy[2] 为机身朝向角(°)。

场地（已确认）：可行驶矩形 x∈[0.0,2.8]、y∈[12.7,15.0]（黄线内侧），本段完全平整无障碍，
  左上角(0.0,15.0)开阔。边界可触碰不可越。
  足球中心(0.50,14.50) r0.10；右下缺口 x=2.80 y∈[12.7,13.10] 通终点圈，
  圈心(3.15,12.85) r0.25。

核心：贴右墙上行→贴顶墙左行到左上角(两墙钉死x/y自定位)→转头到225°→低重心左横移把球顶出角落
  （里程位移≥0.20m）→转身让头朝328°正对球→低重心前推护球
  穿缝进圈→趴下。328°直线从球(0.5,14.5)穿缺口直达圈心(3.15,12.85)，推球段几何天然成立。

两段式控制（顶球固定 / 推球视觉）：
  A~D2（贴墙到角+横移顶球）走纯里程计固定路线——这段几何由两面墙钉死，确定性最高，
  不引入视觉以免误检破坏已验证的行为。
  E/F/G（转身找球+带球推进+穿缝）走视觉闭环——顶球力度、球的滚动方向、墙面反弹都会让
  球偏离理论落点(0.5,14.5)，锁死328°会推着空气走完全程。改为每帧用相机定球的方位角，
  把球压在画面中心推向缺口。视觉采用“固定路线前进基准+识别后实时纠偏”：球被
  顶得太远暂时看不到时仍沿328°接近，球进入视野后立即按横向偏移修正，不原地摆扫。
"""

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
BALL_EXIT_DOG_X = 2.80  # 狗中心到出口线时，位于机身前方的球已经先越过出口
EXIT_ALIGN_Y = 12.90    # 沿328°接近出口；到此高度后转0°正对出口
FINISH_XY_TOL = 0.06    # 导航到终点圆心容差；同时约束x/y，不能只看x
XY_TOL = 0.08          # 航点到达容差

# ── 朝向角度 ──
HDG_UP      = 90    # A 朝向：+y 上行
HDG_LEFT    = 180   # B 朝向：-x 左行
HDG_SWEEP   = 225   # C 目标头朝向：逆时针转到左下方（左侧身体朝右下顶球）
SWEEP_L_DIST = 0.25  # D1 退出：左扫里程位移阈值（把球扫出角落，约25cm）
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

# ── 视觉追球（E/F/G 段闭环，A~D2 顶球段仍走固定路线不受影响）──
# 总开关：置 False 立刻退回纯里程计固定 328° 路线（已验证能跑完的老行为）。
USE_VISION = True

BALL_HSV_LO = (0, 0, 200)      # 白球下界（H,S,V）；world 里球材质 diffuse=1 1 1 1
BALL_HSV_HI = (180, 40, 255)   # 白球上界
BALL_CIRCULARITY_MIN = 0.6     # 圆度下界，滤掉细长的黄线/墙缝/阴影带
BALL_MIN_R = 4                 # 允许识别被侧身顶远的小球；连续帧+圆度负责滤小噪点
BALL_MAX_R = 220               # 像素半径上界，滤掉大片高光墙面
BALL_CONFIRM_FRAMES = 2        # 连续命中才接管转向，抑制单帧反光/白线误检

# 像素偏移→转向增益。相机 horizontal_fov=84°，w=640 时几何值≈84/640=0.13°/px，
# 取其一半做阻尼，避免闭环过冲振荡。现场按实测振荡情况调。
KP_PIX_TO_DEG = 0.065
AIM_TOL_PX    = 40    # E 段对准容差：球心偏移小于此视为已对准
SERVO_TOL_PX  = 35    # F/G 段直接视觉转向死区；640宽画面约等于4.6°
SERVO_FAST_PX = 150   # 偏差很大时使用快转，避免远离路线的球追不上
# 注意实际生效门限受 _turn_step 的 SLOW_DEG(8°) 死区约束：偏移需 > SLOW_DEG/KP
# ≈123px 才真会转，70~123px 之间请求了纠偏但仍直推。这是有意的双层死区——
# 小偏差靠推进过程自然收敛，不值得为此打断步态。嫌追球迟钝就调大 KP_PIX_TO_DEG。
MAX_SERVO_DEG = 25    # 单帧朝向修正上限，防止误检把狗甩飞

# 追球目标朝向 = 球方向与缺口方向的加权混合。纯对准球只会把球推向狗的朝向，
# 不保证推向缺口；纯对准缺口又会丢球。权重偏向球（先粘住球），再掺入缺口分量。
GAP_BIAS_W = 0.35     # 缺口方向权重(0=只追球, 1=只对缺口)，现场标定
GAP_ENTRY_X, GAP_ENTRY_Y = 2.80, 12.90   # 缺口中心(x=2.80, y∈[12.7,13.10])

LOST_MAX_FRAMES = 8   # 最近目标朝向最多保留的漏检帧数（不代表允许持续前推）

# ── 踢球退路开关（倒顶若仿真失败，置 True 切踢射方案）──
USE_KICK_FALLBACK = False

# ── 状态机状态（八阶段）──
_ST_A_GO_TOP      = "A_GO_TOP"       # 转90°贴右墙上行到顶墙
_ST_B_GO_CORNER   = "B_GO_CORNER"    # 转180°贴顶墙左行到左上角（两墙自定位）
_ST_C_AIM_SWEEP   = "C_AIM_SWEEP"    # 转头到225°（左侧身体朝右下对球）
_ST_D1_SWEEP      = "D1_SWEEP"       # 低重心左横移，把球顶出角落
_ST_D2_CLEAR      = "D2_CLEAR"       # 低重心右横移，退开拉间隙便于转身不碰球
_ST_E_FACE_PUSH   = "E_FACE_PUSH"    # 原地转身头朝328°正对被顶出的球
_ST_F_DRIBBLE     = "F_DRIBBLE"      # 锁328°低重心前推带球到缺口前
_ST_G_THROUGH_GAP = "G_THROUGH_GAP"  # 低重心推球穿缝，狗随球进圈
_ST_ALIGN_EXIT    = "ALIGN_EXIT"    # 到出口高度后原地转0°
_ST_PUSH_EXIT     = "PUSH_EXIT"     # 半蹲沿+x推球至出口线
_ST_NAV_FINISH    = "NAV_FINISH"     # 球已出出口，机器狗单独导航到终点圆心
_ST_TURN_FINISH   = "TURN_FINISH"    # 圈内原地转身，头从328°转到正对+x(0°)
_ST_H_LAYDOWN     = "H_LAYDOWN"      # 圈内趴下
_ST_DONE          = "DONE"
# 踢球退路状态（USE_KICK_FALLBACK=True 时启用，收尾复用 H/DONE）
_ST_K_AIM  = "K_AIM"    # 角落外对准缺口方向 328°
_ST_K_KICK = "K_KICK"   # 快步态把球踢/推过缺口，狗随球进圈


# ── 状态机全局变量 ──
_state = None
_laydown_count = 0
_sweep_x0 = None   # D1/D2 段横移起点 x（进 D1/D2 首帧记录）
_sweep_y0 = None   # D1/D2 段横移起点 y
_lost_count = 0        # 连续丢球帧数（E/F/G 追球用）
_last_ball_hdg = None  # 最近一次看到球时的目标朝向，丢球后沿用
_ever_locked = False   # 本段是否曾经锁定过球（决定丢球退路走哪条）
_aim_coarse_done = False  # E 段是否已完成到 328° 的粗对准（之后交给视觉微调）
_ball_hit_count = 0    # 连续识别到球的帧数（视觉去抖）


def reset_segment6():
    """每次比赛/测试前重置赛段6状态。"""
    global _state, _laydown_count, _sweep_x0, _sweep_y0
    global _lost_count, _last_ball_hdg, _ever_locked, _aim_coarse_done, _ball_hit_count
    _state = _ST_A_GO_TOP
    _laydown_count = 0
    _sweep_x0 = None
    _sweep_y0 = None
    _lost_count = 0
    _last_ball_hdg = None
    _ever_locked = False
    _aim_coarse_done = False
    _ball_hit_count = 0


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


def _hdg_to(x, y, tx, ty):
    """从(x,y)指向(tx,ty)的朝向角，度，[0,360)。"""
    import math
    return math.degrees(math.atan2(ty - y, tx - x)) % 360


def _blend_hdg(hdg_a, hdg_b, w):
    """按权重 w 混合两个朝向角（w=0 取 hdg_a，w=1 取 hdg_b），走最短弧。"""
    return (hdg_a + w * _norm(hdg_b - hdg_a)) % 360


def _ball_from_candidates(candidates, frame_width):
    """从候选圆里挑出足球（纯函数，不依赖 cv2，可离线单测）。

    candidates: [(area, cx, radius), ...] 面积/圆心x/半径，均为像素
    返回 (found, u_offset, radius_px)：u_offset 正=球偏右。
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
    而狗在低重心步态下机身俯仰持续摆动；「画面里有没有一个足球大小的圆、偏左还是
    偏右」这个判断对俯仰摆动几乎免疫，且恰好就是追球所需的全部信息。

    frame=None（未传帧/相机故障）→ (False, 0.0, 0.0)，视觉模式下调用方会停推搜球。
    cv2/numpy 延迟导入：保证 frame=None 路径只依赖标准库，无相机环境也能跑单测。
    """
    if frame is None:
        return False, 0.0, 0.0
    import cv2
    import numpy as np
    h, w = frame.shape[:2]
    # 侧身顶球后球可能滚得较远，在画面中上部；从1/4高度开始搜索，而非只看下半区。
    lower = frame[h // 4:, :]
    hsv = cv2.cvtColor(lower, cv2.COLOR_BGR2HSV)
    mask = cv2.inRange(hsv, np.array(BALL_HSV_LO), np.array(BALL_HSV_HI))
    # 黑白足球的白色块会被黑色花纹切碎；闭运算把同一颗球重新连成整体轮廓。
    kernel = np.ones((9, 9), np.uint8)
    mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel, iterations=2)
    mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, np.ones((3, 3), np.uint8))
    cnts, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    candidates = []
    for c in cnts:
        (cx, _cy), r = cv2.minEnclosingCircle(c)
        candidates.append((float(cv2.contourArea(c)), float(cx), float(r)))
    return _ball_from_candidates(candidates, w)


def _track_hdg(x, y, rpy, found, u, gap_x, gap_y, bias_w):
    """追球目标朝向解算，维护丢球计数。返回 (target_hdg, in_deadband)。

    看到球：球方向 = 当前朝向 - 增益*像素偏移（球偏右 u>0 → 顺时针转 → 朝向减小），
    再按 bias_w 掺入「指向缺口」的分量，使推进既粘着球又整体朝缺口去。
    丢球：只在少量漏检帧内沿用上次锁定朝向；长期丢球由状态机停推搜索。
    in_deadband 为 True 表示球已足够居中，调用方可直接前推不必纠偏。
    """
    global _lost_count, _last_ball_hdg, _ever_locked
    if found:
        _lost_count = 0
        _ever_locked = True
        corr = KP_PIX_TO_DEG * u
        if corr > MAX_SERVO_DEG:    corr = MAX_SERVO_DEG
        elif corr < -MAX_SERVO_DEG: corr = -MAX_SERVO_DEG
        ball_hdg = (rpy - corr) % 360
        target = _blend_hdg(ball_hdg, _hdg_to(x, y, gap_x, gap_y), bias_w)
        _last_ball_hdg = target
        return target, abs(u) <= SERVO_TOL_PX
    _lost_count += 1
    if _last_ball_hdg is not None and _lost_count < LOST_MAX_FRAMES:
        return _last_ball_hdg, False      # 短暂丢球：沿用上次朝向，别乱动
    return rpy, False                     # 长期丢球：保持当前朝向，交给状态机搜索


def _confirm_ball(found):
    """连续帧视觉去抖；丢一帧立即清零，避免把旧目标继续当成新命中。"""
    global _ball_hit_count
    _ball_hit_count = _ball_hit_count + 1 if found else 0
    return found and _ball_hit_count >= BALL_CONFIRM_FRAMES


def _visual_turn_step(u, tolerance=SERVO_TOL_PX):
    """直接按球的画面位置选转向步态，不再经过角度增益和8°朝向死区。

    u>0 表示球在画面右侧，机器狗必须右转；u<0 则左转。返回0表示已居中。
    """
    if u > SERVO_FAST_PX:
        return G_FTURN_R
    if u > tolerance:
        return G_TURN_R
    if u < -SERVO_FAST_PX:
        return G_FTURN_L
    if u < -tolerance:
        return G_TURN_L
    return G_STAND


def segment6_control(position, gait_mode, rpy, frame=None):
    """赛段6控制，每帧(~0.2s)调用。返回步态下标；-1=完成。

    position: [x,y,z] 来自 Pos_msg.position（兼容 [x,y]，忽略 z）
    gait_mode: [gait_id, mode] 来自 Gait_msg.gait_mode
    rpy: float 机身朝向角(°) 来自 Pos_msg.rpy[2]
    frame: 相机帧或 None（视觉钩子，默认关闭）
    """
    global _state, _laydown_count, _sweep_x0, _sweep_y0, _aim_coarse_done
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

    # 视觉取球：仅 E/F/G 追球段需要，A~D2 顶球段走固定路线不调用（省算力也避免误检干扰）。
    # frame is None 表示压根没相机/没收到帧，和「有帧但没找到球」是两回事：
    # 前者视觉不可用，立刻走固定路线；后者才值得等几帧或沿用上次朝向。
    vision_on = (USE_VISION and frame is not None
                 and _state in (_ST_E_FACE_PUSH, _ST_F_DRIBBLE,
                                _ST_G_THROUGH_GAP, _ST_PUSH_EXIT))
    found, u_off = False, 0.0
    if vision_on:
        found, u_off, _r_px = find_ball(frame)
        found = _confirm_ball(found)

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
            _state = _ST_C_AIM_SWEEP
            return G_STAND
        return _walk(rpy, HDG_LEFT, G_NAV)

    # ── C：原地转头到225°（左侧身体朝右下对球）──
    elif _state == _ST_C_AIM_SWEEP:
        ts = _turn_step(rpy, HDG_SWEEP)
        if ts != 0:
            return ts
        _state = _ST_D1_SWEEP
        return G_SWEEP_L

    # ── D1：低重心左横移把球扫出角落，里程位移≥SWEEP_L_DIST → 转 D2 ──
    elif _state == _ST_D1_SWEEP:
        if _sweep_x0 is None:               # 进 D1 首帧记起点
            _sweep_x0, _sweep_y0 = x, y
        if _dist(x, y, _sweep_x0, _sweep_y0) >= SWEEP_L_DIST:
            _sweep_x0 = None                # 清起点，留给 D2 重记
            _sweep_y0 = None
            _state = _ST_D2_CLEAR
            return G_SWEEP_R
        ts = _turn_step(rpy, HDG_SWEEP)     # 漂移先转回225°再横移
        if ts != 0:
            return ts
        return G_SWEEP_L

    # ── D2：低重心右横移退开拉间隙，里程位移≥SWEEP_R_DIST → 转 E（此时转身不碰球）──
    elif _state == _ST_D2_CLEAR:
        if _sweep_x0 is None:               # 进 D2 首帧重记起点
            _sweep_x0, _sweep_y0 = x, y
        if _dist(x, y, _sweep_x0, _sweep_y0) >= SWEEP_R_DIST:
            _state = _ST_E_FACE_PUSH
            return G_STAND
        ts = _turn_step(rpy, HDG_SWEEP)
        if ts != 0:
            return ts
        return G_SWEEP_R

    # ── E：先粗对准328°，再用视觉微调真正对准球（球可能已不在原定路线上）──
    elif _state == _ST_E_FACE_PUSH:
        if not _aim_coarse_done:                 # 阶段1：粗对准到328°，把球带进视野
            ts = _turn_step(rpy, HDG_PUSH)
            if ts != 0:
                return ts
            _aim_coarse_done = True
            if vision_on:
                return G_STAND                   # 有帧时站稳一帧，减少转身运动模糊
            _state = _ST_F_DRIBBLE               # 暂无帧/球太远：沿原路线主动接近
            return G_PUSH
        # 阶段2：视觉微调。对准球或长期丢球都进 F（丢球时 _track_hdg 已退回328°）
        _track_hdg(x, y, rpy, found, u_off,
                   GAP_ENTRY_X, GAP_ENTRY_Y, 0.0)   # 更新锁定/丢球诊断
        if found and abs(u_off) > AIM_TOL_PX:
            return _visual_turn_step(u_off, AIM_TOL_PX)
        elif not found:
            _state = _ST_F_DRIBBLE               # 不原地找球，边走边扩大有效识别尺度
            return G_PUSH
        _state = _ST_F_DRIBBLE
        return G_PUSH

    # ── F：视觉闭环带球推进（球偏了就纠偏，把球压在画面中心朝缺口推）──
    elif _state == _ST_F_DRIBBLE:
        if y <= EXIT_ALIGN_Y:
            _state = _ST_ALIGN_EXIT
            return G_STAND
        if x >= KICK_TRIGGER_X:
            # 兼容坐标漂移：即使x先到，也继续保持328°，直到出口高度再转正。
            return _walk(rpy, HDG_PUSH, G_PUSH)
        if not USE_VISION:
            return _walk(rpy, HDG_PUSH, G_PUSH)
        target, in_dead = _track_hdg(x, y, rpy, found, u_off,
                                     GAP_ENTRY_X, GAP_ENTRY_Y, GAP_BIAS_W)
        if not found:
            return _walk(rpy, HDG_PUSH, G_PUSH)
        visual_turn = _visual_turn_step(u_off)
        if visual_turn != G_STAND:
            return visual_turn
        path_turn = _turn_step(rpy, target)
        return G_PUSH if path_turn == G_STAND else path_turn

    # ── ALIGN_EXIT：出口高度处转正，后续不再沿328°斜推 ──
    elif _state == _ST_ALIGN_EXIT:
        ts = _turn_step(rpy, HDG_FINISH)
        if ts != 0:
            return ts
        _state = _ST_PUSH_EXIT
        return G_STAND

    # ── PUSH_EXIT：半蹲低重心沿+x推球到出口线，停在出口处 ──
    elif _state == _ST_PUSH_EXIT:
        if x >= BALL_EXIT_DOG_X:
            _state = _ST_NAV_FINISH
            return G_STAND
        if USE_VISION and found:
            visual_turn = _visual_turn_step(u_off)
            if visual_turn != G_STAND:
                return visual_turn
        return G_PUSH

    # ── G：保留旧状态名兼容外部测试/退路；新主线由 PUSH_EXIT 接管 ──
    elif _state == _ST_G_THROUGH_GAP:
        if y <= EXIT_ALIGN_Y:
            _state = _ST_ALIGN_EXIT
            return G_STAND
        return G_PUSH

    # ── NAV_FINISH：足球已经越过出口；机器狗自身精确进入终点圆心 ──
    elif _state == _ST_NAV_FINISH:
        if (abs(x - FINISH_CX) <= FINISH_XY_TOL and
                abs(y - FINISH_CY) <= FINISH_XY_TOL):
            _state = _ST_TURN_FINISH
            return G_STAND
        return _walk(rpy, _hdg_to(x, y, FINISH_CX, FINISH_CY), G_NAV)

    # ── TURN_FINISH：圈内原地转身，头从328°转到正对+x(0°)，对准后进趴下 ──
    elif _state == _ST_TURN_FINISH:
        ts = _turn_step(rpy, HDG_FINISH)
        if ts != 0:
            return ts
        _state = _ST_H_LAYDOWN
        return G_STAND

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

    # ── K_KICK：快步态把球踢过出口；球出界后狗自行进终点圈 ──
    elif _state == _ST_K_KICK:
        if x >= BALL_EXIT_DOG_X:
            _state = _ST_NAV_FINISH
            return G_STAND
        return _walk(rpy, HDG_PUSH, G_KICK)

    elif _state == _ST_NAV_FINISH:
        if (abs(x - FINISH_CX) <= FINISH_XY_TOL and
                abs(y - FINISH_CY) <= FINISH_XY_TOL):
            _state = _ST_TURN_FINISH
            return G_STAND
        return _walk(rpy, _hdg_to(x, y, FINISH_CX, FINISH_CY), G_NAV)

    # ── TURN_FINISH：圈内原地转身头朝+x(0°)，对准后进趴下（与主线同形）──
    elif _state == _ST_TURN_FINISH:
        ts = _turn_step(rpy, HDG_FINISH)
        if ts != 0:
            return ts
        _state = _ST_H_LAYDOWN
        return G_STAND

    # ── 收尾趴下：本函数自带 H/DONE 副本完成计数（主控仅贡献等待判据对 H/DONE 的豁免）──
    elif _state == _ST_H_LAYDOWN:
        _laydown_count += 1
        if _laydown_count >= 3:
            _state = _ST_DONE
            return -1
        return G_LAY

    elif _state == _ST_DONE:
        return -1

    return G_STAND
