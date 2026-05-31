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
import math

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

# 状态常量占位（Task 2 替换为完整八阶段）：reset_segment6 引用 _ST_A_GO_TOP，
# 先占位定义避免 import 失败；Task 1 测试只调辅助函数，不触达状态机。
_ST_A_GO_TOP = "A_GO_TOP"


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
    """视觉钩子（默认关闭）：返回球相对机身的横向偏移（像素，正=偏右）。

    与赛段1 detect_yellow_line_offset 处理一致：主控默认传 frame=None，
    此时直接返回 0.0，控制走纯里程计。现场需要纠偏时再接入 HSV+圆检测，
    无需改 segment6_control 签名。真实实现见 Task 3（默认仍关闭）。
    """
    if frame is None:
        return 0.0
    return 0.0
