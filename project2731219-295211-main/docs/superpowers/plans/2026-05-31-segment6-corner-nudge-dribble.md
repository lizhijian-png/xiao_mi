# 赛段6「撷金建功」角落倒顶—转身前推—趴下 实现计划

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** 重写 `code/segment6.py`，用「贴墙到角自定位 → 角落倒顶顺球 → 转身前推护球」替代失败的精确摆位入位，把球带过右下缺口进终点圈并趴下。

**Architecture:** 纯函数状态机 `segment6_control(position, gait_mode, rpy, frame=None)` 每帧(~0.2s)调用一次，返回步态下标(−1=完成)。里程计主控，八阶段 A→H：贴右墙上行→贴顶墙左行到角→转尾对准→倒退顺球出角→转身→前推带球→穿缝进圈→趴下。视觉钩子 `detect_ball` 默认关闭，踢球退路用模块级开关 `USE_KICK_FALLBACK` 切换。无外部状态，模块级全局变量 + `reset_segment6()` 复位。

**Tech Stack:** Python 3.8/3.11（仿真/硬件无 pytest 时逻辑用纯函数手跑，运动验证靠 `test6.py` + Gazebo）。仅依赖标准库 `math`。步态由 `code/toml/usergait.toml` 提供，经 `Robot_Ctrl` 按下标发布。

**设计依据：** [2026-05-31-segment6-corner-nudge-dribble-design.md](../specs/2026-05-31-segment6-corner-nudge-dribble-design.md)

---

## 关键既有接口（实现时直接复用，勿改签名）

- `Pos_msg.position` → `[x, y, z]`（米，统一坐标系）
- `Pos_msg.rpy[2]` → `float` 机身朝向角(度，已在接收端 ×180/π 并修正象限)
- `Gait_msg.gait_mode` → `[gait_id, mode]`
- `Robot_Ctrl.num` ← 主循环写入步态下标；`my_ctrl.msg.life_count = (life_count+1)%127` 每帧自增
- 步态切换/趴下等待判据(与赛段5一致)：`(gait==0 and mode==0) or (gait==1 and mode==9) or mode==7` → 返回 `G_STAND(0)` 等待，避免打断动作。**H_LAYDOWN / DONE 例外**(见 Task 2)。

## 步态下标（toml 实测，Robot_Ctrl 按数组下标直接取，注释标签≠下标）

| 名 | 下标 | vel_des | 用途 |
|----|----|----|----|
| G_STAND | 0 | 0,0,0 | 站立/等待 |
| G_NAV | 1 | 0.20,0,0 | 平地前进(A/B 贴墙走) |
| G_TURN_L / G_TURN_R | 2 / 3 | 0,0,±0.25 | 慢转 |
| G_LAY | 4 | mode7 | 趴下 |
| G_BACK_SLOW / G_BACK | 6 / 26 | −0.10 / −0.20,0,0 | 后退(D 段倒退顺球) |
| G_FTURN_L / G_FTURN_R | 14 / 15 | 0,0,±0.60 | 快转 |
| G_KICK | 28 | 0.30,0,0 | 快前进(G 穿缝送球/踢球退路) |
| G_PUSH | 43 | 0.20,0,0 | 低重心推球主步态(F 带球，标签#52) |

> 横移 7/8 **不用**(窄缝横移是最早被废弃方案)。后退慢档 6 保留为 D 段微调备选，默认 26。

## 文件结构

| 文件 | 动作 | 职责 |
|----|----|----|
| `code/segment6.py` | **重写** | 常量 + 纯函数辅助(`_norm`/`_turn_step`/`_walk`/`_arrived`) + `detect_ball` 钩子 + `reset_segment6` + 八阶段 `segment6_control` + 踢球退路 + `__main__` 入口 |
| `code/test_segment6_logic.py` | **重写** | 纯函数逻辑单测(无硬件依赖)：辅助函数 + A→H 状态推进 + D 倒顶分支 + 踢球退路分支 |
| `code/test6.py` | 改 | 顶部摆位注释改为起点 (2.9,13.5)/落地朝向；逻辑主体已通用，无需改动 |

单一文件 `segment6.py` 承载整段逻辑(与赛段1–5一致，便于现场单文件替换)。逻辑单测独立成文件，不污染运行文件。

---

### Task 1: 常量 + 纯函数辅助 + 复位

纯逻辑、无硬件依赖，先建可单测的地基。

**Files:**
- Create: `code/segment6.py`（本任务只写文件头 + 常量 + 状态占位 + 辅助 + reset + detect_ball；状态机留到 Task 2）
- Create: `code/test_segment6_logic.py`

- [ ] **Step 1: 写失败测试（辅助函数行为）**

`code/test_segment6_logic.py`：

```python
"""赛段6纯逻辑单测（无硬件依赖，纯 Python 运行）。"""
import os, sys
sys.path.insert(0, os.path.dirname(__file__))
import segment6 as s6


def test_norm_wraps_to_pm180():
    assert s6._norm(190) == -170
    assert s6._norm(-190) == 170
    assert s6._norm(360) == 0
    assert s6._norm(0) == 0


def test_turn_step_aligned_returns_zero():
    assert s6._turn_step(328, 328) == 0
    assert s6._turn_step(328 + s6.SLOW_DEG - 1, 328) == 0


def test_turn_step_picks_direction_and_speed():
    # 朝向偏大(需右转)：偏差>FAST → 快右转15；中等 → 慢右转3
    assert s6._turn_step(328 + s6.FAST_DEG + 5, 328) == s6.G_FTURN_R
    assert s6._turn_step(328 + s6.SLOW_DEG + 2, 328) == s6.G_TURN_R
    # 朝向偏小(需左转)
    assert s6._turn_step(328 - s6.FAST_DEG - 5, 328) == s6.G_FTURN_L
    assert s6._turn_step(328 - s6.SLOW_DEG - 2, 328) == s6.G_TURN_L


def test_walk_turns_first_then_advances():
    assert s6._walk(300, 328, s6.G_PUSH) == s6._turn_step(300, 328)
    assert s6._walk(328, 328, s6.G_PUSH) == s6.G_PUSH


def test_arrived_within_tol():
    assert s6._arrived(0.20, 0.22) is True       # |0.02| <= XY_TOL
    assert s6._arrived(0.20, 0.40) is False
```

- [ ] **Step 2: 运行测试，确认失败**

Run: `cd code && python -m pytest test_segment6_logic.py -v`
（若无 pytest：`cd code && python -c "import test_segment6_logic as t; t.test_norm_wraps_to_pm180()"`）
Expected: FAIL —「No module named segment6」或 AttributeError（函数未定义）

- [ ] **Step 3: 写 segment6.py 文件头 + 常量**

`code/segment6.py`：

```python
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
```

- [ ] **Step 4: 写辅助函数 + reset + detect_ball（接在常量后）**

```python
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
```

- [ ] **Step 5: 运行测试，确认通过**

Run: `cd code && python -m pytest test_segment6_logic.py -v`
Expected: 5 passed（norm / turn_step×2 / walk / arrived）

- [ ] **Step 6: 提交（仅本地，勿推远程）**

```bash
cd "c:/Users/LENOVO/Desktop/xiaomi/xiao_mi"
git add project2731219-295211-main/code/segment6.py project2731219-295211-main/code/test_segment6_logic.py
git commit -m "feat(segment6): 常量+纯函数辅助(转向/行走/到达判据)+复位"
```

---

### Task 2: 八阶段状态机主体（A→H）

带球主线。先写逻辑单测覆盖「状态推进 + 倒顶/转身分支」，再实现 `segment6_control`。

**Files:**
- Modify: `code/segment6.py`（替换状态占位为八阶段常量 + 追加 `segment6_control`）
- Modify: `code/test_segment6_logic.py`（追加状态推进测试）

- [ ] **Step 1: 追加失败测试（状态推进序列）**

`code/test_segment6_logic.py` 末尾追加：

```python
def _drive(pos, yaw, gait_mode=(11, 0)):
    """跑一帧，返回 (步态, 推进后的状态名)。"""
    step = s6.segment6_control(list(pos), list(gait_mode), yaw)
    return step, s6._state


def test_A_go_top_then_B_at_top_wall():
    s6.reset_segment6()
    step, st = _drive((2.5, 13.5), s6.HDG_UP)
    assert st == s6._ST_A_GO_TOP
    assert step in (s6.G_NAV, s6.G_TURN_L, s6.G_TURN_R, s6.G_FTURN_L, s6.G_FTURN_R)
    step, st = _drive((2.5, s6.TOP_Y + 0.01), s6.HDG_UP)
    assert st == s6._ST_B_GO_CORNER and step == s6.G_STAND


def test_B_go_corner_then_C_at_corner():
    s6.reset_segment6()
    s6._state = s6._ST_B_GO_CORNER
    step, st = _drive((2.0, 14.95), 90)
    assert step == s6._turn_step(90, s6.HDG_LEFT) and st == s6._ST_B_GO_CORNER
    step, st = _drive((2.0, 14.95), s6.HDG_LEFT)
    assert step == s6.G_NAV and st == s6._ST_B_GO_CORNER
    step, st = _drive((s6.CORNER_X - 0.01, 14.95), s6.HDG_LEFT)
    assert st == s6._ST_C_AIM_TAIL and step == s6.G_STAND


def test_C_aim_tail_turns_to_148_then_D():
    s6.reset_segment6()
    s6._state = s6._ST_C_AIM_TAIL
    step, st = _drive((0.15, 14.95), 180)
    assert step == s6._turn_step(180, s6.HDG_HEAD_IN) and st == s6._ST_C_AIM_TAIL
    step, st = _drive((0.15, 14.95), s6.HDG_HEAD_IN)
    assert st == s6._ST_D_NUDGE and step == s6.G_BACK


def test_D_nudge_backs_then_E():
    s6.reset_segment6()
    s6._state = s6._ST_D_NUDGE
    step, st = _drive((0.20, 14.90), s6.HDG_HEAD_IN)
    assert step == s6.G_BACK and st == s6._ST_D_NUDGE
    step, st = _drive((s6.NUDGE_EXIT_X + 0.01, 14.80), s6.HDG_HEAD_IN)
    assert st == s6._ST_E_FACE_PUSH and step == s6.G_STAND


def test_E_face_push_turns_to_328_then_F():
    s6.reset_segment6()
    s6._state = s6._ST_E_FACE_PUSH
    step, st = _drive((0.5, 14.6), s6.HDG_HEAD_IN)
    assert step == s6._turn_step(s6.HDG_HEAD_IN, s6.HDG_PUSH) and st == s6._ST_E_FACE_PUSH
    step, st = _drive((0.5, 14.6), s6.HDG_PUSH)
    assert st == s6._ST_F_DRIBBLE and step == s6.G_PUSH


def test_F_dribble_then_G_kick_trigger():
    s6.reset_segment6()
    s6._state = s6._ST_F_DRIBBLE
    step, st = _drive((2.0, 13.5), s6.HDG_PUSH)
    assert step == s6.G_PUSH and st == s6._ST_F_DRIBBLE
    step, st = _drive((s6.KICK_TRIGGER_X, 13.2), s6.HDG_PUSH)
    assert st == s6._ST_G_THROUGH_GAP and step == s6.G_KICK


def test_G_through_gap_then_H_then_done():
    s6.reset_segment6()
    s6._state = s6._ST_G_THROUGH_GAP
    step, st = _drive((s6.FINISH_STOP_X, 12.85), s6.HDG_PUSH)
    assert st == s6._ST_H_LAYDOWN and step == s6.G_STAND
    last = None
    for _ in range(4):
        last, st = _drive((s6.FINISH_CX, s6.FINISH_CY), s6.HDG_PUSH)
    assert last == -1 and st == s6._ST_DONE


def test_H_laydown_advances_even_when_mode7():
    # 回归保护：趴下发出后 mode 变7，H 须绕过顶部等待判据继续计数到 DONE。
    s6.reset_segment6()
    s6._state = s6._ST_H_LAYDOWN
    last = None
    for _ in range(3):
        last, st = _drive((s6.FINISH_CX, s6.FINISH_CY), s6.HDG_PUSH, gait_mode=(0, 7))
    assert last == -1 and st == s6._ST_DONE
```

- [ ] **Step 2: 运行测试，确认失败**

Run: `cd code && python -m pytest test_segment6_logic.py -k "go_top or go_corner or aim_tail or nudge or face_push or dribble or through_gap or laydown" -v`
Expected: FAIL（`_ST_B_GO_CORNER` 等未定义 / `segment6_control` 未实现）

- [ ] **Step 3: 替换状态占位为完整八阶段常量**

`segment6.py` 中把 Task 1 占位的 `_ST_A_GO_TOP = "A_GO_TOP"` 一行替换为完整状态集：

```python
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
```

> `reset_segment6` 仍设 `_state = _ST_A_GO_TOP`（名字不变，无需改 reset）。

- [ ] **Step 4: 实现 `segment6_control`（接在 `detect_ball` 之后、`__main__` 之前）**

```python
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
        return G_LAY

    elif _state == _ST_DONE:
        return -1

    return -1
```

- [ ] **Step 5: 运行测试，确认通过**

Run: `cd code && python -m pytest test_segment6_logic.py -v`
Expected: 13 passed（Task1 的 5 + 本任务 8）

- [ ] **Step 6: 提交（仅本地，勿推远程）**

```bash
cd "c:/Users/LENOVO/Desktop/xiaomi/xiao_mi"
git add project2731219-295211-main/code/segment6.py project2731219-295211-main/code/test_segment6_logic.py
git commit -m "feat(segment6): 八阶段状态机(贴墙到角/倒顶顺球/转身前推/穿缝/趴下)"
```

---

### Task 3: 踢球退路 + 视觉钩子真实实现

倒顶若仿真反复顶不到球/顶歪，置 `USE_KICK_FALLBACK=True` 切到「角落外对准缺口 → 快步态踢射 → 追球进圈 → 趴下」。复用同一套朝向/步态/状态骨架，去掉 C/D 倒退环节。视觉钩子 `detect_ball` 给出可选真实实现（默认仍关闭，不接则纯里程计）。

**Files:**
- Modify: `code/segment6.py`（追加 `_kick_fallback_control`；补全 `detect_ball` 真实实现）
- Modify: `code/test_segment6_logic.py`（追加踢球退路分支测试）

> 注：踢球退路状态常量 `_ST_K_AIM` / `_ST_K_KICK` 已在 Task 2 Step 3 一并定义，本任务不再重复。

- [ ] **Step 1: 追加失败测试（踢球退路状态推进）**

`code/test_segment6_logic.py` 末尾追加：

```python
def test_kick_fallback_aims_then_kicks():
    s6.reset_segment6()
    s6.USE_KICK_FALLBACK = True
    try:
        # 退路首帧：未对准328° → 先转向，状态切到 K_AIM
        step = s6.segment6_control([0.9, 14.2, 0.0], [11, 0], 270)
        assert step == s6._turn_step(270, s6.HDG_PUSH)
        assert s6._state == s6._ST_K_AIM
        # 已对准328° → 进入踢射，发快步态
        step = s6.segment6_control([0.9, 14.2, 0.0], [11, 0], s6.HDG_PUSH)
        assert step == s6.G_KICK and s6._state == s6._ST_K_KICK
    finally:
        s6.USE_KICK_FALLBACK = False
        s6.reset_segment6()


def test_kick_fallback_chases_into_circle_then_lays():
    s6.reset_segment6()
    s6.USE_KICK_FALLBACK = True
    try:
        s6._state = s6._ST_K_KICK
        # 未到圈心x → 继续追球快步态
        step = s6.segment6_control([2.0, 13.4, 0.0], [11, 0], s6.HDG_PUSH)
        assert step == s6.G_KICK and s6._state == s6._ST_K_KICK
        # 到圈心x → 进入趴下
        step = s6.segment6_control([s6.FINISH_STOP_X, 12.85, 0.0], [11, 0], s6.HDG_PUSH)
        assert step == s6.G_STAND and s6._state == s6._ST_H_LAYDOWN
    finally:
        s6.USE_KICK_FALLBACK = False
        s6.reset_segment6()
```

- [ ] **Step 2: 运行测试，确认失败**

Run: `cd code && python -m pytest test_segment6_logic.py -k "kick_fallback" -v`
Expected: FAIL（`_kick_fallback_control` 未实现）

- [ ] **Step 3: 实现 `_kick_fallback_control`（接在 `segment6_control` 之后）**

```python
def _kick_fallback_control(x, y, rpy):
    """踢球退路：角落外对准328°→快步态踢射→追球进圈→趴下。

    复用主线朝向/步态/状态骨架，去掉 C/D 倒退环节。
    倒顶若仿真反复顶不到球/顶歪时，置 USE_KICK_FALLBACK=True 启用。
    收尾复用主线 _ST_H_LAYDOWN / _ST_DONE。
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

    # ── 收尾复用主线趴下（H/DONE 也由 segment6_control 处理）──
    elif _state == _ST_H_LAYDOWN:
        _laydown_count += 1
        if _laydown_count >= 3:
            _state = _ST_DONE
        return G_LAY

    elif _state == _ST_DONE:
        return -1

    return -1
```

> 退路与主线在 `segment6_control` 顶部等待判据、`detect_ball` 钩子之后分流
> （`if USE_KICK_FALLBACK: return _kick_fallback_control(...)`，Task 2 已写），
> 故退路同样享受「步态切换中/趴下中返回 G_STAND 等待」的保护，无需重复。

- [ ] **Step 4: （可选）补全 `detect_ball` 真实实现**

默认 `frame=None` 仍返回 0.0（纯里程计）。现场要开视觉纠偏时，把 Task 1 的占位体
替换为下述 HSV+圆检测；**不改签名、不改主控**。未装 OpenCV 时该分支不会被触达。

```python
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
```

- [ ] **Step 5: 运行测试，确认通过**

Run: `cd code && python -m pytest test_segment6_logic.py -v`
Expected: 15 passed（Task1 的 5 + Task2 的 8 + 本任务 2）

- [ ] **Step 6: 提交（仅本地，勿推远程）**

```bash
cd "c:/Users/LENOVO/Desktop/xiaomi/xiao_mi"
git add project2731219-295211-main/code/segment6.py project2731219-295211-main/code/test_segment6_logic.py
git commit -m "feat(segment6): 踢球退路(_kick_fallback_control)+视觉钩子真实实现"
```

---

### Task 4: test6.py 入口对齐 + 分阶段仿真验证

`test6.py` 主体已通用（调 `segment6_control(position, gait_mode, rpy[2])`，签名兼容 `frame` 默认值），无需改动主循环。本任务只对齐起点摆位注释，再按设计文档做分阶段仿真。

**Files:**
- Modify: `code/test6.py`（仅顶部 docstring 摆位注释；`num==0` 已 sleep 4s，不动）

- [ ] **Step 1: 对齐起点摆位注释**

`test6.py` 顶部 docstring 起点从旧值改为本方案落点（与设计文档 START 一致）：

```python
"""
test6.py — 赛段6独立测试
使用方式：
  1. 在 Gazebo 里手动把机器人拖到第六赛段入口 (2.9, 13.5)
     （赛段5独木桥终点前50cm跳下落点；状态机 A 阶段会自行转向 90° 贴顶墙上行）
  2. 运行本脚本
"""
```

> 主循环、打印线程、趴下兜底（`num==-1` → `my_ctrl.num=4`）、`num==0` 等待 4s
> 均无需改动。`print_worker` 每帧重新 `from segment6 import _state` 打印实时状态，
> 分阶段验证时可直接观察 A→H 推进。

- [ ] **Step 2: 全套逻辑单测回归**

Run: `cd code && python -m pytest test_segment6_logic.py -v`
（若无 pytest：`cd code && python -c "import test_segment6_logic as t; [getattr(t,n)() for n in dir(t) if n.startswith('test_')]; print('ALL OK')"`）
Expected: 15 passed（Task1×5 + Task2×8 + Task3×2），纯逻辑全绿后再进仿真。

- [ ] **Step 3: 分阶段仿真验证（Gazebo，风险从高到低）**

每步先纯里程计（`USE_KICK_FALLBACK=False`、`detect_ball` 传 None）；偏了再开视觉/退路。

1. **A/B 贴墙到角自定位（先验，最稳）**：摆位 (2.9,13.5)，跑到狗转90°贴右墙上行、
   再转180°贴顶墙左行停在左上角 (x≤0.20)。观察：上行/左行途中是否从球上方 0.2m
   掠过不蹭球；到角是否被左墙挡停（墙当挡块自定位）。
2. **C/D 倒顶顺球出角（最高风险，重点单验）**：到角后头转到 148°、后退步态(26)让
   后体扫过球把球沿 328° 顶出角落（狗 x≥0.35 退出 D）。观察后体能否干净顶到球、
   顶出方向是否接近 328°。若顶不到/顶歪 → 微调 C 头朝向(148°±) / D 后退档(26→6)，
   或置 `USE_KICK_FALLBACK=True` 验退路。
3. **E/F 转身前推**：原地转身头从 148° 转到 328° 正对球，低重心推球(43)带球到 x≈2.40。
   观察转身是否蹭墙（球下方开阔区，蹭墙无妨不越界）、前推球是否沿 328° 滚向缺口。
4. **G/H 穿缝进圈趴下**：换快前进(28)把球送过缺口 x=2.90，狗随球到圈心 x≈3.15
   （零余量，确保后脚越缺口进圈），圈内趴下，确认 `num==-1` 后兜底趴下(4)生效、计时停止。
5. **全程串跑**：(2.9,13.5) 一气跑到圈内趴下。

- [ ] **Step 4: 提交（仅本地，勿推远程）**

```bash
cd "c:/Users/LENOVO/Desktop/xiaomi/xiao_mi"
git add project2731219-295211-main/code/test6.py
git commit -m "test(segment6): test6 起点摆位注释对齐 (2.9,13.5)"
```

---

## 完成标准（Definition of Done）

- [ ] `python -m pytest code/test_segment6_logic.py -v` → 15 passed（或无 pytest 时全函数手跑 ALL OK）。
- [ ] `segment6.py` 无 `_ST_*` / `_kick_fallback_control` / `detect_ball` 前向引用报错（import 即过）。
- [ ] 纯里程计仿真：A→H 全程串跑能贴墙到角、倒退顺球出角、转身护送进缺口并趴下，`segment6_control` 末帧返回 -1。
- [ ] 退路自检：`USE_KICK_FALLBACK=True` 时 K_AIM→K_KICK→H_LAYDOWN→DONE 推进正确。
- [ ] 步态下标全部命中 toml 数组下标（0/1/2/3/4/6/14/15/26/28/43），无 IndexError。
- [ ] 现场可单文件替换：`segment6.py` 仅依赖标准库 `math`（视觉分支按需 import cv2/numpy）。
