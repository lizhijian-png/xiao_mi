# 赛段6「撷金建功」带球—踢球—趴下 实现计划

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** 重写 `code/segment6.py`，用「对角线带球 + 倒退进角」替代失败的窄缝横移，把球带过右下缺口进终点圈并趴下。

**Architecture:** 纯函数状态机 `segment6_control(position, gait_mode, rpy, frame=None)` 每帧（~0.2s）调用一次，返回步态下标（−1=完成）。里程计主控（A→B→C→D→E→F 六阶段），视觉钩子 `detect_ball` 默认关闭，踢球退路用模块级开关 `USE_KICK_FALLBACK` 切换。无外部状态，全部模块级全局变量 + `reset_segment6()` 复位。

**Tech Stack:** Python 3.11（仿真/硬件无 pytest，逻辑用纯函数单测；运动验证靠 `test6.py` + Gazebo）。仅依赖标准库 `math`。步态由 `code/toml/usergait.toml` 提供，经 `Robot_Ctrl` 按下标发布。

**设计依据：** [2026-05-30-segment6-dribble-kick-design.md](../specs/2026-05-30-segment6-dribble-kick-design.md)（commit d160ee1）

---

## 关键既有接口（实现时直接复用，勿改签名）

- `Pos_msg.position` → `[x, y, z]`（米，统一坐标系）
- `Pos_msg.rpy[2]` → `float` 机身朝向角（度，已在接收端 ×180/π 并修正象限）
- `Gait_msg.gait_mode` → `[gait_id, mode]`
- `Robot_Ctrl.num` ← 主循环写入步态下标；`my_ctrl.msg.life_count = (life_count+1)%127` 每帧自增
- 步态切换/趴下等待判据（与赛段5一致）：`(gait==0 and mode==0) or (gait==1 and mode==9) or mode==7` → 返回 `G_STAND(0)` 等待，避免打断动作

## 步态下标（toml 实测，Robot_Ctrl 按数组下标直接取，注释标签≠下标）

| 名 | 下标 | vel_des | 用途 |
|----|----|----|----|
| G_STAND | 0 | 0,0,0 | 站立/等待 |
| G_NAV | 1 | 0.20,0,0 | 平地前进 |
| G_TURN_L / G_TURN_R | 2 / 3 | 0,0,±0.25 | 慢转 |
| G_BACK_SLOW / G_BACK | 6 / 26 | −0.10 / −0.20,0,0 | **后退**（倒退进角用） |
| G_FTURN_L / G_FTURN_R | 14 / 15 | 0,0,±0.60 | 快转 |
| G_PUSH | 43 | 0.20,0,0 | 推球主步态（低重心，标签#52） |
| G_KICK | 28 | 0.30,0,0 | 快前进（穿缝/踢球退路） |
| G_LAY | 4 | mode7 | 趴下 |

> 注：toml 内 step9/10 也是前进、step28=0.30 快前进。本计划只用上表。横移 7/8 **不用**（窄缝横移正是被废弃方案）。

## 文件结构

| 文件 | 动作 | 职责 |
|----|----|----|
| `code/segment6.py` | **重写** | 常量 + 纯函数辅助（`_norm`/`_turn_step`/`_walk`/`_arrived`）+ `detect_ball` 钩子 + `reset_segment6` + `segment6_control` 状态机 + `__main__` 独立入口 |
| `code/test6.py` | 改 | 仅更新顶部「手动摆位」注释为新起点/朝向；逻辑已通用，无需改动主体 |
| `code/test_segment6_logic.py` | **新建** | 纯函数逻辑单测（无硬件依赖，纯 Python 跑）：验证状态推进、朝向选步、到达判据、踢球退路分支 |

单一文件 `segment6.py` 承载整段逻辑（与赛段1–5一致，便于现场单文件替换）。逻辑单测独立成文件，不污染运行文件。

---

### Task 1: 常量 + 纯函数辅助 + 复位

纯逻辑、无硬件依赖，先建可单测的地基。

**Files:**
- Create: `code/segment6.py`（本任务只写文件头 + 常量 + 辅助 + reset，状态机留到 Task 2）
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
    # 已对准 328°，应返回 0（站立）
    assert s6._turn_step(328, 328) == 0
    assert s6._turn_step(328 + s6.SLOW_DEG - 1, 328) == 0


def test_turn_step_picks_direction_and_speed():
    # 朝向偏大（需右转），偏差>FAST → 快右转 15；中等 → 右转 3
    assert s6._turn_step(328 + s6.FAST_DEG + 5, 328) == s6.G_FTURN_R
    assert s6._turn_step(328 + s6.SLOW_DEG + 2, 328) == s6.G_TURN_R
    # 朝向偏小（需左转）
    assert s6._turn_step(328 - s6.FAST_DEG - 5, 328) == s6.G_FTURN_L
    assert s6._turn_step(328 - s6.SLOW_DEG - 2, 328) == s6.G_TURN_L


def test_walk_turns_first_then_advances():
    # 未对准 → 先返回转向步态；对准 → 返回给定行走步态
    assert s6._walk(300, 328, s6.G_PUSH) == s6._turn_step(300, 328)
    assert s6._walk(328, 328, s6.G_PUSH) == s6.G_PUSH


def test_arrived_within_tol():
    assert s6._arrived(0.20, 0.22) is True       # |0.02| <= XY_TOL
    assert s6._arrived(0.20, 0.40) is False
```

- [ ] **Step 2: 运行测试，确认失败**

Run: `cd code && python -m pytest test_segment6_logic.py -v` （若无 pytest：`python -c "import test_segment6_logic as t; t.test_norm_wraps_to_pm180()"`）
Expected: FAIL —「No module named segment6」或 AttributeError（函数未定义）

- [ ] **Step 3: 写 segment6.py 文件头 + 常量**

`code/segment6.py`：

```python
"""
第六赛段：撷金建功（带球 → 踢球 → 趴下）

方案：对角线带球 + 倒退进角（替代失败的窄缝横移）。
设计依据 docs/superpowers/specs/2026-05-30-segment6-dribble-kick-design.md

统一坐标系（与赛段1-5一致）：原点(0,0)在第一赛段中轴线、距左黄线0.6m、
距下黄线0.5m。x 向右为正(0°)，y 向上为正(90°)，rpy[2] 为机身朝向角(°)。

场地（已确认）：可行驶矩形 x∈[-0.10,2.90]、y∈[12.60,15.10]，边界可触碰不可越。
  足球中心(0.50,14.50) r0.10；独木桥占左上角(球左/上各50cm)；
  右下缺口 x=2.90 y∈[12.60,13.10] 通终点圈，圈心(3.15,12.85) r0.25。

核心：球卡左上角，推球接触点须在球左上侧、朝右下328°推（天然对准缺口）。
  墙角无余量原地转身 → 在球右下开阔点 TURN_PT 转好328°，再朝左上(148°)
  倒退进角(后退步态)，左/顶墙当挡块自对正，到 PRE_PUSH 起推。
"""
import math

# ── 场地几何（绝对坐标，米）──
LEFT_WALL_X, RIGHT_WALL_X = -0.10, 2.90
BOT_WALL_Y,  TOP_WALL_Y   = 12.60, 15.10
BALL_X, BALL_Y, BALL_R    = 0.50, 14.50, 0.10
FINISH_CX, FINISH_CY, FINISH_R = 3.15, 12.85, 0.25
GAP_X = 2.90

# ── 路径航点（狗机身中心目标值，绝对坐标）──
TURN_PT_X, TURN_PT_Y = 0.88, 14.27   # A终点/B起点：球右下开阔转身点（离球0.44m）
PRE_PUSH_X, PRE_PUSH_Y = 0.20, 14.69 # B终点：推球位姿，前缘正中贴球左上缘
KICK_TRIGGER_X = 2.40                # D→E：狗到此x改快速步态
FINISH_STOP_X  = FINISH_CX           # E：随球停在圈心x（不留余量，确保后脚进缺口）
XY_TOL = 0.08                        # 航点到达容差

# ── 朝向角度 ──
HDG_PUSH = 328   # 推球/带球朝向：球→终点圈（atan2(-1.65,2.65)≈-31.9°→328.1°）
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

# ── 踢球退路开关（带球入位若仿真失败，置 True 切踢射方案）──
USE_KICK_FALLBACK = False
```

- [ ] **Step 4: 写辅助函数 + reset（接在常量后）**

```python
# ── 状态机全局变量 ──
_state = None
_laydown_count = 0


def reset_segment6():
    """每次比赛/测试前重置赛段6状态。"""
    global _state, _laydown_count
    _state = _ST_A_APPROACH
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
    无需改 segment6_control 签名。
    """
    if frame is None:
        return 0.0
    return 0.0   # 占位：真实实现见 Task 3（默认仍关闭，不影响里程计主控）
```

> 注：`_ST_A_APPROACH` 等状态常量在 Task 2 定义；`reset_segment6` 引用它，
> 故 Task 2 完成前 import segment6 会因 `_ST_A_APPROACH` 未定义而报错——
> 本任务测试只调用 `_norm/_turn_step/_walk/_arrived`，**先把状态常量占位定义**
> 在文件顶部（紧接常量区）加一行 `_ST_A_APPROACH = "A_APPROACH"` 避免 import 失败。

- [ ] **Step 5: 运行测试，确认通过**

Run: `cd code && python -m pytest test_segment6_logic.py -v`
Expected: 5 passed（norm/turn_step×2/walk/arrived）

- [ ] **Step 6: 提交**

```bash
git add code/segment6.py code/test_segment6_logic.py
git commit -m "feat(segment6): 常量+纯函数辅助(转向/行走/到达判据)+复位"
```

---

### Task 2: 状态机主体（A→B→C→D→E→F）

带球主线。先写逻辑单测覆盖「状态推进 + 倒退进角分支」，再实现 `segment6_control`。

**Files:**
- Modify: `code/segment6.py`（追加状态常量 + `segment6_control`）
- Modify: `code/test_segment6_logic.py`（追加状态推进测试）

- [ ] **Step 1: 追加失败测试（状态推进序列）**

`code/test_segment6_logic.py` 末尾追加：

```python
def _drive(pos, yaw, gait_mode=(11, 0)):
    """跑一帧，返回 (步态, 推进后的状态名)。"""
    step = s6.segment6_control(list(pos), list(gait_mode), yaw)
    return step, s6._state


def test_A_navigates_then_turns_at_turn_pt():
    s6.reset_segment6()
    # 远离转身点 → A 阶段导航前进（已对准航向时返回 G_NAV，否则转向）
    step, st = _drive((2.5, 13.5), s6.HDG_PUSH)
    assert st == s6._ST_A_APPROACH
    assert step in (s6.G_NAV, s6.G_TURN_L, s6.G_TURN_R, s6.G_FTURN_L, s6.G_FTURN_R)
    # 到达 TURN_PT 容差内 → 进入 B，先站立
    step, st = _drive((s6.TURN_PT_X, s6.TURN_PT_Y), s6.HDG_PUSH)
    assert st == s6._ST_B_BACK_IN
    assert step == s6.G_STAND


def test_B_turns_to_328_then_backs_in():
    s6.reset_segment6()
    s6._state = s6._ST_B_BACK_IN
    # 在 TURN_PT、朝向未对准 328 → 先转向
    step, st = _drive((s6.TURN_PT_X, s6.TURN_PT_Y), 270)
    assert step == s6._turn_step(270, s6.HDG_PUSH)
    assert st == s6._ST_B_BACK_IN
    # 已对准 328 但还没退到 PRE_PUSH → 后退步态
    step, st = _drive((s6.TURN_PT_X, s6.TURN_PT_Y), s6.HDG_PUSH)
    assert step == s6.G_BACK
    assert st == s6._ST_B_BACK_IN
    # 退到 PRE_PUSH → 进入 C 首推
    step, st = _drive((s6.PRE_PUSH_X, s6.PRE_PUSH_Y), s6.HDG_PUSH)
    assert st == s6._ST_C_PUSH_OUT


def test_D_to_E_kick_trigger():
    s6.reset_segment6()
    s6._state = s6._ST_D_DRIBBLE
    # 未到踢球触发 x → 继续推球
    step, st = _drive((2.0, 13.5), s6.HDG_PUSH)
    assert step == s6.G_PUSH and st == s6._ST_D_DRIBBLE
    # 到触发 x → 进入穿缝，换快步态
    step, st = _drive((s6.KICK_TRIGGER_X, 13.2), s6.HDG_PUSH)
    assert st == s6._ST_E_THROUGH_GAP and step == s6.G_KICK


def test_E_to_F_then_done():
    s6.reset_segment6()
    s6._state = s6._ST_E_THROUGH_GAP
    # 狗中心到圈心 x → 趴下阶段
    step, st = _drive((s6.FINISH_STOP_X, 12.85), s6.HDG_PUSH)
    assert st == s6._ST_F_LAYDOWN and step == s6.G_STAND
    # 趴下计数 3 帧后 DONE，返回 -1
    last = None
    for _ in range(4):
        last, st = _drive((s6.FINISH_CX, s6.FINISH_CY), s6.HDG_PUSH)
    assert last == -1 and st == s6._ST_DONE


def test_F_laydown_advances_even_when_mode7():
    # 回归保护：趴下发出后 mode 变 7，F 阶段须绕过顶部等待判据继续计数到 DONE，
    # 否则 _laydown_count 卡在 1、趴下↔站立抖动。用 gait_mode=(0,7) 模拟趴下中。
    s6.reset_segment6()
    s6._state = s6._ST_F_LAYDOWN
    last = None
    for _ in range(3):
        last, st = _drive((s6.FINISH_CX, s6.FINISH_CY), s6.HDG_PUSH, gait_mode=(0, 7))
    assert last == -1 and st == s6._ST_DONE
```

- [ ] **Step 2: 运行测试，确认失败**

Run: `cd code && python -m pytest test_segment6_logic.py -k "A_navi or B_turns or D_to_E or E_to_F" -v`
Expected: FAIL（`_ST_B_BACK_IN` 等未定义 / `segment6_control` 仍是旧逻辑）

- [ ] **Step 3: 替换状态常量（删除旧 `_ST_*`，写新六阶段）**

`segment6.py` 中把 Task 1 占位的 `_ST_A_APPROACH = "A_APPROACH"` 一行替换为完整状态集：

```python
# ── 状态机状态 ──
_ST_A_APPROACH    = "A_APPROACH"     # 导航到球右下转身点 TURN_PT
_ST_B_BACK_IN     = "B_BACK_IN"      # 转到328° → 朝左上倒退进角到 PRE_PUSH
_ST_C_PUSH_OUT    = "C_PUSH_OUT"     # 朝328°首推，把球带离墙角
_ST_D_DRIBBLE     = "D_DRIBBLE"      # 开阔区锁328°直推带球到缺口前
_ST_E_THROUGH_GAP = "E_THROUGH_GAP"  # 换快步态把球送过缺口，狗随球进圈
_ST_F_LAYDOWN     = "F_LAYDOWN"      # 圈内趴下
_ST_DONE          = "DONE"
```

- [ ] **Step 4: 实现 `segment6_control`（接在 `_arrived` 之后、`__main__` 之前）**

```python
def segment6_control(position, gait_mode, rpy, frame=None):
    """赛段6控制，每帧(~0.2s)调用。返回步态下标；-1=完成。

    position: [x,y,z] 来自 Pos_msg.position
    gait_mode: [gait_id, mode] 来自 Gait_msg.gait_mode
    rpy: float 机身朝向角(°) 来自 Pos_msg.rpy[2]
    frame: 相机帧或 None（视觉钩子，默认关闭）
    """
    global _state, _laydown_count
    x, y, _z = position
    gait, mode = gait_mode

    # 步态切换中/趴下中等待，避免重复发指令打断动作（与赛段5一致）。
    # F_LAYDOWN 例外：故意绕过本判据，让趴下帧计数确定性推进到 DONE——
    # 否则 G_LAY 发出后 mode 变 7 会被此判据锁死，计数卡在 1、且趴下↔站立抖动。
    # 一旦 F 计满返回 -1，harness 再发权威的最终趴下(4)+计时停止。
    if _state != _ST_F_LAYDOWN and (
        (gait == 0 and mode == 0) or (gait == 1 and mode == 9) or mode == 7
    ):
        return G_STAND

    _ = detect_ball(frame)   # 视觉钩子，默认 frame=None 返回 0，不影响里程计主控

    if USE_KICK_FALLBACK:
        return _kick_fallback_control(x, y, rpy)   # Task 3 定义

    # ── A：导航到球右下方开阔转身点 TURN_PT ──
    if _state == _ST_A_APPROACH:
        if _arrived(x, TURN_PT_X) and _arrived(y, TURN_PT_Y):
            _state = _ST_B_BACK_IN
            return G_STAND
        # 先对准朝 TURN_PT 的航向再前进（朝向由当前位置算）
        brg = math.degrees(math.atan2(TURN_PT_Y - y, TURN_PT_X - x))
        return _walk(rpy, brg, G_NAV)

    # ── B：转身+倒退进角（核心）──
    elif _state == _ST_B_BACK_IN:
        # 退到 PRE_PUSH（x、y 均到位）→ 起推
        if _arrived(x, PRE_PUSH_X) and _arrived(y, PRE_PUSH_Y):
            _state = _ST_C_PUSH_OUT
            return G_PUSH
        # 先把朝向转到 328°（开阔区转身，墙角不转）
        ts = _turn_step(rpy, HDG_PUSH)
        if ts != 0:
            return ts
        # 朝向已对准 → 朝左上(机身正后方)倒退进角，墙当挡块
        return G_BACK

    # ── C：朝328°首推，把球带离墙角到开阔区 ──
    elif _state == _ST_C_PUSH_OUT:
        # 推到狗越过球初始 x 一段（球已离角）→ 转入开阔带球
        if x >= BALL_X + 0.30:
            _state = _ST_D_DRIBBLE
            return G_PUSH
        return _walk(rpy, HDG_PUSH, G_PUSH)

    # ── D：开阔区锁328°直推，带球到缺口前 ──
    elif _state == _ST_D_DRIBBLE:
        if x >= KICK_TRIGGER_X:
            _state = _ST_E_THROUGH_GAP
            return G_KICK
        return _walk(rpy, HDG_PUSH, G_PUSH)

    # ── E：换快步态把球送过缺口，狗随球进圈（不留余量，确保后脚进缺口）──
    elif _state == _ST_E_THROUGH_GAP:
        if x >= FINISH_STOP_X:
            _state = _ST_F_LAYDOWN
            return G_STAND
        return _walk(rpy, HDG_PUSH, G_KICK)

    # ── F：圈内趴下，计 3 帧后完成 ──
    elif _state == _ST_F_LAYDOWN:
        _laydown_count += 1
        if _laydown_count >= 3:
            _state = _ST_DONE
        return G_LAY

    elif _state == _ST_DONE:
        return -1

    return -1
```

> C 阶段退出判据用 `x >= BALL_X + 0.30`（狗推着球沿328°前进、x 单调增），
> 是「球已脱离墙角」的里程计代理；现场可微调该 0.30 偏移。

- [ ] **Step 5: 运行测试，确认通过**

Run: `cd code && python -m pytest test_segment6_logic.py -v`
Expected: 10 passed（Task1 的 5 + 本任务 5）

- [ ] **Step 6: 提交**

```bash
git add code/segment6.py code/test_segment6_logic.py
git commit -m "feat(segment6): 六阶段状态机(导航/倒退进角/推球/穿缝/趴下)"
```

---

### Task 3: 踢球退路 + 视觉钩子真实实现

带球若仿真反复蹭球/塞不进墙角，置 `USE_KICK_FALLBACK=True` 切到「墙角外对准缺口
→ 快步态踢射 → 追球进圈 → 趴下」。复用同一套朝向/步态/状态骨架，去掉 B/C/D 推球循环。
视觉钩子 `detect_ball` 给出可选真实实现（默认仍关闭，不接则纯里程计）。

**Files:**
- Modify: `code/segment6.py`（追加 `_kick_fallback_control` + 踢球退路状态常量；补全 `detect_ball` 真实实现注释）
- Modify: `code/test_segment6_logic.py`（追加踢球退路分支测试）

- [ ] **Step 1: 追加失败测试（踢球退路状态推进）**

`code/test_segment6_logic.py` 末尾追加：

```python
def test_kick_fallback_aims_then_kicks():
    s6.reset_segment6()
    s6.USE_KICK_FALLBACK = True
    try:
        # 退路起始：未对准 328° → 先转向
        step = s6.segment6_control([0.9, 14.2, 0.0], [11, 0], 270)
        assert step == s6._turn_step(270, s6.HDG_PUSH)
        assert s6._state == s6._ST_K_AIM
        # 已对准 328° → 进入踢射，发快步态
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
        # 未到圈心 x → 继续追球快步态
        step = s6.segment6_control([2.0, 13.4, 0.0], [11, 0], s6.HDG_PUSH)
        assert step == s6.G_KICK and s6._state == s6._ST_K_KICK
        # 到圈心 x → 进入趴下
        step = s6.segment6_control([s6.FINISH_STOP_X, 12.85, 0.0], [11, 0], s6.HDG_PUSH)
        assert step == s6.G_STAND and s6._state == s6._ST_F_LAYDOWN
    finally:
        s6.USE_KICK_FALLBACK = False
        s6.reset_segment6()
```

- [ ] **Step 2: 运行测试，确认失败**

Run: `cd code && python -m pytest test_segment6_logic.py -k "kick_fallback" -v`
Expected: FAIL（`_ST_K_AIM`/`_ST_K_KICK` 未定义 / `_kick_fallback_control` 未实现）

- [ ] **Step 3: 追加踢球退路状态常量（接在六阶段状态常量后）**

`segment6.py` 状态常量区追加两个退路态（复用 `_ST_F_LAYDOWN`/`_ST_DONE` 收尾）：

```python
# ── 踢球退路状态（USE_KICK_FALLBACK=True 时启用）──
_ST_K_AIM  = "K_AIM"    # 墙角外对准缺口方向 328°
_ST_K_KICK = "K_KICK"   # 快步态把球踢/推过缺口，狗随球进圈
```

> 退路也复用主线的 `_ST_F_LAYDOWN`/`_ST_DONE`：踢射后狗追进圈心 x 即转 `_ST_F_LAYDOWN`，
> 与主线趴下逻辑完全共用，避免重复实现收尾。`reset_segment6` 不变（退路从 `_ST_A_APPROACH`
> 起步，首帧由 `_kick_fallback_control` 接管并自行切到 `_ST_K_AIM`）。

- [ ] **Step 4: 实现 `_kick_fallback_control`（接在 `segment6_control` 之后）**

```python
def _kick_fallback_control(x, y, rpy):
    """踢球退路：墙角外对准328°→快步态踢射→追球进圈→趴下。

    复用主线朝向/步态/状态骨架，去掉 B/C/D 推球循环。
    带球入位若仿真反复蹭球/塞不进墙角时，置 USE_KICK_FALLBACK=True 启用。
    """
    global _state, _laydown_count

    # 退路首帧（_state 仍是 reset 后的 _ST_A_APPROACH）→ 切入对准态
    if _state == _ST_A_APPROACH:
        _state = _ST_K_AIM

    # ── K_AIM：在当前开阔位原地转到328°（对准缺口）──
    if _state == _ST_K_AIM:
        ts = _turn_step(rpy, HDG_PUSH)
        if ts != 0:
            return ts
        _state = _ST_K_KICK
        return G_KICK

    # ── K_KICK：快步态踢/推球过缺口，狗随球进圈（锁328°）──
    elif _state == _ST_K_KICK:
        if x >= FINISH_STOP_X:
            _state = _ST_F_LAYDOWN
            return G_STAND
        return _walk(rpy, HDG_PUSH, G_KICK)

    # ── 收尾复用主线趴下（F/DONE 在 segment6_control 内处理）──
    elif _state == _ST_F_LAYDOWN:
        _laydown_count += 1
        if _laydown_count >= 3:
            _state = _ST_DONE
        return G_LAY

    elif _state == _ST_DONE:
        return -1

    return -1
```

> 退路与主线在 `segment6_control` 顶部的等待判据、`detect_ball` 钩子之后分流：
> `if USE_KICK_FALLBACK: return _kick_fallback_control(x, y, rpy)`（Task 2 已写）。
> 故退路同样享受「步态切换中/趴下中返回 G_STAND 等待」的保护，无需重复。

- [ ] **Step 5: （可选）补全 `detect_ball` 真实实现**

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
    # 足球颜色阈值现场标定（此处示意，需按实际球色调整）
    mask = cv2.inRange(hsv, np.array([0, 0, 200]), np.array([180, 40, 255]))
    cnts, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    if not cnts:
        return 0.0
    (u, _v), _r = cv2.minEnclosingCircle(max(cnts, key=cv2.contourArea))
    return float(u - frame.shape[1] / 2.0)   # 正=球偏右
```

- [ ] **Step 6: 运行测试，确认通过**

Run: `cd code && python -m pytest test_segment6_logic.py -v`
Expected: 12 passed（Task1 的 5 + Task2 的 5 + 本任务 2）

- [ ] **Step 7: 提交**

```bash
git add code/segment6.py code/test_segment6_logic.py
git commit -m "feat(segment6): 踢球退路(_kick_fallback_control)+视觉钩子真实实现"
```

---

### Task 4: test6.py 入口对齐 + 分阶段仿真验证

`test6.py` 主体已通用（调 `segment6_control(position, gait_mode, rpy[2])`，签名兼容 `frame`
默认值），无需改动主循环。本任务只对齐起点摆位/等待时长，再按设计文档做分阶段仿真。

**Files:**
- Modify: `code/test6.py`（仅顶部摆位注释 + `step==0` 等待时长）

- [ ] **Step 1: 对齐起点摆位注释与等待时长**

`test6.py` 顶部 docstring 起点从旧值改为本方案落点（与设计文档 START 一致）：

```python
"""
test6.py — 赛段6独立测试
使用方式：
  1. 在 Gazebo 里手动把机器人拖到第六赛段入口 (2.5, 13.5)，朝向 180°
     （赛段5跳下落点，面向 -x 朝球方向；状态机 A 阶段会自行转向 TURN_PT）
  2. 运行本脚本
"""
```

并把主循环里 `num == 0` 的等待从 `time.sleep(1)` 调到 `time.sleep(4)`——
转身到 328° / 趴下都是耗时动作，需与赛段5一致给足完成时间，避免重复发指令打断：

```python
            if num == 0:
                time.sleep(4)
```

> 主循环、打印线程、趴下兜底（`num==-1` → `my_ctrl.num=4`）均无需改动。
> `print_worker` 已直接打印 `segment6._state`，分阶段验证时可直接观察状态推进。

- [ ] **Step 2: 全套逻辑单测回归**

Run: `cd code && python -m pytest test_segment6_logic.py -v`
（若无 pytest：`python -c "import test_segment6_logic as t; [getattr(t,n)() for n in dir(t) if n.startswith('test_')]; print('ALL OK')"`）
Expected: 12 passed（Task1×5 + Task2×5 + Task3×2），纯逻辑全绿后再进仿真。

- [ ] **Step 3: 分阶段仿真验证（Gazebo，按风险从高到低）**

每步先纯里程计（`USE_KICK_FALLBACK=False`、`detect_ball` 传 None）；偏了再开视觉/退路。

1. **A→B 进墙角位（最高风险，先单独验）**：摆位 (2.5,13.5)/180°，跑到狗转到 328°
   并倒退贴进 PRE_PUSH≈(0.20,14.69)。观察：倒退途中机身侧缘是否蹭球；后右角是否
   触左墙即停（墙当挡块）。若反复蹭球/塞不进 → 置 `USE_KICK_FALLBACK=True` 验退路。
2. **C 首推出角**：确认朝 328° 低重心推球(43)能把球带离左上墙角到开阔区
   （狗 `x ≥ BALL_X+0.30` 退出 C）。若球被推进角而非出角 → 接触点偏，开 `detect_ball`
   校正接触侧，或把 TURN_PT 再往球右下挪。
3. **D→E 带球穿缝进圈**：锁 328° 直推到 `x≈2.40` 换快步态(28)，把球送过缺口 x=2.90，
   狗随球到圈心 x≈3.15（零余量，确保后脚越缺口进圈）。
4. **全程串跑 + F 趴下**：(2.5,13.5)/180° 一气跑到圈内趴下，确认 `num==-1` 后兜底趴下(4)
   生效、计时停止。

- [ ] **Step 4: 提交**

```bash
git add code/test6.py
git commit -m "test(segment6): test6 起点对齐(2.5,13.5)/180° + 站立等待4s"
```

---

## 完成标准（Definition of Done）

- [ ] `python -m pytest code/test_segment6_logic.py -v` → 12 passed（或无 pytest 时全函数手跑 ALL OK）。
- [ ] `segment6.py` 无 `_ST_*` / `_kick_fallback_control` / `detect_ball` 前向引用报错（import 即过）。
- [ ] 纯里程计仿真：A→F 全程串跑能带球进圈并趴下，`segment6_control` 末帧返回 -1。
- [ ] 退路自检：`USE_KICK_FALLBACK=True` 时 K_AIM→K_KICK→F_LAYDOWN→DONE 推进正确。
- [ ] 步态下标全部命中 toml 数组下标（0/1/2/3/4/6/14/15/26/28/43），无 IndexError。
- [ ] 现场可单文件替换：`segment6.py` 仅依赖标准库 `math`（视觉分支按需 import cv2/numpy）。
