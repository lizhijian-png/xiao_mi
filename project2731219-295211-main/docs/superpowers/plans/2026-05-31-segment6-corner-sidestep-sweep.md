# 赛段6 角落转身—低重心横移顶球 实现计划

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** 把赛段6角落机动从「后退顶球」改为「转身225° + 低重心横移顶球」，并把可行驶边界收紧到黄线内侧。

**Architecture:** 修改单文件状态机 `code/segment6.py`：更新边界/航点常量、把 C/D 两阶段重写为转头到 225° + 低重心左横移（里程位移闭环退出），新增 `_dist` 纯函数与 `_sweep_x0/_sweep_y0` 状态变量；在 `code/toml/usergait.toml` 末尾新增下标 44 低重心左横移步态；同步改写 `code/test_segment6_logic.py` 的 C/D 用例。A/B、E→H、踢球退路逻辑不变。

**Tech Stack:** Python 3（标准库 `math`），pytest，TOML 步态表。

设计依据：`docs/superpowers/specs/2026-05-31-segment6-corner-sidestep-sweep-design.md`

---

## 文件结构

| 文件 | 动作 | 职责 |
|----|----|----|
| `code/segment6.py` | 改 | 边界/航点常量、C/D 重写、`HDG_SWEEP`/`SWEEP_DIST`/`G_SWEEP` 常量、`_dist` 纯函数、`_sweep_x0/_sweep_y0` 状态变量 |
| `code/toml/usergait.toml` | 改 | 末尾新增 #53（下标44）低重心左横移步态 |
| `code/test_segment6_logic.py` | 改 | `_dist` 单测、C/D 用例改写、A/B 边界数值跟随 |

每个任务自成一体，结束时单测应保持全绿（`cd code && python -m pytest test_segment6_logic.py -q`）。

> **重要约定**：本仓库步态「下标」= `usergait.toml` 中 `[[step]]` 块的数组序号（从0起），**不是** `#NN` 标签注释。新增步态块追加到文件末尾即为下标 44（当前共 44 块，下标 0–43）。

---

### Task 1: 新增低重心左横移步态（usergait.toml 下标 44）

**Files:**
- Modify: `code/toml/usergait.toml`（追加到文件末尾）

- [ ] **Step 1: 确认当前步态块数为 44**

Run: `cd code && grep -c "\[\[step\]\]" toml/usergait.toml`
Expected: `44`（即现有下标 0–43；新块将成为下标 44）

- [ ] **Step 2: 在 usergait.toml 末尾追加 #53 低重心左横移步态块**

把以下内容原样追加到 `code/toml/usergait.toml` 文件最末（最后一个 `duration = 0` 之后另起一空行）：

```toml

#53 赛段6角落横移专用：低重心左横移（重心降0.08，侧速+0.08，把球顶出角落）
[[step]]
mode = 11
gait_id = 27
contact = 15
life_count = 0
vel_des = [ 0.0, 0.08, 0.0,]
rpy_des = [ 0.0, 0.0, 0.0,]
pos_des = [ 0.0, 0.0, -0.08,]
acc_des = [ 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,]
ctrl_point = [ 0.0, 0.0, 0.0,]
foot_pose = [ 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,]
step_height = [ 0.02, 0.02,]
value = 0
duration = 0
```

- [ ] **Step 3: 确认步态块数变为 45（新下标 44 存在）**

Run: `cd code && grep -c "\[\[step\]\]" toml/usergait.toml`
Expected: `45`

- [ ] **Step 4: 确认新块字段正确**

Run: `cd code && grep -A4 "#53" toml/usergait.toml`
Expected: 输出包含 `vel_des = [ 0.0, 0.08, 0.0,]` 与 `pos_des = [ 0.0, 0.0, -0.08,]`

- [ ] **Step 5: Commit**

```bash
git add code/toml/usergait.toml
git commit -m "feat(segment6): 新增下标44低重心左横移步态（角落顶球）"
```

---

### Task 2: 新增 `_dist` 纯函数（横移位移判据）

**Files:**
- Modify: `code/segment6.py`（在 `_arrived` 之后，约 109 行后新增）
- Test: `code/test_segment6_logic.py`（在 `test_arrived_within_tol` 之后新增）

- [ ] **Step 1: 写失败测试**

在 `code/test_segment6_logic.py` 中 `test_arrived_within_tol` 函数之后新增：

```python
def test_dist_euclidean():
    assert s6._dist(0.0, 0.0, 0.0, 0.0) == 0.0
    assert s6._dist(3.0, 0.0, 0.0, 0.0) == 3.0
    assert s6._dist(0.0, 4.0, 0.0, 0.0) == 4.0
    assert abs(s6._dist(3.0, 4.0, 0.0, 0.0) - 5.0) < 1e-9
    # 与 SWEEP_DIST 阈值同量级：走0.20m 恰好达阈
    assert abs(s6._dist(0.20, 14.50, 0.0, 14.50) - 0.20) < 1e-9
```

- [ ] **Step 2: 运行测试确认失败**

Run: `cd code && python -m pytest test_segment6_logic.py::test_dist_euclidean -v`
Expected: FAIL，报 `AttributeError: module 'segment6' has no attribute '_dist'`

- [ ] **Step 3: 实现 `_dist`**

在 `code/segment6.py` 的 `_arrived` 函数（定义于第107行 `def _arrived(cur, target):`）整段之后新增以下函数。用 `** 0.5` 算开方，**无需 import math**（当前文件无任何顶层 import）：

```python
def _dist(x, y, x0, y0):
    """两点欧氏距离，横移位移判据用。"""
    return ((x - x0) ** 2 + (y - y0) ** 2) ** 0.5
```

- [ ] **Step 4: 运行测试确认通过**

Run: `cd code && python -m pytest test_segment6_logic.py::test_dist_euclidean -v`
Expected: PASS

- [ ] **Step 5: 运行全套确认无回归**

Run: `cd code && python -m pytest test_segment6_logic.py -q`
Expected: 全绿（18 passed）

- [ ] **Step 6: Commit**

```bash
git add code/segment6.py code/test_segment6_logic.py
git commit -m "feat(segment6): 新增 _dist 欧氏距离纯函数（横移位移判据）"
```

---

### Task 3: 收紧边界/航点常量到黄线内侧

物理对象（球、终点圈、HDG_PUSH=328）坐标不变；只改可行驶矩形、缺口 x、A 退出阈值。
测试用符号引用这些常量（非硬编码数值），故本任务不破坏现有测试。

**Files:**
- Modify: `code/segment6.py:20-32`（场地几何 + 路径航点常量）

- [ ] **Step 1: 更新场地几何常量**

把 `code/segment6.py` 中（约20-25行）：

```python
LEFT_WALL_X, RIGHT_WALL_X = -0.10, 2.90
BOT_WALL_Y,  TOP_WALL_Y   = 12.60, 15.10
BALL_X, BALL_Y, BALL_R    = 0.50, 14.50, 0.10
FINISH_CX, FINISH_CY, FINISH_R = 3.15, 12.85, 0.25
GAP_X = 2.90
```

替换为：

```python
LEFT_WALL_X, RIGHT_WALL_X = 0.0, 2.8     # 黄线内侧实际边界（原 -0.10/2.90）
BOT_WALL_Y,  TOP_WALL_Y   = 12.7, 15.0   # 黄线内侧实际边界（原 12.60/15.10）
BALL_X, BALL_Y, BALL_R    = 0.50, 14.50, 0.10   # 物理位置不变
FINISH_CX, FINISH_CY, FINISH_R = 3.15, 12.85, 0.25   # 物理位置不变（矩形外）
GAP_X = 2.80   # 缺口随右边界收紧（原 2.90）
```

- [ ] **Step 2: 更新 A 退出阈值 TOP_Y**

把 `code/segment6.py` 中（约28行）：

```python
TOP_Y        = 14.85   # A 退出：狗中心到此贴顶墙（继续贴墙后中心≈14.95、下缘≈14.80>球顶14.60）
```

替换为：

```python
TOP_Y        = 14.80   # A 退出：贴顶墙（顶墙15.0，继续贴墙后中心≈14.85、下缘≈14.70>球顶14.60）
```

> `CORNER_X=0.20`、`KICK_TRIGGER_X=2.40`、`FINISH_STOP_X=FINISH_CX` 不变。

- [ ] **Step 3: 运行全套确认无回归**

Run: `cd code && python -m pytest test_segment6_logic.py -q`
Expected: 全绿（18 passed）

- [ ] **Step 4: Commit**

```bash
git add code/segment6.py
git commit -m "feat(segment6): 边界收紧到黄线内侧 x[0.0,2.8]/y[12.7,15.0]"
```

---

### Task 4: 重写 C/D 为转身225°+低重心横移顶球

把 C/D 从「转头148° + 后退顶球」改为「转头225° + 低重心左横移」，里程位移闭环退出。
本任务用 TDD：先改 C/D 测试表达新行为（失败），再改实现使其通过。

**Files:**
- Modify: `code/test_segment6_logic.py`（改 `test_B_go_corner...` 第66行断言 + 重写 `test_C_...` / `test_D_...`）
- Modify: `code/segment6.py`（常量、状态名、reset、C/D 控制块）

- [ ] **Step 1: 改测试表达新 C/D 行为**

在 `code/test_segment6_logic.py` 中，把 `test_B_go_corner_then_C_at_corner` 末尾（约66行）的
`assert st == s6._ST_C_AIM_TAIL and step == s6.G_STAND` 改为：

```python
    assert st == s6._ST_C_AIM_SWEEP and step == s6.G_STAND
```

把整个 `test_C_aim_tail_turns_to_148_then_D` 函数替换为：

```python
def test_C_aim_sweep_turns_to_225_then_D():
    s6.reset_segment6()
    s6._state = s6._ST_C_AIM_SWEEP
    # 未对准225° → 先转向，状态留在 C
    step, st = _drive((0.20, 14.95), 180)
    assert step == s6._turn_step(180, s6.HDG_SWEEP) and st == s6._ST_C_AIM_SWEEP
    # 对准225° → 进 D，首发低重心横移步态
    step, st = _drive((0.20, 14.95), s6.HDG_SWEEP)
    assert st == s6._ST_D_SWEEP and step == s6.G_SWEEP
```

把整个 `test_D_nudge_backs_then_E` 函数替换为：

```python
def test_D_sweep_moves_then_E():
    s6.reset_segment6()
    s6._state = s6._ST_D_SWEEP
    # 进 D 首帧记起点(0.20,14.95)，位移0 < SWEEP_DIST → 持续发横移
    step, st = _drive((0.20, 14.95), s6.HDG_SWEEP)
    assert step == s6.G_SWEEP and st == s6._ST_D_SWEEP
    assert s6._sweep_x0 == 0.20 and s6._sweep_y0 == 14.95
    # 位移仍不足 → 继续横移
    step, st = _drive((0.28, 14.85), s6.HDG_SWEEP)
    assert step == s6.G_SWEEP and st == s6._ST_D_SWEEP
    # 位移≥SWEEP_DIST(从起点(0.20,14.95)走 hypot(0.16,0.16)=0.226m≥0.20) → 进 E，发站立
    step, st = _drive((0.36, 14.79), s6.HDG_SWEEP)
    assert st == s6._ST_E_FACE_PUSH and step == s6.G_STAND
```

同一文件中，`test_E_face_push_turns_to_328_then_F` 的前两行（约90-91行）仍引用被删除的
`HDG_HEAD_IN`。把这两行：

```python
    step, st = _drive((0.5, 14.6), s6.HDG_HEAD_IN)
    assert step == s6._turn_step(s6.HDG_HEAD_IN, s6.HDG_PUSH) and st == s6._ST_E_FACE_PUSH
```

替换为（用 `HDG_SWEEP` 作未对准328°的起始朝向，逻辑等价）：

```python
    step, st = _drive((0.5, 14.6), s6.HDG_SWEEP)
    assert step == s6._turn_step(s6.HDG_SWEEP, s6.HDG_PUSH) and st == s6._ST_E_FACE_PUSH
```

- [ ] **Step 2: 运行确认失败**

Run: `cd code && python -m pytest test_segment6_logic.py -q`
Expected: FAIL（`AttributeError: ... _ST_C_AIM_SWEEP` / `HDG_SWEEP` / `G_SWEEP` / `_sweep_x0` 未定义）

- [ ] **Step 3: 加新常量、改状态名**

在 `code/segment6.py` 朝向角度区（约38-40行），把：

```python
HDG_HEAD_IN = 148   # C 目标头朝向：扎进左上角（尾朝328°对准缺口）
```

替换为：

```python
HDG_SWEEP   = 225   # C 目标头朝向：逆时针转到左下方（左侧身体朝右下顶球）
SWEEP_DIST  = 0.20  # D 退出：横移里程位移阈值（约20cm）
```

在步态下标区（约52行 `G_PUSH = 43` 之后）新增：

```python
G_SWEEP  = 44    # 低重心左横移 vel_y+0.08、posZ-0.08（D 段顶球）
```

在状态常量区（约58、60行）把：

```python
_ST_C_AIM_TAIL    = "C_AIM_TAIL"     # 转头到148°（尾朝328°对准缺口）
```
改为
```python
_ST_C_AIM_SWEEP   = "C_AIM_SWEEP"    # 转头到225°（左侧身体朝右下对球）
```
并把：
```python
_ST_D_NUDGE       = "D_NUDGE"        # 后退步态让后体扫过球，把球沿328°顶出角落
```
改为
```python
_ST_D_SWEEP       = "D_SWEEP"        # 低重心左横移，把球顶出角落
```

> 删除现已无引用的 `NUDGE_EXIT_X` 常量（约30行）。`G_BACK` 保留（踢球退路/备选）。

- [ ] **Step 4: 加横移起点状态变量 + reset**

把 `code/segment6.py` 状态机全局变量区（约73-74行）：

```python
_state = None
_laydown_count = 0
```

替换为：

```python
_state = None
_laydown_count = 0
_sweep_x0 = None   # D 段横移起点 x（进 D 首帧记录）
_sweep_y0 = None   # D 段横移起点 y
```

把 `reset_segment6` 函数体替换为：

```python
def reset_segment6():
    """每次比赛/测试前重置赛段6状态。"""
    global _state, _laydown_count, _sweep_x0, _sweep_y0
    _state = _ST_A_GO_TOP
    _laydown_count = 0
    _sweep_x0 = None
    _sweep_y0 = None
```

- [ ] **Step 5: 改 B/C/D 控制块（继续下一步）**

见 Task 4 续。

---

### Task 4（续）: C/D 控制块改写 + 收尾验证

**Files:**
- Modify: `code/segment6.py`（B 段的状态切换目标 + C/D 控制块，约165-187行）

- [ ] **Step 6: 改 B 段切到 C 的状态名**

在 `code/segment6.py` B 段（约165-169行），把：

```python
    elif _state == _ST_B_GO_CORNER:
        if x <= CORNER_X:
            _state = _ST_C_AIM_TAIL
            return G_STAND
        return _walk(rpy, HDG_LEFT, G_NAV)
```

替换为：

```python
    elif _state == _ST_B_GO_CORNER:
        if x <= CORNER_X:
            _state = _ST_C_AIM_SWEEP
            return G_STAND
        return _walk(rpy, HDG_LEFT, G_NAV)
```

- [ ] **Step 7: 重写 C/D 控制块**

在 `code/segment6.py` 中，把整个 C 块（`_ST_C_AIM_TAIL`，约171-177行）和 D 块
（`_ST_D_NUDGE`，约179-187行）一起替换为：

```python
    # ── C：原地转头到225°（左侧身体朝右下对球）──
    elif _state == _ST_C_AIM_SWEEP:
        ts = _turn_step(rpy, HDG_SWEEP)
        if ts != 0:
            return ts
        _state = _ST_D_SWEEP
        return G_SWEEP

    # ── D：低重心左横移把球顶出角落，里程位移≥SWEEP_DIST退出 ──
    elif _state == _ST_D_SWEEP:
        global _sweep_x0, _sweep_y0
        if _sweep_x0 is None:               # 进 D 首帧记起点
            _sweep_x0, _sweep_y0 = x, y
        if _dist(x, y, _sweep_x0, _sweep_y0) >= SWEEP_DIST:
            _state = _ST_E_FACE_PUSH
            return G_STAND
        ts = _turn_step(rpy, HDG_SWEEP)     # 漂移先转回225°再横移
        if ts != 0:
            return ts
        return G_SWEEP
```

> 注：`segment6_control` 函数开头已有 `global _state, _laydown_count`，需把它扩展为
> `global _state, _laydown_count, _sweep_x0, _sweep_y0`，并删除上面 D 块内的局部
> `global _sweep_x0, _sweep_y0` 行（避免函数内重复 global 声明）。改函数首行（约139行）：
> 把 `global _state, _laydown_count` 改为 `global _state, _laydown_count, _sweep_x0, _sweep_y0`，
> 然后 D 块写成：
>
> ```python
>     elif _state == _ST_D_SWEEP:
>         if _sweep_x0 is None:
>             _sweep_x0, _sweep_y0 = x, y
>         if _dist(x, y, _sweep_x0, _sweep_y0) >= SWEEP_DIST:
>             _state = _ST_E_FACE_PUSH
>             return G_STAND
>         ts = _turn_step(rpy, HDG_SWEEP)
>         if ts != 0:
>             return ts
>         return G_SWEEP
> ```

- [ ] **Step 8: 运行 C/D 用例确认通过**

Run: `cd code && python -m pytest test_segment6_logic.py::test_C_aim_sweep_turns_to_225_then_D test_segment6_logic.py::test_D_sweep_moves_then_E -v`
Expected: 两个均 PASS

- [ ] **Step 9: 运行全套确认无回归**

Run: `cd code && python -m pytest test_segment6_logic.py -q`
Expected: 全绿（18 passed；Task4 是改名非新增，故仍 18）

- [ ] **Step 10: 确认无残留旧引用**

Run: `cd code && grep -nE "_ST_C_AIM_TAIL|_ST_D_NUDGE|HDG_HEAD_IN|NUDGE_EXIT_X" segment6.py test_segment6_logic.py`
Expected: 无输出（旧名已全部清除）

- [ ] **Step 11: import 自检（无前向引用错误）**

Run: `cd code && python -c "import segment6; segment6.reset_segment6(); print('import OK, G_SWEEP=', segment6.G_SWEEP, 'HDG_SWEEP=', segment6.HDG_SWEEP)"`
Expected: `import OK, G_SWEEP= 44 HDG_SWEEP= 225`

- [ ] **Step 12: Commit**

```bash
git add code/segment6.py code/test_segment6_logic.py
git commit -m "feat(segment6): C/D 改为转身225°+低重心横移顶球（替换后退顶球）"
```

---

### Task 5: 更新模块 docstring 与状态机注释

代码逻辑已正确；本任务只同步文件顶部 docstring 与状态注释，避免文档与实现漂移。

**Files:**
- Modify: `code/segment6.py:1-18`（顶部 docstring）、`code/segment6.py` 状态常量注释

- [ ] **Step 1: 更新顶部 docstring 的场地与核心描述**

把 `code/segment6.py` 顶部 docstring（约11-17行）中描述旧边界与「倒顶」的文字更新为
新边界与「转身横移顶球」。具体：把 `可行驶矩形 x∈[-0.10,2.90]、y∈[12.60,15.10]` 改为
`可行驶矩形 x∈[0.0,2.8]、y∈[12.7,15.0]（黄线内侧）`；把 `右下缺口 x=2.90` 改为 `右下缺口 x=2.80`；
把核心流程句中 `转尾让头朝148°…后退步态让后体扫过球把球沿328°顶出角落` 改为
`转头到225°→低重心左横移把球顶出角落`。设计依据路径改为
`docs/superpowers/specs/2026-05-31-segment6-corner-sidestep-sweep-design.md`。

- [ ] **Step 2: import 自检**

Run: `cd code && python -c "import segment6; print('OK')"`
Expected: `OK`

- [ ] **Step 3: 运行全套确认无回归**

Run: `cd code && python -m pytest test_segment6_logic.py -q`
Expected: 全绿（18 passed）

- [ ] **Step 4: Commit**

```bash
git add code/segment6.py
git commit -m "docs(segment6): 同步 docstring 到新边界与横移顶球方案"
```

---

## 完成标准（Definition of Done）

- [ ] `cd code && python -m pytest test_segment6_logic.py -q` 全绿（18 passed：原17 + Task2 新增 _dist 单测；Task4 为改名非新增）。
- [ ] `cd code && python -c "import segment6"` 无报错。
- [ ] `usergait.toml` 步态块数=45，下标44 为低重心左横移（vel_y+0.08、posZ-0.08）。
- [ ] 无残留旧名 `_ST_C_AIM_TAIL`/`_ST_D_NUDGE`/`HDG_HEAD_IN`/`NUDGE_EXIT_X`。
- [ ] 边界常量为黄线内侧值（x[0.0,2.8]、y[12.7,15.0]、GAP_X=2.80）。
- [ ] 踢球退路 `USE_KICK_FALLBACK=True` 相关用例仍全绿（未受改动影响）。
