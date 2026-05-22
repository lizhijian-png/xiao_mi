# 赛段1「石径探路」集成设计文档

**日期**：2026-05-20  
**状态**：已批准  

---

## 目标

将赛段1「石径探路」完整集成到主控程序中，同时清理 `test.py` 内与 2026 赛道无关的旧代码，保留可复用的转向辅助函数和 S 弯逻辑骨架。

---

## 方案选择

采用**方案 C：精准清理 + 赛段1集成**：

- 删除 2026 赛道不涉及的 A/B 区装卸货函数及对应 flags
- 保留通用转向函数（`walk_0/90/180/270`）和 S 弯骨架
- `segment1.py` 修正坐标轴后作为独立控制模块，由 `test.py` 导入

---

## 坐标系约定

| 轴 | 方向 | 说明 |
|----|------|------|
| x  | 前进方向（+） | 机器人沿石板路行进方向 |
| y  | 横向（+为左） | 赛道宽度方向，中心线 y=0 |
| z  | 垂直 | 不用于赛段1控制 |
| rpy[2] | 机身朝向角（°） | 0°=x+方向，90°=y+方向（左转后） |

---

## 模块：`segment1.py`

### 职责

独立的赛段1控制模块，对外仅暴露两个接口：

- `segment1_control(position, gait_mode, rpy, frame=None)` → `int`
- `reset_segment1()` → `None`

### 三阶段状态机

```
┌──────────────────────────────────────────────┐
│ 阶段1：石板路行进                              │
│   进入条件：初始状态                           │
│   动作：step 9（高抬腿，step_height=0.12m）   │
│         + y 轴位置纠偏（|y|>0.06m → 步态7/8）│
│   退出条件：x >= STONE_PATH_END_X (2.0m)     │
└──────────────┬───────────────────────────────┘
               ↓
┌──────────────────────────────────────────────┐
│ 阶段2：弯道转向                                │
│   进入条件：x >= STONE_PATH_END_X            │
│   动作：_walk_90(rpy) 调整朝向至 90°          │
│   退出条件：|rpy-90| < 3° 且                  │
│             x >= SEGMENT1_EXIT_X (3.0m)      │
└──────────────┬───────────────────────────────┘
               ↓
        返回 -1（赛段完成）
```

### 关键参数（需现场校准）

| 参数 | 默认值 | 含义 |
|------|--------|------|
| `STONE_PATH_END_X` | `2.0` m | 石板路终点 x 坐标 |
| `TURN_COMMIT_X` | `2.4` m | 开始执行转向的 x 坐标 |
| `SEGMENT1_EXIT_X` | `3.0` m | 后腿离开弯道虚线的 x 坐标 |
| `PATH_CENTER_Y` | `0.0` m | 赛道中心线 y 坐标 |
| `LATERAL_TOLERANCE` | `0.06` m | 横向偏差容限 |
| `TURN_TARGET_HEADING` | `90` ° | 弯道目标朝向（左转） |

### 步态索引使用

| 步态 | 索引 | 用途 |
|------|------|------|
| 高抬腿前进 | 9 | 石板路全程（step_height=0.12m） |
| 左平移 | 7 | y > PATH_CENTER_Y + TOLERANCE 时纠偏 |
| 右平移 | 8 | y < PATH_CENTER_Y - TOLERANCE 时纠偏 |
| 普通前进 | 1 | 弯道朝向对准后穿越虚线 |
| 等待/步态切换 | 0 | gait_mode 处于切换中间态时 |

### 视觉接口（预留）

`detect_yellow_line_offset(frame)` 函数已实现，识别图像中黄色边沿的像素偏移。  
当前 `segment1_control` 传入 `frame=None` 时跳过视觉逻辑，后续视觉纠偏增强时只需传入相机帧即可，**无需修改控制函数签名**。

---

## 模块：`test.py` 清理

### 删除内容

**函数**（5个）：

| 函数 | 删除原因 |
|------|---------|
| `load_a()` | 2026 赛道无装卸货任务 |
| `unload_a()` | 同上 |
| `unload_b()` | 同上 |
| `go_right()` | 2026 赛道无斜坡+黄灯到B区路段 |
| `back_left()` | 2026 赛道无B区回返A区路段 |

**flags 条目**（共 36 个）：

- `A_LOAD_ENDING_FLAG1~5`
- `A_UNLOAD_ENDING_FLAG1~7`
- `GO_ENDING_FLAG1~6`
- `BACK_ENDING_FLAG1~6`
- `BACK_S_ENDING_FLAG1~6`
- `unload_b_flag1~3`、`load_b_flag1`
- `is_ready_load_a/b`、`is_ready_unload_a/b`

**results 条目**（4个）：

- `A_QR`、`B_QR`、`ready_load_a`、`ready_unload_a`

**identify 导入**（按需裁剪）：

- 删除 `QRcode`、`get_ready`（装卸货专用）
- 保留 `arrow`、`yellow_wait`、`yellow_light`（S弯/黄灯，2026仍有可能用到）

### 保留内容

**函数**（8个）：

| 函数 | 保留原因 |
|------|---------|
| `walk_0/90/90_fast/180/270/270_fast` | 转向逻辑通用，2026赛道复用 |
| `pass_s_and_identify_arrow()` | 2026有S弯，骨架保留，坐标值待校准 |
| `pass_s_back()` | 同上 |

**flags 条目**（保留）：

- `ENDING_FLAG1~10`（赛段切换主线）
- `S_ENDING_FLAG1~6`、`is_s_direction_ready0~3`
- `ARROW_executed`、`LIGHT_executed`、`WAIT_executed`

**results 条目**（保留）：

- `ARROW`、`yellow_light`、`ready_wait`

---

## 模块：`test.py` 集成

### 导入层变更

在现有 import 块末尾新增（删除 `QRcode, get_ready`）：

```python
# 删除：from identify import QRcode, get_ready, arrow, yellow_wait, yellow_light
# 修改为：
from identify import arrow, yellow_wait, yellow_light
from segment1 import segment1_control, reset_segment1
```

### `main()` 变更

在 `user_pub()` 调用后、`Robot_Ctrl()` 实例化前，新增一行：

```python
reset_segment1()
```

### `select_step_based_on_position` 变更

**替换**当前第一个条件块：

```python
# 旧代码（删除）：
if x < 0.6 and flags["ENDING_FLAG1"] == False:
    return walk_0(rpy)

# 新代码（替换）：
if flags["ENDING_FLAG1"] == False:
    step = segment1_control(position, gait_mode, rpy, frame=None)
    if step == -1:
        flags["ENDING_FLAG1"] = True
        return 1  # 赛段1完成时机器人朝向已为90°，直接前进进入赛段2
    return step
```

注意：`select_step_based_on_position` 函数签名**不变**，仍接收 `(position, gait_mode, rpy)`。

### 数据流

```
main() 主循环
  with data_lock:
    num = select_step_based_on_position(pos, gait_mode, rpy)
      ├─ ENDING_FLAG1=False → segment1_control() → 步态索引 或 -1
      │     阶段1: step 9 + y轴纠偏
      │     阶段2: _walk_90(rpy)
      │     完成: 返回 -1 → ENDING_FLAG1=True
      └─ ENDING_FLAG1=True  → 赛段2+ 逻辑（S弯等）
  my_ctrl.num = num
```

---

## 不在本次范围内

- 视觉纠偏（frame 参数传入相机帧）— 后续增强
- 赛段 2~6 的具体实现 — 后续迭代
- `pass_s_and_identify_arrow` / `pass_s_back` 坐标值校准 — 现场调试

---

## 变更文件汇总

| 文件 | 操作 |
|------|------|
| `code/segment1.py` | 修改：修正坐标轴（x=前进，y=横向） |
| `code/test.py` | 修改：清理旧代码 + 集成 segment1 |
