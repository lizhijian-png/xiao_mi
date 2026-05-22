# 第一赛段「石径探路」实现说明

## 新增文件

| 文件 | 说明 |
|------|------|
| `code/segment1.py` | 第一赛段完整控制模块 |

---

## 赛段规则回顾

- 机器狗从起点趴下出发，头朝箭头方向
- 站起后沿石板路（4×4 网格，每块 30cm 宽、5cm 高，间隔 20cm）前进
- 走完石板路后完成弯道拐弯
- **结束条件**：后腿足底离开弯道虚线

---

## 实现思路

### 整体结构

`segment1_control(position, gait_mode, rpy, frame)` 是对外暴露的唯一控制接口，每个控制周期调用一次，返回应设置到 `Robot_Ctrl.num` 的步态索引。返回 `-1` 表示本赛段已完成。

内部通过 `_flags` 字典维护三阶段状态机，保证每个阶段只触发一次：

```
[阶段1 石板路行进]
        ↓ y >= STONE_PATH_END_Y
[阶段2 弯道准备与转向]
        ↓ 朝向对准且 y >= SEGMENT1_EXIT_Y
[返回 -1，赛段结束]
```

---

## 核心设计决策

### 1. 石板路专用步态 #9

`usergait.toml` step 9 的抬腿高度为 **0.12m**，远超石板高度 0.05m，能稳定跨越每块石板：

```toml
# step 9 — 高抬腿前进（石板路）
vel_des    = [0.15, 0.0, 0.0]
step_height = [0.12, 0.12]
```

普通前进步态（step 1）的 `step_height = 0.02m`，会被石板绊倒，因此整个石板路段强制使用 step 9。

### 2. 横向纠偏双重保障

石板路宽 100cm，机身约 40cm，需持续保持中心行走。

| 优先级 | 方法 | 数据来源 | 触发条件 |
|--------|------|----------|----------|
| 高 | 视觉纠偏 | 相机图像黄色边沿 | 像素偏移 > 40px |
| 低 | 位置纠偏 | `simulator_lcmt` x 坐标 | x 偏差 > 0.06m |

**视觉纠偏逻辑（`detect_yellow_line_offset`）：**

1. 截取图像下半部分（地面区域）
2. HSV 颜色过滤提取黄色边沿（H:20~35, S:100~255, V:100~255）
3. 开运算去噪后找轮廓
4. 计算左右两侧黄线轮廓中心，取其中点与图像中心的像素差

**纠偏动作映射：**

| 偏差方向 | 步态 | 说明 |
|----------|------|------|
| 偏右（offset > 40px 或 x > 0.06m） | #8（右平移） | `vel_des=[0,-0.05,0]` |
| 偏左（offset < -40px 或 x < -0.06m） | #7（左平移） | `vel_des=[0,0.05,0]` |

### 3. 弯道转向

根据赛道平面图，石板路结束后需向左转（目标朝向 90°）进入第二赛段。转向使用与 `test.py` 一致的 `_walk_90` 函数，朝向误差在 ±3° 内视为对准。

---

## 关键参数（需现场校准）

位于 `segment1.py` 第 35~46 行：

| 参数 | 默认值 | 含义 |
|------|--------|------|
| `STONE_PATH_END_Y` | `2.3` m | 石板路终点 y 坐标 |
| `TURN_COMMIT_Y` | `2.7` m | 开始执行转向的 y 坐标 |
| `SEGMENT1_EXIT_Y` | `3.6` m | 后腿离开弯道虚线 y 坐标 |
| `PATH_CENTER_X` | `0.0` m | 赛道中心线 x 坐标 |
| `LATERAL_TOLERANCE` | `0.06` m | 横向偏差容限 |
| `TURN_TARGET_HEADING` | `90` ° | 弯道目标朝向角 |

> 以上 y 坐标均基于仿真器坐标系，起点 y=0。实际比赛前需在仿真环境中跑一遍，读取各关键节点的位置数据后填入。

---

## 与主程序（test.py）的集成方式

在 `test.py` 的 `select_step_based_on_position` 中，将第一个条件块替换如下：

```python
from segment1 import segment1_control, reset_segment1

# 在 main() 初始化时调用：
reset_segment1()

# 在 select_step_based_on_position 中：
if flags["ENDING_FLAG1"] == False:
    step = segment1_control(position, gait_mode, rpy, frame=None)
    if step == -1:
        flags["ENDING_FLAG1"] = True
        return 1  # 继续前进进入第二赛段
    return step
```

若需要视觉纠偏，将 `frame=None` 替换为从 `CameraNode` 获取的最新帧。

---

## 独立测试

`segment1.py` 可作为独立脚本运行，直接测试第一赛段，无需启动整个 `test.py`：

```bash
cd code
python segment1.py
```

程序会：
1. 发布 usergait 步态文件
2. 等待4秒让控制器就绪
3. 微调朝向后开始执行赛段
4. 实时打印位置、朝向、当前步态索引
5. 第一赛段完成后自动退出

按 `Ctrl+C` 可随时中止，机器狗会切换到高阻尼趴下模式（mode=7）。

---

## 函数速查

| 函数 | 说明 |
|------|------|
| `reset_segment1()` | 重置所有阶段标志，每次比赛前调用 |
| `segment1_control(position, gait_mode, rpy, frame)` | 主控制函数，返回步态索引 |
| `detect_yellow_line_offset(frame)` | 视觉黄线检测，返回像素偏移量 |
| `_walk_90(rpy)` | 调整朝向至90°的步态选择辅助函数 |
| `_lateral_correction(x)` | 基于x坐标的横向纠偏辅助函数 |
