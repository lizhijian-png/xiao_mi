# 赛段4 V2 新规则设计文档

## 一、新规则说明

### 1.1 核心变化

原规则：限高杆固定在左道和右道，中道没有限高杆，中道通过小口进出。

**新规则**：
- **两个限高杆随机分布在三条道中的任意两条**
- **没有限高杆的道只能通过 y=8.6~9.1 的小口进出**
- 有限高杆的道正常通过（需要蹲姿过杆）

### 1.2 三种可能情况

#### 情况 1：左道无杆 (no_bar_lane=1)
- 左道：通过小口进出（小口在左道右侧，即中道左侧）
- 中道：有限高杆，需要蹲姿通过
- 右道：有限高杆，需要蹲姿通过
- 小口位置固定在中道左侧 (x ≈ 0.35)

#### 情况 2：中道无杆 (no_bar_lane=2) 
- 左道：有限高杆，需要蹲姿通过
- 中道：通过小口进出
- 右道：有限高杆，需要蹲姿通过
- 小口位置由 mid_open 参数决定：
  - mid_open=0：小口在中道左侧 (x ≈ 0.35)
  - mid_open=1：小口在中道右侧 (x ≈ 1.65)
- **这是原规则的情况**

#### 情况 3：右道无杆 (no_bar_lane=3)
- 左道：有限高杆，需要蹲姿通过
- 中道：有限高杆，需要蹲姿通过  
- 右道：通过小口进出（小口在右道左侧，即中道右侧）
- 小口位置固定在中道右侧 (x ≈ 1.65)

## 二、输入参数

### 2.1 命令行参数
```
python3 segment4_manual_v2.py <mid_open> <cola_pos> <football_pos> <orange_pos> <no_bar_lane>
```

5 个整数：
- `mid_open`: 0 或 1，仅当 no_bar_lane=2 时有意义
  - 0 = 中道小口在左侧
  - 1 = 中道小口在右侧
- `cola_pos`: 1、2 或 3（左、中、右）
- `football_pos`: 1、2 或 3
- `orange_pos`: 1、2 或 3  
- `no_bar_lane`: **新增参数**，1、2 或 3，表示哪条道没有限高杆

### 2.2 参数示例

**例1**：`python3 segment4_manual_v2.py 0 1 2 3 1`
- 左道无杆，小口在中道左侧
- 可乐在左道，足球在中道，橙球在右道
- 访问顺序：左道（小口）→ 中道（有杆）→ 右道（有杆）

**例2**：`python3 segment4_manual_v2.py 0 1 2 3 2`  
- 中道无杆，小口在中道左侧
- 可乐在左道，足球在中道，橙球在右道
- 访问顺序：左道（有杆）→ 中道（小口，从左侧进出）→ 右道（有杆）

**例3**：`python3 segment4_manual_v2.py 1 1 2 3 3`
- 右道无杆，小口在中道右侧  
- 可乐在左道，足球在中道，橙球在右道
- 访问顺序：左道（有杆）→ 中道（有杆）→ 右道（小口）

## 三、访问顺序逻辑

### 3.1 基本原则
- 必须从底部横向通道 (y=7.20) 依次访问三条道
- 无杆道通过小口 (y=8.85) 进出，有杆道正常通过
- 访问顺序优化：尽量减少横向移动距离

### 3.2 访问顺序决策表

| no_bar_lane | 小口位置 | 建议访问顺序 | 理由 |
|-------------|---------|-------------|------|
| 1 (左道无杆) | 中道左侧 | L → M → R | 左道通过小口进中道最近 |
| 2 (中道无杆) | mid_open=0 (左) | L → M → R | 从左道进中道小口 |
| 2 (中道无杆) | mid_open=1 (右) | L → R → M | 从右道进中道小口 |  
| 3 (右道无杆) | 中道右侧 | L → M → R | 中道通过小口进右道最近 |

## 四、关键实现修改点

### 4.1 参数解析函数修改
```python
def parse_preset_args(argv):
    """解析 5 个整数参数"""
    if len(argv) != 5:
        raise ValueError("需要 5 个整数参数")
    
    mid_open, cola, football, orange, no_bar_lane = [int(a) for a in argv]
    
    # 参数校验
    if mid_open not in (0, 1):
        raise ValueError("mid_open 必须是 0 或 1")
    if no_bar_lane not in (1, 2, 3):
        raise ValueError("no_bar_lane 必须是 1、2 或 3")
    # ... 其他校验
    
    return {
        "mid_open": "left" if mid_open == 0 else "right",
        "no_bar_lane": {1: "left", 2: "mid", 3: "right"}[no_bar_lane],
        "lane_of": { ... },
    }
```

### 4.2 道路配置函数
```python
def _has_bar(lane, no_bar_lane):
    """判断指定道是否有限高杆"""
    return lane != no_bar_lane

def _needs_opening(lane, no_bar_lane):
    """判断指定道是否需要通过小口进出"""
    return lane == no_bar_lane

def _get_opening_position(no_bar_lane, mid_open):
    """获取小口的 x 坐标"""
    if no_bar_lane == "left":
        return MID_OPENING_LEFT_EXIT_X  # 0.35
    elif no_bar_lane == "right":
        return MID_OPENING_RIGHT_EXIT_X  # 1.65
    elif no_bar_lane == "mid":
        return MID_OPENING_LEFT_EXIT_X if mid_open == "left" else MID_OPENING_RIGHT_EXIT_X
```

### 4.3 访问顺序决策
```python
def _determine_lane_order(no_bar_lane, mid_open):
    """根据无杆道和开口位置决定访问顺序"""
    if no_bar_lane == "left":
        # 左道无杆，小口在中道左侧，顺序 L → M → R
        return ["left", "mid", "right"]
    elif no_bar_lane == "right":
        # 右道无杆，小口在中道右侧，顺序 L → M → R  
        return ["left", "mid", "right"]
    elif no_bar_lane == "mid":
        # 中道无杆，根据 mid_open 决定
        if mid_open == "left":
            return ["left", "mid", "right"]  # 从左进中道
        else:
            return ["left", "right", "mid"]  # 从右进中道
```

### 4.4 状态机路由修改

需要修改的核心函数：
1. `_route_general_lane()` - 有杆道的状态机（需要蹲姿过杆）
2. `_route_opening_lane()` - **新增**：无杆道的状态机（通过小口进出）
3. `_begin_lane()` - 根据 lane 是否有杆，进入不同的状态机

**无杆道状态机**需要实现：
- 从底部横道走到小口高度 (y=8.85)
- 横穿小口进入该道
- 转北，对齐中心线
- 向北走到目标物，撞击
- 转南，返回到小口高度
- 横穿小口出该道
- 继续向南回底部横道或进入下一道

### 4.5 蹲姿检测逻辑修改
```python
# 原逻辑
_current_has_bar = _has_bar(_current_lane)  # 根据 lane in ("left", "right")

# 新逻辑  
_current_has_bar = _has_bar(_current_lane, _preset["no_bar_lane"])
```

## 五、测试用例

### 5.1 逻辑测试矩阵

| 测试 | no_bar | mid_open | cola | ball | orange | 预期顺序 |
|------|--------|----------|------|------|--------|---------|
| 1 | 1 (左) | 0 | 1 | 2 | 3 | L(开口)→M(杆)→R(杆) |
| 2 | 2 (中) | 0 | 1 | 2 | 3 | L(杆)→M(开口左)→R(杆) |
| 3 | 2 (中) | 1 | 1 | 2 | 3 | L(杆)→R(杆)→M(开口右) |
| 4 | 3 (右) | 0 | 1 | 2 | 3 | L(杆)→M(杆)→R(开口) |

### 5.2 边界情况测试
- 足球在无杆道：确保不触发蹲姿逻辑
- 橙球在无杆道：跳跃撞球后正常返回
- 小口横穿时的横向纠偏

## 六、实现步骤建议

1. **复制原文件创建 v2 版本**
2. **修改参数解析**：增加 no_bar_lane 参数
3. **添加辅助函数**：`_has_bar()`, `_needs_opening()`, `_get_opening_position()`, `_determine_lane_order()`
4. **实现无杆道状态机**：新增 `_route_opening_lane()` 函数
5. **修改路由调度**：在 `_route()` 中根据道路类型分发到对应状态机
6. **测试验证**：编写测试用例覆盖所有场景

## 七、注意事项

1. **小口位置推导**：
   - 左道无杆 → 小口在中道左侧（固定）
   - 右道无杆 → 小口在中道右侧（固定）
   - 中道无杆 → 小口由 mid_open 参数决定

2. **足球特殊处理**：
   - 有杆道的足球：保持低姿态短退 + 低姿态返回过杆
   - 无杆道的足球：正常后退即可（无需蹲姿）

3. **横向纠偏**：
   - 通过小口时需要精确对齐 y=8.85
   - 横穿时同时纠正横向偏移

4. **兼容性**：
   - 保留原 segment4_manual.py 不变
   - 新版本命名为 segment4_manual_v2.py
   - 测试文件命名为 test_segment4_manual_v2_logic.py
