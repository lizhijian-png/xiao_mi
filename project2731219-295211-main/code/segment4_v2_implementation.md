# segment4_manual_v2.py 关键代码片段

## 1. 参数解析函数（完整替换）

```python
def parse_preset_args(argv):
    """解析 5 个整数参数，返回 preset dict。
    
    返回：
      {
        "mid_open": "left" | "right",  # 仅当 no_bar_lane="mid" 时有意义
        "no_bar_lane": "left" | "mid" | "right",  # 新增：哪条道没有杆
        "lane_of": {"cola": "left"/"mid"/"right",
                    "football": ...,
                    "orange": ...},
      }
    """
    if len(argv) != 5:
        raise ValueError(
            f"需要 5 个整数(mid_open cola football orange no_bar_lane)，实际收到 {len(argv)} 个"
        )
    try:
        nums = [int(a) for a in argv]
    except ValueError as e:
        raise ValueError(f"参数必须是整数，收到: {argv}") from e

    mid_open, cola, football, orange, no_bar_lane = nums
    
    if mid_open not in (0, 1):
        raise ValueError(f"mid_open 必须是 0(左) 或 1(右)，收到: {mid_open}")
    if no_bar_lane not in (1, 2, 3):
        raise ValueError(f"no_bar_lane 必须是 1(左)、2(中) 或 3(右)，收到: {no_bar_lane}")
    
    for name, v in (("cola", cola), ("football", football), ("orange", orange)):
        if v not in (1, 2, 3):
            raise ValueError(f"{name} 位置必须在 1~3 之间，收到: {v}")
    if len({cola, football, orange}) != 3:
        raise ValueError(
            f"三个目标物位置必须互不相同，收到 cola={cola} football={football} orange={orange}"
        )

    lane_name = {1: "left", 2: "mid", 3: "right"}
    return {
        "mid_open": "left" if mid_open == 0 else "right",
        "no_bar_lane": lane_name[no_bar_lane],
        "lane_of": {
            "cola": lane_name[cola],
            "football": lane_name[football],
            "orange": lane_name[orange],
        },
    }
```

## 2. 辅助判断函数（新增）

```python
def _has_bar(lane):
    """判断指定道是否有限高杆"""
    return lane != _preset["no_bar_lane"]


def _needs_opening(lane):
    """判断指定道是否需要通过小口进出"""
    return lane == _preset["no_bar_lane"]


def _get_opening_x(lane):
    """获取指定道的小口 x 坐标
    
    返回：
      - 左道无杆：小口在中道左侧 (0.35)
      - 中道无杆：根据 mid_open 决定 (0.35 or 1.65)
      - 右道无杆：小口在中道右侧 (1.65)
    """
    no_bar = _preset["no_bar_lane"]
    
    if no_bar == "left":
        return MID_OPENING_LEFT_EXIT_X  # 0.35
    elif no_bar == "right":
        return MID_OPENING_RIGHT_EXIT_X  # 1.65
    elif no_bar == "mid":
        mid_open = _preset["mid_open"]
        return MID_OPENING_LEFT_EXIT_X if mid_open == "left" else MID_OPENING_RIGHT_EXIT_X
    return MID_LANE_X  # 不应该到这里
```

## 3. 访问顺序决策（修改 reset 函数）

```python
def reset_segment4_manual_v2(preset):
    """重置状态机。preset 由 parse_preset_args 生成。"""
    global _state, _lane_order, _lane_iter, _current_lane, _current_target
    global _current_has_bar, _mid_open, _preset, _motion_start, _stand_count
    global _jump_frames, _announced, _last_log_time, _last_log_signature

    _preset = preset
    _mid_open = preset["mid_open"]
    no_bar = preset["no_bar_lane"]
    
    # 决定访问顺序
    if no_bar == "left":
        _lane_order = ["left", "mid", "right"]
    elif no_bar == "right":
        _lane_order = ["left", "mid", "right"]
    elif no_bar == "mid":
        # 中道无杆，根据 mid_open 决定
        _lane_order = ["left", "mid", "right"] if _mid_open == "left" else ["left", "right", "mid"]
    
    _lane_iter = 0
    _current_lane = None
    _current_target = None
    _current_has_bar = False
    _motion_start = None
    _stand_count = 0
    _jump_frames = 0
    _announced = set()
    _last_log_time = 0.0
    _last_log_signature = None
    _state = S["TO_START"]
```

## 4. 开始道路函数（修改 _begin_lane）

```python
def _begin_lane():
    """切换到下一条竖道，初始化该道的全局配置。"""
    global _lane_iter, _current_lane, _current_target, _current_has_bar, _state
    global _motion_start, _stand_count
    
    lane = _lane_order[_lane_iter]
    _current_lane = lane
    
    # 找到该道的目标物
    for target, ln in _preset["lane_of"].items():
        if ln == lane:
            _current_target = target
            break
    
    # 判断该道是否有杆
    _current_has_bar = _has_bar(lane)
    _motion_start = None
    _stand_count = 0
    
    # 根据是否需要小口进入不同状态
    if _needs_opening(lane):
        _set_state(S["OPENING_ENTER"], f"begin_opening_lane_{lane}")
    else:
        # 有杆道，正常流程
        _set_state(S["LANE_ENTER_X"], f"begin_bar_lane_{lane}")
```

## 5. 无杆道状态定义（新增到 S 字典）

```python
S = {
    # ... 原有状态 ...
    
    # 无杆道（通过小口进出）
    "OPENING_ENTER": "S4M_OPENING_ENTER",              # 走到小口高度
    "OPENING_CROSS_IN": "S4M_OPENING_CROSS_IN",        # 横穿小口进入
    "OPENING_TURN_NORTH": "S4M_OPENING_TURN_NORTH",    # 转北
    "OPENING_ALIGN_UP": "S4M_OPENING_ALIGN_UP",        # 对齐中心线
    "OPENING_ADVANCE": "S4M_OPENING_ADVANCE",          # 向北到目标
    "OPENING_BACKUP": "S4M_OPENING_BACKUP",            # 撞击后退
    "OPENING_JUMP": "S4M_OPENING_JUMP",                # 跳跃撞橙球
    "OPENING_TURN_SOUTH": "S4M_OPENING_TURN_SOUTH",    # 转南
    "OPENING_ALIGN_DOWN": "S4M_OPENING_ALIGN_DOWN",    # 对齐中心线
    "OPENING_RETURN": "S4M_OPENING_RETURN",            # 返回小口高度
    "OPENING_CROSS_OUT": "S4M_OPENING_CROSS_OUT",      # 横穿小口出去
    "OPENING_EXIT_SOUTH": "S4M_OPENING_EXIT_SOUTH",    # 转南
    "OPENING_TO_BOTTOM": "S4M_OPENING_TO_BOTTOM",      # 下到底部横道
    
    # ... 原有状态 ...
}
```

## 6. 无杆道状态机（新增函数）

```python
def _route_opening_lane(position, rpy, frame):
    """无杆道状态机：通过小口 (y=8.85) 进出，无需蹲姿。"""
    global _lane_iter, _jump_frames
    
    x, y, _ = position
    lane_x = _lane_x(_current_lane)
    opening_x = _get_opening_x(_current_lane)
    target = _current_target
    target_y = TARGET_Y[target]
    announce_y = ANNOUNCE_Y[target]
    backup_dist = BACKUP_DIST[target]
    backup_y = target_y + 0.05
    
    # ── 进入：从底部横道或上一道走到小口高度 ──
    if _state == S["OPENING_ENTER"]:
        # 向北走到 y=8.85
        if y >= OPENING_Y - XY_TOL:
            _set_state(S["OPENING_CROSS_IN"], "opening_reach_y", position)
            return _return_step(0, "opening_reach_y", position, rpy)
        step = _forward_with_lateral(position, rpy, HEADING_NORTH, opening_x, 'x')
        return _return_step(step, "opening_enter", position, rpy)
    
    if _state == S["OPENING_CROSS_IN"]:
        # 横穿进入该道中心线
        return _go_x(position, rpy, lane_x, 
                    HEADING_EAST if lane_x > x else HEADING_WEST,
                    S["OPENING_TURN_NORTH"], "opening_cross_in_done", 
                    center_y=OPENING_Y)
    
    if _state == S["OPENING_TURN_NORTH"]:
        return _turn_state(rpy, HEADING_NORTH, S["OPENING_ALIGN_UP"], 
                          "opening_turn_north_done", position)
    
    if _state == S["OPENING_ALIGN_UP"]:
        return _adjust_x(position, rpy, lane_x, HEADING_NORTH, 
                        S["OPENING_ADVANCE"], "opening_align_up_done")
    
    # ── 向北到目标物 ──
    if _state == S["OPENING_ADVANCE"]:
        detected = _detect_target(frame, target)
        if detected or y >= announce_y:
            _announce_once(target, TARGET_NAME_CN[target])
        if y >= backup_y:
            _motion_start_reset(position)
            if target == "orange":
                _jump_frames = 0
                _set_state(S["OPENING_JUMP"], "opening_orange_jump_start", position)
                return _return_step(ORANGE_JUMP_GAIT, "opening_orange_jump_start", position, rpy)
            _set_state(S["OPENING_BACKUP"], "opening_target_reach", position)
            return _return_step(0, "opening_target_reach", position, rpy)
        step = _forward_lane_step(position, rpy, HEADING_NORTH, lane_x)
        return _return_step(step, "opening_advance", position, rpy)
    
    if _state == S["OPENING_BACKUP"]:
        return _backup_to_distance(position, S["OPENING_TURN_SOUTH"], 
                                   "opening_backup_done", backup_dist)
    
    if _state == S["OPENING_JUMP"]:
        _jump_frames += 1
        if _jump_frames >= JUMP_FRAMES:
            _set_state(S["OPENING_TURN_SOUTH"], "opening_jump_done", position)
            return _return_step(0, "opening_jump_done", position, rpy)
        return _return_step(ORANGE_JUMP_GAIT, "opening_jump", position, rpy)
    
    # ── 返回小口出去 ──
    if _state == S["OPENING_TURN_SOUTH"]:
        return _turn_state(rpy, HEADING_SOUTH, S["OPENING_ALIGN_DOWN"], 
                          "opening_turn_south_done", position)
    
    if _state == S["OPENING_ALIGN_DOWN"]:
        return _adjust_x(position, rpy, lane_x, HEADING_SOUTH, 
                        S["OPENING_RETURN"], "opening_align_down_done")
    
    if _state == S["OPENING_RETURN"]:
        # 向南返回到小口高度
        if y <= OPENING_Y + XY_TOL:
            _set_state(S["OPENING_CROSS_OUT"], "opening_return_to_y", position)
            return _return_step(0, "opening_return_to_y", position, rpy)
        step = _forward_lane_step(position, rpy, HEADING_SOUTH, lane_x)
        return _return_step(step, "opening_return", position, rpy)
    
    if _state == S["OPENING_CROSS_OUT"]:
        # 横穿出小口
        return _go_x(position, rpy, opening_x,
                    HEADING_EAST if opening_x > x else HEADING_WEST,
                    S["OPENING_EXIT_SOUTH"], "opening_cross_out_done",
                    center_y=OPENING_Y)
    
    if _state == S["OPENING_EXIT_SOUTH"]:
        return _turn_state(rpy, HEADING_SOUTH, S["OPENING_TO_BOTTOM"], 
                          "opening_exit_south_done", position)
    
    if _state == S["OPENING_TO_BOTTOM"]:
        # 继续向南到底部横道或进入下一道
        target_y = LANE_SWITCH_Y
        next_lane_idx = _lane_iter + 1
        
        # 检查下一道是否也需要小口（且能直接横穿）
        if next_lane_idx < len(_lane_order):
            next_lane = _lane_order[next_lane_idx]
            if _needs_opening(next_lane):
                # 下一道也是无杆道，直接在当前高度横穿过去
                pass  # 后续实现
        
        if y <= target_y:
            _lane_iter += 1
            if _lane_iter >= len(_lane_order):
                _set_state(S["BRIDGE_TO_X"], "opening_all_done", position)
                return _return_step(0, "opening_all_done", position, rpy)
            _begin_lane()
            return _return_step(0, "opening_next_lane", position, rpy)
        step = _forward_lane_step(position, rpy, HEADING_SOUTH, opening_x)
        return _return_step(step, "opening_to_bottom", position, rpy)
    
    return None
```

## 7. 总路由调度（修改 _route 函数）

```python
def _route(position, rpy, frame):
    """第四段总调度。"""
    if _state == S["TO_START"]:
        return _route_entry(position, rpy)

    if _state in (S["BRIDGE_TO_X"], S["BRIDGE_TURN_UP"]):
        return _route_bridge(position, rpy)

    # 判断当前道是否需要小口
    if _needs_opening(_current_lane):
        result = _route_opening_lane(position, rpy, frame)
        if result is not None:
            return result

    # 有杆道，使用原有状态机
    # 注意：原 _route_mid 逻辑需要判断中道是否有杆
    if _current_lane == "mid" and not _has_bar("mid"):
        # 中道无杆，使用小口状态机（已在上面处理）
        pass
    elif _current_lane == "mid":
        # 中道有杆，使用有杆状态机（需要新实现）
        return _route_general_lane(position, rpy, frame)
    else:
        return _route_general_lane(position, rpy, frame)
```

## 8. 主要修改点总结

### 需要修改的函数：
1. `parse_preset_args()` - 增加 no_bar_lane 参数解析
2. `reset_segment4_manual()` - 根据 no_bar_lane 决定访问顺序
3. `_has_bar()` - 改为读取 preset 判断
4. `_begin_lane()` - 根据是否有杆进入不同状态
5. `_route()` - 增加无杆道路由分发

### 需要新增的函数：
1. `_needs_opening()` - 判断是否需要小口
2. `_get_opening_x()` - 获取小口 x 坐标
3. `_route_opening_lane()` - 无杆道状态机

### 需要新增的状态：
- OPENING_ENTER, OPENING_CROSS_IN, OPENING_TURN_NORTH
- OPENING_ALIGN_UP, OPENING_ADVANCE, OPENING_BACKUP, OPENING_JUMP
- OPENING_TURN_SOUTH, OPENING_ALIGN_DOWN, OPENING_RETURN
- OPENING_CROSS_OUT, OPENING_EXIT_SOUTH, OPENING_TO_BOTTOM

### 不需要修改的部分：
- 视觉检测函数（_detect_cola, _detect_football, 等）
- 步态定义（步态编号保持不变）
- 基础运动函数（_forward_step, _turn_to, 等）
- 桥接状态机（_route_bridge）
