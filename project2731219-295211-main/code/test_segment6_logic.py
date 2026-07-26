"""赛段6纯逻辑单测（无硬件依赖，纯 Python 运行）。"""
import os, sys
sys.path.insert(0, os.path.dirname(__file__))
import segment6 as s6


def test_norm_wraps_to_pm180():
    assert s6._norm(190) == -170
    assert s6._norm(-190) == 170
    assert s6._norm(360) == 0
    assert s6._norm(0) == 0
    assert s6._norm(180) == 180
    assert s6._norm(-180) == 180


def test_turn_step_aligned_returns_zero():
    assert s6._turn_step(328, 328) == 0
    assert s6._turn_step(328 + s6.SLOW_DEG - 1, 328) == 0
    assert s6._turn_step(328 + s6.SLOW_DEG, 328) == 0


def test_turn_step_picks_direction_and_speed():
    # 朝向偏大(需右转)：偏差>FAST → 快右转15；中等 → 慢右转3
    assert s6._turn_step(328 + s6.FAST_DEG + 5, 328) == s6.G_FTURN_R
    assert s6._turn_step(328 + s6.SLOW_DEG + 2, 328) == s6.G_TURN_R
    # 朝向偏小(需左转)
    assert s6._turn_step(328 - s6.FAST_DEG - 5, 328) == s6.G_FTURN_L
    assert s6._turn_step(328 - s6.SLOW_DEG - 2, 328) == s6.G_TURN_L
    assert s6._turn_step(328 + s6.FAST_DEG, 328) == s6.G_TURN_R
    assert s6._turn_step(328 - s6.FAST_DEG, 328) == s6.G_TURN_L


def test_walk_turns_first_then_advances():
    assert s6._walk(300, 328, s6.G_PUSH) == s6._turn_step(300, 328)
    assert s6._walk(328, 328, s6.G_PUSH) == s6.G_PUSH


def test_arrived_within_tol():
    assert s6._arrived(0.20, 0.22) is True       # |0.02| <= XY_TOL
    assert s6._arrived(0.20, 0.40) is False


def test_dist_euclidean():
    assert s6._dist(0.0, 0.0, 0.0, 0.0) == 0.0
    assert s6._dist(3.0, 0.0, 0.0, 0.0) == 3.0
    assert s6._dist(0.0, 4.0, 0.0, 0.0) == 4.0
    assert abs(s6._dist(3.0, 4.0, 0.0, 0.0) - 5.0) < 1e-9
    # 与 SWEEP_L_DIST 阈值同量级：走0.20m 恰好达阈
    assert abs(s6._dist(0.20, 14.50, 0.0, 14.50) - 0.20) < 1e-9


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
    assert st == s6._ST_C_AIM_SWEEP and step == s6.G_STAND


def test_C_aim_sweep_turns_to_225_then_D():
    s6.reset_segment6()
    s6._state = s6._ST_C_AIM_SWEEP
    # 未对准225° → 先转向，状态留在 C
    step, st = _drive((0.20, 14.95), 180)
    assert step == s6._turn_step(180, s6.HDG_SWEEP) and st == s6._ST_C_AIM_SWEEP
    # 对准225° → 进 D1，首发低重心左横移步态
    step, st = _drive((0.20, 14.95), s6.HDG_SWEEP)
    assert st == s6._ST_D1_SWEEP and step == s6.G_SWEEP_L


def test_D1_sweep_left_then_D2():
    s6.reset_segment6()
    s6._state = s6._ST_D1_SWEEP
    # 进 D1 首帧记起点(0.20,14.95)，位移0 < SWEEP_L_DIST → 持续发低重心左移
    step, st = _drive((0.20, 14.95), s6.HDG_SWEEP)
    assert step == s6.G_SWEEP_L and st == s6._ST_D1_SWEEP
    assert s6._sweep_x0 == 0.20 and s6._sweep_y0 == 14.95
    # 位移仍不足 → 继续左移
    step, st = _drive((0.28, 14.85), s6.HDG_SWEEP)
    assert step == s6.G_SWEEP_L and st == s6._ST_D1_SWEEP
    # 位移≥SWEEP_L_DIST(从(0.20,14.95)走 hypot(0.20,0.20)=0.283≥0.25) → 置 D2、起点清空、发右移
    step, st = _drive((0.40, 14.75), s6.HDG_SWEEP)
    assert st == s6._ST_D2_CLEAR and step == s6.G_SWEEP_R
    assert s6._sweep_x0 is None and s6._sweep_y0 is None


def test_D2_clear_right_then_E():
    s6.reset_segment6()
    s6._state = s6._ST_D2_CLEAR
    # 进 D2 首帧记起点(0.40,14.75)（reset 已清 None），位移0 < SWEEP_R_DIST → 发右移
    step, st = _drive((0.40, 14.75), s6.HDG_SWEEP)
    assert step == s6.G_SWEEP_R and st == s6._ST_D2_CLEAR
    assert s6._sweep_x0 == 0.40 and s6._sweep_y0 == 14.75
    # 位移仍不足 → 继续右移
    step, st = _drive((0.34, 14.80), s6.HDG_SWEEP)
    assert step == s6.G_SWEEP_R and st == s6._ST_D2_CLEAR
    # 位移≥SWEEP_R_DIST(从(0.40,14.75)走 hypot(0.13,0.10)=0.164≥0.15) → 进 E，发站立
    step, st = _drive((0.27, 14.85), s6.HDG_SWEEP)
    assert st == s6._ST_E_FACE_PUSH and step == s6.G_STAND


def test_E_face_push_turns_to_328_then_verify():
    # 顶球时头朝225°、球在世界315°方向（差90°）→ 相机看不到球；
    # 转到328°后球才进视野，故 E 之后先进 D_VERIFY 验球，而非直接推球。
    s6.reset_segment6()
    s6._state = s6._ST_E_FACE_PUSH
    step, st = _drive((0.5, 14.6), s6.HDG_SWEEP)
    assert step == s6._turn_step(s6.HDG_SWEEP, s6.HDG_PUSH) and st == s6._ST_E_FACE_PUSH
    step, st = _drive((0.5, 14.6), s6.HDG_PUSH)
    assert st == s6._ST_D_VERIFY and step == s6.G_STAND


def test_F_dribble_then_G_keeps_low_push():
    s6.reset_segment6()
    s6._state = s6._ST_F_DRIBBLE
    step, st = _drive((2.0, 13.5), s6.HDG_PUSH)
    assert step == s6.G_PUSH and st == s6._ST_F_DRIBBLE
    # 进入穿缝阶段仍发低重心推球步态(43)，不再切高步态(28)——防球从身下漏走
    step, st = _drive((s6.KICK_TRIGGER_X, 13.2), s6.HDG_PUSH)
    assert st == s6._ST_G_THROUGH_GAP and step == s6.G_PUSH


def test_G_through_gap_keeps_low_push_then_turn_finish():
    s6.reset_segment6()
    s6._state = s6._ST_G_THROUGH_GAP
    # 穿缝途中仍用低重心推球步态(43)压住球，不换高步态(28)
    step, st = _drive((2.6, 13.0), s6.HDG_PUSH)
    assert step == s6.G_PUSH and st == s6._ST_G_THROUGH_GAP
    # 到圈心x → 进入转身阶段（不再直接趴下）
    step, st = _drive((s6.FINISH_STOP_X, 12.85), s6.HDG_PUSH)
    assert st == s6._ST_TURN_FINISH and step == s6.G_STAND


def test_turn_finish_turns_to_plus_x_then_laydown_then_done():
    s6.reset_segment6()
    s6._state = s6._ST_TURN_FINISH
    # 还朝328° → 先转向对准+x(0°)，不进趴下
    step, st = _drive((s6.FINISH_CX, s6.FINISH_CY), s6.HDG_PUSH)
    assert step == s6._turn_step(s6.HDG_PUSH, s6.HDG_FINISH) and st == s6._ST_TURN_FINISH
    assert step != 0   # 328°离0°有32°，确实需要转
    # 已对准+x → 进趴下
    step, st = _drive((s6.FINISH_CX, s6.FINISH_CY), s6.HDG_FINISH)
    assert st == s6._ST_H_LAYDOWN and step == s6.G_STAND
    # 趴下计数到 DONE
    last = None
    for _ in range(4):
        last, st = _drive((s6.FINISH_CX, s6.FINISH_CY), s6.HDG_FINISH)
    assert last == -1 and st == s6._ST_DONE


def test_H_laydown_advances_even_when_mode7():
    # 回归保护：趴下发出后 mode 变7，H 须绕过顶部等待判据继续计数到 DONE。
    s6.reset_segment6()
    s6._state = s6._ST_H_LAYDOWN
    last = None
    for _ in range(3):
        last, st = _drive((s6.FINISH_CX, s6.FINISH_CY), s6.HDG_PUSH, gait_mode=(0, 7))
    assert last == -1 and st == s6._ST_DONE


def test_DONE_returns_minus1_even_when_mode7():
    # 回归保护：DONE 是终止态，即使 mode==7（趴下中）也须无条件返回 -1，
    # 不能被顶部等待判据拦成 G_STAND（否则赛段永不报完成）。
    s6.reset_segment6()
    s6._state = s6._ST_DONE
    step = s6.segment6_control([s6.FINISH_CX, s6.FINISH_CY, 0.0], [0, 7], s6.HDG_PUSH)
    assert step == -1 and s6._state == s6._ST_DONE


# ── 以下为真机化改动的新增覆盖（停滞判据 / 验球 / 重试 / 超时 / 候选圆过滤）──


def test_is_stalled_triggers_only_after_consecutive_small_moves():
    s6.reset_segment6()
    # 首帧只记基线，不判停滞
    assert s6._is_stalled(14.0, "T") is False
    # 连续小位移累计，达到 STALL_FRAMES 才为 True
    for _ in range(s6.STALL_FRAMES - 1):
        assert s6._is_stalled(14.0, "T") is False
    assert s6._is_stalled(14.0, "T") is True


def test_is_stalled_resets_on_normal_movement():
    s6.reset_segment6()
    s6._is_stalled(14.0, "T")
    for _ in range(s6.STALL_FRAMES - 1):
        s6._is_stalled(14.0, "T")
    # 一次正常步幅位移（>STALL_EPS）把计数清零
    assert s6._is_stalled(14.0 + s6.STALL_EPS * 4, "T") is False
    assert s6._is_stalled(14.0 + s6.STALL_EPS * 4, "T") is False


def test_is_stalled_keys_are_independent():
    s6.reset_segment6()
    s6._is_stalled(1.0, "A")
    for _ in range(s6.STALL_FRAMES):
        s6._is_stalled(1.0, "A")
    # 另一个 key 独立计数，不受 A 的累计影响
    assert s6._is_stalled(1.0, "B") is False


def test_A_exits_on_stall_even_when_coord_never_reached():
    # 真机漂移偏大时坐标阈值可能永不满足：狗顶墙打滑 → 靠停滞判据脱困，不卡死。
    s6.reset_segment6()
    stuck_y = s6.TOP_Y - 0.5          # 坐标始终不达标
    st = None
    for _ in range(s6.STALL_FRAMES + 2):
        _step, st = _drive((2.5, stuck_y), s6.HDG_UP)
    assert st == s6._ST_B_GO_CORNER


def test_B_exits_on_stall_even_when_coord_never_reached():
    s6.reset_segment6()
    s6._set_state(s6._ST_B_GO_CORNER)
    stuck_x = s6.CORNER_X + 0.5
    st = None
    for _ in range(s6.STALL_FRAMES + 2):
        _step, st = _drive((stuck_x, 14.95), s6.HDG_LEFT)
    assert st == s6._ST_C_AIM_SWEEP


def _verify_frame(found):
    """构造 D_VERIFY 用的假帧：monkeypatch find_ball 的返回。"""
    return (True, 0.0, 20.0) if found else (False, 0.0, 0.0)


_FAKE_FRAME = object()   # 非 None 即可：find_ball 被 monkeypatch，不会真解析画面


def _run_verify(hits, total):
    """在 D_VERIFY 状态下驱动 total 帧，前 hits 帧命中。返回最终状态。

    必须传非 None 的 frame —— frame=None 是「相机故障降级」路径，会绕过验球判定。
    """
    calls = {"n": 0}

    def fake_find_ball(frame):
        calls["n"] += 1
        return _verify_frame(calls["n"] <= hits)

    orig = s6.find_ball
    s6.find_ball = fake_find_ball
    try:
        for _ in range(total):
            s6.segment6_control([0.6, 14.4, 0.0], [11, 0], s6.HDG_PUSH,
                                frame=_FAKE_FRAME)
        return s6._state
    finally:
        s6.find_ball = orig


def test_verify_success_when_enough_hits_goes_to_dribble():
    s6.reset_segment6()
    s6._set_state(s6._ST_D_VERIFY)
    st = _run_verify(hits=s6.VERIFY_HITS, total=s6.VERIFY_WINDOW)
    assert st == s6._ST_F_DRIBBLE


def test_verify_failure_goes_to_retry_when_attempts_remain():
    s6.reset_segment6()
    s6._set_state(s6._ST_D_VERIFY)
    # 命中不足 VERIFY_HITS → 判失败；首顶后仍有重试余量 → 进 R_RETURN
    st = _run_verify(hits=s6.VERIFY_HITS - 1, total=s6.VERIFY_WINDOW)
    assert st == s6._ST_R_RETURN


def test_verify_failure_switches_to_kick_when_attempts_exhausted():
    s6.reset_segment6()
    s6._attempt = s6.MAX_ATTEMPT - 1      # 已是最后一次尝试
    s6._set_state(s6._ST_D_VERIFY)
    st = _run_verify(hits=0, total=s6.VERIFY_WINDOW)
    assert st == s6._ST_K_AIM


def test_verify_degrades_to_dribble_when_camera_dead():
    # frame 恒为 None（相机故障）→ 首帧就降级放行，走原纯里程计流程，不卡死。
    s6.reset_segment6()
    s6._set_state(s6._ST_D_VERIFY)
    step, st = _drive((0.6, 14.4), s6.HDG_PUSH)
    assert st == s6._ST_F_DRIBBLE and step == s6.G_PUSH


def test_retry_return_resets_and_bumps_attempt():
    s6.reset_segment6()
    s6._sweep_x0, s6._sweep_y0 = 0.4, 14.7
    s6._verify_hit, s6._verify_frames = 2, 5
    s6._set_state(s6._ST_R_RETURN)
    _step, st = _drive((0.3, 14.8), s6.HDG_PUSH)
    # 复用贴墙机制：状态回到 A，重新「贴墙走到走不动为止」收敛到同一角落
    assert st == s6._ST_A_GO_TOP
    assert s6._attempt == 1
    assert s6._verify_hit == 0 and s6._verify_frames == 0
    assert s6._sweep_x0 is None and s6._sweep_y0 is None
    assert s6._stall == {}


def test_sweep_params_advance_with_attempt():
    s6.reset_segment6()
    assert s6._sweep_params() == s6.RETRY_PROFILE[0]
    s6._attempt = 1
    assert s6._sweep_params() == s6.RETRY_PROFILE[1]
    s6._attempt = 2
    assert s6._sweep_params() == s6.RETRY_PROFILE[2]
    # 越界退化到最后一档，不抛 IndexError
    s6._attempt = 99
    assert s6._sweep_params() == s6.RETRY_PROFILE[-1]


def test_retry_uses_profile_heading_and_distance():
    # 第2次尝试应改用 RETRY_PROFILE[1] 的朝向与位移阈值（换角度换幅度，不是重复同一动作）
    s6.reset_segment6()
    s6._attempt = 1
    dist, hdg = s6.RETRY_PROFILE[1]
    s6._set_state(s6._ST_C_AIM_SWEEP)
    # 朝向按新档位判定：对准 225°(基准) 时仍需继续转向到 235°
    step, st = _drive((0.20, 14.95), s6.HDG_SWEEP)
    assert step == s6._turn_step(s6.HDG_SWEEP, hdg) and st == s6._ST_C_AIM_SWEEP
    # 对准新朝向 → 进 D1
    _step, st = _drive((0.20, 14.95), hdg)
    assert st == s6._ST_D1_SWEEP
    # D1 位移阈值用新档位：走基准 0.25m 还不够（新阈值 0.32m），仍留在 D1
    _step, st = _drive((0.20, 14.95), hdg)
    _step, st = _drive((0.20 + 0.25, 14.95), hdg)
    assert st == s6._ST_D1_SWEEP
    _step, st = _drive((0.20 + dist + 0.01, 14.95), hdg)
    assert st == s6._ST_D2_CLEAR


def _freeze_clock(monkey_t):
    """把 time.monotonic 替换为可控时钟。"""
    s6.time.monotonic = monkey_t


def test_state_timeout_forces_progress():
    # 停滞检测也失效的极端情况（如被卡住但里程计仍缓慢累加）→ 单状态超时强推。
    real = s6.time.monotonic
    s6.reset_segment6()
    s6._set_state(s6._ST_C_AIM_SWEEP)
    t0 = s6._state_enter_t
    try:
        _freeze_clock(lambda: t0 + s6.STATE_TIMEOUT + 1)
        _step, st = _drive((0.2, 14.9), 180)   # 始终不对准，正常永远留在 C
        assert st == s6._ST_D1_SWEEP
    finally:
        s6.time.monotonic = real


def test_push_timeout_goes_to_finish():
    # F/G 无墙可撞只能靠坐标；超时就地转身趴下，保住完赛动作。
    real = s6.time.monotonic
    s6.reset_segment6()
    s6._set_state(s6._ST_F_DRIBBLE)
    t0 = s6._push_start_t
    try:
        _freeze_clock(lambda: t0 + s6.PUSH_TIMEOUT + 1)
        _step, st = _drive((1.0, 14.0), s6.HDG_PUSH)
        assert st == s6._ST_TURN_FINISH
    finally:
        s6.time.monotonic = real


def test_seg_timeout_abandons_ball():
    # 赛段总超时是唯一允许放弃球的地方：卡死不趴下比球没进圈更糟。
    real = s6.time.monotonic
    s6.reset_segment6()
    t0 = s6._seg_start_t
    try:
        _freeze_clock(lambda: t0 + s6.SEG_TIMEOUT + 1)
        _step, st = _drive((1.0, 14.0), s6.HDG_PUSH)
        assert st == s6._ST_ABANDON
    finally:
        s6.time.monotonic = real


def test_abandon_pushes_then_finishes_on_stall():
    real = s6.time.monotonic
    s6.reset_segment6()
    s6._set_state(s6._ST_ABANDON)
    try:
        _freeze_clock(lambda: s6._state_enter_t)   # 冻住时钟，避免单状态超时干扰
        step, st = _drive((1.0, 14.0), s6.HDG_PUSH)
        assert step == s6.G_PUSH and st == s6._ST_ABANDON
        st = None
        for _ in range(s6.STALL_FRAMES + 2):
            _step, st = _drive((1.0, 14.0), s6.HDG_PUSH)
        assert st == s6._ST_TURN_FINISH
    finally:
        s6.time.monotonic = real


def test_ball_from_candidates_picks_largest_round_blob():
    # (area, cx, radius)：圆度 = area/(πr²)
    import math
    r = 20.0
    round_area = math.pi * r * r * 0.9        # 圆度0.9 通过
    found, u, rad = s6._ball_from_candidates([(round_area, 400.0, r)], 640)
    assert found is True and rad == r
    assert u == 400.0 - 320.0                 # 正=球偏右


def test_ball_from_candidates_rejects_elongated_shapes():
    # 细长的黄线/墙缝/阴影带：外接圆很大但面积占比低 → 圆度不达标
    import math
    r = 50.0
    thin_area = math.pi * r * r * 0.2
    found, _u, _rad = s6._ball_from_candidates([(thin_area, 300.0, r)], 640)
    assert found is False


def test_ball_from_candidates_rejects_out_of_range_radius():
    import math
    for r in (s6.BALL_MIN_R - 1, s6.BALL_MAX_R + 1):
        area = math.pi * r * r * 0.95
        found, _u, _rad = s6._ball_from_candidates([(area, 320.0, float(r))], 640)
        assert found is False


def test_ball_from_candidates_empty_returns_not_found():
    assert s6._ball_from_candidates([], 640) == (False, 0.0, 0.0)


def test_find_ball_none_frame_needs_no_cv2():
    # frame=None 路径只依赖标准库（cv2 延迟导入），无相机环境也能跑单测
    assert s6.find_ball(None) == (False, 0.0, 0.0)


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
        # 到圈心x → 进入转身阶段（与主线同形，先转身再趴下）
        step = s6.segment6_control([s6.FINISH_STOP_X, 12.85, 0.0], [11, 0], s6.HDG_PUSH)
        assert step == s6.G_STAND and s6._state == s6._ST_TURN_FINISH
        # 还朝328° → 转身对准+x(0°)
        step = s6.segment6_control([s6.FINISH_CX, s6.FINISH_CY, 0.0], [11, 0], s6.HDG_PUSH)
        assert step == s6._turn_step(s6.HDG_PUSH, s6.HDG_FINISH) and s6._state == s6._ST_TURN_FINISH
        # 对准+x → 进趴下
        step = s6.segment6_control([s6.FINISH_CX, s6.FINISH_CY, 0.0], [11, 0], s6.HDG_FINISH)
        assert step == s6.G_STAND and s6._state == s6._ST_H_LAYDOWN
        # 趴下计数：本函数自带 H/DONE 副本，驱动到 DONE 返回 -1（防与主线漂移）
        last = None
        for _ in range(3):
            last = s6.segment6_control([s6.FINISH_CX, s6.FINISH_CY, 0.0], [11, 0], s6.HDG_FINISH)
        assert last == -1 and s6._state == s6._ST_DONE
    finally:
        s6.USE_KICK_FALLBACK = False
        s6.reset_segment6()
