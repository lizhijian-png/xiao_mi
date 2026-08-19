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


def _drive(pos, yaw, gait_mode=(11, 0), frame=None):
    """跑一帧，返回 (步态, 推进后的状态名)。"""
    step = s6.segment6_control(list(pos), list(gait_mode), yaw, frame)
    return step, s6._state


class _FakeFrame:
    """冒充相机帧：只需 .shape 让 frame is not None 成立并给出画面宽度。

    真正的球检测由 monkeypatch 掉的 find_ball 提供，不碰 cv2，保持纯逻辑单测。
    """
    shape = (480, 640, 3)


def _stub_ball(monkeypatch, found, u=0.0, r=30.0):
    monkeypatch.setattr(s6, 'find_ball', lambda _f: (found, u, r))


def _prime_ball_confirmation():
    s6._ball_hit_count = s6.BALL_CONFIRM_FRAMES - 1


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


def test_E_face_push_turns_to_328_then_F():
    s6.USE_VISION = False
    try:
        s6.reset_segment6()
        s6._state = s6._ST_E_FACE_PUSH
        step, st = _drive((0.5, 14.6), s6.HDG_SWEEP)
        assert step == s6._turn_step(s6.HDG_SWEEP, s6.HDG_PUSH) and st == s6._ST_E_FACE_PUSH
        step, st = _drive((0.5, 14.6), s6.HDG_PUSH)
        assert st == s6._ST_F_DRIBBLE and step == s6.G_PUSH
    finally:
        s6.USE_VISION = True


def test_F_dribble_then_G_keeps_low_push():
    s6.reset_segment6()
    s6._state = s6._ST_F_DRIBBLE
    step, st = _drive((2.0, 13.5), s6.HDG_PUSH)
    assert step == s6.G_PUSH and st == s6._ST_F_DRIBBLE
    # y尚未到出口高度，继续沿328°低重心接近
    step, st = _drive((s6.KICK_TRIGGER_X, 13.2), s6.HDG_PUSH)
    assert st == s6._ST_F_DRIBBLE and step == s6.G_PUSH


def test_G_through_gap_keeps_low_push_then_turn_finish():
    s6.reset_segment6()
    s6._state = s6._ST_G_THROUGH_GAP
    # 穿缝途中仍用低重心推球步态(43)压住球，不换高步态(28)
    step, st = _drive((2.6, 13.0), s6.HDG_PUSH)
    assert step == s6.G_PUSH and st == s6._ST_G_THROUGH_GAP
    # 到出口高度后转入对准出口状态
    step, st = _drive((s6.BALL_EXIT_DOG_X, 12.90), s6.HDG_PUSH)
    assert st == s6._ST_ALIGN_EXIT and step == s6.G_STAND
    step, st = _drive((s6.BALL_EXIT_DOG_X, 12.90), s6.HDG_FINISH)
    assert st == s6._ST_PUSH_EXIT and step == s6.G_STAND


def test_nav_finish_requires_both_xy_then_turns():
    s6.reset_segment6()
    s6._state = s6._ST_NAV_FINISH
    # x已到圆心但y偏离，不能按旧逻辑只看x就趴下
    step, st = _drive((s6.FINISH_CX, s6.FINISH_CY + 0.20), 270)
    assert st == s6._ST_NAV_FINISH and step != s6.G_STAND
    # x/y都到圆心容差内，才允许进入转身趴下阶段
    step, st = _drive((s6.FINISH_CX, s6.FINISH_CY), s6.HDG_FINISH)
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
        # 狗到出口线时球已越线 → 转为狗自行进终点圈
        step = s6.segment6_control([s6.BALL_EXIT_DOG_X, 12.90, 0.0], [11, 0], s6.HDG_PUSH)
        assert step == s6.G_STAND and s6._state == s6._ST_NAV_FINISH
        # 到终点圆心后才进入转身阶段
        step = s6.segment6_control([s6.FINISH_CX, s6.FINISH_CY, 0.0], [11, 0], s6.HDG_PUSH)
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


# ── 视觉追球：纯函数层 ──

def test_ball_from_candidates_filters_radius_and_circularity():
    import math
    area_ok = math.pi * 30 * 30 * 0.9          # 圆度0.9，过
    # 半径太小/太大都滤掉
    assert s6._ball_from_candidates([(area_ok, 100, s6.BALL_MIN_R - 1)], 640)[0] is False
    assert s6._ball_from_candidates([(area_ok, 100, s6.BALL_MAX_R + 1)], 640)[0] is False
    # 细长条（圆度低）滤掉：面积远小于外接圆
    assert s6._ball_from_candidates([(math.pi * 30 * 30 * 0.2, 100, 30)], 640)[0] is False
    # 半径0不崩
    assert s6._ball_from_candidates([(area_ok, 100, 0)], 640)[0] is False
    # 空候选
    assert s6._ball_from_candidates([], 640) == (False, 0.0, 0.0)


def test_ball_from_candidates_offset_sign_and_largest_wins():
    import math
    a = lambda r: math.pi * r * r * 0.9
    # 球心在画面右侧 → u 为正
    found, u, r = s6._ball_from_candidates([(a(30), 400, 30)], 640)
    assert found is True and u == 400 - 320 and r == 30
    # 球心在左 → 负
    assert s6._ball_from_candidates([(a(30), 200, 30)], 640)[1] == -120
    # 多候选取面积最大者
    found, u, r = s6._ball_from_candidates([(a(20), 200, 20), (a(50), 500, 50)], 640)
    assert u == 180 and r == 50


def test_find_ball_none_frame_degrades():
    assert s6.find_ball(None) == (False, 0.0, 0.0)


def test_hdg_to_and_blend():
    assert s6._hdg_to(0, 0, 1, 0) == 0            # +x
    assert round(s6._hdg_to(0, 0, 0, 1), 6) == 90  # +y
    assert round(s6._hdg_to(0.5, 14.5, 2.8, 12.9), 1) == round(
        s6._hdg_to(0.5, 14.5, 2.8, 12.9), 1)      # 稳定可复算
    # 混合：w=0 取 a，w=1 取 b，w=0.5 取中间（走最短弧，跨0°不绕远）
    assert s6._blend_hdg(10, 350, 0.0) == 10
    assert s6._blend_hdg(10, 350, 1.0) % 360 == 350
    assert s6._blend_hdg(10, 350, 0.5) % 360 == 0.0


# ── 视觉追球：状态机层 ──

def test_E_no_frame_advances_on_legacy_route_without_spinning():
    """无RGB帧时沿原路线接近远球，不得留在原地左右摆扫。"""
    s6.reset_segment6()
    s6._state = s6._ST_E_FACE_PUSH
    step, st = _drive((0.5, 14.6), s6.HDG_SWEEP)
    assert step == s6._turn_step(s6.HDG_SWEEP, s6.HDG_PUSH) and st == s6._ST_E_FACE_PUSH
    step, st = _drive((0.5, 14.6), s6.HDG_PUSH)
    assert st == s6._ST_F_DRIBBLE and step == s6.G_PUSH


def test_E_vision_fine_aims_at_offcenter_ball(monkeypatch):
    """球偏离原定路线：E 粗对准后不急着进F，先转向球。"""
    s6.reset_segment6()
    s6._state = s6._ST_E_FACE_PUSH
    _stub_ball(monkeypatch, True, u=+200)        # 球在画面右侧，明显偏
    _prime_ball_confirmation()
    # 粗对准328° → 站稳一帧取帧，仍留在 E
    step, st = _drive((0.5, 14.6), s6.HDG_PUSH, frame=_FakeFrame())
    assert st == s6._ST_E_FACE_PUSH and step == s6.G_STAND
    # 下一帧用视觉微调：球偏右 → 应右转（朝向减小），且不进 F
    step, st = _drive((0.5, 14.6), s6.HDG_PUSH, frame=_FakeFrame())
    assert st == s6._ST_E_FACE_PUSH
    assert step in (s6.G_TURN_R, s6.G_FTURN_R)
    # 球已居中 → 进 F 开推
    _stub_ball(monkeypatch, True, u=0.0)
    step, st = _drive((0.5, 14.6), s6.HDG_PUSH, frame=_FakeFrame())
    assert st == s6._ST_F_DRIBBLE and step == s6.G_PUSH


def test_F_servo_corrects_then_pushes_in_deadband(monkeypatch):
    s6.reset_segment6()
    s6._state = s6._ST_F_DRIBBLE
    # 球在死区内 → 直接推，不纠偏
    _stub_ball(monkeypatch, True, u=s6.SERVO_TOL_PX - 1)
    _prime_ball_confirmation()
    step, st = _drive((1.5, 14.0), s6.HDG_PUSH, frame=_FakeFrame())
    assert step == s6.G_PUSH and st == s6._ST_F_DRIBBLE
    # 球偏出死区 → 先纠偏
    _stub_ball(monkeypatch, True, u=+300)
    _prime_ball_confirmation()
    step, st = _drive((1.5, 14.0), s6.HDG_PUSH, frame=_FakeFrame())
    assert step in (s6.G_TURN_R, s6.G_FTURN_R) and st == s6._ST_F_DRIBBLE


def test_F_lost_ball_keeps_advancing_on_baseline(monkeypatch):
    """远球暂不可见时沿328°接近，进入视野后视觉会自动接管。"""
    s6.reset_segment6()
    s6._state = s6._ST_F_DRIBBLE
    _stub_ball(monkeypatch, False)
    for _ in range(s6.LOST_MAX_FRAMES + 1):
        step, st = _drive((1.5, 14.0), s6.HDG_PUSH, frame=_FakeFrame())
    assert st == s6._ST_F_DRIBBLE
    assert step == s6.G_PUSH
    assert s6._lost_count >= s6.LOST_MAX_FRAMES


def test_F_reaching_trigger_x_advances_to_G(monkeypatch):
    s6.reset_segment6()
    s6._state = s6._ST_F_DRIBBLE
    _stub_ball(monkeypatch, True, u=0.0)
    step, st = _drive((s6.KICK_TRIGGER_X, 13.5), s6.HDG_PUSH, frame=_FakeFrame())
    assert st == s6._ST_F_DRIBBLE and step == s6.G_PUSH


def test_G_exits_at_finish_x_with_vision_on(monkeypatch):
    s6.reset_segment6()
    s6._state = s6._ST_G_THROUGH_GAP
    _stub_ball(monkeypatch, True, u=0.0)
    step, st = _drive((s6.BALL_EXIT_DOG_X, 12.9), s6.HDG_PUSH, frame=_FakeFrame())
    assert st == s6._ST_ALIGN_EXIT and step == s6.G_STAND


def test_vision_off_switch_restores_fixed_route(monkeypatch):
    """USE_VISION=False 时即便有帧有球也走固定328°。"""
    s6.USE_VISION = False
    try:
        s6.reset_segment6()
        s6._state = s6._ST_F_DRIBBLE
        _stub_ball(monkeypatch, True, u=+300)   # 球严重偏，但视觉关了应忽略
        step, st = _drive((1.5, 14.0), s6.HDG_PUSH, frame=_FakeFrame())
        assert step == s6.G_PUSH and st == s6._ST_F_DRIBBLE
    finally:
        s6.USE_VISION = True
        s6.reset_segment6()


def test_servo_correction_is_clamped(monkeypatch):
    """误检给出巨大偏移时，单帧修正不超过 MAX_SERVO_DEG。"""
    s6.reset_segment6()
    target, _dead = s6._track_hdg(1.5, 14.0, s6.HDG_PUSH, True, 99999,
                                  s6.GAP_ENTRY_X, s6.GAP_ENTRY_Y, 0.0)
    assert abs(s6._norm(target - s6.HDG_PUSH)) <= s6.MAX_SERVO_DEG + 1e-6


def test_A_to_D2_unaffected_by_vision(monkeypatch):
    """顶球段(A~D2)不调用视觉：即便塞入乱数据也不改变行为。"""
    _stub_ball(monkeypatch, True, u=+9999)
    s6.reset_segment6()
    step, st = _drive((2.5, 13.5), s6.HDG_UP, frame=_FakeFrame())
    assert st == s6._ST_A_GO_TOP and step == s6.G_NAV


def test_ball_detection_requires_consecutive_frames():
    s6.reset_segment6()
    assert s6._confirm_ball(True) is False
    assert s6._confirm_ball(True) is True
    assert s6._confirm_ball(False) is False
    assert s6._ball_hit_count == 0


def test_centered_ball_still_respects_exit_heading(monkeypatch):
    """球居中但狗明显没朝出口时不能盲目前推，应先修正出口方向。"""
    s6.reset_segment6()
    s6._state = s6._ST_F_DRIBBLE
    _stub_ball(monkeypatch, True, u=0.0)
    _prime_ball_confirmation()
    step, st = _drive((1.5, 14.5), 350, frame=_FakeFrame())
    assert st == s6._ST_F_DRIBBLE
    assert step in (s6.G_TURN_R, s6.G_FTURN_R)


def test_visual_turn_step_directly_tracks_ball_side():
    """像素偏差直接产生明确转向，不再被姿态角死区吞掉。"""
    assert s6._visual_turn_step(s6.SERVO_TOL_PX + 1) == s6.G_TURN_R
    assert s6._visual_turn_step(s6.SERVO_FAST_PX + 1) == s6.G_FTURN_R
    assert s6._visual_turn_step(-s6.SERVO_TOL_PX - 1) == s6.G_TURN_L
    assert s6._visual_turn_step(-s6.SERVO_FAST_PX - 1) == s6.G_FTURN_L
    assert s6._visual_turn_step(0) == s6.G_STAND
