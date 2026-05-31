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
