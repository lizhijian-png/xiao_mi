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
