"""第六赛段独立真机测试：固定侧顶、RGB追球、半蹲推球与趴下。"""

import segment6
from segment_test_runner import run_segment_test


def main():
    run_segment_test(
        segment_name="赛段6 撷金建功",
        reset=segment6.reset_segment6,
        control=segment6.segment6_control,
        state_getter=lambda: (
            f"{segment6._state}/lost={segment6._lost_count}/locked={segment6._ever_locked}"
        ),
        use_camera=segment6.USE_VISION,
        completion_gait=segment6.G_LAY,
        placement_hint="机器狗放在赛段6入口(2.8, 12.7)附近；足球按比赛位置摆放。",
    )


if __name__ == "__main__":
    main()
