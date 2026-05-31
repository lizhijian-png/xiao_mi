"""
test6.py — 赛段6独立测试
使用方式：
  1. 在 Gazebo 里手动把机器人拖到第六赛段入口 (2.9, 13.5)
     （赛段5独木桥终点前50cm跳下落点；状态机 A 阶段会自行转向 90° 贴顶墙上行）
  2. 运行本脚本
"""
import sys
sys.path.insert(0, '/usr/local/lib/python3.8/site-packages')
sys.path.append("./lcm")

import lcm
import time
import threading
from Robot_Ctrl import Robot_Ctrl
from Msg_receive import Pos_msg, Gait_msg
from user_pub import user_pub
from robot_control_cmd_lcmt import robot_control_cmd_lcmt
from segment6 import segment6_control, reset_segment6


def main():
    lcm_cmd = lcm.LCM("udpm://239.255.76.67:7671?ttl=255")
    cmd_msg = robot_control_cmd_lcmt()
    data_lock = threading.Lock()

    try:
        user_pub()
        reset_segment6()

        my_ctrl = Robot_Ctrl()
        pos_msg = Pos_msg(data_lock)
        gait_msg = Gait_msg(data_lock)

        ctrl_thread = threading.Thread(target=my_ctrl.run, daemon=True)
        rec_thread  = threading.Thread(target=pos_msg.run, daemon=True)
        gait_thread = threading.Thread(target=gait_msg.run, daemon=True)

        ctrl_thread.start()
        time.sleep(4)
        my_ctrl.num = 2
        my_ctrl.msg.life_count = (my_ctrl.msg.life_count + 1) % 127
        time.sleep(0.5)
        rec_thread.start()
        gait_thread.start()

        print("=== 第六赛段：撷金建功 开始 ===")

        def print_worker():
            while True:
                from segment6 import _state as s6
                print(
                    f"pos={[round(v,2) for v in pos_msg.position]}  "
                    f"yaw={pos_msg.rpy[2]:.1f}°  "
                    f"step={my_ctrl.num}  state={s6}"
                )
                time.sleep(0.2)

        threading.Thread(target=print_worker, daemon=True).start()

        while True:
            with data_lock:
                num = segment6_control(
                    pos_msg.position,
                    gait_msg.gait_mode,
                    pos_msg.rpy[2]
                )
            if num == -1:
                print("=== 赛段6完成 ===")
                my_ctrl.num = 4
                my_ctrl.msg.life_count = (my_ctrl.msg.life_count + 1) % 127
                time.sleep(3)
                break
            my_ctrl.num = num
            my_ctrl.msg.life_count = (my_ctrl.msg.life_count + 1) % 127
            if num == 0:
                time.sleep(4)

    except KeyboardInterrupt:
        pass
    finally:
        cmd_msg.mode = 7
        cmd_msg.gait_id = 0
        cmd_msg.duration = 0
        cmd_msg.life_count += 1
        lcm_cmd.publish("robot_control_cmd", cmd_msg.encode())
    sys.exit()


if __name__ == '__main__':
    main()
