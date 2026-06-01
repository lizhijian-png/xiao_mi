# 添加lcm模块
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
from segment5 import segment5_control, reset_segment5
from segment6 import segment6_control, reset_segment6


flags = {
    "ENDING_FLAG5": False,
    "ENDING_FLAG6": False,
}


def select_step_based_on_position(position, gait_mode, rpy):
    if not flags["ENDING_FLAG5"]:
        step = segment5_control(position, gait_mode, rpy)
        if step == -1:
            flags["ENDING_FLAG5"] = True
            # 第五段完成后不要再额外站立等待，直接衔接第六段第一条指令。
            # 否则主循环遇到 step=0 会 sleep，容易在跳跃 mode=16 后卡在第六段起点。
            step = segment6_control(position, gait_mode, rpy, frame=None)
            if step == -1:
                flags["ENDING_FLAG6"] = True
                return 4
            return step
        return step

    if not flags["ENDING_FLAG6"]:
        step = segment6_control(position, gait_mode, rpy, frame=None)
        if step == -1:
            flags["ENDING_FLAG6"] = True
            return 4
        return step

    return 4


def main():
    lcm_cmd = lcm.LCM("udpm://239.255.76.67:7671?ttl=255")
    cmd_msg = robot_control_cmd_lcmt()
    data_lock = threading.Lock()

    try:
        user_pub()
        reset_segment5()
        reset_segment6()

        my_ctrl = Robot_Ctrl()
        pos_msg = Pos_msg(data_lock)
        gait_msg = Gait_msg(data_lock)

        ctrl_thread = threading.Thread(target=my_ctrl.run)
        rec_thread = threading.Thread(target=pos_msg.run)
        gait_thread = threading.Thread(target=gait_msg.run)

        ctrl_thread.start()
        time.sleep(1)
        rec_thread.start()
        gait_thread.start()

        def print_worker():
            while True:
                from segment5 import _state as seg5_state
                from segment6 import _state as seg6_state
                if not flags["ENDING_FLAG5"]:
                    active_state = f"seg5={seg5_state}"
                elif not flags["ENDING_FLAG6"]:
                    active_state = f"seg6={seg6_state}"
                else:
                    active_state = "DONE"
                print(
                    f"当前位置: {pos_msg.position} 机身朝向{pos_msg.rpy[2]} "
                    f"选择:{my_ctrl.num} {active_state}"
                )
                print(f"{gait_msg.gait_mode}")
                time.sleep(0.2)

        thread = threading.Thread(target=print_worker)
        thread.start()

        while True:
            with data_lock:
                num = select_step_based_on_position(
                    pos_msg.position,
                    gait_msg.gait_mode,
                    pos_msg.rpy[2],
                )
            my_ctrl.num = num
            my_ctrl.msg.life_count = (my_ctrl.msg.life_count + 1) % 127
            if num == 0:
                print("站立")
                time.sleep(1)

    except KeyboardInterrupt:
        cmd_msg.mode = 7
        cmd_msg.gait_id = 0
        cmd_msg.duration = 0
        cmd_msg.life_count += 1
        lcm_cmd.publish("robot_control_cmd", cmd_msg.encode())
        pass
    sys.exit()


if __name__ == '__main__':
    main()
