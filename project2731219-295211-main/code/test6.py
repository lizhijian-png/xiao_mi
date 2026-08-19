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
from segment6 import segment6_control, reset_segment6, USE_VISION
from ball_camera import BallCamera


def main():
    lcm_cmd = lcm.LCM("udpm://239.255.76.67:7671?ttl=255")
    cmd_msg = robot_control_cmd_lcmt()
    data_lock = threading.Lock()
    cam = None

    try:
        if USE_VISION:
            cam = BallCamera().start()
            if not cam.wait_ready(15.0):
                raise RuntimeError(f"RGB相机没有真实图像，拒绝开始第六赛段：{cam.diagnostics()}")
            print(f"相机就绪，图像话题={cam.active_topic()}，诊断={cam.diagnostics()}")

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
                import segment6 as s6m
                # 追球段额外打印视觉状态：标定 KP_PIX_TO_DEG / GAP_BIAS_W 全靠看这几个值
                vis = ""
                if s6m.USE_VISION:
                    f = cam.frame() if cam is not None else None
                    ok, u, r = s6m.find_ball(f)
                    diag = cam.diagnostics()
                    vis = (f"  camera={diag['active_topic']} frames={diag['frame_count']} "
                           f"age={diag['frame_age']} ball={'Y' if ok else 'N'} "
                           f"u={u:+.0f} r={r:.0f} lost={s6m._lost_count} "
                           f"cam_error={diag['error']}")
                print(
                    f"pos={[round(v,2) for v in pos_msg.position]}  "
                    f"yaw={pos_msg.rpy[2]:.1f}°  "
                    f"step={my_ctrl.num}  state={s6m._state}{vis}"
                )
                time.sleep(0.2)

        threading.Thread(target=print_worker, daemon=True).start()

        while True:
            frame = cam.frame() if cam is not None else None
            with data_lock:
                num = segment6_control(
                    pos_msg.position,
                    gait_msg.gait_mode,
                    pos_msg.rpy[2],
                    frame
                )
            if num == -1:
                print("=== 赛段6完成 ===")
                my_ctrl.num = 4
                my_ctrl.msg.life_count = (my_ctrl.msg.life_count + 1) % 127
                time.sleep(3)
                break
            my_ctrl.num = num
            my_ctrl.msg.life_count = (my_ctrl.msg.life_count + 1) % 127
            # 视觉状态机需要持续取帧；站立指令也只等待一个控制周期，不能阻塞4秒。
            time.sleep(0.2)

    except KeyboardInterrupt:
        pass
    finally:
        if cam is not None:
            cam.stop()
        cmd_msg.mode = 7
        cmd_msg.gait_id = 0
        cmd_msg.duration = 0
        cmd_msg.life_count += 1
        lcm_cmd.publish("robot_control_cmd", cmd_msg.encode())
    sys.exit()


if __name__ == '__main__':
    main()
