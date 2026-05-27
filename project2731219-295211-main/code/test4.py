# 添加lcm模块
import sys
sys.path.insert(0, '/usr/local/lib/python3.8/site-packages')
sys.path.append("./lcm")

import lcm
import time
import threading
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy

from Robot_Ctrl import Robot_Ctrl
from Msg_receive import Pos_msg, Gait_msg
from user_pub import user_pub
from robot_control_cmd_lcmt import robot_control_cmd_lcmt
from segment4 import segment4_control, reset_segment4
from segment5 import segment5_control, reset_segment5


flags = {
    "ENDING_FLAG4": False,
    "ENDING_FLAG5": False,
}


def select_step_based_on_position(position, gait_mode, rpy, frame=None):
    if not flags["ENDING_FLAG4"]:
        step = segment4_control(position, gait_mode, rpy, frame=frame)
        if step == -1:
            flags["ENDING_FLAG4"] = True
            return 0
        return step

    if not flags["ENDING_FLAG5"]:
        step = segment5_control(position, gait_mode, rpy)
        if step == -1:
            flags["ENDING_FLAG5"] = True
            return 0
        return step

    return 4


class CameraNode(Node):
    def __init__(self):
        super().__init__("test4_camera")
        self.bridge = CvBridge()
        self.frame = None
        qos = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=1,
        )
        self.create_subscription(Image, "/rgb_camera/image_raw", self._cb, qos)

    def _cb(self, msg):
        self.frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")


def main():
    lcm_cmd = lcm.LCM("udpm://239.255.76.67:7671?ttl=255")
    cmd_msg = robot_control_cmd_lcmt()
    data_lock = threading.Lock()

    try:
        user_pub()
        reset_segment4()
        reset_segment5()

        my_ctrl = Robot_Ctrl()
        pos_msg = Pos_msg(data_lock)
        gait_msg = Gait_msg(data_lock)

        ctrl_thread = threading.Thread(target=my_ctrl.run)
        rec_thread = threading.Thread(target=pos_msg.run)
        gait_thread = threading.Thread(target=gait_msg.run)

        rclpy.init(args=None)
        cam_node = CameraNode()
        ros_thread = threading.Thread(target=lambda: rclpy.spin(cam_node), daemon=True)

        ctrl_thread.start()
        time.sleep(1)
        rec_thread.start()
        gait_thread.start()
        ros_thread.start()

        def print_worker():
            while True:
                from segment4 import _state as seg4_state, _obstacle_idx, _target_idx
                print(
                    f"当前位置: {pos_msg.position} 机身朝向{pos_msg.rpy[2]} "
                    f"选择:{my_ctrl.num} seg4={seg4_state}/obs{_obstacle_idx}/target{_target_idx}"
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
                    cam_node.frame,
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
