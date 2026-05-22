"""
第一赛段：石径探路

赛道描述：
- 起点趴下站起，沿石板路（4块石板，30cm宽/5cm高/间隔20cm，共约1.8m）前进
- 走完石板路后进入弯道，完成左转（0°→90°）
- 后腿足底离开弯道虚线后，第一赛段结束

坐标系（与 test.py 一致）：
- x: 前进方向（+），石板路段沿 x+ 行进
- y: 横向（+为左），转向后沿 y+ 方向前进穿越弯道虚线
- z: 垂直
- rpy[2]: 机身朝向角（°），0°=x+方向，90°=y+方向

usergait.toml 步态索引（部分）：
- 0: 站立 (mode=12)
- 1: 普通前进 (vel=[0.2,0,0], step_h=0.02)
- 2: 左转 (vel=[0,0,0.25])
- 3: 右转 (vel=[0,0,-0.25])
- 4: 高阻尼趴下
- 7: 左平移
- 8: 右平移
- 9: 高抬腿前进 (vel=[0.15,0,0], step_h=0.12)  ← 石板路专用
- 14: 快速左转 (vel=[0,0,0.6])
- 15: 快速右转 (vel=[0,0,-0.6])
"""

import cv2
import numpy as np


# ─────────────────────────────────────────
# 赛道几何参数（单位：米，需根据实际场地校准）
# ─────────────────────────────────────────
# 石板路: 4块 × 30cm + 3间距 × 20cm = 180cm，机器人沿 x+ 方向行进
STONE_PATH_END_X   = 2.6   # 石板路结束的 x 坐标（进入弯道准备区）
TURN_COMMIT_X      = 3.0  # 开始执行转向的 x 坐标（石板路结束即开始转）

# 弯道退出：转向完成后沿 y+ 方向前进，后腿越过弯道虚线
# 机身长约 0.5m，前腿过线后再走约 0.5m 后腿才离线，故设 0.5m 余量
SEGMENT1_EXIT_Y    = 0.5   # 后腿离开弯道虚线时的 y 坐标（需现场校准）

# 赛道中心线 y 坐标（宽100cm，中心取0）
PATH_CENTER_Y      = 0.0
LATERAL_TOLERANCE  = 0.06  # 允许的横向偏差（m）

# 转向方向：石板路结束后向左转（0° → 90°，进入第二赛段球场）
TURN_TARGET_HEADING = 90   # 目标朝向角（度）


# ─────────────────────────────────────────
# 阶段标志（避免重复触发）
# ─────────────────────────────────────────
_flags = {
    "stone_path_done": False,
    "turn_done":       False,
    "seg1_done":       False,
}

_path_center_y = None   # 首次调用时从实际位置动态捕获


def reset_segment1():
    """重置第一赛段状态（每次比赛前调用）"""
    global _path_center_y
    for k in _flags:
        _flags[k] = False
    _path_center_y = None


# ─────────────────────────────────────────
# 转向辅助函数（与 test.py 保持一致）
# ─────────────────────────────────────────
def _walk_90(rpy):
    """调整朝向至 90°（朝 x 负方向）后前进"""
    if 88 < rpy < 92:
        return 9   # 对准后用高抬腿步态继续走
    elif 92 <= rpy < 95:
        return 3   # 微右转
    elif 95 <= rpy < 270:
        return 15  # 快速右转
    elif 270 <= rpy <= 360 or 0 <= rpy < 85:
        return 14  # 快速左转
    else:
        return 2   # 微左转


def _lateral_correction(y):
    """
    根据横向偏移返回纠偏步态索引。
    返回 0 表示无需纠偏。
    """
    offset = y - PATH_CENTER_Y
    if offset > LATERAL_TOLERANCE:
        return 7   # 偏左（y>0）→ 向右平移校正
    if offset < -LATERAL_TOLERANCE:
        return 8   # 偏右（y<0）→ 向左平移校正
    return 0


# ─────────────────────────────────────────
# 视觉辅助：检测黄色边沿中心偏移
# ─────────────────────────────────────────
def detect_yellow_line_offset(frame):
    """
    识别相机图像中黄色边沿，计算机器人横向偏移。

    Args:
        frame: BGR 图像（来自相机）

    Returns:
        float: 像素偏移量（正值=偏右，负值=偏左），无法识别时返回 0.0
    """
    h, w = frame.shape[:2]
    # 只看下半部分（地面更清晰）
    roi = frame[h // 2:, :]

    hsv = cv2.cvtColor(roi, cv2.COLOR_BGR2HSV)
    # 黄色边沿 HSV 范围（RGB: 255,255,0）
    lower = np.array([20, 100, 100])
    upper = np.array([35, 255, 255])
    mask = cv2.inRange(hsv, lower, upper)

    kernel = np.ones((5, 5), np.uint8)
    mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)

    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    if not contours:
        return 0.0

    # 找左右两侧的黄线轮廓
    left_cx, right_cx = [], []
    for cnt in contours:
        if cv2.contourArea(cnt) < 300:
            continue
        M = cv2.moments(cnt)
        if M["m00"] == 0:
            continue
        cx = int(M["m10"] / M["m00"])
        if cx < w // 2:
            left_cx.append(cx)
        else:
            right_cx.append(cx)

    if left_cx and right_cx:
        center = (min(left_cx) + max(right_cx)) / 2
        return center - w / 2  # 正：偏右，负：偏左
    return 0.0


# ─────────────────────────────────────────
# 第一赛段主控制函数
# ─────────────────────────────────────────
def segment1_control(position, gait_mode, rpy, frame=None):
    """
    第一赛段（石径探路）控制逻辑。

    在主循环中每次调用，返回 Robot_Ctrl.num 应设置的步态索引。

    Args:
        position:  [x, y, z]   来自 Pos_msg.position
        gait_mode: [gait_id, mode]  来自 Gait_msg.gait_mode
        rpy:       float  机身朝向角（度），来自 Pos_msg.rpy[2]
        frame:     np.ndarray or None  当前相机帧（可选，用于视觉纠偏）

    Returns:
        int: 步态索引（对应 usergait.toml 中的 step 序号）
              -1 表示第一赛段已结束，请切换到第二赛段
    """
    global _path_center_y
    x, y, _ = position
    gait, mode = gait_mode

    # 首次调用时捕获实际路径中心 y（复用 2025 back_left 动态基准方案）
    if _path_center_y is None:
        _path_center_y = y

    # 步态切换中，等待
    if (gait == 0 and mode == 0) or (gait == 1 and mode == 9):
        return 0

    # ── 阶段1：石板路行进 ─────────────────────
    if not _flags["stone_path_done"]:
        # 视觉横向纠偏（有相机帧时使用）
        if frame is not None:
            px_offset = detect_yellow_line_offset(frame)
            if px_offset > 40:
                return 8   # 偏右 → 向右平移校正
            if px_offset < -40:
                return 7   # 偏左 → 向左平移校正
        else:
            # 无视觉时靠位置纠偏（基准为首次捕获的 y）
            corr = _lateral_correction(y - _path_center_y + PATH_CENTER_Y)
            if corr:
                return corr

        if x >= STONE_PATH_END_X:
            _flags["stone_path_done"] = True
            return 9  # 继续高抬腿步态进入过渡区

        # 石板路专用：高抬腿前进（step_height=0.12 可跨越5cm石板）
        return 9

    # ── 阶段2：弯道转向 ──────────────────────
    elif not _flags["turn_done"]:
        if x >= TURN_COMMIT_X:
            # 开始转向，目标朝向 90°（左转）
            heading_diff = rpy - TURN_TARGET_HEADING
            if abs(heading_diff) < 3:
                # 朝向已对准，沿 y+ 前进穿越弯道虚线
                # 转向后前进用 y 坐标判断后腿是否离开虚线
                if y >= SEGMENT1_EXIT_Y:
                    _flags["turn_done"] = True
                    _flags["seg1_done"] = True
                    return -1  # 第一赛段完成
                return 1  # 普通前进穿越弯道虚线
            else:
                return _walk_90(rpy)  # 调整朝向
        else:
            # 靠近弯道，继续高抬腿前进
            return 9

    # ── 已完成 ────────────────────────────────
    else:
        return -1  # 第一赛段已结束


# ─────────────────────────────────────────
# 独立测试入口（在实际硬件上跑单赛段时使用）
# ─────────────────────────────────────────
if __name__ == "__main__":
    import sys
    sys.path.insert(0, '/usr/local/lib/python3.8/site-packages')
    sys.path.append("./lcm")

    import time
    import threading
    import lcm
    from Robot_Ctrl import Robot_Ctrl
    from Msg_receive import Pos_msg, Gait_msg
    from user_pub import user_pub
    from robot_control_cmd_lcmt import robot_control_cmd_lcmt
    import rclpy
    from rclpy.node import Node
    from sensor_msgs.msg import Image
    from cv_bridge import CvBridge
    from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy

    # ─── 相机订阅 ────────────────────────────
    class CameraNode(Node):
        def __init__(self):
            super().__init__("seg1_camera")
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

    # ─── 主循环 ──────────────────────────────
    def main():
        reset_segment1()

        lcm_cmd = lcm.LCM("udpm://239.255.76.67:7671?ttl=255")
        cmd_msg = robot_control_cmd_lcmt()
        data_lock = threading.Lock()

        user_pub()
        my_ctrl = Robot_Ctrl()
        pos_msg = Pos_msg(data_lock)
        gait_msg = Gait_msg(data_lock)

        # 启动 ROS2 相机节点（可选）
        rclpy.init(args=None)
        cam_node = CameraNode()

        ctrl_thread = threading.Thread(target=my_ctrl.run, daemon=True)
        rec_thread  = threading.Thread(target=pos_msg.run, daemon=True)
        gait_thread = threading.Thread(target=gait_msg.run, daemon=True)
        ros_thread  = threading.Thread(
            target=lambda: rclpy.spin(cam_node), daemon=True
        )

        ctrl_thread.start()
        time.sleep(4)                          # 等待步态控制器就绪

        # 起步：先微调朝向再出发
        my_ctrl.num = 2                        # 左转微调对正
        my_ctrl.msg.life_count = (my_ctrl.msg.life_count + 1) % 127
        time.sleep(0.5)

        rec_thread.start()
        gait_thread.start()
        ros_thread.start()

        print("=== 第一赛段：石径探路 开始 ===")
        try:
            while True:
                with data_lock:
                    pos  = list(pos_msg.position)
                    gait = list(gait_msg.gait_mode)
                    yaw  = pos_msg.rpy[2]

                frame = cam_node.frame       # 最新相机帧（可能为 None）
                step  = segment1_control(pos, gait, yaw, frame)

                if step == -1:
                    print("=== 第一赛段完成，进入第二赛段 ===")
                    break

                my_ctrl.num = step
                my_ctrl.msg.life_count = (my_ctrl.msg.life_count + 1) % 127

                print(
                    f"pos={[round(v,2) for v in pos]}  "
                    f"yaw={yaw:.1f}°  step={step}"
                )

                if step == 0:
                    time.sleep(4)

        except KeyboardInterrupt:
            pass
        finally:
            cmd_msg.mode = 7
            cmd_msg.gait_id = 0
            cmd_msg.duration = 0
            cmd_msg.life_count += 1
            lcm_cmd.publish("robot_control_cmd", cmd_msg.encode())
            rclpy.shutdown()
            sys.exit()

    main()
