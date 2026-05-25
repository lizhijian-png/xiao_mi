#添加lcm模块
import sys
sys.path.insert(0, '/usr/local/lib/python3.8/site-packages')
sys.path.append("./lcm")

import lcm
import time
import toml
import copy
import math
import threading
import os
import numpy as np
import cv2
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy
from Robot_Ctrl import Robot_Ctrl
from Msg_receive import Pos_msg, Gait_msg
from user_pub import user_pub
from robot_control_cmd_lcmt import robot_control_cmd_lcmt
from identify import arrow,yellow_wait,yellow_light
from segment1 import segment1_control, reset_segment1
from segment2 import segment2_control, reset_segment2
from segment3 import segment3_control, reset_segment3
from segment5 import segment5_control, reset_segment5
flags ={
    "ENDING_FLAG1" : False,# 赛段1标志
    "ENDING_FLAG2" : False,# 赛段2标志
    "ENDING_FLAG3" : False,# A->B过S弯标志
    "ENDING_FLAG4" : False,# 赛段4标志
    "ENDING_FLAG5" : False,# 赛段5标志
    "ENDING_FLAG6" : False,# 赛段6标志
    "ENDING_FLAG7" : False,# B->A过S弯标志
    "ENDING_FLAG8" : False,# 赛段8标志
    "ENDING_FLAG9" : False,# 赛段9标志
    "ENDING_FLAG10" : False,# 结束标志
    # S弯标志
    "S_ENDING_FLAG1" : False,
    "S_ENDING_FLAG2" : False,
    "S_ENDING_FLAG3" : False,
    "S_ENDING_FLAG4" : False,
    "S_ENDING_FLAG5" : False,
    "S_ENDING_FLAG6" : False,
    # S弯回程标志
    "BACK_S_ENDING_FLAG1" : False,
    "BACK_S_ENDING_FLAG2" : False,
    "BACK_S_ENDING_FLAG3" : False,
    "BACK_S_ENDING_FLAG4" : False,
    "BACK_S_ENDING_FLAG5" : False,
    "BACK_S_ENDING_FLAG6" : False,
    # S弯方向就绪标志
    "is_s_direction_ready0" : False,
    "is_s_direction_ready1" : False,
    "is_s_direction_ready2" : False,
    "is_s_direction_ready3" : False,
    # 箭头识别标志
    "ARROW_executed" : False,
    # 黄灯识别标志
    "LIGHT_executed" : False,
    # 黄灯停止标志
    "WAIT_executed" : False,
}
results = {
    "ARROW" : 0,# 箭头识别结果  朝左：1     朝右：2
    "yellow_light": 0,# 黄灯识别标志    未识别：0   已识别：1
    "ready_wait" : 0,# 黄灯停止标志     未等待：0   已等待：1
}

# 根据位置和标志决定狗子执行的动作
def select_step_based_on_position(position, gait_mode, rpy, frame=None):
    global flags,results
    x, y, z = position
    gait, mode = gait_mode
    rpy=rpy
    if gait==0 and mode==0 or gait ==1 and mode==9:
        return 0
    else:
        # 赛段1：石径探路
        if flags["ENDING_FLAG1"] == False:
            step = segment1_control(position, gait_mode, rpy, frame=None)
            if step == -1:
                flags["ENDING_FLAG1"] = True
                return 1  # 赛段1完成时机器人朝向已为90°，直接前进进入赛段2
            return step

        # 赛段2：荒野寻珠
        elif flags["ENDING_FLAG2"] == False:
            step = segment2_control(position, gait_mode, rpy, frame=frame)
            if step == -1:
                flags["ENDING_FLAG2"] = True
                return walk_90(rpy)  # 进入第三赛段（S弯）
            return step

        # 赛段3：S型弯道
        elif flags["ENDING_FLAG3"] == False:
            step = segment3_control(position, gait_mode, rpy)
            if step == -1:
                flags["ENDING_FLAG3"] = True
            return step if step != -1 else walk_90(rpy)

        # 赛段3出口：朝 90° 前进到 y >= 7.0
        elif flags["ENDING_FLAG4"] == False:
            if y >= 7.0:
                flags["ENDING_FLAG4"] = True
                return 0
            return walk_90(rpy)

        # 赛段5：螺旋爬坡+不平整路面+跳下
        elif flags["ENDING_FLAG5"] == False:
            step = segment5_control(position, gait_mode, rpy)
            if step == -1:
                flags["ENDING_FLAG5"] = True
                return 0
            return step

        # 进入S弯回程
        elif (y>0.5 or (x>1.2 and y>0.1)) and flags["ENDING_FLAG7"] == False:
            flags["ENDING_FLAG6"] = True
            return pass_s_back(position,rpy)

        # 结束
        elif flags["ENDING_FLAG10"] == False:
            flags["ENDING_FLAG9"] = True
            if rpy<175:
                return 19
            elif x>-0.15 :
                return walk_180(rpy)
            else:
                flags["ENDING_FLAG10"] = True
                return 4
        else:
            return 4
        
        
# 转向0°方向前进
def walk_0(rpy):
    if rpy<2 or rpy>358:
        return 1
    elif 5<=rpy<180:
        return 15 # 快右转
    elif 180<=rpy<355:
        return 14 # 快左转
    elif 2<=rpy<5:
        return 3 # 右转
    else:
        return 2 # 左转

# 转向90°方向前进
def walk_90(rpy):
    if 88<rpy<92:
        return 1
    elif 92<=rpy<95:
        return 3 # 右转
    elif 95<=rpy<270:
        return 15 # 快右转
    elif 270<=rpy<=360 or 0<=rpy<85:
        return 14 # 快左转
    else:
        return 2 # 左转
    
# 转向90°方向前进
def walk_90_fast(rpy):
    if 88<rpy<92:
        return 28
    elif 92<=rpy<95:
        return 3 # 右转
    elif 95<=rpy<270:
        return 15 # 快右转
    elif 270<=rpy<=360 or 0<=rpy<85:
        return 14 # 快左转
    else:
        return 2 # 左转
    
# 转向180°方向前进
def walk_180(rpy):
    if 178<rpy<182:
        return 1
    elif 182<=rpy<185:
        return 3 # 右转
    elif 185<=rpy<=360:
        return 15 # 快右转
    elif 0<=rpy<=175:
        return 14 # 快左转
    else:
        return 2 # 左转

# 转向270°方向前进
def walk_270(rpy):
    if 268<rpy<272:
        return 1
    elif 265<rpy<=268:
        return 2 # 左转
    elif 90<=rpy<=265:
        return 14 # 快左转
    elif 0<=rpy<90 or 275<=rpy<=360:
        return 15 # 快右转
    else:
        return 3 # 右转
    
# 转向270°方向前进
def walk_270_fast(rpy):
    if 268<rpy<272:
        return 28
    elif 265<rpy<=268:
        return 2 # 左转
    elif 90<=rpy<=265:
        return 14 # 快左转
    elif 0<=rpy<90 or 275<=rpy<=360:
        return 15 # 快右转
    else:
        return 3 # 右转
        
# 过S弯，并在最后一个S弯识别箭头方向（朝右）
def pass_s_and_identify_arrow(position,rpy):
    global flags,results
    x, y, z = position
    rpy=rpy
    if x< 1.47 and flags["S_ENDING_FLAG1"] == False:
        return walk_0(rpy)
    elif 0.95 <x and flags["S_ENDING_FLAG2"] == False:
        flags["S_ENDING_FLAG1"] = True
        distance = math.sqrt((x - 1.47)**2 + (y - 0.95)**2)
        # print(distance)
        target_rpy = math.degrees(math.atan2(y-0.95,x-1.47))+180
        target_rpy -= 90
        delta_rpy = rpy - target_rpy
        # print(delta_rpy)
        if(delta_rpy>5 and delta_rpy<80):
            return 3
        elif(delta_rpy<-5 and delta_rpy>-80):
            return 2
        if(distance<0.72 and distance>=0.68):
            return 21
        elif(distance <0.68):
            return 8
        elif(distance>0.74 and distance<=0.78 ):
            return 20
        elif(distance>0.78):
            return 7
        return 25
    elif x<=0.95 and flags["S_ENDING_FLAG3"] == False:
        flags["S_ENDING_FLAG2"] = True
        if flags["is_s_direction_ready0"] == False:
            if 222<rpy<=228:
                flags["is_s_direction_ready0"] = True
            elif 45<rpy<=222:
                return 2  # 左转
            else:
                return 3  # 右转
        distance = math.sqrt((x - 0.45)**2 + (y - 2)**2)
        # print(distance)
        target_rpy = math.degrees(math.atan2(y-2,x-0.45))+180
        if(target_rpy>=0 and target_rpy<270):
            target_rpy+=90
        else:
            target_rpy-=270
        delta_rpy = rpy-target_rpy
        if(delta_rpy>5 and delta_rpy<80):
            return 3
        elif(delta_rpy<-5 and delta_rpy>-80):
            return 2
        if(distance<0.72 and distance>=0.68):
            return 23
        elif(distance<0.68):
            return 7
        elif(distance>0.74 and distance<=0.78):
            return 22
        elif(distance>0.78):
            return 8
        return 24
    elif (0.95 < x and y < 3) or 1.5 < x and flags["S_ENDING_FLAG4"] == False:
        flags["S_ENDING_FLAG3"] = True
        if flags["is_s_direction_ready1"] == False:
            if 312 < rpy < 318:
                flags["is_s_direction_ready1"] = True
            elif 135 < rpy <= 312:
                return 2  # 左转
            else:
                return 3  # 右转
        distance = math.sqrt((x - 1.47) ** 2 + (y - 3.05) ** 2)
        target_rpy = math.degrees(math.atan2(y - 3.05, x - 1.47)) + 180
        if target_rpy >= 45 and target_rpy < 90:
            target_rpy += 270
        else:
            target_rpy -= 90
        delta_rpy = rpy - target_rpy
        if delta_rpy > 5 and delta_rpy < 50:
            return 3
        elif delta_rpy < -5 and delta_rpy > -50:
            return 2
        # print(distance)
        if distance < 0.72 and distance >= 0.68:
            return 21
        elif distance < 0.68:
            return 8
        elif distance > 0.74 and distance <= 0.78:
            return 20
        elif distance > 0.78:
            return 7
        return 25
    elif y < 4.55 and flags["S_ENDING_FLAG5"] == False:
        flags["S_ENDING_FLAG4"] = True
        distance = math.sqrt((x - 1.47) ** 2 + (y - 4.55) ** 2)
        target_rpy = math.degrees(math.atan2(y - 4.55, x - 1.47)) + 180
        target_rpy += 90
        delta_rpy = rpy - target_rpy
        if delta_rpy > 5 and delta_rpy < 50:
            return 3
        elif delta_rpy < -5 and delta_rpy > -50:
            return 2
        # print(distance)
        if distance > 0.74 and distance <= 0.78:
            return 22
        elif distance > 0.78:
            return 8
        elif distance < 0.72 and distance >= 0.68:
            return 23
        elif distance < 0.68:
            return 7
        return 24
    elif y < 4.9 and flags["S_ENDING_FLAG6"] == False:
        flags["S_ENDING_FLAG5"] = True
        if results["ARROW"] == 0:
            if flags["ARROW_executed"] == False:
                flags["ARROW_executed"] = True
                return 0
            else:
                results["ARROW"] = arrow()
                return 24
        else:
            return 24
    else:
        flags["S_ENDING_FLAG6"] = True
        return walk_90(rpy)

# 回S弯
def pass_s_back(position,rpy):
    global flags,results
    x, y, z = position
    rpy=rpy
    if y> 4.55 and flags["BACK_S_ENDING_FLAG1"] == False:
        return walk_270(rpy)
    elif x<1.5 and flags["BACK_S_ENDING_FLAG2"] == False:
        flags["BACK_S_ENDING_FLAG1"] = True
        distance = math.sqrt((x - 1.47)**2 + (y - 4.55)**2)
        #print(distance)
        target_rpy = math.degrees(math.atan2(y - 4.55, x - 1.47)) + 180
        target_rpy+=270
        delta_rpy = rpy - target_rpy
        if delta_rpy > 5 and delta_rpy < 50:
            return 3
        elif delta_rpy < -5 and delta_rpy > -50:
            return 2
        if(0.68 <= distance < 0.72):
            return 21
        elif(distance<0.68):
            return 8
        elif(0.74 < distance <= 0.78):
            return 20
        elif(distance>0.78):
            return 7
        return 25
    elif y>3 or x>0.95  and flags["BACK_S_ENDING_FLAG3"] == False:
        flags["BACK_S_ENDING_FLAG2"] = True
        distance = math.sqrt((x - 1.47)**2 + (y - 3.05)**2)
        target_rpy = math.degrees(math.atan2(y - 3.05, x - 1.47)) + 180
        target_rpy+=90
        delta_rpy = rpy - target_rpy
        if delta_rpy > 5 and delta_rpy < 50:
            return 3
        elif delta_rpy < -5 and delta_rpy > -50:
            return 2
        if(distance<0.72 and distance>=0.68):
            return 23
        elif(distance<0.68):
            return 7
        elif(distance>0.74 and distance <=0.78):
            return 22
        elif(distance>0.78):
            return 8
        return 24
    elif x<=0.95 and flags["BACK_S_ENDING_FLAG4"] == False:
        flags["BACK_S_ENDING_FLAG3"] = True
        if flags["is_s_direction_ready2"] == False:
            if 132<rpy<=138:
                flags["is_s_direction_ready2"] = True
            elif 138<rpy<=315:
                return 3  
            else:
                return 2  
        distance = math.sqrt((x - 0.45)**2 + (y - 2)**2)
        target_rpy = math.degrees(math.atan2(y-2,x-0.45))+180
        if(target_rpy>0 and target_rpy<=90):
            target_rpy+=270
        else:
            target_rpy-=90
        delta_rpy = rpy-target_rpy
        if(delta_rpy>5 and delta_rpy<50):
            return 3
        elif(delta_rpy<-5 and delta_rpy>-50):
            return 2
        #print(distance)
        if(distance>0.74 and distance<=0.78):
            return 20
        elif(distance>0.78):
            return 7
        elif(distance<0.72 and distance>=0.68):
            return 21
        elif(distance<0.68):
            return 8
        return 25
    elif y>0.95 or x>1.47 and flags["BACK_S_ENDING_FLAG5"] == False:
        flags["BACK_S_ENDING_FLAG4"] = True
        if flags["is_s_direction_ready3"] == False:
            if 42<rpy<=48:
                flags["is_s_direction_ready3"] = True
            elif 48<rpy<=225:
                return 3  
            else:
                return 2  
        distance = math.sqrt((x - 1.47)**2 + (y - 0.95)**2)
        target_rpy = math.degrees(math.atan2(y-0.95,x-1.47))+180
        if(target_rpy>90 and target_rpy<=270):
            target_rpy+=90
        else:
            target_rpy-=270
        delta_rpy = rpy - target_rpy
        if(delta_rpy>5 and delta_rpy<50):
            return 3
        elif(delta_rpy<-5 and delta_rpy>-50):
            return 2
        #print(distance)
        if(distance>0.74 and distance<=0.78):
            return 22
        elif(distance > 0.78):
            return 8
        elif(distance<0.72 and distance>=0.68):
            return 23
        elif(distance<0.68):
            return 7
        return 24
    else:
        return walk_180(rpy)


###########################################################################################
class CameraNode(Node):
    def __init__(self):
        super().__init__("test_camera")
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
    global turn_count
    turn_count = 0
    lcm_cmd = lcm.LCM("udpm://239.255.76.67:7671?ttl=255")
    cmd_msg = robot_control_cmd_lcmt()    
    data_lock = threading.Lock()
    
    try:
        user_pub()
        reset_segment1()
        reset_segment2()
        reset_segment3()
        reset_segment5()
        my_ctrl = Robot_Ctrl()
        pos_msg = Pos_msg(data_lock)
        gait_msg = Gait_msg(data_lock)
        ctrl_thread = threading.Thread(target=my_ctrl.run)
        rec_thread = threading.Thread(target=pos_msg.run)
        gait_thread= threading.Thread(target=gait_msg.run)

        rclpy.init(args=None)
        cam_node = CameraNode()
        ros_thread = threading.Thread(target=lambda: rclpy.spin(cam_node), daemon=True)

        ctrl_thread.start()
        time.sleep(4)
        my_ctrl.num = 2# 起步左转一下调正机位
        my_ctrl.msg.life_count =(my_ctrl.msg.life_count + 1) % 127
        time.sleep(0.5)
        rec_thread.start()
        gait_thread.start()
        ros_thread.start()
        def print_worker():
            while True:
                from segment2 import _state as seg2_state, _target_idx as seg2_row
                print(f"当前位置: {pos_msg.position} 机身朝向{pos_msg.rpy[2]} 箭头识别结果{results['ARROW']} 选择:{my_ctrl.num} seg2={seg2_state}/row{seg2_row}")
                print(f"{gait_msg.gait_mode}")
                time.sleep(0.2)
        thread = threading.Thread(target=print_worker)
        thread.start()

        while True:
            # time.sleep(0.2)
            with data_lock:
                num = select_step_based_on_position(pos_msg.position, gait_msg.gait_mode, pos_msg.rpy[2], cam_node.frame)
                # print(f"当前位置: {pos_msg.position} 机身朝向{pos_msg.rpy[2]} 箭头识别结果{results['ARROW']} 选择:{num}")
                # print(f"{gait_msg.gait_mode}")
            my_ctrl.num = num
            my_ctrl.msg.life_count =(my_ctrl.msg.life_count + 1) % 127
            if num==0:
                print("站立")
                time.sleep(4)           

    except KeyboardInterrupt:
        cmd_msg.mode = 7  # PureDamper before KeyboardInterrupt
        cmd_msg.gait_id = 0
        cmd_msg.duration = 0
        cmd_msg.life_count += 1
        lcm_cmd.publish("robot_control_cmd", cmd_msg.encode())
        pass
    sys.exit()


if __name__ == '__main__':
    main()
