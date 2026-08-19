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
from Robot_Ctrl import Robot_Ctrl
from Msg_receive import Pos_msg, Gait_msg
from user_pub import user_pub
from robot_control_cmd_lcmt import robot_control_cmd_lcmt
from identify import arrow,yellow_wait,yellow_light
from segment1 import segment1_control, reset_segment1
from segment2 import segment2_control, reset_segment2
from segment3 import segment3_control, reset_segment3
from segment4 import segment4_control, reset_segment4, speech_ready
from segment5 import segment5_control, reset_segment5
from segment6 import segment6_control, reset_segment6
from ball_camera import BallCamera
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

        # 赛段3出口：朝 90° 前进，对准中心线 x=3.15 后进入赛段5
        elif flags["ENDING_FLAG4"] == False:
            step = segment4_control(position, gait_mode, rpy, frame=frame)
            if step == -1:
                flags["ENDING_FLAG4"] = True
                return 0
            return step

        # 赛段5：螺旋爬坡+不平整路面+跳下
        elif flags["ENDING_FLAG5"] == False:
            step = segment5_control(position, gait_mode, rpy)
            if step == -1:
                flags["ENDING_FLAG5"] = True
                return 0
            return step

        # 赛段6：撷金建功（角落横移顶球→前推进圈→趴下，整条赛道最后一棒）
        elif flags["ENDING_FLAG6"] == False:
            step = segment6_control(position, gait_mode, rpy, frame=frame)
            if step == -1:
                flags["ENDING_FLAG6"] = True
                return 4   # 趴下（segment6 内部已趴下，这里收尾保持趴下）
            return step

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
        reset_segment4()
        reset_segment5()
        reset_segment6()
        my_ctrl = Robot_Ctrl()
        pos_msg = Pos_msg(data_lock)
        gait_msg = Gait_msg(data_lock)
        ctrl_thread = threading.Thread(target=my_ctrl.run)
        rec_thread = threading.Thread(target=pos_msg.run)
        gait_thread= threading.Thread(target=gait_msg.run)

        # 按开发者手册固定使用真机 RGB 相机 /image_rgb，不接 AI 相机 /image。
        cam_node = BallCamera().start()

        ctrl_thread.start()
        time.sleep(4)
        my_ctrl.num = 2# 起步左转一下调正机位
        my_ctrl.msg.life_count =(my_ctrl.msg.life_count + 1) % 127
        time.sleep(0.5)
        rec_thread.start()
        gait_thread.start()
        if not cam_node.wait_ready(15.0):
            raise RuntimeError(f"RGB相机没有真实图像，拒绝启动比赛：{cam_node.diagnostics()}")
        if not speech_ready():
            raise RuntimeError("未找到第四赛段语音引擎；请安装 spd-say/espeak-ng，或设置 SEGMENT4_TTS")
        print(f"相机就绪，图像话题={cam_node.active_topic()}，诊断={cam_node.diagnostics()}")
        def print_worker():
            while True:
                from segment2 import _state as seg2_state, _target_idx as seg2_row
                from segment5 import _state as seg5_state
                import segment6 as seg6
                if flags["ENDING_FLAG5"] == False and flags["ENDING_FLAG4"] == True:
                    active_state = f"seg5={seg5_state}"
                elif flags["ENDING_FLAG6"] == False and flags["ENDING_FLAG5"] == True:
                    frame = cam_node.frame()
                    found, offset, radius = seg6.find_ball(frame)
                    diag = cam_node.diagnostics()
                    active_state = (f"seg6={seg6._state} camera={diag['active_topic']} "
                                    f"frames={diag['frame_count']} age={diag['frame_age']} "
                                    f"ball={'Y' if found else 'N'} u={offset:+.0f} "
                                    f"r={radius:.0f} lost={seg6._lost_count} "
                                    f"cam_error={diag['error']}")
                else:
                    active_state = f"seg2={seg2_state}/row{seg2_row}"
                print(f"当前位置: {pos_msg.position} 机身朝向{pos_msg.rpy[2]} 箭头识别结果{results['ARROW']} 选择:{my_ctrl.num} {active_state}")
                print(f"{gait_msg.gait_mode}")
                time.sleep(0.2)
        thread = threading.Thread(target=print_worker)
        thread.start()

        while True:
            # time.sleep(0.2)
            with data_lock:
                num = select_step_based_on_position(pos_msg.position, gait_msg.gait_mode, pos_msg.rpy[2], cam_node.frame())
            my_ctrl.num = num
            my_ctrl.msg.life_count =(my_ctrl.msg.life_count + 1) % 127
            if flags["ENDING_FLAG3"] and not flags["ENDING_FLAG4"]:
                # 第四赛段视觉/限高状态机需要约5Hz连续更新；站立0只是一拍状态交接。
                time.sleep(0.2)
            elif num == 0 and flags["ENDING_FLAG5"] == False:
                print("站立")
                time.sleep(4)
            elif flags["ENDING_FLAG5"] == True:
                # 第六赛段视觉闭环保持约5Hz；包括返回站立指令的帧。
                time.sleep(0.2)

    except KeyboardInterrupt:
        cmd_msg.mode = 7  # PureDamper before KeyboardInterrupt
        cmd_msg.gait_id = 0
        cmd_msg.duration = 0
        cmd_msg.life_count += 1
        lcm_cmd.publish("robot_control_cmd", cmd_msg.encode())
        pass
    finally:
        if 'cam_node' in locals():
            cam_node.stop()
    sys.exit()


if __name__ == '__main__':
    main()
