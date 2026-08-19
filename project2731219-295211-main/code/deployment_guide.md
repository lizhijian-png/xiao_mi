# 机器狗部署与运行操作文档

本文档针对本仓库的 [test.py](test.py) 赛道流程，从 0 开始把代码在小米铁蛋（CyberDog / CyberDog2）真机上跑起来。每一步都是操作指令，跟着做就行。

---

## 步骤 0：先看一眼这份代码依赖什么

- **通信**：LCM（`udpm://239.255.76.67`，端口 7667 / 7670 / 7671）走 UDP 多播
- **相机**：ROS2 话题订阅（`rclpy` + `cv_bridge`），默认订阅 `/rgb_camera/image_raw`
- **步态**：从 [toml/usergait.toml](toml/usergait.toml) 加载 57 个 `[[step]]`，通过 `file_send_lcmt` 下发到狗子
- **入口**：[test.py](test.py)，主线程死循环调 `select_step_based_on_position()` 选择动作序号

---

## 步骤 1：硬件准备

**清单**

| 物品 | 说明 |
| --- | --- |
| 小米铁蛋 CyberDog / CyberDog2 | 电量 ≥ 50%，非电池维护模式 |
| 网线一根（Cat5e 以上） | 板载 GLAN 网口用 |
| 开发主机 | Ubuntu 20.04 / 22.04 推荐；有 ROS2 环境 |
| 赛道场地 | 起点区域预留 1×1m 安全区，方便冒烟测试 |
| 急停 | 手边留 App 或物理按钮 |

**开机**
1. 短按狗子电池仓电源键，等 LED 常亮
2. 等约 60 秒直到播报"就绪音效"
3. 用 App 或长按背部按钮让它站立进入 Motion 状态

---

## 步骤 2：搭建开发主机环境

### 2.1 装系统依赖
```bash
sudo apt update
sudo apt install -y python3-pip python3-dev build-essential \
                    ros-foxy-ros-base ros-foxy-cv-bridge ros-foxy-sensor-msgs \
                    net-tools iputils-ping
```
> 如果用 Ubuntu 22.04，把 `foxy` 换成 `humble`。

### 2.2 装 Python 依赖
```bash
pip3 install --user lcm toml numpy opencv-python
```
> 如果 `pip install lcm` 报错，从 [lcm-proj/lcm](https://github.com/lcm-proj/lcm) 源码编译：
> ```bash
> git clone https://github.com/lcm-proj/lcm.git && cd lcm
> mkdir build && cd build && cmake .. && make -j4 && sudo make install
> cd ../lcm-python && sudo python3 setup.py install
> ```

### 2.3 让 ROS2 环境每次终端自动生效
```bash
echo "source /opt/ros/foxy/setup.bash" >> ~/.bashrc
source ~/.bashrc
```

---

## 步骤 3：建立主机与狗子的网络连接

### 3.1 物理连接
用网线把主机网口和狗子 GLAN 口连起来。

### 3.2 配置主机网口 IP
狗子板载 IP 默认 `192.168.55.1`，把主机同一网段：
```bash
# eth0 换成你实际用的网口名（ip addr 查看）
sudo ifconfig eth0 192.168.55.100 netmask 255.255.255.0 up
```

### 3.3 验证连通
```bash
ping 192.168.55.1
# 应该有稳定回包
```

### 3.4 开启多播（关键！）
LCM 用 UDP 多播 `239.255.76.67`，必须让多播流量走连狗子的那个网口：
```bash
sudo ifconfig eth0 multicast
sudo route add -net 224.0.0.0 netmask 240.0.0.0 dev eth0
```
> 每次主机重启都要重新执行，除非写进 `/etc/network/interfaces` 或 systemd。

### 3.5 SSH 到狗子（可选，但强烈建议）
```bash
ssh mi@192.168.55.1
# 默认密码：123
```
用来查看狗子端日志、启动相机、必要时把代码拷进去跑。

---

## 步骤 4：把代码放到开发主机

### 4.1 拷贝仓库到主机
如果你现在是在 Windows 上，用 scp 或 U 盘把整个 `code/` 目录拷到 Ubuntu 主机：
```bash
# 从 Windows 侧
scp -r C:/Users/ALICE/Desktop/xiao_mi/project2731219-295211-main/code/ \
       user@ubuntu-host:~/xiao_mi/
```

### 4.2 检查关键文件都在
```bash
cd ~/xiao_mi/code
ls
# 应能看到：
# test.py  Robot_Ctrl.py  Msg_receive.py  user_pub.py  identify.py
# segment1.py ... segment6.py  smoke_test.py  action_test.py
# lcm/  toml/  xacro/
```

### 4.3 修正 Python 路径（如果需要）
[test.py:3](test.py#L3) 有硬编码：
```python
sys.path.insert(0, '/usr/local/lib/python3.8/site-packages')
```
如果你用的不是 Python 3.8 或 lcm 装在别处，删掉这行或改成实际路径：
```bash
python3 -c "import lcm; print(lcm.__file__)"   # 查看实际路径
```

---

## 步骤 5：启动狗子端的相机服务

在 **狗子上**（SSH 进去以后）执行：

### 5.1 起 RGB + 鱼眼相机
```bash
ros2 launch camera_test stereo_camera.py
```
另开一个 SSH 窗口做 lifecycle 转换：
```bash
ros2 lifecycle set /stereo_camera configure
ros2 lifecycle set /stereo_camera activate
```

### 5.2 确认话题
```bash
ros2 topic list | grep image
# 期望看到：
# /image_left
# /image_right
# /image_rgb
```
如果实际话题名不是 `/rgb_camera/image_raw`，**记下真实名字**，等下要改代码。

### 5.3 快速验证有数据
```bash
ros2 topic hz /image_rgb
# 期望有约 10-30 Hz 的输出
```

---

## 步骤 6：改代码里的相机话题名（如需）

如果步骤 5.2 里真实话题名不是 `/rgb_camera/image_raw`，改 [test.py:456](test.py#L456)：
```python
self.create_subscription(Image, "/image_rgb", self._cb, qos)
```
如果 [identify.py](identify.py) 也订阅了相机话题，同样要改。

---

## 步骤 7：冒烟测试（先不跑正式流程）

按 [smoke_test.md](smoke_test.md) 分三步验证：

### 7.1 位姿反馈
```bash
cd ~/xiao_mi/code
python3 smoke_test.py --step 1
```
**通过标准**：10 秒内看到 `position` 或 `rpy` 出现非零值。全 0 → 回到步骤 3.4 检查多播。

### 7.2 指令下发
把狗子放在开阔地面：
```bash
python3 smoke_test.py --step 2 --walk 2.0
```
**通过标准**：狗子先站立 3 秒，前进 2 秒，再站立 2 秒。不动 → 检查狗子是否 Motion 状态。

### 7.3 相机订阅
```bash
python3 smoke_test.py --step 3 --topic /image_rgb
```
**通过标准**：帧数持续增长。0 帧 → 步骤 5 里 lifecycle 没 activate 或话题名不对。

---

## 步骤 8：单动作分别测试（可选，赛前建议）

用 [action_test.py](action_test.py) 挨个验证 [toml/usergait.toml](toml/usergait.toml) 里赛道会用到的动作：

```bash
# 先看动作表
python3 action_test.py --list

# 挨个验证核心动作
python3 action_test.py --run 1:2    # 前进
python3 action_test.py --run 2:2    # 左转
python3 action_test.py --run 3:2    # 右转
python3 action_test.py --run 14:1   # 快左转
python3 action_test.py --run 15:1   # 快右转
python3 action_test.py --run 43:2   # 跳跃（！开阔地面！）
```
每个动作跑完看输出里的 `Δ pos` 和 `Δ yaw`，确认真机效果和 toml 参数一致。详见 [smoke_test.md](smoke_test.md) 第七章。

---

## 步骤 9：把狗子摆到赛道起点

**关键：[test.py](test.py) 里所有 `x`, `y` 阈值都是基于赛道地图坐标系写死的**（比如 [test.py:217](test.py#L217) 里的 `x < 1.47`），起点位姿必须和地图对齐。

**摆位检查**
1. 把狗子放到赛道起点的物理原点
2. 让机身正面对准赛道地图的 y 轴正方向（对应 rpy_z ≈ 0°）
3. 跑：
   ```bash
   python3 smoke_test.py --step 1
   ```
   观察输出的 `position` 应接近 `[0, 0, ~0.3]`，`rpy_z` 应接近 `0` 或 `360`
4. 如果偏差大：
   - 位置偏 → 物理挪动狗子
   - 角度偏 → 手动转正机身，或改起步动作（[test.py:490](test.py#L490) 的 `my_ctrl.num = 2` 就是起步左转调正）

---

## 步骤 10：正式跑赛道流程

### 10.1 确认清单
在起点前逐项确认：
- [ ] 狗子站立，电量足
- [ ] 主机能 ping 通狗子（步骤 3.3）
- [ ] 多播路由已加（步骤 3.4）
- [ ] 相机在 activate 状态（步骤 5）
- [ ] 冒烟测试三步都过（步骤 7）
- [ ] 位姿零点已对齐（步骤 9）
- [ ] 手边有急停

### 10.2 启动主程序
```bash
cd ~/xiao_mi/code
source /opt/ros/foxy/setup.bash
python3 test.py
```

### 10.3 预期输出
终端每 0.2 秒打印一行：
```
当前位置: [0.03, 0.12, 0.31] 机身朝向 89.7 箭头识别结果 0 选择:1 seg2=INIT/row0
[27, 3]
```
- `当前位置` / `机身朝向`：LCM 位姿反馈
- `箭头识别结果`：`identify.py` 处理相机帧的结果（0=未识别，1=左，2=右）
- `选择`：当前下发的 `num`
- `seg2=/seg5=/seg6=` 段：当前赛段状态机

### 10.4 中止与急停
- **正常结束**：`Ctrl+C`，程序会自动发 `mode=7`（PureDamper）让狗子软掉电趴下
- **紧急情况**：App 里点急停 / 拍物理急停按钮

---

## 步骤 11：赛后处理

```bash
# 让狗子回到趴下姿态
python3 action_test.py --run 4:3  --no-init

# 关代码进程（Ctrl+C 已经处理）
# 关狗子：长按电源键 3 秒
```

---

## 常见问题速查

| 现象 | 定位 | 处理 |
| --- | --- | --- |
| `ImportError: lcm` | Python 找不到 lcm | 步骤 2.2 里的编译安装 |
| 位姿全 0 | 多播不通 | 重跑步骤 3.4 的 route add |
| 狗子完全不动 | 未 Motion / 步态未上传 | App 里让它站立；确认 `user_pub()` 无异常 |
| 相机 0 帧 | lifecycle 未 activate / 话题名错 | 步骤 5.1 / 步骤 6 |
| 走偏 | 起点位姿没对齐 | 回到步骤 9 |
| Δ yaw 与 toml 不符 | 步态没成功下发 | 重跑 `user_pub()`（重启 test.py 会自动执行） |
| 跳跃动作抖动/摔倒 | 场地太滑或落地条件不合 | 检查 num=43 → num=44/56 的切换时序 |
| Ctrl+C 后狗子没趴下 | 急停 LCM 包丢了 | App 里主动急停 |

---

## 目录结构参考

```
code/
├── test.py                 主入口，赛道整体编排
├── Robot_Ctrl.py           LCM 指令下发线程
├── Msg_receive.py          LCM 位姿/步态回传订阅
├── user_pub.py             usergait 自定义步态下发
├── identify.py             相机图像处理（箭头/黄灯识别）
├── segment{1..6}.py        各赛段状态机
├── lcm/                    LCM 消息类型
├── toml/
│   ├── usergait.toml       57 个动作（num 索引即此文件顺序）
│   ├── usergait_def.toml   步态定义
│   └── usergait_param*.toml步态参数
├── smoke_test.py           冒烟测试（本部署文档步骤 7）
├── action_test.py          单动作测试（本部署文档步骤 8）
├── smoke_test.md           冒烟 & 单动作测试详细说明
├── developer_guide.md      官方开发者手册（ROS 接口参考）
└── deployment_guide.md     本文档
```

---

## 附：机器狗端 vs 开发主机端选择

本文档默认代码跑在**开发主机**上，通过网络组播和狗子通信。如果想直接跑在狗子 NX 板上：

1. `scp -r ~/xiao_mi/code mi@192.168.55.1:/home/mi/`
2. `ssh mi@192.168.55.1`
3. 在狗子上直接跑 `python3 /home/mi/code/test.py`

**利**：不用配主机路由，多播天然通
**弊**：狗子板性能有限，OpenCV 处理相机帧会慢；改代码要重新 scp

推荐调试用主机、比赛用狗子板。
