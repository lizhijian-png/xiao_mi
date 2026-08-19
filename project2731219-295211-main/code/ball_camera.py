"""常驻相机取帧线程（赛段6视觉追球用）。

为什么不复用 identify.py 的写法：那边每个函数各自 rclpy.init() → 阻塞等一帧 →
shutdown()，一次性用法。赛段6的控制循环约 5Hz 连续跑，每帧都这样开关节点会
把循环拖死，而且 rclpy 反复 init/shutdown 在同进程里并不可靠。这里改成
「起一次、后台常驻、主循环随时取最新帧」，取帧不阻塞控制节奏。

用法：
    from ball_camera import BallCamera
    cam = BallCamera(); cam.start()
    frame = cam.frame()      # 最新帧或 None（还没收到/已关闭）
    ...
    cam.stop()
"""
import threading
import time


# 开发者手册规定真机 RGB=/image_rgb；本项目 Gazebo RGB=/rgb_camera/image_raw。
# 两者都是 RGB 相机，不包含 AI 相机 /image。采用实际最先/最新收到帧的来源。
RGB_CAMERA_TOPIC = '/image_rgb'
SIM_CAMERA_TOPIC = '/rgb_camera/image_raw'
RGB_CAMERA_TOPICS = (RGB_CAMERA_TOPIC, SIM_CAMERA_TOPIC)
FRAME_STALE_SEC = 1.0


class BallCamera:
    """后台订阅相机话题，只保留最新一帧。

    帧不排队：追球只关心「现在球在哪」，旧帧毫无价值，留着反而引入延迟。
    取帧加锁但不拷贝——cv_bridge 每次回调产出新数组，主循环拿到的引用不会被
    后续回调改写。
    """

    def __init__(self, topics=RGB_CAMERA_TOPICS):
        if isinstance(topics, str):
            topics = (topics,)
        self._topics = tuple(topics)
        self._active_topic = None
        self._frame = None
        self._lock = threading.Lock()
        self._thread = None
        self._stop = threading.Event()
        self._node = None
        self._owns_rclpy = False
        self._frame_count = 0
        self._last_frame_time = None
        self._error = None

    def start(self):
        """起后台线程开始收帧。重复调用无副作用。"""
        if self._thread is not None:
            return self
        self._thread = threading.Thread(target=self._run, daemon=True)
        self._thread.start()
        return self

    def _run(self):
        try:
            self._run_ros()
        except Exception as exc:
            # 后台线程异常不能静默，否则外部只会误以为“识别不到球”。
            self._error = f'{type(exc).__name__}: {exc}'

    def _run_ros(self):
        import rclpy
        from sensor_msgs.msg import Image
        from cv_bridge import CvBridge
        from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy

        if not rclpy.ok():
            rclpy.init(args=None)
            self._owns_rclpy = True      # 只有自己 init 的才负责 shutdown
        bridge = CvBridge()
        self._node = rclpy.create_node('segment6_ball_camera')
        # BEST_EFFORT + depth 1：图像流丢帧无所谓，要的是最新，不要积压。
        qos = QoSProfile(reliability=QoSReliabilityPolicy.BEST_EFFORT,
                         history=QoSHistoryPolicy.KEEP_LAST, depth=1)

        def make_callback(topic):
            def on_image(msg):
                try:
                    img = bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
                except Exception:
                    return
                with self._lock:
                    self._frame = img
                    self._active_topic = topic
                    self._frame_count += 1
                    self._last_frame_time = time.monotonic()
            return on_image

        # 保存引用，避免 Python GC 后停止收帧。
        self._subscriptions = [
            self._node.create_subscription(Image, topic, make_callback(topic), qos)
            for topic in self._topics
        ]
        try:
            while not self._stop.is_set() and rclpy.ok():
                rclpy.spin_once(self._node, timeout_sec=0.05)
        finally:
            try:
                self._node.destroy_node()
            except Exception:
                pass
            if self._owns_rclpy and rclpy.ok():
                try:
                    rclpy.shutdown()
                except Exception:
                    pass

    def frame(self):
        """返回1秒内的最新帧；无帧或发布已停止则返回 None。"""
        with self._lock:
            if (self._last_frame_time is None or
                    time.monotonic() - self._last_frame_time > FRAME_STALE_SEC):
                return None
            return self._frame

    def active_topic(self):
        """返回当前实际收到图像的话题；None 表示所有候选话题均无帧。"""
        with self._lock:
            return self._active_topic

    def diagnostics(self):
        """返回可打印的相机状态，区分无发布、线程异常和正常收帧。"""
        with self._lock:
            age = (None if self._last_frame_time is None
                   else max(0.0, time.monotonic() - self._last_frame_time))
            return {
                'topics': self._topics,
                'active_topic': self._active_topic,
                'frame_count': self._frame_count,
                'frame_age': age,
                'error': self._error,
                'thread_alive': self._thread is not None and self._thread.is_alive(),
            }

    def wait_ready(self, timeout=5.0):
        """等到收到第一帧，返回是否成功。仅供启动前自检，控制循环里别调。"""
        import time
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            if self.frame() is not None:
                return True
            time.sleep(0.05)
        return False

    def stop(self):
        self._stop.set()
        if self._thread is not None:
            self._thread.join(timeout=2.0)
            self._thread = None
