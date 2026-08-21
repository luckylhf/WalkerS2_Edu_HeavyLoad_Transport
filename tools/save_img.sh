#!/bin/bash
source ~/demo3/install/setup.bash
OUT_DIR="${HOME}/demo3/images"
mkdir -p "${OUT_DIR}"
cd "${OUT_DIR}"

python3 << 'PYEOF'
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from shm_msgs.msg import Image2m
from datetime import datetime
from pathlib import Path
from PIL import Image

class FD(Node):
    def __init__(self):
        super().__init__('img_downloader')
        self.done = False
        qos = QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT, history=HistoryPolicy.KEEP_LAST, depth=5)
        self.sub = self.create_subscription(Image2m, '/sensor/camera/stereo/color/raw', self.cb, qos)
    def cb(self, msg):
        enc = bytes(msg.encoding.data[:msg.encoding.size]).split(b'\0',1)[0].decode('ascii','replace').lower()
        ch = 3 if 'bgr' in enc or 'rgb' in enc else 1
        is_bgr = 'bgr' in enc
        mode = 'RGB' if ch == 3 else 'L'

        if msg.width < 10 or msg.height < 10:
            print(f'Bad: {msg.width}x{msg.height}')
            self.done = True
            return

        out = Path('stereo_color_' + datetime.now().strftime('%Y%m%d_%H%M%S') + '.png')
        raw = msg.data

        # 去除 step 填充，构建连续像素 buffer
        buf = bytearray()
        for y in range(msg.height):
            off = y * msg.step
            row = raw[off:off + msg.width * ch]
            if is_bgr and ch == 3:
                for i in range(0, len(row), 3):
                    buf.extend(row[i+2:i+3])  # R
                    buf.extend(row[i+1:i+2])  # G
                    buf.extend(row[i:i+1])    # B
            else:
                buf.extend(row)

        img = Image.frombytes(mode, (msg.width, msg.height), bytes(buf))
        img.save(str(out), 'PNG')
        print(f'{out} ({msg.width}x{msg.height})')
        # Do not call rclpy.shutdown() from inside the subscription callback:
        # the executor may wait for this callback while shutting itself down.
        self.done = True

print('等待 /sensor/camera/stereo/color/raw ...')
rclpy.init()
n = FD()
try:
    # 让回调只设置 done 标志，退出由主线程完成，避免在回调内关闭
    # executor 导致 rclpy.shutdown() 等待当前回调而卡住。
    while rclpy.ok() and not n.done:
        rclpy.spin_once(n, timeout_sec=1.0)
finally:
    n.destroy_node()
    if rclpy.ok(): rclpy.shutdown()
PYEOF
ls -lt "${OUT_DIR}/"*.png 2>/dev/null | head -3
