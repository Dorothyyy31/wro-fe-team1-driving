#!/usr/bin/env python3
# 树莓派5 + IMX219 相机（Picamera2）
# 功能：仅相机自动驾驶 —— 指定颜色避让 + 地面可通行区域引导
# 驱动：L298N（差速）
# 说明：使用 Picamera2 低延迟视频流；先自动曝光/白平衡，稳定后锁定，降低 HSV 偏移

import cv2, numpy as np, time, sys, signal
import RPi.GPIO as GPIO
from picamera2 import Picamera2

############################################
# === 配置（可按场地/小车微调） ===
############################################
# 图像尺寸/帧率（IMX219 支持更高分辨率，这里用 640x360 以降低延迟与算力）
FRAME_W, FRAME_H = 640, 360
FPS_LIMIT = 30
SHOW_DEBUG = False            # 有显示器时可设 True 以便调参

# 颜色避障（从 HSV_RANGES 的键里选）
TARGET_COLORS = ["red", "blue"]

# HSV 范围（需按现场光照微调；红色跨 0/179 需两段）
HSV_RANGES = {
    "red":    [((0, 100, 80),  (10, 255, 255)), ((170, 90, 80), (179, 255, 255))],
    "blue":   [((95,  80, 80), (130, 255, 255))],
    "yellow": [((18, 120, 80), (35,  255, 255))],
    "green":  [((40,  70, 60), (85,  255, 255))]
}
FOCUS_BAND_Y0 = 0.45         # 仅分析图像底部 55%（0..1）
AREA_MIN_PIXELS = 1500       # 色块面积小于此值忽略
CENTER_DEAD_BAND = 0.08      # 近中心时给更强推离

# 地面学习（用于可通行区域引导）
ROI_FLOOR_Y = 0.85           # 取底部 15% 采样为地面
HSV_TOL_H, HSV_TOL_S, HSV_TOL_V = 18, 60, 60
FLOOR_BLOCK_THRESH = 0.08     # 下半区“地面像素占比”低于此 → 认为阻塞
FLOOR_HARD_BLOCK  = 0.03      # 极低占比 → 短促倒退

# 电机（L298N 引脚）
PWM_FREQ = 1000
LEFT_IN1, LEFT_IN2, LEFT_EN   = 20, 21, 12
RIGHT_IN1, RIGHT_IN2, RIGHT_EN = 19, 26, 13

# 控制参数
BASE_SPEED = 0.55
STEER_GAIN = 0.55
SMOOTH_ALPHA = 0.4
COLOR_WEIGHT = 0.7            # 颜色避障与地面引导的融合权重
SLOW_BY_COLOR = 0.7           # 检到颜色障碍时整体降速系数
REVERSE_SPEED = -0.35         # 极端阻塞倒退速度
REVERSE_TIME  = 0.25          # 倒退时间（秒）

############################################
# === 电机驱动（差速） ===
############################################
class DualMotor:
    def __init__(self):
        GPIO.setmode(GPIO.BCM)
        for p in [LEFT_IN1, LEFT_IN2, LEFT_EN, RIGHT_IN1, RIGHT_IN2, RIGHT_EN]:
            GPIO.setup(p, GPIO.OUT)
        self.pl = GPIO.PWM(LEFT_EN, PWM_FREQ);  self.pl.start(0)
        self.pr = GPIO.PWM(RIGHT_EN, PWM_FREQ); self.pr.start(0)
        self._l, self._r = 0.0, 0.0

    def _drive_one(self, in1, in2, pwm, value):
        # value ∈ [-1,1]；正前进、负后退
        v = max(-1.0, min(1.0, float(value)))
        if v >= 0:
            GPIO.output(in1, 1); GPIO.output(in2, 0)
            pwm.ChangeDutyCycle(100*v)
        else:
            GPIO.output(in1, 0); GPIO.output(in2, 1)
            pwm.ChangeDutyCycle(100*(-v))

    def drive(self, left, right):
        # 一阶平滑，降低抖动
        self._l = 0.6*self._l + 0.4*left
        self._r = 0.6*self._r + 0.4*right
        self._drive_one(LEFT_IN1, LEFT_IN2, self.pl, self._l)
        self._drive_one(RIGHT_IN1, RIGHT_IN2, self.pr, self._r)

    def stop(self):
        try:
            self.drive(0,0); time.sleep(0.05)
            self.pl.stop(); self.pr.stop()
            for p in [LEFT_IN1, LEFT_IN2, RIGHT_IN1, RIGHT_IN2]:
                GPIO.output(p, 0)
            GPIO.cleanup()
        except: pass

############################################
# === Picamera2 相机封装（IMX219） ===
############################################
class IMX219Camera:
    """使用 Picamera2 采集帧（RGB888），并转换为 OpenCV BGR"""
    def __init__(self, width=FRAME_W, height=FRAME_H, fps=FPS_LIMIT):
        self.picam2 = Picamera2()
        cfg = self.picam2.create_video_configuration(
            main={"size": (width, height), "format": "RGB888"},
            controls={
                "FrameDurationLimits": (int(1e6//fps), int(1e6//fps)),  # 固定帧间隔
            }
        )
        self.picam2.configure(cfg)
        self.picam2.start()
        # 先让 AE/AWB 工作一小段时间以稳定
        time.sleep(1.5)
        try:
            md = self.picam2.capture_metadata()
            # 锁定当前的曝光/白平衡，减少 HSV 漂移
            controls = {"AeEnable": False, "AwbEnable": False}
            if "ExposureTime" in md: controls["ExposureTime"] = md["ExposureTime"]
            if "AnalogueGain" in md: controls["AnalogueGain"] = md["AnalogueGain"]
            if "ColourGains" in md:  controls["ColourGains"]  = md["ColourGains"]
            self.picam2.set_controls(controls)
        except Exception as e:
            print("[相机] 锁定 AE/AWB 失败，继续使用自动模式：", e)

    def grab_bgr(self):
        # Picamera2 返回 RGB；转为 BGR 给 OpenCV
        frame_rgb = self.picam2.capture_array()
        return cv2.cvtColor(frame_rgb, cv2.COLOR_RGB2BGR)

    def stop(self):
        try: self.picam2.stop()
        except: pass

############################################
# === 地面颜色学习（开机短采样） ===
############################################
def learn_floor_hsv(get_frame_bgr, n_frames=25):
    """get_frame_bgr 为无参函数，返回一帧 BGR"""
    hs, ss, vs = [], [], []
    for _ in range(n_frames):
        frame = get_frame_bgr()
        if frame is None: continue
        frame = cv2.resize(frame, (FRAME_W, FRAME_H))
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        y0 = int(FRAME_H*ROI_FLOOR_Y)
        roi = hsv[y0:FRAME_H, :]
        h,s,v = cv2.split(roi)
        hs.append(np.median(h)); ss.append(np.median(s)); vs.append(np.median(v))
    if len(hs)==0:
        # 兜底（不建议）：全通过
        return (0,0,0), (179,255,255)
    h0,s0,v0 = np.median(hs), np.median(ss), np.median(vs)
    lower = (max(0, h0-HSV_TOL_H), max(0, s0-HSV_TOL_S), max(0, v0-HSV_TOL_V))
    upper = (min(179, h0+HSV_TOL_H), min(255, s0+HSV_TOL_S), min(255, v0+HSV_TOL_V))
    return tuple(map(int, lower)), tuple(map(int, upper))

############################################
# === 颜色避障方向估计 ===
############################################
def color_avoid_dir(frame_bgr):
    """
    返回 (steer_away, seen, vis, area_norm)
    - steer_away ∈ [-1,1]：正→向右转（障碍在左）
    - seen：是否检测到目标颜色
    - area_norm：色块占下半区比例（0..1）
    """
    frame = cv2.resize(frame_bgr, (FRAME_W, FRAME_H))
    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
    y0 = int(FRAME_H*FOCUS_BAND_Y0)
    roi = hsv[y0:, :]

    mask_total = np.zeros((roi.shape[0], roi.shape[1]), dtype=np.uint8)
    for cname in TARGET_COLORS:
        for (lo, hi) in HSV_RANGES[cname]:
            mask_total |= cv2.inRange(roi, np.array(lo, np.uint8), np.array(hi, np.uint8))

    mask_total = cv2.medianBlur(mask_total, 5)
    kernel = np.ones((5,5), np.uint8)
    mask_total = cv2.morphologyEx(mask_total, cv2.MORPH_CLOSE, kernel, iterations=2)

    cnts, _ = cv2.findContours(mask_total, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    total_pixels = mask_total.size
    area_norm = 0.0

    if not cnts:
        if SHOW_DEBUG: return 0.0, False, cv2.cvtColor(roi, cv2.COLOR_HSV2BGR), 0.0
        return 0.0, False, None, 0.0

    cnt = max(cnts, key=cv2.contourArea)
    area = cv2.contourArea(cnt); area_norm = float(area/total_pixels)
    if area < AREA_MIN_PIXELS:
        if SHOW_DEBUG: return 0.0, False, cv2.cvtColor(roi, cv2.COLOR_HSV2BGR), area_norm
        return 0.0, False, None, area_norm

    M = cv2.moments(cnt)
    if M["m00"] == 0:
        return 0.0, False, None, area_norm
    cx = int(M["m10"]/M["m00"])

    nx = (cx - FRAME_W/2) / (FRAME_W/2)    # [-1,1]
    steer_away = float(np.clip(-nx, -1, 1)) # 左侧障碍 → 右转（正）

    if abs(nx) < CENTER_DEAD_BAND:
        steer_away = float(np.sign(steer_away)) * 0.5

    if SHOW_DEBUG:
        vis = frame.copy()
        cv2.rectangle(vis, (0, y0), (FRAME_W-1, FRAME_H-1), (255,255,255), 1)
        cv2.circle(vis, (int(cx), FRAME_H-5), 8, (0,0,255), -1)
        cv2.putText(vis, f"color_steer={steer_away:+.2f} area%={area_norm*100:.1f}",
                    (10,30), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0,255,0), 2)
        return steer_away, True, vis, area_norm
    return steer_away, True, None, area_norm

############################################
# === 地面可通行区域引导（质心） ===
############################################
def floor_guidance(frame_bgr, hsv_floor_lower, hsv_floor_upper):
    """
    返回 (steer_floor, blocked, vis, cover_ratio)
    - steer_floor ∈ [-1,1]：以“可通行区域”质心相对中心的偏移
    - blocked：地面像素占比过低（认为受阻）
    - cover_ratio：地面像素占比（0..1）
    """
    frame = cv2.resize(frame_bgr, (FRAME_W, FRAME_H))
    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

    floor = cv2.inRange(hsv, np.array(hsv_floor_lower), np.array(hsv_floor_upper))
    floor = cv2.medianBlur(floor, 5)
    kernel = np.ones((5,5), np.uint8)
    floor = cv2.morphologyEx(floor, cv2.MORPH_CLOSE, kernel, iterations=2)

    bottom = floor[int(FRAME_H*0.45):, :]
    cover_ratio = float(np.count_nonzero(bottom)/bottom.size)

    M = cv2.moments(bottom, binaryImage=True)
    cx_global = FRAME_W//2
    blocked = cover_ratio < FLOOR_BLOCK_THRESH

    if M["m00"] > 1000:
        cx_global = int(M["m10"]/M["m00"])
    steer = (cx_global - FRAME_W/2)/(FRAME_W/2)
    steer = float(np.clip(steer, -1, 1))

    if SHOW_DEBUG:
        vis = frame.copy()
        cv2.line(vis, (FRAME_W//2, FRAME_H-1), (int(cx_global), FRAME_H-1), (0,255,0), 2)
        txt = f"floor_steer={steer:+.2f} cover={cover_ratio*100:.1f}% blocked={int(blocked)}"
        cv2.putText(vis, txt, (10,60), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0,200,255), 2)
        return steer, blocked, vis, cover_ratio
    return steer, blocked, None, cover_ratio

############################################
# === 主程序 ===
############################################
def main():
    # 相机（IMX219 + Picamera2）
    cam = IMX219Camera(width=FRAME_W, height=FRAME_H, fps=FPS_LIMIT)

    # 地面颜色学习（请把车放在将要行驶的地面上）
    print("[相机] 正在学习地面颜色...")
    low, up = learn_floor_hsv(cam.grab_bgr, n_frames=25)
    print(f"[相机] 地面 HSV 范围：lower={low}, upper={up}")

    # 电机
    motors = DualMotor(); time.sleep(0.3)

    steer_cmd, speed_cmd = 0.0, 0.0
    last_info = time.time()

    def on_sigint(sig, frm): raise KeyboardInterrupt
    signal.signal(signal.SIGINT, on_sigint)

    try:
        while True:
            frame = cam.grab_bgr()
            if frame is None:
                print("[相机] 取帧失败"); break

            # 1) 颜色避障方向 + 威胁面积
            steer_color, seen_color, vis_color, area_norm = color_avoid_dir(frame)

            # 2) 地面可通行引导
            steer_floor, blocked, vis_floor, cover_ratio = floor_guidance(frame, low, up)

            # 3) 融合方向：有颜色障碍 → 按权重融合；否则纯地面引导
            steer = COLOR_WEIGHT*steer_color + (1.0-COLOR_WEIGHT)*steer_floor if seen_color else steer_floor
            steer = float(np.clip(steer, -1, 1))

            # 4) 速度策略：基础速度 ×（颜色/阻塞系数）
            speed = BASE_SPEED
            if seen_color:
                speed *= SLOW_BY_COLOR * (1.0 - min(0.6, area_norm*2.0))
            if blocked:
                speed *= 0.6
            speed = float(np.clip(speed, 0.0, 1.0))

            # 5) 极端阻塞：短促倒退
            if cover_ratio < FLOOR_HARD_BLOCK:
                motors.drive(REVERSE_SPEED, REVERSE_SPEED)
                time.sleep(REVERSE_TIME)
                continue

            # 6) 平滑并映射到左右轮
            steer_cmd = SMOOTH_ALPHA*steer + (1-SMOOTH_ALPHA)*steer_cmd
            speed_cmd = SMOOTH_ALPHA*speed + (1-SMOOTH_ALPHA)*speed_cmd
            left_out  = np.clip(speed_cmd + STEER_GAIN*(-steer_cmd), -1, 1)
            right_out = np.clip(speed_cmd + STEER_GAIN*(+steer_cmd), -1, 1)
            motors.drive(left_out, right_out)

            # 7) 可视化/心跳
            if SHOW_DEBUG:
                vis = frame.copy()
                if vis_color is not None: vis = vis_color
                if vis_floor is not None:
                    cv2.putText(vis, f"steer={steer_cmd:+.2f} speed={speed_cmd:.2f}",
                                (10,90), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255,200,0), 2)
                cv2.imshow("debug", vis)
                if cv2.waitKey(1) & 0xFF == 27: break
            else:
                now = time.time()
                if now - last_info > 0.5:
                    print(f"color={seen_color} area%={area_norm*100:.1f} "
                          f"cover={cover_ratio*100:.1f}% steer={steer_cmd:+.2f} speed={speed_cmd:.2f}")
                    last_info = now

    except KeyboardInterrupt:
        print("\n[退出] Ctrl+C")
    finally:
        try: motors.stop()
        except: pass
        try: cam.stop()
        except: pass
        if SHOW_DEBUG:
            try: cv2.destroyAllWindows()
            except: pass

if __name__ == "__main__":
    main()
