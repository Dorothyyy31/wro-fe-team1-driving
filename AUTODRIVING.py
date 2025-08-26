#!/usr/bin/env python3
# Pi5 + IMX219：仅相机导航；通过串口把左右轮速度发给 Arduino Nano
# 协议：每帧发送 "S <L> <R>\n" ，范围 -255..255

import cv2, numpy as np, time, sys, signal, serial
from picamera2 import Picamera2

# ======== 参数 ========
FRAME_W, FRAME_H = 640, 360
FPS_LIMIT = 30
SHOW_DEBUG = False

TARGET_COLORS = ["red", "blue"]
HSV_RANGES = {
    "red":    [((0, 100, 80),  (10, 255, 255)), ((170, 90, 80), (179, 255, 255))],
    "blue":   [((95,  80, 80), (130, 255, 255))],
    "yellow": [((18, 120, 80), (35,  255, 255))],
    "green":  [((40,  70, 60), (85,  255, 255))]
}
FOCUS_BAND_Y0 = 0.45
AREA_MIN_PIXELS = 1500
CENTER_DEAD_BAND = 0.08

ROI_FLOOR_Y = 0.85
HSV_TOL_H, HSV_TOL_S, HSV_TOL_V = 18, 60, 60
FLOOR_BLOCK_THRESH = 0.08
FLOOR_HARD_BLOCK  = 0.03

# 运动学参数（保持与原程序一致）
BASE_SPEED = 0.55
STEER_GAIN = 0.55
SMOOTH_ALPHA = 0.4
COLOR_WEIGHT = 0.7
SLOW_BY_COLOR = 0.7
REVERSE_SPEED = -0.35
REVERSE_TIME  = 0.25

# 串口：Nano 可能是 /dev/ttyACM0（ATmega16u2）或 /dev/ttyUSB0（CH340/FT232）
SERIAL_PORT = "/dev/ttyACM0"
BAUDRATE = 115200
SEND_HZ = 50

# ======== 相机封装 ========
class IMX219Camera:
    def __init__(self, width=FRAME_W, height=FRAME_H, fps=FPS_LIMIT):
        self.picam2 = Picamera2()
        cfg = self.picam2.create_video_configuration(
            main={"size": (width, height), "format": "RGB888"},
            controls={"FrameDurationLimits": (int(1e6//fps), int(1e6//fps))}
        )
        self.picam2.configure(cfg)
        self.picam2.start()
        time.sleep(1.5)
        try:
            md = self.picam2.capture_metadata()
            controls = {"AeEnable": False, "AwbEnable": False}
            if "ExposureTime" in md:  controls["ExposureTime"] = md["ExposureTime"]
            if "AnalogueGain" in md:  controls["AnalogueGain"]  = md["AnalogueGain"]
            if "ColourGains" in md:   controls["ColourGains"]   = md["ColourGains"]
            self.picam2.set_controls(controls)
        except Exception as e:
            print("[相机] AE/AWB 锁定失败：", e)

    def grab_bgr(self):
        rgb = self.picam2.capture_array()
        return cv2.cvtColor(rgb, cv2.COLOR_RGB2BGR)

    def stop(self):
        try: self.picam2.stop()
        except: pass

# ======== 串口电机封装 ========
class SerialMotor:
    """把 [-1,1] 左右速度映射为 -255..255，经串口发给 Nano"""
    def __init__(self, port=SERIAL_PORT, baud=BAUDRATE, send_hz=SEND_HZ):
        self.ser = serial.Serial(port, baudrate=baud, timeout=0.02)
        self.last_send = 0.0
        self.dt = 1.0 / float(send_hz)
        time.sleep(0.5)  # 等待 Nano 复位
        self.ser.reset_input_buffer(); self.ser.reset_output_buffer()
        # 可读取 READY
        try:
            rd = self.ser.readline().decode(errors='ignore').strip()
            if rd: print("[Nano]", rd)
        except: pass

    def drive(self, left, right):
        now = time.time()
        if now - self.last_send < self.dt:
            return
        self.last_send = now
        # 映射 [-1,1] -> [-255,255]
        def mapv(v):
            v = max(-1.0, min(1.0, float(v)))
            return int(round(v * 255.0))
        L = mapv(left); R = mapv(right)
        cmd = f"S {L} {R}\n"
        try:
            self.ser.write(cmd.encode())
        except Exception as e:
            print("[串口] 写入失败：", e)

    def stop(self):
        try:
            self.ser.write(b"S 0 0\n")
            self.ser.close()
        except: pass

# ======== 工具函数（与原相同逻辑） ========
def learn_floor_hsv(get_bgr, n_frames=25):
    hs, ss, vs = [], [], []
    for _ in range(n_frames):
        frame = get_bgr()
        if frame is None: continue
        frame = cv2.resize(frame, (FRAME_W, FRAME_H))
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        y0 = int(FRAME_H*ROI_FLOOR_Y)
        roi = hsv[y0:FRAME_H, :]
        h,s,v = cv2.split(roi)
        hs.append(np.median(h)); ss.append(np.median(s)); vs.append(np.median(v))
    if not hs: return (0,0,0), (179,255,255)
    h0,s0,v0 = np.median(hs), np.median(ss), np.median(vs)
    lower = (max(0, h0-HSV_TOL_H), max(0, s0-HSV_TOL_S), max(0, v0-HSV_TOL_V))
    upper = (min(179, h0+HSV_TOL_H), min(255, s0+HSV_TOL_S), min(255, v0+HSV_TOL_V))
    return tuple(map(int, lower)), tuple(map(int, upper))

def color_avoid_dir(frame_bgr):
    frame = cv2.resize(frame_bgr, (FRAME_W, FRAME_H))
    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
    y0 = int(FRAME_H*FOCUS_BAND_Y0)
    roi = hsv[y0:, :]
    mask = np.zeros((roi.shape[0], roi.shape[1]), dtype=np.uint8)
    for cname in TARGET_COLORS:
        for (lo, hi) in HSV_RANGES[cname]:
            mask |= cv2.inRange(roi, np.array(lo, np.uint8), np.array(hi, np.uint8))
    mask = cv2.medianBlur(mask, 5)
    mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, np.ones((5,5),np.uint8), iterations=2)
    cnts, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    total = mask.size
    area_norm = 0.0
    if not cnts:
        return 0.0, False, None, 0.0
    c = max(cnts, key=cv2.contourArea)
    area = cv2.contourArea(c); area_norm = float(area/total)
    if area < AREA_MIN_PIXELS:
        return 0.0, False, None, area_norm
    M = cv2.moments(c)
    if M["m00"] == 0: return 0.0, False, None, area_norm
    cx = int(M["m10"]/M["m00"])
    nx = (cx - FRAME_W/2) / (FRAME_W/2)
    steer_away = float(np.clip(-nx, -1, 1))
    if abs(nx) < CENTER_DEAD_BAND: steer_away = float(np.sign(steer_away))*0.5
    if SHOW_DEBUG:
        vis = frame.copy()
        cv2.rectangle(vis,(0,y0),(FRAME_W-1,FRAME_H-1),(255,255,255),1)
        cv2.circle(vis,(int(cx),FRAME_H-5),8,(0,0,255),-1)
        cv2.putText(vis, f"color_steer={steer_away:+.2f} area%={area_norm*100:.1f}", (10,30),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0,255,0), 2)
        return steer_away, True, vis, area_norm
    return steer_away, True, None, area_norm

def floor_guidance(frame_bgr, lower, upper):
    frame = cv2.resize(frame_bgr, (FRAME_W, FRAME_H))
    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
    floor = cv2.inRange(hsv, np.array(lower), np.array(upper))
    floor = cv2.medianBlur(floor, 5)
    floor = cv2.morphologyEx(floor, cv2.MORPH_CLOSE, np.ones((5,5),np.uint8), iterations=2)
    bottom = floor[int(FRAME_H*0.45):, :]
    cover = float(np.count_nonzero(bottom)/bottom.size)
    M = cv2.moments(bottom, binaryImage=True)
    cx = FRAME_W//2
    blocked = cover < FLOOr_BLOCK_THRESH if False else (cover < FLOOR_BLOCK_THRESH)
    if M["m00"] > 1000: cx = int(M["m10"]/M["m00"])
    steer = (cx - FRAME_W/2)/(FRAME_W/2)
    steer = float(np.clip(steer, -1, 1))
    if SHOW_DEBUG:
        vis = frame.copy()
        cv2.line(vis,(FRAME_W//2,FRAME_H-1),(int(cx),FRAME_H-1),(0,255,0),2)
        txt = f"floor_steer={steer:+.2f} cover={cover*100:.1f}% blocked={int(blocked)}"
        cv2.putText(vis, txt, (10,60), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0,200,255), 2)
        return steer, blocked, vis, cover
    return steer, blocked, None, cover

def main():
    cam = IMX219Camera(FRAME_W, FRAME_H, FPS_LIMIT)
    print("[相机] 学习地面颜色中...")
    low, up = learn_floor_hsv(cam.grab_bgr, 25)
    print(f"[相机] 地面HSV：{low} ~ {up}")

    motors = SerialMotor(SERIAL_PORT, BAUDRATE)
    steer_cmd, speed_cmd = 0.0, 0.0
    last_info = time.time()

    def on_sigint(sig, frm): raise KeyboardInterrupt
    signal.signal(signal.SIGINT, on_sigint)

    try:
        while True:
            frame = cam.grab_bgr()
            steer_color, seen_color, vis_color, area_norm = color_avoid_dir(frame)
            steer_floor, blocked, vis_floor, cover = floor_guidance(frame, low, up)

            steer = COLOR_WEIGHT*steer_color + (1.0-COLOR_WEIGHT)*steer_floor if seen_color else steer_floor
            steer = float(np.clip(steer, -1, 1))

            speed = BASE_SPEED
            if seen_color: speed *= SLOW_BY_COLOR * (1.0 - min(0.6, area_norm*2.0))
            if blocked:    speed *= 0.6
            speed = float(np.clip(speed, 0.0, 1.0))

            if cover < FLOOR_HARD_BLOCK:
                motors.drive(REVERSE_SPEED, REVERSE_SPEED)
                time.sleep(REVERSE_TIME)
                continue

            steer_cmd = SMOOTH_ALPHA*steer + (1-SMOOTH_ALPHA)*steer_cmd
            speed_cmd = SMOOTH_ALPHA*speed + (1-SMOOTH_ALPHA)*speed_cmd
            left  = np.clip(speed_cmd + STEER_GAIN*(-steer_cmd), -1, 1)
            right = np.clip(speed_cmd + STEER_GAIN*(+steer_cmd), -1, 1)
            motors.drive(left, right)

            if SHOW_DEBUG:
                vis = frame.copy()
                if vis_color is not None: vis = vis_color
                if vis_floor is not None:
                    cv2.putText(vis, f"steer={steer_cmd:+.2f} speed={speed_cmd:.2f}",
                                (10,90), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255,200,0), 2)
                cv2.imshow("debug", vis)
                if cv2.waitKey(1) & 0xFF == 27: break
            else:
                if time.time() - last_info > 0.5:
                    print(f"seen_color={seen_color} area%={area_norm*100:.1f} cover={cover*100:.1f}% "
                          f"steer={steer_cmd:+.2f} speed={speed_cmd:.2f}")
                    last_info = time.time()
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
