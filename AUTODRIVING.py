#!/usr/bin/env python3
# Pi5 + IMX219：仅相机导航；通过串口把转向(舵机)与油门(ESC)发给 Arduino Nano
# 协议：每帧发送 "C <steer_us> <throttle_us>\n" ，范围见下方参数

import cv2, numpy as np, time, sys, signal
import serial
from picamera2 import Picamera2


# ======== 参数 ========
FRAME_W, FRAME_H = 640, 360
FPS_LIMIT = 30
SHOW_DEBUG = False

TARGET_COLORS = ["red", "green"]  # 彩色避障仅针对红/绿“交通柱”
HSV_RANGES = {
    "red":      [((0, 100, 80),  (10, 255, 255)), ((170, 90, 80), (179, 255, 255))],
    "blue":     [((95,  80, 80), (130, 255, 255))],
    "yellow":   [((18, 120, 80), (35,  255, 255))],
    "green":    [((40,  70, 60), (85,  255, 255))],
    "orange":   [((8, 120, 90),  (22, 255, 255))],
    "magenta":  [((135, 90, 90), (165, 255, 255))],
}
FOCUS_BAND_Y0 = 0.45
AREA_MIN_PIXELS = 1500
CENTER_DEAD_BAND = 0.08

ROI_FLOOR_Y = 0.85
HSV_TOL_H, HSV_TOL_S, HSV_TOL_V = 18, 60, 60
FLOOR_BLOCK_THRESH = 0.08
FLOOR_HARD_BLOCK  = 0.03

# 运动学参数
BASE_SPEED = 0.55
STEER_GAIN = 0.55
SMOOTH_ALPHA = 0.4
COLOR_WEIGHT = 0.7
SLOW_BY_COLOR = 0.7
REVERSE_SPEED = -0.35
REVERSE_TIME  = 0.25

# ======== 新增策略参数（WRO规则 & 自定义扩展） ========
# 红柱：靠右通过；绿柱：靠左通过（WRO 9.19）
SIGN_SIDE_GAIN = 0.65            # 交通柱侧向偏置强度
SIGN_SIDE_GAIN_MAX = 0.95        # 上限
SIGN_SIDE_GAIN_AREA_SCALE = 2.8  # 按目标面积放大（越近越强）

# 圈线：先看到蓝线 => 逆时针；先看到橙线 => 顺时针（用户自定规则）
DIRECTION_BIAS_GAIN = 0.12       # 驾驶方向偏置强度
DIRECTION_LOCK_FRAMES = 12       # 连续确认帧数以“锁定”方向

# 洋红线（magenta）：平行泊车触发
PARK_DETECT_MIN_AREA = 1200      # 检出阈值（像素）
PARK_COOLDOWN_SEC = 6.0          # 停车动作冷却(避免误触发)
# —— 简易平行泊车动作参数 ——
PK_TURN_SPEED = 0.45
PK_BACK_TIME_1 = 0.9
PK_BACK_TIME_2 = 0.7
PK_ALIGN_TIME  = 0.8

# 串口：Nano 可能是 /dev/ttyACM0（ATmega16u2）或 /dev/ttyUSB0（CH340/FT232）
DEFAULT_PORTS = ["/dev/ttyUSB0"]
BAUDRATE = 115200
SEND_HZ = 50

# ======== 实用函数 ========
def find_serial_port(candidates=DEFAULT_PORTS):
    """尝试打开候选串口，返回第一个可用端口名。若全部失败则抛出异常。"""
    for p in candidates:
        try:
            s = serial.Serial(p, baudrate=BAUDRATE, timeout=0.02)
            s.close()
            print(f"[串口] 使用 {p}")
            return p
        except Exception:
            continue
    # 最后尝试通过 list_ports 自动发现
    try:
        from serial.tools import list_ports
        ports = [info.device for info in list_ports.comports()]
        for p in ports:
            try:
                s = serial.Serial(p, baudrate=BAUDRATE, timeout=0.02)
                s.close()
                print(f"[串口] 自动发现 {p}")
                return p
            except Exception:
                pass
        raise RuntimeError(f"未找到可用串口。候选: {candidates} + 自动发现 {ports}")
    except Exception:
        raise RuntimeError(f"未找到可用串口。候选: {candidates}")

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

# ======== 串口伺服车封装 ========
class ServoCar:
    """通过串口向 Nano 发送转向舵机与油门(ESC)脉宽(微秒)。
    协议：C <steer_us> <throttle_us>\n
    steer: [-1,1] -> [SERVO_CENTER_US - SERVO_RANGE_US, SERVO_CENTER_US + SERVO_RANGE_US]
    throttle: [-1,1] -> [ESC_MIN_US..ESC_MAX_US]，0≈停止，负值为倒车(需 ESC 支持反转)"""
    def __init__(self, port, baud=BAUDRATE, send_hz=SEND_HZ,
                 SERVO_CENTER_US=1500, SERVO_RANGE_US=350,
                 ESC_MIN_US=1000, ESC_STOP_US=1500, ESC_MAX_US=2000):
        self.ser = serial.Serial(port, baudrate=baud, timeout=0.02)
        self.last_send = 0.0
        self.dt = 1.0 / float(send_hz)
        self.SERVO_CENTER_US = int(SERVO_CENTER_US)
        self.SERVO_RANGE_US  = int(SERVO_RANGE_US)
        self.ESC_MIN_US      = int(ESC_MIN_US)
        self.ESC_STOP_US     = int(ESC_STOP_US)
        self.ESC_MAX_US      = int(ESC_MAX_US)
        time.sleep(0.5)
        try:
            self.ser.reset_input_buffer(); self.ser.reset_output_buffer()
            rd = self.ser.readline().decode(errors='ignore').strip()
            if rd: print("[Nano]", rd)
        except: pass

    def _clip(self, v, lo, hi):
        return lo if v < lo else (hi if v > hi else v)

    def control(self, throttle, steer):
        # throttle ∈ [-1,1]，steer ∈ [-1,1]
        now = time.time()
        if now - self.last_send < self.dt:
            return
        self.last_send = now

        t = float(self._clip(throttle, -1.0, 1.0))
        s = float(self._clip(steer, -1.0, 1.0))

        steer_us = int(round(self.SERVO_CENTER_US + s * self.SERVO_RANGE_US))
        # 油门：-1..0..+1 => 1000..1500..2000
        if t >= 0:
            th_us = int(round(self.ESC_STOP_US + t * (self.ESC_MAX_US - self.ESC_STOP_US)))
        else:
            th_us = int(round(self.ESC_STOP_US + t * (self.ESC_STOP_US - self.ESC_MIN_US)))

        cmd = f"C {steer_us} {th_us}\n"
        try:
            self.ser.write(cmd.encode('ascii', errors='ignore'))
        except Exception as e:
            print("[串口] 写入失败：", e)

    def stop(self):
        try:
            self.ser.write(f"C {self.SERVO_CENTER_US} {self.ESC_STOP_US}\n".encode('ascii'))
            self.ser.close()
        except: pass

# ======== 工具函数 ========
def learn_floor_hsv(get_bgr, n_frames=25):
    hs, ss, vs = [], [], []
    for _ in range(n_frames):
        frame = get_bgr()
        if frame is None: 
            continue
        frame = cv2.resize(frame, (FRAME_W, FRAME_H))
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        y0 = int(FRAME_H*ROI_FLOOR_Y)
        roi = hsv[y0:FRAME_H, :]
        h,s,v = cv2.split(roi)
        hs.append(np.median(h)); ss.append(np.median(s)); vs.append(np.median(v))
    if not hs: 
        return (0,0,0), (179,255,255)
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
    if M["m00"] == 0: 
        return 0.0, False, None, area_norm
    cx = int(M["m10"]/M["m00"])
    nx = (cx - FRAME_W/2) / (FRAME_W/2)
    steer_away = float(np.clip(-nx, -1, 1))
    if abs(nx) < CENTER_DEAD_BAND: 
        steer_away = float(np.sign(steer_away))*0.5
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
    # 修正：原始代码中有拼写错误(FLOOr_BLOCK_THRESH)并使用了恒假条件；这里直接按阈值判断
    blocked = (cover < FLOOR_BLOCK_THRESH)
    if M["m00"] > 1000: 
        cx = int(M["m10"]/M["m00"])
    steer = (cx - FRAME_W/2)/(FRAME_W/2)
    steer = float(np.clip(steer, -1, 1))
    if SHOW_DEBUG:
        vis = frame.copy()
        cv2.line(vis,(FRAME_W//2,FRAME_H-1),(int(cx),FRAME_H-1),(0,255,0),2)
        txt = f"floor_steer={steer:+.2f} cover={cover*100:.1f}% blocked={int(blocked)}"
        cv2.putText(vis, txt, (10,60), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0,200,255), 2)
        return steer, blocked, vis, cover
    return steer, blocked, None, cover

# ======== 新增感知与动作函数 ========
def detect_sign_and_side(frame_bgr):
    # 检测红/绿交通柱；根据颜色返回应该“靠哪一侧通过”的偏置：
    # red -> +1 (靠右)；green -> -1 (靠左)；如果未见到则 0
    # 返回: (side_bias, seen, area_norm)
    frame = cv2.resize(frame_bgr, (FRAME_W, FRAME_H))
    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
    y0 = int(FRAME_H*FOCUS_BAND_Y0)
    roi = hsv[y0:, :]

    def mask_color(cname):
        m = np.zeros((roi.shape[0], roi.shape[1]), dtype=np.uint8)
        for (lo, hi) in HSV_RANGES.get(cname, []):
            m |= cv2.inRange(roi, np.array(lo, np.uint8), np.array(hi, np.uint8))
        m = cv2.medianBlur(m, 5)
        m = cv2.morphologyEx(m, cv2.MORPH_CLOSE, np.ones((5,5),np.uint8), iterations=2)
        return m

    mask_r = mask_color("red")
    mask_g = mask_color("green")
    total = mask_r.size
    area_r = float(np.count_nonzero(mask_r))/total
    area_g = float(np.count_nonzero(mask_g))/total

    if area_r < (AREA_MIN_PIXELS/total) and area_g < (AREA_MIN_PIXELS/total):
        return 0.0, False, 0.0

    # 选择面积较大的目标作为“当前有效交通柱”
    side_bias = 0.0
    area_norm = 0.0
    if area_r >= area_g:
        side_bias = +1.0  # 红 -> 靠右
        area_norm = area_r
    else:
        side_bias = -1.0  # 绿 -> 靠左
        area_norm = area_g

    return side_bias, True, float(area_norm)


def detect_line_direction(frame_bgr, needed_frames, state):
    # 检测前景带状蓝/橙线，决定初始行进方向（蓝=逆时针，橙=顺时针）。
    # 'state' 为包含历史计数的字典，返回 (dir_sign, locked, state)
    # dir_sign: +1 顺时针；-1 逆时针；0 未确定
    frame = cv2.resize(frame_bgr, (FRAME_W, FRAME_H))
    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

    # 取靠近车前方的一条带状ROI
    y0 = int(FRAME_H*0.65); y1 = int(FRAME_H*0.9)
    roi = hsv[y0:y1, :]

    def strong_mask(cname):
        m = np.zeros((roi.shape[0], roi.shape[1]), dtype=np.uint8)
        for (lo, hi) in HSV_RANGES.get(cname, []):
            m |= cv2.inRange(roi, np.array(lo, np.uint8), np.array(hi, np.uint8))
        m = cv2.morphologyEx(m, cv2.MORPH_CLOSE, np.ones((5,5),np.uint8), iterations=1)
        return float(np.count_nonzero(m)) / float(m.size)

    cover_blue = strong_mask("blue")
    cover_oran = strong_mask("orange")

    if cover_blue > 0.02 and cover_blue > cover_oran*1.2:
        state["blue_cnt"] = state.get("blue_cnt", 0) + 1
    else:
        state["blue_cnt"] = 0

    if cover_oran > 0.02 and cover_oran > cover_blue*1.2:
        state["orange_cnt"] = state.get("orange_cnt", 0) + 1
    else:
        state["orange_cnt"] = 0

    if state.get("blue_cnt",0) >= needed_frames:
        return -1, True, state   # 蓝 -> 逆时针
    if state.get("orange_cnt",0) >= needed_frames:
        return +1, True, state   # 橙 -> 顺时针
    return 0, False, state


def detect_parking_trigger(frame_bgr, last_trigger_time):
    # 检测洋红/粉色（magenta）停车线；满足面积阈值触发平行泊车。
    # 返回 (should_park:bool, now_time:float)
    frame = cv2.resize(frame_bgr, (FRAME_W, FRAME_H))
    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
    # 仅取下部区域
    y0 = int(FRAME_H*0.7); roi = hsv[y0:, :]
    mask = np.zeros((roi.shape[0], roi.shape[1]), dtype=np.uint8)
    for (lo, hi) in HSV_RANGES.get("magenta", []):
        mask |= cv2.inRange(roi, np.array(lo, np.uint8), np.array(hi, np.uint8))
    mask = cv2.medianBlur(mask, 5)
    cnts, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    area = sum(cv2.contourArea(c) for c in cnts)
    now = time.time()
    if area >= PARK_DETECT_MIN_AREA and (now - last_trigger_time) > PARK_COOLDOWN_SEC:
        return True, now
    return False, last_trigger_time


def do_parallel_parking(car):
    # 极简“平行泊车”动作序列（示意版）。
    # 真实比赛应替换为基于传感/里程计的几何控制。
    # 1) 倒车向右
    car.control(-PK_TURN_SPEED, +0.6)   # 倒车向右打方向
    time.sleep(PK_BACK_TIME_1)
    # 2) 倒车向左让车身回正
    car.control(-PK_TURN_SPEED, -0.6)   # 倒车向左打方向
    time.sleep(PK_BACK_TIME_2)
    # 3) 前进微调对正
    car.control(+PK_TURN_SPEED*0.6, 0.0)
    time.sleep(PK_ALIGN_TIME)
    # 4) 刹停
    car.control(0.0, 0.0)


def main():
    cam = IMX219Camera(FRAME_W, FRAME_H, FPS_LIMIT)
    print("[相机] 学习地面颜色中...")
    low, up = learn_floor_hsv(cam.grab_bgr, 25)
    print(f"[相机] 地面HSV：{low} ~ {up}")

    port = find_serial_port()
    car = ServoCar(port=port)

    steer_cmd, speed_cmd = 0.0, 0.0
    last_info = time.time()

    # 方向/泊车状态在循环外初始化（避免每帧 try/except NameError）
    direction_state = {}
    dir_sign, dir_locked = 0, False
    last_park_time = 0.0

    def on_sigint(sig, frm): 
        raise KeyboardInterrupt
    signal.signal(signal.SIGINT, on_sigint)

    try:
        while True:
            frame = cam.grab_bgr()
            if frame is None:
                continue

            # 方向判定（蓝=逆时针 -1，橙=顺时针 +1）
            dir_sign, dir_locked, direction_state = detect_line_direction(
                frame, DIRECTION_LOCK_FRAMES, direction_state
            )

            # 交通柱侧通过偏置（红右 / 绿左）
            side_bias, seen_sign, sign_area = detect_sign_and_side(frame)

            # 地面引导 + 彩色避障
            steer_color, seen_color, vis_color, area_norm = color_avoid_dir(frame)
            steer_floor, blocked, vis_floor, cover = floor_guidance(frame, low, up)

            steer = (COLOR_WEIGHT*steer_color + (1.0-COLOR_WEIGHT)*steer_floor) if seen_color else steer_floor

            # 规则：靠左/靠右
            if seen_sign:
                k = min(SIGN_SIDE_GAIN_MAX, SIGN_SIDE_GAIN * (1.0 + SIGN_SIDE_GAIN_AREA_SCALE*sign_area))
                steer += k * side_bias

            # 规则：环形方向偏置
            if dir_sign != 0:
                steer += DIRECTION_BIAS_GAIN * float(dir_sign)

            steer = float(np.clip(steer, -1, 1))

            # 速度策略
            speed = BASE_SPEED
            if seen_color:
                speed *= SLOW_BY_COLOR * (1.0 - min(0.6, area_norm*2.0))
            if blocked:
                speed *= 0.6
            speed = float(np.clip(speed, 0.0, 1.0))

            # 硬阻塞：后退
            if cover < FLOOR_HARD_BLOCK:
                car.control(-abs(REVERSE_SPEED), 0.0)
                time.sleep(REVERSE_TIME)
                continue

            # 停车触发：检测到洋红线 -> 执行平行泊车动作
            should_park, last_park_time = detect_parking_trigger(frame, last_park_time)
            if should_park:
                do_parallel_parking(car)
                break  # 泊车完成后结束主循环（可按需改为继续）

            # 平滑 + 输出
            steer_cmd = SMOOTH_ALPHA*steer + (1-SMOOTH_ALPHA)*steer_cmd
            speed_cmd = SMOOTH_ALPHA*speed + (1-SMOOTH_ALPHA)*speed_cmd

            # 伺服车控制：throttle=speed_cmd, steer=steer_cmd
            car.control(speed_cmd, float(np.clip(steer_cmd, -1, 1)))

            if SHOW_DEBUG:
                vis = frame.copy()
                if vis_color is not None: 
                    vis = vis_color
                if vis_floor is not None:
                    cv2.putText(vis, f"steer={steer_cmd:+.2f} speed={speed_cmd:.2f} dir={dir_sign:+d} side_bias={side_bias:+.0f}",
                                (10,90), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255,200,0), 2)
                cv2.imshow("debug", vis)
                if cv2.waitKey(1) & 0xFF == 27:
                    break
            else:
                if time.time() - last_info > 0.5:
                    print(f"sign_seen={seen_sign} side_bias={side_bias:+.0f} sign_area%={sign_area*100:.1f} "
                          f"line_dir={'CW' if dir_sign>0 else ('CCW' if dir_sign<0 else 'UNK')} "
                          f"seen_color={seen_color} area%={area_norm*100:.1f} cover={cover*100:.1f}% "
                          f"steer={steer_cmd:+.2f} speed={speed_cmd:.2f}")
                    last_info = time.time()
    except KeyboardInterrupt:
        print("\n[退出] Ctrl+C")
    finally:
        try: car.stop()
        except: pass
        try: cam.stop()
        except: pass
        if SHOW_DEBUG:
            try: cv2.destroyAllWindows()
            except: pass

if __name__ == "__main__":
    main()
