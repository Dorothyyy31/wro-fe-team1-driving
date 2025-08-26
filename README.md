# wro-fe-team1-driving
This respository is the software we developed for the autonomous driving



Project summary

This project turns a Raspberry Pi 5 with an IMX219 (Pi Camera v2) to make use of computer vision to develop a autonomously-driven car . The software detects specific colors and simultaneously learns the traversable floor appearance to steer toward open space. It outputs left/right wheel commands through PWM into an Arduino nano board, driving two DC motors. everything runs in Python except Arduino use c++ with OpenCV and Picamera2. 

Hardware & electromechanical mapping
Compute/controller: Raspberry Pi 5 (runs all perception + control; it is the “controller”).
Camera: IMX219 via CSI; captured with Picamera2 at e.g. 640×360 @ 30 FPS for low latency.
Motor driver:  Arduino nano board+L298N dual H-bridge.
Motors: Two DC gear motors (differential drive).
Power: 5 V rail for Pi (stable supply), separate 6–12 V for motors → common ground between Pi GND and L298N GND.

Software architecture (modules & responsibilities)
IMX219Camera
Where: class IMX219Camera
Does: Creates a Picamera2 video config (RGB888) at the requested size/FPS, starts the stream, captures a few frames, reads metadata, and disables AE/AWB using the last stable values (exposure, gains, color gains).
Output: grab_bgr() returns a BGR frame ready for OpenCV.

DualMotor
Where: class DualMotor
Does: Initializes GPIO, sets up two PWM channels on pins 12 and 13, and provides drive(left,right) with values in [-1, 1] (negative means reverse). It internally smooths commands to reduce jerk and wheel slip. stop() safely halts and cleans up GPIO.
Hardware link: Directly maps to the L298N INx/ENx pins, thus to the motors.

learn_floor_hsv
Where: function learn_floor_hsv(...)
Does: On startup, samples the bottom strip of the scene (e.g., bottom 15%) across ~25 frames, computes median HSV, and builds a floor HSV band with configurable tolerances. This adapts the “traversable” definition to your actual surface.
Output: (lower_hsv, upper_hsv) threshold pair.

color_avoid_dir
Where: function color_avoid_dir(frame)
Does: Builds an HSV mask for your selected TARGET_COLORS (with robust handling for red’s wrap-around). It filters the bottom half of the image, keeps the largest blob, and computes its centroid.
Output:
steer_away ∈ [-1,1] (positive = steer right),
seen (did we detect a color obstacle?),
area_norm (blob area fraction in ROI, used as a “threat” for slowing).

floor_guidance
Where: function floor_guidance(frame, lower, upper)
Does: Thresholds the frame with the learned “floor” HSV band, focuses on the lower image half, and finds the centroid of traversable pixels; computes a steering offset toward that centroid. Also returns coverage ratio to decide if the path is blocked (very low coverage → likely an obstacle or dead end).
Output:
steer_floor ∈ [-1,1],
blocked (coverage < threshold),
cover_ratio (0..1).

Main control loop
Where: main()
Pipeline:
Grab frame → (A) color avoidance → (B) floor guidance → fuse directions → compute speed → safety (reverse briefly if extremely blocked) → map to left/right wheel PWM with low-pass smoothing → drive motors.
Fusion logic: if a color obstacle is seen, steer = COLOR_WEIGHT * steer_color + (1-COLOR_WEIGHT) * steer_floor; else just steer_floor. Speed is BASE_SPEED, then reduced by color “threat” and by floor blockage.
Actuation: left = speed - STEER_GAIN*steer, right = speed + STEER_GAIN*steer, both clipped to [-1,1].

Why it’s good?
Adaptive floor model: learns your surface once at boot → resilient to textures/marks typical of that floor.
Color-specific obstacle: lets you encode semantics (e.g., avoid red cones even if floor looks open).
Command smoothing: reduces wheelspin and oscillations.
AE/AWB lock: keeps HSV thresholds meaningful across frames.
Parameter knobs you’ll actually touch
TARGET_COLORS and HSV_RANGES: define what to avoid; tune per lighting.
ROI settings: FOCUS_BAND_Y0, ROI_FLOOR_Y determine where we “look.”
Thresholds: AREA_MIN_PIXELS, FLOOR_BLOCK_THRESH, FLOOR_HARD_BLOCK.
Dynamics: BASE_SPEED, STEER_GAIN, SMOOTH_ALPHA, COLOR_WEIGHT, SLOW_BY_COLOR.
Reverse behavior: REVERSE_SPEED, REVERSE_TIME.

Build / run / deploy 
 You install dependencies, copy the script, and run it. The Pi is the controller; the L298N is only a driver.
OS & camera enable

sudo apt update
sudo apt install -y python3-opencv python3-numpy python3-rpi.gpio python3-picamera2
sudo raspi-config   # Interface Options → Camera → Enable → Reboot


Get the code onto the Pi
Option A (edit on Pi): create autodrive_cam_imx219.py and paste the code.
Option B (from your PC via SSH/SCP):

scp autodrive_cam_imx219.py pi@<pi-ip>:/home/pi/


Wire check (very important)
L298N IN1=BCM20, IN2=BCM21, ENA=BCM12 (left motor)
L298N IN3=BCM19, IN4=BCM26, ENB=BCM13 (right motor)
L298N GND ↔ Pi GND (common ground), motor supply to L298N +12V/+6V as per your motors.
If a side runs backward, swap its IN1/IN2 (or flip wires at the motor).
Sanity tests

Camera:
python3 - <<'PY'
import cv2; from picamera2 import Picamera2
pc=Picamera2(); pc.configure(pc.create_preview_configuration(main={"size":(640,360),"format":"RGB888"}))
pc.start(); import time; time.sleep(0.5)
img=pc.capture_array(); pc.stop()
print(img.shape)
PY

- Motors (brief pulse, wheels off ground):  
```bash
python3 - <<'PY'
import time, RPi.GPIO as GPIO
L1,L2,LE=20,21,12; R1,R2,RE=19,26,13
GPIO.setmode(GPIO.BCM)
for p in [L1,L2,LE,R1,R2,RE]: GPIO.setup(p, GPIO.OUT)
pl=GPIO.PWM(LE,1000); pr=GPIO.PWM(RE,1000); pl.start(0); pr.start(0)
# forward 0.5s
GPIO.output(L1,1); GPIO.output(L2,0); pl.ChangeDutyCycle(60)
GPIO.output(R1,1); GPIO.output(R2,0); pr.ChangeDutyCycle(60)
time.sleep(0.5)
pl.stop(); pr.stop(); GPIO.cleanup()
PY


Run the autopilot

python3 ~/autodrive_cam_imx219.py


Place the car on the actual floor before launch so the “floor learning” step captures the right HSV.

If you have HDMI and want visuals, set SHOW_DEBUG=True.

Autostart as a systemd service (optional)

sudo tee /etc/systemd/system/autodrive.service >/dev/null <<'UNIT'
[Unit]
Description=Camera-only Autodrive
After=multi-user.target

[Service]
ExecStart=/usr/bin/python3 /home/pi/autodrive_cam_imx219.py
WorkingDirectory=/home/pi
StandardOutput=journal
StandardError=journal
Restart=on-failure

[Install]
WantedBy=multi-user.target
UNIT

sudo systemctl daemon-reload
sudo systemctl enable --now autodrive.service


Stop it later with sudo systemctl stop autodrive.service.
The motor controller is the L298N H-bridge (no firmware), driven directly by Pi’s GPIO PWM. 

