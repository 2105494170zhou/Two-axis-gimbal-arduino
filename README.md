# Phone Gimbal (Arduino)

I built a 2-axis phone gimbal so my running videos are smoother.  
It uses SimpleFOC for motor control and an MPU6050 IMU.

## How to run
1. Open `src/gimbal.ino` in Arduino IDE.
2. Install SimpleFOC + Adafruit_MPU6050.
3. Select the right board/port, then upload.

## What it does
- Stabilizes pitch and roll.
- Basic PID tuning so it doesn’t shake.
- Works best if the gimbal is balanced first.
