import pigpio
import sys
import time

SERVO_PIN = 18
# サーボモーターの制御パルス幅の範囲
SERVO_MIN = 500
SERVO_MAX = 2500

pi = pigpio.pi()
if not pi.connected:
    print("pigpioデーモンに接続できません")
    sys.exit(1)

if len(sys.argv) != 2:
    print("引数として目標角度を指定してください")
    sys.exit(1)

target_position = float(sys.argv[1])

# 角度をパルス幅に変換
if target_position < 0:
    target_position = 0
elif target_position > 180:
    target_position = 180
servo_pulse = int(SERVO_MIN + (SERVO_MAX - SERVO_MIN) * target_position / 180.0)

# サーボモーターを回転させる
pi.set_servo_pulsewidth(SERVO_PIN, servo_pulse)

time.sleep(1)
pi.set_servo_pulsewidth(SERVO_PIN, 0)
pi.stop()
