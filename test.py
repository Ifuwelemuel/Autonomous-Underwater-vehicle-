# test_servo.py — run this standalone to verify
from pymavlink import mavutil
import time

mav = mavutil.mavlink_connection('/dev/ttyACM0', baud=57600)
mav.wait_heartbeat()
print("Connected!")

while True:
    # Sweep pitch servo back and forth
    for pwm in [1200, 1500, 1800, 1500]:
        mav.mav.command_long_send(
            mav.target_system, mav.target_component,
            mavutil.mavlink.MAV_CMD_DO_SET_SERVO,
            0, 10, pwm, 0, 0, 0, 0, 0)
        print(f"Sent PWM: {pwm}")
        time.sleep(1)