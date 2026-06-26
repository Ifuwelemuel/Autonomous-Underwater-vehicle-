#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from mavros_msgs.srv import CommandLong

class ServoHold(Node):
    def __init__(self):
        super().__init__('servo_hold')
        self.client = self.create_client(CommandLong, '/mavros/cmd/command')
        self.client.wait_for_service()
        self.get_logger().info('Connected to MAVROS')
        # Send at 20Hz (every 0.05s) — enough to hold steady
        self.timer = self.create_timer(0.05, self.send_servo)
        self.channel = 2      # AUX 1
        self.pwm = 1500       # start at center
    def send_servo(self):
        req = CommandLong.Request()
        req.command = 183
        req.param1 = float(self.channel)
        req.param2 = float(self.pwm)
        self.client.call_async(req)

def main():
    rclpy.init()
    node = ServoHold()
    # Test sequence
    import time
    print("=== CENTER (1500) for 3 sec ===")
    node.pwm = 1500
    start = time.time()
    while time.time() - start < 3.0:
        rclpy.spin_once(node, timeout_sec=0.01)
    print("=== DEFLECT (1000) for 3 sec ===")
    node.pwm = 800
    start = time.time()
    while time.time() - start < 3.0:
        rclpy.spin_once(node, timeout_sec=0.01)
    print("=== OTHER WAY (2000) for 3 sec ===")
    node.pwm = 2200
    start = time.time()
    while time.time() - start < 3.0:
        rclpy.spin_once(node, timeout_sec=0.01)
    print("=== BACK TO CENTER (1500) ===")
    node.pwm = 1500
    start = time.time()
    while time.time() - start < 2.0:
        rclpy.spin_once(node, timeout_sec=0.01)
    print("Done!")
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
