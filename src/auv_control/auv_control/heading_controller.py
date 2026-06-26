import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import Float32MultiArray  # 1. Import the message type

class HeadingController(Node):
    def __init__(self):
        super().__init__('heading_controller')

        # Parameters
        self.pitch_max_rate = 1.0      # rad/s
        self.yaw_max_rate = 1.0        # rad/s
        self.max_deflection = 60.0     # degrees
        self.center_angle = 90.0       # servo neutral

        # Subscriber
        self.subscription = self.create_subscription(
            Twist,
            '/cmd_vel',
            self.cmd_callback,
            10,
        )

        # 2. Create the Publisher on your custom topic name ('/servo_angles')
        self.servo_pub = self.create_publisher(
            Float32MultiArray,
            '/servo_angles',
            10
        )
        
        self.thruster_pub = self.create_publisher(
            Float32MultiArray,
            '/thruster_command',
            10
        )

    def saturate(self, value, min_val, max_val):
        return max(min(value, max_val), min_val)

    def cmd_callback(self, msg: Twist):
        # Extract twist commands
        pitch_cmd = msg.angular.y
        yaw_cmd = msg.angular.z
        thruster_cmd = msg.linear.x  

        #  Normalize
        #pitch_norm = self.saturate(pitch_cmd / self.pitch_max_rate, -1.0, 1.0)
        #yaw_norm = self.saturate(yaw_cmd / self.yaw_max_rate, -1.0, 1.0)
        #  Convert to servo deflection
            
        pitch_deflection = pitch_cmd * self.max_deflection
        yaw_deflection = yaw_cmd * self.max_deflection
            
        # Add center offset
        pitch_servo_angle = self.center_angle + pitch_deflection
        yaw_servo_angle = self.center_angle + yaw_deflection
        
        # Clamp servo to valid range (0-180)
        pitch_servo_angle = self.saturate(pitch_servo_angle, 0.0, 180.0)
        yaw_servo_angle = self.saturate(yaw_servo_angle, 0.0, 180.0)
        
        # 3. Populate the message and publish it
        servo_msg = Float32MultiArray()
        # Ensure they are standard python floats
        servo_msg.data = [float(pitch_servo_angle), float(yaw_servo_angle)] 
        self.servo_pub.publish(servo_msg)
        
        
        thruster_msg = Float32MultiArray()
        #mapping thruster to pwm range 1100-1900 us, where 1500 is neutral
        
        x = max(min(thruster_cmd, 1.0), -1.0)  # clamp to [-1, 1]
        pwm_value = int(1500 + 500 * x)
        
        thruster_msg.data = [float(pwm_value)]
        self.thruster_pub.publish(thruster_msg)
        
        self.get_logger().info(
            f"Pitch Servo: {pitch_servo_angle:.2f}°, "
            f"Yaw Servo: {yaw_servo_angle:.2f}°,"
            f"Thruster PWM: {pwm_value}µs")
        
        


def main(args=None):
    rclpy.init(args=args)
    node = HeadingController()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()