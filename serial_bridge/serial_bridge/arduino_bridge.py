import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray
import serial
from sensor_msgs.msg import Imu
import glob

class ArduinoSerialBridge(Node):
    def __init__(self):
        super().__init__('arduino_serial_bridge')

        # --- Serial Configuration ---
        self.declare_parameter('serial_device', '/dev/ttyACM0')
        self.declare_parameter('baud_rate', 115200)
        self.declare_parameter('serial_timeout', 1.0)
        self.declare_parameter('auto_detect', True)
        self.declare_parameter('reconnect_period_sec', 2.0)

        self.serial_device = self.get_parameter('serial_device').value
        self.baud_rate = int(self.get_parameter('baud_rate').value)
        self.serial_timeout = float(self.get_parameter('serial_timeout').value)
        self.auto_detect = bool(self.get_parameter('auto_detect').value)
        reconnect_period = float(self.get_parameter('reconnect_period_sec').value)

        # --- Serial Connection ---
        self.serial_port = None
        self._last_connect_error = None
        self._connect_to_arduino()
        
        # --- State Variables ---
        # These map directly to the Arduino's expected variables
        
        self.target_pitch = 0  
        self.target_yaw = 0   
        self.manual_throttle = 1500 # Default neutral PWM
        self.ballast_request = 0  # 0 = Off
        
        # --- Subscribers ---
        self.create_subscription(Float32MultiArray, '/servo_angles', self.servo_callback, 10)
        self.create_subscription(Float32MultiArray, '/thruster_command', self.thruster_callback, 10)
        ##imu pub
        self.imu_publisher_ = self.create_publisher(Imu, 'imu/data_raw', 10)
        
        ##confirmation of commands from arduino---
        self.echo_pub_ = self.create_publisher(Float32MultiArray, '/arduino/echo_command', 10)
        
        
        self.get_logger().info("Serial Bridge listening for /servo_angles and /thruster_command...")
        

        # --- Timers ---
        # Send commands at 5Hz
        self.write_timer = self.create_timer(0.2, self.send_to_arduino)
        
        # Read incoming MPU data rapidly (50Hz) to keep the buffer clean
        self.read_timer = self.create_timer(0.02, self.read_from_arduino)

        # Retry serial connection if it is not available at startup or gets disconnected.
        self.reconnect_timer = self.create_timer(reconnect_period, self.ensure_serial_connection)
        
        self.get_logger().info("Serial Bridge ready.")

    def get_candidate_ports(self):
        """Returns preferred serial device plus common Arduino device paths."""
        if not self.auto_detect:
            return [self.serial_device]

        candidates = [self.serial_device]
        for pattern in ('/dev/ttyACM*', '/dev/ttyUSB*'):
            for device in sorted(glob.glob(pattern)):
                if device not in candidates:
                    candidates.append(device)
        return candidates

    def ensure_serial_connection(self):
        """Keeps trying to reconnect when serial is disconnected."""
        if self.serial_port is None or not self.serial_port.is_open:
            self._connect_to_arduino()

    def _connect_to_arduino(self):
        errors = []
        for device in self.get_candidate_ports():
            try:
                self.serial_port = serial.Serial(device, self.baud_rate, timeout=self.serial_timeout)
                self.get_logger().info(f"Connected to Arduino on {device} at {self.baud_rate} baud.")
                self._last_connect_error = None
                return
            except serial.SerialException as e:
                errors.append(f"{device}: {e}")

        self.serial_port = None
        error_summary = "No Arduino serial device found." if not errors else "; ".join(errors)
        if error_summary != self._last_connect_error:
            self.get_logger().error(f"Failed to connect to Arduino: {error_summary}")
            self._last_connect_error = error_summary

    def _handle_serial_disconnect(self, context, error):
        self.get_logger().error(f"{context}: {error}")
        if self.serial_port is not None:
            try:
                self.serial_port.close()
            except serial.SerialException:
                pass
        self.serial_port = None
        
    def servo_callback(self, msg):
        """Updates pitch and yaw, then sends to Arduino."""
        if len(msg.data) >= 2:
            self.target_pitch = msg.data[0]
            self.target_yaw = msg.data[1]
            #self.send_to_arduino()
            self.get_logger().info(f"Transmitted: {self.target_pitch}, {self.target_yaw}\n")
            
    def thruster_callback(self, msg):
        """Updates throttle, then sends to Arduino."""
        if len(msg.data) >= 1:
            self.manual_throttle = int(msg.data[0])
            #self.send_to_arduino()
            #self.get_logger().info(f"Transmitted: {self.manual_throttle}\n")
            
    def update_arduino(self):
        """Fires at a fixed rate to send the latest known state to the Arduino."""
        self.send_to_arduino()

            
    
    def send_to_arduino(self):
        """Sends the latest known command state to Arduino."""
        if self.serial_port is None or not self.serial_port.is_open:
            return

        command_string = f"C {self.target_pitch:.2f},{self.target_yaw:.2f},{self.manual_throttle},{self.ballast_request}\n"
        try:
            #sending command string to arduino
            self.serial_port.write(command_string.encode('ascii'))
        except serial.SerialTimeoutException:
            self.get_logger().warn("Serial write timed out!")
        except serial.serialutil.SerialException as e:
            self._handle_serial_disconnect("Serial write failed", e)
                
    def read_from_arduino(self):
        """Asynchronously reads incoming data (like MPU stats) from Arduino."""
        if self.serial_port is None or not self.serial_port.is_open:
            return
            
        try:
            # Read all available lines currently in the buffer
            while self.serial_port.in_waiting > 0:
                line = self.serial_port.readline().decode('utf-8').strip()
                data = line.split(',')
                
                # Check if it's a valid IMU line (6 values)
                if len(data) == 6:
                    msg = Imu()
                    msg.header.stamp = self.get_clock().now().to_msg()
                    msg.header.frame_id = "imu_link" 
                    
                    msg.linear_acceleration.x = float(data[0])
                    msg.linear_acceleration.y = float(data[1])
                    msg.linear_acceleration.z = float(data[2])
                    msg.angular_velocity.x = float(data[3])
                    msg.angular_velocity.y = float(data[4])
                    msg.angular_velocity.z = float(data[5])
                    msg.orientation_covariance[0] = -1.0
                    
                    self.imu_publisher_.publish(msg)

                # Check if it's an Echo line (4 values)
                elif len(data) == 4:
                    try:
                        echo_msg = Float32MultiArray()
                        # data[0]=Pitch, data[1]=Yaw, data[2]=Thruster, data[3]=Ballast
                        echo_msg.data = [
                            float(data[0]),
                            float(data[1]),
                            float(data[2]),
                            float(data[3])
                        ]
                        self.echo_pub_.publish(echo_msg)
                    except ValueError:
                        # Ignore if conversion to float fails due to serial noise
                        pass
                        
        except serial.SerialException as e:
            self._handle_serial_disconnect("Serial read failed", e)
        except Exception as e:
            self.get_logger().warn(f"Serial read error: {e}")        
                
                
                
                
                
                
                
                
                
def main(args=None):
    rclpy.init(args=args)
    bridge_node = ArduinoSerialBridge()
    
    try:
        rclpy.spin(bridge_node)
    except KeyboardInterrupt:
        pass
    finally:
        # Safely close the serial port when shutting down
        if hasattr(bridge_node, 'serial_port') and bridge_node.serial_port.is_open:
            bridge_node.serial_port.close()
        bridge_node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
