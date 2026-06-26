import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2

class PiCameraNode(Node):
    def __init__(self):
        super().__init__('pi_camera_node')
        
        self.publisher_ = self.create_publisher(Image, '/camera/image_raw', 10)
        
        timer_period = 0.1  # (10 FPS)
        self.timer = self.create_timer(timer_period, self.timer_callback)
        
        # Initialize OpenCV VideoCapture (0 is usually the default camera)
        self.cap = cv2.VideoCapture(0)
        # Initialize CvBridge to convert OpenCV images to ROS messages
        self.br = CvBridge()
        self.get_logger().info('Pi Camera Node has been started.')

    def timer_callback(self):
        ret, frame = self.cap.read()
        
        if ret:
            # Convert the OpenCV image (BGR) to a ROS Image message
            img_msg = self.br.cv2_to_imgmsg(frame, encoding="bgr8")
            # Publish the message
            self.publisher_.publish(img_msg)
            self.get_logger().debug('Published video frame')
        else:
            self.get_logger().warning('Failed to capture frame from camera')

    def destroy_node(self):
        # Release the camera when the node is shut down
        self.cap.release()
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    camera_node = PiCameraNode()
    
    try:
        rclpy.spin(camera_node)
    except KeyboardInterrupt:
        pass
    finally:
        camera_node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()