import cv2
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge

class GreenObjectDetectorNode(Node):
    def __init__(self):
        super().__init__('green_object_detector')
        self.bridge = CvBridge()
        self.image_sub = self.create_subscription(Image, '/camera/image_raw', self.image_callback, 10) #might need to be edited
        self.image_pub = self.create_publisher(Image, '/detected_green_objects', 10) #might need to be edited

    def image_callback(self, msg):
        cv_image = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
        
        hsv_image = cv2.cvtColor(cv_image, cv2.COLOR_BGR2HSV)
        
        lower_green = (29, 86, 6)
        upper_green = (64, 255, 255)

        mask = cv2.inRange(hsv_image, lower_green, upper_green)

        res = cv2.bitwise_and(cv_image, cv_image, mask=mask)

        self.image_pub.publish(self.bridge.cv2_to_imgmsg(res, 'bgr8')) 

def main(args=None):
    rclpy.init(args=args)
    green_detector = GreenObjectDetectorNode()
    rclpy.spin(green_detector)

if __name__ == '__main__':
    main()