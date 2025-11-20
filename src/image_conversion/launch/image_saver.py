import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import os

class ImageSaver(Node):
    def __init__(self):
        super().__init__('image_saver')
        self.subscription = self.create_subscription(Image, '/image_converted', self.callback, 10)
        self.bridge = CvBridge()
        self.count = 0
        os.makedirs('/tmp/ros2_images', exist_ok=True)

    def callback(self, msg):
        cv_image = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
        filename = f'/tmp/ros2_images/img_{self.count:04d}.png'
        cv2.imwrite(filename, cv_image)
        self.count += 1
        if self.count % 10 == 0:
            self.get_logger().info(f'Saved {self.count} images')

rclpy.init()
node = ImageSaver()
rclpy.spin(node)
