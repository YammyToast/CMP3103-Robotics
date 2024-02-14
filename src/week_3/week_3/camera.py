#!/usr/bin/env python

# Written for humble
# cv2 image types - http://wiki.ros.org/cv_bridge/Tutorials/ConvertingBetweenROSImagesAndOpenCVImagesPython

import rclpy
from rclpy.node import Node

from sensor_msgs.msg import Image
from cv_bridge import CvBridge # Package to convert between ROS and OpenCV Images
import cv2
import numpy as np

from std_msgs.msg import String

class OpencvBridge(Node):
    """
    A class that demonstrates how to use the ROS2 CVBridge for image processing.

    This class sets up the necessary subscriptions and provides a callback function
    for processing camera data. It converts ROS Image messages to OpenCV images,
    performs various image processing operations, and displays the results.

    Attributes:
        br: CvBridge object for converting ROS Image messages to OpenCV images.

    Subscriptions:
        /limo/depth_camera_link/image_raw: ROS Image topic for receiving camera data.

    Published Topics:
        None

    """

    def __init__(self):
        """
        Initializes the OpencvBridge class.

        This method sets up the necessary subscriptions and initializes the CvBridge object.

        Args:
            None

        Returns:
            None
        """
        super().__init__('opencv_bridge')
        # self.create_subscription(Image, '/limo/depth_camera_link/image_raw', self.camera_callback, 10)
        self.create_subscription(Image, '/camera/color/image_raw', self.camera_callback, 10)

        self.br = CvBridge()

        self.publisher_ = self.create_publisher(String, '/camera_info', 1);
        self.timer = self.create_timer(1.0, self.timer_callback)
        self.frame_mean = 0.0

    def camera_callback(self, data):
        """
        Callback function for processing camera data.

        Args:
            data: ROS Image message containing camera data.

        Returns:
            None
        """
        # cv2.namedWindow("Image window")
        # cv2.namedWindow("blur")
        # cv2.namedWindow("canny")
        cv2.namedWindow("Feed")
        cv2.namedWindow("CurrentFrameHSV")
        cv2.namedWindow("Masked")

        # Convert ROS Image message to OpenCV image
        # cv_image = self.br.imgmsg_to_cv2(data, desired_encoding='bgr8')

        # gray_img = cv2.cvtColor(cv_image, cv2.COLOR_BGR2GRAY)
        # gray_img_small = cv2.resize(gray_img, (0,0), fx=0.2, fy=0.2) 
        # print(np.mean(gray_img_small))

        # blur_img = cv2.blur(gray_img, (3, 3))
        # blur_img_small = cv2.resize(blur_img, (0,0), fx=0.2, fy=0.2) # reduce image size
        # cv2.imshow("blur", blur_img_small)

        # canny_img = cv2.Canny(gray_img, 10, 200)
        # canny_img_small = cv2.resize(canny_img, (0,0), fx=0.7, fy=0.7) # reduce image size
        # cv2.imshow("canny", canny_img_small)

        # cv_image_small = cv2.resize(cv_image, (0,0), fx=0.2, fy=0.2) # reduce image size
        # cv2.imshow("Image window", cv_image_small)
        cv_image = self.br.imgmsg_to_cv2(data, desired_encoding='bgr8')
        cv2.imshow("Feed", cv_image)
        current_frame_hsv = cv2.cvtColor(cv_image, cv2.COLOR_BGR2HSV)
        current_frame_resize = cv2.resize(current_frame_hsv, (0,0), fx=0.5, fy=0.5)
        cv2.imshow("CurrentFrameHSV", current_frame_resize)
        current_frame_mask = cv2.inRange(current_frame_hsv, np.array((0, 150, 50)),
            np.array((255, 255, 255)))
        cv2.imshow("Masked", current_frame_mask)

        hsv_thresh = cv2.inRange(current_frame_hsv,
            np.array((0, 150, 50)),
            np.array((255, 255, 255)))
        # self.frame_mean = (
        #     cv2.mean(current_frame_hsv[:, :, 0], mask = hsv_thresh)[0],
        #     cv2.mean(current_frame_hsv[:, :, 1], mask = hsv_thresh)[0],
        #     cv2.mean(current_frame_hsv[:, :, 2], mask = hsv_thresh)[0]
        # )
        
        
        cv2.waitKey(1)

    def timer_callback(self):
        msg = String()
        msg.data = "test"
        # msg.data = f"b:{self.frame_mean[0]}|g:{self.frame_mean[1]}|r:{self.frame_mean[2]}"
        self.publisher_.publish(msg)

def main(args=None):
    print('Starting opencv_bridge.py.')

    rclpy.init(args=args)

    opencv_bridge = OpencvBridge()

    rclpy.spin(opencv_bridge)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    opencv_bridge.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()