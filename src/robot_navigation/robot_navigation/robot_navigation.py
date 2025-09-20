import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image 
from geometry_msgs.msg import TwistStamped
import cv2
from cv_bridge import CvBridge 
import numpy as np 

# linear speed
LINEAR_SPEED = 0.2 

# angular speed 
KP = 0.015

class ImageSubscriber(Node):
    def __init__(self):
        super().__init__("image_subscriber")
        self.pub_ = self.create_publisher(
            TwistStamped,
            "/robot_diff_drive_controller/cmd_vel",
            10)
        self.sub_ = self.create_subscription(
            Image,
            "/camera_sensor/image_raw",
            self.listener_callback,
            10)
        self.sub_
        self.bridge_ = CvBridge()

    def listener_callback(self,msg):
        # getting the camera stream
        frame_raw = self.bridge_.imgmsg_to_cv2(msg,desired_encoding='bgr8')
        frame = cv2.resize(frame_raw, (480, 320))

        # converting bgr to hsv
        hsv_img = cv2.cvtColor(frame,cv2.COLOR_BGR2HSV)

        # blue color ranges in hsv
        lower_bound = np.array([100,50,50]) 
        upper_bound = np.array([130,255,255])

        # create a binary mask
        mask = cv2.inRange(hsv_img,lower_bound,upper_bound)

        # apply the mask to the original image
        blue_roi_img = cv2.bitwise_and(frame,frame,mask=mask)

        # getting line contour
        line = self.get_contours(mask)

        if line:
            cv2.circle(blue_roi_img,(line['x'],line['y']),5,(0,0,255),7)
        
        # publishing velocity commands to our robot
        cmd_msg = TwistStamped()
        
        # self.get_logger().info(f"frame shape : {frame.shape}")
        cv2.imshow("blue line ",blue_roi_img)
        cv2.waitKey(1)

    def get_contours(self,mask):
        """Returns the centroid of the largest contour in the binary image (mask)"""
        MIN_AREA = 50

        # get list of contours 
        contours , _ = cv2.findContours(mask,cv2.RETR_EXTERNAL,cv2.CHAIN_APPROX_NONE)
        
        line = {}
        for contour in contours:
            M = cv2.moments(contour)
            if (M['m00'] > MIN_AREA):
                self.get_logger().info(f"M : {M}")
                line['x'] = int(M['m10']/M['m00'])
                line['y'] = int(M['m01']/M['m00'])
        return (line)
def main(args=None):
    rclpy.init(args = args)
    image_subscriber = ImageSubscriber()
    rclpy.spin(image_subscriber)
    image_subscriber.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()
    