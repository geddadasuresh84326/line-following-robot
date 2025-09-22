import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image , CameraInfo
from geometry_msgs.msg import TwistStamped
import cv2
from cv_bridge import CvBridge 
import numpy as np 
from constants.constants import aruco_distance_threshold
# linear speed
LINEAR_SPEED = 0.3 

# angular speed 
KP = 0.006

class ImageSubscriber(Node):
    def __init__(self):
        super().__init__("image_subscriber")
        self.pub_ = self.create_publisher(
            TwistStamped,
            "/robot_diff_drive_controller/cmd_vel",
            10)
        self.camera_image_sub_ = self.create_subscription(
            Image,
            "/camera_sensor/image_raw",
            self.image_callback,
            10)
        self.camera_info_sub_ = self.create_subscription(CameraInfo,"/camera_sensor/camera_info",self.camera_info_callback,10)

        self.camera_image_sub_
        self.camera_info_sub_
        self.bridge_ = CvBridge()
        self.camera_matrix = None 
        self.dist_coeffs = None 
        self.marker_length = 1.0
        self.aruco_distance = None
        self.marker_id = None
    def camera_info_callback(self, msg):
        self.camera_matrix = np.array(msg.k).reshape(3, 3)
        self.dist_coeffs = np.array(msg.d)

    def image_callback(self,msg):
        # aruco marker identification
        if self.camera_matrix is None:
            return
        # getting the camera stream
        cv_image = self.bridge_.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        gray = cv2.cvtColor(cv_image, cv2.COLOR_BGR2GRAY)

        aruco_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50)
        parameters = cv2.aruco.DetectorParameters_create()
        corners, ids, rejected = cv2.aruco.detectMarkers(gray, aruco_dict, parameters=parameters)

        # corners, ids, _ = detector.detectMarkers(gray)

        if ids is not None:
            self.get_logger().info(f"Marker id : {ids}")
            rvecs, tvecs, _ = cv2.aruco.estimatePoseSingleMarkers(
                corners, self.marker_length, self.camera_matrix, self.dist_coeffs)

            for rvec, tvec, marker_id in zip(rvecs, tvecs, ids):
                cv2.aruco.drawAxis(cv_image, self.camera_matrix, self.dist_coeffs, rvec, tvec, 0.03)
                self.get_logger().info(f"Marker {marker_id}: Position {tvec}, Rotation {rvec} x position: {tvec[0][2]}")
                self.aruco_distance = tvec[0][2]
                self.marker_id = marker_id[0]
                self.get_logger().info(f"marker id : {self.marker_id}")
        cv2.imshow("Aruco Detection", cv_image)
        # cv2.waitKey(1)

        # line segmentation 
        h,w = cv_image.shape[:2]
        # getting image right part to turn right
        if self.marker_id == 1 and self.aruco_distance < aruco_distance_threshold:
            self.get_logger().info(f"distance : {self.aruco_distance}")
            cv_image = cv_image[:,w//2+50:]
        # getting image left part to turn left
        elif self.marker_id == 0 and self.aruco_distance < aruco_distance_threshold:
            self.get_logger().info(f"distance : {self.aruco_distance}")
            cv_image = cv_image[:,:w//2+50]
        # stop the robot
        elif self.marker_id == 2 and self.aruco_distance < 2.7:
            cmd_msg = TwistStamped()
            cmd_msg.twist.linear.x = 0.0
            cmd_msg.twist.angular.z = 0.0
            self.pub_.publish(cmd_msg)
            return

        # converting bgr to hsv
        hsv_img = cv2.cvtColor(cv_image,cv2.COLOR_BGR2HSV)

        # blue color ranges in hsv
        lower_bound = np.array([100,50,50]) 
        upper_bound = np.array([130,255,255])

        # create a binary mask
        mask = cv2.inRange(hsv_img,lower_bound,upper_bound)

        # apply the mask to the original image
        blue_roi_img = cv2.bitwise_and(cv_image,cv_image,mask=mask)

        # getting line contour
        cmd_msg = TwistStamped()
        line = self.get_contours(mask)
        _,w = blue_roi_img.shape[:2]
        if line:
            x = line['x']
            error = x - w//2
            cv2.circle(blue_roi_img,(line['x'],line['y']),5,(0,0,255),7)
        
        # publishing velocity commands to our robot
        cmd_msg.twist.linear.x = LINEAR_SPEED
        if error < 0:
            cmd_msg.twist.angular.z = float(error) * -KP
        else:
            cmd_msg.twist.angular.z = float(error) * -KP
        self.get_logger().info(f"angular speed : {cmd_msg.twist.angular.z} and error : {error}")
        self.pub_.publish(cmd_msg)
        cv2.imshow("blue line ",blue_roi_img)
        cv2.imshow("cv image ",cv_image)
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
    