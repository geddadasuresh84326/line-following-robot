import rclpy
from rclpy.node import Node 
from sensor_msgs.msg import Image,CameraInfo
from cv_bridge import CvBridge 
import cv2 
import numpy as np 


class ArucoPoseEstimator(Node):
    def __init__(self,node_name):
        super().__init__(node_name)
        self.bridge = CvBridge()
        self.camera_matrix = None 
        self.dist_coeffs = None 
        self.marker_length = 1.0

        self.create_subscription(CameraInfo,"/camera_sensor/camera_info",self.camera_info_callback,10)
        self.create_subscription(Image,"/camera_sensor/image_raw",self.image_callback,10)
    
    def camera_info_callback(self, msg):
        self.camera_matrix = np.array(msg.k).reshape(3, 3)
        self.dist_coeffs = np.array(msg.d)

    def image_callback(self, msg):
        if self.camera_matrix is None:
            return
        cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
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
                self.get_logger().info(f"Marker {marker_id}: Position {tvec}, Rotation {rvec}")

        cv2.imshow("Aruco Detection", cv_image)
        cv2.waitKey(1)

def main(args=None):
    rclpy.init(args=args)
    node = ArucoPoseEstimator("aruco_pose_estimator")
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()