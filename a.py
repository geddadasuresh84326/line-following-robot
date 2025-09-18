import cv2
import cv2.aruco as aruco

aruco_dict = aruco.getPredefinedDictionary(aruco.DICT_4X4_50)
img = aruco.drawMarker(aruco_dict, 1, 1000)  # marker ID 0, size 1000px
cv2.imwrite("marker_1.png", img)
